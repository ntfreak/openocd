/***************************************************************************
 *   Copyright (C) 2015 by Oleksij Rempel                                  *
 *   linux@rempel-privat.de                                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "arm.h"
#include "armv7a.h"
#include "armv7a_cache.h"
#include <helper/time_support.h>
#include "arm_opcodes.h"

static int armv7a_l1_d_cache_sanity_check(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	/*  check that cache data is on at target halt */
	if (!armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled) {
		LOG_DEBUG("data cache is not enabled");
		return ERROR_TARGET_INVALID;
	}

	return ERROR_OK;
}

static int armv7a_l1_i_cache_sanity_check(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	/*  check that cache data is on at target halt */
	if (!armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled) {
		LOG_DEBUG("instruction cache is not enabled");
		return ERROR_TARGET_INVALID;
	}

	return ERROR_OK;
}

static int armv7a_l1_d_cache_flush_level(struct arm_dpm *dpm, struct armv7a_cachesize *size, int cl)
{
	int retval = ERROR_OK;
	int32_t c_way, c_index = size->index;

	LOG_DEBUG("cl %" PRId32, cl);
	do {
		keep_alive();
		c_way = size->way;
		do {
			uint32_t value = (c_index << size->index_shift)
				| (c_way << size->way_shift) | (cl << 1);
			/*
			 * DCCISW - Clean and invalidate data cache
			 * line by Set/Way.
			 */
			retval = dpm->instr_write_data_r0(dpm,
					ARMV4_5_MCR(15, 0, 0, 7, 14, 2),
					value);
			if (retval != ERROR_OK)
				goto done;
			c_way -= 1;
		} while (c_way >= 0);
		c_index -= 1;
	} while (c_index >= 0);

 done:
	keep_alive();
	return retval;
}

static int armv7a_l1_d_cache_clean_inval_all(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv7a_cache_common *cache = &(armv7a->armv7a_mmu.armv7a_cache);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	int cl;
	int retval;

	retval = armv7a_l1_d_cache_sanity_check(target);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	for (cl = 0; cl < cache->loc; cl++) {
		/* skip i-only caches */
		if (cache->arch[cl].ctype < CACHE_LEVEL_HAS_D_CACHE)
			continue;

		armv7a_l1_d_cache_flush_level(dpm, &cache->arch[cl].d_u_size, cl);
	}

	retval = dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("clean invalidate failed");
	dpm->finish(dpm);

	return retval;
}

int armv7a_cache_auto_flush_all_data(struct target *target)
{
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (!armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled)
		return ERROR_OK;

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			if (curr->state == TARGET_HALTED)
				retval = armv7a_l1_d_cache_clean_inval_all(curr);

			head = head->next;
		}
	} else
		retval = armv7a_l1_d_cache_clean_inval_all(target);

	if (retval != ERROR_OK)
		return retval;

	/* do outer cache flushing after inner caches have been flushed */
	return arm7a_l2x_flush_all_data(target);
}


int armv7a_l1_d_cache_inval_virt(struct target *target, uint32_t virt,
					uint32_t size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	struct armv7a_cache_common *armv7a_cache = &armv7a->armv7a_mmu.armv7a_cache;
	uint32_t linelen = armv7a_cache->dminline;
	uint32_t va_line, va_end;
	int retval, i = 0;

	retval = armv7a_l1_d_cache_sanity_check(target);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = virt & (-linelen);
	va_end = virt + size;

	/* handle unaligned start */
	if (virt != va_line) {
		/* DCCIMVAC */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 14, 1), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	/* handle unaligned end */
	if ((va_end & (linelen-1)) != 0) {
		va_end &= (-linelen);
		/* DCCIMVAC */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 14, 1), va_end);
		if (retval != ERROR_OK)
			goto done;
	}

	while (va_line < va_end) {
		if ((i++ & 0x3f) == 0)
			keep_alive();
		/* DCIMVAC - Invalidate data cache line by VA to PoC. */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 6, 1), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	keep_alive();
	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");
	keep_alive();
	dpm->finish(dpm);

	return retval;
}

int armv7a_l1_d_cache_clean_virt(struct target *target, uint32_t virt,
					unsigned int size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	struct armv7a_cache_common *armv7a_cache = &armv7a->armv7a_mmu.armv7a_cache;
	uint32_t linelen = armv7a_cache->dminline;
	uint32_t va_line, va_end;
	int retval, i = 0;

	retval = armv7a_l1_d_cache_sanity_check(target);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = virt & (-linelen);
	va_end = virt + size;

	while (va_line < va_end) {
		if ((i++ & 0x3f) == 0)
			keep_alive();
		/* DCCMVAC - Data Cache Clean by MVA to PoC */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 10, 1), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	keep_alive();
	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");
	keep_alive();
	dpm->finish(dpm);

	return retval;
}

int armv7a_l1_d_cache_flush_virt(struct target *target, uint32_t virt,
					unsigned int size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	struct armv7a_cache_common *armv7a_cache = &armv7a->armv7a_mmu.armv7a_cache;
	uint32_t linelen = armv7a_cache->dminline;
	uint32_t va_line, va_end;
	int retval, i = 0;

	retval = armv7a_l1_d_cache_sanity_check(target);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = virt & (-linelen);
	va_end = virt + size;

	while (va_line < va_end) {
		if ((i++ & 0x3f) == 0)
			keep_alive();
		/* DCCIMVAC */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 14, 1), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	keep_alive();
	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");
	keep_alive();
	dpm->finish(dpm);

	return retval;
}

int armv7a_l1_i_cache_inval_all(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	int retval;

	retval = armv7a_l1_i_cache_sanity_check(target);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	if (target->smp) {
		/* ICIALLUIS */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 1, 0), 0);
	} else {
		/* ICIALLU */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 5, 0), 0);
	}

	if (retval != ERROR_OK)
		goto done;

	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("i-cache invalidate failed");
	dpm->finish(dpm);

	return retval;
}

int armv7a_l1_i_cache_inval_virt(struct target *target, uint32_t virt,
					uint32_t size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	struct armv7a_cache_common *armv7a_cache =
				&armv7a->armv7a_mmu.armv7a_cache;
	uint32_t linelen = armv7a_cache->iminline;
	uint32_t va_line, va_end;
	int retval, i = 0;

	retval = armv7a_l1_i_cache_sanity_check(target);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = virt & (-linelen);
	va_end = virt + size;

	while (va_line < va_end) {
		if ((i++ & 0x3f) == 0)
			keep_alive();
		/* ICIMVAU - Invalidate instruction cache by VA to PoU. */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 5, 1), va_line);
		if (retval != ERROR_OK)
			goto done;
		/* BPIMVA */
		retval = dpm->instr_write_data_r0(dpm,
				ARMV4_5_MCR(15, 0, 0, 7, 5, 7), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}
	keep_alive();
	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("i-cache invalidate failed");
	keep_alive();
	dpm->finish(dpm);

	return retval;
}

int armv7a_cache_flush_virt(struct target *target, uint32_t virt,
				uint32_t size)
{
	armv7a_l1_d_cache_flush_virt(target, virt, size);
	armv7a_l2x_cache_flush_virt(target, virt, size);

	return ERROR_OK;
}

/*
 * We assume that target core was chosen correctly. It means if same data
 * was handled by two cores, other core will loose the changes. Since it
 * is impossible to know (FIXME) which core has correct data, keep in mind
 * that some kind of data lost or corruption is possible.
 * Possible scenario:
 *  - core1 loaded and changed data on 0x12345678
 *  - we halted target and modified same data on core0
 *  - data on core1 will be lost.
 */
int armv7a_cache_auto_flush_on_write(struct target *target, uint32_t virt,
					uint32_t size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (!armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled)
		return ERROR_OK;

	return armv7a_cache_flush_virt(target, virt, size);
}

COMMAND_HANDLER(arm7a_l1_cache_info_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	return armv7a_handle_cache_info_command(CMD,
			&armv7a->armv7a_mmu.armv7a_cache);
}

COMMAND_HANDLER(armv7a_l1_d_cache_clean_inval_all_cmd)
{
	struct target *target = get_current_target(CMD_CTX);

	armv7a_l1_d_cache_clean_inval_all(target);

	return 0;
}

COMMAND_HANDLER(arm7a_l1_d_cache_inval_virt_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t virt, size;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	else
		size = 1;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], virt);

	return armv7a_l1_d_cache_inval_virt(target, virt, size);
}

COMMAND_HANDLER(arm7a_l1_d_cache_clean_virt_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t virt, size;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	else
		size = 1;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], virt);

	return armv7a_l1_d_cache_clean_virt(target, virt, size);
}

COMMAND_HANDLER(armv7a_i_cache_clean_inval_all_cmd)
{
	struct target *target = get_current_target(CMD_CTX);

	armv7a_l1_i_cache_inval_all(target);

	return 0;
}

COMMAND_HANDLER(arm7a_l1_i_cache_inval_virt_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t virt, size;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	else
		size = 1;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], virt);

	return armv7a_l1_i_cache_inval_virt(target, virt, size);
}

COMMAND_HANDLER(arm7a_cache_disable_auto_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (CMD_ARGC == 0) {
		command_print(CMD, "auto cache is %s",
			armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled ? "enabled" : "disabled");
		return ERROR_OK;
	}

	if (CMD_ARGC == 1) {
		uint32_t set;

		COMMAND_PARSE_ENABLE(CMD_ARGV[0], set);
		armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled = !!set;
		return ERROR_OK;
	}

	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const struct command_registration arm7a_l1_d_cache_commands[] = {
	{
		.name = "flush_all",
		.handler = armv7a_l1_d_cache_clean_inval_all_cmd,
		.mode = COMMAND_ANY,
		.help = "flush (clean and invalidate) complete l1 d-cache",
		.usage = "",
	},
	{
		.name = "inval",
		.handler = arm7a_l1_d_cache_inval_virt_cmd,
		.mode = COMMAND_ANY,
		.help = "invalidate l1 d-cache by virtual address offset and range size",
		.usage = "<virt_addr> [size]",
	},
	{
		.name = "clean",
		.handler = arm7a_l1_d_cache_clean_virt_cmd,
		.mode = COMMAND_ANY,
		.help = "clean l1 d-cache by virtual address address offset and range size",
		.usage = "<virt_addr> [size]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration arm7a_l1_i_cache_commands[] = {
	{
		.name = "inval_all",
		.handler = armv7a_i_cache_clean_inval_all_cmd,
		.mode = COMMAND_ANY,
		.help = "invalidate complete l1 i-cache",
		.usage = "",
	},
	{
		.name = "inval",
		.handler = arm7a_l1_i_cache_inval_virt_cmd,
		.mode = COMMAND_ANY,
		.help = "invalidate l1 i-cache by virtual address offset and range size",
		.usage = "<virt_addr> [size]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration arm7a_l1_di_cache_group_handlers[] = {
	{
		.name = "info",
		.handler = arm7a_l1_cache_info_cmd,
		.mode = COMMAND_ANY,
		.help = "print cache related information",
		.usage = "",
	},
	{
		.name = "d",
		.mode = COMMAND_ANY,
		.help = "l1 d-cache command group",
		.usage = "",
		.chain = arm7a_l1_d_cache_commands,
	},
	{
		.name = "i",
		.mode = COMMAND_ANY,
		.help = "l1 i-cache command group",
		.usage = "",
		.chain = arm7a_l1_i_cache_commands,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration arm7a_cache_group_handlers[] = {
	{
		.name = "auto",
		.handler = arm7a_cache_disable_auto_cmd,
		.mode = COMMAND_ANY,
		.help = "disable or enable automatic cache handling.",
		.usage = "(1|0)",
	},
	{
		.name = "l1",
		.mode = COMMAND_ANY,
		.help = "l1 cache command group",
		.usage = "",
		.chain = arm7a_l1_di_cache_group_handlers,
	},
	{
		.chain = arm7a_l2x_cache_command_handler,
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arm7a_cache_command_handlers[] = {
	{
		.name = "cache",
		.mode = COMMAND_ANY,
		.help = "cache command group",
		.usage = "",
		.chain = arm7a_cache_group_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
