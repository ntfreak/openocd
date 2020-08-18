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
#include "target.h"
#include "target_type.h"

static int arm7a_l2x_sanity_check(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a->armv7a_mmu.armv7a_cache.outer_cache);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!l2x_cache || !l2x_cache->base) {
		LOG_DEBUG("l2x is not configured!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}
/*
 * clean and invalidate complete l2x cache
 */
int arm7a_l2x_flush_all_data(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a->armv7a_mmu.armv7a_cache.outer_cache);
	uint32_t l2_way_val;
	int retval;

	retval = arm7a_l2x_sanity_check(target);
	if (retval)
		return retval;

	l2_way_val = (1 << l2x_cache->way) - 1;

	return target_write_phys_u32(target,
			l2x_cache->base + L2X0_CLEAN_INV_WAY,
			l2_way_val);
}

int armv7a_l2x_cache_flush_virt(struct target *target, target_addr_t virt,
					uint32_t size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a->armv7a_mmu.armv7a_cache.outer_cache);
	/* FIXME: different controllers have different linelen? */
	uint32_t i, linelen = 32;
	int retval;

	retval = arm7a_l2x_sanity_check(target);
	if (retval)
		return retval;

	for (i = 0; i < size; i += linelen) {
		target_addr_t pa, offs = virt + i;

		/* FIXME: use less verbose virt2phys? */
		retval = target->type->virt2phys(target, offs, &pa);
		if (retval != ERROR_OK)
			goto done;

		retval = target_write_phys_u32(target,
				l2x_cache->base + L2X0_CLEAN_INV_LINE_PA, pa);
		if (retval != ERROR_OK)
			goto done;
	}
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");

	return retval;
}

static int armv7a_l2x_cache_inval_virt(struct target *target, target_addr_t virt,
					uint32_t size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a->armv7a_mmu.armv7a_cache.outer_cache);
	/* FIXME: different controllers have different linelen */
	uint32_t i, linelen = 32;
	int retval;

	retval = arm7a_l2x_sanity_check(target);
	if (retval)
		return retval;

	for (i = 0; i < size; i += linelen) {
		target_addr_t pa, offs = virt + i;

		/* FIXME: use less verbose virt2phys? */
		retval = target->type->virt2phys(target, offs, &pa);
		if (retval != ERROR_OK)
			goto done;

		retval = target_write_phys_u32(target,
				l2x_cache->base + L2X0_INV_LINE_PA, pa);
		if (retval != ERROR_OK)
			goto done;
	}
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");

	return retval;
}

static int armv7a_l2x_cache_clean_virt(struct target *target, target_addr_t virt,
					unsigned int size)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a->armv7a_mmu.armv7a_cache.outer_cache);
	/* FIXME: different controllers have different linelen */
	uint32_t i, linelen = 32;
	int retval;

	retval = arm7a_l2x_sanity_check(target);
	if (retval)
		return retval;

	for (i = 0; i < size; i += linelen) {
		target_addr_t pa, offs = virt + i;

		/* FIXME: use less verbose virt2phys? */
		retval = target->type->virt2phys(target, offs, &pa);
		if (retval != ERROR_OK)
			goto done;

		retval = target_write_phys_u32(target,
				l2x_cache->base + L2X0_CLEAN_LINE_PA, pa);
		if (retval != ERROR_OK)
			goto done;
	}
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");

	return retval;
}

static int arm7a_handle_l2x_cache_info_command(struct command_invocation *cmd,
	struct armv7a_cache_common *armv7a_cache)
{
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a_cache->outer_cache);

	if (armv7a_cache->info == -1) {
		command_print(cmd, "cache not yet identified");
		return ERROR_OK;
	}

	command_print(cmd,
		      "L2 unified cache Base Address 0x%" PRIx32 ", %" PRIu32 " ways",
		      l2x_cache->base, l2x_cache->way);

	return ERROR_OK;
}

static int armv7a_l2x_cache_init(struct target *target, uint32_t base, uint32_t way)
{
	struct armv7a_l2x_cache *l2x_cache;
	struct target_list *head = target->head;
	struct target *curr;

	struct armv7a_common *armv7a = target_to_armv7a(target);
	if (armv7a->armv7a_mmu.armv7a_cache.outer_cache) {
		LOG_ERROR("L2 cache was already initialised\n");
		return ERROR_FAIL;
	}

	l2x_cache = calloc(1, sizeof(struct armv7a_l2x_cache));
	l2x_cache->base = base;
	l2x_cache->way = way;
	armv7a->armv7a_mmu.armv7a_cache.outer_cache = l2x_cache;

	/*  initialize all targets in this cluster (smp target)
	 *  l2 cache must be configured after smp declaration */
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if (curr != target) {
			armv7a = target_to_armv7a(curr);
			if (armv7a->armv7a_mmu.armv7a_cache.outer_cache) {
				LOG_ERROR("smp target : cache l2 already initialized\n");
				return ERROR_FAIL;
			}
			armv7a->armv7a_mmu.armv7a_cache.outer_cache = l2x_cache;
		}
		head = head->next;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(arm7a_l2x_cache_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int retval;

	retval = arm7a_l2x_sanity_check(target);
	if (retval)
		return retval;

	return arm7a_handle_l2x_cache_info_command(CMD,
			&armv7a->armv7a_mmu.armv7a_cache);
}

COMMAND_HANDLER(arm7a_l2x_cache_flush_all_command)
{
	struct target *target = get_current_target(CMD_CTX);

	return arm7a_l2x_flush_all_data(target);
}

COMMAND_HANDLER(arm7a_l2x_cache_flush_virt_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	target_addr_t virt;
	uint32_t size;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	else
		size = 1;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], virt);

	return armv7a_l2x_cache_flush_virt(target, virt, size);
}

COMMAND_HANDLER(arm7a_l2x_cache_inval_virt_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	target_addr_t virt;
	uint32_t size;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	else
		size = 1;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], virt);

	return armv7a_l2x_cache_inval_virt(target, virt, size);
}

COMMAND_HANDLER(arm7a_l2x_cache_clean_virt_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	target_addr_t virt;
	uint32_t size;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);
	else
		size = 1;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], virt);

	return armv7a_l2x_cache_clean_virt(target, virt, size);
}

/* FIXME: should we configure way size? or controller type? */
COMMAND_HANDLER(armv7a_l2x_cache_conf_cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t base, way;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* command_print(CMD, "%s %s", CMD_ARGV[0], CMD_ARGV[1]); */
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], way);

	/* AP address is in bits 31:24 of DP_SELECT */
	return armv7a_l2x_cache_init(target, base, way);
}

static const struct command_registration arm7a_l2x_cache_commands[] = {
	{
		.name = "conf",
		.handler = armv7a_l2x_cache_conf_cmd,
		.mode = COMMAND_ANY,
		.help = "configure l2x cache ",
		.usage = "<base_addr> <number_of_way>",
	},
	{
		.name = "info",
		.handler = arm7a_l2x_cache_info_command,
		.mode = COMMAND_ANY,
		.help = "print cache related information",
		.usage = "",
	},
	{
		.name = "flush_all",
		.handler = arm7a_l2x_cache_flush_all_command,
		.mode = COMMAND_ANY,
		.help = "flush complete l2x cache",
		.usage = "",
	},
	{
		.name = "flush",
		.handler = arm7a_l2x_cache_flush_virt_cmd,
		.mode = COMMAND_ANY,
		.help = "flush (clean and invalidate) l2x cache by virtual address offset and range size",
		.usage = "<virt_addr> [size]",
	},
	{
		.name = "inval",
		.handler = arm7a_l2x_cache_inval_virt_cmd,
		.mode = COMMAND_ANY,
		.help = "invalidate l2x cache by virtual address offset and range size",
		.usage = "<virt_addr> [size]",
	},
	{
		.name = "clean",
		.handler = arm7a_l2x_cache_clean_virt_cmd,
		.mode = COMMAND_ANY,
		.help = "clean l2x cache by virtual address address offset and range size",
		.usage = "<virt_addr> [size]",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arm7a_l2x_cache_command_handler[] = {
	{
		.name = "l2x",
		.mode = COMMAND_ANY,
		.help = "l2x cache command group",
		.usage = "",
		.chain = arm7a_l2x_cache_commands,
	},
	COMMAND_REGISTRATION_DONE
};
