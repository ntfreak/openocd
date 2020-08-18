/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *   matthias.welwarsky@sysgo.com                                          *
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

#include "armv8_cache.h"
#include "armv8_dpm.h"
#include "armv8_opcodes.h"

/* CLIDR cache types */
#define CACHE_LEVEL_HAS_UNIFIED_CACHE	0x4
#define CACHE_LEVEL_HAS_D_CACHE		0x2
#define CACHE_LEVEL_HAS_I_CACHE		0x1

static int armv8_d_cache_sanity_check(struct armv8_common *armv8)
{
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;

	if (armv8_cache->d_u_cache_enabled)
		return ERROR_OK;

	return ERROR_TARGET_INVALID;
}

static int armv8_i_cache_sanity_check(struct armv8_common *armv8)
{
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;

	if (armv8_cache->i_cache_enabled)
		return ERROR_OK;

	return ERROR_TARGET_INVALID;
}

static int armv8_cache_d_inner_flush_level(struct armv8_common *armv8, struct armv8_cachesize *size, int cl)
{
	struct arm_dpm *dpm = armv8->arm.dpm;
	int retval = ERROR_OK;
	int32_t c_way, c_index = size->index;

	LOG_DEBUG("cl %" PRId32, cl);
	do {
		c_way = size->way;
		do {
			uint32_t value = (c_index << size->index_shift)
				| (c_way << size->way_shift) | (cl << 1);
			/*
			 * DC CISW - Clean and invalidate data cache
			 * line by Set/Way.
			 */
			retval = dpm->instr_write_data_r0(dpm,
					armv8_opcode(armv8, ARMV8_OPC_DCCISW), value);
			if (retval != ERROR_OK)
				goto done;
			c_way -= 1;
		} while (c_way >= 0);
		c_index -= 1;
	} while (c_index >= 0);

 done:
	return retval;
}

static int armv8_cache_d_inner_clean_inval_all(struct armv8_common *armv8)
{
	struct armv8_cache_common *cache = &(armv8->armv8_mmu.armv8_cache);
	struct arm_dpm *dpm = armv8->arm.dpm;
	int cl;
	int retval;

	retval = armv8_d_cache_sanity_check(armv8);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	for (cl = 0; cl < cache->loc; cl++) {
		/* skip i-only caches */
		if (cache->arch[cl].ctype < CACHE_LEVEL_HAS_D_CACHE)
			continue;

		armv8_cache_d_inner_flush_level(armv8, &cache->arch[cl].d_u_size, cl);
	}

	retval = dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("clean invalidate failed");
	dpm->finish(dpm);

	return retval;
}

int armv8_cache_d_inner_flush_virt(struct armv8_common *armv8, target_addr_t va, size_t size)
{
	struct arm_dpm *dpm = armv8->arm.dpm;
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;
	uint64_t linelen = armv8_cache->dminline;
	target_addr_t va_line, va_end;
	int retval;

	retval = armv8_d_cache_sanity_check(armv8);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = va & (-linelen);
	va_end = va + size;

	while (va_line < va_end) {
		/* DC CIVAC */
		/* Aarch32: DCCIMVAC: ARMV4_5_MCR(15, 0, 0, 7, 14, 1) */
		retval = dpm->instr_write_data_r0_64(dpm,
				armv8_opcode(armv8, ARMV8_OPC_DCCIVAC), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");
	dpm->finish(dpm);

	return retval;
}

int armv8_cache_i_inner_inval_virt(struct armv8_common *armv8, target_addr_t va, size_t size)
{
	struct arm_dpm *dpm = armv8->arm.dpm;
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;
	uint64_t linelen = armv8_cache->iminline;
	target_addr_t va_line, va_end;
	int retval;

	retval = armv8_i_cache_sanity_check(armv8);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = va & (-linelen);
	va_end = va + size;

	while (va_line < va_end) {
		/* IC IVAU - Invalidate instruction cache by VA to PoU. */
		retval = dpm->instr_write_data_r0_64(dpm,
				armv8_opcode(armv8, ARMV8_OPC_ICIVAU), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");
	dpm->finish(dpm);

	return retval;
}

static int armv8_handle_inner_cache_info_command(struct command_invocation *cmd,
	struct armv8_cache_common *armv8_cache)
{
	int cl;

	if (armv8_cache->info == -1) {
		command_print(cmd, "cache not yet identified");
		return ERROR_OK;
	}

	for (cl = 0; cl < armv8_cache->loc; cl++) {
		struct armv8_arch_cache *arch = &(armv8_cache->arch[cl]);

		if (arch->ctype & 1) {
			command_print(cmd,
				"L%d I-Cache: linelen %" PRIu32
				", associativity %" PRIu32
				", nsets %" PRIu32
				", cachesize %" PRIu32 " KBytes",
				cl+1,
				arch->i_size.linelen,
				arch->i_size.associativity,
				arch->i_size.nsets,
				arch->i_size.cachesize);
		}

		if (arch->ctype >= 2) {
			command_print(cmd,
				"L%d D-Cache: linelen %" PRIu32
				", associativity %" PRIu32
				", nsets %" PRIu32
				", cachesize %" PRIu32 " KBytes",
				cl+1,
				arch->d_u_size.linelen,
				arch->d_u_size.associativity,
				arch->d_u_size.nsets,
				arch->d_u_size.cachesize);
		}
	}

	return ERROR_OK;
}

static int _armv8_flush_all_data(struct target *target)
{
	return armv8_cache_d_inner_clean_inval_all(target_to_armv8(target));
}

static int  armv8_flush_all_data(struct target *target)
{
	int retval = ERROR_FAIL;
	/*  check that armv8_cache is correctly identify */
	struct armv8_common *armv8 = target_to_armv8(target);
	if (armv8->armv8_mmu.armv8_cache.info == -1) {
		LOG_ERROR("trying to flush un-identified cache");
		return retval;
	}

	if (target->smp) {
		/*  look if all the other target have been flushed in order to flush level
		 *  2 */
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			if (curr->state == TARGET_HALTED) {
				LOG_INFO("Wait flushing data l1 on core %" PRId32, curr->coreid);
				retval = _armv8_flush_all_data(curr);
			}
			head = head->next;
		}
	} else
		retval = _armv8_flush_all_data(target);
	return retval;
}

static int get_cache_info(struct arm_dpm *dpm, int cl, int ct, uint32_t *cache_reg)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval = ERROR_OK;

	/*  select cache level */
	retval = dpm->instr_write_data_r0(dpm,
			armv8_opcode(armv8, WRITE_REG_CSSELR),
			(cl << 1) | (ct == 1 ? 1 : 0));
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			armv8_opcode(armv8, READ_REG_CCSIDR),
			cache_reg);
 done:
	return retval;
}

static struct armv8_cachesize decode_cache_reg(uint32_t cache_reg)
{
	struct armv8_cachesize size;
	int i = 0;

	size.linelen = 16 << (cache_reg & 0x7);
	size.associativity = ((cache_reg >> 3) & 0x3ff) + 1;
	size.nsets = ((cache_reg >> 13) & 0x7fff) + 1;
	size.cachesize = size.linelen * size.associativity * size.nsets / 1024;

	/*  compute info for set way operation on cache */
	size.index_shift = (cache_reg & 0x7) + 4;
	size.index = (cache_reg >> 13) & 0x7fff;
	size.way = ((cache_reg >> 3) & 0x3ff);

	while (((size.way << i) & 0x80000000) == 0)
		i++;
	size.way_shift = i;

	return size;
}

int armv8_identify_cache(struct armv8_common *armv8)
{
	/*  read cache descriptor */
	int retval = ERROR_FAIL;
	struct arm *arm = &armv8->arm;
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t csselr, clidr, ctr;
	uint32_t cache_reg;
	int cl, ctype;
	struct armv8_cache_common *cache = &(armv8->armv8_mmu.armv8_cache);

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* check if we're in an unprivileged mode */
	if (armv8_curel_from_core_mode(arm->core_mode) < SYSTEM_CUREL_EL1) {
		retval = armv8_dpm_modeswitch(dpm, ARMV8_64_EL1H);
		if (retval != ERROR_OK)
			return retval;
	}

	/* retrieve CTR */
	retval = dpm->instr_read_data_r0(dpm,
			armv8_opcode(armv8, READ_REG_CTR), &ctr);
	if (retval != ERROR_OK)
		goto done;

	cache->iminline = 4UL << (ctr & 0xf);
	cache->dminline = 4UL << ((ctr & 0xf0000) >> 16);
	LOG_DEBUG("ctr %" PRIx32 " ctr.iminline %" PRIu32 " ctr.dminline %" PRIu32,
		 ctr, cache->iminline, cache->dminline);

	/*  retrieve CLIDR */
	retval = dpm->instr_read_data_r0(dpm,
			armv8_opcode(armv8, READ_REG_CLIDR), &clidr);
	if (retval != ERROR_OK)
		goto done;

	cache->loc = (clidr & 0x7000000) >> 24;
	LOG_DEBUG("Number of cache levels to PoC %" PRId32, cache->loc);

	/*  retrieve selected cache for later restore
	 *  MRC p15, 2,<Rd>, c0, c0, 0; Read CSSELR */
	retval = dpm->instr_read_data_r0(dpm,
			armv8_opcode(armv8, READ_REG_CSSELR), &csselr);
	if (retval != ERROR_OK)
		goto done;

	/* retrieve all available inner caches */
	for (cl = 0; cl < cache->loc; clidr >>= 3, cl++) {

		/* isolate cache type at current level */
		ctype = clidr & 7;

		/* skip reserved values */
		if (ctype > CACHE_LEVEL_HAS_UNIFIED_CACHE)
			continue;

		/* separate d or unified d/i cache at this level ? */
		if (ctype & (CACHE_LEVEL_HAS_UNIFIED_CACHE | CACHE_LEVEL_HAS_D_CACHE)) {
			/* retrieve d-cache info */
			retval = get_cache_info(dpm, cl, 0, &cache_reg);
			if (retval != ERROR_OK)
				goto done;
			cache->arch[cl].d_u_size = decode_cache_reg(cache_reg);

			LOG_DEBUG("data/unified cache index %" PRIu32 " << %" PRIu32 ", way %" PRIu32 " << %" PRIu32,
					cache->arch[cl].d_u_size.index,
					cache->arch[cl].d_u_size.index_shift,
					cache->arch[cl].d_u_size.way,
					cache->arch[cl].d_u_size.way_shift);

			LOG_DEBUG("cacheline %" PRIu32 " bytes %" PRIu32 " KBytes asso %" PRIu32 " ways",
					cache->arch[cl].d_u_size.linelen,
					cache->arch[cl].d_u_size.cachesize,
					cache->arch[cl].d_u_size.associativity);
		}

		/* separate i-cache at this level ? */
		if (ctype & CACHE_LEVEL_HAS_I_CACHE) {
			/* retrieve i-cache info */
			retval = get_cache_info(dpm, cl, 1, &cache_reg);
			if (retval != ERROR_OK)
				goto done;
			cache->arch[cl].i_size = decode_cache_reg(cache_reg);

			LOG_DEBUG("instruction cache index %" PRIu32 " << %" PRIu32 ", way %" PRIu32 " << %" PRIu32,
					cache->arch[cl].i_size.index,
					cache->arch[cl].i_size.index_shift,
					cache->arch[cl].i_size.way,
					cache->arch[cl].i_size.way_shift);

			LOG_DEBUG("cacheline %" PRIu32 " bytes %" PRIu32 " KBytes asso %" PRIu32 " ways",
					cache->arch[cl].i_size.linelen,
					cache->arch[cl].i_size.cachesize,
					cache->arch[cl].i_size.associativity);
		}

		cache->arch[cl].ctype = ctype;
	}

	/*  restore selected cache  */
	dpm->instr_write_data_r0(dpm,
			armv8_opcode(armv8, WRITE_REG_CSSELR), csselr);
	if (retval != ERROR_OK)
		goto done;

	armv8->armv8_mmu.armv8_cache.info = 1;

	/*  if no l2 cache initialize l1 data cache flush function function */
	if (armv8->armv8_mmu.armv8_cache.flush_all_data_cache == NULL) {
		armv8->armv8_mmu.armv8_cache.display_cache_info =
			armv8_handle_inner_cache_info_command;
		armv8->armv8_mmu.armv8_cache.flush_all_data_cache =
			armv8_flush_all_data;
	}

done:
	armv8_dpm_modeswitch(dpm, ARM_MODE_ANY);
	dpm->finish(dpm);
	return retval;

}
