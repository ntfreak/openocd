/***************************************************************************
 *    Copyright (C) 2009 by David Brownell                                 *
 *                                                                         *
 *    Copyright (C) ST-Ericsson SA 2011 michel.jaouen@stericsson.com       *
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

#include <helper/replacements.h>

#include "armv7a.h"
#include "armv7a_mmu.h"
#include "arm_disassembler.h"

#include "register.h"
#include <helper/binarybuffer.h>
#include <helper/command.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "arm_opcodes.h"
#include "target.h"
#include "target_type.h"

static void armv7a_show_fault_registers(struct target *target)
{
	uint32_t dfsr, ifsr, dfar, ifar;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return;

	/* ARMV4_5_MRC(cpnum, op1, r0, CRn, CRm, op2) */

	/* c5/c0 - {data, instruction} fault status registers */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 5, 0, 0),
			&dfsr);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 5, 0, 1),
			&ifsr);
	if (retval != ERROR_OK)
		goto done;

	/* c6/c0 - {data, instruction} fault address registers */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 6, 0, 0),
			&dfar);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 6, 0, 2),
			&ifar);
	if (retval != ERROR_OK)
		goto done;

	LOG_USER("Data fault registers        DFSR: %8.8" PRIx32
		", DFAR: %8.8" PRIx32, dfsr, dfar);
	LOG_USER("Instruction fault registers IFSR: %8.8" PRIx32
		", IFAR: %8.8" PRIx32, ifsr, ifar);

done:
	/* (void) */ dpm->finish(dpm);
}


/*  retrieve main id register  */
static int armv7a_read_midr(struct target *target)
{
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t midr;
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/* MRC p15,0,<Rd>,c0,c0,0; read main id register*/

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 0, 0, 0),
			&midr);
	if (retval != ERROR_OK)
		goto done;

	armv7a->rev = (midr & 0xf);
	armv7a->partnum = (midr >> 4) & 0xfff;
	armv7a->arch = (midr >> 16) & 0xf;
	armv7a->variant = (midr >> 20) & 0xf;
	armv7a->implementor = (midr >> 24) & 0xff;
	LOG_INFO("%s rev %" PRIx32 ", partnum %" PRIx32 ", arch %" PRIx32
			 ", variant %" PRIx32 ", implementor %" PRIx32,
		 target->cmd_name,
		 armv7a->rev,
		 armv7a->partnum,
		 armv7a->arch,
		 armv7a->variant,
		 armv7a->implementor);

done:
	dpm->finish(dpm);
	return retval;
}

int armv7a_read_ttbcr(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t ttbcr, ttbcr_n;
	int ttbidx;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/*  MRC p15,0,<Rt>,c2,c0,2 ; Read CP15 Translation Table Base Control Register*/
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 2, 0, 2),
			&ttbcr);
	if (retval != ERROR_OK)
		goto done;

	LOG_DEBUG("ttbcr %" PRIx32, ttbcr);

	ttbcr_n = ttbcr & 0x7;
	armv7a->armv7a_mmu.ttbcr = ttbcr;
	armv7a->armv7a_mmu.cached = 1;

	for (ttbidx = 0; ttbidx < 2; ttbidx++) {
		/*  MRC p15,0,<Rt>,c2,c0,ttbidx */
		retval = dpm->instr_read_data_r0(dpm,
				ARMV4_5_MRC(15, 0, 0, 2, 0, ttbidx),
				&armv7a->armv7a_mmu.ttbr[ttbidx]);
		if (retval != ERROR_OK)
			goto done;
	}

	/*
	 * ARM Architecture Reference Manual (ARMv7-A and ARMv7-R edition),
	 * document # ARM DDI 0406C
	 */
	armv7a->armv7a_mmu.ttbr_range[0]  = 0xffffffff >> ttbcr_n;
	armv7a->armv7a_mmu.ttbr_range[1] = 0xffffffff;
	armv7a->armv7a_mmu.ttbr_mask[0] = 0xffffffff << (14 - ttbcr_n);
	armv7a->armv7a_mmu.ttbr_mask[1] = 0xffffffff << 14;
	armv7a->armv7a_mmu.cached = 1;

	retval = armv7a_read_midr(target);
	if (retval != ERROR_OK)
		goto done;

	/* FIXME: why this special case based on part number? */
	if ((armv7a->partnum & 0xf) == 0) {
		/*  ARM DDI 0344H , ARM DDI 0407F */
		armv7a->armv7a_mmu.ttbr_mask[0]  = 7 << (32 - ttbcr_n);
	}

	LOG_DEBUG("ttbr1 %s, ttbr0_mask %" PRIx32 " ttbr1_mask %" PRIx32,
		  (ttbcr_n != 0) ? "used" : "not used",
		  armv7a->armv7a_mmu.ttbr_mask[0],
		  armv7a->armv7a_mmu.ttbr_mask[1]);

done:
	dpm->finish(dpm);
	return retval;
}

/* FIXME: remove it */
static int armv7a_l2x_cache_init(struct target *target, uint32_t base, uint32_t way)
{
	struct armv7a_l2x_cache *l2x_cache;
	struct target_list *head = target->head;
	struct target *curr;

	struct armv7a_common *armv7a = target_to_armv7a(target);
	l2x_cache = calloc(1, sizeof(struct armv7a_l2x_cache));
	l2x_cache->base = base;
	l2x_cache->way = way;
	/*LOG_INFO("cache l2 initialized base %x  way %d",
	l2x_cache->base,l2x_cache->way);*/
	if (armv7a->armv7a_mmu.armv7a_cache.outer_cache)
		LOG_INFO("outer cache already initialized\n");
	armv7a->armv7a_mmu.armv7a_cache.outer_cache = l2x_cache;
	/*  initialize all target in this cluster (smp target)
	 *  l2 cache must be configured after smp declaration */
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if (curr != target) {
			armv7a = target_to_armv7a(curr);
			if (armv7a->armv7a_mmu.armv7a_cache.outer_cache)
				LOG_ERROR("smp target : outer cache already initialized\n");
			armv7a->armv7a_mmu.armv7a_cache.outer_cache = l2x_cache;
		}
		head = head->next;
	}
	return JIM_OK;
}

/* FIXME: remove it */
COMMAND_HANDLER(handle_cache_l2x)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t base, way;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* command_print(CMD, "%s %s", CMD_ARGV[0], CMD_ARGV[1]); */
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], way);

	/* AP address is in bits 31:24 of DP_SELECT */
	armv7a_l2x_cache_init(target, base, way);

	return ERROR_OK;
}

int armv7a_handle_cache_info_command(struct command_invocation *cmd,
	struct armv7a_cache_common *armv7a_cache)
{
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a_cache->outer_cache);

	int cl;

	if (armv7a_cache->info == -1) {
		command_print(cmd, "cache not yet identified");
		return ERROR_OK;
	}

	for (cl = 0; cl < armv7a_cache->loc; cl++) {
		struct armv7a_arch_cache *arch = &(armv7a_cache->arch[cl]);

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

	if (l2x_cache != NULL)
		command_print(cmd, "Outer unified cache Base Address 0x%" PRIx32 ", %" PRIu32 " ways",
			l2x_cache->base, l2x_cache->way);

	return ERROR_OK;
}

/*  retrieve core id cluster id  */
static int armv7a_read_mpidr(struct target *target)
{
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t mpidr;
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/* MRC p15,0,<Rd>,c0,c0,5; read Multiprocessor ID register*/

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 0, 0, 5),
			&mpidr);
	if (retval != ERROR_OK)
		goto done;

	/* Is register in Multiprocessing Extensions register format? */
	if (mpidr & MPIDR_MP_EXT) {
		LOG_DEBUG("%s: MPIDR 0x%" PRIx32, target_name(target), mpidr);
		armv7a->multi_processor_system = (mpidr >> 30) & 1;
		armv7a->multi_threading_processor = (mpidr >> 24) & 1;
		armv7a->level2_id = (mpidr >> 16) & 0xf;
		armv7a->cluster_id = (mpidr >> 8) & 0xf;
		armv7a->cpu_id = mpidr & 0xf;
		LOG_INFO("%s: MPIDR level2 %x, cluster %x, core %x, %s, %s",
			target_name(target),
			armv7a->level2_id,
			armv7a->cluster_id,
			armv7a->cpu_id,
			armv7a->multi_processor_system == 0 ? "multi core" : "mono core",
			armv7a->multi_threading_processor == 1 ? "SMT" : "no SMT");

	} else
		LOG_ERROR("MPIDR not in multiprocessor format");

done:
	dpm->finish(dpm);
	return retval;


}

static int get_cache_info(struct arm_dpm *dpm, int cl, int ct, uint32_t *cache_reg)
{
	int retval = ERROR_OK;

	/*  select cache level */
	retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_MCR(15, 2, 0, 0, 0, 0),
			(cl << 1) | (ct == 1 ? 1 : 0));
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 1, 0, 0, 0, 0),
			cache_reg);
 done:
	return retval;
}

static struct armv7a_cachesize decode_cache_reg(uint32_t cache_reg)
{
	struct armv7a_cachesize size;
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

int armv7a_identify_cache(struct target *target)
{
	/*  read cache descriptor */
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t csselr, clidr, ctr;
	uint32_t cache_reg;
	int cl, ctype;
	struct armv7a_cache_common *cache =
		&(armv7a->armv7a_mmu.armv7a_cache);

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* retrieve CTR
	 * mrc p15, 0, r0, c0, c0, 1		@ read ctr */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 0, 0, 1),
			&ctr);
	if (retval != ERROR_OK)
		goto done;

	cache->iminline = 4UL << (ctr & 0xf);
	cache->dminline = 4UL << ((ctr & 0xf0000) >> 16);
	LOG_DEBUG("ctr %" PRIx32 " ctr.iminline %" PRIu32 " ctr.dminline %" PRIu32,
		 ctr, cache->iminline, cache->dminline);

	/*  retrieve CLIDR
	 *  mrc	p15, 1, r0, c0, c0, 1		@ read clidr */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 1, 0, 0, 0, 1),
			&clidr);
	if (retval != ERROR_OK)
		goto done;

	cache->loc = (clidr & 0x7000000) >> 24;
	LOG_DEBUG("Number of cache levels to PoC %" PRId32, cache->loc);

	/*  retrieve selected cache for later restore
	 *  MRC p15, 2,<Rd>, c0, c0, 0; Read CSSELR */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
			&csselr);
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
		ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
		csselr);

	if (retval != ERROR_OK)
		goto done;

	/*  if no l2 cache initialize l1 data cache flush function function */
	if (armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache == NULL) {
		armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache =
			armv7a_cache_auto_flush_all_data;
	}

	armv7a->armv7a_mmu.armv7a_cache.info = 1;
done:
	dpm->finish(dpm);
	armv7a_read_mpidr(target);
	return retval;

}

static int armv7a_setup_semihosting(struct target *target, int enable)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t vcr;
	int ret;

	ret = mem_ap_read_atomic_u32(armv7a->debug_ap,
					 armv7a->debug_base + CPUDBG_VCR,
					 &vcr);
	if (ret < 0) {
		LOG_ERROR("Failed to read VCR register\n");
		return ret;
	}

	if (enable)
		vcr |= DBG_VCR_SVC_MASK;
	else
		vcr &= ~DBG_VCR_SVC_MASK;

	ret = mem_ap_write_atomic_u32(armv7a->debug_ap,
					  armv7a->debug_base + CPUDBG_VCR,
					  vcr);
	if (ret < 0)
		LOG_ERROR("Failed to write VCR register\n");

	return ret;
}

int armv7a_init_arch_info(struct target *target, struct armv7a_common *armv7a)
{
	struct arm *arm = &armv7a->arm;
	arm->arch_info = armv7a;
	target->arch_info = &armv7a->arm;
	arm->setup_semihosting = armv7a_setup_semihosting;
	/*  target is useful in all function arm v4 5 compatible */
	armv7a->arm.target = target;
	armv7a->arm.common_magic = ARM_COMMON_MAGIC;
	armv7a->common_magic = ARMV7_COMMON_MAGIC;
	armv7a->armv7a_mmu.armv7a_cache.info = -1;
	armv7a->armv7a_mmu.armv7a_cache.outer_cache = NULL;
	armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache = NULL;
	armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled = 1;
	return ERROR_OK;
}

int armv7a_arch_state(struct target *target)
{
	static const char *state[] = {
		"disabled", "enabled"
	};

	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;

	if (armv7a->common_magic != ARMV7_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-ARMv7A target");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	arm_arch_state(target);

	if (armv7a->is_armv7r) {
		LOG_USER("D-Cache: %s, I-Cache: %s",
			state[armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled],
			state[armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled]);
	} else {
		LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s",
			state[armv7a->armv7a_mmu.mmu_enabled],
			state[armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled],
			state[armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled]);
	}

	if (arm->core_mode == ARM_MODE_ABT)
		armv7a_show_fault_registers(target);
	if (target->debug_reason == DBG_REASON_WATCHPOINT)
		LOG_USER("Watchpoint triggered at PC %#08x",
			(unsigned) armv7a->dpm.wp_pc);

	return ERROR_OK;
}

static const struct command_registration l2_cache_commands[] = {
	{
		.name = "l2x",
		.handler = handle_cache_l2x,
		.mode = COMMAND_EXEC,
		.help = "configure l2x cache",
		.usage = "[base_addr] [number_of_way]",
	},
	COMMAND_REGISTRATION_DONE

};

static const struct command_registration l2x_cache_command_handlers[] = {
	{
		.name = "cache_config",
		.mode = COMMAND_EXEC,
		.help = "cache configuration for a target",
		.usage = "",
		.chain = l2_cache_commands,
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration armv7a_command_handlers[] = {
	{
		.chain = l2x_cache_command_handlers,
	},
	{
		.chain = arm7a_cache_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
