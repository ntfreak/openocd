/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2010 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
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

#include "arm946e.h"
#include "target_type.h"
#include "arm_opcodes.h"

#include "breakpoints.h"

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

#define NB_CACHE_WAYS 4

#define CP15_CTL		0x02
#define CP15_CTL_DCACHE (1<<2)
#define CP15_CTL_ICACHE (1<<12)

/**
 * flag to give info about cache manipulation during debug :
 * "0"	-	cache lines are invalidated "on the fly", for affected addresses.
 *			This is preferred from performance point of view.
 * "1"	-	cache is invalidated and switched off on debug_entry, and switched back on on restore.
 *			It is kept off during debugging.
 */
static uint8_t arm946e_preserve_cache;

static int arm946e_post_debug_entry(struct target *target);
static void arm946e_pre_restore_context(struct target *target);
static int arm946e_read_cp15(struct target *target, int reg_addr, uint32_t *value);

int arm946e_init_arch_info(struct target *target,
	struct arm946e_common *arm946e,
	struct jtag_tap *tap)
{
	struct arm7_9_common *arm7_9 = &arm946e->arm7_9_common;

	/* initialize arm7/arm9 specific info (including armv4_5) */
	arm9tdmi_init_arch_info(target, arm7_9, tap);

	arm946e->common_magic = ARM946E_COMMON_MAGIC;

	/**
	 * The ARM946E-S implements the ARMv5TE architecture which
	 * has the BKPT instruction, so we don't have to use a watchpoint comparator
	 */
	arm7_9->arm_bkpt = ARMV5_BKPT(0x0);
	arm7_9->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;


	arm7_9->post_debug_entry = arm946e_post_debug_entry;
	arm7_9->pre_restore_context = arm946e_pre_restore_context;

	/**
	 * disabling linefills leads to lockups, so keep them enabled for now
	 * this doesn't affect correctness, but might affect timing issues, if
	 * important data is evicted from the cache during the debug session
	 */
	arm946e_preserve_cache = 0;

	/* override hw single-step capability from ARM9TDMI */
	/* arm7_9->has_single_step = 1; */

	return ERROR_OK;
}

static int arm946e_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm946e_common *arm946e = calloc(1, sizeof(struct arm946e_common));

	arm946e_init_arch_info(target, arm946e, target->tap);

	return ERROR_OK;
}

static void arm946e_deinit_target(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm946e_common *arm946e = target_to_arm946(target);

	arm7_9_deinit(target);
	arm_free_reg_cache(arm);
	free(arm946e);
}

static int arm946e_verify_pointer(struct command_invocation *cmd,
	struct arm946e_common *arm946e)
{
	if (arm946e->common_magic != ARM946E_COMMON_MAGIC) {
		command_print(cmd, "target is not an ARM946");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/*
 * Update cp15_control_reg, saved on debug_entry.
 */
static void arm946e_update_cp15_caches(struct target *target, uint32_t value)
{
	struct arm946e_common *arm946e = target_to_arm946(target);
	arm946e->cp15_control_reg = (arm946e->cp15_control_reg & ~(CP15_CTL_DCACHE|CP15_CTL_ICACHE))
		| (value & (CP15_CTL_DCACHE|CP15_CTL_ICACHE));
}

/*
 * REVISIT:  The "read_cp15" and "write_cp15" commands could hook up
 * to eventual mrc() and mcr() routines ... the reg_addr values being
 * constructed (for CP15 only) from Opcode_1, Opcode_2, and CRn values.
 * See section 7.3 of the ARM946E-S TRM.
 */
static int arm946e_read_cp15(struct target *target, int reg_addr, uint32_t *value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct scan_field fields[3];
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 0;

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	/* REVISIT: table 7-2 shows that bits 31-31 need to be
	 * specified for accessing BIST registers ...
	 */
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

	fields[0].in_value = (uint8_t *)value;
	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

	jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)value);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, *value);
#endif

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int arm946e_write_cp15(struct target *target, int reg_addr, uint32_t value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct scan_field fields[3];
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 1;
	uint8_t value_buf[4];

	buf_set_u32(value_buf, 0, 32, value);

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = value_buf;
	fields[0].in_value = NULL;

	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, value);
#endif

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

#define GET_ICACHE_SIZE  6
#define GET_DCACHE_SIZE 18

/*
 * \param target struct target pointer
 * \param idsel  select GET_ICACHE_SIZE or GET_DCACHE_SIZE
 * \returns      cache size, given in bytes
 */
static uint32_t arm946e_cp15_get_csize(struct target *target, int idsel)
{
	struct arm946e_common *arm946e = target_to_arm946(target);
	uint32_t csize = arm946e->cp15_cache_info;
	if (csize == 0) {
		if (arm946e_read_cp15(target, 0x01, &csize) == ERROR_OK)
			arm946e->cp15_cache_info = csize;
	}
	if (csize & (1<<(idsel-4)))	/* cache absent */
		return 0;
	csize = (csize >> idsel) & 0x0F;
	return csize ? 1 << (12 + (csize-3)) : 0;
}

static uint32_t arm946e_invalidate_whole_dcache(struct target *target)
{
	uint32_t csize = arm946e_cp15_get_csize(target, GET_DCACHE_SIZE);
	if (csize == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* One line (index) is 32 bytes (8 words) long, 4-way assoc
	 * ARM DDI 0201D, Section 3.3.5
	 */
	int nb_idx = (csize / (4*8*NB_CACHE_WAYS));	/* gives nb of lines (indexes) in the cache */

	/* Loop for all segments (i.e. ways) */
	uint32_t seg;
	for (seg = 0; seg < NB_CACHE_WAYS; seg++) {
		/* Loop for all indexes */
		int idx;
		for (idx = 0; idx < nb_idx; idx++) {
			/* Form and write cp15 index (segment + line idx) */
			uint32_t cp15_idx = seg << 30 | idx << 5;
			int retval = arm946e_write_cp15(target, 0x3a, cp15_idx);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR writing index");
				return retval;
			}

			/* Read dtag */
			uint32_t dtag;
			retval = arm946e_read_cp15(target, 0x16, &dtag);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR reading dtag");
				return retval;
			}

			/* Check cache line VALID bit */
			if (!(dtag >> 4 & 0x1))
				continue;

			/* Clean data cache line */
			retval = arm946e_write_cp15(target, 0x35, 0x1);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR cleaning cache line");
				return retval;
			}

			/* Flush data cache line */
			retval = arm946e_write_cp15(target, 0x1a, 0x1);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR flushing cache line");
				return retval;
			}
		}
	}

	return ERROR_OK;
}

static uint32_t arm946e_invalidate_whole_icache(struct target *target)
{
	/* Check cache presence before flushing - avoid undefined behavior */
	uint32_t csize = arm946e_cp15_get_csize(target, GET_ICACHE_SIZE);
	if (csize == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	LOG_DEBUG("FLUSHING I$");
	/**
	 *  Invalidate (flush) I$
	 *  mcr	15, 0, r0, cr7, cr5, {0}
	 */
	int retval = arm946e_write_cp15(target, 0x0f, 0x1);
	if (retval != ERROR_OK) {
		LOG_DEBUG("ERROR flushing I$");
		return retval;
	}

	return ERROR_OK;
}

static int arm946e_post_debug_entry(struct target *target)
{
	uint32_t ctr_reg = 0x0;
	uint32_t retval = ERROR_OK;
	struct arm946e_common *arm946e = target_to_arm946(target);

	/* See if CACHES are enabled, and save that info
	 * in the context bits, so that arm946e_pre_restore_context() can use them */
	arm946e_read_cp15(target, CP15_CTL, &ctr_reg);

	/* Save control reg in the context */
	arm946e->cp15_control_reg = ctr_reg;

	if (arm946e_preserve_cache) {
		if (ctr_reg & CP15_CTL_DCACHE) {
			/* Clean and flush D$ */
			arm946e_invalidate_whole_dcache(target);

			/* Disable D$ */
			ctr_reg &= ~CP15_CTL_DCACHE;
		}

		if (ctr_reg & CP15_CTL_ICACHE) {
			/* Flush I$ */
			arm946e_invalidate_whole_icache(target);

			/* Disable I$ */
			ctr_reg &= ~CP15_CTL_ICACHE;
		}

		/* Write the new configuration */
		retval = arm946e_write_cp15(target, CP15_CTL, ctr_reg);
		if (retval != ERROR_OK) {
			LOG_DEBUG("ERROR disabling cache");
			return retval;
		}
	}	/* if preserve_cache */

	return ERROR_OK;
}

static void arm946e_pre_restore_context(struct target *target)
{
	uint32_t ctr_reg = 0x0;
	uint32_t retval;

	if (arm946e_preserve_cache) {
		struct arm946e_common *arm946e = target_to_arm946(target);
		/* Get the contents of the CTR reg */
		arm946e_read_cp15(target, CP15_CTL, &ctr_reg);

		/**
		 * Read-modify-write CP15 control
		 * to reenable I/D-cache operation
		 * NOTE: It is not possible to disable cache by CP15.
		 * if arm946e_preserve_cache debugging flag enabled.
		 */
		ctr_reg |= arm946e->cp15_control_reg & (CP15_CTL_DCACHE|CP15_CTL_ICACHE);

		/* Write the new configuration */
		retval = arm946e_write_cp15(target, CP15_CTL, ctr_reg);
		if (retval != ERROR_OK)
			LOG_DEBUG("ERROR enabling cache");
	}	/* if preserve_cache */
}

static uint32_t arm946e_invalidate_dcache(struct target *target, uint32_t address,
	uint32_t size, uint32_t count)
{
	uint32_t cur_addr = 0x0;
	uint32_t cp15_idx, set, way, dtag;
	uint32_t i = 0;
	int retval;

	for (i = 0; i < count*size; i++) {
		cur_addr = address + i;


		set = (cur_addr >> 5) & 0xff;	/* set field is 8 bits long */

		for (way = 0; way < NB_CACHE_WAYS; way++) {
			/**
			 * Find if the affected address is kept in the cache.
			 * Because JTAG Scan Chain 15 offers limited approach,
			 * we have to loop through all cache ways (segments) and
			 * read cache tags, then compare them with with address.
			 */

			/* Form and write cp15 index (segment + line idx) */
			cp15_idx = way << 30 | set << 5;
			retval = arm946e_write_cp15(target, 0x3a, cp15_idx);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR writing index");
				return retval;
			}

			/* Read dtag */
			retval = arm946e_read_cp15(target, 0x16, &dtag);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR reading dtag");
				return retval;
			}

			/* Check cache line VALID bit */
			if (!(dtag >> 4 & 0x1))
				continue;

			/* If line is valid and corresponds to affected address - invalidate it */
			if (dtag >> 5 == cur_addr >> 5) {
				/* Clean data cache line */
				retval = arm946e_write_cp15(target, 0x35, 0x1);
				if (retval != ERROR_OK) {
					LOG_DEBUG("ERROR cleaning cache line");
					return retval;
				}

				/* Flush data cache line */
				retval = arm946e_write_cp15(target, 0x1c, 0x1);
				if (retval != ERROR_OK) {
					LOG_DEBUG("ERROR flushing cache line");
					return retval;
				}

				break;
			}
		}	/* loop through all 4 ways */
	}	/* loop through all addresses */

	return ERROR_OK;
}

static uint32_t arm946e_invalidate_icache(struct target *target, uint32_t address,
	uint32_t size, uint32_t count)
{
	uint32_t cur_addr = 0x0;
	uint32_t cp15_idx, set, way, itag;
	uint32_t i = 0;
	int retval;

	for (i = 0; i < count*size; i++) {
		cur_addr = address + i;

		set = (cur_addr >> 5) & 0xff;	/* set field is 8 bits long */

		for (way = 0; way < NB_CACHE_WAYS; way++) {
			/* Form and write cp15 index (segment + line idx) */
			cp15_idx = way << 30 | set << 5;
			retval = arm946e_write_cp15(target, 0x3a, cp15_idx);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR writing index");
				return retval;
			}

			/* Read itag */
			retval = arm946e_read_cp15(target, 0x17, &itag);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR reading itag");
				return retval;
			}

			/* Check cache line VALID bit */
			if (!(itag >> 4 & 0x1))
				continue;

			/* If line is valid and corresponds to affected address - invalidate it */
			if (itag >> 5 == cur_addr >> 5) {
				/* Flush I$ line */
				retval = arm946e_write_cp15(target, 0x1d, 0x0);
				if (retval != ERROR_OK) {
					LOG_DEBUG("ERROR flushing cache line");
					return retval;
				}

				break;
			}
		}	/* way loop */
	}	/* addr loop */

	return ERROR_OK;
}

/** Writes a buffer, in the specified word size, with current MMU settings. */
static int arm946e_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;

	LOG_DEBUG("-");

	struct arm946e_common *arm946e = target_to_arm946(target);
	/* Invalidate D$ if it is ON */
	if (!arm946e_preserve_cache && (arm946e->cp15_control_reg & CP15_CTL_DCACHE))
		arm946e_invalidate_dcache(target, address, size, count);

	/**
	 * Write memory
	 */
	retval = arm7_9_write_memory_opt(target, address, size, count, buffer);
	if (retval != ERROR_OK)
		return retval;

	/* *
	 * Invalidate I$ if it is ON.
	 *
	 * D$ has been cleaned and flushed before mem write thus forcing it to behave like write-through,
	 * because arm7_9_write_memory() has seen non-valid bit in D$
	 * and wrote data into physical RAM (without touching or allocating the cache line).
	 * From ARM946ES Technical Reference Manual we can see that it uses "allocate on read-miss"
	 * policy for both I$ and D$ (Chapter 3.2 and 3.3)
	 *
	 * Explanation :
	 * "ARM system developer's guide: designing and optimizing system software" by
	 * Andrew N. Sloss, Dominic Symes and Chris Wright,
	 * Chapter 12.3.3 Allocating Policy on a Cache Miss :
	 * A read allocate on cache miss policy allocates a cache line only during a read from main memory.
	 * If the victim cache line contains valid data, then it is written to main memory before the cache line
	 * is filled with new data.
	 * Under this strategy, a write of new data to memory does not update the contents of the cache memory
	 * unless a cache line was allocated on a previous read from main memory.
	 * If the cache line contains valid data, then the write updates the cache and may update the main memory if
	 * the cache write policy is write-through.
	 * If the data is not in the cache, the controller writes to main memory only.
	 */
	if (!arm946e_preserve_cache && (arm946e->cp15_control_reg & CP15_CTL_ICACHE))
		arm946e_invalidate_icache(target, address, size, count);

	return ERROR_OK;

}

static int arm946e_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;

	LOG_DEBUG("-");

	retval = arm7_9_read_memory(target, address, size, count, buffer);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(arm946e_handle_cp15)
{
	/* one or two arguments, access a single register (write if second argument is given) */
	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	struct arm946e_common *arm946e = target_to_arm946(target);
	int retval = arm946e_verify_pointer(CMD, arm946e);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

	if (CMD_ARGC == 1) {
		uint32_t value;
		retval = arm946e_read_cp15(target, address, &value);
		if (retval != ERROR_OK) {
			command_print(CMD, "%s cp15 reg %" PRIu32 " access failed", target_name(target), address);
			return retval;
		}
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		/* Return value in hex format */
		command_print(CMD, "0x%08" PRIx32, value);
	} else if (CMD_ARGC == 2) {
		uint32_t value;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

		retval = arm946e_write_cp15(target, address, value);
		if (retval != ERROR_OK) {
			command_print(CMD, "%s cp15 reg %" PRIu32 " access failed", target_name(target), address);
			return retval;
		}
		if (address == CP15_CTL)
			arm946e_update_cp15_caches(target, value);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(arm946e_handle_idcache)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct arm946e_common *arm946e = target_to_arm946(target);

	retval = arm946e_verify_pointer(CMD, arm946e);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_TARGET_NOT_HALTED;
	}

	bool icache = (strcmp(CMD_NAME, "icache") == 0);
	uint32_t csize = arm946e_cp15_get_csize(target, icache ? GET_ICACHE_SIZE : GET_DCACHE_SIZE) / 1024;
	if (CMD_ARGC == 0) {
		bool  bena = ((arm946e->cp15_control_reg & (icache ? CP15_CTL_ICACHE : CP15_CTL_DCACHE)) != 0)
			  && (arm946e->cp15_control_reg & 0x1);
		if (csize == 0)
			command_print(CMD, "%s-cache absent", icache ? "I" : "D");
		else
			command_print(CMD, "%s-cache size: %" PRIu32 "K, %s",
				      icache ? "I" : "D", csize, bena ? "enabled" : "disabled");
		return ERROR_OK;
	}

	bool flush = false;
	bool enable = false;
	retval = command_parse_bool_arg(CMD_ARGV[0], &enable);
	if (retval == ERROR_COMMAND_SYNTAX_ERROR) {
		if (strcmp(CMD_ARGV[0], "flush") == 0) {
			flush = true;
			retval = ERROR_OK;
		} else
			return retval;
	}

	/* Do not invalidate or change state, if cache is absent */
	if (csize == 0) {
		command_print(CMD, "%s-cache absent, '%s' operation undefined", icache ? "I" : "D", CMD_ARGV[0]);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* NOTE: flushing entire cache will not preserve lock-down cache regions */
	if (icache) {
		if ((arm946e->cp15_control_reg & CP15_CTL_ICACHE) && !enable)
			retval = arm946e_invalidate_whole_icache(target);
	} else {
		if ((arm946e->cp15_control_reg & CP15_CTL_DCACHE) && !enable)
			retval = arm946e_invalidate_whole_dcache(target);
	}

	if (retval != ERROR_OK || flush)
		return retval;

	uint32_t value;
	retval = arm946e_read_cp15(target, CP15_CTL, &value);
	if (retval != ERROR_OK)
		return retval;

	uint32_t vnew = value;
	uint32_t cmask = icache ? CP15_CTL_ICACHE : CP15_CTL_DCACHE;
	if (enable) {
		if ((value & 0x1) == 0)
			LOG_WARNING("arm946e: MPU must be enabled for cache to operate");
		vnew |= cmask;
	} else
		vnew &= ~cmask;

	if (vnew == value)
		return ERROR_OK;

	retval = arm946e_write_cp15(target, CP15_CTL, vnew);
	if (retval != ERROR_OK)
		return retval;

	arm946e_update_cp15_caches(target, vnew);
	return ERROR_OK;
}

static const struct command_registration arm946e_exec_command_handlers[] = {
	{
		.name = "cp15",
		.handler = arm946e_handle_cp15,
		.mode = COMMAND_EXEC,
		.usage = "regnum [value]",
		.help = "read/modify cp15 register",
	},
	{
		.name = "icache",
		.handler = arm946e_handle_idcache,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable'|'flush']",
		.help = "I-cache info and operations",
	},
	{
		.name = "dcache",
		.handler = arm946e_handle_idcache,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable'|'flush']",
		.help = "D-cache info and operations",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arm946e_command_handlers[] = {
	{
		.chain = arm9tdmi_command_handlers,
	},
	{
		.name = "arm946e",
		.mode = COMMAND_ANY,
		.help = "arm946e command group",
		.usage = "",
		.chain = arm946e_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM946 targets. */
struct target_type arm946e_target = {
	.name = "arm946e",

	.poll = arm7_9_poll,
	.arch_state = arm_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm7_9_soft_reset_halt,

	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	/* .read_memory = arm7_9_read_memory, */
	/* .write_memory = arm7_9_write_memory, */
	.read_memory = arm946e_read_memory,
	.write_memory = arm946e_write_memory,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	/* .add_breakpoint = arm946e_add_breakpoint, */
	/* .remove_breakpoint = arm946e_remove_breakpoint, */

	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.commands = arm946e_command_handlers,
	.target_create = arm946e_target_create,
	.init_target = arm9tdmi_init_target,
	.deinit_target = arm946e_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
