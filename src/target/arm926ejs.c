/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008,2009 by Øyvind Harboe                         *
 *   oyvind.harboe@zylin.com                                               *
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

#include "arm926ejs.h"
#include <helper/time_support.h>
#include "target_type.h"
#include "register.h"
#include "arm_opcodes.h"


/*
 * The ARM926 is built around the ARM9EJ-S core, and most JTAG docs
 * are in the ARM9EJ-S Technical Reference Manual (ARM DDI 0222B) not
 * the ARM926 manual (ARM DDI 0198E).  The scan chains are:
 *
 *   1 ... core debugging
 *   2 ... EmbeddedICE
 *   3 ... external boundary scan (SoC-specific, unused here)
 *   6 ... ETM
 *   15 ... coprocessor 15
 */

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

#define ARM926EJS_CP15_ADDR(opcode_1, opcode_2, CRn, CRm) ((opcode_1 << 11) | (opcode_2 << 8) | (CRn << 4) | (CRm << 0))

static int arm926ejs_cp15_read(struct target *target, uint32_t op1, uint32_t op2,
		uint32_t CRn, uint32_t CRm, uint32_t *value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	uint32_t address = ARM926EJS_CP15_ADDR(op1, op2, CRn, CRm);
	struct scan_field fields[4];
	uint8_t address_buf[2] = {0, 0};
	uint8_t nr_w_buf = 0;
	uint8_t access_t = 1;

	buf_set_u32(address_buf, 0, 14, address);

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = (uint8_t *)value;

	fields[1].num_bits = 1;
	fields[1].out_value = &access_t;
	fields[1].in_value = &access_t;

	fields[2].num_bits = 14;
	fields[2].out_value = address_buf;
	fields[2].in_value = NULL;

	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);

	int64_t then = timeval_ms();

	for (;;) {
		/* rescan with NOP, to wait for the access to complete */
		access_t = 0;
		nr_w_buf = 0;
		jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);

		jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)value);

		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		if (buf_get_u32(&access_t, 0, 1) == 1)
			break;

		/* 10ms timeout */
		if ((timeval_ms()-then) > 10) {
			LOG_ERROR("cp15 read operation timed out");
			return ERROR_FAIL;
		}
	}

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", address, *value);
#endif

	retval = arm_jtag_set_instr(jtag_info->tap, 0xc, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int arm926ejs_mrc(struct target *target, int cpnum, uint32_t op1,
		uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t *value)
{
	if (cpnum != 15) {
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}
	return arm926ejs_cp15_read(target, op1, op2, CRn, CRm, value);
}

static int arm926ejs_cp15_write(struct target *target, uint32_t op1, uint32_t op2,
		uint32_t CRn, uint32_t CRm, uint32_t value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	uint32_t address = ARM926EJS_CP15_ADDR(op1, op2, CRn, CRm);
	struct scan_field fields[4];
	uint8_t value_buf[4];
	uint8_t address_buf[2] = {0, 0};
	uint8_t nr_w_buf = 1;
	uint8_t access_t = 1;

	buf_set_u32(address_buf, 0, 14, address);
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

	fields[1].num_bits = 1;
	fields[1].out_value = &access_t;
	fields[1].in_value = &access_t;

	fields[2].num_bits = 14;
	fields[2].out_value = address_buf;
	fields[2].in_value = NULL;

	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);

	int64_t then = timeval_ms();

	for (;;) {
		/* rescan with NOP, to wait for the access to complete */
		access_t = 0;
		nr_w_buf = 0;
		jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		if (buf_get_u32(&access_t, 0, 1) == 1)
			break;

		/* 10ms timeout */
		if ((timeval_ms()-then) > 10) {
			LOG_ERROR("cp15 write operation timed out");
			return ERROR_FAIL;
		}
	}

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", address, value);
#endif

	retval = arm_jtag_set_instr(jtag_info->tap, 0xf, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int arm926ejs_mcr(struct target *target, int cpnum, uint32_t op1,
		uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t value)
{
	if (cpnum != 15) {
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}
	return arm926ejs_cp15_write(target, op1, op2, CRn, CRm, value);
}

static int arm926ejs_examine_debug_reason(struct target *target)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct reg *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
	int debug_reason;
	int retval;

	embeddedice_read_reg(dbg_stat);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* Method-Of-Entry (MOE) field */
	debug_reason = buf_get_u32(dbg_stat->value, 6, 4);

	switch (debug_reason) {
		case 0:
			LOG_DEBUG("no *NEW* debug entry (?missed one?)");
			/* ... since last restart or debug reset ... */
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case 1:
			LOG_DEBUG("breakpoint from EICE unit 0");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 2:
			LOG_DEBUG("breakpoint from EICE unit 1");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 3:
			LOG_DEBUG("soft breakpoint (BKPT instruction)");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 4:
			LOG_DEBUG("vector catch breakpoint");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 5:
			LOG_DEBUG("external breakpoint");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 6:
			LOG_DEBUG("watchpoint from EICE unit 0");
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case 7:
			LOG_DEBUG("watchpoint from EICE unit 1");
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case 8:
			LOG_DEBUG("external watchpoint");
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case 9:
			LOG_DEBUG("internal debug request");
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case 10:
			LOG_DEBUG("external debug request");
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case 11:
			LOG_DEBUG("debug re-entry from system speed access");
			/* This is normal when connecting to something that's
			 * already halted, or in some related code paths, but
			 * otherwise is surprising (and presumably wrong).
			 */
			switch (target->debug_reason) {
			case DBG_REASON_DBGRQ:
				break;
			default:
				LOG_ERROR("unexpected -- debug re-entry");
				/* FALLTHROUGH */
			case DBG_REASON_UNDEFINED:
				target->debug_reason = DBG_REASON_DBGRQ;
				break;
			}
			break;
		case 12:
			/* FIX!!!! here be dragons!!! We need to fail here so
			 * the target will interpreted as halted but we won't
			 * try to talk to it right now... a resume + halt seems
			 * to sync things up again. Please send an email to
			 * openocd development mailing list if you have hardware
			 * to donate to look into this problem....
			 */
			LOG_WARNING("WARNING: mystery debug reason MOE = 0xc. Try issuing a resume + halt.");
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		default:
			LOG_WARNING("WARNING: unknown debug reason: 0x%x", debug_reason);
			/* Oh agony! should we interpret this as a halt request or
			 * that the target stopped on it's own accord?
			 */
			target->debug_reason = DBG_REASON_DBGRQ;
			/* if we fail here, we won't talk to the target and it will
			 * be reported to be in the halted state */
			break;
	}

	return ERROR_OK;
}

static int arm926ejs_get_ttb(struct target *target, uint32_t *result)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);
	int retval;
	uint32_t ttb = 0x0;

	retval = arm926ejs->read_cp15(target, 0, 0, 2, 0, &ttb);
	if (retval != ERROR_OK)
		return retval;

	*result = ttb;

	return ERROR_OK;
}

static int arm926ejs_disable_mmu_caches(struct target *target, int mmu,
		int d_u_cache, int i_cache)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);
	uint32_t cp15_control;
	int retval;

	/* read cp15 control register */
	retval = arm926ejs->read_cp15(target, 0, 0, 1, 0, &cp15_control);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (mmu) {
		/* invalidate TLB */
		retval = arm926ejs->write_cp15(target, 0, 0, 8, 7, 0x0);
		if (retval != ERROR_OK)
			return retval;

		cp15_control &= ~0x1U;
	}

	if (d_u_cache) {
		uint32_t debug_override;
		/* read-modify-write CP15 debug override register
		 * to enable "test and clean all" */
		retval = arm926ejs->read_cp15(target, 0, 0, 15, 0, &debug_override);
		if (retval != ERROR_OK)
			return retval;
		debug_override |= 0x80000;
		retval = arm926ejs->write_cp15(target, 0, 0, 15, 0, debug_override);
		if (retval != ERROR_OK)
			return retval;

		/* clean and invalidate DCache */
		retval = arm926ejs->write_cp15(target, 0, 0, 7, 5, 0x0);
		if (retval != ERROR_OK)
			return retval;

		/* write CP15 debug override register
		 * to disable "test and clean all" */
		debug_override &= ~0x80000;
		retval = arm926ejs->write_cp15(target, 0, 0, 15, 0, debug_override);
		if (retval != ERROR_OK)
			return retval;

		cp15_control &= ~0x4U;
	}

	if (i_cache) {
		/* invalidate ICache */
		retval = arm926ejs->write_cp15(target, 0, 0, 7, 5, 0x0);
		if (retval != ERROR_OK)
			return retval;

		cp15_control &= ~0x1000U;
	}

	retval = arm926ejs->write_cp15(target, 0, 0, 1, 0, cp15_control);
	return retval;
}

static int arm926ejs_enable_mmu_caches(struct target *target, int mmu,
		int d_u_cache, int i_cache)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);
	uint32_t cp15_control;
	int retval;

	/* read cp15 control register */
	retval = arm926ejs->read_cp15(target, 0, 0, 1, 0, &cp15_control);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (mmu)
		cp15_control |= 0x1U;

	if (d_u_cache)
		cp15_control |= 0x4U;

	if (i_cache)
		cp15_control |= 0x1000U;

	retval = arm926ejs->write_cp15(target, 0, 0, 1, 0, cp15_control);
	return retval;
}

static int arm926ejs_post_debug_entry(struct target *target)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);
	int retval;

	/* examine cp15 control reg */
	retval = arm926ejs->read_cp15(target, 0, 0, 1, 0, &arm926ejs->cp15_control_reg);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("cp15_control_reg: %8.8" PRIx32 "", arm926ejs->cp15_control_reg);

	if (arm926ejs->armv4_5_mmu.armv4_5_cache.ctype == -1) {
		uint32_t cache_type_reg;
		/* identify caches */
		retval = arm926ejs->read_cp15(target, 0, 1, 0, 0, &cache_type_reg);
		if (retval != ERROR_OK)
			return retval;
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;
		armv4_5_identify_cache(cache_type_reg, &arm926ejs->armv4_5_mmu.armv4_5_cache);
	}

	arm926ejs->armv4_5_mmu.mmu_enabled = (arm926ejs->cp15_control_reg & 0x1U) ? 1 : 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = (arm926ejs->cp15_control_reg & 0x4U) ? 1 : 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled = (arm926ejs->cp15_control_reg & 0x1000U) ? 1 : 0;

	/* save i/d fault status and address register */
	retval = arm926ejs->read_cp15(target, 0, 0, 5, 0, &arm926ejs->d_fsr);
	if (retval != ERROR_OK)
		return retval;
	retval = arm926ejs->read_cp15(target, 0, 1, 5, 0, &arm926ejs->i_fsr);
	if (retval != ERROR_OK)
		return retval;
	retval = arm926ejs->read_cp15(target, 0, 0, 6, 0, &arm926ejs->d_far);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("D FSR: 0x%8.8" PRIx32 ", D FAR: 0x%8.8" PRIx32 ", I FSR: 0x%8.8" PRIx32 "",
		arm926ejs->d_fsr, arm926ejs->d_far, arm926ejs->i_fsr);

	uint32_t cache_dbg_ctrl;

	/* read-modify-write CP15 cache debug control register
	 * to disable I/D-cache linefills and force WT */
	retval = arm926ejs->read_cp15(target, 7, 0, 15, 0, &cache_dbg_ctrl);
	if (retval != ERROR_OK)
		return retval;
	cache_dbg_ctrl |= 0x7;
	retval = arm926ejs->write_cp15(target, 7, 0, 15, 0, cache_dbg_ctrl);
	return retval;
}

static void arm926ejs_pre_restore_context(struct target *target)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	/* restore i/d fault status and address register */
	arm926ejs->write_cp15(target, 0, 0, 5, 0, arm926ejs->d_fsr);
	arm926ejs->write_cp15(target, 0, 1, 5, 0, arm926ejs->i_fsr);
	arm926ejs->write_cp15(target, 0, 0, 6, 0, arm926ejs->d_far);

	uint32_t cache_dbg_ctrl;

	/* read-modify-write CP15 cache debug control register
	 * to reenable I/D-cache linefills and disable WT */
	arm926ejs->read_cp15(target, 7, 0, 15, 0, &cache_dbg_ctrl);
	cache_dbg_ctrl &= ~0x7;
	arm926ejs->write_cp15(target, 7, 0, 15, 0, cache_dbg_ctrl);
}

static const char arm926_not[] = "target is not an ARM926";

static int arm926ejs_verify_pointer(struct command_invocation *cmd,
		struct arm926ejs_common *arm926)
{
	if (arm926->common_magic != ARM926EJS_COMMON_MAGIC) {
		command_print(cmd, arm926_not);
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/** Logs summary of ARM926 state for a halted target. */
int arm926ejs_arch_state(struct target *target)
{
	static const char *state[] = {
		"disabled", "enabled"
	};

	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	if (arm926ejs->common_magic != ARM926EJS_COMMON_MAGIC) {
		LOG_ERROR("BUG: %s", arm926_not);
		return ERROR_TARGET_INVALID;
	}

	arm_arch_state(target);
	LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s",
			 state[arm926ejs->armv4_5_mmu.mmu_enabled],
			 state[arm926ejs->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled],
			 state[arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled]);

	return ERROR_OK;
}

int arm926ejs_soft_reset_halt(struct target *target)
{
	int retval = ERROR_OK;
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm *arm = &arm7_9->arm;
	struct reg *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	int64_t then = timeval_ms();
	int timeout;
	while (!(timeout = ((timeval_ms()-then) > 1000))) {
		if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1) == 0) {
			embeddedice_read_reg(dbg_stat);
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;
		} else
			break;
		if (debug_level >= 1) {
			/* do not eat all CPU, time out after 1 se*/
			alive_sleep(100);
		} else
			keep_alive();
	}
	if (timeout) {
		LOG_ERROR("Failed to halt CPU after 1 sec");
		return ERROR_TARGET_TIMEOUT;
	}

	target->state = TARGET_HALTED;

	/* SVC, ARM state, IRQ and FIQ disabled */
	uint32_t cpsr;

	cpsr = buf_get_u32(arm->cpsr->value, 0, 32);
	cpsr &= ~0xff;
	cpsr |= 0xd3;
	arm_set_cpsr(arm, cpsr);
	arm->cpsr->dirty = true;

	/* start fetching from 0x0 */
	buf_set_u32(arm->pc->value, 0, 32, 0x0);
	arm->pc->dirty = true;
	arm->pc->valid = true;

	retval = arm926ejs_disable_mmu_caches(target, 1, 1, 1);
	if (retval != ERROR_OK)
		return retval;
	arm926ejs->armv4_5_mmu.mmu_enabled = 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	return target_call_event_callbacks(target, TARGET_EVENT_HALTED);
}

/** Writes a buffer, in the specified word size, with current MMU settings. */
int arm926ejs_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	/* FIX!!!! this should be cleaned up and made much more general. The
	 * plan is to write up and test on arm926ejs specifically and
	 * then generalize and clean up afterwards.
	 *
	 *
	 * Also it should be moved to the callbacks that handle breakpoints
	 * specifically and not the generic memory write fn's. See XScale code.
	 **/
	if (arm926ejs->armv4_5_mmu.mmu_enabled && (count == 1) && ((size == 2) || (size == 4))) {
		/* special case the handling of single word writes to bypass MMU
		 * to allow implementation of breakpoints in memory marked read only
		 * by MMU */
		if (arm926ejs->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled) {
			/* flush and invalidate data cache
			 *
			 * MCR p15,0,p,c7,c10,1 - clean cache line using virtual address
			 *
			 */
			retval = arm926ejs->write_cp15(target, 0, 1, 7, 10, address&~0x3);
			if (retval != ERROR_OK)
				return retval;
		}

		target_addr_t pa;
		retval = target->type->virt2phys(target, address, &pa);
		if (retval != ERROR_OK)
			return retval;

		/* write directly to physical memory bypassing any read only MMU bits, etc. */
		retval = armv4_5_mmu_write_physical(target, &arm926ejs->armv4_5_mmu, pa, size, count, buffer);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = arm7_9_write_memory(target, address, size, count, buffer);
		if (retval != ERROR_OK)
			return retval;
	}

	/* If ICache is enabled, we have to invalidate affected ICache lines
	 * the DCache is forced to write-through, so we don't have to clean it here
	 */
	if (arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled) {
		if (count <= 1) {
			/* invalidate ICache single entry with MVA */
			arm926ejs->write_cp15(target, 0, 1, 7, 5, address);
		} else {
			/* invalidate ICache */
			arm926ejs->write_cp15(target, 0, 0, 7, 5, address);
		}
	}

	return retval;
}

static int arm926ejs_write_phys_memory(struct target *target,
		target_addr_t address, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	return armv4_5_mmu_write_physical(target, &arm926ejs->armv4_5_mmu,
			address, size, count, buffer);
}

static int arm926ejs_read_phys_memory(struct target *target,
		target_addr_t address, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	return armv4_5_mmu_read_physical(target, &arm926ejs->armv4_5_mmu,
			address, size, count, buffer);
}

int arm926ejs_init_arch_info(struct target *target, struct arm926ejs_common *arm926ejs,
		struct jtag_tap *tap)
{
	struct arm7_9_common *arm7_9 = &arm926ejs->arm7_9_common;

	arm7_9->arm.mrc = arm926ejs_mrc;
	arm7_9->arm.mcr = arm926ejs_mcr;

	/* initialize arm7/arm9 specific info (including armv4_5) */
	arm9tdmi_init_arch_info(target, arm7_9, tap);

	arm926ejs->common_magic = ARM926EJS_COMMON_MAGIC;

	arm7_9->post_debug_entry = arm926ejs_post_debug_entry;
	arm7_9->pre_restore_context = arm926ejs_pre_restore_context;
	arm7_9->write_memory = arm926ejs_write_memory;

	arm926ejs->read_cp15 = arm926ejs_cp15_read;
	arm926ejs->write_cp15 = arm926ejs_cp15_write;
	arm926ejs->armv4_5_mmu.armv4_5_cache.ctype = -1;
	arm926ejs->armv4_5_mmu.get_ttb = arm926ejs_get_ttb;
	arm926ejs->armv4_5_mmu.read_memory = arm7_9_read_memory;
	arm926ejs->armv4_5_mmu.write_memory = arm7_9_write_memory;
	arm926ejs->armv4_5_mmu.disable_mmu_caches = arm926ejs_disable_mmu_caches;
	arm926ejs->armv4_5_mmu.enable_mmu_caches = arm926ejs_enable_mmu_caches;
	arm926ejs->armv4_5_mmu.has_tiny_pages = 1;
	arm926ejs->armv4_5_mmu.mmu_enabled = 0;

	arm7_9->examine_debug_reason = arm926ejs_examine_debug_reason;

	/* The ARM926EJ-S implements the ARMv5TE architecture which
	 * has the BKPT instruction, so we don't have to use a watchpoint comparator
	 */
	arm7_9->arm_bkpt = ARMV5_BKPT(0x0);
	arm7_9->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;

	return ERROR_OK;
}

static int arm926ejs_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm926ejs_common *arm926ejs = calloc(1, sizeof(struct arm926ejs_common));

	/* ARM9EJ-S core always reports 0x1 in Capture-IR */
	target->tap->ir_capture_mask = 0x0f;

	return arm926ejs_init_arch_info(target, arm926ejs, target->tap);
}

static void arm926ejs_deinit_target(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	arm7_9_deinit(target);
	arm_free_reg_cache(arm);
	free(arm926ejs);
}

COMMAND_HANDLER(arm926ejs_handle_cache_info_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	retval = arm926ejs_verify_pointer(CMD, arm926ejs);
	if (retval != ERROR_OK)
		return retval;

	return armv4_5_handle_cache_info_command(CMD, &arm926ejs->armv4_5_mmu.armv4_5_cache);
}

static int arm926ejs_virt2phys(struct target *target, target_addr_t virtual, target_addr_t *physical)
{
	uint32_t cb;
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	uint32_t ret;
	int retval = armv4_5_mmu_translate_va(target, &arm926ejs->armv4_5_mmu,
			virtual, &cb, &ret);
	if (retval != ERROR_OK)
		return retval;
	*physical = ret;
	return ERROR_OK;
}

static int arm926ejs_mmu(struct target *target, int *enabled)
{
	struct arm926ejs_common *arm926ejs = target_to_arm926(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_INVALID;
	}
	*enabled = arm926ejs->armv4_5_mmu.mmu_enabled;
	return ERROR_OK;
}

static const struct command_registration arm926ejs_exec_command_handlers[] = {
	{
		.name = "cache_info",
		.handler = arm926ejs_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "display information about target caches",

	},
	COMMAND_REGISTRATION_DONE
};
const struct command_registration arm926ejs_command_handlers[] = {
	{
		.chain = arm9tdmi_command_handlers,
	},
	{
		.name = "arm926ejs",
		.mode = COMMAND_ANY,
		.help = "arm926ejs command group",
		.usage = "",
		.chain = arm926ejs_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM926 targets. */
struct target_type arm926ejs_target = {
	.name = "arm926ejs",

	.poll = arm7_9_poll,
	.arch_state = arm926ejs_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm926ejs_soft_reset_halt,

	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = arm7_9_read_memory,
	.write_memory = arm7_9_write_memory_opt,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.commands = arm926ejs_command_handlers,
	.target_create = arm926ejs_target_create,
	.init_target = arm9tdmi_init_target,
	.deinit_target = arm926ejs_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
	.virt2phys = arm926ejs_virt2phys,
	.mmu = arm926ejs_mmu,

	.read_phys_memory = arm926ejs_read_phys_memory,
	.write_phys_memory = arm926ejs_write_phys_memory,
};
