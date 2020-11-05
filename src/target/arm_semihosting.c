/***************************************************************************
 *   Copyright (C) 2009 by Marvell Technology Group Ltd.                   *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
 *                                                                         *
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 *                                                                         *
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   <ilg@livius.net>                                                      *
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

/**
 * @file
 * Hold ARM semihosting support.
 *
 * Semihosting enables code running on an ARM target to use the I/O
 * facilities on the host computer. The target application must be linked
 * against a library that forwards operation requests by using the SVC
 * instruction trapped at the Supervisor Call vector by the debugger.
 * Details can be found in chapter 8 of DUI0203I_rvct_developer_guide.pdf
 * from ARM Ltd.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "armv4_5.h"
#include "arm7_9_common.h"
#include "armv7m.h"
#include "armv7a.h"
#include "armv8.h"
#include "cortex_m.h"
#include "register.h"
#include "arm_opcodes.h"
#include "target_type.h"
#include "arm_semihosting.h"
#include <helper/binarybuffer.h>
#include <helper/log.h>
#include <sys/stat.h>

static int arm_semihosting_resume(struct target *target, int *retval)
{
	if (is_armv8(target_to_armv8(target))) {
		struct armv8_common *armv8 = target_to_armv8(target);
		if (armv8->last_run_control_op == ARMV8_RUNCONTROL_RESUME) {
			*retval = target_resume(target, 1, 0, 0, 0);
			if (*retval != ERROR_OK) {
				LOG_ERROR("Failed to resume target");
				return 0;
			}
		} else if (armv8->last_run_control_op == ARMV8_RUNCONTROL_STEP)
			target->debug_reason = DBG_REASON_SINGLESTEP;
	} else {
		*retval = target_resume(target, 1, 0, 0, 0);
		if (*retval != ERROR_OK) {
			LOG_ERROR("Failed to resume target");
			return 0;
		}
	}
	return 1;
}

static int post_result(struct target *target)
{
	struct arm *arm = target_to_arm(target);

	if (!target->semihosting)
		return ERROR_FAIL;

	/* REVISIT this looks wrong ... ARM11 and Cortex-A8
	 * should work this way at least sometimes.
	 */
	if (is_arm7_9(target_to_arm7_9(target)) ||
	    is_armv7a(target_to_armv7a(target))) {
		uint32_t spsr;

		/* return value in R0 */
		buf_set_u32(arm->core_cache->reg_list[0].value, 0, 32, target->semihosting->result);
		arm->core_cache->reg_list[0].dirty = true;

		/* LR --> PC */
		buf_set_u32(arm->core_cache->reg_list[15].value, 0, 32,
			buf_get_u32(arm_reg_current(arm, 14)->value, 0, 32));
		arm->core_cache->reg_list[15].dirty = true;

		/* saved PSR --> current PSR */
		spsr = buf_get_u32(arm->spsr->value, 0, 32);

		/* REVISIT should this be arm_set_cpsr(arm, spsr)
		 * instead of a partially unrolled version?
		 */

		buf_set_u32(arm->cpsr->value, 0, 32, spsr);
		arm->cpsr->dirty = true;
		arm->core_mode = spsr & 0x1f;
		if (spsr & 0x20)
			arm->core_state = ARM_STATE_THUMB;

	} else if (is_armv8(target_to_armv8(target))) {
		if (arm->core_state == ARM_STATE_AARCH64) {
			/* return value in R0 */
			buf_set_u64(arm->core_cache->reg_list[0].value, 0, 64, target->semihosting->result);
			arm->core_cache->reg_list[0].dirty = true;

			uint64_t pc = buf_get_u64(arm->core_cache->reg_list[32].value, 0, 64);
			buf_set_u64(arm->pc->value, 0, 64, pc + 4);
			arm->pc->dirty = true;
		}
	} else {
		/* resume execution, this will be pc+2 to skip over the
		 * bkpt instruction */

		/* return result in R0 */
		buf_set_u32(arm->core_cache->reg_list[0].value, 0, 32, target->semihosting->result);
		arm->core_cache->reg_list[0].dirty = true;
	}

	return ERROR_OK;
}

/**
 * Initialize ARM semihosting support.
 *
 * @param target Pointer to the ARM target to initialize.
 * @return An error status if there is a problem during initialization.
 */
int arm_semihosting_init(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	assert(arm->setup_semihosting);
	semihosting_common_init(target, arm->setup_semihosting, post_result);

	return ERROR_OK;
}

/**
 * Checks for and processes an ARM semihosting request.  This is meant
 * to be called when the target is stopped due to a debug mode entry.
 * If the value 0 is returned then there was nothing to process. A non-zero
 * return value signifies that a request was processed and the target resumed,
 * or an error was encountered, in which case the caller must return
 * immediately.
 *
 * @param target Pointer to the ARM target to process.  This target must
 *	not represent an ARMv6-M or ARMv7-M processor.
 * @param retval Pointer to a location where the return code will be stored
 * @return non-zero value if a request was processed or an error encountered
 */
int arm_semihosting(struct target *target, int *retval)
{
	struct arm *arm = target_to_arm(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t pc, lr, spsr;
	struct reg *r;

	struct semihosting *semihosting = target->semihosting;
	if (!semihosting)
		return 0;

	if (!semihosting->is_active)
		return 0;

	if (is_arm7_9(target_to_arm7_9(target)) ||
	    is_armv7a(armv7a)) {
		uint32_t vbar = 0x00000000;

		if (arm->core_mode != ARM_MODE_SVC)
			return 0;

		if (is_armv7a(armv7a)) {
			struct arm_dpm *dpm = armv7a->arm.dpm;

			*retval = dpm->prepare(dpm);
			if (*retval == ERROR_OK) {
				*retval = dpm->instr_read_data_r0(dpm,
								 ARMV4_5_MRC(15, 0, 0, 12, 0, 0),
								 &vbar);

				dpm->finish(dpm);

				if (*retval != ERROR_OK)
					return 1;
			} else {
				return 1;
			}
		}

		/* Check for PC == 0x00000008 or 0xffff0008: Supervisor Call vector. */
		r = arm->pc;
		pc = buf_get_u32(r->value, 0, 32);
		if (pc != (vbar + 0x00000008) && pc != 0xffff0008)
			return 0;

		r = arm_reg_current(arm, 14);
		lr = buf_get_u32(r->value, 0, 32);

		/* Core-specific code should make sure SPSR is retrieved
		 * when the above checks pass...
		 */
		if (!arm->spsr->valid) {
			LOG_ERROR("SPSR not valid!");
			*retval = ERROR_FAIL;
			return 1;
		}

		spsr = buf_get_u32(arm->spsr->value, 0, 32);

		/* check instruction that triggered this trap */
		if (spsr & (1 << 5)) {
			/* was in Thumb (or ThumbEE) mode */
			uint8_t insn_buf[2];
			uint16_t insn;

			*retval = target_read_memory(target, lr-2, 2, 1, insn_buf);
			if (*retval != ERROR_OK)
				return 1;
			insn = target_buffer_get_u16(target, insn_buf);

			/* SVC 0xab */
			if (insn != 0xDFAB)
				return 0;
		} else if (spsr & (1 << 24)) {
			/* was in Jazelle mode */
			return 0;
		} else {
			/* was in ARM mode */
			uint8_t insn_buf[4];
			uint32_t insn;

			*retval = target_read_memory(target, lr-4, 4, 1, insn_buf);
			if (*retval != ERROR_OK)
				return 1;
			insn = target_buffer_get_u32(target, insn_buf);

			/* SVC 0x123456 */
			if (insn != 0xEF123456)
				return 0;
		}
	} else if (is_armv7m(target_to_armv7m(target))) {
		uint16_t insn;

		if (target->debug_reason != DBG_REASON_BREAKPOINT)
			return 0;

		r = arm->pc;
		pc = buf_get_u32(r->value, 0, 32);

		pc &= ~1;
		*retval = target_read_u16(target, pc, &insn);
		if (*retval != ERROR_OK)
			return 1;

		/* bkpt 0xAB */
		if (insn != 0xBEAB)
			return 0;
	} else if (is_armv8(target_to_armv8(target))) {
		if (target->debug_reason != DBG_REASON_BREAKPOINT)
			return 0;

		if (arm->core_state == ARM_STATE_AARCH64) {
			uint32_t insn = 0;
			r = arm->pc;
			uint64_t pc64 = buf_get_u64(r->value, 0, 64);
			*retval = target_read_u32(target, pc64, &insn);

			if (*retval != ERROR_OK)
				return 1;

			/* bkpt 0xAB */
			if (insn != 0xD45E0000)
				return 0;
		} else
			return 1;
	} else {
		LOG_ERROR("Unsupported semi-hosting Target");
		return 0;
	}

	/* Perform semihosting if we are not waiting on a fileio
	 * operation to complete.
	 */
	if (!semihosting->hit_fileio) {
		if (is_armv8(target_to_armv8(target)) &&
				arm->core_state == ARM_STATE_AARCH64) {
			/* Read op and param from register x0 and x1 respectively. */
			semihosting->op = buf_get_u64(arm->core_cache->reg_list[0].value, 0, 64);
			semihosting->param = buf_get_u64(arm->core_cache->reg_list[1].value, 0, 64);
			semihosting->word_size_bytes = 8;
		} else {
			/* Read op and param from register r0 and r1 respectively. */
			semihosting->op = buf_get_u32(arm->core_cache->reg_list[0].value, 0, 32);
			semihosting->param = buf_get_u32(arm->core_cache->reg_list[1].value, 0, 32);
			semihosting->word_size_bytes = 4;
		}

		/* Check for ARM operation numbers. */
		if (0 <= semihosting->op && semihosting->op <= 0x31) {
			*retval = semihosting_common(target);
			if (*retval != ERROR_OK) {
				LOG_ERROR("Failed semihosting operation (0x%02X)", semihosting->op);
				return 0;
			}
		} else {
			/* Unknown operation number, not a semihosting call. */
			return 0;
		}
	}

	/* Resume if target it is resumable and we are not waiting on a fileio
	 * operation to complete:
	 */
	if (semihosting->is_resumable && !semihosting->hit_fileio)
		return arm_semihosting_resume(target, retval);

	return 0;
}
