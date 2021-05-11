/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
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

#include "arm7tdmi.h"
#include "target_type.h"
#include "register.h"
#include "arm_opcodes.h"
#include "arm_semihosting.h"

/*
 * For information about ARM7TDMI, see ARM DDI 0210C (r4p1)
 * or ARM DDI 0029G (r3).  "Debug In Depth", Appendix B,
 * covers JTAG support.
 */

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

static int arm7tdmi_examine_debug_reason(struct target *target)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);

	/* only check the debug reason if we don't know it already */
	if ((target->debug_reason != DBG_REASON_DBGRQ)
			&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {
		struct scan_field fields[2];
		uint8_t databus[4];
		uint8_t breakpoint;

		fields[0].num_bits = 1;
		fields[0].out_value = NULL;
		fields[0].in_value = &breakpoint;

		fields[1].num_bits = 32;
		fields[1].out_value = NULL;
		fields[1].in_value = databus;

		retval = arm_jtag_scann(&arm7_9->jtag_info, 0x1, TAP_DRPAUSE);
		if (retval != ERROR_OK)
			return retval;
		retval = arm_jtag_set_instr(arm7_9->jtag_info.tap, arm7_9->jtag_info.intest_instr, NULL, TAP_DRPAUSE);
		if (retval != ERROR_OK)
			return retval;

		jtag_add_dr_scan(arm7_9->jtag_info.tap, 2, fields, TAP_DRPAUSE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		fields[0].in_value = NULL;
		fields[0].out_value = &breakpoint;
		fields[1].in_value = NULL;
		fields[1].out_value = databus;

		jtag_add_dr_scan(arm7_9->jtag_info.tap, 2, fields, TAP_DRPAUSE);

		if (breakpoint & 1)
			target->debug_reason = DBG_REASON_WATCHPOINT;
		else
			target->debug_reason = DBG_REASON_BREAKPOINT;
	}

	return ERROR_OK;
}

static const int arm7tdmi_num_bits[] = {1, 32};

static inline int arm7tdmi_clock_out_inner(struct arm_jtag *jtag_info, uint32_t out, int breakpoint)
{
	uint8_t bp = breakpoint ? 1 : 0;
	uint8_t out_value[4];
	buf_set_u32(out_value, 0, 32, flip_u32(out, 32));

	struct scan_field fields[2] = {
			{ .num_bits = arm7tdmi_num_bits[0], .out_value = &bp },
			{ .num_bits = arm7tdmi_num_bits[1], .out_value = out_value },
	};

	jtag_add_dr_scan(jtag_info->tap,
			2,
			fields,
			TAP_DRPAUSE);

	jtag_add_runtest(0, TAP_DRPAUSE);

	return ERROR_OK;
}

/* put an instruction in the ARM7TDMI pipeline or write the data bus,
 * and optionally read data
 */
static inline int arm7tdmi_clock_out(struct arm_jtag *jtag_info,
		uint32_t out, int breakpoint)
{
	int retval;
	retval = arm_jtag_scann(jtag_info, 0x1, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	return arm7tdmi_clock_out_inner(jtag_info, out, breakpoint);
}

/* clock the target, reading the databus */
static int arm7tdmi_clock_data_in(struct arm_jtag *jtag_info, uint32_t *in)
{
	int retval = ERROR_OK;
	struct scan_field fields[2];

	retval = arm_jtag_scann(jtag_info, 0x1, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 1;
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].in_value = (uint8_t *)in;

	jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_DRPAUSE);

	jtag_add_callback(arm7flip32, (jtag_callback_data_t)in);

	jtag_add_runtest(0, TAP_DRPAUSE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (in)
		LOG_DEBUG("in: 0x%8.8x", *in);
	else
		LOG_ERROR("BUG: called with in == NULL");
#endif

	return ERROR_OK;
}

/* clock the target, and read the databus
 * the *in pointer points to a buffer where elements of 'size' bytes
 * are stored in big (be == 1) or little (be == 0) endianness
 */
static int arm7tdmi_clock_data_in_endianness(struct arm_jtag *jtag_info,
		void *in, int size, int be)
{
	int retval = ERROR_OK;
	struct scan_field fields[3];

	retval = arm_jtag_scann(jtag_info, 0x1, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 1;
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	if (size == 4) {
		fields[1].num_bits = 32;
		fields[1].out_value = NULL;
		fields[1].in_value = in;
	} else {
		/* Discard irrelevant bits of the scan, making sure we don't write more
		 * than size bytes to in */
		fields[1].num_bits = 32 - size * 8;
		fields[1].out_value = NULL;
		fields[1].in_value = NULL;

		fields[2].num_bits = size * 8;
		fields[2].out_value = NULL;
		fields[2].in_value = in;
	}

	jtag_add_dr_scan(jtag_info->tap, size == 4 ? 2 : 3, fields, TAP_DRPAUSE);

	jtag_add_callback4(arm7_9_endianness_callback,
		(jtag_callback_data_t)in,
		(jtag_callback_data_t)size,
		(jtag_callback_data_t)be,
		(jtag_callback_data_t)1);

	jtag_add_runtest(0, TAP_DRPAUSE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
{
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		if (in)
			LOG_DEBUG("in: 0x%8.8x", *(uint32_t *)in);
		else
			LOG_ERROR("BUG: called with in == NULL");
}
#endif

	return ERROR_OK;
}

static void arm7tdmi_change_to_arm(struct target *target,
		uint32_t *r0, uint32_t *pc)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* save r0 before using it and put system in ARM state
	 * to allow common handling of ARM and THUMB debugging */

	/* fetch STR r0, [r0] */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	/* nothing fetched, STR r0, [r0] in Execute (2) */
	arm7tdmi_clock_data_in(jtag_info, r0);

	/* MOV r0, r15 fetched, STR in Decode */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_MOV(0, 15), 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	/* nothing fetched, STR r0, [r0] in Execute (2) */
	arm7tdmi_clock_data_in(jtag_info, pc);

	/* use pc-relative LDR to clear r0[1:0] (for switch to ARM mode) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	/* nothing fetched, data for LDR r0, [PC, #0] */
	arm7tdmi_clock_out(jtag_info, 0x0, 0);
	/* nothing fetched, data from previous cycle is written to register */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);

	/* fetch BX */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_BX(0), 0);
	/* NOP fetched, BX in Decode, MOV in Execute */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	/* NOP fetched, BX in Execute (1) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);

	jtag_execute_queue();

	/* fix program counter:
	 * MOV r0, r15 was the 4th instruction (+6)
	 * reading PC in Thumb state gives address of instruction + 4
	 */
	*pc -= 0xa;
}

/* FIX!!! is this a potential performance bottleneck w.r.t. requiring too many
 * roundtrips when jtag_execute_queue() has a large overhead(e.g. for USB)s?
 *
 * The solution is to arrange for a large out/in scan in this loop and
 * and convert data afterwards.
 */
static void arm7tdmi_read_core_regs(struct target *target,
		uint32_t mask, uint32_t *core_regs[16])
{
	int i;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* STMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0);

	/* fetch NOP, STM in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);

	for (i = 0; i <= 15; i++) {
		if (mask & (1 << i))
			/* nothing fetched, STM still in EXECUTE (1 + i cycle) */
			arm7tdmi_clock_data_in(jtag_info, core_regs[i]);
	}
}

static void arm7tdmi_read_core_regs_target_buffer(struct target *target,
		uint32_t mask, void *buffer, int size)
{
	int i;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	int be = (target->endianness == TARGET_BIG_ENDIAN) ? 1 : 0;
	uint32_t *buf_u32 = buffer;
	uint16_t *buf_u16 = buffer;
	uint8_t *buf_u8 = buffer;

	/* STMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0);

	/* fetch NOP, STM in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);

	for (i = 0; i <= 15; i++) {
		/* nothing fetched, STM still in EXECUTE (1 + i cycle), read databus */
		if (mask & (1 << i)) {
			switch (size) {
				case 4:
					arm7tdmi_clock_data_in_endianness(jtag_info, buf_u32++, 4, be);
					break;
				case 2:
					arm7tdmi_clock_data_in_endianness(jtag_info, buf_u16++, 2, be);
					break;
				case 1:
					arm7tdmi_clock_data_in_endianness(jtag_info, buf_u8++, 1, be);
					break;
			}
		}
	}
}

static void arm7tdmi_read_xpsr(struct target *target, uint32_t *xpsr, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* MRS r0, cpsr */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MRS(0, spsr & 1), 0);

	/* STR r0, [r15] */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STR(0, 15), 0);
	/* fetch NOP, STR in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, STR in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, STR still in EXECUTE (2nd cycle) */
	arm7tdmi_clock_data_in(jtag_info, xpsr);
}

static void arm7tdmi_write_xpsr(struct target *target, uint32_t xpsr, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr: %8.8" PRIx32 ", spsr: %i", xpsr, spsr);

	/* MSR1 fetched */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr & 0xff, 0, 1, spsr), 0);
	/* MSR2 fetched, MSR1 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff00) >> 8, 0xc, 2, spsr), 0);
	/* MSR3 fetched, MSR1 in EXECUTE (1), MSR2 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff0000) >> 16, 0x8, 4, spsr), 0);
	/* nothing fetched, MSR1 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* MSR4 fetched, MSR2 in EXECUTE (1), MSR3 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff000000) >> 24, 0x4, 8, spsr), 0);
	/* nothing fetched, MSR2 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* NOP fetched, MSR3 in EXECUTE (1), MSR4 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, MSR3 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* NOP fetched, MSR4 in EXECUTE (1) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, MSR4 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
}

static void arm7tdmi_write_xpsr_im8(struct target *target,
		uint8_t xpsr_im, int rot, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);

	/* MSR fetched */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr_im, rot, 1, spsr), 0);
	/* NOP fetched, MSR in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* NOP fetched, MSR in EXECUTE (1) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, MSR in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
}

static void arm7tdmi_write_core_regs(struct target *target,
		uint32_t mask, uint32_t core_regs[16])
{
	int i;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* LDMIA r0-15, [r0] at debug speed
	* register values will start to appear on 4th DCLK
	*/
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 0), 0);

	/* fetch NOP, LDM in DECODE stage */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);

	for (i = 0; i <= 15; i++) {
		if (mask & (1 << i))
			/* nothing fetched, LDM still in EXECUTE (1 + i cycle) */
			arm7tdmi_clock_out_inner(jtag_info, core_regs[i], 0);
	}
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
}

static void arm7tdmi_load_word_regs(struct target *target, uint32_t mask)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load-multiple into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 1), 0);
}

static void arm7tdmi_load_hword_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load half-word into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDRH_IP(num, 0), 0);
}

static void arm7tdmi_load_byte_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load byte into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDRB_IP(num, 0), 0);
}

static void arm7tdmi_store_word_regs(struct target *target, uint32_t mask)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store-multiple into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask, 0, 1), 0);
}

static void arm7tdmi_store_hword_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store half-word into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STRH_IP(num, 0), 0);
}

static void arm7tdmi_store_byte_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store byte into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STRB_IP(num, 0), 0);
}

static void arm7tdmi_write_pc(struct target *target, uint32_t pc)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* LDMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x8000, 0, 0), 0);
	/* fetch NOP, LDM in DECODE stage */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, LDM in EXECUTE stage (1st cycle) load register */
	arm7tdmi_clock_out_inner(jtag_info, pc, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) load register */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) load register */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (4th cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (5th cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
}

static void arm7tdmi_branch_resume(struct target *target)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 1);
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_B(0xfffffa, 0), 0);
}

static void arm7tdmi_branch_resume_thumb(struct target *target)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm *arm = &arm7_9->arm;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct reg *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	LOG_DEBUG("-");

	/* LDMIA r0, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x1, 0, 0), 0);

	/* fetch NOP, LDM in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) */
	arm7tdmi_clock_out(jtag_info, buf_get_u32(arm->pc->value, 0, 32) | 1, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);

	/* Branch and eXchange */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_BX(0), 0);

	embeddedice_read_reg(dbg_stat);

	/* fetch NOP, BX in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);

	/* target is now in Thumb state */
	embeddedice_read_reg(dbg_stat);

	/* fetch NOP, BX in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0);

	/* target is now in Thumb state */
	embeddedice_read_reg(dbg_stat);

	/* load r0 value */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), 0);
	/* fetch NOP, LDR in Decode */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	/* fetch NOP, LDR in Execute */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	/* nothing fetched, LDR in EXECUTE stage (2nd cycle) */
	arm7tdmi_clock_out(jtag_info, buf_get_u32(arm->core_cache->reg_list[0].value, 0, 32), 0);
	/* nothing fetched, LDR in EXECUTE stage (3rd cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);

	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0);

	embeddedice_read_reg(dbg_stat);

	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_B(0x7f8), 0);
}

static void arm7tdmi_build_reg_cache(struct target *target)
{
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct arm *arm = target_to_arm(target);

	(*cache_p) = arm_build_reg_cache(target, arm);
}

static void arm7tdmi_free_reg_cache(struct target *target)
{
	struct arm *arm = target_to_arm(target);

	arm_free_reg_cache(arm);
}

int arm7tdmi_init_target(struct command_context *cmd_ctx, struct target *target)
{
	arm7tdmi_build_reg_cache(target);
	arm_semihosting_init(target);
	return ERROR_OK;
}

void arm7tdmi_deinit_target(struct target *target)
{
	arm7tdmi_free_reg_cache(target);
}

int arm7tdmi_init_arch_info(struct target *target,
		struct arm7_9_common *arm7_9, struct jtag_tap *tap)
{
	/* prepare JTAG information for the new target */
	arm7_9->jtag_info.tap = tap;
	arm7_9->jtag_info.scann_size = 4;

	/* register arch-specific functions */
	arm7_9->examine_debug_reason = arm7tdmi_examine_debug_reason;
	arm7_9->change_to_arm = arm7tdmi_change_to_arm;
	arm7_9->read_core_regs = arm7tdmi_read_core_regs;
	arm7_9->read_core_regs_target_buffer = arm7tdmi_read_core_regs_target_buffer;
	arm7_9->read_xpsr = arm7tdmi_read_xpsr;

	arm7_9->write_xpsr = arm7tdmi_write_xpsr;
	arm7_9->write_xpsr_im8 = arm7tdmi_write_xpsr_im8;
	arm7_9->write_core_regs = arm7tdmi_write_core_regs;

	arm7_9->load_word_regs = arm7tdmi_load_word_regs;
	arm7_9->load_hword_reg = arm7tdmi_load_hword_reg;
	arm7_9->load_byte_reg = arm7tdmi_load_byte_reg;

	arm7_9->store_word_regs = arm7tdmi_store_word_regs;
	arm7_9->store_hword_reg = arm7tdmi_store_hword_reg;
	arm7_9->store_byte_reg = arm7tdmi_store_byte_reg;

	arm7_9->write_pc = arm7tdmi_write_pc;
	arm7_9->branch_resume = arm7tdmi_branch_resume;
	arm7_9->branch_resume_thumb = arm7tdmi_branch_resume_thumb;

	arm7_9->enable_single_step = arm7_9_enable_eice_step;
	arm7_9->disable_single_step = arm7_9_disable_eice_step;

	arm7_9->write_memory = arm7_9_write_memory;
	arm7_9->bulk_write_memory = arm7_9_bulk_write_memory;

	arm7_9->post_debug_entry = NULL;

	arm7_9->pre_restore_context = NULL;

	/* initialize arch-specific breakpoint handling */
	arm7_9->arm_bkpt = 0xdeeedeee;
	arm7_9->thumb_bkpt = 0xdeee;

	arm7_9->dbgreq_adjust_pc = 2;

	arm7_9_init_arch_info(target, arm7_9);

	return ERROR_OK;
}

static int arm7tdmi_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm7_9_common *arm7_9;

	arm7_9 = calloc(1, sizeof(struct arm7_9_common));
	arm7tdmi_init_arch_info(target, arm7_9, target->tap);
	arm7_9->arm.arch = ARM_ARCH_V4;

	return ERROR_OK;
}

/** Holds methods for ARM7TDMI targets. */
struct target_type arm7tdmi_target = {
	.name = "arm7tdmi",

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

	.read_memory = arm7_9_read_memory,
	.write_memory = arm7_9_write_memory_opt,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.commands  = arm7_9_command_handlers,
	.target_create  = arm7tdmi_target_create,
	.init_target = arm7tdmi_init_target,
	.deinit_target = arm7tdmi_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
