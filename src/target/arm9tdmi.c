/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by Hongtao Zheng                                   *
 *   hontor@126.com                                                        *
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

#include "arm9tdmi.h"
#include "target_type.h"
#include "register.h"
#include "arm_opcodes.h"
#include "arm_semihosting.h"

/*
 * NOTE:  this holds code that's used with multiple ARM9 processors:
 *  - ARM9TDMI (ARMv4T) ... in ARM920, ARM922, and ARM940 cores
 *  - ARM9E-S (ARMv5TE) ... in ARM946, ARM966, and ARM968 cores
 *  - ARM9EJS (ARMv5TEJ) ... in ARM926 core
 *
 * In short, the file name is a misnomer ... it is NOT specific to
 * that first generation ARM9 processor, or cores using it.
 */

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

enum arm9tdmi_vector_bit {
	ARM9TDMI_RESET_VECTOR = 0x01,
	ARM9TDMI_UNDEF_VECTOR = 0x02,
	ARM9TDMI_SWI_VECTOR = 0x04,
	ARM9TDMI_PABT_VECTOR = 0x08,
	ARM9TDMI_DABT_VECTOR = 0x10,
	/* BIT(5) reserved -- must be zero */
	ARM9TDMI_IRQ_VECTOR = 0x40,
	ARM9TDMI_FIQ_VECTOR = 0x80,
};

static const struct arm9tdmi_vector {
	const char *name;
	uint32_t value;
} arm9tdmi_vectors[] = {
	{"reset", ARM9TDMI_RESET_VECTOR},
	{"undef", ARM9TDMI_UNDEF_VECTOR},
	{"swi", ARM9TDMI_SWI_VECTOR},
	{"pabt", ARM9TDMI_PABT_VECTOR},
	{"dabt", ARM9TDMI_DABT_VECTOR},
	{"irq", ARM9TDMI_IRQ_VECTOR},
	{"fiq", ARM9TDMI_FIQ_VECTOR},
	{0, 0},
};

int arm9tdmi_examine_debug_reason(struct target *target)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);

	/* only check the debug reason if we don't know it already */
	if ((target->debug_reason != DBG_REASON_DBGRQ)
			&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {
		struct scan_field fields[3];
		uint8_t databus[4];
		uint8_t instructionbus[4];
		uint8_t debug_reason;

		fields[0].num_bits = 32;
		fields[0].out_value = NULL;
		fields[0].in_value = databus;

		fields[1].num_bits = 3;
		fields[1].out_value = NULL;
		fields[1].in_value = &debug_reason;

		fields[2].num_bits = 32;
		fields[2].out_value = NULL;
		fields[2].in_value = instructionbus;

		retval = arm_jtag_scann(&arm7_9->jtag_info, 0x1, TAP_DRPAUSE);
		if (retval != ERROR_OK)
			return retval;
		retval = arm_jtag_set_instr(arm7_9->jtag_info.tap, arm7_9->jtag_info.intest_instr, NULL, TAP_DRPAUSE);
		if (retval != ERROR_OK)
			return retval;

		jtag_add_dr_scan(arm7_9->jtag_info.tap, 3, fields, TAP_DRPAUSE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		fields[0].in_value = NULL;
		fields[0].out_value = databus;
		fields[1].in_value = NULL;
		fields[1].out_value = &debug_reason;
		fields[2].in_value = NULL;
		fields[2].out_value = instructionbus;

		jtag_add_dr_scan(arm7_9->jtag_info.tap, 3, fields, TAP_DRPAUSE);

		if (debug_reason & 0x4)
			if (debug_reason & 0x2)
				target->debug_reason = DBG_REASON_WPTANDBKPT;
		else
			target->debug_reason = DBG_REASON_WATCHPOINT;
		else
			target->debug_reason = DBG_REASON_BREAKPOINT;
	}

	return ERROR_OK;
}

/* put an instruction in the ARM9TDMI pipeline or write the data bus,
 * and optionally read data
 */
int arm9tdmi_clock_out(struct arm_jtag *jtag_info, uint32_t instr,
		uint32_t out, uint32_t *in, int sysspeed)
{
	int retval = ERROR_OK;
	struct scan_field fields[3];
	uint8_t out_buf[4];
	uint8_t instr_buf[4];
	uint8_t sysspeed_buf = 0x0;

	/* prepare buffer */
	buf_set_u32(out_buf, 0, 32, out);

	buf_set_u32(instr_buf, 0, 32, flip_u32(instr, 32));

	if (sysspeed)
		buf_set_u32(&sysspeed_buf, 2, 1, 1);

	retval = arm_jtag_scann(jtag_info, 0x1, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = out_buf;
	fields[0].in_value = NULL;

	fields[1].num_bits = 3;
	fields[1].out_value = &sysspeed_buf;
	fields[1].in_value = NULL;

	fields[2].num_bits = 32;
	fields[2].out_value = instr_buf;
	fields[2].in_value = NULL;

	if (in) {
		fields[0].in_value = (uint8_t *)in;
		jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_DRPAUSE);

		jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)in);
	} else
		jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_DRPAUSE);

	jtag_add_runtest(0, TAP_DRPAUSE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	{
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		if (in)
			LOG_DEBUG("instr: 0x%8.8x, out: 0x%8.8x, in: 0x%8.8x", instr, out, *in);
		else
			LOG_DEBUG("instr: 0x%8.8x, out: 0x%8.8x", instr, out);
	}
#endif

	return ERROR_OK;
}

/* just read data (instruction and data-out = don't care) */
int arm9tdmi_clock_data_in(struct arm_jtag *jtag_info, uint32_t *in)
{
	int retval = ERROR_OK;
	struct scan_field fields[3];

	retval = arm_jtag_scann(jtag_info, 0x1, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = (uint8_t *)in;

	fields[1].num_bits = 3;
	fields[1].out_value = NULL;
	fields[1].in_value = NULL;

	fields[2].num_bits = 32;
	fields[2].out_value = NULL;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_DRPAUSE);

	jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)in);

	jtag_add_runtest(0, TAP_DRPAUSE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	{
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		if (in)
			LOG_DEBUG("in: 0x%8.8x", *in);
		else
			LOG_ERROR("BUG: called with in == NULL");
	}
#endif

	return ERROR_OK;
}

/* clock the target, and read the databus
 * the *in pointer points to a buffer where elements of 'size' bytes
 * are stored in big (be == 1) or little (be == 0) endianness
 */
int arm9tdmi_clock_data_in_endianness(struct arm_jtag *jtag_info,
		void *in, int size, int be)
{
	int retval = ERROR_OK;
	struct scan_field fields[2];

	retval = arm_jtag_scann(jtag_info, 0x1, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	if (size == 4) {
		fields[0].num_bits = 32;
		fields[0].out_value = NULL;
		fields[0].in_value = in;

		fields[1].num_bits = 3 + 32;
		fields[1].out_value = NULL;
		fields[1].in_value = NULL;
	} else {
		/* Discard irrelevant bits of the scan, making sure we don't write more
		 * than size bytes to in */
		fields[0].num_bits = size * 8;
		fields[0].out_value = NULL;
		fields[0].in_value = in;

		fields[1].num_bits = 3 + 32 + 32 - size * 8;
		fields[1].out_value = NULL;
		fields[1].in_value = NULL;
	}

	jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_DRPAUSE);

	jtag_add_callback4(arm7_9_endianness_callback,
		(jtag_callback_data_t)in,
		(jtag_callback_data_t)size,
		(jtag_callback_data_t)be,
		(jtag_callback_data_t)0);

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

static void arm9tdmi_change_to_arm(struct target *target,
		uint32_t *r0, uint32_t *pc)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* save r0 before using it and put system in ARM state
	 * to allow common handling of ARM and THUMB debugging */

	/* fetch STR r0, [r0] */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* STR r0, [r0] in Memory */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, r0, 0);

	/* MOV r0, r15 fetched, STR in Decode */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_MOV(0, 15), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* nothing fetched, STR r0, [r0] in Memory */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, pc, 0);

	/* use pc-relative LDR to clear r0[1:0] (for switch to ARM mode) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), 0, NULL, 0);
	/* LDR in Decode */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* LDR in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* LDR in Memory (to account for interlock) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	/* fetch BX */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_BX(0), 0, NULL, 0);
	/* NOP fetched, BX in Decode, MOV in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* NOP fetched, BX in Execute (1) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return;

	/* fix program counter:
	 * MOV r0, r15 was the 5th instruction (+8)
	 * reading PC in Thumb state gives address of instruction + 4
	 */
	*pc -= 0xc;
}

void arm9tdmi_read_core_regs(struct target *target,
		uint32_t mask, uint32_t *core_regs[16])
{
	int i;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* STMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);

	/* fetch NOP, STM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++) {
		if (mask & (1 << i))
			/* nothing fetched, STM in MEMORY (i'th cycle) */
			arm9tdmi_clock_data_in(jtag_info, core_regs[i]);
	}
}

static void arm9tdmi_read_core_regs_target_buffer(struct target *target,
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
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);

	/* fetch NOP, STM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++) {
		if (mask & (1 << i))
			/* nothing fetched, STM in MEMORY (i'th cycle) */
			switch (size) {
				case 4:
					arm9tdmi_clock_data_in_endianness(jtag_info, buf_u32++, 4, be);
					break;
				case 2:
					arm9tdmi_clock_data_in_endianness(jtag_info, buf_u16++, 2, be);
					break;
				case 1:
					arm9tdmi_clock_data_in_endianness(jtag_info, buf_u8++, 1, be);
					break;
			}
	}
}

static void arm9tdmi_read_xpsr(struct target *target, uint32_t *xpsr, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* MRS r0, cpsr */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MRS(0, spsr & 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* STR r0, [r15] */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STR(0, 15), 0, NULL, 0);
	/* fetch NOP, STR in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STR in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, STR in MEMORY */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, xpsr, 0);
}

static void arm9tdmi_write_xpsr(struct target *target, uint32_t xpsr, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr: %8.8" PRIx32 ", spsr: %i", xpsr, spsr);

	/* MSR1 fetched */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr & 0xff, 0, 1, spsr), 0, NULL, 0);
	/* MSR2 fetched, MSR1 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff00) >> 8, 0xc, 2, spsr), 0, NULL, 0);
	/* MSR3 fetched, MSR1 in EXECUTE (1), MSR2 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff0000) >> 16, 0x8, 4, spsr), 0, NULL, 0);
	/* nothing fetched, MSR1 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR1 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* MSR4 fetched, MSR2 in EXECUTE (1), MSR3 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff000000) >> 24, 0x4, 8, spsr), 0, NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR3 in EXECUTE (1), MSR4 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR3 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR3 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR4 in EXECUTE (1) */
	/* last MSR writes flags, which takes only one cycle */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void arm9tdmi_write_xpsr_im8(struct target *target,
		uint8_t xpsr_im, int rot, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);

	/* MSR fetched */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr_im, rot, 1, spsr), 0, NULL, 0);
	/* NOP fetched, MSR in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR in EXECUTE (1) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* rot == 4 writes flags, which takes only one cycle */
	if (rot != 4) {
		/* nothing fetched, MSR in EXECUTE (2) */
		arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
		/* nothing fetched, MSR in EXECUTE (3) */
		arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	}
}

void arm9tdmi_write_core_regs(struct target *target,
		uint32_t mask, uint32_t core_regs[16])
{
	int i;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* LDMIA r0-15, [r0] at debug speed
	* register values will start to appear on 4th DCLK
	*/
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++) {
		if (mask & (1 << i))
			/* nothing fetched, LDM still in EXECUTE (1 + i cycle) */
			arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, core_regs[i], NULL, 0);
	}
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void arm9tdmi_load_word_regs(struct target *target, uint32_t mask)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load-multiple into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

void arm9tdmi_load_hword_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load half-word into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDRH_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

void arm9tdmi_load_byte_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load byte into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDRB_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

void arm9tdmi_store_word_regs(struct target *target, uint32_t mask)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store-multiple into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask, 0, 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

void arm9tdmi_store_hword_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store half-word into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STRH_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

void arm9tdmi_store_byte_reg(struct target *target, int num)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store byte into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STRB_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

static void arm9tdmi_write_pc(struct target *target, uint32_t pc)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* LDMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x8000, 0, 0), 0, NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) (output data) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, pc, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (4th cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (5th cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void arm9tdmi_branch_resume(struct target *target)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_B(0xfffffc, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

static void arm9tdmi_branch_resume_thumb(struct target *target)
{
	LOG_DEBUG("-");

	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm *arm = &arm7_9->arm;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct reg *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	/* LDMIA r0-15, [r0] at debug speed
	* register values will start to appear on 4th DCLK
	*/
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x1, 0, 0), 0, NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP,
			buf_get_u32(arm->pc->value, 0, 32) | 1, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* Branch and eXchange */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_BX(0), 0, NULL, 0);

	embeddedice_read_reg(dbg_stat);

	/* fetch NOP, BX in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	embeddedice_read_reg(dbg_stat);

	/* fetch NOP, BX in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* target is now in Thumb state */
	embeddedice_read_reg(dbg_stat);

	/* load r0 value, MOV_IM in Decode*/
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), 0, NULL, 0);
	/* fetch NOP, LDR in Decode, MOV_IM in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* fetch NOP, LDR in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* nothing fetched, LDR in EXECUTE stage (2nd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP,
			buf_get_u32(arm->core_cache->reg_list[0].value, 0, 32), NULL, 0);
	/* nothing fetched, LDR in EXECUTE stage (3rd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	embeddedice_read_reg(dbg_stat);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_B(0x7f7), 0, NULL, 1);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
}

void arm9tdmi_enable_single_step(struct target *target, uint32_t next_pc)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);

	if (arm7_9->has_single_step) {
		buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 3, 1, 1);
		embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);
	} else
		arm7_9_enable_eice_step(target, next_pc);
}

void arm9tdmi_disable_single_step(struct target *target)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);

	if (arm7_9->has_single_step) {
		buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 3, 1, 0);
		embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);
	} else
		arm7_9_disable_eice_step(target);
}

static void arm9tdmi_build_reg_cache(struct target *target)
{
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct arm *arm = target_to_arm(target);

	(*cache_p) = arm_build_reg_cache(target, arm);
}

int arm9tdmi_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	arm9tdmi_build_reg_cache(target);
	arm_semihosting_init(target);
	return ERROR_OK;
}

int arm9tdmi_init_arch_info(struct target *target,
		struct arm7_9_common *arm7_9, struct jtag_tap *tap)
{
	/* prepare JTAG information for the new target */
	arm7_9->jtag_info.tap = tap;
	arm7_9->jtag_info.scann_size = 5;

	/* register arch-specific functions */
	arm7_9->examine_debug_reason = arm9tdmi_examine_debug_reason;
	arm7_9->change_to_arm = arm9tdmi_change_to_arm;
	arm7_9->read_core_regs = arm9tdmi_read_core_regs;
	arm7_9->read_core_regs_target_buffer = arm9tdmi_read_core_regs_target_buffer;
	arm7_9->read_xpsr = arm9tdmi_read_xpsr;

	arm7_9->write_xpsr = arm9tdmi_write_xpsr;
	arm7_9->write_xpsr_im8 = arm9tdmi_write_xpsr_im8;
	arm7_9->write_core_regs = arm9tdmi_write_core_regs;

	arm7_9->load_word_regs = arm9tdmi_load_word_regs;
	arm7_9->load_hword_reg = arm9tdmi_load_hword_reg;
	arm7_9->load_byte_reg = arm9tdmi_load_byte_reg;

	arm7_9->store_word_regs = arm9tdmi_store_word_regs;
	arm7_9->store_hword_reg = arm9tdmi_store_hword_reg;
	arm7_9->store_byte_reg = arm9tdmi_store_byte_reg;

	arm7_9->write_pc = arm9tdmi_write_pc;
	arm7_9->branch_resume = arm9tdmi_branch_resume;
	arm7_9->branch_resume_thumb = arm9tdmi_branch_resume_thumb;

	arm7_9->enable_single_step = arm9tdmi_enable_single_step;
	arm7_9->disable_single_step = arm9tdmi_disable_single_step;

	arm7_9->write_memory = arm7_9_write_memory;
	arm7_9->bulk_write_memory = arm7_9_bulk_write_memory;

	arm7_9->post_debug_entry = NULL;

	arm7_9->pre_restore_context = NULL;

	/* initialize arch-specific breakpoint handling */
	arm7_9->arm_bkpt = 0xdeeedeee;
	arm7_9->thumb_bkpt = 0xdeee;

	arm7_9->dbgreq_adjust_pc = 3;

	arm7_9_init_arch_info(target, arm7_9);

	/* override use of DBGRQ, this is safe on ARM9TDMI */
	arm7_9->use_dbgrq = 1;

	/* all ARM9s have the vector catch register */
	arm7_9->has_vector_catch = 1;

	return ERROR_OK;
}

static int arm9tdmi_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm7_9_common *arm7_9 = calloc(1, sizeof(struct arm7_9_common));

	arm9tdmi_init_arch_info(target, arm7_9, target->tap);
	arm7_9->arm.is_armv4 = true;

	return ERROR_OK;
}

void arm9tdmi_deinit_target(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);

	arm7_9_deinit(target);
	arm_free_reg_cache(arm);
	free(arm7_9);
}

COMMAND_HANDLER(handle_arm9tdmi_catch_vectors_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct reg *vector_catch;
	uint32_t vector_catch_value;

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	/* it's uncommon, but some ARM7 chips can support this */
	if (arm7_9->common_magic != ARM7_9_COMMON_MAGIC
			|| !arm7_9->has_vector_catch) {
		command_print(CMD, "target doesn't have EmbeddedICE "
				"with vector_catch");
		return ERROR_TARGET_INVALID;
	}

	vector_catch = &arm7_9->eice_cache->reg_list[EICE_VEC_CATCH];

	/* read the vector catch register if necessary */
	if (!vector_catch->valid)
		embeddedice_read_reg(vector_catch);

	/* get the current setting */
	vector_catch_value = buf_get_u32(vector_catch->value, 0, 8);

	if (CMD_ARGC > 0) {
		vector_catch_value = 0x0;
		if (strcmp(CMD_ARGV[0], "all") == 0)
			vector_catch_value = 0xdf;
		else if (strcmp(CMD_ARGV[0], "none") == 0) {
			/* do nothing */
		} else {
			for (unsigned i = 0; i < CMD_ARGC; i++) {
				/* go through list of vectors */
				unsigned j;
				for (j = 0; arm9tdmi_vectors[j].name; j++) {
					if (strcmp(CMD_ARGV[i], arm9tdmi_vectors[j].name) == 0) {
						vector_catch_value |= arm9tdmi_vectors[j].value;
						break;
					}
				}

				/* complain if vector wasn't found */
				if (!arm9tdmi_vectors[j].name) {
					command_print(CMD, "vector '%s' not found, leaving current setting unchanged", CMD_ARGV[i]);

					/* reread current setting */
					vector_catch_value = buf_get_u32(
							vector_catch->value,
							0, 8);
					break;
				}
			}
		}

		/* store new settings */
		buf_set_u32(vector_catch->value, 0, 8, vector_catch_value);
		embeddedice_store_reg(vector_catch);
	}

	/* output current settings */
	for (unsigned i = 0; arm9tdmi_vectors[i].name; i++) {
		command_print(CMD, "%s: %s", arm9tdmi_vectors[i].name,
			(vector_catch_value & arm9tdmi_vectors[i].value)
				? "catch" : "don't catch");
	}

	return ERROR_OK;
}

static const struct command_registration arm9tdmi_exec_command_handlers[] = {
	{
		.name = "vector_catch",
		.handler = handle_arm9tdmi_catch_vectors_command,
		.mode = COMMAND_EXEC,
		.help = "Display, after optionally updating, configuration "
			"of vector catch unit.",
		.usage = "[all|none|(reset|undef|swi|pabt|dabt|irq|fiq)*]",
	},
	COMMAND_REGISTRATION_DONE
};
const struct command_registration arm9tdmi_command_handlers[] = {
	{
		.chain = arm7_9_command_handlers,
	},
	{
		.name = "arm9",
		.mode = COMMAND_ANY,
		.help = "arm9 command group",
		.usage = "",
		.chain = arm9tdmi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM9TDMI targets. */
struct target_type arm9tdmi_target = {
	.name = "arm9tdmi",

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

	.commands = arm9tdmi_command_handlers,
	.target_create = arm9tdmi_target_create,
	.init_target = arm9tdmi_init_target,
	.deinit_target = arm9tdmi_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
