/***************************************************************************
 *   Copyright (C) 2008-2009 by Marvell Semiconductors, Inc.                    *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
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

/*
 * Marvell Feroceon/Dragonite support.
 *
 * The Feroceon core, as found in the Orion and Kirkwood SoCs amongst others,
 * mimics the ARM926 ICE interface with the following differences:
 *
 * - the MOE (method of entry) reporting is not implemented
 *
 * - breakpoint/watchpoint comparator #1 is seemingly not implemented
 *
 * - due to a different pipeline implementation, some injected debug
 *   instruction sequences have to be somewhat different
 *
 * Other issues:
 *
 * - asserting DBGRQ doesn't work if target is looping on the undef vector
 *
 * - the EICE version signature in the COMMS_CTL reg is next to the flow bits
 *   not at the top, and rather meaningless due to existing discrepancies
 *
 * - the DCC channel is half duplex (only one FIFO for both directions) with
 *   seemingly no proper flow control.
 *
 * The Dragonite core is the non-mmu version based on the ARM966 model, and
 * it shares the above issues as well.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm926ejs.h"
#include "arm966e.h"
#include "target_type.h"
#include "register.h"
#include "arm_opcodes.h"

static int feroceon_assert_reset(struct target *target)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	int ud = arm7_9->use_dbgrq;

	/* TODO: apply hw reset signal in not examined state */
	if (!(target_was_examined(target))) {
		LOG_WARNING("Reset is not asserted because the target is not examined.");
		LOG_WARNING("Use a reset button or power cycle the target.");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	arm7_9->use_dbgrq = 0;
	if (target->reset_halt)
		arm7_9_halt(target);
	arm7_9->use_dbgrq = ud;
	return arm7_9_assert_reset(target);
}

static int feroceon_dummy_clock_out(struct arm_jtag *jtag_info, uint32_t instr)
{
	struct scan_field fields[3];
	uint8_t out_buf[4];
	uint8_t instr_buf[4];
	uint8_t sysspeed_buf = 0x0;
	int retval;

	/* prepare buffer */
	buf_set_u32(out_buf, 0, 32, 0);

	buf_set_u32(instr_buf, 0, 32, flip_u32(instr, 32));

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

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_DRPAUSE);

	/* no jtag_add_runtest(0, TAP_DRPAUSE) here */

	return ERROR_OK;
}

static void feroceon_change_to_arm(struct target *target, uint32_t *r0,
		uint32_t *pc)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/*
	 * save r0 before using it and put system in ARM state
	 * to allow common handling of ARM and THUMB debugging
	 */

	feroceon_dummy_clock_out(jtag_info, ARMV4_5_T_NOP);
	feroceon_dummy_clock_out(jtag_info, ARMV4_5_T_NOP);
	feroceon_dummy_clock_out(jtag_info, ARMV4_5_T_NOP);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, r0, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_MOV(0, 15), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, pc, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_BX(15), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	jtag_execute_queue();

	/*
	 * fix program counter:
	 * MOV R0, PC was the 7th instruction (+12)
	 * reading PC in Thumb state gives address of instruction + 4
	 */
	*pc -= (12 + 4);
}

static void feroceon_read_core_regs(struct target *target,
		uint32_t mask, uint32_t *core_regs[16])
{
	int i;
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++)
		if (mask & (1 << i))
			arm9tdmi_clock_data_in(jtag_info, core_regs[i]);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void feroceon_read_core_regs_target_buffer(struct target *target,
		uint32_t mask, void *buffer, int size)
{
	int i;
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	int be = (target->endianness == TARGET_BIG_ENDIAN) ? 1 : 0;
	uint32_t *buf_u32 = buffer;
	uint16_t *buf_u16 = buffer;
	uint8_t *buf_u8 = buffer;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++) {
		if (mask & (1 << i)) {
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

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void feroceon_read_xpsr(struct target *target, uint32_t *xpsr, int spsr)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MRS(0, spsr & 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, 1, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, xpsr, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void feroceon_write_xpsr(struct target *target, uint32_t xpsr, int spsr)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr: %8.8" PRIx32 ", spsr: %i", xpsr, spsr);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr & 0xff, 0, 1, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff00) >> 8, 0xc, 2, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff0000) >> 16, 0x8, 4, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff000000) >> 24, 0x4, 8, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void feroceon_write_xpsr_im8(struct target *target,
		uint8_t xpsr_im, int rot, int spsr)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr_im, rot, 1, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void feroceon_write_core_regs(struct target *target,
		uint32_t mask, uint32_t core_regs[16])
{
	int i;
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++)
		if (mask & (1 << i))
			arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, core_regs[i], NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void feroceon_branch_resume(struct target *target)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_B(0xfffff9, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

	arm7_9->need_bypass_before_restart = 1;
}

static void feroceon_branch_resume_thumb(struct target *target)
{
	LOG_DEBUG("-");

	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	uint32_t r0 = buf_get_u32(arm->core_cache->reg_list[0].value, 0, 32);
	uint32_t pc = buf_get_u32(arm->pc->value, 0, 32);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, 0xE28F0001, 0, NULL, 0); /* add r0,pc,#1 */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_BX(0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_LDMIA(0, 0x1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, r0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	pc = (pc & 2) >> 1;
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_B(0x7e9 + pc), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 1);

	arm7_9->need_bypass_before_restart = 1;
}

static int feroceon_read_cp15(struct target *target, uint32_t op1,
		uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t *value)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	int err;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MRC(15, op1, 0, CRn, CRm, op2), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	err = arm7_9_execute_sys_speed(target);
	if (err != ERROR_OK)
		return err;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, 1, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, value, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	return jtag_execute_queue();
}

static int feroceon_write_cp15(struct target *target, uint32_t op1,
		uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t value)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 1, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, value, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MCR(15, op1, 0, CRn, CRm, op2), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	return arm7_9_execute_sys_speed(target);
}

static void feroceon_set_dbgrq(struct target *target)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	struct reg *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];

	buf_set_u32(dbg_ctrl->value, 0, 8, 2);
	embeddedice_store_reg(dbg_ctrl);
}

static void feroceon_enable_single_step(struct target *target, uint32_t next_pc)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;

	/* set a breakpoint there */
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE], next_pc);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x100);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], 0xf7);
}

static void feroceon_disable_single_step(struct target *target)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;

	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE]);
}

static int feroceon_examine_debug_reason(struct target *target)
{
	/* the MOE is not implemented */
	if (target->debug_reason != DBG_REASON_SINGLESTEP)
		target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int feroceon_bulk_write_memory(struct target *target,
		target_addr_t address, uint32_t count, const uint8_t *buffer)
{
	int retval;
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	enum arm_state core_state = arm->core_state;
	uint32_t x, flip, shift, save[7];
	uint32_t i;

	/*
	 * We can't use the dcc flow control bits, so let's transfer data
	 * with 31 bits and flip the MSB each time a new data word is sent.
	 */
	static uint32_t dcc_code[] = {
		0xee115e10,	/* 3:	mrc	p14, 0, r5, c1, c0, 0	*/
		0xe3a0301e,	/* 1:	mov	r3, #30			*/
		0xe3a04002,	/*	mov	r4, #2			*/
		0xee111e10,	/* 2:	mrc	p14, 0, r1, c1, c0, 0	*/
		0xe1310005,	/*	teq	r1, r5			*/
		0x0afffffc,	/*	beq	1b			*/
		0xe1a05001,	/*	mov	r5, r1			*/
		0xe1a01081,	/*	mov	r1, r1, lsl #1		*/
		0xee112e10,	/* 3:	mrc	p14, 0, r2, c1, c0, 0	*/
		0xe1320005,	/*	teq	r2, r5			*/
		0x0afffffc,	/*	beq	3b			*/
		0xe1a05002,	/*	mov	r5, r2			*/
		0xe3c22102,	/*	bic	r2, r2, #0x80000000	*/
		0xe1811332,	/*	orr	r1, r1, r2, lsr r3	*/
		0xe2533001,	/*	subs	r3, r3, #1		*/
		0xe4801004,	/*	str	r1, [r0], #4		*/
		0xe1a01412,	/*	mov	r1, r2, lsl r4		*/
		0xe2844001,	/*	add	r4, r4, #1		*/
		0x4affffed,	/*	bmi	1b			*/
		0xeafffff3,	/*	b	3b			*/
	};

	uint32_t dcc_size = sizeof(dcc_code);

	if (address % 4 != 0)
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (!arm7_9->dcc_downloads)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* regrab previously allocated working_area, or allocate a new one */
	if (!arm7_9->dcc_working_area) {
		uint8_t dcc_code_buf[dcc_size];

		/* make sure we have a working area */
		if (target_alloc_working_area(target, dcc_size, &arm7_9->dcc_working_area) != ERROR_OK) {
			LOG_INFO("no working area available, falling back to memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		/* copy target instructions to target endianness */
		target_buffer_set_u32_array(target, dcc_code_buf, ARRAY_SIZE(dcc_code), dcc_code);

		/* write DCC code to working area, using the non-optimized
		 * memory write to avoid ending up here again */
		retval = arm7_9_write_memory_no_opt(target,
				arm7_9->dcc_working_area->address, 4, dcc_size/4, dcc_code_buf);
		if (retval != ERROR_OK)
			return retval;
	}

	/* backup clobbered processor state */
	for (i = 0; i <= 5; i++)
		save[i] = buf_get_u32(arm->core_cache->reg_list[i].value, 0, 32);
	save[i] = buf_get_u32(arm->pc->value, 0, 32);

	/* set up target address in r0 */
	buf_set_u32(arm->core_cache->reg_list[0].value, 0, 32, address);
	arm->core_cache->reg_list[0].valid = true;
	arm->core_cache->reg_list[0].dirty = true;
	arm->core_state = ARM_STATE_ARM;

	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], 0);
	arm7_9_resume(target, 0, arm7_9->dcc_working_area->address, 1, 1);

	/* send data over */
	x = 0;
	flip = 0;
	shift = 1;
	for (i = 0; i < count; i++) {
		uint32_t y = target_buffer_get_u32(target, buffer);
		uint32_t z = (x >> 1) | (y >> shift) | (flip ^= 0x80000000);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], z);
		x = y << (32 - shift);
		if (++shift >= 32 || i + 1 >= count) {
			z = (x >> 1) | (flip ^= 0x80000000);
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], z);
			x = 0;
			shift = 1;
		}
		buffer += 4;
	}

	retval = target_halt(target);
	if (retval == ERROR_OK)
		retval = target_wait_state(target, TARGET_HALTED, 500);
	if (retval == ERROR_OK) {
		uint32_t endaddress =
			buf_get_u32(arm->core_cache->reg_list[0].value, 0, 32);
		if (endaddress != address + count*4) {
			LOG_ERROR("DCC write failed,"
				" expected end address 0x%08" TARGET_PRIxADDR
				" got 0x%0" PRIx32 "",
				address + count*4, endaddress);
			retval = ERROR_FAIL;
		}
	}

	/* restore target state */
	for (i = 0; i <= 5; i++) {
		buf_set_u32(arm->core_cache->reg_list[i].value, 0, 32, save[i]);
		arm->core_cache->reg_list[i].valid = true;
		arm->core_cache->reg_list[i].dirty = true;
	}
	buf_set_u32(arm->pc->value, 0, 32, save[i]);
	arm->pc->valid = true;
	arm->pc->dirty = true;
	arm->core_state = core_state;

	return retval;
}

static int feroceon_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	arm9tdmi_init_target(cmd_ctx, target);
	return ERROR_OK;
}

static void feroceon_deinit_target(struct target *target)
{
	arm9tdmi_deinit_target(target);
}

static void feroceon_common_setup(struct target *target)
{
	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;

	/* override some insn sequence functions */
	arm7_9->change_to_arm = feroceon_change_to_arm;
	arm7_9->read_core_regs = feroceon_read_core_regs;
	arm7_9->read_core_regs_target_buffer = feroceon_read_core_regs_target_buffer;
	arm7_9->read_xpsr = feroceon_read_xpsr;
	arm7_9->write_xpsr = feroceon_write_xpsr;
	arm7_9->write_xpsr_im8 = feroceon_write_xpsr_im8;
	arm7_9->write_core_regs = feroceon_write_core_regs;
	arm7_9->branch_resume = feroceon_branch_resume;
	arm7_9->branch_resume_thumb = feroceon_branch_resume_thumb;

	/* must be implemented with only one comparator */
	arm7_9->enable_single_step = feroceon_enable_single_step;
	arm7_9->disable_single_step = feroceon_disable_single_step;

	arm7_9->bulk_write_memory = feroceon_bulk_write_memory;

	/* MOE is not implemented */
	arm7_9->examine_debug_reason = feroceon_examine_debug_reason;

	/* Note: asserting DBGRQ might not win over the undef exception.
	   If that happens then just use "arm7_9 dbgrq disable". */
	arm7_9->use_dbgrq = 1;
	arm7_9->set_special_dbgrq = feroceon_set_dbgrq;

	/* only one working comparator */
	arm7_9->wp_available_max = 1;
	arm7_9->wp1_used_default = -1;
}

static int feroceon_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm926ejs_common *arm926ejs = calloc(1, sizeof(struct arm926ejs_common));

	arm926ejs_init_arch_info(target, arm926ejs, target->tap);
	feroceon_common_setup(target);

	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	arm7_9->write_memory = arm926ejs_write_memory;

	/* the standard ARM926 methods don't always work (don't ask...) */
	arm926ejs->read_cp15 = feroceon_read_cp15;
	arm926ejs->write_cp15 = feroceon_write_cp15;

	return ERROR_OK;
}

static int dragonite_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm966e_common *arm966e = calloc(1, sizeof(struct arm966e_common));

	arm966e_init_arch_info(target, arm966e, target->tap);
	feroceon_common_setup(target);

	struct arm *arm = target->arch_info;
	struct arm7_9_common *arm7_9 = arm->arch_info;
	arm7_9->write_memory = arm7_9_write_memory;

	return ERROR_OK;
}

static int feroceon_examine(struct target *target)
{
	struct arm *arm;
	struct arm7_9_common *arm7_9;
	int retval;

	retval = arm7_9_examine(target);
	if (retval != ERROR_OK)
		return retval;

	arm = target->arch_info;
	arm7_9 = arm->arch_info;

	/* the COMMS_CTRL bits are all contiguous */
	if (buf_get_u32(arm7_9->eice_cache->reg_list[EICE_COMMS_CTRL].value, 2, 4) != 6)
		LOG_ERROR("unexpected Feroceon EICE version signature");

	arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].size = 6;
	arm7_9->eice_cache->reg_list[EICE_DBG_STAT].size = 5;
	arm7_9->has_monitor_mode = 1;

	/* vector catch reg is not initialized on reset */
	embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_VEC_CATCH], 0);

	/* clear monitor mode, enable comparators */
	embeddedice_read_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);
	jtag_execute_queue();
	buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 4, 1, 0);
	buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 5, 1, 0);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);

	return ERROR_OK;
}

struct target_type feroceon_target = {
	.name = "feroceon",

	.poll = arm7_9_poll,
	.arch_state = arm926ejs_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = feroceon_assert_reset,
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
	.target_create = feroceon_target_create,
	.init_target = feroceon_init_target,
	.deinit_target = feroceon_deinit_target,
	.examine = feroceon_examine,
};

struct target_type dragonite_target = {
	.name = "dragonite",

	.poll = arm7_9_poll,
	.arch_state = arm_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = feroceon_assert_reset,
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

	.commands = arm966e_command_handlers,
	.target_create = dragonite_target_create,
	.init_target = feroceon_init_target,
	.examine = feroceon_examine,
};
