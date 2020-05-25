/***************************************************************************
 *   Copyright (C) 2009 by Paulius Zaleckas                                *
 *   paulius.zaleckas@gmail.com                                            *
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
 * FA526 is very similar to ARM920T with following differences:
 *
 * - execution pipeline is 6 steps
 * - Unified TLB
 * - has Branch Target Buffer
 * - does not support reading of I/D cache contents
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm920t.h"
#include "target_type.h"
#include "arm_opcodes.h"

static void fa526_change_to_arm(struct target *target, uint32_t *r0, uint32_t *pc)
{
	LOG_ERROR("%s: there is no Thumb state on FA526", __func__);
}

static void fa526_read_core_regs(struct target *target,
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
	/* fetch NOP, STM in SHIFT stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++) {
		if (mask & (1 << i))
			/* nothing fetched, STM in MEMORY (i'th cycle) */
			arm9tdmi_clock_data_in(jtag_info, core_regs[i]);
	}
}

static void fa526_read_core_regs_target_buffer(struct target *target,
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
	/* fetch NOP, STM in SHIFT stage */
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

static void fa526_read_xpsr(struct target *target, uint32_t *xpsr, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* MRS r0, cpsr */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MRS(0, spsr & 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* STR r0, [r15] */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STR(0, 15), 0, NULL, 0);
	/* fetch NOP, STR in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STR in SHIFT stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STR in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, STR in MEMORY */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, xpsr, 0);
}

static void fa526_write_xpsr(struct target *target, uint32_t xpsr, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr: %8.8" PRIx32 ", spsr: %i", xpsr, spsr);

	/* MSR1 fetched */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr & 0xff, 0, 1, spsr), 0, NULL, 0);
	/* MSR2 fetched, MSR1 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff00) >> 8, 0xc, 2, spsr), 0, NULL, 0);
	/* MSR3 fetched, MSR1 in SHIFT, MSR2 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff0000) >> 16, 0x8, 4, spsr), 0, NULL, 0);
	/* MSR4 fetched, MSR1 in EXECUTE (1), MSR2 in SHIFT, MSR3 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff000000) >> 24, 0x4, 8, spsr), 0, NULL, 0);
	/* nothing fetched, MSR1 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR1 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (1), MSR3 in SHIFT, MSR4 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR3 in EXECUTE (1), MSR4 in SHIFT */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR3 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR3 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR4 in EXECUTE (1) */
	/* last MSR writes flags, which takes only one cycle */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

static void fa526_write_xpsr_im8(struct target *target,
		uint8_t xpsr_im, int rot, int spsr)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);

	/* MSR fetched */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr_im, rot, 1, spsr), 0, NULL, 0);
	/* NOP fetched, MSR in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR in SHIFT */
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

static void fa526_write_core_regs(struct target *target,
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
	/* fetch NOP, LDM in SHIFT stage */
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

static void fa526_write_pc(struct target *target, uint32_t pc)
{
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;

	/* LDMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x8000, 0, 0), 0, NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in SHIFT stage */
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

static void fa526_branch_resume_thumb(struct target *target)
{
	LOG_ERROR("%s: there is no Thumb state on FA526", __func__);
}

static int fa526_init_arch_info_2(struct target *target,
		struct arm7_9_common *arm7_9, struct jtag_tap *tap)
{
	/* prepare JTAG information for the new target */
	arm7_9->jtag_info.tap = tap;
	arm7_9->jtag_info.scann_size = 5;

	/* register arch-specific functions */
	arm7_9->examine_debug_reason = arm9tdmi_examine_debug_reason;
	arm7_9->change_to_arm = fa526_change_to_arm;
	arm7_9->read_core_regs = fa526_read_core_regs;
	arm7_9->read_core_regs_target_buffer = fa526_read_core_regs_target_buffer;
	arm7_9->read_xpsr = fa526_read_xpsr;

	arm7_9->write_xpsr = fa526_write_xpsr;
	arm7_9->write_xpsr_im8 = fa526_write_xpsr_im8;
	arm7_9->write_core_regs = fa526_write_core_regs;

	arm7_9->load_word_regs = arm9tdmi_load_word_regs;
	arm7_9->load_hword_reg = arm9tdmi_load_hword_reg;
	arm7_9->load_byte_reg = arm9tdmi_load_byte_reg;

	arm7_9->store_word_regs = arm9tdmi_store_word_regs;
	arm7_9->store_hword_reg = arm9tdmi_store_hword_reg;
	arm7_9->store_byte_reg = arm9tdmi_store_byte_reg;

	arm7_9->write_pc = fa526_write_pc;
	arm7_9->branch_resume = arm9tdmi_branch_resume;
	arm7_9->branch_resume_thumb = fa526_branch_resume_thumb;

	arm7_9->enable_single_step = arm9tdmi_enable_single_step;
	arm7_9->disable_single_step = arm9tdmi_disable_single_step;

	arm7_9->write_memory = arm920t_write_memory;
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

static int fa526_init_arch_info(struct target *target,
		struct arm920t_common *arm920t, struct jtag_tap *tap)
{
	struct arm7_9_common *arm7_9 = &arm920t->arm7_9_common;

	/* initialize arm7/arm9 specific info (including armv4_5) */
	fa526_init_arch_info_2(target, arm7_9, tap);

	arm920t->common_magic = ARM920T_COMMON_MAGIC;

	arm7_9->post_debug_entry = arm920t_post_debug_entry;
	arm7_9->pre_restore_context = arm920t_pre_restore_context;

	arm920t->armv4_5_mmu.armv4_5_cache.ctype = -1;
	arm920t->armv4_5_mmu.get_ttb = arm920t_get_ttb;
	arm920t->armv4_5_mmu.read_memory = arm7_9_read_memory;
	arm920t->armv4_5_mmu.write_memory = arm7_9_write_memory;
	arm920t->armv4_5_mmu.disable_mmu_caches = arm920t_disable_mmu_caches;
	arm920t->armv4_5_mmu.enable_mmu_caches = arm920t_enable_mmu_caches;
	arm920t->armv4_5_mmu.has_tiny_pages = 1;
	arm920t->armv4_5_mmu.mmu_enabled = 0;

	/* disabling linefills leads to lockups, so keep them enabled for now
	 * this doesn't affect correctness, but might affect timing issues, if
	 * important data is evicted from the cache during the debug session
	 * */
	arm920t->preserve_cache = 0;

	/* override hw single-step capability from ARM9TDMI */
	arm7_9->has_single_step = 1;

	return ERROR_OK;
}

static int fa526_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm920t_common *arm920t = calloc(1, sizeof(struct arm920t_common));

	return fa526_init_arch_info(target, arm920t, target->tap);
}

static void fa526_deinit_target(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm920t_common *arm920t = target_to_arm920(target);

	arm7_9_deinit(target);
	arm_free_reg_cache(arm);
	free(arm920t);
}

/** Holds methods for FA526 targets. */
struct target_type fa526_target = {
	.name = "fa526",

	.poll = arm7_9_poll,
	.arch_state = arm920t_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm920t_soft_reset_halt,

	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = arm920t_read_memory,
	.write_memory = arm7_9_write_memory_opt,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.commands = arm920t_command_handlers,
	.target_create = fa526_target_create,
	.init_target = arm9tdmi_init_target,
	.deinit_target = fa526_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
