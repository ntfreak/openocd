/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2009 by Øyvind Harboe                                   *
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

#include "arm720t.h"
#include <helper/time_support.h>
#include "target_type.h"
#include "register.h"
#include "arm_opcodes.h"


/*
 * ARM720 is an ARM7TDMI-S with MMU and ETM7.  For information, see
 * ARM DDI 0229C especially Chapter 9 about debug support.
 */

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

static int arm720t_scan_cp15(struct target *target,
		uint32_t out, uint32_t *in, int instruction, int clock_arg)
{
	int retval;
	struct arm720t_common *arm720t = target_to_arm720(target);
	struct arm_jtag *jtag_info;
	struct scan_field fields[2];
	uint8_t out_buf[4];
	uint8_t instruction_buf = instruction;

	jtag_info = &arm720t->arm7_9_common.jtag_info;

	buf_set_u32(out_buf, 0, 32, flip_u32(out, 32));

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_DRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 1;
	fields[0].out_value = &instruction_buf;
	fields[0].in_value = NULL;

	fields[1].num_bits = 32;
	fields[1].out_value = out_buf;
	fields[1].in_value = NULL;

	if (in) {
		fields[1].in_value = (uint8_t *)in;
		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_DRPAUSE);
		jtag_add_callback(arm7flip32, (jtag_callback_data_t)in);
	} else
		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_DRPAUSE);

	if (clock_arg)
		jtag_add_runtest(0, TAP_DRPAUSE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (in)
		LOG_DEBUG("out: %8.8x, in: %8.8x, instruction: %i, clock: %i", out, *in, instruction, clock);
	else
		LOG_DEBUG("out: %8.8x, instruction: %i, clock: %i", out, instruction, clock_arg);
#else
		LOG_DEBUG("out: %8.8" PRIx32 ", instruction: %i, clock: %i", out, instruction, clock_arg);
#endif

	return ERROR_OK;
}

static int arm720t_read_cp15(struct target *target, uint32_t opcode, uint32_t *value)
{
	/* fetch CP15 opcode */
	arm720t_scan_cp15(target, opcode, NULL, 1, 1);
	/* "DECODE" stage */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 1);
	/* "EXECUTE" stage (1) */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 0);
	arm720t_scan_cp15(target, 0x0, NULL, 0, 1);
	/* "EXECUTE" stage (2) */
	arm720t_scan_cp15(target, 0x0, NULL, 0, 1);
	/* "EXECUTE" stage (3), CDATA is read */
	arm720t_scan_cp15(target, ARMV4_5_NOP, value, 1, 1);

	return ERROR_OK;
}

static int arm720t_write_cp15(struct target *target, uint32_t opcode, uint32_t value)
{
	/* fetch CP15 opcode */
	arm720t_scan_cp15(target, opcode, NULL, 1, 1);
	/* "DECODE" stage */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 1);
	/* "EXECUTE" stage (1) */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 0);
	arm720t_scan_cp15(target, 0x0, NULL, 0, 1);
	/* "EXECUTE" stage (2) */
	arm720t_scan_cp15(target, value, NULL, 0, 1);
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 1);

	return ERROR_OK;
}

static int arm720t_get_ttb(struct target *target, uint32_t *result)
{
	uint32_t ttb = 0x0;

	int retval;

	retval = arm720t_read_cp15(target, 0xee120f10, &ttb);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	ttb &= 0xffffc000;

	*result = ttb;

	return ERROR_OK;
}

static int arm720t_disable_mmu_caches(struct target *target,
		int mmu, int d_u_cache, int i_cache)
{
	uint32_t cp15_control;
	int retval;

	/* read cp15 control register */
	retval = arm720t_read_cp15(target, 0xee110f10, &cp15_control);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (mmu)
		cp15_control &= ~0x1U;

	if (d_u_cache || i_cache)
		cp15_control &= ~0x4U;

	retval = arm720t_write_cp15(target, 0xee010f10, cp15_control);
	return retval;
}

static int arm720t_enable_mmu_caches(struct target *target,
		int mmu, int d_u_cache, int i_cache)
{
	uint32_t cp15_control;
	int retval;

	/* read cp15 control register */
	retval = arm720t_read_cp15(target, 0xee110f10, &cp15_control);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (mmu)
		cp15_control |= 0x1U;

	if (d_u_cache || i_cache)
		cp15_control |= 0x4U;

	retval = arm720t_write_cp15(target, 0xee010f10, cp15_control);
	return retval;
}

static int arm720t_post_debug_entry(struct target *target)
{
	struct arm720t_common *arm720t = target_to_arm720(target);
	int retval;

	/* examine cp15 control reg */
	retval = arm720t_read_cp15(target, 0xee110f10, &arm720t->cp15_control_reg);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("cp15_control_reg: %8.8" PRIx32 "", arm720t->cp15_control_reg);

	arm720t->armv4_5_mmu.mmu_enabled = (arm720t->cp15_control_reg & 0x1U) ? 1 : 0;
	arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = (arm720t->cp15_control_reg & 0x4U) ? 1 : 0;
	arm720t->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	/* save i/d fault status and address register */
	retval = arm720t_read_cp15(target, 0xee150f10, &arm720t->fsr_reg);
	if (retval != ERROR_OK)
		return retval;
	retval = arm720t_read_cp15(target, 0xee160f10, &arm720t->far_reg);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	return retval;
}

static void arm720t_pre_restore_context(struct target *target)
{
	struct arm720t_common *arm720t = target_to_arm720(target);

	/* restore i/d fault status and address register */
	arm720t_write_cp15(target, 0xee050f10, arm720t->fsr_reg);
	arm720t_write_cp15(target, 0xee060f10, arm720t->far_reg);
}

static int arm720t_arch_state(struct target *target)
{
	struct arm720t_common *arm720t = target_to_arm720(target);

	static const char *state[] = {
		"disabled", "enabled"
	};

	arm_arch_state(target);
	LOG_USER("MMU: %s, Cache: %s",
			 state[arm720t->armv4_5_mmu.mmu_enabled],
			 state[arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled]);

	return ERROR_OK;
}

static int arm720_mmu(struct target *target, int *enabled)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_INVALID;
	}

	*enabled = target_to_arm720(target)->armv4_5_mmu.mmu_enabled;
	return ERROR_OK;
}

static int arm720_virt2phys(struct target *target,
		target_addr_t virtual, target_addr_t *physical)
{
	uint32_t cb;
	struct arm720t_common *arm720t = target_to_arm720(target);

	uint32_t ret;
	int retval = armv4_5_mmu_translate_va(target,
			&arm720t->armv4_5_mmu, virtual, &cb, &ret);
	if (retval != ERROR_OK)
		return retval;
	*physical = ret;
	return ERROR_OK;
}

static int arm720t_read_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;
	struct arm720t_common *arm720t = target_to_arm720(target);

	/* disable cache, but leave MMU enabled */
	if (arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled) {
		retval = arm720t_disable_mmu_caches(target, 0, 1, 0);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = arm7_9_read_memory(target, address, size, count, buffer);

	if (arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled) {
		retval = arm720t_enable_mmu_caches(target, 0, 1, 0);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int arm720t_read_phys_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct arm720t_common *arm720t = target_to_arm720(target);

	return armv4_5_mmu_read_physical(target, &arm720t->armv4_5_mmu, address, size, count, buffer);
}

static int arm720t_write_phys_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct arm720t_common *arm720t = target_to_arm720(target);

	return armv4_5_mmu_write_physical(target, &arm720t->armv4_5_mmu, address, size, count, buffer);
}

static int arm720t_soft_reset_halt(struct target *target)
{
	int retval = ERROR_OK;
	struct arm720t_common *arm720t = target_to_arm720(target);
	struct reg *dbg_stat = &arm720t->arm7_9_common
			.eice_cache->reg_list[EICE_DBG_STAT];
	struct arm *arm = &arm720t->arm7_9_common.arm;

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
		if (debug_level >= 3)
			alive_sleep(100);
		else
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

	retval = arm720t_disable_mmu_caches(target, 1, 1, 1);
	if (retval != ERROR_OK)
		return retval;
	arm720t->armv4_5_mmu.mmu_enabled = 0;
	arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 0;
	arm720t->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	retval = target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int arm720t_init_target(struct command_context *cmd_ctx, struct target *target)
{
	return arm7tdmi_init_target(cmd_ctx, target);
}

static void arm720t_deinit_target(struct target *target)
{
	arm7tdmi_deinit_target(target);
}

/* FIXME remove forward decls */
static int arm720t_mrc(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2,
		uint32_t crn, uint32_t crm,
		uint32_t *value);
static int arm720t_mcr(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2,
		uint32_t crn, uint32_t crm,
		uint32_t value);

static int arm720t_init_arch_info(struct target *target,
		struct arm720t_common *arm720t, struct jtag_tap *tap)
{
	struct arm7_9_common *arm7_9 = &arm720t->arm7_9_common;

	arm7_9->arm.mrc = arm720t_mrc;
	arm7_9->arm.mcr = arm720t_mcr;

	arm7tdmi_init_arch_info(target, arm7_9, tap);

	arm720t->common_magic = ARM720T_COMMON_MAGIC;

	arm7_9->post_debug_entry = arm720t_post_debug_entry;
	arm7_9->pre_restore_context = arm720t_pre_restore_context;

	arm720t->armv4_5_mmu.armv4_5_cache.ctype = -1;
	arm720t->armv4_5_mmu.get_ttb = arm720t_get_ttb;
	arm720t->armv4_5_mmu.read_memory = arm7_9_read_memory;
	arm720t->armv4_5_mmu.write_memory = arm7_9_write_memory;
	arm720t->armv4_5_mmu.disable_mmu_caches = arm720t_disable_mmu_caches;
	arm720t->armv4_5_mmu.enable_mmu_caches = arm720t_enable_mmu_caches;
	arm720t->armv4_5_mmu.has_tiny_pages = 0;
	arm720t->armv4_5_mmu.mmu_enabled = 0;

	return ERROR_OK;
}

static int arm720t_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm720t_common *arm720t = calloc(1, sizeof(*arm720t));

	arm720t->arm7_9_common.arm.arch = ARM_ARCH_V4;
	return arm720t_init_arch_info(target, arm720t, target->tap);
}

static int arm720t_mrc(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2,
		uint32_t crn, uint32_t crm,
		uint32_t *value)
{
	if (cpnum != 15) {
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}

	/* read "to" r0 */
	return arm720t_read_cp15(target,
			ARMV4_5_MRC(cpnum, op1, 0, crn, crm, op2),
			value);

}

static int arm720t_mcr(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2,
		uint32_t crn, uint32_t crm,
		uint32_t value)
{
	if (cpnum != 15) {
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}

	/* write "from" r0 */
	return arm720t_write_cp15(target,
			ARMV4_5_MCR(cpnum, op1, 0, crn, crm, op2),
			value);
}

static const struct command_registration arm720t_command_handlers[] = {
	{
		.chain = arm7_9_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM720 targets. */
struct target_type arm720t_target = {
	.name = "arm720t",

	.poll = arm7_9_poll,
	.arch_state = arm720t_arch_state,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm720t_soft_reset_halt,

	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = arm720t_read_memory,
	.write_memory = arm7_9_write_memory_opt,
	.read_phys_memory = arm720t_read_phys_memory,
	.write_phys_memory = arm720t_write_phys_memory,
	.mmu = arm720_mmu,
	.virt2phys = arm720_virt2phys,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.commands = arm720t_command_handlers,
	.target_create = arm720t_target_create,
	.init_target = arm720t_init_target,
	.deinit_target = arm720t_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
