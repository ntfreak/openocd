/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#include "arm966e.h"
#include "target_type.h"
#include "arm_opcodes.h"

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

int arm966e_init_arch_info(struct target *target, struct arm966e_common *arm966e, struct jtag_tap *tap)
{
	struct arm7_9_common *arm7_9 = &arm966e->arm7_9_common;

	/* initialize arm7/arm9 specific info (including armv4_5) */
	arm9tdmi_init_arch_info(target, arm7_9, tap);

	arm966e->common_magic = ARM966E_COMMON_MAGIC;

	/* The ARM966E-S implements the ARMv5TE architecture which
	 * has the BKPT instruction, so we don't have to use a watchpoint comparator
	 */
	arm7_9->arm_bkpt = ARMV5_BKPT(0x0);
	arm7_9->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;

	return ERROR_OK;
}

static int arm966e_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm966e_common *arm966e = calloc(1, sizeof(struct arm966e_common));

	return arm966e_init_arch_info(target, arm966e, target->tap);
}

static void arm966e_deinit_target(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm966e_common *arm966e = target_to_arm966(target);

	arm7_9_deinit(target);
	arm_free_reg_cache(arm);
	free(arm966e);
}

static int arm966e_verify_pointer(struct command_invocation *cmd,
		struct arm966e_common *arm966e)
{
	if (arm966e->common_magic != ARM966E_COMMON_MAGIC) {
		command_print(cmd, "target is not an ARM966");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/*
 * REVISIT:  The "read_cp15" and "write_cp15" commands could hook up
 * to eventual mrc() and mcr() routines ... the reg_addr values being
 * constructed (for CP15 only) from Opcode_1, Opcode_2, and CRn values.
 * See section 7.3 of the ARM966E-S TRM.
 */

static int arm966e_read_cp15(struct target *target, int reg_addr, uint32_t *value)
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

	fields[1].in_value = (uint8_t *)value;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

	jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)value);


#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, *value);
#endif

	return ERROR_OK;
}

/* EXPORTED to str9x (flash) */
int arm966e_write_cp15(struct target *target, int reg_addr, uint32_t value)
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

	return ERROR_OK;
}

COMMAND_HANDLER(arm966e_handle_cp15_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct arm966e_common *arm966e = target_to_arm966(target);

	retval = arm966e_verify_pointer(CMD, arm966e);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* one or more argument, access a single register (write if second argument is given */
	if (CMD_ARGC >= 1) {
		uint32_t address;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

		if (CMD_ARGC == 1) {
			uint32_t value;
			retval = arm966e_read_cp15(target, address, &value);
			if (retval != ERROR_OK) {
				command_print(CMD,
						"couldn't access reg %" PRIu32,
						address);
				return ERROR_OK;
			}
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;

			command_print(CMD, "%" PRIu32 ": %8.8" PRIx32,
					address, value);
		} else if (CMD_ARGC == 2) {
			uint32_t value;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
			retval = arm966e_write_cp15(target, address, value);
			if (retval != ERROR_OK) {
				command_print(CMD,
						"couldn't access reg %" PRIu32,
						address);
				return ERROR_OK;
			}
			command_print(CMD, "%" PRIu32 ": %8.8" PRIx32,
					address, value);
		}
	}

	return ERROR_OK;
}

static const struct command_registration arm966e_exec_command_handlers[] = {
	{
		.name = "cp15",
		.handler = arm966e_handle_cp15_command,
		.mode = COMMAND_EXEC,
		.usage = "regnum [value]",
		.help = "display/modify cp15 register",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arm966e_command_handlers[] = {
	{
		.chain = arm9tdmi_command_handlers,
	},
	{
		.name = "arm966e",
		.mode = COMMAND_ANY,
		.help = "arm966e command group",
		.usage = "",
		.chain = arm966e_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM966 targets. */
struct target_type arm966e_target = {
	.name = "arm966e",

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

	.commands = arm966e_command_handlers,
	.target_create = arm966e_target_create,
	.init_target = arm9tdmi_init_target,
	.deinit_target = arm966e_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
