/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
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

#include "avrt.h"
#include "target.h"
#include "target_type.h"

#define AVR_JTAG_INS_LEN	4

/* forward declarations */
static int avr_target_create(struct target *target, Jim_Interp *interp);
static int avr_init_target(struct command_context *cmd_ctx, struct target *target);

static int avr_arch_state(struct target *target);
static int avr_poll(struct target *target);
static int avr_halt(struct target *target);
static int avr_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution);
static int avr_step(struct target *target, int current, target_addr_t address,
		int handle_breakpoints);

static int avr_assert_reset(struct target *target);
static int avr_deassert_reset(struct target *target);

/* IR and DR functions */
static int mcu_write_ir(struct jtag_tap *tap, uint8_t *ir_in, uint8_t *ir_out, int ir_len, int rti);
static int mcu_write_dr(struct jtag_tap *tap, uint8_t *dr_in, uint8_t *dr_out, int dr_len, int rti);
static int mcu_write_ir_u8(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out, int ir_len, int rti);
static int mcu_write_dr_u32(struct jtag_tap *tap, uint32_t *ir_in, uint32_t ir_out, int dr_len, int rti);

struct target_type avr_target = {
	.name = "avr",

	.poll = avr_poll,
	.arch_state = avr_arch_state,

	.halt = avr_halt,
	.resume = avr_resume,
	.step = avr_step,

	.assert_reset = avr_assert_reset,
	.deassert_reset = avr_deassert_reset,
/*
	.get_gdb_reg_list = avr_get_gdb_reg_list,

	.read_memory = avr_read_memory,
	.write_memory = avr_write_memory,
	.bulk_write_memory = avr_bulk_write_memory,
	.checksum_memory = avr_checksum_memory,
	.blank_check_memory = avr_blank_check_memory,

	.run_algorithm = avr_run_algorithm,

	.add_breakpoint = avr_add_breakpoint,
	.remove_breakpoint = avr_remove_breakpoint,
	.add_watchpoint = avr_add_watchpoint,
	.remove_watchpoint = avr_remove_watchpoint,
*/
	.target_create = avr_target_create,
	.init_target = avr_init_target,
};

static int avr_target_create(struct target *target, Jim_Interp *interp)
{
	struct avr_common *avr = calloc(1, sizeof(struct avr_common));

	avr->jtag_info.tap = target->tap;
	target->arch_info = avr;

	return ERROR_OK;
}

static int avr_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_poll(struct target *target)
{
	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_DEBUG_RUNNING))
		target->state = TARGET_HALTED;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_step(struct target *target, int current, target_addr_t address, int handle_breakpoints)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_assert_reset(struct target *target)
{
	target->state = TARGET_RESET;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int avr_deassert_reset(struct target *target)
{
	target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

int avr_jtag_senddat(struct jtag_tap *tap, uint32_t *dr_in, uint32_t dr_out,
		int len)
{
	return mcu_write_dr_u32(tap, dr_in, dr_out, len, 1);
}

int avr_jtag_sendinstr(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out)
{
	return mcu_write_ir_u8(tap, ir_in, ir_out, AVR_JTAG_INS_LEN, 1);
}

/* IR and DR functions */
static int mcu_write_ir(struct jtag_tap *tap, uint8_t *ir_in, uint8_t *ir_out,
		int ir_len, int rti)
{
	if (!tap) {
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}
	if (ir_len != tap->ir_length) {
		LOG_ERROR("invalid ir_len");
		return ERROR_FAIL;
	}

	{
		jtag_add_plain_ir_scan(tap->ir_length, ir_out, ir_in, TAP_IDLE);
	}

	return ERROR_OK;
}

static int mcu_write_dr(struct jtag_tap *tap, uint8_t *dr_in, uint8_t *dr_out,
		int dr_len, int rti)
{
	if (!tap) {
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}

	{
		jtag_add_plain_dr_scan(dr_len, dr_out, dr_in, TAP_IDLE);
	}

	return ERROR_OK;
}

static int mcu_write_ir_u8(struct jtag_tap *tap, uint8_t *ir_in,
		uint8_t ir_out, int ir_len, int rti)
{
	if (ir_len > 8) {
		LOG_ERROR("ir_len overflow, maximum is 8");
		return ERROR_FAIL;
	}

	mcu_write_ir(tap, ir_in, &ir_out, ir_len, rti);

	return ERROR_OK;
}

static int mcu_write_dr_u32(struct jtag_tap *tap, uint32_t *dr_in,
		uint32_t dr_out, int dr_len, int rti)
{
	if (dr_len > 32) {
		LOG_ERROR("dr_len overflow, maximum is 32");
		return ERROR_FAIL;
	}

	mcu_write_dr(tap, (uint8_t *)dr_in, (uint8_t *)&dr_out, dr_len, rti);

	return ERROR_OK;
}

int mcu_execute_queue(void)
{
	return jtag_execute_queue();
}
