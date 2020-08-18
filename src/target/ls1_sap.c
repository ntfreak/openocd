/***************************************************************************
 *   Copyright (C) 2015 by Esben Haabendal                                 *
 *   eha@deif.com                                                          *
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
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"

#include <jtag/jtag.h>

struct ls1_sap {
	struct jtag_tap *tap;
};

static int ls1_sap_target_create(struct target *target, Jim_Interp *interp)
{
	struct ls1_sap *ls1_sap = calloc(1, sizeof(struct ls1_sap));

	ls1_sap->tap = target->tap;
	target->arch_info = ls1_sap;

	return ERROR_OK;
}

static int ls1_sap_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int ls1_sap_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int ls1_sap_poll(struct target *target)
{
	if ((target->state == TARGET_UNKNOWN) ||
	    (target->state == TARGET_RUNNING) ||
	    (target->state == TARGET_DEBUG_RUNNING))
		target->state = TARGET_HALTED;

	return ERROR_OK;
}

static int ls1_sap_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int ls1_sap_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int ls1_sap_step(struct target *target, int current, target_addr_t address,
				int handle_breakpoints)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int ls1_sap_assert_reset(struct target *target)
{
	target->state = TARGET_RESET;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int ls1_sap_deassert_reset(struct target *target)
{
	target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static void ls1_sap_set_instr(struct jtag_tap *tap, uint32_t new_instr)
{
	struct scan_field field;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) == new_instr)
		return;

	field.num_bits = tap->ir_length;
	uint8_t *t = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, new_instr);
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);
	free(t);
}

static void ls1_sap_set_addr_high(struct jtag_tap *tap, uint16_t addr_high)
{
	struct scan_field field;
	uint8_t buf[2] = { 0 };

	ls1_sap_set_instr(tap, 0x21);

	field.num_bits = 16;
	field.out_value = buf;
	buf_set_u32(buf, 0, 16, addr_high);
	field.in_value = NULL;
	field.check_value = NULL;
	field.check_mask = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

static void ls1_sap_memory_cmd(struct jtag_tap *tap, uint32_t address,
			       int32_t size, bool rnw)
{
	struct scan_field field;
	uint8_t cmd[8] = { 0 };

	ls1_sap_set_instr(tap, 0x24);

	field.num_bits = 64;
	field.out_value = cmd;
	buf_set_u64(cmd, 0, 9, 0);
	buf_set_u64(cmd, 9, 3, size);
	buf_set_u64(cmd, 12, 1, rnw);
	buf_set_u64(cmd, 13, 3, 0);
	buf_set_u64(cmd, 16, 32, address);
	buf_set_u64(cmd, 48, 16, 0);
	field.in_value = NULL;
	field.check_value = NULL;
	field.check_mask = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

static void ls1_sap_memory_read(struct jtag_tap *tap, uint32_t size,
				uint8_t *value)
{
	struct scan_field field;

	ls1_sap_set_instr(tap, 0x25);

	field.num_bits = 8 * size;
	field.out_value = NULL;
	field.in_value = value;
	field.check_value = NULL;
	field.check_mask = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

static void ls1_sap_memory_write(struct jtag_tap *tap, uint32_t size,
				 const uint8_t *value)
{
	struct scan_field field;

	ls1_sap_set_instr(tap, 0x25);

	field.num_bits = 8 * size;
	field.out_value = value;
	field.in_value = NULL;
	field.check_value = NULL;
	field.check_mask = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

static int ls1_sap_read_memory(struct target *target, target_addr_t address,
			       uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("Reading memory at physical address 0x%" TARGET_PRIxADDR
		  "; size %" PRIu32 "; count %" PRIu32, address, size, count);

	if (count == 0 || buffer == NULL)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ls1_sap_set_addr_high(target->tap, 0);

	while (count--) {
		ls1_sap_memory_cmd(target->tap, address, size, true);
		ls1_sap_memory_read(target->tap, size, buffer);
		address += size;
		buffer += size;
	}

	return jtag_execute_queue();
}

static int ls1_sap_write_memory(struct target *target, target_addr_t address,
				uint32_t size, uint32_t count,
				const uint8_t *buffer)
{
	LOG_DEBUG("Writing memory at physical address 0x%" TARGET_PRIxADDR
		  "; size %" PRIu32 "; count %" PRIu32, address, size, count);


	if (count == 0 || buffer == NULL)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ls1_sap_set_addr_high(target->tap, 0);

	while (count--) {
		ls1_sap_memory_cmd(target->tap, address, size, false);
		ls1_sap_memory_write(target->tap, size, buffer);
		address += size;
		buffer += size;
	}

	return jtag_execute_queue();
}

struct target_type ls1_sap_target = {
	.name = "ls1_sap",

	.target_create = ls1_sap_target_create,
	.init_target = ls1_sap_init_target,

	.poll = ls1_sap_poll,
	.arch_state = ls1_sap_arch_state,

	.halt = ls1_sap_halt,
	.resume = ls1_sap_resume,
	.step = ls1_sap_step,

	.assert_reset = ls1_sap_assert_reset,
	.deassert_reset = ls1_sap_deassert_reset,

	.read_memory = ls1_sap_read_memory,
	.write_memory = ls1_sap_write_memory,
};
