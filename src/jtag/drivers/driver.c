/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
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

#include <jtag/jtag.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/minidriver.h>
#include <helper/command.h>

struct jtag_callback_entry {
	struct jtag_callback_entry *next;

	jtag_callback_t callback;
	jtag_callback_data_t data0;
	jtag_callback_data_t data1;
	jtag_callback_data_t data2;
	jtag_callback_data_t data3;
};

static struct jtag_callback_entry *jtag_callback_queue_head;
static struct jtag_callback_entry *jtag_callback_queue_tail;

static void jtag_callback_queue_reset(void)
{
	jtag_callback_queue_head = NULL;
	jtag_callback_queue_tail = NULL;
}

/**
 * see jtag_add_ir_scan()
 *
 */
int interface_jtag_add_ir_scan(struct jtag_tap *active,
		const struct scan_field *in_fields, tap_state_t state)
{
	size_t num_taps = jtag_tap_count_enabled();

	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));
	struct scan_command *scan = cmd_queue_alloc(sizeof(struct scan_command));
	struct scan_field *out_fields = cmd_queue_alloc(num_taps  * sizeof(struct scan_field));

	jtag_queue_command(cmd);

	cmd->type = JTAG_SCAN;
	cmd->cmd.scan = scan;

	scan->ir_scan = true;
	scan->num_fields = num_taps;	/* one field per device */
	scan->fields = out_fields;
	scan->end_state = state;

	struct scan_field *field = out_fields;	/* keep track where we insert data */

	/* loop over all enabled TAPs */

	for (struct jtag_tap *tap = jtag_tap_next_enabled(NULL); tap; tap = jtag_tap_next_enabled(tap)) {
		/* search the input field list for fields for the current TAP */

		if (tap == active) {
			/* if TAP is listed in input fields, copy the value */
			tap->bypass = 0;

			jtag_scan_field_clone(field, in_fields);
		} else {
			/* if a TAP isn't listed in input fields, set it to BYPASS */

			tap->bypass = 1;

			field->num_bits = tap->ir_length;
			field->out_value = buf_set_ones(cmd_queue_alloc(DIV_ROUND_UP(tap->ir_length, 8)), tap->ir_length);
			field->in_value = NULL; /* do not collect input for tap's in bypass */
		}

		/* update device information */
		buf_cpy(field->out_value, tap->cur_instr, tap->ir_length);

		field++;
	}
	/* paranoia: jtag_tap_count_enabled() and jtag_tap_next_enabled() not in sync */
	assert(field == out_fields + num_taps);

	return ERROR_OK;
}

/**
 * see jtag_add_dr_scan()
 *
 */
int interface_jtag_add_dr_scan(struct jtag_tap *active, int in_num_fields,
		const struct scan_field *in_fields, tap_state_t state)
{
	/* count devices in bypass */

	size_t bypass_devices = 0;

	for (struct jtag_tap *tap = jtag_tap_next_enabled(NULL); tap; tap = jtag_tap_next_enabled(tap)) {
		if (tap->bypass)
			bypass_devices++;
	}

	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));
	struct scan_command *scan = cmd_queue_alloc(sizeof(struct scan_command));
	struct scan_field *out_fields = cmd_queue_alloc((in_num_fields + bypass_devices) * sizeof(struct scan_field));

	jtag_queue_command(cmd);

	cmd->type = JTAG_SCAN;
	cmd->cmd.scan = scan;

	scan->ir_scan = false;
	scan->num_fields = in_num_fields + bypass_devices;
	scan->fields = out_fields;
	scan->end_state = state;

	struct scan_field *field = out_fields;	/* keep track where we insert data */

	/* loop over all enabled TAPs */

	for (struct jtag_tap *tap = jtag_tap_next_enabled(NULL); tap; tap = jtag_tap_next_enabled(tap)) {
		/* if TAP is not bypassed insert matching input fields */

		if (!tap->bypass) {
			assert(active == tap);
#ifndef NDEBUG
			/* remember initial position for assert() */
			struct scan_field *start_field = field;
#endif /* NDEBUG */

			for (int j = 0; j < in_num_fields; j++) {
				jtag_scan_field_clone(field, in_fields + j);

				field++;
			}

			assert(field > start_field);	/* must have at least one input field per not bypassed TAP */
		}

		/* if a TAP is bypassed, generated a dummy bit*/
		else {
			field->num_bits = 1;
			field->out_value = NULL;
			field->in_value = NULL;

			field++;
		}
	}

	assert(field == out_fields + scan->num_fields); /* no superfluous input fields permitted */

	return ERROR_OK;
}

static int jtag_add_plain_scan(int num_bits, const uint8_t *out_bits,
		uint8_t *in_bits, tap_state_t state, bool ir_scan)
{
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));
	struct scan_command *scan = cmd_queue_alloc(sizeof(struct scan_command));
	struct scan_field *out_fields = cmd_queue_alloc(sizeof(struct scan_field));

	jtag_queue_command(cmd);

	cmd->type = JTAG_SCAN;
	cmd->cmd.scan = scan;

	scan->ir_scan = ir_scan;
	scan->num_fields = 1;
	scan->fields = out_fields;
	scan->end_state = state;

	out_fields->num_bits = num_bits;
	out_fields->out_value = buf_cpy(out_bits, cmd_queue_alloc(DIV_ROUND_UP(num_bits, 8)), num_bits);
	out_fields->in_value = in_bits;

	return ERROR_OK;
}

int interface_jtag_add_plain_dr_scan(int num_bits, const uint8_t *out_bits, uint8_t *in_bits, tap_state_t state)
{
	return jtag_add_plain_scan(num_bits, out_bits, in_bits, state, false);
}

int interface_jtag_add_plain_ir_scan(int num_bits, const uint8_t *out_bits, uint8_t *in_bits, tap_state_t state)
{
	return jtag_add_plain_scan(num_bits, out_bits, in_bits, state, true);
}

int interface_jtag_add_tlr(void)
{
	tap_state_t state = TAP_RESET;

	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_TLR_RESET;

	cmd->cmd.statemove = cmd_queue_alloc(sizeof(struct statemove_command));
	cmd->cmd.statemove->end_state = state;

	return ERROR_OK;
}

int interface_add_tms_seq(unsigned num_bits, const uint8_t *seq, enum tap_state state)
{
	struct jtag_command *cmd;

	cmd = cmd_queue_alloc(sizeof(struct jtag_command));
	if (!cmd)
		return ERROR_FAIL;

	cmd->type = JTAG_TMS;
	cmd->cmd.tms = cmd_queue_alloc(sizeof(*cmd->cmd.tms));
	if (!cmd->cmd.tms)
		return ERROR_FAIL;

	/* copy the bits; our caller doesn't guarantee they'll persist */
	cmd->cmd.tms->num_bits = num_bits;
	cmd->cmd.tms->bits = buf_cpy(seq,
			cmd_queue_alloc(DIV_ROUND_UP(num_bits, 8)), num_bits);
	if (!cmd->cmd.tms->bits)
		return ERROR_FAIL;

	jtag_queue_command(cmd);

	return ERROR_OK;
}

int interface_jtag_add_pathmove(int num_states, const tap_state_t *path)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_PATHMOVE;

	cmd->cmd.pathmove = cmd_queue_alloc(sizeof(struct pathmove_command));
	cmd->cmd.pathmove->num_states = num_states;
	cmd->cmd.pathmove->path = cmd_queue_alloc(sizeof(tap_state_t) * num_states);

	for (int i = 0; i < num_states; i++)
		cmd->cmd.pathmove->path[i] = path[i];

	return ERROR_OK;
}

int interface_jtag_add_runtest(int num_cycles, tap_state_t state)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_RUNTEST;

	cmd->cmd.runtest = cmd_queue_alloc(sizeof(struct runtest_command));
	cmd->cmd.runtest->num_cycles = num_cycles;
	cmd->cmd.runtest->end_state = state;

	return ERROR_OK;
}

int interface_jtag_add_clocks(int num_cycles)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_STABLECLOCKS;

	cmd->cmd.stableclocks = cmd_queue_alloc(sizeof(struct stableclocks_command));
	cmd->cmd.stableclocks->num_cycles = num_cycles;

	return ERROR_OK;
}

int interface_jtag_add_reset(int req_trst, int req_srst)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_RESET;

	cmd->cmd.reset = cmd_queue_alloc(sizeof(struct reset_command));
	cmd->cmd.reset->trst = req_trst;
	cmd->cmd.reset->srst = req_srst;

	return ERROR_OK;
}

int interface_jtag_add_sleep(uint32_t us)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_SLEEP;

	cmd->cmd.sleep = cmd_queue_alloc(sizeof(struct sleep_command));
	cmd->cmd.sleep->us = us;

	return ERROR_OK;
}

/* add callback to end of queue */
void interface_jtag_add_callback4(jtag_callback_t callback,
		jtag_callback_data_t data0, jtag_callback_data_t data1,
		jtag_callback_data_t data2, jtag_callback_data_t data3)
{
	struct jtag_callback_entry *entry = cmd_queue_alloc(sizeof(struct jtag_callback_entry));

	entry->next = NULL;
	entry->callback = callback;
	entry->data0 = data0;
	entry->data1 = data1;
	entry->data2 = data2;
	entry->data3 = data3;

	if (!jtag_callback_queue_head) {
		jtag_callback_queue_head = entry;
		jtag_callback_queue_tail = entry;
	} else {
		jtag_callback_queue_tail->next = entry;
		jtag_callback_queue_tail = entry;
	}
}

int interface_jtag_execute_queue(void)
{
	static int reentry;

	assert(reentry == 0);
	reentry++;

	int retval = default_interface_jtag_execute_queue();
	if (retval == ERROR_OK) {
		struct jtag_callback_entry *entry;
		for (entry = jtag_callback_queue_head; entry; entry = entry->next) {
			retval = entry->callback(entry->data0, entry->data1, entry->data2, entry->data3);
			if (retval != ERROR_OK)
				break;
		}
	}

	jtag_command_queue_reset();
	jtag_callback_queue_reset();

	reentry--;

	return retval;
}

static int jtag_convert_to_callback4(jtag_callback_data_t data0,
		jtag_callback_data_t data1, jtag_callback_data_t data2, jtag_callback_data_t data3)
{
	((jtag_callback1_t)data1)(data0);
	return ERROR_OK;
}

void interface_jtag_add_callback(jtag_callback1_t callback, jtag_callback_data_t data0)
{
	jtag_add_callback4(jtag_convert_to_callback4, data0, (jtag_callback_data_t)callback, 0, 0);
}

void jtag_add_callback(jtag_callback1_t f, jtag_callback_data_t data0)
{
	interface_jtag_add_callback(f, data0);
}

void jtag_add_callback4(jtag_callback_t f, jtag_callback_data_t data0,
		jtag_callback_data_t data1, jtag_callback_data_t data2,
		jtag_callback_data_t data3)
{
	interface_jtag_add_callback4(f, data0, data1, data2, data3);
}
