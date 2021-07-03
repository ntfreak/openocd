/***************************************************************************
*   Copyright (C) 2007 by Pavel Chromy                                    *
*   chromy@asix.cz                                                        *
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
#include "bitq.h"
#include <jtag/interface.h>

struct bitq_interface *bitq_interface; /* low level bit queue interface */

/* state of input queue */
struct bitq_state {
	struct jtag_command *cmd; /* command currently processed */
	int field_idx; /* index of field currently being processed */
	int bit_pos; /* position of bit currently being processed */
	int status; /* processing status */
};
static struct bitq_state bitq_in_state;

/*
 * input queue processing does not use jtag_read_buffer() to avoid unnecessary overhead
 * no parameters, makes use of stored state information
 */
static void bitq_in_proc(void)
{
	/* loop through the queue */
	while (bitq_in_state.cmd) {
		/* only JTAG_SCAN command may return data */
		if (bitq_in_state.cmd->type == JTAG_SCAN) {
			/* loop through the fields */
			while (bitq_in_state.field_idx < bitq_in_state.cmd->cmd.scan->num_fields) {
				struct scan_field *field;
				field = &bitq_in_state.cmd->cmd.scan->fields[bitq_in_state.field_idx];
				if (field->in_value) {
					/* field scanning */
					while (bitq_in_state.bit_pos < field->num_bits) {
						/* index of byte being scanned */
						int in_idx = bitq_in_state.bit_pos / 8;
						/* mask of next bit to be scanned */
						uint8_t in_mask = 1 << (bitq_in_state.bit_pos % 8);

						int tdo = bitq_interface->in();
						if (tdo < 0) {
							LOG_DEBUG_IO("bitq in EOF");
							return;
						}
						if (in_mask == 0x01)
							field->in_value[in_idx] = 0;
						if (tdo)
							field->in_value[in_idx] |= in_mask;
						bitq_in_state.bit_pos++;
					}
				}

				bitq_in_state.field_idx++;  /* advance to next field */
				bitq_in_state.bit_pos = 0;  /* start next field from the first bit */
			}
		}
		bitq_in_state.cmd = bitq_in_state.cmd->next;    /* advance to next command */
		bitq_in_state.field_idx = 0;                    /* preselect first field */
	}
}

static void bitq_io(int tms, int tdi, int tdo_req)
{
	bitq_interface->out(tms, tdi, tdo_req);
	/* check and process the input queue */
	if (bitq_interface->in_rdy())
		bitq_in_proc();
}

static void bitq_end_state(tap_state_t state)
{
	if (!tap_is_state_stable(state)) {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
	tap_set_end_state(state);
}

static void bitq_state_move(tap_state_t new_state)
{
	int i = 0;
	uint8_t  tms_scan;

	if (!tap_is_state_stable(tap_get_state()) || !tap_is_state_stable(new_state)) {
		LOG_ERROR("TAP move from or to unstable state");
		exit(-1);
	}

	tms_scan = tap_get_tms_path(tap_get_state(), new_state);
	int tms_count = tap_get_tms_path_len(tap_get_state(), new_state);

	for (i = 0; i < tms_count; i++) {
		bitq_io(tms_scan & 1, 0, 0);
		tms_scan >>= 1;
	}

	tap_set_state(new_state);
}

static void bitq_path_move(struct pathmove_command *cmd)
{
	int i;

	for (i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[i])
			bitq_io(0, 0, 0);
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			bitq_io(1, 0, 0);
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(
							 tap_get_state()), tap_state_name(cmd->path[i]));
			exit(-1);
		}

		tap_set_state(cmd->path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static void bitq_runtest(int num_cycles)
{
	int i;

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
		bitq_state_move(TAP_IDLE);

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
		bitq_io(0, 0, 0);

	/* finish in end_state */
	if (tap_get_state() != tap_get_end_state())
		bitq_state_move(tap_get_end_state());
}

static void bitq_scan_field(struct scan_field *field, int do_pause)
{
	int bit_cnt;
	int tdo_req;

	const uint8_t *out_ptr;
	uint8_t  out_mask;

	if (field->in_value)
		tdo_req = 1;
	else
		tdo_req = 0;

	if (!field->out_value) {
		/* just send zeros and request data from TDO */
		for (bit_cnt = field->num_bits; bit_cnt > 1; bit_cnt--)
			bitq_io(0, 0, tdo_req);

		bitq_io(do_pause, 0, tdo_req);
	} else {
		/* send data, and optionally request TDO */
		out_mask = 0x01;
		out_ptr  = field->out_value;
		for (bit_cnt = field->num_bits; bit_cnt > 1; bit_cnt--) {
			bitq_io(0, ((*out_ptr) & out_mask) != 0, tdo_req);
			if (out_mask == 0x80) {
				out_mask = 0x01;
				out_ptr++;
			} else
				out_mask <<= 1;
		}

		bitq_io(do_pause, ((*out_ptr) & out_mask) != 0, tdo_req);
	}

	if (do_pause) {
		bitq_io(0, 0, 0);
		if (tap_get_state() == TAP_IRSHIFT)
			tap_set_state(TAP_IRPAUSE);
		else if (tap_get_state() == TAP_DRSHIFT)
			tap_set_state(TAP_DRPAUSE);
	}
}

static void bitq_scan(struct scan_command *cmd)
{
	int i;

	if (cmd->ir_scan)
		bitq_state_move(TAP_IRSHIFT);
	else
		bitq_state_move(TAP_DRSHIFT);

	for (i = 0; i < cmd->num_fields - 1; i++)
		bitq_scan_field(&cmd->fields[i], 0);

	bitq_scan_field(&cmd->fields[i], 1);
}

int bitq_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue; /* currently processed command */

	bitq_in_state.cmd = jtag_command_queue;
	bitq_in_state.field_idx = 0;
	bitq_in_state.bit_pos   = 0;
	bitq_in_state.status    = ERROR_OK;

	while (cmd) {
		switch (cmd->type) {
		case JTAG_RESET:
			LOG_DEBUG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			if ((cmd->cmd.reset->trst == 1) ||
					(cmd->cmd.reset->srst &&
					(jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
				tap_set_state(TAP_RESET);
			bitq_interface->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			if (bitq_interface->in_rdy())
				bitq_in_proc();
			break;

		case JTAG_RUNTEST:
			LOG_DEBUG_IO("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
			bitq_end_state(cmd->cmd.runtest->end_state);
			bitq_runtest(cmd->cmd.runtest->num_cycles);
			break;

		case JTAG_TLR_RESET:
			LOG_DEBUG_IO("statemove end in %i", cmd->cmd.statemove->end_state);
			bitq_end_state(cmd->cmd.statemove->end_state);
			bitq_state_move(tap_get_end_state());   /* unconditional TAP move */
			break;

		case JTAG_PATHMOVE:
			LOG_DEBUG_IO("pathmove: %i states, end in %i", cmd->cmd.pathmove->num_states,
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
			bitq_path_move(cmd->cmd.pathmove);
			break;

		case JTAG_SCAN:
			LOG_DEBUG_IO("scan end in %i", cmd->cmd.scan->end_state);
			LOG_DEBUG_IO("scan %s", cmd->cmd.scan->ir_scan ? "ir" : "dr");
			bitq_end_state(cmd->cmd.scan->end_state);
			bitq_scan(cmd->cmd.scan);
			if (tap_get_state() != tap_get_end_state())
				bitq_state_move(tap_get_end_state());
			break;

		case JTAG_SLEEP:
			LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
			bitq_interface->sleep(cmd->cmd.sleep->us);
			if (bitq_interface->in_rdy())
				bitq_in_proc();
			break;

		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
		}

		cmd = cmd->next;
	}

	bitq_interface->flush();
	bitq_in_proc();

	if (bitq_in_state.cmd) {
		LOG_ERROR("missing data from bitq interface");
		return ERROR_JTAG_QUEUE_FAILED;
	}
	if (bitq_interface->in() >= 0) {
		LOG_ERROR("extra data from bitq interface");
		return ERROR_JTAG_QUEUE_FAILED;
	}

	return bitq_in_state.status;
}

void bitq_cleanup(void)
{
}
