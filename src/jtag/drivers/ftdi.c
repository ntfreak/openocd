/**************************************************************************
*   Copyright (C) 2012 by Andreas Fritiofson                              *
*   andreas.fritiofson@gmail.com                                          *
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

/**
 * @file
 * JTAG adapters based on the FT2232 full and high speed USB parts are
 * popular low cost JTAG debug solutions.  Many FT2232 based JTAG adapters
 * are discrete, but development boards may integrate them as alternatives
 * to more capable (and expensive) third party JTAG pods.
 *
 * JTAG uses only one of the two communications channels ("MPSSE engines")
 * on these devices.  Adapters based on FT4232 parts have four ports/channels
 * (A/B/C/D), instead of just two (A/B).
 *
 * Especially on development boards integrating one of these chips (as
 * opposed to discrete pods/dongles), the additional channels can be used
 * for a variety of purposes, but OpenOCD only uses one channel at a time.
 *
 *  - As a USB-to-serial adapter for the target's console UART ...
 *    which may be able to support ROM boot loaders that load initial
 *    firmware images to flash (or SRAM).
 *
 *  - On systems which support ARM's SWD in addition to JTAG, or instead
 *    of it, that second port can be used for reading SWV/SWO trace data.
 *
 *  - Additional JTAG links, e.g. to a CPLD or * FPGA.
 *
 * FT2232 based JTAG adapters are "dumb" not "smart", because most JTAG
 * request/response interactions involve round trips over the USB link.
 * A "smart" JTAG adapter has intelligence close to the scan chain, so it
 * can for example poll quickly for a status change (usually taking on the
 * order of microseconds not milliseconds) before beginning a queued
 * transaction which require the previous one to have completed.
 *
 * There are dozens of adapters of this type, differing in details which
 * this driver needs to understand.  Those "layout" details are required
 * as part of FT2232 driver configuration.
 *
 * This code uses information contained in the MPSSE specification which was
 * found here:
 * https://www.ftdichip.com/Support/Documents/AppNotes/AN2232C-01_MPSSE_Cmnd.pdf
 * Hereafter this is called the "MPSSE Spec".
 *
 * The datasheet for the ftdichip.com's FT2232H part is here:
 * https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT2232H.pdf
 *
 * Also note the issue with code 0x4b (clock data to TMS) noted in
 * http://developer.intra2net.com/mailarchive/html/libftdi/2009/msg00292.html
 * which can affect longer JTAG state paths.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/drivers/jtag_usb_common.h>
#include <jtag/interface.h>
#include <jtag/swd.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#if IS_CYGWIN == 1
#include <windows.h>
#endif

#include <assert.h>

/* FTDI access library includes */
#include "mpsse.h"

#define JTAG_MODE (LSB_FIRST | POS_EDGE_IN | NEG_EDGE_OUT)
#define JTAG_MODE_ALT (LSB_FIRST | NEG_EDGE_IN | NEG_EDGE_OUT)
#define SWD_MODE (LSB_FIRST | POS_EDGE_IN | NEG_EDGE_OUT)

static char *ftdi_device_desc;
static char *ftdi_serial;
static uint8_t ftdi_channel;
static uint8_t ftdi_jtag_mode = JTAG_MODE;

static bool swd_mode;

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t ftdi_vid[MAX_USB_IDS + 1] = { 0 };
static uint16_t ftdi_pid[MAX_USB_IDS + 1] = { 0 };

static struct mpsse_ctx *mpsse_ctx;

struct signal {
	const char *name;
	uint16_t data_mask;
	uint16_t input_mask;
	uint16_t oe_mask;
	bool invert_data;
	bool invert_input;
	bool invert_oe;
	struct signal *next;
};

static struct signal *signals;

/* FIXME: Where to store per-instance data? We need an SWD context. */
static struct swd_cmd_queue_entry {
	uint8_t cmd;
	uint32_t *dst;
	uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
} *swd_cmd_queue;
static size_t swd_cmd_queue_length;
static size_t swd_cmd_queue_alloced;
static int queued_retval;
static int freq;

static uint16_t output;
static uint16_t direction;
static uint16_t jtag_output_init;
static uint16_t jtag_direction_init;

static int ftdi_swd_switch_seq(enum swd_special_seq seq);

static struct signal *find_signal_by_name(const char *name)
{
	for (struct signal *sig = signals; sig; sig = sig->next) {
		if (strcmp(name, sig->name) == 0)
			return sig;
	}
	return NULL;
}

static struct signal *create_signal(const char *name)
{
	struct signal **psig = &signals;
	while (*psig)
		psig = &(*psig)->next;

	*psig = calloc(1, sizeof(**psig));
	if (*psig == NULL)
		return NULL;

	(*psig)->name = strdup(name);
	if ((*psig)->name == NULL) {
		free(*psig);
		*psig = NULL;
	}
	return *psig;
}

static int ftdi_set_signal(const struct signal *s, char value)
{
	bool data;
	bool oe;

	if (s->data_mask == 0 && s->oe_mask == 0) {
		LOG_ERROR("interface doesn't provide signal '%s'", s->name);
		return ERROR_FAIL;
	}
	switch (value) {
	case '0':
		data = s->invert_data;
		oe = !s->invert_oe;
		break;
	case '1':
		if (s->data_mask == 0) {
			LOG_ERROR("interface can't drive '%s' high", s->name);
			return ERROR_FAIL;
		}
		data = !s->invert_data;
		oe = !s->invert_oe;
		break;
	case 'z':
	case 'Z':
		if (s->oe_mask == 0) {
			LOG_ERROR("interface can't tri-state '%s'", s->name);
			return ERROR_FAIL;
		}
		data = s->invert_data;
		oe = s->invert_oe;
		break;
	default:
		assert(0 && "invalid signal level specifier");
		return ERROR_FAIL;
	}

	uint16_t old_output = output;
	uint16_t old_direction = direction;

	output = data ? output | s->data_mask : output & ~s->data_mask;
	if (s->oe_mask == s->data_mask)
		direction = oe ? direction | s->oe_mask : direction & ~s->oe_mask;
	else
		output = oe ? output | s->oe_mask : output & ~s->oe_mask;

	if ((output & 0xff) != (old_output & 0xff) || (direction & 0xff) != (old_direction & 0xff))
		mpsse_set_data_bits_low_byte(mpsse_ctx, output & 0xff, direction & 0xff);
	if ((output >> 8 != old_output >> 8) || (direction >> 8 != old_direction >> 8))
		mpsse_set_data_bits_high_byte(mpsse_ctx, output >> 8, direction >> 8);

	return ERROR_OK;
}

static int ftdi_get_signal(const struct signal *s, uint16_t *value_out)
{
	uint8_t data_low = 0;
	uint8_t data_high = 0;

	if (s->input_mask == 0) {
		LOG_ERROR("interface doesn't provide signal '%s'", s->name);
		return ERROR_FAIL;
	}

	if (s->input_mask & 0xff)
		mpsse_read_data_bits_low_byte(mpsse_ctx, &data_low);
	if (s->input_mask >> 8)
		mpsse_read_data_bits_high_byte(mpsse_ctx, &data_high);

	mpsse_flush(mpsse_ctx);

	*value_out = (((uint16_t)data_high) << 8) | data_low;

	if (s->invert_input)
		*value_out = ~(*value_out);

	*value_out &= s->input_mask;

	return ERROR_OK;
}

/**
 * Function move_to_state
 * moves the TAP controller from the current state to a
 * \a goal_state through a path given by tap_get_tms_path().  State transition
 * logging is performed by delegation to clock_tms().
 *
 * @param goal_state is the destination state for the move.
 */
static void move_to_state(tap_state_t goal_state)
{
	tap_state_t start_state = tap_get_state();

	/*	goal_state is 1/2 of a tuple/pair of states which allow convenient
		lookup of the required TMS pattern to move to this state from the
		start state.
	*/

	/* do the 2 lookups */
	uint8_t tms_bits  = tap_get_tms_path(start_state, goal_state);
	int tms_count = tap_get_tms_path_len(start_state, goal_state);
	assert(tms_count <= 8);

	LOG_DEBUG_IO("start=%s goal=%s", tap_state_name(start_state), tap_state_name(goal_state));

	/* Track state transitions step by step */
	for (int i = 0; i < tms_count; i++)
		tap_set_state(tap_state_transition(tap_get_state(), (tms_bits >> i) & 1));

	mpsse_clock_tms_cs_out(mpsse_ctx,
		&tms_bits,
		0,
		tms_count,
		false,
		ftdi_jtag_mode);
}

static int ftdi_speed(int speed)
{
	int retval;
	retval = mpsse_set_frequency(mpsse_ctx, speed);

	if (retval < 0) {
		LOG_ERROR("couldn't set FTDI TCK speed");
		return retval;
	}

	if (!swd_mode && speed >= 10000000 && ftdi_jtag_mode != JTAG_MODE_ALT)
		LOG_INFO("ftdi: if you experience problems at higher adapter clocks, try "
			 "the command \"ftdi_tdo_sample_edge falling\"");
	return ERROR_OK;
}

static int ftdi_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int ftdi_khz(int khz, int *jtag_speed)
{
	if (khz == 0 && !mpsse_is_high_speed(mpsse_ctx)) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static void ftdi_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %s is not a stable end state", tap_state_name(state));
		exit(-1);
	}
}

static void ftdi_execute_runtest(struct jtag_command *cmd)
{
	int i;
	uint8_t zero = 0;

	LOG_DEBUG_IO("runtest %i cycles, end in %s",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(cmd->cmd.runtest->end_state));

	if (tap_get_state() != TAP_IDLE)
		move_to_state(TAP_IDLE);

	/* TODO: Reuse ftdi_execute_stableclocks */
	i = cmd->cmd.runtest->num_cycles;
	while (i > 0) {
		/* there are no state transitions in this code, so omit state tracking */
		unsigned this_len = i > 7 ? 7 : i;
		mpsse_clock_tms_cs_out(mpsse_ctx, &zero, 0, this_len, false, ftdi_jtag_mode);
		i -= this_len;
	}

	ftdi_end_state(cmd->cmd.runtest->end_state);

	if (tap_get_state() != tap_get_end_state())
		move_to_state(tap_get_end_state());

	LOG_DEBUG_IO("runtest: %i, end in %s",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(tap_get_end_state()));
}

static void ftdi_execute_statemove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("statemove end in %s",
		tap_state_name(cmd->cmd.statemove->end_state));

	ftdi_end_state(cmd->cmd.statemove->end_state);

	/* shortest-path move to desired end state */
	if (tap_get_state() != tap_get_end_state() || tap_get_end_state() == TAP_RESET)
		move_to_state(tap_get_end_state());
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine. REVISIT: Not the best method, perhaps.
 */
static void ftdi_execute_tms(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("TMS: %d bits", cmd->cmd.tms->num_bits);

	/* TODO: Missing tap state tracking, also missing from ft2232.c! */
	mpsse_clock_tms_cs_out(mpsse_ctx,
		cmd->cmd.tms->bits,
		0,
		cmd->cmd.tms->num_bits,
		false,
		ftdi_jtag_mode);
}

static void ftdi_execute_pathmove(struct jtag_command *cmd)
{
	tap_state_t *path = cmd->cmd.pathmove->path;
	int num_states  = cmd->cmd.pathmove->num_states;

	LOG_DEBUG_IO("pathmove: %i states, current: %s  end: %s", num_states,
		tap_state_name(tap_get_state()),
		tap_state_name(path[num_states-1]));

	int state_count = 0;
	unsigned bit_count = 0;
	uint8_t tms_byte = 0;

	LOG_DEBUG_IO("-");

	/* this loop verifies that the path is legal and logs each state in the path */
	while (num_states--) {

		/* either TMS=0 or TMS=1 must work ... */
		if (tap_state_transition(tap_get_state(), false)
		    == path[state_count])
			buf_set_u32(&tms_byte, bit_count++, 1, 0x0);
		else if (tap_state_transition(tap_get_state(), true)
			 == path[state_count]) {
			buf_set_u32(&tms_byte, bit_count++, 1, 0x1);

			/* ... or else the caller goofed BADLY */
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid "
				"TAP state transition",
				tap_state_name(tap_get_state()),
				tap_state_name(path[state_count]));
			exit(-1);
		}

		tap_set_state(path[state_count]);
		state_count++;

		if (bit_count == 7 || num_states == 0) {
			mpsse_clock_tms_cs_out(mpsse_ctx,
					&tms_byte,
					0,
					bit_count,
					false,
					ftdi_jtag_mode);
			bit_count = 0;
		}
	}
	tap_set_end_state(tap_get_state());
}

static void ftdi_execute_scan(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN",
		jtag_scan_type(cmd->cmd.scan));

	/* Make sure there are no trailing fields with num_bits == 0, or the logic below will fail. */
	while (cmd->cmd.scan->num_fields > 0
			&& cmd->cmd.scan->fields[cmd->cmd.scan->num_fields - 1].num_bits == 0) {
		cmd->cmd.scan->num_fields--;
		LOG_DEBUG_IO("discarding trailing empty field");
	}

	if (cmd->cmd.scan->num_fields == 0) {
		LOG_DEBUG_IO("empty scan, doing nothing");
		return;
	}

	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT)
			move_to_state(TAP_IRSHIFT);
	} else {
		if (tap_get_state() != TAP_DRSHIFT)
			move_to_state(TAP_DRSHIFT);
	}

	ftdi_end_state(cmd->cmd.scan->end_state);

	struct scan_field *field = cmd->cmd.scan->fields;
	unsigned scan_size = 0;

	for (int i = 0; i < cmd->cmd.scan->num_fields; i++, field++) {
		scan_size += field->num_bits;
		LOG_DEBUG_IO("%s%s field %d/%d %d bits",
			field->in_value ? "in" : "",
			field->out_value ? "out" : "",
			i,
			cmd->cmd.scan->num_fields,
			field->num_bits);

		if (i == cmd->cmd.scan->num_fields - 1 && tap_get_state() != tap_get_end_state()) {
			/* Last field, and we're leaving IRSHIFT/DRSHIFT. Clock last bit during tap
			 * movement. This last field can't have length zero, it was checked above. */
			mpsse_clock_data(mpsse_ctx,
				field->out_value,
				0,
				field->in_value,
				0,
				field->num_bits - 1,
				ftdi_jtag_mode);
			uint8_t last_bit = 0;
			if (field->out_value)
				bit_copy(&last_bit, 0, field->out_value, field->num_bits - 1, 1);
			uint8_t tms_bits = 0x01;
			mpsse_clock_tms_cs(mpsse_ctx,
					&tms_bits,
					0,
					field->in_value,
					field->num_bits - 1,
					1,
					last_bit,
					ftdi_jtag_mode);
			tap_set_state(tap_state_transition(tap_get_state(), 1));
			mpsse_clock_tms_cs_out(mpsse_ctx,
					&tms_bits,
					1,
					1,
					last_bit,
					ftdi_jtag_mode);
			tap_set_state(tap_state_transition(tap_get_state(), 0));
		} else
			mpsse_clock_data(mpsse_ctx,
				field->out_value,
				0,
				field->in_value,
				0,
				field->num_bits,
				ftdi_jtag_mode);
	}

	if (tap_get_state() != tap_get_end_state())
		move_to_state(tap_get_end_state());

	LOG_DEBUG_IO("%s scan, %i bits, end in %s",
		(cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
		tap_state_name(tap_get_end_state()));
}

static int ftdi_reset(int trst, int srst)
{
	struct signal *sig_ntrst = find_signal_by_name("nTRST");
	struct signal *sig_nsrst = find_signal_by_name("nSRST");

	LOG_DEBUG_IO("reset trst: %i srst %i", trst, srst);

	if (!swd_mode) {
		if (trst == 1) {
			if (sig_ntrst)
				ftdi_set_signal(sig_ntrst, '0');
			else
				LOG_ERROR("Can't assert TRST: nTRST signal is not defined");
		} else if (sig_ntrst && jtag_get_reset_config() & RESET_HAS_TRST &&
				trst == 0) {
			if (jtag_get_reset_config() & RESET_TRST_OPEN_DRAIN)
				ftdi_set_signal(sig_ntrst, 'z');
			else
				ftdi_set_signal(sig_ntrst, '1');
		}
	}

	if (srst == 1) {
		if (sig_nsrst)
			ftdi_set_signal(sig_nsrst, '0');
		else
			LOG_ERROR("Can't assert SRST: nSRST signal is not defined");
	} else if (sig_nsrst && jtag_get_reset_config() & RESET_HAS_SRST &&
			srst == 0) {
		if (jtag_get_reset_config() & RESET_SRST_PUSH_PULL)
			ftdi_set_signal(sig_nsrst, '1');
		else
			ftdi_set_signal(sig_nsrst, 'z');
	}

	return mpsse_flush(mpsse_ctx);
}

static void ftdi_execute_sleep(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);

	mpsse_flush(mpsse_ctx);
	jtag_sleep(cmd->cmd.sleep->us);
	LOG_DEBUG_IO("sleep %" PRIu32 " usec while in %s",
		cmd->cmd.sleep->us,
		tap_state_name(tap_get_state()));
}

static void ftdi_execute_stableclocks(struct jtag_command *cmd)
{
	/* this is only allowed while in a stable state.  A check for a stable
	 * state was done in jtag_add_clocks()
	 */
	int num_cycles = cmd->cmd.stableclocks->num_cycles;

	/* 7 bits of either ones or zeros. */
	uint8_t tms = tap_get_state() == TAP_RESET ? 0x7f : 0x00;

	/* TODO: Use mpsse_clock_data with in=out=0 for this, if TMS can be set to
	 * the correct level and remain there during the scan */
	while (num_cycles > 0) {
		/* there are no state transitions in this code, so omit state tracking */
		unsigned this_len = num_cycles > 7 ? 7 : num_cycles;
		mpsse_clock_tms_cs_out(mpsse_ctx, &tms, 0, this_len, false, ftdi_jtag_mode);
		num_cycles -= this_len;
	}

	LOG_DEBUG_IO("clocks %i while in %s",
		cmd->cmd.stableclocks->num_cycles,
		tap_state_name(tap_get_state()));
}

static void ftdi_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_RUNTEST:
			ftdi_execute_runtest(cmd);
			break;
		case JTAG_TLR_RESET:
			ftdi_execute_statemove(cmd);
			break;
		case JTAG_PATHMOVE:
			ftdi_execute_pathmove(cmd);
			break;
		case JTAG_SCAN:
			ftdi_execute_scan(cmd);
			break;
		case JTAG_SLEEP:
			ftdi_execute_sleep(cmd);
			break;
		case JTAG_STABLECLOCKS:
			ftdi_execute_stableclocks(cmd);
			break;
		case JTAG_TMS:
			ftdi_execute_tms(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered: %d", cmd->type);
			break;
	}
}

static int ftdi_execute_queue(void)
{
	/* blink, if the current layout has that feature */
	struct signal *led = find_signal_by_name("LED");
	if (led)
		ftdi_set_signal(led, '1');

	for (struct jtag_command *cmd = jtag_command_queue; cmd; cmd = cmd->next) {
		/* fill the write buffer with the desired command */
		ftdi_execute_command(cmd);
	}

	if (led)
		ftdi_set_signal(led, '0');

	int retval = mpsse_flush(mpsse_ctx);
	if (retval != ERROR_OK)
		LOG_ERROR("error while flushing MPSSE queue: %d", retval);

	return retval;
}

static int ftdi_initialize(void)
{
	if (tap_get_tms_path_len(TAP_IRPAUSE, TAP_IRPAUSE) == 7)
		LOG_DEBUG("ftdi interface using 7 step jtag state transitions");
	else
		LOG_DEBUG("ftdi interface using shortest path jtag state transitions");

	if (!ftdi_vid[0] && !ftdi_pid[0]) {
		LOG_ERROR("Please specify ftdi_vid_pid");
		return ERROR_JTAG_INIT_FAILED;
	}

	for (int i = 0; ftdi_vid[i] || ftdi_pid[i]; i++) {
		mpsse_ctx = mpsse_open(&ftdi_vid[i], &ftdi_pid[i], ftdi_device_desc,
				ftdi_serial, jtag_usb_get_location(), ftdi_channel);
		if (mpsse_ctx)
			break;
	}

	if (!mpsse_ctx)
		return ERROR_JTAG_INIT_FAILED;

	output = jtag_output_init;
	direction = jtag_direction_init;

	if (swd_mode) {
		struct signal *sig = find_signal_by_name("SWD_EN");
		if (!sig) {
			LOG_ERROR("SWD mode is active but SWD_EN signal is not defined");
			return ERROR_JTAG_INIT_FAILED;
		}
		/* A dummy SWD_EN would have zero mask */
		if (sig->data_mask)
			ftdi_set_signal(sig, '1');
	}

	mpsse_set_data_bits_low_byte(mpsse_ctx, output & 0xff, direction & 0xff);
	mpsse_set_data_bits_high_byte(mpsse_ctx, output >> 8, direction >> 8);

	mpsse_loopback_config(mpsse_ctx, false);

	freq = mpsse_set_frequency(mpsse_ctx, jtag_get_speed_khz() * 1000);

	return mpsse_flush(mpsse_ctx);
}

static int ftdi_quit(void)
{
	mpsse_close(mpsse_ctx);

	struct signal *sig = signals;
	while (sig) {
		struct signal *next = sig->next;
		free((void *)sig->name);
		free(sig);
		sig = next;
	}

	free(ftdi_device_desc);
	free(ftdi_serial);

	free(swd_cmd_queue);

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_device_desc_command)
{
	if (CMD_ARGC == 1) {
		free(ftdi_device_desc);
		ftdi_device_desc = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR("expected exactly one argument to ftdi_device_desc <description>");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_serial_command)
{
	if (CMD_ARGC == 1) {
		free(ftdi_serial);
		ftdi_serial = strdup(CMD_ARGV[0]);
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_channel_command)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], ftdi_channel);
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_layout_init_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], jtag_output_init);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], jtag_direction_init);

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_layout_signal_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bool invert_data = false;
	uint16_t data_mask = 0;
	bool invert_input = false;
	uint16_t input_mask = 0;
	bool invert_oe = false;
	uint16_t oe_mask = 0;
	for (unsigned i = 1; i < CMD_ARGC; i += 2) {
		if (strcmp("-data", CMD_ARGV[i]) == 0) {
			invert_data = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], data_mask);
		} else if (strcmp("-ndata", CMD_ARGV[i]) == 0) {
			invert_data = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], data_mask);
		} else if (strcmp("-input", CMD_ARGV[i]) == 0) {
			invert_input = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], input_mask);
		} else if (strcmp("-ninput", CMD_ARGV[i]) == 0) {
			invert_input = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], input_mask);
		} else if (strcmp("-oe", CMD_ARGV[i]) == 0) {
			invert_oe = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], oe_mask);
		} else if (strcmp("-noe", CMD_ARGV[i]) == 0) {
			invert_oe = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], oe_mask);
		} else if (!strcmp("-alias", CMD_ARGV[i]) ||
			   !strcmp("-nalias", CMD_ARGV[i])) {
			if (!strcmp("-nalias", CMD_ARGV[i])) {
				invert_data = true;
				invert_input = true;
			}
			struct signal *sig = find_signal_by_name(CMD_ARGV[i + 1]);
			if (!sig) {
				LOG_ERROR("signal %s is not defined", CMD_ARGV[i + 1]);
				return ERROR_FAIL;
			}
			data_mask = sig->data_mask;
			input_mask = sig->input_mask;
			oe_mask = sig->oe_mask;
			invert_input ^= sig->invert_input;
			invert_oe = sig->invert_oe;
			invert_data ^= sig->invert_data;
		} else {
			LOG_ERROR("unknown option '%s'", CMD_ARGV[i]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	struct signal *sig;
	sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig)
		sig = create_signal(CMD_ARGV[0]);
	if (!sig) {
		LOG_ERROR("failed to create signal %s", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	sig->invert_data = invert_data;
	sig->data_mask = data_mask;
	sig->invert_input = invert_input;
	sig->input_mask = input_mask;
	sig->invert_oe = invert_oe;
	sig->oe_mask = oe_mask;

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_set_signal_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct signal *sig;
	sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig) {
		LOG_ERROR("interface configuration doesn't define signal '%s'", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	switch (*CMD_ARGV[1]) {
	case '0':
	case '1':
	case 'z':
	case 'Z':
		/* single character level specifier only */
		if (CMD_ARGV[1][1] == '\0') {
			ftdi_set_signal(sig, *CMD_ARGV[1]);
			break;
		}
		/* fallthrough */
	default:
		LOG_ERROR("unknown signal level '%s', use 0, 1 or z", CMD_ARGV[1]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return mpsse_flush(mpsse_ctx);
}

COMMAND_HANDLER(ftdi_handle_get_signal_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct signal *sig;
	uint16_t sig_data = 0;
	sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig) {
		LOG_ERROR("interface configuration doesn't define signal '%s'", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	int ret = ftdi_get_signal(sig, &sig_data);
	if (ret != ERROR_OK)
		return ret;

	LOG_USER("Signal %s = %#06x", sig->name, sig_data);

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in ftdi_vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete ftdi_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], ftdi_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], ftdi_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * ftdi_vid_pid.
	 */
	ftdi_vid[i >> 1] = ftdi_pid[i >> 1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_tdo_sample_edge_command)
{
	Jim_Nvp *n;
	static const Jim_Nvp nvp_ftdi_jtag_modes[] = {
		{ .name = "rising", .value = JTAG_MODE },
		{ .name = "falling", .value = JTAG_MODE_ALT },
		{ .name = NULL, .value = -1 },
	};

	if (CMD_ARGC > 0) {
		n = Jim_Nvp_name2value_simple(nvp_ftdi_jtag_modes, CMD_ARGV[0]);
		if (n->name == NULL)
			return ERROR_COMMAND_SYNTAX_ERROR;
		ftdi_jtag_mode = n->value;

	}

	n = Jim_Nvp_value2name_simple(nvp_ftdi_jtag_modes, ftdi_jtag_mode);
	command_print(CMD, "ftdi samples TDO on %s edge of TCK", n->name);

	return ERROR_OK;
}

static const struct command_registration ftdi_command_handlers[] = {
	{
		.name = "ftdi_device_desc",
		.handler = &ftdi_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the FTDI device",
		.usage = "description_string",
	},
	{
		.name = "ftdi_serial",
		.handler = &ftdi_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the FTDI device",
		.usage = "serial_string",
	},
	{
		.name = "ftdi_channel",
		.handler = &ftdi_handle_channel_command,
		.mode = COMMAND_CONFIG,
		.help = "set the channel of the FTDI device that is used as JTAG",
		.usage = "(0-3)",
	},
	{
		.name = "ftdi_layout_init",
		.handler = &ftdi_handle_layout_init_command,
		.mode = COMMAND_CONFIG,
		.help = "initialize the FTDI GPIO signals used "
			"to control output-enables and reset signals",
		.usage = "data direction",
	},
	{
		.name = "ftdi_layout_signal",
		.handler = &ftdi_handle_layout_signal_command,
		.mode = COMMAND_ANY,
		.help = "define a signal controlled by one or more FTDI GPIO as data "
			"and/or output enable",
		.usage = "name [-data mask|-ndata mask] [-oe mask|-noe mask] [-alias|-nalias name]",
	},
	{
		.name = "ftdi_set_signal",
		.handler = &ftdi_handle_set_signal_command,
		.mode = COMMAND_EXEC,
		.help = "control a layout-specific signal",
		.usage = "name (1|0|z)",
	},
	{
		.name = "ftdi_get_signal",
		.handler = &ftdi_handle_get_signal_command,
		.mode = COMMAND_EXEC,
		.help = "read the value of a layout-specific signal",
		.usage = "name",
	},
	{
		.name = "ftdi_vid_pid",
		.handler = &ftdi_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the FTDI device",
		.usage = "(vid pid)* ",
	},
	{
		.name = "ftdi_tdo_sample_edge",
		.handler = &ftdi_handle_tdo_sample_edge_command,
		.mode = COMMAND_ANY,
		.help = "set which TCK clock edge is used for sampling TDO "
			"- default is rising-edge (Setting to falling-edge may "
			"allow signalling speed increase)",
		.usage = "(rising|falling)",
	},
	COMMAND_REGISTRATION_DONE
};

static int create_default_signal(const char *name, uint16_t data_mask)
{
	struct signal *sig = create_signal(name);
	if (!sig) {
		LOG_ERROR("failed to create signal %s", name);
		return ERROR_FAIL;
	}
	sig->invert_data = false;
	sig->data_mask = data_mask;
	sig->invert_oe = false;
	sig->oe_mask = 0;

	return ERROR_OK;
}

static int create_signals(void)
{
	if (create_default_signal("TCK", 0x01) != ERROR_OK)
		return ERROR_FAIL;
	if (create_default_signal("TDI", 0x02) != ERROR_OK)
		return ERROR_FAIL;
	if (create_default_signal("TDO", 0x04) != ERROR_OK)
		return ERROR_FAIL;
	if (create_default_signal("TMS", 0x08) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int ftdi_swd_init(void)
{
	LOG_INFO("FTDI SWD mode enabled");
	swd_mode = true;

	if (create_signals() != ERROR_OK)
		return ERROR_FAIL;

	swd_cmd_queue_alloced = 10;
	swd_cmd_queue = malloc(swd_cmd_queue_alloced * sizeof(*swd_cmd_queue));

	return swd_cmd_queue != NULL ? ERROR_OK : ERROR_FAIL;
}

static void ftdi_swd_swdio_en(bool enable)
{
	struct signal *oe = find_signal_by_name("SWDIO_OE");
	if (oe) {
		if (oe->data_mask)
			ftdi_set_signal(oe, enable ? '1' : '0');
		else {
			/* Sets TDI/DO pin to input during rx when both pins are connected
			   to SWDIO */
			if (enable)
				direction |= jtag_direction_init & 0x0002U;
			else
				direction &= ~0x0002U;
			mpsse_set_data_bits_low_byte(mpsse_ctx, output & 0xff, direction & 0xff);
		}
	}
}

/**
 * Flush the MPSSE queue and process the SWD transaction queue
 * @return
 */
static int ftdi_swd_run_queue(void)
{
	LOG_DEBUG_IO("Executing %zu queued transactions", swd_cmd_queue_length);
	int retval;
	struct signal *led = find_signal_by_name("LED");

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG_IO("Skipping due to previous errors: %d", queued_retval);
		goto skip;
	}

	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	mpsse_clock_data_out(mpsse_ctx, NULL, 0, 8, SWD_MODE);

	/* Terminate the "blink", if the current layout has that feature */
	if (led)
		ftdi_set_signal(led, '0');

	queued_retval = mpsse_flush(mpsse_ctx);
	if (queued_retval != ERROR_OK) {
		LOG_ERROR("MPSSE failed");
		goto skip;
	}

	for (size_t i = 0; i < swd_cmd_queue_length; i++) {
		if (0 == ((swd_cmd_queue[i].cmd ^ swd_cmd(false, false, DP_TARGETSEL)) &
				(SWD_CMD_APnDP|SWD_CMD_RnW|SWD_CMD_A32))) {
			/* Targetsel has no ack so force it */
			buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1, 3, SWD_ACK_OK);
		}

		int ack = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1, 3);

		LOG_DEBUG_IO("%s %s %s reg %X = %08"PRIx32,
				ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
				swd_cmd_queue[i].cmd & SWD_CMD_APnDP ? "AP" : "DP",
				swd_cmd_queue[i].cmd & SWD_CMD_RnW ? "read" : "write",
				(swd_cmd_queue[i].cmd & SWD_CMD_A32) >> 1,
				buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn,
						1 + 3 + (swd_cmd_queue[i].cmd & SWD_CMD_RnW ? 0 : 1), 32));

		if (ack != SWD_ACK_OK) {
			queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
			goto skip;

		} else if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
			uint32_t data = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3, 32);
			int parity = buf_get_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 32, 1);

			if (parity != parity_u32(data)) {
				LOG_ERROR("SWD Read data parity mismatch");
				queued_retval = ERROR_FAIL;
				goto skip;
			}

			if (swd_cmd_queue[i].dst != NULL)
				*swd_cmd_queue[i].dst = data;
		}
	}

skip:
	/* Defensive cleanup - seems like a bad idea to have potentially stale pointers sticking around */
	for (size_t i = 0; i < swd_cmd_queue_length; i++)
		swd_cmd_queue[i].dst = NULL;

	swd_cmd_queue_length = 0;
	retval = queued_retval;
	queued_retval = ERROR_OK;

	/* Queue a new "blink" */
	if (led && retval == ERROR_OK)
		ftdi_set_signal(led, '1');

	return retval;
}

static void ftdi_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	if (swd_cmd_queue_length >= swd_cmd_queue_alloced) {
		/* Not enough room in the queue. Run the queue and increase its size for next time.
		 * Note that it's not possible to avoid running the queue here, because mpsse contains
		 * pointers into the queue which may be invalid after the realloc. */
		queued_retval = ftdi_swd_run_queue();
		struct swd_cmd_queue_entry *q = realloc(swd_cmd_queue, swd_cmd_queue_alloced * 2 * sizeof(*swd_cmd_queue));
		if (q != NULL) {
			swd_cmd_queue = q;
			swd_cmd_queue_alloced *= 2;
			LOG_DEBUG("Increased SWD command queue to %zu elements", swd_cmd_queue_alloced);
		}
	}

	if (queued_retval != ERROR_OK)
		return;

	size_t i = swd_cmd_queue_length++;
	swd_cmd_queue[i].cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

	mpsse_clock_data_out(mpsse_ctx, &swd_cmd_queue[i].cmd, 0, 8, SWD_MODE);

	if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
		/* Queue a read transaction */
		swd_cmd_queue[i].dst = dst;

		ftdi_swd_swdio_en(false);
		mpsse_clock_data_in(mpsse_ctx, swd_cmd_queue[i].trn_ack_data_parity_trn,
				0, 1 + 3 + 32 + 1 + 1, SWD_MODE);
		ftdi_swd_swdio_en(true);
	} else {
		/* Queue a write transaction */
		ftdi_swd_swdio_en(false);

		mpsse_clock_data_in(mpsse_ctx, swd_cmd_queue[i].trn_ack_data_parity_trn,
				0, 1 + 3 + 1, SWD_MODE);

		ftdi_swd_swdio_en(true);

		buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1, 32, data);
		buf_set_u32(swd_cmd_queue[i].trn_ack_data_parity_trn, 1 + 3 + 1 + 32, 1, parity_u32(data));

		mpsse_clock_data_out(mpsse_ctx, swd_cmd_queue[i].trn_ack_data_parity_trn,
				1 + 3 + 1, 32 + 1, SWD_MODE);
	}

	/* Insert idle cycles after AP accesses to avoid WAIT */
	if (cmd & SWD_CMD_APnDP)
		mpsse_clock_data_out(mpsse_ctx, NULL, 0, ap_delay_clk, SWD_MODE);

}

static void ftdi_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	ftdi_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void ftdi_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	ftdi_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int ftdi_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		ftdi_swd_swdio_en(true);
		mpsse_clock_data_out(mpsse_ctx, swd_seq_line_reset, 0, swd_seq_line_reset_len, SWD_MODE);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		ftdi_swd_swdio_en(true);
		mpsse_clock_data_out(mpsse_ctx, swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len, SWD_MODE);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		ftdi_swd_swdio_en(true);
		mpsse_clock_data_out(mpsse_ctx, swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len, SWD_MODE);
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		ftdi_swd_swdio_en(true);
		mpsse_clock_data_out(mpsse_ctx, swd_seq_dormant_to_swd, 0, swd_seq_dormant_to_swd_len, SWD_MODE);
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		ftdi_swd_swdio_en(true);
		mpsse_clock_data_out(mpsse_ctx, swd_seq_swd_to_dormant, 0, swd_seq_swd_to_dormant_len, SWD_MODE);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct swd_driver ftdi_swd = {
	.init = ftdi_swd_init,
	.switch_seq = ftdi_swd_switch_seq,
	.read_reg = ftdi_swd_read_reg,
	.write_reg = ftdi_swd_write_reg,
	.run = ftdi_swd_run_queue,
};

static const char * const ftdi_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface ftdi_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ftdi_execute_queue,
};

struct adapter_driver ftdi_adapter_driver = {
	.name = "ftdi",
	.transports = ftdi_transports,
	.commands = ftdi_command_handlers,

	.init = ftdi_initialize,
	.quit = ftdi_quit,
	.reset = ftdi_reset,
	.speed = ftdi_speed,
	.khz = ftdi_khz,
	.speed_div = ftdi_speed_div,

	.jtag_ops = &ftdi_interface,
	.swd_ops = &ftdi_swd,
};
