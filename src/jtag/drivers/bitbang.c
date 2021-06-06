/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
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

/* 2014-12: Addition of the SWD protocol support is based on the initial work
 * by Paul Fertser and modifications by Jean-Christian de Rivaz. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "bitbang.h"
#include <jtag/interface.h>
#include <jtag/commands.h>

/**
 * Function bitbang_stableclocks
 * issues a number of clock cycles while staying in a stable state.
 * Because the TMS value required to stay in the RESET state is a 1, whereas
 * the TMS value required to stay in any of the other stable states is a 0,
 * this function checks the current stable state to decide on the value of TMS
 * to use.
 */
static int bitbang_stableclocks(int num_cycles);

static void bitbang_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk);

struct bitbang_interface *bitbang_interface;

/* DANGER!!!! clock absolutely *MUST* be 0 in idle or reset won't work!
 *
 * Set this to 1 and str912 reset halt will fail.
 *
 * If someone can submit a patch with an explanation it will be greatly
 * appreciated, but as far as I can tell (ØH) DCLK is generated upon
 * clk = 0 in TAP_IDLE. Good luck deducing that from the ARM documentation!
 * The ARM documentation uses the term "DCLK is asserted while in the TAP_IDLE
 * state". With hardware there is no such thing as *while* in a state. There
 * are only edges. So clk => 0 is in fact a very subtle state transition that
 * happens *while* in the TAP_IDLE state. "#&¤"#¤&"#&"#&
 *
 * For "reset halt" the last thing that happens before srst is asserted
 * is that the breakpoint is set up. If DCLK is not wiggled one last
 * time before the reset, then the breakpoint is not set up and
 * "reset halt" will fail to halt.
 *
 */
#define CLOCK_IDLE() 0

/* The bitbang driver leaves the TCK 0 when in idle */
static void bitbang_end_state(tap_state_t state)
{
	assert(tap_is_state_stable(state));
	tap_set_end_state(state);
}

static int bitbang_state_move(int skip)
{
	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = skip; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		if (bitbang_interface->write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (bitbang_interface->write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (bitbang_interface->write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	tap_set_state(tap_get_end_state());
	return ERROR_OK;
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine.
 */
static int bitbang_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;

	LOG_DEBUG_IO("TMS: %d bits", num_bits);

	int tms = 0;
	for (unsigned i = 0; i < num_bits; i++) {
		tms = ((bits[i/8] >> (i % 8)) & 1);
		if (bitbang_interface->write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (bitbang_interface->write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (bitbang_interface->write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int bitbang_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
			tms = 0;
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		if (bitbang_interface->write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (bitbang_interface->write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	if (bitbang_interface->write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	tap_set_end_state(tap_get_state());
	return ERROR_OK;
}

static int bitbang_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		bitbang_end_state(TAP_IDLE);
		if (bitbang_state_move(0) != ERROR_OK)
			return ERROR_FAIL;
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++) {
		if (bitbang_interface->write(0, 0, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (bitbang_interface->write(1, 0, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (bitbang_interface->write(CLOCK_IDLE(), 0, 0) != ERROR_OK)
		return ERROR_FAIL;

	/* finish in end_state */
	bitbang_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		if (bitbang_state_move(0) != ERROR_OK)
			return ERROR_FAIL;

	return ERROR_OK;
}

static int bitbang_stableclocks(int num_cycles)
{
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);
	int i;

	/* send num_cycles clocks onto the cable */
	for (i = 0; i < num_cycles; i++) {
		if (bitbang_interface->write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (bitbang_interface->write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int bitbang_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
		unsigned scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	unsigned bit_cnt;

	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			bitbang_end_state(TAP_IRSHIFT);
		else
			bitbang_end_state(TAP_DRSHIFT);

		if (bitbang_state_move(0) != ERROR_OK)
			return ERROR_FAIL;
		bitbang_end_state(saved_end_state);
	}

	size_t buffered = 0;
	for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
		int tms = (bit_cnt == scan_size-1) ? 1 : 0;
		int tdi;
		int bytec = bit_cnt/8;
		int bcval = 1 << (bit_cnt % 8);

		/* if we're just reading the scan, but don't care about the output
		 * default to outputting 'low', this also makes valgrind traces more readable,
		 * as it removes the dependency on an uninitialised value
		 */
		tdi = 0;
		if ((type != SCAN_IN) && (buffer[bytec] & bcval))
			tdi = 1;

		if (bitbang_interface->write(0, tms, tdi) != ERROR_OK)
			return ERROR_FAIL;

		if (type != SCAN_OUT) {
			if (bitbang_interface->buf_size) {
				if (bitbang_interface->sample() != ERROR_OK)
					return ERROR_FAIL;
				buffered++;
			} else {
				switch (bitbang_interface->read()) {
					case BB_LOW:
						buffer[bytec] &= ~bcval;
						break;
					case BB_HIGH:
						buffer[bytec] |= bcval;
						break;
					default:
						return ERROR_FAIL;
				}
			}
		}

		if (bitbang_interface->write(1, tms, tdi) != ERROR_OK)
			return ERROR_FAIL;

		if (type != SCAN_OUT && bitbang_interface->buf_size &&
				(buffered == bitbang_interface->buf_size ||
				 bit_cnt == scan_size - 1)) {
			for (unsigned i = bit_cnt + 1 - buffered; i <= bit_cnt; i++) {
				switch (bitbang_interface->read_sample()) {
					case BB_LOW:
						buffer[i/8] &= ~(1 << (i % 8));
						break;
					case BB_HIGH:
						buffer[i/8] |= 1 << (i % 8);
						break;
					default:
						return ERROR_FAIL;
				}
			}
			buffered = 0;
		}
	}

	if (tap_get_state() != tap_get_end_state()) {
		/* we *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		if (bitbang_state_move(1) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

int bitbang_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval;

	if (!bitbang_interface) {
		LOG_ERROR("BUG: Bitbang interface called, but not yet initialized");
		exit(-1);
	}

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	if (bitbang_interface->blink) {
		if (bitbang_interface->blink(1) != ERROR_OK)
			return ERROR_FAIL;
	}

	while (cmd) {
		switch (cmd->type) {
			case JTAG_RUNTEST:
				LOG_DEBUG_IO("runtest %i cycles, end in %s",
						cmd->cmd.runtest->num_cycles,
						tap_state_name(cmd->cmd.runtest->end_state));
				bitbang_end_state(cmd->cmd.runtest->end_state);
				if (bitbang_runtest(cmd->cmd.runtest->num_cycles) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case JTAG_STABLECLOCKS:
				/* this is only allowed while in a stable state.  A check for a stable
				 * state was done in jtag_add_clocks()
				 */
				if (bitbang_stableclocks(cmd->cmd.stableclocks->num_cycles) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case JTAG_TLR_RESET:
				LOG_DEBUG_IO("statemove end in %s",
						tap_state_name(cmd->cmd.statemove->end_state));
				bitbang_end_state(cmd->cmd.statemove->end_state);
				if (bitbang_state_move(0) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_PATHMOVE:
				LOG_DEBUG_IO("pathmove: %i states, end in %s",
						cmd->cmd.pathmove->num_states,
						tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
				if (bitbang_path_move(cmd->cmd.pathmove) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_SCAN:
				bitbang_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				LOG_DEBUG_IO("%s scan %d bits; end in %s",
						(cmd->cmd.scan->ir_scan) ? "IR" : "DR",
						scan_size,
					tap_state_name(cmd->cmd.scan->end_state));
				type = jtag_scan_type(cmd->cmd.scan);
				if (bitbang_scan(cmd->cmd.scan->ir_scan, type, buffer,
							scan_size) != ERROR_OK)
					return ERROR_FAIL;
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				free(buffer);
				break;
			case JTAG_SLEEP:
				LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			case JTAG_TMS:
				retval = bitbang_execute_tms(cmd);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}
	if (bitbang_interface->blink) {
		if (bitbang_interface->blink(0) != ERROR_OK)
			return ERROR_FAIL;
	}

	return retval;
}

static int queued_retval;

static int bitbang_swd_init(void)
{
	LOG_DEBUG("bitbang_swd_init");
	return ERROR_OK;
}

static void bitbang_swd_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
	LOG_DEBUG("bitbang_swd_exchange");

	if (bitbang_interface->blink) {
		/* FIXME: we should manage errors */
		bitbang_interface->blink(1);
	}

	for (unsigned int i = offset; i < bit_cnt + offset; i++) {
		int bytec = i/8;
		int bcval = 1 << (i % 8);
		int swdio = !rnw && (buf[bytec] & bcval);

		bitbang_interface->swd_write(0, swdio);

		if (rnw && buf) {
			if (bitbang_interface->swdio_read())
				buf[bytec] |= bcval;
			else
				buf[bytec] &= ~bcval;
		}

		bitbang_interface->swd_write(1, swdio);
	}

	if (bitbang_interface->blink) {
		/* FIXME: we should manage errors */
		bitbang_interface->blink(0);
	}
}

static int bitbang_swd_switch_seq(enum swd_special_seq seq)
{
	LOG_DEBUG("bitbang_swd_switch_seq");

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_line_reset, 0, swd_seq_line_reset_len);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void swd_clear_sticky_errors(void)
{
	bitbang_swd_write_reg(swd_cmd(false,  false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
}

static void bitbang_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	LOG_DEBUG("bitbang_swd_read_reg");
	assert(cmd & SWD_CMD_RNW);

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skip bitbang_swd_read_reg because queued_retval=%d", queued_retval);
		return;
	}

	for (;;) {
		uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];

		cmd |= SWD_CMD_START | (1 << 7);
		bitbang_swd_exchange(false, &cmd, 0, 8);

		bitbang_interface->swdio_drive(false);
		bitbang_swd_exchange(true, trn_ack_data_parity_trn, 0, 1 + 3 + 32 + 1 + 1);
		bitbang_interface->swdio_drive(true);

		int ack = buf_get_u32(trn_ack_data_parity_trn, 1, 3);
		uint32_t data = buf_get_u32(trn_ack_data_parity_trn, 1 + 3, 32);
		int parity = buf_get_u32(trn_ack_data_parity_trn, 1 + 3 + 32, 1);

		LOG_DEBUG("%s %s %s reg %X = %08"PRIx32,
			  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			  cmd & SWD_CMD_APNDP ? "AP" : "DP",
			  cmd & SWD_CMD_RNW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1,
			  data);

		switch (ack) {
		 case SWD_ACK_OK:
			if (parity != parity_u32(data)) {
				LOG_DEBUG("Wrong parity detected");
				queued_retval = ERROR_FAIL;
				return;
			}
			if (value)
				*value = data;
			if (cmd & SWD_CMD_APNDP)
				bitbang_swd_exchange(true, NULL, 0, ap_delay_clk);
			return;
		 case SWD_ACK_WAIT:
			LOG_DEBUG("SWD_ACK_WAIT");
			swd_clear_sticky_errors();
			break;
		 case SWD_ACK_FAULT:
			LOG_DEBUG("SWD_ACK_FAULT");
			queued_retval = ack;
			return;
		 default:
			LOG_DEBUG("No valid acknowledge: ack=%d", ack);
			queued_retval = ack;
			return;
		}
	}
}

static void bitbang_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	LOG_DEBUG("bitbang_swd_write_reg");
	assert(!(cmd & SWD_CMD_RNW));

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skip bitbang_swd_write_reg because queued_retval=%d", queued_retval);
		return;
	}

	for (;;) {
		uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
		buf_set_u32(trn_ack_data_parity_trn, 1 + 3 + 1, 32, value);
		buf_set_u32(trn_ack_data_parity_trn, 1 + 3 + 1 + 32, 1, parity_u32(value));

		cmd |= SWD_CMD_START | (1 << 7);
		bitbang_swd_exchange(false, &cmd, 0, 8);

		bitbang_interface->swdio_drive(false);
		bitbang_swd_exchange(true, trn_ack_data_parity_trn, 0, 1 + 3 + 1);
		bitbang_interface->swdio_drive(true);
		bitbang_swd_exchange(false, trn_ack_data_parity_trn, 1 + 3 + 1, 32 + 1);

		int ack = buf_get_u32(trn_ack_data_parity_trn, 1, 3);
		LOG_DEBUG("%s %s %s reg %X = %08"PRIx32,
			  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			  cmd & SWD_CMD_APNDP ? "AP" : "DP",
			  cmd & SWD_CMD_RNW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1,
			  buf_get_u32(trn_ack_data_parity_trn, 1 + 3 + 1, 32));

		switch (ack) {
		 case SWD_ACK_OK:
			if (cmd & SWD_CMD_APNDP)
				bitbang_swd_exchange(true, NULL, 0, ap_delay_clk);
			return;
		 case SWD_ACK_WAIT:
			LOG_DEBUG("SWD_ACK_WAIT");
			swd_clear_sticky_errors();
			break;
		 case SWD_ACK_FAULT:
			LOG_DEBUG("SWD_ACK_FAULT");
			queued_retval = ack;
			return;
		 default:
			LOG_DEBUG("No valid acknowledge: ack=%d", ack);
			queued_retval = ack;
			return;
		}
	}
}

static int bitbang_swd_run_queue(void)
{
	LOG_DEBUG("bitbang_swd_run_queue");
	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	bitbang_swd_exchange(true, NULL, 0, 8);

	int retval = queued_retval;
	queued_retval = ERROR_OK;
	LOG_DEBUG("SWD queue return value: %02x", retval);
	return retval;
}

const struct swd_driver bitbang_swd = {
	.init = bitbang_swd_init,
	.switch_seq = bitbang_swd_switch_seq,
	.read_reg = bitbang_swd_read_reg,
	.write_reg = bitbang_swd_write_reg,
	.run = bitbang_swd_run_queue,
};
