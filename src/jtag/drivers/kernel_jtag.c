/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2020  Gagar.IN LLC
 * by Paul Fertser <fercerpav@gmail.com>
 *
 * Copyright (C) 2012 by Andreas Fritiofson
 * andreas.fritiofson@gmail.com
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/drivers/jtag_usb_common.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#include <assert.h>

#include "kernel_jtag.h"
#include <sys/ioctl.h>

static int jtag_fd;

static uint8_t tap_state_to_kernel(tap_state_t state)
{
	switch (state) {
	case TAP_DREXIT2:
		return JTAG_STATE_EXIT2DR;
	case TAP_DREXIT1:
		return JTAG_STATE_EXIT1DR;
	case TAP_DRSHIFT:
		return JTAG_STATE_SHIFTDR;
	case TAP_DRPAUSE:
		return JTAG_STATE_PAUSEDR;
	case TAP_IRSELECT:
		return JTAG_STATE_SELECTIR;
	case TAP_DRUPDATE:
		return JTAG_STATE_UPDATEDR;
	case TAP_DRCAPTURE:
		return JTAG_STATE_CAPTUREDR;
	case TAP_DRSELECT:
		return JTAG_STATE_SELECTDR;
	case TAP_IREXIT2:
		return JTAG_STATE_EXIT2IR;
	case TAP_IREXIT1:
		return JTAG_STATE_EXIT1IR;
	case TAP_IRSHIFT:
		return JTAG_STATE_SHIFTIR;
	case TAP_IRPAUSE:
		return JTAG_STATE_PAUSEIR;
	case TAP_IDLE:
		return JTAG_STATE_IDLE;
	case TAP_IRUPDATE:
		return JTAG_STATE_UPDATEIR;
	case TAP_IRCAPTURE:
		return JTAG_STATE_CAPTUREIR;
	case TAP_RESET:
		return JTAG_STATE_TLRESET;
	case TAP_INVALID:
	default:
		return JTAG_STATE_UPDATEIR + 1;
	}
}

/**
 * Function move_to_state
 * moves the TAP controller from the current state to a
 * \a goal_state through a path given by tap_get_tms_path().  State transition
 * logging is performed by delegation to clock_tms().
 *
 * @param goal_state is the destination state for the move.
 */
static int move_to_state(tap_state_t goal_state)
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

	struct jtag_end_tap_state endstate;
	endstate.endstate = tap_state_to_kernel(goal_state);
	endstate.reset = JTAG_NO_RESET;
	endstate.tck = 0;
	int retval = ioctl(jtag_fd, JTAG_SIOCSTATE, &endstate);
	if (retval < 0) {
		LOG_ERROR("couldn't set move to state");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int kernel_jtag_speed(int speed)
{
	int retval;
	uint32_t freq = speed;

	retval = ioctl(jtag_fd, JTAG_SIOCFREQ, &freq);

	if (retval < 0) {
		LOG_ERROR("couldn't set JTAG TCK speed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kernel_jtag_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int kernel_jtag_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		/* TODO */
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static void kernel_jtag_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %s is not a stable end state", tap_state_name(state));
		exit(-1);
	}
}

static void kernel_jtag_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("runtest %i cycles, end in %s",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(cmd->cmd.runtest->end_state));

	if (tap_get_state() != TAP_IDLE)
		move_to_state(TAP_IDLE);

	/* TODO: Reuse kernel_jtag_execute_stableclocks */

	move_to_state(TAP_IDLE);

	struct tck_bitbang bitbang;
	bitbang.tms = 0;
	bitbang.tdi = 0;

	for (int i = 0; i < cmd->cmd.runtest->num_cycles; i++) {
		int retval = ioctl(jtag_fd, JTAG_IOCBITBANG, &bitbang);
		if (retval < 0) {
			LOG_ERROR("CBITBANG error");
			return;
		}
	}

	kernel_jtag_end_state(cmd->cmd.runtest->end_state);

	if (tap_get_state() != tap_get_end_state())
		move_to_state(tap_get_end_state());

	LOG_DEBUG_IO("runtest: %i, end in %s",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(tap_get_end_state()));
}

static void kernel_jtag_execute_statemove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("statemove end in %s",
		tap_state_name(cmd->cmd.statemove->end_state));

	kernel_jtag_end_state(cmd->cmd.statemove->end_state);

	/* shortest-path move to desired end state */
	if (tap_get_state() != tap_get_end_state() || tap_get_end_state() == TAP_RESET)
		move_to_state(tap_get_end_state());
}

static void kernel_jtag_execute_pathmove(struct jtag_command *cmd)
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
			/* TODO mpsse_clock_tms_cs_out(mpsse_ctx,
					&tms_byte,
					0,
					bit_count,
					false,
					ftdi_jtag_mode); */
			bit_count = 0;
		}
	}
	tap_set_end_state(tap_get_state());
	move_to_state(tap_get_state());
}

static void kernel_jtag_execute_scan(struct jtag_command *cmd)
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

	kernel_jtag_end_state(cmd->cmd.scan->end_state);

	struct scan_field *field = cmd->cmd.scan->fields;
	unsigned scan_size = 0;
	struct jtag_xfer xfer;
	xfer.type = cmd->cmd.scan->ir_scan ? JTAG_SIR_XFER : JTAG_SDR_XFER;
	/* TODO why doesn't this work ...
	 * xfer.endstate = cmd->cmd.scan->ir_scan ? JTAG_STATE_PAUSEIR : JTAG_STATE_PAUSEDR;
	*/
	xfer.endstate = JTAG_STATE_IDLE;
	xfer.padding = 0;

	for (int i = 0; i < cmd->cmd.scan->num_fields; i++, field++) {
		scan_size += field->num_bits;
		LOG_DEBUG_IO("%s%s field %d/%d %d bits",
			field->in_value ? "in" : "",
			field->out_value ? "out" : "",
			i,
			cmd->cmd.scan->num_fields,
			field->num_bits);

		xfer.length = field->num_bits;

		/*if (i == cmd->cmd.scan->num_fields - 1)
		  xfer.endstate = tap_state_to_kernel(tap_get_end_state());*/

		if (field->in_value && field->out_value)
			xfer.direction = JTAG_READ_WRITE_XFER;
		else if (field->in_value)
			xfer.direction = JTAG_READ_XFER;
		else
			xfer.direction = JTAG_WRITE_XFER;

		if (field->out_value)
			xfer.tdio = (uint64_t)(uintptr_t)field->out_value;
		else
			xfer.tdio = (uint64_t)(uintptr_t)field->in_value;

		int retval = ioctl(jtag_fd, JTAG_IOCXFER, &xfer);
		if (retval < 0) {
			LOG_ERROR("Error doing JTAG_IOCXFER");
			return;
		}

		if (field->in_value && field->out_value)
			bit_copy(field->in_value, 0, field->out_value, 0, field->num_bits);
	}

	tap_set_state(tap_get_end_state());

	LOG_DEBUG_IO("%s scan, %i bits, end in %s",
		(cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
		tap_state_name(tap_get_end_state()));
}

static void kernel_jtag_execute_sleep(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("sleep %" PRIi32, cmd->cmd.sleep->us);

	jtag_sleep(cmd->cmd.sleep->us);
	LOG_DEBUG_IO("sleep %" PRIi32 " usec while in %s",
		cmd->cmd.sleep->us,
		tap_state_name(tap_get_state()));
}

static void kernel_jtag_execute_stableclocks(struct jtag_command *cmd)
{
	/* this is only allowed while in a stable state.  A check for a stable
	 * state was done in jtag_add_clocks()
	 */
	struct tck_bitbang bitbang;
	bitbang.tms = 0;
	bitbang.tdi = 0;

	for (int i = 0; i < cmd->cmd.runtest->num_cycles; i++) {
		int retval = ioctl(jtag_fd, JTAG_IOCBITBANG, &bitbang);
		if (retval < 0) {
			LOG_ERROR("CBITBANG error");
			return;
		}
	}

	LOG_DEBUG_IO("clocks %i while in %s",
		cmd->cmd.stableclocks->num_cycles,
		tap_state_name(tap_get_state()));
}

static void kernel_jtag_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_RESET:
			/* TODO kernel_jtag_execute_reset(cmd); */
			break;
		case JTAG_RUNTEST:
			kernel_jtag_execute_runtest(cmd);
			break;
		case JTAG_TLR_RESET:
			kernel_jtag_execute_statemove(cmd);
			break;
		case JTAG_PATHMOVE:
			kernel_jtag_execute_pathmove(cmd);
			break;
		case JTAG_SCAN:
			kernel_jtag_execute_scan(cmd);
			break;
		case JTAG_SLEEP:
			kernel_jtag_execute_sleep(cmd);
			break;
		case JTAG_STABLECLOCKS:
			kernel_jtag_execute_stableclocks(cmd);
			break;
		case JTAG_TMS:
			/* TODO kernel_jtag_execute_tms(cmd); */
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered: %d", cmd->type);
			break;
	}
}

static int kernel_jtag_initialize(void)
{
	jtag_fd = open("/dev/jtag0", O_RDWR);
	if (jtag_fd == -1) {
		LOG_ERROR("Can't open JTAG device node");
		return ERROR_FAIL;
	}

	kernel_jtag_speed(jtag_get_speed_khz() * 1000);

	/* TODO HW mode doesn't work */
	struct jtag_mode mode = { .feature = JTAG_XFER_MODE,
				  .mode = JTAG_XFER_SW_MODE };
	ioctl(jtag_fd, JTAG_SIOCMODE, &mode);

	return ERROR_OK;
}

static int kernel_jtag_quit(void)
{
	close(jtag_fd);
	return ERROR_OK;
}

static int kernel_jtag_execute_queue(void)
{
	for (struct jtag_command *cmd = jtag_command_queue; cmd; cmd = cmd->next)
		kernel_jtag_execute_command(cmd);

	return ERROR_OK;
}

static const char * const kernel_jtag_transports[] = { "jtag", NULL };

static struct jtag_interface kernel_jtag_interface = {
	/* TODO .supported = DEBUG_CAP_TMS_SEQ, */
	.execute_queue = kernel_jtag_execute_queue,
};

struct adapter_driver kernel_jtag_adapter_driver = {
	.name = "kernel_jtag",
	.transports = kernel_jtag_transports,

	.init = kernel_jtag_initialize,
	.quit = kernel_jtag_quit,
	.speed = kernel_jtag_speed,
	.speed_div = kernel_jtag_speed_div,
	.khz = kernel_jtag_khz,
	.jtag_ops = &kernel_jtag_interface,
};
