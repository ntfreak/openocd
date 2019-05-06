/***************************************************************************
 *   Copyright (C) 2007-2010 by Øyvind Harboe                              *
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

/* This file supports the zy1000 debugger:
 *
 * http://www.ultsol.com/index.php/component/content/article/8/33-zylin-zy1000-jtag-probe
 *
 * The zy1000 is a standalone debugger that has a web interface and
 * requires no drivers on the developer host as all communication
 * is via TCP/IP. The zy1000 gets it performance(~400-700kBytes/s
 * DCC downloads @ 16MHz target) as it has an FPGA to hardware
 * accelerate the JTAG commands, while offering *very* low latency
 * between OpenOCD and the FPGA registers.
 *
 * The disadvantage of the zy1000 is that it has a feeble CPU compared to
 * a PC(ca. 50-500 DMIPS depending on how one counts it), whereas a PC
 * is on the order of 10000 DMIPS(i.e. at a factor of 20-200).
 *
 * The zy1000 revc hardware is using an Altera Nios CPU, whereas the
 * revb is using ARM7 + Xilinx.
 *
 * See Zylin web pages or contact Zylin for more information.
 *
 * The reason this code is in OpenOCD rather than OpenOCD linked with the
 * ZY1000 code is that OpenOCD is the long road towards getting
 * libopenocd into place. libopenocd will support both low performance,
 * low latency systems(embedded) and high performance high latency
 * systems(PCs).
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pthread.h>

#include <target/embeddedice.h>
#include <jtag/minidriver.h>
#include <jtag/interface.h>
#include <time.h>
#include <helper/time_support.h>

#include <netinet/tcp.h>

/* Assume we're connecting to a revc w/60MHz clock. */
#define ZYLIN_KHZ 60000

/* The software needs to check if it's in RCLK mode or not */
static bool zy1000_rclk;

static int zy1000_khz(int khz, int *jtag_speed)
{
	if (khz == 0)
		*jtag_speed = 0;
	else {
		int speed;
		/* Round speed up to nearest divisor.
		 *
		 * E.g. 16000kHz
		 * (64000 + 15999) / 16000 = 4
		 * (4 + 1) / 2 = 2
		 * 2 * 2 = 4
		 *
		 * 64000 / 4 = 16000
		 *
		 * E.g. 15999
		 * (64000 + 15998) / 15999 = 5
		 * (5 + 1) / 2 = 3
		 * 3 * 2 = 6
		 *
		 * 64000 / 6 = 10666
		 *
		 */
		speed = (ZYLIN_KHZ + (khz - 1)) / khz;
		speed = (speed + 1) / 2;
		speed *= 2;
		if (speed > 8190) {
			/* maximum dividend */
			speed = 8190;
		}
		*jtag_speed = speed;
	}
	return ERROR_OK;
}

static int zy1000_speed_div(int speed, int *khz)
{
	if (speed == 0)
		*khz = 0;
	else
		*khz = ZYLIN_KHZ / speed;

	return ERROR_OK;
}

static bool readPowerDropout(void)
{
	uint32_t state;
	/* sample and clear power dropout */
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x80);
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, state);
	bool powerDropout;
	powerDropout = (state & 0x80) != 0;
	return powerDropout;
}


static bool readSRST(void)
{
	uint32_t state;
	/* sample and clear SRST sensing */
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x00000040);
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, state);
	bool srstAsserted;
	srstAsserted = (state & 0x40) != 0;
	return srstAsserted;
}

static int zy1000_srst_asserted(int *srst_asserted)
{
	*srst_asserted = readSRST();
	return ERROR_OK;
}

static int zy1000_power_dropout(int *dropout)
{
	*dropout = readPowerDropout();
	return ERROR_OK;
}

/* Wait for SRST to assert or deassert */
static void waitSRST(bool asserted)
{
	bool first = true;
	int64_t start = 0;
	int64_t total = 0;
	const char *mode = asserted ? "assert" : "deassert";

	for (;; ) {
		bool srstAsserted = readSRST();
		if ((asserted && srstAsserted) || (!asserted && !srstAsserted)) {
			if (total > 1)
				LOG_USER("SRST took %dms to %s", (int)total, mode);
			break;
		}

		if (first) {
			first = false;
			start = timeval_ms();
		}

		total = timeval_ms() - start;

		keep_alive();

		if (total > 5000) {
			LOG_ERROR("SRST took too long to %s: %" PRId64 "ms", mode, total);
			break;
		}
	}
}

void zy1000_reset(int trst, int srst)
{
	LOG_DEBUG("zy1000 trst=%d, srst=%d", trst, srst);

	/* flush the JTAG FIFO. Not flushing the queue before messing with
	 * reset has such interesting bugs as causing hard to reproduce
	 * RCLK bugs as RCLK will stop responding when TRST is asserted
	 */
	waitIdle();

	if (!srst)
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x00000001);
	else {
		/* Danger!!! if clk != 0 when in
		 * idle in TAP_IDLE, reset halt on str912 will fail.
		 */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x00000001);

		waitSRST(true);
	}

	if (!trst)
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x00000002);
	else {
		/* assert reset */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x00000002);
	}

	if (trst || (srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST))) {
		/* we're now in the RESET state until trst is deasserted */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, TAP_RESET);
	} else {
		/* We'll get RCLK failure when we assert TRST, so clear any false positives here */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x400);
	}

	/* wait for srst to float back up */
	if ((!srst && ((jtag_get_reset_config() & RESET_TRST_PULLS_SRST) == 0)) ||
			(!srst && !trst && (jtag_get_reset_config() & RESET_TRST_PULLS_SRST)))
		waitSRST(false);
}

int zy1000_speed(int speed)
{
	/* flush JTAG master FIFO before setting speed */
	waitIdle();

	zy1000_rclk = false;

	if (speed == 0) {
		/*0 means RCLK*/
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x100);
		zy1000_rclk = true;
		LOG_DEBUG("jtag_speed using RCLK");
	} else {
		if (speed > 8190 || speed < 2) {
			LOG_USER(
				"valid ZY1000 jtag_speed=[8190,2]. With divisor is %dkHz / even values between 8190-2, i.e. min %dHz, max %dMHz",
				ZYLIN_KHZ,
				(ZYLIN_KHZ * 1000) / 8190,
				ZYLIN_KHZ / (2 * 1000));
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		int khz;
		speed &= ~1;
		zy1000_speed_div(speed, &khz);
		LOG_USER("jtag_speed %d => JTAG clk=%d kHz", speed, khz);
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x100);
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x1c, speed);
	}
	return ERROR_OK;
}

static bool savePower;

static void setPower(bool power)
{
	savePower = power;
	if (power)
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x8);
	else
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x8);
}

COMMAND_HANDLER(handle_power_command)
{
	switch (CMD_ARGC) {
		case 1: {
			bool enable;
			COMMAND_PARSE_ON_OFF(CMD_ARGV[0], enable);
			setPower(enable);
		}
			/* fall through */
		case 0:
			LOG_INFO("Target power %s", savePower ? "on" : "off");
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

#if !BUILD_ZY1000_MASTER
static char *tcp_server = "notspecified";
static int jim_zy1000_server(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	if (argc != 2)
		return JIM_ERR;

	tcp_server = strdup(Jim_GetString(argv[1], NULL));

	return JIM_OK;
}
#endif

static int zylinjtag_Jim_Command_powerstatus(Jim_Interp *interp,
	int argc,
	Jim_Obj * const *argv)
{
	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "powerstatus");
		return JIM_ERR;
	}

	bool dropout = readPowerDropout();

	Jim_SetResult(interp, Jim_NewIntObj(interp, dropout));

	return JIM_OK;
}

int zy1000_quit(void)
{

	return ERROR_OK;
}

int interface_jtag_execute_queue(void)
{
	uint32_t empty;

	waitIdle();

	/* We must make sure to write data read back to memory location before we return
	 * from this fn
	 */
	zy1000_flush_readqueue();

	/* and handle any callbacks... */
	zy1000_flush_callbackqueue();

	if (zy1000_rclk) {
		/* Only check for errors when using RCLK to speed up
		 * jtag over TCP/IP
		 */
		ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, empty);
		/* clear JTAG error register */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x400);

		if ((empty&0x400) != 0) {
			LOG_WARNING("RCLK timeout");
			/* the error is informative only as we don't want to break the firmware if there
			 * is a false positive.
			 */
			/*		return ERROR_FAIL; */
		}
	}
	return ERROR_OK;
}

static void writeShiftValue(uint8_t *data, int bits);

/* here we shuffle N bits out/in */
static inline void scanBits(const uint8_t *out_value,
	uint8_t *in_value,
	int num_bits,
	bool pause_now,
	tap_state_t shiftState,
	tap_state_t end_state)
{
	tap_state_t pause_state = shiftState;
	for (int j = 0; j < num_bits; j += 32) {
		int k = num_bits - j;
		if (k > 32) {
			k = 32;
			/* we have more to shift out */
		} else if (pause_now) {
			/* this was the last to shift out this time */
			pause_state = end_state;
		}

		/* we have (num_bits + 7)/8 bytes of bits to toggle out. */
		/* bits are pushed out LSB to MSB */
		uint32_t value;
		value = 0;
		if (out_value != NULL) {
			for (int l = 0; l < k; l += 8)
				value |= out_value[(j + l)/8]<<l;
		}
		/* mask away unused bits for easier debugging */
		if (k < 32)
			value &= ~(((uint32_t)0xffffffff) << k);
		else {
			/* Shifting by >= 32 is not defined by the C standard
			 * and will in fact shift by &0x1f bits on nios */
		}

		shiftValueInner(shiftState, pause_state, k, value);

		if (in_value != NULL)
			writeShiftValue(in_value + (j/8), k);
	}
}

static inline void scanFields(int num_fields,
	const struct scan_field *fields,
	tap_state_t shiftState,
	tap_state_t end_state)
{
	for (int i = 0; i < num_fields; i++) {
		scanBits(fields[i].out_value,
			fields[i].in_value,
			fields[i].num_bits,
			(i == num_fields-1),
			shiftState,
			end_state);
	}
}

int interface_jtag_add_ir_scan(struct jtag_tap *active,
	const struct scan_field *fields,
	tap_state_t state)
{
	int scan_size = 0;
	struct jtag_tap *tap, *nextTap;
	tap_state_t pause_state = TAP_IRSHIFT;

	for (tap = jtag_tap_next_enabled(NULL); tap != NULL; tap = nextTap) {
		nextTap = jtag_tap_next_enabled(tap);
		if (nextTap == NULL)
			pause_state = state;
		scan_size = tap->ir_length;

		/* search the list */
		if (tap == active) {
			scanFields(1, fields, TAP_IRSHIFT, pause_state);
			/* update device information */
			buf_cpy(fields[0].out_value, tap->cur_instr, scan_size);

			tap->bypass = 0;
		} else {
			/* if a device isn't listed, set it to BYPASS */
			assert(scan_size <= 32);
			shiftValueInner(TAP_IRSHIFT, pause_state, scan_size, 0xffffffff);

			/* Optimization code will check what the cur_instr is set to, so
			 * we must set it to bypass value.
			 */
			buf_set_ones(tap->cur_instr, tap->ir_length);

			tap->bypass = 1;
		}
	}

	return ERROR_OK;
}

int interface_jtag_add_plain_ir_scan(int num_bits,
	const uint8_t *out_bits,
	uint8_t *in_bits,
	tap_state_t state)
{
	scanBits(out_bits, in_bits, num_bits, true, TAP_IRSHIFT, state);
	return ERROR_OK;
}

int interface_jtag_add_dr_scan(struct jtag_tap *active,
	int num_fields,
	const struct scan_field *fields,
	tap_state_t state)
{
	struct jtag_tap *tap, *nextTap;
	tap_state_t pause_state = TAP_DRSHIFT;
	for (tap = jtag_tap_next_enabled(NULL); tap != NULL; tap = nextTap) {
		nextTap = jtag_tap_next_enabled(tap);
		if (nextTap == NULL)
			pause_state = state;

		/* Find a range of fields to write to this tap */
		if (tap == active) {
			assert(!tap->bypass);

			scanFields(num_fields, fields, TAP_DRSHIFT, pause_state);
		} else {
			/* Shift out a 0 for disabled tap's */
			assert(tap->bypass);
			shiftValueInner(TAP_DRSHIFT, pause_state, 1, 0);
		}
	}
	return ERROR_OK;
}

int interface_jtag_add_plain_dr_scan(int num_bits,
	const uint8_t *out_bits,
	uint8_t *in_bits,
	tap_state_t state)
{
	scanBits(out_bits, in_bits, num_bits, true, TAP_DRSHIFT, state);
	return ERROR_OK;
}

int interface_jtag_add_tlr(void)
{
	setCurrentState(TAP_RESET);
	return ERROR_OK;
}

int interface_jtag_add_reset(int req_trst, int req_srst)
{
	zy1000_reset(req_trst, req_srst);
	return ERROR_OK;
}

static int zy1000_jtag_add_clocks(int num_cycles, tap_state_t state, tap_state_t clockstate)
{
	/* num_cycles can be 0 */
	setCurrentState(clockstate);

	/* execute num_cycles, 32 at the time. */
	int i;
	for (i = 0; i < num_cycles; i += 32) {
		int num;
		num = 32;
		if (num_cycles-i < num)
			num = num_cycles-i;
		shiftValueInner(clockstate, clockstate, num, 0);
	}

#if !TEST_MANUAL()
	/* finish in end_state */
	setCurrentState(state);
#else
	tap_state_t t = TAP_IDLE;
	/* test manual drive code on any target */
	int tms;
	uint8_t tms_scan = tap_get_tms_path(t, state);
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		waitIdle();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x28,  tms);
	}
	waitIdle();
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, state);
#endif

	return ERROR_OK;
}

int interface_jtag_add_runtest(int num_cycles, tap_state_t state)
{
	return zy1000_jtag_add_clocks(num_cycles, state, TAP_IDLE);
}

int interface_jtag_add_clocks(int num_cycles)
{
	return zy1000_jtag_add_clocks(num_cycles, cmd_queue_cur_state, cmd_queue_cur_state);
}

int interface_add_tms_seq(unsigned num_bits, const uint8_t *seq, enum tap_state state)
{
	/*wait for the fifo to be empty*/
	waitIdle();

	for (unsigned i = 0; i < num_bits; i++) {
		int tms;

		if (((seq[i/8] >> (i % 8)) & 1) == 0)
			tms = 0;
		else
			tms = 1;

		waitIdle();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, tms);
	}

	waitIdle();
	if (state != TAP_INVALID)
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, state);
	else {
		/* this would be normal if
		 * we are switching to SWD mode */
	}
	return ERROR_OK;
}

int interface_jtag_add_pathmove(int num_states, const tap_state_t *path)
{
	int state_count;
	int tms = 0;

	state_count = 0;

	tap_state_t cur_state = cmd_queue_cur_state;

	uint8_t seq[16];
	memset(seq, 0, sizeof(seq));
	assert(num_states < (int)((sizeof(seq) * 8)));

	while (num_states) {
		if (tap_state_transition(cur_state, false) == path[state_count])
			tms = 0;
		else if (tap_state_transition(cur_state, true) == path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(cur_state), tap_state_name(path[state_count]));
			exit(-1);
		}

		seq[state_count/8] = seq[state_count/8] | (tms << (state_count % 8));

		cur_state = path[state_count];
		state_count++;
		num_states--;
	}

	return interface_add_tms_seq(state_count, seq, cur_state);
}

static void jtag_pre_post_bits(struct jtag_tap *tap, int *pre, int *post)
{
	/* bypass bits before and after */
	int pre_bits = 0;
	int post_bits = 0;

	bool found = false;
	struct jtag_tap *cur_tap, *nextTap;
	for (cur_tap = jtag_tap_next_enabled(NULL); cur_tap != NULL; cur_tap = nextTap) {
		nextTap = jtag_tap_next_enabled(cur_tap);
		if (cur_tap == tap)
			found = true;
		else {
			if (found)
				post_bits++;
			else
				pre_bits++;
		}
	}
	*pre = pre_bits;
	*post = post_bits;
}

void embeddedice_write_dcc(struct jtag_tap *tap,
	int reg_addr,
	const uint8_t *buffer,
	int little,
	int count)
{
#if 0
	int i;
	for (i = 0; i < count; i++) {
		embeddedice_write_reg_inner(tap, reg_addr, fast_target_buffer_get_u32(buffer,
				little));
		buffer += 4;
	}
#else
	int pre_bits;
	int post_bits;
	jtag_pre_post_bits(tap, &pre_bits, &post_bits);

	if ((pre_bits > 32) || (post_bits + 6 > 32)) {
		int i;
		for (i = 0; i < count; i++) {
			embeddedice_write_reg_inner(tap, reg_addr,
				fast_target_buffer_get_u32(buffer, little));
			buffer += 4;
		}
	} else {
		int i;
		for (i = 0; i < count; i++) {
			/* Fewer pokes means we get to use the FIFO more efficiently */
			shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, pre_bits, 0);
			shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, 32,
				fast_target_buffer_get_u32(buffer, little));
			/* Danger! here we need to exit into the TAP_IDLE state to make
			 * DCC pick up this value.
			 */
			shiftValueInner(TAP_DRSHIFT, TAP_IDLE, 6 + post_bits,
				(reg_addr | (1 << 5)));
			buffer += 4;
		}
	}
#endif
}

int arm11_run_instr_data_to_core_noack_inner(struct jtag_tap *tap,
	uint32_t opcode,
	uint32_t *data,
	size_t count)
{
	/* bypass bits before and after */
	int pre_bits;
	int post_bits;
	jtag_pre_post_bits(tap, &pre_bits, &post_bits);
	post_bits += 2;

	if ((pre_bits > 32) || (post_bits > 32)) {
		int arm11_run_instr_data_to_core_noack_inner_default(struct jtag_tap *tap,
				uint32_t opcode, uint32_t *data, size_t count);
		return arm11_run_instr_data_to_core_noack_inner_default(tap, opcode, data, count);
	} else {
		static const uint8_t zero;

		/* FIX!!!!!! the target_write_memory() API started this nasty problem
		 * with unaligned uint32_t * pointers... */
		const uint8_t *t = (const uint8_t *)data;

		while (--count > 0) {
#if 1
			/* Danger! This code doesn't update cmd_queue_cur_state, so
			 * invoking jtag_add_pathmove() before jtag_add_dr_scan() after
			 * this loop would fail!
			 */
			shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, pre_bits, 0);

			uint32_t value;
			value = *t++;
			value |= (*t++<<8);
			value |= (*t++<<16);
			value |= (*t++<<24);

			shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, 32, value);
			/* minimum 2 bits */
			shiftValueInner(TAP_DRSHIFT, TAP_DRPAUSE, post_bits, 0);

			/* copy & paste from arm11_dbgtap.c */
			/* TAP_DREXIT2, TAP_DRUPDATE, TAP_IDLE, TAP_IDLE, TAP_IDLE, TAP_DRSELECT,
			 * TAP_DRCAPTURE, TAP_DRSHIFT */
			/* KLUDGE! we have to flush the fifo or the Nios CPU locks up.
			 * This is probably a bug in the Avalon bus(cross clocking bridge?)
			 * or in the jtag registers module.
			 */
			waitIdle();
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 1);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 1);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 0);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 0);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 0);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 1);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 0);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 0);
			/* we don't have to wait for the queue to empty here */
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, TAP_DRSHIFT);
			waitIdle();
#else
			static const tap_state_t arm11_MOVE_DRPAUSE_IDLE_DRPAUSE_with_delay[] = {
				TAP_DREXIT2, TAP_DRUPDATE, TAP_IDLE, TAP_IDLE, TAP_IDLE,
				TAP_DRSELECT, TAP_DRCAPTURE, TAP_DRSHIFT
			};

			struct scan_field fields[2] = {
					{ .num_bits = 32, .out_value = t },
					{ .num_bits = 2, .out_value = &zero },
			};
			t += 4;

			jtag_add_dr_scan(tap,
				2,
				fields,
				TAP_IDLE);

			jtag_add_pathmove(ARRAY_SIZE(arm11_MOVE_DRPAUSE_IDLE_DRPAUSE_with_delay),
				arm11_MOVE_DRPAUSE_IDLE_DRPAUSE_with_delay);
#endif
		}

		struct scan_field fields[2] = {
				{ .num_bits = 32, .out_value = t },
				{ .num_bits = 2, .out_value = &zero },
		};

		/* This will happen on the last iteration updating cmd_queue_cur_state
		 * so we don't have to track it during the common code path
		 */
		jtag_add_dr_scan(tap,
			2,
			fields,
			TAP_IDLE);

		return jtag_execute_queue();
	}
}

static const struct command_registration zy1000_commands[] = {
	{
		.name = "power",
		.handler = handle_power_command,
		.mode = COMMAND_ANY,
		.help = "Turn power switch to target on/off. "
			"With no arguments, prints status.",
		.usage = "('on'|'off)",
	},
#if !BUILD_ZY1000_MASTER
	{
		.name = "zy1000_server",
		.mode = COMMAND_ANY,
		.jim_handler = jim_zy1000_server,
		.help = "Tcpip address for ZY1000 server.",
		.usage = "address",
	},
#endif
	{
		.name = "powerstatus",
		.mode = COMMAND_ANY,
		.jim_handler = zylinjtag_Jim_Command_powerstatus,
		.help = "Returns power status of target",
	},
	COMMAND_REGISTRATION_DONE
};

#if !BUILD_ZY1000_MASTER

static int tcp_ip = -1;

/* Write large packets if we can */
static size_t out_pos;
static uint8_t out_buffer[16384];
static size_t in_pos;
static size_t in_write;
static uint8_t in_buffer[16384];

static bool flush_writes(void)
{
	bool ok = (write(tcp_ip, out_buffer, out_pos) == (int)out_pos);
	out_pos = 0;
	return ok;
}

static bool writeLong(uint32_t l)
{
	int i;
	for (i = 0; i < 4; i++) {
		uint8_t c = (l >> (i*8))&0xff;
		out_buffer[out_pos++] = c;
		if (out_pos >= sizeof(out_buffer)) {
			if (!flush_writes())
				return false;
		}
	}
	return true;
}

static bool readLong(uint32_t *out_data)
{
	uint32_t data = 0;
	int i;
	for (i = 0; i < 4; i++) {
		uint8_t c;
		if (in_pos == in_write) {
			/* If we have some data that we can send, send them before
			 * we wait for more data
			 */
			if (out_pos > 0) {
				if (!flush_writes())
					return false;
			}

			/* read more */
			int t;
			t = read(tcp_ip, in_buffer, sizeof(in_buffer));
			if (t < 1)
				return false;
			in_write = (size_t) t;
			in_pos = 0;
		}
		c = in_buffer[in_pos++];

		data |= (c << (i*8));
	}
	*out_data = data;
	return true;
}

enum ZY1000_CMD {
	ZY1000_CMD_POKE = 0x0,
	ZY1000_CMD_PEEK = 0x8,
	ZY1000_CMD_SLEEP = 0x1,
	ZY1000_CMD_WAITIDLE = 2
};

#include <sys/socket.h>	/* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>	/* for sockaddr_in and inet_addr() */

/* We initialize this late since we need to know the server address
 * first.
 */
static void tcpip_open(void)
{
	if (tcp_ip >= 0)
		return;

	struct sockaddr_in echoServAddr;/* Echo server address */

	/* Create a reliable, stream socket using TCP */
	tcp_ip = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (tcp_ip < 0) {
		fprintf(stderr, "Failed to connect to zy1000 server\n");
		exit(-1);
	}

	/* Construct the server address structure */
	memset(&echoServAddr, 0, sizeof(echoServAddr));	/* Zero out structure */
	echoServAddr.sin_family = AF_INET;	/* Internet address family */
	echoServAddr.sin_addr.s_addr = inet_addr(tcp_server);	/* Server IP address */
	echoServAddr.sin_port = htons(7777);	/* Server port */

	/* Establish the connection to the echo server */
	if (connect(tcp_ip, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0) {
		fprintf(stderr, "Failed to connect to zy1000 server\n");
		exit(-1);
	}

	int flag = 1;
	setsockopt(tcp_ip,	/* socket affected */
		IPPROTO_TCP,			/* set option at TCP level */
		TCP_NODELAY,			/* name of option */
		(char *)&flag,			/* the cast is historical cruft */
		sizeof(int));			/* length of option value */

}

/* send a poke */
void zy1000_tcpout(uint32_t address, uint32_t data)
{
	tcpip_open();
	if (!writeLong((ZY1000_CMD_POKE << 24) | address) || !writeLong(data)) {
		fprintf(stderr, "Could not write to zy1000 server\n");
		exit(-1);
	}
}

/* By sending the wait to the server, we avoid a readback
 * of status. Radically improves performance for this operation
 * with long ping times.
 */
void waitIdle(void)
{
	tcpip_open();
	if (!writeLong((ZY1000_CMD_WAITIDLE << 24))) {
		fprintf(stderr, "Could not write to zy1000 server\n");
		exit(-1);
	}
}

uint32_t zy1000_tcpin(uint32_t address)
{
	tcpip_open();

	zy1000_flush_readqueue();

	uint32_t data;
	if (!writeLong((ZY1000_CMD_PEEK << 24) | address) || !readLong(&data)) {
		fprintf(stderr, "Could not read from zy1000 server\n");
		exit(-1);
	}
	return data;
}

int interface_jtag_add_sleep(uint32_t us)
{
	tcpip_open();
	if (!writeLong((ZY1000_CMD_SLEEP << 24)) || !writeLong(us)) {
		fprintf(stderr, "Could not read from zy1000 server\n");
		exit(-1);
	}
	return ERROR_OK;
}

/* queue a readback */
#define readqueue_size 16384
static struct {
	uint8_t *dest;
	int bits;
} readqueue[readqueue_size];

static int readqueue_pos;

/* flush the readqueue, this means reading any data that
 * we're expecting and store them into the final position
 */
void zy1000_flush_readqueue(void)
{
	if (readqueue_pos == 0) {
		/* simply debugging by allowing easy breakpoints when there
		 * is something to do. */
		return;
	}
	int i;
	tcpip_open();
	for (i = 0; i < readqueue_pos; i++) {
		uint32_t value;
		if (!readLong(&value)) {
			fprintf(stderr, "Could not read from zy1000 server\n");
			exit(-1);
		}

		uint8_t *in_value = readqueue[i].dest;
		int k = readqueue[i].bits;

		/* we're shifting in data to MSB, shift data to be aligned for returning the value */
		value >>= 32-k;

		for (int l = 0; l < k; l += 8)
			in_value[l/8] = (value >> l)&0xff;
	}
	readqueue_pos = 0;
}

/* By queuing the callback's we avoid flushing the
 * read queue until jtag_execute_queue(). This can
 * reduce latency dramatically for cases where
 * callbacks are used extensively.
*/
#define callbackqueue_size 128
static struct callbackentry {
	jtag_callback_t callback;
	jtag_callback_data_t data0;
	jtag_callback_data_t data1;
	jtag_callback_data_t data2;
	jtag_callback_data_t data3;
} callbackqueue[callbackqueue_size];

static int callbackqueue_pos;

void zy1000_jtag_add_callback4(jtag_callback_t callback,
	jtag_callback_data_t data0,
	jtag_callback_data_t data1,
	jtag_callback_data_t data2,
	jtag_callback_data_t data3)
{
	if (callbackqueue_pos >= callbackqueue_size)
		zy1000_flush_callbackqueue();

	callbackqueue[callbackqueue_pos].callback = callback;
	callbackqueue[callbackqueue_pos].data0 = data0;
	callbackqueue[callbackqueue_pos].data1 = data1;
	callbackqueue[callbackqueue_pos].data2 = data2;
	callbackqueue[callbackqueue_pos].data3 = data3;
	callbackqueue_pos++;

	/* KLUDGE!
	 * make callbacks synchronous for now as minidriver requires callback
	 * to be synchronous.
	 *
	 * We can get away with making read and writes asynchronous so we
	 * don't completely kill performance.
	 */
	zy1000_flush_callbackqueue();
}

static int zy1000_jtag_convert_to_callback4(jtag_callback_data_t data0,
	jtag_callback_data_t data1,
	jtag_callback_data_t data2,
	jtag_callback_data_t data3)
{
	((jtag_callback1_t)data1)(data0);
	return ERROR_OK;
}

void zy1000_jtag_add_callback(jtag_callback1_t callback, jtag_callback_data_t data0)
{
	zy1000_jtag_add_callback4(zy1000_jtag_convert_to_callback4,
		data0,
		(jtag_callback_data_t)callback,
		0,
		0);
}

void zy1000_flush_callbackqueue(void)
{
	/* we have to flush the read queue so we have access to
	 the data the callbacks will use
	*/
	zy1000_flush_readqueue();
	int i;
	for (i = 0; i < callbackqueue_pos; i++) {
		struct callbackentry *entry = &callbackqueue[i];
		jtag_set_error(entry->callback(entry->data0, entry->data1, entry->data2,
				entry->data3));
	}
	callbackqueue_pos = 0;
}

static void writeShiftValue(uint8_t *data, int bits)
{
	waitIdle();

	if (!writeLong((ZY1000_CMD_PEEK << 24) | (ZY1000_JTAG_BASE + 0xc))) {
		fprintf(stderr, "Could not read from zy1000 server\n");
		exit(-1);
	}

	if (readqueue_pos >= readqueue_size)
		zy1000_flush_readqueue();

	readqueue[readqueue_pos].dest = data;
	readqueue[readqueue_pos].bits = bits;
	readqueue_pos++;

	/* KLUDGE!!! minidriver requires readqueue to be synchronous */
	zy1000_flush_readqueue();
}

#else

static void writeShiftValue(uint8_t *data, int bits)
{
	uint32_t value;
	waitIdle();
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0xc, value);
	VERBOSE(LOG_INFO("getShiftValue %08x", value));

	/* data in, LSB to MSB */
	/* we're shifting in data to MSB, shift data to be aligned for returning the value */
	value >>= 32 - bits;

	for (int l = 0; l < bits; l += 8)
		data[l/8] = (value >> l)&0xff;
}

#endif

#if BUILD_ZY1000_MASTER

#ifdef WATCHDOG_BASE
/* If we connect to port 8888 we must send a char every 10s or the board resets itself */
static void watchdog_server(cyg_addrword_t data)
{
	int so_reuseaddr_option = 1;

	int fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == -1) {
		LOG_ERROR("error creating socket: %s", strerror(errno));
		exit(-1);
	}

	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (void *) &so_reuseaddr_option,
		sizeof(int));

	struct sockaddr_in sin;
	unsigned int address_size;
	address_size = sizeof(sin);
	memset(&sin, 0, sizeof(sin));
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = INADDR_ANY;
	sin.sin_port = htons(8888);

	if (bind(fd, (struct sockaddr *) &sin, sizeof(sin)) == -1) {
		LOG_ERROR("couldn't bind to socket: %s", strerror(errno));
		exit(-1);
	}

	if (listen(fd, 1) == -1) {
		LOG_ERROR("couldn't listen on socket: %s", strerror(errno));
		exit(-1);
	}


	for (;; ) {
		int watchdog_ip = accept(fd, (struct sockaddr *) &sin, &address_size);

		/* Start watchdog, must be reset every 10 seconds. */
		HAL_WRITE_UINT32(WATCHDOG_BASE + 4, 4);

		if (watchdog_ip < 0) {
			LOG_ERROR("couldn't open watchdog socket: %s", strerror(errno));
			exit(-1);
		}

		int flag = 1;
		setsockopt(watchdog_ip,	/* socket affected */
			IPPROTO_TCP,			/* set option at TCP level */
			TCP_NODELAY,			/* name of option */
			(char *)&flag,			/* the cast is historical cruft */
			sizeof(int));			/* length of option value */


		char buf;
		for (;; ) {
			if (read(watchdog_ip, &buf, 1) == 1) {
				/* Reset timer */
				HAL_WRITE_UINT32(WATCHDOG_BASE + 8, 0x1234);
				/* Echo so we can telnet in and see that resetting works */
				write(watchdog_ip, &buf, 1);
			} else {
				/* Stop tickling the watchdog, the CPU will reset in < 10 seconds
				 * now.
				 */
				return;
			}

		}

		/* Never reached */
	}
}
#endif

#endif

#if BUILD_ZY1000_MASTER
int interface_jtag_add_sleep(uint32_t us)
{
	jtag_sleep(us);
	return ERROR_OK;
}
#endif

#if BUILD_ZY1000_MASTER
volatile void *zy1000_jtag_master;
#include <sys/mman.h>
#endif

int zy1000_init(void)
{
#if BUILD_ZY1000_MASTER
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		LOG_ERROR("No access to /dev/mem");
		return ERROR_FAIL;
	}
#ifndef REGISTERS_BASE
#define REGISTERS_BASE 0x9002000
#define REGISTERS_SPAN 128
#endif

	zy1000_jtag_master = mmap(0,
			REGISTERS_SPAN,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			fd,
			REGISTERS_BASE);

	if (zy1000_jtag_master == (void *) -1) {
		close(fd);
		LOG_ERROR("No access to /dev/mem");
		return ERROR_FAIL;
	}
#endif

	ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x30);	/* Turn on LED1 & LED2 */

	setPower(true);	/* on by default */

	/* deassert resets. Important to avoid infinite loop waiting for SRST to deassert */
	zy1000_reset(0, 0);

	return ERROR_OK;
}

static struct jtag_interface zy1000_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = NULL,
};

struct adapter_driver zy1000_adapter_driver = {
	.name = "ZY1000",
	.transports = jtag_only,
	.commands = zy1000_commands,

	.init = zy1000_init,
	.quit = zy1000_quit,
	.speed = zy1000_speed,
	.khz = zy1000_khz,
	.speed_div = zy1000_speed_div,
	.power_dropout = zy1000_power_dropout,
	.srst_asserted = zy1000_srst_asserted,

	.jtag_ops = &zy1000_interface,
};
