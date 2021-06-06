/***************************************************************************
 *   Copyright (C) 2009-2010 by David Brownell                             *
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

#ifndef OPENOCD_JTAG_SWD_H
#define OPENOCD_JTAG_SWD_H

#include <target/arm_adi_v5.h>

/* Bits in SWD command packets, written from host to target
 * first bit on the wire is START
 */
#define SWD_CMD_START	(1 << 0)	/* always set */
#define SWD_CMD_APNDP	(1 << 1)	/* set only for AP access */
#define SWD_CMD_RNW	(1 << 2)		/* set only for read access */
#define SWD_CMD_A32	(3 << 3)		/* bits A[3:2] of register addr */
#define SWD_CMD_PARITY	(1 << 5)	/* parity of APnDP|RnW|A32 */
#define SWD_CMD_STOP	(0 << 6)	/* always clear for synch SWD */
#define SWD_CMD_PARK	(1 << 7)	/* driven high by host */
/* followed by TRN, 3-bits of ACK, TRN */

/**
 * Construct a "cmd" byte, in lSB bit order, which swd_driver.read_reg()
 * and swd_driver.write_reg() methods will use directly.
 */
static inline uint8_t swd_cmd(bool is_read, bool is_ap, uint8_t regnum)
{
	uint8_t cmd = (is_ap ? SWD_CMD_APNDP : 0)
		| (is_read ? SWD_CMD_RNW : 0)
		| ((regnum & 0xc) << 1);

	/* 8 cmd bits 4:1 may be set */
	if (parity_u32(cmd))
		cmd |= SWD_CMD_PARITY;

	/* driver handles START, STOP, and TRN */

	return cmd;
}

/* SWD_ACK_* bits are defined in <target/arm_adi_v5.h> */

/*
 * The following sequences are updated to
 * ARM(tm) Debug Interface v5 Architecture Specification    ARM IHI 0031E
 */

/**
 * SWD Line reset.
 *
 * SWD Line reset is at least 50 SWCLK cycles with SWDIO driven high,
 * followed by at least two idle (low) cycle.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_line_reset[] = {
	/* At least 50 SWCLK cycles with SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* At least 2 idle (low) cycles */
	0x00,
};
static const unsigned swd_seq_line_reset_len = 64;

/**
 * JTAG-to-SWD sequence.
 *
 * The JTAG-to-SWD sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally a line reset in case the SWJ-DP was
 * already in SWD mode.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_jtag_to_swd[] = {
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching sequence from JTAG to SWD */
	0x9e, 0xe7,
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* At least 2 idle (low) cycles */
	0x00,
};
static const unsigned swd_seq_jtag_to_swd_len = 136;

/**
 * SWD-to-JTAG sequence.
 *
 * The SWD-to-JTAG sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally at least 5 TCK/SWCLK cycles with
 * TMS/SWDIO high to put the JTAG TAP in Test-Logic-Reset state.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_swd_to_jtag[] = {
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching sequence from SWD to JTAG */
	0x3c, 0xe7,
	/* At least 5 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff,
};
static const unsigned swd_seq_swd_to_jtag_len = 80;

/**
 * SWD-to-dormant sequence.
 *
 * This is at least 50 SWCLK cycles with SWDIO high to put the interface
 * in reset state, followed by a specific 16-bit sequence.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_swd_to_dormant[] = {
	/* At least 50 SWCLK cycles with SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	 /* Switching sequence from SWD to dormant */
	0xbc, 0xe3,
};
static const unsigned swd_seq_swd_to_dormant_len = 72;

/**
 * Dormant-to-SWD sequence.
 *
 * This is at least 8 TCK/SWCLK cycles with TMS/SWDIO high to abort any ongoing
 * selection alert sequence, followed by a specific 128-bit selection alert
 * sequence, followed by 4 TCK/SWCLK cycles with TMS/SWDIO low, followed by
 * a specific protocol-dependent activation code. For SWD the activation code
 * is an 8-bit sequence. The sequence ends with a line reset.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_dormant_to_swd[] = {
	/* At least 8 SWCLK cycles with SWDIO high */
	0xff,
	/* Selection alert sequence */
	0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
	0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
	/*
	 * 4 SWCLK cycles with SWDIO low ...
	 * + SWD activation code 0x1a ...
	 * + at least 8 SWCLK cycles with SWDIO high
	 */
	0xa0, /* ((0x00)      & GENMASK(3, 0)) | ((0x1a << 4) & GENMASK(7, 4)) */
	0xf1, /* ((0x1a >> 4) & GENMASK(3, 0)) | ((0xff << 4) & GENMASK(7, 4)) */
	0xff,
	/* At least 50 SWCLK cycles with SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* At least 2 idle (low) cycles */
	0x00,
};
static const unsigned swd_seq_dormant_to_swd_len = 224;

/**
 * JTAG-to-dormant sequence.
 *
 * This is at least 5 TCK cycles with TMS high to put the interface
 * in test-logic-reset state, followed by a specific 31-bit sequence.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_jtag_to_dormant[] = {
	/* At least 5 TCK cycles with TMS high */
	0xff,
	/*
	 * Still one TCK cycle with TMS high followed by 31 bits JTAG-to-DS
	 * select sequence 0xba, 0xbb, 0xbb, 0x33,
	 */
	0x75, /* ((0xff >> 7) & GENMASK(0, 0)) | ((0xba << 1) & GENMASK(7, 1)) */
	0x77, /* ((0xba >> 7) & GENMASK(0, 0)) | ((0xbb << 1) & GENMASK(7, 1)) */
	0x77, /* ((0xbb >> 7) & GENMASK(0, 0)) | ((0xbb << 1) & GENMASK(7, 1)) */
	0x67, /* ((0xbb >> 7) & GENMASK(0, 0)) | ((0x33 << 1) & GENMASK(7, 1)) */
};
static const unsigned swd_seq_jtag_to_dormant_len = 40;

/**
 * Dormant-to-JTAG sequence.
 *
 * This is at least 8 TCK/SWCLK cycles with TMS/SWDIO high to abort any ongoing
 * selection alert sequence, followed by a specific 128-bit selection alert
 * sequence, followed by 4 TCK/SWCLK cycles with TMS/SWDIO low, followed by
 * a specific protocol-dependent activation code. For JTAG there are two
 * possible activation codes:
 * - "JTAG-Serial": 12 bits 0x00, 0x00
 * - "Arm CoreSight JTAG-DP": 8 bits 0x0a
 * We use "JTAG-Serial" only, which seams more generic.
 * Since the target TAP can be either in Run/Test Idle or in Test-Logic-Reset
 * states, Arm recommends to put the TAP in Run/Test Idle using one TCK cycle
 * with TMS low. To keep the sequence length multiple of 8, 8 TCK cycle with
 * TMS low are sent (allowed by JTAG state machine).
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_dormant_to_jtag[] = {
	/* At least 8 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff,
	/* Selection alert sequence */
	0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
	0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
	/*
	 * 4 TCK/SWCLK cycles with TMS/SWDIO low ...
	 * + 12 bits JTAG-serial activation code 0x00, 0x00
	 */
	0x00, 0x00,
	/* put the TAP in Run/Test Idle */
	0x00,
};
static const unsigned swd_seq_dormant_to_jtag_len = 160;

struct swd_driver {
	/**
	 * Initialize the debug link so it can perform SWD operations.
	 *
	 * As an example, this would switch a dual-mode debug adapter
	 * into SWD mode and out of JTAG mode.
	 *
	 * @return ERROR_OK on success, else a negative fault code.
	 */
	int (*init)(void);

	/**
	 * Queue a special SWDIO sequence.
	 *
	 * @param seq The special sequence to generate.
	 * @return ERROR_OK if the sequence was queued, negative error if the
	 * sequence is unsupported.
	 */
	int (*switch_seq)(enum swd_special_seq seq);

	/**
	 * Queued read of an AP or DP register.
	 *
	 * @param Command byte with APnDP/RnW/addr/parity bits
	 * @param Where to store value to read from register
	 * @param ap_delay_hint Number of idle cycles that may be
	 * needed after an AP access to avoid WAITs
	 */
	void (*read_reg)(uint8_t cmd, uint32_t *value, uint32_t ap_delay_hint);

	/**
	 * Queued write of an AP or DP register.
	 *
	 * @param Command byte with APnDP/RnW/addr/parity bits
	 * @param Value to be written to the register
	 * @param ap_delay_hint Number of idle cycles that may be
	 * needed after an AP access to avoid WAITs
	 */
	void (*write_reg)(uint8_t cmd, uint32_t value, uint32_t ap_delay_hint);

	/**
	 * Execute any queued transactions and collect the result.
	 *
	 * @return ERROR_OK on success, Ack response code on WAIT/FAULT
	 * or negative error code on other kinds of failure.
	 */
	int (*run)(void);

	/**
	 * Configures data collection from the Single-wire
	 * trace (SWO) signal.
	 * @param swo true if SWO data collection should be routed.
	 *
	 * For example,  some debug adapters include a UART which
	 * is normally connected to a microcontroller's UART TX,
	 * but which may instead be connected to SWO for use in
	 * collecting ITM (and possibly ETM) trace data.
	 *
	 * @return ERROR_OK on success, else a negative fault code.
	 */
	int *(*trace)(bool swo);
};

int swd_init_reset(struct command_context *cmd_ctx);

#endif /* OPENOCD_JTAG_SWD_H */
