/***************************************************************************
 *   Copyright (C) 2009 by Mathias Kuester                                 *
 *   mkdorg@users.sourceforge.net                                          *
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

#include <jim.h>

#include "target.h"
#include "target_type.h"
#include "register.h"
#include "dsp563xx.h"
#include "dsp563xx_once.h"

#define JTAG_STATUS_STATIC_MASK		0x03
#define JTAG_STATUS_STATIC_VALUE	0x01

#define JTAG_STATUS_NORMAL		0x01
#define JTAG_STATUS_STOPWAIT		0x05
#define JTAG_STATUS_BUSY		0x09
#define JTAG_STATUS_DEBUG		0x0d

#define JTAG_INSTR_EXTEST		0x00
#define JTAG_INSTR_SAMPLE_PRELOAD	0x01
#define JTAG_INSTR_IDCODE		0x02
#define JTAG_INSTR_HIZ			0x04
#define JTAG_INSTR_CLAMP		0x05
#define JTAG_INSTR_ENABLE_ONCE		0x06
#define JTAG_INSTR_DEBUG_REQUEST	0x07
#define JTAG_INSTR_BYPASS		0x0F

/** */
static inline int dsp563xx_write_dr(struct jtag_tap *tap, uint8_t *dr_in, uint8_t *dr_out, int dr_len, int rti)
{
	jtag_add_plain_dr_scan(dr_len, dr_out, dr_in, TAP_IDLE);

	return ERROR_OK;
}

/** */
static inline int dsp563xx_write_dr_u8(struct jtag_tap *tap, uint8_t *dr_in, uint8_t dr_out, int dr_len, int rti)
{
	return dsp563xx_write_dr(tap, dr_in, &dr_out, dr_len, rti);
}

/** */
static inline int dsp563xx_write_dr_u32(struct jtag_tap *tap, uint32_t *dr_in, uint32_t dr_out, int dr_len, int rti)
{
	return dsp563xx_write_dr(tap, (uint8_t *) dr_in, (uint8_t *) &dr_out, dr_len, rti);
}

/** single word instruction */
static inline int dsp563xx_once_ir_exec(struct jtag_tap *tap, int flush, uint8_t instr,
										uint8_t rw, uint8_t go, uint8_t ex)
{
	int err;

	err = dsp563xx_write_dr_u8(tap, 0, instr | (ex << 5) | (go << 6) | (rw << 7), 8, 0);
	if (err != ERROR_OK)
		return err;
	if (flush)
		err = jtag_execute_queue();
	return err;
}

/* IR and DR functions */
static inline int dsp563xx_write_ir(struct jtag_tap *tap, uint8_t *ir_in, uint8_t *ir_out, int ir_len, int rti)
{
	jtag_add_plain_ir_scan(tap->ir_length, ir_out, ir_in, TAP_IDLE);

	return ERROR_OK;
}

static inline int dsp563xx_write_ir_u8(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out, int ir_len, int rti)
{
	return dsp563xx_write_ir(tap, ir_in, &ir_out, ir_len, rti);
}

static inline int dsp563xx_jtag_sendinstr(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out)
{
	return dsp563xx_write_ir_u8(tap, ir_in, ir_out, tap->ir_length, 1);
}

/** */
int dsp563xx_once_target_status(struct jtag_tap *tap)
{
	int err;
	uint8_t jtag_status;

	err = dsp563xx_jtag_sendinstr(tap, &jtag_status, JTAG_INSTR_ENABLE_ONCE);
	if (err != ERROR_OK)
		return TARGET_UNKNOWN;
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return TARGET_UNKNOWN;

	/* verify correct static status pattern */
	if ((jtag_status & JTAG_STATUS_STATIC_MASK) != JTAG_STATUS_STATIC_VALUE)
		return TARGET_UNKNOWN;

	if (jtag_status != JTAG_STATUS_DEBUG)
		return TARGET_RUNNING;

	return TARGET_HALTED;
}

/** */
int dsp563xx_once_request_debug(struct jtag_tap *tap, int reset_state)
{
	int err;
	uint8_t ir_in = 0, pattern = 0;
	uint32_t retry = 0;

	/* in reset state we only get a ACK
	 * from the interface */
	if (reset_state)
		pattern = 1;
	else
		pattern = JTAG_STATUS_DEBUG;

	/* wait until we get the ack */
	while (ir_in != pattern) {
		err = dsp563xx_jtag_sendinstr(tap, &ir_in, JTAG_INSTR_DEBUG_REQUEST);
		if (err != ERROR_OK)
			return err;
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		LOG_DEBUG("debug request: %02X", ir_in);

		if (retry++ == 100)
			return ERROR_TARGET_FAILURE;
	}

	/* we cant enable the once in reset state */
	if (pattern == 1)
		return ERROR_OK;

	/* try to enable once */
	retry = 0;
	ir_in = 0;
	while (ir_in != pattern) {
		err = dsp563xx_jtag_sendinstr(tap, &ir_in, JTAG_INSTR_ENABLE_ONCE);
		if (err != ERROR_OK)
			return err;
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;

		LOG_DEBUG("enable once: %02X", ir_in);

		if (retry++ == 100) {
			LOG_DEBUG("error");
			return ERROR_TARGET_FAILURE;
		}
	}

	if (ir_in != JTAG_STATUS_DEBUG)
		return ERROR_TARGET_FAILURE;

	return ERROR_OK;
}

/** once read registers */
int dsp563xx_once_read_register(struct jtag_tap *tap, int flush, struct once_reg *regs, int len)
{
	int i;
	int err = ERROR_OK;

	for (i = 0; i < len; i++) {
		err = dsp563xx_once_reg_read_ex(tap, flush, regs[i].addr, regs[i].len, &regs[i].reg);
		if (err != ERROR_OK)
			return err;
	}

	if (flush)
		err = jtag_execute_queue();
	return err;
}

/** once read register with register len */
int dsp563xx_once_reg_read_ex(struct jtag_tap *tap, int flush, uint8_t reg, uint8_t len, uint32_t *data)
{
	int err;

	err = dsp563xx_once_ir_exec(tap, 1, reg, 1, 0, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_write_dr_u32(tap, data, 0x00, len, 0);
	if (err != ERROR_OK)
		return err;
	if (flush)
		err = jtag_execute_queue();

	return err;
}

/** once read register */
int dsp563xx_once_reg_read(struct jtag_tap *tap, int flush, uint8_t reg, uint32_t *data)
{
	int err;

	err = dsp563xx_once_ir_exec(tap, flush, reg, 1, 0, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_write_dr_u32(tap, data, 0x00, 24, 0);
	if (err != ERROR_OK)
		return err;
	if (flush)
		err = jtag_execute_queue();

	return err;
}

/** once write register */
int dsp563xx_once_reg_write(struct jtag_tap *tap, int flush, uint8_t reg, uint32_t data)
{
	int err;

	err = dsp563xx_once_ir_exec(tap, flush, reg, 0, 0, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_write_dr_u32(tap, 0x00, data, 24, 0);
	if (err != ERROR_OK)
		return err;
	if (flush)
		err = jtag_execute_queue();
	return err;
}

/** single word instruction */
int dsp563xx_once_execute_sw_ir(struct jtag_tap *tap, int flush, uint32_t opcode)
{
	int err;

	err = dsp563xx_once_ir_exec(tap, flush, DSP563XX_ONCE_OPDBR, 0, 1, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0);
	if (err != ERROR_OK)
		return err;
	if (flush)
		err = jtag_execute_queue();
	return err;
}

/** double word instruction */
int dsp563xx_once_execute_dw_ir(struct jtag_tap *tap, int flush, uint32_t opcode, uint32_t operand)
{
	int err;

	err = dsp563xx_once_ir_exec(tap, flush, DSP563XX_ONCE_OPDBR, 0, 0, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0);
	if (err != ERROR_OK)
		return err;
	if (flush) {
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;
	}

	err = dsp563xx_once_ir_exec(tap, flush, DSP563XX_ONCE_OPDBR, 0, 1, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_write_dr_u32(tap, 0, operand, 24, 0);
	if (err != ERROR_OK)
		return err;
	if (flush) {
		err = jtag_execute_queue();
		if (err != ERROR_OK)
			return err;
	}

	return ERROR_OK;
}
