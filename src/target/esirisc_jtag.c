/***************************************************************************
 *   Copyright (C) 2018 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 *   James Zhao <hjz@squareup.com>                                         *
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

#include <helper/binarybuffer.h>
#include <helper/log.h>
#include <helper/types.h>
#include <jtag/jtag.h>
#include <jtag/commands.h>
#include <jtag/interface.h>

#include "esirisc_jtag.h"

static void esirisc_jtag_set_instr(struct esirisc_jtag *jtag_info, uint32_t new_instr)
{
	struct jtag_tap *tap = jtag_info->tap;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr) {
		struct scan_field field;
		uint8_t t[4] = { 0 };

		field.num_bits = tap->ir_length;
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);
		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);
	}
}

/*
 * The data register is latched every 8 bits while in the Shift-DR state
 * (Update-DR is not supported). This necessitates prepending padding
 * bits to ensure data is aligned when multiple TAPs are present.
 */
static int esirisc_jtag_get_padding(void)
{
	int padding = 0;
	int bypass_devices = 0;

	for (struct jtag_tap *tap = jtag_tap_next_enabled(NULL); tap;
			tap = jtag_tap_next_enabled(tap))
		if (tap->bypass)
			bypass_devices++;

	int num_bits = bypass_devices % 8;
	if (num_bits > 0)
		padding = 8 - num_bits;

	return padding;
}

static int esirisc_jtag_count_bits(int num_fields, struct scan_field *fields)
{
	int bit_count = 0;

	for (int i = 0; i < num_fields; ++i)
		bit_count += fields[i].num_bits;

	return bit_count;
}

/*
 * Data received from the target will be byte-stuffed if it contains
 * either the pad byte (0xAA) or stuffing marker (0x55). Buffers should
 * be sized twice the expected length to account for stuffing overhead.
 */
static void esirisc_jtag_unstuff(uint8_t *data, size_t len)
{
	uint8_t *r, *w;
	uint8_t *end;

	r = w = data;
	end = data + len;
	while (r < end) {
		if (*r == STUFF_MARKER) {
			r++; /* skip stuffing marker */
			assert(r < end);
			*w++ = *r++ ^ STUFF_MARKER;
		} else
			*w++ = *r++;
	}
}

/*
 * The eSi-Debug protocol defines a byte-oriented command/response
 * channel that operates over serial or JTAG. While not strictly
 * required, separate DR scans are used for sending and receiving data.
 * This allows the TAP to recover gracefully if the byte stream is
 * corrupted at the expense of sending additional padding bits.
 */

static int esirisc_jtag_send(struct esirisc_jtag *jtag_info, uint8_t command,
		int num_out_fields, struct scan_field *out_fields)
{
	int num_fields = 2 + num_out_fields;
	struct scan_field *fields = cmd_queue_alloc(num_fields * sizeof(struct scan_field));

	esirisc_jtag_set_instr(jtag_info, INSTR_DEBUG);

	fields[0].num_bits = esirisc_jtag_get_padding();
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].num_bits = 8;
	fields[1].out_value = &command;
	fields[1].in_value = NULL;

	/* append command data */
	for (int i = 0; i < num_out_fields; ++i)
		jtag_scan_field_clone(&fields[2+i], &out_fields[i]);

	jtag_add_dr_scan(jtag_info->tap, num_fields, fields, TAP_IDLE);

	return jtag_execute_queue();
}

static int esirisc_jtag_recv(struct esirisc_jtag *jtag_info,
		int num_in_fields, struct scan_field *in_fields)
{
	int num_in_bits = esirisc_jtag_count_bits(num_in_fields, in_fields);
	int num_in_bytes = DIV_ROUND_UP(num_in_bits, 8);

	struct scan_field fields[3];
	uint8_t r[num_in_bytes * 2];

	esirisc_jtag_set_instr(jtag_info, INSTR_DEBUG);

	fields[0].num_bits = esirisc_jtag_get_padding() + 1;
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].num_bits = 8;
	fields[1].out_value = NULL;
	fields[1].in_value = &jtag_info->status;

	fields[2].num_bits = num_in_bits * 2;
	fields[2].out_value = NULL;
	fields[2].in_value = r;

	jtag_add_dr_scan(jtag_info->tap, ARRAY_SIZE(fields), fields, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* unstuff response data and write back to caller */
	if (num_in_fields > 0) {
		esirisc_jtag_unstuff(r, ARRAY_SIZE(r));

		int bit_count = 0;
		for (int i = 0; i < num_in_fields; ++i) {
			buf_set_buf(r, bit_count, in_fields[i].in_value, 0, in_fields[i].num_bits);
			bit_count += in_fields[i].num_bits;
		}
	}

	return ERROR_OK;
}

static int esirisc_jtag_check_status(struct esirisc_jtag *jtag_info)
{
	uint8_t eid = esirisc_jtag_get_eid(jtag_info);
	if (eid != EID_NONE) {
		LOG_ERROR("esirisc_jtag: bad status: 0x%02" PRIx8 " (DA: %" PRId32 ", "
				"S: %" PRId32 ", EID: 0x%02" PRIx8 ")",
				jtag_info->status, esirisc_jtag_is_debug_active(jtag_info),
				esirisc_jtag_is_stopped(jtag_info), eid);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int esirisc_jtag_send_and_recv(struct esirisc_jtag *jtag_info, uint8_t command,
		int num_out_fields, struct scan_field *out_fields,
		int num_in_fields, struct scan_field *in_fields)
{
	int retval;

	jtag_info->status = 0;	/* clear status */

	retval = esirisc_jtag_send(jtag_info, command, num_out_fields, out_fields);
	if (retval != ERROR_OK) {
		LOG_ERROR("esirisc_jtag: send failed (command: 0x%02" PRIx8 ")", command);
		return ERROR_FAIL;
	}

	retval = esirisc_jtag_recv(jtag_info, num_in_fields, in_fields);
	if (retval != ERROR_OK) {
		LOG_ERROR("esirisc_jtag: recv failed (command: 0x%02" PRIx8 ")", command);
		return ERROR_FAIL;
	}

	return esirisc_jtag_check_status(jtag_info);
}

/*
 * Status is automatically updated after each command completes;
 * these functions make each field available to the caller.
 */

bool esirisc_jtag_is_debug_active(struct esirisc_jtag *jtag_info)
{
	return !!(jtag_info->status & 1<<7);	/* DA */
}

bool esirisc_jtag_is_stopped(struct esirisc_jtag *jtag_info)
{
	return !!(jtag_info->status & 1<<6);	/* S */
}

uint8_t esirisc_jtag_get_eid(struct esirisc_jtag *jtag_info)
{
	return jtag_info->status & 0x3f;		/* EID */
}

/*
 * Most commands manipulate target data (eg. memory and registers); each
 * command returns a status byte that indicates success. Commands must
 * transmit multibyte values in big-endian order, however response
 * values are in little-endian order. Target endianness does not have an
 * effect on this ordering.
 */

int esirisc_jtag_read_byte(struct esirisc_jtag *jtag_info, uint32_t address, uint8_t *data)
{
	struct scan_field out_fields[1];
	uint8_t a[4];

	out_fields[0].num_bits = 32;
	out_fields[0].out_value = a;
	h_u32_to_be(a, address);
	out_fields[0].in_value = NULL;

	struct scan_field in_fields[1];
	uint8_t d[1];

	in_fields[0].num_bits = 8;
	in_fields[0].out_value = NULL;
	in_fields[0].in_value = d;

	int retval = esirisc_jtag_send_and_recv(jtag_info, DEBUG_READ_BYTE,
			ARRAY_SIZE(out_fields), out_fields, ARRAY_SIZE(in_fields), in_fields);
	if (retval != ERROR_OK)
		return retval;

	*data = *d;
	LOG_DEBUG("address: 0x%" PRIx32 ", data: 0x%" PRIx8, address, *data);

	return ERROR_OK;
}

int esirisc_jtag_read_hword(struct esirisc_jtag *jtag_info, uint32_t address, uint16_t *data)
{
	struct scan_field out_fields[1];
	uint8_t a[4];

	out_fields[0].num_bits = 32;
	out_fields[0].out_value = a;
	h_u32_to_be(a, address);
	out_fields[0].in_value = NULL;

	struct scan_field in_fields[1];
	uint8_t d[2];

	in_fields[0].num_bits = 16;
	in_fields[0].out_value = NULL;
	in_fields[0].in_value = d;

	int retval = esirisc_jtag_send_and_recv(jtag_info, DEBUG_READ_HWORD,
			ARRAY_SIZE(out_fields), out_fields, ARRAY_SIZE(in_fields), in_fields);
	if (retval != ERROR_OK)
		return retval;

	*data = le_to_h_u16(d);
	LOG_DEBUG("address: 0x%" PRIx32 ", data: 0x%" PRIx16, address, *data);

	return ERROR_OK;
}

int esirisc_jtag_read_word(struct esirisc_jtag *jtag_info, uint32_t address, uint32_t *data)
{
	struct scan_field out_fields[1];
	uint8_t a[4];

	out_fields[0].num_bits = 32;
	out_fields[0].out_value = a;
	h_u32_to_be(a, address);
	out_fields[0].in_value = NULL;

	struct scan_field in_fields[1];
	uint8_t d[4];

	in_fields[0].num_bits = 32;
	in_fields[0].out_value = NULL;
	in_fields[0].in_value = d;

	int retval = esirisc_jtag_send_and_recv(jtag_info, DEBUG_READ_WORD,
			ARRAY_SIZE(out_fields), out_fields, ARRAY_SIZE(in_fields), in_fields);
	if (retval != ERROR_OK)
		return retval;

	*data = le_to_h_u32(d);
	LOG_DEBUG("address: 0x%" PRIx32 ", data: 0x%" PRIx32, address, *data);

	return ERROR_OK;
}

int esirisc_jtag_write_byte(struct esirisc_jtag *jtag_info, uint32_t address, uint8_t data)
{
	struct scan_field out_fields[2];
	uint8_t a[4];

	LOG_DEBUG("address: 0x%" PRIx32 ", data: 0x%" PRIx8, address, data);

	out_fields[0].num_bits = 32;
	out_fields[0].out_value = a;
	h_u32_to_be(a, address);
	out_fields[0].in_value = NULL;

	out_fields[1].num_bits = 8;
	out_fields[1].out_value = &data;
	out_fields[1].in_value = NULL;

	return esirisc_jtag_send_and_recv(jtag_info, DEBUG_WRITE_BYTE,
			ARRAY_SIZE(out_fields), out_fields, 0, NULL);
}

int esirisc_jtag_write_hword(struct esirisc_jtag *jtag_info, uint32_t address, uint16_t data)
{
	struct scan_field out_fields[2];
	uint8_t a[4], d[2];

	LOG_DEBUG("address: 0x%" PRIx32 ", data: 0x%" PRIx16, address, data);

	out_fields[0].num_bits = 32;
	out_fields[0].out_value = a;
	h_u32_to_be(a, address);
	out_fields[0].in_value = NULL;

	out_fields[1].num_bits = 16;
	out_fields[1].out_value = d;
	h_u16_to_be(d, data);
	out_fields[1].in_value = NULL;

	return esirisc_jtag_send_and_recv(jtag_info, DEBUG_WRITE_HWORD,
			ARRAY_SIZE(out_fields), out_fields, 0, NULL);
}

int esirisc_jtag_write_word(struct esirisc_jtag *jtag_info, uint32_t address, uint32_t data)
{
	struct scan_field out_fields[2];
	uint8_t a[4], d[4];

	LOG_DEBUG("address: 0x%" PRIx32 ", data: 0x%" PRIx32, address, data);

	out_fields[0].num_bits = 32;
	out_fields[0].out_value = a;
	h_u32_to_be(a, address);
	out_fields[0].in_value = NULL;

	out_fields[1].num_bits = 32;
	out_fields[1].out_value = d;
	h_u32_to_be(d, data);
	out_fields[1].in_value = NULL;

	return esirisc_jtag_send_and_recv(jtag_info, DEBUG_WRITE_WORD,
			ARRAY_SIZE(out_fields), out_fields, 0, NULL);
}

int esirisc_jtag_read_reg(struct esirisc_jtag *jtag_info, uint8_t reg, uint32_t *data)
{
	struct scan_field out_fields[1];

	out_fields[0].num_bits = 8;
	out_fields[0].out_value = &reg;
	out_fields[0].in_value = NULL;

	struct scan_field in_fields[1];
	uint8_t d[4];

	in_fields[0].num_bits = 32;
	in_fields[0].out_value = NULL;
	in_fields[0].in_value = d;

	int retval = esirisc_jtag_send_and_recv(jtag_info, DEBUG_READ_REG,
			ARRAY_SIZE(out_fields), out_fields, ARRAY_SIZE(in_fields), in_fields);
	if (retval != ERROR_OK)
		return retval;

	*data = le_to_h_u32(d);
	LOG_DEBUG("register: 0x%" PRIx8 ", data: 0x%" PRIx32, reg, *data);

	return ERROR_OK;
}

int esirisc_jtag_write_reg(struct esirisc_jtag *jtag_info, uint8_t reg, uint32_t data)
{
	struct scan_field out_fields[2];
	uint8_t d[4];

	LOG_DEBUG("register: 0x%" PRIx8 ", data: 0x%" PRIx32, reg, data);

	out_fields[0].num_bits = 8;
	out_fields[0].out_value = &reg;
	out_fields[0].in_value = NULL;

	out_fields[1].num_bits = 32;
	out_fields[1].out_value = d;
	h_u32_to_be(d, data);
	out_fields[1].in_value = NULL;

	return esirisc_jtag_send_and_recv(jtag_info, DEBUG_WRITE_REG,
			ARRAY_SIZE(out_fields), out_fields, 0, NULL);
}

int esirisc_jtag_read_csr(struct esirisc_jtag *jtag_info, uint8_t bank, uint8_t csr, uint32_t *data)
{
	struct scan_field out_fields[1];
	uint8_t c[2];

	out_fields[0].num_bits = 16;
	out_fields[0].out_value = c;
	h_u16_to_be(c, (csr << 5) | bank);
	out_fields[0].in_value = NULL;

	struct scan_field in_fields[1];
	uint8_t d[4];

	in_fields[0].num_bits = 32;
	in_fields[0].out_value = NULL;
	in_fields[0].in_value = d;

	int retval = esirisc_jtag_send_and_recv(jtag_info, DEBUG_READ_CSR,
			ARRAY_SIZE(out_fields), out_fields, ARRAY_SIZE(in_fields), in_fields);
	if (retval != ERROR_OK)
		return retval;

	*data = le_to_h_u32(d);
	LOG_DEBUG("bank: 0x%" PRIx8 ", csr: 0x%" PRIx8 ", data: 0x%" PRIx32, bank, csr, *data);

	return ERROR_OK;
}

int esirisc_jtag_write_csr(struct esirisc_jtag *jtag_info, uint8_t bank, uint8_t csr, uint32_t data)
{
	struct scan_field out_fields[2];
	uint8_t c[2], d[4];

	LOG_DEBUG("bank: 0x%" PRIx8 ", csr: 0x%" PRIx8 ", data: 0x%" PRIx32, bank, csr, data);

	out_fields[0].num_bits = 16;
	out_fields[0].out_value = c;
	h_u16_to_be(c, (csr << 5) | bank);
	out_fields[0].in_value = NULL;

	out_fields[1].num_bits = 32;
	out_fields[1].out_value = d;
	h_u32_to_be(d, data);
	out_fields[1].in_value = NULL;

	return esirisc_jtag_send_and_recv(jtag_info, DEBUG_WRITE_CSR,
			ARRAY_SIZE(out_fields), out_fields, 0, NULL);
}

/*
 * Control commands affect CPU operation; these commands send no data
 * and return a status byte.
 */

static inline int esirisc_jtag_send_ctrl(struct esirisc_jtag *jtag_info, uint8_t command)
{
	return esirisc_jtag_send_and_recv(jtag_info, command, 0, NULL, 0, NULL);
}

int esirisc_jtag_enable_debug(struct esirisc_jtag *jtag_info)
{
	return esirisc_jtag_send_ctrl(jtag_info, DEBUG_ENABLE_DEBUG);
}

int esirisc_jtag_disable_debug(struct esirisc_jtag *jtag_info)
{
	return esirisc_jtag_send_ctrl(jtag_info, DEBUG_DISABLE_DEBUG);
}

int esirisc_jtag_assert_reset(struct esirisc_jtag *jtag_info)
{
	return esirisc_jtag_send_ctrl(jtag_info, DEBUG_ASSERT_RESET);
}

int esirisc_jtag_deassert_reset(struct esirisc_jtag *jtag_info)
{
	return esirisc_jtag_send_ctrl(jtag_info, DEBUG_DEASSERT_RESET);
}

int esirisc_jtag_break(struct esirisc_jtag *jtag_info)
{
	return esirisc_jtag_send_ctrl(jtag_info, DEBUG_BREAK);
}

int esirisc_jtag_continue(struct esirisc_jtag *jtag_info)
{
	return esirisc_jtag_send_ctrl(jtag_info, DEBUG_CONTINUE);
}

int esirisc_jtag_flush_caches(struct esirisc_jtag *jtag_info)
{
	return esirisc_jtag_send_ctrl(jtag_info, DEBUG_FLUSH_CACHES);
}
