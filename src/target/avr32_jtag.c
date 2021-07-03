/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
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

#include "target.h"
#include "jtag/jtag.h"
#include "avr32_jtag.h"

static int avr32_jtag_set_instr(struct avr32_jtag *jtag_info, int new_instr)
{
	struct jtag_tap *tap;
	int busy = 0;

	tap = jtag_info->tap;
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != (uint32_t)new_instr) {
		do {
			struct scan_field field;
			uint8_t t[4] = { 0 };
			uint8_t ret[4];

			field.num_bits = tap->ir_length;
			field.out_value = t;
			buf_set_u32(t, 0, field.num_bits, new_instr);
			field.in_value = ret;

			jtag_add_ir_scan(tap, &field, TAP_IDLE);
			if (jtag_execute_queue() != ERROR_OK) {
				LOG_ERROR("%s: setting address failed", __func__);
				return ERROR_FAIL;
			}
			busy = buf_get_u32(ret, 2, 1);
		} while (busy); /* check for busy bit */
	}

	return ERROR_OK;
}

static int avr32_jtag_nexus_set_address(struct avr32_jtag *jtag_info,
		uint32_t addr, int mode)
{
	struct scan_field fields[2];
	uint8_t addr_buf[4];
	uint8_t busy_buf[4];
	int busy;

	memset(fields, 0, sizeof(fields));

	do {
		memset(addr_buf, 0, sizeof(addr_buf));
		memset(busy_buf, 0, sizeof(busy_buf));

		buf_set_u32(addr_buf, 0, 1, mode);
		buf_set_u32(addr_buf, 1, 7, addr);

		fields[0].num_bits = 26;
		fields[0].in_value = NULL;
		fields[0].out_value = NULL;

		fields[1].num_bits = 8;
		fields[1].in_value = busy_buf;
		fields[1].out_value = addr_buf;

		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_IDLE);
		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: setting address failed", __func__);
			return ERROR_FAIL;
		}
		busy = buf_get_u32(busy_buf, 6, 1);
	} while (busy);

	return ERROR_OK;
}


static int avr32_jtag_nexus_read_data(struct avr32_jtag *jtag_info,
	uint32_t *pdata)
{

	struct scan_field fields[2];
	uint8_t data_buf[4];
	uint8_t busy_buf[4];
	int busy;

	do {
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));

		fields[0].num_bits = 32;
		fields[0].out_value = NULL;
		fields[0].in_value = data_buf;


		fields[1].num_bits = 2;
		fields[1].in_value = busy_buf;
		fields[1].out_value = NULL;

		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: reading data  failed", __func__);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 1);
	} while (busy);

	*pdata = buf_get_u32(data_buf, 0, 32);

	return ERROR_OK;
}

static int avr32_jtag_nexus_write_data(struct avr32_jtag *jtag_info,
		uint32_t data)
{

	struct scan_field fields[2];
	uint8_t data_buf[4];
	uint8_t busy_buf[4];
	uint8_t dummy_buf[4];
	int busy;

	do {
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));
		memset(dummy_buf, 0, sizeof(dummy_buf));

		fields[0].num_bits = 2;
		fields[0].in_value = busy_buf;
		fields[0].out_value = dummy_buf;


		buf_set_u32(data_buf, 0, 32, data);
		fields[1].num_bits = 32;
		fields[1].in_value = NULL;
		fields[1].out_value = data_buf;

		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: reading data  failed", __func__);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 0);
	} while (busy);


	return ERROR_OK;
}

int avr32_jtag_nexus_read(struct avr32_jtag *jtag_info,
		uint32_t addr, uint32_t *value)
{
	avr32_jtag_set_instr(jtag_info, AVR32_INST_NEXUS_ACCESS);
	avr32_jtag_nexus_set_address(jtag_info, addr, MODE_READ);
	return avr32_jtag_nexus_read_data(jtag_info, value);
}

int avr32_jtag_nexus_write(struct avr32_jtag *jtag_info,
		uint32_t addr, uint32_t value)
{
	avr32_jtag_set_instr(jtag_info, AVR32_INST_NEXUS_ACCESS);
	avr32_jtag_nexus_set_address(jtag_info, addr, MODE_WRITE);
	return avr32_jtag_nexus_write_data(jtag_info, value);
}

static int avr32_jtag_mwa_set_address(struct avr32_jtag *jtag_info, int slave,
		uint32_t addr, int mode)
{
	struct scan_field fields[2];
	uint8_t addr_buf[4];
	uint8_t slave_buf[4];
	uint8_t busy_buf[4];
	int busy;

	memset(fields, 0, sizeof(fields));

	do {
		memset(addr_buf, 0, sizeof(addr_buf));
		memset(busy_buf, 0, sizeof(busy_buf));
		memset(slave_buf, 0, sizeof(slave_buf));

		buf_set_u32(slave_buf, 0, 4, slave);
		buf_set_u32(addr_buf, 0, 1, mode);
		buf_set_u32(addr_buf, 1, 30, addr >> 2);

		fields[0].num_bits = 31;
		fields[0].in_value = NULL;
		fields[0].out_value = addr_buf;

		fields[1].num_bits = 4;
		fields[1].in_value = busy_buf;
		fields[1].out_value = slave_buf;

		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_IDLE);
		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: setting address failed", __func__);
			return ERROR_FAIL;
		}
		busy = buf_get_u32(busy_buf, 1, 1);
	} while (busy);

	return ERROR_OK;
}

static int avr32_jtag_mwa_read_data(struct avr32_jtag *jtag_info,
	uint32_t *pdata)
{

	struct scan_field fields[2];
	uint8_t data_buf[4];
	uint8_t busy_buf[4];
	int busy;

	do {
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));

		fields[0].num_bits = 32;
		fields[0].out_value = NULL;
		fields[0].in_value = data_buf;


		fields[1].num_bits = 3;
		fields[1].in_value = busy_buf;
		fields[1].out_value = NULL;

		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: reading data  failed", __func__);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 1);
	} while (busy);

	*pdata = buf_get_u32(data_buf, 0, 32);

	return ERROR_OK;
}

static int avr32_jtag_mwa_write_data(struct avr32_jtag *jtag_info,
	uint32_t data)
{

	struct scan_field fields[2];
	uint8_t data_buf[4];
	uint8_t busy_buf[4];
	uint8_t zero_buf[4];
	int busy;

	do {
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));
		memset(zero_buf, 0, sizeof(zero_buf));

		buf_set_u32(data_buf, 0, 32, data);
		fields[0].num_bits = 3;
		fields[0].in_value = busy_buf;
		fields[0].out_value = zero_buf;

		fields[1].num_bits = 32;
		fields[1].out_value = data_buf;
		fields[1].in_value = NULL;


		jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: reading data  failed", __func__);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 1);
	} while (busy);

	return ERROR_OK;
}

int avr32_jtag_mwa_read(struct avr32_jtag *jtag_info, int slave,
		uint32_t addr, uint32_t *value)
{
	avr32_jtag_set_instr(jtag_info, AVR32_INST_MW_ACCESS);
	avr32_jtag_mwa_set_address(jtag_info, slave, addr, MODE_READ);
	avr32_jtag_mwa_read_data(jtag_info, value);

	return ERROR_OK;
}

int avr32_jtag_mwa_write(struct avr32_jtag *jtag_info, int slave,
		uint32_t addr, uint32_t value)
{
	avr32_jtag_set_instr(jtag_info, AVR32_INST_MW_ACCESS);
	avr32_jtag_mwa_set_address(jtag_info, slave, addr, MODE_WRITE);
	avr32_jtag_mwa_write_data(jtag_info, value);

	return ERROR_OK;
}

int avr32_jtag_exec(struct avr32_jtag *jtag_info, uint32_t inst)
{
	int retval;
	uint32_t ds;

	retval = avr32_jtag_nexus_write(jtag_info, AVR32_OCDREG_DINST, inst);
	if (retval != ERROR_OK)
		return retval;

	do {
		retval = avr32_jtag_nexus_read(jtag_info, AVR32_OCDREG_DS, &ds);
		if (retval != ERROR_OK)
			return retval;
	} while ((ds & OCDREG_DS_DBA) && !(ds & OCDREG_DS_INC));

	return ERROR_OK;
}

int avr32_ocd_setbits(struct avr32_jtag *jtag, int reg, uint32_t bits)
{
	uint32_t value;
	int res;

	res = avr32_jtag_nexus_read(jtag, reg, &value);
	if (res)
		return res;

	value |= bits;
	res = avr32_jtag_nexus_write(jtag, reg, value);
	if (res)
		return res;

	return ERROR_OK;
}

int avr32_ocd_clearbits(struct avr32_jtag *jtag, int reg, uint32_t bits)
{
	uint32_t value;
	int res;

	res = avr32_jtag_nexus_read(jtag, reg, &value);
	if (res)
		return res;

	value &= ~bits;
	res = avr32_jtag_nexus_write(jtag, reg, value);
	if (res)
		return res;

	return ERROR_OK;
}
