/***************************************************************************
 *   Copyright (C) 2017 by James Murray <james@nscc.info                   *
 *   Based on code:                                                        *
 *       Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>   *
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
#include "mpc5xxx_jtag.h"
#include "jtag/interface.h"

#define READ_MAXLOOPS 500

int mpc5xxx_jtag_set_instr(struct mpc5xxx_jtag *jtag_info, int new_instr)
{
	struct jtag_tap *tap;
	int busy = 0;

	tap = jtag_info->tap;
	if (tap == NULL)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != (uint32_t)new_instr) {
		do {
			struct scan_field field;
			uint8_t t[4];
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

int mpc5xxx_jtag_read_data(struct mpc5xxx_jtag *jtag_info,
	uint32_t *pdata, uint32_t size)
{

	struct scan_field field;
	uint8_t data_buf[4];
	uint8_t busy_buf[4];
	int busy;

	if (size > 32)
		size = 32;


	do {
		int jterr;
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));

		field.num_bits = size;
		field.out_value = NULL;
		field.in_value = data_buf;

		jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

		jterr = jtag_execute_queue();
		if (jterr != ERROR_OK) {
			LOG_ERROR("%s: reading data failed with error %d", __func__, jterr);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 1);
	} while (busy);

	*pdata = buf_get_u32(data_buf, 0, 32);

	return ERROR_OK;
}

int mpc5xxx_jtag_write_data(struct mpc5xxx_jtag *jtag_info,
		uint32_t data, uint32_t size)
{

	struct scan_field field;
	uint8_t data_buf[4];
	uint8_t busy_buf[4];
	int busy;

	if (size > 32)
		size = 32;


	do {
		int jterr ;
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));

		buf_set_u32(data_buf, 0, size, data);
		field.num_bits = size;
		field.in_value = NULL;
		field.out_value = data_buf;

		jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

		jterr = jtag_execute_queue();
		if (jterr != ERROR_OK) {
			LOG_ERROR("%s: write data failed with error %d", __func__, jterr);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 0);
	} while (busy);


	return ERROR_OK;
}

int mpc5xxx_jtag_access_once(struct mpc5xxx_jtag *jtag_info)
{
	int res;
	uint32_t did;

	if (jtag_info->once == MPC5XXX_TAP_INVALID) {
		printf("OnCE not yet set.\n");
		return ERROR_FAIL;
	}

	if (jtag_info->current_tap == jtag_info->once)
			return ERROR_OK;

	if (jtag_info->current_tap != MPC5XXX_TAP_JTAG) {
		/* Revert to JTAGC first */
		res = mpc5xxx_jtag_access_jtagc(jtag_info);
		if (res)
			return res;
	}

	printf("Switching to ONCE, (TAP id = 0x%02x)\n", jtag_info->once);

	res = mpc5xxx_jtag_set_instr(jtag_info, jtag_info->once);
	if (res)
		return res;

	jtag_info->current_tap = jtag_info->once;
	/* OnCE uses 10 bits */
	jtag_info->tap->ir_length = 10;

		/* Read the OnCE JTAG ID for info */
		res = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DID, &did, 32);
	if (res)
		return res;

	if (did != jtag_info->tap->expected_ids[1]) {
		printf("Serious JTAG problem! Didn't find OnCE ID (0x%08x), found 0x%08x.\n",
				jtag_info->tap->expected_ids[1], did);
		printf("Exiting!!!\n");
		exit(2);
		return ERROR_FAIL;
	}
printf("Switched to ONCE ok.\n");
	return ERROR_OK;
}

int mpc5xxx_jtag_access_jtagc(struct mpc5xxx_jtag *jtag_info)
{
	int res;
	uint32_t did;

#define JTAG_TAP_ID 0x3e

	if (jtag_info->current_tap == MPC5XXX_TAP_JTAG)

		return ERROR_OK;

	printf("Switching to JTAGC\n");
	(void)interface_jtag_add_reset(0, 0);

	/* Get out of whatever TAP we were in and return to JTAGC */
	res = jtag_add_statemove(TAP_DRPAUSE);
	if (res)
		return res;

	/* Run-test/idle for JTAGC */
	res = jtag_add_statemove(TAP_IDLE);
	if (res)
		return res;

	jtag_info->current_tap = MPC5XXX_TAP_JTAG;
	jtag_info->tap->ir_length = jtag_info->jtag_irlen;

	/* Validate that the correct IDcode is here */
	res = mpc5xxx_jtagc_read(jtag_info, MPC5XXX_INST_IDCODE, &did, 32);
	if (res)
		return res;
	if (did != jtag_info->tap->expected_ids[0]) {
		printf("Serious JTAG problem! Didn't find JTAG ID (0x%08x), found 0x%08x.\n",
				jtag_info->tap->expected_ids[0], did);
		printf("Exiting!!!\n");
		exit(2);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int mpc5xxx_jtagc_read(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t *value, uint32_t size)
{
	int res;

	res = mpc5xxx_jtag_access_jtagc(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, addr);
	if (res)
		return res;

	res = mpc5xxx_jtag_read_data(jtag_info, value, size);
	if (res)
		return res;

	return ERROR_OK;
}

int mpc5xxx_jtagc_write(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t value, uint32_t size)
{
	int res;

	res = mpc5xxx_jtag_access_jtagc(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, addr);
	if (res)
		return res;

	res = mpc5xxx_jtag_write_data(jtag_info, value, size);
	if (res)
		return res;

	return ERROR_OK;
}

int mpc5xxx_once_read(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t *value, uint32_t size)
{
	int res;

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, MPC5XXX_ONCE_READ | addr);
	if (res)
		return res;

	res = mpc5xxx_jtag_read_data(jtag_info, value, size);
	if (res)
		return res;

	return ERROR_OK;
}

int mpc5xxx_once_write(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t value, uint32_t size)
{
	int res;

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, addr);
	if (res)
		return res;

	res = mpc5xxx_jtag_write_data(jtag_info, value, size);
	if (res)
		return res;

	return ERROR_OK;
}

int mpc5xxx_once_osr_read(struct mpc5xxx_jtag *jtag_info, uint32_t *in)
{
#define OSR_SIZE 10
	struct scan_field field;
	uint8_t data_buf[4];
	uint8_t busy_buf[4];
	uint8_t in_buf[4];
	int busy, res;

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	do {
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));
		memset(in_buf, 0, sizeof(in_buf));

		buf_set_u32(data_buf, 0, OSR_SIZE, MPC5XXX_ONCE_READ | MPC5XXX_ONCE_NOREG);
		field.num_bits = OSR_SIZE;
		field.in_value = in_buf;
		field.out_value = data_buf;

		jtag_add_ir_scan(jtag_info->tap, &field, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: reading data  failed", __func__);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 0);
	} while (busy);

	*in = buf_get_u32(in_buf, 0, OSR_SIZE);

	return ERROR_OK;
}

int mpc5xxx_once_check_edm(struct mpc5xxx_jtag *jtag_info)
{
	int retval;
	uint32_t val;

	retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DBCR0, &val, 32);
	if (retval != ERROR_OK)
		return retval;
	printf("Value returned from DBCR0 was 0x%x\n", val);
	if ((val & MPC5XXX_ONCE_DBCR0_EDM) == 0) {
		printf("EDM bit not set, try to set it\n");

		retval = mpc5xxx_once_write(jtag_info, MPC5XXX_ONCE_DBCR0, MPC5XXX_ONCE_DBCR0_EDM, 32);
		if (retval != ERROR_OK)
			return retval;

		/* See if it was actually set */
		retval = mpc5xxx_once_read(jtag_info, MPC5XXX_ONCE_DBCR0, &val, 32);
		if (retval != ERROR_OK)
			return retval;

		if ((val & MPC5XXX_ONCE_DBCR0_EDM) == 0) {
			printf("Seems we are unable to set the EDM bit. Arse!\n");
			exit(1);
		} else {
			printf("IS OK NOW!\n");
		}
	}
	return ERROR_OK;
}

#define CPUSCR_SIZE 192
/* reads 6 * 32bits from CPUSCR */
int mpc5xxx_once_cpuscr_read(struct mpc5xxx_jtag *jtag_info, struct mpc5xxx_cpuscr *cpuscr)
{
	struct scan_field field;
	uint8_t data_buf[24];
	uint8_t busy_buf[24];
	uint8_t in_buf[24];
	int busy, res;

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, MPC5XXX_ONCE_READ | MPC5XXX_ONCE_CPUSCR);
	if (res)
		return res;

	do {
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));
		memset(in_buf, 0, sizeof(in_buf));

		field.num_bits = CPUSCR_SIZE;
		field.in_value = in_buf;
		field.out_value = NULL;

		jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: reading data  failed", __func__);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 0);
	} while (busy);

	cpuscr->wbbrl = buf_get_u32(in_buf, 0, 32);
	cpuscr->wbbrh = buf_get_u32(in_buf, 32, 32);
	cpuscr->msr = buf_get_u32(in_buf, 64, 32);
	cpuscr->pc = buf_get_u32(in_buf, 96, 32);
	cpuscr->ir = buf_get_u32(in_buf, 128, 32);
	cpuscr->ctl = buf_get_u32(in_buf, 160, 32);
	/* printf("CPU MSR=0x%08x\n", cpuscr->msr); */

	return ERROR_OK;
}

/* writes 6 * 32bits to CPUSCR */
int mpc5xxx_once_cpuscr_write(struct mpc5xxx_jtag *jtag_info, struct mpc5xxx_cpuscr *cpuscr)
{
	struct scan_field field;
	uint8_t data_buf[24];
	uint8_t busy_buf[24];
	int busy, res;

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	res = mpc5xxx_jtag_set_instr(jtag_info, MPC5XXX_ONCE_CPUSCR);
	if (res)
		return res;

	do {
		memset(data_buf, 0, sizeof(data_buf));
		memset(busy_buf, 0, sizeof(busy_buf));
		memcpy(data_buf, cpuscr, CPUSCR_SIZE / 8);

		field.num_bits = CPUSCR_SIZE;
		field.in_value = NULL;
		field.out_value = data_buf;

		jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

		if (jtag_execute_queue() != ERROR_OK) {
			LOG_ERROR("%s: reading data  failed", __func__);
			return ERROR_FAIL;
		}

		busy = buf_get_u32(busy_buf, 0, 0);
	} while (busy);

	return ERROR_OK;
}

int mpc5xxx_once_nexus_read(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t *value, uint32_t size)
{
	int res;
	uint32_t val, count;

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	/* Tell ONCE to access NEXUS.
	 * Note: The code already checks if this is the current instruction, so
	 * this won't waste JTAG clocks.
	 */
	res = mpc5xxx_jtag_set_instr(jtag_info, MPC5XXX_ONCE_NEXUS);
	if (res)
		return res;

	/* Set up RWA with the desired address
	 * Next is write */
	mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWA | MPC5XXX_ONCE_NEXUS_WRITE, 8);
	mpc5xxx_jtag_write_data(jtag_info, addr, 32);

	/* Now programme the RWCS reg to initiate the read
	 * Next is write */
	mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWCS | MPC5XXX_ONCE_NEXUS_WRITE, 8);
	if (size == 8) {
		val = MPC5XXX_ONCE_NEXUS_RWCS_READ1_8;
	} else if (size == 16) {
		val = MPC5XXX_ONCE_NEXUS_RWCS_READ1_16;
	} else {
		val = MPC5XXX_ONCE_NEXUS_RWCS_READ1_32;
		size = 32;
	}

	mpc5xxx_jtag_write_data(jtag_info, val, 32);

	/* Now poll RWCS until the read completed */
	val = 0;
	count = 0;
	while (((val & MPC5XXX_ONCE_NEXUS_RWCS_MASK) == 0) && (count < READ_MAXLOOPS)) {
		/* next is read */
		mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWCS, 8);
		mpc5xxx_jtag_read_data(jtag_info, &val, 32);
		count++;
	}
	if (((val & MPC5XXX_ONCE_NEXUS_RWCS_MASK) != MPC5XXX_ONCE_NEXUS_RWCS_READOK)
		|| (count >= READ_MAXLOOPS)) {
		LOG_ERROR("Failed to read memory!");
		return ERROR_FAIL;
	}

	/* Now read the actual data
	 * next is read */
	mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWD, 8);

	mpc5xxx_jtag_read_data(jtag_info, &val, size);

	/* Byte swap for endianism target is BE, but Nexus reads are
	 * supposed to be LE. Appears that something is converting
	 * in the middle?
	 * How do we know that host is LE ?
	 */
	if (size == 8) {
		*value = val;
	} else if (size == 16) {
		*value = ((val & 0x00ff) << 8) | ((val & 0xff00) >> 8);
	} else {
		*value = ((val & 0x000000ff) << 24)
				| ((val & 0x0000ff00) << 8)
				| ((val & 0x00ff0000) >> 8)
				| ((val & 0xff000000) >> 24);
	}
	return ERROR_OK;
}

int mpc5xxx_once_nexus_write(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t value, uint32_t size)
{
	int res;
	uint32_t val, count;

	/* Byte swap for endianism target is BE, but Nexus reads are
	 * supposed to be LE. Appears that something is converting
	 * in the middle?
	 * How do we know that host is LE ?
	 */
	if (size == 8) {
		/* no change */
	} else if (size == 16) {
		value = ((value & 0x00ff) << 8) | ((value & 0xff00) >> 8);
	} else {
		value = ((value & 0x000000ff) << 24)
				| ((value & 0x0000ff00) << 8)
				| ((value & 0x00ff0000) >> 8)
				| ((value & 0xff000000) >> 24);
	}

	res = mpc5xxx_jtag_access_once(jtag_info);
	if (res)
		return res;

	/* Tell ONCE to access NEXUS. */
	res = mpc5xxx_jtag_set_instr(jtag_info, MPC5XXX_ONCE_NEXUS);
	if (res)
		return res;

	/* Set up RWA with the desired address
	 * Next is write */
	res = mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWA | MPC5XXX_ONCE_NEXUS_WRITE, 8);
	if (res)
		return res;
	res = mpc5xxx_jtag_write_data(jtag_info, addr, 32);
	if (res)
		return res;

	/* Now programme the RWCS reg to prepare the write
	 * Next is write */
	res = mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWCS | MPC5XXX_ONCE_NEXUS_WRITE, 8);
	if (res)
		return res;
	if (size == 8) {
		val = MPC5XXX_ONCE_NEXUS_RWCS_WRITE1_8;
	} else if (size == 16) {
		val = MPC5XXX_ONCE_NEXUS_RWCS_WRITE1_16;
	} else {
		val = MPC5XXX_ONCE_NEXUS_RWCS_WRITE1_32;
		size = 32;
	}

	res = mpc5xxx_jtag_write_data(jtag_info, val, 32);
	if (res)
		return res;

	/* Now programme the RWD reg to initiate the write
	 * Next is write */
	res = mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWD | MPC5XXX_ONCE_NEXUS_WRITE, 8);
	if (res)
		return res;
	res = mpc5xxx_jtag_write_data(jtag_info, value, size);
	if (res)
		return res;

	/* Now poll RWCS until the read completed */
	val = 0;
	count = 0;
	while (((val & MPC5XXX_ONCE_NEXUS_RWCS_DVMASK) != 0) && (count < READ_MAXLOOPS)) {
		/* next is read */
		res = mpc5xxx_jtag_write_data(jtag_info, MPC5XXX_ONCE_NEXUS_RWCS, 8);
		if (res)
			return res;
		res = mpc5xxx_jtag_read_data(jtag_info, &val, 32);
		if (res)
			return res;
	}
	if (((val & MPC5XXX_ONCE_NEXUS_RWCS_MASK) != MPC5XXX_ONCE_NEXUS_RWCS_WRITEOK)
		|| (count >= READ_MAXLOOPS)) {
		LOG_ERROR("Failed to read!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}
