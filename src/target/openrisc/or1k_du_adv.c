/***************************************************************************
 *   Copyright (C) 2013 by Franck Jullien                                  *
 *   elec4fun@gmail.com                                                    *
 *                                                                         *
 *   Inspired from adv_jtag_bridge which is:                                *
 *   Copyright (C) 2008-2010 Nathan Yawn                                   *
 *   nyawn@opencores.net                                                   *
 *                                                                         *
 *   And the Mohor interface version of this file which is:                *
 *   Copyright (C) 2011 by Julius Baxter                                   *
 *   julius@opencores.org                                                  *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "or1k_tap.h"
#include "or1k.h"
#include "or1k_du.h"

#include <target/target.h>
#include <jtag/jtag.h>

/* This an option to the adv debug unit.
 * If this is defined, status bits will be skipped on burst
 * reads and writes to improve download speeds.
 * This option must match the RTL configured option.
 */
#define ADBG_USE_HISPEED		1

/* Definitions for the top-level debug unit.  This really just consists
 * of a single register, used to select the active debug module ("chain").
 */
#define DBG_MODULE_SELECT_REG_SIZE	2
#define DBG_MAX_MODULES			4

#define DC_WISHBONE			0
#define DC_CPU0				1
#define DC_CPU1				2
#define DC_JSP				3

/* CPU control register bits mask */
#define DBG_CPU_CR_STALL		0x01
#define DBG_CPU_CR_RESET		0x02

/* Polynomial for the CRC calculation
 * Yes, it's backwards.  Yes, this is on purpose.
 * The hardware is designed this way to save on logic and routing,
 * and it's really all the same to us here.
 */
#define ADBG_CRC_POLY			0xedb88320

/* These are for the internal registers in the Wishbone module
 * The first is the length of the index register,
 * the indexes of the various registers are defined after that.
 */
#define DBG_WB_REG_SEL_LEN		1
#define DBG_WB_REG_ERROR		0

/* Opcode definitions for the Wishbone module. */
#define DBG_WB_OPCODE_LEN		4
#define DBG_WB_CMD_NOP			0x0
#define DBG_WB_CMD_BWRITE8		0x1
#define DBG_WB_CMD_BWRITE16		0x2
#define DBG_WB_CMD_BWRITE32		0x3
#define DBG_WB_CMD_BREAD8		0x5
#define DBG_WB_CMD_BREAD16		0x6
#define DBG_WB_CMD_BREAD32		0x7
#define DBG_WB_CMD_IREG_WR		0x9
#define DBG_WB_CMD_IREG_SEL		0xd

/* Internal register definitions for the CPU0 module. */
#define DBG_CPU0_REG_SEL_LEN		1
#define DBG_CPU0_REG_STATUS		0

/* Opcode definitions for the first CPU module. */
#define DBG_CPU0_OPCODE_LEN		4
#define DBG_CPU0_CMD_NOP		0x0
#define DBG_CPU0_CMD_BWRITE32		0x3
#define DBG_CPU0_CMD_BREAD32		0x7
#define DBG_CPU0_CMD_IREG_WR		0x9
#define DBG_CPU0_CMD_IREG_SEL		0xd

/* Internal register definitions for the CPU1 module. */
#define DBG_CPU1_REG_SEL_LEN		1
#define DBG_CPU1_REG_STATUS		0

/* Opcode definitions for the second CPU module. */
#define DBG_CPU1_OPCODE_LEN		4
#define DBG_CPU1_CMD_NOP		0x0
#define DBG_CPU1_CMD_BWRITE32		0x3
#define DBG_CPU1_CMD_BREAD32		0x7
#define DBG_CPU1_CMD_IREG_WR		0x9
#define DBG_CPU1_CMD_IREG_SEL		0xd

#define MAX_READ_STATUS_WAIT		10
#define MAX_READ_BUSY_RETRY		2
#define MAX_READ_CRC_RETRY		2
#define MAX_WRITE_CRC_RETRY		2
#define BURST_READ_READY		1
#define MAX_BUS_ERRORS			2

#define STATUS_BYTES			1
#define CRC_LEN				4

static struct or1k_du or1k_du_adv;

static const char * const chain_name[] = {"WISHBONE", "CPU0", "CPU1", "JSP"};

static uint32_t adbg_compute_crc(uint32_t crc, uint32_t data_in,
				 int length_bits)
{
	for (int i = 0; i < length_bits; i++) {

		uint32_t d, c;

		d = ((data_in >> i) & 0x1) ? 0xffffffff : 0;
		c = (crc & 0x1) ? 0xffffffff : 0;
		crc = crc >> 1;
		crc = crc ^ ((d ^ c) & ADBG_CRC_POLY);
	}

	return crc;
}

static int find_status_bit(void *_buf, int len)
{
	int i = 0;
	int count = 0;
	int ret = -1;
	uint8_t *buf = _buf;

	while (!(buf[i] & (1 << count++)) && (i < len)) {
		if (count == 8) {
			count = 0;
			i++;
		}
	}

	if (i < len)
		ret = (i * 8) + count;

	return ret;
}

static int or1k_adv_jtag_init(struct or1k_jtag *jtag_info)
{
	struct or1k_tap_ip *tap_ip = jtag_info->tap_ip;

	int retval = tap_ip->init(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("TAP initialization failed");
		return retval;
	}

	/* TAP should now be configured to communicate with
	 * debug interface
	 */
	jtag_info->or1k_jtag_inited = 1;

	/* TAP reset - not sure what state debug module chain is in now */
	jtag_info->or1k_jtag_module_selected = -1;

	jtag_info->current_reg_idx = malloc(DBG_MAX_MODULES * sizeof(unsigned long));
	memset(jtag_info->current_reg_idx, 0, DBG_MAX_MODULES * sizeof(unsigned long));

	if (or1k_du_adv.options & ADBG_USE_HISPEED)
		LOG_INFO("adv debug unit is configured with option ADBG_USE_HISPEED");

	LOG_DEBUG("Init done");

	return ERROR_OK;

}

/* Selects one of the modules in the debug unit
 * (e.g. wishbone unit, CPU0, etc.)
 */
static int adbg_select_module(struct or1k_jtag *jtag_info, int chain)
{

	if (jtag_info->or1k_jtag_module_selected == chain)
		return ERROR_OK;

	/* MSB of the data out must be set to 1, indicating a module
	 * select command
	 */
	uint8_t data = chain | (1 << DBG_MODULE_SELECT_REG_SIZE);

	LOG_DEBUG("Select module: %s", chain_name[chain]);

	struct scan_field field;

	field.num_bits = (DBG_MODULE_SELECT_REG_SIZE + 1);
	field.out_value = &data;
	field.in_value = NULL;
	jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	jtag_info->or1k_jtag_module_selected = chain;

	return ERROR_OK;
}

/* Set the index of the desired register in the currently selected module
 * 1 bit module select command
 * 4 bits opcode
 * n bits index
 */
static int adbg_select_ctrl_reg(struct or1k_jtag *jtag_info,
				unsigned long regidx)
{
	int index_len;
	uint32_t opcode;
	uint32_t opcode_len;

	/* If this reg is already selected, don't do a JTAG transaction */
	if (jtag_info->current_reg_idx[jtag_info->or1k_jtag_module_selected] == regidx)
		return ERROR_OK;

	switch (jtag_info->or1k_jtag_module_selected) {
	case DC_WISHBONE:
		index_len = DBG_WB_REG_SEL_LEN;
		opcode = DBG_WB_CMD_IREG_SEL;
		opcode_len = DBG_WB_OPCODE_LEN;
		break;
	case DC_CPU0:
		index_len = DBG_CPU0_REG_SEL_LEN;
		opcode = DBG_CPU0_CMD_IREG_SEL;
		opcode_len = DBG_CPU0_OPCODE_LEN;
		break;
	case DC_CPU1:
		index_len = DBG_CPU1_REG_SEL_LEN;
		opcode = DBG_CPU1_CMD_IREG_SEL;
		opcode_len = DBG_CPU1_OPCODE_LEN;
		break;
	default:
		LOG_ERROR("Illegal debug chain selected (%i) while selecting control register",
			  jtag_info->or1k_jtag_module_selected);
		return ERROR_FAIL;
	}

	/* MSB must be 0 to access modules */
	uint32_t data = (opcode & ~(1 << opcode_len)) << index_len;
	data |= regidx;

	struct scan_field field;

	field.num_bits = (opcode_len + 1) + index_len;
	field.out_value = (uint8_t *)&data;
	field.in_value = NULL;
	jtag_add_dr_scan(jtag_info->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	jtag_info->current_reg_idx[jtag_info->or1k_jtag_module_selected] = regidx;

	return ERROR_OK;
}

/* Sends out a generic command to the selected debug unit module, LSB first.  Fields are:
 * MSB: 1-bit module command
 * 4-bit opcode
 * m-bit register index
 * n-bit data (LSB)
 * Note that in the data array, the LSB of data[0] will be sent first,
 * (and become the LSB of the command)
 * up through the MSB of data[0], then the LSB of data[1], etc.
 */
static int adbg_ctrl_write(struct or1k_jtag *jtag_info, unsigned long regidx,
			   uint32_t *cmd_data, int length_bits)
{
	int index_len;
	uint32_t opcode;
	uint32_t opcode_len;

	LOG_DEBUG("Write control register %ld: 0x%08x", regidx, cmd_data[0]);

	int retval = adbg_select_ctrl_reg(jtag_info, regidx);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling adbg_select_ctrl_reg");
		return retval;
	}

	switch (jtag_info->or1k_jtag_module_selected) {
	case DC_WISHBONE:
		index_len = DBG_WB_REG_SEL_LEN;
		opcode = DBG_WB_CMD_IREG_WR;
		opcode_len = DBG_WB_OPCODE_LEN;
		break;
	case DC_CPU0:
		index_len = DBG_CPU0_REG_SEL_LEN;
		opcode = DBG_CPU0_CMD_IREG_WR;
		opcode_len = DBG_CPU0_OPCODE_LEN;
		break;
	case DC_CPU1:
		index_len = DBG_CPU1_REG_SEL_LEN;
		opcode = DBG_CPU1_CMD_IREG_WR;
		opcode_len = DBG_CPU1_OPCODE_LEN;
		break;
	default:
		LOG_ERROR("Illegal debug chain selected (%i) while doing control write",
			  jtag_info->or1k_jtag_module_selected);
		return ERROR_FAIL;
	}

	int num_32bit_fields = length_bits / 32;
	int spare_bits = length_bits % 32;
	int nb_fields = num_32bit_fields + !!spare_bits + 1;

	struct scan_field *field = malloc(nb_fields * sizeof(struct scan_field));

	/* MSB must be 0 to access modules */
	uint32_t data = (opcode & ~(1 << opcode_len)) << index_len;
	data |= regidx;

	int i;

	for (i = 0 ; i < num_32bit_fields; i++) {
		field[i].num_bits = 32;
		field[i].out_value = (uint8_t *)&cmd_data[i * 4];
		field[i].in_value = NULL;
	}

	if (spare_bits) {
		field[i].num_bits = spare_bits;
		field[i].out_value = (uint8_t *)&cmd_data[i * 4];
		field[i].in_value = NULL;
	}

	field[i + 1].num_bits = (opcode_len + 1) + index_len;
	field[i + 1].out_value = (uint8_t *)&data;
	field[i + 1].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, nb_fields, field, TAP_IDLE);

	retval = jtag_execute_queue();

	free(field);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/* Reads control register (internal to the debug unit)
 * Currently only 1 register in the CPU module, so no register select
 */
static int adbg_ctrl_read(struct or1k_jtag *jtag_info, uint32_t regidx,
			  uint32_t *data, int length_bits)
{
	int opcode_len;
	uint32_t opcode;

	/*LOG_DEBUG("Read control register %ld", regidx);*/

	int retval = adbg_select_ctrl_reg(jtag_info, regidx);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling adbg_select_ctrl_reg");
		return retval;
	}

	/* There is no 'read' command, We write a NOP to read */
	switch (jtag_info->or1k_jtag_module_selected) {
	case DC_WISHBONE:
		opcode = DBG_WB_CMD_NOP;
		opcode_len = DBG_WB_OPCODE_LEN;
		break;
	case DC_CPU0:
		opcode = DBG_CPU0_CMD_NOP;
		opcode_len = DBG_CPU0_OPCODE_LEN;
		break;
	case DC_CPU1:
		opcode = DBG_CPU1_CMD_NOP;
		opcode_len = DBG_CPU1_OPCODE_LEN;
		break;
	default:
		LOG_ERROR("Illegal debug chain selected (%i) while doing control read",
			  jtag_info->or1k_jtag_module_selected);
		 return ERROR_FAIL;
	}

	/* Zero MSB = op for module, not top-level debug unit */
	uint32_t outdata = opcode & ~(0x1 << opcode_len);

	int num_32bit_fields = length_bits / 32;
	int spare_bits = length_bits % 32;
	int nb_fields = num_32bit_fields + !!spare_bits + 1;

	struct scan_field *field = malloc(nb_fields * sizeof(struct scan_field));

	int i;

	for (i = 0 ; i < num_32bit_fields; i++) {
		field[i].num_bits = 32;
		field[i].out_value = NULL;
		field[i].in_value = (uint8_t *)&data[i * 4];
	}

	if (spare_bits) {
		field[i].num_bits = spare_bits;
		field[i].out_value = NULL;
		field[i].in_value = (uint8_t *)&data[i * 4];
	}

	field[i + 1].num_bits = opcode_len + 1;
	field[i + 1].out_value = (uint8_t *)&outdata;
	field[i + 1].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, nb_fields, field, TAP_IDLE);

	retval = jtag_execute_queue();

	free(field);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/* sends out a burst command to the selected module in the debug unit (MSB to LSB):
 * 1-bit module command
 * 4-bit opcode
 * 32-bit address
 * 16-bit length (of the burst, in words)
 */
static int adbg_burst_command(struct or1k_jtag *jtag_info, uint32_t opcode,
			      uint32_t address, int length_words)
{
	struct scan_field field[2];
	uint32_t data[2];

	/* Set up the data */
	data[0] = length_words | (address << 16);
	/* MSB must be 0 to access modules */
	data[1] = ((address >> 16) | ((opcode & 0xf) << 16)) & ~(0x1 << 20);

	field[0].num_bits = 32;
	field[0].out_value = (uint8_t *)&data[0];
	field[0].in_value = NULL;

	field[1].num_bits = 21;
	field[1].out_value = (uint8_t *)&data[1];
	field[1].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 2, field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int adbg_wb_burst_read(struct or1k_jtag *jtag_info, int word_size_bytes,
			      int word_count, uint32_t start_address, void *data)
{
	int retry_full_crc = 0;
	int retry_full_busy = 0;
	int retval;
	uint8_t opcode;

	LOG_DEBUG("Doing burst read, word size %d, word count %d, start address 0x%08x",
		  word_size_bytes, word_count, start_address);

	if (word_count <= 0) {
		LOG_ERROR("Ignoring illegal read burst length (%d)", word_count);
		return ERROR_FAIL;
	}

	/* Select the appropriate opcode */
	switch (jtag_info->or1k_jtag_module_selected) {
	case DC_WISHBONE:
		if (word_size_bytes == 1)
			opcode = DBG_WB_CMD_BREAD8;
		else if (word_size_bytes == 2)
			opcode = DBG_WB_CMD_BREAD16;
		else if (word_size_bytes == 4)
			opcode = DBG_WB_CMD_BREAD32;
		else {
			LOG_WARNING("Tried burst read with invalid word size (%d),"
				  "defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_WB_CMD_BREAD32;
		}
		break;
	case DC_CPU0:
		if (word_size_bytes == 4)
			opcode = DBG_CPU0_CMD_BREAD32;
		else {
			LOG_WARNING("Tried burst read with invalid word size (%d),"
				  "defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_CPU0_CMD_BREAD32;
		}
		break;
	case DC_CPU1:
		if (word_size_bytes == 4)
			opcode = DBG_CPU1_CMD_BREAD32;
		else {
			LOG_WARNING("Tried burst read with invalid word size (%d),"
				  "defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_CPU0_CMD_BREAD32;
		}
		break;
	default:
		LOG_ERROR("Illegal debug chain selected (%i) while doing burst read",
			  jtag_info->or1k_jtag_module_selected);
		return ERROR_FAIL;
	}

	/* Calculate transfer params */
	int total_size_bytes = (word_count * word_size_bytes) + CRC_LEN + STATUS_BYTES;
	int num_32bit_fields = total_size_bytes / 4;
	int spare_bytes = total_size_bytes % 4;

	/* Allocate correct number of scan fields */
	int nb_fields = num_32bit_fields + !!spare_bytes;
	struct scan_field *field = malloc(nb_fields * sizeof(struct scan_field));

	uint8_t *in_buffer = malloc(total_size_bytes);

retry_read_full:

	/* Send the BURST READ command, returns TAP to idle state */
	retval = adbg_burst_command(jtag_info, opcode, start_address, word_count);
	if (retval != ERROR_OK)
		goto out;

	int i;

	for (i = 0; i < num_32bit_fields; i++) {
		field[i].num_bits = 32;
		field[i].out_value = NULL;
		field[i].in_value = &in_buffer[i * 4];
	}

	if (spare_bytes) {
		field[i].num_bits = spare_bytes * 8;
		field[i].out_value = NULL;
		field[i].in_value = &in_buffer[i * 4];
	}

	jtag_add_dr_scan(jtag_info->tap, nb_fields, field, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		goto out;

	/* Look for the start bit in the first (STATUS_BYTES * 8) bits */
	int shift = find_status_bit(in_buffer, STATUS_BYTES);

	/* We expect the status bit to be in the first byte */
	if (shift < 0) {
		if (retry_full_busy++ < MAX_READ_BUSY_RETRY) {
			LOG_WARNING("Burst read timed out");
			goto retry_read_full;
		} else {
			LOG_ERROR("Burst read failed");
			retval = ERROR_FAIL;
			goto out;
		}
	}

	buffer_shr(in_buffer, (word_count * word_size_bytes) + CRC_LEN + STATUS_BYTES, shift);

	uint32_t crc_read;
	memcpy(data, in_buffer, word_count * word_size_bytes);
	memcpy(&crc_read, &in_buffer[word_count * word_size_bytes], 4);

	uint32_t crc_calc = 0xffffffff;
	for (i = 0; i < (word_count * word_size_bytes); i++)
		crc_calc = adbg_compute_crc(crc_calc, ((uint8_t *)data)[i], 8);

	if (crc_calc != crc_read) {
		LOG_WARNING("CRC ERROR! Computed 0x%08x, read CRC 0x%08x", crc_calc, crc_read);
		if (retry_full_crc++ < MAX_READ_CRC_RETRY)
			goto retry_read_full;
		else {
			LOG_ERROR("Burst read failed");
			retval = ERROR_FAIL;
			goto out;
		}
	} else
		LOG_DEBUG("CRC OK!");

	/* Now, read the error register, and retry/recompute as necessary */
	if (jtag_info->or1k_jtag_module_selected == DC_WISHBONE &&
	    !(or1k_du_adv.options & ADBG_USE_HISPEED)) {

		uint32_t err_data[2] = {0, 0};
		uint32_t addr;
		int bus_error_retries = 0;

		/* First, just get 1 bit...read address only if necessary */
		retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 1);
		if (retval != ERROR_OK)
			goto out;

		/* Then we have a problem */
		if (err_data[0] & 0x1) {

			retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 33);
			if (retval != ERROR_OK)
				goto out;

			addr = (err_data[0] >> 1) | (err_data[1] << 31);
			LOG_WARNING("WB bus error during burst read, address 0x%08x, retrying!", addr);

			bus_error_retries++;
			if (bus_error_retries > MAX_BUS_ERRORS) {
				LOG_ERROR("Max WB bus errors reached during burst read");
				retval = ERROR_FAIL;
				goto out;
			}

			/* Don't call retry_do(), a JTAG reset won't help a WB bus error */
			/* Write 1 bit, to reset the error register */
			err_data[0] = 1;
			retval = adbg_ctrl_write(jtag_info, DBG_WB_REG_ERROR, err_data, 1);
			if (retval != ERROR_OK)
				goto out;

			goto retry_read_full;
		}
	}

out:
	free(field);
	free(in_buffer);

	return retval;
}

/* Set up and execute a burst write to a contiguous set of addresses */
static int adbg_wb_burst_write(struct or1k_jtag *jtag_info, const void *data, int word_size_bytes,
			int word_count, unsigned long start_address)
{
	int retry_full_crc = 0;
	int retval;
	uint8_t opcode;

	if (word_count <= 0) {
		LOG_DEBUG("Ignoring illegal burst write size (%d)", word_count);
		return ERROR_FAIL;
	}

	LOG_DEBUG("Doing burst write, word size %d, word count %d, start address 0x%08lx",
	word_size_bytes, word_count, start_address);

	int word_size_bits = word_size_bytes * 8;

	/* Select the appropriate opcode */
	switch (jtag_info->or1k_jtag_module_selected) {
	case DC_WISHBONE:
		if (word_size_bytes == 1)
			opcode = DBG_WB_CMD_BWRITE8;
		else if (word_size_bytes == 2)
			opcode = DBG_WB_CMD_BWRITE16;
		else if (word_size_bytes == 4)
			opcode = DBG_WB_CMD_BWRITE32;
		else {
			LOG_DEBUG("Tried WB burst write with invalid word size (%d),"
				  "defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_WB_CMD_BWRITE32;
		}
		break;
	case DC_CPU0:
		if (word_size_bytes == 4)
			opcode = DBG_CPU0_CMD_BWRITE32;
		else {
			LOG_DEBUG("Tried CPU0 burst write with invalid word size (%d),"
				  "defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_CPU0_CMD_BWRITE32;
		}
		break;
	case DC_CPU1:
		if (word_size_bytes == 4)
			opcode = DBG_CPU1_CMD_BWRITE32;
		else {
			LOG_DEBUG("Tried CPU1 burst write with invalid word size (%d),"
				  "defaulting to 4-byte words", word_size_bytes);
			opcode = DBG_CPU0_CMD_BWRITE32;
		}
		break;
	default:
		LOG_ERROR("Illegal debug chain selected (%i) while doing burst write",
			  jtag_info->or1k_jtag_module_selected);
		return ERROR_FAIL;
	}

	/* Calculate transfer params */
	int total_size_bytes = (word_count * word_size_bytes) + 4;
	int num_32bit_fields = total_size_bytes / 4;
	int spare_bytes = total_size_bytes % 4;

	/* Allocate correct number of scan fields */
	int nb_fields = num_32bit_fields + !!spare_bytes + 1;
	struct scan_field *field = malloc(nb_fields * sizeof(struct scan_field));

	uint8_t *out_buffer = malloc(total_size_bytes);

retry_full_write:

	/* Send the BURST WRITE command, returns TAP to idle state */
	retval = adbg_burst_command(jtag_info, opcode, start_address, word_count);
	if (retval != ERROR_OK)
		goto out;

	/* Write a start bit so it knows when to start counting */
	uint8_t value = 1;
	field[0].num_bits = 1;
	field[0].out_value = &value;
	field[0].in_value = NULL;

	int i;
	uint32_t datawords;
	uint32_t crc_calc = 0xffffffff;
	for (i = 0; i < word_count; i++) {
		if (word_size_bytes == 4)
			datawords = ((const uint32_t *)data)[i];
		else if (word_size_bytes == 2)
			datawords = ((const uint16_t *)data)[i];
		else
			datawords = ((const uint8_t *)data)[i];

		crc_calc = adbg_compute_crc(crc_calc, datawords, word_size_bits);
	}

	memcpy(out_buffer, data, (word_count * word_size_bytes));
	memcpy(&out_buffer[(word_count * word_size_bytes)], &crc_calc, 4);

	for (i = 0; i < num_32bit_fields; i++) {
		field[i + 1].num_bits = 32;
		field[i + 1].out_value = &out_buffer[i * 4];
		field[i + 1].in_value = NULL;
	}

	if (spare_bytes) {
		field[i + 1].num_bits = spare_bytes * 8;
		field[i + 1].out_value = &out_buffer[i * 4];
		field[i + 1].in_value = NULL;
	}

	jtag_add_dr_scan(jtag_info->tap, nb_fields, field, TAP_DRSHIFT);

	/* Read the 'CRC match' bit, and go to idle */
	field[0].num_bits = 1;
	field[0].out_value = NULL;
	field[0].in_value = &value;
	jtag_add_dr_scan(jtag_info->tap, 1, &field[0], TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		goto out;

	if (!value) {
		LOG_WARNING("CRC ERROR! match bit after write is %i (computed CRC 0x%08x)", value, crc_calc);
		if (retry_full_crc++ < MAX_WRITE_CRC_RETRY)
			goto retry_full_write;
		else {
			retval = ERROR_FAIL;
			goto out;
		}
	} else
		LOG_DEBUG("CRC OK!\n");

	/* Now, read the error register, and retry/recompute as necessary */
	if (jtag_info->or1k_jtag_module_selected == DC_WISHBONE &&
	    !(or1k_du_adv.options & ADBG_USE_HISPEED)) {
		uint32_t addr;
		int bus_error_retries = 0;
		uint32_t err_data[2] = {0, 0};

		/* First, just get 1 bit...read address only if necessary */
		retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 1);
		if (retval != ERROR_OK)
			goto out;

		/* Then we have a problem */
		if (err_data[0] & 0x1) {

			retval = adbg_ctrl_read(jtag_info, DBG_WB_REG_ERROR, err_data, 33);
			if (retval != ERROR_OK)
				goto out;

			addr = (err_data[0] >> 1) | (err_data[1] << 31);
			LOG_WARNING("WB bus error during burst write, address 0x%08x, retrying!", addr);

			bus_error_retries++;
			if (bus_error_retries > MAX_BUS_ERRORS) {
				LOG_ERROR("Max WB bus errors reached during burst read");
				retval = ERROR_FAIL;
				goto out;
			}

			/* Don't call retry_do(), a JTAG reset won't help a WB bus error */
			/* Write 1 bit, to reset the error register */
			err_data[0] = 1;
			retval = adbg_ctrl_write(jtag_info, DBG_WB_REG_ERROR, err_data, 1);
			if (retval != ERROR_OK)
				goto out;

			goto retry_full_write;
		}
	}

out:
	free(out_buffer);
	free(field);

	return ERROR_OK;
}

/* Currently hard set in functions to 32-bits */
static int or1k_adv_jtag_read_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, int count, uint32_t *value)
{
	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_read(jtag_info, 4, count, addr, value);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int or1k_adv_jtag_write_cpu(struct or1k_jtag *jtag_info,
		uint32_t addr, int count, const uint32_t *value)
{
	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_write(jtag_info, value, 4, count, addr);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int or1k_adv_cpu_stall(struct or1k_jtag *jtag_info, int action)
{
	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	uint32_t cpu_cr;
	retval = adbg_ctrl_read(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);
	if (retval != ERROR_OK)
		return retval;

	if (action == CPU_STALL)
		cpu_cr |= DBG_CPU_CR_STALL;
	else
		cpu_cr &= ~DBG_CPU_CR_STALL;

	retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_ctrl_write(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int or1k_adv_is_cpu_running(struct or1k_jtag *jtag_info, int *running)
{
	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	uint32_t cpu_cr = 0;
	retval = adbg_ctrl_read(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);
	if (retval != ERROR_OK)
		return retval;

	if (cpu_cr & DBG_CPU_CR_STALL)
		*running = 0;
	else
		*running = 1;

	return ERROR_OK;
}

static int or1k_adv_cpu_reset(struct or1k_jtag *jtag_info, int action)
{
	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	uint32_t cpu_cr;
	retval = adbg_ctrl_read(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);
	if (retval != ERROR_OK)
		return retval;

	if (action == CPU_RESET)
		cpu_cr |= DBG_CPU_CR_RESET;
	else
		cpu_cr &= ~DBG_CPU_CR_RESET;

	retval = adbg_select_module(jtag_info, DC_CPU0);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_ctrl_write(jtag_info, DBG_CPU0_REG_STATUS, &cpu_cr, 2);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int or1k_adv_jtag_read_memory32(struct or1k_jtag *jtag_info,
			    uint32_t addr, int count, uint32_t *buffer)
{
	LOG_DEBUG("Reading WB32 at 0x%08X", addr);

	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_WISHBONE);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_read(jtag_info, 4, count, addr, buffer);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 0 ; i < count; i++)
		h_u32_to_be((uint8_t *) &buffer[i], buffer[i]);

	return ERROR_OK;

}

static int or1k_adv_jtag_read_memory16(struct or1k_jtag *jtag_info,
			    uint32_t addr, int count, uint16_t *buffer)
{
	LOG_DEBUG("Reading WB16 at 0x%08x", addr);

	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_WISHBONE);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_read(jtag_info, 2, count, addr, buffer);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int or1k_adv_jtag_read_memory8(struct or1k_jtag *jtag_info,
			   uint32_t addr, int count, uint8_t *buffer)
{
	LOG_DEBUG("Reading WB8 at 0x%08x", addr);

	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_WISHBONE);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_read(jtag_info, 1, count, addr, buffer);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int or1k_adv_jtag_write_memory32(struct or1k_jtag *jtag_info,
			     uint32_t addr, int count, const uint32_t *buffer)
{
	LOG_DEBUG("Writing WB32 at 0x%08x", addr);

	for (int i = 0 ; i < count; i++)
		h_u32_to_be((uint8_t *) &buffer[i], buffer[i]);

	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_WISHBONE);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_write(jtag_info, buffer, 4, count, addr);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;

}

static int or1k_adv_jtag_write_memory16(struct or1k_jtag *jtag_info,
			     uint32_t addr, int count, const uint16_t *buffer)
{
	LOG_DEBUG("Writing WB16 at 0x%08x", addr);

	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_WISHBONE);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_write(jtag_info, buffer, 2, count, addr);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int or1k_adv_jtag_write_memory8(struct or1k_jtag *jtag_info,
			    uint32_t addr, int count, const uint8_t *buffer)
{
	LOG_DEBUG("Writing WB8 at 0x%08x", addr);

	if (!jtag_info->or1k_jtag_inited)
		or1k_adv_jtag_init(jtag_info);

	int retval = adbg_select_module(jtag_info, DC_WISHBONE);
	if (retval != ERROR_OK)
		return retval;

	retval = adbg_wb_burst_write(jtag_info, buffer, 1, count, addr);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static struct or1k_du or1k_du_adv = {
	.name                     = "adv",
	.options                  = ADBG_USE_HISPEED,
	.or1k_jtag_init           = or1k_adv_jtag_init,

	.or1k_is_cpu_running      = or1k_adv_is_cpu_running,
	.or1k_cpu_stall           = or1k_adv_cpu_stall,
	.or1k_cpu_reset           = or1k_adv_cpu_reset,

	.or1k_jtag_read_cpu       = or1k_adv_jtag_read_cpu,
	.or1k_jtag_write_cpu      = or1k_adv_jtag_write_cpu,

	.or1k_jtag_read_memory32  = or1k_adv_jtag_read_memory32,
	.or1k_jtag_read_memory16  = or1k_adv_jtag_read_memory16,
	.or1k_jtag_read_memory8   = or1k_adv_jtag_read_memory8,

	.or1k_jtag_write_memory32 = or1k_adv_jtag_write_memory32,
	.or1k_jtag_write_memory16 = or1k_adv_jtag_write_memory16,
	.or1k_jtag_write_memory8  = or1k_adv_jtag_write_memory8,
};

int or1k_du_adv_register(void)
{
	list_add_tail(&or1k_du_adv.list, &du_list);
	return 0;
}
