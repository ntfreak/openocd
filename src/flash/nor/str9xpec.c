/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#include "imp.h"
#include <target/arm7_9_common.h>

/* ISC commands */

#define ISC_IDCODE				0xFE
#define ISC_MFG_READ			0x4C
#define ISC_CONFIGURATION		0x07
#define ISC_ENABLE				0x0C
#define ISC_DISABLE				0x0F
#define ISC_NOOP				0x10
#define ISC_ADDRESS_SHIFT		0x11
#define ISC_CLR_STATUS			0x13
#define ISC_PROGRAM				0x20
#define ISC_PROGRAM_SECURITY	0x22
#define ISC_PROGRAM_UC			0x23
#define ISC_ERASE				0x30
#define ISC_READ				0x50
#define ISC_BLANK_CHECK			0x60

/* ISC_DEFAULT bit definitions */

#define ISC_STATUS_SECURITY		0x40
#define ISC_STATUS_INT_ERROR	0x30
#define ISC_STATUS_MODE			0x08
#define ISC_STATUS_BUSY			0x04
#define ISC_STATUS_ERROR		0x03

/* Option bytes definitions */

#define STR9XPEC_OPT_CSMAPBIT		48
#define STR9XPEC_OPT_LVDTHRESBIT	49
#define STR9XPEC_OPT_LVDSELBIT		50
#define STR9XPEC_OPT_LVDWARNBIT		51
#define STR9XPEC_OPT_OTPBIT			63

enum str9xpec_status_codes {
	STR9XPEC_INVALID_COMMAND = 1,
	STR9XPEC_ISC_SUCCESS = 2,
	STR9XPEC_ISC_DISABLED = 3,
	STR9XPEC_ISC_INTFAIL = 32,
};

struct str9xpec_flash_controller {
	struct jtag_tap *tap;
	uint32_t *sector_bits;
	int chain_pos;
	int isc_enable;
	uint8_t options[8];
};

static int str9xpec_erase_area(struct flash_bank *bank, unsigned int first,
		unsigned int last);
static int str9xpec_set_address(struct flash_bank *bank, uint8_t sector);
static int str9xpec_write_options(struct flash_bank *bank);

static int str9xpec_set_instr(struct jtag_tap *tap, uint32_t new_instr, tap_state_t end_state)
{
	if (!tap)
		return ERROR_TARGET_INVALID;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr) {
		struct scan_field field;

		field.num_bits = tap->ir_length;
		void *t = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);
		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, end_state);

		free(t);
	}

	return ERROR_OK;
}

static uint8_t str9xpec_isc_status(struct jtag_tap *tap)
{
	struct scan_field field;
	uint8_t status;

	if (str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE) != ERROR_OK)
		return ISC_STATUS_ERROR;

	field.num_bits = 8;
	field.out_value = NULL;
	field.in_value = &status;


	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_execute_queue();

	LOG_DEBUG("status: 0x%2.2x", status);

	if (status & ISC_STATUS_SECURITY)
		LOG_INFO("Device Security Bit Set");

	return status;
}

static int str9xpec_isc_enable(struct flash_bank *bank)
{
	uint8_t status;
	struct jtag_tap *tap;
	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (str9xpec_info->isc_enable)
		return ERROR_OK;

	/* enter isc mode */
	if (str9xpec_set_instr(tap, ISC_ENABLE, TAP_IDLE) != ERROR_OK)
		return ERROR_TARGET_INVALID;

	/* check ISC status */
	status = str9xpec_isc_status(tap);
	if (status & ISC_STATUS_MODE) {
		/* we have entered isc mode */
		str9xpec_info->isc_enable = 1;
		LOG_DEBUG("ISC_MODE Enabled");
	}

	return ERROR_OK;
}

static int str9xpec_isc_disable(struct flash_bank *bank)
{
	uint8_t status;
	struct jtag_tap *tap;
	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable)
		return ERROR_OK;

	if (str9xpec_set_instr(tap, ISC_DISABLE, TAP_IDLE) != ERROR_OK)
		return ERROR_TARGET_INVALID;

	/* delay to handle aborts */
	jtag_add_sleep(50);

	/* check ISC status */
	status = str9xpec_isc_status(tap);
	if (!(status & ISC_STATUS_MODE)) {
		/* we have left isc mode */
		str9xpec_info->isc_enable = 0;
		LOG_DEBUG("ISC_MODE Disabled");
	}

	return ERROR_OK;
}

static int str9xpec_read_config(struct flash_bank *bank)
{
	struct scan_field field;
	uint8_t status;
	struct jtag_tap *tap;

	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	LOG_DEBUG("ISC_CONFIGURATION");

	/* execute ISC_CONFIGURATION command */
	str9xpec_set_instr(tap, ISC_CONFIGURATION, TAP_IRPAUSE);

	field.num_bits = 64;
	field.out_value = NULL;
	field.in_value = str9xpec_info->options;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_execute_queue();

	status = str9xpec_isc_status(tap);

	return status;
}

static int str9xpec_build_block_list(struct flash_bank *bank)
{
	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	int i;
	unsigned int num_sectors;
	int b0_sectors = 0, b1_sectors = 0;
	uint32_t offset = 0;
	int b1_size = 0x2000;

	switch (bank->size) {
		case (256 * 1024):
			b0_sectors = 4;
			break;
		case (512 * 1024):
			b0_sectors = 8;
			break;
		case (1024 * 1024):
			b0_sectors = 16;
			break;
		case (2048 * 1024):
			b0_sectors = 32;
			break;
		case (128 * 1024):
			b1_size = 0x4000;
			b1_sectors = 8;
			break;
		case (32 * 1024):
			b1_sectors = 4;
			break;
		default:
			LOG_ERROR("BUG: unknown bank->size encountered");
			exit(-1);
	}

	num_sectors = b0_sectors + b1_sectors;

	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	str9xpec_info->sector_bits = malloc(sizeof(uint32_t) * num_sectors);

	num_sectors = 0;

	for (i = 0; i < b0_sectors; i++) {
		bank->sectors[num_sectors].offset = offset;
		bank->sectors[num_sectors].size = 0x10000;
		offset += bank->sectors[i].size;
		bank->sectors[num_sectors].is_erased = -1;
		bank->sectors[num_sectors].is_protected = 1;
		str9xpec_info->sector_bits[num_sectors++] = i;
	}

	for (i = 0; i < b1_sectors; i++) {
		bank->sectors[num_sectors].offset = offset;
		bank->sectors[num_sectors].size = b1_size;
		offset += bank->sectors[i].size;
		bank->sectors[num_sectors].is_erased = -1;
		bank->sectors[num_sectors].is_protected = 1;
		str9xpec_info->sector_bits[num_sectors++] = i + 32;
	}

	return ERROR_OK;
}

/* flash bank str9x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(str9xpec_flash_bank_command)
{
	struct str9xpec_flash_controller *str9xpec_info;
	struct arm *arm = NULL;
	struct arm7_9_common *arm7_9 = NULL;
	struct arm_jtag *jtag_info = NULL;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	str9xpec_info = malloc(sizeof(struct str9xpec_flash_controller));
	bank->driver_priv = str9xpec_info;

	/* REVISIT verify that the jtag position of flash controller is
	 * right after *THIS* core, which must be a STR9xx core ...
	 */
	arm = bank->target->arch_info;
	arm7_9 = arm->arch_info;
	jtag_info = &arm7_9->jtag_info;

	/* The core is the next tap after the flash controller in the chain */
	str9xpec_info->tap = jtag_tap_by_position(jtag_info->tap->abs_chain_position - 1);
	str9xpec_info->isc_enable = 0;

	str9xpec_build_block_list(bank);

	/* clear option byte register */
	buf_set_u32(str9xpec_info->options, 0, 64, 0);

	return ERROR_OK;
}

static int str9xpec_blank_check(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct scan_field field;
	uint8_t status;
	struct jtag_tap *tap;
	uint8_t *buffer = NULL;

	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable)
		str9xpec_isc_enable(bank);

	if (!str9xpec_info->isc_enable)
		return ERROR_FLASH_OPERATION_FAILED;

	buffer = calloc(DIV_ROUND_UP(64, 8), 1);

	LOG_DEBUG("blank check: first_bank: %u, last_bank: %u", first, last);

	for (unsigned int i = first; i <= last; i++)
		buf_set_u32(buffer, str9xpec_info->sector_bits[i], 1, 1);

	/* execute ISC_BLANK_CHECK command */
	str9xpec_set_instr(tap, ISC_BLANK_CHECK, TAP_IRPAUSE);

	field.num_bits = 64;
	field.out_value = buffer;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_sleep(40000);

	/* read blank check result */
	field.num_bits = 64;
	field.out_value = NULL;
	field.in_value = buffer;

	jtag_add_dr_scan(tap, 1, &field, TAP_IRPAUSE);
	jtag_execute_queue();

	status = str9xpec_isc_status(tap);

	for (unsigned int i = first; i <= last; i++) {
		if (buf_get_u32(buffer, str9xpec_info->sector_bits[i], 1))
			bank->sectors[i].is_erased = 0;
		else
			bank->sectors[i].is_erased = 1;
	}

	free(buffer);

	str9xpec_isc_disable(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;
	return ERROR_OK;
}

static int str9xpec_protect_check(struct flash_bank *bank)
{
	uint8_t status;

	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	status = str9xpec_read_config(bank);

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (buf_get_u32(str9xpec_info->options, str9xpec_info->sector_bits[i], 1))
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;
	return ERROR_OK;
}

static int str9xpec_erase_area(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct scan_field field;
	uint8_t status;
	struct jtag_tap *tap;
	uint8_t *buffer = NULL;

	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable)
		str9xpec_isc_enable(bank);

	if (!str9xpec_info->isc_enable)
		return ISC_STATUS_ERROR;

	buffer = calloc(DIV_ROUND_UP(64, 8), 1);

	LOG_DEBUG("erase: first_bank: %u, last_bank: %u", first, last);

	/* last bank: 0xFF signals a full erase (unlock complete device) */
	/* last bank: 0xFE signals a option byte erase */
	if (last == 0xFF) {
		for (unsigned int i = 0; i < 64; i++)
			buf_set_u32(buffer, i, 1, 1);
	} else if (last == 0xFE)
		buf_set_u32(buffer, 49, 1, 1);
	else {
		for (unsigned int i = first; i <= last; i++)
			buf_set_u32(buffer, str9xpec_info->sector_bits[i], 1, 1);
	}

	LOG_DEBUG("ISC_ERASE");

	/* execute ISC_ERASE command */
	str9xpec_set_instr(tap, ISC_ERASE, TAP_IRPAUSE);

	field.num_bits = 64;
	field.out_value = buffer;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_execute_queue();

	jtag_add_sleep(10);

	/* wait for erase completion */
	while (!((status = str9xpec_isc_status(tap)) & ISC_STATUS_BUSY))
		alive_sleep(1);

	free(buffer);

	str9xpec_isc_disable(bank);

	return status;
}

static int str9xpec_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int status;

	status = str9xpec_erase_area(bank, first, last);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

static int str9xpec_lock_device(struct flash_bank *bank)
{
	struct scan_field field;
	uint8_t status;
	struct jtag_tap *tap;
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable)
		str9xpec_isc_enable(bank);

	if (!str9xpec_info->isc_enable)
		return ISC_STATUS_ERROR;

	/* set security address */
	str9xpec_set_address(bank, 0x80);

	/* execute ISC_PROGRAM command */
	str9xpec_set_instr(tap, ISC_PROGRAM_SECURITY, TAP_IDLE);

	str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

	do {
		field.num_bits = 8;
		field.out_value = NULL;
		field.in_value = &status;

		jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
		jtag_execute_queue();

	} while (!(status & ISC_STATUS_BUSY));

	str9xpec_isc_disable(bank);

	return status;
}

static int str9xpec_unlock_device(struct flash_bank *bank)
{
	uint8_t status;

	status = str9xpec_erase_area(bank, 0, 255);

	return status;
}

static int str9xpec_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	uint8_t status;

	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	status = str9xpec_read_config(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	LOG_DEBUG("protect: first_bank: %u, last_bank: %u", first, last);

	/* last bank: 0xFF signals a full device protect */
	if (last == 0xFF) {
		if (set)
			status = str9xpec_lock_device(bank);
		else {
			/* perform full erase to unlock device */
			status = str9xpec_unlock_device(bank);
		}
	} else {
		for (unsigned int i = first; i <= last; i++) {
			if (set)
				buf_set_u32(str9xpec_info->options, str9xpec_info->sector_bits[i], 1, 1);
			else
				buf_set_u32(str9xpec_info->options, str9xpec_info->sector_bits[i], 1, 0);
		}

		status = str9xpec_write_options(bank);
	}

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

static int str9xpec_set_address(struct flash_bank *bank, uint8_t sector)
{
	struct jtag_tap *tap;
	struct scan_field field;
	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	/* set flash controller address */
	str9xpec_set_instr(tap, ISC_ADDRESS_SHIFT, TAP_IRPAUSE);

	field.num_bits = 8;
	field.out_value = &sector;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IRPAUSE);

	return ERROR_OK;
}

static int str9xpec_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct str9xpec_flash_controller *str9xpec_info = bank->driver_priv;
	uint32_t dwords_remaining = (count / 8);
	uint32_t bytes_remaining = (count & 0x00000007);
	uint32_t bytes_written = 0;
	uint8_t status;
	uint32_t check_address = offset;
	struct jtag_tap *tap;
	struct scan_field field;
	uint8_t *scanbuf;
	unsigned int first_sector = 0;
	unsigned int last_sector = 0;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable)
		str9xpec_isc_enable(bank);

	if (!str9xpec_info->isc_enable)
		return ERROR_FLASH_OPERATION_FAILED;

	if (offset & 0x7) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 8-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		uint32_t sec_start = bank->sectors[i].offset;
		uint32_t sec_end = sec_start + bank->sectors[i].size;

		/* check if destination falls within the current sector */
		if ((check_address >= sec_start) && (check_address < sec_end)) {
			/* check if destination ends in the current sector */
			if (offset + count < sec_end)
				check_address = offset + count;
			else
				check_address = sec_end;
		}

		if ((offset >= sec_start) && (offset < sec_end))
			first_sector = i;

		if ((offset + count >= sec_start) && (offset + count < sec_end))
			last_sector = i;
	}

	if (check_address != offset + count)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	LOG_DEBUG("first_sector: %i, last_sector: %i", first_sector, last_sector);

	scanbuf = calloc(DIV_ROUND_UP(64, 8), 1);

	LOG_DEBUG("ISC_PROGRAM");

	for (unsigned int i = first_sector; i <= last_sector; i++) {
		str9xpec_set_address(bank, str9xpec_info->sector_bits[i]);

		dwords_remaining = dwords_remaining < (bank->sectors[i].size/8)
				? dwords_remaining : (bank->sectors[i].size/8);

		while (dwords_remaining > 0) {
			str9xpec_set_instr(tap, ISC_PROGRAM, TAP_IRPAUSE);

			field.num_bits = 64;
			field.out_value = (buffer + bytes_written);
			field.in_value = NULL;

			jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

			/* small delay before polling */
			jtag_add_sleep(50);

			str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

			do {
				field.num_bits = 8;
				field.out_value = NULL;
				field.in_value = scanbuf;

				jtag_add_dr_scan(tap, 1, &field, TAP_IRPAUSE);
				jtag_execute_queue();

				status = buf_get_u32(scanbuf, 0, 8);

			} while (!(status & ISC_STATUS_BUSY));

			if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
				return ERROR_FLASH_OPERATION_FAILED;

			/* if ((status & ISC_STATUS_INT_ERROR) != STR9XPEC_ISC_INTFAIL)
				return ERROR_FLASH_OPERATION_FAILED; */

			dwords_remaining--;
			bytes_written += 8;
		}
	}

	if (bytes_remaining) {
		uint8_t last_dword[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

		/* copy the last remaining bytes into the write buffer */
		memcpy(last_dword, buffer+bytes_written, bytes_remaining);

		str9xpec_set_instr(tap, ISC_PROGRAM, TAP_IRPAUSE);

		field.num_bits = 64;
		field.out_value = last_dword;
		field.in_value = NULL;

		jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

		/* small delay before polling */
		jtag_add_sleep(50);

		str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

		do {
			field.num_bits = 8;
			field.out_value = NULL;
			field.in_value = scanbuf;

			jtag_add_dr_scan(tap, 1, &field, TAP_IRPAUSE);
			jtag_execute_queue();

			status = buf_get_u32(scanbuf, 0, 8);

		} while (!(status & ISC_STATUS_BUSY));

		if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
			return ERROR_FLASH_OPERATION_FAILED;

		/* if ((status & ISC_STATUS_INT_ERROR) != STR9XPEC_ISC_INTFAIL)
			return ERROR_FLASH_OPERATION_FAILED; */
	}

	free(scanbuf);

	str9xpec_isc_disable(bank);

	return ERROR_OK;
}

static int str9xpec_probe(struct flash_bank *bank)
{
	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_part_id_command)
{
	struct scan_field field;
	uint8_t *buffer = NULL;
	struct jtag_tap *tap;
	uint32_t idcode;
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	buffer = calloc(DIV_ROUND_UP(32, 8), 1);

	str9xpec_set_instr(tap, ISC_IDCODE, TAP_IRPAUSE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = buffer;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_execute_queue();

	idcode = buf_get_u32(buffer, 0, 32);

	command_print(CMD, "str9xpec part id: 0x%8.8" PRIx32 "", idcode);

	free(buffer);

	return ERROR_OK;
}

static int str9xpec_erase_check(struct flash_bank *bank)
{
	return str9xpec_blank_check(bank, 0, bank->num_sectors - 1);
}

COMMAND_HANDLER(str9xpec_handle_flash_options_read_command)
{
	uint8_t status;
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;

	status = str9xpec_read_config(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	/* boot bank */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_CSMAPBIT, 1))
		command_print(CMD, "CS Map: bank1");
	else
		command_print(CMD, "CS Map: bank0");

	/* OTP lock */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_OTPBIT, 1))
		command_print(CMD, "OTP Lock: OTP Locked");
	else
		command_print(CMD, "OTP Lock: OTP Unlocked");

	/* LVD Threshold */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_LVDTHRESBIT, 1))
		command_print(CMD, "LVD Threshold: 2.7v");
	else
		command_print(CMD, "LVD Threshold: 2.4v");

	/* LVD reset warning */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_LVDWARNBIT, 1))
		command_print(CMD, "LVD Reset Warning: VDD or VDDQ Inputs");
	else
		command_print(CMD, "LVD Reset Warning: VDD Input Only");

	/* LVD reset select */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_LVDSELBIT, 1))
		command_print(CMD, "LVD Reset Selection: VDD or VDDQ Inputs");
	else
		command_print(CMD, "LVD Reset Selection: VDD Input Only");

	return ERROR_OK;
}

static int str9xpec_write_options(struct flash_bank *bank)
{
	struct scan_field field;
	uint8_t status;
	struct jtag_tap *tap;
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	/* erase config options first */
	status = str9xpec_erase_area(bank, 0xFE, 0xFE);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return status;

	if (!str9xpec_info->isc_enable)
		str9xpec_isc_enable(bank);

	if (!str9xpec_info->isc_enable)
		return ISC_STATUS_ERROR;

	/* according to data 64th bit has to be set */
	buf_set_u32(str9xpec_info->options, 63, 1, 1);

	/* set option byte address */
	str9xpec_set_address(bank, 0x50);

	/* execute ISC_PROGRAM command */
	str9xpec_set_instr(tap, ISC_PROGRAM, TAP_IRPAUSE);

	field.num_bits = 64;
	field.out_value = str9xpec_info->options;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	/* small delay before polling */
	jtag_add_sleep(50);

	str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

	do {
		field.num_bits = 8;
		field.out_value = NULL;
		field.in_value = &status;

		jtag_add_dr_scan(tap, 1, &field, TAP_IRPAUSE);
		jtag_execute_queue();

	} while (!(status & ISC_STATUS_BUSY));

	str9xpec_isc_disable(bank);

	return status;
}

COMMAND_HANDLER(str9xpec_handle_flash_options_write_command)
{
	uint8_t status;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	status = str9xpec_write_options(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	command_print(CMD, "str9xpec write options complete.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_options_cmap_command)
{
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;

	if (strcmp(CMD_ARGV[1], "bank1") == 0)
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_CSMAPBIT, 1, 1);
	else
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_CSMAPBIT, 1, 0);

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_options_lvdthd_command)
{
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;

	if (strcmp(CMD_ARGV[1], "2.7v") == 0)
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDTHRESBIT, 1, 1);
	else
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDTHRESBIT, 1, 0);

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_options_lvdsel_command)
{
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;

	if (strcmp(CMD_ARGV[1], "vdd_vddq") == 0)
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDSELBIT, 1, 1);
	else
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDSELBIT, 1, 0);

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_options_lvdwarn_command)
{
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;

	if (strcmp(CMD_ARGV[1], "vdd_vddq") == 0)
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDWARNBIT, 1, 1);
	else
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDWARNBIT, 1, 0);

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_lock_command)
{
	uint8_t status;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	status = str9xpec_lock_device(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_unlock_command)
{
	uint8_t status;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	status = str9xpec_unlock_device(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	command_print(CMD, "str9xpec unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_enable_turbo_command)
{
	struct jtag_tap *tap0;
	struct jtag_tap *tap1;
	struct jtag_tap *tap2;
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;

	/* remove arm core from chain - enter turbo mode */
	tap0 = str9xpec_info->tap;
	if (!tap0) {
		/* things are *WRONG* */
		command_print(CMD, "**STR9FLASH** (tap0) invalid chain?");
		return ERROR_FAIL;
	}
	tap1 = tap0->next_tap;
	if (!tap1) {
		/* things are *WRONG* */
		command_print(CMD, "**STR9FLASH** (tap1) invalid chain?");
		return ERROR_FAIL;
	}
	tap2 = tap1->next_tap;
	if (!tap2) {
		/* things are *WRONG* */
		command_print(CMD, "**STR9FLASH** (tap2) invalid chain?");
		return ERROR_FAIL;
	}

	/* enable turbo mode - TURBO-PROG-ENABLE */
	str9xpec_set_instr(tap2, 0xD, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* modify scan chain - str9 core has been removed */
	tap1->enabled = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(str9xpec_handle_flash_disable_turbo_command)
{
	struct jtag_tap *tap;
	struct str9xpec_flash_controller *str9xpec_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	if (!tap)
		return ERROR_FAIL;

	/* exit turbo mode via RESET */
	str9xpec_set_instr(tap, ISC_NOOP, TAP_IDLE);
	jtag_add_tlr();
	jtag_execute_queue();

	/* restore previous scan chain */
	if (tap->next_tap)
		tap->next_tap->enabled = 1;

	return ERROR_OK;
}

static const struct command_registration str9xpec_config_command_handlers[] = {
	{
		.name = "enable_turbo",
		.usage = "<bank>",
		.handler = str9xpec_handle_flash_enable_turbo_command,
		.mode = COMMAND_EXEC,
		.help = "enable str9xpec turbo mode",
	},
	{
		.name = "disable_turbo",
		.usage = "<bank>",
		.handler = str9xpec_handle_flash_disable_turbo_command,
		.mode = COMMAND_EXEC,
		.help = "disable str9xpec turbo mode",
	},
	{
		.name = "options_cmap",
		.usage = "<bank> <bank0 | bank1>",
		.handler = str9xpec_handle_flash_options_cmap_command,
		.mode = COMMAND_EXEC,
		.help = "configure str9xpec boot sector",
	},
	{
		.name = "options_lvdthd",
		.usage = "<bank> <2.4v | 2.7v>",
		.handler = str9xpec_handle_flash_options_lvdthd_command,
		.mode = COMMAND_EXEC,
		.help = "configure str9xpec lvd threshold",
	},
	{
		.name = "options_lvdsel",
		.usage = "<bank> <vdd | vdd_vddq>",
		.handler = str9xpec_handle_flash_options_lvdsel_command,
		.mode = COMMAND_EXEC,
		.help = "configure str9xpec lvd selection",
	},
	{
		.name = "options_lvdwarn",
		.usage = "<bank> <vdd | vdd_vddq>",
		.handler = str9xpec_handle_flash_options_lvdwarn_command,
		.mode = COMMAND_EXEC,
		.help = "configure str9xpec lvd warning",
	},
	{
		.name = "options_read",
		.usage = "<bank>",
		.handler = str9xpec_handle_flash_options_read_command,
		.mode = COMMAND_EXEC,
		.help = "read str9xpec options",
	},
	{
		.name = "options_write",
		.usage = "<bank>",
		.handler = str9xpec_handle_flash_options_write_command,
		.mode = COMMAND_EXEC,
		.help = "write str9xpec options",
	},
	{
		.name = "lock",
		.usage = "<bank>",
		.handler = str9xpec_handle_flash_lock_command,
		.mode = COMMAND_EXEC,
		.help = "lock str9xpec device",
	},
	{
		.name = "unlock",
		.usage = "<bank>",
		.handler = str9xpec_handle_flash_unlock_command,
		.mode = COMMAND_EXEC,
		.help = "unlock str9xpec device",
	},
	{
		.name = "part_id",
		.usage = "<bank>",
		.handler = str9xpec_handle_part_id_command,
		.mode = COMMAND_EXEC,
		.help = "print part id of str9xpec flash bank",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration str9xpec_command_handlers[] = {
	{
		.name = "str9xpec",
		.mode = COMMAND_ANY,
		.help = "str9xpec flash command group",
		.usage = "",
		.chain = str9xpec_config_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver str9xpec_flash = {
	.name = "str9xpec",
	.commands = str9xpec_command_handlers,
	.flash_bank_command = str9xpec_flash_bank_command,
	.erase = str9xpec_erase,
	.protect = str9xpec_protect,
	.write = str9xpec_write,
	.read = default_flash_read,
	.probe = str9xpec_probe,
	.auto_probe = str9xpec_probe,
	.erase_check = str9xpec_erase_check,
	.protect_check = str9xpec_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
