/***************************************************************************
 *   Copyright (C) 2015 Robert Jordens <jordens@gmail.com>                 *
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
#include <jtag/jtag.h>
#include <flash/nor/spi.h>
#include <helper/time_support.h>

#define JTAGSPI_MAX_TIMEOUT 3000


struct jtagspi_flash_bank {
	struct jtag_tap *tap;
	const struct flash_device *dev;
	bool probed;
	uint32_t ir;
};

FLASH_BANK_COMMAND_HANDLER(jtagspi_flash_bank_command)
{
	struct jtagspi_flash_bank *info;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	info = malloc(sizeof(struct jtagspi_flash_bank));
	if (!info) {
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	bank->driver_priv = info;

	info->tap = NULL;
	info->probed = false;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->ir);

	return ERROR_OK;
}

static void jtagspi_set_ir(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct scan_field field;
	uint8_t buf[4] = { 0 };

	LOG_DEBUG("loading jtagspi ir");
	buf_set_u32(buf, 0, info->tap->ir_length, info->ir);
	field.num_bits = info->tap->ir_length;
	field.out_value = buf;
	field.in_value = NULL;
	jtag_add_ir_scan(info->tap, &field, TAP_IDLE);
}

static void flip_u8(uint8_t *in, uint8_t *out, int len)
{
	for (int i = 0; i < len; i++)
		out[i] = flip_u32(in[i], 8);
}

static int jtagspi_cmd(struct flash_bank *bank, uint8_t cmd,
		uint32_t *addr, uint8_t *data, int len)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct scan_field fields[6];
	uint8_t marker = 1;
	uint8_t xfer_bits_buf[4];
	uint8_t addr_buf[3];
	uint8_t *data_buf;
	uint32_t xfer_bits;
	int is_read, lenb, n;

	/* LOG_DEBUG("cmd=0x%02x len=%i", cmd, len); */

	is_read = (len < 0);
	if (is_read)
		len = -len;

	n = 0;

	fields[n].num_bits = 1;
	fields[n].out_value = &marker;
	fields[n].in_value = NULL;
	n++;

	xfer_bits = 8 + len - 1;
	/* cmd + read/write - 1 due to the counter implementation */
	if (addr)
		xfer_bits += 24;
	h_u32_to_be(xfer_bits_buf, xfer_bits);
	flip_u8(xfer_bits_buf, xfer_bits_buf, 4);
	fields[n].num_bits = 32;
	fields[n].out_value = xfer_bits_buf;
	fields[n].in_value = NULL;
	n++;

	cmd = flip_u32(cmd, 8);
	fields[n].num_bits = 8;
	fields[n].out_value = &cmd;
	fields[n].in_value = NULL;
	n++;

	if (addr) {
		h_u24_to_be(addr_buf, *addr);
		flip_u8(addr_buf, addr_buf, 3);
		fields[n].num_bits = 24;
		fields[n].out_value = addr_buf;
		fields[n].in_value = NULL;
		n++;
	}

	lenb = DIV_ROUND_UP(len, 8);
	data_buf = malloc(lenb);
	if (lenb > 0) {
		if (!data_buf) {
			LOG_ERROR("no memory for spi buffer");
			return ERROR_FAIL;
		}
		if (is_read) {
			fields[n].num_bits = jtag_tap_count_enabled();
			fields[n].out_value = NULL;
			fields[n].in_value = NULL;
			n++;

			fields[n].out_value = NULL;
			fields[n].in_value = data_buf;
		} else {
			flip_u8(data, data_buf, lenb);
			fields[n].out_value = data_buf;
			fields[n].in_value = NULL;
		}
		fields[n].num_bits = len;
		n++;
	}

	jtagspi_set_ir(bank);
	/* passing from an IR scan to SHIFT-DR clears BYPASS registers */
	jtag_add_dr_scan(info->tap, n, fields, TAP_IDLE);
	int retval = jtag_execute_queue();

	if (is_read)
		flip_u8(data_buf, data, lenb);
	free(data_buf);
	return retval;
}

static int jtagspi_probe(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	struct flash_sector *sectors;
	uint8_t in_buf[3];
	uint32_t id, sectorsize;

	if (info->probed)
		free(bank->sectors);
	info->probed = false;

	if (!bank->target->tap) {
		LOG_ERROR("Target has no JTAG tap");
		return ERROR_FAIL;
	}
	info->tap = bank->target->tap;

	jtagspi_cmd(bank, SPIFLASH_READ_ID, NULL, in_buf, -24);
	/* the table in spi.c has the manufacturer byte (first) as the lsb */
	id = le_to_h_u24(in_buf);

	info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			info->dev = p;
			break;
		}

	if (!(info->dev)) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		info->dev->name, info->dev->device_id);

	/* Set correct size value */
	bank->size = info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = info->dev->sectorsize ?
		info->dev->sectorsize : info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = info->dev->size_in_bytes / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	info->probed = true;
	return ERROR_OK;
}

static int jtagspi_read_status(struct flash_bank *bank, uint32_t *status)
{
	uint8_t buf;
	int err = jtagspi_cmd(bank, SPIFLASH_READ_STATUS, NULL, &buf, -8);
	if (err == ERROR_OK) {
		*status = buf;
		/* LOG_DEBUG("status=0x%08" PRIx32, *status); */
	}

	return err;
}

static int jtagspi_wait(struct flash_bank *bank, int timeout_ms)
{
	uint32_t status;
	int64_t t0 = timeval_ms();
	int64_t dt;

	do {
		dt = timeval_ms() - t0;

		int retval = jtagspi_read_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & SPIFLASH_BSY_BIT) == 0) {
			LOG_DEBUG("waited %" PRId64 " ms", dt);
			return ERROR_OK;
		}
		alive_sleep(1);
	} while (dt <= timeout_ms);

	LOG_ERROR("timeout, device still busy");
	return ERROR_FAIL;
}

static int jtagspi_write_enable(struct flash_bank *bank)
{
	uint32_t status;

	jtagspi_cmd(bank, SPIFLASH_WRITE_ENABLE, NULL, NULL, 0);

	int retval = jtagspi_read_status(bank, &status);
	if (retval != ERROR_OK)
		return retval;

	if ((status & SPIFLASH_WE_BIT) == 0) {
		LOG_ERROR("Cannot enable write to flash. Status=0x%08" PRIx32, status);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int jtagspi_bulk_erase(struct flash_bank *bank)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;
	int64_t t0 = timeval_ms();

	if (info->dev->chip_erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;
	jtagspi_cmd(bank, info->dev->chip_erase_cmd, NULL, NULL, 0);
	retval = jtagspi_wait(bank, bank->num_sectors*JTAGSPI_MAX_TIMEOUT);
	LOG_INFO("took %" PRId64 " ms", timeval_ms() - t0);
	return retval;
}

static int jtagspi_sector_erase(struct flash_bank *bank, unsigned int sector)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;
	int64_t t0 = timeval_ms();

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;
	jtagspi_cmd(bank, info->dev->erase_cmd, &bank->sectors[sector].offset, NULL, 0);
	retval = jtagspi_wait(bank, JTAGSPI_MAX_TIMEOUT);
	LOG_INFO("sector %u took %" PRId64 " ms", sector, timeval_ms() - t0);
	return retval;
}

static int jtagspi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("erase from sector %u to sector %u", first, last);

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	if (first == 0 && last == (bank->num_sectors - 1)
		&& info->dev->chip_erase_cmd != info->dev->erase_cmd) {
		LOG_DEBUG("Trying bulk erase.");
		retval = jtagspi_bulk_erase(bank);
		if (retval == ERROR_OK)
			return retval;
		else
			LOG_WARNING("Bulk flash erase failed. Falling back to sector erase.");
	}

	if (info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = jtagspi_sector_erase(bank, sector);
		if (retval != ERROR_OK) {
			LOG_ERROR("Sector erase failed.");
			break;
		}
	}

	return retval;
}

static int jtagspi_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int jtagspi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not yet probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	jtagspi_cmd(bank, SPIFLASH_READ, &offset, buffer, -count*8);
	return ERROR_OK;
}

static int jtagspi_page_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	retval = jtagspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;
	jtagspi_cmd(bank, SPIFLASH_PAGE_PROGRAM, &offset, (uint8_t *) buffer, count*8);
	return jtagspi_wait(bank, JTAGSPI_MAX_TIMEOUT);
}

static int jtagspi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;
	int retval;
	uint32_t n, pagesize;

	if (!(info->probed)) {
		LOG_ERROR("Flash bank not yet probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* if no write pagesize, use reasonable default */
	pagesize = info->dev->pagesize ? info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	for (n = 0; n < count; n += pagesize) {
		retval = jtagspi_page_write(bank, buffer + n, offset + n,
				MIN(count - n, pagesize));
		if (retval != ERROR_OK) {
			LOG_ERROR("page write error");
			return retval;
		}
		LOG_DEBUG("wrote page at 0x%08" PRIx32, offset + n);
	}
	return ERROR_OK;
}

static int jtagspi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct jtagspi_flash_bank *info = bank->driver_priv;

	if (!(info->probed)) {
		command_print_sameline(cmd, "\nJTAGSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "\nSPIFI flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		info->dev->name, info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver jtagspi_flash = {
	.name = "jtagspi",
	.flash_bank_command = jtagspi_flash_bank_command,
	.erase = jtagspi_erase,
	.protect = jtagspi_protect,
	.write = jtagspi_write,
	.read = jtagspi_read,
	.probe = jtagspi_probe,
	.auto_probe = jtagspi_probe,
	.erase_check = default_flash_blank_check,
	.info = jtagspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
