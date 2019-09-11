/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *   Copyright (C) 2017-2018 Tomas Vanek <vanekt@fbl.cz>                   *
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
#include <helper/time_support.h>
#include <target/image.h>

/**
 * @file
 * Implements Tcl commands used to access NOR flash facilities.
 */

COMMAND_HELPER(flash_command_get_bank_maybe_probe, unsigned name_index,
	       struct flash_bank **bank, bool do_probe)
{
	const char *name = CMD_ARGV[name_index];
	int retval;
	if (do_probe) {
		retval = get_flash_bank_by_name(name, bank);
	} else {
		*bank  = get_flash_bank_by_name_noprobe(name);
		retval = ERROR_OK;
	}

	if (retval != ERROR_OK)
		return retval;
	if (*bank)
		return ERROR_OK;

	unsigned bank_num;
	COMMAND_PARSE_NUMBER(uint, name, bank_num);

	if (do_probe) {
		return get_flash_bank_by_num(bank_num, bank);
	} else {
		*bank  = get_flash_bank_by_num_noprobe(bank_num);
		retval = (bank) ? ERROR_OK : ERROR_FAIL;
		return retval;
	}
}

COMMAND_HELPER(flash_command_get_bank, unsigned name_index,
	struct flash_bank **bank)
{
	return CALL_COMMAND_HANDLER(flash_command_get_bank_maybe_probe,
				    name_index, bank, true);
}

COMMAND_HANDLER(handle_flash_info_command)
{
	struct flash_bank *p;
	int j = 0;
	int retval;
	bool show_sectors = false;
	bool prot_block_available;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 2) {
		if (strcmp("sectors", CMD_ARGV[1]) == 0)
			show_sectors = true;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (retval != ERROR_OK)
		return retval;

	if (p != NULL) {
		char buf[1024];
		int num_blocks;
		struct flash_sector *block_array;

		/* attempt auto probe */
		retval = p->driver->auto_probe(p);
		if (retval != ERROR_OK)
			return retval;

		/* If the driver does not implement protection, we show the default
		 * state of is_protected array - usually protection state unknown */
		if (p->driver->protect_check == NULL) {
			retval = ERROR_FLASH_OPER_UNSUPPORTED;
		} else {
			/* We must query the hardware to avoid printing stale information! */
			retval = p->driver->protect_check(p);
			if (retval != ERROR_OK && retval != ERROR_FLASH_OPER_UNSUPPORTED)
				return retval;
		}
		if (retval == ERROR_FLASH_OPER_UNSUPPORTED)
			LOG_WARNING("Flash protection check is not implemented.");

		command_print(CMD,
			"#%d : %s at " TARGET_ADDR_FMT ", size 0x%8.8" PRIx32
			", buswidth %i, chipwidth %i",
			p->bank_number,
			p->driver->name,
			p->base,
			p->size,
			p->bus_width,
			p->chip_width);

		prot_block_available = p->num_prot_blocks && p->prot_blocks;
		if (!show_sectors && prot_block_available) {
			block_array = p->prot_blocks;
			num_blocks = p->num_prot_blocks;
		} else {
			block_array = p->sectors;
			num_blocks = p->num_sectors;
		}

		for (j = 0; j < num_blocks; j++) {
			char *protect_state = "";

			if (block_array[j].is_protected == 0)
				protect_state = "not protected";
			else if (block_array[j].is_protected == 1)
				protect_state = "protected";
			else if (!show_sectors || !prot_block_available)
				protect_state = "protection state unknown";

			command_print(CMD,
				"\t#%3i: 0x%8.8" PRIx32 " (0x%" PRIx32 " %" PRIi32 "kB) %s",
				j,
				block_array[j].offset,
				block_array[j].size,
				block_array[j].size >> 10,
				protect_state);
		}

		if (p->driver->info != NULL) {
			retval = p->driver->info(p, buf, sizeof(buf));
			if (retval == ERROR_OK)
				command_print(CMD, "%s", buf);
			else
				LOG_ERROR("error retrieving flash info");
		}
	}

	return retval;
}

COMMAND_HANDLER(handle_flash_probe_command)
{
	struct flash_bank *p;
	int retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank_maybe_probe, 0, &p, false);
	if (retval != ERROR_OK)
		return retval;

	if (p) {
		retval = p->driver->probe(p);
		if (retval == ERROR_OK)
			command_print(CMD,
				"flash '%s' found at " TARGET_ADDR_FMT,
				p->driver->name,
				p->base);
	} else {
		command_print(CMD, "flash bank '#%s' is out of bounds", CMD_ARGV[0]);
		retval = ERROR_FAIL;
	}

	return retval;
}

COMMAND_HANDLER(handle_flash_erase_check_command)
{
	bool blank = true;
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *p;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	int j;
	retval = p->driver->erase_check(p);
	if (retval == ERROR_OK)
		command_print(CMD, "successfully checked erase state");
	else {
		command_print(CMD,
			"unknown error when checking erase state of flash bank #%s at "
			TARGET_ADDR_FMT,
			CMD_ARGV[0],
			p->base);
	}

	for (j = 0; j < p->num_sectors; j++) {
		char *erase_state;

		if (p->sectors[j].is_erased == 0)
			erase_state = "not erased";
		else if (p->sectors[j].is_erased == 1)
			continue;
		else
			erase_state = "erase state unknown";

		blank = false;
		command_print(CMD,
			"\t#%3i: 0x%8.8" PRIx32 " (0x%" PRIx32 " %" PRIi32 "kB) %s",
			j,
			p->sectors[j].offset,
			p->sectors[j].size,
			p->sectors[j].size >> 10,
			erase_state);
	}

	if (blank)
		command_print(CMD, "\tBank is erased");
	return retval;
}

COMMAND_HANDLER(handle_flash_erase_address_command)
{
	struct flash_bank *p;
	int retval = ERROR_OK;
	target_addr_t address;
	uint32_t length;
	bool do_pad = false;
	bool do_unlock = false;
	struct target *target = get_current_target(CMD_CTX);

	while (CMD_ARGC >= 3) {
		/* Optionally pad out the address range to block/sector
		 * boundaries.  We can't know if there's data in that part
		 * of the flash; only do padding if we're told to.
		 */
		if (strcmp("pad", CMD_ARGV[0]) == 0)
			do_pad = true;
		else if (strcmp("unlock", CMD_ARGV[0]) == 0)
			do_unlock = true;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], length);

	if (length <= 0) {
		command_print(CMD, "Length must be >0");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = get_flash_bank_by_addr(target, address, true, &p);
	if (retval != ERROR_OK)
		return retval;

	/* We can't know if we did a resume + halt, in which case we no longer know the erased state
	 **/
	flash_set_dirty();

	struct duration bench;
	duration_start(&bench);

	if (do_unlock)
		retval = flash_unlock_address_range(target, address, length);

	if (retval == ERROR_OK)
		retval = flash_erase_address_range(target, do_pad, address, length);

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK)) {
		command_print(CMD, "erased address " TARGET_ADDR_FMT " (length %"
				PRIi32 ")"
			" in %fs (%0.3f KiB/s)", address, length,
			duration_elapsed(&bench), duration_kbps(&bench, length));
	}

	return retval;
}

COMMAND_HANDLER(handle_flash_erase_command)
{
	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t first;
	uint32_t last;

	struct flash_bank *p;
	int retval;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (retval != ERROR_OK)
		return retval;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], first);
	if (strcmp(CMD_ARGV[2], "last") == 0)
		last = p->num_sectors - 1;
	else
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], last);

	if (!(first <= last)) {
		command_print(CMD, "ERROR: "
			"first sector must be <= last");
		return ERROR_FAIL;
	}

	if (!(last <= (uint32_t)(p->num_sectors - 1))) {
		command_print(CMD, "ERROR: "
			"last sector must be <= %" PRIu32,
			p->num_sectors - 1);
		return ERROR_FAIL;
	}

	struct duration bench;
	duration_start(&bench);

	retval = flash_driver_erase(p, first, last);

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK)) {
		command_print(CMD, "erased sectors %" PRIu32 " "
			"through %" PRIu32 " on flash bank %d "
			"in %fs", first, last, p->bank_number, duration_elapsed(&bench));
	}

	return retval;
}

COMMAND_HANDLER(handle_flash_protect_command)
{
	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t first;
	uint32_t last;

	struct flash_bank *p;
	int retval;
	int num_blocks;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (retval != ERROR_OK)
		return retval;

	if (p->num_prot_blocks)
		num_blocks = p->num_prot_blocks;
	else
		num_blocks = p->num_sectors;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], first);
	if (strcmp(CMD_ARGV[2], "last") == 0)
		last = num_blocks - 1;
	else
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], last);

	bool set;
	COMMAND_PARSE_ON_OFF(CMD_ARGV[3], set);

	if (!(first <= last)) {
		command_print(CMD, "ERROR: "
			"first %s must be <= last",
			(p->num_prot_blocks) ? "block" : "sector");
		return ERROR_FAIL;
	}

	if (!(last <= (uint32_t)(num_blocks - 1))) {
		command_print(CMD, "ERROR: "
			"last %s must be <= %" PRIu32,
			(p->num_prot_blocks) ? "block" : "sector",
			num_blocks - 1);
		return ERROR_FAIL;
	}

	retval = flash_driver_protect(p, set, first, last);
	if (retval == ERROR_OK) {
		command_print(CMD, "%s protection for %s %" PRIu32
			" through %" PRIu32 " on flash bank %d",
			(set) ? "set" : "cleared",
			(p->num_prot_blocks) ? "blocks" : "sectors",
			first, last, p->bank_number);
	}

	return retval;
}

COMMAND_HANDLER(handle_flash_write_image_command)
{
	struct target *target = get_current_target(CMD_CTX);

	struct image image;
	uint32_t written;

	int retval;

	/* flash auto-erase is disabled by default*/
	int auto_erase = 0;
	bool auto_unlock = false;

	while (CMD_ARGC) {
		if (strcmp(CMD_ARGV[0], "erase") == 0) {
			auto_erase = 1;
			CMD_ARGV++;
			CMD_ARGC--;
			command_print(CMD, "auto erase enabled");
		} else if (strcmp(CMD_ARGV[0], "unlock") == 0) {
			auto_unlock = true;
			CMD_ARGV++;
			CMD_ARGC--;
			command_print(CMD, "auto unlock enabled");
		} else
			break;
	}

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!target) {
		LOG_ERROR("no target selected");
		return ERROR_FAIL;
	}

	struct duration bench;
	duration_start(&bench);

	if (CMD_ARGC >= 2) {
		image.base_address_set = 1;
		COMMAND_PARSE_NUMBER(llong, CMD_ARGV[1], image.base_address);
	} else {
		image.base_address_set = 0;
		image.base_address = 0x0;
	}

	image.start_address_set = 0;

	retval = image_open(&image, CMD_ARGV[0], (CMD_ARGC == 3) ? CMD_ARGV[2] : NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = flash_write_unlock(target, &image, &written, auto_erase, auto_unlock);
	if (retval != ERROR_OK) {
		image_close(&image);
		return retval;
	}

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK)) {
		command_print(CMD, "wrote %" PRIu32 " bytes from file %s "
			"in %fs (%0.3f KiB/s)", written, CMD_ARGV[0],
			duration_elapsed(&bench), duration_kbps(&bench, written));
	}

	image_close(&image);

	return retval;
}

COMMAND_HANDLER(handle_flash_fill_command)
{
	target_addr_t address;
	uint32_t pattern;
	uint32_t count;
	struct target *target = get_current_target(CMD_CTX);
	unsigned i;
	uint32_t wordsize;
	int retval;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], pattern);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);

	struct flash_bank *bank;
	retval = get_flash_bank_by_addr(target, address, true, &bank);
	if (retval != ERROR_OK)
		return retval;

	switch (CMD_NAME[4]) {
		case 'w':
			wordsize = 4;
			break;
		case 'h':
			wordsize = 2;
			break;
		case 'b':
			wordsize = 1;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (count == 0)
		return ERROR_OK;

	if (address + count * wordsize > bank->base + bank->size) {
		LOG_ERROR("Cannot cross flash bank borders");
		return ERROR_FAIL;
	}

	uint32_t size_bytes = count * wordsize;
	target_addr_t aligned_start = flash_write_align_start(bank, address);
	target_addr_t end_addr = address + size_bytes - 1;
	target_addr_t aligned_end = flash_write_align_end(bank, end_addr);
	uint32_t aligned_size = aligned_end + 1 - aligned_start;
	uint32_t padding_at_start = address - aligned_start;
	uint32_t padding_at_end = aligned_end - end_addr;

	uint8_t *buffer = malloc(aligned_size);
	if (buffer == NULL)
		return ERROR_FAIL;

	if (padding_at_start) {
		memset(buffer, bank->default_padded_value, padding_at_start);
		LOG_WARNING("Start address " TARGET_ADDR_FMT
			" breaks the required alignment of flash bank %s",
			address, bank->name);
		LOG_WARNING("Padding %" PRId32 " bytes from " TARGET_ADDR_FMT,
		    padding_at_start, aligned_start);
	}

	uint8_t *ptr = buffer + padding_at_start;

	switch (wordsize) {
		case 4:
			for (i = 0; i < count; i++, ptr += wordsize)
				target_buffer_set_u32(target, ptr, pattern);
			break;
		case 2:
			for (i = 0; i < count; i++, ptr += wordsize)
				target_buffer_set_u16(target, ptr, pattern);
			break;
		case 1:
			memset(ptr, pattern, count);
			ptr += count;
			break;
		default:
			LOG_ERROR("BUG: can't happen");
			exit(-1);
	}

	if (padding_at_end) {
		memset(ptr, bank->default_padded_value, padding_at_end);
		LOG_INFO("Padding at " TARGET_ADDR_FMT " with %" PRId32
			" bytes (bank write end alignment)",
			end_addr + 1, padding_at_end);
	}

	struct duration bench;
	duration_start(&bench);

	retval = flash_driver_write(bank, buffer, aligned_start - bank->base, aligned_size);
	if (retval != ERROR_OK)
		goto done;

	retval = flash_driver_read(bank, buffer, address - bank->base, size_bytes);
	if (retval != ERROR_OK)
		goto done;

	for (i = 0, ptr = buffer; i < count; i++) {
		uint32_t readback = 0;

		switch (wordsize) {
			case 4:
				readback = target_buffer_get_u32(target, ptr);
				break;
			case 2:
				readback = target_buffer_get_u16(target, ptr);
				break;
			case 1:
				readback = *ptr;
				break;
		}
		if (readback != pattern) {
			LOG_ERROR(
				"Verification error address " TARGET_ADDR_FMT
				", read back 0x%02" PRIx32 ", expected 0x%02" PRIx32,
				address + i * wordsize, readback, pattern);
			retval = ERROR_FAIL;
			goto done;
		}
		ptr += wordsize;
	}

	if ((retval == ERROR_OK) && (duration_measure(&bench) == ERROR_OK)) {
		command_print(CMD, "wrote %" PRIu32 " bytes to " TARGET_ADDR_FMT
			" in %fs (%0.3f KiB/s)", size_bytes, address,
			duration_elapsed(&bench), duration_kbps(&bench, size_bytes));
	}

done:
	free(buffer);

	return retval;
}

COMMAND_HANDLER(handle_flash_write_bank_command)
{
	uint32_t offset;
	uint8_t *buffer;
	size_t length;
	struct fileio *fileio;

	if (CMD_ARGC < 2 || CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct duration bench;
	duration_start(&bench);

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	offset = 0;

	if (CMD_ARGC > 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], offset);

	if (offset > bank->size) {
		LOG_ERROR("Offset 0x%8.8" PRIx32 " is out of range of the flash bank",
			offset);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (fileio_open(&fileio, CMD_ARGV[1], FILEIO_READ, FILEIO_BINARY) != ERROR_OK)
		return ERROR_FAIL;

	size_t filesize;
	retval = fileio_size(fileio, &filesize);
	if (retval != ERROR_OK) {
		fileio_close(fileio);
		return retval;
	}

	length = MIN(filesize, bank->size - offset);

	if (!length) {
		LOG_INFO("Nothing to write to flash bank");
		fileio_close(fileio);
		return ERROR_OK;
	}

	if (length != filesize)
		LOG_INFO("File content exceeds flash bank size. Only writing the "
			"first %zu bytes of the file", length);

	target_addr_t start_addr = bank->base + offset;
	target_addr_t aligned_start = flash_write_align_start(bank, start_addr);
	target_addr_t end_addr = start_addr + length - 1;
	target_addr_t aligned_end = flash_write_align_end(bank, end_addr);
	uint32_t aligned_size = aligned_end + 1 - aligned_start;
	uint32_t padding_at_start = start_addr - aligned_start;
	uint32_t padding_at_end = aligned_end - end_addr;

	buffer = malloc(aligned_size);
	if (buffer == NULL) {
		fileio_close(fileio);
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	if (padding_at_start) {
		memset(buffer, bank->default_padded_value, padding_at_start);
		LOG_WARNING("Start offset 0x%08" PRIx32
			" breaks the required alignment of flash bank %s",
			offset, bank->name);
		LOG_WARNING("Padding %" PRId32 " bytes from " TARGET_ADDR_FMT,
		    padding_at_start, aligned_start);
	}

	uint8_t *ptr = buffer + padding_at_start;
	size_t buf_cnt;
	if (fileio_read(fileio, length, ptr, &buf_cnt) != ERROR_OK) {
		free(buffer);
		fileio_close(fileio);
		return ERROR_FAIL;
	}

	if (buf_cnt != length) {
		LOG_ERROR("Short read");
		free(buffer);
		return ERROR_FAIL;
	}

	ptr += length;

	if (padding_at_end) {
		memset(ptr, bank->default_padded_value, padding_at_end);
		LOG_INFO("Padding at " TARGET_ADDR_FMT " with %" PRId32
			" bytes (bank write end alignment)",
			end_addr + 1, padding_at_end);
	}

	retval = flash_driver_write(bank, buffer, aligned_start - bank->base, aligned_size);

	free(buffer);

	if ((ERROR_OK == retval) && (duration_measure(&bench) == ERROR_OK)) {
		command_print(CMD, "wrote %zu bytes from file %s to flash bank %u"
			" at offset 0x%8.8" PRIx32 " in %fs (%0.3f KiB/s)",
			length, CMD_ARGV[1], bank->bank_number, offset,
			duration_elapsed(&bench), duration_kbps(&bench, length));
	}

	fileio_close(fileio);

	return retval;
}

COMMAND_HANDLER(handle_flash_read_bank_command)
{
	uint32_t offset;
	uint8_t *buffer;
	struct fileio *fileio;
	uint32_t length;
	size_t written;

	if (CMD_ARGC < 2 || CMD_ARGC > 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct duration bench;
	duration_start(&bench);

	struct flash_bank *p;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);

	if (ERROR_OK != retval)
		return retval;

	offset = 0;

	if (CMD_ARGC > 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], offset);

	if (offset > p->size) {
		LOG_ERROR("Offset 0x%8.8" PRIx32 " is out of range of the flash bank",
			offset);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	length = p->size - offset;

	if (CMD_ARGC > 3)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], length);

	if (offset + length > p->size) {
		LOG_ERROR("Length of %" PRIu32 " bytes with offset 0x%8.8" PRIx32
			" is out of range of the flash bank", length, offset);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	buffer = malloc(length);
	if (buffer == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	retval = flash_driver_read(p, buffer, offset, length);
	if (retval != ERROR_OK) {
		LOG_ERROR("Read error");
		free(buffer);
		return retval;
	}

	retval = fileio_open(&fileio, CMD_ARGV[1], FILEIO_WRITE, FILEIO_BINARY);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not open file");
		free(buffer);
		return retval;
	}

	retval = fileio_write(fileio, length, buffer, &written);
	fileio_close(fileio);
	free(buffer);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not write file");
		return ERROR_FAIL;
	}

	if (duration_measure(&bench) == ERROR_OK)
		command_print(CMD, "wrote %zd bytes to file %s from flash bank %u"
			" at offset 0x%8.8" PRIx32 " in %fs (%0.3f KiB/s)",
			written, CMD_ARGV[1], p->bank_number, offset,
			duration_elapsed(&bench), duration_kbps(&bench, written));

	return retval;
}


COMMAND_HANDLER(handle_flash_verify_bank_command)
{
	uint32_t offset;
	uint8_t *buffer_file, *buffer_flash;
	struct fileio *fileio;
	size_t read_cnt;
	size_t filesize;
	size_t length;
	int differ;

	if (CMD_ARGC < 2 || CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct duration bench;
	duration_start(&bench);

	struct flash_bank *p;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	offset = 0;

	if (CMD_ARGC > 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], offset);

	if (offset > p->size) {
		LOG_ERROR("Offset 0x%8.8" PRIx32 " is out of range of the flash bank",
			offset);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	retval = fileio_open(&fileio, CMD_ARGV[1], FILEIO_READ, FILEIO_BINARY);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not open file");
		return retval;
	}

	retval = fileio_size(fileio, &filesize);
	if (retval != ERROR_OK) {
		fileio_close(fileio);
		return retval;
	}

	length = MIN(filesize, p->size - offset);

	if (!length) {
		LOG_INFO("Nothing to compare with flash bank");
		fileio_close(fileio);
		return ERROR_OK;
	}

	if (length != filesize)
		LOG_INFO("File content exceeds flash bank size. Only comparing the "
			"first %zu bytes of the file", length);

	buffer_file = malloc(length);
	if (buffer_file == NULL) {
		LOG_ERROR("Out of memory");
		fileio_close(fileio);
		return ERROR_FAIL;
	}

	retval = fileio_read(fileio, length, buffer_file, &read_cnt);
	fileio_close(fileio);
	if (retval != ERROR_OK) {
		LOG_ERROR("File read failure");
		free(buffer_file);
		return retval;
	}

	if (read_cnt != length) {
		LOG_ERROR("Short read");
		free(buffer_file);
		return ERROR_FAIL;
	}

	buffer_flash = malloc(length);
	if (buffer_flash == NULL) {
		LOG_ERROR("Out of memory");
		free(buffer_file);
		return ERROR_FAIL;
	}

	retval = flash_driver_read(p, buffer_flash, offset, length);
	if (retval != ERROR_OK) {
		LOG_ERROR("Flash read error");
		free(buffer_flash);
		free(buffer_file);
		return retval;
	}

	if (duration_measure(&bench) == ERROR_OK)
		command_print(CMD, "read %zd bytes from file %s and flash bank %u"
			" at offset 0x%8.8" PRIx32 " in %fs (%0.3f KiB/s)",
			length, CMD_ARGV[1], p->bank_number, offset,
			duration_elapsed(&bench), duration_kbps(&bench, length));

	differ = memcmp(buffer_file, buffer_flash, length);
	command_print(CMD, "contents %s", differ ? "differ" : "match");
	if (differ) {
		uint32_t t;
		int diffs = 0;
		for (t = 0; t < length; t++) {
			if (buffer_flash[t] == buffer_file[t])
				continue;
			command_print(CMD, "diff %d address 0x%08x. Was 0x%02x instead of 0x%02x",
					diffs, t + offset, buffer_flash[t], buffer_file[t]);
			if (diffs++ >= 127) {
				command_print(CMD, "More than 128 errors, the rest are not printed.");
				break;
			}
			keep_alive();
		}
	}
	free(buffer_flash);
	free(buffer_file);

	return differ ? ERROR_FAIL : ERROR_OK;
}

void flash_set_dirty(void)
{
	struct flash_bank *c;
	int i;

	/* set all flash to require erasing */
	for (c = flash_bank_list(); c; c = c->next) {
		for (i = 0; i < c->num_sectors; i++)
			c->sectors[i].is_erased = 0;
	}
}

COMMAND_HANDLER(handle_flash_padded_value_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *p;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], p->default_padded_value);

	command_print(CMD, "Default padded value set to 0x%" PRIx8 " for flash bank %u", \
			p->default_padded_value, p->bank_number);

	return retval;
}

static const struct command_registration flash_exec_command_handlers[] = {
	{
		.name = "probe",
		.handler = handle_flash_probe_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Identify a flash bank.",
	},
	{
		.name = "info",
		.handler = handle_flash_info_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ['sectors']",
		.help = "Print information about a flash bank.",
	},
	{
		.name = "erase_check",
		.handler = handle_flash_erase_check_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Check erase state of all blocks in a "
			"flash bank.",
	},
	{
		.name = "erase_sector",
		.handler = handle_flash_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id first_sector_num (last_sector_num|'last')",
		.help = "Erase a range of sectors in a flash bank.",
	},
	{
		.name = "erase_address",
		.handler = handle_flash_erase_address_command,
		.mode = COMMAND_EXEC,
		.usage = "['pad'] ['unlock'] address length",
		.help = "Erase flash sectors starting at address and "
			"continuing for length bytes.  If 'pad' is specified, "
			"data outside that range may also be erased: the start "
			"address may be decreased, and length increased, so "
			"that all of the first and last sectors are erased. "
			"If 'unlock' is specified, then the flash is unprotected "
			"before erasing.",

	},
	{
		.name = "fillw",
		.handler = handle_flash_fill_command,
		.mode = COMMAND_EXEC,
		.usage = "address value n",
		.help = "Fill n words with 32-bit value, starting at "
			"word address.  (No autoerase.)",
	},
	{
		.name = "fillh",
		.handler = handle_flash_fill_command,
		.mode = COMMAND_EXEC,
		.usage = "address value n",
		.help = "Fill n halfwords with 16-bit value, starting at "
			"word address.  (No autoerase.)",
	},
	{
		.name = "fillb",
		.handler = handle_flash_fill_command,
		.mode = COMMAND_EXEC,
		.usage = "address value n",
		.help = "Fill n bytes with 8-bit value, starting at "
			"word address.  (No autoerase.)",
	},
	{
		.name = "write_bank",
		.handler = handle_flash_write_bank_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename [offset]",
		.help = "Write binary data from file to flash bank. Allow optional "
			"offset from beginning of the bank (defaults to zero).",
	},
	{
		.name = "write_image",
		.handler = handle_flash_write_image_command,
		.mode = COMMAND_EXEC,
		.usage = "[erase] [unlock] filename [offset [file_type]]",
		.help = "Write an image to flash.  Optionally first unprotect "
			"and/or erase the region to be used.  Allow optional "
			"offset from beginning of bank (defaults to zero)",
	},
	{
		.name = "read_bank",
		.handler = handle_flash_read_bank_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename [offset [length]]",
		.help = "Read binary data from flash bank to file. Allow optional "
			"offset from beginning of the bank (defaults to zero).",
	},
	{
		.name = "verify_bank",
		.handler = handle_flash_verify_bank_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename [offset]",
		.help = "Compare the contents of a file with the contents of the "
			"flash bank. Allow optional offset from beginning of the bank "
			"(defaults to zero).",
	},
	{
		.name = "protect",
		.handler = handle_flash_protect_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id first_block [last_block|'last'] "
			"('on'|'off')",
		.help = "Turn protection on or off for a range of protection "
			"blocks or sectors in a given flash bank. "
			"See 'flash info' output for a list of blocks.",
	},
	{
		.name = "padded_value",
		.handler = handle_flash_padded_value_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id value",
		.help = "Set default flash padded value",
	},
	COMMAND_REGISTRATION_DONE
};

static int flash_init_drivers(struct command_context *cmd_ctx)
{
	if (!flash_bank_list())
		return ERROR_OK;

	struct command *parent = command_find_in_context(cmd_ctx, "flash");
	return register_commands(cmd_ctx, parent, flash_exec_command_handlers);
}

COMMAND_HANDLER(handle_flash_bank_command)
{
	if (CMD_ARGC < 7) {
		LOG_ERROR("usage: flash bank <name> <driver> "
			"<base> <size> <chip_width> <bus_width> <target>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	/* save bank name and advance arguments for compatibility */
	const char *bank_name = *CMD_ARGV++;
	CMD_ARGC--;

	struct target *target = get_target(CMD_ARGV[5]);
	if (target == NULL) {
		LOG_ERROR("target '%s' not defined", CMD_ARGV[5]);
		return ERROR_FAIL;
	}

	const char *driver_name = CMD_ARGV[0];
	const struct flash_driver *driver = flash_driver_find_by_name(driver_name);
	if (NULL == driver) {
		/* no matching flash driver found */
		LOG_ERROR("flash driver '%s' not found", driver_name);
		return ERROR_FAIL;
	}

	/* check the flash bank name is unique */
	if (get_flash_bank_by_name_noprobe(bank_name) != NULL) {
		/* flash bank name already exists  */
		LOG_ERROR("flash bank name '%s' already exists", bank_name);
		return ERROR_FAIL;
	}

	/* register flash specific commands */
	if (NULL != driver->commands) {
		int retval = register_commands(CMD_CTX, NULL,
				driver->commands);
		if (ERROR_OK != retval) {
			LOG_ERROR("couldn't register '%s' commands",
				driver_name);
			return ERROR_FAIL;
		}
	}

	struct flash_bank *c = calloc(1, sizeof(*c));
	c->name = strdup(bank_name);
	c->target = target;
	c->driver = driver;
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[1], c->base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], c->size);
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], c->chip_width);
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[4], c->bus_width);
	c->default_padded_value = c->erased_value = 0xff;
	c->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;

	int retval;
	retval = CALL_COMMAND_HANDLER(driver->flash_bank_command, c);
	if (ERROR_OK != retval) {
		LOG_ERROR("'%s' driver rejected flash bank at " TARGET_ADDR_FMT
				"; usage: %s", driver_name, c->base, driver->usage);
		free(c);
		return retval;
	}

	if (driver->usage == NULL)
		LOG_DEBUG("'%s' driver usage field missing", driver_name);

	flash_bank_add(c);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_flash_banks_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned n = 0;
	for (struct flash_bank *p = flash_bank_list(); p; p = p->next, n++) {
		command_print(CMD, "#%d : %s (%s) at " TARGET_ADDR_FMT ", size 0x%8.8" PRIx32 ", "
			"buswidth %u, chipwidth %u", p->bank_number,
			p->name, p->driver->name, p->base, p->size,
			p->bus_width, p->chip_width);
	}
	return ERROR_OK;
}

static int jim_flash_list(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv,
			"no arguments to 'flash list' command");
		return JIM_ERR;
	}

	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);

	for (struct flash_bank *p = flash_bank_list(); p; p = p->next) {
		Jim_Obj *elem = Jim_NewListObj(interp, NULL, 0);

		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "name", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, p->driver->name, -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "base", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->base));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "size", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->size));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "bus_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->bus_width));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "chip_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->chip_width));

		Jim_ListAppendElement(interp, list, elem);
	}

	Jim_SetResult(interp, list);

	return JIM_OK;
}

COMMAND_HANDLER(handle_flash_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool flash_initialized;
	if (flash_initialized) {
		LOG_INFO("'flash init' has already been called");
		return ERROR_OK;
	}
	flash_initialized = true;

	LOG_DEBUG("Initializing flash devices...");
	return flash_init_drivers(CMD_CTX);
}

static const struct command_registration flash_config_command_handlers[] = {
	{
		.name = "bank",
		.handler = handle_flash_bank_command,
		.mode = COMMAND_CONFIG,
		.usage = "bank_id driver_name base_address size_bytes "
			"chip_width_bytes bus_width_bytes target "
			"[driver_options ...]",
		.help = "Define a new bank with the given name, "
			"using the specified NOR flash driver.",
	},
	{
		.name = "init",
		.mode = COMMAND_CONFIG,
		.handler = handle_flash_init_command,
		.help = "Initialize flash devices.",
		.usage = "",
	},
	{
		.name = "banks",
		.mode = COMMAND_ANY,
		.handler = handle_flash_banks_command,
		.help = "Display table with information about flash banks.",
		.usage = "",
	},
	{
		.name = "list",
		.mode = COMMAND_ANY,
		.jim_handler = jim_flash_list,
		.help = "Returns a list of details about the flash banks.",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration flash_command_handlers[] = {
	{
		.name = "flash",
		.mode = COMMAND_ANY,
		.help = "NOR flash command group",
		.chain = flash_config_command_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int flash_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, flash_command_handlers);
}
