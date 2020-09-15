/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This implements the flash driver for the NXP PN7462.
 * Copyright (C) 2020 Rick Veens <rickveens92@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <target/algorithm.h>
#include <target/arm_opcodes.h>
#include <target/armv7m.h>


/**
 * @file
 * flash programming support for NXP PN7462 device.
 *
 */

#define PN7462_FLASH_START_ADDRESS  0x00203000
#define PN7462_FLASH_END_ADDRESS    (PN7462_FLASH_START_ADDRESS + (158*1024) - 1)
#define PN7462_FLASH_PAGE_SIZE      128
#define PN7462_WITHIN_PAGEFLASH(ADD)			\
	((ADD) >= PN7462_FLASH_START_ADDRESS		\
		&&										\
		(ADD) <= PN7462_FLASH_END_ADDRESS)


#define PN7462_EEPROM_MEM_START_ADDRESS     0x00201000UL
#define PN7462_EEPROM_DATA_END_ADDRESS      0x00201FFFUL
#define PN7462_EEPROM_PAGE_SIZE             64
#define PN7462_WITHIN_EEPROM(ADD)				\
	((ADD) >= PN7462_EEPROM_MEM_START_ADDRESS	\
		&&										\
		(ADD) <= PN7462_EEPROM_DATA_END_ADDRESS	\
	)

#define EE_DYN                                                               (0x00200004UL)
#define EE_DYN_PROG_0_COD_POS                                                BIT(16)
#define EE_DYN_PROG_1_COD_POS                                                BIT(20)
#define EE_DYN_PROG_DAT_POS                                                  BIT(0)

#define EE_STAT_COD                                                          (0x0020000CUL)
#define EE_STAT_COD_HVERR_0_COD_POS                                          BIT(0)
#define EE_STAT_COD_HVERR_1_COD_POS                                          BIT(13)

#define EE_STAT_DAT                                                          (0x00200008UL)
#define EE_STAT_DAT_HVERR_DAT_POS                                            BIT(0)


/*
 * flash bank pn7462 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(pn7462_flash_bank_command)
{
	if (CMD_ARGC < 5)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static int pn7462_write_flash(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	int retval;

	if (!PN7462_WITHIN_PAGEFLASH(bank->base + offset)) {
		LOG_ERROR(
			"pn7462_write_flash: given address to write (" TARGET_ADDR_FMT ") not within flash memory address space.",
			bank->base + offset);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (count % PN7462_FLASH_PAGE_SIZE != 0) {
		LOG_ERROR("pn7462_write_flash: given size (%u) is not a multiple of %u.", count, PN7462_FLASH_PAGE_SIZE);

		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	uint32_t bytes_remaining = count;
	uint32_t index = 0;

	while (bytes_remaining >= PN7462_FLASH_PAGE_SIZE) {
		retval = target_write_buffer(bank->target, bank->base + offset + index, PN7462_FLASH_PAGE_SIZE, buffer + index);
		if (retval != ERROR_OK)
			return retval;

		index += PN7462_FLASH_PAGE_SIZE;
		bytes_remaining -= PN7462_FLASH_PAGE_SIZE;

		retval = target_write_u32(bank->target, EE_DYN,
			(EE_DYN_PROG_0_COD_POS | EE_DYN_PROG_1_COD_POS));
		if (retval != ERROR_OK)
			return retval;

		uint32_t eestatcod;
		retval = target_read_u32(bank->target, EE_STAT_COD, &eestatcod);
		if (retval != ERROR_OK)
			return retval;

		if ((eestatcod & EE_STAT_COD_HVERR_0_COD_POS) ||
			(eestatcod & EE_STAT_COD_HVERR_1_COD_POS))
			return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int pn7462_write_eeprom(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	int retval;

	if (!PN7462_WITHIN_EEPROM(bank->base + offset)) {
		LOG_ERROR(
			"pn7462_write_eeprom: given address to write (" TARGET_ADDR_FMT ") not within eeprom memory address space.",
			bank->base + offset);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (count % PN7462_EEPROM_PAGE_SIZE != 0) {
		LOG_ERROR("pn7462_write_eeprom: given size (%u) is not a multiple of %u.", count, PN7462_EEPROM_PAGE_SIZE);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	uint32_t bytes_remaining = count;
	uint32_t index = 0;

	while (bytes_remaining >= PN7462_EEPROM_PAGE_SIZE) {
		retval = target_write_buffer(bank->target,
									 bank->base + offset + index,
									 PN7462_EEPROM_PAGE_SIZE,
									 buffer + index);
		if (retval != ERROR_OK)
			return retval;

		index += PN7462_EEPROM_PAGE_SIZE;
		bytes_remaining -= PN7462_EEPROM_PAGE_SIZE;

		retval = target_write_u32(bank->target, EE_DYN, EE_DYN_PROG_DAT_POS);
		if (retval != ERROR_OK)
			return retval;

		uint32_t eestatdat;
		retval = target_read_u32(bank->target, EE_STAT_DAT, &eestatdat);
		if (retval != ERROR_OK)
			return retval;

		if (eestatdat & EE_STAT_DAT_HVERR_DAT_POS) {
			LOG_ERROR("pn7462_write_eeprom: EE_STAT_DAT_HVERR_DAT_POS error");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int pn7462_write(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}


	LOG_DEBUG("pn7462_write called with address " TARGET_ADDR_FMT ", count:%d. isflash: %d. iseeprom: %d",
		bank->base + offset,
		count,
		PN7462_WITHIN_PAGEFLASH(bank->base + offset),
		PN7462_WITHIN_EEPROM(bank->base + offset)
		);

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (PN7462_WITHIN_PAGEFLASH(bank->base + offset))
		return pn7462_write_flash(bank, buffer, offset, count);
	else if (PN7462_WITHIN_EEPROM(bank->base + offset))
		return pn7462_write_eeprom(bank, buffer, offset, count);
	else
		return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int pn7462_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int lverr = ERROR_OK;

	/* if first and last equals 0, erase eeprom
	 * if first and last equals 1, erase flash */

	if (first == 0 && last == 0) {
		const uint8_t empty_page[PN7462_FLASH_PAGE_SIZE];

		if (!(bank->bus_width == PN7462_EEPROM_PAGE_SIZE || bank->bus_width == PN7462_FLASH_PAGE_SIZE))
			return ERROR_FLASH_SECTOR_NOT_ERASED;

		for (unsigned i = 0; i * bank->bus_width < bank->sectors[0].size; i++) {
			lverr = pn7462_write(bank, empty_page, i * bank->bus_width, bank->bus_width);
			if (lverr != ERROR_OK)
				return lverr;
		}
	} else
		return ERROR_FLASH_SECTOR_NOT_ERASED;

	return ERROR_OK;
}

static int pn7462_probe(struct flash_bank *bank)
{
	/* the flash controller hides the sectors. */

	/* create one sector for the flash memory and eeprom */
	bank->sectors = malloc(sizeof(struct flash_sector));

	if (bank->sectors == NULL)
		return ERROR_FAIL;

	bank->sectors[0].is_protected = 0;
	bank->sectors[0].size = bank->size;
	bank->sectors[0].offset = 0;
	bank->sectors[0].is_erased = -1;

	bank->num_sectors = 1;

	return ERROR_OK;
}

static int pn7462_erase_check(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

static int get_pn7462_info(struct flash_bank *bank, char *buf, int buf_size)
{
	if (PN7462_WITHIN_PAGEFLASH(bank->base))
		snprintf(buf, buf_size, "pn7462 flash");
	else if (PN7462_WITHIN_EEPROM(bank->base))
		snprintf(buf, buf_size, "pn7462 eeprom");

	return ERROR_OK;
}

const struct flash_driver pn7462_flash = {
	.name = "pn7462",
	.flash_bank_command = pn7462_flash_bank_command,
	.erase = pn7462_erase,
	.write = pn7462_write,
	.read = default_flash_read,
	.probe = pn7462_probe,
	.auto_probe = pn7462_probe,
	.erase_check = pn7462_erase_check,
	.info = get_pn7462_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
