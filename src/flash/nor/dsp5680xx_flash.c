/***************************************************************************
 *   Copyright (C) 2011 by Rodrigo L. Rosa                                 *
 *   rodrigorosa.LG@gmail.com                                              *
 *                                                                         *
 *   Based on a file written by:                                           *
 *   Kevin McGuire                                                         *
 *   Marcel Wijlaars                                                       *
 *   Michael Ashton                                                        *
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

/**
 * @file   dsp5680xx_flash.c
 * @author Rodrigo L. Rosa <rodrigorosa.LG@gmail.com>
 * @date   Thu Jun  9 18:21:58 2011
 *
 * @brief  This file implements the basic functions to run flashing commands
 * from the TCL interface.
 * It allows the user to flash the Freescale 5680xx DSP.
 *
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/dsp5680xx.h>

static int dsp5680xx_build_sector_list(struct flash_bank *bank)
{
	uint32_t offset = HFM_FLASH_BASE_ADDR;

	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (unsigned int i = 0; i < bank->num_sectors; ++i) {
		bank->sectors[i].offset = i * HFM_SECTOR_SIZE;
		bank->sectors[i].size = HFM_SECTOR_SIZE;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}
	LOG_USER("%s not tested yet.", __func__);
	return ERROR_OK;

}

/* flash bank dsp5680xx 0 0 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(dsp5680xx_flash_bank_command)
{
	bank->base = HFM_FLASH_BASE_ADDR;
	bank->size = HFM_SIZE_BYTES; /* top 4k not accessible */
	bank->num_sectors = HFM_SECTOR_COUNT;
	dsp5680xx_build_sector_list(bank);

	return ERROR_OK;
}

/**
 * A memory mapped register (PROT) holds information regarding sector protection.
 * Protection refers to undesired core access.
 * The value in this register is loaded from flash upon reset.
 *
 * @param bank
 *
 * @return
 */
static int dsp5680xx_flash_protect_check(struct flash_bank *bank)
{
	int retval = ERROR_OK;

	uint16_t protected = 0;

	retval = dsp5680xx_f_protect_check(bank->target, &protected);
	if (retval != ERROR_OK) {
		for (int i = 0; i < HFM_SECTOR_COUNT; i++)
			bank->sectors[i].is_protected = -1;
		return ERROR_OK;
	}
	for (int i = 0; i < HFM_SECTOR_COUNT / 2; i++) {
		if (protected & 1) {
			bank->sectors[2 * i].is_protected = 1;
			bank->sectors[2 * i + 1].is_protected = 1;
		} else {
			bank->sectors[2 * i].is_protected = 0;
			bank->sectors[2 * i + 1].is_protected = 0;
		}
		protected = (protected >> 1);
	}
	return retval;
}

/**
 * Protection functionality is not implemented.
 * The current implementation applies/removes security on the chip.
 * The chip is effectively secured/unsecured after the first reset
 * following the execution of this function.
 *
 * @param bank
 * @param set Apply or remove security on the chip.
 * @param first This parameter is ignored.
 * @param last This parameter is ignored.
 *
 * @return
 */
static int dsp5680xx_flash_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
/**
 * This applies security to flash module after next reset, it does
 * not actually apply protection (protection refers to undesired access from the core)
 */
	int retval;

	if (set)
		retval = dsp5680xx_f_lock(bank->target);
	else {
		retval = dsp5680xx_f_unlock(bank->target);
		if (retval == ERROR_OK) {
			/* mark all as erased */
			for (int i = 0; i <= (HFM_SECTOR_COUNT - 1); i++)
				/* FM does not recognize it as erased if erased via JTAG. */
				bank->sectors[i].is_erased = 1;
		}
	}
	return retval;
}

/**
 * The dsp5680xx use word addressing. The "/2" that appear in the following code
 * are a workaround for the fact that OpenOCD uses byte addressing.
 *
 * @param bank
 * @param buffer Data to write to flash.
 * @param offset
 * @param count In bytes (2 bytes per address).
 *
 * @return
 */
static int dsp5680xx_flash_write(struct flash_bank *bank, const uint8_t *buffer,
				 uint32_t offset, uint32_t count)
{
	int retval;

	if ((offset + count / 2) > bank->size) {
		LOG_ERROR("%s: Flash bank cannot fit data.", __func__);
		return ERROR_FAIL;
	}
	if (offset % 2) {
		/**
		 * Writing to odd addresses not supported.
		 * This chip uses word addressing, Openocd only supports byte addressing.
		 * The workaround results in disabling writing to odd byte addresses
		 */
		LOG_ERROR("%s: Writing to odd addresses not supported for this target", __func__);
		return ERROR_FAIL;
	}
	retval = dsp5680xx_f_wr(bank->target, buffer, bank->base + offset / 2, count, 0);
	uint32_t addr_word;

	for (addr_word = bank->base + offset / 2; addr_word < count / 2;
			addr_word += (HFM_SECTOR_SIZE / 2)) {
		if (retval == ERROR_OK)
			bank->sectors[addr_word / (HFM_SECTOR_SIZE / 2)].is_erased = 0;
		else
			bank->sectors[addr_word / (HFM_SECTOR_SIZE / 2)].is_erased = -1;
	}
	return retval;
}

static int dsp5680xx_probe(struct flash_bank *bank)
{
	LOG_DEBUG("%s not implemented", __func__);
	return ERROR_OK;
}

/**
 * The flash module (FM) on the dsp5680xx supports both individual sector
 * and mass erase of the flash memory.
 * If this function is called with @a first == @a last == 0 or if @a first is the
 * first sector (#0) and @a last is the last sector then the mass erase command
 * is executed (much faster than erasing each sector individually).
 *
 * @param bank
 * @param first
 * @param last
 *
 * @return
 */
static int dsp5680xx_flash_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval;

	retval = dsp5680xx_f_erase(bank->target, (uint32_t) first, (uint32_t) last);
	if ((!(first | last)) || ((first == 0) && (last == (HFM_SECTOR_COUNT - 1))))
		last = HFM_SECTOR_COUNT - 1;
	if (retval == ERROR_OK)
		for (unsigned int i = first; i <= last; i++)
			bank->sectors[i].is_erased = 1;
	else
		/**
		 * If an error occurred unknown status
		 *is set even though some sector could have been correctly erased.
		 */
		for (unsigned int i = first; i <= last; i++)
			bank->sectors[i].is_erased = -1;
	return retval;
}

/**
 * The flash module (FM) on the dsp5680xx support a blank check function.
 * This function executes the FM's blank check functionality on each and every sector.
 *
 * @param bank
 *
 * @return
 */
static int dsp5680xx_flash_erase_check(struct flash_bank *bank)
{
	int retval = ERROR_OK;

	uint8_t erased = 0;

	uint32_t i;

	for (i = 0; i < HFM_SECTOR_COUNT; i++) {
		if (bank->sectors[i].is_erased == -1) {
			retval = dsp5680xx_f_erase_check(bank->target, &erased, i);
			if (retval != ERROR_OK) {
				bank->sectors[i].is_erased = -1;
			} else {
				if (erased)
					bank->sectors[i].is_erased = 1;
				else
					bank->sectors[i].is_erased = 0;
			}
		}
	}
	return retval;
}

const struct flash_driver dsp5680xx_flash = {
	.name = "dsp5680xx_flash",
	.flash_bank_command = dsp5680xx_flash_bank_command,
	.erase = dsp5680xx_flash_erase,
	.protect = dsp5680xx_flash_protect,
	.write = dsp5680xx_flash_write,
	/* .read = default_flash_read, */
	.probe = dsp5680xx_probe,
	.auto_probe = dsp5680xx_probe,
	.erase_check = dsp5680xx_flash_erase_check,
	.protect_check = dsp5680xx_flash_protect_check,
};
