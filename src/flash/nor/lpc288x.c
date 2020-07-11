/***************************************************************************
 *   Copyright (C) 2008 by			                                       *
 *   Karl RobinSod <karl.robinsod@gmail.com>                               *
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

/***************************************************************************
* There are some things to notice
*
* You need to unprotect flash sectors each time you connect the OpenOCD
* Dumping 1MB takes about 60 Seconds
* Full erase (sectors 0-22 inclusive) takes 2-4 seconds
* Writing 1MB takes 88 seconds
*
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>

#define LOAD_TIMER_ERASE        0
#define LOAD_TIMER_WRITE        1

#define FLASH_PAGE_SIZE         512

/* LPC288X control registers */
#define DBGU_CIDR               0x8000507C
/* LPC288X flash registers */
#define F_CTRL                  0x80102000	/* Flash control register R/W 0x5 */
#define F_STAT                  0x80102004	/* Flash status register RO 0x45 */
#define F_PROG_TIME             0x80102008	/* Flash program time register R/W 0 */
#define F_WAIT                  0x80102010	/* Flash read wait state register R/W 0xC004 */
#define F_CLK_TIME              0x8010201C	/* Flash clock divider for 66 kHz generation R/W 0
						 **/
#define F_INTEN_CLR             0x80102FD8	/* Clear interrupt enable bits WO - */
#define F_INTEN_SET             0x80102FDC	/* Set interrupt enable bits WO - */
#define F_INT_STAT              0x80102FE0	/* Interrupt status bits RO 0 */
#define F_INTEN                 0x80102FE4	/* Interrupt enable bits RO 0 */
#define F_INT_CLR               0x80102FE8	/* Clear interrupt status bits WO */
#define F_INT_SET               0x80102FEC	/* Set interrupt status bits WO - */
#define FLASH_PD                0x80005030	/* Allows turning off the Flash memory for power
						 *savings. R/W 1*/
#define FLASH_INIT              0x80005034	/* Monitors Flash readiness, such as recovery from
						 *Power Down mode. R/W -*/

/* F_CTRL bits */
#define FC_CS                   0x0001
#define FC_FUNC                 0x0002
#define FC_WEN                  0x0004
#define FC_RD_LATCH             0x0020
#define FC_PROTECT              0x0080
#define FC_SET_DATA             0x0400
#define FC_RSSL                 0x0800
#define FC_PROG_REQ             0x1000
#define FC_CLR_BUF              0x4000
#define FC_LOAD_REQ             0x8000
/* F_STAT bits */
#define FS_DONE                 0x0001
#define FS_PROGGNT              0x0002
#define FS_RDY                  0x0004
#define FS_ERR                  0x0020
/* F_PROG_TIME */
#define FPT_TIME_MASK   0x7FFF

#define FPT_ENABLE              0x8000
/* F_WAIT */
#define FW_WAIT_STATES_MASK             0x00FF
#define FW_SET_MASK                             0xC000

/* F_CLK_TIME */
#define FCT_CLK_DIV_MASK    0x0FFF

struct lpc288x_flash_bank {
	uint32_t working_area;
	uint32_t working_area_size;

	/* chip id register */
	uint32_t cidr;
	const char *target_name;
	uint32_t cclk;

	uint32_t sector_size_break;
};

static uint32_t lpc288x_wait_status_busy(struct flash_bank *bank, int timeout);
static void lpc288x_load_timer(int erase, struct target *target);
static void lpc288x_set_flash_clk(struct flash_bank *bank);
static uint32_t lpc288x_system_ready(struct flash_bank *bank);

static uint32_t lpc288x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	struct target *target = bank->target;
	do {
		alive_sleep(1);
		timeout--;
		target_read_u32(target, F_STAT, &status);
	} while (((status & FS_DONE) == 0) && timeout);

	if (timeout == 0) {
		LOG_DEBUG("Timedout!");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

/* Read device id register and fill in driver info structure */
static int lpc288x_read_part_info(struct flash_bank *bank)
{
	struct lpc288x_flash_bank *lpc288x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t cidr;

	int i = 0;
	uint32_t offset;

	if (lpc288x_info->cidr == 0x0102100A)
		return ERROR_OK;/* already probed, multiple probes may cause memory leak, not
				 *allowed */

	/* Read and parse chip identification register */
	target_read_u32(target, DBGU_CIDR, &cidr);

	if (cidr != 0x0102100A) {
		LOG_WARNING("Cannot identify target as an LPC288X (%08" PRIx32 ")", cidr);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	lpc288x_info->cidr = cidr;
	lpc288x_info->sector_size_break = 0x000F0000;
	lpc288x_info->target_name = "LPC288x";

	/* setup the sector info... */
	offset = bank->base;
	bank->num_sectors = 23;
	bank->sectors = malloc(sizeof(struct flash_sector) * 23);

	for (i = 0; i < 15; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = 64 * 1024;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}
	for (i = 15; i < 23; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = 8 * 1024;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	return ERROR_OK;
}

/* TODO: Revisit! Is it impossible to read protection status? */
static int lpc288x_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

/* flash_bank LPC288x 0 0 0 0 <target#> <cclk> */
FLASH_BANK_COMMAND_HANDLER(lpc288x_flash_bank_command)
{
	struct lpc288x_flash_bank *lpc288x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	lpc288x_info = malloc(sizeof(struct lpc288x_flash_bank));
	bank->driver_priv = lpc288x_info;

	/* part wasn't probed for info yet */
	lpc288x_info->cidr = 0;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], lpc288x_info->cclk);

	return ERROR_OK;
}

/* The frequency is the AHB clock frequency divided by (CLK_DIV ×3) + 1.
 * This must be programmed such that the Flash Programming clock frequency is 66 kHz ± 20%.
 * AHB = 12 MHz ?
 * 12000000/66000 = 182
 * CLK_DIV = 60 ? */
static void lpc288x_set_flash_clk(struct flash_bank *bank)
{
	uint32_t clk_time;
	struct lpc288x_flash_bank *lpc288x_info = bank->driver_priv;
	clk_time = (lpc288x_info->cclk / 66000) / 3;
	target_write_u32(bank->target, F_CTRL, FC_CS | FC_WEN);
	target_write_u32(bank->target, F_CLK_TIME, clk_time);
}

/* AHB tcyc (in ns) 83 ns
 * LOAD_TIMER_ERASE		FPT_TIME	= ((400,000,000 / AHB tcyc (in ns)) - 2) / 512
 *									= 9412 (9500) (AN10548 9375)
 * LOAD_TIMER_WRITE		FPT_TIME	= ((1,000,000 / AHB tcyc (in ns)) - 2) / 512
 *									= 23 (75) (AN10548 72 - is this wrong?)
 * TODO: Sort out timing calcs ;) */
static void lpc288x_load_timer(int erase, struct target *target)
{
	if (erase == LOAD_TIMER_ERASE)
		target_write_u32(target, F_PROG_TIME, FPT_ENABLE | 9500);
	else
		target_write_u32(target, F_PROG_TIME, FPT_ENABLE | 75);
}

static uint32_t lpc288x_system_ready(struct flash_bank *bank)
{
	struct lpc288x_flash_bank *lpc288x_info = bank->driver_priv;
	if (lpc288x_info->cidr == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	return ERROR_OK;
}

static int lpc288x_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	uint32_t status;
	struct target *target = bank->target;

	status = lpc288x_system_ready(bank);	/* probed? halted? */
	if (status != ERROR_OK)
		return status;

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_INFO("Bad sector range");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* Configure the flash controller timing */
	lpc288x_set_flash_clk(bank);

	for (unsigned int sector = first; sector <= last; sector++) {
		if (lpc288x_wait_status_busy(bank, 1000) != ERROR_OK)
			return ERROR_FLASH_OPERATION_FAILED;

		lpc288x_load_timer(LOAD_TIMER_ERASE, target);

		target_write_u32(target, bank->sectors[sector].offset, 0x00);

		target_write_u32(target, F_CTRL, FC_PROG_REQ | FC_PROTECT | FC_CS);
	}
	if (lpc288x_wait_status_busy(bank, 1000) != ERROR_OK)
		return ERROR_FLASH_OPERATION_FAILED;
	return ERROR_OK;
}

static int lpc288x_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	uint8_t page_buffer[FLASH_PAGE_SIZE];
	uint32_t status, source_offset, dest_offset;
	struct target *target = bank->target;
	uint32_t bytes_remaining = count;
	uint32_t first_sector, last_sector, sector, page;

	/* probed? halted? */
	status = lpc288x_system_ready(bank);
	if (status != ERROR_OK)
		return status;

	/* Initialise search indices */
	first_sector = last_sector = 0xffffffff;

	/* validate the write range... */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if ((offset >= bank->sectors[i].offset) &&
				(offset < (bank->sectors[i].offset + bank->sectors[i].size)) &&
				(first_sector == 0xffffffff)) {
			first_sector = i;
			/* all writes must start on a sector boundary... */
			if (offset % bank->sectors[i].size) {
				LOG_INFO(
					"offset 0x%" PRIx32 " breaks required alignment 0x%" PRIx32 "",
					offset,
					bank->sectors[i].size);
				return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
			}
		}
		if (((offset + count) > bank->sectors[i].offset) &&
				((offset + count) <= (bank->sectors[i].offset + bank->sectors[i].size)) &&
				(last_sector == 0xffffffff))
			last_sector = i;
	}

	/* Range check... */
	if (first_sector == 0xffffffff || last_sector == 0xffffffff) {
		LOG_INFO("Range check failed %" PRIx32 " %" PRIx32 "", offset, count);
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}

	/* Configure the flash controller timing */
	lpc288x_set_flash_clk(bank);

	/* initialise the offsets */
	source_offset = 0;
	dest_offset = 0;

	for (sector = first_sector; sector <= last_sector; sector++) {
		for (page = 0; page < bank->sectors[sector].size / FLASH_PAGE_SIZE; page++) {
			if (bytes_remaining == 0) {
				count = 0;
				memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);
			} else if (bytes_remaining < FLASH_PAGE_SIZE) {
				count = bytes_remaining;
				memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);
				memcpy(page_buffer, &buffer[source_offset], count);
			} else {
				count = FLASH_PAGE_SIZE;
				memcpy(page_buffer, &buffer[source_offset], count);
			}

			/* Wait for flash to become ready */
			if (lpc288x_wait_status_busy(bank, 1000) != ERROR_OK)
				return ERROR_FLASH_OPERATION_FAILED;

			/* fill flash data latches with 1's */
			target_write_u32(target, F_CTRL, FC_CS | FC_SET_DATA | FC_WEN | FC_FUNC);

			target_write_u32(target, F_CTRL, FC_CS | FC_WEN | FC_FUNC);

			if (target_write_buffer(target, offset + dest_offset, FLASH_PAGE_SIZE,
					page_buffer) != ERROR_OK) {
				LOG_INFO("Write to flash buffer failed");
				return ERROR_FLASH_OPERATION_FAILED;
			}

			dest_offset += FLASH_PAGE_SIZE;
			source_offset += count;
			bytes_remaining -= count;

			lpc288x_load_timer(LOAD_TIMER_WRITE, target);

			target_write_u32(target, F_CTRL, FC_PROG_REQ | FC_PROTECT | FC_FUNC |
				FC_CS);
		}
	}

	return ERROR_OK;
}

static int lpc288x_probe(struct flash_bank *bank)
{
	/* we only deal with LPC2888 so flash config is fixed */
	struct lpc288x_flash_bank *lpc288x_info = bank->driver_priv;
	int retval;

	if (lpc288x_info->cidr != 0)
		return ERROR_OK;/* already probed */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = lpc288x_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static int lpc288x_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	int status;
	uint32_t value;
	struct target *target = bank->target;

	/* probed? halted? */
	status = lpc288x_system_ready(bank);
	if (status != ERROR_OK)
		return status;

	if ((last < first) || (last >= bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	/* Configure the flash controller timing */
	lpc288x_set_flash_clk(bank);

	for (unsigned int lockregion = first; lockregion <= last; lockregion++) {
		if (set) {
			/* write an odd value to base address to protect... */
			value = 0x01;
		} else {
			/* write an even value to base address to unprotect... */
			value = 0x00;
		}
		target_write_u32(target, bank->sectors[lockregion].offset, value);
		target_write_u32(target, F_CTRL, FC_LOAD_REQ | FC_PROTECT | FC_WEN | FC_FUNC |
			FC_CS);
	}

	return ERROR_OK;
}

const struct flash_driver lpc288x_flash = {
	.name = "lpc288x",
	.flash_bank_command = lpc288x_flash_bank_command,
	.erase = lpc288x_erase,
	.protect = lpc288x_protect,
	.write = lpc288x_write,
	.read = default_flash_read,
	.probe = lpc288x_probe,
	.auto_probe = lpc288x_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = lpc288x_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
