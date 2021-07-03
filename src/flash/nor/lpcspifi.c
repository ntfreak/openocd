/***************************************************************************
 *   Copyright (C) 2012 by George Harris                                   *
 *   george@luminairecoffee.com                                            *
 *																		   *
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
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* Offsets from ssp_base into config & data registers */
#define SSP_CR0		(0x00)  /* Control register 0 */
#define SSP_CR1		(0x04)  /* Control register 1 */
#define SSP_DATA	(0x08)  /* Data register (TX and RX) */
#define SSP_SR		(0x0C)  /* Status register */
#define SSP_CPSR	(0x10)  /* Clock prescale register */

/* Status register fields */
#define SSP_BSY		(0x00000010)

/* Timeout in ms */
#define SSP_CMD_TIMEOUT   (100)
#define SSP_PROBE_TIMEOUT (100)
#define SSP_MAX_TIMEOUT  (3000)

/* Size of the stack to alloc in the working area for the execution of
 * the ROM spifi_init() function */
#define SPIFI_INIT_STACK_SIZE  512

struct lpcspifi_flash_bank {
	bool probed;
	uint32_t ssp_base;
	uint32_t io_base;
	uint32_t ioconfig_base;
	uint32_t bank_num;
	uint32_t max_spi_clock_mhz;
	const struct flash_device *dev;
};

/* flash_bank lpcspifi <base> <size> <chip_width> <bus_width> <target>
 */
FLASH_BANK_COMMAND_HANDLER(lpcspifi_flash_bank_command)
{
	struct lpcspifi_flash_bank *lpcspifi_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	lpcspifi_info = malloc(sizeof(struct lpcspifi_flash_bank));
	if (!lpcspifi_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = lpcspifi_info;
	lpcspifi_info->probed = false;

	return ERROR_OK;
}

static inline int ioconfig_write_reg(struct target *target, uint32_t ioconfig_base, uint32_t offset, uint32_t value)
{
	return target_write_u32(target, ioconfig_base + offset, value);
}

static inline int ssp_write_reg(struct target *target, uint32_t ssp_base, uint32_t offset, uint32_t value)
{
	return target_write_u32(target, ssp_base + offset, value);
}

static inline int io_write_reg(struct target *target, uint32_t io_base, uint32_t offset, uint32_t value)
{
	return target_write_u32(target, io_base + offset, value);
}

static inline int ssp_read_reg(struct target *target, uint32_t ssp_base, uint32_t offset, uint32_t *value)
{
	return target_read_u32(target, ssp_base + offset, value);
}

static int ssp_setcs(struct target *target, uint32_t io_base, unsigned int value)
{
	return io_write_reg(target, io_base, 0x12ac, value ? 0xffffffff : 0x00000000);
}

/* Poll the SSP busy flag. When this comes back as 0, the transfer is complete
 * and the controller is idle. */
static int poll_ssp_busy(struct target *target, uint32_t ssp_base, int timeout)
{
	int64_t endtime;
	uint32_t value;
	int retval;

	retval = ssp_read_reg(target, ssp_base, SSP_SR, &value);
	if ((retval == ERROR_OK) && (value & SSP_BSY) == 0)
		return ERROR_OK;
	else if (retval != ERROR_OK)
		return retval;

	endtime = timeval_ms() + timeout;
	do {
		alive_sleep(1);
		retval = ssp_read_reg(target, ssp_base, SSP_SR, &value);
		if ((retval == ERROR_OK) && (value & SSP_BSY) == 0)
			return ERROR_OK;
		else if (retval != ERROR_OK)
			return retval;
	} while (timeval_ms() < endtime);

	LOG_ERROR("Timeout while polling BSY");
	return ERROR_FLASH_OPERATION_FAILED;
}

/* Un-initialize the ssp module and initialize the SPIFI module */
static int lpcspifi_set_hw_mode(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	uint32_t ssp_base = lpcspifi_info->ssp_base;
	struct armv7m_algorithm armv7m_info;
	struct working_area *spifi_init_algorithm;
	struct reg_param reg_params[2];
	int retval = ERROR_OK;

	LOG_DEBUG("Uninitializing LPC43xx SSP");
	/* Turn off the SSP module */
	retval = ssp_write_reg(target, ssp_base, SSP_CR1, 0x00000000);
	if (retval != ERROR_OK)
		return retval;

	/* see contrib/loaders/flash/lpcspifi_init.S for src */
	static const uint8_t spifi_init_code[] = {
		0x4f, 0xea, 0x00, 0x08, 0xa1, 0xb0, 0x00, 0xaf,
		0x4f, 0xf4, 0xc0, 0x43, 0xc4, 0xf2, 0x08, 0x03,
		0x4f, 0xf0, 0xf3, 0x02, 0xc3, 0xf8, 0x8c, 0x21,
		0x4f, 0xf4, 0xc0, 0x43, 0xc4, 0xf2, 0x08, 0x03,
		0x4f, 0xf4, 0xc0, 0x42, 0xc4, 0xf2, 0x08, 0x02,
		0x4f, 0xf4, 0xc0, 0x41, 0xc4, 0xf2, 0x08, 0x01,
		0x4f, 0xf4, 0xc0, 0x40, 0xc4, 0xf2, 0x08, 0x00,
		0x4f, 0xf0, 0xd3, 0x04, 0xc0, 0xf8, 0x9c, 0x41,
		0x20, 0x46, 0xc1, 0xf8, 0x98, 0x01, 0x01, 0x46,
		0xc2, 0xf8, 0x94, 0x11, 0xc3, 0xf8, 0x90, 0x11,
		0x4f, 0xf4, 0xc0, 0x43, 0xc4, 0xf2, 0x08, 0x03,
		0x4f, 0xf0, 0x13, 0x02, 0xc3, 0xf8, 0xa0, 0x21,
		0x40, 0xf2, 0x18, 0x13, 0xc1, 0xf2, 0x40, 0x03,
		0x1b, 0x68, 0x1c, 0x68, 0x40, 0xf2, 0xb4, 0x30,
		0xc1, 0xf2, 0x00, 0x00, 0x4f, 0xf0, 0x03, 0x01,
		0x4f, 0xf0, 0xc0, 0x02, 0x4f, 0xea, 0x08, 0x03,
		0xa0, 0x47, 0x00, 0xf0, 0x00, 0xb8, 0x00, 0xbe
	};

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;


	LOG_DEBUG("Allocating working area for SPIFI init algorithm");
	/* Get memory for spifi initialization algorithm */
	retval = target_alloc_working_area(target, sizeof(spifi_init_code)
		+ SPIFI_INIT_STACK_SIZE, &spifi_init_algorithm);
	if (retval != ERROR_OK) {
		LOG_ERROR("Insufficient working area to initialize SPIFI "
			"module. You must allocate at least %zdB of working "
			"area in order to use this driver.",
			sizeof(spifi_init_code) + SPIFI_INIT_STACK_SIZE
		);

		return retval;
	}

	LOG_DEBUG("Writing algorithm to working area at " TARGET_ADDR_FMT,
		spifi_init_algorithm->address);
	/* Write algorithm to working area */
	retval = target_write_buffer(target,
		spifi_init_algorithm->address,
		sizeof(spifi_init_code),
		spifi_init_code
	);

	if (retval != ERROR_OK) {
		target_free_working_area(target, spifi_init_algorithm);
		return retval;
	}

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);		/* spifi clk speed */
	/* the spifi_init() rom API makes use of the stack */
	init_reg_param(&reg_params[1], "sp", 32, PARAM_OUT);

	/* For now, the algorithm will set up the SPIFI module
	 * @ the IRC clock speed. In the future, it could be made
	 * a bit smarter to use other clock sources if the user has
	 * already configured them in order to speed up memory-
	 * mapped reads. */
	buf_set_u32(reg_params[0].value, 0, 32, 12);
	/* valid stack pointer */
	buf_set_u32(reg_params[1].value, 0, 32, (spifi_init_algorithm->address +
		sizeof(spifi_init_code) + SPIFI_INIT_STACK_SIZE) & ~7UL);

	/* Run the algorithm */
	LOG_DEBUG("Running SPIFI init algorithm");
	retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
		spifi_init_algorithm->address,
		spifi_init_algorithm->address + sizeof(spifi_init_code) - 2,
		1000, &armv7m_info);

	if (retval != ERROR_OK)
		LOG_ERROR("Error executing SPIFI init algorithm");

	target_free_working_area(target, spifi_init_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	return retval;
}

/* Initialize the ssp module */
static int lpcspifi_set_sw_mode(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	uint32_t ssp_base = lpcspifi_info->ssp_base;
	uint32_t io_base = lpcspifi_info->io_base;
	uint32_t ioconfig_base = lpcspifi_info->ioconfig_base;
	int retval = ERROR_OK;

	/* Re-initialize SPIFI. There are a couple of errata on this, so this makes
	sure that nothing's in an unhappy state. */
	retval = lpcspifi_set_hw_mode(bank);

	/* If we couldn't initialize hardware mode, don't even bother continuing */
	if (retval != ERROR_OK)
		return retval;

	/* Initialize the pins */
	retval = ioconfig_write_reg(target, ioconfig_base, 0x194, 0x00000040);
	if (retval == ERROR_OK)
		retval = ioconfig_write_reg(target, ioconfig_base, 0x1a0, 0x00000044);
	if (retval == ERROR_OK)
		retval = ioconfig_write_reg(target, ioconfig_base, 0x190, 0x00000040);
	if (retval == ERROR_OK)
		retval = ioconfig_write_reg(target, ioconfig_base, 0x19c, 0x000000ed);
	if (retval == ERROR_OK)
		retval = ioconfig_write_reg(target, ioconfig_base, 0x198, 0x000000ed);
	if (retval == ERROR_OK)
		retval = ioconfig_write_reg(target, ioconfig_base, 0x18c, 0x000000ea);

	/* Set CS high & as an output */
	if (retval == ERROR_OK)
		retval = io_write_reg(target, io_base, 0x12ac, 0xffffffff);
	if (retval == ERROR_OK)
		retval = io_write_reg(target, io_base, 0x2014, 0x00000800);

	/* Initialize the module */
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_CR0, 0x00000007);
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_CR1, 0x00000000);
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_CPSR, 0x00000008);
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_CR1, 0x00000002);

	/* If something didn't work out, attempt to return SPIFI to HW mode */
	if (retval != ERROR_OK)
		lpcspifi_set_hw_mode(bank);

	return retval;
}

/* Read the status register of the external SPI flash chip. */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	uint32_t ssp_base = lpcspifi_info->ssp_base;
	uint32_t io_base = lpcspifi_info->io_base;
	uint32_t value;
	int retval = ERROR_OK;

	retval = ssp_setcs(target, io_base, 0);
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, SPIFLASH_READ_STATUS);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);
	/* Dummy write to clock in the register */
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, 0x00);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_setcs(target, io_base, 1);

	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);
	if (retval == ERROR_OK)
		*status = value;

	return retval;
}

/* check for BSY bit in flash status register */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval;
	int64_t endtime;

	endtime = timeval_ms() + timeout;
	do {
		/* read flash status register */
		retval = read_status_reg(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & SPIFLASH_BSY_BIT) == 0)
			return ERROR_OK;
		alive_sleep(1);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout waiting for flash to finish write/erase operation");
	return ERROR_FAIL;
}

/* Send "write enable" command to SPI flash chip. */
static int lpcspifi_write_enable(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	uint32_t ssp_base = lpcspifi_info->ssp_base;
	uint32_t io_base = lpcspifi_info->io_base;
	uint32_t status, value;
	int retval = ERROR_OK;

	retval = ssp_setcs(target, io_base, 0);
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, SPIFLASH_WRITE_ENABLE);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);
	if (retval == ERROR_OK)
		retval = ssp_setcs(target, io_base, 1);

	/* read flash status register */
	if (retval == ERROR_OK)
		retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		return retval;

	/* Check write enabled */
	if ((status & SPIFLASH_WE_BIT) == 0) {
		LOG_ERROR("Cannot enable write to flash. Status=0x%08" PRIx32, status);
		return ERROR_FAIL;
	}

	return retval;
}

static int lpcspifi_bulk_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	uint32_t ssp_base = lpcspifi_info->ssp_base;
	uint32_t io_base = lpcspifi_info->io_base;
	uint32_t value;
	int retval = ERROR_OK;

	if (lpcspifi_info->dev->chip_erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	retval = lpcspifi_set_sw_mode(bank);

	if (retval == ERROR_OK)
		retval = lpcspifi_write_enable(bank);

	/* send SPI command "bulk erase" */
	if (retval == ERROR_OK)
		ssp_setcs(target, io_base, 0);
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, lpcspifi_info->dev->chip_erase_cmd);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);
	if (retval == ERROR_OK)
		retval = ssp_setcs(target, io_base, 1);

	/* poll flash BSY for self-timed bulk erase */
	if (retval == ERROR_OK)
		retval = wait_till_ready(bank, bank->num_sectors*SSP_MAX_TIMEOUT);

	return retval;
}

static int lpcspifi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	struct working_area *erase_algorithm;
	int retval = ERROR_OK;

	LOG_DEBUG("erase from sector %u to sector %u", first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(lpcspifi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	/* If we're erasing the entire chip and the flash supports
	 * it, use a bulk erase instead of going sector-by-sector. */
	if (first == 0 && last == (bank->num_sectors - 1)
		&& lpcspifi_info->dev->chip_erase_cmd != lpcspifi_info->dev->erase_cmd) {
		LOG_DEBUG("Chip supports the bulk erase command."
		" Will use bulk erase instead of sector-by-sector erase.");
		retval = lpcspifi_bulk_erase(bank);

		if (retval == ERROR_OK) {
			retval = lpcspifi_set_hw_mode(bank);
			return retval;
		} else
			LOG_WARNING("Bulk flash erase failed. Falling back to sector-by-sector erase.");
	}

	if (lpcspifi_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	retval = lpcspifi_set_hw_mode(bank);
	if (retval != ERROR_OK)
		return retval;

	/* see contrib/loaders/flash/lpcspifi_erase.S for src */
	static const uint8_t lpcspifi_flash_erase_code[] = {
		0x4f, 0xf4, 0xc0, 0x4a, 0xc4, 0xf2, 0x08, 0x0a,
		0x4f, 0xf0, 0xea, 0x08, 0xca, 0xf8, 0x8c, 0x81,
		0x4f, 0xf0, 0x40, 0x08, 0xca, 0xf8, 0x90, 0x81,
		0x4f, 0xf0, 0x40, 0x08, 0xca, 0xf8, 0x94, 0x81,
		0x4f, 0xf0, 0xed, 0x08, 0xca, 0xf8, 0x98, 0x81,
		0x4f, 0xf0, 0xed, 0x08, 0xca, 0xf8, 0x9c, 0x81,
		0x4f, 0xf0, 0x44, 0x08, 0xca, 0xf8, 0xa0, 0x81,
		0x4f, 0xf4, 0xc0, 0x4a, 0xc4, 0xf2, 0x0f, 0x0a,
		0x4f, 0xf4, 0x00, 0x68, 0xca, 0xf8, 0x14, 0x80,
		0x4f, 0xf4, 0x80, 0x4a, 0xc4, 0xf2, 0x0f, 0x0a,
		0x4f, 0xf0, 0xff, 0x08, 0xca, 0xf8, 0xab, 0x80,
		0x4f, 0xf0, 0x00, 0x0a, 0xc4, 0xf2, 0x05, 0x0a,
		0x4f, 0xf0, 0x00, 0x08, 0xc0, 0xf2, 0x00, 0x18,
		0xca, 0xf8, 0x94, 0x80, 0x4f, 0xf4, 0x00, 0x5a,
		0xc4, 0xf2, 0x05, 0x0a, 0x4f, 0xf0, 0x01, 0x08,
		0xca, 0xf8, 0x00, 0x87, 0x4f, 0xf4, 0x40, 0x5a,
		0xc4, 0xf2, 0x08, 0x0a, 0x4f, 0xf0, 0x07, 0x08,
		0xca, 0xf8, 0x00, 0x80, 0x4f, 0xf0, 0x02, 0x08,
		0xca, 0xf8, 0x10, 0x80, 0xca, 0xf8, 0x04, 0x80,
		0x00, 0xf0, 0x52, 0xf8, 0x4f, 0xf0, 0x06, 0x09,
		0x00, 0xf0, 0x3b, 0xf8, 0x00, 0xf0, 0x48, 0xf8,
		0x00, 0xf0, 0x4a, 0xf8, 0x4f, 0xf0, 0x05, 0x09,
		0x00, 0xf0, 0x33, 0xf8, 0x4f, 0xf0, 0x00, 0x09,
		0x00, 0xf0, 0x2f, 0xf8, 0x00, 0xf0, 0x3c, 0xf8,
		0x19, 0xf0, 0x02, 0x0f, 0x00, 0xf0, 0x45, 0x80,
		0x00, 0xf0, 0x3a, 0xf8, 0x4f, 0xea, 0x02, 0x09,
		0x00, 0xf0, 0x23, 0xf8, 0x4f, 0xea, 0x10, 0x49,
		0x00, 0xf0, 0x1f, 0xf8, 0x4f, 0xea, 0x10, 0x29,
		0x00, 0xf0, 0x1b, 0xf8, 0x4f, 0xea, 0x00, 0x09,
		0x00, 0xf0, 0x17, 0xf8, 0x00, 0xf0, 0x24, 0xf8,
		0x00, 0xf0, 0x26, 0xf8, 0x4f, 0xf0, 0x05, 0x09,
		0x00, 0xf0, 0x0f, 0xf8, 0x4f, 0xf0, 0x00, 0x09,
		0x00, 0xf0, 0x0b, 0xf8, 0x00, 0xf0, 0x18, 0xf8,
		0x19, 0xf0, 0x01, 0x0f, 0x7f, 0xf4, 0xf0, 0xaf,
		0x01, 0x39, 0xf9, 0xb1, 0x18, 0x44, 0xff, 0xf7,
		0xbf, 0xbf, 0x4f, 0xf4, 0x40, 0x5a, 0xc4, 0xf2,
		0x08, 0x0a, 0xca, 0xf8, 0x08, 0x90, 0xda, 0xf8,
		0x0c, 0x90, 0x19, 0xf0, 0x10, 0x0f, 0x7f, 0xf4,
		0xfa, 0xaf, 0xda, 0xf8, 0x08, 0x90, 0x70, 0x47,
		0x4f, 0xf0, 0xff, 0x08, 0x00, 0xf0, 0x02, 0xb8,
		0x4f, 0xf0, 0x00, 0x08, 0x4f, 0xf4, 0x80, 0x4a,
		0xc4, 0xf2, 0x0f, 0x0a, 0xca, 0xf8, 0xab, 0x80,
		0x70, 0x47, 0x00, 0x20, 0x00, 0xbe, 0xff, 0xff
	};

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;


	/* Get memory for spifi initialization algorithm */
	retval = target_alloc_working_area(target, sizeof(lpcspifi_flash_erase_code),
		&erase_algorithm);
	if (retval != ERROR_OK) {
		LOG_ERROR("Insufficient working area. You must configure a working"
			" area of at least %zdB in order to erase SPIFI flash.",
			sizeof(lpcspifi_flash_erase_code));
		return retval;
	}

	/* Write algorithm to working area */
	retval = target_write_buffer(target, erase_algorithm->address,
		sizeof(lpcspifi_flash_erase_code), lpcspifi_flash_erase_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, erase_algorithm);
		return retval;
	}

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* Start address */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* Sector count */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* Erase command */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* Sector size */

	buf_set_u32(reg_params[0].value, 0, 32, bank->sectors[first].offset);
	buf_set_u32(reg_params[1].value, 0, 32, last - first + 1);
	buf_set_u32(reg_params[2].value, 0, 32, lpcspifi_info->dev->erase_cmd);
	buf_set_u32(reg_params[3].value, 0, 32, bank->sectors[first].size);

	/* Run the algorithm */
	retval = target_run_algorithm(target, 0, NULL, 4, reg_params,
		erase_algorithm->address,
		erase_algorithm->address + sizeof(lpcspifi_flash_erase_code) - 4,
		3000*(last - first + 1), &armv7m_info);

	if (retval != ERROR_OK)
		LOG_ERROR("Error executing flash erase algorithm");

	target_free_working_area(target, erase_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	retval = lpcspifi_set_hw_mode(bank);

	return retval;
}

static int lpcspifi_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int lpcspifi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	uint32_t page_size, fifo_size;
	struct working_area *fifo;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	struct working_area *write_algorithm;
	int retval = ERROR_OK;

	LOG_DEBUG("offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > lpcspifi_info->dev->size_in_bytes) {
		LOG_WARNING("Writes past end of flash. Extra data discarded.");
		count = lpcspifi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
				(bank->sectors[sector].offset + bank->sectors[sector].size))
			&& ((offset + count - 1) >= bank->sectors[sector].offset)
			&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	/* if no valid page_size, use reasonable default */
	page_size = lpcspifi_info->dev->pagesize ?
		lpcspifi_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	retval = lpcspifi_set_hw_mode(bank);
	if (retval != ERROR_OK)
		return retval;

	/* see contrib/loaders/flash/lpcspifi_write.S for src */
	static const uint8_t lpcspifi_flash_write_code[] = {
		0x4f, 0xf4, 0xc0, 0x4a, 0xc4, 0xf2, 0x08, 0x0a,
		0x4f, 0xf0, 0xea, 0x08, 0xca, 0xf8, 0x8c, 0x81,
		0x4f, 0xf0, 0x40, 0x08, 0xca, 0xf8, 0x90, 0x81,
		0x4f, 0xf0, 0x40, 0x08, 0xca, 0xf8, 0x94, 0x81,
		0x4f, 0xf0, 0xed, 0x08, 0xca, 0xf8, 0x98, 0x81,
		0x4f, 0xf0, 0xed, 0x08, 0xca, 0xf8, 0x9c, 0x81,
		0x4f, 0xf0, 0x44, 0x08, 0xca, 0xf8, 0xa0, 0x81,
		0x4f, 0xf4, 0xc0, 0x4a, 0xc4, 0xf2, 0x0f, 0x0a,
		0x4f, 0xf4, 0x00, 0x68, 0xca, 0xf8, 0x14, 0x80,
		0x4f, 0xf4, 0x80, 0x4a, 0xc4, 0xf2, 0x0f, 0x0a,
		0x4f, 0xf0, 0xff, 0x08, 0xca, 0xf8, 0xab, 0x80,
		0x4f, 0xf0, 0x00, 0x0a, 0xc4, 0xf2, 0x05, 0x0a,
		0x4f, 0xf0, 0x00, 0x08, 0xc0, 0xf2, 0x00, 0x18,
		0xca, 0xf8, 0x94, 0x80, 0x4f, 0xf4, 0x00, 0x5a,
		0xc4, 0xf2, 0x05, 0x0a, 0x4f, 0xf0, 0x01, 0x08,
		0xca, 0xf8, 0x00, 0x87, 0x4f, 0xf4, 0x40, 0x5a,
		0xc4, 0xf2, 0x08, 0x0a, 0x4f, 0xf0, 0x07, 0x08,
		0xca, 0xf8, 0x00, 0x80, 0x4f, 0xf0, 0x02, 0x08,
		0xca, 0xf8, 0x10, 0x80, 0xca, 0xf8, 0x04, 0x80,
		0x4f, 0xf0, 0x00, 0x0b, 0xa3, 0x44, 0x93, 0x45,
		0x7f, 0xf6, 0xfc, 0xaf, 0x00, 0xf0, 0x6a, 0xf8,
		0x4f, 0xf0, 0x06, 0x09, 0x00, 0xf0, 0x53, 0xf8,
		0x00, 0xf0, 0x60, 0xf8, 0x00, 0xf0, 0x62, 0xf8,
		0x4f, 0xf0, 0x05, 0x09, 0x00, 0xf0, 0x4b, 0xf8,
		0x4f, 0xf0, 0x00, 0x09, 0x00, 0xf0, 0x47, 0xf8,
		0x00, 0xf0, 0x54, 0xf8, 0x19, 0xf0, 0x02, 0x0f,
		0x00, 0xf0, 0x5d, 0x80, 0x00, 0xf0, 0x52, 0xf8,
		0x4f, 0xf0, 0x02, 0x09, 0x00, 0xf0, 0x3b, 0xf8,
		0x4f, 0xea, 0x12, 0x49, 0x00, 0xf0, 0x37, 0xf8,
		0x4f, 0xea, 0x12, 0x29, 0x00, 0xf0, 0x33, 0xf8,
		0x4f, 0xea, 0x02, 0x09, 0x00, 0xf0, 0x2f, 0xf8,
		0xd0, 0xf8, 0x00, 0x80, 0xb8, 0xf1, 0x00, 0x0f,
		0x00, 0xf0, 0x47, 0x80, 0x47, 0x68, 0x47, 0x45,
		0x3f, 0xf4, 0xf6, 0xaf, 0x17, 0xf8, 0x01, 0x9b,
		0x00, 0xf0, 0x21, 0xf8, 0x8f, 0x42, 0x28, 0xbf,
		0x00, 0xf1, 0x08, 0x07, 0x47, 0x60, 0x01, 0x3b,
		0xbb, 0xb3, 0x02, 0xf1, 0x01, 0x02, 0x93, 0x45,
		0x7f, 0xf4, 0xe6, 0xaf, 0x00, 0xf0, 0x22, 0xf8,
		0xa3, 0x44, 0x00, 0xf0, 0x23, 0xf8, 0x4f, 0xf0,
		0x05, 0x09, 0x00, 0xf0, 0x0c, 0xf8, 0x4f, 0xf0,
		0x00, 0x09, 0x00, 0xf0, 0x08, 0xf8, 0x00, 0xf0,
		0x15, 0xf8, 0x19, 0xf0, 0x01, 0x0f, 0x7f, 0xf4,
		0xf0, 0xaf, 0xff, 0xf7, 0xa7, 0xbf, 0x4f, 0xf4,
		0x40, 0x5a, 0xc4, 0xf2, 0x08, 0x0a, 0xca, 0xf8,
		0x08, 0x90, 0xda, 0xf8, 0x0c, 0x90, 0x19, 0xf0,
		0x10, 0x0f, 0x7f, 0xf4, 0xfa, 0xaf, 0xda, 0xf8,
		0x08, 0x90, 0x70, 0x47, 0x4f, 0xf0, 0xff, 0x08,
		0x00, 0xf0, 0x02, 0xb8, 0x4f, 0xf0, 0x00, 0x08,
		0x4f, 0xf4, 0x80, 0x4a, 0xc4, 0xf2, 0x0f, 0x0a,
		0xca, 0xf8, 0xab, 0x80, 0x70, 0x47, 0x00, 0x20,
		0x50, 0x60, 0xff, 0xf7, 0xef, 0xff, 0x30, 0x46,
		0x00, 0xbe, 0xff, 0xff
	};

	if (target_alloc_working_area(target, sizeof(lpcspifi_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_ERROR("Insufficient working area. You must configure"
			" a working area > %zdB in order to write to SPIFI flash.",
			sizeof(lpcspifi_flash_write_code));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(lpcspifi_flash_write_code),
			lpcspifi_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* FIFO allocation */
	fifo_size = target_get_working_area_avail(target);

	if (fifo_size == 0) {
		/* if we already allocated the writing code but failed to get fifo
		 * space, free the algorithm */
		target_free_working_area(target, write_algorithm);

		LOG_ERROR("Insufficient working area. Please allocate at least"
			" %zdB of working area to enable flash writes.",
			sizeof(lpcspifi_flash_write_code) + 1
		);

		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (fifo_size < page_size)
		LOG_WARNING("Working area size is limited; flash writes may be"
			" slow. Increase working area size to at least %zdB"
			" to reduce write times.",
			(size_t)(sizeof(lpcspifi_flash_write_code) + page_size)
		);
	else if (fifo_size > 0x2000) /* Beyond this point, we start to get diminishing returns */
		fifo_size = 0x2000;

	if (target_alloc_working_area(target, fifo_size, &fifo) != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count (halfword-16bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);		/* page size */

	buf_set_u32(reg_params[0].value, 0, 32, fifo->address);
	buf_set_u32(reg_params[1].value, 0, 32, fifo->address + fifo->size);
	buf_set_u32(reg_params[2].value, 0, 32, offset);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, page_size);

	retval = target_run_flash_async_algorithm(target, buffer, count, 1,
			0, NULL,
			5, reg_params,
			fifo->address, fifo->size,
			write_algorithm->address, 0,
			&armv7m_info
	);

	if (retval != ERROR_OK)
		LOG_ERROR("Error executing flash write algorithm");

	target_free_working_area(target, fifo);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	/* Switch to HW mode before return to prompt */
	retval = lpcspifi_set_hw_mode(bank);
	return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int lpcspifi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	uint32_t ssp_base = lpcspifi_info->ssp_base;
	uint32_t io_base = lpcspifi_info->io_base;
	uint32_t value;
	uint8_t id_buf[3] = {0, 0, 0};
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("Getting ID");
	retval = lpcspifi_set_sw_mode(bank);
	if (retval != ERROR_OK)
		return retval;

	/* poll WIP */
	if (retval == ERROR_OK)
		retval = wait_till_ready(bank, SSP_PROBE_TIMEOUT);

	/* Send SPI command "read ID" */
	if (retval == ERROR_OK)
		retval = ssp_setcs(target, io_base, 0);
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, SPIFLASH_READ_ID);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);

	/* Dummy write to clock in data */
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, 0x00);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);
	if (retval == ERROR_OK)
		id_buf[0] = value;

	/* Dummy write to clock in data */
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, 0x00);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);
	if (retval == ERROR_OK)
		id_buf[1] = value;

	/* Dummy write to clock in data */
	if (retval == ERROR_OK)
		retval = ssp_write_reg(target, ssp_base, SSP_DATA, 0x00);
	if (retval == ERROR_OK)
		retval = poll_ssp_busy(target, ssp_base, SSP_CMD_TIMEOUT);
	if (retval == ERROR_OK)
		retval = ssp_read_reg(target, ssp_base, SSP_DATA, &value);
	if (retval == ERROR_OK)
		id_buf[2] = value;

	if (retval == ERROR_OK)
		retval = ssp_setcs(target, io_base, 1);
	if (retval == ERROR_OK)
		*id = id_buf[2] << 16 | id_buf[1] << 8 | id_buf[0];

	return retval;
}

static int lpcspifi_probe(struct flash_bank *bank)
{
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	int retval;
	uint32_t sectorsize;

	/* If we've already probed, we should be fine to skip this time. */
	if (lpcspifi_info->probed)
		return ERROR_OK;
	lpcspifi_info->probed = false;

	lpcspifi_info->ssp_base = 0x40083000;
	lpcspifi_info->io_base = 0x400F4000;
	lpcspifi_info->ioconfig_base = 0x40086000;
	lpcspifi_info->bank_num = bank->bank_number;

	/* read and decode flash ID; returns in SW mode */
	retval = lpcspifi_read_flash_id(bank, &id);
	if (retval != ERROR_OK)
		return retval;

	retval = lpcspifi_set_hw_mode(bank);
	if (retval != ERROR_OK)
		return retval;

	lpcspifi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			lpcspifi_info->dev = p;
			break;
		}

	if (!lpcspifi_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		lpcspifi_info->dev->name, lpcspifi_info->dev->device_id);

	/* Set correct size value */
	bank->size = lpcspifi_info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = lpcspifi_info->dev->sectorsize ?
		lpcspifi_info->dev->sectorsize : lpcspifi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = lpcspifi_info->dev->size_in_bytes / sectorsize;
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

	lpcspifi_info->probed = true;
	return ERROR_OK;
}

static int lpcspifi_auto_probe(struct flash_bank *bank)
{
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;
	if (lpcspifi_info->probed)
		return ERROR_OK;
	return lpcspifi_probe(bank);
}

static int lpcspifi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_lpcspifi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct lpcspifi_flash_bank *lpcspifi_info = bank->driver_priv;

	if (!(lpcspifi_info->probed)) {
		command_print_sameline(cmd, "\nSPIFI flash bank not probed yet\n");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "\nSPIFI flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		lpcspifi_info->dev->name, lpcspifi_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver lpcspifi_flash = {
	.name = "lpcspifi",
	.flash_bank_command = lpcspifi_flash_bank_command,
	.erase = lpcspifi_erase,
	.protect = lpcspifi_protect,
	.write = lpcspifi_write,
	.read = default_flash_read,
	.probe = lpcspifi_probe,
	.auto_probe = lpcspifi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = lpcspifi_protect_check,
	.info = get_lpcspifi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
