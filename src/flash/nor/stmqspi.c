/***************************************************************************
 *   Copyright (C) 2016 - 2019 by Andreas Bolsch                           *
 *   andreas.bolsch@mni.thm.de                                             *
 *                                                                         *
 *   Copyright (C) 2010 by Antonio Borneo                                  *
 *   borneo.antonio@gmail.com                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
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

/* STM QuadSPI (QSPI) and OctoSPI (OCTOSPI) controller are SPI bus controllers
 * specifically designed for SPI memories.
 * Two working modes are available:
 * - indirect mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus.
 * - memory mapped mode: the SPI is under QSPI/OCTOSPI control. Memory content
 *   is directly accessible in CPU memory space. CPU can read and execute from
 *   memory (but not write to) */

/* ATTENTION:
 * To have flash mapped in CPU memory space, the QSPI/OCTOSPI controller
 * has to be in "memory mapped mode". This requires following constraints:
 * 1) The command "reset init" has to initialize QSPI/OCTOSPI controller and put
 *    it in memory mapped mode;
 * 2) every command in this file has to return to prompt in memory mapped mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/image.h>
#include "stmqspi.h"
#include "sfdp.h"

/* deprecated */
#undef SPIFLASH_READ
#undef SPIFLASH_PAGE_PROGRAM

#define READ_REG(a)												\
({																\
	uint32_t _result;											\
																\
	retval = target_read_u32(target, io_base + (a), &_result);	\
	(retval == ERROR_OK) ? _result : 0x0;						\
})

/* saved mode settings */
#define QSPI_MODE (stmqspi_info->saved_ccr & \
	(0xF0000000U | QSPI_DCYC_MASK | QSPI_4LINE_MODE | QSPI_ALTB_MODE | QSPI_ADDR4))

/* saved read mode settings but indirect read instead of memory mapped
 * in particular, use the dummy cycle setting from this saved setting */
#define	QSPI_CCR_READ (QSPI_READ_MODE | (stmqspi_info->saved_ccr & \
	(0xF0000000U | QSPI_DCYC_MASK | QSPI_4LINE_MODE | QSPI_ALTB_MODE | QSPI_ADDR4 | 0xFF)))

/* QSPI_CCR for various other commands, these never use dummy cycles nor alternate bytes */
#define	QSPI_CCR_READ_STATUS \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB) | \
	(QSPI_READ_MODE | SPIFLASH_READ_STATUS))

#define	QSPI_CCR_READ_ID \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB) | \
	(QSPI_READ_MODE | SPIFLASH_READ_ID))

#define	QSPI_CCR_READ_MID \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB) | \
	(QSPI_READ_MODE | SPIFLASH_READ_MID))

/* always use 3-byte addresses for read SFDP */
#define	QSPI_CCR_READ_SFDP \
	((QSPI_MODE & ~QSPI_DCYC_MASK & ~QSPI_ADDR4 & QSPI_NO_ALTB) | \
	(QSPI_READ_MODE | QSPI_ADDR3 | SPIFLASH_READ_SFDP))

#define QSPI_CCR_WRITE_ENABLE \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB & QSPI_NO_DATA) | \
	(QSPI_WRITE_MODE | SPIFLASH_WRITE_ENABLE))

#define QSPI_CCR_SECTOR_ERASE \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB & QSPI_NO_DATA) | \
	(QSPI_WRITE_MODE | stmqspi_info->dev.erase_cmd))

#define QSPI_CCR_MASS_ERASE \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ADDR & QSPI_NO_ALTB & QSPI_NO_DATA) | \
	(QSPI_WRITE_MODE | stmqspi_info->dev.chip_erase_cmd))

#define QSPI_CCR_PAGE_PROG \
	((QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB) | \
	(QSPI_WRITE_MODE | stmqspi_info->dev.pprog_cmd))

/* saved mode settings */
#define OCTOSPI_MODE (stmqspi_info->saved_cr & 0xCFFFFFFF)

#define OPI_MODE ((stmqspi_info->saved_ccr & OCTOSPI_ISIZE_MASK) != 0)

#define OCTOSPI_MODE_CCR (stmqspi_info->saved_ccr & \
	(0xF0000000U | OCTOSPI_8LINE_MODE | OCTOSPI_ALTB_MODE | OCTOSPI_ADDR4))

/* use saved ccr for read */
#define OCTOSPI_CCR_READ OCTOSPI_MODE_CCR

/* OCTOSPI_CCR for various other commands, these never use alternate bytes	*
 * for READ_STATUS and READ_ID, 4-byte address 0							*
 * 4 dummy cycles must sent in OPI mode when DQS is disabled. However, when	*
 * DQS is enabled, some STM32 devices need at least 6 dummy cycles for		*
 * proper operation, but otherwise the actual number has no effect!			*
 * E.g. RM0432 Rev. 7 is incorrect regarding this: L4R9 works well with 4	*
 * dummy clocks whereas L4P5 not at all.									*
 */
#define OPI_DUMMY \
	((stmqspi_info->saved_ccr & OCTOSPI_DQSEN) ? 6U : 4U)

#define	OCTOSPI_CCR_READ_STATUS \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_DDTR & \
	(OPI_MODE ? ~0U : OCTOSPI_NO_ADDR) & OCTOSPI_NO_ALTB))

#define	OCTOSPI_CCR_READ_ID \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_DDTR & \
	(OPI_MODE ? ~0U : OCTOSPI_NO_ADDR) & OCTOSPI_NO_ALTB))

#define	OCTOSPI_CCR_READ_MID OCTOSPI_CCR_READ_ID

/* 4-byte address in octo mode, else 3-byte address for read SFDP */
#define	OCTOSPI_CCR_READ_SFDP(len) \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_DDTR & ~OCTOSPI_ADDR4 & OCTOSPI_NO_ALTB) | \
	(((len) < 4) ? OCTOSPI_ADDR3 : OCTOSPI_ADDR4))

#define OCTOSPI_CCR_WRITE_ENABLE \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_ADDR & OCTOSPI_NO_ALTB & OCTOSPI_NO_DATA))

#define OCTOSPI_CCR_SECTOR_ERASE \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_ALTB & OCTOSPI_NO_DATA))

#define OCTOSPI_CCR_MASS_ERASE \
	((OCTOSPI_MODE_CCR & OCTOSPI_NO_ADDR & OCTOSPI_NO_ALTB & OCTOSPI_NO_DATA))

#define OCTOSPI_CCR_PAGE_PROG \
	((OCTOSPI_MODE_CCR & QSPI_NO_ALTB))

#define SPI_ADSIZE (((stmqspi_info->saved_ccr >> SPI_ADSIZE_POS) & 0x3) + 1)

#define OPI_CMD(cmd) ((OPI_MODE ? ((((uint16_t)(cmd)) << 8) | (~(cmd) & 0xFFU)) : (cmd)))

#define OCTOSPI_CMD(mode, ccr, ir)										\
({																		\
	retval = target_write_u32(target, io_base + OCTOSPI_CR,				\
		OCTOSPI_MODE | (mode));											\
	if (retval == ERROR_OK)												\
		retval = target_write_u32(target, io_base + OCTOSPI_TCR,		\
			(stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK) |			\
			((OPI_MODE && ((mode) == OCTOSPI_READ_MODE)) ?				\
			(OPI_DUMMY << OCTOSPI_DCYC_POS) : 0));						\
	if (retval == ERROR_OK)												\
		retval = target_write_u32(target, io_base + OCTOSPI_CCR, ccr);	\
	if (retval == ERROR_OK)												\
		retval = target_write_u32(target, io_base + OCTOSPI_IR,			\
			OPI_CMD(ir));												\
	retval;																\
})

/* convert uint32_t into 4 uint8_t in little endian byte order */
static inline uint32_t h_to_le_32(uint32_t val)
{
	uint32_t result;

	h_u32_to_le((uint8_t *)&result, val);
	return result;
}

/* Timeout in ms */
#define SPI_CMD_TIMEOUT			(100)
#define SPI_PROBE_TIMEOUT		(100)
#define SPI_MAX_TIMEOUT			(2000)
#define SPI_MASS_ERASE_TIMEOUT	(400000)

struct sector_info {
	uint32_t offset;
	uint32_t size;
	uint32_t result;
};

struct stmqspi_flash_bank {
	bool probed;
	char devname[32];
	bool octo;
	struct flash_device dev;
	uint32_t io_base;
	uint32_t saved_cr;	/* in particalar FSEL, DFM bit mask in QUADSPI_CR *AND* OCTOSPI_CR */
	uint32_t saved_ccr; /* different meaning for QUADSPI and OCTOSPI */
	uint32_t saved_tcr;	/* only for OCTOSPI */
	uint32_t saved_ir;	/* only for OCTOSPI */
	unsigned int sfdp_dummy1;	/* number of dummy bytes for SFDP read for flash1 and octo */
	unsigned int sfdp_dummy2;	/* number of dummy bytes for SFDP read for flash2 */
};

FLASH_BANK_COMMAND_HANDLER(stmqspi_flash_bank_command)
{
	struct stmqspi_flash_bank *stmqspi_info;
	uint32_t io_base;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], io_base);

	stmqspi_info = malloc(sizeof(struct stmqspi_flash_bank));
	if (stmqspi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = stmqspi_info;
	stmqspi_info->sfdp_dummy1 = 0;
	stmqspi_info->sfdp_dummy2 = 0;
	stmqspi_info->probed = false;
	stmqspi_info->io_base = io_base;

	return ERROR_OK;
}

/* Poll busy flag */
/* timeout in ms */
static int poll_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint32_t spi_sr;
	int retval;
	long long endtime;

	endtime = timeval_ms() + timeout;
	do {
		spi_sr = READ_REG(SPI_SR);
		if ((spi_sr & BIT(SPI_BUSY)) == 0) {
			if (retval == ERROR_OK) {
				/* Clear transmit finished flag */
				retval = target_write_u32(target, io_base + SPI_FCR, BIT(SPI_TCF));
			}
			return retval;
		} else
			LOG_DEBUG("busy: 0x%08X", spi_sr);
		alive_sleep(1);
	} while (timeval_ms() < endtime);

	LOG_ERROR("Timeout while polling BUSY");
	return ERROR_FLASH_OPERATION_FAILED;
}

/* Set to memory-mapped mode, e.g. after an error */
static int set_mm_mode(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	int retval;

	/* Reset Address register bits 0 and 1, see various errata sheets */
	retval = target_write_u32(target, io_base + SPI_AR, 0x0);
	if (retval != ERROR_OK)
		return retval;

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Finally switch to memory mapped mode */
	if (IS_OCTOSPI) {
		retval = target_write_u32(target, io_base + OCTOSPI_CR,
			OCTOSPI_MODE | OCTOSPI_MM_MODE);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, io_base + OCTOSPI_CCR,
				stmqspi_info->saved_ccr);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, io_base + OCTOSPI_TCR,
				stmqspi_info->saved_tcr);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, io_base + OCTOSPI_IR,
				stmqspi_info->saved_ir);
	} else {
		retval = target_write_u32(target, io_base + QSPI_CR,
			stmqspi_info->saved_cr);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, io_base + QSPI_CCR,
				stmqspi_info->saved_ccr);
	}
	return retval;
}

/* Read the status register of the external SPI flash chip(s). */
static int read_status_reg(struct flash_bank *bank, uint16_t *status)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint8_t data;
	int count, retval;

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Read always two (for DTR mode) bytes per chip */
	count = 2;
	retval = target_write_u32(target, io_base + SPI_DLR,
		((stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 2 * count : count) - 1);
	if (retval != ERROR_OK)
		goto err;

	/* Read status */
	if (IS_OCTOSPI) {
		retval = OCTOSPI_CMD(OCTOSPI_READ_MODE, OCTOSPI_CCR_READ_STATUS, SPIFLASH_READ_STATUS);
		if (OPI_MODE) {
			/* Dummy address 0, only required for 8-line mode */
			retval = target_write_u32(target, io_base + SPI_AR, 0);
			if (retval != ERROR_OK)
				goto err;
		}
	} else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_READ_STATUS);
	if (retval != ERROR_OK)
		goto err;

	*status = 0;

	/* for debugging only */
	(void)READ_REG(SPI_SR);

	for ( ; count > 0; --count) {
		if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH)))
			!= BIT(SPI_FSEL_FLASH)) {
			/* get status of flash 1 in dual mode or flash 1 only mode */
			retval = target_read_u8(target, io_base + SPI_DR, &data);
			if (retval != ERROR_OK)
				goto err;
			*status |= data;
		}

		if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH))) != 0) {
			/* get status of flash 2 in dual mode or flash 2 only mode */
			retval = target_read_u8(target, io_base + SPI_DR, &data);
			if (retval != ERROR_OK)
				goto err;
			*status |= ((uint16_t)data) << 8;
		}
	}

	LOG_DEBUG("flash status regs: 0x%04" PRIx16, *status);

err:
	return retval;
}

/* check for WIP (write in progress) bit(s) in status register(s) */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	uint16_t status;
	int retval;
	long long endtime;

	endtime = timeval_ms() + timeout;
	do {
		/* Read flash status register(s) */
		retval = read_status_reg(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & ((SPIFLASH_BSY_BIT << 8) | SPIFLASH_BSY_BIT)) == 0)
			return retval;
		alive_sleep(25);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FLASH_OPERATION_FAILED;
}

/* Send "write enable" command to SPI flash chip(s). */
static int qspi_write_enable(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint16_t status;
	int retval;

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Send write enable command */
	if (IS_OCTOSPI) {
		retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE, OCTOSPI_CCR_WRITE_ENABLE, SPIFLASH_WRITE_ENABLE);
		if (OPI_MODE) {
			/* Dummy address 0, only required for 8-line mode */
			retval = target_write_u32(target, io_base + SPI_AR, 0);
			if (retval != ERROR_OK)
				goto err;
		}
	} else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_WRITE_ENABLE);
	if (retval != ERROR_OK)
		goto err;


	/* Wait for transmit of command completed */
	poll_busy(bank, SPI_CMD_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Read flash status register */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		goto err;

	/* Check write enabled for flash 1 */
	if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH)))
		!= BIT(SPI_FSEL_FLASH))
		if ((status & (SPIFLASH_WE_BIT | SPIFLASH_BSY_BIT)) != SPIFLASH_WE_BIT) {
			LOG_ERROR("Cannot write enable flash1. Status=0x%02x",
				status & 0xFFU);
			return ERROR_FLASH_OPERATION_FAILED;
		}

	/* Check write enabled for flash 2 */
	status >>= 8;
	if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH))) != 0)
		if ((status & (SPIFLASH_WE_BIT | SPIFLASH_BSY_BIT)) != SPIFLASH_WE_BIT) {
			LOG_ERROR("Cannot write enable flash2. Status=0x%02x",
				status & 0xFFU);
			return ERROR_FLASH_OPERATION_FAILED;
		}

err:
	return retval;
}

COMMAND_HANDLER(stmqspi_handle_mass_erase_command)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct stmqspi_flash_bank *stmqspi_info;
	struct duration bench;
	uint32_t io_base;
	uint16_t status;
	unsigned int sector;
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stmqspi_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (stmqspi_info->dev.chip_erase_cmd == 0x00) {
		LOG_ERROR("Mass erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	for (sector = 0; sector < bank->num_sectors; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FLASH_PROTECTED;
		}
	}

	io_base = stmqspi_info->io_base;
	duration_start(&bench);

	retval = qspi_write_enable(bank);
	if (retval != ERROR_OK)
		goto err;

	/* Send Mass Erase command */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE, OCTOSPI_CCR_MASS_ERASE,
			stmqspi_info->dev.chip_erase_cmd);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_MASS_ERASE);
	if (retval != ERROR_OK)
		goto err;

	/* Wait for transmit of command completed */
	poll_busy(bank, SPI_CMD_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Read flash status register(s) */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		goto err;

	/* Check for command in progress for flash 1 */
	if (((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH)))
		!= BIT(SPI_FSEL_FLASH)) && ((status & SPIFLASH_BSY_BIT) == 0) &&
		((status & SPIFLASH_WE_BIT) != 0)) {
		LOG_ERROR("Mass erase command not accepted by flash1. Status=0x%02x",
			status & 0xFFU);
		retval = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Check for command in progress for flash 2 */
	status >>= 8;
	if (((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH))) != 0) &&
		((status & SPIFLASH_BSY_BIT) == 0) &&
		((status & SPIFLASH_WE_BIT) != 0)) {
		LOG_ERROR("Mass erase command not accepted by flash2. Status=0x%02x",
			status & 0xFFU);
		retval = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Poll WIP for end of self timed Sector Erase cycle */
	retval = wait_till_ready(bank, SPI_MASS_ERASE_TIMEOUT);

	duration_measure(&bench);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (sector = 0; sector < bank->num_sectors; sector++)
			bank->sectors[sector].is_erased = 1;

		command_print(CMD, "stmqspi mass erase completed in %fs (%0.3f KiB/s)",
			duration_elapsed(&bench),
			duration_kbps(&bench, bank->size));
	} else {
		command_print(CMD, "stmqspi mass erase not completed even after %fs",
			duration_elapsed(&bench));
	}

err:
	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int log2u(uint32_t word)
{
	int result;

	for (result = 0; (unsigned int) result < sizeof(uint32_t) * CHAR_BIT; result++)
		if (word == BIT(result))
			return result;

	return -1;
}

COMMAND_HANDLER(stmqspi_handle_set)
{
	struct flash_bank *bank = NULL;
	struct target *target = NULL;
	struct stmqspi_flash_bank *stmqspi_info = NULL;
	struct flash_sector *sectors = NULL;
	uint32_t io_base;
	unsigned int index = 0, dual, fsize;
	int retval;

	LOG_DEBUG("%s", __func__);

	dual = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 1 : 0;

	/* chip_erase_cmd, sectorsize and erase_cmd are optional */
	if ((CMD_ARGC < 7) || (CMD_ARGC > 10))
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, index++, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	stmqspi_info = bank->driver_priv;

	/* invalidate all old info */
	if (stmqspi_info->probed)
		free(bank->sectors);
	bank->size = 0;
	bank->num_sectors = 0;
	bank->sectors = NULL;
	stmqspi_info->sfdp_dummy1 = 0;
	stmqspi_info->sfdp_dummy2 = 0;
	stmqspi_info->probed = false;
	memset(&stmqspi_info->dev, 0, sizeof(stmqspi_info->dev));
	stmqspi_info->dev.name = "unknown";

	strncpy(stmqspi_info->devname, CMD_ARGV[index++], sizeof(stmqspi_info->devname) - 1);
	stmqspi_info->devname[sizeof(stmqspi_info->devname) - 1] = '\0';

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], stmqspi_info->dev.size_in_bytes);
	if (log2u(stmqspi_info->dev.size_in_bytes) < 8) {
		command_print(CMD, "stmqspi: device size must be 2^n with n >= 8");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], stmqspi_info->dev.pagesize);
	if (stmqspi_info->dev.pagesize > stmqspi_info->dev.size_in_bytes ||
		(log2u(stmqspi_info->dev.pagesize) < 0)) {
		command_print(CMD, "stmqspi: page size must be 2^n and <= device size");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.read_cmd);
	if ((stmqspi_info->dev.read_cmd != 0x03) &&
		(stmqspi_info->dev.read_cmd != 0x13)) {
		command_print(CMD, "stmqspi: only 0x03/0x13 READ cmd allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.qread_cmd);
	if ((stmqspi_info->dev.qread_cmd != 0x00) &&
		(stmqspi_info->dev.qread_cmd != 0x0B) &&
		(stmqspi_info->dev.qread_cmd != 0x0C) &&
		(stmqspi_info->dev.qread_cmd != 0x3B) &&
		(stmqspi_info->dev.qread_cmd != 0x3C) &&
		(stmqspi_info->dev.qread_cmd != 0x6B) &&
		(stmqspi_info->dev.qread_cmd != 0x6C) &&
		(stmqspi_info->dev.qread_cmd != 0xBB) &&
		(stmqspi_info->dev.qread_cmd != 0xBC) &&
		(stmqspi_info->dev.qread_cmd != 0xEB) &&
		(stmqspi_info->dev.qread_cmd != 0xEC) &&
		(stmqspi_info->dev.qread_cmd != 0xEE)) {
		command_print(CMD, "stmqspi: only 0x0B/0x0C/0x3B/0x3C/"
			"0x6B/0x6C/0xBB/0xBC/0xEB/0xEC/0xEE QREAD allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.pprog_cmd);
	if ((stmqspi_info->dev.pprog_cmd != 0x02) &&
		(stmqspi_info->dev.pprog_cmd != 0x12) &&
		(stmqspi_info->dev.pprog_cmd != 0x32)) {
		command_print(CMD, "stmqspi: only 0x02/0x12/0x32 PPRG cmd allowed");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (index < CMD_ARGC)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.chip_erase_cmd);
	else
		stmqspi_info->dev.chip_erase_cmd = 0x00;

	if (index < CMD_ARGC) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], stmqspi_info->dev.sectorsize);
		if ((stmqspi_info->dev.sectorsize > stmqspi_info->dev.size_in_bytes) ||
			(stmqspi_info->dev.sectorsize < stmqspi_info->dev.pagesize) ||
			(log2u(stmqspi_info->dev.sectorsize) < 0)) {
			command_print(CMD, "stmqspi: sector size must be 2^n and <= device size");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (index < CMD_ARGC)
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], stmqspi_info->dev.erase_cmd);
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	} else {
		/* no sector size / sector erase cmd given, treat whole bank as a single sector */
		stmqspi_info->dev.erase_cmd = 0x00;
		stmqspi_info->dev.sectorsize = stmqspi_info->dev.size_in_bytes;
	}

	/* set correct size value */
	bank->size = stmqspi_info->dev.size_in_bytes << dual;

	io_base = stmqspi_info->io_base;
	fsize = (READ_REG(SPI_DCR) >> SPI_FSIZE_POS) & (BIT(SPI_FSIZE_LEN) - 1);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("FSIZE = 0x%04x", fsize);
	if (bank->size == BIT(fsize + 1))
		LOG_DEBUG("FSIZE in DCR(1) matches actual capacity. Beware of silicon bug in H7, L4+, MP1.");
	else if (bank->size == BIT(fsize + 0))
		LOG_DEBUG("FSIZE in DCR(1) is off by one regarding actual capacity. Fix for silicon bug?");
	else
		LOG_ERROR("FSIZE in DCR(1) doesn't match actual capacity.");

	/* create and fill sectors array */
	bank->num_sectors =
		stmqspi_info->dev.size_in_bytes / stmqspi_info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * (stmqspi_info->dev.sectorsize << dual);
		sectors[sector].size = (stmqspi_info->dev.sectorsize << dual);
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	stmqspi_info->dev.name = stmqspi_info->devname;
	if (stmqspi_info->dev.size_in_bytes / 4096)
		LOG_INFO("flash \'%s\' id = unknown\nchip size = %" PRIu32 "kbytes,"
			" bank size = %" PRIu32 "kbytes", stmqspi_info->dev.name,
			stmqspi_info->dev.size_in_bytes / 1024,
			(stmqspi_info->dev.size_in_bytes / 1024) << dual);
	else
		LOG_INFO("flash \'%s\' id = unknown\nchip size = %" PRIu32 "bytes,"
			" bank size = %" PRIu32 "bytes", stmqspi_info->dev.name,
			stmqspi_info->dev.size_in_bytes,
			stmqspi_info->dev.size_in_bytes << dual);

	stmqspi_info->probed = true;

	return ERROR_OK;
}

COMMAND_HANDLER(stmqspi_handle_cmd)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct stmqspi_flash_bank *stmqspi_info = NULL;
	uint32_t io_base, addr;
	uint8_t num_write, num_read, cmd_byte, data;
	unsigned int count;
	const int max = 21;
	char temp[4], output[(2 + max + 256) * 3 + 8];
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	num_write = CMD_ARGC - 2;
	if (num_write > max) {
		LOG_ERROR("at most %d bytes may be sent", max);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	stmqspi_info = bank->driver_priv;
	io_base = stmqspi_info->io_base;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], num_read);
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2], cmd_byte);

	if (num_read == 0) {
		/* nothing to read, then one command byte and for dual flash
		 * an *even* number of data bytes to follow */
		if (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) {
			if ((num_write & 1) == 0) {
				LOG_ERROR("number of data bytes to write must be even in dual mode");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		}
	} else {
		/* read mode, one command byte and up to four following address bytes */
		if (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) {
			if ((num_read & 1) != 0) {
				LOG_ERROR("number of bytes to read must be even in dual mode");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		}
		if ((num_write < 1) || (num_write > 5)) {
			LOG_ERROR("one cmd and up to four addr bytes must be send when reading");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* send command byte */
	snprintf(output, sizeof(output), "spi: %02x ", cmd_byte);
	if (num_read == 0) {
		/* write, send cmd byte */
		retval = target_write_u32(target, io_base + SPI_DLR, ((uint32_t)num_write) - 2);
		if (retval != ERROR_OK)
			goto err;

		if (IS_OCTOSPI)
			retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE,
				(OCTOSPI_MODE_CCR & OCTOSPI_NO_ALTB & OCTOSPI_NO_ADDR &
				((num_write == 1) ? OCTOSPI_NO_DATA : ~0U)), cmd_byte);
		else
			retval = target_write_u32(target, io_base + QSPI_CCR,
				(QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB & QSPI_NO_ADDR &
				((num_write == 1) ? QSPI_NO_DATA : ~0U)) |
				(QSPI_WRITE_MODE | cmd_byte));
		if (retval != ERROR_OK)
			goto err;

		/* send additional data bytes */
		for (count = 3; count < CMD_ARGC; count++) {
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[count], data);
			snprintf(temp, sizeof(temp), "%02" PRIx8 " ", data);
			retval = target_write_u8(target, io_base + SPI_DR, data);
			if (retval != ERROR_OK)
				goto err;
			strncat(output, temp, sizeof(output) - strlen(output) - 1);
		}
		strncat(output, "-> ", sizeof(output) - strlen(output) - 1);
	} else {
		/* read, pack additional bytes into address */
		addr = 0;
		for (count = 3; count < CMD_ARGC; count++) {
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[count], data);
			snprintf(temp, sizeof(temp), "%02" PRIx8 " ", data);
			addr = (addr << 8) | data;
			strncat(output, temp, sizeof(output) - strlen(output) - 1);
		}
		strncat(output, "-> ", sizeof(output) - strlen(output) - 1);

		/* send cmd byte, if ADMODE indicates no address, this already triggers command */
		retval = target_write_u32(target, io_base + SPI_DLR, ((uint32_t)num_read) - 1);
		if (retval != ERROR_OK)
			goto err;
		if (IS_OCTOSPI)
			retval = OCTOSPI_CMD(OCTOSPI_READ_MODE,
				(OCTOSPI_MODE_CCR & OCTOSPI_NO_DDTR & OCTOSPI_NO_ALTB & ~OCTOSPI_ADDR4 &
					((num_write == 1) ? OCTOSPI_NO_ADDR : ~0U)) |
				(((num_write - 2) & 0x3U) << SPI_ADSIZE_POS), cmd_byte);
		else
			retval = target_write_u32(target, io_base + QSPI_CCR,
				(QSPI_MODE & ~QSPI_DCYC_MASK & QSPI_NO_ALTB & ~QSPI_ADDR4 &
					((num_write == 1) ? QSPI_NO_ADDR : ~0U)) |
				((QSPI_READ_MODE | (((num_write - 2) & 0x3U) << SPI_ADSIZE_POS) | cmd_byte)));
		if (retval != ERROR_OK)
			goto err;

		if (num_write > 1) {
			/* if ADMODE indicates address required, only the write to AR triggers command */
			retval = target_write_u32(target, io_base + SPI_AR, addr);
			if (retval != ERROR_OK)
				goto err;
		}

		/* read response bytes */
		for ( ; num_read > 0; num_read--) {
			retval = target_read_u8(target, io_base + SPI_DR, &data);
			if (retval != ERROR_OK)
				goto err;
			snprintf(temp, sizeof(temp), "%02" PRIx8 " ", data);
			strncat(output, temp, sizeof(output) - strlen(output) - 1);
		}
	}
	command_print(CMD, "%s", output);

err:
	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int qspi_erase_sector(struct flash_bank *bank, unsigned int sector)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint16_t status;
	int retval;

	retval = qspi_write_enable(bank);
	if (retval != ERROR_OK)
		goto err;

	/* Send Sector Erase command */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_WRITE_MODE, OCTOSPI_CCR_SECTOR_ERASE,
			stmqspi_info->dev.erase_cmd);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_SECTOR_ERASE);
	if (retval != ERROR_OK)
		goto err;

	/* Address is sector offset, this write initiates command transmission */
	retval = target_write_u32(target, io_base + SPI_AR, bank->sectors[sector].offset);
	if (retval != ERROR_OK)
		goto err;

	/* Wait for transmit of command completed */
	poll_busy(bank, SPI_CMD_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Read flash status register(s) */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		goto err;

	LOG_DEBUG("erase status regs: 0x%04" PRIx16, status);

	/* Check for command in progress for flash 1 */
	/* If BSY and WE are already cleared the erase did probably complete already */
	if (((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH)))
		!= BIT(SPI_FSEL_FLASH)) && ((status & SPIFLASH_BSY_BIT) == 0) &&
		((status & SPIFLASH_WE_BIT) != 0)) {
		LOG_ERROR("Sector erase command not accepted by flash1. Status=0x%02x",
			status & 0xFFU);
		retval = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Check for command in progress for flash 2 */
	/* If BSY and WE are already cleared the erase did probably complete already */
	status >>= 8;
	if (((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH))) != 0) &&
		((status & SPIFLASH_BSY_BIT) == 0) &&
		((status & SPIFLASH_WE_BIT) != 0)) {
		LOG_ERROR("Sector erase command not accepted by flash2. Status=0x%02x",
			status & 0xFFU);
		retval = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Erase takes a long time, so some sort of progress message is a good idea */
	LOG_DEBUG("erasing sector %4u", sector);

	/* Poll WIP for end of self timed Sector Erase cycle */
	retval = wait_till_ready(bank, SPI_MAX_TIMEOUT);

err:
	return retval;
}

static int stmqspi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	unsigned int sector;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: from sector %u to sector %u", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (stmqspi_info->dev.erase_cmd == 0x00) {
		LOG_ERROR("Sector erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FLASH_PROTECTED;
		}
	}

	for (sector = first; sector <= last; sector++) {
		retval = qspi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		alive_sleep(10);
		keep_alive();
	}

	if (retval != ERROR_OK)
		LOG_ERROR("Flash sector_erase failed on sector %u", sector);

	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int stmqspi_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	unsigned int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;

	if (set)
		LOG_WARNING("setting soft protection only, not related to flash's hardware write protection");

	return ERROR_OK;
}

/* Check whether flash is blank */
static int stmqspi_blank_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	struct duration bench;
	struct reg_param reg_params[2];
	struct armv7m_algorithm armv7m_info;
	struct working_area *algorithm;
	const uint8_t *code;
	struct sector_info erase_check_info;
	uint32_t codesize, maxsize, result, exit_point;
	unsigned int count, index, num_sectors, sector;
	int retval;
	const uint32_t erased = 0x00FF;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* see contrib/loaders/flash/stmqspi/stmqspi_erase_check.S for src */
	static const uint8_t stmqspi_erase_check_code[] = {
		#include "../../../contrib/loaders/flash/stmqspi/stmqspi_erase_check.inc"
	};

	/* see contrib/loaders/flash/stmqspi/stmoctospi_erase_check.S for src */
	static const uint8_t stmoctospi_erase_check_code[] = {
		#include "../../../contrib/loaders/flash/stmqspi/stmoctospi_erase_check.inc"
	};

	if (IS_OCTOSPI) {
		code = stmoctospi_erase_check_code;
		codesize = sizeof(stmoctospi_erase_check_code);
	} else {
		code = stmqspi_erase_check_code;
		codesize = sizeof(stmqspi_erase_check_code);
	}

	/* This will overlay the last 4 words of stmqspi/stmoctospi_erase_check_code in target */
	/* for read use the saved settings (memory mapped mode) but indirect read mode */
	uint32_t ccr_buffer[][4] = {
		/* cr  (not used for QSPI)			*
		 * ccr (for both QSPI and OCTOSPI)	*
		 * tcr (not used for QSPI)			*
		 * ir  (not used for QSPI)			*/
		{
			h_to_le_32(OCTOSPI_MODE | OCTOSPI_READ_MODE),
			h_to_le_32(IS_OCTOSPI ? OCTOSPI_CCR_READ : QSPI_CCR_READ),
			h_to_le_32(stmqspi_info->saved_tcr),
			h_to_le_32(stmqspi_info->saved_ir),
		},
	};

	maxsize = target_get_working_area_avail(target);
	if (maxsize < codesize + sizeof(erase_check_info)) {
		LOG_ERROR("Not enough working area, can't do QSPI blank check");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	num_sectors = (maxsize - codesize) / sizeof(erase_check_info);
	num_sectors = (bank->num_sectors < num_sectors) ? bank->num_sectors : num_sectors;

	if (target_alloc_working_area_try(target,
			codesize + num_sectors * sizeof(erase_check_info), &algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* prepare blank check code, excluding ccr_buffer */
	retval = target_write_buffer(target, algorithm->address,
		codesize - sizeof(ccr_buffer), code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare QSPI/OCTOSPI_CCR register values */
	retval = target_write_buffer(target, algorithm->address
		+ codesize - sizeof(ccr_buffer),
		sizeof(ccr_buffer), (uint8_t *)ccr_buffer);
	if (retval != ERROR_OK)
		goto err;

	duration_start(&bench);

	/* after breakpoint instruction (halfword), one nop (halfword) and
	 * port_buffer till end of code */
	exit_point = algorithm->address + codesize - sizeof(uint32_t) - sizeof(ccr_buffer);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);	/* sector count */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* QSPI/OCTOSPI io_base */

	sector = 0;
	while (sector < bank->num_sectors) {
		/* at most num_sectors sectors to handle in one run */
		count = bank->num_sectors - sector;
		if (count > num_sectors)
			count = num_sectors;

		for (index = 0; index < count; index++) {
			erase_check_info.offset = h_to_le_32(bank->sectors[sector + index].offset);
			erase_check_info.size = h_to_le_32(bank->sectors[sector + index].size);
			erase_check_info.result = h_to_le_32(erased);

			retval = target_write_buffer(target, algorithm->address
				+ codesize + index * sizeof(erase_check_info),
					sizeof(erase_check_info), (uint8_t *)&erase_check_info);
			if (retval != ERROR_OK)
				goto err;
		}

		buf_set_u32(reg_params[0].value, 0, 32, count);
		buf_set_u32(reg_params[1].value, 0, 32, stmqspi_info->io_base);

		armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
		armv7m_info.core_mode = ARM_MODE_THREAD;

		LOG_DEBUG("checking sectors %u to %u", sector, sector + count - 1);
		/* check a block of sectors */
		retval = target_run_algorithm(target,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			algorithm->address, exit_point,
			count * ((bank->sectors[sector].size >> 6) + 1) + 1000,
			&armv7m_info);
		if (retval != ERROR_OK)
			break;

		for (index = 0; index < count; index++) {
			retval = target_read_buffer(target, algorithm->address
				+ codesize + index * sizeof(erase_check_info),
					sizeof(erase_check_info), (uint8_t *)&erase_check_info);
			if (retval != ERROR_OK)
				goto err;

			if ((erase_check_info.offset != h_to_le_32(bank->sectors[sector + index].offset)) ||
				(erase_check_info.size != 0)) {
				LOG_ERROR("corrupted blank check info");
				goto err;
			}

			/* we need le_32_to_h, but that's the same as h_to_le_32 */
			result = h_to_le_32(erase_check_info.result);
			bank->sectors[sector + index].is_erased = ((result & 0xFF) == 0xFF);
			LOG_DEBUG("Flash sector %u checked: 0x%04x", sector + index, result & 0xFFFFU);
		}
		keep_alive();
		sector += count;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	duration_measure(&bench);
	LOG_INFO("stmqspi blank checked in %fs (%0.3f KiB/s)", duration_elapsed(&bench),
		duration_kbps(&bench, bank->size));

err:
	target_free_working_area(target, algorithm);

	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

/* Verify checksum */
static int qspi_verify(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	struct working_area *algorithm;
	const uint8_t *code;
	uint32_t pagesize, codesize, crc32, result, exit_point;
	int retval;

	/* see contrib/loaders/flash/stmqspi/stmqspi_crc32.S for src */
	static const uint8_t stmqspi_crc32_code[] = {
		#include "../../../contrib/loaders/flash/stmqspi/stmqspi_crc32.inc"
	};

	/* see contrib/loaders/flash/stmqspi/stmoctospi_crc32.S for src */
	static const uint8_t stmoctospi_crc32_code[] = {
		#include "../../../contrib/loaders/flash/stmqspi/stmoctospi_crc32.inc"
	};

	if (IS_OCTOSPI) {
		code = stmoctospi_crc32_code;
		codesize = sizeof(stmoctospi_crc32_code);
	} else {
		code = stmqspi_crc32_code;
		codesize = sizeof(stmqspi_crc32_code);
	}

	/* block size doesn't matter that much here */
	pagesize = stmqspi_info->dev.sectorsize;
	if (pagesize == 0)
		pagesize = stmqspi_info->dev.pagesize;
	if (pagesize == 0)
		pagesize = SPIFLASH_DEF_PAGESIZE;

	/* adjust size according to dual flash mode */
	pagesize = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? pagesize << 1 : pagesize;

	/* This will overlay the last 4 words of stmqspi/stmoctospi_crc32_code in target */
	/* for read use the saved settings (memory mapped mode) but indirect read mode */
	uint32_t ccr_buffer[][4] = {
		/* cr  (not used for QSPI)			*
		 * ccr (for both QSPI and OCTOSPI)	*
		 * tcr (not used for QSPI)			*
		 * ir  (not used for QSPI)			*/
		{
			h_to_le_32(OCTOSPI_MODE | OCTOSPI_READ_MODE),
			h_to_le_32(IS_OCTOSPI ? OCTOSPI_CCR_READ : QSPI_CCR_READ),
			h_to_le_32(stmqspi_info->saved_tcr),
			h_to_le_32(stmqspi_info->saved_ir),
		},
	};

	if (target_alloc_working_area_try(target, codesize, &algorithm) != ERROR_OK) {
		LOG_ERROR("Not enough working area, can't do QSPI verify");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* prepare verify code, excluding ccr_buffer */
	retval = target_write_buffer(target, algorithm->address,
		codesize - sizeof(ccr_buffer), code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare QSPI/OCTOSPI_CCR register values */
	retval = target_write_buffer(target, algorithm->address
		+ codesize - sizeof(ccr_buffer),
		sizeof(ccr_buffer), (uint8_t *)ccr_buffer);
	if (retval != ERROR_OK)
		goto err;

	/* after breakpoint instruction (halfword), one nop (halfword) and
	 * port_buffer till end of code */
	exit_point = algorithm->address + codesize - sizeof(uint32_t) - sizeof(ccr_buffer);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* count (in), crc32 (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* pagesize */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* offset into flash address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* QSPI/OCTOSPI io_base */

	buf_set_u32(reg_params[0].value, 0, 32, count);
	buf_set_u32(reg_params[1].value, 0, 32, pagesize);
	buf_set_u32(reg_params[2].value, 0, 32, offset);
	buf_set_u32(reg_params[3].value, 0, 32, stmqspi_info->io_base);


	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_algorithm(target,
		0, NULL,
		ARRAY_SIZE(reg_params), reg_params,
		algorithm->address, exit_point,
		(count >> 5) + 1000,
		&armv7m_info);
	keep_alive();

	image_calculate_checksum(buffer, count, &crc32);

	if (retval == ERROR_OK) {
		result = buf_get_u32(reg_params[0].value, 0, 32);
		LOG_DEBUG("addr " TARGET_ADDR_FMT ", len 0x%08" PRIx32 ", crc 0x%08" PRIx32 " 0x%08" PRIx32,
			offset + bank->base, count, ~crc32, result);
		if (~crc32 != result)
			retval = ERROR_FAIL;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

err:
	target_free_working_area(target, algorithm);

	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int qspi_read_write_block(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count, bool write)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	struct working_area *algorithm;
	uint32_t pagesize, fifo_start, fifosize, remaining;
	uint32_t maxsize, codesize, exit_point;
	const uint8_t *code = NULL;
	unsigned int dual;
	int retval;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " len=0x%08" PRIx32,
		__func__, offset, count);

	dual = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 1 : 0;

	/* see contrib/loaders/flash/stmqspi/stmqspi_read.S for src */
	static const uint8_t stmqspi_read_code[] = {
#include "../../../contrib/loaders/flash/stmqspi/stmqspi_read.inc"
	};

	/* see contrib/loaders/flash/stmqspi/stmoctospi_read.S for src */
	static const uint8_t stmoctospi_read_code[] = {
#include "../../../contrib/loaders/flash/stmqspi/stmoctospi_read.inc"
	};

	/* see contrib/loaders/flash/stmqspi/stmqspi_write.S for src */
	static const uint8_t stmqspi_write_code[] = {
#include "../../../contrib/loaders/flash/stmqspi/stmqspi_write.inc"
	};

	/* see contrib/loaders/flash/stmqspi/stmoctospi_write.S for src */
	static const uint8_t stmoctospi_write_code[] = {
#include "../../../contrib/loaders/flash/stmqspi/stmoctospi_write.inc"
	};

	/* This will overlay the last 12 words of stmqspi/stmoctospi_read/write_code in target */
	/* for read use the saved settings (memory mapped mode) but indirect read mode */
	uint32_t ccr_buffer[][4] = {
		/* cr  (not used for QSPI)			*
		 * ccr (for both QSPI and OCTOSPI)	*
		 * tcr (not used for QSPI)			*
		 * ir  (not used for QSPI)			*/
		{
			h_to_le_32(OCTOSPI_MODE | OCTOSPI_READ_MODE),
			h_to_le_32(IS_OCTOSPI ? OCTOSPI_CCR_READ_STATUS : QSPI_CCR_READ_STATUS),
			h_to_le_32((stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK) |
						(OPI_MODE ? (OPI_DUMMY << OCTOSPI_DCYC_POS) : 0)),
			h_to_le_32(OPI_CMD(SPIFLASH_READ_STATUS)),
		},
		{
			h_to_le_32(OCTOSPI_MODE | OCTOSPI_WRITE_MODE),
			h_to_le_32(IS_OCTOSPI ? OCTOSPI_CCR_WRITE_ENABLE : QSPI_CCR_WRITE_ENABLE),
			h_to_le_32(stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK),
			h_to_le_32(OPI_CMD(SPIFLASH_WRITE_ENABLE)),
		},
		{
			h_to_le_32(OCTOSPI_MODE | (write ? OCTOSPI_WRITE_MODE : OCTOSPI_READ_MODE)),
			h_to_le_32(write ? (IS_OCTOSPI ? OCTOSPI_CCR_PAGE_PROG : QSPI_CCR_PAGE_PROG) :
				(IS_OCTOSPI ? OCTOSPI_CCR_READ : QSPI_CCR_READ)),
			h_to_le_32(write ? (stmqspi_info->saved_tcr & ~OCTOSPI_DCYC_MASK) :
				stmqspi_info->saved_tcr),
			h_to_le_32(write ? OPI_CMD(stmqspi_info->dev.pprog_cmd) : stmqspi_info->saved_ir),
		},
	};

	/* force reasonable defaults */
	fifosize = stmqspi_info->dev.sectorsize ?
		stmqspi_info->dev.sectorsize : stmqspi_info->dev.size_in_bytes;

	if (write) {
		if (IS_OCTOSPI) {
			code = stmoctospi_write_code;
			codesize = sizeof(stmoctospi_write_code);
		} else {
			code = stmqspi_write_code;
			codesize = sizeof(stmqspi_write_code);
		}
	} else {
		if (IS_OCTOSPI) {
			code = stmoctospi_read_code;
			codesize = sizeof(stmoctospi_read_code);
		} else {
			code = stmqspi_read_code;
			codesize = sizeof(stmqspi_read_code);
		}
	}

	/* for write, pagesize must be taken into account */
	/* for read, the page size doesn't matter that much */
	pagesize = stmqspi_info->dev.pagesize;
	if (pagesize == 0)
		pagesize = (fifosize <= SPIFLASH_DEF_PAGESIZE) ?
			fifosize : SPIFLASH_DEF_PAGESIZE;

	/* adjust sizes according to dual flash mode */
	pagesize <<= dual;
	fifosize <<= dual;

	/* memory buffer, we assume sectorsize to be a power of 2 times pagesize */
	maxsize = target_get_working_area_avail(target);
	if (maxsize < codesize + 2 * sizeof(uint32_t) + pagesize) {
		LOG_ERROR("not enough working area, can't do QSPI page reads/writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* fifo size at most sector size, and multiple of page size */
	maxsize -= (codesize + 2 * sizeof(uint32_t));
	fifosize = ((maxsize < fifosize) ? maxsize : fifosize) & ~(pagesize - 1);

	if (target_alloc_working_area_try(target,
		codesize + 2 * sizeof(uint32_t) + fifosize, &algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* prepare flash write code, excluding ccr_buffer */
	retval = target_write_buffer(target, algorithm->address,
		codesize - sizeof(ccr_buffer), code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare QSPI/OCTOSPI_CCR register values */
	retval = target_write_buffer(target, algorithm->address
		+ codesize - sizeof(ccr_buffer),
		sizeof(ccr_buffer), (uint8_t *)ccr_buffer);
	if (retval != ERROR_OK)
		goto err;

	/* target buffer starts right after flash_write_code, i.e.
	 * wp and rp are implicitly included in buffer!!! */
	fifo_start = algorithm->address + codesize + 2 * sizeof(uint32_t);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* count (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* pagesize */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);	/* offset into flash address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* QSPI/OCTOSPI io_base */
	init_reg_param(&reg_params[4], "r8", 32, PARAM_OUT);	/* fifo start */
	init_reg_param(&reg_params[5], "r9", 32, PARAM_OUT);	/* fifo end + 1 */

	buf_set_u32(reg_params[0].value, 0, 32, count);
	buf_set_u32(reg_params[1].value, 0, 32, pagesize);
	buf_set_u32(reg_params[2].value, 0, 32, offset);
	buf_set_u32(reg_params[3].value, 0, 32, io_base);
	buf_set_u32(reg_params[4].value, 0, 32, fifo_start);
	buf_set_u32(reg_params[5].value, 0, 32, fifo_start + fifosize);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* after breakpoint instruction (halfword), one nop (halfword) and
	 * ccr_buffer follow till end of code */
	exit_point = algorithm->address + codesize
		- (sizeof(ccr_buffer) + sizeof(uint32_t));

	if (write) {
		retval = target_run_flash_async_algorithm(target, buffer, count, 1,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algorithm->address + codesize,
				fifosize + 2 * sizeof(uint32_t),
				algorithm->address, exit_point,
				&armv7m_info);
	} else {
		retval = target_run_read_async_algorithm(target, buffer, count, 1,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algorithm->address + codesize,
				fifosize + 2 * sizeof(uint32_t),
				algorithm->address, exit_point,
				&armv7m_info);
	}

	remaining = buf_get_u32(reg_params[0].value, 0, 32);
	if ((retval == ERROR_OK) && remaining)
		retval = ERROR_FLASH_OPERATION_FAILED;

	if (retval != ERROR_OK) {
		offset = buf_get_u32(reg_params[2].value, 0, 32);
		LOG_ERROR("flash %s failed at address 0x%" PRIx32 ", remaining 0x%" PRIx32,
			write ? "write" : "read", offset, remaining);
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

err:
	target_free_working_area(target, algorithm);

	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int stmqspi_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	int retval;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Read beyond end of flash. Extra data to be ignored.");
		count = bank->size - offset;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return qspi_read_write_block(bank, buffer, offset, count, false);
}

static int stmqspi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	unsigned int dual, sector;
	bool octal_dtr;
	int retval;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	dual = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 1 : 0;
	octal_dtr = IS_OCTOSPI && (stmqspi_info->saved_ccr & BIT(OCTOSPI_DDTR));

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Write beyond end of flash. Extra data discarded.");
		count = bank->size - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset < (bank->sectors[sector].offset + bank->sectors[sector].size)) &&
			((offset + count - 1) >= bank->sectors[sector].offset) &&
			bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FLASH_PROTECTED;
		}
	}

	if ((dual || octal_dtr) && ((offset & 1) != 0 || (count & 1) != 0)) {
		LOG_ERROR("In dual-QSPI and octal-DTR modes writes must be two byte aligned: "
			"%s: address=0x%08" PRIx32 " len=0x%08" PRIx32, __func__, offset, count);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return qspi_read_write_block(bank, (uint8_t *)buffer, offset, count, true);
}

static int stmqspi_verify(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	unsigned int dual;
	bool octal_dtr;
	int retval;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	dual = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 1 : 0;
	octal_dtr = IS_OCTOSPI && (stmqspi_info->saved_ccr & BIT(OCTOSPI_DDTR));

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(stmqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Verify beyond end of flash. Extra data ignored.");
		count = bank->size - offset;
	}

	if ((dual || octal_dtr) && ((offset & 1) != 0 || (count & 1) != 0)) {
		LOG_ERROR("In dual-QSPI and octal-DTR modes reads must be two byte aligned: "
			"%s: address=0x%08" PRIx32 " len=0x%08" PRIx32, __func__, offset, count);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return qspi_verify(bank, (uint8_t *)buffer, offset, count);
}

/* Find appropriate dummy setting, in particular octo mode */
static int find_sfdp_dummy(struct flash_bank *bank, int len)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint8_t data;
	unsigned int dual, count;
	bool flash1 = !(stmqspi_info->saved_cr & BIT(SPI_FSEL_FLASH));
	int retval;
	const unsigned int max_bytes = 64;

	dual = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 1 : 0;

	LOG_DEBUG("%s: len=%d, dual=%u, flash1=%d",
		__func__, len, dual, flash1);

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		stmqspi_info->saved_cr | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		goto err;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Switch to saved_cr (had to be set accordingly before this call) */
	retval = target_write_u32(target, io_base + SPI_CR, stmqspi_info->saved_cr);
	if (retval != ERROR_OK)
		goto err;

	/* Read at most that many bytes */
	retval = target_write_u32(target, io_base + SPI_DLR, (max_bytes << dual) - 1);
	if (retval != ERROR_OK)
		return retval;

	/* Read SFDP block */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_READ_MODE, OCTOSPI_CCR_READ_SFDP(len),
			SPIFLASH_READ_SFDP);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_READ_SFDP);
	if (retval != ERROR_OK)
		goto err;

	/* Read from start of sfdp block */
	retval = target_write_u32(target, io_base + SPI_AR, 0);
	if (retval != ERROR_OK)
		goto err;

	for (count = 0 ; count < max_bytes; count++) {
		if ((dual != 0) && !flash1) {
			/* discard even byte in dual flash-mode if flash2 */
			retval = target_read_u8(target, io_base + SPI_DR, &data);
			if (retval != ERROR_OK)
				goto err;
		}

		retval = target_read_u8(target, io_base + SPI_DR, &data);
		if (retval != ERROR_OK)
			goto err;

		if (data == 0x53) {
			LOG_DEBUG("start of SFDP header for flash%c after %u dummy bytes",
				flash1 ? '1' : '2', count);
			if (flash1)
				stmqspi_info->sfdp_dummy1 = count;
			else
				stmqspi_info->sfdp_dummy2 = count;
			return ERROR_OK;
		}

		if ((dual != 0) && flash1) {
			/* discard odd byte in dual flash-mode if flash1 */
			retval = target_read_u8(target, io_base + SPI_DR, &data);
			if (retval != ERROR_OK)
				goto err;
		}
	}

	retval = ERROR_FAIL;
	LOG_DEBUG("no start of SFDP header even after %u dummy bytes", count);

err:
	/* Abort operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));

	return retval;
}

/* Read SFDP parameter block */
static int read_sfdp_block(struct flash_bank *bank, uint32_t addr,
	uint32_t words, uint32_t *buffer)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	bool flash1 = !(stmqspi_info->saved_cr & BIT(SPI_FSEL_FLASH));
	unsigned int dual, count, len, *dummy;
	int retval;

	dual = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 1 : 0;

	if (IS_OCTOSPI && (((stmqspi_info->saved_ccr >> SPI_DMODE_POS) & 0x7) > 3)) {
		/* in OCTO mode 4-byte address and (yet) unknown number of dummy clocks */
		len = 4;

		/* in octo mode, use sfdp_dummy1 only */
		dummy = &stmqspi_info->sfdp_dummy1;
		if (*dummy == 0) {
			retval = find_sfdp_dummy(bank, len);
			if (retval != ERROR_OK)
				return retval;
		}
	} else {
		/* in all other modes 3-byte-address and 8(?) dummy clocks */
		len = 3;

		/* use sfdp_dummy1/2 according to currently selected flash */
		dummy = (stmqspi_info->saved_cr & BIT(SPI_FSEL_FLASH)) ?
			&stmqspi_info->sfdp_dummy2 : &stmqspi_info->sfdp_dummy1;

		/* according to SFDP standard, there should always be 8 dummy *CLOCKS*
		 * giving 1, 2 or 4 dummy *BYTES*, however, this is apparently not
		 * always implemented correctly, so determine the number of dummy bytes
		 * dynamically */
		if (*dummy == 0) {
			retval = find_sfdp_dummy(bank, len);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	LOG_DEBUG("%s: addr=0x%08" PRIx32 " words=0x%08" PRIx32 " dummy=%u",
		__func__, addr, words, *dummy);

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		stmqspi_info->saved_cr | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		goto err;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	/* Switch to one flash only */
	retval = target_write_u32(target, io_base + SPI_CR, stmqspi_info->saved_cr);
	if (retval != ERROR_OK)
		goto err;

	/* Read that many words plus dummy bytes */
	retval = target_write_u32(target, io_base + SPI_DLR,
		((*dummy + words * sizeof(uint32_t)) << dual) - 1);
	if (retval != ERROR_OK)
		goto err;

	/* Read SFDP block */
	if (IS_OCTOSPI)
		retval = OCTOSPI_CMD(OCTOSPI_READ_MODE, OCTOSPI_CCR_READ_SFDP(len),
			SPIFLASH_READ_SFDP);
	else
		retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_READ_SFDP);
	if (retval != ERROR_OK)
		goto err;

	retval = target_write_u32(target, io_base + SPI_AR, addr << dual);
	if (retval != ERROR_OK)
		goto err;

	/* dummy clocks */
	for (count = *dummy << dual; count > 0; --count) {
		retval = target_read_u8(target, io_base + SPI_DR, (uint8_t *)buffer);
		if (retval != ERROR_OK)
			goto err;
	}

	for ( ; words > 0; words--) {
		if (dual != 0) {
			uint32_t word1, word2;

			retval = target_read_u32(target, io_base + SPI_DR, &word1);
			if (retval != ERROR_OK)
				goto err;
			retval = target_read_u32(target, io_base + SPI_DR, &word2);
			if (retval != ERROR_OK)
				goto err;

			if (!flash1) {
				/* shift odd numbered bytes into even numbered ones */
				word1 >>= 8;
				word2 >>= 8;
			}

			/* pack even numbered bytes into one word */
			*buffer = (word1 & 0xFFU) | ((word1 & 0xFF0000U) >> 8) |
				((word2 & 0xFFU) << 16) | ((word2 & 0xFF0000U) << 8);


		} else {
			retval = target_read_u32(target, io_base + SPI_DR, buffer);
			if (retval != ERROR_OK)
				goto err;
		}
		LOG_DEBUG("raw SFDP data 0x%08" PRIx32, *buffer);

		/* endian correction, sfdp data is always le uint32_t based */
		*buffer = le_to_h_u32((uint8_t *)buffer);
		buffer++;
	}

err:
	return retval;
}

/* Return ID of flash device(s) */
/* On exit, indirect mode is kept */
static int read_flash_id(struct flash_bank *bank, uint32_t *id1, uint32_t *id2)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	uint32_t io_base = stmqspi_info->io_base;
	uint8_t byte;
	unsigned int type, count, len1, len2;
	int retval = ERROR_OK;

	/* invalidate both ids */
	*id1 = 0;
	*id2 = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* SPIFLASH_READ_MID causes device in octal mode to go berserk, so don't use in this case */
	for (type = (IS_OCTOSPI && OPI_MODE) ? 1 : 0; type < 2 ; type++) {
		/* Abort any previous operation */
		retval = target_write_u32(target, io_base + SPI_CR,
			READ_REG(SPI_CR) | BIT(SPI_ABORT));
		if (retval != ERROR_OK)
			goto err;

		/* Poll WIP */
		retval = wait_till_ready(bank, SPI_PROBE_TIMEOUT);
		if (retval != ERROR_OK)
			goto err;

		/* Wait for busy to be cleared */
		retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
		if (retval != ERROR_OK)
			goto err;

		/* Read at most 16 bytes per chip */
		count = 16;
		retval = target_write_u32(target, io_base + SPI_DLR,
			(stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH) ? count * 2 : count) - 1);
		if (retval != ERROR_OK)
			goto err;

		/* Read id: one particular flash chip (N25Q128) switches back to SPI mode when receiving
		 * SPI_FLASH_READ_ID in QPI mode, hence try SPIFLASH_READ_MID first */
		switch (type) {
			case 0:
				if (IS_OCTOSPI)
					retval = OCTOSPI_CMD(OCTOSPI_READ_MODE, OCTOSPI_CCR_READ_MID, SPIFLASH_READ_MID);
				else
					retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_READ_MID);
				break;

			case 1:
				if (IS_OCTOSPI)
					retval = OCTOSPI_CMD(OCTOSPI_READ_MODE, OCTOSPI_CCR_READ_ID, SPIFLASH_READ_ID);
				else
					retval = target_write_u32(target, io_base + QSPI_CCR, QSPI_CCR_READ_ID);
				break;

			default:
				return ERROR_FAIL;
		}

		if (retval != ERROR_OK)
			goto err;

		/* Dummy address 0, only required for 8-line mode */
		if (IS_OCTOSPI && OPI_MODE) {
			retval = target_write_u32(target, io_base + SPI_AR, 0);
			if (retval != ERROR_OK)
				goto err;
		}

		/* for debugging only */
		(void)READ_REG(SPI_SR);

		/* Read ID from Data Register */
		for (len1 = 0, len2 = 0; count > 0; --count) {
			if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) |
				BIT(SPI_FSEL_FLASH))) != BIT(SPI_FSEL_FLASH)) {
				retval = target_read_u8(target, io_base + SPI_DR, &byte);
				if (retval != ERROR_OK)
					goto err;
				/* collect 3 bytes without continuation codes */
				if ((byte != 0x7F) && (len1 < 3)) {
					*id1 = (*id1 >> 8) | ((uint32_t)byte) << 16;
					len1++;
				}
			}
			if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) |
				BIT(SPI_FSEL_FLASH))) != 0) {
				retval = target_read_u8(target, io_base + SPI_DR, &byte);
				if (retval != ERROR_OK)
					goto err;
				/* collect 3 bytes without continuation codes */
				if ((byte != 0x7F) && (len2 < 3)) {
					*id2 = (*id2 >> 8) | ((uint32_t)byte) << 16;
					len2++;
				}
			}
		}

		if (((*id1 != 0x000000) && (*id1 != 0xFFFFFF)) ||
			((*id2 != 0x000000) && (*id2 != 0xFFFFFF)))
			break;
	}

	if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) |
		BIT(SPI_FSEL_FLASH))) != BIT(SPI_FSEL_FLASH)) {
		if ((*id1 == 0x000000) || (*id1 == 0xFFFFFF)) {
			/* no id retrieved, so id must be set manually */
			LOG_INFO("No id from flash1");
			retval = ERROR_FLASH_BANK_NOT_PROBED;
		}
	}

	if ((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) | BIT(SPI_FSEL_FLASH))) != 0) {
		if ((*id2 == 0x000000) || (*id2 == 0xFFFFFF)) {
			/* no id retrieved, so id must be set manually */
			LOG_INFO("No id from flash2");
			retval = ERROR_FLASH_BANK_NOT_PROBED;
		}
	}

err:
	return retval;
}

static int stmqspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	uint32_t io_base = stmqspi_info->io_base;
	uint32_t id1 = 0, id2 = 0, data = 0;
	const struct flash_device *p;
	const uint32_t magic = 0xAEF1510E;
	unsigned int dual, fsize;
	bool octal_dtr;
	int retval;

	if (stmqspi_info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		free(bank->sectors);
		bank->sectors = NULL;
		memset(&stmqspi_info->dev, 0, sizeof(stmqspi_info->dev));
		stmqspi_info->sfdp_dummy1 = 0;
		stmqspi_info->sfdp_dummy2 = 0;
		stmqspi_info->probed = false;
	}

	/* Abort any previous operation */
	retval = target_write_u32(target, io_base + SPI_CR,
		READ_REG(SPI_CR) | BIT(SPI_ABORT));
	if (retval != ERROR_OK)
		return retval;

	/* Wait for busy to be cleared */
	retval = poll_busy(bank, SPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* check whether QSPI_ABR is writeable and readback returns the value written */
	retval = target_write_u32(target, io_base + QSPI_ABR, magic);
	if (retval == ERROR_OK) {
		retval = target_read_u32(target, io_base + QSPI_ABR, &data);
		retval = target_write_u32(target, io_base + QSPI_ABR, 0);
	}

	if (data == magic) {
		LOG_DEBUG("QSPI_ABR register present");
		stmqspi_info->octo = false;
	} else if (READ_REG(OCTOSPI_MAGIC) == OCTO_MAGIC_ID) {
		LOG_DEBUG("OCTOSPI_MAGIC present");
		stmqspi_info->octo = true;
	} else {
		LOG_ERROR("No QSPI, no OCTOSPI at 0x%08" PRIx32, io_base);
		stmqspi_info->probed = false;
		stmqspi_info->dev.name = "none";
		return ERROR_FAIL;
	}

	/* save current FSEL and DFM bits in QSPI/OCTOSPI_CR, current QSPI/OCTOSPI_CCR value */
	stmqspi_info->saved_cr = READ_REG(SPI_CR);
	if (retval == ERROR_OK)
		stmqspi_info->saved_ccr = READ_REG(SPI_CCR);

	if (IS_OCTOSPI) {
		uint32_t mtyp;

		mtyp = ((READ_REG(OCTOSPI_DCR1) & OCTOSPI_MTYP_MASK)) >> OCTOSPI_MTYP_POS;
		if (retval == ERROR_OK)
			stmqspi_info->saved_tcr = READ_REG(OCTOSPI_TCR);
		if (retval == ERROR_OK)
			stmqspi_info->saved_ir = READ_REG(OCTOSPI_IR);
		if ((mtyp != 0x0) && (mtyp != 0x1)) {
			retval = ERROR_FAIL;
			LOG_ERROR("Only regular SPI protocol supported in OCTOSPI");
		}
		if (retval == ERROR_OK) {
			LOG_DEBUG("OCTOSPI at 0x%08" PRIx64 ", io_base at 0x%08" PRIx32 ", OCTOSPI_CR 0x%08"
				PRIx32 ", OCTOSPI_CCR 0x%08" PRIx32 ", %d-byte addr", bank->base, io_base,
				stmqspi_info->saved_cr, stmqspi_info->saved_ccr, SPI_ADSIZE);
		} else {
			LOG_ERROR("No OCTOSPI at io_base 0x%08" PRIx32, io_base);
			stmqspi_info->probed = false;
			stmqspi_info->dev.name = "none";
			return ERROR_FAIL;
		}
	} else {
		if (retval == ERROR_OK) {
			LOG_DEBUG("QSPI at 0x%08" PRIx64 ", io_base at 0x%08" PRIx32 ", QSPI_CR 0x%08"
				PRIx32 ", QSPI_CCR 0x%08" PRIx32 ", %d-byte addr", bank->base, io_base,
				stmqspi_info->saved_cr, stmqspi_info->saved_ccr, SPI_ADSIZE);
				if (stmqspi_info->saved_ccr & (1U << QSPI_DDRM))
					LOG_WARNING("DDR mode is untested and suffers from some silicon bugs");
		} else {
			LOG_ERROR("No QSPI at io_base 0x%08" PRIx32, io_base);
			stmqspi_info->probed = false;
			stmqspi_info->dev.name = "none";
			return ERROR_FAIL;
		}
	}

	dual = (stmqspi_info->saved_cr & BIT(SPI_DUAL_FLASH)) ? 1 : 0;
	octal_dtr = IS_OCTOSPI && (stmqspi_info->saved_ccr & BIT(OCTOSPI_DDTR));
	if (dual || octal_dtr)
		bank->write_start_alignment = bank->write_end_alignment = 2;
	else
		bank->write_start_alignment = bank->write_end_alignment = 1;

	/* read and decode flash ID; returns in indirect mode */
	retval = read_flash_id(bank, &id1, &id2);
	LOG_DEBUG("id1 0x%06" PRIx32 ", id2 0x%06" PRIx32, id1, id2);
	if (retval == ERROR_FLASH_BANK_NOT_PROBED) {
		/* no id retrieved, so id must be set manually */
		LOG_INFO("No id - set flash parameters manually");
		retval = ERROR_OK;
		goto err;
	}

	if (retval != ERROR_OK)
		goto err;

	/* identify flash1 */
	for (p = flash_devices; id1 && p->name ; p++) {
		if (p->device_id == id1) {
			memcpy(&stmqspi_info->dev, p, sizeof(stmqspi_info->dev));
			if (p->size_in_bytes / 4096)
				LOG_INFO("flash1 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
					"kbytes", p->name, id1, p->size_in_bytes / 1024);
			else
				LOG_INFO("flash1 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
					"bytes", p->name, id1, p->size_in_bytes);
			break;
		}
	}

	if (id1 && !p->name) {
		/* chip not been identified by id, then try SFDP */
		struct flash_device temp;
		uint32_t saved_cr = stmqspi_info->saved_cr;

		/* select flash1 */
		stmqspi_info->saved_cr = stmqspi_info->saved_cr & ~BIT(SPI_FSEL_FLASH);
		retval = spi_sfdp(bank, &temp, &read_sfdp_block);

		/* restore saved_cr */
		stmqspi_info->saved_cr = saved_cr;

		if (retval == ERROR_OK) {
			LOG_INFO("flash1 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
				"kbytes", temp.name, id1, temp.size_in_bytes / 1024);
			/* save info and retrieved *good* id as spi_sfdp clears all info */
			memcpy(&stmqspi_info->dev, &temp, sizeof(stmqspi_info->dev));
			stmqspi_info->dev.device_id = id1;
		} else {
			/* even not identified by SFDP, then give up */
			LOG_WARNING("Unknown flash1 device id = 0x%06" PRIx32
				" - set flash parameters manually", id1);
			retval = ERROR_OK;
			goto err;
		}
	}

	/* identify flash2 */
	for (p = flash_devices; id2 && p->name ; p++) {
		if (p->device_id == id2) {
			if (p->size_in_bytes / 4096)
				LOG_INFO("flash2 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
					"kbytes", p->name, id2, p->size_in_bytes / 1024);
			else
				LOG_INFO("flash2 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
					"bytes", p->name, id2, p->size_in_bytes);

			if (!id1)
				memcpy(&stmqspi_info->dev, p, sizeof(stmqspi_info->dev));
			else {
				if ((stmqspi_info->dev.read_cmd != p->read_cmd) ||
					(stmqspi_info->dev.qread_cmd != p->qread_cmd) ||
					(stmqspi_info->dev.pprog_cmd != p->pprog_cmd) ||
					(stmqspi_info->dev.erase_cmd != p->erase_cmd) ||
					(stmqspi_info->dev.chip_erase_cmd != p->chip_erase_cmd) ||
					(stmqspi_info->dev.sectorsize != p->sectorsize) ||
					(stmqspi_info->dev.size_in_bytes != p->size_in_bytes)) {
					LOG_ERROR("Incompatible flash1/flash2 devices");
					goto err;
				}
				/* page size is optional in SFDP, so accept smallest value */
				if (p->pagesize < stmqspi_info->dev.pagesize)
					stmqspi_info->dev.pagesize = p->pagesize;
			}
			break;
		}
	}

	if (id2 && !p->name) {
		/* chip not been identified by id, then try SFDP */
		struct flash_device temp;
		uint32_t saved_cr = stmqspi_info->saved_cr;

		/* select flash2 */
		stmqspi_info->saved_cr = stmqspi_info->saved_cr | BIT(SPI_FSEL_FLASH);
		retval = spi_sfdp(bank, &temp, &read_sfdp_block);

		/* restore saved_cr */
		stmqspi_info->saved_cr = saved_cr;

		if (retval == ERROR_OK)
			LOG_INFO("flash2 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
				"kbytes", temp.name, id2, temp.size_in_bytes / 1024);
		else {
			/* even not identified by SFDP, then give up */
			LOG_WARNING("Unknown flash2 device id = 0x%06" PRIx32
				" - set flash parameters manually", id2);
			retval = ERROR_OK;
			goto err;
		}

		if (!id1)
			memcpy(&stmqspi_info->dev, &temp, sizeof(stmqspi_info->dev));
		else {
			if ((stmqspi_info->dev.read_cmd != temp.read_cmd) ||
				(stmqspi_info->dev.qread_cmd != temp.qread_cmd) ||
				(stmqspi_info->dev.pprog_cmd != temp.pprog_cmd) ||
				(stmqspi_info->dev.erase_cmd != temp.erase_cmd) ||
				(stmqspi_info->dev.chip_erase_cmd != temp.chip_erase_cmd) ||
				(stmqspi_info->dev.sectorsize != temp.sectorsize) ||
				(stmqspi_info->dev.size_in_bytes != temp.size_in_bytes)) {
				LOG_ERROR("Incompatible flash1/flash2 devices");
				goto err;
			}
			/* page size is optional in SFDP, so accept smallest value */
			if (temp.pagesize < stmqspi_info->dev.pagesize)
				stmqspi_info->dev.pagesize = temp.pagesize;
		}
	}

	/* Set correct size value */
	bank->size = stmqspi_info->dev.size_in_bytes << dual;

	fsize = ((READ_REG(SPI_DCR) >> SPI_FSIZE_POS) & (BIT(SPI_FSIZE_LEN) - 1));
	if (retval != ERROR_OK)
		goto err;

	LOG_DEBUG("FSIZE = 0x%04x", fsize);
	if (bank->size == BIT((fsize + 1)))
		LOG_DEBUG("FSIZE in DCR(1) matches actual capacity. Beware of silicon bug in H7, L4+, MP1.");
	else if (bank->size == BIT((fsize + 0)))
		LOG_DEBUG("FSIZE in DCR(1) is off by one regarding actual capacity. Fix for silicon bug?");
	else
		LOG_ERROR("FSIZE in DCR(1) doesn't match actual capacity.");

	/* if no sectors, then treat whole flash as single sector */
	if (stmqspi_info->dev.sectorsize == 0)
		stmqspi_info->dev.sectorsize = stmqspi_info->dev.size_in_bytes;
	/* if no page_size, then use sectorsize as page_size */
	if (stmqspi_info->dev.pagesize == 0)
		stmqspi_info->dev.pagesize = stmqspi_info->dev.sectorsize;

	/* create and fill sectors array */
	bank->num_sectors = stmqspi_info->dev.size_in_bytes / stmqspi_info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		retval = ERROR_FAIL;
		goto err;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * (stmqspi_info->dev.sectorsize << dual);
		sectors[sector].size = (stmqspi_info->dev.sectorsize << dual);
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	stmqspi_info->probed = true;

err:
	/* Switch to memory mapped mode before return to prompt */
	set_mm_mode(bank);

	return retval;
}

static int stmqspi_auto_probe(struct flash_bank *bank)
{
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;

	if (stmqspi_info->probed)
		return ERROR_OK;
	stmqspi_probe(bank);
	return ERROR_OK;
}

static int stmqspi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_stmqspi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stmqspi_flash_bank *stmqspi_info = bank->driver_priv;

	if (!(stmqspi_info->probed)) {
		snprintf(buf, buf_size,
			"\nQSPI flash bank not probed yet\n");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	snprintf(buf, buf_size, "flash%s%s \'%s\', device id = 0x%06" PRIx32
			", flash size = %" PRIu32 "%sbytes\n(page size = %" PRIu32
			", read = 0x%02" PRIx8 ", qread = 0x%02" PRIx8
			", pprog = 0x%02" PRIx8 ", mass_erase = 0x%02" PRIx8
			", sector size = %" PRIu32 "%sbytes, sector_erase = 0x%02" PRIx8 ")",
			((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) |
			BIT(SPI_FSEL_FLASH))) != BIT(SPI_FSEL_FLASH)) ? "1" : "",
			((stmqspi_info->saved_cr & (BIT(SPI_DUAL_FLASH) |
			BIT(SPI_FSEL_FLASH))) != 0) ? "2" : "",
			stmqspi_info->dev.name, stmqspi_info->dev.device_id,
			bank->size / 4096 ? bank->size / 1024 : bank->size,
			bank->size / 4096 ? "k" : "", stmqspi_info->dev.pagesize,
			stmqspi_info->dev.read_cmd, stmqspi_info->dev.qread_cmd,
			stmqspi_info->dev.pprog_cmd, stmqspi_info->dev.chip_erase_cmd,
			stmqspi_info->dev.sectorsize / 4096 ?
				stmqspi_info->dev.sectorsize / 1024 : stmqspi_info->dev.sectorsize,
			stmqspi_info->dev.sectorsize / 4096 ? "k" : "",
			stmqspi_info->dev.erase_cmd);

	return ERROR_OK;
}

static const struct command_registration stmqspi_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = stmqspi_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	{
		.name = "set",
		.handler = stmqspi_handle_set,
		.mode = COMMAND_EXEC,
		.usage = "bank_id name chip_size page_size read_cmd qread_cmd pprg_cmd "
			"[ mass_erase_cmd ] [ sector_size sector_erase_cmd ]",
		.help = "Set params of single flash chip",
	},
	{
		.name = "cmd",
		.handler = stmqspi_handle_cmd,
		.mode = COMMAND_EXEC,
		.usage = "bank_id num_resp cmd_byte ...",
		.help = "Send low-level command cmd_byte and following bytes or read num_resp.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stmqspi_command_handlers[] = {
	{
		.name = "stmqspi",
		.mode = COMMAND_ANY,
		.help = "stmqspi flash command group",
		.usage = "",
		.chain = stmqspi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stmqspi_flash = {
	.name = "stmqspi",
	.commands = stmqspi_command_handlers,
	.flash_bank_command = stmqspi_flash_bank_command,
	.erase = stmqspi_erase,
	.protect = stmqspi_protect,
	.write = stmqspi_write,
	.read = stmqspi_read,
	.verify = stmqspi_verify,
	.probe = stmqspi_probe,
	.auto_probe = stmqspi_auto_probe,
	.erase_check = stmqspi_blank_check,
	.protect_check = stmqspi_protect_check,
	.info = get_stmqspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
