/***************************************************************************
 *   Copyright (C) 2017 by STMicroelectronics                              *
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
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>


/* Erase time can be as high as 1000ms, 10x this and it's toast... */
#define FLASH_ERASE_TIMEOUT 10000
#define FLASH_WRITE_TIMEOUT 5

/* RM 433 */
/* Same Flash registers for both banks, */
/* access depends on Flash Base address */
#define FLASH_ACR       0x00
#define FLASH_KEYR      0x04
#define FLASH_OPTKEYR   0x08
#define FLASH_CR        0x0C
#define FLASH_SR        0x10
#define FLASH_CCR       0x14
#define FLASH_OPTCR     0x18
#define FLASH_OPTSR_CUR 0x1C
#define FLASH_OPTSR_PRG 0x20
#define FLASH_OPTCCR    0x24
#define FLASH_WPSN_CUR  0x38
#define FLASH_WPSN_PRG  0x3C


/* FLASH_CR register bits */
#define FLASH_LOCK     (1 << 0)
#define FLASH_PG       (1 << 1)
#define FLASH_SER      (1 << 2)
#define FLASH_BER      (1 << 3)
#define FLASH_PSIZE_8  (0 << 4)
#define FLASH_PSIZE_16 (1 << 4)
#define FLASH_PSIZE_32 (2 << 4)
#define FLASH_PSIZE_64 (3 << 4)
#define FLASH_FW       (1 << 6)
#define FLASH_START    (1 << 7)

#define FLASH_SNB(a)   ((a) << 8)

/* FLASH_SR register bits */
#define FLASH_BSY      (1 << 0)  /* Operation in progress */
#define FLASH_QW       (1 << 2)  /* Operation queue in progress */
#define FLASH_WRPERR   (1 << 17) /* Write protection error */
#define FLASH_PGSERR   (1 << 18) /* Programming sequence error */
#define FLASH_STRBERR  (1 << 19) /* Strobe error */
#define FLASH_INCERR   (1 << 21) /* Inconsistency error */
#define FLASH_OPERR    (1 << 22) /* Operation error */
#define FLASH_RDPERR   (1 << 23) /* Read Protection error */
#define FLASH_RDSERR   (1 << 24) /* Secure Protection error */
#define FLASH_SNECCERR (1 << 25) /* Single ECC error */
#define FLASH_DBECCERR (1 << 26) /* Double ECC error */

#define FLASH_ERROR (FLASH_WRPERR | FLASH_PGSERR | FLASH_STRBERR | FLASH_INCERR | FLASH_OPERR | \
					 FLASH_RDPERR | FLASH_RDSERR | FLASH_SNECCERR | FLASH_DBECCERR)

/* FLASH_OPTCR register bits */
#define OPT_LOCK       (1 << 0)
#define OPT_START      (1 << 1)

/* register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

#define DBGMCU_IDCODE_REGISTER  0x5C001000
#define FLASH_BANK0_ADDRESS     0x08000000
#define FLASH_BANK1_ADDRESS     0x08100000
#define FLASH_REG_BASE_B0       0x52002000
#define FLASH_REG_BASE_B1       0x52002100
#define FLASH_SIZE_ADDRESS      0x1FF1E880
#define FLASH_BLOCK_SIZE        32

struct stm32h7x_rev {
	uint16_t rev;
	const char *str;
};

struct stm32x_options {
	uint8_t RDP;
	uint32_t protection;  /* bank1 WRP */
	uint32_t protection2; /* bank2 WRP */
	uint8_t user_options;
	uint8_t user2_options;
	uint8_t user3_options;
};

struct stm32h7x_part_info {
	uint16_t id;
	const char *device_str;
	const struct stm32h7x_rev *revs;
	size_t num_revs;
	unsigned int page_size;
	uint16_t max_flash_size_kb;
	uint8_t has_dual_bank;
	uint16_t first_bank_size_kb; /* Used when has_dual_bank is true */
	uint32_t flash_base;         /* Flash controller registers location */
	uint32_t fsize_base;         /* Location of FSIZE register */
};

struct stm32h7x_flash_bank {
	int probed;
	uint32_t idcode;
	uint32_t user_bank_size;
	uint32_t flash_base;    /* Address of flash reg controller */
	struct stm32x_options option_bytes;
	const struct stm32h7x_part_info *part_info;
};

static const struct stm32h7x_rev stm32_450_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x1003, "Y" }, { 0x2001, "X"  },
};

static const struct stm32h7x_part_info stm32h7x_parts[] = {
	{
	.id					= 0x450,
	.revs				= stm32_450_revs,
	.num_revs			= ARRAY_SIZE(stm32_450_revs),
	.device_str			= "STM32H74x/75x",
	.page_size			= 128,  /* 128 KB */
	.max_flash_size_kb	= 2048,
	.first_bank_size_kb	= 1024,
	.has_dual_bank		= 1,
	.flash_base			= FLASH_REG_BASE_B0,
	.fsize_base			= FLASH_SIZE_ADDRESS,
	},
};

static int stm32x_unlock_reg(struct flash_bank *bank);
static int stm32x_lock_reg(struct flash_bank *bank);
static int stm32x_probe(struct flash_bank *bank);

/* flash bank stm32x <base> <size> 0 0 <target#> */

FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32h7x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32x_info = malloc(sizeof(struct stm32h7x_flash_bank));
	bank->driver_priv = stm32x_info;

	stm32x_info->probed = 0;
	stm32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline uint32_t stm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	return reg + stm32x_info->flash_base;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, stm32x_get_flash_reg(bank, FLASH_SR), status);
}

static int stm32x_wait_flash_op_queue(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	uint32_t status;
	int retval;

	/* wait for flash operations completion */
	for (;;) {
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK) {
			LOG_INFO("wait_flash_op_queue, target_read_u32 : error : remote address 0x%x", stm32x_info->flash_base);
			return retval;
		}

		if ((status & FLASH_QW) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_INFO("wait_flash_op_queue, time out expired, status: 0x%" PRIx32 "", status);
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPERR) {
		LOG_INFO("wait_flash_op_queue, WRPERR : error : remote address 0x%x", stm32x_info->flash_base);
		retval = ERROR_FAIL;
	}

	/* Clear error + EOP flags but report errors */
	if (status & FLASH_ERROR) {
		if (retval == ERROR_OK)
			retval = ERROR_FAIL;
		/* If this operation fails, we ignore it and report the original retval */
		target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CCR), status);
	}
	return retval;
}

static int stm32x_unlock_reg(struct flash_bank *bank)
{
	uint32_t ctrl;
	struct target *target = bank->target;

	/* first check if not already unlocked
	 * otherwise writing on FLASH_KEYR will fail
	 */
	int retval = target_read_u32(target, stm32x_get_flash_reg(bank, FLASH_CR), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers for bank */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, stm32x_get_flash_reg(bank, FLASH_CR), &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_LOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CRx: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}
	return ERROR_OK;
}

static int stm32x_unlock_option_reg(struct flash_bank *bank)
{
	uint32_t ctrl;
	struct target *target = bank->target;

	int retval = target_read_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & OPT_LOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & OPT_LOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_OPTCR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32x_lock_reg(struct flash_bank *bank)
{
	struct target *target = bank->target;

	/* Lock bank reg */
	int retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct stm32h7x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTSR_CUR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* decode option data */
	stm32x_info->option_bytes.user_options = optiondata & 0xfc;
	stm32x_info->option_bytes.RDP = (optiondata >> 8) & 0xff;
	stm32x_info->option_bytes.user2_options = (optiondata >> 16) & 0xff;
	stm32x_info->option_bytes.user3_options = (optiondata >> 24) & 0xa3;

	if (stm32x_info->option_bytes.RDP != 0xAA)
		LOG_INFO("Device Security Bit Set");

	/* read current WPSN option bytes */
	retval = target_read_u32(target, FLASH_REG_BASE_B0 + FLASH_WPSN_CUR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32x_info->option_bytes.protection = optiondata & 0xff;

	/* read current WPSN2 option bytes */
	retval = target_read_u32(target, FLASH_REG_BASE_B1 + FLASH_WPSN_CUR, &optiondata);
	if (retval != ERROR_OK)
		return retval;
	stm32x_info->option_bytes.protection2 = optiondata & 0xff;

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32h7x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint32_t optiondata;

	stm32x_info = bank->driver_priv;

	int retval = stm32x_unlock_option_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* rebuild option data */
	optiondata = stm32x_info->option_bytes.user_options;
	optiondata |= (stm32x_info->option_bytes.RDP << 8);
	optiondata |= (stm32x_info->option_bytes.user2_options & 0xff) << 16;
	optiondata |= (stm32x_info->option_bytes.user3_options & 0xa3) << 24;

	/* program options */
	retval = target_write_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTSR_PRG, optiondata);
	if (retval != ERROR_OK)
		return retval;

	optiondata = stm32x_info->option_bytes.protection & 0xff;
	/* Program protection WPSNPRG */
	retval = target_write_u32(target, FLASH_REG_BASE_B0 + FLASH_WPSN_PRG, optiondata);
	if (retval != ERROR_OK)
		return retval;

	optiondata = stm32x_info->option_bytes.protection2 & 0xff;
	/* Program protection WPSNPRG2 */
	retval = target_write_u32(target, FLASH_REG_BASE_B1 + FLASH_WPSN_PRG, optiondata);
	if (retval != ERROR_OK)
		return retval;

	optiondata = 0x40000000;
	/* Remove OPT error flag before programming */
	retval = target_write_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTCCR, optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* start programming cycle */
	retval = target_write_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTCR, OPT_START);
	if (retval != ERROR_OK)
		return retval;

	/* wait for completion */
	int timeout = FLASH_ERASE_TIMEOUT;
	for (;;) {
		uint32_t status;
		retval = target_read_u32(target, FLASH_REG_BASE_B0 + FLASH_SR, &status);
		if (retval != ERROR_OK) {
			LOG_INFO("stm32x_write_options: wait_flash_op_queue : error");
			return retval;
		}
		if ((status & FLASH_QW) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_INFO("wait_flash_op_queue, time out expired, status: 0x%" PRIx32 "", status);
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	/* relock option registers */
	retval = target_write_u32(target, FLASH_REG_BASE_B0 + FLASH_OPTCR, OPT_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;

	/* read 'write protection' settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = 0; i < bank->num_sectors; i++) {
		if (stm32x_info->flash_base == FLASH_REG_BASE_B0) {
			if (stm32x_info->option_bytes.protection & (1 << i))
				bank->sectors[i].is_protected = 0;
			else
				bank->sectors[i].is_protected = 1;
		} else {
			if (stm32x_info->option_bytes.protection2 & (1 << i))
				bank->sectors[i].is_protected = 0;
			else
				bank->sectors[i].is_protected = 1;
		}
	}
	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int retval;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by checking the QW bit in the
	  FLASH_SR register
	2. Set the SER bit and select the sector
	  you wish to erase (SNB) in the FLASH_CR register
	3. Set the STRT bit in the FLASH_CR register
	4. Wait for flash operations completion
	 */
	for (int i = first; i <= last; i++) {
		LOG_DEBUG("erase sector %d", i);
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR),
				FLASH_SER | FLASH_SNB(i) | FLASH_PSIZE_64);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error erase sector %d", i);
			return retval;
		}
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR),
				FLASH_SER | FLASH_SNB(i) | FLASH_PSIZE_64 | FLASH_START);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error erase sector %d", i);
			return retval;
		}
		retval = stm32x_wait_flash_op_queue(bank, FLASH_ERASE_TIMEOUT);

		if (retval != ERROR_OK) {
			LOG_ERROR("erase time-out or operation error sector %d", i);
			return retval;
		}
		bank->sectors[i].is_erased = 1;
	}

	retval = stm32x_lock_reg(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("error during the lock of flash");
		return retval;
	}

	return ERROR_OK;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	/* read protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	for (int i = first; i <= last; i++) {
		if (stm32x_info->flash_base == FLASH_REG_BASE_B0) {
			if (set)
				stm32x_info->option_bytes.protection &= ~(1 << i);
			else
				stm32x_info->option_bytes.protection |= (1 << i);
		} else {
			if (set)
				stm32x_info->option_bytes.protection2 &= ~(1 << i);
			else
				stm32x_info->option_bytes.protection2 |= (1 << i);
		}
	}

	LOG_INFO("stm32x_protect, option_bytes written WRP1 0x%x , WRP2 0x%x",
	  (stm32x_info->option_bytes.protection & 0xff), (stm32x_info->option_bytes.protection2 & 0xff));

	retval = stm32x_write_options(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	/*
	 * If the size of the data part of the buffer is not a multiple of FLASH_BLOCK_SIZE, we get
	 * "corrupted fifo read" pointer in target_run_flash_async_algorithm()
	 */
	uint32_t data_size = 512 * FLASH_BLOCK_SIZE;	/* 16384 */
	uint32_t buffer_size = 8 + data_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	int retval = ERROR_OK;

	static const uint8_t stm32x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32h7x.inc"
	};

	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32x_flash_write_code),
			stm32x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		data_size /= 2;
		buffer_size = 8 + data_size;
		if (data_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	LOG_DEBUG("target_alloc_working_area_try : buffer_size -> 0x%x", buffer_size);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count (word-256 bits) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);		/* flash reg base */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, stm32x_info->flash_base);

	retval = target_run_flash_async_algorithm(target,
						  buffer,
						  count,
						  FLASH_BLOCK_SIZE,
						  0, NULL,
						  5, reg_params,
						  source->address, source->size,
						  write_algorithm->address, 0,
						  &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_INFO("error executing stm32h7x flash write algorithm");

		uint32_t flash_sr = buf_get_u32(reg_params[0].value, 0, 32);

		if (flash_sr & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if ((flash_sr & FLASH_ERROR) != 0) {
			LOG_ERROR("flash write failed, FLASH_SR = %08" PRIx32, flash_sr);
			/* Clear error + EOP flags but report errors */
			target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CCR), flash_sr);
			retval = ERROR_FAIL;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	return retval;
}

static int stm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t address = bank->base + offset;
	int retval, retval2;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset % FLASH_BLOCK_SIZE) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 32-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t blocks_remaining = count / FLASH_BLOCK_SIZE;
	uint32_t bytes_remaining = count % FLASH_BLOCK_SIZE;

	/* multiple words (32-bytes) to be programmed in block */
	if (blocks_remaining) {
		retval = stm32x_write_block(bank, buffer, offset, blocks_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
		} else {
			buffer += blocks_remaining * FLASH_BLOCK_SIZE;
			address += blocks_remaining * FLASH_BLOCK_SIZE;
			blocks_remaining = 0;
		}
		if ((retval != ERROR_OK) && (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE))
			goto flash_lock;
	}

	/*
	Standard programming
	The Flash memory programming sequence is as follows:
	1. Check that no main Flash memory operation is ongoing by checking the QW bit in the
	   FLASH_SR register.
	2. Set the PG bit in the FLASH_CR register
	3. 8 x Word access (or Force Write FW)
	4. Wait for flash operations completion
	*/
	while (blocks_remaining > 0) {
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR), FLASH_PG | FLASH_PSIZE_64);
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = target_write_buffer(target, address, FLASH_BLOCK_SIZE, buffer);
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = stm32x_wait_flash_op_queue(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			goto flash_lock;

		buffer += FLASH_BLOCK_SIZE;
		address += FLASH_BLOCK_SIZE;
		blocks_remaining--;
	}

	if (bytes_remaining) {
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR), FLASH_PG | FLASH_PSIZE_64);
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = target_write_buffer(target, address, bytes_remaining, buffer);
		if (retval != ERROR_OK)
			goto flash_lock;

		/* Force Write buffer of FLASH_BLOCK_SIZE = 32 bytes */
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR), FLASH_PG | FLASH_PSIZE_64 | FLASH_FW);
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = stm32x_wait_flash_op_queue(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			goto flash_lock;
	}

flash_lock:
	retval2 = stm32x_lock_reg(bank);
	if (retval2 != ERROR_OK)
		LOG_ERROR("error during the lock of flash");

	if (retval == ERROR_OK)
		retval = retval2;

	return retval;
}

static void setup_sector(struct flash_bank *bank, int start, int num, int size)
{
	for (int i = start; i < (start + num) ; i++) {
		assert(i < bank->num_sectors);
		bank->sectors[i].offset = bank->size;
		bank->sectors[i].size = size;
		bank->size += bank->sectors[i].size;
	}
}

static int stm32x_read_id_code(struct flash_bank *bank, uint32_t *id)
{
	/* read stm32 device id register */
	int retval = target_read_u32(bank->target, DBGMCU_IDCODE_REGISTER, id);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static int stm32x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb;
	uint32_t device_id;
	uint32_t base_address = FLASH_BANK0_ADDRESS;
	uint32_t second_bank_base;

	stm32x_info->probed = 0;
	stm32x_info->part_info = NULL;

	int retval = stm32x_read_id_code(bank, &stm32x_info->idcode);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("device id = 0x%08" PRIx32 "", stm32x_info->idcode);

	device_id = stm32x_info->idcode & 0xfff;

	for (unsigned int n = 0; n < ARRAY_SIZE(stm32h7x_parts); n++) {
		if (device_id == stm32h7x_parts[n].id)
			stm32x_info->part_info = &stm32h7x_parts[n];
	}
	if (!stm32x_info->part_info) {
		LOG_WARNING("Cannot identify target as a STM32H7xx family.");
		return ERROR_FAIL;
	} else {
		LOG_INFO("Device: %s", stm32x_info->part_info->device_str);
	}

	/* update the address of controller from data base */
	stm32x_info->flash_base = stm32x_info->part_info->flash_base;

	/* get flash size from target */
	retval = target_read_u16(target, stm32x_info->part_info->fsize_base, &flash_size_in_kb);
	if (retval != ERROR_OK) {
		/* read error when device has invalid value, set max flash size */
		flash_size_in_kb = stm32x_info->part_info->max_flash_size_kb;
	} else
		LOG_INFO("flash size probed value %d", flash_size_in_kb);

	/* Lower flash size devices are single bank */
	if (stm32x_info->part_info->has_dual_bank && (flash_size_in_kb > stm32x_info->part_info->first_bank_size_kb)) {
		/* Use the configured base address to determine if this is the first or second flash bank.
		 * Verify that the base address is reasonably correct and determine the flash bank size
		 */
		second_bank_base = base_address + stm32x_info->part_info->first_bank_size_kb * 1024;
		if (bank->base == second_bank_base) {
			/* This is the second bank  */
			base_address = second_bank_base;
			flash_size_in_kb = flash_size_in_kb - stm32x_info->part_info->first_bank_size_kb;
			/* bank1 also uses a register offset */
			stm32x_info->flash_base = FLASH_REG_BASE_B1;
		} else if (bank->base == base_address) {
			/* This is the first bank */
			flash_size_in_kb = stm32x_info->part_info->first_bank_size_kb;
		} else {
			LOG_WARNING("STM32H flash bank base address config is incorrect. "
				    TARGET_ADDR_FMT " but should rather be 0x%" PRIx32 " or 0x%" PRIx32,
					bank->base, base_address, second_bank_base);
			return ERROR_FAIL;
		}
		LOG_INFO("STM32H flash has dual banks. Bank (%d) size is %dkb, base address is 0x%" PRIx32,
				bank->bank_number, flash_size_in_kb, base_address);
	} else {
		LOG_INFO("STM32H flash size is %dkb, base address is 0x%" PRIx32, flash_size_in_kb, base_address);
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have an invalid flash size register value */
	if (stm32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = stm32x_info->user_bank_size / 1024;
	} else if (flash_size_in_kb == 0xffff) {
		/* die flash size */
		flash_size_in_kb = stm32x_info->part_info->max_flash_size_kb;
	}

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb / stm32x_info->part_info->page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	if (bank->sectors == NULL) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}
	bank->size = 0;

	/* fixed memory */
	setup_sector(bank, 0, num_pages, stm32x_info->part_info->page_size * 1024);

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	stm32x_info->probed = 1;
	return ERROR_OK;
}

static int stm32x_auto_probe(struct flash_bank *bank)
{
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;

	if (stm32x_info->probed)
		return ERROR_OK;

	return stm32x_probe(bank);
}

/* This method must return a string displaying information about the bank */
static int stm32x_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	const struct stm32h7x_part_info *info = stm32x_info->part_info;

	if (!stm32x_info->probed) {
		int retval = stm32x_probe(bank);
		if (retval != ERROR_OK) {
			snprintf(buf, buf_size, "Unable to find bank information.");
			return retval;
		}
	}

	if (info) {
		const char *rev_str = NULL;
		uint16_t rev_id = stm32x_info->idcode >> 16;

		for (unsigned int i = 0; i < info->num_revs; i++)
			if (rev_id == info->revs[i].rev)
				rev_str = info->revs[i].str;

		if (rev_str != NULL) {
			snprintf(buf, buf_size, "%s - Rev: %s",
				stm32x_info->part_info->device_str, rev_str);
		} else {
			snprintf(buf, buf_size,
				 "%s - Rev: unknown (0x%04x)",
				stm32x_info->part_info->device_str, rev_id);
		}
	} else {
	  snprintf(buf, buf_size, "Cannot identify target as a STM32H7x");
	  return ERROR_FAIL;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32h7x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	/* if we have a dual flash bank device then
	 * we need to perform option byte lock on bank0 only */
	if (stm32x_info->flash_base != FLASH_REG_BASE_B0) {
		LOG_ERROR("Option Byte Lock Operation must use bank0");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to read options",
			      bank->driver->name);
		return ERROR_OK;
	}
	/* set readout protection */
	stm32x_info->option_bytes.RDP = 0;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to lock device",
			      bank->driver->name);
		return ERROR_OK;
	}
	command_print(CMD, "%s locked", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
{
	struct target *target = NULL;
	struct stm32h7x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	/* if we have a dual flash bank device then
	 * we need to perform option byte unlock on bank0 only */
	if (stm32x_info->flash_base != FLASH_REG_BASE_B0) {
		LOG_ERROR("Option Byte Unlock Operation must use bank0");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* clear readout protection option byte
	 * this will also force a device unlock if set */
	stm32x_info->option_bytes.RDP = 0xAA;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}
	command_print(CMD, "%s unlocked.\n", bank->driver->name);

	return ERROR_OK;
}

static int stm32x_mass_erase(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory bank */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR), FLASH_BER | FLASH_PSIZE_64);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, FLASH_CR),
							  FLASH_BER | FLASH_PSIZE_64 | FLASH_START);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_flash_op_queue(bank, 30000);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_lock_reg(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("error during the lock of flash");
		return retval;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD, "stm32h7x mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "stm32h7x mass erase complete");
	} else {
		command_print(CMD, "stm32h7x mass erase failed");
	}

	return retval;
}

static const struct command_registration stm32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32h7x",
		.mode = COMMAND_ANY,
		.help = "stm32h7x flash command group",
		.usage = "",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver stm32h7x_flash = {
	.name = "stm32h7x",
	.commands = stm32x_command_handlers,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.read = default_flash_read,
	.probe = stm32x_probe,
	.auto_probe = stm32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32x_protect_check,
	.info = stm32x_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
