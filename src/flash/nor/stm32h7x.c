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

/* FLASH_OPTSR register bits */
#define OPT_BSY        (1 << 0)
#define OPT_RDP_POS    8
#define OPT_RDP_MASK   (0xff << OPT_RDP_POS)
#define OPT_OPTCHANGEERR (1 << 30)

/* FLASH_OPTCCR register bits */
#define OPT_CLR_OPTCHANGEERR (1 << 30)

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

struct stm32h7x_rev {
	uint16_t rev;
	const char *str;
};

/* stm32h7x_part_info permits the store each device information and specificities.
 * the default unit is byte unless the suffix '_kb' is used. */

struct stm32h7x_part_info {
	uint16_t id;
	const char *device_str;
	const struct stm32h7x_rev *revs;
	size_t num_revs;
	unsigned int page_size_kb;
	unsigned int block_size;    /* flash write word size in bytes */
	uint16_t max_flash_size_kb;
	bool has_dual_bank;
	uint16_t max_bank_size_kb;  /* Used when has_dual_bank is true */
	uint32_t flash_regs_base;   /* Flash controller registers location */
	uint32_t fsize_addr;        /* Location of FSIZE register */
	uint32_t wps_group_size;    /* write protection group sectors' count */
	uint32_t wps_mask;
	/* function to compute flash_cr register values */
	uint32_t (*compute_flash_cr)(uint32_t cmd, int snb);
};

struct stm32h7x_flash_bank {
	bool probed;
	uint32_t idcode;
	uint32_t user_bank_size;
	uint32_t flash_regs_base;    /* Address of flash reg controller */
	const struct stm32h7x_part_info *part_info;
};

enum stm32h7x_opt_rdp {
	OPT_RDP_L0 = 0xaa,
	OPT_RDP_L1 = 0x00,
	OPT_RDP_L2 = 0xcc
};

static const struct stm32h7x_rev stm32_450_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x1003, "Y" }, { 0x2001, "X"  }, { 0x2003, "V"  },
};

static const struct stm32h7x_rev stm32_480_revs[] = {
	{ 0x1000, "A"},
};

static uint32_t stm32x_compute_flash_cr_450(uint32_t cmd, int snb)
{
	return cmd | (snb << 8);
}

static uint32_t stm32x_compute_flash_cr_480(uint32_t cmd, int snb)
{
	/* save FW and START bits, to be right shifted by 2 bits later */
	const uint32_t tmp = cmd & (FLASH_FW | FLASH_START);

	/* mask parallelism (ignored), FW and START bits */
	cmd &= ~(FLASH_PSIZE_64 | FLASH_FW | FLASH_START);

	return cmd | (tmp >> 2) | (snb << 6);
}

static const struct stm32h7x_part_info stm32h7x_parts[] = {
	{
	.id					= 0x450,
	.revs				= stm32_450_revs,
	.num_revs			= ARRAY_SIZE(stm32_450_revs),
	.device_str			= "STM32H74x/75x",
	.page_size_kb		= 128,
	.block_size			= 32,
	.max_flash_size_kb	= 2048,
	.max_bank_size_kb	= 1024,
	.has_dual_bank		= true,
	.flash_regs_base	= FLASH_REG_BASE_B0,
	.fsize_addr			= 0x1FF1E880,
	.wps_group_size		= 1,
	.wps_mask			= 0xFF,
	.compute_flash_cr	= stm32x_compute_flash_cr_450,
	},
	{
	.id					= 0x480,
	.revs				= stm32_480_revs,
	.num_revs			= ARRAY_SIZE(stm32_480_revs),
	.device_str			= "STM32H7Ax/7Bx",
	.page_size_kb		= 8,
	.block_size			= 16,
	.max_flash_size_kb	= 2048,
	.max_bank_size_kb	= 1024,
	.has_dual_bank		= true,
	.flash_regs_base	= FLASH_REG_BASE_B0,
	.fsize_addr			= 0x08FFF80C,
	.wps_group_size		= 4,
	.wps_mask			= 0xFFFFFFFF,
	.compute_flash_cr	= stm32x_compute_flash_cr_480,
	},
};

/* flash bank stm32x <base> <size> 0 0 <target#> */

FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32h7x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32x_info = malloc(sizeof(struct stm32h7x_flash_bank));
	bank->driver_priv = stm32x_info;

	stm32x_info->probed = false;
	stm32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline uint32_t stm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg_offset)
{
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	return reg_offset + stm32x_info->flash_regs_base;
}

static inline int stm32x_read_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t *value)
{
	uint32_t reg_addr = stm32x_get_flash_reg(bank, reg_offset);
	int retval = target_read_u32(bank->target, reg_addr, value);

	if (retval != ERROR_OK)
		LOG_ERROR("error while reading from address 0x%" PRIx32, reg_addr);

	return retval;
}

static inline int stm32x_write_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t value)
{
	uint32_t reg_addr = stm32x_get_flash_reg(bank, reg_offset);
	int retval = target_write_u32(bank->target, reg_addr, value);

	if (retval != ERROR_OK)
		LOG_ERROR("error while writing to address 0x%" PRIx32, reg_addr);

	return retval;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	return stm32x_read_flash_reg(bank, FLASH_SR, status);
}

static int stm32x_wait_flash_op_queue(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval;

	/* wait for flash operations completion */
	for (;;) {
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & FLASH_QW) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_ERROR("wait_flash_op_queue, time out expired, status: 0x%" PRIx32 "", status);
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPERR) {
		LOG_ERROR("wait_flash_op_queue, WRPERR detected");
		retval = ERROR_FAIL;
	}

	/* Clear error + EOP flags but report errors */
	if (status & FLASH_ERROR) {
		if (retval == ERROR_OK)
			retval = ERROR_FAIL;
		/* If this operation fails, we ignore it and report the original retval */
		stm32x_write_flash_reg(bank, FLASH_CCR, status);
	}
	return retval;
}

static int stm32x_unlock_reg(struct flash_bank *bank)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on FLASH_KEYR will fail
	 */
	int retval = stm32x_read_flash_reg(bank, FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers for bank */
	retval = stm32x_write_flash_reg(bank, FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_write_flash_reg(bank, FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_read_flash_reg(bank, FLASH_CR, &ctrl);
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

	int retval = stm32x_read_flash_reg(bank, FLASH_OPTCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & OPT_LOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = stm32x_write_flash_reg(bank, FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_write_flash_reg(bank, FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_read_flash_reg(bank, FLASH_OPTCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & OPT_LOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_OPTCR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static inline int stm32x_lock_reg(struct flash_bank *bank)
{
	return stm32x_write_flash_reg(bank, FLASH_CR, FLASH_LOCK);
}

static inline int stm32x_lock_option_reg(struct flash_bank *bank)
{
	return stm32x_write_flash_reg(bank, FLASH_OPTCR, OPT_LOCK);
}

static int stm32x_write_option(struct flash_bank *bank, uint32_t reg_offset, uint32_t value)
{
	int retval, retval2;

	/* unlock option bytes for modification */
	retval = stm32x_unlock_option_reg(bank);
	if (retval != ERROR_OK)
		goto flash_options_lock;

	/* write option bytes */
	retval = stm32x_write_flash_reg(bank, reg_offset, value);
	if (retval != ERROR_OK)
		goto flash_options_lock;

	/* Remove OPT error flag before programming */
	retval = stm32x_write_flash_reg(bank, FLASH_OPTCCR, OPT_CLR_OPTCHANGEERR);
	if (retval != ERROR_OK)
		goto flash_options_lock;

	/* start programming cycle */
	retval = stm32x_write_flash_reg(bank, FLASH_OPTCR, OPT_START);
	if (retval != ERROR_OK)
		goto flash_options_lock;

	/* wait for completion */
	int timeout = FLASH_ERASE_TIMEOUT;
	uint32_t status;
	for (;;) {
		retval = stm32x_read_flash_reg(bank, FLASH_OPTSR_CUR, &status);
		if (retval != ERROR_OK) {
			LOG_ERROR("stm32x_options_program: failed to read FLASH_OPTSR_CUR");
			goto flash_options_lock;
		}
		if ((status & OPT_BSY) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_ERROR("waiting for OBL launch, time out expired, OPTSR: 0x%" PRIx32 "", status);
			retval = ERROR_FAIL;
			goto flash_options_lock;
		}
		alive_sleep(1);
	}

	/* check for failure */
	if (status & OPT_OPTCHANGEERR) {
		LOG_ERROR("error changing option bytes (OPTCHANGEERR=1)");
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

flash_options_lock:
	retval2 = stm32x_lock_option_reg(bank);
	if (retval2 != ERROR_OK)
		LOG_ERROR("error during the lock of flash options");

	return (retval == ERROR_OK) ? retval2 : retval;
}

static int stm32x_modify_option(struct flash_bank *bank, uint32_t reg_offset, uint32_t value, uint32_t mask)
{
	uint32_t data;

	int retval = stm32x_read_flash_reg(bank, reg_offset, &data);
	if (retval != ERROR_OK)
		return retval;

	data = (data & ~mask) | (value & mask);

	return stm32x_write_option(bank, reg_offset, data);
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	uint32_t protection;

	/* read 'write protection' settings */
	int retval = stm32x_read_flash_reg(bank, FLASH_WPSN_CUR, &protection);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read WPSN_CUR register");
		return retval;
	}

	for (int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = protection & (1 << i) ? 0 : 1;

	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	int retval, retval2;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto flash_lock;

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
		retval = stm32x_write_flash_reg(bank, FLASH_CR,
				stm32x_info->part_info->compute_flash_cr(FLASH_SER | FLASH_PSIZE_64, i));
		if (retval != ERROR_OK) {
			LOG_ERROR("Error erase sector %d", i);
			goto flash_lock;
		}
		retval = stm32x_write_flash_reg(bank, FLASH_CR,
				stm32x_info->part_info->compute_flash_cr(FLASH_SER | FLASH_PSIZE_64 | FLASH_START, i));
		if (retval != ERROR_OK) {
			LOG_ERROR("Error erase sector %d", i);
			goto flash_lock;
		}
		retval = stm32x_wait_flash_op_queue(bank, FLASH_ERASE_TIMEOUT);

		if (retval != ERROR_OK) {
			LOG_ERROR("erase time-out or operation error sector %d", i);
			goto flash_lock;
		}
		bank->sectors[i].is_erased = 1;
	}

flash_lock:
	retval2 = stm32x_lock_reg(bank);
	if (retval2 != ERROR_OK)
		LOG_ERROR("error during the lock of flash");

	return (retval == ERROR_OK) ? retval2 : retval;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	uint32_t protection;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read 'write protection' settings */
	int retval = stm32x_read_flash_reg(bank, FLASH_WPSN_CUR, &protection);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read WPSN_CUR register");
		return retval;
	}

	for (int i = first; i <= last; i++) {
		if (set)
			protection &= ~(1 << i);
		else
			protection |= (1 << i);
	}

	/* apply WRPSN mask */
	protection &= 0xff;

	LOG_DEBUG("stm32x_protect, option_bytes written WPSN 0x%" PRIx32, protection);

	/* apply new option value */
	return stm32x_write_option(bank, FLASH_WPSN_PRG, protection);
}

static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	/*
	 * If the size of the data part of the buffer is not a multiple of .block_size, we get
	 * "corrupted fifo read" pointer in target_run_flash_async_algorithm()
	 */
	uint32_t data_size = 512 * stm32x_info->part_info->block_size;
	uint32_t buffer_size = 8 + data_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
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

	LOG_DEBUG("target_alloc_working_area_try : buffer_size -> 0x%" PRIx32, buffer_size);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count of words (word size = .block_size (bytes) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);		/* word size in bytes */
	init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);		/* flash reg base */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, stm32x_info->part_info->block_size);
	buf_set_u32(reg_params[5].value, 0, 32, stm32x_info->flash_regs_base);

	retval = target_run_flash_async_algorithm(target,
						  buffer,
						  count,
						  stm32x_info->part_info->block_size,
						  0, NULL,
						  ARRAY_SIZE(reg_params), reg_params,
						  source->address, source->size,
						  write_algorithm->address, 0,
						  &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32h7x flash write algorithm");

		uint32_t flash_sr = buf_get_u32(reg_params[0].value, 0, 32);

		if (flash_sr & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if ((flash_sr & FLASH_ERROR) != 0) {
			LOG_ERROR("flash write failed, FLASH_SR = %08" PRIx32, flash_sr);
			/* Clear error + EOP flags but report errors */
			stm32x_write_flash_reg(bank, FLASH_CCR, flash_sr);
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
	destroy_reg_param(&reg_params[5]);
	return retval;
}

static int stm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;
	uint32_t address = bank->base + offset;
	int retval, retval2;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* should be enforced via bank->write_start_alignment */
	assert(!(offset % stm32x_info->part_info->block_size));

	/* should be enforced via bank->write_end_alignment */
	assert(!(count % stm32x_info->part_info->block_size));

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto flash_lock;

	uint32_t blocks_remaining = count / stm32x_info->part_info->block_size;

	/* multiple words (n * .block_size) to be programmed in block */
	if (blocks_remaining) {
		retval = stm32x_write_block(bank, buffer, offset, blocks_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
		} else {
			buffer += blocks_remaining * stm32x_info->part_info->block_size;
			address += blocks_remaining * stm32x_info->part_info->block_size;
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
		retval = stm32x_write_flash_reg(bank, FLASH_CR,
				stm32x_info->part_info->compute_flash_cr(FLASH_PG | FLASH_PSIZE_64, 0));
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = target_write_buffer(target, address, stm32x_info->part_info->block_size, buffer);
		if (retval != ERROR_OK)
			goto flash_lock;

		retval = stm32x_wait_flash_op_queue(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			goto flash_lock;

		buffer += stm32x_info->part_info->block_size;
		address += stm32x_info->part_info->block_size;
		blocks_remaining--;
	}

flash_lock:
	retval2 = stm32x_lock_reg(bank);
	if (retval2 != ERROR_OK)
		LOG_ERROR("error during the lock of flash");

	return (retval == ERROR_OK) ? retval2 : retval;
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
	uint16_t flash_size_in_kb;
	uint32_t device_id;

	stm32x_info->probed = false;
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
	stm32x_info->flash_regs_base = stm32x_info->part_info->flash_regs_base;

	/* get flash size from target */
	retval = target_read_u16(target, stm32x_info->part_info->fsize_addr, &flash_size_in_kb);
	if (retval != ERROR_OK) {
		/* read error when device has invalid value, set max flash size */
		flash_size_in_kb = stm32x_info->part_info->max_flash_size_kb;
	} else
		LOG_INFO("flash size probed value %d", flash_size_in_kb);




	/* setup bank size */
	const uint32_t bank1_base = FLASH_BANK0_ADDRESS;
	const uint32_t bank2_base = bank1_base + stm32x_info->part_info->max_bank_size_kb * 1024;
	bool has_dual_bank = stm32x_info->part_info->has_dual_bank;

	switch (device_id) {
	case 0x450:
	case 0x480:
		/* For STM32H74x/75x and STM32H7Ax/Bx
		 *  - STM32H7xxxI devices contains dual bank, 1 Mbyte each
		 *  - STM32H7xxxG devices contains dual bank, 512 Kbyte each
		 *  - STM32H7xxxB devices contains single bank, 128 Kbyte
		 *  - the second bank starts always from 0x08100000
		 */
		if (flash_size_in_kb == 128)
			has_dual_bank = false;
		else
			/* flash size is 2M or 1M */
			flash_size_in_kb /= 2;
		break;
	default:
		LOG_ERROR("unsupported device");
		return ERROR_FAIL;
	}

	if (has_dual_bank) {
		LOG_INFO("STM32H7 flash has dual banks");
		if (bank->base != bank1_base && bank->base != bank2_base) {
			LOG_ERROR("STM32H7 flash bank base address config is incorrect. "
					TARGET_ADDR_FMT " but should rather be 0x%" PRIx32 " or 0x%" PRIx32,
					bank->base, bank1_base, bank2_base);
			return ERROR_FAIL;
		}
	} else {
		LOG_INFO("STM32H7 flash has a single bank");
		if (bank->base == bank2_base) {
			LOG_ERROR("this device has a single bank only");
			return ERROR_FAIL;
		} else if (bank->base != bank1_base) {
			LOG_ERROR("STM32H7 flash bank base address config is incorrect. "
					TARGET_ADDR_FMT " but should be 0x%" PRIx32,
					bank->base, bank1_base);
			return ERROR_FAIL;
		}
	}

	LOG_INFO("Bank (%d) size is %d kb, base address is 0x%" PRIx32,
		bank->bank_number, flash_size_in_kb, (uint32_t) bank->base);

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
	bank->size = flash_size_in_kb * 1024;
	bank->write_start_alignment = stm32x_info->part_info->block_size;
	bank->write_end_alignment = stm32x_info->part_info->block_size;

	/* setup sectors */
	bank->num_sectors = flash_size_in_kb / stm32x_info->part_info->page_size_kb;
	assert(bank->num_sectors > 0);

	if (bank->sectors)
		free(bank->sectors);

	bank->sectors = alloc_block_array(0, stm32x_info->part_info->page_size_kb * 1024,
			bank->num_sectors);

	if (bank->sectors == NULL) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	/* setup protection blocks */
	const uint32_t wpsn = stm32x_info->part_info->wps_group_size;
	assert(bank->num_sectors % wpsn == 0);

	bank->num_prot_blocks = bank->num_sectors / wpsn;
	assert(bank->num_prot_blocks > 0);

	if (bank->prot_blocks)
		free(bank->prot_blocks);

	bank->prot_blocks = alloc_block_array(0, stm32x_info->part_info->page_size_kb * wpsn * 1024,
			bank->num_prot_blocks);

	if (bank->prot_blocks == NULL) {
		LOG_ERROR("failed to allocate bank prot_block");
		return ERROR_FAIL;
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

static int stm32x_set_rdp(struct flash_bank *bank, enum stm32h7x_opt_rdp new_rdp)
{
	struct target *target = bank->target;
	uint32_t optsr, cur_rdp;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_read_flash_reg(bank, FLASH_OPTSR_PRG, &optsr);

	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read FLASH_OPTSR_PRG register");
		return retval;
	}

	/* get current RDP, and check if there is a change */
	cur_rdp = (optsr & OPT_RDP_MASK) >> OPT_RDP_POS;
	if (new_rdp == cur_rdp) {
		LOG_INFO("the requested RDP value is already programmed");
		return ERROR_OK;
	}

	switch (new_rdp) {
	case OPT_RDP_L0:
		LOG_WARNING("unlocking the entire flash device");
		break;
	case OPT_RDP_L1:
		LOG_WARNING("locking the entire flash device");
		break;
	case OPT_RDP_L2:
		LOG_WARNING("locking the entire flash device, irreversible");
		break;
	}

	/* apply new RDP */
	optsr = (optsr & ~OPT_RDP_MASK) | (new_rdp << OPT_RDP_POS);

	/* apply new option value */
	return stm32x_write_option(bank, FLASH_OPTSR_PRG, optsr);
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_set_rdp(bank, OPT_RDP_L1);

	if (retval != ERROR_OK)
		command_print(CMD, "%s failed to lock device", bank->driver->name);
	else
		command_print(CMD, "%s locked", bank->driver->name);

	return retval;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_set_rdp(bank, OPT_RDP_L0);

	if (retval != ERROR_OK)
		command_print(CMD, "%s failed to unlock device", bank->driver->name);
	else
		command_print(CMD, "%s unlocked", bank->driver->name);

	return retval;
}

static int stm32x_mass_erase(struct flash_bank *bank)
{
	int retval, retval2;
	struct target *target = bank->target;
	struct stm32h7x_flash_bank *stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* mass erase flash memory bank */
	retval = stm32x_write_flash_reg(bank, FLASH_CR,
			stm32x_info->part_info->compute_flash_cr(FLASH_BER | FLASH_PSIZE_64, 0));
	if (retval != ERROR_OK)
		goto flash_lock;

	retval = stm32x_write_flash_reg(bank, FLASH_CR,
			stm32x_info->part_info->compute_flash_cr(FLASH_BER | FLASH_PSIZE_64 | FLASH_START, 0));
	if (retval != ERROR_OK)
		goto flash_lock;

	retval = stm32x_wait_flash_op_queue(bank, 30000);
	if (retval != ERROR_OK)
		goto flash_lock;

flash_lock:
	retval2 = stm32x_lock_reg(bank);
	if (retval2 != ERROR_OK)
		LOG_ERROR("error during the lock of flash");

	return (retval == ERROR_OK) ? retval2 : retval;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
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
		for (int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "stm32h7x mass erase complete");
	} else {
		command_print(CMD, "stm32h7x mass erase failed");
	}

	return retval;
}

COMMAND_HANDLER(stm32x_handle_option_read_command)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "stm32h7x option_read <bank> <option_reg offset>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	uint32_t reg_offset, value;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg_offset);
	retval = stm32x_read_flash_reg(bank, reg_offset, &value);
	if (ERROR_OK != retval)
		return retval;

	command_print(CMD, "Option Register: <0x%" PRIx32 "> = 0x%" PRIx32 "",
			stm32x_get_flash_reg(bank, reg_offset), value);

	return retval;
}

COMMAND_HANDLER(stm32x_handle_option_write_command)
{
	if (CMD_ARGC < 3) {
		command_print(CMD, "stm32h7x option_write <bank> <option_reg offset> <value> [mask]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	uint32_t reg_offset, value, mask = 0xffffffff;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg_offset);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], value);
	if (CMD_ARGC > 3)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], mask);

	return stm32x_modify_option(bank, reg_offset, value, mask);
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
	{
		.name = "option_read",
		.handler = stm32x_handle_option_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "option_write",
		.handler = stm32x_handle_option_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset value [mask]",
		.help = "Write device option bit fields with provided value.",
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
