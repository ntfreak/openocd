/***************************************************************************
 *   Copyright (C) 2015 by Uwe Bonnes                                      *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
 *                                                                         *
 *   Copyright (C) 2019 by Tarek Bochkati for STMicroelectronics           *
 *   tarek.bouchkati@gmail.com                                             *
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
#include "bits.h"

/* STM32L4xxx series for reference.
 *
 * RM0351 (STM32L4x5/STM32L4x6)
 * http://www.st.com/resource/en/reference_manual/dm00083560.pdf
 *
 * RM0394 (STM32L43x/44x/45x/46x)
 * http://www.st.com/resource/en/reference_manual/dm00151940.pdf
 *
 * RM0432 (STM32L4R/4Sxx)
 * http://www.st.com/resource/en/reference_manual/dm00310109.pdf
 *
 * STM32L476RG Datasheet (for erase timing)
 * http://www.st.com/resource/en/datasheet/stm32l476rg.pdf
 *
 * The RM0351 devices have normally two banks, but on 512 and 256 kiB devices
 * an option byte is available to map all sectors to the first bank.
 * Both STM32 banks are treated as one OpenOCD bank, as other STM32 devices
 * handlers do!
 *
 * RM0394 devices have a single bank only.
 *
 * RM0432 devices have single and dual bank operating modes.
 *  - for STM32L4R/Sxx the FLASH size is 2Mbyte or 1Mbyte.
 *  - for STM32L4P/Q5x the FLASH size is 1Mbyte or 512Kbyte.
 * Bank page (sector) size is 4Kbyte (dual mode) or 8Kbyte (single mode).
 *
 * Bank mode is controlled by two different bits in option bytes register.
 *  - for STM32L4R/Sxx
 *    In 2M FLASH devices bit 22 (DBANK) controls Dual Bank mode.
 *    In 1M FLASH devices bit 21 (DB1M) controls Dual Bank mode.
 *  - for STM32L4P5/Q5x
 *    In 1M FLASH devices bit 22 (DBANK) controls Dual Bank mode.
 *    In 512K FLASH devices bit 21 (DB512K) controls Dual Bank mode.
 *
 */

/* Erase time can be as high as 25ms, 10x this and assume it's toast... */

#define FLASH_ERASE_TIMEOUT 250

/* Flash registers offsets */
#define STM32_FLASH_ACR     0x00
#define STM32_FLASH_KEYR    0x08
#define STM32_FLASH_OPTKEYR 0x0c
#define STM32_FLASH_SR      0x10
#define STM32_FLASH_CR      0x14
#define STM32_FLASH_OPTR    0x20
#define STM32_FLASH_WRP1AR  0x2c
#define STM32_FLASH_WRP1BR  0x30
#define STM32_FLASH_WRP2AR  0x4c
#define STM32_FLASH_WRP2BR  0x50

/* FLASH_CR register bits */
#define FLASH_PG        (1 << 0)
#define FLASH_PER       (1 << 1)
#define FLASH_MER1      (1 << 2)
#define FLASH_PAGE_SHIFT      3
#define FLASH_CR_BKER   (1 << 11)
#define FLASH_MER2      (1 << 15)
#define FLASH_STRT      (1 << 16)
#define FLASH_OPTSTRT   (1 << 17)
#define FLASH_EOPIE     (1 << 24)
#define FLASH_ERRIE     (1 << 25)
#define FLASH_OBLLAUNCH (1 << 27)
#define FLASH_OPTLOCK   (1 << 30)
#define FLASH_LOCK      (1 << 31)

/* FLASH_SR register bits */
#define FLASH_BSY      (1 << 16)
/* Fast programming not used => related errors not used*/
#define FLASH_PGSERR   (1 << 7) /* Programming sequence error */
#define FLASH_SIZERR   (1 << 6) /* Size error */
#define FLASH_PGAERR   (1 << 5) /* Programming alignment error */
#define FLASH_WRPERR   (1 << 4) /* Write protection error */
#define FLASH_PROGERR  (1 << 3) /* Programming error */
#define FLASH_OPERR    (1 << 1) /* Operation error */
#define FLASH_EOP      (1 << 0) /* End of operation */
#define FLASH_ERROR (FLASH_PGSERR | FLASH_PGSERR | FLASH_PGAERR | FLASH_WRPERR | FLASH_OPERR)

/* register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

#define RDP_LEVEL_0	   0xAA
#define RDP_LEVEL_1	   0xBB
#define RDP_LEVEL_2	   0xCC


/* other registers */
#define DBGMCU_IDCODE	0xE0042000


struct stm32l4_rev {
	const uint16_t rev;
	const char *str;
};

struct stm32l4_part_info {
	uint16_t id;
	const char *device_str;
	const struct stm32l4_rev *revs;
	const size_t num_revs;
	const uint16_t max_flash_size_kb;
	const bool has_dual_bank;
	const uint32_t flash_regs_base;
	const uint32_t fsize_addr;
};

struct stm32l4_flash_bank {
	int probed;
	uint32_t idcode;
	int bank1_sectors;
	bool dual_bank_mode;
	int hole_sectors;
	const struct stm32l4_part_info *part_info;
};

static const struct stm32l4_rev stm32_415_revs[] = {
	{ 0x1000, "1" }, { 0x1001, "2" }, { 0x1003, "3" }, { 0x1007, "4" }
};

static const struct stm32l4_rev stm32_435_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x2001, "Y" },
};

static const struct stm32l4_rev stm32_461_revs[] = {
	{ 0x1000, "A" }, { 0x2000, "B" },
};

static const struct stm32l4_rev stm32_462_revs[] = {
		{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x2001, "Y" },
};

static const struct stm32l4_rev stm32_464_revs[] = {
	{ 0x1000, "A" },
};

static const struct stm32l4_rev stm32_470_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x1003, "Y" }, { 0x100F, "W" },
};

static const struct stm32l4_rev stm32_471_revs[] = {
	{ 0x1000, "1" },
};

static const struct stm32l4_rev stm32_495_revs[] = {
	{ 0x2001, "2.1" },
};

static const struct stm32l4_part_info stm32l4_parts[] = {
	{
	  .id                    = 0x415,
	  .revs                  = stm32_415_revs,
	  .num_revs              = ARRAY_SIZE(stm32_415_revs),
	  .device_str            = "STM32L47/L48xx",
	  .max_flash_size_kb     = 1024,
	  .has_dual_bank         = true,
	  .flash_regs_base       = 0x40022000,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x435,
	  .revs                  = stm32_435_revs,
	  .num_revs              = ARRAY_SIZE(stm32_435_revs),
	  .device_str            = "STM32L43/L44xx",
	  .max_flash_size_kb     = 256,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x40022000,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x461,
	  .revs                  = stm32_461_revs,
	  .num_revs              = ARRAY_SIZE(stm32_461_revs),
	  .device_str            = "STM32L49/L4Axx",
	  .max_flash_size_kb     = 1024,
	  .has_dual_bank         = true,
	  .flash_regs_base       = 0x40022000,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x462,
	  .revs                  = stm32_462_revs,
	  .num_revs              = ARRAY_SIZE(stm32_462_revs),
	  .device_str            = "STM32L45/L46xx",
	  .max_flash_size_kb     = 512,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x40022000,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x464,
	  .revs                  = stm32_464_revs,
	  .num_revs              = ARRAY_SIZE(stm32_464_revs),
	  .device_str            = "STM32L41/L42xx",
	  .max_flash_size_kb     = 128,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x40022000,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x470,
	  .revs                  = stm32_470_revs,
	  .num_revs              = ARRAY_SIZE(stm32_470_revs),
	  .device_str            = "STM32L4R/L4Sxx",
	  .max_flash_size_kb     = 2048,
	  .has_dual_bank         = true,
	  .flash_regs_base       = 0x40022000,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x471,
	  .revs                  = stm32_471_revs,
	  .num_revs              = ARRAY_SIZE(stm32_471_revs),
	  .device_str            = "STM32L4P5/L4Q5x",
	  .max_flash_size_kb     = 1024,
	  .has_dual_bank         = true,
	  .flash_regs_base       = 0x40022000,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x495,
	  .revs                  = stm32_495_revs,
	  .num_revs              = ARRAY_SIZE(stm32_495_revs),
	  .device_str            = "STM32WB5x",
	  .max_flash_size_kb     = 1024,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x58004000,
	  .fsize_addr            = 0x1FFF75E0,
	},
};

/* flash bank stm32l4x <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(stm32l4_flash_bank_command)
{
	struct stm32l4_flash_bank *stm32l4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32l4_info = malloc(sizeof(struct stm32l4_flash_bank));
	if (!stm32l4_info)
		return ERROR_FAIL; /* Checkme: What better error to use?*/
	bank->driver_priv = stm32l4_info;

	/* The flash write must be aligned to a double word (8-bytes) boundary.
	 * Ask the flash infrastructure to ensure required alignment */
	bank->write_start_alignment = bank->write_end_alignment = 8;

	stm32l4_info->probed = 0;

	return ERROR_OK;
}

static inline uint32_t stm32l4_get_flash_reg(struct flash_bank *bank, uint32_t reg_offset)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	return stm32l4_info->part_info->flash_regs_base + reg_offset;
}

static inline int stm32l4_read_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t *value)
{
	return target_read_u32(bank->target, stm32l4_get_flash_reg(bank, reg_offset), value);
}

static inline int stm32l4_write_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t value)
{
	return target_write_u32(bank->target, stm32l4_get_flash_reg(bank, reg_offset), value);
}

static int stm32l4_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32l4_read_flash_reg(bank, STM32_FLASH_SR, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}


	if (status & FLASH_WRPERR) {
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR) {
		if (retval == ERROR_OK)
			retval = ERROR_FAIL;
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		stm32l4_write_flash_reg(bank, STM32_FLASH_SR, status & FLASH_ERROR);
	}

	return retval;
}

static int stm32l4_unlock_reg(struct flash_bank *bank)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on STM32_FLASH_KEYR will fail
	 */
	int retval = stm32l4_read_flash_reg(bank, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_read_flash_reg(bank, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_LOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l4_unlock_option_reg(struct flash_bank *bank)
{
	uint32_t ctrl;

	int retval = stm32l4_read_flash_reg(bank, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_OPTLOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_read_flash_reg(bank, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_OPTLOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l4_write_option(struct flash_bank *bank, uint32_t reg_offset, uint32_t value, uint32_t mask)
{
	uint32_t optiondata;

	int retval = stm32l4_read_flash_reg(bank, reg_offset, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_unlock_option_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	optiondata = (optiondata & ~mask) | (value & mask);

	retval = stm32l4_write_flash_reg(bank, reg_offset, optiondata);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_CR, FLASH_OPTSTRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32l4_protect_check(struct flash_bank *bank)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	uint32_t wrp1ar, wrp1br, wrp2ar, wrp2br;
	stm32l4_read_flash_reg(bank, STM32_FLASH_WRP1AR, &wrp1ar);
	stm32l4_read_flash_reg(bank, STM32_FLASH_WRP1BR, &wrp1br);
	stm32l4_read_flash_reg(bank, STM32_FLASH_WRP2AR, &wrp2ar);
	stm32l4_read_flash_reg(bank, STM32_FLASH_WRP2BR, &wrp2br);

	const uint8_t wrp1a_start = wrp1ar & 0xFF;
	const uint8_t wrp1a_end = (wrp1ar >> 16) & 0xFF;
	const uint8_t wrp1b_start = wrp1br & 0xFF;
	const uint8_t wrp1b_end = (wrp1br >> 16) & 0xFF;
	const uint8_t wrp2a_start = wrp2ar & 0xFF;
	const uint8_t wrp2a_end = (wrp2ar >> 16) & 0xFF;
	const uint8_t wrp2b_start = wrp2br & 0xFF;
	const uint8_t wrp2b_end = (wrp2br >> 16) & 0xFF;

	for (int i = 0; i < bank->num_sectors; i++) {
		if (i < stm32l4_info->bank1_sectors) {
			if (((i >= wrp1a_start) &&
				 (i <= wrp1a_end)) ||
				((i >= wrp1b_start) &&
				 (i <= wrp1b_end)))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		} else {
			uint8_t snb;
			snb = i - stm32l4_info->bank1_sectors;
			if (((snb >= wrp2a_start) &&
				 (snb <= wrp2a_end)) ||
				((snb >= wrp2b_start) &&
				 (snb <= wrp2b_end)))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		}
	}
	return ERROR_OK;
}

static int stm32l4_erase(struct flash_bank *bank, int first, int last)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	int i;
	int retval;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32l4_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by
       checking the BSY bit in the FLASH_SR register
	2. Set the PER bit and select the page and bank
	   you wish to erase in the FLASH_CR register
	3. Set the STRT bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	 */

	for (i = first; i <= last; i++) {
		uint32_t erase_flags;
		erase_flags = FLASH_PER | FLASH_STRT;

		if (i >= stm32l4_info->bank1_sectors) {
			uint8_t snb;
			snb = i - stm32l4_info->bank1_sectors;
			erase_flags |= snb << FLASH_PAGE_SHIFT | FLASH_CR_BKER;
		} else
			erase_flags |= i << FLASH_PAGE_SHIFT;
		retval = stm32l4_write_flash_reg(bank, STM32_FLASH_CR, erase_flags);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32l4_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = ERROR_OK;
	/* Bank 2 */
	uint32_t reg_value = 0xFF; /* Default to bank un-protected */
	if (last >= stm32l4_info->bank1_sectors) {
		if (set == 1) {
			uint8_t begin = first > stm32l4_info->bank1_sectors ? first : 0x00;
			reg_value = ((last & 0xFF) << 16) | begin;
		}

		ret = stm32l4_write_option(bank, STM32_FLASH_WRP2AR, reg_value, 0xffffffff);
	}
	/* Bank 1 */
	reg_value = 0xFF; /* Default to bank un-protected */
	if (first < stm32l4_info->bank1_sectors) {
		if (set == 1) {
			uint8_t end = last >= stm32l4_info->bank1_sectors ? 0xFF : last;
			reg_value = (end << 16) | (first & 0xFF);
		}

		ret = stm32l4_write_option(bank, STM32_FLASH_WRP1AR, reg_value, 0xffffffff);
	}

	return ret;
}

/* Count is in halfwords */
static int stm32l4_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t stm32l4_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32l4x.inc"
	};

	if (target_alloc_working_area(target, sizeof(stm32l4_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32l4_flash_write_code),
			stm32l4_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) !=
		   ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("large enough working area not available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* count (double word-64bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);	/* flash base */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count / 4);
	buf_set_u32(reg_params[4].value, 0, 32, stm32l4_info->part_info->flash_regs_base);

	retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32l4 flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR;

		if (error & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			stm32l4_write_flash_reg(bank, STM32_FLASH_SR, error);
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

static int stm32l4_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The flash write must be aligned to a double word (8-bytes) boundary.
	 * The flash infrastructure ensures it, do just a security check */
	assert(offset % 8 == 0);
	assert(count % 8 == 0);

	retval = stm32l4_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_block(bank, buffer, offset, count / 2);
	if (retval != ERROR_OK) {
		LOG_WARNING("block write failed");
		return retval;
	}

	LOG_WARNING("block write succeeded");
	return stm32l4_write_flash_reg(bank, STM32_FLASH_CR, FLASH_LOCK);
}

static int stm32l4_read_idcode(struct flash_bank *bank, uint32_t *id)
{
	int retval = target_read_u32(bank->target, DBGMCU_IDCODE, id);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32l4_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	const struct stm32l4_part_info *part_info;
	int i;
	uint16_t flash_size_in_kb = 0xffff;
	uint32_t device_id;
	uint32_t options;

	stm32l4_info->probed = 0;

	/* read stm32 device id register */
	int retval = stm32l4_read_idcode(bank, &stm32l4_info->idcode);
	if (retval != ERROR_OK)
		return retval;

	device_id = stm32l4_info->idcode & 0xFFF;

	for (unsigned int n = 0; n < ARRAY_SIZE(stm32l4_parts); n++) {
		if (device_id == stm32l4_parts[n].id)
			stm32l4_info->part_info = &stm32l4_parts[n];
	}

	if (!stm32l4_info->part_info) {
		LOG_WARNING("Cannot identify target as an STM32 L4 or WB family device.");
		return ERROR_FAIL;
	}

	part_info = stm32l4_info->part_info;

	char device_info[1024];
	retval = bank->driver->info(bank, device_info, sizeof(device_info));
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device idcode = 0x%08" PRIx32 " (%s)", stm32l4_info->idcode, device_info);

	/* get flash size from target. */
	retval = target_read_u16(target, part_info->fsize_addr, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0
			|| flash_size_in_kb > part_info->max_flash_size_kb) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
			part_info->max_flash_size_kb);
		flash_size_in_kb = part_info->max_flash_size_kb;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	/* did we assign a flash size? */
	assert((flash_size_in_kb != 0xffff) && flash_size_in_kb);

	/* read flash option register */
	retval = stm32l4_read_flash_reg(bank, STM32_FLASH_OPTR, &options);
	if (retval != ERROR_OK)
		return retval;

	stm32l4_info->bank1_sectors = 0;
	stm32l4_info->hole_sectors = 0;

	int num_pages = 0;
	int page_size = 0;

	stm32l4_info->dual_bank_mode = false;

	switch (device_id) {
	case 0x415:
	case 0x461:
		/* if flash size is max (1M) the device is always dual bank
		 * 0x415: has variants with 512K
		 * 0x461: has variants with 512 and 256
		 * for these variants:
		 *   if DUAL_BANK = 0 -> single bank
		 *   else -> dual bank without gap
		 * note: the page size is invariant
		 */
		page_size = 2048;
		num_pages = flash_size_in_kb / 2;
		stm32l4_info->bank1_sectors = num_pages;

		/* check DUAL_BANK bit[21] if the flash is less than 1M */
		if (flash_size_in_kb == 1024 || (options & BIT(21))) {
			stm32l4_info->dual_bank_mode = true;
			stm32l4_info->bank1_sectors = num_pages / 2;
		}
		break;
	case 0x435:
	case 0x462:
	case 0x464:
		/* single bank flash */
		page_size = 2048;
		num_pages = flash_size_in_kb / 2;
		stm32l4_info->bank1_sectors = num_pages;
		break;
	case 0x470:
	case 0x471:
		/* STM32L4R/S can be single/dual bank:
		 *   if size = 2M check DBANK bit(22)
		 *   if size = 1M check DB1M bit(21)
		 * STM32L4P/Q can be single/dual bank
		 *   if size = 1M check DBANK bit(22)
		 *   if size = 512K check DB512K bit(21)
		 * in single bank configuration the page size is 8K
		 * else (dual bank) the page size is 4K without gap between banks
		 */
		page_size = 8192;
		num_pages = flash_size_in_kb / 8;
		stm32l4_info->bank1_sectors = num_pages;
		const bool use_dbank_bit = flash_size_in_kb == part_info->max_flash_size_kb;
		if ((use_dbank_bit && (options & BIT(22))) ||
			(!use_dbank_bit && (options & BIT(21)))) {
			stm32l4_info->dual_bank_mode = true;
			page_size = 4096;
			num_pages = flash_size_in_kb / 4;
			stm32l4_info->bank1_sectors = num_pages / 2;
		}
		break;
	case 0x495:
		/* single bank flash */
		page_size = 4096;
		num_pages = flash_size_in_kb / 4;
		stm32l4_info->bank1_sectors = num_pages;
		break;
	default:
		LOG_ERROR("unsupported device");
		return ERROR_FAIL;
	}

	LOG_INFO("flash mode : %s-bank", stm32l4_info->dual_bank_mode ? "dual" : "single");

	const int gap_size = stm32l4_info->hole_sectors * page_size;

	if (stm32l4_info->dual_bank_mode & gap_size) {
		LOG_INFO("gap detected starting from %0x08" PRIx32 " to %0x08" PRIx32,
				0x8000000 + stm32l4_info->bank1_sectors * page_size,
				0x8000000 + stm32l4_info->bank1_sectors * page_size + gap_size);
	}

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->size = flash_size_in_kb * 1024 + gap_size;
	bank->base = 0x08000000;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (bank->sectors == NULL) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	for (i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * page_size;
		/* in dual bank configuration, if there is a gap between banks
		 * we fix up the sector offset to consider this gap */
		if (i >= stm32l4_info->bank1_sectors && stm32l4_info->hole_sectors)
			bank->sectors[i].offset += gap_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32l4_info->probed = 1;
	return ERROR_OK;
}

static int stm32l4_auto_probe(struct flash_bank *bank)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	if (stm32l4_info->probed)
		return ERROR_OK;

	return stm32l4_probe(bank);
}

static int get_stm32l4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	const struct stm32l4_part_info *part_info = stm32l4_info->part_info;

	if (part_info) {
		const char *rev_str = NULL;
		uint16_t rev_id = stm32l4_info->idcode >> 16;
		for (unsigned int i = 0; i < part_info->num_revs; i++) {
			if (rev_id == part_info->revs[i].rev) {
				rev_str = part_info->revs[i].str;

				if (rev_str != NULL) {
					snprintf(buf, buf_size, "%s - Rev: %s",
							part_info->device_str, rev_str);
					return ERROR_OK;
				}
			}
		}

		snprintf(buf, buf_size, "%s - Rev: unknown (0x%04x)",
				part_info->device_str, rev_id);
		return ERROR_OK;
	} else {
		snprintf(buf, buf_size, "Cannot identify target as an STM32 L4 or WB device");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int stm32l4_mass_erase(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	uint32_t action = FLASH_MER1;

	if (stm32l4_info->part_info->has_dual_bank)
		action |= FLASH_MER2;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32l4_unlock_reg(bank);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT / 10);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_CR, action);
	if (retval != ERROR_OK)
		return retval;
	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_CR, action | FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_wait_status_busy(bank,  FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l4_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD, "stm32l4x mass_erase <STM32L4 bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32l4_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "stm32l4x mass erase complete");
	} else {
		command_print(CMD, "stm32l4x mass erase failed");
	}

	return retval;
}

COMMAND_HANDLER(stm32l4_handle_option_read_command)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "stm32l4x option_read <STM32L4 bank> <option_reg offset>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	uint32_t reg_offset, reg_addr;
	uint32_t value = 0;

	reg_offset = strtoul(CMD_ARGV[1], NULL, 16);
	reg_addr = stm32l4_get_flash_reg(bank, reg_offset);

	retval = stm32l4_read_flash_reg(bank, reg_offset, &value);
	if (ERROR_OK != retval)
		return retval;

	command_print(CMD, "Option Register: <0x%" PRIx32 "> = 0x%" PRIx32 "", reg_addr, value);

	return retval;
}

COMMAND_HANDLER(stm32l4_handle_option_write_command)
{
	if (CMD_ARGC < 3) {
		command_print(CMD, "stm32l4x option_write <STM32L4 bank> <option_reg offset> <value> [mask]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	uint32_t reg_offset;
	uint32_t value = 0;
	uint32_t mask = 0xFFFFFFFF;

	reg_offset = strtoul(CMD_ARGV[1], NULL, 16);
	value = strtoul(CMD_ARGV[2], NULL, 16);
	if (CMD_ARGC > 3)
		mask = strtoul(CMD_ARGV[3], NULL, 16);

	command_print(CMD, "%s Option written.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.", bank->driver->name);

	retval = stm32l4_write_option(bank, reg_offset, value, mask);
	return retval;
}

COMMAND_HANDLER(stm32l4_handle_option_load_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32l4_unlock_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32l4_unlock_option_reg(bank);
	if (ERROR_OK != retval)
		return retval;

	/* Write the OBLLAUNCH bit in CR -> Cause device "POR" and option bytes reload */
	retval = stm32l4_write_flash_reg(bank, STM32_FLASH_CR, FLASH_OBLLAUNCH);

	command_print(CMD, "stm32l4x option load (POR) completed.");
	return retval;
}

COMMAND_HANDLER(stm32l4_handle_lock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* set readout protection level 1 by erasing the RDP option byte */
	if (stm32l4_write_option(bank, STM32_FLASH_OPTR, 0, 0x000000FF) != ERROR_OK) {
		command_print(CMD, "%s failed to lock device", bank->driver->name);
		return ERROR_OK;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l4_handle_unlock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32l4_write_option(bank, STM32_FLASH_OPTR, RDP_LEVEL_0, 0x000000FF) != ERROR_OK) {
		command_print(CMD, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}

	return ERROR_OK;
}

static const struct command_registration stm32l4_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32l4_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32l4_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32l4_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "option_read",
		.handler = stm32l4_handle_option_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset",
		.help = "Read & Display device option bytes.",
	},
	{
		.name = "option_write",
		.handler = stm32l4_handle_option_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset value mask",
		.help = "Write device option bit fields with provided value.",
	},
	{
		.name = "option_load",
		.handler = stm32l4_handle_option_load_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Force re-load of device options (will cause device reset).",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32l4_command_handlers[] = {
	{
		.name = "stm32l4x",
		.mode = COMMAND_ANY,
		.help = "stm32l4x flash command group",
		.usage = "",
		.chain = stm32l4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver stm32l4x_flash = {
	.name = "stm32l4x",
	.commands = stm32l4_command_handlers,
	.flash_bank_command = stm32l4_flash_bank_command,
	.erase = stm32l4_erase,
	.protect = stm32l4_protect,
	.write = stm32l4_write,
	.read = default_flash_read,
	.probe = stm32l4_probe,
	.auto_probe = stm32l4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32l4_protect_check,
	.info = get_stm32l4_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
