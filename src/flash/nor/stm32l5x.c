/***************************************************************************
 *   Copyright (C) 2015 by Uwe Bonnes                                      *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
 *                                                                         *
 *   Copyright (C) 2019 by Michael Jung                                    *
 *   mijung@gmx.net                                                        *
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

/* STM32L562 series for reference.
 *
 * RM0428 (STM32L552xx/STM32L562xx)
 * http://www.st.com/resource/en/reference_manual/dm00346336.pdf
 */

/* Erase time can be as high as 25ms, 10x this and assume it's toast... */

#define FLASH_ERASE_TIMEOUT 250

#define STM32_FLASH_BASE    0x40022000
#define STM32_FLASH_ACR     0x40022000
#define STM32_FLASH_NSKEYR  0x40022008
#define STM32_FLASH_OPTKEYR 0x40022010
#define STM32_FLASH_NSSR    0x40022020
#define STM32_FLASH_NSCR    0x40022028
#define STM32_FLASH_OPTR    0x40022040
#define STM32_FLASH_WRP1AR  0x40022058
#define STM32_FLASH_WRP1BR  0x4002205c
#define STM32_FLASH_WRP2AR  0x40022068
#define STM32_FLASH_WRP2BR  0x4002206c

/* FLASH_NSCR register bits */

#define FLASH_NSPG       (1 << 0)
#define FLASH_NSPER      (1 << 1)
#define FLASH_NSMER1     (1 << 2)
#define FLASH_PAGE_SHIFT       3
#define FLASH_CR_NSBKER  (1 << 11)
#define FLASH_NSMER2     (1 << 15)
#define FLASH_NSSTRT     (1 << 16)
#define FLASH_NSOPTSTRT  (1 << 17)
#define FLASH_NSEOPIE    (1 << 24)
#define FLASH_NSERRIE    (1 << 25)
#define FLASH_OBL_LAUNCH (1 << 27)
#define FLASH_OPTLOCK    (1 << 30)
#define FLASH_NSLOCK     (1 << 31)

/* FLASH_NSSR register bits */

#define FLASH_NSBSY      (1 << 16)
/* Fast programming not used => related errors not used*/
#define FLASH_NSPGSERR   (1 << 7) /* Programming sequence error */
#define FLASH_NSSIZERR   (1 << 6) /* Size error */
#define FLASH_NSPGAERR   (1 << 5) /* Programming alignment error */
#define FLASH_NSWRPERR   (1 << 4) /* Write protection error */
#define FLASH_NSPROGERR  (1 << 3) /* Programming error */
#define FLASH_NSOPERR    (1 << 1) /* Operation error */
#define FLASH_NSEOP      (1 << 0) /* End of operation */

#define FLASH_ERROR (FLASH_NSPGSERR | FLASH_NSSIZERR | FLASH_NSPGAERR | FLASH_NSWRPERR | FLASH_NSOPERR)

/* STM32_FLASH_OBR bit definitions (reading) */

#define OPT_DBANK_C_256K (1 << 21)	/* dual bank for devices with 256 KiB flash */
#define OPT_DBANK_E_512K (1 << 22)	/* dual bank for devices with 512 KiB flash */

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
#define DBGMCU_IDCODE	0xE0044000
#define FLASH_SIZE_REG  0x0BFA05E0

/* Power control macro (PWR) */

#define STM32_PWR_BASE	0x40007000
#define STM32_PWR_CR1	0x40007000
#define STM32_PWR_SR2	0x40007014

/* PWR_CR1 register bits */

#define PWR_CR1_VOS_MASK	0x00000600
#define  PWR_CR1_VOS_RANGE_0	0x00000000
#define  PWR_CR1_VOS_RANGE_1	0x00000200
#define  PWR_CR1_VOS_RANGE_2	0x00000400

#define PWR_SR2_VOSF		0x00000400

/* Reset and Clock Control macro (RCC) */

#define STM32_RCC_BASE		0x40021000
#define STM32_RCC_APB1ENR1	0x40021058

/* RCC_APB1ENR1 register bits */

#define RCC_APB1ENR1_PWREN	0x10000000

struct stm32l5_flash_bank {
	uint16_t bank2_start;
	int probed;
};

/* flash bank stm32l5x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32l5_flash_bank_command)
{
	struct stm32l5_flash_bank *stm32l5_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32l5_info = malloc(sizeof(struct stm32l5_flash_bank));
	if (!stm32l5_info)
		return ERROR_FAIL; /* Checkme: What better error to use?*/
	bank->driver_priv = stm32l5_info;

	stm32l5_info->probed = 0;

	return ERROR_OK;
}

static inline int stm32l5_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	return reg;
}

static inline int stm32l5_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(
		target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSSR), status);
}

static int stm32l5_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32l5_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_NSBSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}


	if (status & FLASH_NSWRPERR) {
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
		target_write_u32(target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSSR),
				status & FLASH_ERROR);
	}
	return retval;
}

static int stm32l5_unlock_reg(struct target *target)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on STM32_FLASH_NSKEYR will fail
	 */
	int retval = target_read_u32(target, STM32_FLASH_NSCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_NSLOCK) == 0)
		return ERROR_OK;

	/* Flash write and erase operations are possible only in medium or
	 * high performance voltage ranges (range 1 or 0, respectively).  Per
	 * default, devices come up in range 2 (low-power range).  Thus, ensure
	 * we are running in an appropriate voltage range.
	 */

	retval = target_read_u32(target, STM32_PWR_CR1, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (((ctrl & PWR_CR1_VOS_MASK) != PWR_CR1_VOS_RANGE_0) &&
	    ((ctrl & PWR_CR1_VOS_MASK) != PWR_CR1_VOS_RANGE_1)) {

		uint32_t enable;
		uint32_t status;
		int i;

		/* The device has to be reconfigured to run in voltage range 1.
		 * To be able to do this, the peripheral clock of the PWR macro
		 * has to be enabled first.
		 */

		retval = target_read_u32(target, STM32_RCC_APB1ENR1, &enable);
		if (retval != ERROR_OK)
			return retval;

		enable |= RCC_APB1ENR1_PWREN;

		retval = target_write_u32(target, STM32_RCC_APB1ENR1, enable);
		if (retval != ERROR_OK)
			return retval;

		/* Now switch to voltage range 1 ... */

		ctrl = (ctrl & ~PWR_CR1_VOS_MASK) | PWR_CR1_VOS_RANGE_1;

		retval = target_write_u32(target, STM32_PWR_CR1, ctrl);
		if (retval != ERROR_OK)
			return retval;

		/* ... and wait for the power controller to complete */

		for (i = 0; i < 100; i++) {
			retval = target_read_u32(target, STM32_PWR_SR2,
					         &status);
			if (retval != ERROR_OK)
				return retval;

			if (!(status & PWR_SR2_VOSF))
				break;
		}

		if (status & PWR_SR2_VOSF) {
			LOG_ERROR("Dynamic regulator failed to enter selected "
				  "voltage range. PWR_SR2: 0x%08" PRIx32,
				  status);
			return ERROR_TARGET_FAILURE;
		}

		retval = target_read_u32(target, STM32_PWR_CR1, &ctrl);
		if (retval != ERROR_OK)
			return retval;

		if ((ctrl & PWR_CR1_VOS_MASK) != PWR_CR1_VOS_RANGE_1) {
			LOG_ERROR("Dynamic regulator still not in voltage "
				  "range 1. PWR_CR1: 0x%08" PRIx32, ctrl);
			return ERROR_TARGET_FAILURE;
		}
	}

	/* unlock flash registers */
	retval = target_write_u32(target, STM32_FLASH_NSKEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_NSKEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_NSCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_NSLOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l5_unlock_option_reg(struct target *target)
{
	uint32_t ctrl;

	int retval = target_read_u32(target, STM32_FLASH_NSCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_OPTLOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, STM32_FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_NSCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_OPTLOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l5_read_option(struct flash_bank *bank, uint32_t address, uint32_t* value)
{
	struct target *target = bank->target;
	return target_read_u32(target, address, value);
}

static int stm32l5_write_option(struct flash_bank *bank, uint32_t address, uint32_t value, uint32_t mask)
{
	struct target *target = bank->target;
	uint32_t optiondata;

	int retval = target_read_u32(target, address, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l5_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l5_unlock_option_reg(target);
	if (retval != ERROR_OK)
		return retval;

	optiondata = (optiondata & ~mask) | (value & mask);

	retval = target_write_u32(target, address, optiondata);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSCR), FLASH_NSOPTSTRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l5_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32l5_protect_check(struct flash_bank *bank)
{
	struct stm32l5_flash_bank *stm32l5_info = bank->driver_priv;
	uint32_t wrp1ar, wrp1br, wrp2ar, wrp2br;
	stm32l5_read_option(bank, STM32_FLASH_WRP1AR, &wrp1ar);
	stm32l5_read_option(bank, STM32_FLASH_WRP1BR, &wrp1br);
	stm32l5_read_option(bank, STM32_FLASH_WRP2AR, &wrp2ar);
	stm32l5_read_option(bank, STM32_FLASH_WRP2BR, &wrp2br);

	const uint8_t wrp1a_start = wrp1ar & 0xFF;
	const uint8_t wrp1a_end = (wrp1ar >> 16) & 0xFF;
	const uint8_t wrp1b_start = wrp1br & 0xFF;
	const uint8_t wrp1b_end = (wrp1br >> 16) & 0xFF;
	const uint8_t wrp2a_start = wrp2ar & 0xFF;
	const uint8_t wrp2a_end = (wrp2ar >> 16) & 0xFF;
	const uint8_t wrp2b_start = wrp2br & 0xFF;
	const uint8_t wrp2b_end = (wrp2br >> 16) & 0xFF;

	for (int i = 0; i < bank->num_sectors; i++) {
		if (i < stm32l5_info->bank2_start) {
			if (((i >= wrp1a_start) &&
				 (i <= wrp1a_end)) ||
				((i >= wrp1b_start) &&
				 (i <= wrp1b_end)))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		} else {
			uint8_t snb;
			snb = i - stm32l5_info->bank2_start;
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

static int stm32l5_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = stm32l5_unlock_reg(target);
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
	struct stm32l5_flash_bank *stm32l5_info = bank->driver_priv;

	for (i = first; i <= last; i++) {
		uint32_t erase_flags;
		erase_flags = FLASH_NSPER | FLASH_NSSTRT;

		if (i >= stm32l5_info->bank2_start) {
			uint8_t snb;
			snb = i - stm32l5_info->bank2_start;
			erase_flags |= snb << FLASH_PAGE_SHIFT | FLASH_CR_NSBKER;
		} else
			erase_flags |= i << FLASH_PAGE_SHIFT;
		retval = target_write_u32(target,
				stm32l5_get_flash_reg(bank, STM32_FLASH_NSCR), erase_flags);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32l5_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(
		target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSCR), FLASH_NSLOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32l5_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32l5_flash_bank *stm32l5_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = ERROR_OK;
	/* Bank 2 */
	uint32_t reg_value = 0xFF; /* Default to bank un-protected */
	if (last >= stm32l5_info->bank2_start) {
		if (set == 1) {
			uint8_t begin = first > stm32l5_info->bank2_start ? first : 0x00;
			reg_value = ((last & 0xFF) << 16) | begin;
		}

		ret = stm32l5_write_option(bank, STM32_FLASH_WRP2AR, reg_value, 0xffffffff);
	}
	/* Bank 1 */
	reg_value = 0xFF; /* Default to bank un-protected */
	if (first < stm32l5_info->bank2_start) {
		if (set == 1) {
			uint8_t end = last >= stm32l5_info->bank2_start ? 0xFF : last;
			reg_value = (end << 16) | (first & 0xFF);
		}

		ret = stm32l5_write_option(bank, STM32_FLASH_WRP1AR, reg_value, 0xffffffff);
	}

	return ret;
}

/* Count is in halfwords */
static int stm32l5_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t stm32l5_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32l5x.inc"
	};

	if (target_alloc_working_area(target, sizeof(stm32l5_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32l5_flash_write_code),
			stm32l5_flash_write_code);
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
	buf_set_u32(reg_params[4].value, 0, 32, STM32_FLASH_BASE);

	retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32l5 flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR;

		if (error & FLASH_NSWRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_NSSR, error);
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

static int stm32l5_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x7) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 8-byte alignment",
					offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x7) {
		LOG_WARNING("Padding %d bytes to keep 8-byte write size",
					count & 7);
		count = (count + 7) & ~7;
		/* This pads the write chunk with random bytes by overrunning the
		 * write buffer. Padding with the erased pattern 0xff is purely
		 * cosmetical, as 8-byte flash words are ECC secured and the first
		 * write will program the ECC bits. A second write would need
		 * to reprogramm these ECC bits.
		 * But this can only be done after erase!
		 */
	}

	retval = stm32l5_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* Only full double words (8-byte) can be programmed*/
	retval = stm32l5_write_block(bank, buffer, offset, count / 2);
	if (retval != ERROR_OK) {
		LOG_WARNING("block write failed");
		return retval;
		}

	LOG_WARNING("block write succeeded");
	return target_write_u32(target, STM32_FLASH_NSCR, FLASH_NSLOCK);
}

static int stm32l5_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l5_flash_bank *stm32l5_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb = 0xffff;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	uint32_t options;
	uint32_t base_address = 0x08000000;

	stm32l5_info->probed = 0;

	/* read stm32 device id register */
	int retval = target_read_u32(target, DBGMCU_IDCODE, &device_id);
	if (retval != ERROR_OK)
		return retval;
	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	/* set max flash size depending on family */
	switch (device_id & 0xfff) {
	case 0x472:
		max_flash_size_in_kb = 512;
		break;
	default:
		LOG_WARNING("Cannot identify target as an STM32L5 family device.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = target_read_u16(target, FLASH_SIZE_REG, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	/* did we assign a flash size? */
	assert((flash_size_in_kb != 0xffff) && flash_size_in_kb);

	/* get options for DUAL BANK. */
	retval = target_read_u32(target, STM32_FLASH_OPTR, &options);

	if (retval != ERROR_OK)
		return retval;

	int num_pages = 0;
	int page_size = 0;

	switch (device_id & 0xfff) {
		case 0x472:
			/* L552/L562 have 512 KiB FLASH and dual/single bank mode.
			 * Page size is 2K or 4K.*/
			if (flash_size_in_kb == 512) {
				stm32l5_info->bank2_start = 128;
				if (options & OPT_DBANK_E_512K) {
					page_size = 2048;
					num_pages = 256;
				} else {
					page_size = 4096;
					num_pages = 128;
				}
				break;
			}
			/* Invalid FLASH size for this device. */
			LOG_WARNING("Invalid flash size for STM32L5+ family device.");
			return ERROR_FAIL;
		default:
			/* Invalid FLASH size for this device. */
			LOG_WARNING("Invalid flash size for STM32L5+ family device.");
			return ERROR_FAIL;
	}

	/* Release sector table if allocated. */
	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* Set bank configuration and construct sector table. */
	bank->base = base_address;
	bank->size = num_pages * page_size;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	if (!bank->sectors)
		return ERROR_FAIL; /* Checkme: What better error to use?*/

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32l5_info->probed = 1;

	return ERROR_OK;
}

static int stm32l5_auto_probe(struct flash_bank *bank)
{
	struct stm32l5_flash_bank *stm32l5_info = bank->driver_priv;
	if (stm32l5_info->probed)
		return ERROR_OK;
	return stm32l5_probe(bank);
}

static int get_stm32l5_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	uint32_t dbgmcu_idcode;

	/* read stm32 device id register */
	int retval = target_read_u32(target, DBGMCU_IDCODE, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint8_t rev_id = dbgmcu_idcode >> 28;
	uint8_t rev_minor = 0;
	int i;

	for (i = 16; i < 28; i++) {
		if (dbgmcu_idcode & (1 << i))
			rev_minor++;
		else
			break;
	}

	const char *device_str;

	switch (device_id) {
	case 0x472:
		device_str = "STM32L552/562";
		break;

	default:
		snprintf(buf, buf_size, "Cannot identify target as a STM32L5\n");
		return ERROR_FAIL;
	}

	snprintf(buf, buf_size, "%s - Rev: %1d.%02d",
			 device_str, rev_id, rev_minor);

	return ERROR_OK;
}

static int stm32l5_mass_erase(struct flash_bank *bank, uint32_t action)
{
	int retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32l5_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(
		target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSCR), action);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(
		target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSCR),
		action | FLASH_NSSTRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l5_wait_status_busy(bank,  FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(
		target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSCR), FLASH_NSLOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l5_handle_mass_erase_command)
{
	int i;
	uint32_t action;

	if (CMD_ARGC < 1) {
		command_print(CMD, "stm32l5x mass_erase <STM32L5 bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	action =  FLASH_NSMER1 |  FLASH_NSMER2;
	retval = stm32l5_mass_erase(bank, action);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "stm32l5x mass erase complete");
	} else {
		command_print(CMD, "stm32l5x mass erase failed");
	}

	return retval;
}

COMMAND_HANDLER(stm32l5_handle_option_read_command)
{
	if (CMD_ARGC < 2) {
		command_print(CMD, "stm32l5x option_read <STM32L5 bank> <option_reg offset>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	uint32_t reg_addr = STM32_FLASH_BASE;
	uint32_t value = 0;

	reg_addr += strtoul(CMD_ARGV[1], NULL, 16);

	retval = stm32l5_read_option(bank, reg_addr, &value);
	if (ERROR_OK != retval)
		return retval;

	command_print(CMD, "Option Register: <0x%" PRIx32 "> = 0x%" PRIx32 "", reg_addr, value);

	return retval;
}

COMMAND_HANDLER(stm32l5_handle_option_write_command)
{
	if (CMD_ARGC < 3) {
		command_print(CMD, "stm32l5x option_write <STM32L5 bank> <option_reg offset> <value> [mask]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	uint32_t reg_addr = STM32_FLASH_BASE;
	uint32_t value = 0;
	uint32_t mask = 0xFFFFFFFF;

	reg_addr += strtoul(CMD_ARGV[1], NULL, 16);
	value = strtoul(CMD_ARGV[2], NULL, 16);
	if (CMD_ARGC > 3)
		mask = strtoul(CMD_ARGV[3], NULL, 16);

	command_print(CMD, "%s Option written.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.", bank->driver->name);

	retval = stm32l5_write_option(bank, reg_addr, value, mask);
	return retval;
}

COMMAND_HANDLER(stm32l5_handle_option_load_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct target *target = bank->target;

	retval = stm32l5_unlock_reg(target);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32l5_unlock_option_reg(target);
	if (ERROR_OK != retval)
		return retval;

	/* Write the OBLLAUNCH bit in CR -> Cause device "POR" and option bytes reload */
	retval = target_write_u32(target, stm32l5_get_flash_reg(bank, STM32_FLASH_NSCR), FLASH_OBL_LAUNCH);

	command_print(CMD, "stm32l5x option load (POR) completed.");
	return retval;
}

COMMAND_HANDLER(stm32l5_handle_lock_command)
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
	if (stm32l5_write_option(bank, STM32_FLASH_OPTR, 0, 0x000000FF) != ERROR_OK) {
		command_print(CMD, "%s failed to lock device", bank->driver->name);
		return ERROR_OK;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l5_handle_unlock_command)
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

	if (stm32l5_write_option(bank, STM32_FLASH_OPTR, RDP_LEVEL_0, 0x000000FF) != ERROR_OK) {
		command_print(CMD, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}

	return ERROR_OK;
}

static const struct command_registration stm32l5_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32l5_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32l5_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32l5_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "option_read",
		.handler = stm32l5_handle_option_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset",
		.help = "Read & Display device option bytes.",
	},
	{
		.name = "option_write",
		.handler = stm32l5_handle_option_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id reg_offset value mask",
		.help = "Write device option bit fields with provided value.",
	},
	{
		.name = "option_load",
		.handler = stm32l5_handle_option_load_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Force re-load of device options (will cause device reset).",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32l5_command_handlers[] = {
	{
		.name = "stm32l5x",
		.mode = COMMAND_ANY,
		.help = "stm32l5x flash command group",
		.usage = "",
		.chain = stm32l5_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver stm32l5x_flash = {
	.name = "stm32l5x",
	.commands = stm32l5_command_handlers,
	.flash_bank_command = stm32l5_flash_bank_command,
	.erase = stm32l5_erase,
	.protect = stm32l5_protect,
	.write = stm32l5_write,
	.read = default_flash_read,
	.probe = stm32l5_probe,
	.auto_probe = stm32l5_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32l5_protect_check,
	.info = get_stm32l5_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
