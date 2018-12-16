/***************************************************************************
 *   Copyright (C) 2015 by Uwe Bonnes                                      *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
 *
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
#include "stm32l4x.h"

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
 * The FLASH size is 1Mbyte or 2Mbyte.
 * Bank page (sector) size is 4Kbyte (dual mode) or 8Kbyte (single mode).
 *
 * Bank mode is controlled by two different bits in option bytes register.
 * In 2M FLASH devices bit 22 (DBANK) controls Dual Bank mode.
 * In 1M FLASH devices bit 21 (DB1M) controls Dual Bank mode.
 *
 */

/*
 * STM32G0xxx series for reference.
 *
 * RM0444 (STM32G0x1)
 * http://www.st.com/resource/en/reference_manual/dm00371828.pdf
 *
 * RM0454 (STM32G0x0)
 * http://www.st.com/resource/en/reference_manual/dm00463896.pdf
 */

/*
 * STM32G4xxx series for reference.
 *
 * RM0440
 * http://www.st.com/resource/en/reference_manual/dm00355726.pdf
 *
 * Cat. 2 devices have single bank only, page size is 2KByte.
 *
 * Cat. 3 devices have single and dual bank operating modes,
 * Page size is 2kByte (dual mode) or 4 kByte (single mode).
 *
 * Bank mode is controlled by bit 22 (DBANK) in option bytes register.
 * Both banks are treated as a single OpenOCD bank.
 */

/* Erase time can be as high as 25ms, 10x this and assume it's toast... */

#define FLASH_ERASE_TIMEOUT 250

struct stm32l4_flash_bank {
	uint16_t bank2_start;
	bool hasWRP2;
	int probed;
};

/* flash bank stm32l4x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32l4_flash_bank_command)
{
	struct stm32l4_flash_bank *stm32l4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32l4_info = malloc(sizeof(struct stm32l4_flash_bank));
	if (!stm32l4_info)
		return ERROR_FAIL; /* Checkme: What better error to use?*/
	bank->driver_priv = stm32l4_info;

	stm32l4_info->probed = 0;

	return ERROR_OK;
}

static inline int stm32l4_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	return reg;
}

static inline int stm32l4_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target,
		stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_SR_OFFS), status);
}

static int stm32l4_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32l4_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & (1UL << FLASH_BSY)) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}


	if (status & (1UL << FLASH_WRPERR)) {
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR_MASK) {
		if (retval == ERROR_OK)
			retval = ERROR_FAIL;
		/* If this operation fails, we ignore it and report the original retval */
		target_write_u32(target, stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_SR_OFFS),
			status & FLASH_ERROR_MASK);
	}
	return retval;
}

static int stm32l4_unlock_reg(struct target *target)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on FLASH_KEYR will fail
	 */
	int retval = target_read_u32(target, STM32_FLASH_BASE + FLASH_CR_OFFS, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & (1UL << FLASH_LOCK)) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = target_write_u32(target, STM32_FLASH_BASE + FLASH_KEYR_OFFS, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_BASE + FLASH_KEYR_OFFS, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_BASE + FLASH_CR_OFFS, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & (1UL << FLASH_LOCK)) {
		LOG_ERROR("flash not unlocked FLASH_CR_OFFS: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l4_unlock_option_reg(struct target *target)
{
	uint32_t ctrl;

	int retval = target_read_u32(target, STM32_FLASH_BASE + FLASH_CR_OFFS, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & (1UL << FLASH_OPTLOCK)) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, STM32_FLASH_BASE + FLASH_OPTKEYR_OFFS, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_BASE + FLASH_OPTKEYR_OFFS, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_BASE + FLASH_CR_OFFS, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & (1UL << FLASH_OPTLOCK)) {
		LOG_ERROR("options not unlocked FLASH_CR_OFFS: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l4_read_option(struct flash_bank *bank, uint32_t address, uint32_t* value)
{
	struct target *target = bank->target;
	return target_read_u32(target, address, value);
}

static int stm32l4_write_option(struct flash_bank *bank, uint32_t address, uint32_t value, uint32_t mask)
{
	struct target *target = bank->target;
	uint32_t optiondata;

	int retval = target_read_u32(target, address, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_unlock_option_reg(target);
	if (retval != ERROR_OK)
		return retval;

	optiondata = (optiondata & ~mask) | (value & mask);

	retval = target_write_u32(target, address, optiondata);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_CR_OFFS),
		(1UL << FLASH_OPTSTRT));
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
	uint32_t wrp1ar = 0, wrp1br = 0, wrp2ar = 0, wrp2br = 0;

	stm32l4_read_option(bank, STM32_FLASH_BASE + FLASH_WRP1AR_OFFS, &wrp1ar);
	stm32l4_read_option(bank, STM32_FLASH_BASE + FLASH_WRP1BR_OFFS, &wrp1br);
	if (stm32l4_info->hasWRP2) {
		stm32l4_read_option(bank, STM32_FLASH_BASE + FLASH_WRP2AR_OFFS, &wrp2ar);
		stm32l4_read_option(bank, STM32_FLASH_BASE + FLASH_WRP2BR_OFFS, &wrp2br);
	}

	const uint8_t wrp1a_start = wrp1ar & 0xFF;
	const uint8_t wrp1a_end = (wrp1ar >> 16) & 0xFF;
	const uint8_t wrp1b_start = wrp1br & 0xFF;
	const uint8_t wrp1b_end = (wrp1br >> 16) & 0xFF;
	const uint8_t wrp2a_start = wrp2ar & 0xFF;
	const uint8_t wrp2a_end = (wrp2ar >> 16) & 0xFF;
	const uint8_t wrp2b_start = wrp2br & 0xFF;
	const uint8_t wrp2b_end = (wrp2br >> 16) & 0xFF;

	for (int i = 0; i < bank->num_sectors; i++) {
		if (i < stm32l4_info->bank2_start) {
			if (((i >= wrp1a_start) &&
				 (i <= wrp1a_end)) ||
				((i >= wrp1b_start) &&
				 (i <= wrp1b_end)))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		} else {
			uint8_t snb;
			snb = i - stm32l4_info->bank2_start;
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
	struct target *target = bank->target;
	int i;

	assert(first < bank->num_sectors);
	assert(last < bank->num_sectors);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = stm32l4_unlock_reg(target);
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
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	for (i = first; i <= last; i++) {
		uint32_t erase_flags;
		erase_flags = (1UL << FLASH_PER) | (1UL << FLASH_STRT);

		if (i >= stm32l4_info->bank2_start) {
			uint8_t snb;
			snb = i - stm32l4_info->bank2_start;
			erase_flags |= (snb << FLASH_PAGE_SHIFT) | (1UL << FLASH_CR_BKER);
		} else
			erase_flags |= i << FLASH_PAGE_SHIFT;
		retval = target_write_u32(target,
				stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_CR_OFFS), erase_flags);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target,
		stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_CR_OFFS), (1UL << FLASH_LOCK));
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
	if (stm32l4_info->hasWRP2 && (last >= stm32l4_info->bank2_start)) {
		if (set == 1) {
			uint8_t begin = first > stm32l4_info->bank2_start ? first : 0x00;
			reg_value = ((last & 0xFF) << 16) | begin;
		}

		ret = stm32l4_write_option(bank, STM32_FLASH_BASE + FLASH_WRP2AR_OFFS,
			reg_value, 0xffffffff);
	}
	/* Bank 1 */
	reg_value = 0xFF; /* Default to bank un-protected */
	if (first < stm32l4_info->bank2_start) {
		if (set == 1) {
			uint8_t end = last >= stm32l4_info->bank2_start ? 0xFF : last;
			reg_value = (end << 16) | (first & 0xFF);
		}

		ret = stm32l4_write_option(bank, STM32_FLASH_BASE + FLASH_WRP1AR_OFFS,
			reg_value, 0xffffffff);
	}

	return ret;
}

/* Count is in halfwords */
static int stm32l4_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size;
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

	/* memory buffer, size *must* be multiple of dword plus one dword for rp and wp */
	buffer_size = target_get_working_area_avail(target) & ~(2 * sizeof(uint32_t) - 1);
	if (buffer_size < 256) {
		LOG_WARNING("large enough working area not available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (buffer_size > 16384) {
		/* probably won't benefit from more than 16k ... */
		buffer_size = 16384;
	}

	if (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* count (bytes) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);	/* flash base */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, STM32_FLASH_BASE);

	retval = target_run_flash_async_algorithm(target, buffer, count, 1,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32l4 flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR_MASK;

		if (error & (1UL << FLASH_WRPERR))
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_BASE + FLASH_SR_OFFS, error);
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
		/* Only full double words (8-byte) can be programmed */
		/* Padding by 0xFF bytes is done in flash loader */
		LOG_WARNING("Padding %d bytes to keep 8-byte write size", 8 - (count & 7));
	}

	retval = stm32l4_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_block(bank, buffer, offset, count);
	if (retval != ERROR_OK) {
		LOG_WARNING("block write failed");
		return retval;
		}

	LOG_WARNING("block write succeeded");
	return target_write_u32(target, STM32_FLASH_BASE + FLASH_CR_OFFS, (1UL << FLASH_LOCK));
}

static int stm32l4_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	int i, retval;
	uint16_t flash_size_in_kb = 0xffff;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	uint32_t options;
	uint32_t base_address = 0x08000000;

	stm32l4_info->hasWRP2 = false;	/* second bank: WRP2A/B and MER2 */
	stm32l4_info->probed = 0;

	/* try stm32l4/g4 id register first, then stm32g0 id register */
	retval = target_read_u32(target, DBGMCU_IDCODE_L4_G4, &device_id);
	if ((retval != ERROR_OK) || ((device_id & 0xfff) == 0) || ((device_id & 0xfff) == 0xfff)) {
		retval = target_read_u32(target, DBGMCU_IDCODE_G0, &device_id);
		if ((retval != ERROR_OK) || ((device_id & 0xfff) == 0) || ((device_id & 0xfff) == 0xfff)) {
			LOG_ERROR("can't get device id");
			return retval;
		}
	}
	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	/* set max flash size depending on family */
	switch (device_id & 0xfff) {
	case 0x415: /* STM32L475/L476/L486 */
	case 0x461: /* STM32L496/L4A6 */
		max_flash_size_in_kb = 1024;
		stm32l4_info->hasWRP2 = true;
		break;
	case 0x435: /* STM32L43x/L44x */
		max_flash_size_in_kb = 256;
		break;
	case 0x462: /* STM32L45x/L46x */
	case 0x464: /* STM32L41x/L42x */
		max_flash_size_in_kb = 512;
		break;
	case 0x460: /* STM32G07x/G08x */
		max_flash_size_in_kb = 512;
		break;
	case 0x466: /* STM32G03x/G04x */
		max_flash_size_in_kb = 512;
		break;
	case 0x468: /* STM32G43x/G44x Cat. 2 */
		max_flash_size_in_kb = 128;
		break;
	case 0x469: /* STM32G47x/G48x Cat. 3 */
		max_flash_size_in_kb = 512;
		stm32l4_info->hasWRP2 = true;
		break;
	case 0x470: /* STM32L4Rx/L4Sx */
		max_flash_size_in_kb = 2048;
		stm32l4_info->hasWRP2 = true;
		break;
	default:
		LOG_WARNING("Cannot identify target as an STM32L4/G4/G0 family device.");
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
	retval = target_read_u32(target, STM32_FLASH_BASE + FLASH_OPTR_OFFS, &options);
	if (retval != ERROR_OK)
		return retval;

	int num_pages = 0;
	int page_size = 0;

	switch (device_id & 0xfff) {
		case 0x470:
			/* L4R/S have 1M or 2M FLASH and dual/single bank mode.
			 * Page size is 4K (dual bank) or 8K (single bank mode).*/
			if (flash_size_in_kb == 2048) {
				stm32l4_info->bank2_start = 256;
				if (options & (1UL << OPT_DBANK_GE_2M)) {
					page_size = 4096;
					num_pages = 512;
					LOG_INFO("Dual Bank %d kiB STM32L4Rx/L4Sx found", flash_size_in_kb);
				} else {
					page_size = 8192;
					num_pages = 256;
					LOG_INFO("Single Bank %d kiB STM32L4Rx/L4Sx found", flash_size_in_kb);
				}
				break;
			}
			if (flash_size_in_kb == 1024) {
				stm32l4_info->bank2_start = 128;
				if (options & (1UL << OPT_DBANK_LE_1M)) {
					page_size = 4096;
					num_pages = 256;
					LOG_INFO("Dual Bank %d kiB STM32L4Rx/L4Sx found", flash_size_in_kb);
				} else {
					page_size = 8192;
					num_pages = 128;
					LOG_INFO("Single Bank %d kiB STM32L4Rx/L4Sx found", flash_size_in_kb);
				}
				break;
			}
			/* Invalid FLASH size for this device. */
			LOG_WARNING("Invalid flash size for STM32L4Rx/L4Sx device.");
			return ERROR_FAIL;
		case 0x461:
		case 0x415:
			/* These are dual-bank devices, we need to check the OPT_DBANK_LE_1M bit here */
			page_size = 2048;
			num_pages = flash_size_in_kb / 2;
			/* check that calculation result makes sense */
			assert(num_pages > 0);
			if ((flash_size_in_kb == 1024) || !(options & (1UL << OPT_DBANK_LE_1M))) {
				stm32l4_info->bank2_start = 256;
				LOG_INFO("Dual Bank %d kiB STM32L475/L476/L486/L496/L4A6 found", flash_size_in_kb);
			} else {
				stm32l4_info->bank2_start = num_pages / 2;
				LOG_INFO("Single Bank %d kiB STM32L475/L476/L486/L496/L4A6 found", flash_size_in_kb);
			}
			break;
		case 0x460:
		case 0x466:
			/* G0x0/G0x1 have single bank only, page size always 2K. */
			page_size = 2048;
			num_pages = flash_size_in_kb / 2;
			/* check that calculation result makes sense */
			assert(num_pages > 0);
			stm32l4_info->bank2_start = UINT16_MAX;
			LOG_INFO("Single Bank %d kiB STM32G03x/G04x/G07x/G08x found", flash_size_in_kb);
			break;
		case 0x468:
			/* G43x/G44x Cat. 2 have single bank only, page size always 2K. */
			page_size = 2048;
			num_pages = flash_size_in_kb / 2;
			/* check that calculation result makes sense */
			assert(num_pages > 0);
			stm32l4_info->bank2_start = UINT16_MAX;
			LOG_INFO("Single Bank %d kiB STM32G43x/G44x Cat. 2 found", flash_size_in_kb);
			break;
		case 0x469:
			/* G47x/G48x Cat. 3 have dual/single bank depending on DBANK.
			 * Page size 2K in dual bank and 4K in single bank mode. */
			if (options & (1UL << OPT_DBANK_GE_2M)) {
				page_size = 2048;
				num_pages = flash_size_in_kb / 2;
				stm32l4_info->bank2_start = num_pages / 2;
				LOG_INFO("Dual Bank %d kiB STM32G43x/G44x Cat. 3 found", flash_size_in_kb);
			} else {
				page_size = 4096;
				num_pages = flash_size_in_kb / 4;
				stm32l4_info->bank2_start = UINT16_MAX;
				LOG_INFO("Single Bank %d kiB STM32G47x/G48x Cat. 3 found", flash_size_in_kb);
			}
			/* check that calculation result makes sense */
			assert(num_pages > 0);
			break;
		case 0x435:
		case 0x462:
		case 0x463:
			/* These are single-bank devices, all have 2K pages. */
			page_size = 2048;
			num_pages = flash_size_in_kb / 2;
			/* check that calculation result makes sense */
			assert(num_pages > 0);
			stm32l4_info->bank2_start = UINT16_MAX;
			LOG_INFO("Single Bank %d kiB STM32L41x/L42x/L43x/L44x/L45x/L46x found", flash_size_in_kb);
			break;
		default:
			/* can't happen */
			assert(0);
			break;
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
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	uint32_t device_id;
	int retval;

	/* try stm32l4 id register first, then stm32g0 id register */
	retval = target_read_u32(target, DBGMCU_IDCODE_L4_G4, &device_id);
	if ((retval != ERROR_OK) || ((device_id & 0xfff) == 0) || ((device_id & 0xfff) == 0xfff)) {
		retval = target_read_u32(target, DBGMCU_IDCODE_G0, &device_id);
		if ((retval != ERROR_OK) || ((device_id & 0xfff) == 0) || ((device_id & 0xfff) == 0xfff)) {
			LOG_ERROR("can't get device id");
			return retval;
		}
	}

	uint8_t rev_id = device_id >> 28;
	uint8_t rev_minor = 0;
	int i;

	for (i = 16; i < 28; i++) {
		if (device_id & (1 << i))
			rev_minor++;
		else
			break;
	}

	const char *device_str;

	switch (device_id & 0xfff) {
	case 0x415:
		device_str = "STM32L475/L476/L486";
		break;

	case 0x435:
		device_str = "STM32L43x/L44x";
		break;

	case 0x460:
		device_str = "STM32G07x/G08x";
		break;

	case 0x461:
		device_str = "STM32L496/L4A6";
		break;

	case 0x462:
		device_str = "STM32L45x/L46x";
		break;

	case 0x464:
		device_str = "STM32L41x/L42x";
		break;

	case 0x466:
		device_str = "STM32G03x/G04x";
		break;

	case 0x468:
		device_str = "STM32G43x/G44x";
		break;

	case 0x469:
		device_str = "STM32G47x/G48x";
		break;

	case 0x470:
		device_str = "STM32L4Rx/L4Sx";
		break;

	default:
		snprintf(buf, buf_size, "Cannot identify target as a STM32L4/G4/G0\n");
		return ERROR_FAIL;
	}

	snprintf(buf, buf_size, "%s (%s) - Rev: %1d.%02d",
			device_str, (stm32l4_info->bank2_start < bank->num_sectors) ?
			"dual bank" : "single bank", rev_id, rev_minor);

	return ERROR_OK;
}

static int stm32l4_mass_erase(struct flash_bank *bank, uint32_t action)
{
	int retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32l4_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(
		target, stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_CR_OFFS), action);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(
		target, stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_CR_OFFS),
		action | (1UL << FLASH_STRT));
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_wait_status_busy(bank,  FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target,
		stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_CR_OFFS), (1UL << FLASH_LOCK));
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32l4_handle_mass_erase_command)
{
	int i;
	uint32_t action;

	if (CMD_ARGC < 1) {
		command_print(CMD, "stm32l4x mass_erase <STM32L4 bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	action = (1UL << FLASH_MER1) | (1UL << FLASH_MER2);
	retval = stm32l4_mass_erase(bank, action);
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

	uint32_t reg_addr = STM32_FLASH_BASE;
	uint32_t value = 0;

	reg_addr += strtoul(CMD_ARGV[1], NULL, 16);

	retval = stm32l4_read_option(bank, reg_addr, &value);
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

	retval = stm32l4_write_option(bank, reg_addr, value, mask);
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

	struct target *target = bank->target;

	retval = stm32l4_unlock_reg(target);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32l4_unlock_option_reg(target);
	if (ERROR_OK != retval)
		return retval;

	/* Write the OBLLAUNCH bit in CR -> Cause device "POR" and option bytes reload */
	retval = target_write_u32(target, stm32l4_get_flash_reg(bank, STM32_FLASH_BASE + FLASH_CR_OFFS),
		(1UL << FLASH_OBLLAUNCH));

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
	if (stm32l4_write_option(bank, STM32_FLASH_BASE + FLASH_OPTR_OFFS, 0, 0x000000FF) != ERROR_OK) {
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

	if (stm32l4_write_option(bank, STM32_FLASH_BASE + FLASH_OPTR_OFFS, RDP_LEVEL_0, 0x000000FF) != ERROR_OK) {
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
