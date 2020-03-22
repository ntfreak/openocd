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

/* STM32WBxxx series for reference.
 *
 * RM0434 (STM32WB55)
 * http://www.st.com/resource/en/reference_manual/dm00318631.pdf
 *
 * RM0471 (STM32WB50)
 * http://www.st.com/resource/en/reference_manual/dm00622834.pdf
 */

/* STM32WLxxx series for reference.
 *
 * RM0461 (STM32WLEx)
 * http://www.st.com/resource/en/reference_manual/dm00530369.pdf
 */

/* STM32G0xxx series for reference.
 *
 * RM0444 (STM32G0x1)
 * http://www.st.com/resource/en/reference_manual/dm00371828.pdf
 *
 * RM0454 (STM32G0x0)
 * http://www.st.com/resource/en/reference_manual/dm00463896.pdf
 */

/* STM32G4xxx series for reference.
 *
 * RM0440 (STM32G43x/44x/47x/48x/49x/4Ax)
 * http://www.st.com/resource/en/reference_manual/dm00355726.pdf
 *
 * Cat. 2 devices have single bank only, page size is 2kByte.
 *
 * Cat. 3 devices have single and dual bank operating modes,
 * Page size is 2kByte (dual mode) or 4kByte (single mode).
 *
 * Bank mode is controlled by bit 22 (DBANK) in option bytes register.
 * Both banks are treated as a single OpenOCD bank.
 *
 * Cat. 4 devices have single bank only, page size is 2kByte.
 */

/* STM32L5xxx series for reference.
 *
 * RM0428 (STM32L552xx/STM32L562xx)
 * http://www.st.com/resource/en/reference_manual/dm00346336.pdf
 */

/* Erase time can be as high as 25ms, 10x this and assume it's toast... */

#define FLASH_ERASE_TIMEOUT 250

enum stm32l4_flash_reg_index {
	STM32_FLASH_ACR_INDEX,
	STM32_FLASH_KEYR_INDEX,
	STM32_FLASH_OPTKEYR_INDEX,
	STM32_FLASH_SR_INDEX,
	STM32_FLASH_CR_INDEX,
	STM32_FLASH_OPTR_INDEX,
	STM32_FLASH_WRP1AR_INDEX,
	STM32_FLASH_WRP1BR_INDEX,
	STM32_FLASH_WRP2AR_INDEX,
	STM32_FLASH_WRP2BR_INDEX,
	STM32_FLASH_REG_INDEX_NUM,
};

static const uint32_t stm32l4_flash_regs[STM32_FLASH_REG_INDEX_NUM] = {
	[STM32_FLASH_ACR_INDEX]      = 0x000,
	[STM32_FLASH_KEYR_INDEX]     = 0x008,
	[STM32_FLASH_OPTKEYR_INDEX]  = 0x00C,
	[STM32_FLASH_SR_INDEX]       = 0x010,
	[STM32_FLASH_CR_INDEX]       = 0x014,
	[STM32_FLASH_OPTR_INDEX]     = 0x020,
	[STM32_FLASH_WRP1AR_INDEX]   = 0x02C,
	[STM32_FLASH_WRP1BR_INDEX]   = 0x030,
	[STM32_FLASH_WRP2AR_INDEX]   = 0x04C,
	[STM32_FLASH_WRP2BR_INDEX]   = 0x050,
};

static const uint32_t stm32l5_ns_flash_regs[STM32_FLASH_REG_INDEX_NUM] = {
	[STM32_FLASH_ACR_INDEX]      = 0x000,
	[STM32_FLASH_KEYR_INDEX]     = 0x008,
	[STM32_FLASH_OPTKEYR_INDEX]  = 0x010,
	[STM32_FLASH_SR_INDEX]       = 0x020,
	[STM32_FLASH_CR_INDEX]       = 0x028,
	[STM32_FLASH_OPTR_INDEX]     = 0x040,
	[STM32_FLASH_WRP1AR_INDEX]   = 0x058,
	[STM32_FLASH_WRP1BR_INDEX]   = 0x05C,
	[STM32_FLASH_WRP2AR_INDEX]   = 0x068,
	[STM32_FLASH_WRP2BR_INDEX]   = 0x06C,
};

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
	const uint32_t *default_flash_regs;
	const uint32_t fsize_addr;
};

struct stm32l4_flash_bank {
	bool probed;
	uint32_t idcode;
	unsigned int bank1_sectors;
	bool dual_bank_mode;
	int hole_sectors;
	uint32_t user_bank_size;
	uint32_t wrpxxr_mask;
	const struct stm32l4_part_info *part_info;
	const uint32_t *flash_regs;
};

/* human readable list of families this drivers supports (sorted alphabetically) */
static const char *device_families = "STM32G0/G4/L4/L4+/L5/WB/WL";

static const struct stm32l4_rev stm32_415_revs[] = {
	{ 0x1000, "1" }, { 0x1001, "2" }, { 0x1003, "3" }, { 0x1007, "4" }
};

static const struct stm32l4_rev stm32_435_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x2001, "Y" },
};

static const struct stm32l4_rev stm32_460_revs[] = {
	{ 0x1000, "A/Z" } /* A and Z, no typo in RM! */, { 0x2000, "B" },
};

static const struct stm32l4_rev stm32_461_revs[] = {
	{ 0x1000, "A" }, { 0x2000, "B" },
};

static const struct stm32l4_rev stm32_462_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x2001, "Y" },
};

static const struct stm32l4_rev stm32_464_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x2001, "Y" },
};

static const struct stm32l4_rev stm32_466_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x2000, "B" },
};

static const struct stm32l4_rev stm32_468_revs[] = {
	{ 0x1000, "A" }, { 0x2000, "B" }, { 0x2001, "Z" },
};

static const struct stm32l4_rev stm32_469_revs[] = {
	{ 0x1000, "A" }, { 0x2000, "B" }, { 0x2001, "Z" },
};

static const struct stm32l4_rev stm32_470_revs[] = {
	{ 0x1000, "A" }, { 0x1001, "Z" }, { 0x1003, "Y" }, { 0x100F, "W" },
};

static const struct stm32l4_rev stm32_471_revs[] = {
	{ 0x1001, "Z" },
};

static const struct stm32l4_rev stm32_472_revs[] = {
	{ 0x1000, "A" }, { 0x2000, "B" },
};

static const struct stm32l4_rev stm32_479_revs[] = {
	{ 0x1000, "A" },
};

static const struct stm32l4_rev stm32_495_revs[] = {
	{ 0x2001, "2.1" },
};

static const struct stm32l4_rev stm32_496_revs[] = {
	{ 0x1000, "A" },
};

static const struct stm32l4_rev stm32_497_revs[] = {
	{ 0x1000, "1.0" },
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
	  .default_flash_regs    = stm32l4_flash_regs,
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
	  .default_flash_regs    = stm32l4_flash_regs,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x460,
	  .revs                  = stm32_460_revs,
	  .num_revs              = ARRAY_SIZE(stm32_460_revs),
	  .device_str            = "STM32G07/G08xx",
	  .max_flash_size_kb     = 128,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x40022000,
	  .default_flash_regs    = stm32l4_flash_regs,
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
	  .default_flash_regs    = stm32l4_flash_regs,
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
	  .default_flash_regs    = stm32l4_flash_regs,
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
	  .default_flash_regs    = stm32l4_flash_regs,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x466,
	  .revs                  = stm32_466_revs,
	  .num_revs              = ARRAY_SIZE(stm32_466_revs),
	  .device_str            = "STM32G03/G04xx",
	  .max_flash_size_kb     = 64,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x40022000,
	  .default_flash_regs    = stm32l4_flash_regs,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x468,
	  .revs                  = stm32_468_revs,
	  .num_revs              = ARRAY_SIZE(stm32_468_revs),
	  .device_str            = "STM32G43/G44xx",
	  .max_flash_size_kb     = 128,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x40022000,
	  .default_flash_regs    = stm32l4_flash_regs,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x469,
	  .revs                  = stm32_469_revs,
	  .num_revs              = ARRAY_SIZE(stm32_469_revs),
	  .device_str            = "STM32G47/G48xx",
	  .max_flash_size_kb     = 512,
	  .has_dual_bank         = true,
	  .flash_regs_base       = 0x40022000,
	  .default_flash_regs    = stm32l4_flash_regs,
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
	  .default_flash_regs    = stm32l4_flash_regs,
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
	  .default_flash_regs    = stm32l4_flash_regs,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x472,
	  .revs                  = stm32_472_revs,
	  .num_revs              = ARRAY_SIZE(stm32_472_revs),
	  .device_str            = "STM32L55/L56xx",
	  .max_flash_size_kb     = 512,
	  .has_dual_bank         = true,
	  .flash_regs_base       = 0x40022000,
	  .default_flash_regs    = stm32l5_ns_flash_regs,
	  .fsize_addr            = 0x0BFA05E0,
	},
	{
	  .id                    = 0x479,
	  .revs                  = stm32_479_revs,
	  .num_revs              = ARRAY_SIZE(stm32_479_revs),
	  .device_str            = "STM32G49/G4Axx",
	  .max_flash_size_kb     = 512,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x40022000,
	  .default_flash_regs    = stm32l4_flash_regs,
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
	  .default_flash_regs    = stm32l4_flash_regs,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x496,
	  .revs                  = stm32_496_revs,
	  .num_revs              = ARRAY_SIZE(stm32_496_revs),
	  .device_str            = "STM32WB3x",
	  .max_flash_size_kb     = 512,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x58004000,
	  .default_flash_regs    = stm32l4_flash_regs,
	  .fsize_addr            = 0x1FFF75E0,
	},
	{
	  .id                    = 0x497,
	  .revs                  = stm32_497_revs,
	  .num_revs              = ARRAY_SIZE(stm32_497_revs),
	  .device_str            = "STM32WLEx",
	  .max_flash_size_kb     = 256,
	  .has_dual_bank         = false,
	  .flash_regs_base       = 0x58004000,
	  .default_flash_regs    = stm32l4_flash_regs,
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

	stm32l4_info->probed = false;
	stm32l4_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline uint32_t stm32l4_get_flash_reg(struct flash_bank *bank, uint32_t reg_offset)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	return stm32l4_info->part_info->flash_regs_base + reg_offset;
}

static inline uint32_t stm32l4_get_flash_reg_by_index(struct flash_bank *bank,
	enum stm32l4_flash_reg_index reg_index)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	return stm32l4_get_flash_reg(bank, stm32l4_info->flash_regs[reg_index]);
}

static inline int stm32l4_read_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t *value)
{
	return target_read_u32(bank->target, stm32l4_get_flash_reg(bank, reg_offset), value);
}

static inline int stm32l4_read_flash_reg_by_index(struct flash_bank *bank,
	enum stm32l4_flash_reg_index reg_index, uint32_t *value)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	return stm32l4_read_flash_reg(bank, stm32l4_info->flash_regs[reg_index], value);
}

static inline int stm32l4_write_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t value)
{
	return target_write_u32(bank->target, stm32l4_get_flash_reg(bank, reg_offset), value);
}

static inline int stm32l4_write_flash_reg_by_index(struct flash_bank *bank,
	enum stm32l4_flash_reg_index reg_index, uint32_t value)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	return stm32l4_write_flash_reg(bank, stm32l4_info->flash_regs[reg_index], value);
}

static int stm32l4_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX, &status);
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
		stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX, status & FLASH_ERROR);
	}

	return retval;
}

static int stm32l4_unlock_reg(struct flash_bank *bank)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on STM32_FLASH_KEYR will fail
	 */
	int retval = stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_KEYR_INDEX, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_KEYR_INDEX, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, &ctrl);
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

	int retval = stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_OPTLOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_OPTKEYR_INDEX, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_OPTKEYR_INDEX, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_OPTLOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32l4_write_option(struct flash_bank *bank, uint32_t reg_offset,
	uint32_t value, uint32_t mask)
{
	uint32_t optiondata;
	int retval, retval2;

	retval = stm32l4_read_flash_reg(bank, reg_offset, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32l4_unlock_option_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

	optiondata = (optiondata & ~mask) | (value & mask);

	retval = stm32l4_write_flash_reg(bank, reg_offset, optiondata);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_OPTSTRT);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

err_lock:
	retval2 = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_LOCK | FLASH_OPTLOCK);

	if (retval != ERROR_OK)
		return retval;

	return retval2;
}

static int stm32l4_protect_check(struct flash_bank *bank)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;

	uint32_t wrp1ar, wrp1br, wrp2ar, wrp2br;
	stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_WRP1AR_INDEX, &wrp1ar);
	stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_WRP1BR_INDEX, &wrp1br);
	if (stm32l4_info->part_info->has_dual_bank) {
		stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_WRP2AR_INDEX, &wrp2ar);
		stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_WRP2BR_INDEX, &wrp2br);
	} else {
		/* prevent uninitialized errors */
		wrp2ar = 0;
		wrp2br = 0;
	}

	const uint8_t wrp1a_start = wrp1ar & stm32l4_info->wrpxxr_mask;
	const uint8_t wrp1a_end = (wrp1ar >> 16) & stm32l4_info->wrpxxr_mask;
	const uint8_t wrp1b_start = wrp1br & stm32l4_info->wrpxxr_mask;
	const uint8_t wrp1b_end = (wrp1br >> 16) & stm32l4_info->wrpxxr_mask;
	const uint8_t wrp2a_start = wrp2ar & stm32l4_info->wrpxxr_mask;
	const uint8_t wrp2a_end = (wrp2ar >> 16) & stm32l4_info->wrpxxr_mask;
	const uint8_t wrp2b_start = wrp2br & stm32l4_info->wrpxxr_mask;
	const uint8_t wrp2b_end = (wrp2br >> 16) & stm32l4_info->wrpxxr_mask;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (i < stm32l4_info->bank1_sectors) {
			if (((i >= wrp1a_start) &&
				 (i <= wrp1a_end)) ||
				((i >= wrp1b_start) &&
				 (i <= wrp1b_end)))
				bank->sectors[i].is_protected = 1;
			else
				bank->sectors[i].is_protected = 0;
		} else {
			assert(stm32l4_info->part_info->has_dual_bank == true);
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

static int stm32l4_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	int retval, retval2;

	assert((first <= last) && (last < bank->num_sectors));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32l4_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

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

	for (unsigned int i = first; i <= last; i++) {
		uint32_t erase_flags;
		erase_flags = FLASH_PER | FLASH_STRT;

		if (i >= stm32l4_info->bank1_sectors) {
			uint8_t snb;
			snb = i - stm32l4_info->bank1_sectors;
			erase_flags |= snb << FLASH_PAGE_SHIFT | FLASH_CR_BKER;
		} else
			erase_flags |= i << FLASH_PAGE_SHIFT;
		retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, erase_flags);
		if (retval != ERROR_OK)
			break;

		retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			break;

		bank->sectors[i].is_erased = 1;
	}

err_lock:
	retval2 = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_LOCK);

	if (retval != ERROR_OK)
		return retval;

	return retval2;
}

static int stm32l4_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
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

		ret = stm32l4_write_option(bank, stm32l4_info->flash_regs[STM32_FLASH_WRP2AR_INDEX], reg_value, 0xffffffff);
	}
	/* Bank 1 */
	reg_value = 0xFF; /* Default to bank un-protected */
	if (first < stm32l4_info->bank1_sectors) {
		if (set == 1) {
			uint8_t end = last >= stm32l4_info->bank1_sectors ? 0xFF : last;
			reg_value = (end << 16) | (first & 0xFF);
		}

		ret = stm32l4_write_option(bank, stm32l4_info->flash_regs[STM32_FLASH_WRP1AR_INDEX], reg_value, 0xffffffff);
	}

	return ret;
}

/* Count is in double-words */
static int stm32l4_write_block(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[6];
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

	/* memory buffer, size *must* be multiple of dword plus one dword for rp and one for wp */
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
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* count (double word-64bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);	/* flash status register */
	init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);	/* flash control register */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, stm32l4_get_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX));
	buf_set_u32(reg_params[5].value, 0, 32, stm32l4_get_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX));

	retval = target_run_flash_async_algorithm(target, buffer, count, 8,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
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
			stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_SR_INDEX, error);
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

static int stm32l4_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK, retval2;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The flash write must be aligned to a double word (8-bytes) boundary.
	 * The flash infrastructure ensures it, do just a security check */
	assert(offset % 8 == 0);
	assert(count % 8 == 0);

	/* STM32G4xxx Cat. 3 devices may have gaps between banks, check whether
	 * data to be written does not go into a gap:
	 * suppose buffer is fully contained in bank from sector 0 to sector
	 * num->sectors - 1 and sectors are ordered according to offset
	 */
	struct flash_sector *head = &bank->sectors[0];
	struct flash_sector *tail = &bank->sectors[bank->num_sectors - 1];

	while ((head < tail) && (offset >= (head + 1)->offset)) {
		/* buffer does not intersect head nor gap behind head */
		head++;
	}

	while ((head < tail) && (offset + count <= (tail - 1)->offset + (tail - 1)->size)) {
		/* buffer does not intersect tail nor gap before tail */
		--tail;
	}

	LOG_DEBUG("data: 0x%08" PRIx32 " - 0x%08" PRIx32 ", sectors: 0x%08" PRIx32 " - 0x%08" PRIx32,
		offset, offset + count - 1, head->offset, tail->offset + tail->size - 1);

	/* Now check that there is no gap from head to tail, this should work
	 * even for multiple or non-symmetric gaps
	 */
	while (head < tail) {
		if (head->offset + head->size != (head + 1)->offset) {
			LOG_ERROR("write into gap from " TARGET_ADDR_FMT " to " TARGET_ADDR_FMT,
				bank->base + head->offset + head->size,
				bank->base + (head + 1)->offset - 1);
			retval = ERROR_FLASH_DST_OUT_OF_BANK;
		}
		head++;
	}

	if (retval != ERROR_OK)
		return retval;

	retval = stm32l4_unlock_reg(bank);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32l4_write_block(bank, buffer, offset, count / 8);

err_lock:
	retval2 = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_LOCK);

	if (retval != ERROR_OK) {
		LOG_ERROR("block write failed");
		return retval;
	}
	return retval2;
}

static int stm32l4_read_idcode(struct flash_bank *bank, uint32_t *id)
{
	int retval;

	/* try reading possible IDCODE registers, in the following order */
	uint32_t DBGMCU_IDCODE[] = {DBGMCU_IDCODE_L4_G4, DBGMCU_IDCODE_G0, DBGMCU_IDCODE_L5};

	for (unsigned int i = 0; i < ARRAY_SIZE(DBGMCU_IDCODE); i++) {
		retval = target_read_u32(bank->target, DBGMCU_IDCODE[i], id);
		if ((retval == ERROR_OK) && ((*id & 0xfff) != 0) && ((*id & 0xfff) != 0xfff))
			return ERROR_OK;
	}

	LOG_ERROR("can't get the device id");
	return (retval == ERROR_OK) ? ERROR_FAIL : retval;
}

static int stm32l4_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	const struct stm32l4_part_info *part_info;
	uint16_t flash_size_kb = 0xffff;
	uint32_t device_id;
	uint32_t options;

	stm32l4_info->probed = false;

	/* read stm32 device id registers */
	int retval = stm32l4_read_idcode(bank, &stm32l4_info->idcode);
	if (retval != ERROR_OK)
		return retval;

	device_id = stm32l4_info->idcode & 0xFFF;

	for (unsigned int n = 0; n < ARRAY_SIZE(stm32l4_parts); n++) {
		if (device_id == stm32l4_parts[n].id)
			stm32l4_info->part_info = &stm32l4_parts[n];
	}

	if (!stm32l4_info->part_info) {
		LOG_WARNING("Cannot identify target as an %s family device.", device_families);
		return ERROR_FAIL;
	}

	part_info = stm32l4_info->part_info;
	stm32l4_info->flash_regs = stm32l4_info->part_info->default_flash_regs;

	char device_info[1024];
	retval = bank->driver->info(bank, device_info, sizeof(device_info));
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device idcode = 0x%08" PRIx32 " (%s)", stm32l4_info->idcode, device_info);

	/* get flash size from target. */
	retval = target_read_u16(target, part_info->fsize_addr, &flash_size_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_kb == 0xffff || flash_size_kb == 0
			|| flash_size_kb > part_info->max_flash_size_kb) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
			part_info->max_flash_size_kb);
		flash_size_kb = part_info->max_flash_size_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (stm32l4_info->user_bank_size) {
		LOG_WARNING("overriding size register by configured bank size - MAY CAUSE TROUBLE");
		flash_size_kb = stm32l4_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_kb);

	/* did we assign a flash size? */
	assert((flash_size_kb != 0xffff) && flash_size_kb);

	/* read flash option register */
	retval = stm32l4_read_flash_reg_by_index(bank, STM32_FLASH_OPTR_INDEX, &options);
	if (retval != ERROR_OK)
		return retval;

	stm32l4_info->bank1_sectors = 0;
	stm32l4_info->hole_sectors = 0;

	int num_pages = 0;
	int page_size_kb = 0;

	stm32l4_info->dual_bank_mode = false;
	bool use_dbank_bit = false;

	switch (device_id) {
	case 0x415: /* STM32L47/L48xx */
	case 0x461: /* STM32L49/L4Axx */
		/* if flash size is max (1M) the device is always dual bank
		 * 0x415: has variants with 512K
		 * 0x461: has variants with 512 and 256
		 * for these variants:
		 *   if DUAL_BANK = 0 -> single bank
		 *   else -> dual bank without gap
		 * note: the page size is invariant
		 */
		page_size_kb = 2;
		num_pages = flash_size_kb / page_size_kb;
		stm32l4_info->bank1_sectors = num_pages;

		/* check DUAL_BANK bit[21] if the flash is less than 1M */
		if (flash_size_kb == 1024 || (options & BIT(21))) {
			stm32l4_info->dual_bank_mode = true;
			stm32l4_info->bank1_sectors = num_pages / 2;
		}
		break;
	case 0x435: /* STM32L43/L44xx */
	case 0x460: /* STM32G07/G08xx */
	case 0x462: /* STM32L45/L46xx */
	case 0x464: /* STM32L41/L42xx */
	case 0x466: /* STM32G03/G04xx */
	case 0x468: /* STM32G43/G44xx */
	case 0x479: /* STM32G49/G4Axx */
	case 0x497: /* STM32WLEx */
		/* single bank flash */
		page_size_kb = 2;
		num_pages = flash_size_kb / page_size_kb;
		stm32l4_info->bank1_sectors = num_pages;
		break;
	case 0x469: /* STM32G47/G48xx */
		/* STM32G47/8 can be single/dual bank:
		 *   if DUAL_BANK = 0 -> single bank
		 *   else -> dual bank WITH gap
		 */
		page_size_kb = 4;
		num_pages = flash_size_kb / page_size_kb;
		stm32l4_info->bank1_sectors = num_pages;
		if (options & BIT(22)) {
			stm32l4_info->dual_bank_mode = true;
			page_size_kb = 2;
			num_pages = flash_size_kb / page_size_kb;
			stm32l4_info->bank1_sectors = num_pages / 2;

			/* for devices with trimmed flash, there is a gap between both banks */
			stm32l4_info->hole_sectors =
				(part_info->max_flash_size_kb - flash_size_kb) / (2 * page_size_kb);
		}
		break;
	case 0x470: /* STM32L4R/L4Sxx */
	case 0x471: /* STM32L4P5/L4Q5x */
		/* STM32L4R/S can be single/dual bank:
		 *   if size = 2M check DBANK bit(22)
		 *   if size = 1M check DB1M bit(21)
		 * STM32L4P/Q can be single/dual bank
		 *   if size = 1M check DBANK bit(22)
		 *   if size = 512K check DB512K bit(21)
		 */
		page_size_kb = 8;
		num_pages = flash_size_kb / page_size_kb;
		stm32l4_info->bank1_sectors = num_pages;
		use_dbank_bit = flash_size_kb == part_info->max_flash_size_kb;
		if ((use_dbank_bit && (options & BIT(22))) ||
			(!use_dbank_bit && (options & BIT(21)))) {
			stm32l4_info->dual_bank_mode = true;
			page_size_kb = 4;
			num_pages = flash_size_kb / page_size_kb;
			stm32l4_info->bank1_sectors = num_pages / 2;
		}
		break;
	case 0x472: /* STM32L55/L56xx */
		/* STM32L55/L56xx can be single/dual bank:
		 *   if size = 512K check DBANK bit(22)
		 *   if size = 256K check DB256K bit(21)
		 */
		page_size_kb = 4;
		num_pages = flash_size_kb / page_size_kb;
		stm32l4_info->bank1_sectors = num_pages;
		use_dbank_bit = flash_size_kb == part_info->max_flash_size_kb;
		if ((use_dbank_bit && (options & BIT(22))) ||
			(!use_dbank_bit && (options & BIT(21)))) {
			stm32l4_info->dual_bank_mode = true;
			page_size_kb = 2;
			num_pages = flash_size_kb / page_size_kb;
			stm32l4_info->bank1_sectors = num_pages / 2;
		}
		break;
	case 0x495: /* STM32WB5x */
	case 0x496: /* STM32WB3x */
		/* single bank flash */
		page_size_kb = 4;
		num_pages = flash_size_kb / page_size_kb;
		stm32l4_info->bank1_sectors = num_pages;
		break;
	default:
		LOG_ERROR("unsupported device");
		return ERROR_FAIL;
	}

	LOG_INFO("flash mode : %s-bank", stm32l4_info->dual_bank_mode ? "dual" : "single");

	const int gap_size_kb = stm32l4_info->hole_sectors * page_size_kb;

	if (gap_size_kb != 0) {
		LOG_INFO("gap detected from 0x%08x to 0x%08x",
			STM32_FLASH_BANK_BASE + stm32l4_info->bank1_sectors
				* page_size_kb * 1024,
			STM32_FLASH_BANK_BASE + (stm32l4_info->bank1_sectors
				* page_size_kb + gap_size_kb) * 1024 - 1);
	}

	/* number of significant bits in WRPxxR differs per device,
	 * always right adjusted, on some devices non-implemented
	 * bits read as '0', on others as '1' ...
	 * notably G4 Cat. 2 implement only 6 bits, contradicting the RM
	 */

	/* use *max_flash_size* instead of actual size as the trimmed versions
	 * certainly use the same number of bits
	 * max_flash_size is always power of two, so max_pages too
	 */
	uint32_t max_pages = stm32l4_info->part_info->max_flash_size_kb / page_size_kb;
	assert((max_pages & (max_pages - 1)) == 0);

	/* in dual bank mode number of pages is doubled, but extra bit is bank selection */
	stm32l4_info->wrpxxr_mask = ((max_pages >> (stm32l4_info->dual_bank_mode ? 1 : 0)) - 1);
	assert((stm32l4_info->wrpxxr_mask & 0xFFFF0000) == 0);
	LOG_DEBUG("WRPxxR mask 0x%04" PRIx16, (uint16_t)stm32l4_info->wrpxxr_mask);

	free(bank->sectors);

	bank->size = (flash_size_kb + gap_size_kb) * 1024;
	bank->base = STM32_FLASH_BANK_BASE;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (bank->sectors == NULL) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * page_size_kb * 1024;
		/* in dual bank configuration, if there is a gap between banks
		 * we fix up the sector offset to consider this gap */
		if (i >= stm32l4_info->bank1_sectors && stm32l4_info->hole_sectors)
			bank->sectors[i].offset += gap_size_kb * 1024;
		bank->sectors[i].size = page_size_kb * 1024;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32l4_info->probed = true;
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
				break;
			}
		}

		int buf_len = snprintf(buf, buf_size, "%s - Rev %s : 0x%04x",
				part_info->device_str, rev_str ? rev_str : "'unknown'", rev_id);

		if (stm32l4_info->probed)
			snprintf(buf + buf_len, buf_size - buf_len, " - %s-bank",
					stm32l4_info->dual_bank_mode ? "Flash dual" : "Flash single");

		return ERROR_OK;
	} else {
		snprintf(buf, buf_size, "Cannot identify target as an %s device", device_families);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int stm32l4_mass_erase(struct flash_bank *bank)
{
	int retval, retval2;
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
		goto err_lock;

	/* mass erase flash memory */
	retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT / 10);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, action);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, action | FLASH_STRT);
	if (retval != ERROR_OK)
		goto err_lock;

	retval = stm32l4_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

err_lock:
	retval2 = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_LOCK);

	if (retval != ERROR_OK)
		return retval;

	return retval2;
}

COMMAND_HANDLER(stm32l4_handle_mass_erase_command)
{
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
		for (unsigned int i = 0; i < bank->num_sectors; i++)
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
	if (CMD_ARGC != 1)
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

	/* Set OBL_LAUNCH bit in CR -> system reset and option bytes reload,
	 * but the RMs explicitly do *NOT* list this as power-on reset cause, and:
	 * "Note: If the read protection is set while the debugger is still
	 * connected through JTAG/SWD, apply a POR (power-on reset) instead of a system reset."
	 */
	retval = stm32l4_write_flash_reg_by_index(bank, STM32_FLASH_CR_INDEX, FLASH_OBL_LAUNCH);

	command_print(CMD, "stm32l4x option load completed. Power-on reset might be required");

	/* Need to re-probe after change */
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	stm32l4_info->probed = false;

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
	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	if (stm32l4_write_option(bank, stm32l4_info->flash_regs[STM32_FLASH_OPTR_INDEX], 0, 0x000000FF) != ERROR_OK) {
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

	struct stm32l4_flash_bank *stm32l4_info = bank->driver_priv;
	if (stm32l4_write_option(bank, stm32l4_info->flash_regs[STM32_FLASH_OPTR_INDEX],
			RDP_LEVEL_0, 0x000000FF) != ERROR_OK) {
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
