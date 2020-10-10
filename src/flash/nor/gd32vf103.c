/***************************************************************************
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

/* gd32vf103 register locations */

#define FLASH_REG_BASE_B0 0x40022000
#define FLASH_REG_BASE_B1 0x40022040

#define FMC_WS    0x00
#define FMC_KEY    0x04
#define FMC_OBKEY 0x08
#define FMC_STAT      0x0C
#define FMC_CTL      0x10
#define FMC_ADDR      0x14
#define FMC_OBSTAT     0x1C

/* TODO: Check if code using these really should be hard coded to bank 0.
 * There are valid cases, on dual flash devices the protection of the
 * second bank is done on the bank0 reg's. */
#define FMC_WS_B0     0x40022000
#define FMC_KEY_B0    0x40022004
#define FMC_OBKEY_B0 0x40022008
#define FMC_STAT_B0      0x4002200C
#define FMC_CTL_B0      0x40022010
#define FMC_ADDR_B0      0x40022014
#define FMC_OBSTAT_B0     0x4002201C
#define FMC_WP_B0    0x40022020

/* option byte location */

#define FMC_OB_RDP		0x1FFFF800

/* FMC_CTL register bits */

#define FMC_CTL_PG		(1 << 0)
#define FMC_CTL_PER		(1 << 1)
#define FMC_CTL_MER		(1 << 2)
#define FMC_CTL_OBPG	(1 << 4)
#define FMC_CTL_OBER	(1 << 5)
#define FMC_CTL_START	(1 << 6)
#define FMC_CTL_LK		(1 << 7)
#define FMC_CTL_OBWEN	(1 << 9)

/* FMC_STAT register bits */

#define FMC_STAT_BUSY		(1 << 0)
#define FMC_STAT_PGERR		(1 << 2)
#define FMC_STAT_WPERR	(1 << 4)
#define FMC_STAT_ENDF		(1 << 5)

/* FMC_OBSTAT bit definitions (reading) */

#define FMC_OBSTAT_OBERR		0
#define FMC_OBSTAT_SPC		1
#define FMC_OBSTAT_WDG_SW		2
#define FMC_OBSTAT_RST_DSLEEP		3
#define FMC_OBSTAT_RST_STDBY	4
#define FMC_OBSTAT_BB		5	/* dual flash bank only */

/* register unlock keys */

#define UNLOCK_KEY0			0x45670123
#define UNLOCK_KEY1			0xCDEF89AB

/* timeout values */

#define FLASH_WRITE_TIMEOUT 500
#define FLASH_ERASE_TIMEOUT 5000

struct gd32vf103_options {
	uint16_t RDP;
	uint16_t user_options;
	uint16_t user_data;
	uint16_t protection[4];
};

struct gd32vf103_flash_bank {
	struct gd32vf103_options option_bytes;
	int ppage_size;
	int probed;

	bool has_dual_banks;
	/* used to access dual flash bank gd32vf103 */
	uint32_t register_base;
	uint16_t default_rdp;
	int user_data_offset;
	int option_offset;
	uint32_t user_bank_size;
};

static int gd32vf103_mass_erase(struct flash_bank *bank);
static int get_gd32vf103_info(struct flash_bank *bank, char *buf, int buf_size);
static int gd32vf103_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count);

/* flash bank gd32vf103 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(gd32vf103_flash_bank_command)
{
	struct gd32vf103_flash_bank *gd32vf103_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	gd32vf103_info = malloc(sizeof(struct gd32vf103_flash_bank));

	bank->driver_priv = gd32vf103_info;
	gd32vf103_info->probed = 0;
	gd32vf103_info->has_dual_banks = false;
	gd32vf103_info->register_base = FLASH_REG_BASE_B0;
	gd32vf103_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline int gd32vf103_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;
	return reg + gd32vf103_info->register_base;
}

static inline int gd32vf103_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, gd32vf103_get_flash_reg(bank, FMC_STAT), status);
}

static int gd32vf103_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = gd32vf103_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FMC_STAT_BUSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FMC_STAT_WPERR) {
		LOG_ERROR("gd32vf103 device protected");
		retval = ERROR_FAIL;
	}

	if (status & FMC_STAT_PGERR) {
		LOG_ERROR("gd32vf103 device programming failed");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & (FMC_STAT_WPERR | FMC_STAT_PGERR)) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_STAT),
				FMC_STAT_WPERR | FMC_STAT_PGERR);
	}
	return retval;
}

static int gd32vf103_check_operation_supported(struct flash_bank *bank)
{
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;

	/* if we have a dual flash bank device then
	 * we need to perform option byte stuff on bank0 only */
	if (gd32vf103_info->register_base != FLASH_REG_BASE_B0) {
		LOG_ERROR("Option Byte Operation's must use bank0");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int gd32vf103_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;

	gd32vf103_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, FMC_OBSTAT_B0, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	gd32vf103_info->option_bytes.user_options = (optiondata >> gd32vf103_info->option_offset >> 2) & 0xffff;
	gd32vf103_info->option_bytes.user_data = (optiondata >> gd32vf103_info->user_data_offset) & 0xffff;
	gd32vf103_info->option_bytes.RDP = (optiondata & (1 << FMC_OBSTAT_SPC)) ? 0xFFFF : 0x5AA5;

	if (optiondata & (1 << FMC_OBSTAT_SPC))
		LOG_INFO("Device Security Bit Set");

	/* each bit refers to a 4bank protection */
	retval = target_read_u32(target, FMC_WP_B0, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	gd32vf103_info->option_bytes.protection[0] = (uint16_t)optiondata;
	gd32vf103_info->option_bytes.protection[1] = (uint16_t)(optiondata >> 8);
	gd32vf103_info->option_bytes.protection[2] = (uint16_t)(optiondata >> 16);
	gd32vf103_info->option_bytes.protection[3] = (uint16_t)(optiondata >> 24);

	return ERROR_OK;
}

static int gd32vf103_erase_options(struct flash_bank *bank)
{
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;

	gd32vf103_info = bank->driver_priv;

	/* read current options */
	gd32vf103_read_options(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, FMC_KEY_B0, UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FMC_KEY_B0, UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, FMC_OBKEY_B0, UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FMC_OBKEY_B0, UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	/* erase option bytes */
	retval = target_write_u32(target, FMC_CTL_B0, FMC_CTL_OBER | FMC_CTL_OBWEN);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FMC_CTL_B0, FMC_CTL_OBER | FMC_CTL_START | FMC_CTL_OBWEN);
	if (retval != ERROR_OK)
		return retval;

	retval = gd32vf103_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	gd32vf103_info->option_bytes.RDP = gd32vf103_info->default_rdp;

	return ERROR_OK;
}

static int gd32vf103_write_options(struct flash_bank *bank)
{
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;

	gd32vf103_info = bank->driver_priv;

	/* unlock flash registers */
	int retval = target_write_u32(target, FMC_KEY_B0, UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FMC_KEY_B0, UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, FMC_OBKEY_B0, UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FMC_OBKEY_B0, UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	/* program option bytes */
	retval = target_write_u32(target, FMC_CTL_B0, FMC_CTL_OBPG | FMC_CTL_OBWEN);
	if (retval != ERROR_OK)
		return retval;

	uint8_t opt_bytes[16];

	/* SPC */
	target_buffer_set_u16(target, opt_bytes, gd32vf103_info->option_bytes.RDP);
	/* USER */
	target_buffer_set_u16(target, opt_bytes + 2, gd32vf103_info->option_bytes.user_options);
	/* DATA[7:0] */
	target_buffer_set_u16(target, opt_bytes + 4, gd32vf103_info->option_bytes.user_data & 0xff);
	/* DATA[15:8] */
	target_buffer_set_u16(target, opt_bytes + 6, (gd32vf103_info->option_bytes.user_data >> 8) & 0xff);
	/* WP[7:0] */
	target_buffer_set_u16(target, opt_bytes + 8, gd32vf103_info->option_bytes.protection[0]);
	/* WP[15:8] */
	target_buffer_set_u16(target, opt_bytes + 10, gd32vf103_info->option_bytes.protection[1]);
	/* WP[23:16]] */
	target_buffer_set_u16(target, opt_bytes + 12, gd32vf103_info->option_bytes.protection[2]);
	/* WP[31:24] */
	target_buffer_set_u16(target, opt_bytes + 14, gd32vf103_info->option_bytes.protection[3]);

	uint32_t offset = FMC_OB_RDP - bank->base;
	retval = gd32vf103_write_block(bank, opt_bytes, offset, sizeof(opt_bytes) / 2);
	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			LOG_ERROR("working area required to erase options bytes");
		return retval;
	}

	retval = target_write_u32(target, FMC_CTL_B0, FMC_CTL_LK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int gd32vf103_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;

	uint32_t protection;
	int i, s;
	int num_bits;
	int set;

	int retval = gd32vf103_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	/* medium density - each bit refers to a 4bank protection
	 * high density - each bit refers to a 2bank protection */
	retval = target_read_u32(target, FMC_WP_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	/* medium density - each protection bit is for 4 * 1K pages
	 * high density - each protection bit is for 2 * 2K pages */
	num_bits = (bank->num_sectors / gd32vf103_info->ppage_size);

	if (gd32vf103_info->ppage_size == 2) {
		/* high density flash/connectivity line protection */

		set = 1;

		if (protection & (1 << 31))
			set = 0;

		/* bit 31 controls sector 62 - 255 protection for high density
		 * bit 31 controls sector 62 - 127 protection for connectivity line */
		for (s = 62; s < bank->num_sectors; s++)
			bank->sectors[s].is_protected = set;

		if (bank->num_sectors > 61)
			num_bits = 31;

		for (i = 0; i < num_bits; i++) {
			set = 1;

			if (protection & (1 << i))
				set = 0;

			for (s = 0; s < gd32vf103_info->ppage_size; s++)
				bank->sectors[(i * gd32vf103_info->ppage_size) + s].is_protected = set;
		}
	} else {
		/* low/medium density flash protection */
		for (i = 0; i < num_bits; i++) {
			set = 1;

			if (protection & (1 << i))
				set = 0;

			for (s = 0; s < gd32vf103_info->ppage_size; s++)
				bank->sectors[(i * gd32vf103_info->ppage_size) + s].is_protected = set;
		}
	}

	return ERROR_OK;
}

static int gd32vf103_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;
	uint32_t optiondata;
	uint32_t obstat;

	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	gd32vf103_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	target_read_u32(target, FMC_WP_B0, &optiondata);
	target_read_u32(target, FMC_OBSTAT_B0, &obstat);
	if ((0xFFFFFFFF != optiondata) || ((obstat & 0x2) != 0)) {
		gd32vf103_erase_options(bank);
		optiondata = 0xFFFFFFFF;
		gd32vf103_info->option_bytes.RDP = 0x5AA5;
		gd32vf103_info->option_bytes.protection[0] = (uint16_t)optiondata;
		gd32vf103_info->option_bytes.protection[1] = (uint16_t)(optiondata >> 8);
		gd32vf103_info->option_bytes.protection[2] = (uint16_t)(optiondata >> 16);
		gd32vf103_info->option_bytes.protection[3] = (uint16_t)(optiondata >> 24);

		gd32vf103_write_options(bank);
		LOG_INFO("Unlock flash Sucess !!! Pls Reset Platfrom !!\n");
		return ERROR_FAIL;
	}
	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return gd32vf103_mass_erase(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	for (i = first; i <= last; i++) {
		retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_CTL), FMC_CTL_PER);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_ADDR),
				bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target,
				gd32vf103_get_flash_reg(bank, FMC_CTL), FMC_CTL_PER | FMC_CTL_START);
		if (retval != ERROR_OK)
			return retval;

		retval = gd32vf103_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_CTL), FMC_CTL_LK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int gd32vf103_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;
	uint16_t prot_reg[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	int i, reg, bit;
	int status;
	uint32_t protection;

	gd32vf103_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = gd32vf103_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if ((first % gd32vf103_info->ppage_size) != 0) {
		LOG_WARNING("aligned start protect sector to a %d sector boundary",
				gd32vf103_info->ppage_size);
		first = first - (first % gd32vf103_info->ppage_size);
	}
	if (((last + 1) % gd32vf103_info->ppage_size) != 0) {
		LOG_WARNING("aligned end protect sector to a %d sector boundary",
				gd32vf103_info->ppage_size);
		last++;
		last = last - (last % gd32vf103_info->ppage_size);
		last--;
	}

	/* medium density - each bit refers to a 4bank protection
	 * high density - each bit refers to a 2bank protection */
	retval = target_read_u32(target, FMC_WP_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	prot_reg[0] = (uint16_t)protection;
	prot_reg[1] = (uint16_t)(protection >> 8);
	prot_reg[2] = (uint16_t)(protection >> 16);
	prot_reg[3] = (uint16_t)(protection >> 24);

	if (gd32vf103_info->ppage_size == 2) {
		/* high density flash */

		/* bit 7 controls sector 62 - 255 protection */
		if (last > 61) {
			if (set)
				prot_reg[3] &= ~(1 << 7);
			else
				prot_reg[3] |= (1 << 7);
		}

		if (first > 61)
			first = 62;
		if (last > 61)
			last = 61;

		for (i = first; i <= last; i++) {
			reg = (i / gd32vf103_info->ppage_size) / 8;
			bit = (i / gd32vf103_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	} else {
		/* medium density flash */
		for (i = first; i <= last; i++) {
			reg = (i / gd32vf103_info->ppage_size) / 8;
			bit = (i / gd32vf103_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	}

	status = gd32vf103_erase_options(bank);
	if (status != ERROR_OK)
		return status;

	gd32vf103_info->option_bytes.protection[0] = prot_reg[0];
	gd32vf103_info->option_bytes.protection[1] = prot_reg[1];
	gd32vf103_info->option_bytes.protection[2] = prot_reg[2];
	gd32vf103_info->option_bytes.protection[3] = prot_reg[3];

	return gd32vf103_write_options(bank);
}

static int gd32vf103_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	int retval = ERROR_OK;

	static const uint8_t gd32vf103_flash_write_code[] = {
#include "../../../contrib/loaders/flash/gd32vf103/gd32vf103.inc"
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(gd32vf103_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(gd32vf103_flash_write_code), gd32vf103_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "a0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "a3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "a4", 32, PARAM_IN_OUT);	/* target address */


	uint32_t wp_addr = source->address;
	uint32_t rp_addr = source->address + 4;
	uint32_t fifo_start_addr = source->address + 8;
	uint32_t fifo_end_addr = source->address + source->size;

	uint32_t wp = fifo_start_addr;
	uint32_t rp = fifo_start_addr;

	/* (2:block size) */
	uint32_t thisrun_bytes = fifo_end_addr-fifo_start_addr-2;

	retval = target_write_u32(target, rp_addr, rp);
	if (retval != ERROR_OK)
		return retval;

	while (count > 0) {
		retval = target_read_u32(target, rp_addr, &rp);
		if (retval != ERROR_OK) {
			LOG_ERROR("failed to get read pointer");
			break;
		}

		if (wp != rp) {
			LOG_ERROR("Failed to write flash ;;  rp = 0x%x ;;; wp = 0x%x", rp, wp);
			break;
		}
		wp = fifo_start_addr;
		rp = fifo_start_addr;
		retval = target_write_u32(target, rp_addr, rp);
		if (retval != ERROR_OK)
			break;
		/* Limit to the amount of data we actually want to write */
		if (thisrun_bytes > count * 2)
			thisrun_bytes = count * 2;

		/* Write data to fifo */
		retval = target_write_buffer(target, wp, thisrun_bytes, buffer);
		if (retval != ERROR_OK)
			break;

		/* Update counters and wrap write pointer */
		buffer += thisrun_bytes;
		count -= thisrun_bytes / 2;
		rp = fifo_start_addr;
		wp = fifo_start_addr+thisrun_bytes;

		/* Store updated write pointer to target */
		retval = target_write_u32(target, wp_addr, wp);
		if (retval != ERROR_OK)
			break;
		retval = target_write_u32(target, rp_addr, rp);
		if (retval != ERROR_OK)
			return retval;

		buf_set_u32(reg_params[0].value, 0, 32, gd32vf103_info->register_base);
		buf_set_u32(reg_params[1].value, 0, 32, thisrun_bytes/2);
		buf_set_u32(reg_params[2].value, 0, 32, source->address);
		buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
		buf_set_u32(reg_params[4].value, 0, 32, address);

		retval = target_run_algorithm(target, 0, NULL, 5, reg_params,
				write_algorithm->address, write_algorithm->address+4,
				10000, NULL);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d",
					write_algorithm->address, retval);
			return retval;
			}
		address += thisrun_bytes;
	}


	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) & FMC_STAT_PGERR) {
			LOG_ERROR("flash memory not erased before writing");
			/* Clear but report errors */
			target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_STAT), FMC_STAT_PGERR);
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) & FMC_STAT_WPERR) {
			LOG_ERROR("flash memory write protected");
			/* Clear but report errors */
			target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_STAT), FMC_STAT_WPERR);
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

static int gd32vf103_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint8_t *new_buffer = NULL;


	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of bytes, the data has to be padded. Duplicate
	 * the buffer and use the normal code path with a single block write since
	 * it's probably cheaper than to special case the last odd write using
	 * discrete accesses. */
	if (count & 1) {
		new_buffer = malloc(count + 1);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write, padding with 0xff");
		buffer = memcpy(new_buffer, buffer, count);
		new_buffer[count++] = 0xff;
	}

	uint32_t words_remaining = count / 2;
	int retval, retval2;

	/* unlock flash registers */
	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY0);
	if (retval != ERROR_OK)
		goto cleanup;
	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY1);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_CTL), FMC_CTL_PG);
	if (retval != ERROR_OK)
		goto cleanup;

	/* try using a block write */
	retval = gd32vf103_write_block(bank, buffer, offset, words_remaining);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (words_remaining > 0) {
			uint16_t value;
			memcpy(&value, buffer, sizeof(uint16_t));

			retval = target_write_u16(target, bank->base + offset, value);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			retval = gd32vf103_wait_status_busy(bank, 5);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			words_remaining--;
			buffer += 2;
			offset += 2;
		}
	}

reset_pg_and_lock:
	retval2 = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_CTL), FMC_CTL_LK);
	if (retval == ERROR_OK)
		retval = retval2;

cleanup:
	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int gd32vf103_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{

	struct target *target = bank->target;
	uint32_t device_id_register = 0xE0042000;
	/* read GD32VF103 device id register */
	int retval = target_read_u32(target, device_id_register, device_id);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int gd32vf103_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
	struct target *target = bank->target;
	uint32_t  flash_size_reg = 0x1FFFF7E0;

	int  retval = target_read_u16(target, flash_size_reg, flash_size_in_kb);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int gd32vf103_probe(struct flash_bank *bank)
{
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	int page_size;
	uint32_t base_address = 0x08000000;

	gd32vf103_info->probed = 0;
	gd32vf103_info->register_base = FLASH_REG_BASE_B0;
	gd32vf103_info->user_data_offset = 10;
	gd32vf103_info->option_offset = 0;

	/* default factory protection level */
	gd32vf103_info->default_rdp = 0x5AA5;

	/* read gd32vf103 device id register */
	int retval = gd32vf103_get_device_id(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	/* set page size, protection granularity and max flash size depending on family */
	switch (device_id & 0xfff) {
		case 0x410:
		case 0x418: /* connectivity line density */
			page_size = 1024;
			gd32vf103_info->ppage_size = 4;
			max_flash_size_in_kb = 128;
			break;
		default:
			LOG_WARNING("Cannot identify target as a gd32vf103 family.");
			return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = gd32vf103_get_flash_size(bank, &flash_size_in_kb);
	LOG_INFO("flash_size_in_kb = 0x%08" PRIx32 "", flash_size_in_kb);
	/* failed reading flash size or flash size invalid, default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("gd32vf103 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	if (gd32vf103_info->has_dual_banks) {
		/* split reported size into matching bank */
		if (bank->base != 0x08080000) {
			/* bank 0 will be fixed 512k */
			flash_size_in_kb = 512;
		} else {
			flash_size_in_kb -= 512;
			/* bank1 also uses a register offset */
			gd32vf103_info->register_base = FLASH_REG_BASE_B1;
			base_address = 0x08080000;
		}
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (gd32vf103_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = gd32vf103_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	gd32vf103_info->probed = 1;

	return ERROR_OK;
}

static int gd32vf103_auto_probe(struct flash_bank *bank)
{
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;
	if (gd32vf103_info->probed)
		return ERROR_OK;
	return gd32vf103_probe(bank);
}

static int get_gd32vf103_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint32_t dbgmcu_idcode;

		/* read gd32vf103 device id register */
	int retval = gd32vf103_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint16_t rev_id = dbgmcu_idcode >> 16;
	const char *device_str;
	const char *rev_str = NULL;

	switch (device_id) {

	case 0x418:
		device_str = "gd32vf103 (gdm32501)";

		switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;

		case 0x1001:
			rev_str = "B";
			break;
		}
		break;
	default:
		snprintf(buf, buf_size, "Cannot identify target as a GD32VF103 x\n");
		return ERROR_FAIL;
	}

	if (rev_str != NULL)
		snprintf(buf, buf_size, "%s - Rev: %s", device_str, rev_str);
	else
		snprintf(buf, buf_size, "%s - Rev: unknown (0x%04x)", device_str, rev_id);

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_lock_command)
{
	struct target *target = NULL;
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	gd32vf103_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = gd32vf103_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if (gd32vf103_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	gd32vf103_info->option_bytes.RDP = 0;

	if (gd32vf103_write_options(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "gd32vf103 locked");

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_unlock_command)
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

	retval = gd32vf103_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	if (gd32vf103_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to unlock device");
		return ERROR_OK;
	}

	if (gd32vf103_write_options(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "gd32vf103 unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_options_read_command)
{
	uint32_t optionbyte;
	struct target *target = NULL;
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	gd32vf103_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = gd32vf103_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = target_read_u32(target, FMC_OBSTAT_B0, &optionbyte);
	if (retval != ERROR_OK)
		return retval;
	command_print(CMD, "Option Byte: 0x%" PRIx32 "", optionbyte);

	int user_data = optionbyte;

	if (optionbyte >> FMC_OBSTAT_OBERR & 1)
		command_print(CMD, "Option Byte Complement Error");

	if (optionbyte >> FMC_OBSTAT_SPC & 1)
		command_print(CMD, "Readout Protection On");
	else
		command_print(CMD, "Readout Protection Off");

	/* user option bytes are offset depending on variant */
	optionbyte >>= gd32vf103_info->option_offset;

	if (optionbyte >> FMC_OBSTAT_WDG_SW & 1)
		command_print(CMD, "Software Watchdog");
	else
		command_print(CMD, "Hardware Watchdog");

	if (optionbyte >> FMC_OBSTAT_RST_DSLEEP & 1)
		command_print(CMD, "Stop: No reset generated");
	else
		command_print(CMD, "Stop: Reset generated");

	if (optionbyte >> FMC_OBSTAT_RST_STDBY & 1)
		command_print(CMD, "Standby: No reset generated");
	else
		command_print(CMD, "Standby: Reset generated");

	if (gd32vf103_info->has_dual_banks) {
		if (optionbyte >> FMC_OBSTAT_BB & 1)
			command_print(CMD, "Boot: Bank 0");
		else
			command_print(CMD, "Boot: Bank 1");
	}

	command_print(CMD, "User Option0: 0x%02" PRIx8,
			(uint8_t)((user_data >> gd32vf103_info->user_data_offset) & 0xff));
	command_print(CMD, "User Option1: 0x%02" PRIx8,
			(uint8_t)((user_data >> (gd32vf103_info->user_data_offset + 8)) & 0xff));

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_options_write_command)
{
	struct target *target = NULL;
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	uint16_t optionbyte;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	gd32vf103_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = gd32vf103_check_operation_supported(bank);
	if (ERROR_OK != retval)
		return retval;

	retval = gd32vf103_read_options(bank);
	if (ERROR_OK != retval)
		return retval;

	/* start with current options */
	optionbyte = gd32vf103_info->option_bytes.user_options;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	while (CMD_ARGC) {
		if (strcmp("SWWDG", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 0);
		else if (strcmp("HWWDG", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 0);
		else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 1);
		else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 1);
		else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 2);
		else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 2);
		else if (gd32vf103_info->has_dual_banks) {
			if (strcmp("BOOT0", CMD_ARGV[0]) == 0)
				optionbyte |= (1 << 3);
			else if (strcmp("BOOT1", CMD_ARGV[0]) == 0)
				optionbyte &= ~(1 << 3);
			else
				return ERROR_COMMAND_SYNTAX_ERROR;
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}

	if (gd32vf103_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to erase options");
		return ERROR_OK;
	}

	gd32vf103_info->option_bytes.user_options = optionbyte;

	if (gd32vf103_write_options(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to write options");
		return ERROR_OK;
	}

	command_print(CMD, "gd32vf103 write options complete.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.");

	return ERROR_OK;
}

static int gd32vf103_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock option flash registers */
	int retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_CTL), FMC_CTL_MER);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_CTL),
			FMC_CTL_MER | FMC_CTL_START);
	if (retval != ERROR_OK)
		return retval;

	retval = gd32vf103_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, gd32vf103_get_flash_reg(bank, FMC_CTL), FMC_CTL_LK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = gd32vf103_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "gd32vf103 mass erase complete");
	} else
		command_print(CMD, "gd32vf103 mass erase failed");

	return retval;
}

static const struct command_registration gd32vf103_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = gd32vf103_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = gd32vf103_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = gd32vf103_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = gd32vf103_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option byte.",
	},
	{
		.name = "options_write",
		.handler = gd32vf103_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP')",
		.help = "Replace bits in device option byte.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration gd32vf103_command_handlers[] = {
	{
		.name = "gd32vf103",
		.mode = COMMAND_ANY,
		.help = "gd32vf103 flash command group",
		.usage = "",
		.chain = gd32vf103_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver gd32vf103_flash = {
	.name = "gd32vf103",
	.commands = gd32vf103_command_handlers,
	.flash_bank_command = gd32vf103_flash_bank_command,
	.erase = gd32vf103_erase,
	.protect = gd32vf103_protect,
	.write = gd32vf103_write,
	.read = default_flash_read,
	.probe = gd32vf103_probe,
	.auto_probe = gd32vf103_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = gd32vf103_protect_check,
	.info = get_gd32vf103_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
