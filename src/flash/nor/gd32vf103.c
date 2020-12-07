/***************************************************************************
 *   Copyright (C) 2020 by GigaDevice Semiconductor, Inc.                  *
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
#define BIT(x)                          ((uint32_t)((uint32_t)0x01U<<(x)))
#define BITS(start, end)                ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end))))
#define GET_BITS(regval, start, end)    (((regval) & BITS((start), (end))) >> (start))

/* FMC and option byte definition */
#define FMC_BASE                         0x40022000                /*!< FMC register base address */
#define OB_BASE                          0x1FFFF800                /*!< option byte base address */

/* registers definitions */
#define FMC_WS                           0x40022000                /*!< FMC wait state register */
#define FMC_KEY                          0x40022004                /*!< FMC unlock key register */
#define FMC_OBKEY                        0x40022008                /*!< FMC option bytes unlock key register */
#define FMC_STAT                         0x4002200C                /*!< FMC status register */
#define FMC_CTL                          0x40022010                /*!< FMC control register */
#define FMC_ADDR                         0x40022014                /*!< FMC address register */
#define FMC_OBSTAT                       0x4002201C                /*!< FMC option bytes status register */
#define FMC_WP                           0x40022020                /*!< FMC write protection register */
#define FMC_PID                          0x40022100                /*!< FMC product ID register */

#define OB_SPC_USER                      0x1FFFF800                /*!< option byte spc and user value */
#define OB_DATA                          0x1FFFF804                /*!< option byte data value*/
#define OB_WP                            0x1FFFF808                /*!< option byte write protection */

/* FMC_STAT */
#define FMC_STAT_BUSY                     BIT(0)                   /*!< flash busy flag bit */
#define FMC_STAT_PGERR                    BIT(2)                   /*!< flash program error flag bit */
#define FMC_STAT_WPERR                    BIT(4)                   /*!< flash write protection error flag bit */
#define FMC_STAT_ENDF                     BIT(5)                   /*!< end of operation flag bit */

/* FMC_CTL */
#define FMC_CTL_PG                        BIT(0)                   /*!< main flash program command bit */
#define FMC_CTL_PER                       BIT(1)                   /*!< main flash page erase bit */
#define FMC_CTL_MER                       BIT(2)                   /*!< main flash mass erase bit */
#define FMC_CTL_OBPG                      BIT(4)                   /*!< option bytes program command bit */
#define FMC_CTL_OBER                      BIT(5)                   /*!< option bytes erase command bit */
#define FMC_CTL_START                     BIT(6)                   /*!< send erase command to FMC bit */
#define FMC_CTL_LK                        BIT(7)                   /*!< flash lock bit */
#define FMC_CTL_OBWEN                     BIT(9)                   /*!< option bytes erase/program enable bit */
#define FMC_CTL_ERRIE                     BIT(10)                  /*!< error interrupt enable bit */
#define FMC_CTL_ENDIE                     BIT(12)                  /*!< end of operation interrupt enable bit */

/* FMC_OBSTAT */
#define FMC_OBSTAT_OBERR                  BIT(0)                   /*!< option bytes read error bit. */
#define FMC_OBSTAT_SPC                    BIT(1)                   /*!< option bytes security protection code */
#define FMC_OBSTAT_USER                   BITS(2, 9)                /*!< USER after system reset */
#define FMC_OBSTAT_DATA                   BITS(10, 25)              /*!< DATA after system reset */

/* option byte user bit offset */
#define OB_FWDGT_SW_OFFSET                ((uint8_t)BIT(0))        /*!< software free watchdog timer */
#define OB_DEEPSLEEP_NRST_OFFSET          ((uint8_t)BIT(1))        /*!< no reset when entering deepsleep mode */
#define OB_STDBY_NRST_OFFSET              ((uint8_t)BIT(2))        /*!< no reset when entering deepsleep mode */
#define OB_BOOT_B0_OFFSET                 ((uint8_t)BIT(3))        /*!< boot from bank0 */

/* unlock keys */
#define UNLOCK_KEY0                       0x45670123
#define UNLOCK_KEY1                       0xCDEF89AB

/* read protect configuration */
#define FMC_NSPC                          ((uint16_t)0x5AA5U)       /*!< no security protection */

/* FMC time out */
#define FMC_TIMEOUT_COUNT 5000

/* number of flash bank */
#define GD32VF103_FLASH_BANKS 1

/* option bytes structure */
struct gd32vf103_options {
	uint16_t spc;
	uint16_t user;
	uint32_t data;
	uint16_t wrp[4];
};

/* gd32vf103 flash bank type structure */
struct gd32vf103_flash_bank_type {
	uint32_t bank_base;
	uint32_t bank_size;
	uint32_t bank_page_size;
	uint32_t wrp_page_size;
};

/* gd32vf103 flash bank structure */
struct gd32vf103_flash_bank {
	struct gd32vf103_options option_bytes;
	int probed;
	uint32_t cpu_id;
	uint32_t dbg_id;
	uint32_t flash_size;
	struct gd32vf103_flash_bank_type gd32vf103_bank[GD32VF103_FLASH_BANKS];
};

static int gd32vf103_mass_erase(struct flash_bank *bank);
static int gd32vf103_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count);

FLASH_BANK_COMMAND_HANDLER(gd32vf103_flash_bank_command)
{
	struct gd32vf103_flash_bank *gd32vf103_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	gd32vf103_info = malloc(sizeof(struct gd32vf103_flash_bank));

	bank->driver_priv = gd32vf103_info;
	gd32vf103_info->probed = 0;
	gd32vf103_info->cpu_id = 0;
	gd32vf103_info->dbg_id = 0;
	gd32vf103_info->flash_size = bank->size;
	gd32vf103_info->gd32vf103_bank[0].bank_base = 0x08000000;
	gd32vf103_info->gd32vf103_bank[0].bank_size = bank->size;
	gd32vf103_info->gd32vf103_bank[0].bank_page_size = 1024;
	gd32vf103_info->gd32vf103_bank[0].wrp_page_size = 4;

	return ERROR_OK;
}

static int gd32vf103_ready_wait(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for FMC ready */
	do {
		alive_sleep(1);
		timeout--;
		retval = target_read_u32(target, FMC_STAT, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
	} while (((status & FMC_STAT_BUSY) != 0) && timeout);

	if (timeout == 0) {
		LOG_DEBUG("GD32: Flash ready wait ... timed out waiting for flash ready");
		return ERROR_FAIL;
	}

	if (status & FMC_STAT_WPERR) {
		LOG_ERROR("GD32: Flash ready wait ... gd32vf103 device is write/erase protected");
		retval = ERROR_FAIL;
	}

	if (status & FMC_STAT_PGERR) {
		LOG_ERROR("GD32: Flash ready wait ... gd32vf103 device programming failed");
		retval = ERROR_FAIL;
	}

	/* clear all FMC status errors */
	if (status & (FMC_STAT_WPERR | FMC_STAT_PGERR)) {
		target_write_u32(target, FMC_STAT,
				FMC_STAT_WPERR | FMC_STAT_PGERR);
	}
	return retval;
}


static int gd32vf103_ob_get(struct flash_bank *bank)
{
	uint32_t fmc_obstat_reg, fmc_wp_reg, ob_data;
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;

	gd32vf103_info = bank->driver_priv;

	/* read current option byte status register */
	int retval = target_read_u32(target, FMC_OBSTAT, &fmc_obstat_reg);
	if (retval != ERROR_OK) {
		return retval;
		LOG_INFO("GD32: Get option bytes status ... read FMC_OBSTAT error");
	}
	/* get option byte status register OB_DATA and OB_USER data */
	gd32vf103_info->option_bytes.user = (uint16_t) (GET_BITS(fmc_obstat_reg, 2U, 9U));
	gd32vf103_info->option_bytes.data = (uint16_t) (GET_BITS(fmc_obstat_reg, 10U, 25U));

	if (fmc_obstat_reg & (1 << FMC_OBSTAT_SPC)) {
		gd32vf103_info->option_bytes.spc = 0x5AA5;
		LOG_INFO("GD32: Get option bytes ... device protection bit set");
	} else {
		/* read current option bytes */
		target_read_u32(target, OB_BASE, &ob_data);
		gd32vf103_info->option_bytes.spc = ob_data & 0xffff;
		LOG_INFO("GD32: Get option bytes ... device no protection");
	}

	/* each bit refers to a 4 pages protection */
	retval = target_read_u32(target, FMC_WP, &fmc_wp_reg);
	if (retval != ERROR_OK) {
		return retval;
		LOG_INFO("GD32: Get option bytes ... read FMC_WP error");
	}

	gd32vf103_info->option_bytes.wrp[0] = (uint16_t)fmc_wp_reg;
	gd32vf103_info->option_bytes.wrp[1] = (uint16_t)(fmc_wp_reg >> 8);
	gd32vf103_info->option_bytes.wrp[2] = (uint16_t)(fmc_wp_reg >> 16);
	gd32vf103_info->option_bytes.wrp[3] = (uint16_t)(fmc_wp_reg >> 24);

	return ERROR_OK;
}

static int gd32vf103_ob_erase(struct flash_bank *bank)
{
	uint32_t retry = 100;
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;
	uint32_t fmc_ctl_reg;
	gd32vf103_info = bank->driver_priv;

	/* read current options */
	gd32vf103_ob_get(bank);

	/* unlock the main FMC operation */
	do {
		target_write_u32(target, FMC_KEY, UNLOCK_KEY0);
		target_write_u32(target, FMC_KEY, UNLOCK_KEY1);

		target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
		fmc_ctl_reg &= FMC_CTL_LK;
	} while ((retry--) && fmc_ctl_reg);
	if (retry == 0) {
		LOG_DEBUG("GD32: Option bytes erase ... timed out waiting for flash unlock");
		return ERROR_FAIL;
	}
	/* unlock the option bytes operation */
	do {
		target_write_u32(target, FMC_OBKEY, UNLOCK_KEY0);
		target_write_u32(target, FMC_OBKEY, UNLOCK_KEY1);

		target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
		fmc_ctl_reg &= FMC_CTL_OBWEN;
	} while ((retry--) && fmc_ctl_reg);
	if (retry == 0) {
		LOG_DEBUG("GD32: Option bytes erase ... timed out waiting for flash option byte unlock");
		return ERROR_FAIL;
	}

	/* erase option bytes */
	target_write_u32(target, FMC_CTL, FMC_CTL_OBER | FMC_CTL_OBWEN);
	target_write_u32(target, FMC_CTL, FMC_CTL_OBER | FMC_CTL_START | FMC_CTL_OBWEN);

	int retval = gd32vf103_ready_wait(bank, FMC_TIMEOUT_COUNT);
	if (retval != ERROR_OK) {
		LOG_DEBUG("GD32: Option byte erase ... waiting for option bytes erase failed");
		return retval;
	}
	/* lock the main FMC operation */
	target_write_u32(target, FMC_CTL, FMC_CTL_LK);

	/* set SPC to no security protection */
	gd32vf103_info->option_bytes.spc = FMC_NSPC;

	return ERROR_OK;
}

static int gd32vf103_ob_write(struct flash_bank *bank)
{
	uint32_t retry = 100;
	uint32_t fmc_ctl_reg;
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;

	gd32vf103_info = bank->driver_priv;

	/* unlock the main FMC operation */
	do {
		target_write_u32(target, FMC_KEY, UNLOCK_KEY0);
		target_write_u32(target, FMC_KEY, UNLOCK_KEY1);

		target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
		fmc_ctl_reg &= FMC_CTL_LK;
	} while ((retry--) && fmc_ctl_reg);
	if (retry == 0) {
		LOG_DEBUG("GD32: Option bytes write ... timed out waiting for flash unlock");
		return ERROR_FAIL;
	}

	/* unlock the option bytes operation */
	do {
		target_write_u32(target, FMC_OBKEY, UNLOCK_KEY0);
		target_write_u32(target, FMC_OBKEY, UNLOCK_KEY1);

		target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
		fmc_ctl_reg &= FMC_CTL_OBWEN;
	} while ((retry--) && fmc_ctl_reg);
	if (retry == 0) {
		LOG_DEBUG("GD32: Option bytes wirte ... timed out waiting for flash option byte unlock");
		return ERROR_FAIL;
	}

	/* program option bytes */
	target_write_u32(target, FMC_CTL, FMC_CTL_OBPG | FMC_CTL_OBWEN);

	target_write_u32(target, OB_BASE, (gd32vf103_info->option_bytes.user << 16) | gd32vf103_info->option_bytes.spc);
	target_write_u16(target, OB_BASE + 4, gd32vf103_info->option_bytes.data & 0xff);
	target_write_u16(target, OB_BASE + 6, gd32vf103_info->option_bytes.data >> 8 & 0xff);
	target_write_u16(target, OB_BASE + 8, gd32vf103_info->option_bytes.wrp[0]);
	target_write_u16(target, OB_BASE + 10, gd32vf103_info->option_bytes.wrp[1]);
	target_write_u16(target, OB_BASE + 12, gd32vf103_info->option_bytes.wrp[2]);
	target_write_u16(target, OB_BASE + 14, gd32vf103_info->option_bytes.wrp[3]);

	target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
	fmc_ctl_reg &= ~FMC_CTL_OBPG;
	target_write_u32(target, FMC_CTL, fmc_ctl_reg);
	/* lock the main FMC operation */
	target_write_u32(target, FMC_CTL, FMC_CTL_LK);

	return ERROR_OK;
}

static int gd32vf103_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;

	uint32_t fmc_wp_reg;
	uint32_t i, s;
	unsigned int num_bits;
	int set;

	target_read_u32(target, FMC_WP, &fmc_wp_reg);
	LOG_INFO("fmc_wp_reg = 0x%08" PRIx32 "", fmc_wp_reg);
	/* each protection bit is for 4 * 1K pages */
	num_bits = (bank->num_sectors / gd32vf103_info->gd32vf103_bank[0].wrp_page_size);

		/* flash write/erase protection */
		for (i = 0; i < num_bits; i++) {
			set = 1;

			if (fmc_wp_reg & (1 << i))
				set = 0;

			for (s = 0; s < gd32vf103_info->gd32vf103_bank[0].wrp_page_size; s++)
				bank->sectors[(i * gd32vf103_info->gd32vf103_bank[0].wrp_page_size) + s].is_protected = set;
		}

	return ERROR_OK;
}

static int gd32vf103_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	unsigned int i;
	uint32_t fmc_wp_reg, fmc_obstat_reg, fmc_ctl_reg;
	uint32_t retry = 100;

	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	gd32vf103_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("GD32: Flash erase ... sector erase(%u to %u)", first, last);

	target_read_u32(target, FMC_WP, &fmc_wp_reg);
	target_read_u32(target, FMC_OBSTAT, &fmc_obstat_reg);
	if ((0xFFFFFFFF != fmc_wp_reg) || ((fmc_obstat_reg & 0x2) != 0)) {
		gd32vf103_ob_erase(bank);
		fmc_wp_reg = 0xFFFFFFFF;
		gd32vf103_info->option_bytes.spc = FMC_NSPC;
		gd32vf103_info->option_bytes.wrp[0] = (uint16_t)fmc_wp_reg;
		gd32vf103_info->option_bytes.wrp[1] = (uint16_t)(fmc_wp_reg >> 8);
		gd32vf103_info->option_bytes.wrp[2] = (uint16_t)(fmc_wp_reg >> 16);
		gd32vf103_info->option_bytes.wrp[3] = (uint16_t)(fmc_wp_reg >> 24);

		gd32vf103_ob_write(bank);
		LOG_INFO("GD32: Flash erase ... device is proteced, please reset device !!\n");
		return ERROR_FAIL;
	}
	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return gd32vf103_mass_erase(bank);

	/* unlock the main FMC operation */
	do {
		target_write_u32(target, FMC_KEY, UNLOCK_KEY0);
		target_write_u32(target, FMC_KEY, UNLOCK_KEY1);

		target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
		fmc_ctl_reg = fmc_ctl_reg & FMC_CTL_LK;
	} while ((retry--) && fmc_ctl_reg);

	if (retry == 0) {
		LOG_INFO("GD32: Flash erase ...timed out waiting for flash unlock");
		return ERROR_FAIL;
	}

	for (i = first; i <= last; i++) {
		target_write_u32(target, FMC_CTL, FMC_CTL_PER);
		target_write_u32(target, FMC_ADDR, bank->base + bank->sectors[i].offset);
		target_write_u32(target, FMC_CTL, FMC_CTL_PER | FMC_CTL_START);

		int retval = gd32vf103_ready_wait(bank, FMC_TIMEOUT_COUNT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	/* lock the main FMC operation */
	target_write_u32(target, FMC_CTL, FMC_CTL_LK);
	LOG_INFO("erase ok");

	return ERROR_OK;
}

static int gd32vf103_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct gd32vf103_flash_bank *gd32vf103_info = NULL;
	struct target *target = bank->target;
	uint16_t wrp_tmp[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	unsigned int i, reg, bit;
	int status;
	uint32_t fmc_wp_reg;

	gd32vf103_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first % gd32vf103_info->gd32vf103_bank[0].wrp_page_size) != 0) {
		LOG_WARNING("aligned start protect sector to a %d sector boundary",
				gd32vf103_info->gd32vf103_bank[0].wrp_page_size);
		first = first - (first % gd32vf103_info->gd32vf103_bank[0].wrp_page_size);
	}
	if (((last + 1) % gd32vf103_info->gd32vf103_bank[0].wrp_page_size) != 0) {
		LOG_WARNING("aligned end protect sector to a %d sector boundary",
				gd32vf103_info->gd32vf103_bank[0].wrp_page_size);
		last++;
		last = last - (last % gd32vf103_info->gd32vf103_bank[0].wrp_page_size);
		last--;
	}

	int retval = target_read_u32(target, FMC_WP, &fmc_wp_reg);
	if (retval != ERROR_OK)
		return retval;

	wrp_tmp[0] = (uint16_t)fmc_wp_reg;
	wrp_tmp[1] = (uint16_t)(fmc_wp_reg >> 8);
	wrp_tmp[2] = (uint16_t)(fmc_wp_reg >> 16);
	wrp_tmp[3] = (uint16_t)(fmc_wp_reg >> 24);

	for (i = first; i <= last; i++) {
		reg = (i / gd32vf103_info->gd32vf103_bank[0].wrp_page_size) / 8;
		bit = (i / gd32vf103_info->gd32vf103_bank[0].wrp_page_size) - (reg * 8);

		if (set)
			wrp_tmp[reg] &= ~(1 << bit);
		else
			wrp_tmp[reg] |= (1 << bit);
	}


	status = gd32vf103_ob_erase(bank);
	if (status != ERROR_OK)
		return status;

	gd32vf103_info->option_bytes.wrp[0] = wrp_tmp[0];
	gd32vf103_info->option_bytes.wrp[1] = wrp_tmp[1];
	gd32vf103_info->option_bytes.wrp[2] = wrp_tmp[2];
	gd32vf103_info->option_bytes.wrp[3] = wrp_tmp[3];
	LOG_INFO("wrp_tmp[0] = 0x%08" PRIx32 "", wrp_tmp[0]);
	LOG_INFO("wrp_tmp[1] = 0x%08" PRIx32 "", wrp_tmp[1]);
	LOG_INFO("wrp_tmp[2] = 0x%08" PRIx32 "", wrp_tmp[2]);
	LOG_INFO("wrp_tmp[3] = 0x%08" PRIx32 "", wrp_tmp[3]);
	return gd32vf103_ob_write(bank);
	return ERROR_OK;
}

static int gd32vf103_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	int retval = ERROR_OK;

	/* see contrib/loaders/flash/gd32vf103.S for src */
	static const uint8_t gd32vf103_flash_write_code[] = {
0x6f, 0x00, 0x80, 0x00, 0x73, 0x00, 0x10, 0x00, 0x03, 0x2b, 0x06, 0x00, 0x63, 0x0c, 0x0b, 0x04,
0x83, 0x2a, 0x46, 0x00, 0xb3, 0x87, 0x6a, 0x41, 0xe3, 0x88, 0x07, 0xfe, 0x03, 0xdb, 0x0a, 0x00,
0x23, 0x10, 0x67, 0x01, 0x93, 0x8a, 0x2a, 0x00, 0x13, 0x07, 0x27, 0x00, 0x83, 0x2b, 0xc5, 0x00,
0x93, 0xf7, 0x1b, 0x00, 0xe3, 0x9c, 0x07, 0xfe, 0x93, 0xf7, 0x4b, 0x01, 0x63, 0x90, 0x07, 0x02,
0x63, 0xe6, 0xda, 0x00, 0x93, 0x0a, 0x06, 0x00, 0x93, 0x8a, 0x8a, 0x00, 0x23, 0x22, 0x56, 0x01,
0x93, 0x85, 0xf5, 0xff, 0x63, 0x88, 0x05, 0x00, 0x6f, 0xf0, 0x1f, 0xfb, 0x13, 0x05, 0x00, 0x00,
0x23, 0x22, 0xa6, 0x00, 0x13, 0x85, 0x0b, 0x00, 0x6f, 0xf0, 0xdf, 0xf9};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(gd32vf103_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("GD32: Flash block write ... no working area for block memory writes");
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

			LOG_WARNING("GD32: Flash block write ... no large enough working area available, can't do block memory writes");
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
	uint32_t thisrun_bytes = fifo_end_addr-fifo_start_addr-2;

	retval = target_write_u32(target, rp_addr, rp);
	if (retval != ERROR_OK)
		return retval;

	while (count > 0) {
		retval = target_read_u32(target, rp_addr, &rp);
		if (retval != ERROR_OK) {
			LOG_ERROR("GD32: Flash block write ... failed to get read pointer");
			break;
		}

		if (wp != rp) {
			LOG_ERROR("GD32: Flash block write ... failed to write flash ;;  rp = 0x%x ;;; wp = 0x%x", rp, wp);
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

		buf_set_u32(reg_params[0].value, 0, 32, FMC_BASE);
		buf_set_u32(reg_params[1].value, 0, 32, thisrun_bytes/2);
		buf_set_u32(reg_params[2].value, 0, 32, source->address);
		buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
		buf_set_u32(reg_params[4].value, 0, 32, address);

		retval = target_run_algorithm(target, 0, NULL, 5, reg_params,
				write_algorithm->address, write_algorithm->address+4,
				10000, NULL);

		if (retval != ERROR_OK) {
			LOG_ERROR("GD32: Flash block write ... failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d",
					write_algorithm->address, retval);
			return retval;
			}
		address += thisrun_bytes;
	}

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("GD32: Flash block write ... error %d executing gd32xxx flash write algorithm", retval);

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
	uint32_t add_bytes;
	uint32_t fmc_ctl_reg;
	uint32_t retry = 100;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("bank=%p buffer=%p offset=%08" PRIx32 " count=%08" PRIx32 "",
		bank, buffer, offset, count);

	if (offset & 0x1) {
		LOG_ERROR("GD32: Flash write ... offset size must be half-word aligned");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* support half-word program */
	add_bytes = 2 - count % 2;
	if (add_bytes) {
		new_buffer = malloc(count + add_bytes);
		if (new_buffer == NULL) {
			LOG_ERROR("GD32: Flash write ... not half-word to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("GD32: Flash write ... not double words to write, padding with 0xff");
		memcpy(new_buffer, buffer, count);
		for (unsigned int i = 0; i < add_bytes; i++)
			new_buffer[count+i] = 0xff;
	}

	uint32_t halfword_remaining = (count + add_bytes) / 2;
	int retval;

	/* unlock the main FMC operation */
	do {
		target_write_u32(target, FMC_KEY, UNLOCK_KEY0);
		target_write_u32(target, FMC_KEY, UNLOCK_KEY1);

		target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
		fmc_ctl_reg = fmc_ctl_reg & FMC_CTL_LK;
	} while ((retry--) && fmc_ctl_reg);

	if (retry == 0) {
		LOG_DEBUG("GD32: Flash write ... timed out waiting for flash unlock");
		return ERROR_FAIL;
	}

	target_write_u32(target, FMC_CTL, FMC_CTL_PG);

	LOG_INFO("GD32: Flash write ... half word to be prgrammed = 0x%08" PRIx32 "", halfword_remaining);

	/* write block data */
	retval = gd32vf103_write_block(bank, new_buffer, offset, halfword_remaining);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("GD32: Flash write ... couldn't use block writes, falling back to single memory accesses");

		while (halfword_remaining > 0) {
			uint16_t value;
			memcpy(&value, new_buffer, sizeof(uint16_t));

			retval = target_write_u16(target, bank->base + offset, value);

			retval = gd32vf103_ready_wait(bank, 5);
			if (retval != ERROR_OK)
				return retval;

			halfword_remaining--;
			buffer += 2;
			offset += 2;
		}
	}

	target_write_u32(target, FMC_CTL, FMC_CTL_LK);

	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int gd32vf103_probe(struct flash_bank *bank)
{
	struct gd32vf103_flash_bank *gd32vf103_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flash_size_reg, dbg_id_reg;
	unsigned int i;
	uint16_t max_flash_size_in_kb, flash_size;
	uint32_t base_address = 0x08000000;
	int retval;

	gd32vf103_info->probed = 0;

	/* gd32vf103 device id register address */
	dbg_id_reg = 0xE0042000;

	/* read gd32vf103 device id register */
	target_read_u32(target, dbg_id_reg, &gd32vf103_info->dbg_id);
	if (0x410 == (gd32vf103_info->dbg_id & 0xfff)) {
		flash_size_reg = 0x1FFFF7E0;
		gd32vf103_info->gd32vf103_bank[0].bank_page_size = 1024;
		gd32vf103_info->gd32vf103_bank[0].wrp_page_size = 4;
		max_flash_size_in_kb = 128;
	} else {
		LOG_WARNING("Cannot identify target as a GD32VF103\n");
	}
	LOG_INFO("device id = 0x%08" PRIx32 "", gd32vf103_info->dbg_id);

	/* get target device flash size */
	retval = target_read_u16(target, flash_size_reg, &flash_size);
	gd32vf103_info->flash_size = (uint32_t)flash_size;

	/* target device default flash size */
	if (retval != ERROR_OK || gd32vf103_info->flash_size == 0xffff || gd32vf103_info->flash_size == 0) {
		LOG_WARNING("gd32vf103 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		gd32vf103_info->flash_size = max_flash_size_in_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (gd32vf103_info->gd32vf103_bank[0].bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		gd32vf103_info->flash_size = gd32vf103_info->gd32vf103_bank[0].bank_size / 1024;
	}

	LOG_INFO("flash size = %dkbytes", gd32vf103_info->flash_size);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* bank->sectors */
	bank->base = base_address;
	bank->num_sectors = gd32vf103_info->flash_size * 1024 / gd32vf103_info->gd32vf103_bank[0].bank_page_size;
	bank->size = (bank->num_sectors * gd32vf103_info->gd32vf103_bank[0].bank_page_size);
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * gd32vf103_info->gd32vf103_bank[0].bank_page_size;
		bank->sectors[i].size = gd32vf103_info->gd32vf103_bank[0].bank_page_size;
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


	if (gd32vf103_ob_erase(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to erase options");
		return ERROR_OK;
	}

	/* set security protection */
	gd32vf103_info->option_bytes.spc = 0;

	if (gd32vf103_ob_write(bank) != ERROR_OK) {
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

	if (gd32vf103_ob_erase(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to unlock device");
		return ERROR_OK;
	}

	if (gd32vf103_ob_write(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "gd32vf103 unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_ob_read_command)
{
	uint32_t fmc_obstat_reg;
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

	retval = target_read_u32(target, FMC_OBSTAT, &fmc_obstat_reg);
	if (retval != ERROR_OK)
		return retval;
	command_print(CMD, "Option byte: 0x%" PRIx32 "", fmc_obstat_reg);

	if (fmc_obstat_reg  & 1)
		command_print(CMD, "Option byte error");

	if (fmc_obstat_reg >> 1 & 1)
		command_print(CMD, "Security protection on");
	else
		command_print(CMD, "No security protection");

	command_print(CMD, "Data option0: 0x%02" PRIx8,
			(uint8_t)((fmc_obstat_reg >> 16) & 0xff));
	command_print(CMD, "Data option1: 0x%02" PRIx8,
			(uint8_t)((fmc_obstat_reg >> 24) & 0xff));

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_ob_write_command)
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


	retval = gd32vf103_ob_get(bank);
	if (ERROR_OK != retval)
		return retval;

	/* start with current options */
	optionbyte = gd32vf103_info->option_bytes.user;

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
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}

	if (gd32vf103_ob_erase(bank) != ERROR_OK) {
		command_print(CMD, "gd32vf103 failed to erase options");
		return ERROR_OK;
	}

	gd32vf103_info->option_bytes.user = optionbyte;

	if (gd32vf103_ob_write(bank) != ERROR_OK) {
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
	uint32_t fmc_ctl_reg;
	uint32_t retry = 100;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock the main FMC operation */
	do {
		target_write_u32(target, FMC_KEY, UNLOCK_KEY0);
		target_write_u32(target, FMC_KEY, UNLOCK_KEY1);

		target_read_u32 (target, FMC_CTL, &fmc_ctl_reg);
		fmc_ctl_reg = fmc_ctl_reg & FMC_CTL_LK;
	} while ((retry--) && fmc_ctl_reg);

	if (retry == 0) {
		LOG_DEBUG("GD32: Flash mass erase ...timed out waiting for flash unlock");
		return ERROR_FAIL;
	}

	/* mass erase flash memory */
	target_write_u32(target, FMC_CTL, FMC_CTL_MER);
	target_write_u32(target, FMC_CTL, FMC_CTL_MER | FMC_CTL_START);

	int retval = gd32vf103_ready_wait(bank, FMC_TIMEOUT_COUNT);
	if (retval != ERROR_OK)
		return retval;

	target_write_u32(target, FMC_CTL, FMC_CTL_LK);

	return ERROR_OK;
}

COMMAND_HANDLER(gd32vf103_handle_mass_erase_command)
{
	unsigned int i;

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
		.name = "ob_read",
		.handler = gd32vf103_handle_ob_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option byte.",
	},
	{
		.name = "ob_write",
		.handler = gd32vf103_handle_ob_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTDPSLP'|'NORSTSTDPSLP') "
			"('RSTSTDBY'|'NORSTSTDBY')",
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

const struct flash_driver gd32vf103_flash = {
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
	.free_driver_priv = default_flash_free_driver_priv,
};
