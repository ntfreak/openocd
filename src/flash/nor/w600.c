/***************************************************************************
 *   Copyright (C) 2018 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
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

#define W600_FLASH_SECSIZE		0x1000
#define W600_FLASH_PAGESIZE		0x100
#define W600_FLASH_BASE			0x08000000
#define W600_FLASH_PROTECT_SIZE	0x2000

/* w600 register locations */

#define QFLASH_REGBASE			0X40002000
#define QFLASH_CMD_INFO			(QFLASH_REGBASE + 0)
#define QFLASH_CMD_START		(QFLASH_REGBASE + 4)
#define QFLASH_BUFFER			(QFLASH_REGBASE + 0X200)

#define QFLASH_CMD_READ			(1ul << 14)
#define QFLASH_CMD_WRITE		0
#define QFLASH_CMD_ADDR			(1ul << 31)
#define QFLASH_CMD_DATA			(1ul << 15)
#define QFLASH_CMD_DATALEN(len)	(((len) & 0x3FF) << 16)

#define QFLASH_CMD_RDID			(QFLASH_CMD_READ | 0x9F)
#define QFLASH_CMD_WREN			(QFLASH_CMD_WRITE | 0x06)
#define QFLASH_CMD_WRDI			(QFLASH_CMD_WRITE | 0x04)
#define QFLASH_CMD_SE			(QFLASH_CMD_WRITE | QFLASH_CMD_ADDR | (1ul << 11) | 0x20)
#define QFLASH_CMD_PP			(QFLASH_CMD_WRITE | QFLASH_CMD_ADDR | (1ul << 12) | 0x02)

#define QFLASH_START			(1ul << 28)
#define QFLASH_ADDR(addr)		(((addr) & 0xFFFFF) << 8)
#define QFLASH_CRM(crm)			(((crm) & 0xFF) << 0)

struct w600_flash_param {
	uint8_t id;
	uint8_t se_delay;
	uint8_t pp_delay;
};
static const struct w600_flash_param w600_param[] = {
	{
		.id = 0x85,
		.se_delay = 8,
		.pp_delay = 2,
	},
	{
		.id = 0x1C,
		.se_delay = 50,
		.pp_delay = 1,
	},
	{
		.id = 0xC8,
		.se_delay = 45,
		.pp_delay = 1,
	},
	{
		.id = 0x0B,
		.se_delay = 60,
		.pp_delay = 1,
	},
	{
		.id = 0x68,
		.se_delay = 50,
		.pp_delay = 1,
	},
};

struct w600_flash_bank {
	bool probed;

	uint32_t id;
	const struct w600_flash_param *param;
	uint32_t register_base;
	uint32_t user_bank_size;
};

/* flash bank w600 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(w600_flash_bank_command)
{
	struct w600_flash_bank *w600_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	w600_info = malloc(sizeof(struct w600_flash_bank));

	bank->driver_priv = w600_info;
	w600_info->probed = false;
	w600_info->register_base = QFLASH_REGBASE;
	w600_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static int w600_get_delay(struct flash_bank *bank, uint32_t cmd)
{
	struct w600_flash_bank *w600_info = bank->driver_priv;

	if (!w600_info->param)
		return 0;

	switch (cmd) {
	case QFLASH_CMD_SE:
		return w600_info->param->se_delay;
	case QFLASH_CMD_PP:
		return w600_info->param->pp_delay;
	default:
		return 0;
	}
}

static int w600_start_do(struct flash_bank *bank, uint32_t cmd, uint32_t addr,
		uint32_t len, int timeout)
{
	struct target *target = bank->target;

	if (len > 0)
		cmd |= QFLASH_CMD_DATALEN(len - 1) | QFLASH_CMD_DATA;

	LOG_DEBUG("WRITE CMD: 0x%08" PRIx32 "", cmd);
	int retval = target_write_u32(target, QFLASH_CMD_INFO, cmd);
	if (retval != ERROR_OK)
		return retval;

	addr |= QFLASH_START;
	LOG_DEBUG("WRITE START: 0x%08" PRIx32 "", addr);
	retval = target_write_u32(target, QFLASH_CMD_START, addr);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("DELAY %dms", timeout);
	alive_sleep(timeout);

	int retry = 100;
	uint32_t status;
	for (;;) {
		LOG_DEBUG("READ START...");
		retval = target_read_u32(target, QFLASH_CMD_START, &status);
		if (retval == ERROR_OK)
			LOG_DEBUG("READ START: 0x%08" PRIx32 "", status);
		else
			LOG_DEBUG("READ START FAILED");

		if ((retval != ERROR_OK) || (status & QFLASH_START)) {
			if (retry-- <= 0) {
				LOG_ERROR("timed out waiting for flash");
				return ERROR_FAIL;
			}
			continue;
		}
		break;
	}

	return retval;
}

static int w600_write_enable(struct flash_bank *bank)
{
	return w600_start_do(bank, QFLASH_CMD_WREN, 0, 0, 0);
}

static int w600_write_disable(struct flash_bank *bank)
{
	return w600_start_do(bank, QFLASH_CMD_WRDI, 0, 0, 0);
}

static int w600_start(struct flash_bank *bank, uint32_t cmd, uint32_t addr,
		uint32_t len)
{
	int retval = w600_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = w600_start_do(bank, cmd, addr, len, w600_get_delay(bank, cmd));
	if (retval != ERROR_OK)
		return retval;

	retval = w600_write_disable(bank);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int w600_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	if (first < W600_FLASH_PROTECT_SIZE / W600_FLASH_SECSIZE) {
		LOG_ERROR("can not erase protected area");
		return ERROR_FAIL;
	}

	for (unsigned int i = first; i <= last; i++) {
		retval = w600_start(bank, QFLASH_CMD_SE,
			QFLASH_ADDR(bank->sectors[i].offset), 0);
		if (retval != ERROR_OK)
			break;
	}

	return retval;
}

static int w600_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((offset % W600_FLASH_PAGESIZE) != 0) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required %d-byte alignment",
			offset, W600_FLASH_PAGESIZE);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if ((count % W600_FLASH_PAGESIZE) != 0) {
		LOG_WARNING("count 0x%" PRIx32 " breaks required %d-byte alignment",
			offset, W600_FLASH_PAGESIZE);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	while (count > 0) {
		retval = target_write_buffer(target, QFLASH_BUFFER, W600_FLASH_PAGESIZE, buffer);
		if (retval != ERROR_OK)
			break;

		retval = w600_start(bank, QFLASH_CMD_PP, QFLASH_ADDR(offset),
				W600_FLASH_PAGESIZE);
		if (retval != ERROR_OK)
			break;

		count -= W600_FLASH_PAGESIZE;
		offset += W600_FLASH_PAGESIZE;
		buffer += W600_FLASH_PAGESIZE;
	}

	return retval;
}

static int w600_get_flash_id(struct flash_bank *bank, uint32_t *flash_id)
{
	struct target *target = bank->target;

	int retval = w600_start(bank, QFLASH_CMD_RDID, 0, 4);
	if (retval != ERROR_OK)
		return retval;

	return target_read_u32(target, QFLASH_BUFFER, flash_id);
}

static int w600_probe(struct flash_bank *bank)
{
	struct w600_flash_bank *w600_info = bank->driver_priv;
	uint32_t flash_size;
	uint32_t flash_id;
	size_t i;

	w600_info->probed = false;

	/* read stm32 device id register */
	int retval = w600_get_flash_id(bank, &flash_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("flash_id id = 0x%08" PRIx32 "", flash_id);
	w600_info->id = flash_id;
	w600_info->param = NULL;
	for (i = 0; i < ARRAY_SIZE(w600_param); i++) {
		if (w600_param[i].id == (flash_id & 0xFF)) {
			w600_info->param = &w600_param[i];
			break;
		}
	}
	if (!w600_info->param) {
		LOG_ERROR("flash_id not supported for w600");
		return ERROR_FAIL;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (w600_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size = w600_info->user_bank_size;
	} else {
		flash_size = ((flash_id & 0xFFFFFF) >> 16) & 0xFF;
		if ((flash_size != 0x14) && (flash_size != 0x13)) {
			LOG_ERROR("w600 flash size failed, probe inaccurate");
			return ERROR_FAIL;
		}

		flash_size = 1 << flash_size;
	}

	LOG_INFO("flash size = %" PRIu32 "kbytes", flash_size / 1024);

	/* calculate numbers of pages */
	size_t num_pages = flash_size / W600_FLASH_SECSIZE;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	free(bank->sectors);
	bank->sectors = NULL;

	bank->base = W600_FLASH_BASE;
	bank->size = num_pages * W600_FLASH_SECSIZE;
	bank->num_sectors = num_pages;
	bank->write_start_alignment = W600_FLASH_PAGESIZE;
	bank->write_end_alignment = W600_FLASH_PAGESIZE;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i * W600_FLASH_SECSIZE;
		bank->sectors[i].size = W600_FLASH_SECSIZE;
		bank->sectors[i].is_erased = -1;
		/* offset 0 to W600_FLASH_PROTECT_SIZE should be protected */
		bank->sectors[i].is_protected = (i < W600_FLASH_PROTECT_SIZE / W600_FLASH_SECSIZE);
	}

	w600_info->probed = true;

	return ERROR_OK;
}

static int w600_auto_probe(struct flash_bank *bank)
{
	struct w600_flash_bank *w600_info = bank->driver_priv;
	if (w600_info->probed)
		return ERROR_OK;
	return w600_probe(bank);
}

static int get_w600_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	uint32_t flash_id;

	/* read w600 device id register */
	int retval = w600_get_flash_id(bank, &flash_id);
	if (retval != ERROR_OK)
		return retval;

	command_print_sameline(cmd, "w600 : 0x%08" PRIx32 "", flash_id);
	return ERROR_OK;
}

const struct flash_driver w600_flash = {
	.name = "w600",
	.flash_bank_command = w600_flash_bank_command,
	.erase = w600_erase,
	.write = w600_write,
	.read = default_flash_read,
	.probe = w600_probe,
	.auto_probe = w600_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = get_w600_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
