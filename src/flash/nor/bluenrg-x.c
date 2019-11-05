/***************************************************************************
 *   Copyright (C) 2017 by Michele Sardo                                   *
 *   msmttchr@gmail.com                                                    *
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

#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>
#include "imp.h"

#define BLUENRG2_IDCODE      (0x0200A041)
#define BLUENRGLP_IDCODE     (0x0201E041)
#define BLUENRG2_JTAG_REG	 (flash_priv_data_2.jtag_idcode_reg)
#define BLUENRGLP_JTAG_REG	 (flash_priv_data_lp.jtag_idcode_reg)

#define FLASH_SIZE_REG(bluenrgx_info)       (bluenrgx_info->flash_ptr->flash_size_reg)
#define DIE_ID_REG(bluenrgx_info)           (bluenrgx_info->flash_ptr->die_id_reg)
#define JTAG_IDCODE_REG(bluenrgx_info)      (bluenrgx_info->flash_ptr->jtag_idcode_reg)
#define FLASH_BASE(bluenrgx_info)           (bluenrgx_info->flash_ptr->flash_base)
#define FLASH_PAGE_SIZE(bluenrgx_info)      (bluenrgx_info->flash_ptr->flash_page_size)
#define FLASH_REG_COMMAND(bluenrgx_info)    (bluenrgx_info->flash_ptr->flash_reg_command)
#define FLASH_REG_IRQRAW(bluenrgx_info)     (bluenrgx_info->flash_ptr->flash_reg_irqraw)
#define FLASH_REG_ADDRESS(bluenrgx_info)    (bluenrgx_info->flash_ptr->flash_reg_address)
#define FLASH_REG_DATA(bluenrgx_info)       (bluenrgx_info->flash_ptr->flash_reg_data)
#define FLASH_CMD_ERASE_PAGE 0x11
#define FLASH_CMD_MASSERASE  0x22
#define FLASH_CMD_WRITE      0x33
#define FLASH_CMD_BURSTWRITE 0xCC
#define FLASH_INT_CMDDONE    0x01
#define FLASH_WORD_LEN       4

/* See contrib/loaders/flash/bluenrg-x/bluenrg-x_write.c for source and
 * hints how to generate the data!
 */
static const uint8_t bluenrgx_flash_write_code_2[] = {
#include "../../../contrib/loaders/flash/bluenrg-x/bluenrg-2_write.inc"
			};

static const uint8_t bluenrgx_flash_write_code_lp[] = {
#include "../../../contrib/loaders/flash/bluenrg-x/bluenrg-lp_write.inc"
			};

struct flash_ctrl_priv_data {
	uint32_t flash_size_reg;
	uint32_t die_id_reg;
	uint32_t jtag_idcode_reg;
	uint32_t flash_base;
	uint32_t flash_page_size;
	uint32_t flash_reg_command;
	uint32_t flash_reg_irqraw;
	uint32_t flash_reg_address;
	uint32_t flash_reg_data;
	uint32_t jtag_idcode;
	char *part_name;
	const uint8_t *flash_write_code;
	uint32_t flash_write_code_size;
};

const struct flash_ctrl_priv_data flash_priv_data_1 = {
	.flash_size_reg = 0x40100014,
	.die_id_reg = 0x4090001C,
	.jtag_idcode_reg = 0x40900028,
	.flash_base = 0x10040000,
	.flash_page_size = 2048,
	.flash_reg_command = 0x40100000,
	.flash_reg_irqraw = 0x40100010,
	.flash_reg_address = 0x40100018,
	.flash_reg_data = 0x40100040,
	.jtag_idcode = 0x00000000,
	.part_name = "BLUENRG-1",
	.flash_write_code = bluenrgx_flash_write_code_2,
	.flash_write_code_size = sizeof(bluenrgx_flash_write_code_2),
};

const struct flash_ctrl_priv_data flash_priv_data_2 = {
	.flash_size_reg = 0x40100014,
	.die_id_reg = 0x4090001C,
	.jtag_idcode_reg = 0x40900028,
	.flash_base = 0x10040000,
	.flash_page_size = 2048,
	.flash_reg_command = 0x40100000,
	.flash_reg_irqraw = 0x40100010,
	.flash_reg_address = 0x40100018,
	.flash_reg_data = 0x40100040,
	.jtag_idcode = BLUENRG2_IDCODE,
	.part_name = "BLUENRG-2",
	.flash_write_code = bluenrgx_flash_write_code_2,
	.flash_write_code_size = sizeof(bluenrgx_flash_write_code_2),
};

const struct flash_ctrl_priv_data flash_priv_data_lp = {
	.flash_size_reg = 0x40001014,
	.die_id_reg = 0x40000000,
	.jtag_idcode_reg = 0x40000004,
	.flash_base = 0x10040000,
	.flash_page_size = 2048,
	.flash_reg_command = 0x40001000,
	.flash_reg_irqraw = 0x40001010,
	.flash_reg_address = 0x40001018,
	.flash_reg_data = 0x40001040,
	.jtag_idcode = BLUENRGLP_IDCODE,
	.part_name = "BLUENRG-LP",
	.flash_write_code = bluenrgx_flash_write_code_lp,
	.flash_write_code_size = sizeof(bluenrgx_flash_write_code_lp),
};

struct bluenrgx_flash_bank {
	int probed;
	uint32_t die_id;
	const struct flash_ctrl_priv_data *flash_ptr;
	const uint8_t *flash_write_code;
	uint32_t flash_write_code_size;
};

const struct flash_ctrl_priv_data *flash_ctrl[] = {&flash_priv_data_1, &flash_priv_data_2, &flash_priv_data_lp};

static int bluenrgx_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

/* flash_bank bluenrg-x 0 0 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(bluenrgx_flash_bank_command)
{
	struct bluenrgx_flash_bank *bluenrgx_info;
	/* Create the bank structure */
	bluenrgx_info = calloc(1, sizeof(*bluenrgx_info));

	/* Check allocation */
	if (bluenrgx_info == NULL) {
		LOG_ERROR("failed to allocate bank structure");
		return ERROR_FAIL;
	}

	bank->driver_priv = bluenrgx_info;

	bluenrgx_info->probed = 0;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static int bluenrgx_erase(struct flash_bank *bank, int first, int last)
{
	int retval = ERROR_OK;
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;
	int num_sectors = (last - first + 1);
	int mass_erase = (num_sectors == bank->num_sectors);
	struct target *target = bank->target;
	uint32_t address, command;

	/* check preconditions */
	if (bluenrgx_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	/* Disable blue module */
	if (target_write_u32(target, 0x200000c0, 0) != ERROR_OK) {
		LOG_ERROR("Blue disable failed");
		return ERROR_FAIL;
	}

	if (mass_erase) {
		command = FLASH_CMD_MASSERASE;
		address = bank->base;
		if (target_write_u32(target, FLASH_REG_IRQRAW(bluenrgx_info), 0x3f) != ERROR_OK) {
			LOG_ERROR("Register write failed");
			return ERROR_FAIL;
		}

		if (target_write_u32(target, FLASH_REG_ADDRESS(bluenrgx_info),
								(address - FLASH_BASE(bluenrgx_info)) >> 2) != ERROR_OK) {
			LOG_ERROR("Register write failed");
			return ERROR_FAIL;
		}

		if (target_write_u32(target, FLASH_REG_COMMAND(bluenrgx_info), command) != ERROR_OK) {
			LOG_ERROR("Register write failed");
			return ERROR_FAIL;
		}

		for (int i = 0; i < 100; i++) {
			uint32_t value;
			if (target_read_u32(target, FLASH_REG_IRQRAW(bluenrgx_info), &value)) {
				LOG_ERROR("Register write failed");
				return ERROR_FAIL;
			}
			if (value & FLASH_INT_CMDDONE)
				break;
			if (i == 99) {
				LOG_ERROR("Mass erase command failed (timeout)");
				retval = ERROR_FAIL;
			}
		}

	} else {
		command = FLASH_CMD_ERASE_PAGE;
		for (int i = first; i <= last; i++) {
			address = bank->base+i*FLASH_PAGE_SIZE(bluenrgx_info);
			LOG_DEBUG("address = %08x, index = %d", address, i);

			if (target_write_u32(target, FLASH_REG_IRQRAW(bluenrgx_info), 0x3f) != ERROR_OK) {
				LOG_ERROR("Register write failed");
				return ERROR_FAIL;
			}

			if (target_write_u32(target, FLASH_REG_ADDRESS(bluenrgx_info),
									(address - FLASH_BASE(bluenrgx_info)) >> 2) != ERROR_OK) {
				LOG_ERROR("Register write failed");
				return ERROR_FAIL;
			}

			if (target_write_u32(target, FLASH_REG_COMMAND(bluenrgx_info), command) != ERROR_OK) {
				LOG_ERROR("Failed");
				return ERROR_FAIL;
			}

			for (int j = 0; j < 100; j++) {
				uint32_t value;
				if (target_read_u32(target, FLASH_REG_IRQRAW(bluenrgx_info), &value)) {
					LOG_ERROR("Register write failed");
					return ERROR_FAIL;
				}
				if (value & FLASH_INT_CMDDONE)
					break;
				if (j == 99) {
					LOG_ERROR("Erase command failed (timeout)");
					retval = ERROR_FAIL;
				}
			}
		}
	}

	return retval;

}

static int bluenrgx_protect(struct flash_bank *bank, int set, int first, int last)
{
	/* Protection is only handled in software: no hardware write protection
	   available in BlueNRG-x devices */
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int bluenrgx_write_word(struct flash_bank *bank, uint32_t address_base, uint8_t *values, uint32_t count)
{
	int retval = ERROR_OK;
	struct target *target =  bank->target;
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;

	retval = target_write_u32(target, FLASH_REG_IRQRAW(bluenrgx_info), 0x3f);
	if (retval != ERROR_OK) {
		LOG_ERROR("Register write failed, error code: %d", retval);
		return retval;
	}

	for (uint32_t i = 0; i < count; i++) {
		uint32_t address = address_base + i * FLASH_WORD_LEN;

		retval = target_write_u32(target, FLASH_REG_ADDRESS(bluenrgx_info),
									(address - FLASH_BASE(bluenrgx_info)) >> 2);
		if (retval != ERROR_OK) {
			LOG_ERROR("Register write failed, error code: %d", retval);
			return retval;
		}

		retval = target_write_buffer(target, FLASH_REG_DATA(bluenrgx_info),
										FLASH_WORD_LEN, values + i * FLASH_WORD_LEN);
		if (retval != ERROR_OK) {
			LOG_ERROR("Register write failed, error code: %d", retval);
			return retval;
		}

		retval = target_write_u32(target, FLASH_REG_COMMAND(bluenrgx_info), FLASH_CMD_WRITE);
		if (retval != ERROR_OK) {
			LOG_ERROR("Register write failed, error code: %d", retval);
			return retval;
		}

		for (int j = 0; j < 100; j++) {
			uint32_t reg_value;
			retval = target_read_u32(target, FLASH_REG_IRQRAW(bluenrgx_info), &reg_value);

			if (retval != ERROR_OK) {
				LOG_ERROR("Register read failed, error code: %d", retval);
				return retval;
			}

			if (reg_value & FLASH_INT_CMDDONE)
				break;

			if (j == 99) {
				LOG_ERROR("Write command failed (timeout)");
				return ERROR_FAIL;
			}
		}
	}
	return retval;
}

static int bluenrgx_write_bytes(struct flash_bank *bank, uint32_t address_base, uint8_t *buffer, uint32_t count)
{
	int retval = ERROR_OK;
	struct target *target =  bank->target;
	uint8_t *new_buffer = NULL;
	uint32_t pre_bytes = 0, post_bytes = 0, pre_word, post_word, pre_address, post_address;

	if (count == 0) {
		/* Just return if there are no bytes to write */
		return retval;
	}

	if (address_base & 3) {
		pre_bytes = address_base & 3;
		pre_address = address_base - pre_bytes;
	}

	if ((count + pre_bytes) & 3) {
		post_bytes = ((count + pre_bytes + 3) & ~3) - (count + pre_bytes);
		post_address = (address_base + count) & ~3;
	}

	if (pre_bytes || post_bytes) {
		uint32_t old_count = count;

		count = old_count + pre_bytes + post_bytes;

		new_buffer = malloc(count);

		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory "
				  "for padding buffer");
			return ERROR_FAIL;
		}

		LOG_INFO("Requested number of bytes to write and/or address not word aligned (%" PRIu32 "), extending to %"
			 PRIu32 " ", old_count, count);

		if (pre_bytes) {
			if (target_read_u32(target, pre_address, &pre_word)) {
				LOG_ERROR("Memory read failed");
				free(new_buffer);
				return ERROR_FAIL;
			}

		}

		if (post_bytes) {
			if (target_read_u32(target, post_address, &post_word)) {
				LOG_ERROR("Memory read failed");
				free(new_buffer);
				return ERROR_FAIL;
			}

		}

		memcpy(new_buffer, &pre_word, pre_bytes);
		memcpy((new_buffer+((pre_bytes+old_count) & ~3)), &post_word, 4);
		memcpy(new_buffer+pre_bytes, buffer, old_count);
		buffer = new_buffer;
	}

	retval = bluenrgx_write_word(bank, address_base - pre_bytes, buffer, count/4);

	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int bluenrgx_write(struct flash_bank *bank, const uint8_t *buffer,
			  uint32_t offset, uint32_t count)
{
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384 + 8;
	struct working_area *write_algorithm;
	struct working_area *write_algorithm_sp;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	uint32_t pre_size = 0, fast_size = 0, post_size = 0;
	uint32_t pre_offset = 0, fast_offset = 0, post_offset = 0;

	/* check preconditions */
	if (bluenrgx_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((offset + count) > bank->size) {
		LOG_ERROR("Requested write past beyond of flash size: (offset+count) = %d, size=%d",
			  (offset + count),
			  bank->size);
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* We are good here and we need to compute pre_size, fast_size, post_size */
	pre_size  = MIN(count, ((offset+0xF) & ~0xF) - offset);
	pre_offset = offset;
	fast_size = 16*((count - pre_size) / 16);
	fast_offset = offset + pre_size;
	post_size = (count-pre_size-fast_size) % 16;
	post_offset = fast_offset + fast_size;

	LOG_DEBUG("pre_size = %08x, pre_offset=%08x", pre_size, pre_offset);
	LOG_DEBUG("fast_size = %08x, fast_offset=%08x", fast_size, fast_offset);
	LOG_DEBUG("post_size = %08x, post_offset=%08x", post_size, post_offset);

	/* Program initial chunk not 16 bytes aligned */
	retval = bluenrgx_write_bytes(bank, bank->base+pre_offset, (uint8_t *) buffer, pre_size);
	if (retval) {
		LOG_ERROR("bluenrgx_write_bytes failed %d", retval);
		return ERROR_FAIL;
	}

	/* Program chunk 16 bytes aligned in fast mode */
	if (fast_size) {

		if (target_alloc_working_area(target, bluenrgx_info->flash_write_code_size,
						  &write_algorithm) != ERROR_OK) {
			LOG_WARNING("no working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		retval = target_write_buffer(target, write_algorithm->address,
						 bluenrgx_info->flash_write_code_size,
						 bluenrgx_info->flash_write_code);
		if (retval != ERROR_OK)
			return retval;

		/* memory buffer */
		if (target_alloc_working_area(target, buffer_size, &source)) {
			LOG_WARNING("no large enough working area available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		/* Stack pointer area */
		if (target_alloc_working_area(target, 64,
						  &write_algorithm_sp) != ERROR_OK) {
			LOG_DEBUG("no working area for write code stack pointer");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
		armv7m_info.core_mode = ARM_MODE_THREAD;

		init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
		init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
		init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
		init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
		init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

		/* FIFO start address (first two words used for write and read pointers) */
		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		/* FIFO end address (first two words used for write and read pointers) */
		buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
		/* Flash memory address */
		buf_set_u32(reg_params[2].value, 0, 32, address+pre_size);
		/* Number of bytes */
		buf_set_u32(reg_params[3].value, 0, 32, fast_size);
		/* Stack pointer for program working area */
		buf_set_u32(reg_params[4].value, 0, 32, write_algorithm_sp->address);

		LOG_DEBUG("source->address = " TARGET_ADDR_FMT, source->address);
		LOG_DEBUG("source->address+ source->size = " TARGET_ADDR_FMT, source->address+source->size);
		LOG_DEBUG("write_algorithm_sp->address = " TARGET_ADDR_FMT, write_algorithm_sp->address);
		LOG_DEBUG("address = %08x", address+pre_size);
		LOG_DEBUG("count = %08x", count);

		retval = target_run_flash_async_algorithm(target,
							  buffer+pre_size,
							  fast_size/16,
							  16, /* Block size: we write in block of 16 bytes to enjoy burstwrite speed */
							  0,
							  NULL,
							  5,
							  reg_params,
							  source->address,
							  source->size,
							  write_algorithm->address,
							  0,
							  &armv7m_info);

		if (retval == ERROR_FLASH_OPERATION_FAILED) {
			LOG_ERROR("error executing bluenrg-x flash write algorithm");

			uint32_t error = buf_get_u32(reg_params[0].value, 0, 32);

			if (error != 0)
				LOG_ERROR("flash write failed = %08" PRIx32, error);
		}
		if (retval == ERROR_OK) {
			uint32_t rp;
			/* Read back rp and check that is valid */
			retval = target_read_u32(target, source->address+4, &rp);
			if (retval == ERROR_OK) {
				if ((rp < source->address+8) || (rp > (source->address + source->size))) {
					LOG_ERROR("flash write failed = %08" PRIx32, rp);
					retval = ERROR_FLASH_OPERATION_FAILED;
				}
			}
		}
		target_free_working_area(target, source);
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, write_algorithm_sp);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		if (retval != ERROR_OK)
			return retval;

	}

	/* Program chunk at end, not addressable by fast burst write algorithm */
	retval = bluenrgx_write_bytes(bank, bank->base+post_offset,
									 (uint8_t *) (buffer+pre_size+fast_size), post_size);
	if (retval) {
		LOG_ERROR("bluenrgx_write_bytes failed %d", retval);
		return ERROR_FAIL;
	}
	return retval;
}

static int bluenrgx_probe(struct flash_bank *bank)
{
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;
	uint32_t idcode, size_info, die_id;
	int i;
	int retval = target_read_u32(bank->target, BLUENRGLP_JTAG_REG, &idcode);

	if (retval != ERROR_OK)
		return retval;

	if (idcode != BLUENRGLP_IDCODE) {
		retval = target_read_u32(bank->target, BLUENRG2_JTAG_REG, &idcode);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Default device is BlueNRG-1 */
	bluenrgx_info->flash_ptr = &flash_priv_data_1;
	bluenrgx_info->flash_write_code = flash_priv_data_1.flash_write_code;
	bluenrgx_info->flash_write_code_size = flash_priv_data_1.flash_write_code_size;

	for (i = 0; i < (int)(sizeof(flash_ctrl)/sizeof(*flash_ctrl)); i++) {
		if (idcode == (*flash_ctrl[i]).jtag_idcode) {
			bluenrgx_info->flash_ptr = flash_ctrl[i];
			bluenrgx_info->flash_write_code = (*flash_ctrl[i]).flash_write_code;
			bluenrgx_info->flash_write_code_size = (*flash_ctrl[i]).flash_write_code_size;
			break;
		}
	}

	retval = target_read_u32(bank->target, FLASH_SIZE_REG(bluenrgx_info), &size_info);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(bank->target, DIE_ID_REG(bluenrgx_info), &die_id);
	if (retval != ERROR_OK)
		return retval;

	bank->size = (size_info + 1) * FLASH_WORD_LEN;
	bank->base = FLASH_BASE(bluenrgx_info);
	bank->num_sectors = bank->size/FLASH_PAGE_SIZE(bluenrgx_info);
	bank->sectors = realloc(bank->sectors, sizeof(struct flash_sector) * bank->num_sectors);

	for (i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * FLASH_PAGE_SIZE(bluenrgx_info);
		bank->sectors[i].size = FLASH_PAGE_SIZE(bluenrgx_info);
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	bluenrgx_info->probed = 1;
	bluenrgx_info->die_id = die_id;

	return ERROR_OK;
}

static int bluenrgx_auto_probe(struct flash_bank *bank)
{
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;

	if (bluenrgx_info->probed)
		return ERROR_OK;

	return bluenrgx_probe(bank);
}

/* This method must return a string displaying information about the bank */
static int bluenrgx_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;
	int mask_number, cut_number;

	if (!bluenrgx_info->probed) {
		int retval = bluenrgx_probe(bank);
		if (retval != ERROR_OK) {
			snprintf(buf, buf_size,
				 "Unable to find bank information.");
			return retval;
		}
	}

	mask_number = (bluenrgx_info->die_id >> 4) & 0xF;
	cut_number = bluenrgx_info->die_id & 0xF;

	snprintf(buf, buf_size,
		 "%s - Rev: %d.%d", bluenrgx_info->flash_ptr->part_name, mask_number, cut_number);
	return ERROR_OK;
}

const struct flash_driver bluenrgx_flash = {
	.name = "bluenrg-x",
	.flash_bank_command = bluenrgx_flash_bank_command,
	.erase = bluenrgx_erase,
	.protect = bluenrgx_protect,
	.write = bluenrgx_write,
	.read = default_flash_read,
	.probe = bluenrgx_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = bluenrgx_protect_check,
	.auto_probe = bluenrgx_auto_probe,
	.info = bluenrgx_get_info,
};
