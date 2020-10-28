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
#include "bluenrg-x.h"

#define BLUENRG2_JTAG_REG	 (flash_priv_data_2.jtag_idcode_reg)
#define BLUENRGLP_JTAG_REG	 (flash_priv_data_lp.jtag_idcode_reg)

#define DIE_ID_REG(bluenrgx_info)           (bluenrgx_info->flash_ptr->die_id_reg)
#define JTAG_IDCODE_REG(bluenrgx_info)      (bluenrgx_info->flash_ptr->jtag_idcode_reg)
#define FLASH_PAGE_SIZE(bluenrgx_info)      (bluenrgx_info->flash_ptr->flash_page_size)

struct flash_ctrl_priv_data {
	uint32_t die_id_reg;
	uint32_t jtag_idcode_reg;
	uint32_t flash_base;
	uint32_t flash_regs_base;
	uint32_t flash_page_size;
	uint32_t jtag_idcode;
	char *part_name;
};

static const struct flash_ctrl_priv_data flash_priv_data_1 = {
	.die_id_reg = 0x4090001C,
	.jtag_idcode_reg = 0x40900028,
	.flash_base = 0x10040000,
	.flash_regs_base = 0x40100000,
	.flash_page_size = 2048,
	.jtag_idcode = 0x00000000,
	.part_name = "BLUENRG-1",
};

static const struct flash_ctrl_priv_data flash_priv_data_2 = {
	.die_id_reg = 0x4090001C,
	.jtag_idcode_reg = 0x40900028,
	.flash_base = 0x10040000,
	.flash_regs_base = 0x40100000,
	.flash_page_size = 2048,
	.jtag_idcode = 0x0200A041,
	.part_name = "BLUENRG-2",
};

static const struct flash_ctrl_priv_data flash_priv_data_lp = {
	.die_id_reg = 0x40000000,
	.jtag_idcode_reg = 0x40000004,
	.flash_base = 0x10040000,
	.flash_regs_base = 0x40001000,
	.flash_page_size = 2048,
	.jtag_idcode = 0x0201E041,
	.part_name = "BLUENRG-LP",
};

struct bluenrgx_flash_bank {
	bool probed;
	uint32_t die_id;
	const struct flash_ctrl_priv_data *flash_ptr;
};

static const struct flash_ctrl_priv_data *flash_ctrl[] = {
	&flash_priv_data_1,
	&flash_priv_data_2,
	&flash_priv_data_lp
};

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

	bank->write_start_alignment = 16;
	bank->write_end_alignment = 16;

	bank->driver_priv = bluenrgx_info;

	bluenrgx_info->probed = false;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static inline uint32_t bluenrgx_get_flash_reg(struct flash_bank *bank, uint32_t reg_offset)
{
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;
	return bluenrgx_info->flash_ptr->flash_regs_base + reg_offset;
}

static inline int bluenrgx_read_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t *value)
{
	return target_read_u32(bank->target, bluenrgx_get_flash_reg(bank, reg_offset), value);
}

static inline int bluenrgx_write_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t value)
{
	return target_write_u32(bank->target, bluenrgx_get_flash_reg(bank, reg_offset), value);
}

static int bluenrgx_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval = ERROR_OK;
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;
	unsigned int num_sectors = (last - first + 1);
	const bool mass_erase = (num_sectors == bank->num_sectors);
	struct target *target = bank->target;
	uint32_t address, command;

	/* check preconditions */
	if (!bluenrgx_info->probed)
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
		if (bluenrgx_write_flash_reg(bank, FLASH_REG_IRQRAW, 0x3f) != ERROR_OK) {
			LOG_ERROR("Register write failed");
			return ERROR_FAIL;
		}

		if (bluenrgx_write_flash_reg(bank, FLASH_REG_ADDRESS,
								(address - bank->base) >> 2) != ERROR_OK) {
			LOG_ERROR("Register write failed");
			return ERROR_FAIL;
		}

		if (bluenrgx_write_flash_reg(bank, FLASH_REG_COMMAND, command) != ERROR_OK) {
			LOG_ERROR("Register write failed");
			return ERROR_FAIL;
		}

		for (unsigned int i = 0; i < 100; i++) {
			uint32_t value;
			if (bluenrgx_read_flash_reg(bank, FLASH_REG_IRQRAW, &value)) {
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
		for (unsigned int i = first; i <= last; i++) {
			address = bank->base+i*FLASH_PAGE_SIZE(bluenrgx_info);
			LOG_DEBUG("address = %08" PRIx32 ", index = %u", address, i);

			if (bluenrgx_write_flash_reg(bank, FLASH_REG_IRQRAW, 0x3f) != ERROR_OK) {
				LOG_ERROR("Register write failed");
				return ERROR_FAIL;
			}

			if (bluenrgx_write_flash_reg(bank, FLASH_REG_ADDRESS,
									(address - bank->base) >> 2) != ERROR_OK) {
				LOG_ERROR("Register write failed");
				return ERROR_FAIL;
			}

			if (bluenrgx_write_flash_reg(bank, FLASH_REG_COMMAND, command) != ERROR_OK) {
				LOG_ERROR("Failed");
				return ERROR_FAIL;
			}

			for (unsigned int j = 0; j < 100; j++) {
				uint32_t value;
				if (bluenrgx_read_flash_reg(bank, FLASH_REG_IRQRAW, &value)) {
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
	struct mem_param mem_params[1];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* See contrib/loaders/flash/bluenrg-x/bluenrg-x_write.c for source and
	 * hints how to generate the data!
	 */
	static const uint8_t bluenrgx_flash_write_code[] = {
#include "../../../contrib/loaders/flash/bluenrg-x/bluenrg-x_write.inc"
	};

	/* check preconditions */
	if (!bluenrgx_info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((offset + count) > bank->size) {
		LOG_ERROR("Requested write past beyond of flash size: (offset+count) = %" PRIu32 ", size=%" PRIu32,
			  (offset + count),
			  bank->size);
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (target_alloc_working_area(target, sizeof(bluenrgx_flash_write_code),
					  &write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
					 sizeof(bluenrgx_flash_write_code),
					 bluenrgx_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	if (target_alloc_working_area(target, buffer_size, &source)) {
		LOG_WARNING("no large enough working area available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Stack pointer area */
	if (target_alloc_working_area(target, 128,
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
	/* Put the parameter at the first available stack location */
	init_mem_param(&mem_params[0], write_algorithm_sp->address + 80, 32, PARAM_OUT);

	/* FIFO start address (first two words used for write and read pointers) */
	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	/* FIFO end address (first two words used for write and read pointers) */
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	/* Flash memory address */
	buf_set_u32(reg_params[2].value, 0, 32, address);
	/* Number of bytes */
	buf_set_u32(reg_params[3].value, 0, 32, count);
	/* Stack pointer for program working area */
	buf_set_u32(reg_params[4].value, 0, 32, write_algorithm_sp->address);
	/* Flash register base address */
	buf_set_u32(mem_params[0].value, 0, 32, bluenrgx_info->flash_ptr->flash_regs_base);

	LOG_DEBUG("source->address = " TARGET_ADDR_FMT, source->address);
	LOG_DEBUG("source->address+ source->size = " TARGET_ADDR_FMT, source->address+source->size);
	LOG_DEBUG("write_algorithm_sp->address = " TARGET_ADDR_FMT, write_algorithm_sp->address);
	LOG_DEBUG("address = %08" PRIx32, address);
	LOG_DEBUG("count = %08" PRIx32, count);

	retval = target_run_flash_async_algorithm(target,
						  buffer,
						  count/16,
						  16, /* Block size: we write in block of 16 bytes to enjoy burstwrite speed */
						  1,
						  mem_params,
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
	destroy_mem_param(&mem_params[0]);

	return retval;
}

static int bluenrgx_probe(struct flash_bank *bank)
{
	struct bluenrgx_flash_bank *bluenrgx_info = bank->driver_priv;
	uint32_t idcode, size_info, die_id;
	int retval = target_read_u32(bank->target, BLUENRGLP_JTAG_REG, &idcode);

	if (retval != ERROR_OK)
		return retval;

	if (idcode != flash_priv_data_lp.jtag_idcode) {
		retval = target_read_u32(bank->target, BLUENRG2_JTAG_REG, &idcode);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Default device is BlueNRG-1 */
	bluenrgx_info->flash_ptr = &flash_priv_data_1;
	bank->base = flash_priv_data_1.flash_base;

	for (size_t i = 0; i < ARRAY_SIZE(flash_ctrl); i++) {
		if (idcode == (*flash_ctrl[i]).jtag_idcode) {
			bluenrgx_info->flash_ptr = flash_ctrl[i];
			bank->base = (*flash_ctrl[i]).flash_base;
			break;
		}
	}
	retval = bluenrgx_read_flash_reg(bank, FLASH_SIZE_REG, &size_info);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(bank->target, DIE_ID_REG(bluenrgx_info), &die_id);
	if (retval != ERROR_OK)
		return retval;

	bank->size = (size_info + 1) * FLASH_WORD_LEN;
	bank->num_sectors = bank->size/FLASH_PAGE_SIZE(bluenrgx_info);
	bank->sectors = realloc(bank->sectors, sizeof(struct flash_sector) * bank->num_sectors);

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * FLASH_PAGE_SIZE(bluenrgx_info);
		bank->sectors[i].size = FLASH_PAGE_SIZE(bluenrgx_info);
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	bluenrgx_info->probed = true;
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
	.protect = NULL,
	.write = bluenrgx_write,
	.read = default_flash_read,
	.probe = bluenrgx_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.auto_probe = bluenrgx_auto_probe,
	.info = bluenrgx_get_info,
};
