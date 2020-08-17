/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2013 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
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

#define MD_RST_CLK		0x40020000
#define MD_PER_CLOCK		(MD_RST_CLK + 0x1C)
#define MD_PER_CLOCK_EEPROM	(1 << 3)
#define MD_PER_CLOCK_RST_CLK	(1 << 4)

#define FLASH_REG_BASE	0x40018000
#define FLASH_CMD	(FLASH_REG_BASE + 0x00)
#define FLASH_ADR	(FLASH_REG_BASE + 0x04)
#define FLASH_DI	(FLASH_REG_BASE + 0x08)
#define FLASH_DO	(FLASH_REG_BASE + 0x0C)
#define FLASH_KEY	(FLASH_REG_BASE + 0x10)

#define FLASH_NVSTR	(1 << 13)
#define FLASH_PROG	(1 << 12)
#define FLASH_MAS1	(1 << 11)
#define FLASH_ERASE	(1 << 10)
#define FLASH_IFREN	(1 << 9)
#define FLASH_SE	(1 << 8)
#define FLASH_YE	(1 << 7)
#define FLASH_XE	(1 << 6)
#define FLASH_RD	(1 << 2)
#define FLASH_WR	(1 << 1)
#define FLASH_CON	(1 << 0)
#define FLASH_DELAY_MASK	(7 << 3)

#define KEY		0x8AAA5551

struct mdr_flash_bank {
	bool probed;
	unsigned int mem_type;
	unsigned int page_count;
	unsigned int sec_count;
};

/* flash bank <name> mdr <base> <size> 0 0 <target#> <type> <page_count> <sec_count> */
FLASH_BANK_COMMAND_HANDLER(mdr_flash_bank_command)
{
	struct mdr_flash_bank *mdr_info;

	if (CMD_ARGC < 9)
		return ERROR_COMMAND_SYNTAX_ERROR;

	mdr_info = malloc(sizeof(struct mdr_flash_bank));

	bank->driver_priv = mdr_info;
	mdr_info->probed = false;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[6], mdr_info->mem_type);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[7], mdr_info->page_count);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[8], mdr_info->sec_count);
	return ERROR_OK;
}

static int mdr_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	uint32_t flash_cmd;
	int retval;
	unsigned int i;

	retval = target_read_u32(target, FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < mdr_info->sec_count; i++) {
		retval = target_write_u32(target, FLASH_ADR, i << 2);
		if (retval != ERROR_OK)
			return retval;

		flash_cmd |= FLASH_XE | FLASH_MAS1 | FLASH_ERASE;
		retval = target_write_u32(target, FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			return retval;
		flash_cmd |= FLASH_NVSTR;
		retval = target_write_u32(target, FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			return retval;
		flash_cmd &= ~FLASH_ERASE;
		retval = target_write_u32(target, FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			return retval;
		flash_cmd &= ~(FLASH_XE | FLASH_MAS1 | FLASH_NVSTR);
		retval = target_write_u32(target, FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int mdr_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	int retval, retval2;
	unsigned int j;
	uint32_t flash_cmd, cur_per_clock;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = target_read_u32(target, MD_PER_CLOCK, &cur_per_clock);
	if (retval != ERROR_OK)
		return retval;

	if (!(cur_per_clock & 0x10)) {
		LOG_ERROR("Target needs reset before flash operations");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	retval = target_write_u32(target, MD_PER_CLOCK, cur_per_clock | MD_PER_CLOCK_EEPROM);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FLASH_KEY, KEY);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* Switch on register access */
	flash_cmd = (flash_cmd & FLASH_DELAY_MASK) | FLASH_CON;
	if (mdr_info->mem_type)
		flash_cmd |= FLASH_IFREN;
	retval = target_write_u32(target, FLASH_CMD, flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	if ((first == 0) && (last == (bank->num_sectors - 1)) &&
		!mdr_info->mem_type) {
		retval = mdr_mass_erase(bank);
		goto reset_pg_and_lock;
	}

	unsigned int page_size = bank->size / mdr_info->page_count;
	for (unsigned int i = first; i <= last; i++) {
		for (j = 0; j < mdr_info->sec_count; j++) {
			retval = target_write_u32(target, FLASH_ADR, (i * page_size) | (j << 2));
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			flash_cmd |= FLASH_XE | FLASH_ERASE;
			retval = target_write_u32(target, FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;
			flash_cmd |= FLASH_NVSTR;
			retval = target_write_u32(target, FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;
			flash_cmd &= ~FLASH_ERASE;
			retval = target_write_u32(target, FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;
			flash_cmd &= ~(FLASH_XE | FLASH_NVSTR);
			retval = target_write_u32(target, FLASH_CMD, flash_cmd);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;
		}
		bank->sectors[i].is_erased = 1;
	}

reset_pg_and_lock:
	flash_cmd &= FLASH_DELAY_MASK;
	retval2 = target_write_u32(target, FLASH_CMD, flash_cmd);
	if (retval == ERROR_OK)
		retval = retval2;

	retval2 = target_write_u32(target, FLASH_KEY, 0);
	if (retval == ERROR_OK)
		retval = retval2;

	return retval;
}

static int mdr_write_block(struct flash_bank *bank, const uint8_t *buffer,
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

	/* see contrib/loaders/flash/mdr32fx.S for src */
	static const uint8_t mdr32fx_flash_write_code[] = {
		0x07, 0x68, 0x16, 0x68, 0x00, 0x2e, 0x2e, 0xd0, 0x55, 0x68, 0xb5, 0x42,
		0xf9, 0xd0, 0x2e, 0x68, 0x44, 0x60, 0x86, 0x60, 0x17, 0x4e, 0x37, 0x43,
		0x07, 0x60, 0x05, 0x26, 0x00, 0xf0, 0x25, 0xf8, 0x15, 0x4e, 0x37, 0x43,
		0x07, 0x60, 0x0d, 0x26, 0x00, 0xf0, 0x1f, 0xf8, 0x80, 0x26, 0x37, 0x43,
		0x07, 0x60, 0x3d, 0x26, 0x00, 0xf0, 0x19, 0xf8, 0x80, 0x26, 0xb7, 0x43,
		0x07, 0x60, 0x0f, 0x4e, 0xb7, 0x43, 0x07, 0x60, 0x05, 0x26, 0x00, 0xf0,
		0x10, 0xf8, 0x0d, 0x4e, 0xb7, 0x43, 0x07, 0x60, 0x04, 0x35, 0x04, 0x34,
		0x9d, 0x42, 0x01, 0xd3, 0x15, 0x46, 0x08, 0x35, 0x55, 0x60, 0x01, 0x39,
		0x00, 0x29, 0x00, 0xd0, 0xcd, 0xe7, 0x30, 0x46, 0x00, 0xbe, 0x01, 0x3e,
		0x00, 0x2e, 0xfc, 0xd1, 0x70, 0x47, 0x00, 0x00, 0x40, 0x10, 0x00, 0x00,
		0x00, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(mdr32fx_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(mdr32fx_flash_write_code), mdr32fx_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

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

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (32bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, FLASH_REG_BASE);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, count, 4,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int mdr_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	uint8_t *new_buffer = NULL;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of bytes, the data has to be padded. Duplicate
	 * the buffer and use the normal code path with a single block write since
	 * it's probably cheaper than to special case the last odd write using
	 * discrete accesses. */
	int rem = count % 4;
	if (rem) {
		new_buffer = malloc(count + rem);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write, padding with 0xff");
		buffer = memcpy(new_buffer, buffer, count);
		while (rem--)
			new_buffer[count++] = 0xff;
	}

	uint32_t flash_cmd, cur_per_clock;
	int retval, retval2;

	retval = target_read_u32(target, MD_PER_CLOCK, &cur_per_clock);
	if (retval != ERROR_OK)
		goto free_buffer;

	if (!(cur_per_clock & MD_PER_CLOCK_RST_CLK)) {
		/* Something's very wrong if the RST_CLK module is not clocked */
		LOG_ERROR("Target needs reset before flash operations");
		retval = ERROR_FLASH_OPERATION_FAILED;
		goto free_buffer;
	}

	retval = target_write_u32(target, MD_PER_CLOCK, cur_per_clock | MD_PER_CLOCK_EEPROM);
	if (retval != ERROR_OK)
		goto free_buffer;

	retval = target_write_u32(target, FLASH_KEY, KEY);
	if (retval != ERROR_OK)
		goto free_buffer;

	retval = target_read_u32(target, FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* Switch on register access */
	flash_cmd = (flash_cmd & FLASH_DELAY_MASK) | FLASH_CON;
	if (mdr_info->mem_type)
		flash_cmd |= FLASH_IFREN;
	retval = target_write_u32(target, FLASH_CMD, flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* try using block write */
	retval = mdr_write_block(bank, buffer, offset, count/4);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("Can't use block writes, falling back to single memory accesses");

		unsigned int page_size = bank->size / mdr_info->page_count;
		unsigned int page_mask = page_size - 1;
		while (count > 0) {
			unsigned int i, j;
			unsigned int cur_page = offset & ~page_mask;
			unsigned int bytes_to_write = cur_page + page_size - offset;
			if (count < bytes_to_write)
				bytes_to_write = count;

			/*LOG_INFO("Selecting next page: %08x", cur_page);*/

			for (i = 0; i < mdr_info->sec_count; i++) {
				retval = target_write_u32(target, FLASH_ADR, offset + i*4);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;
				/*LOG_INFO("Selecting page/sector: %08x", offset + i*4);*/

				flash_cmd |= FLASH_XE | FLASH_PROG;
				retval = target_write_u32(target, FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				flash_cmd |= FLASH_NVSTR;
				retval = target_write_u32(target, FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				for (j = 0;
				     (((offset + j + i*4) & ~page_mask) == cur_page) &&
					     (j + i*4 < count);
				     j += mdr_info->sec_count*4) {
					uint32_t value;
					memcpy(&value, buffer + j + i*4, sizeof(uint32_t));
					retval = target_write_u32(target, FLASH_DI, value);
					if (retval != ERROR_OK)
						goto reset_pg_and_lock;
					/*LOG_INFO("Writing to addr %08x", offset + j + i*4);*/
					retval = target_write_u32(target, FLASH_ADR, offset + j + i*4);
					if (retval != ERROR_OK)
						goto reset_pg_and_lock;

					flash_cmd |= FLASH_YE;
					retval = target_write_u32(target, FLASH_CMD, flash_cmd);
					if (retval != ERROR_OK)
						goto reset_pg_and_lock;
					flash_cmd &= ~FLASH_YE;
					retval = target_write_u32(target, FLASH_CMD, flash_cmd);
					if (retval != ERROR_OK)
						goto reset_pg_and_lock;
				}
				flash_cmd &= ~FLASH_NVSTR;
				retval = target_write_u32(target, FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;

				flash_cmd &= ~(FLASH_XE | FLASH_PROG);
				retval = target_write_u32(target, FLASH_CMD, flash_cmd);
				if (retval != ERROR_OK)
					goto reset_pg_and_lock;
			}

			buffer += bytes_to_write;
			offset += bytes_to_write;
			count -= bytes_to_write;
		}
	}

reset_pg_and_lock:
	flash_cmd &= FLASH_DELAY_MASK;
	retval2 = target_write_u32(target, FLASH_CMD, flash_cmd);
	if (retval == ERROR_OK)
		retval = retval2;

	retval2 = target_write_u32(target, FLASH_KEY, 0);
	if (retval == ERROR_OK)
		retval = retval2;

free_buffer:
	free(new_buffer);

	/* read some bytes bytes to flush buffer in flash accelerator.
	 * See errata for 1986VE1T and 1986VE3. Error 0007 */
	if ((retval == ERROR_OK) && (!mdr_info->mem_type)) {
		uint32_t tmp;
		target_checksum_memory(bank->target, bank->base, 64, &tmp);
	}

	return retval;
}

static int mdr_read(struct flash_bank *bank, uint8_t *buffer,
		    uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	int retval, retval2;

	if (!mdr_info->mem_type)
		return default_flash_read(bank, buffer, offset, count);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x3) {
		LOG_ERROR("count 0x%" PRIx32 " breaks required 4-byte alignment", count);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	uint32_t flash_cmd, cur_per_clock;

	retval = target_read_u32(target, MD_PER_CLOCK, &cur_per_clock);
	if (retval != ERROR_OK)
		goto err;

	if (!(cur_per_clock & MD_PER_CLOCK_RST_CLK)) {
		/* Something's very wrong if the RST_CLK module is not clocked */
		LOG_ERROR("Target needs reset before flash operations");
		retval = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	retval = target_write_u32(target, MD_PER_CLOCK, cur_per_clock | MD_PER_CLOCK_EEPROM);
	if (retval != ERROR_OK)
		goto err;

	retval = target_write_u32(target, FLASH_KEY, KEY);
	if (retval != ERROR_OK)
		goto err;

	retval = target_read_u32(target, FLASH_CMD, &flash_cmd);
	if (retval != ERROR_OK)
		goto err_lock;

	/* Switch on register access */
	flash_cmd = (flash_cmd & FLASH_DELAY_MASK) | FLASH_CON | FLASH_IFREN;
	retval = target_write_u32(target, FLASH_CMD, flash_cmd);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	for (uint32_t i = 0; i < count; i += 4) {
		retval = target_write_u32(target, FLASH_ADR, offset + i);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		retval = target_write_u32(target, FLASH_CMD, flash_cmd |
					  FLASH_XE | FLASH_YE | FLASH_SE);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		uint32_t buf;
		retval = target_read_u32(target, FLASH_DO, &buf);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		buf_set_u32(buffer, i * 8, 32, buf);

		retval = target_write_u32(target, FLASH_CMD, flash_cmd);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

	}

reset_pg_and_lock:
	flash_cmd &= FLASH_DELAY_MASK;
	retval2 = target_write_u32(target, FLASH_CMD, flash_cmd);
	if (retval == ERROR_OK)
		retval = retval2;

err_lock:
	retval2 = target_write_u32(target, FLASH_KEY, 0);
	if (retval == ERROR_OK)
		retval = retval2;

err:
	return retval;
}

static int mdr_probe(struct flash_bank *bank)
{
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	unsigned int page_count, page_size, i;

	page_count = mdr_info->page_count;
	page_size = bank->size / page_count;

	free(bank->sectors);

	bank->num_sectors = page_count;
	bank->sectors = malloc(sizeof(struct flash_sector) * page_count);

	for (i = 0; i < page_count; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	mdr_info->probed = true;

	return ERROR_OK;
}

static int mdr_auto_probe(struct flash_bank *bank)
{
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	if (mdr_info->probed)
		return ERROR_OK;
	return mdr_probe(bank);
}

static int get_mdr_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct mdr_flash_bank *mdr_info = bank->driver_priv;
	snprintf(buf, buf_size, "MDR32Fx - %s",
		 mdr_info->mem_type ? "info memory" : "main memory");

	return ERROR_OK;
}

const struct flash_driver mdr_flash = {
	.name = "mdr",
	.usage = "flash bank <name> mdr <base> <size> 0 0 <target#> <type> <page_count> <sec_count>\n"
	"<type>: 0 for main memory, 1 for info memory",
	.flash_bank_command = mdr_flash_bank_command,
	.erase = mdr_erase,
	.write = mdr_write,
	.read = mdr_read,
	.probe = mdr_probe,
	.auto_probe = mdr_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = get_mdr_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
