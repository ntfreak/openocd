/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
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

/* stm32x register locations */

#define FLASH_REG_BASE 0x40022000

#define STM32_FLASH_ACR     0x00
#define STM32_FLASH_KEYR    0x04
#define STM32_FLASH_OPTKEYR 0x08
#define STM32_FLASH_SR      0x0C
#define STM32_FLASH_CR      0x10
#define STM32_FLASH_AR      0x14
#define STM32_FLASH_OBR     0x1C
#define STM32_FLASH_WRPR    0x20

#define STM32G0_FLASH_KEYR      0x08
#define STM32G0_FLASH_OPTKEYR   0x0C
#define STM32G0_FLASH_SR        0x10
#define STM32G0_FLASH_CR        0x14
#define STM32G0_FLASH_WRP1AR    0x2C
#define STM32G0_FLASH_WRP1BR    0x30

/* option byte location */

#define STM32_OB_USER_RDP		0x1FFF7800
#define STM32_OB_PCROP1A_STRT	0x1FFF7808
#define STM32_OB_PCROP1A_END	0x1FFF7810
#define STM32_OB_WRP1A			0x1FFF7818
#define STM32_OB_WRP1B			0x1FFF7820
#define STM32_OB_PCROP1B_STRT	0x1FFF7828
#define STM32_OB_PCROP1B_END	0x1FFF7830
#define STM32_OB_BOOT_SEC		0x1FFF7870

/* FLASH_CR register bits */

#define FLASH_PG			(1 << 0)
#define FLASH_PER			(1 << 1)
#define FLASH_MER			(1 << 2)
#define FLASH_OPTPG			(1 << 4)
#define FLASH_OPTER			(1 << 5)
#define FLASH_STRT			(1 << 6)
#define FLASH_LOCK			(1 << 7)
#define FLASH_OPTWRE		(1 << 9)
#define FLASH_OBL_LAUNCH	(1 << 13)	/* except stm32f1x series */

#define STM32G0_FLASH_PG        (1 << 0)
#define STM32G0_FLASH_PER       (1 << 1)
#define STM32G0_FLASH_MER       (1 << 2)
#define STM32G0_FLASH_STRT      (1 << 16)
#define STM32G0_FLASH_OPTSTRT   (1 << 17)
#define STM32G0_FLASH_OBL_LAUNCH (1 << 27)
#define STM32G0_FLASH_LOCK      (1 << 31)

/* FLASH_SR register bits */

#define FLASH_BSY		(1 << 0)
#define FLASH_PGERR		(1 << 2)
#define FLASH_WRPRTERR	(1 << 4)
#define FLASH_EOP		(1 << 5)

#define STM32G0_FLASH_EOP       (1 << 0)    /* end of operation */
#define STM32G0_FLASH_OPERR     (1 << 1)    /* operation error */
#define STM32G0_FLASH_PROGERR   (1 << 3)    /* programming error */
#define STM32G0_FLASH_WRPERR    (1 << 4)    /* write protection error */
#define STM32G0_FLASH_BSY1      (1 << 16)   /* busy */

/* STM32_FLASH_OBR bit definitions (reading) */

#define OPT_ERROR		0
#define OPT_READOUT		1
#define OPT_RDWDGSW		2
#define OPT_RDRSTSTOP	3
#define OPT_RDRSTSTDBY	4

/* register unlock keys */

#define KEY1			0x45670123
#define KEY2			0xCDEF89AB

#define OPTKEY1         0x08192A3B
#define OPTKEY2         0x4C5D6E7F

/* timeout values */

#define FLASH_WRITE_TIMEOUT 10
#define FLASH_ERASE_TIMEOUT 100

struct stm32g0x_options {
	union {
		struct {
			unsigned rdp:8;
			unsigned bor_en:1;
			unsigned borf_lev:2;
			unsigned borr_lev:2;
			unsigned rst_stop:1;
			unsigned rst_stdby:1;
			unsigned rst_shdw:1;
			unsigned iwdg_sw:1;
			unsigned iwdg_stop:1;
			unsigned iwdg_stdby:1;
			unsigned wwdg_sw:1;
			unsigned unused1:2;
			unsigned ram_parity_check:1;
			unsigned unused2:1;
			unsigned boot_sel:1;
			unsigned boot1:1;
			unsigned boot0:1;
			unsigned nrst_mode:2;
			unsigned irhen:1;
		};
		uint32_t user_rdp;
	};
	uint32_t pcrop1a;
	uint32_t pcrop1b;
	uint32_t wrp1a;
	uint32_t wrp1b;
	uint32_t security;
};

struct stm32g0x_flash_bank {
	struct stm32g0x_options option_bytes;
	int probed;

	uint32_t register_base;
	uint8_t default_rdp;
	uint32_t user_bank_size;
};

static int stm32x_mass_erase(struct flash_bank *bank);
static int stm32x_get_device_id(struct flash_bank *bank, uint32_t *device_id);
static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t count);

/* flash bank stm32x <base> <size> 0 0 <target#>
*/
FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32g0x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stm32x_info = malloc(sizeof(struct stm32g0x_flash_bank));

	bank->driver_priv = stm32x_info;
	stm32x_info->probed = 0;
	stm32x_info->register_base = FLASH_REG_BASE;
	stm32x_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline int stm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct stm32g0x_flash_bank *stm32x_info = bank->driver_priv;
	return reg + stm32x_info->register_base;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_SR), status);
}

static int stm32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & STM32G0_FLASH_BSY1) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & STM32G0_FLASH_WRPERR) {
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	if (status & STM32G0_FLASH_PROGERR) {
		LOG_ERROR("stm32x device programming failed");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & (STM32G0_FLASH_WRPERR | STM32G0_FLASH_PROGERR)) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_SR),
				STM32G0_FLASH_WRPERR | STM32G0_FLASH_PROGERR);
	}
	return retval;
}

static int stm32x_read_options(struct flash_bank *bank)
{
	struct stm32g0x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	/* read user and read protection option bytes */
	retval = target_read_u32(target, STM32_OB_USER_RDP, &(stm32x_info->option_bytes.user_rdp));
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_OB_WRP1A, &(stm32x_info->option_bytes.wrp1a));
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_OB_WRP1B, &(stm32x_info->option_bytes.wrp1b));
	if (retval != ERROR_OK)
		return retval;

	uint32_t start, end;
	retval = target_read_u32(target, STM32_OB_PCROP1A_STRT, &start);
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u32(target, STM32_OB_PCROP1A_END, &end);
	if (retval != ERROR_OK)
		return retval;
	stm32x_info->option_bytes.pcrop1a = start + (end << 16);

	retval = target_read_u32(target, STM32_OB_PCROP1B_STRT, &start);
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u32(target, STM32_OB_PCROP1B_END, &end);
	if (retval != ERROR_OK)
		return retval;
	stm32x_info->option_bytes.pcrop1b = start + (end << 16);

	retval = target_read_u32(target, STM32_OB_BOOT_SEC, &(stm32x_info->option_bytes.security));
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32g0x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* unlock flash registers */
	int retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_OPTKEYR), OPTKEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_OPTKEYR), OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	/* program option dwords */
	uint8_t opt_bytes[64];

	target_buffer_set_u64(target, opt_bytes, stm32x_info->option_bytes.user_rdp);
	target_buffer_set_u64(target, opt_bytes + 8, stm32x_info->option_bytes.pcrop1a & 0xff);
	target_buffer_set_u64(target, opt_bytes + 16, (stm32x_info->option_bytes.pcrop1a >> 16) & 0xff);
	target_buffer_set_u64(target, opt_bytes + 24, stm32x_info->option_bytes.wrp1a);
	target_buffer_set_u64(target, opt_bytes + 32, stm32x_info->option_bytes.wrp1b);
	target_buffer_set_u64(target, opt_bytes + 40, stm32x_info->option_bytes.pcrop1b);
	target_buffer_set_u64(target, opt_bytes + 48, (stm32x_info->option_bytes.pcrop1b >> 16) & 0xff);
	target_buffer_set_u64(target, opt_bytes + 56, stm32x_info->option_bytes.security);

	retval = stm32x_write_block(bank, opt_bytes, STM32_OB_USER_RDP, sizeof(opt_bytes) / 8);
	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			LOG_ERROR("working area required to erase options bytes");
		return retval;
	}

	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), STM32G0_FLASH_OPTSTRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), STM32G0_FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t protection;

	for (int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = 0;

	/* area A register */
	/* bits 21:16 - WRP1A_END[5:0], bits 5:0 - WRP1A_START[5:0] */
	int retval = target_read_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_WRP1AR), &protection);
	if (retval != ERROR_OK)
		return retval;

	/* the bank is protected, if it is in the area defined by the start and end */
	uint8_t start = protection & 0x3F;
	uint8_t end = (protection >> 16) & 0x3F;
	if (start <= end) {
		for (int i = 0; i < bank->num_prot_blocks; i++) {
			if ((i >= start) && (i <= end))
				bank->prot_blocks[i].is_protected = 1;
		}
	}

	/* area B register */
	/* bits 21:16 - WRP1B_END[5:0], bits 5:0 - WRP1B_START[5:0] */
	retval = target_read_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_WRP1BR), &protection);
	if (retval != ERROR_OK)
		return retval;

	/* the bank is protected, if it is in the area defined by the start and end */
	start = protection & 0x3F;
	end = (protection >> 16) & 0x3F;
	if (start <= end) {
		for (int i = 0; i < bank->num_prot_blocks; i++) {
			if ((i >= start) && (i <= end))
				bank->prot_blocks[i].is_protected = 1;
		}
	}

	return ERROR_OK;
}

static int stm32gx_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;

	/* unlock flash registers */
	int retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	uint32_t val = 0x00L;
	for (int i = first; i <= last; i++) {
		retval = target_read_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), &val);
		if (retval != ERROR_OK)
			return retval;

		val &= ~(0x3F << 3);
		val |= (STM32G0_FLASH_PER | (i << 3) | STM32G0_FLASH_STRT);

		retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), val);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), STM32G0_FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return stm32x_mass_erase(bank);

	int retval = stm32gx_erase(bank, first, last);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	struct stm32g0x_flash_bank *stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (first > last) {
		LOG_ERROR("Invalid memory range");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (set)
		stm32x_info->option_bytes.wrp1a = first + (last << 16);
	else
		stm32x_info->option_bytes.wrp1a = 0;

	return stm32x_write_options(bank);
}

static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t count)
{
	struct stm32g0x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t stm32x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/stm32/stm32g0x.inc"
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code),
				&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32x_flash_write_code), stm32x_flash_write_code);
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

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, stm32x_info->register_base);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, count, 8,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) & FLASH_PGERR) {
			LOG_ERROR("flash memory not erased before writing");
			/* Clear but report errors */
			target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR), FLASH_PGERR);
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) & FLASH_WRPRTERR) {
			LOG_ERROR("flash memory write protected");
			/* Clear but report errors */
			target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR), FLASH_WRPRTERR);
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

static int stm32gx_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	uint8_t *new_buffer = NULL;

	if (offset & 0x7) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 8-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of bytes, the data has to be padded. Duplicate
	 * the buffer and use the normal code path with a single block write since
	 * it's probably cheaper than to special case the last odd write using
	 * discrete accesses. */
	if (count & 7) {
		int pad_count = 8 - (count & 7);
		new_buffer = malloc(count + pad_count);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write, padding with 0xff");
		buffer = memcpy(new_buffer, buffer, count);
		for (int i = 0; i < pad_count; i++)
			new_buffer[count++] = 0xff;
	}

	uint32_t dwords_remaining = count / 8;
	int retval, retval2;

	/* unlock flash registers */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		goto cleanup;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		goto cleanup;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), STM32G0_FLASH_PG);
	if (retval != ERROR_OK)
		goto cleanup;

	/* try using a block write */
	retval = stm32x_write_block(bank, buffer, bank->base + offset, dwords_remaining);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) double word accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (dwords_remaining > 0) {
			uint64_t value;
			memcpy(&value, buffer, sizeof(uint64_t));

			retval = target_write_u64(target, bank->base + offset, value);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			retval = stm32x_wait_status_busy(bank, 5);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			dwords_remaining--;
			buffer += 8;
			offset += 8;
		}
	}

reset_pg_and_lock:
	retval2 = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), STM32G0_FLASH_LOCK);
	if (retval == ERROR_OK)
		retval = retval2;

cleanup:
	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int stm32x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = stm32gx_write(bank, buffer, offset, count);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	/* This check the device CPUID core register to detect
	 * the M0 from the M3 devices. */

	struct target *target = bank->target;
	uint32_t cpuid, device_id_register = 0;

	/* Get the CPUID from the ARM Core
	 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0432c/DDI0432C_cortex_m0_r0p0_trm.pdf 4.2.1 */
	int retval = target_read_u32(target, 0xE000ED00, &cpuid);
	if (retval != ERROR_OK)
		return retval;

	if (((cpuid >> 4) & 0xFFF) == 0xC60) {
		/* 0xC60 is M0+ devices */
		device_id_register = 0x40015800;
	} else {
		LOG_ERROR("Cannot identify target as a stm32g0x");
		return ERROR_FAIL;
	}

	/* read stm32 device id register */
	retval = target_read_u32(target, device_id_register, device_id);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32x_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
	struct target *target = bank->target;
	uint32_t cpuid, flash_size_reg;

	int retval = target_read_u32(target, 0xE000ED00, &cpuid);
	if (retval != ERROR_OK)
		return retval;

	if (((cpuid >> 4) & 0xFFF) == 0xC60) {
		/* 0xC60 is M0+ devices */
		flash_size_reg = 0x1FFF75E0;
	} else {
		LOG_ERROR("Cannot identify target as a stm32g0x");
		return ERROR_FAIL;
	}

	retval = target_read_u16(target, flash_size_reg, flash_size_in_kb);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int stm32x_probe(struct flash_bank *bank)
{
	struct stm32g0x_flash_bank *stm32x_info = bank->driver_priv;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	int page_size;
	uint32_t base_address = 0x08000000;

	stm32x_info->probed = 0;
	stm32x_info->register_base = FLASH_REG_BASE;

	/* default factory read protection level 0 */
	stm32x_info->default_rdp = 0xAA;

	/* read stm32 device id register */
	int retval = stm32x_get_device_id(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	/* set page size, protection granularity and max flash size depending on family */
	switch (device_id & 0xfff) {
		case 0x460: /* stm32g07x */
			page_size = 2048;
			max_flash_size_in_kb = 128;
			break;
		default:
			LOG_WARNING("Cannot identify target as a STM32 family.");
			return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = stm32x_get_flash_size(bank, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
				max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (stm32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = stm32x_info->user_bank_size / 1024;
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

	if (bank->prot_blocks) {
		free(bank->prot_blocks);
		bank->prot_blocks = NULL;
	}

	bank->base = base_address;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;
	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	bank->num_prot_blocks = num_pages;
	bank->prot_blocks = alloc_block_array(0, page_size, num_pages);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	stm32x_info->probed = 1;

	return ERROR_OK;
}

static int stm32x_auto_probe(struct flash_bank *bank)
{
	struct stm32g0x_flash_bank *stm32x_info = bank->driver_priv;
	if (stm32x_info->probed)
		return ERROR_OK;
	return stm32x_probe(bank);
}

#if 0
COMMAND_HANDLER(stm32x_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static const char *get_stm32g0_revision(uint16_t rev_id)
{
	const char *rev_str = NULL;

	switch (rev_id) {
		case 0x1000:
			rev_str = "A";
			break;
		case 0x2000:
			rev_str = "B";
			break;
	}
	return rev_str;
}

static int get_stm32x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint32_t dbgmcu_idcode;

	/* read stm32 device id register */
	int retval = stm32x_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint16_t rev_id = dbgmcu_idcode >> 16;
	const char *device_str;
	const char *rev_str = NULL;

	switch (device_id) {

		case 0x460:
			device_str = "STM32G07x";
			rev_str = get_stm32g0_revision(rev_id);
			break;

		default:
			snprintf(buf, buf_size, "Cannot identify target as a STM32G0\n");
			return ERROR_FAIL;
	}

	if (rev_str != NULL)
		snprintf(buf, buf_size, "%s - Rev: %s", device_str, rev_str);
	else
		snprintf(buf, buf_size, "%s - Rev: unknown (0x%04x)", device_str, rev_id);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32g0x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* set readout protection */
	stm32x_info->option_bytes.rdp = 0;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x locked");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
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

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to unlock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_options_read_command)
{
	struct target *target = NULL;
  struct stm32g0x_flash_bank *stm32x_info;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct stm32g0x_options options;

	retval = target_read_u32(target, STM32_OB_USER_RDP, &(options.user_rdp));
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_OB_WRP1A, &(options.wrp1a));
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_OB_WRP1B, &(options.wrp1b));
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD_CTX, "option byte register = 0x%" PRIx32 "", options.user_rdp);
	command_print(CMD_CTX, "write protection register A = 0x%" PRIx32 "", options.wrp1a);
	command_print(CMD_CTX, "write protection register B = 0x%" PRIx32 "", options.wrp1b);

	command_print(CMD_CTX, "window watchdog: %sware",
			options.wwdg_sw ? "soft" : "hard");

	command_print(CMD_CTX, "stop mode: %sreset generated upon entry",
			options.rst_stop ? "no " : "");

	command_print(CMD_CTX, "standby mode: %sreset generated upon entry",
			options.rst_stdby ? "no " : "");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_options_write_command)
{
	struct target *target = NULL;
	struct stm32g0x_flash_bank *stm32x_info = NULL;
	struct stm32g0x_options options;
	uint8_t bor_level;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = stm32x_read_options(bank);
	if (ERROR_OK != retval)
		return retval;

	/* start with current options */
	options = stm32x_info->option_bytes;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	while (CMD_ARGC) {
		if (strcmp("BOREN", CMD_ARGV[0]) == 0)
			options.bor_en = 1;
		else if (strcmp("NOBOREN", CMD_ARGV[0]) == 0)
			options.bor_en = 0;
		else if (strcmp("BORFLEV", CMD_ARGV[0]) == 0) {
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], bor_level);
			options.borf_lev = bor_level;
			CMD_ARGC--;
			CMD_ARGV++;
		} else if (strcmp("BORRLEV", CMD_ARGV[0]) == 0) {
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], bor_level);
			options.borr_lev = bor_level;
			CMD_ARGC--;
			CMD_ARGV++;
		} else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
			options.rst_stop = 1;
		else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
			options.rst_stop = 0;
		else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
			options.rst_stdby = 1;
		else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
			options.rst_stdby = 0;
		else if (strcmp("NORSTSHDW", CMD_ARGV[0]) == 0)
			options.rst_shdw = 1;
		else if (strcmp("RSTSHDW", CMD_ARGV[0]) == 0)
			options.rst_shdw = 0;
		else if (strcmp("IDWGSW", CMD_ARGV[0]) == 0)
			options.iwdg_sw = 1;
		else if (strcmp("IDWGHW", CMD_ARGV[0]) == 0)
			options.iwdg_sw = 0;
		else if (strcmp("IDWGSTOP", CMD_ARGV[0]) == 0)
			options.iwdg_stop = 1;
		else if (strcmp("NOIDWGSTOP", CMD_ARGV[0]) == 0)
			options.iwdg_stop = 0;
		else if (strcmp("IDWGSTDBY", CMD_ARGV[0]) == 0)
			options.iwdg_stdby = 1;
		else if (strcmp("NOIDWGSTDBY", CMD_ARGV[0]) == 0)
			options.iwdg_stdby = 0;
		else if (strcmp("WWDGSW", CMD_ARGV[0]) == 0)
			options.wwdg_sw = 1;
		else if (strcmp("WWDGHW", CMD_ARGV[0]) == 0)
			options.wwdg_sw = 0;
		else if (strcmp("PARITY", CMD_ARGV[0]) == 0)
			options.ram_parity_check = 1;
		else if (strcmp("NOPARITY", CMD_ARGV[0]) == 0)
			options.ram_parity_check = 0;
		else if (strcmp("BOOTSEL", CMD_ARGV[0]) == 0)
			options.boot_sel = 1;
		else if (strcmp("NOBOOTSEL", CMD_ARGV[0]) == 0)
			options.boot_sel = 0;
		else if (strcmp("BOOT1", CMD_ARGV[0]) == 0)
			options.boot1 = 1;
		else if (strcmp("NOBOOT1", CMD_ARGV[0]) == 0)
			options.boot1 = 0;
		else if (strcmp("BOOT0", CMD_ARGV[0]) == 0)
			options.boot0 = 1;
		else if (strcmp("NOBOOT0", CMD_ARGV[0]) == 0)
			options.boot0 = 0;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}

	stm32x_info->option_bytes = options;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32x failed to write options");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x write options complete.\n"
			"INFO: %spower cycle is required "
			"for the new settings to take effect.",
			"'stm32g0x options_load' command or ");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_options_load_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock flash registers */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_OPTKEYR), OPTKEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_OPTKEYR), OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	/* force re-load of option bytes - generates software reset */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), STM32G0_FLASH_OBL_LAUNCH);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32gx_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	/* unlock option flash registers */
	int retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR),
			STM32G0_FLASH_MER | STM32G0_FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32G0_FLASH_CR), STM32G0_FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}


static int stm32x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = stm32gx_mass_erase(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "stm32x mass erase complete");
	} else
		command_print(CMD_CTX, "stm32x mass erase failed");

	return retval;
}

static const struct command_registration stm32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = stm32x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "options_write",
		.handler = stm32x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP') ('USEROPT' user_data)",
		.help = "Replace bits in device option bytes.",
	},
	{
		.name = "options_load",
		.handler = stm32x_handle_options_load_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Force re-load of device option bytes.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32g0x",
		.mode = COMMAND_ANY,
		.help = "stm32g0x flash command group",
		.usage = "",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver stm32g0x_flash = {
	.name = "stm32g0x",
	.commands = stm32x_command_handlers,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.read = default_flash_read,
	.probe = stm32x_probe,
	.auto_probe = stm32x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stm32x_protect_check,
	.info = get_stm32x_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
