/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *
 *   Copyright (C) 2011 by Erik Botö
 *   erik.boto@pelagicore.com
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

/* em357 register locations */

#define EM357_FLASH_ACR         0x40008000
#define EM357_FLASH_KEYR        0x40008004
#define EM357_FLASH_OPTKEYR     0x40008008
#define EM357_FLASH_SR          0x4000800C
#define EM357_FLASH_CR          0x40008010
#define EM357_FLASH_AR          0x40008014
#define EM357_FLASH_OBR         0x4000801C
#define EM357_FLASH_WRPR        0x40008020

#define EM357_FPEC_CLK          0x4000402c
/* option byte location */

#define EM357_OB_RDP            0x08040800
#define EM357_OB_WRP0           0x08040808
#define EM357_OB_WRP1           0x0804080A
#define EM357_OB_WRP2           0x0804080C

/* FLASH_CR register bits */

#define FLASH_PG                (1 << 0)
#define FLASH_PER               (1 << 1)
#define FLASH_MER               (1 << 2)
#define FLASH_OPTPG             (1 << 4)
#define FLASH_OPTER             (1 << 5)
#define FLASH_STRT              (1 << 6)
#define FLASH_LOCK              (1 << 7)
#define FLASH_OPTWRE    (1 << 9)

/* FLASH_SR register bits */

#define FLASH_BSY               (1 << 0)
#define FLASH_PGERR             (1 << 2)
#define FLASH_WRPRTERR  (1 << 4)
#define FLASH_EOP               (1 << 5)

/* EM357_FLASH_OBR bit definitions (reading) */

#define OPT_ERROR               0
#define OPT_READOUT             1

/* register unlock keys */

#define KEY1                    0x45670123
#define KEY2                    0xCDEF89AB

struct em357_options {
	uint16_t RDP;
	uint16_t user_options;
	uint16_t protection[3];
};

struct em357_flash_bank {
	struct em357_options option_bytes;
	int ppage_size;
	bool probed;
};

static int em357_mass_erase(struct flash_bank *bank);

/* flash bank em357 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(em357_flash_bank_command)
{
	struct em357_flash_bank *em357_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	em357_info = malloc(sizeof(struct em357_flash_bank));
	bank->driver_priv = em357_info;

	em357_info->probed = false;

	return ERROR_OK;
}

static inline int em357_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, EM357_FLASH_SR, status);
}

static int em357_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;; ) {
		retval = em357_get_flash_status(bank, &status);
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

	if (status & FLASH_WRPRTERR) {
		LOG_ERROR("em357 device protected");
		retval = ERROR_FAIL;
	}

	if (status & FLASH_PGERR) {
		LOG_ERROR("em357 device programming failed");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & (FLASH_WRPRTERR | FLASH_PGERR)) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, EM357_FLASH_SR, FLASH_WRPRTERR | FLASH_PGERR);
	}
	return retval;
}

static int em357_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct em357_flash_bank *em357_info = NULL;
	struct target *target = bank->target;

	em357_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, EM357_FLASH_OBR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	em357_info->option_bytes.user_options = (uint16_t)0xFFFC | ((optiondata >> 2) & 0x03);
	em357_info->option_bytes.RDP = (optiondata & (1 << OPT_READOUT)) ? 0xFFFF : 0x5AA5;

	if (optiondata & (1 << OPT_READOUT))
		LOG_INFO("Device Security Bit Set");

	/* each bit refers to a 4bank protection */
	retval = target_read_u32(target, EM357_FLASH_WRPR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

	em357_info->option_bytes.protection[0] = (uint16_t)optiondata;
	em357_info->option_bytes.protection[1] = (uint16_t)(optiondata >> 8);
	em357_info->option_bytes.protection[2] = (uint16_t)(optiondata >> 16);

	return ERROR_OK;
}

static int em357_erase_options(struct flash_bank *bank)
{
	struct em357_flash_bank *em357_info = NULL;
	struct target *target = bank->target;

	em357_info = bank->driver_priv;

	/* read current options */
	em357_read_options(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, EM357_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, EM357_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, EM357_FLASH_OPTKEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_OPTKEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* erase option bytes */
	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_OPTER | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_OPTER | FLASH_STRT | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;

	retval = em357_wait_status_busy(bank, 10);
	if (retval != ERROR_OK)
		return retval;

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	em357_info->option_bytes.RDP = 0x5AA5;

	return ERROR_OK;
}

static int em357_write_options(struct flash_bank *bank)
{
	struct em357_flash_bank *em357_info = NULL;
	struct target *target = bank->target;

	em357_info = bank->driver_priv;

	/* unlock flash registers */
	int retval = target_write_u32(target, EM357_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, EM357_FLASH_OPTKEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_OPTKEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* program option bytes */
	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_OPTPG | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		return retval;

	retval = em357_wait_status_busy(bank, 10);
	if (retval != ERROR_OK)
		return retval;

	/* write protection byte 1 */
	retval = target_write_u16(target, EM357_OB_WRP0, em357_info->option_bytes.protection[0]);
	if (retval != ERROR_OK)
		return retval;

	retval = em357_wait_status_busy(bank, 10);
	if (retval != ERROR_OK)
		return retval;

	/* write protection byte 2 */
	retval = target_write_u16(target, EM357_OB_WRP1, em357_info->option_bytes.protection[1]);
	if (retval != ERROR_OK)
		return retval;

	retval = em357_wait_status_busy(bank, 10);
	if (retval != ERROR_OK)
		return retval;

	/* write protection byte 3 */
	retval = target_write_u16(target, EM357_OB_WRP2, em357_info->option_bytes.protection[2]);
	if (retval != ERROR_OK)
		return retval;

	retval = em357_wait_status_busy(bank, 10);
	if (retval != ERROR_OK)
		return retval;

	/* write readout protection bit */
	retval = target_write_u16(target, EM357_OB_RDP, em357_info->option_bytes.RDP);
	if (retval != ERROR_OK)
		return retval;

	retval = em357_wait_status_busy(bank, 10);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int em357_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct em357_flash_bank *em357_info = bank->driver_priv;

	uint32_t protection;
	int i, s;
	int num_bits;
	int set;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* each bit refers to a 4bank protection (bit 0-23) */
	int retval = target_read_u32(target, EM357_FLASH_WRPR, &protection);
	if (retval != ERROR_OK)
		return retval;

	/* each protection bit is for 4 * 2K pages */
	num_bits = (bank->num_sectors / em357_info->ppage_size);

	for (i = 0; i < num_bits; i++) {
		set = 1;
		if (protection & (1 << i))
			set = 0;

		for (s = 0; s < em357_info->ppage_size; s++)
			bank->sectors[(i * em357_info->ppage_size) + s].is_protected = set;
	}

	return ERROR_OK;
}

static int em357_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return em357_mass_erase(bank);

	/* Enable FPEC clock */
	target_write_u32(target, EM357_FPEC_CLK, 0x00000001);

	/* unlock flash registers */
	int retval = target_write_u32(target, EM357_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = first; i <= last; i++) {
		retval = target_write_u32(target, EM357_FLASH_CR, FLASH_PER);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, EM357_FLASH_AR,
				bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, EM357_FLASH_CR, FLASH_PER | FLASH_STRT);
		if (retval != ERROR_OK)
			return retval;

		retval = em357_wait_status_busy(bank, 100);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int em357_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct em357_flash_bank *em357_info = NULL;
	struct target *target = bank->target;
	uint16_t prot_reg[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	int reg, bit;
	int status;
	uint32_t protection;

	em357_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first % em357_info->ppage_size) != 0) {
		LOG_WARNING("aligned start protect sector to a %d sector boundary",
			em357_info->ppage_size);
		first = first - (first % em357_info->ppage_size);
	}
	if (((last + 1) % em357_info->ppage_size) != 0) {
		LOG_WARNING("aligned end protect sector to a %d sector boundary",
			em357_info->ppage_size);
		last++;
		last = last - (last % em357_info->ppage_size);
		last--;
	}

	/* each bit refers to a 4bank protection */
	int retval = target_read_u32(target, EM357_FLASH_WRPR, &protection);
	if (retval != ERROR_OK)
		return retval;

	prot_reg[0] = (uint16_t)protection;
	prot_reg[1] = (uint16_t)(protection >> 8);
	prot_reg[2] = (uint16_t)(protection >> 16);

	for (unsigned int i = first; i <= last; i++) {
		reg = (i / em357_info->ppage_size) / 8;
		bit = (i / em357_info->ppage_size) - (reg * 8);

		LOG_WARNING("reg, bit: %d, %d", reg, bit);
		if (set)
			prot_reg[reg] &= ~(1 << bit);
		else
			prot_reg[reg] |= (1 << bit);
	}

	status = em357_erase_options(bank);
	if (retval != ERROR_OK)
		return status;

	em357_info->option_bytes.protection[0] = prot_reg[0];
	em357_info->option_bytes.protection[1] = prot_reg[1];
	em357_info->option_bytes.protection[2] = prot_reg[2];

	return em357_write_options(bank);
}

static int em357_write_block(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* see contrib/loaders/flash/stm32x.s for src, the same is used here except for
	 * a modified *_FLASH_BASE */

	static const uint8_t em357_flash_write_code[] = {
		/* #define EM357_FLASH_CR_OFFSET	0x10
		 * #define EM357_FLASH_SR_OFFSET	0x0C
		 * write: */
		0x08, 0x4c,					/* ldr	r4, EM357_FLASH_BASE */
		0x1c, 0x44,					/* add	r4, r3 */
		/* write_half_word: */
		0x01, 0x23,					/* movs	r3, #0x01 */
		0x23, 0x61,					/* str	r3, [r4,
								 *#EM357_FLASH_CR_OFFSET] */
		0x30, 0xf8, 0x02, 0x3b,		/* ldrh	r3, [r0], #0x02 */
		0x21, 0xf8, 0x02, 0x3b,		/* strh	r3, [r1], #0x02 */
		/* busy: */
		0xe3, 0x68,					/* ldr	r3, [r4,
								 *#EM357_FLASH_SR_OFFSET] */
		0x13, 0xf0, 0x01, 0x0f,		/* tst	r3, #0x01 */
		0xfb, 0xd0,					/* beq	busy */
		0x13, 0xf0, 0x14, 0x0f,		/* tst	r3, #0x14 */
		0x01, 0xd1,					/* bne	exit */
		0x01, 0x3a,					/* subs	r2, r2, #0x01 */
		0xf0, 0xd1,					/* bne	write_half_word */
		/* exit: */
		0x00, 0xbe,					/* bkpt	#0x00 */
		0x00, 0x80, 0x00, 0x40,		/* EM357_FLASH_BASE: .word 0x40008000 */
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(em357_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(em357_flash_write_code), em357_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING(
				"no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN_OUT);

	while (count > 0) {
		uint32_t thisrun_count = (count > (buffer_size / 2)) ?
			(buffer_size / 2) : count;

		retval = target_write_buffer(target, source->address, thisrun_count * 2, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);
		buf_set_u32(reg_params[3].value, 0, 32, 0);

		retval = target_run_algorithm(target, 0, NULL, 4, reg_params,
				write_algorithm->address, 0, 10000, &armv7m_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("error executing em357 flash write algorithm");
			break;
		}

		if (buf_get_u32(reg_params[3].value, 0, 32) & FLASH_PGERR) {
			LOG_ERROR("flash memory not erased before writing");
			/* Clear but report errors */
			target_write_u32(target, EM357_FLASH_SR, FLASH_PGERR);
			retval = ERROR_FAIL;
			break;
		}

		if (buf_get_u32(reg_params[3].value, 0, 32) & FLASH_WRPRTERR) {
			LOG_ERROR("flash memory write protected");
			/* Clear but report errors */
			target_write_u32(target, EM357_FLASH_SR, FLASH_WRPRTERR);
			retval = ERROR_FAIL;
			break;
		}

		buffer += thisrun_count * 2;
		address += thisrun_count * 2;
		count -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}

static int em357_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t words_remaining = (count / 2);
	uint32_t bytes_remaining = (count & 0x00000001);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* unlock flash registers */
	retval = target_write_u32(target, EM357_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	target_write_u32(target, EM357_FPEC_CLK, 0x00000001);

	/* multiple half words (2-byte) to be programmed? */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = em357_write_block(bank, buffer, offset, words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING(
					"couldn't use block writes, falling back to single memory accesses");
			}
		} else {
			buffer += words_remaining * 2;
			address += words_remaining * 2;
			words_remaining = 0;
		}
	}

	if ((retval != ERROR_OK) && (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE))
		return retval;

	while (words_remaining > 0) {
		uint16_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint16_t));

		retval = target_write_u32(target, EM357_FLASH_CR, FLASH_PG);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u16(target, address, value);
		if (retval != ERROR_OK)
			return retval;

		retval = em357_wait_status_busy(bank, 5);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 2;
		words_remaining--;
		address += 2;
	}

	if (bytes_remaining) {
		uint16_t value = 0xffff;
		memcpy(&value, buffer + bytes_written, bytes_remaining);

		retval = target_write_u32(target, EM357_FLASH_CR, FLASH_PG);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u16(target, address, value);
		if (retval != ERROR_OK)
			return retval;

		retval = em357_wait_status_busy(bank, 5);
		if (retval != ERROR_OK)
			return retval;
	}

	return target_write_u32(target, EM357_FLASH_CR, FLASH_LOCK);
}

static int em357_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct em357_flash_bank *em357_info = bank->driver_priv;
	int i;
	uint16_t num_pages;
	int page_size;
	uint32_t base_address = 0x08000000;

	em357_info->probed = false;

	switch (bank->size) {
		case 0x10000:
			/* 64k -- 64 1k pages */
			num_pages = 64;
			page_size = 1024;
			break;
		case 0x20000:
			/* 128k -- 128 1k pages */
			num_pages = 128;
			page_size = 1024;
			break;
		case 0x30000:
			/* 192k -- 96 2k pages */
			num_pages = 96;
			page_size = 2048;
			break;
		case 0x40000:
			/* 256k -- 128 2k pages */
			num_pages = 128;
			page_size = 2048;
			break;
		case 0x80000:
			/* 512k -- 256 2k pages */
			num_pages = 256;
			page_size = 2048;
			break;
		default:
			LOG_WARNING("No size specified for em357 flash driver, assuming 192k!");
			num_pages = 96;
			page_size = 2048;
			break;
	}

	/* Enable FPEC CLK */
	int retval = target_write_u32(target, EM357_FPEC_CLK, 0x00000001);
	if (retval != ERROR_OK)
		return retval;

	em357_info->ppage_size = 4;

	LOG_INFO("flash size = %dkbytes", num_pages*page_size/1024);

	free(bank->sectors);

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

	em357_info->probed = true;

	return ERROR_OK;
}

static int em357_auto_probe(struct flash_bank *bank)
{
	struct em357_flash_bank *em357_info = bank->driver_priv;
	if (em357_info->probed)
		return ERROR_OK;
	return em357_probe(bank);
}

COMMAND_HANDLER(em357_handle_lock_command)
{
	struct target *target = NULL;
	struct em357_flash_bank *em357_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	em357_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (em357_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "em357 failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	em357_info->option_bytes.RDP = 0;

	if (em357_write_options(bank) != ERROR_OK) {
		command_print(CMD, "em357 failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "em357 locked");

	return ERROR_OK;
}

COMMAND_HANDLER(em357_handle_unlock_command)
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

	if (em357_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "em357 failed to unlock device");
		return ERROR_OK;
	}

	if (em357_write_options(bank) != ERROR_OK) {
		command_print(CMD, "em357 failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "em357 unlocked.\n"
		"INFO: a reset or power cycle is required "
		"for the new settings to take effect.");

	return ERROR_OK;
}

static int em357_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Make sure the flash clock is on */
	target_write_u32(target, EM357_FPEC_CLK, 0x00000001);

	/* unlock option flash registers */
	int retval = target_write_u32(target, EM357_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_MER);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_MER | FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = em357_wait_status_busy(bank, 100);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, EM357_FLASH_CR, FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(em357_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = em357_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "em357 mass erase complete");
	} else
		command_print(CMD, "em357 mass erase failed");

	return retval;
}

static const struct command_registration em357_exec_command_handlers[] = {
	{
		.name = "lock",
		.usage = "<bank>",
		.handler = em357_handle_lock_command,
		.mode = COMMAND_EXEC,
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.usage = "<bank>",
		.handler = em357_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.usage = "<bank>",
		.handler = em357_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration em357_command_handlers[] = {
	{
		.name = "em357",
		.mode = COMMAND_ANY,
		.help = "em357 flash command group",
		.usage = "",
		.chain = em357_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver em357_flash = {
	.name = "em357",
	.commands = em357_command_handlers,
	.flash_bank_command = em357_flash_bank_command,
	.erase = em357_erase,
	.protect = em357_protect,
	.write = em357_write,
	.read = default_flash_read,
	.probe = em357_probe,
	.auto_probe = em357_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = em357_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
