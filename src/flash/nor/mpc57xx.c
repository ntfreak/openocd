/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
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

/* MPC version (C) 2015 James Murray */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "imp.h"
#include <target/algorithm.h>
#include <target/mpc57xx_jtag.h>
#include <target/mpc57xx.h>

#define MPC57XX_MANUF_ID	0x029 /* will be wrong */

struct mpc57xx_flash_bank { /* all unknown at the moment */
	uint32_t *block_bits_lmlr;
	uint32_t *block_bits_hlr;
	uint32_t disable_bit;
	uint32_t busy_bits;
	uint32_t *register_base;
	int probed;
};

struct mpc57xx_mem_layout {
	uint32_t block_start;
	uint32_t block_size;
	uint32_t lmlr_bit;
	uint32_t hlr_bit;
};

static const struct mpc57xx_mem_layout mem_layout_array0[] = {
	{0x00000000, 0x04000, 0x01, 0x00},
	{0x00004000, 0x04000, 0x02, 0x00},
	{0x00008000, 0x08000, 0x04, 0x00},
	{0x00010000, 0x08000, 0x08, 0x00},
	{0x00018000, 0x04000, 0x10, 0x00},
	{0x0001c000, 0x04000, 0x20, 0x00},
	{0x00020000, 0x10000, 0x40, 0x00},
	{0x00030000, 0x10000, 0x80, 0x00},
	{0x00040000, 0x20000, 0x10000, 0x00}, /* NA in 768K deriv */
	{0x00060000, 0x20000, 0x20000, 0x00}  /* NA in 768K deriv */
};
static const struct mpc57xx_mem_layout mem_layout_array1[] = {
	{0x00080000, 0x20000, 0x00, 0x01},
	{0x000a0000, 0x20000, 0x00, 0x02},
	{0x000c0000, 0x20000, 0x00, 0x04},
	{0x000e0000, 0x20000, 0x00, 0x08}
};
static const struct mpc57xx_mem_layout mem_layout_array2[] = {
		{0x00100000, 0x20000, 0x00, 0x01}, /* NA in 768K or 1M deriv */
		{0x00120000, 0x20000, 0x00, 0x02}, /* NA in 768K or 1M deriv */
		{0x00140000, 0x20000, 0x00, 0x04}, /* NA in 768K or 1M deriv */
		{0x00160000, 0x20000, 0x00, 0x08}  /* NA in 768K or 1M deriv */
};

static const uint32_t mpc57xx_flash_regs[3] = {
		0xc3f88000,
		0xc3fb0000,
		0xc3fb4000
};

#define MPC57XX_BIUCR 0xc3f8801C
#define MPC57XX_BIUAPR 0xc3f88020
#define MPC57XX_BIUCR2 0xc3f88024
#define MPC57XX_PFCR3 0xc3f88028

#define MPC57XX_MCR_OFFSET		0x00
#define MPC57XX_LMLR_OFFSET		0x04
#define MPC57XX_HLR_OFFSET		0x08
#define MPC57XX_SLMLR_OFFSET	0x0c
#define MPC57XX_LMSR_OFFSET		0x10
#define MPC57XX_HSR_OFFSET		0x14
#define MPC57XX_AR_OFFSET		0x18

#define MPC57XX_LMLR_LME_PASS	0xA1A11111
#define MPC57XX_HLR_HBE_PASS	0xB2B22222
#define MPC57XX_SLMLR_SLE_PASS	0xC3C33333
/*
 * DEVID values
 */

static const struct mpc57xx_devs_s {
	uint32_t devid;
	const char *name;
} mpc57xx_devs[] = {
	{0x1834601d, "MPC574R JTAG"}, /* Developed against this chip only. */
	{0x00000000, NULL}
};

#if 0
static int mpc57xx_get_flash_adr(struct flash_bank *bank, uint32_t reg)
{
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	return mpc57xx_info->register_base | reg;
}
#endif

static int mpc57xx_build_block_list(struct flash_bank *bank)
{
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;

	int i;
	int num_blocks;
	int b[3];
	b[0] = 0;
	b[1] = 0;
	b[2] = 0;

	switch (bank->size) {
		case 0x180000: /* 1.5M */
			b[0] = 10;
			b[1] = 4;
			b[2] = 4;
			break;
		case 0x100000: /* 1 M */
			b[0] = 10;
			b[1] = 4;
			b[2] = 0;
			break;
		case 0x0c0000: /* 768M */
			b[0] = 8;
			b[1] = 4;
			b[2] = 0;
			break;
		default:
			LOG_ERROR("BUG: unknown bank->size encountered. Only 0x180000, 0x100000, 0xc0000 supported.");
			exit(-1);
	}

	num_blocks = b[0] + b[1] + b[2];

	printf("b0=%d, b1=%d, b2=%d\n", b[0], b[1], b[2]);

	bank->num_sectors = num_blocks;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_blocks);
	mpc57xx_info->block_bits_lmlr = malloc(sizeof(uint32_t) * num_blocks);
	mpc57xx_info->block_bits_hlr = malloc(sizeof(uint32_t) * num_blocks);
	mpc57xx_info->register_base = malloc(sizeof(uint32_t) * num_blocks);

	num_blocks = 0;

	for (i = 0; i < b[0]; i++) {
		bank->sectors[num_blocks].offset = mem_layout_array0[i].block_start;
		bank->sectors[num_blocks].size = mem_layout_array0[i].block_size;
		bank->sectors[num_blocks].is_erased = -1;
		bank->sectors[num_blocks].is_protected = 1; /* protection bits likely set by hardware */
		mpc57xx_info->block_bits_lmlr[num_blocks] = mem_layout_array0[i].lmlr_bit;
		mpc57xx_info->block_bits_hlr[num_blocks] = mem_layout_array0[i].hlr_bit;
		mpc57xx_info->register_base[num_blocks++] = mpc57xx_flash_regs[0]; /* base register address per block */
	}

	for (i = 0; i < b[1]; i++) {
		bank->sectors[num_blocks].offset = mem_layout_array1[i].block_start;
		bank->sectors[num_blocks].size = mem_layout_array1[i].block_size;
		bank->sectors[num_blocks].is_erased = -1;
		bank->sectors[num_blocks].is_protected = 1; /* protection bits likely set by hardware */
		mpc57xx_info->block_bits_lmlr[num_blocks] = mem_layout_array1[i].lmlr_bit;
		mpc57xx_info->block_bits_hlr[num_blocks] = mem_layout_array1[i].hlr_bit;
		mpc57xx_info->register_base[num_blocks++] = mpc57xx_flash_regs[1]; /* base register address per block */
	}

	for (i = 0; i < b[2]; i++) {
		bank->sectors[num_blocks].offset = mem_layout_array2[i].block_start;
		bank->sectors[num_blocks].size = mem_layout_array2[i].block_size;
		bank->sectors[num_blocks].is_erased = -1;
		bank->sectors[num_blocks].is_protected = 1; /* protection bits likely set by hardware */
		mpc57xx_info->block_bits_lmlr[num_blocks] = mem_layout_array2[i].lmlr_bit;
		mpc57xx_info->block_bits_hlr[num_blocks] = mem_layout_array2[i].hlr_bit;
		mpc57xx_info->register_base[num_blocks++] = mpc57xx_flash_regs[2]; /* base register address per block */
	}

	return ERROR_OK;
}

/* flash bank mpc57xx 0 0 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(mpc57xx_flash_bank_command)
{
	struct mpc57xx_flash_bank *mpc57xx_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	mpc57xx_info = malloc(sizeof(struct mpc57xx_flash_bank));
	bank->driver_priv = mpc57xx_info;

	mpc57xx_info->probed = 1; /* ensure probe does not occur */

	/*printf("ARGV[6] = %s\n", CMD_ARGV[6]);*/
	/* varies per bank - FIXME */
	/*mpc57xx_info->register_base = 0xc3f80000;*/
	/* set up other bits perhaps */

	mpc57xx_build_block_list(bank);

	return ERROR_OK;
}

static int mpc57xx_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	/*struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;*/

	/*uint32_t config0_address;
	uint32_t devcfg0;*/
	int s;
	int num_pages;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

#if 0
	config0_address = MPC57XX_DEVCFG0;

	target_read_u32(target, config0_address, &devcfg0);

	if ((devcfg0 & (1 << 28)) == 0) /* code protect bit */
		num_pages = 0xffff;			/* All pages protected */
	else if (Virt2Phys(bank->base) == MPC57XX_PHYS_BOOT_FLASH) {
		if (devcfg0 & (1 << 24))
			num_pages = 0;			/* All pages unprotected */
		else
			num_pages = 0xffff;		/* All pages protected */
	} else {
		/* pgm flash */
		num_pages = (~devcfg0 >> 12) & 0xff;
	}
#endif
	num_pages = 0xffff; /* FIXME */

	for (s = 0; s < bank->num_sectors && s < num_pages; s++)
		bank->sectors[s].is_protected = 1;
	for (; s < bank->num_sectors; s++)
		bank->sectors[s].is_protected = 0;

	return ERROR_OK;
}

static int mpc57xx_unlock_block(struct flash_bank *bank, uint32_t block_num)
{
	struct target *target = bank->target;
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	int retval;
	uint32_t val;
	uint32_t reg_base;

	reg_base = mpc57xx_info->register_base[block_num];

	/* See if this flash block uses HLR and unlock */
	if (mpc57xx_info->block_bits_hlr[block_num]) {
		retval = target_read_memory(target, reg_base + MPC57XX_HLR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Original HLR= 0x%08x\n", val);
		if ((val & 0x80000000) == 0) {
			/* unlock password */
			retval = target_write_u32(target, reg_base + MPC57XX_HLR_OFFSET, MPC57XX_HLR_HBE_PASS);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_memory(target, reg_base + MPC57XX_HLR_OFFSET, 4, 1,
					(uint8_t *)&val);
			val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
			printf("Now HLR= 0x%08x\n", val);
		}
		val &= ~mpc57xx_info->block_bits_hlr[block_num]; /* Set appropriate bits to zero to unlock */
		retval = target_write_u32(target, reg_base + MPC57XX_HLR_OFFSET, val);
		if (retval != ERROR_OK)
			return retval;

		retval = target_read_memory(target, reg_base + MPC57XX_HLR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Now HLR= 0x%08x\n", val);
	}

	/* See if this flash block uses LMLR and unlock */
	if (mpc57xx_info->block_bits_lmlr[block_num]) {
		retval = target_read_memory(target, reg_base + MPC57XX_LMLR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Original LMLR= 0x%08x\n", val);
		if ((val & 0x80000000) == 0) {
			/* unlock password */
			retval = target_write_u32(target, reg_base + MPC57XX_LMLR_OFFSET, MPC57XX_LMLR_LME_PASS);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_memory(target, reg_base + MPC57XX_LMLR_OFFSET, 4, 1,
					(uint8_t *)&val);
			val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
			printf("Now LMLR= 0x%08x\n", val);
		}
		val &= ~mpc57xx_info->block_bits_lmlr[block_num]; /* Set appropriate bits to zero to unlock */
		retval = target_write_u32(target, reg_base + MPC57XX_LMLR_OFFSET, val);
		if (retval != ERROR_OK)
			return retval;

		retval = target_read_memory(target, reg_base + MPC57XX_LMLR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Now LMLR= 0x%08x\n", val);

		/* Now do same again for secondary reg */

		retval = target_read_memory(target, reg_base + MPC57XX_SLMLR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Original SLMLR= 0x%08x\n", val);
		if ((val & 0x80000000) == 0) {
			/* unlock password */
			retval = target_write_u32(target, reg_base + MPC57XX_SLMLR_OFFSET, MPC57XX_SLMLR_SLE_PASS);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_memory(target, reg_base + MPC57XX_SLMLR_OFFSET, 4, 1,
					(uint8_t *)&val);
			val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
			printf("Now SLMLR= 0x%08x\n", val);
		}
		val &= ~mpc57xx_info->block_bits_lmlr[block_num]; /* Set appropriate bits to zero to unlock */
		retval = target_write_u32(target, reg_base + MPC57XX_SLMLR_OFFSET, val);
		if (retval != ERROR_OK)
			return retval;

		retval = target_read_memory(target, reg_base + MPC57XX_SLMLR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Now SLMLR= 0x%08x\n", val);
	}

	return ERROR_OK;
}

static int mpc57xx_flash_set_sr(struct flash_bank *bank, uint32_t block_num)
{
	struct target *target = bank->target;
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	int retval;
	uint32_t val;
	uint32_t reg_base;

	reg_base = mpc57xx_info->register_base[block_num];

	/* See if this flash block uses HLR / HSR */
	if (mpc57xx_info->block_bits_hlr[block_num]) {
		retval = target_read_memory(target, reg_base + MPC57XX_HSR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Original HSR= 0x%08x\n", val);
		val |= mpc57xx_info->block_bits_hlr[block_num]; /* Set appropriate bits to one */
		retval = target_write_u32(target, reg_base + MPC57XX_HSR_OFFSET, val);
		if (retval != ERROR_OK)
			return retval;

		retval = target_read_memory(target, reg_base + MPC57XX_HSR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Now HLR= 0x%08x\n", val);
	}

	/* See if this flash block uses LMLR / LMSR */
	if (mpc57xx_info->block_bits_lmlr[block_num]) {
		retval = target_read_memory(target, reg_base + MPC57XX_LMSR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Original LMSR= 0x%08x\n", val);
		val |= mpc57xx_info->block_bits_lmlr[block_num]; /* Set appropriate bits to one */
		retval = target_write_u32(target, reg_base + MPC57XX_LMSR_OFFSET, val);
		if (retval != ERROR_OK)
			return retval;

		retval = target_read_memory(target, reg_base + MPC57XX_LMSR_OFFSET, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		printf("Now LMSR= 0x%08x\n", val);
	}

	return ERROR_OK;
}

static int mpc57xx_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	int retval, retry;
	uint32_t val;
	uint32_t reg_base;
	int block_num;

	printf("Got a call to erase!\n");

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (block_num = first; block_num <= last; block_num++) {

		reg_base = mpc57xx_info->register_base[block_num];

		retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000000); /* Clear MCR */
		if (retval != ERROR_OK)
			return retval;

		retval = mpc57xx_unlock_block(bank, block_num);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000004); /* select ERS operation */
		if (retval != ERROR_OK)
			return retval;

		retval = mpc57xx_flash_set_sr(bank, block_num); /* Set LMSR or HSR */
		if (retval != ERROR_OK)
			return retval;

		/* write something to the first address in that flash bank */
		retval = target_write_u32(target, bank->sectors[block_num].offset, 0xffffffff);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000005); /* select ERS+EHV operation */
		if (retval != ERROR_OK)
			return retval;

		/* Wait for Done=1 */
		retry = 100; /* arbitrary limit */
		val = 0;
		while (retry && ((val & 0x00000400) == 0)) {
			retval = target_read_memory(target, reg_base + MPC57XX_MCR_OFFSET, 4, 1,
					(uint8_t *)&val);
			val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
			if (retval != ERROR_OK)
				return retval;
			retry--;
		}

		retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000004); /* clear EHV */
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000000); /* clear ERS */
		if (retval != ERROR_OK)
			return retval;

		if ((val & 0x00000200) == 0) {
			LOG_ERROR("Received error on flash erase");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int mpc57xx_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

/* Determines sector details give a flash address */
static int mpc57xx_get_sect(struct flash_bank *bank, uint32_t address, uint32_t *start, uint32_t *end)
{
	int i, block_num;
	uint32_t sec_start, sec_end;

	sec_start = 0;
	sec_end = 0;

	/* Validate address */
	block_num = -1;
	for (i = 0; i < bank->num_sectors; i++) {
		sec_start = bank->sectors[i].offset;
		sec_end = sec_start + bank->sectors[i].size;

		/* check if destination falls within the current sector */
		if ((address >= sec_start) && (address < sec_end)) {
			block_num = i;
			break;
		}
	}

	*start = sec_start;
	*end = sec_end - 1;

	return block_num;
}

static int mpc57xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_written = 0;
	uint32_t bytes_remaining = (count & 0x00000007);
	uint32_t address = bank->base + offset;
	int retval, retry;
	uint32_t val;
	uint32_t reg_base, cur_sec_start, cur_sec_end;
	int block_num, tail_end = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("writing to flash at address 0x%08" PRIx32 " at offset 0x%8.8" PRIx32
			" count: 0x%8.8" PRIx32 "", bank->base, offset, count);

	if (offset & 0x7) {
		LOG_WARNING("offset 0x%" PRIx32 "breaks required 8-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		/* Could workaround this, but not presently. */
	}

	printf("Entered write_block code!\n");

	block_num = -1;
	cur_sec_start = 1; /* intentionally invalid */
	cur_sec_end = 0;
	reg_base = 0;

	while (words_remaining) {
		uint32_t value, value2;

		if ((address < cur_sec_start) || (address > cur_sec_end)) {
			printf("Address = 0x%08x\n", address);
			if (block_num >= 0) {
				/* Ensure MCR for existing sector is cleared */
				/* reg_base will have been set */
				retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000000);
				if (retval != ERROR_OK)
					return retval;
			}
			block_num = mpc57xx_get_sect(bank, address, &cur_sec_start, &cur_sec_end);

			if (block_num < 0)
				return ERROR_FLASH_DST_OUT_OF_BANK;

			reg_base = mpc57xx_info->register_base[block_num];

			retval = mpc57xx_unlock_block(bank, block_num);
			if (retval != ERROR_OK)
				return retval;

			/* Ensure MCR for new sector is cleared */
			retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000000);
			if (retval != ERROR_OK)
				return retval;

			retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000010); /* Select PGM */
			if (retval != ERROR_OK)
				return retval;
		}

		if (tail_end) {
			uint8_t byte_buf[8];
			LOG_INFO("Tail end data flash, not sure this is correct. Please check.");
			printf("Handling the stragglers %d\n", bytes_remaining);
			memset(byte_buf, 0, 8);
			memcpy(byte_buf, buffer + bytes_written, sizeof(uint32_t));
			value = be_to_h_u32((uint8_t *)&byte_buf[0]); /* read from buffer BE */
			value2 = be_to_h_u32((uint8_t *)&byte_buf[4]);
			words_remaining = 2;
			bytes_remaining = 0;
		} else {
			memcpy(&value, buffer + bytes_written, sizeof(uint32_t));
			memcpy(&value2, buffer + bytes_written + 4, sizeof(uint32_t));
			value = be_to_h_u32((uint8_t *)&value); /* swap ends again */
			value2 = be_to_h_u32((uint8_t *)&value2); /* swap ends again */
		}

		/* Skip writes if the data is the same as erased flash value */
		if ((value != 0xffffffff) && (value2 != 0xffffffff)) {
			retval = target_write_u32(target, address, value); /* Write first 32bits */
			if (retval != ERROR_OK)
				return retval;

			retval = target_write_u32(target, address + 4, value2); /* Write second 32bits */
			if (retval != ERROR_OK)
				return retval;

			retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000011); /* Set EHV */
			if (retval != ERROR_OK)
				return retval;

			/* Wait for Done=1 */
			retry = 100; /* arbitrary limit */
			val = 0;
			while (retry && ((val & 0x00000400) == 0)) {
				retval = target_read_memory(target, reg_base + MPC57XX_MCR_OFFSET, 4, 1,
						(uint8_t *)&val);
				val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
				if (retval != ERROR_OK)
					return retval;
				retry--;
			}

			/* Clear EHV for each 64bit programmed */
			retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000010);
			if (retval != ERROR_OK)
				return retval;

			if ((val & 0x00000200) == 0) {
				LOG_ERROR("Received error on flash erase");
				return ERROR_FAIL;
			}
		}
		if ((words_remaining <= 2) && (bytes_remaining))
			tail_end = 1;
		bytes_written += 8;
		words_remaining -= 2;
		address += 8;
	}

	if (block_num >= 0) {
		/* Ensure MCR for existing sector is cleared */
		/* reg_base will have been set */
		retval = target_write_u32(target, reg_base + MPC57XX_MCR_OFFSET, 0x00000000);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mpc57xx_probe(struct flash_bank *bank)
{
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;

	if (mpc57xx_info->probed == 0) {
		printf("Probe bypassed.\n");
		LOG_DEBUG("Probe bypassed");
	}

	return ERROR_OK;
}

static int mpc57xx_auto_probe(struct flash_bank *bank)
{
	struct mpc57xx_flash_bank *mpc57xx_info = bank->driver_priv;
	if (mpc57xx_info->probed)
		return ERROR_OK;
	return mpc57xx_probe(bank);
}

static int mpc57xx_info(struct flash_bank *bank, char *buf, int buf_size)
{
	/*struct target *target = bank->target;
	struct mpc5634_common *mpc5634 = target->arch_info;
	struct mpc5634_jtag *jtag_info = &mpc5634->jtag;*/
	uint32_t device_id;
	int printed = 0, i;

	/*device_id = jtag_info->idcode;*/
	device_id = 0x87654321;

	if (((device_id >> 1) & 0x7ff) != MPC57XX_MANUF_ID) {
		snprintf(buf, buf_size,
				 "Cannot identify target as a MPC57XX family (manufacturer 0x%03d != 0x%03d)\n",
				 (unsigned)((device_id >> 1) & 0x7ff),
				 MPC57XX_MANUF_ID);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	for (i = 0; mpc57xx_devs[i].name != NULL; i++) {
		if (mpc57xx_devs[i].devid == (device_id & 0x0fffffff)) {
			printed = snprintf(buf, buf_size, "MPC57XX%s", mpc57xx_devs[i].name);
			break;
		}
	}

	if (mpc57xx_devs[i].name == NULL)
		printed = snprintf(buf, buf_size, "Unknown");

	buf += printed;
	buf_size -= printed;
	snprintf(buf, buf_size, " Ver: 0x%02x",
			(unsigned)((device_id >> 28) & 0xf));

	return ERROR_OK;
}

#if 0
COMMAND_HANDLER(mpc57xx_handle_pgm_word_command)
{
	uint32_t address, value;
	int status, res;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 2, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (address < bank->base || address >= (bank->base + bank->size)) {
		command_print(CMD_CTX, "flash address '%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	res = ERROR_OK;
	status = mpc57xx_write_word(bank, address, value);
	if (status & NVMCON_NVMERR)
		res = ERROR_FLASH_OPERATION_FAILED;
	if (status & NVMCON_LVDERR)
		res = ERROR_FLASH_OPERATION_FAILED;

	if (res == ERROR_OK)
		command_print(CMD_CTX, "mpc57xx pgm word complete");
	else
		command_print(CMD_CTX, "mpc57xx pgm word failed (status = 0x%x)", status);

	return ERROR_OK;
}
#endif

COMMAND_HANDLER(mpc57xx_handle_unlock_command)
{
	/*uint32_t mchip_cmd;
	struct target *target = NULL;
	struct mips_m4k_common *mips_m4k;
	struct mips_ejtag *ejtag_info;
	int timeout = 10;*/

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "mpc57xx unlock <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;
printf("Not handled!\n");
return ERROR_FAIL;
#if 0
	not handled yet
	target = bank->target;
	mpc5634 = target_to_mpc5634(target);
	jtag_info = &mpc5634->mpc5634.jtag_info;

	/* we have to use the MTAP to perform a full erase */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_MTAP);
	mips_ejtag_set_instr(ejtag_info, MTAP_COMMAND);

	/* first check status of device */
	mchip_cmd = MCHP_STATUS;
	mips_ejtag_drscan_8(ejtag_info, &mchip_cmd);
	if (mchip_cmd & (1 << 7)) {
		/* device is not locked */
		command_print(CMD_CTX, "mpc57xx is already unlocked, erasing anyway");
	}

	/* unlock/erase device */
	mips_ejtag_drscan_8_out(ejtag_info, MCHP_ASERT_RST);
	jtag_add_sleep(200);

	mips_ejtag_drscan_8_out(ejtag_info, MCHP_ERASE);

	do {
		mchip_cmd = MCHP_STATUS;
		mips_ejtag_drscan_8(ejtag_info, &mchip_cmd);
		if (timeout-- == 0) {
			LOG_DEBUG("timeout waiting for unlock: 0x%" PRIx32 "", mchip_cmd);
			break;
		}
		alive_sleep(1);
	} while ((mchip_cmd & (1 << 2)) || (!(mchip_cmd & (1 << 3))));

	mips_ejtag_drscan_8_out(ejtag_info, MCHP_DE_ASSERT_RST);

	/* select ejtag tap */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_ETAP);

	command_print(CMD_CTX, "mpc57xx unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
#endif
}

static const struct command_registration mpc57xx_exec_command_handlers[] = {
/*	{
		.name = "pgm_word",
		.usage = "<addr> <value> <bank>",
		.handler = mpc57xx_handle_pgm_word_command,
		.mode = COMMAND_EXEC,
		.help = "program a word",
	},*/
	{
		.name = "unlock",
		.handler = mpc57xx_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "[bank_id]",
		.help = "Unlock/Erase entire device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration mpc57xx_command_handlers[] = {
	{
		.name = "mpc57xx",
		.mode = COMMAND_ANY,
		.help = "mpc57xx flash command group",
		.usage = "",
		.chain = mpc57xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver mpc57xx_flash = {
	.name = "mpc57xx",
	.commands = mpc57xx_command_handlers,
	.flash_bank_command = mpc57xx_flash_bank_command,
	.erase = mpc57xx_erase,
	.protect = mpc57xx_protect,
	.write = mpc57xx_write,
	.read = default_flash_read,
	.probe = mpc57xx_probe,
	.auto_probe = mpc57xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mpc57xx_protect_check,
	.info = mpc57xx_info,
};
