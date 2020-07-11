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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "imp.h"
#include <target/algorithm.h>
#include <target/mips32.h>
#include <target/mips_m4k.h>

#define PIC32MX_MANUF_ID	0x029

/* pic32mx memory locations */

#define PIC32MX_PHYS_RAM			0x00000000
#define PIC32MX_PHYS_PGM_FLASH		0x1D000000
#define PIC32MX_PHYS_PERIPHERALS	0x1F800000
#define PIC32MX_PHYS_BOOT_FLASH		0x1FC00000

/*
 * Translate Virtual and Physical addresses.
 * Note: These macros only work for KSEG0/KSEG1 addresses.
 */

#define Virt2Phys(v)	((v) & 0x1FFFFFFF)

/* pic32mx configuration register locations */

#define PIC32MX_DEVCFG0_1xx_2xx	0xBFC00BFC
#define PIC32MX_DEVCFG0		0xBFC02FFC
#define PIC32MX_DEVCFG1		0xBFC02FF8
#define PIC32MX_DEVCFG2		0xBFC02FF4
#define PIC32MX_DEVCFG3		0xBFC02FF0
#define PIC32MX_DEVID		0xBF80F220

#define PIC32MX_BMXPFMSZ	0xBF882060
#define PIC32MX_BMXBOOTSZ	0xBF882070
#define PIC32MX_BMXDRMSZ	0xBF882040

/* pic32mx flash controller register locations */

#define PIC32MX_NVMCON		0xBF80F400
#define PIC32MX_NVMCONCLR	0xBF80F404
#define PIC32MX_NVMCONSET	0xBF80F408
#define PIC32MX_NVMCONINV	0xBF80F40C
#define NVMCON_NVMWR		(1 << 15)
#define NVMCON_NVMWREN		(1 << 14)
#define NVMCON_NVMERR		(1 << 13)
#define NVMCON_LVDERR		(1 << 12)
#define NVMCON_LVDSTAT		(1 << 11)
#define NVMCON_OP_PFM_ERASE		0x5
#define NVMCON_OP_PAGE_ERASE	0x4
#define NVMCON_OP_ROW_PROG		0x3
#define NVMCON_OP_WORD_PROG		0x1
#define NVMCON_OP_NOP			0x0

#define PIC32MX_NVMKEY		0xBF80F410
#define PIC32MX_NVMADDR		0xBF80F420
#define PIC32MX_NVMADDRCLR	0xBF80F424
#define PIC32MX_NVMADDRSET	0xBF80F428
#define PIC32MX_NVMADDRINV	0xBF80F42C
#define PIC32MX_NVMDATA		0xBF80F430
#define PIC32MX_NVMSRCADDR	0xBF80F440

/* flash unlock keys */

#define NVMKEY1			0xAA996655
#define NVMKEY2			0x556699AA

#define MX_1xx_2xx			1	/* PIC32mx1xx/2xx */
#define MX_17x_27x			2	/* PIC32mx17x/27x */

struct pic32mx_flash_bank {
	bool probed;
	int dev_type;		/* Default 0. 1 for Pic32MX1XX/2XX variant */
};

/*
 * DEVID values as per PIC32MX Flash Programming Specification Rev N
 */

static const struct pic32mx_devs_s {
	uint32_t devid;
	const char *name;
} pic32mx_devs[] = {
	{0x04A07053, "110F016B"},
	{0x04A09053, "110F016C"},
	{0x04A0B053, "110F016D"},
	{0x04A06053, "120F032B"},
	{0x04A08053, "120F032C"},
	{0x04A0A053, "120F032D"},
	{0x04D07053, "130F064B"},
	{0x04D09053, "130F064C"},
	{0x04D0B053, "130F064D"},
	{0x04D06053, "150F128B"},
	{0x04D08053, "150F128C"},
	{0x04D0A053, "150F128D"},
	{0x06610053, "170F256B"},
	{0x0661A053, "170F256D"},
	{0x04A01053, "210F016B"},
	{0x04A03053, "210F016C"},
	{0x04A05053, "210F016D"},
	{0x04A00053, "220F032B"},
	{0x04A02053, "220F032C"},
	{0x04A04053, "220F032D"},
	{0x04D01053, "230F064B"},
	{0x04D03053, "230F064C"},
	{0x04D05053, "230F064D"},
	{0x04D00053, "250F128B"},
	{0x04D02053, "250F128C"},
	{0x04D04053, "250F128D"},
	{0x06600053, "270F256B"},
	{0x0660A053, "270F256D"},
	{0x05600053, "330F064H"},
	{0x05601053, "330F064L"},
	{0x05602053, "430F064H"},
	{0x05603053, "430F064L"},
	{0x0570C053, "350F128H"},
	{0x0570D053, "350F128L"},
	{0x0570E053, "450F128H"},
	{0x0570F053, "450F128L"},
	{0x05704053, "350F256H"},
	{0x05705053, "350F256L"},
	{0x05706053, "450F256H"},
	{0x05707053, "450F256L"},
	{0x05808053, "370F512H"},
	{0x05809053, "370F512L"},
	{0x0580A053, "470F512H"},
	{0x0580B053, "470F512L"},
	{0x00938053, "360F512L"},
	{0x00934053, "360F256L"},
	{0x0092D053, "340F128L"},
	{0x0092A053, "320F128L"},
	{0x00916053, "340F512H"},
	{0x00912053, "340F256H"},
	{0x0090D053, "340F128H"},
	{0x0090A053, "320F128H"},
	{0x00906053, "320F064H"},
	{0x00902053, "320F032H"},
	{0x00978053, "460F512L"},
	{0x00974053, "460F256L"},
	{0x0096D053, "440F128L"},
	{0x00952053, "440F256H"},
	{0x00956053, "440F512H"},
	{0x0094D053, "440F128H"},
	{0x00942053, "420F032H"},
	{0x04307053, "795F512L"},
	{0x0430E053, "795F512H"},
	{0x04306053, "775F512L"},
	{0x0430D053, "775F512H"},
	{0x04312053, "775F256L"},
	{0x04303053, "775F256H"},
	{0x04417053, "764F128L"},
	{0x0440B053, "764F128H"},
	{0x04341053, "695F512L"},
	{0x04325053, "695F512H"},
	{0x04311053, "675F512L"},
	{0x0430C053, "675F512H"},
	{0x04305053, "675F256L"},
	{0x0430B053, "675F256H"},
	{0x04413053, "664F128L"},
	{0x04407053, "664F128H"},
	{0x04411053, "664F064L"},
	{0x04405053, "664F064H"},
	{0x0430F053, "575F512L"},
	{0x04309053, "575F512H"},
	{0x04333053, "575F256L"},
	{0x04317053, "575F256H"},
	{0x0440F053, "564F128L"},
	{0x04403053, "564F128H"},
	{0x0440D053, "564F064L"},
	{0x04401053, "564F064H"},
	{0x04400053, "534F064H"},
	{0x0440C053, "534F064L"},
	{0x00000000, NULL}
};

/* flash bank pic32mx <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(pic32mx_flash_bank_command)
{
	struct pic32mx_flash_bank *pic32mx_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pic32mx_info = malloc(sizeof(struct pic32mx_flash_bank));
	bank->driver_priv = pic32mx_info;

	pic32mx_info->probed = false;
	pic32mx_info->dev_type = 0;

	return ERROR_OK;
}

static uint32_t pic32mx_get_flash_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;

	target_read_u32(target, PIC32MX_NVMCON, &status);

	return status;
}

static uint32_t pic32mx_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;

	/* wait for busy to clear */
	while (((status = pic32mx_get_flash_status(bank)) & NVMCON_NVMWR) && (timeout-- > 0)) {
		LOG_DEBUG("status: 0x%" PRIx32, status);
		alive_sleep(1);
	}
	if (timeout <= 0)
		LOG_DEBUG("timeout: status: 0x%" PRIx32, status);

	return status;
}

static int pic32mx_nvm_exec(struct flash_bank *bank, uint32_t op, uint32_t timeout)
{
	struct target *target = bank->target;
	uint32_t status;

	target_write_u32(target, PIC32MX_NVMCON, NVMCON_NVMWREN | op);

	/* unlock flash registers */
	target_write_u32(target, PIC32MX_NVMKEY, NVMKEY1);
	target_write_u32(target, PIC32MX_NVMKEY, NVMKEY2);

	/* start operation */
	target_write_u32(target, PIC32MX_NVMCONSET, NVMCON_NVMWR);

	status = pic32mx_wait_status_busy(bank, timeout);

	/* lock flash registers */
	target_write_u32(target, PIC32MX_NVMCONCLR, NVMCON_NVMWREN);

	return status;
}

static int pic32mx_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct pic32mx_flash_bank *pic32mx_info = bank->driver_priv;

	uint32_t config0_address;
	uint32_t devcfg0;
	unsigned int s, num_pages;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	switch (pic32mx_info->dev_type) {
	case	MX_1xx_2xx:
	case	MX_17x_27x:
		config0_address = PIC32MX_DEVCFG0_1xx_2xx;
		break;
	default:
		config0_address = PIC32MX_DEVCFG0;
		break;
	}

	target_read_u32(target, config0_address, &devcfg0);

	if ((devcfg0 & (1 << 28)) == 0) /* code protect bit */
		num_pages = 0xffff;			/* All pages protected */
	else if (Virt2Phys(bank->base) == PIC32MX_PHYS_BOOT_FLASH) {
		if (devcfg0 & (1 << 24))
			num_pages = 0;			/* All pages unprotected */
		else
			num_pages = 0xffff;		/* All pages protected */
	} else {
		/* pgm flash */
		switch (pic32mx_info->dev_type) {
		case	MX_1xx_2xx:
			num_pages = (~devcfg0 >> 10) & 0x7f;
			break;
		case	MX_17x_27x:
			num_pages = (~devcfg0 >> 10) & 0x1ff;
			break;
		default:
			num_pages = (~devcfg0 >> 12) & 0xff;
			break;
		}
	}

	for (s = 0; s < bank->num_sectors && s < num_pages; s++)
		bank->sectors[s].is_protected = 1;
	for (; s < bank->num_sectors; s++)
		bank->sectors[s].is_protected = 0;

	return ERROR_OK;
}

static int pic32mx_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	uint32_t status;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1))
		&& (Virt2Phys(bank->base) == PIC32MX_PHYS_PGM_FLASH)) {
		/* this will only erase the Program Flash (PFM), not the Boot Flash (BFM)
		 * we need to use the MTAP to perform a full erase */
		LOG_DEBUG("Erasing entire program flash");
		status = pic32mx_nvm_exec(bank, NVMCON_OP_PFM_ERASE, 50);
		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		return ERROR_OK;
	}

	for (unsigned int i = first; i <= last; i++) {
		target_write_u32(target, PIC32MX_NVMADDR, Virt2Phys(bank->base + bank->sectors[i].offset));

		status = pic32mx_nvm_exec(bank, NVMCON_OP_PAGE_ERASE, 10);

		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int pic32mx_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

/* see contrib/loaders/flash/pic32mx.s for src */

static uint32_t pic32mx_flash_write_code[] = {
					/* write: */
	0x3C08AA99,		/* lui $t0, 0xaa99 */
	0x35086655,		/* ori $t0, 0x6655 */
	0x3C095566,		/* lui $t1, 0x5566 */
	0x352999AA,		/* ori $t1, 0x99aa */
	0x3C0ABF80,		/* lui $t2, 0xbf80 */
	0x354AF400,		/* ori $t2, 0xf400 */
	0x340B4003,		/* ori $t3, $zero, 0x4003 */
	0x340C8000,		/* ori $t4, $zero, 0x8000 */
					/* write_row: */
	0x2CD30080,		/* sltiu $s3, $a2, 128 */
	0x16600008,		/* bne $s3, $zero, write_word */
	0x340D4000,		/* ori $t5, $zero, 0x4000 */
	0xAD450020,		/* sw $a1, 32($t2) */
	0xAD440040,		/* sw $a0, 64($t2) */
	0x04110016,		/* bal progflash */
	0x24840200,		/* addiu $a0, $a0, 512 */
	0x24A50200,		/* addiu $a1, $a1, 512 */
	0x1000FFF7,		/* beq $zero, $zero, write_row */
	0x24C6FF80,		/* addiu $a2, $a2, -128 */
					/* write_word: */
	0x3C15A000,		/* lui $s5, 0xa000 */
	0x36B50000,		/* ori $s5, $s5, 0x0 */
	0x00952025,		/* or $a0, $a0, $s5 */
	0x10000008,		/* beq $zero, $zero, next_word */
	0x340B4001,		/* ori $t3, $zero, 0x4001 */
					/* prog_word: */
	0x8C940000,		/* lw $s4, 0($a0) */
	0xAD540030,		/* sw $s4, 48($t2) */
	0xAD450020,		/* sw $a1, 32($t2) */
	0x04110009,		/* bal progflash */
	0x24840004,		/* addiu $a0, $a0, 4 */
	0x24A50004,		/* addiu $a1, $a1, 4 */
	0x24C6FFFF,		/* addiu $a2, $a2, -1 */
					/* next_word: */
	0x14C0FFF8,		/* bne $a2, $zero, prog_word */
	0x00000000,		/* nop */
					/* done: */
	0x10000002,		/* beq $zero, $zero, exit */
	0x24040000,		/* addiu $a0, $zero, 0 */
					/* error: */
	0x26240000,		/* addiu $a0, $s1, 0 */
					/* exit: */
	0x7000003F,		/* sdbbp */
					/* progflash: */
	0xAD4B0000,		/* sw $t3, 0($t2) */
	0xAD480010,		/* sw $t0, 16($t2) */
	0xAD490010,		/* sw $t1, 16($t2) */
	0xAD4C0008,		/* sw $t4, 8($t2) */
					/* waitflash: */
	0x8D500000,		/* lw $s0, 0($t2) */
	0x020C8024,		/* and $s0, $s0, $t4 */
	0x1600FFFD,		/* bne $s0, $zero, waitflash */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x8D510000,		/* lw $s1, 0($t2) */
	0x30113000,		/* andi $s1, $zero, 0x3000 */
	0x1620FFEF,		/* bne $s1, $zero, error */
	0xAD4D0004,		/* sw $t5, 4($t2) */
	0x03E00008,		/* jr $ra */
	0x00000000		/* nop */
};

static int pic32mx_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[3];
	uint32_t row_size;
	int retval = ERROR_OK;

	struct pic32mx_flash_bank *pic32mx_info = bank->driver_priv;
	struct mips32_algorithm mips32_info;

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(pic32mx_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Change values for counters and row size, depending on variant */
	switch (pic32mx_info->dev_type) {
	case	MX_1xx_2xx:
	case	MX_17x_27x:
		/* 128 byte row */
		pic32mx_flash_write_code[8] = 0x2CD30020;
		pic32mx_flash_write_code[14] = 0x24840080;
		pic32mx_flash_write_code[15] = 0x24A50080;
		pic32mx_flash_write_code[17] = 0x24C6FFE0;
		row_size = 128;
		break;
	default:
		/* 512 byte row */
		pic32mx_flash_write_code[8] = 0x2CD30080;
		pic32mx_flash_write_code[14] = 0x24840200;
		pic32mx_flash_write_code[15] = 0x24A50200;
		pic32mx_flash_write_code[17] = 0x24C6FF80;
		row_size = 512;
		break;
	}

	uint8_t code[sizeof(pic32mx_flash_write_code)];
	target_buffer_set_u32_array(target, code, ARRAY_SIZE(pic32mx_flash_write_code),
			pic32mx_flash_write_code);
	retval = target_write_buffer(target, write_algorithm->address, sizeof(code), code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	mips32_info.common_magic = MIPS32_COMMON_MAGIC;
	mips32_info.isa_mode = MIPS32_ISA_MIPS32;

	init_reg_param(&reg_params[0], "r4", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r5", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r6", 32, PARAM_OUT);

	int row_offset = offset % row_size;
	uint8_t *new_buffer = NULL;
	if (row_offset && (count >= (row_size / 4))) {
		new_buffer = malloc(buffer_size);
		if (new_buffer == NULL) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		memset(new_buffer,  0xff, row_offset);
		address -= row_offset;
	} else
		row_offset = 0;

	while (count > 0) {
		uint32_t status;
		uint32_t thisrun_count;

		if (row_offset) {
			thisrun_count = (count > ((buffer_size - row_offset) / 4)) ?
				((buffer_size - row_offset) / 4) : count;

			memcpy(new_buffer + row_offset, buffer, thisrun_count * 4);

			retval = target_write_buffer(target, source->address,
				row_offset + thisrun_count * 4, new_buffer);
			if (retval != ERROR_OK)
				break;
		} else {
			thisrun_count = (count > (buffer_size / 4)) ?
					(buffer_size / 4) : count;

			retval = target_write_buffer(target, source->address,
					thisrun_count * 4, buffer);
			if (retval != ERROR_OK)
				break;
		}

		buf_set_u32(reg_params[0].value, 0, 32, Virt2Phys(source->address));
		buf_set_u32(reg_params[1].value, 0, 32, Virt2Phys(address));
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count + row_offset / 4);

		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
				write_algorithm->address,
				0, 10000, &mips32_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("error executing pic32mx flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		status = buf_get_u32(reg_params[0].value, 0, 32);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 4;
		address += thisrun_count * 4;
		count -= thisrun_count;
		if (row_offset) {
			address += row_offset;
			row_offset = 0;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	if (new_buffer != NULL)
		free(new_buffer);
	return retval;
}

static int pic32mx_write_word(struct flash_bank *bank, uint32_t address, uint32_t word)
{
	struct target *target = bank->target;

	target_write_u32(target, PIC32MX_NVMADDR, Virt2Phys(address));
	target_write_u32(target, PIC32MX_NVMDATA, word);

	return pic32mx_nvm_exec(bank, NVMCON_OP_WORD_PROG, 5);
}

static int pic32mx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_remaining = (count & 0x00000003);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	uint32_t status;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("writing to flash at address " TARGET_ADDR_FMT " at offset 0x%8.8" PRIx32
			" count: 0x%8.8" PRIx32 "", bank->base, offset, count);

	if (offset & 0x3) {
		LOG_WARNING("offset 0x%" PRIx32 "breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* multiple words (4-byte) to be programmed? */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = pic32mx_write_block(bank, buffer, offset, words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			} else if (retval == ERROR_FLASH_OPERATION_FAILED) {
				LOG_ERROR("flash writing failed");
				return retval;
			}
		} else {
			buffer += words_remaining * 4;
			address += words_remaining * 4;
			words_remaining = 0;
		}
	}

	while (words_remaining > 0) {
		uint32_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint32_t));

		status = pic32mx_write_word(bank, address, value);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bytes_written += 4;
		words_remaining--;
		address += 4;
	}

	if (bytes_remaining) {
		uint32_t value = 0xffffffff;
		memcpy(&value, buffer + bytes_written, bytes_remaining);

		status = pic32mx_write_word(bank, address, value);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int pic32mx_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct pic32mx_flash_bank *pic32mx_info = bank->driver_priv;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int i;
	uint32_t num_pages = 0;
	uint32_t device_id;
	int page_size;

	pic32mx_info->probed = false;

	device_id = ejtag_info->idcode;
	LOG_INFO("device id = 0x%08" PRIx32 " (manuf 0x%03x dev 0x%04x, ver 0x%02x)",
			  device_id,
			  (unsigned)((device_id >> 1) & 0x7ff),
			  (unsigned)((device_id >> 12) & 0xffff),
			  (unsigned)((device_id >> 28) & 0xf));

	if (((device_id >> 1) & 0x7ff) != PIC32MX_MANUF_ID) {
		LOG_WARNING("Cannot identify target as a PIC32MX family.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check for PIC32mx1xx/2xx */
	for (i = 0; pic32mx_devs[i].name != NULL; i++) {
		if (pic32mx_devs[i].devid == (device_id & 0x0fffffff)) {
			if ((pic32mx_devs[i].name[0] == '1') || (pic32mx_devs[i].name[0] == '2'))
				pic32mx_info->dev_type = (pic32mx_devs[i].name[1] == '7') ? MX_17x_27x : MX_1xx_2xx;
			break;
		}
	}

	switch (pic32mx_info->dev_type) {
	case	MX_1xx_2xx:
	case	MX_17x_27x:
		page_size = 1024;
		break;
	default:
		page_size = 4096;
		break;
	}

	if (Virt2Phys(bank->base) == PIC32MX_PHYS_BOOT_FLASH) {
		/* 0x1FC00000: Boot flash size */
#if 0
		/* for some reason this register returns 8k for the boot bank size
		 * this does not match the docs, so for now set the boot bank at a
		 * fixed 12k */
		if (target_read_u32(target, PIC32MX_BMXBOOTSZ, &num_pages) != ERROR_OK) {
			LOG_WARNING("PIC32MX flash size failed, probe inaccurate - assuming 12k flash");
			num_pages = (12 * 1024);
		}
#else
		/* fixed 12k boot bank - see comments above */
		switch (pic32mx_info->dev_type) {
		case	MX_1xx_2xx:
		case	MX_17x_27x:
			num_pages = (3 * 1024);
			break;
		default:
			num_pages = (12 * 1024);
			break;
		}
#endif
	} else {
		/* read the flash size from the device */
		if (target_read_u32(target, PIC32MX_BMXPFMSZ, &num_pages) != ERROR_OK) {
			switch (pic32mx_info->dev_type) {
			case	MX_1xx_2xx:
			case	MX_17x_27x:
				LOG_WARNING("PIC32MX flash size failed, probe inaccurate - assuming 32k flash");
				num_pages = (32 * 1024);
				break;
			default:
				LOG_WARNING("PIC32MX flash size failed, probe inaccurate - assuming 512k flash");
				num_pages = (512 * 1024);
				break;
			}
		}
	}

	LOG_INFO("flash size = %" PRId32 "kbytes", num_pages / 1024);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* calculate numbers of pages */
	num_pages /= page_size;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < (int)num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	pic32mx_info->probed = true;

	return ERROR_OK;
}

static int pic32mx_auto_probe(struct flash_bank *bank)
{
	struct pic32mx_flash_bank *pic32mx_info = bank->driver_priv;
	if (pic32mx_info->probed)
		return ERROR_OK;
	return pic32mx_probe(bank);
}

static int pic32mx_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	uint32_t device_id;
	int printed = 0, i;

	device_id = ejtag_info->idcode;

	if (((device_id >> 1) & 0x7ff) != PIC32MX_MANUF_ID) {
		snprintf(buf, buf_size,
				 "Cannot identify target as a PIC32MX family (manufacturer 0x%03x != 0x%03x)\n",
				 (unsigned)((device_id >> 1) & 0x7ff),
				 PIC32MX_MANUF_ID);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	for (i = 0; pic32mx_devs[i].name != NULL; i++) {
		if (pic32mx_devs[i].devid == (device_id & 0x0fffffff)) {
			printed = snprintf(buf, buf_size, "PIC32MX%s", pic32mx_devs[i].name);
			break;
		}
	}

	if (pic32mx_devs[i].name == NULL)
		printed = snprintf(buf, buf_size, "Unknown");

	buf += printed;
	buf_size -= printed;
	snprintf(buf, buf_size, " Ver: 0x%02x",
			(unsigned)((device_id >> 28) & 0xf));

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mx_handle_pgm_word_command)
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
		command_print(CMD, "flash address '%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	res = ERROR_OK;
	status = pic32mx_write_word(bank, address, value);
	if (status & NVMCON_NVMERR)
		res = ERROR_FLASH_OPERATION_FAILED;
	if (status & NVMCON_LVDERR)
		res = ERROR_FLASH_OPERATION_FAILED;

	if (res == ERROR_OK)
		command_print(CMD, "pic32mx pgm word complete");
	else
		command_print(CMD, "pic32mx pgm word failed (status = 0x%x)", status);

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mx_handle_unlock_command)
{
	struct target *target = NULL;
	struct mips_m4k_common *mips_m4k;
	struct mips_ejtag *ejtag_info;
	int timeout = 10;

	if (CMD_ARGC < 1) {
		command_print(CMD, "pic32mx unlock <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	mips_m4k = target_to_m4k(target);
	ejtag_info = &mips_m4k->mips32.ejtag_info;

	/* we have to use the MTAP to perform a full erase */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_MTAP);
	mips_ejtag_set_instr(ejtag_info, MTAP_COMMAND);

	/* first check status of device */
	uint8_t mchip_cmd = MCHP_STATUS;
	mips_ejtag_drscan_8(ejtag_info, &mchip_cmd);
	if (mchip_cmd & (1 << 7)) {
		/* device is not locked */
		command_print(CMD, "pic32mx is already unlocked, erasing anyway");
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

	command_print(CMD, "pic32mx unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

static const struct command_registration pic32mx_exec_command_handlers[] = {
	{
		.name = "pgm_word",
		.usage = "<addr> <value> <bank>",
		.handler = pic32mx_handle_pgm_word_command,
		.mode = COMMAND_EXEC,
		.help = "program a word",
	},
	{
		.name = "unlock",
		.handler = pic32mx_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "[bank_id]",
		.help = "Unlock/Erase entire device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pic32mx_command_handlers[] = {
	{
		.name = "pic32mx",
		.mode = COMMAND_ANY,
		.help = "pic32mx flash command group",
		.usage = "",
		.chain = pic32mx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver pic32mx_flash = {
	.name = "pic32mx",
	.commands = pic32mx_command_handlers,
	.flash_bank_command = pic32mx_flash_bank_command,
	.erase = pic32mx_erase,
	.protect = pic32mx_protect,
	.write = pic32mx_write,
	.read = default_flash_read,
	.probe = pic32mx_probe,
	.auto_probe = pic32mx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pic32mx_protect_check,
	.info = pic32mx_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
