/***************************************************************************
 *   Copyright (C) 2018 by Antoine Calando                                 *
 *   acalando@free.fr                                                      *
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

#include "log.h"
#include "imp.h"

#include <target/xpc56.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif


#define CFLASH0_BASE	0x0
#define CFLASH0_CONF	0xC3F88000

#define DFLASH0_BASE	0x800000
#define DFLASH0_CONF	0xC3F8C000

/* SPC564Bxx or SCP56ECxx
#define CFLASH1_BASE	0x180000
#define CFLASH1_CONF	0xC3FB8000
*/

/* xpc56 program functions */

FLASH_BANK_COMMAND_HANDLER(xpc56f_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/*
	struct xpc56f_flash_bank *xpc56f_info;
	xpc56f_info = malloc(sizeof(struct xpc56f_flash_bank));
	bank->driver_priv = xpc56f_info;
	xpc56f_info->probed = 0;
	*/

	return ERROR_OK;
}

static int xpc56f_protect_check(struct flash_bank *bank)
{
	LOG_INFO("%s", __func__);

	struct target *target = bank->target;
	uintptr_t conf = (uintptr_t)bank->driver_priv;

	uint32_t reg_lml = xpc56_read_u32(target, conf + 0x4);
	uint32_t reg_sll = xpc56_read_u32(target, conf + 0xC);

	for (int i = 0; i < bank->num_sectors; i++) {
		struct flash_sector *sect = bank->sectors + i;
		sect->is_protected = ((reg_lml | reg_sll) >> i) & 1;
	}
	return ERROR_OK;
}

static int xpc56f_protect(struct flash_bank *bank, int set, int first, int last)
{
	LOG_INFO("%s", __func__);

	int r = xpc56f_protect_check(bank);
	if (r != ERROR_OK)
		return r;

	set = !!set;
	assert(last < bank->num_sectors);

	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uintptr_t conf = (uintptr_t)bank->driver_priv;

	uint32_t change = 0;

	for (int i = first; i <= last; i++) {
		if (bank->sectors[i].is_protected != set) {
			change |= BIT(i);
			bank->sectors[i].is_protected = set;
		}
	}

	if (change) {
		/* todo: manage high address space */
		uint32_t lml = xpc56_read_u32(target, conf + 0x4);
		uint32_t sll = xpc56_read_u32(target, conf + 0xC);

		/* bug somewhere! WA here */
		lml = xpc56_read_u32(target, conf + 0x4);

		printf("protect change %08x %08x -- %04x\n", lml, sll, change);

		if ((lml & BIT(31)) == 0 || (sll & BIT(31)) == 0) {
			/* write passwords */
			xpc56_write_u32(target, conf + 0x4, 0xa1a11111);
			xpc56_write_u32(target, conf + 0xC, 0xc3c33333);
		}

		if (set) {
			lml |= change;
			sll |= change;
		} else {
			lml &= ~change;
			sll &= ~change;
		}

		/*
		lml = 0;
		sll = 0;
		*/

		/* write same lml based value in both registers */
		xpc56_write_u32(target, conf + 0x4, lml);
		xpc56_write_u32(target, conf + 0xC, sll);
	}

	/* exit(0); */

	return ERROR_OK;
}

static int xpc56f_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t to_erase = 0;
	uint32_t offset = 0;
	uintptr_t conf = (uintptr_t)bank->driver_priv;

	for (int i = first; i <= last; i++) {
		if (bank->sectors[i].is_erased != 1) {
			to_erase |= BIT(i);
			offset = bank->sectors[i].offset;
		}
	}

	if (to_erase) {
		/* todo: should test and fail */
		xpc56f_protect(bank, 0, first, last);
		/* select erase op */
		xpc56_write_u32(target, conf, 0x4);
		/* select sectors to erase */
		xpc56_write_u32(target, conf + 0x10, to_erase);
		/* interlock write */
		xpc56_write_u32(target, bank->base + offset, 0x12345678);
		/* start erasing */
		xpc56_write_u32(target, conf, 0x5);

		uint32_t mcr;
		uint32_t cnt = 0;
		do {
			/* check end */
			mcr = xpc56_read_u32(target, conf);
		} while ((mcr & 0x400) == 0 && cnt++ < 10000);
		/* note: cnt ~= 1500 at clock 10000kHz for 6 sectors/256k */

		if ((mcr & 0x600) != 0x600) {
			LOG_ERROR("Erase failed: mcr=0x%x\n", mcr);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		for (int i = first; i <= last; i++)
			bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}


static int xpc56f_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	const uint32_t *buffer_wa = (uint32_t *)buffer;

	int sect_first = -1;
	int sect_last = -1;

	/*
	compute first and last sect
	to unprotect -- assume sectors sorted
	*/
	for (int i = 0;; i++) {
		assert(i < bank->num_sectors);
		struct flash_sector *sect = bank->sectors + i;
		/*
		we could check more precisly
		buffer content != 0xFF
		*/
		if (sect_first == -1 && sect->offset <= offset && offset < sect->offset + sect->size)
			sect_first = i;

		if (sect_last == -1 && offset + count <= sect->offset + sect->size) {
			sect_last = i;
			break;
		}
	}

	xpc56f_protect(bank, 0, sect_first, sect_last);

	uintptr_t conf = (uintptr_t)bank->driver_priv;

	printf("RD 00  %08x\n", xpc56_read_u32(target, conf));
	printf("RD 04  %08x\n", xpc56_read_u32(target, conf+4));
	printf("RD 0C  %08x\n", xpc56_read_u32(target, conf+0xC));
	printf("RD 10  %08x\n", xpc56_read_u32(target, conf+0x10));

	/* select pgm op */
	xpc56_write_u32(target, conf, 0x10);

	for (int i = 0 ; count; i++) {
		assert(i < bank->num_sectors);
		struct flash_sector *sect = bank->sectors + i;

		if (sect->offset <= offset && offset < sect->offset + sect->size) {
			uint32_t sect_offset = offset - sect->offset;
			uint32_t sect_size = MIN(sect->size - sect_offset, count);

			printf("W %x [%d] %x %x (%d)\n", bank->base, i, sect->offset, sect_offset, sect_size);
			for (unsigned j = 0; j < sect_size ; j += sizeof(uint64_t)) {
				uint64_t dw = 0;
				unsigned k = 0;
				for ( ; k < sizeof(uint64_t) && j + k < sect_size ; k += 1) {
					dw <<= 8;
					dw |= buffer[sect_offset + j + k];
				}
				for ( ; k < sizeof(uint64_t) ; k += 1) {
					dw <<= 8;
					dw |= 0xFF;
				}

				/* todo: manage non aligned starting offset */
				assert(((bank->base + sect_offset + j) & 7) == 0);
				if (dw != -1ULL) {

					/* load first word to write */
					xpc56_write_u32(target, bank->base + sect_offset + j, dw >> 32);
					/* load second word to write */
					xpc56_write_u32(target, bank->base + sect_offset + j + 4, dw & 0xFFFFFFFF);
					/* start prog */
					xpc56_write_u32(target, conf, 0x11);

					uint32_t mcr;
					uint32_t cnt = 0;
					do {
						/* check end */
						mcr = xpc56_read_u32(target, conf);
					} while ((mcr & 0x400) == 0 && cnt++ < 10000);
					/* check PEG set */
					assert(mcr & 0x200 && cnt < 10000);
					/* end prog, but keep prog mode */
					xpc56_write_u32(target, conf, 0x10);
				}
			}

			count -= sect_size;
			offset += sect_size;
			buffer += sect_size;
		}
	}

	/*
	bug again somewhere!
	must rewrite 2 first words
	*/
	{
		/* load first word to write */
		xpc56_write_u32(target, 0, ntohl(buffer_wa[0]));
		/* load second word to write */
		xpc56_write_u32(target, 4, ntohl(buffer_wa[1]));
		/* start prog */
		xpc56_write_u32(target, conf, 0x11);

		uint32_t mcr;
		uint32_t cnt = 0;
		do {
			/* check end */
			mcr = xpc56_read_u32(target, conf);
		} while ((mcr & 0x400) == 0 && cnt++ < 10000);
		/* check PEG set */
		assert(mcr & 0x200 && cnt < 10000);
		/* end prog, but keep prog mode */
		xpc56_write_u32(target, conf, 0x10);
	}

	/* unselect pgm op */
	xpc56_write_u32(target, conf, 0x0);


	return ERROR_OK;
}

static int xpc56f_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	printf("BANK   %d %x %x # %x %x # %d %p # %d %p # %p\n", bank->bank_number
	, bank->base
	, bank->size
	, bank->erased_value
	, bank->default_padded_value
	, bank->num_sectors
	, bank->sectors
	, bank->num_prot_blocks
	, bank->prot_blocks
	, bank->next
	);

	if (bank->size)
		return ERROR_OK;

	uintptr_t conf;
	if (bank->base == CFLASH0_BASE)
		conf = CFLASH0_CONF;
	else if (bank->base == DFLASH0_BASE)
		conf = DFLASH0_CONF;
	else {
		LOG_ERROR("Base address not matching known configurations");
		return ERROR_FLASH_BANK_INVALID;
	}
	/* member hijacking */
	bank->driver_priv = (void *) conf;

	uint32_t reg_mcr = xpc56_read_u32(target, conf);
	uint32_t reg_lml = xpc56_read_u32(target, conf + 0x4);
	uint32_t reg_sll = xpc56_read_u32(target, conf + 0xC);

	/* total bank size in kb */
	const uint16_t MCR_SIZE[] = {128, 256, 512, 1024, 1536, 2048, 64, 96 };
	bank->size = MCR_SIZE[(reg_mcr >> 24) & 7] * 1024;

	printf("MCR %08x   %d   %08x (%d)\n", reg_mcr, (reg_mcr >> 24) & 7, bank->base, bank->size);

	/*
	 * sector cfg in kb for low address space,
	 * 0-terminated
	 */
	const uint8_t MCR_LAS[][9] = {
		{0},
		{128, 128, 0},
		{32, 16, 16, 32, 32, 128, 0},
		{0},
		{0},
		{16, 16, 16, 16, 16, 16, 0},
		{16, 16, 16, 16, 0},
		{16, 16, 32, 32, 16, 16, 64, 64, 0},
	};
	const uint8_t *las = MCR_LAS[(reg_mcr >> 20) & 7];
	while (las[bank->num_sectors])
		bank->num_sectors++;

	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	uint32_t offset = 0;
	for (int i = 0; i < bank->num_sectors; i++) {
		struct flash_sector *sect = bank->sectors + i;
		sect->offset = offset;
		sect->size = las[i] * 1024;
		offset += sect->size;
		sect->is_erased = -1;
		sect->is_protected = ((reg_lml | reg_sll) >> i) & 1;
		printf("    sect[%d] %08x (%d)\n", i, sect->offset, sect->size);
	}

	/*
	 * todo: add support for mid
	 * and high address space cfg
	 */
	assert(offset == bank->size);

	int r = xpc56f_protect_check(bank);
	if (r != ERROR_OK)
		return r;

	return ERROR_OK;
}

void xpc56f_flash_free_driver_priv(struct flash_bank *bank)
{
	bank->driver_priv = NULL;
}

static int xpc56f_auto_probe(struct flash_bank *bank)
{
	if (bank->size)
		return ERROR_OK;
	else
		return xpc56f_probe(bank);
}

static int xpc56f_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	snprintf(buf, buf_size, "xpc56 flash dummy info\n");
	return ERROR_OK;
}


COMMAND_HANDLER(xpc56f_handle_dummy_command)
{
	return ERROR_OK;
}

static const struct command_registration xpc56f_exec_command_handlers[] = {
	{
		.name = "dummy",
		.usage = "foo",
		.handler = xpc56f_handle_dummy_command,
		.mode = COMMAND_EXEC,
		.help = "bar",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration xpc56f_command_handlers[] = {
	{
		.name = "xpc56",
		.mode = COMMAND_ANY,
		.help = "xPC56 flash command group",
		.usage = "",
		.chain = xpc56f_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver xpc56_flash = {
	.name = "xpc56",
	.commands = xpc56f_command_handlers,
	.flash_bank_command = xpc56f_flash_bank_command,
	.erase = xpc56f_erase,
	.protect = xpc56f_protect,
	.write = xpc56f_write,
	.read = default_flash_read,
	.probe = xpc56f_probe,
	.auto_probe = xpc56f_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = xpc56f_protect_check,
	.info = xpc56f_info,
	.free_driver_priv = xpc56f_flash_free_driver_priv,
};
