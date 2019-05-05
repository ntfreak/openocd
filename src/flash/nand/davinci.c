/***************************************************************************
 *   Copyright (C) 2009 by David Brownell                                  *
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

/*
 * DaVinci family NAND controller support for OpenOCD.
 *
 * This driver uses hardware ECC (1-bit or 4-bit) unless
 * the chip is accessed in "raw" mode.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "arm_io.h"
#include <target/target.h>

enum ecc {
	HWECC1,		/* all controllers support 1-bit ECC */
	HWECC4,		/* newer chips also have 4-bit ECC hardware */
	HWECC4_INFIX,	/* avoid this layout, except maybe for boot code */
};

struct davinci_nand {
	uint8_t chipsel;		/* chipselect 0..3 == CS2..CS5 */
	uint8_t eccmode;

	/* Async EMIF controller base */
	uint32_t aemif;

	/* NAND chip addresses */
	uint32_t data;				/* without CLE or ALE */
	uint32_t cmd;				/* with CLE */
	uint32_t addr;				/* with ALE */

	/* write acceleration */
	struct arm_nand_data io;

	/* page i/o for the relevant flavor of hardware ECC */
	int (*read_page)(struct nand_device *nand, uint32_t page,
			 uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size);
	int (*write_page)(struct nand_device *nand, uint32_t page,
			  uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size);
};

#define NANDFCR         0x60		/* flash control register */
#define NANDFSR         0x64		/* flash status register */
#define NANDFECC        0x70		/* 1-bit ECC data, CS0, 1st of 4 */
#define NAND4BITECCLOAD 0xbc		/* 4-bit ECC, load saved values */
#define NAND4BITECC     0xc0		/* 4-bit ECC data, 1st of 4 */
#define NANDERRADDR     0xd0		/* 4-bit ECC err addr, 1st of 2 */
#define NANDERRVAL      0xd8		/* 4-bit ECC err value, 1st of 2 */

static int halted(struct target *target, const char *label)
{
	if (target->state == TARGET_HALTED)
		return true;

	LOG_ERROR("Target must be halted to use NAND controller (%s)", label);
	return false;
}

static int davinci_init(struct nand_device *nand)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t nandfcr;

	if (!halted(target, "init"))
		return ERROR_NAND_OPERATION_FAILED;

	/* We require something else to have configured AEMIF to talk
	 * to NAND chip in this range (including timings and width).
	 */
	target_read_u32(target, info->aemif + NANDFCR, &nandfcr);
	if (!(nandfcr & (1 << info->chipsel))) {
		LOG_ERROR("chip address %08" PRIx32 " not NAND-enabled?", info->data);
		return ERROR_NAND_OPERATION_FAILED;
	}

	/* REVISIT verify:  AxCR must be in 8-bit mode, since that's all we
	 * tested.  16 bit support should work too; but not with 4-bit ECC.
	 */

	return ERROR_OK;
}

static int davinci_reset(struct nand_device *nand)
{
	return ERROR_OK;
}

static int davinci_nand_ready(struct nand_device *nand, int timeout)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t nandfsr;

	/* NOTE: return code is zero/error, else success; not ERROR_* */

	if (!halted(target, "ready"))
		return 0;

	do {
		target_read_u32(target, info->aemif + NANDFSR, &nandfsr);

		if (nandfsr & 0x01)
			return 1;

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

static int davinci_command(struct nand_device *nand, uint8_t command)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!halted(target, "command"))
		return ERROR_NAND_OPERATION_FAILED;

	target_write_u8(target, info->cmd, command);
	return ERROR_OK;
}

static int davinci_address(struct nand_device *nand, uint8_t address)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!halted(target, "address"))
		return ERROR_NAND_OPERATION_FAILED;

	target_write_u8(target, info->addr, address);
	return ERROR_OK;
}

static int davinci_write_data(struct nand_device *nand, uint16_t data)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!halted(target, "write_data"))
		return ERROR_NAND_OPERATION_FAILED;

	target_write_u8(target, info->data, data);
	return ERROR_OK;
}

static int davinci_read_data(struct nand_device *nand, void *data)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!halted(target, "read_data"))
		return ERROR_NAND_OPERATION_FAILED;

	target_read_u8(target, info->data, data);
	return ERROR_OK;
}

/* REVISIT a bit of native code should let block reads be MUCH faster */

static int davinci_read_block_data(struct nand_device *nand,
	uint8_t *data, int data_size)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t nfdata = info->data;
	uint32_t tmp;

	if (!halted(target, "read_block"))
		return ERROR_NAND_OPERATION_FAILED;

	while (data_size >= 4) {
		target_read_u32(target, nfdata, &tmp);

		data[0] = tmp;
		data[1] = tmp >> 8;
		data[2] = tmp >> 16;
		data[3] = tmp >> 24;

		data_size -= 4;
		data += 4;
	}

	while (data_size > 0) {
		target_read_u8(target, nfdata, data);

		data_size -= 1;
		data += 1;
	}

	return ERROR_OK;
}

static int davinci_write_block_data(struct nand_device *nand,
	uint8_t *data, int data_size)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t nfdata = info->data;
	uint32_t tmp;
	int status;

	if (!halted(target, "write_block"))
		return ERROR_NAND_OPERATION_FAILED;

	/* try the fast way first */
	status = arm_nandwrite(&info->io, data, data_size);
	if (status != ERROR_NAND_NO_BUFFER)
		return status;

	/* else do it slowly */
	while (data_size >= 4) {
		tmp = le_to_h_u32(data);
		target_write_u32(target, nfdata, tmp);

		data_size -= 4;
		data += 4;
	}

	while (data_size > 0) {
		target_write_u8(target, nfdata, *data);

		data_size -= 1;
		data += 1;
	}

	return ERROR_OK;
}

static int davinci_write_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	struct davinci_nand *info = nand->controller_priv;
	uint8_t *ooballoc = NULL;
	int status;

	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;
	if (!halted(nand->target, "write_page"))
		return ERROR_NAND_OPERATION_FAILED;

	/* Always write both data and OOB ... we are not "raw" I/O! */
	if (!data) {
		LOG_ERROR("Missing NAND data; try 'nand raw_access enable'");
		return ERROR_NAND_OPERATION_FAILED;
	}

	/* If we're not given OOB, write 0xff where we don't write ECC codes. */
	switch (nand->page_size) {
		case 512:
			oob_size = 16;
			break;
		case 2048:
			oob_size = 64;
			break;
		case 4096:
			oob_size = 128;
			break;
		default:
			return ERROR_NAND_OPERATION_FAILED;
	}
	if (!oob) {
		ooballoc = malloc(oob_size);
		if (!ooballoc)
			return ERROR_NAND_OPERATION_FAILED;
		oob = ooballoc;
		memset(oob, 0x0ff, oob_size);
	}

	/* REVISIT avoid wasting SRAM:  unless nand->use_raw is set,
	 * use 512 byte chunks.  Read side support will often want
	 * to include oob_size ...
	 */
	info->io.chunk_size = nand->page_size;

	status = info->write_page(nand, page, data, data_size, oob, oob_size);
	free(ooballoc);
	return status;
}

static int davinci_read_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	struct davinci_nand *info = nand->controller_priv;

	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;
	if (!halted(nand->target, "read_page"))
		return ERROR_NAND_OPERATION_FAILED;

	return info->read_page(nand, page, data, data_size, oob, oob_size);
}

static void davinci_write_pagecmd(struct nand_device *nand, uint8_t cmd, uint32_t page)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	int page3 = nand->address_cycles - (nand->page_size == 512);

	/* write command ({page,otp}x{read,program} */
	target_write_u8(target, info->cmd, cmd);

	/* column address (beginning-of-page) */
	target_write_u8(target, info->addr, 0);
	if (nand->page_size > 512)
		target_write_u8(target, info->addr, 0);

	/* page address */
	target_write_u8(target, info->addr, page);
	target_write_u8(target, info->addr, page >> 8);
	if (page3)
		target_write_u8(target, info->addr, page >> 16);
	if (page3 == 2)
		target_write_u8(target, info->addr, page >> 24);
}

static int davinci_seek_column(struct nand_device *nand, uint16_t column)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	/* Random read, we must have issued a page read already */
	target_write_u8(target, info->cmd, NAND_CMD_RNDOUT);

	target_write_u8(target, info->addr, column);

	if (nand->page_size > 512) {
		target_write_u8(target, info->addr, column >> 8);
		target_write_u8(target, info->cmd, NAND_CMD_RNDOUTSTART);
	}

	if (!davinci_nand_ready(nand, 100))
		return ERROR_NAND_OPERATION_TIMEOUT;

	return ERROR_OK;
}

static int davinci_writepage_tail(struct nand_device *nand,
	uint8_t *oob, uint32_t oob_size)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	uint8_t status;

	if (oob_size)
		davinci_write_block_data(nand, oob, oob_size);

	/* non-cachemode page program */
	target_write_u8(target, info->cmd, NAND_CMD_PAGEPROG);

	if (!davinci_nand_ready(nand, 100))
		return ERROR_NAND_OPERATION_TIMEOUT;

	if (nand_read_status(nand, &status) != ERROR_OK) {
		LOG_ERROR("couldn't read status");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (status & NAND_STATUS_FAIL) {
		LOG_ERROR("write operation failed, status: 0x%02x", status);
		return ERROR_NAND_OPERATION_FAILED;
	}

	return ERROR_OK;
}

/*
 * All DaVinci family chips support 1-bit ECC on a per-chipselect basis.
 */
static int davinci_write_page_ecc1(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	unsigned oob_offset;
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	const uint32_t fcr_addr = info->aemif + NANDFCR;
	const uint32_t ecc1_addr = info->aemif + NANDFECC + (4 * info->chipsel);
	uint32_t fcr, ecc1;

	/* Write contiguous ECC bytes starting at specified offset.
	 * NOTE: Linux reserves twice as many bytes as we need; and
	 * for 16-bit OOB, those extra bytes are discontiguous.
	 */
	switch (nand->page_size) {
		case 512:
			oob_offset = 0;
			break;
		case 2048:
			oob_offset = 40;
			break;
		default:
			oob_offset = 80;
			break;
	}

	davinci_write_pagecmd(nand, NAND_CMD_SEQIN, page);

	/* scrub any old ECC state */
	target_read_u32(target, ecc1_addr, &ecc1);

	target_read_u32(target, fcr_addr, &fcr);
	fcr |= 1 << (8 + info->chipsel);

	do {
		/* set "start csX 1bit ecc" bit */
		target_write_u32(target, fcr_addr, fcr);

		/* write 512 bytes */
		davinci_write_block_data(nand, data, 512);
		data += 512;
		data_size -= 512;

		/* read the ecc, pack to 3 bytes, and invert so the ecc
		 * in an erased block is correct
		 */
		target_read_u32(target, ecc1_addr, &ecc1);
		ecc1 = (ecc1 & 0x0fff) | ((ecc1 & 0x0fff0000) >> 4);
		ecc1 = ~ecc1;

		/* save correct ECC code into oob data */
		oob[oob_offset++] = (uint8_t)(ecc1);
		oob[oob_offset++] = (uint8_t)(ecc1 >> 8);
		oob[oob_offset++] = (uint8_t)(ecc1 >> 16);

	} while (data_size);

	/* write OOB into spare area */
	return davinci_writepage_tail(nand, oob, oob_size);
}

/*
 * Preferred "new style" ECC layout for use with 4-bit ECC.  This somewhat
 * slows down large page reads done with error correction (since the OOB
 * is read first, so its ECC data can be used incrementally), but the
 * manufacturer bad block markers are safe.  Contrast:  old "infix" style.
 */
static int davinci_write_page_ecc4(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	static const uint8_t ecc512[] = {
		0, 1, 2, 3, 4,	/* 5== mfr badblock */
		6, 7, /* 8..12 for BBT or JFFS2 */ 13, 14, 15,
	};
	static const uint8_t ecc2048[] = {
		24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
		34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
		44, 45, 46, 47, 48, 49, 50, 51, 52, 53,
		54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
	};
	static const uint8_t ecc4096[] = {
		48,  49,  50,  51,  52,  53,  54,  55,  56,  57,
		58,  59,  60,  61,  62,  63,  64,  65,  66,  67,
		68,  69,  70,  71,  72,  73,  74,  75,  76,  77,
		78,  79,  80,  81,  82,  83,  84,  85,  86,  87,
		88,  89,  90,  91,  92,  93,  94,  95,  96,  97,
		98,  99, 100, 101, 102, 103, 104, 105, 106, 107,
		108, 109, 110, 111, 112, 113, 114, 115, 116, 117,
		118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
	};

	struct davinci_nand *info = nand->controller_priv;
	const uint8_t *l;
	struct target *target = nand->target;
	const uint32_t fcr_addr = info->aemif + NANDFCR;
	const uint32_t ecc4_addr = info->aemif + NAND4BITECC;
	uint32_t fcr, ecc4;

	/* Use the same ECC layout Linux uses.  For small page chips
	 * it's a bit cramped.
	 *
	 * NOTE:  at this writing, 4KB pages have issues in Linux
	 * because they need more than 64 bytes of ECC data, which
	 * the standard ECC logic can't handle.
	 */
	switch (nand->page_size) {
		case 512:
			l = ecc512;
			break;
		case 2048:
			l = ecc2048;
			break;
		default:
			l = ecc4096;
			break;
	}

	davinci_write_pagecmd(nand, NAND_CMD_SEQIN, page);

	/* scrub any old ECC state */
	target_read_u32(target, info->aemif + NANDERRVAL, &ecc4);

	target_read_u32(target, fcr_addr, &fcr);
	fcr &= ~(0x03 << 4);
	fcr |= (1 << 12) | (info->chipsel << 4);

	do {
		uint32_t raw_ecc[4], *p;
		int i;

		/* start 4bit ecc on csX */
		target_write_u32(target, fcr_addr, fcr);

		/* write 512 bytes */
		davinci_write_block_data(nand, data, 512);
		data += 512;
		data_size -= 512;

		/* read the ecc, then save it into 10 bytes in the oob */
		for (i = 0; i < 4; i++) {
			target_read_u32(target, ecc4_addr + 4 * i, &raw_ecc[i]);
			raw_ecc[i] &= 0x03ff03ff;
		}
		for (i = 0, p = raw_ecc; i < 2; i++, p += 2) {
			oob[*l++] = p[0]        & 0xff;
			oob[*l++] = ((p[0] >>  8) & 0x03) | ((p[0] >> 14) & 0xfc);
			oob[*l++] = ((p[0] >> 22) & 0x0f) | ((p[1] <<  4) & 0xf0);
			oob[*l++] = ((p[1] >>  4) & 0x3f) | ((p[1] >> 10) & 0xc0);
			oob[*l++] = (p[1] >> 18) & 0xff;
		}

	} while (data_size);

	/* write OOB into spare area */
	return davinci_writepage_tail(nand, oob, oob_size);
}

/*
 * "Infix" OOB ... like Linux ECC_HW_SYNDROME.  Avoided because it trashes
 * manufacturer bad block markers, except on small page chips.  Once you
 * write to a page using this scheme, you need specialized code to update
 * it (code which ignores now-invalid bad block markers).
 *
 * This is needed *only* to support older firmware.  Older ROM Boot Loaders
 * need it to read their second stage loader (UBL) into SRAM, but from then
 * on the whole system can use the cleaner non-infix layouts.  Systems with
 * older second stage loaders (ABL/U-Boot, etc) or other system software
 * (MVL 4.x/5.x kernels, filesystems, etc) may need it more generally.
 */
static int davinci_write_page_ecc4infix(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	struct davinci_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	const uint32_t fcr_addr = info->aemif + NANDFCR;
	const uint32_t ecc4_addr = info->aemif + NAND4BITECC;
	uint32_t fcr, ecc4;

	davinci_write_pagecmd(nand, NAND_CMD_SEQIN, page);

	/* scrub any old ECC state */
	target_read_u32(target, info->aemif + NANDERRVAL, &ecc4);

	target_read_u32(target, fcr_addr, &fcr);
	fcr &= ~(0x03 << 4);
	fcr |= (1 << 12) | (info->chipsel << 4);

	do {
		uint32_t raw_ecc[4], *p;
		uint8_t *l;
		int i;

		/* start 4bit ecc on csX */
		target_write_u32(target, fcr_addr, fcr);

		/* write 512 bytes */
		davinci_write_block_data(nand, data, 512);
		data += 512;
		data_size -= 512;

		/* read the ecc */
		for (i = 0; i < 4; i++) {
			target_read_u32(target, ecc4_addr + 4 * i, &raw_ecc[i]);
			raw_ecc[i] &= 0x03ff03ff;
		}

		/* skip 6 bytes of prepad, then pack 10 packed ecc bytes */
		for (i = 0, l = oob + 6, p = raw_ecc; i < 2; i++, p += 2) {
			*l++ = p[0]        & 0xff;
			*l++ = ((p[0] >>  8) & 0x03) | ((p[0] >> 14) & 0xfc);
			*l++ = ((p[0] >> 22) & 0x0f) | ((p[1] <<  4) & 0xf0);
			*l++ = ((p[1] >>  4) & 0x3f) | ((p[1] >> 10) & 0xc0);
			*l++ = (p[1] >> 18) & 0xff;
		}

		/* write this "out-of-band" data -- infix */
		davinci_write_block_data(nand, oob, 16);
		oob += 16;
		oob_size -= 16;

	} while (data_size);

	/* the last data and OOB writes included the spare area */
	return davinci_writepage_tail(nand, NULL, 0);
}

static int davinci_read_page_ecc4infix(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	int read_size;
	int want_col, at_col;
	int ret;

	davinci_write_pagecmd(nand, NAND_CMD_READ0, page);

	/* large page devices need a start command */
	if (nand->page_size > 512)
		davinci_command(nand, NAND_CMD_READSTART);

	if (!davinci_nand_ready(nand, 100))
		return ERROR_NAND_OPERATION_TIMEOUT;

	/* NOTE:  not bothering to compute and use ECC data for now */

	want_col = 0;
	at_col = 0;
	while ((data && data_size) || (oob && oob_size)) {

		if (data && data_size) {
			if (want_col != at_col) {
				/* Reads are slow, so seek past them when we can */
				ret  = davinci_seek_column(nand, want_col);
				if (ret != ERROR_OK)
					return ret;
				at_col = want_col;
			}
			/* read 512 bytes or data_size, whichever is smaller*/
			read_size = data_size > 512 ? 512 : data_size;
			davinci_read_block_data(nand, data, read_size);
			data += read_size;
			data_size -= read_size;
			at_col += read_size;
		}
		want_col += 512;

		if (oob && oob_size) {
			if (want_col != at_col) {
				ret  = davinci_seek_column(nand, want_col);
				if (ret != ERROR_OK)
					return ret;
				at_col = want_col;
			}
			/* read this "out-of-band" data -- infix */
			read_size = oob_size > 16 ? 16 : oob_size;
			davinci_read_block_data(nand, oob, read_size);
			oob += read_size;
			oob_size -= read_size;
			at_col += read_size;
		}
		want_col += 16;
	}
	return ERROR_OK;
}

NAND_DEVICE_COMMAND_HANDLER(davinci_nand_device_command)
{
	struct davinci_nand *info;
	unsigned long chip, aemif;
	enum ecc eccmode;
	int chipsel;

	/* arguments:
	 *  - "davinci"
	 *  - target
	 *  - nand chip address
	 *  - ecc mode
	 *  - aemif address
	 * Plus someday, optionally, ALE and CLE masks.
	 */
	if (CMD_ARGC < 5)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[2], chip);
	if (chip == 0) {
		LOG_ERROR("Invalid NAND chip address %s", CMD_ARGV[2]);
		goto fail;
	}

	if (strcmp(CMD_ARGV[3], "hwecc1") == 0)
		eccmode = HWECC1;
	else if (strcmp(CMD_ARGV[3], "hwecc4") == 0)
		eccmode = HWECC4;
	else if (strcmp(CMD_ARGV[3], "hwecc4_infix") == 0)
		eccmode = HWECC4_INFIX;
	else {
		LOG_ERROR("Invalid ecc mode %s", CMD_ARGV[3]);
		goto fail;
	}

	COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[4], aemif);
	if (aemif == 0) {
		LOG_ERROR("Invalid AEMIF controller address %s", CMD_ARGV[4]);
		goto fail;
	}

	/* REVISIT what we'd *like* to do is look up valid ranges using
	 * target-specific declarations, and not even need to pass the
	 * AEMIF controller address.
	 */
	if (aemif == 0x01e00000			/* dm6446, dm357 */
			|| aemif == 0x01e10000		/* dm335, dm355 */
			|| aemif == 0x01d10000		/* dm365 */
		) {
		if (chip < 0x02000000 || chip >= 0x0a000000) {
			LOG_ERROR("NAND address %08lx out of range?", chip);
			goto fail;
		}
		chipsel = (chip - 0x02000000) >> 25;
	} else {
		LOG_ERROR("unrecognized AEMIF controller address %08lx", aemif);
		goto fail;
	}

	info = calloc(1, sizeof(*info));
	if (info == NULL)
		goto fail;

	info->eccmode = eccmode;
	info->chipsel = chipsel;
	info->aemif = aemif;
	info->data = chip;
	info->cmd = chip | 0x10;
	info->addr = chip | 0x08;

	nand->controller_priv = info;

	info->io.target = nand->target;
	info->io.data = info->data;
	info->io.op = ARM_NAND_NONE;

	/* NOTE:  for now we don't do any error correction on read.
	 * Nothing else in OpenOCD currently corrects read errors,
	 * and in any case it's *writing* that we care most about.
	 */
	info->read_page = nand_read_page_raw;

	switch (eccmode) {
		case HWECC1:
			/* ECC_HW, 1-bit corrections, 3 bytes ECC per 512 data bytes */
			info->write_page = davinci_write_page_ecc1;
			break;
		case HWECC4:
			/* ECC_HW, 4-bit corrections, 10 bytes ECC per 512 data bytes */
			info->write_page = davinci_write_page_ecc4;
			break;
		case HWECC4_INFIX:
			/* Same 4-bit ECC HW, with problematic page/ecc layout */
			info->read_page = davinci_read_page_ecc4infix;
			info->write_page = davinci_write_page_ecc4infix;
			break;
	}

	return ERROR_OK;

fail:
	return ERROR_NAND_OPERATION_FAILED;
}

struct nand_flash_controller davinci_nand_controller = {
	.name                   = "davinci",
	.usage                  = "chip_addr hwecc_mode aemif_addr",
	.nand_device_command    = davinci_nand_device_command,
	.init                   = davinci_init,
	.reset                  = davinci_reset,
	.command                = davinci_command,
	.address                = davinci_address,
	.write_data             = davinci_write_data,
	.read_data              = davinci_read_data,
	.write_page             = davinci_write_page,
	.read_page              = davinci_read_page,
	.write_block_data       = davinci_write_block_data,
	.read_block_data        = davinci_read_block_data,
	.nand_ready             = davinci_nand_ready,
};
