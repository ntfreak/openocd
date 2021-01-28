/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *                                                                         *
 *   Partially based on linux/include/linux/mtd/nand.h                     *
 *   Copyright (C) 2000 David Woodhouse <dwmw2@mvhi.com>                   *
 *   Copyright (C) 2000 Steven J. Hill <sjhill@realitydiluted.com>         *
 *   Copyright (C) 2000 Thomas Gleixner <tglx@linutronix.de>               *
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

#ifndef OPENOCD_FLASH_NAND_CORE_H
#define OPENOCD_FLASH_NAND_CORE_H

#include <flash/common.h>

/**
 * Representation of a single NAND block in a NAND device.
 */
struct nand_block {
	/** Offset to the block. */
	uint32_t offset;

	/** Size of the block. */
	uint32_t size;

	/** True if the block has been erased. */
	int is_erased;

	/** True if the block is bad. */
	int is_bad;
};

struct nand_oobfree {
	int offset;
	int length;
};

struct nand_ecclayout {
	int eccbytes;
	int eccpos[64];
	int oobavail;
	struct nand_oobfree oobfree[2];
};

struct nand_device {
	const char *name;
	struct target *target;
	struct nand_flash_controller *controller;
	void *controller_priv;
	struct nand_manufacturer *manufacturer;
	struct nand_info *device;
	int bus_width;
	int address_cycles;
	int page_size;
	int erase_size;
	bool use_raw;
	int num_blocks;
	struct nand_block *blocks;
	struct nand_device *next;
};

/* NAND Flash Manufacturer ID Codes
 */
enum {
	NAND_MFR_TOSHIBA = 0x98,
	NAND_MFR_SAMSUNG = 0xec,
	NAND_MFR_FUJITSU = 0x04,
	NAND_MFR_NATIONAL = 0x8f,
	NAND_MFR_RENESAS = 0x07,
	NAND_MFR_STMICRO = 0x20,
	NAND_MFR_HYNIX = 0xad,
	NAND_MFR_MICRON = 0x2c,
};

struct nand_manufacturer {
	int id;
	const char *name;
};

struct nand_info {
	int mfr_id;
	int id;
	int page_size;
	int chip_size;
	int erase_size;
	int options;
	const char *name;
};

/* Option constants for bizarre dysfunctionality and real features
 */
enum {
	/* Chip can not auto increment pages */
	NAND_NO_AUTOINCR = 0x00000001,

	/* Buswitdh is 16 bit */
	NAND_BUSWIDTH_16 = 0x00000002,

	/* Device supports partial programming without padding */
	NAND_NO_PADDING = 0x00000004,

	/* Chip has cache program function */
	NAND_CACHEPRG = 0x00000008,

	/* Chip has copy back function */
	NAND_COPYBACK = 0x00000010,

	/* AND Chip which has 4 banks and a confusing page / block
	 * assignment. See Renesas datasheet for further information */
	NAND_IS_AND = 0x00000020,

	/* Chip has a array of 4 pages which can be read without
	 * additional ready /busy waits */
	NAND_4PAGE_ARRAY = 0x00000040,

	/* Chip requires that BBT is periodically rewritten to prevent
	 * bits from adjacent blocks from 'leaking' in altering data.
	 * This happens with the Renesas AG-AND chips, possibly others.  */
	BBT_AUTO_REFRESH = 0x00000080,

	/* Chip does not require ready check on read. True
	 * for all large page devices, as they do not support
	 * autoincrement.*/
	NAND_NO_READRDY = 0x00000100,

	/* Options valid for Samsung large page devices */
	NAND_SAMSUNG_LP_OPTIONS = (NAND_NO_PADDING | NAND_CACHEPRG | NAND_COPYBACK),

	/* Options for new chips with large page size. The pagesize and the
	 * erasesize is determined from the extended id bytes
	 */
	LP_OPTIONS = (NAND_SAMSUNG_LP_OPTIONS | NAND_NO_READRDY | NAND_NO_AUTOINCR),
	LP_OPTIONS16 = (LP_OPTIONS | NAND_BUSWIDTH_16),
};

enum {
	/* Standard NAND flash commands */
	NAND_CMD_READ0 = 0x0,
	NAND_CMD_READ1 = 0x1,
	NAND_CMD_RNDOUT = 0x5,
	NAND_CMD_PAGEPROG = 0x10,
	NAND_CMD_READOOB = 0x50,
	NAND_CMD_ERASE1 = 0x60,
	NAND_CMD_STATUS = 0x70,
	NAND_CMD_STATUS_MULTI = 0x71,
	NAND_CMD_SEQIN = 0x80,
	NAND_CMD_RNDIN = 0x85,
	NAND_CMD_READID = 0x90,
	NAND_CMD_ERASE2 = 0xd0,
	NAND_CMD_RESET = 0xff,

	/* Extended commands for large page devices */
	NAND_CMD_READSTART = 0x30,
	NAND_CMD_RNDOUTSTART = 0xE0,
	NAND_CMD_CACHEDPROG = 0x15,
};

/* Status bits */
enum {
	NAND_STATUS_FAIL = 0x01,
	NAND_STATUS_FAIL_N1 = 0x02,
	NAND_STATUS_TRUE_READY = 0x20,
	NAND_STATUS_READY = 0x40,
	NAND_STATUS_WP = 0x80,
};

/* OOB (spare) data formats */
enum oob_formats {
	NAND_OOB_NONE = 0x0,	/* no OOB data at all */
	NAND_OOB_RAW = 0x1,		/* raw OOB data (16 bytes for 512b page sizes, 64 bytes for
					 *2048b page sizes) */
	NAND_OOB_ONLY = 0x2,	/* only OOB data */
	NAND_OOB_SW_ECC = 0x10,	/* when writing, use SW ECC (as opposed to no ECC) */
	NAND_OOB_HW_ECC = 0x20,	/* when writing, use HW ECC (as opposed to no ECC) */
	NAND_OOB_SW_ECC_KW = 0x40,	/* when writing, use Marvell's Kirkwood bootrom format */
	NAND_OOB_JFFS2 = 0x100,	/* when writing, use JFFS2 OOB layout */
	NAND_OOB_YAFFS2 = 0x100,/* when writing, use YAFFS2 OOB layout */
};


struct nand_device *get_nand_device_by_num(int num);

int nand_page_command(struct nand_device *nand, uint32_t page,
		      uint8_t cmd, bool oob_only);

int nand_read_data_page(struct nand_device *nand, uint8_t *data, uint32_t size);
int nand_write_data_page(struct nand_device *nand,
			 uint8_t *data, uint32_t size);

int nand_write_finish(struct nand_device *nand);

int nand_read_page_raw(struct nand_device *nand, uint32_t page,
		       uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size);
int nand_write_page_raw(struct nand_device *nand, uint32_t page,
			uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size);

int nand_read_status(struct nand_device *nand, uint8_t *status);

int nand_calculate_ecc(struct nand_device *nand,
		       const uint8_t *dat, uint8_t *ecc_code);
int nand_calculate_ecc_kw(struct nand_device *nand,
			  const uint8_t *dat, uint8_t *ecc_code);

int nand_register_commands(struct command_context *cmd_ctx);

/** helper for parsing a nand device command argument string */
COMMAND_HELPER(nand_command_get_device, unsigned name_index,
	struct nand_device **nand);


#define         ERROR_NAND_DEVICE_INVALID               (-1100)
#define         ERROR_NAND_OPERATION_FAILED             (-1101)
#define         ERROR_NAND_OPERATION_TIMEOUT    (-1102)
#define         ERROR_NAND_OPERATION_NOT_SUPPORTED      (-1103)
#define         ERROR_NAND_DEVICE_NOT_PROBED    (-1104)
#define         ERROR_NAND_ERROR_CORRECTION_FAILED      (-1105)
#define         ERROR_NAND_NO_BUFFER                    (-1106)

#endif /* OPENOCD_FLASH_NAND_CORE_H */
