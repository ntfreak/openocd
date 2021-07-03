/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   LPC1700 support Copyright (C) 2009 by Audrius Urmanavicius            *
 *   didele.deze@gmail.com                                                 *
 *                                                                         *
 *   LPC1100 variant and auto-probing support Copyright (C) 2014           *
 *   by Cosmin Gorgovan cosmin [at] linux-geek [dot] org                   *
 *                                                                         *
 *   LPC800/LPC1500/LPC54100 support Copyright (C) 2013/2014               *
 *   by Nemui Trinomius                                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   LPC8N04/HNS31xx support Copyright (C) 2018                            *
 *   by Jean-Christian de Rivaz jcdr [at] innodelec [dot] ch               *
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
#include <target/arm_opcodes.h>
#include <target/armv7m.h>

/**
 * @file
 * flash programming support for NXP LPC8xx,LPC1xxx,LPC4xxx,LP5410x,LPC2xxx and NHS31xx devices.
 *
 * @todo Provide a way to update CCLK after declaring the flash bank. The value which is correct after chip reset will
 * rarely still work right after the clocks switch to use the PLL (e.g. 4MHz --> 100 MHz).
 */
/*
 * currently supported devices:
 * variant 1 (lpc2000_v1):
 * - 2104 | 5 | 6
 * - 2114 | 9
 * - 2124 | 9
 * - 2194
 * - 2212 | 4
 * - 2292 | 4
 *
 * variant 2 (lpc2000_v2):
 * - 213x
 * - 214x
 * - 2101 | 2 | 3
 * - 2364 | 6 | 8
 * - 2378
 *
 * lpc1700:
 * - 175x
 * - 176x (tested with LPC1768)
 * - 177x
 * - 178x (tested with LPC1788)
 *
 * lpc4000: (lpc1700's alias)
 * - 407x
 * - 408x (tested with LPC4088)
 *
 * lpc4300: (also available as lpc1800 - alias)
 * - 43x2 | 3 | 5 | 7 (tested with LPC4337/LPC4357)
 * - 18x2 | 3 | 5 | 7
 *
 * lpc800:
 * - 810 | 1 | 2 (tested with LPC810/LPC811/LPC812)
 * - 822 | 4 (tested with LPC824)
 * - 8N04
 * - NHS31xx (tested with NHS3100)
 * - 844 | 5 (tested with LPC845)
 *
 * lpc1100:
 * - 11xx
 * - 11Axx
 * - 11Cxx
 * - 11Dxx
 * - 11Exx
 * - 11Uxx (tested with LPC11U34)
 * - 131x
 * - 134x
 *
 * lpc1500:
 * - 15x7 | 8 | 9 (tested with LPC1549)
 *
 * lpc54100:
 * - 54101 | 2 (tested with LPC54102)
 *
 * The auto variant auto-detects parts from the following series:
 * - 11xx
 * - 11Axx
 * - 11Cxx
 * - 11Dxx
 * - 11Exx
 * - 11Uxx
 * - 131x
 * - 134x
 * - 175x
 * - 176x
 * - 177x
 * - 178x
 * - 407x
 * - 408x
 * - 81x
 * - 82x
 * - 8N04
 * - NHS31xx
 */

/* Part IDs for autodetection */
/* A script which can automatically extract part ids from user manuals is available here:
 * https://github.com/lgeek/lpc_part_ids
 */
#define LPC1110_1      0x0A07102B
#define LPC1110_2      0x1A07102B
#define LPC1111_002_1  0x0A16D02B
#define LPC1111_002_2  0x1A16D02B
#define LPC1111_101_1  0x041E502B
#define LPC1111_101_2  0x2516D02B
#define LPC1111_103_1  0x00010013
#define LPC1111_201_1  0x0416502B
#define LPC1111_201_2  0x2516902B
#define LPC1111_203_1  0x00010012
#define LPC1112_101_1  0x042D502B
#define LPC1112_101_2  0x2524D02B
#define LPC1112_102_1  0x0A24902B
#define LPC1112_102_2  0x1A24902B
#define LPC1112_103_1  0x00020023
#define LPC1112_201_1  0x0425502B
#define LPC1112_201_2  0x2524902B
#define LPC1112_203_1  0x00020022
#define LPC1113_201_1  0x0434502B
#define LPC1113_201_2  0x2532902B
#define LPC1113_203_1  0x00030032
#define LPC1113_301_1  0x0434102B
#define LPC1113_301_2  0x2532102B
#define LPC1113_303_1  0x00030030
#define LPC1114_102_1  0x0A40902B
#define LPC1114_102_2  0x1A40902B
#define LPC1114_201_1  0x0444502B
#define LPC1114_201_2  0x2540902B
#define LPC1114_203_1  0x00040042
#define LPC1114_301_1  0x0444102B
#define LPC1114_301_2  0x2540102B
#define LPC1114_303_1  0x00040040
#define LPC1114_323_1  0x00040060
#define LPC1114_333_1  0x00040070
#define LPC1115_303_1  0x00050080

#define LPC11A02_1     0x4D4C802B
#define LPC11A04_1     0x4D80002B
#define LPC11A11_001_1 0x455EC02B
#define LPC11A12_101_1 0x4574802B
#define LPC11A13_201_1 0x458A402B
#define LPC11A14_301_1 0x35A0002B
#define LPC11A14_301_2 0x45A0002B

#define LPC11C12_301_1 0x1421102B
#define LPC11C14_301_1 0x1440102B
#define LPC11C22_301_1 0x1431102B
#define LPC11C24_301_1 0x1430102B

#define LPC11E11_101   0x293E902B
#define LPC11E12_201   0x2954502B
#define LPC11E13_301   0x296A102B
#define LPC11E14_401   0x2980102B
#define LPC11E36_501   0x00009C41
#define LPC11E37_401   0x00007C45
#define LPC11E37_501   0x00007C41

#define LPC11U12_201_1 0x095C802B
#define LPC11U12_201_2 0x295C802B
#define LPC11U13_201_1 0x097A802B
#define LPC11U13_201_2 0x297A802B
#define LPC11U14_201_1 0x0998802B
#define LPC11U14_201_2 0x2998802B
#define LPC11U23_301   0x2972402B
#define LPC11U24_301   0x2988402B
#define LPC11U24_401   0x2980002B
#define LPC11U34_311   0x0003D440
#define LPC11U34_421   0x0001CC40
#define LPC11U35_401   0x0001BC40
#define LPC11U35_501   0x0000BC40
#define LPC11U36_401   0x00019C40
#define LPC11U37_401   0x00017C40
#define LPC11U37H_401  0x00007C44
#define LPC11U37_501   0x00007C40

#define LPC11E66       0x0000DCC1
#define LPC11E67       0x0000BC81
#define LPC11E68       0x00007C01

#define LPC11U66       0x0000DCC8
#define LPC11U67_1     0x0000BC88
#define LPC11U67_2     0x0000BC80
#define LPC11U68_1     0x00007C08
#define LPC11U68_2     0x00007C00

#define LPC1311        0x2C42502B
#define LPC1311_1      0x1816902B
#define LPC1313        0x2C40102B
#define LPC1313_1      0x1830102B
#define LPC1315        0x3A010523
#define LPC1316        0x1A018524
#define LPC1317        0x1A020525
#define LPC1342        0x3D01402B
#define LPC1343        0x3D00002B
#define LPC1343_1      0x3000002B
#define LPC1345        0x28010541
#define LPC1346        0x08018542
#define LPC1347        0x08020543

#define LPC1751_1      0x25001110
#define LPC1751_2      0x25001118
#define LPC1752        0x25001121
#define LPC1754        0x25011722
#define LPC1756        0x25011723
#define LPC1758        0x25013F37
#define LPC1759        0x25113737
#define LPC1763        0x26012033
#define LPC1764        0x26011922
#define LPC1765        0x26013733
#define LPC1766        0x26013F33
#define LPC1767        0x26012837
#define LPC1768        0x26013F37
#define LPC1769        0x26113F37
#define LPC1774        0x27011132
#define LPC1776        0x27191F43
#define LPC1777        0x27193747
#define LPC1778        0x27193F47
#define LPC1785        0x281D1743
#define LPC1786        0x281D1F43
#define LPC1787        0x281D3747
#define LPC1788        0x281D3F47

#define LPC4072        0x47011121
#define LPC4074        0x47011132
#define LPC4076        0x47191F43
#define LPC4078        0x47193F47
#define LPC4088        0x481D3F47

#define LPC810_021     0x00008100
#define LPC811_001     0x00008110
#define LPC812_101     0x00008120
#define LPC812_101_1   0x00008121
#define LPC812_101_2   0x00008122
#define LPC812_101_3   0x00008123

#define LPC822_101     0x00008221
#define LPC822_101_1   0x00008222
#define LPC824_201     0x00008241
#define LPC824_201_1   0x00008242

#define LPC8N04        0x00008A04
#define NHS3100        0x4e310020
#define NHS3152        0x4e315220
#define NHS3153        0x4e315320 /* Only specified in Rev.1 of the datasheet */

#define LPC844_201     0x00008441
#define LPC844_201_1   0x00008442
#define LPC844_201_2   0x00008444

#define LPC845_301     0x00008451
#define LPC845_301_1   0x00008452
#define LPC845_301_2   0x00008453
#define LPC845_301_3   0x00008454

#define IAP_CODE_LEN 0x34

#define LPC11XX_REG_SECTORS	24

typedef enum {
	LPC2000_V1,
	LPC2000_V2,
	LPC1700,
	LPC4300,
	LPC800,
	LPC1100,
	LPC1500,
	LPC54100,
	LPC_AUTO,
} lpc2000_variant;

struct lpc2000_flash_bank {
	lpc2000_variant variant;
	uint32_t cclk;
	int cmd51_dst_boundary;
	int calc_checksum;
	uint32_t cmd51_max_buffer;
	int checksum_vector;
	uint32_t iap_max_stack;
	uint32_t lpc4300_bank;
	uint32_t iap_entry_alternative;
	bool probed;
};

enum lpc2000_status_codes {
	LPC2000_CMD_SUCCESS = 0,
	LPC2000_INVALID_COMMAND = 1,
	LPC2000_SRC_ADDR_ERROR = 2,
	LPC2000_DST_ADDR_ERROR = 3,
	LPC2000_SRC_ADDR_NOT_MAPPED = 4,
	LPC2000_DST_ADDR_NOT_MAPPED = 5,
	LPC2000_COUNT_ERROR = 6,
	LPC2000_INVALID_SECTOR = 7,
	LPC2000_SECTOR_NOT_BLANK = 8,
	LPC2000_SECTOR_NOT_PREPARED = 9,
	LPC2000_COMPARE_ERROR = 10,
	LPC2000_BUSY = 11,
	LPC2000_PARAM_ERROR = 12,
	LPC2000_ADDR_ERROR = 13,
	LPC2000_ADDR_NOT_MAPPED = 14,
	LPC2000_CMD_NOT_LOCKED = 15,
	LPC2000_INVALID_CODE = 16,
	LPC2000_INVALID_BAUD_RATE = 17,
	LPC2000_INVALID_STOP_BIT = 18,
	LPC2000_CRP_ENABLED = 19,
	LPC2000_INVALID_FLASH_UNIT = 20,
	LPC2000_USER_CODE_CHECKSUM = 21,
	LCP2000_ERROR_SETTING_ACTIVE_PARTITION = 22,
};

static int lpc2000_build_sector_list(struct flash_bank *bank)
{
	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;
	uint32_t offset = 0;

	/* default to a 4096 write buffer */
	lpc2000_info->cmd51_max_buffer = 4096;

	if (lpc2000_info->variant == LPC2000_V1) {
		lpc2000_info->cmd51_dst_boundary = 512;
		lpc2000_info->checksum_vector = 5;
		lpc2000_info->iap_max_stack = 128;

		/* variant 1 has different layout for 128kb and 256kb flashes */
		if (bank->size == 128 * 1024) {
			bank->num_sectors = 16;
			bank->sectors = malloc(sizeof(struct flash_sector) * 16);
			for (int i = 0; i < 16; i++) {
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 8 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
		} else if (bank->size == 256 * 1024) {
			bank->num_sectors = 18;
			bank->sectors = malloc(sizeof(struct flash_sector) * 18);

			for (int i = 0; i < 8; i++) {
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 8 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
			for (int i = 8; i < 10; i++) {
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 64 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
			for (int i = 10; i < 18; i++) {
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 8 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
		} else {
			LOG_ERROR("BUG: unknown bank->size encountered");
			exit(-1);
		}
	} else if (lpc2000_info->variant == LPC2000_V2) {
		lpc2000_info->cmd51_dst_boundary = 256;
		lpc2000_info->checksum_vector = 5;
		lpc2000_info->iap_max_stack = 128;

		/* variant 2 has a uniform layout, only number of sectors differs */
		switch (bank->size) {
			case 4 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024;
				bank->num_sectors = 1;
				break;
			case 8 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024;
				bank->num_sectors = 2;
				break;
			case 16 * 1024:
				bank->num_sectors = 4;
				break;
			case 32 * 1024:
				bank->num_sectors = 8;
				break;
			case 64 * 1024:
				bank->num_sectors = 9;
				break;
			case 128 * 1024:
				bank->num_sectors = 11;
				break;
			case 256 * 1024:
				bank->num_sectors = 15;
				break;
			case 500 * 1024:
				bank->num_sectors = 27;
				break;
			case 512 * 1024:
			case 504 * 1024:
				bank->num_sectors = 28;
				break;
			default:
				LOG_ERROR("BUG: unknown bank->size encountered");
				exit(-1);
				break;
		}

		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			if (i < 8) {
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 4 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			} else if (i < 22) {
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 32 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			} else if (i < 28) {
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 4 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
		}
	} else if (lpc2000_info->variant == LPC1700) {
		lpc2000_info->cmd51_dst_boundary = 256;
		lpc2000_info->checksum_vector = 7;
		lpc2000_info->iap_max_stack = 128;

		switch (bank->size) {
			case 4 * 1024:
				lpc2000_info->cmd51_max_buffer = 256;
				bank->num_sectors = 1;
				break;
			case 8 * 1024:
				lpc2000_info->cmd51_max_buffer = 512;
				bank->num_sectors = 2;
				break;
			case 16 * 1024:
				lpc2000_info->cmd51_max_buffer = 512;
				bank->num_sectors = 4;
				break;
			case 32 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024;
				bank->num_sectors = 8;
				break;
			case 64 * 1024:
				bank->num_sectors = 16;
				break;
			case 128 * 1024:
				bank->num_sectors = 18;
			break;
			case 256 * 1024:
				bank->num_sectors = 22;
				break;
			case 512 * 1024:
				bank->num_sectors = 30;
				break;
			default:
				LOG_ERROR("BUG: unknown bank->size encountered");
				exit(-1);
		}

		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			/* sectors 0-15 are 4kB-sized, 16 and above are 32kB-sized for LPC17xx/LPC40xx devices */
			bank->sectors[i].size = (i < 16) ? 4 * 1024 : 32 * 1024;
			offset += bank->sectors[i].size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;
		}
	} else if (lpc2000_info->variant == LPC4300) {
		lpc2000_info->cmd51_dst_boundary = 512;
		lpc2000_info->checksum_vector = 7;
		lpc2000_info->iap_max_stack = 208;

		switch (bank->size) {
			case 256 * 1024:
				bank->num_sectors = 11;
				break;
			case 384 * 1024:
				bank->num_sectors = 13;
				break;
			case 512 * 1024:
				bank->num_sectors = 15;
				break;
			default:
				LOG_ERROR("BUG: unknown bank->size encountered");
				exit(-1);
		}

		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			/* sectors 0-7 are 8kB-sized, 8 and above are 64kB-sized for LPC43xx devices */
			bank->sectors[i].size = (i < 8) ? 8 * 1024 : 64 * 1024;
			offset += bank->sectors[i].size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;
		}

	} else if (lpc2000_info->variant == LPC800) {
		lpc2000_info->cmd51_dst_boundary = 64;
		lpc2000_info->checksum_vector = 7;
		lpc2000_info->iap_max_stack = 208;		/* 148byte for LPC81x,208byte for LPC82x. */
		lpc2000_info->cmd51_max_buffer = 256;	/* smallest MCU in the series, LPC810, has 1 kB of SRAM */

		switch (bank->size) {
			case 4 * 1024:
				bank->num_sectors = 4;
				break;
			case 8 * 1024:
				bank->num_sectors = 8;
				break;
			case 16 * 1024:
				bank->num_sectors = 16;
				break;
			case 30 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024;	/* For LPC8N04 and NHS31xx, have 8kB of SRAM */
				bank->num_sectors = 30;			/* There have only 30kB of writable Flash out of 32kB */
				break;
			case 32 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024; /* For LPC824, has 8kB of SRAM */
				bank->num_sectors = 32;
				break;
			case 64 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024; /* For LPC844, has 8kB of SRAM */
				bank->num_sectors = 64;
				break;
			default:
				LOG_ERROR("BUG: unknown bank->size encountered");
				exit(-1);
		}

		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			/* all sectors are 1kB-sized for LPC8xx devices */
			bank->sectors[i].size = 1 * 1024;
			offset += bank->sectors[i].size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;
		}

	} else if (lpc2000_info->variant == LPC1100) {
		lpc2000_info->cmd51_dst_boundary = 256;
		lpc2000_info->checksum_vector = 7;
		lpc2000_info->iap_max_stack = 128;

		if ((bank->size % (4 * 1024)) != 0) {
			LOG_ERROR("BUG: unknown bank->size encountered,\nLPC1100 flash size must be a multiple of 4096");
			exit(-1);
		}
		lpc2000_info->cmd51_max_buffer = 512; /* smallest MCU in the series, LPC1110, has 1 kB of SRAM */
		unsigned int large_sectors = 0;
		unsigned int normal_sectors = bank->size / 4096;

		if (normal_sectors > LPC11XX_REG_SECTORS) {
			large_sectors = (normal_sectors - LPC11XX_REG_SECTORS) / 8;
			normal_sectors = LPC11XX_REG_SECTORS;
		}

		bank->num_sectors = normal_sectors + large_sectors;

		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			bank->sectors[i].size = (i < LPC11XX_REG_SECTORS ? 4 : 32) * 1024;
			offset += bank->sectors[i].size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;
		}

	} else if (lpc2000_info->variant == LPC1500) {
		lpc2000_info->cmd51_dst_boundary = 256;
		lpc2000_info->checksum_vector = 7;
		lpc2000_info->iap_max_stack = 128;

		switch (bank->size) {
			case 64 * 1024:
				bank->num_sectors = 16;
				break;
			case 128 * 1024:
				bank->num_sectors = 32;
				break;
			case 256 * 1024:
				bank->num_sectors = 64;
				break;
			default:
				LOG_ERROR("BUG: unknown bank->size encountered");
				exit(-1);
		}

		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			/* all sectors are 4kB-sized */
			bank->sectors[i].size = 4 * 1024;
			offset += bank->sectors[i].size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;
		}

	} else if (lpc2000_info->variant == LPC54100) {
		lpc2000_info->cmd51_dst_boundary = 256;
		lpc2000_info->checksum_vector = 7;
		lpc2000_info->iap_max_stack = 128;

		switch (bank->size) {
			case 256 * 1024:
				bank->num_sectors = 8;
				break;
			case 512 * 1024:
				bank->num_sectors = 16;
				break;
			default:
				LOG_ERROR("BUG: unknown bank->size encountered");
				exit(-1);
		}

		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = offset;
			/* all sectors are 32kB-sized */
			bank->sectors[i].size = 32 * 1024;
			offset += bank->sectors[i].size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 1;
		}

	} else {
		LOG_ERROR("BUG: unknown lpc2000_info->variant encountered");
		exit(-1);
	}

	return ERROR_OK;
}

/* this function allocates and initializes working area used for IAP algorithm
 * uses 52 + max IAP stack bytes working area
 * 0x0 to 0x7: jump gate (BX to thumb state, b -2 to wait)
 * 0x8 to 0x1f: command parameter table (1+5 words)
 * 0x20 to 0x33: command result table (1+4 words)
 * 0x34 to 0xb3|0x104: stack
 *        (128b needed for lpc1xxx/2000/5410x, 208b for lpc43xx/lpc82x and 148b for lpc81x)
 */

static int lpc2000_iap_working_area_init(struct flash_bank *bank, struct working_area **iap_working_area)
{
	struct target *target = bank->target;
	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;

	if (target_alloc_working_area(target, IAP_CODE_LEN + lpc2000_info->iap_max_stack, iap_working_area) != ERROR_OK) {
		LOG_ERROR("no working area specified, can't write LPC2000 internal flash");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	uint8_t jump_gate[8];

	/* write IAP code to working area */
	switch (lpc2000_info->variant) {
		case LPC800:
		case LPC1100:
		case LPC1500:
		case LPC1700:
		case LPC4300:
		case LPC54100:
		case LPC_AUTO:
			target_buffer_set_u32(target, jump_gate, ARMV4_5_T_BX(12));
			target_buffer_set_u32(target, jump_gate + 4, ARMV5_T_BKPT(0));
			break;
		case LPC2000_V1:
		case LPC2000_V2:
			target_buffer_set_u32(target, jump_gate, ARMV4_5_BX(12));
			target_buffer_set_u32(target, jump_gate + 4, ARMV4_5_B(0xfffffe, 0));
			break;
		default:
			LOG_ERROR("BUG: unknown lpc2000_info->variant encountered");
			exit(-1);
	}

	int retval = target_write_memory(target, (*iap_working_area)->address, 4, 2, jump_gate);
	if (retval != ERROR_OK) {
		LOG_ERROR("Write memory at address " TARGET_ADDR_FMT " failed (check work_area definition)",
				(*iap_working_area)->address);
		target_free_working_area(target, *iap_working_area);
	}

	return retval;
}

/* call LPC8xx/LPC1xxx/LPC4xxx/LPC5410x/LPC2000 IAP function */

static int lpc2000_iap_call(struct flash_bank *bank, struct working_area *iap_working_area, int code,
		uint32_t param_table[5], uint32_t result_table[4])
{
	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;
	struct target *target = bank->target;

	struct arm_algorithm arm_algo;	/* for LPC2000 */
	struct armv7m_algorithm armv7m_info;	/* for LPC8xx/LPC1xxx/LPC4xxx/LPC5410x */
	uint32_t iap_entry_point = 0;	/* to make compiler happier */

	switch (lpc2000_info->variant) {
		case LPC800:
		case LPC1100:
		case LPC1700:
		case LPC_AUTO:
			armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
			armv7m_info.core_mode = ARM_MODE_THREAD;
			iap_entry_point = 0x1fff1ff1;
			break;
		case LPC1500:
		case LPC54100:
			armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
			armv7m_info.core_mode = ARM_MODE_THREAD;
			iap_entry_point = 0x03000205;
			break;
		case LPC2000_V1:
		case LPC2000_V2:
			arm_algo.common_magic = ARM_COMMON_MAGIC;
			arm_algo.core_mode = ARM_MODE_SVC;
			arm_algo.core_state = ARM_STATE_ARM;
			iap_entry_point = 0x7ffffff1;
			break;
		case LPC4300:
			armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
			armv7m_info.core_mode = ARM_MODE_THREAD;
			/* read out IAP entry point from ROM driver table at 0x10400100 */
			target_read_u32(target, 0x10400100, &iap_entry_point);
			break;
		default:
			LOG_ERROR("BUG: unknown lpc2000->variant encountered");
			exit(-1);
	}

	if (lpc2000_info->iap_entry_alternative != 0x0)
		iap_entry_point = lpc2000_info->iap_entry_alternative;

	struct mem_param mem_params[2];

	/* command parameter table */
	init_mem_param(&mem_params[0], iap_working_area->address + 8, 6 * 4, PARAM_OUT);
	target_buffer_set_u32(target, mem_params[0].value, code);
	target_buffer_set_u32(target, mem_params[0].value + 0x04, param_table[0]);
	target_buffer_set_u32(target, mem_params[0].value + 0x08, param_table[1]);
	target_buffer_set_u32(target, mem_params[0].value + 0x0c, param_table[2]);
	target_buffer_set_u32(target, mem_params[0].value + 0x10, param_table[3]);
	target_buffer_set_u32(target, mem_params[0].value + 0x14, param_table[4]);

	struct reg_param reg_params[5];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, iap_working_area->address + 0x08);

	/* command result table */
	init_mem_param(&mem_params[1], iap_working_area->address + 0x20, 5 * 4, PARAM_IN);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, iap_working_area->address + 0x20);

	/* IAP entry point */
	init_reg_param(&reg_params[2], "r12", 32, PARAM_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, iap_entry_point);

	switch (lpc2000_info->variant) {
		case LPC800:
		case LPC1100:
		case LPC1500:
		case LPC1700:
		case LPC4300:
		case LPC54100:
		case LPC_AUTO:
			/* IAP stack */
			init_reg_param(&reg_params[3], "sp", 32, PARAM_OUT);
			buf_set_u32(reg_params[3].value, 0, 32,
				iap_working_area->address + IAP_CODE_LEN + lpc2000_info->iap_max_stack);

			/* return address */
			init_reg_param(&reg_params[4], "lr", 32, PARAM_OUT);
			buf_set_u32(reg_params[4].value, 0, 32, (iap_working_area->address + 0x04) | 1);
			/* bit0 of LR = 1 to return in Thumb mode */

			target_run_algorithm(target, 2, mem_params, 5, reg_params, iap_working_area->address, 0, 10000,
					&armv7m_info);
			break;
		case LPC2000_V1:
		case LPC2000_V2:
			/* IAP stack */
			init_reg_param(&reg_params[3], "sp_svc", 32, PARAM_OUT);
			buf_set_u32(reg_params[3].value, 0, 32,
				iap_working_area->address + IAP_CODE_LEN + lpc2000_info->iap_max_stack);

			/* return address */
			init_reg_param(&reg_params[4], "lr_svc", 32, PARAM_OUT);
			buf_set_u32(reg_params[4].value, 0, 32, iap_working_area->address + 0x04);

			target_run_algorithm(target, 2, mem_params, 5, reg_params, iap_working_area->address,
					iap_working_area->address + 0x4, 10000, &arm_algo);
			break;
		default:
			LOG_ERROR("BUG: unknown lpc2000->variant encountered");
			exit(-1);
	}

	int status_code = target_buffer_get_u32(target, mem_params[1].value);
	result_table[0] = target_buffer_get_u32(target, mem_params[1].value + 0x04);
	result_table[1] = target_buffer_get_u32(target, mem_params[1].value + 0x08);
	result_table[2] = target_buffer_get_u32(target, mem_params[1].value + 0x0c);
	result_table[3] = target_buffer_get_u32(target, mem_params[1].value + 0x10);

	LOG_DEBUG("IAP command = %i (0x%8.8" PRIx32 ", 0x%8.8" PRIx32 ", 0x%8.8" PRIx32 ", 0x%8.8" PRIx32 ", 0x%8.8" PRIx32
			") completed with result = %8.8x",
			code, param_table[0], param_table[1], param_table[2], param_table[3], param_table[4], status_code);

	destroy_mem_param(&mem_params[0]);
	destroy_mem_param(&mem_params[1]);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return status_code;
}

static int lpc2000_iap_blank_check(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	if (last >= bank->num_sectors)
		return ERROR_FLASH_SECTOR_INVALID;

	uint32_t param_table[5] = {0};
	uint32_t result_table[4];
	struct working_area *iap_working_area;

	int retval = lpc2000_iap_working_area_init(bank, &iap_working_area);

	if (retval != ERROR_OK)
		return retval;

	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;
	if (lpc2000_info->variant == LPC4300)
		param_table[2] = lpc2000_info->lpc4300_bank;

	for (unsigned int i = first; i <= last && retval == ERROR_OK; i++) {
		/* check single sector */
		param_table[0] = param_table[1] = i;
		int status_code = lpc2000_iap_call(bank, iap_working_area, 53, param_table, result_table);

		switch (status_code) {
			case ERROR_FLASH_OPERATION_FAILED:
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			case LPC2000_CMD_SUCCESS:
				bank->sectors[i].is_erased = 1;
				break;
			case LPC2000_SECTOR_NOT_BLANK:
				bank->sectors[i].is_erased = 0;
				break;
			case LPC2000_INVALID_SECTOR:
				bank->sectors[i].is_erased = 0;
				break;
			case LPC2000_BUSY:
				retval = ERROR_FLASH_BUSY;
				break;
			default:
				LOG_ERROR("BUG: unknown LPC2000 status code %i", status_code);
				exit(-1);
		}
	}

	struct target *target = bank->target;
	target_free_working_area(target, iap_working_area);

	return retval;
}

/*
 * flash bank lpc2000 <base> <size> 0 0 <target#> <lpc_variant> <cclk> [calc_checksum]
 */
FLASH_BANK_COMMAND_HANDLER(lpc2000_flash_bank_command)
{
	if (CMD_ARGC < 8)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct lpc2000_flash_bank *lpc2000_info = calloc(1, sizeof(*lpc2000_info));
	lpc2000_info->probed = false;

	bank->driver_priv = lpc2000_info;

	if (strcmp(CMD_ARGV[6], "lpc2000_v1") == 0) {
		lpc2000_info->variant = LPC2000_V1;
	} else if (strcmp(CMD_ARGV[6], "lpc2000_v2") == 0) {
		lpc2000_info->variant = LPC2000_V2;
	} else if (strcmp(CMD_ARGV[6], "lpc1700") == 0 || strcmp(CMD_ARGV[6], "lpc4000") == 0) {
		lpc2000_info->variant = LPC1700;
	} else if (strcmp(CMD_ARGV[6], "lpc1800") == 0 || strcmp(CMD_ARGV[6], "lpc4300") == 0) {
		lpc2000_info->variant = LPC4300;
	} else if (strcmp(CMD_ARGV[6], "lpc800") == 0) {
		lpc2000_info->variant = LPC800;
	} else if (strcmp(CMD_ARGV[6], "lpc1100") == 0) {
		lpc2000_info->variant = LPC1100;
	} else if (strcmp(CMD_ARGV[6], "lpc1500") == 0) {
		lpc2000_info->variant = LPC1500;
	} else if (strcmp(CMD_ARGV[6], "lpc54100") == 0) {
		lpc2000_info->variant = LPC54100;
	} else if (strcmp(CMD_ARGV[6], "auto") == 0) {
		lpc2000_info->variant = LPC_AUTO;
	} else {
		LOG_ERROR("unknown LPC2000 variant: %s", CMD_ARGV[6]);
		free(lpc2000_info);
		return ERROR_FLASH_BANK_INVALID;
	}

	/* Maximum size required for the IAP stack.
	   This value only gets used when probing, only for auto, lpc1100 and lpc1700.
	   We use the maximum size for any part supported by the driver(!) to be safe
	   in case the auto variant is mistakenly used on a MCU from one of the series
	   for which we don't support auto-probing. */
	lpc2000_info->iap_max_stack = 208;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], lpc2000_info->cclk);
	lpc2000_info->calc_checksum = 0;

	uint32_t temp_base = 0;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], temp_base);
	if (temp_base >= 0x1B000000)
		lpc2000_info->lpc4300_bank = 1; /* bank B */
	else
		lpc2000_info->lpc4300_bank = 0; /* bank A */

	if (CMD_ARGC >= 9) {
		if (strcmp(CMD_ARGV[8], "calc_checksum") == 0)
			lpc2000_info->calc_checksum = 1;
	}
	if (CMD_ARGC >= 10 && !lpc2000_info->iap_entry_alternative)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[9], lpc2000_info->iap_entry_alternative);

	return ERROR_OK;
}

static int lpc2000_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;
	uint32_t param_table[5] = {0};

	param_table[0] = first;
	param_table[1] = last;

	if (lpc2000_info->variant == LPC4300)
		param_table[2] = lpc2000_info->lpc4300_bank;
	else
		param_table[2] = lpc2000_info->cclk;

	uint32_t result_table[4];
	struct working_area *iap_working_area;

	int retval = lpc2000_iap_working_area_init(bank, &iap_working_area);

	if (retval != ERROR_OK)
		return retval;

	if (lpc2000_info->variant == LPC4300)
		/* Init IAP Anyway */
		lpc2000_iap_call(bank, iap_working_area, 49, param_table, result_table);

	/* Prepare sectors */
	int status_code = lpc2000_iap_call(bank, iap_working_area, 50, param_table, result_table);
	switch (status_code) {
		case ERROR_FLASH_OPERATION_FAILED:
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		case LPC2000_CMD_SUCCESS:
			break;
		case LPC2000_INVALID_SECTOR:
			retval = ERROR_FLASH_SECTOR_INVALID;
			break;
		default:
			LOG_WARNING("lpc2000 prepare sectors returned %i", status_code);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
	}

	if (retval == ERROR_OK) {
		/* Erase sectors */
		param_table[2] = lpc2000_info->cclk;
		if (lpc2000_info->variant == LPC4300)
			param_table[3] = lpc2000_info->lpc4300_bank;

		status_code = lpc2000_iap_call(bank, iap_working_area, 52, param_table, result_table);
		switch (status_code) {
			case ERROR_FLASH_OPERATION_FAILED:
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			case LPC2000_CMD_SUCCESS:
				break;
			case LPC2000_INVALID_SECTOR:
				retval = ERROR_FLASH_SECTOR_INVALID;
				break;
			default:
				LOG_WARNING("lpc2000 erase sectors returned %i", status_code);
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
		}
	}

	struct target *target = bank->target;
	target_free_working_area(target, iap_working_area);

	return retval;
}

static int lpc2000_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;

	uint32_t dst_min_alignment = lpc2000_info->cmd51_dst_boundary;

	if (offset % dst_min_alignment) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required alignment 0x%" PRIx32, offset, dst_min_alignment);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	int first_sector = 0;
	int last_sector = 0;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (offset >= bank->sectors[i].offset)
			first_sector = i;
		if (offset + DIV_ROUND_UP(count, dst_min_alignment) * dst_min_alignment > bank->sectors[i].offset)
			last_sector = i;
	}

	LOG_DEBUG("first_sector: %i, last_sector: %i", first_sector, last_sector);

	/* check if exception vectors should be flashed */
	if ((offset == 0) && (count >= 0x20) && lpc2000_info->calc_checksum) {
		assert(lpc2000_info->checksum_vector < 8);
		uint32_t checksum = 0;
		for (int i = 0; i < 8; i++) {
			LOG_DEBUG("Vector 0x%2.2x: 0x%8.8" PRIx32, i * 4, buf_get_u32(buffer + (i * 4), 0, 32));
			if (i != lpc2000_info->checksum_vector)
				checksum += buf_get_u32(buffer + (i * 4), 0, 32);
		}
		checksum = 0 - checksum;
		LOG_DEBUG("checksum: 0x%8.8" PRIx32, checksum);

		uint32_t original_value = buf_get_u32(buffer + (lpc2000_info->checksum_vector * 4), 0, 32);
		if (original_value != checksum) {
			LOG_WARNING("Boot verification checksum in image (0x%8.8" PRIx32 ") to be written to flash is "
					"different from calculated vector checksum (0x%8.8" PRIx32 ").", original_value, checksum);
			LOG_WARNING("OpenOCD will write the correct checksum. To remove this warning modify build tools on developer PC to inject correct LPC vector "
					"checksum.");
		}

		/* FIXME: WARNING! This code is broken because it modifies the callers buffer in place. */
		buf_set_u32((uint8_t *)buffer + (lpc2000_info->checksum_vector * 4), 0, 32, checksum);
	}

	struct working_area *iap_working_area;

	int retval = lpc2000_iap_working_area_init(bank, &iap_working_area);

	if (retval != ERROR_OK)
		return retval;

	struct working_area *download_area;

	/* allocate a working area */
	if (target_alloc_working_area(target, lpc2000_info->cmd51_max_buffer, &download_area) != ERROR_OK) {
		LOG_ERROR("no working area specified, can't write LPC2000 internal flash");
		target_free_working_area(target, iap_working_area);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	uint32_t bytes_remaining = count;
	uint32_t bytes_written = 0;
	uint32_t param_table[5] = {0};
	uint32_t result_table[4];

	if (lpc2000_info->variant == LPC4300)
		/* Init IAP Anyway */
		lpc2000_iap_call(bank, iap_working_area, 49, param_table, result_table);

	while (bytes_remaining > 0) {
		uint32_t thisrun_bytes;
		if (bytes_remaining >= lpc2000_info->cmd51_max_buffer)
			thisrun_bytes = lpc2000_info->cmd51_max_buffer;
		else
			thisrun_bytes = lpc2000_info->cmd51_dst_boundary;

		/* Prepare sectors */
		param_table[0] = first_sector;
		param_table[1] = last_sector;

		if (lpc2000_info->variant == LPC4300)
			param_table[2] = lpc2000_info->lpc4300_bank;
		else
			param_table[2] = lpc2000_info->cclk;

		int status_code = lpc2000_iap_call(bank, iap_working_area, 50, param_table, result_table);
		switch (status_code) {
			case ERROR_FLASH_OPERATION_FAILED:
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			case LPC2000_CMD_SUCCESS:
				break;
			case LPC2000_INVALID_SECTOR:
				retval = ERROR_FLASH_SECTOR_INVALID;
				break;
			default:
				LOG_WARNING("lpc2000 prepare sectors returned %i", status_code);
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
		}

		/* Exit if error occurred */
		if (retval != ERROR_OK)
			break;

		if (bytes_remaining >= thisrun_bytes) {
			retval = target_write_buffer(bank->target, download_area->address, thisrun_bytes, buffer + bytes_written);
			if (retval != ERROR_OK) {
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			}
		} else {
			uint8_t *last_buffer = malloc(thisrun_bytes);
			memcpy(last_buffer, buffer + bytes_written, bytes_remaining);
			memset(last_buffer + bytes_remaining, 0xff, thisrun_bytes - bytes_remaining);
			target_write_buffer(bank->target, download_area->address, thisrun_bytes, last_buffer);
			free(last_buffer);
		}

		LOG_DEBUG("writing 0x%" PRIx32 " bytes to address " TARGET_ADDR_FMT,
				thisrun_bytes, bank->base + offset + bytes_written);

		/* Write data */
		param_table[0] = bank->base + offset + bytes_written;
		param_table[1] = download_area->address;
		param_table[2] = thisrun_bytes;
		param_table[3] = lpc2000_info->cclk;
		status_code = lpc2000_iap_call(bank, iap_working_area, 51, param_table, result_table);
		switch (status_code) {
			case ERROR_FLASH_OPERATION_FAILED:
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			case LPC2000_CMD_SUCCESS:
				break;
			case LPC2000_INVALID_SECTOR:
				retval = ERROR_FLASH_SECTOR_INVALID;
				break;
			default:
				LOG_WARNING("lpc2000 returned %i", status_code);
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
		}

		/* Exit if error occurred */
		if (retval != ERROR_OK)
			break;

		if (bytes_remaining > thisrun_bytes)
			bytes_remaining -= thisrun_bytes;
		else
			bytes_remaining = 0;
		bytes_written += thisrun_bytes;
	}

	target_free_working_area(target, iap_working_area);
	target_free_working_area(target, download_area);

	return retval;
}

static int get_lpc2000_part_id(struct flash_bank *bank, uint32_t *part_id)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t param_table[5] = {0};
	uint32_t result_table[4];
	struct working_area *iap_working_area;

	int retval = lpc2000_iap_working_area_init(bank, &iap_working_area);

	if (retval != ERROR_OK)
		return retval;

	/* The status seems to be bogus with the part ID command on some IAP
	   firmwares, so ignore it. */
	lpc2000_iap_call(bank, iap_working_area, 54, param_table, result_table);

	struct target *target = bank->target;
	target_free_working_area(target, iap_working_area);

	/* If the result is zero, the command probably didn't work out. */
	if (result_table[0] == 0)
		return LPC2000_INVALID_COMMAND;

	*part_id = result_table[0];
	return LPC2000_CMD_SUCCESS;
}

static int lpc2000_auto_probe_flash(struct flash_bank *bank)
{
	uint32_t part_id;
	int retval;
	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = get_lpc2000_part_id(bank, &part_id);
	if (retval != LPC2000_CMD_SUCCESS) {
		LOG_ERROR("Could not get part ID");
		return retval;
	}

	switch (part_id) {
		case LPC1110_1:
		case LPC1110_2:
			lpc2000_info->variant = LPC1100;
			bank->size = 4 * 1024;
			break;

		case LPC1111_002_1:
		case LPC1111_002_2:
		case LPC1111_101_1:
		case LPC1111_101_2:
		case LPC1111_103_1:
		case LPC1111_201_1:
		case LPC1111_201_2:
		case LPC1111_203_1:
		case LPC11A11_001_1:
		case LPC11E11_101:
		case LPC1311:
		case LPC1311_1:
			lpc2000_info->variant = LPC1100;
			bank->size = 8 * 1024;
			break;

		case LPC1112_101_1:
		case LPC1112_101_2:
		case LPC1112_102_1:
		case LPC1112_102_2:
		case LPC1112_103_1:
		case LPC1112_201_1:
		case LPC1112_201_2:
		case LPC1112_203_1:
		case LPC11A02_1:
		case LPC11C12_301_1:
		case LPC11C22_301_1:
		case LPC11A12_101_1:
		case LPC11E12_201:
		case LPC11U12_201_1:
		case LPC11U12_201_2:
		case LPC1342:
			lpc2000_info->variant = LPC1100;
			bank->size = 16 * 1024;
			break;

		case LPC1113_201_1:
		case LPC1113_201_2:
		case LPC1113_203_1:
		case LPC1113_301_1:
		case LPC1113_301_2:
		case LPC1113_303_1:
		case LPC11A13_201_1:
		case LPC11E13_301:
		case LPC11U13_201_1:
		case LPC11U13_201_2:
		case LPC11U23_301:
			lpc2000_info->variant = LPC1100;
			bank->size = 24 * 1024;
			break;

		case LPC1114_102_1:
		case LPC1114_102_2:
		case LPC1114_201_1:
		case LPC1114_201_2:
		case LPC1114_203_1:
		case LPC1114_301_1:
		case LPC1114_301_2:
		case LPC1114_303_1:
		case LPC11A04_1:
		case LPC11A14_301_1:
		case LPC11A14_301_2:
		case LPC11C14_301_1:
		case LPC11C24_301_1:
		case LPC11E14_401:
		case LPC11U14_201_1:
		case LPC11U14_201_2:
		case LPC11U24_301:
		case LPC11U24_401:
		case LPC1313:
		case LPC1313_1:
		case LPC1315:
		case LPC1343:
		case LPC1343_1:
		case LPC1345:
			lpc2000_info->variant = LPC1100;
			bank->size = 32 * 1024;
			break;

		case LPC1751_1:
		case LPC1751_2:
			lpc2000_info->variant = LPC1700;
			bank->size = 32 * 1024;
			break;

		case LPC11U34_311:
			lpc2000_info->variant = LPC1100;
			bank->size = 40 * 1024;
			break;

		case LPC1114_323_1:
		case LPC11U34_421:
		case LPC1316:
		case LPC1346:
			lpc2000_info->variant = LPC1100;
			bank->size = 48 * 1024;
			break;

		case LPC1114_333_1:
			lpc2000_info->variant = LPC1100;
			bank->size = 56 * 1024;
			break;

		case LPC1115_303_1:
		case LPC11U35_401:
		case LPC11U35_501:
		case LPC11E66:
		case LPC11U66:
		case LPC1317:
		case LPC1347:
			lpc2000_info->variant = LPC1100;
			bank->size = 64 * 1024;
			break;

		case LPC1752:
		case LPC4072:
			lpc2000_info->variant = LPC1700;
			bank->size = 64 * 1024;
			break;

		case LPC11E36_501:
		case LPC11U36_401:
			lpc2000_info->variant = LPC1100;
			bank->size = 96 * 1024;
			break;

		case LPC11E37_401:
		case LPC11E37_501:
		case LPC11U37_401:
		case LPC11U37H_401:
		case LPC11U37_501:
		case LPC11E67:
		case LPC11E68:
		case LPC11U67_1:
		case LPC11U67_2:
			lpc2000_info->variant = LPC1100;
			bank->size = 128 * 1024;
			break;

		case LPC1754:
		case LPC1764:
		case LPC1774:
		case LPC4074:
			lpc2000_info->variant = LPC1700;
			bank->size = 128 * 1024;
			break;

		case LPC11U68_1:
		case LPC11U68_2:
			lpc2000_info->variant = LPC1100;
			bank->size = 256 * 1024;
			break;

		case LPC1756:
		case LPC1763:
		case LPC1765:
		case LPC1766:
		case LPC1776:
		case LPC1785:
		case LPC1786:
		case LPC4076:
			lpc2000_info->variant = LPC1700;
			bank->size = 256 * 1024;
			break;

		case LPC1758:
		case LPC1759:
		case LPC1767:
		case LPC1768:
		case LPC1769:
		case LPC1777:
		case LPC1778:
		case LPC1787:
		case LPC1788:
		case LPC4078:
		case LPC4088:
			lpc2000_info->variant = LPC1700;
			bank->size = 512 * 1024;
			break;

		case LPC810_021:
			lpc2000_info->variant = LPC800;
			bank->size = 4 * 1024;
			break;

		case LPC811_001:
			lpc2000_info->variant = LPC800;
			bank->size = 8 * 1024;
			break;

		case LPC812_101:
		case LPC812_101_1:
		case LPC812_101_2:
		case LPC812_101_3:
		case LPC822_101:
		case LPC822_101_1:
			lpc2000_info->variant = LPC800;
			bank->size = 16 * 1024;
			break;

		case LPC824_201:
		case LPC824_201_1:
			lpc2000_info->variant = LPC800;
			bank->size = 32 * 1024;
			break;

		case LPC8N04:
		case NHS3100:
		case NHS3152:
		case NHS3153:
			lpc2000_info->variant = LPC800;
			bank->size = 30 * 1024;
			break;

		case LPC844_201:
		case LPC844_201_1:
		case LPC844_201_2:
		case LPC845_301:
		case LPC845_301_1:
		case LPC845_301_2:
		case LPC845_301_3:
			lpc2000_info->variant = LPC800;
			bank->size = 64 * 1024;
			break;

		default:
			LOG_ERROR("BUG: unknown Part ID encountered: 0x%" PRIx32, part_id);
			exit(-1);
	}

	return ERROR_OK;
}

static int lpc2000_probe(struct flash_bank *bank)
{
	int status;
	uint32_t part_id;
	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;

	if (!lpc2000_info->probed) {
		if (lpc2000_info->variant == LPC_AUTO) {
			status = lpc2000_auto_probe_flash(bank);
			if (status != ERROR_OK)
				return status;
		} else if (lpc2000_info->variant == LPC1100 || lpc2000_info->variant == LPC1700) {
			status = get_lpc2000_part_id(bank, &part_id);
			if (status == LPC2000_CMD_SUCCESS)
				LOG_INFO("If auto-detection fails for this part, please email "
					"openocd-devel@lists.sourceforge.net, citing part id 0x%" PRIx32 ".\n", part_id);
		}

		lpc2000_build_sector_list(bank);
		lpc2000_info->probed = true;
	}

	return ERROR_OK;
}

static int lpc2000_erase_check(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return lpc2000_iap_blank_check(bank, 0, bank->num_sectors - 1);
}

static int get_lpc2000_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct lpc2000_flash_bank *lpc2000_info = bank->driver_priv;

	command_print_sameline(cmd, "lpc2000 flash driver variant: %i, clk: %" PRIu32 "kHz",
			lpc2000_info->variant, lpc2000_info->cclk);

	return ERROR_OK;
}

COMMAND_HANDLER(lpc2000_handle_part_id_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t part_id;
	int status_code = get_lpc2000_part_id(bank, &part_id);
	if (status_code != 0x0) {
		if (status_code == ERROR_FLASH_OPERATION_FAILED) {
			command_print(CMD, "no sufficient working area specified, can't access LPC2000 IAP interface");
		} else
			command_print(CMD, "lpc2000 IAP returned status code %i", status_code);
	} else
		command_print(CMD, "lpc2000 part id: 0x%8.8" PRIx32, part_id);

	return retval;
}

static const struct command_registration lpc2000_exec_command_handlers[] = {
	{
		.name = "part_id",
		.handler = lpc2000_handle_part_id_command,
		.mode = COMMAND_EXEC,
		.help = "print part id of lpc2000 flash bank <num>",
		.usage = "<bank>",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration lpc2000_command_handlers[] = {
	{
		.name = "lpc2000",
		.mode = COMMAND_ANY,
		.help = "lpc2000 flash command group",
		.usage = "",
		.chain = lpc2000_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver lpc2000_flash = {
	.name = "lpc2000",
	.commands = lpc2000_command_handlers,
	.flash_bank_command = lpc2000_flash_bank_command,
	.erase = lpc2000_erase,
	.write = lpc2000_write,
	.read = default_flash_read,
	.probe = lpc2000_probe,
	.auto_probe = lpc2000_probe,
	.erase_check = lpc2000_erase_check,
	.info = get_lpc2000_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
