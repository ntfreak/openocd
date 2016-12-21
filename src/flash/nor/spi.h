/***************************************************************************
 *   Copyright (C) 2018-2019 by Andreas Bolsch                             *
 *   andreas.bolsch@mni.thm.de                                             *
 *                                                                         *
 *   Copyright (C) 2012 by George Harris                                   *
 *   george@luminairecoffee.com                                            *
 *                                                                         *
 *   Copyright (C) 2010 by Antonio Borneo                                  *
 *   borneo.antonio@gmail.com                                              *
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

#ifndef OPENOCD_FLASH_NOR_SPI_H
#define OPENOCD_FLASH_NOR_SPI_H

#ifndef __ASSEMBLER__

/* data structure to maintain flash ids from different vendors */
struct flash_device {
	const char *name;
	uint8_t read_cmd;
	uint8_t qread_cmd;
	uint8_t pprog_cmd;
	uint8_t erase_cmd;
	uint8_t chip_erase_cmd;
	uint32_t device_id;
	uint32_t pagesize;
	uint32_t sectorsize;
	uint32_t size_in_bytes;
};

#define FLASH_ID(n, re, qr, pp, es, ces, id, psize, ssize, size) \
{	                                \
	.name = n,                      \
	.read_cmd = re,                 \
	.qread_cmd = qr,                \
	.pprog_cmd = pp,                \
	.erase_cmd = es,                \
	.chip_erase_cmd = ces,          \
	.device_id = id,                \
	.pagesize = psize,              \
	.sectorsize = ssize,            \
	.size_in_bytes = size,          \
}

#define FRAM_ID(n, re, qr, pp, id, size) \
{	                                \
	.name = n,                      \
	.read_cmd = re,                 \
	.qread_cmd = qr,                \
	.pprog_cmd = pp,                \
	.erase_cmd = 0x00,              \
	.chip_erase_cmd = 0x00,         \
	.device_id = id,                \
	.pagesize = 0,                  \
	.sectorsize = 0,                \
	.size_in_bytes = size,          \
}

extern const struct flash_device flash_devices[];

#endif

/* fields in SPI flash status register */
#define	SPIFLASH_BSY		0
#define SPIFLASH_BSY_BIT	(1 << SPIFLASH_BSY)	/* WIP Bit of SPI SR */
#define	SPIFLASH_WE			1
#define SPIFLASH_WE_BIT		(1 << SPIFLASH_WE)	/* WEL Bit of SPI SR */

/* SPI Flash Commands */
#define SPIFLASH_READ_ID		0x9F /* Read Flash Identification */
#define SPIFLASH_READ_MID		0xAF /* Read Flash Identification, multi-io */
#define SPIFLASH_READ_STATUS	0x05 /* Read Status Register */
#define SPIFLASH_WRITE_ENABLE	0x06 /* Write Enable */
#define SPIFLASH_PAGE_PROGRAM	0x02 /* Page Program */
#define SPIFLASH_FAST_READ		0x0B /* Fast Read */
#define SPIFLASH_READ			0x03 /* Normal Read */
#define SPIFLASH_MASS_ERASE		0xC7 /* Mass Erase */
#define SPIFLASH_READ_SFDP		0x5A /* Read Serial Flash Discoverable Parameters */

#define SPIFLASH_DEF_PAGESIZE	256  /* default for non-page-oriented devices (FRAMs) */

#endif /* OPENOCD_FLASH_NOR_SPI_H */
