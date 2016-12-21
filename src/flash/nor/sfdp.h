/***************************************************************************
 *   Copyright (C) 2019 by Andreas Bolsch <andreas.bolsch@mni.thm.de	   *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
 *   (at your option) any later version.								   *
 *																		   *
 *   This program is distributed in the hope that it will be useful,	   *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *   GNU General Public License for more details.						   *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License	   *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_SFDP_H
#define OPENOCD_FLASH_NOR_SFDP_H

/* per JESD216D 'addr' is *byte* based but must be word aligned,
 * 'buffer' is word based, word aligned and always little-endian encoded,
 * in the flash, 'addr_len' is 3 or 4, 'dummy' ***usually*** 8
 *
 * the actual number of dummy clocks should be worked out by this function
 * dynamically, i.e. by scanning the first few bytes for the SFDP signature
 *
 * buffer contents is supposed to be returned in ***host*** endianness */
typedef int (*read_sfdp_block_t)(struct flash_bank *bank, uint32_t addr,
	uint32_t words, uint32_t *buffer);

extern int spi_sfdp(struct flash_bank *bank, struct flash_device *dev,
	read_sfdp_block_t read_sfdp_block);

#endif /* OPENOCD_FLASH_NOR_SFDP_H */
