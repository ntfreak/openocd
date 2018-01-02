/***************************************************************************
 *   Copyright (C) 2018 by Andreas Bolsch                                  *
 *   andreas.bolsch@mni.thm.de                                             *
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

#ifndef OPENOCD_FLASH_NOR_CMSPI_H
#define OPENOCD_FLASH_NOR_CMSPI_H

/* mapping of SPI <-> I2C pins */
#define SPI_SDA					ncs
#define SPI_SCL					clk
#define I2C_SDA					1
#define I2C_SCL					0

#define	I2C_FIRST	(0x1 << 4)							/* first 'legal' I2C 7-bit address, R/W = 0 */
#define	I2C_LAST	((0xF << 4) - 1)					/* last 'legal' I2C 7-bit address, R/W = 1 */

#define SCAN_SIZE	(((I2C_LAST - I2C_FIRST + 1) + 31) >> 5)	/* two bits per 7-bit address into words */

#endif
