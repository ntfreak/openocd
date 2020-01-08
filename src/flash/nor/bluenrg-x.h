/***************************************************************************
 *   Copyright (C) 2019 by STMicroelectronics.                             *
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

#ifndef OPENOCD_FLASH_NOR_BLUENRGX_H
#define OPENOCD_FLASH_NOR_BLUENRGX_H

/* Flash Controller registers offsets */
#define FLASH_REG_COMMAND 0x00
#define FLASH_REG_CONFIG  0x04
#define FLASH_REG_IRQSTAT 0x08
#define FLASH_REG_IRQMASK 0x0C
#define FLASH_REG_IRQRAW  0x10
#define FLASH_REG_ADDRESS 0x18
#define FLASH_REG_UNLOCKM 0x1C
#define FLASH_REG_UNLOCKL 0x20
#define FLASH_REG_DATA0   0x40
#define FLASH_REG_DATA1   0x44
#define FLASH_REG_DATA2   0x48
#define FLASH_REG_DATA3   0x4C
#define FLASH_SIZE_REG    0x14

/* Flash Controller commands */
#define FLASH_CMD_ERASE_PAGE 0x11
#define FLASH_CMD_MASSERASE  0x22
#define FLASH_CMD_WRITE      0x33
#define FLASH_CMD_BURSTWRITE 0xCC
#define FLASH_INT_CMDDONE    0x01

#define FLASH_WORD_LEN       4

#endif /* OPENOCD_FLASH_NOR_BLUENRGX_H */
