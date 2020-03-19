/***************************************************************************
 *   Copyright (C) 2015 by Uwe Bonnes                                      *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
 *
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

#ifndef OPENOCD_FLASH_NOR_STM32L4X
#define OPENOCD_FLASH_NOR_STM32L4X

/* FLASH_CR register bits */
#define FLASH_PG				(1 << 0)
#define FLASH_PER				(1 << 1)
#define FLASH_MER1				(1 << 2)
#define FLASH_PAGE_SHIFT		3
#define FLASH_CR_BKER			(1 << 11)
#define FLASH_MER2				(1 << 15)
#define FLASH_STRT				(1 << 16)
#define FLASH_OPTSTRT			(1 << 17)
#define FLASH_EOPIE				(1 << 24)
#define FLASH_ERRIE				(1 << 25)
#define FLASH_OBL_LAUNCH		(1 << 27)
#define FLASH_OPTLOCK			(1 << 30)
#define FLASH_LOCK				(1 << 31)

/* FLASH_SR register bits */
#define FLASH_BSY				(1 << 16)

/* Fast programming not used => related errors not used*/
#define FLASH_PGSERR			(1 << 7) /* Programming sequence error */
#define FLASH_SIZERR			(1 << 6) /* Size error */
#define FLASH_PGAERR			(1 << 5) /* Programming alignment error */
#define FLASH_WRPERR			(1 << 4) /* Write protection error */
#define FLASH_PROGERR			(1 << 3) /* Programming error */
#define FLASH_OPERR				(1 << 1) /* Operation error */
#define FLASH_EOP				(1 << 0) /* End of operation */
#define FLASH_ERROR				(FLASH_PGSERR | FLASH_SIZERR | FLASH_PGAERR | \
								FLASH_WRPERR | FLASH_PROGERR | FLASH_OPERR)

/* register unlock keys */
#define KEY1					0x45670123
#define KEY2					0xCDEF89AB

/* option register unlock key */
#define OPTKEY1					0x08192A3B
#define OPTKEY2					0x4C5D6E7F

#define RDP_LEVEL_0				0xAA
#define RDP_LEVEL_1				0xBB
#define RDP_LEVEL_2				0xCC

/* other registers */
#define DBGMCU_IDCODE_G0		0x40015800
#define DBGMCU_IDCODE_L4_G4		0xE0042000
#define DBGMCU_IDCODE_L5		0xE0044000

#define STM32_FLASH_BANK_BASE	0x08000000

#endif
