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

#ifndef _STM32L4X_H_
#define _STM32L4X_H_

#define STM32_FLASH_BASE    0x40022000

/* register offsets */
#define FLASH_ACR_OFFS		0x00
#define FLASH_KEYR_OFFS		0x08
#define FLASH_OPTKEYR_OFFS	0x0c
#define FLASH_SR_OFFS		0x10
#define FLASH_CR_OFFS		0x14
#define FLASH_OPTR_OFFS		0x20
#define FLASH_WRP1AR_OFFS	0x2c
#define FLASH_PCROP1SR_OFFS	0x24
#define FLASH_PCROP1ER_OFFS	0x28
#define FLASH_WRP1BR_OFFS	0x30
#define FLASH_PCROP2SR_OFFS	0x44
#define FLASH_PCROP2ER_OFFS	0x48
#define FLASH_WRP2AR_OFFS	0x4c
#define FLASH_WRP2BR_OFFS	0x50
#define FLASH_SEC1R_OFFS	0x70
#define FLASH_SEC2R_OFFS	0x74
#define	FLASH_SECR_OFFS		0x80

/* FLASH_CR register bits */
#define FLASH_PG			0
#define FLASH_PER			1
#define FLASH_MER1			2
#define FLASH_PAGE_SHIFT	3
#define FLASH_CR_BKER		11
#define FLASH_MER2			15
#define FLASH_STRT			16
#define FLASH_OPTSTRT		17
#define FLASH_EOPIE			24
#define FLASH_ERRIE			25
#define FLASH_OBLLAUNCH		27
#define FLASH_OPTLOCK		30
#define FLASH_LOCK			31

/* FLASH_SR register bits */
#define FLASH_BSY			16
/* Fast programming not used => related errors not used */
#define FLASH_PGSERR		7	/* Programming sequence error */
#define FLASH_SIZERR		6	/* Size error */
#define FLASH_PGAERR		5	/* Programming alignment error */
#define FLASH_WRPERR		4	/* Write protection error */
#define FLASH_PROGERR		3	/* Programming error */
#define FLASH_OPERR			1	/* Operation error */
#define FLASH_EOP			0	/* End of operation */

#define FLASH_ERROR_MASK ((1UL << FLASH_PGSERR) | (1UL << FLASH_SIZERR) | \
	(1UL << FLASH_PGSERR) | (1UL << FLASH_PGAERR) | \
	(1UL << FLASH_WRPERR) | (1UL << FLASH_OPERR))

/* STM32_FLASH_OBR bit definitions (reading) */
#define OPT_DBANK_LE_1M		21	/* dual bank for devices up to 1M flash */
#define OPT_DBANK_GE_2M		22	/* dual bank for devices with 2M flash */

/* register unlock keys */
#define KEY1				0x45670123
#define KEY2				0xCDEF89AB

/* option register unlock key */
#define OPTKEY1				0x08192A3B
#define OPTKEY2				0x4C5D6E7F

#define RDP_LEVEL_0			0xAA
#define RDP_LEVEL_1			0xBB
#define RDP_LEVEL_2			0xCC

/* other registers */
#define DBGMCU_IDCODE_L4_G4	0xE0042000
#define DBGMCU_IDCODE_G0	0x40015800
#define FLASH_SIZE_REG		0x1FFF75E0

#endif
