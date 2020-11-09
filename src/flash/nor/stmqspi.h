/***************************************************************************
 *   Copyright (C) 2016 - 2018 by Andreas Bolsch                           *
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

#ifndef OPENOCD_FLASH_NOR_STMQSPI_H
#define OPENOCD_FLASH_NOR_STMQSPI_H

#include "spi.h"

/* QSPI register offsets */
#define QSPI_CR			(0x00)	/* Control register */
#define QSPI_DCR		(0x04)	/* Device configuration register */
#define QSPI_SR			(0x08)	/* Status register */
#define QSPI_FCR		(0x0C)	/* Flag clear register */
#define QSPI_DLR		(0x10)	/* Data length register */
#define QSPI_CCR		(0x14)	/* Communication configuration register */
#define QSPI_AR			(0x18)	/* Address register */
#define QSPI_ABR		(0x1C)	/* Alternate bytes register */
#define QSPI_DR			(0x20)	/* Data register */

/* common bits in QSPI_CR and OCTOSPI_CR */
#define SPI_FSEL_FLASH	7		/* Select flash 2 */
#define SPI_DUAL_FLASH	6		/* Dual flash mode */
#define SPI_ABORT		1		/* Abort bit */

/* common bits in QSPI_DCR and OCTOSPI_DCR1 */
#define SPI_FSIZE_POS	16		/* bit position of FSIZE */
#define SPI_FSIZE_LEN	5		/* width of FSIZE field */

/* common bits in QSPI_SR/FCR and OCTOSPI_SR/FCR */
#define SPI_BUSY		5		/* Busy flag */
#define SPI_FTF			2		/* FIFO threshold flag */
#define SPI_TCF			1		/* Transfer complete flag */

/* fields in QSPI_CCR */
#define QSPI_DDRM			31					/* position of DDRM bit */
#define SPI_DMODE_POS		24					/* bit position of DMODE */
#define QSPI_DCYC_POS		18					/* bit position of DCYC */
#define QSPI_DCYC_LEN		5					/* width of DCYC field */
#define QSPI_DCYC_MASK		((BIT(QSPI_DCYC_LEN) - 1) << QSPI_DCYC_POS)
#define SPI_ADSIZE_POS		12					/* bit position of ADSIZE */

#define QSPI_WRITE_MODE		0x00000000U			/* indirect write mode */
#define QSPI_READ_MODE		0x04000000U			/* indirect read mode */
#define QSPI_MM_MODE		0x0C000000U			/* memory mapped mode */
#define QSPI_ALTB_MODE		0x0003C000U			/* alternate byte mode */
#define QSPI_4LINE_MODE		0x03000F00U			/* 4 lines for data, addr, instr */
#define QSPI_NO_DATA		(~0x03000000U)		/* no data */
#define QSPI_NO_ALTB		(~QSPI_ALTB_MODE)	/* no alternate */
#define QSPI_NO_ADDR		(~0x00000C00U)		/* no address */
#define QSPI_ADDR3			(0x2U << SPI_ADSIZE_POS)	/* 3 byte address */
#define QSPI_ADDR4			(0x3U << SPI_ADSIZE_POS)	/* 4 byte address */

/* OCTOSPI register offsets */
#define OCTOSPI_CR		(0x000)	/* Control register */
#define OCTOSPI_DCR1	(0x008)	/* Device configuration register 1 */
#define OCTOSPI_DCR2	(0x00C)	/* Device configuration register 2 */
#define OCTOSPI_DCR3	(0x010)	/* Device configuration register 3 */
#define	OCTOSPI_SR		(0x020)	/* Status register */
#define OCTOSPI_FCR		(0x024)	/* Flag clear register */
#define OCTOSPI_DLR		(0x040)	/* Data length register */
#define OCTOSPI_AR		(0x048)	/* Address register */
#define OCTOSPI_DR		(0x050)	/* Data register */
#define OCTOSPI_CCR		(0x100)	/* Communication configuration register */
#define OCTOSPI_TCR		(0x108)	/* Timing configuration register */
#define OCTOSPI_IR		(0x110)	/* Instruction register */
#define OCTOSPI_WCCR	(0x180)	/* Write communication configuration register */
#define OCTOSPI_WIR		(0x190)	/* Write instruction register */
#define OCTOSPI_MAGIC	(0x3FC)	/* Magic ID register, deleted from RM, why? */

#define OCTO_MAGIC_ID	0xA3C5DD01	/* Magic ID, deleted from RM, why? */

/* additional bits in OCTOSPI_CR */
#define OCTOSPI_WRITE_MODE	0x00000000U			/* indirect write mode */
#define OCTOSPI_READ_MODE	0x10000000U			/* indirect read mode */
#define OCTOSPI_MM_MODE		0x30000000U			/* memory mapped mode */

/* additional fields in OCTOSPI_DCR1 */
#define OCTOSPI_MTYP_POS	(24)				/* bit position of MTYP */
#define OCTOSPI_MTYP_LEN	(3)					/* width of MTYP field */
#define OCTOSPI_MTYP_MASK	((BIT(OCTOSPI_MTYP_LEN) - 1) << OCTOSPI_MTYP_POS)

/* fields in OCTOSPI_CCR */
#define OCTOSPI_ALTB_MODE	0x001F0000U				/* alternate byte mode */
#define OCTOSPI_8LINE_MODE	0x0F003F3FU				/* 8 lines DTR for data, addr, instr */
#define OCTOSPI_NO_DATA		(~0x0F000000U)			/* no data */
#define OCTOSPI_NO_ALTB		(~OCTOSPI_ALTB_MODE)	/* no alternate */
#define OCTOSPI_NO_ADDR		(~0x00000F00U)			/* no address */
#define OCTOSPI_ADDR3		(0x2U << SPI_ADSIZE_POS)	/* 3 byte address */
#define OCTOSPI_ADDR4		(0x3U << SPI_ADSIZE_POS)	/* 4 byte address */
#define OCTOSPI_DQSEN		29						/* DQS enable */
#define OCTOSPI_DDTR		27						/* DTR for data */
#define OCTOSPI_NO_DDTR		(~BIT(OCTOSPI_DDTR))	/* no DTR for data, but maybe still DQS */
#define OCTOSPI_ISIZE_MASK	(0x30)					/* ISIZE field */

/* fields in OCTOSPI_TCR */
#define OCTOSPI_DCYC_POS	0					/* bit position of DCYC */
#define OCTOSPI_DCYC_LEN	5					/* width of DCYC field */
#define OCTOSPI_DCYC_MASK	((BIT(OCTOSPI_DCYC_LEN) - 1) << OCTOSPI_DCYC_POS)

#define IS_OCTOSPI			(stmqspi_info->octo)
#define SPI_CR				(IS_OCTOSPI ? OCTOSPI_CR : QSPI_CR)
#define SPI_DCR				(IS_OCTOSPI ? OCTOSPI_DCR1 : QSPI_DCR)
#define	SPI_SR				(IS_OCTOSPI ? OCTOSPI_SR : QSPI_SR)
#define SPI_FCR				(IS_OCTOSPI ? OCTOSPI_FCR : QSPI_FCR)
#define SPI_DLR				(IS_OCTOSPI ? OCTOSPI_DLR : QSPI_DLR)
#define SPI_AR				(IS_OCTOSPI ? OCTOSPI_AR : QSPI_AR)
#define SPI_DR				(IS_OCTOSPI ? OCTOSPI_DR : QSPI_DR)
#define SPI_CCR				(IS_OCTOSPI ? OCTOSPI_CCR : QSPI_CCR)

#endif /* OPENOCD_FLASH_NOR_STMQSPI_H */
