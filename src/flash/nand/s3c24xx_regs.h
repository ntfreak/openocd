/***************************************************************************
 *   Copyright (C) 2004, 2005 by Simtec Electronics                        *
 *   linux@simtec.co.uk                                                    *
 *   http://www.simtec.co.uk/products/SWLINUX/                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License.               *
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
 * S3C2410 NAND register definitions
 */

#ifndef OPENOCD_FLASH_NAND_S3C24XX_REGS_H
#define OPENOCD_FLASH_NAND_S3C24XX_REGS_H

#define S3C2410_NFREG(x) (x)

#define S3C2410_NFCONF  S3C2410_NFREG(0x00)
#define S3C2410_NFCMD   S3C2410_NFREG(0x04)
#define S3C2410_NFADDR  S3C2410_NFREG(0x08)
#define S3C2410_NFDATA  S3C2410_NFREG(0x0C)
#define S3C2410_NFSTAT  S3C2410_NFREG(0x10)
#define S3C2410_NFECC   S3C2410_NFREG(0x14)

#define S3C2440_NFCONT   S3C2410_NFREG(0x04)
#define S3C2440_NFCMD    S3C2410_NFREG(0x08)
#define S3C2440_NFADDR   S3C2410_NFREG(0x0C)
#define S3C2440_NFDATA   S3C2410_NFREG(0x10)
#define S3C2440_NFECCD0  S3C2410_NFREG(0x14)
#define S3C2440_NFECCD1  S3C2410_NFREG(0x18)
#define S3C2440_NFECCD   S3C2410_NFREG(0x1C)
#define S3C2440_NFSTAT   S3C2410_NFREG(0x20)
#define S3C2440_NFESTAT0 S3C2410_NFREG(0x24)
#define S3C2440_NFESTAT1 S3C2410_NFREG(0x28)
#define S3C2440_NFMECC0  S3C2410_NFREG(0x2C)
#define S3C2440_NFMECC1  S3C2410_NFREG(0x30)
#define S3C2440_NFSECC   S3C2410_NFREG(0x34)
#define S3C2440_NFSBLK   S3C2410_NFREG(0x38)
#define S3C2440_NFEBLK   S3C2410_NFREG(0x3C)

#define S3C2412_NFSBLK		S3C2410_NFREG(0x20)
#define S3C2412_NFEBLK		S3C2410_NFREG(0x24)
#define S3C2412_NFSTAT		S3C2410_NFREG(0x28)
#define S3C2412_NFMECC_ERR0	S3C2410_NFREG(0x2C)
#define S3C2412_NFMECC_ERR1	S3C2410_NFREG(0x30)
#define S3C2412_NFMECC0		S3C2410_NFREG(0x34)
#define S3C2412_NFMECC1		S3C2410_NFREG(0x38)
#define S3C2412_NFSECC		S3C2410_NFREG(0x3C)

#define S3C2410_NFCONF_EN          (1 << 15)
#define S3C2410_NFCONF_512BYTE     (1 << 14)
#define S3C2410_NFCONF_4STEP       (1 << 13)
#define S3C2410_NFCONF_INITECC     (1 << 12)
#define S3C2410_NFCONF_NFCE        (1 << 11)
#define S3C2410_NFCONF_TACLS(x)    ((x) << 8)
#define S3C2410_NFCONF_TWRPH0(x)   ((x) << 4)
#define S3C2410_NFCONF_TWRPH1(x)   ((x) << 0)

#define S3C2410_NFSTAT_BUSY        (1 << 0)

#define S3C2440_NFCONF_BUSWIDTH_8	(0 << 0)
#define S3C2440_NFCONF_BUSWIDTH_16	(1 << 0)
#define S3C2440_NFCONF_ADVFLASH		(1 << 3)
#define S3C2440_NFCONF_TACLS(x)		((x) << 12)
#define S3C2440_NFCONF_TWRPH0(x)	((x) << 8)
#define S3C2440_NFCONF_TWRPH1(x)	((x) << 4)

#define S3C2440_NFCONT_LOCKTIGHT	(1 << 13)
#define S3C2440_NFCONT_SOFTLOCK		(1 << 12)
#define S3C2440_NFCONT_ILLEGALACC_EN	(1 << 10)
#define S3C2440_NFCONT_RNBINT_EN	(1 << 9)
#define S3C2440_NFCONT_RN_FALLING	(1 << 8)
#define S3C2440_NFCONT_SPARE_ECCLOCK	(1 << 6)
#define S3C2440_NFCONT_MAIN_ECCLOCK	(1 << 5)
#define S3C2440_NFCONT_INITECC		(1 << 4)
#define S3C2440_NFCONT_NFCE			(1 << 1)
#define S3C2440_NFCONT_ENABLE		(1 << 0)

#define S3C2440_NFSTAT_READY		(1 << 0)
#define S3C2440_NFSTAT_NCE			(1 << 1)
#define S3C2440_NFSTAT_RNB_CHANGE	(1 << 2)
#define S3C2440_NFSTAT_ILLEGAL_ACCESS	(1 << 3)

#define S3C2412_NFCONF_NANDBOOT		(1 << 31)
#define S3C2412_NFCONF_ECCCLKCON	(1 << 30)
#define S3C2412_NFCONF_ECC_MLC		(1 << 24)
#define S3C2412_NFCONF_TACLS_MASK	(7 << 12)	/* 1 extra bit of Tacls */

#define S3C2412_NFCONT_ECC4_DIRWR	(1 << 18)
#define S3C2412_NFCONT_LOCKTIGHT	(1 << 17)
#define S3C2412_NFCONT_SOFTLOCK		(1 << 16)
#define S3C2412_NFCONT_ECC4_ENCINT	(1 << 13)
#define S3C2412_NFCONT_ECC4_DECINT	(1 << 12)
#define S3C2412_NFCONT_MAIN_ECC_LOCK	(1 << 7)
#define S3C2412_NFCONT_INIT_MAIN_ECC	(1 << 5)
#define S3C2412_NFCONT_NFCE1		(1 << 2)
#define S3C2412_NFCONT_NFCE0		(1 << 1)

#define S3C2412_NFSTAT_ECC_ENCDONE	(1 << 7)
#define S3C2412_NFSTAT_ECC_DECDONE	(1 << 6)
#define S3C2412_NFSTAT_ILLEGAL_ACCESS	(1 << 5)
#define S3C2412_NFSTAT_RNB_CHANGE	(1 << 4)
#define S3C2412_NFSTAT_NFCE1		(1 << 3)
#define S3C2412_NFSTAT_NFCE0		(1 << 2)
#define S3C2412_NFSTAT_RES1			(1 << 1)
#define S3C2412_NFSTAT_READY		(1 << 0)

#define S3C2412_NFECCERR_SERRDATA(x)	(((x) >> 21) & 0xf)
#define S3C2412_NFECCERR_SERRBIT(x)		(((x) >> 18) & 0x7)
#define S3C2412_NFECCERR_MERRDATA(x)	(((x) >> 7) & 0x3ff)
#define S3C2412_NFECCERR_MERRBIT(x)		(((x) >> 4) & 0x7)
#define S3C2412_NFECCERR_SPARE_ERR(x)	(((x) >> 2) & 0x3)
#define S3C2412_NFECCERR_MAIN_ERR(x)	(((x) >> 2) & 0x3)
#define S3C2412_NFECCERR_NONE		(0)
#define S3C2412_NFECCERR_1BIT		(1)
#define S3C2412_NFECCERR_MULTIBIT	(2)
#define S3C2412_NFECCERR_ECCAREA	(3)

#endif /* OPENOCD_FLASH_NAND_S3C24XX_REGS_H */
