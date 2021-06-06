/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#ifndef OPENOCD_TARGET_NDS32_INSN_H
#define OPENOCD_TARGET_NDS32_INSN_H

#define NOP						(0x40000009)
#define DSB						(0x64000008)
#define ISB						(0x64000009)
#define BEQ_MINUS_12			(0x4C000000 | 0x3FFA)
#define MTSR_DTR(a)				(0x64000003 | (((0x03 << 7) | (0x08 << 3) | (0x00 << 0)) << 10) | (((a) & 0x1F) << 20))
#define MFSR_DTR(a)				(0x64000002 | (((0x03 << 7) | (0x08 << 3) | (0x00 << 0)) << 10) | (((a) & 0x1F) << 20))
#define SETHI(a, b)				(0x46000000 | ((a) << 20) | (b))
#define ORI(a, b, c)			(0x58000000 | ((a) << 20) | ((b) << 15) | (c))
#define LWI_BI(a, b)			(0x0C000001 | (a << 20) | (b << 15))
#define LHI_BI(a, b)			(0x0A000001 | (a << 20) | (b << 15))
#define LBI_BI(a, b)			(0x08000001 | (a << 20) | (b << 15))
#define SWI_BI(a, b)			(0x1C000001 | (a << 20) | (b << 15))
#define SHI_BI(a, b)			(0x1A000001 | (a << 20) | (b << 15))
#define SBI_BI(a, b)			(0x18000001 | (a << 20) | (b << 15))
#define IRET					(0x64000004)
#define L1D_IX_WB(a)			(0x64000021 | ((a) << 15))
#define L1D_IX_INVAL(a)			(0x64000001 | ((a) << 15))
#define L1D_VA_INVAL(a)			(0x64000101 | ((a) << 15))
#define L1D_VA_WB(a)			(0x64000121 | ((a) << 15))
#define L1D_IX_RTAG(a)			(0x64000061 | ((a) << 15))
#define L1D_IX_RWD(a)			(0x64000081 | ((a) << 15))
#define L1I_IX_INVAL(a)			(0x64000201 | ((a) << 15))
#define L1I_VA_INVAL(a)			(0x64000301 | ((a) << 15))
#define L1I_IX_RTAG(a)			(0x64000261 | ((a) << 15))
#define L1I_IX_RWD(a)			(0x64000281 | ((a) << 15))
#define L1I_VA_FILLCK(a)		(0x64000361 | ((a) << 15))
#define ISYNC(a)				(0x6400000d | ((a) << 20))
#define MSYNC_STORE				(0x6400002c)
#define MSYNC_ALL				(0x6400000c)
#define TLBOP_TARGET_READ(a)	(0x6400000e | ((a) << 15))
#define TLBOP_TARGET_PROBE(a, b)	(0x640000AE | ((a) << 20) | ((b) << 15))
#define MFCPD(a, b, c)			(0x6A000041 | (a << 20) | (b << 8) | (c << 4))
#define MFCPW(a, b, c)			(0x6A000001 | (a << 20) | (b << 8) | (c << 4))
#define MTCPD(a, b, c)			(0x6A000049 | (a << 20) | (b << 8) | (c << 4))
#define MTCPW(a, b, c)			(0x6A000009 | (a << 20) | (b << 8) | (c << 4))
#define MOVI_(a, b)				(0x44000000 | (a << 20) | (b & 0xFFFFF))
#define MFUSR_G0(a, b)			(0x42000020 | (a << 20) | (b << 15))
#define MTUSR_G0(a, b)			(0x42000021 | (a << 20) | (b << 15))
#define MFSR(a, b)				(0x64000002 | (b << 10) | (a << 20))
#define MTSR(a, b)				(0x64000003 | (b << 10) | (a << 20))
#define AMFAR(a, b)				(0x60300060 | (a << 15) | b)
#define AMTAR(a, b)				(0x60300040 | (a << 15) | b)
#define AMFAR2(a, b)			(0x60300260 | (a << 15) | b)
#define AMTAR2(a, b)			(0x60300240 | (a << 15) | b)
#define FMFCSR					(0x6A000701)
#define FMTCSR					(0x6A000709)
#define FMFCFG					(0x6A000301)
#define FMFSR(a, b)				(0x6A000001 | ((a) << 20) | ((b) << 15))
#define FMTSR(a, b)				(0x6A000009 | ((a) << 20) | ((b) << 15))
#define FMFDR(a, b)				(0x6A000041 | ((a) << 20) | ((b) << 15))
#define FMTDR(a, b)				(0x6A000049 | ((a) << 20) | ((b) << 15))

/* break instructions */
#define NDS32_BREAK_16			(0x00EA)
#define NDS32_BREAK_32			(0x0A000064)

#endif /* OPENOCD_TARGET_NDS32_INSN_H */
