/***************************************************************************
 *   Copyright (C) 2017 by James Murray <james@nscc.info                   *
 *   Based on code:                                                        *
 *       Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>   *
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

#ifndef MPC57XX_JTAG
#define MPC57XX_JTAG

#include "mpc5xxx_jtag.h"

/* TAP instructions from MPC5746RRM */

#define MPC57XX_TAP_NAR		0x21
#define MPC57XX_TAP_JDC		0x26
#define MPC57XX_TAP_ONCE0	0x28
#define MPC57XX_TAP_ONCE1	0x29
#define MPC57XX_TAP_ETPU	0x30
#define MPC57XX_TAP_NXMC_0	0x34
#define MPC57XX_TAP_NXMC_1	0x35
#define MPC57XX_TAP_SPU		0x3a
#define MPC57XX_TAP_JTAGC_PD	0x3e /* only applies when Buddy Die in use */

/*
 * Registers
 */
/* Registers are lower 7 bits of 10 bit commands */
#define MPC57XX_ONCE_DID		0b0000010
#define MPC57XX_ONCE_CPUSCR		0b0010000
#define MPC57XX_ONCE_NOREG		0b0010001
/* See MPC5XXX_ONCE_OCR */
#define MPC57XX_ONCE_DAC3		0b0011000
#define MPC57XX_ONCE_DAC4		0b0011001
#define MPC57XX_ONCE_IAC1		0b0100000
#define MPC57XX_ONCE_IAC2		0b0100001
#define MPC57XX_ONCE_IAC3		0b0100010
#define MPC57XX_ONCE_IAC4		0b0100011
#define MPC57XX_ONCE_DAC1		0b0100100
#define MPC57XX_ONCE_DAC2		0b0100101
#define MPC57XX_ONCE_DVC1		0b0100110
#define MPC57XX_ONCE_DVC2		0b0100111
#define MPC57XX_ONCE_IAC5		0b0101000
#define MPC57XX_ONCE_IAC6		0b0101001
#define MPC57XX_ONCE_IAC7		0b0101010
#define MPC57XX_ONCE_IAC8		0b0101011
#define MPC57XX_ONCE_DDEAR		0b0101100
#define MPC57XX_ONCE_EDDEAR		0b0101101
#define MPC57XX_ONCE_EDBCR0		0b0101110
#define MPC57XX_ONCE_EDBSR0		0b0101111
#define MPC57XX_ONCE_DBSR		0b0110000
#define MPC57XX_ONCE_DBCR0		0b0110001
#define MPC57XX_ONCE_DBCR1		0b0110010
#define MPC57XX_ONCE_DBCR2		0b0110011
/* There is no DBCR3 */
#define MPC57XX_ONCE_DBCR4		0b0110101
#define MPC57XX_ONCE_DBCR5		0b0110110
#define MPC57XX_ONCE_DBCR6		0b0110111
#define MPC57XX_ONCE_DBCR7		0b0111000
#define MPC57XX_ONCE_DBCR8		0b0111001
#define MPC57XX_ONCE_EDBSRMSK0	0b0111100
#define MPC57XX_ONCE_DDAM		0b0111101
#define MPC57XX_ONCE_DEVENT		0b0111110
#define MPC57XX_ONCE_EDBRAC0	0b0111111

#define MPC57XX_ONCE_MPU0CSR0	0b1101101
#define MPC57XX_ONCE_PERFMON	0b1101110
/* GPR selects [0:9] */
#define MPC57XX_ONCE_CDACTL		0b1111010
#define MPC57XX_ONCE_CDADATA	0b1111011
/*  MPC56XX_ONCE_NEXUS use MPC5XXX_ONCE_NEXUS */
#define MPC57XX_ONCE_LSRL		0b1111101
#define MPC57XX_ONCE_ENABLE		0b1111110 /* Datasheet rather mysterious on this */
#define MPC57XX_ONCE_BYPASS		0b1111111

#define MPC57XX_ONCE_EDBRAC0_DEFAULT 0x00000180

/* Many registers the same as 56xx.
 * Nexus via OnCE the same.
 */

#define MPC57XX_NOP	0x60000000 /* need VLE equivalent */
#define MPC57XX_SWBP	0x00000000 /* need VLE equivalent */
#define MPC57XX_CPUSCR_CTL_FFRA	0x00000400
#define MPC57XX_CPUSCR_CTL_PCOFST4	0x00001000

#define MPC57XX_DBCR0_IAC1	0x00800000 /* masks apply to DBCR0 */
#define MPC57XX_DBCR0_IAC2	0x00400000
#define MPC57XX_DBCR0_IAC3	0x00200000
#define MPC57XX_DBCR0_IAC4	0x00100000
#define MPC57XX_DBCR0_IAC5	0x00004000
#define MPC57XX_DBCR0_IAC6	0x00002000
#define MPC57XX_DBCR0_IAC7	0x00001000
#define MPC57XX_DBCR0_IAC8	0x00000800
#define MPC57XX_DBCR0_DAC1R	0x00080000
#define MPC57XX_DBCR0_DAC1W	0x00040000
#define MPC57XX_DBCR0_DAC2R	0x00020000
#define MPC57XX_DBCR0_DAC2W	0x00010000
#define MPC57XX_DBSR_DACW	0x00040000
#define MPC57XX_DBSR_DACR	0x00080000
#define MPC57XX_DBSR_IAC	0x00800000
#define MPC57XX_DBSR_VLES	0x00000010

#define MPC57XX_NUM_BPS	8
#define MPC57XX_NUM_WPS	2 /* actually 4, but 3,4 not yet implemented */

#define MPC57XX_EI_VAL		0x01 /* ? */
#define MPC57XX_EI_INC		0x02 /* ? */
#define MPC57XX_EI_MASKINT	0x04 /* ? */

int mpc57xx_enter_debug(struct mpc5xxx_jtag *jtag_info, int async_flag);
int mpc57xx_leave_debug(struct mpc5xxx_jtag *jtag_info);
int mpc57xx_enable_once(struct mpc5xxx_jtag *jtag_info);
int mpc57xx_exec_inst(struct mpc5xxx_jtag *jtag_info, uint32_t inst, uint32_t in,
		uint32_t *out, uint32_t flag);
int mpc57xx_exit_debug(struct mpc5xxx_jtag *jtag_info, uint32_t addr, int sw_bp, uint32_t ctl_on_entry);
int mpc57xx_jtag_set_bp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);
int mpc57xx_jtag_clr_bps_wps(struct mpc5xxx_jtag *jtag_info);
int mpc57xx_jtag_clr_bp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);
int mpc57xx_jtag_set_wp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);
int mpc57xx_jtag_clr_wp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);

#endif /* MPC57XX_JTAG */
