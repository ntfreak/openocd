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

#ifndef MPC56XX_JTAG
#define MPC56XX_JTAG

#include "mpc5xxx_jtag.h"

/* tap instructions */

#define MPC56XX_TAP_NEXUS	0x10
#define MPC56XX_TAP_ONCE	0x11
#define MPC56XX_TAP_ETPU	0x12
#define MPC56XX_TAP_EDMA	0x13
#define MPC56XX_TAP_FLEXRAY	0x14

/*
 * Registers
 */
/* Registers are lower 7 bits of 10 bit commands */
#define MPC56XX_ONCE_DID	0b0000010
#define MPC56XX_ONCE_CPUSCR	0b0010000
#define MPC56XX_ONCE_NOREG	0b0010001
/* See MPC5XXX_ONCE_OCR */
#define MPC56XX_ONCE_IAC1	0b0100000
#define MPC56XX_ONCE_IAC2	0b0100001
#define MPC56XX_ONCE_IAC3	0b0100010
#define MPC56XX_ONCE_IAC4	0b0100011
#define MPC56XX_ONCE_DAC1	0b0100100
#define MPC56XX_ONCE_DAC2	0b0100101
#define MPC56XX_ONCE_DBCNT	0b0101100
#define MPC56XX_ONCE_PCFIFO	0b0101101
#define MPC56XX_ONCE_DBSR	0b0110000
/* See #define MPC5XXX_ONCE_DBCR0 */
#define MPC56XX_ONCE_DBCR1	0b0110010
#define MPC56XX_ONCE_DBCR2	0b0110011
#define MPC56XX_ONCE_DBCR3	0b0110100
#define MPC56XX_ONCE_DBERC0	0b0111111
#define MPC56XX_ONCE_CDACTL 0b1111010
#define MPC56XX_ONCE_CDADATA	0b1111011
/*  MPC56XX_ONCE_NEXUS use MPC5XXX_ONCE_NEXUS */
#define MPC56XX_ONCE_ENABLE	0b1111110 /* Datasheet rather mysterious on this */

#define MPC56XX_NOP	0x60000000
#define MPC56XX_SWBP	0x00000000

#define MPC56XX_DB_IAC1	0x00800000 /* masks apply to DBSR and DBCR0 */
#define MPC56XX_DB_IAC2	0x00400000
#define MPC56XX_DB_IAC3	0x00200000
#define MPC56XX_DB_IAC4	0x00100000
#define MPC56XX_DB_DAC1R	0x00080000
#define MPC56XX_DB_DAC1W	0x00040000
#define MPC56XX_DB_DAC2R	0x00020000
#define MPC56XX_DB_DAC2W	0x00010000
#define MPC56XX_NUM_BPS	4
#define MPC56XX_NUM_WPS	2

#define MPC56XX_EI_VAL		0x01
#define MPC56XX_EI_INC		0x02
#define MPC56XX_EI_MASKINT	0x04

int mpc56xx_enter_debug(struct mpc5xxx_jtag *jtag_info, int async_flag);
int mpc56xx_leave_debug(struct mpc5xxx_jtag *jtag_info);
int mpc56xx_enable_once(struct mpc5xxx_jtag *jtag_info);
int mpc56xx_exec_inst(struct mpc5xxx_jtag *jtag_info, uint32_t inst, uint32_t in,
		uint32_t *out, uint32_t flag);
int mpc56xx_exit_debug(struct mpc5xxx_jtag *jtag_info, uint32_t addr, int sw_bp, uint32_t ctl_on_entry);
int mpc56xx_jtag_set_bp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);
int mpc56xx_jtag_clr_bps_wps(struct mpc5xxx_jtag *jtag_info);
int mpc56xx_jtag_clr_bp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);
int mpc56xx_jtag_set_wp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);
int mpc56xx_jtag_clr_wp(struct mpc5xxx_jtag *jtag_info, int bp_num, uint32_t addr);

#endif /* MPC56XX_JTAG */
