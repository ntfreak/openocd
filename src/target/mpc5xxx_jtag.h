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

#ifndef MPC5XXX_JTAG
#define MPC5XXX_JTAG

#define MPC5XXX_INST_IDCODE	0x01
#define MPC5XXX_TAP_JTAG    0x01 /* Not TAP number, but used to identify JTAG */
#define MPC5XXX_TAP_INVALID	-1

#define MPC5XXX_ONCE_NEXUS_RWCS	(0x07 << 1)	/* 32bit control reg */
#define MPC5XXX_ONCE_NEXUS_RWA	(0x09 << 1)	/* 32bit address reg */
#define MPC5XXX_ONCE_NEXUS_RWD	(0x0a << 1)	/* 32bit data reg */
#define MPC5XXX_ONCE_NEXUS_WRITE	1			/* lsb determines read=0 or write=1 */
#define MPC5XXX_ONCE_NEXUS_RWCS_READ1_32	0x90000000	/* single 32bit read */
#define MPC5XXX_ONCE_NEXUS_RWCS_READ1_16	0x88000000	/* single 16bit read */
#define MPC5XXX_ONCE_NEXUS_RWCS_READ1_8	0x80000000	/* single 8bit read */
#define MPC5XXX_ONCE_NEXUS_RWCS_WRITE1_32	0xd0000000	/* single 32bit write */
#define MPC5XXX_ONCE_NEXUS_RWCS_WRITE1_16	0xc8000000	/* single 16bit write */
#define MPC5XXX_ONCE_NEXUS_RWCS_WRITE1_8	0xc0000000	/* single 8bit write */
#define MPC5XXX_ONCE_NEXUS_RWCS_MASK	0x00000003	/* ERR, DV mask */
#define MPC5XXX_ONCE_NEXUS_RWCS_DVMASK	0x00000001	/* DV mask */
#define MPC5XXX_ONCE_NEXUS_RWCS_READOK 0x00000001
#define MPC5XXX_ONCE_NEXUS_RWCS_WRITEOK 0x00000000
#define MPC5XXX_OCR_DEBUG1	0x05
#define MPC5XXX_OCR_DEBUG2	0x06
#define MPC5XXX_OCR_DEBUG_OFF	0x04 /* Still leave WKUP bit set */
#define MPC5XXX_OCR_FDB	0x02 /* Allow SW BPs */

/* Some ONCE registers are the same */
#define MPC5XXX_ONCE_DID	0b0000010
#define MPC5XXX_ONCE_CPUSCR	0b0010000
#define MPC5XXX_ONCE_NOREG	0b0010001
#define MPC5XXX_ONCE_OCR	0b0010010
#define MPC5XXX_ONCE_DBCR0	0b0110001
#define MPC5XXX_ONCE_NEXUS	0b1111100 /* access Nexus registers via OnCE, 56xx and 57xx */

#define MPC5XXX_ONCE_DBCR0_EDM	0x80000000	/* EDM bit */

#define MPC5XXX_ONCE_EX		0b0010000000 /* EX mask. EXit debug */
#define MPC5XXX_ONCE_GO		0b0100000000 /* GO mask. Execute command */
#define MPC5XXX_ONCE_READ	0b1000000000 /* READ mask. */

#define MPC5XXX_OSR_HALT	0x20
#define MPC5XXX_OSR_DEBUG	0x08
#define MPC5XXX_OSR_HALT_DEBUG	0x28
#define MPC5XXX_OSR_ERR	0x100

#define MPC5XXX_MSR_EE	0x8000
#define MPC5XXX_MSR_CE	0x20000

#define MPC5XXX_CPUSCR_CTL_FFRA 0x00000400
#define MPC5XXX_CPUSCR_CTL_PCOFST4 0x00001000

struct mpc5xxx_jtag {
	struct jtag_tap *tap;
	uint32_t dpc; /* Debug PC value */
	int once; /* Which Once we are talking to */
	int current_tap; /* Which Jtag tap we are talking to */
	int jtag_irlen; /* irlen used for Jtag TAP. Ignores command line option. */
};

struct mpc5xxx_cpuscr {
	uint32_t wbbrl, wbbrh,  msr, pc, ir, ctl;
};
int mpc5xxx_jtag_set_instr(struct mpc5xxx_jtag *jtag_info, int new_instr);
int mpc5xxx_jtag_read_data(struct mpc5xxx_jtag *jtag_info,
	uint32_t *pdata, uint32_t size);
int mpc5xxx_jtag_write_data(struct mpc5xxx_jtag *jtag_info,
		uint32_t data, uint32_t size);
int mpc5xxx_jtag_access_jtagc(struct mpc5xxx_jtag *jtag_info);
int mpc5xxx_jtag_access_once(struct mpc5xxx_jtag *jtag_info);
int mpc5xxx_jtagc_read(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t *value, uint32_t size);
int mpc5xxx_jtagc_write(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t value, uint32_t size);
int mpc5xxx_once_read(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t *value, uint32_t size);
int mpc5xxx_once_write(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t value, uint32_t size);
int mpc5xxx_once_nexus_read(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t *value, uint32_t size);
int mpc5xxx_once_nexus_write(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, uint32_t value, uint32_t size);
int mpc5xxx_once_osr_read(struct mpc5xxx_jtag *jtag_info, uint32_t *in);
int mpc5xxx_once_cpuscr_read(struct mpc5xxx_jtag *jtag_info, struct mpc5xxx_cpuscr *in);
int mpc5xxx_once_cpuscr_write(struct mpc5xxx_jtag *jtag_info, struct mpc5xxx_cpuscr *in);


#endif /* MPC56XX_JTAG */
