/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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

#ifndef OPENOCD_TARGET_MIPS_EJTAG_H
#define OPENOCD_TARGET_MIPS_EJTAG_H

#include <jtag/jtag.h>

/* tap instructions */
#define EJTAG_INST_IDCODE		0x01
#define EJTAG_INST_IMPCODE		0x03
#define EJTAG_INST_ADDRESS		0x08
#define EJTAG_INST_DATA			0x09
#define EJTAG_INST_CONTROL		0x0A
#define EJTAG_INST_ALL			0x0B
#define EJTAG_INST_EJTAGBOOT	0x0C
#define EJTAG_INST_NORMALBOOT	0x0D
#define EJTAG_INST_FASTDATA		0x0E
#define EJTAG_INST_TCBCONTROLA	0x10
#define EJTAG_INST_TCBCONTROLB	0x11
#define EJTAG_INST_TCBDATA		0x12
#define EJTAG_INST_TCBCONTROLC	0x13
#define EJTAG_INST_PCSAMPLE		0x14
#define EJTAG_INST_TCBCONTROLD	0x15
#define EJTAG_INST_TCBCONTROLE	0x16
#define EJTAG_INST_FDC			0x17
#define EJTAG_INST_BYPASS		0xFF

/* microchip PIC32MX specific instructions */
#define MTAP_SW_MTAP			0x04
#define MTAP_SW_ETAP			0x05
#define MTAP_COMMAND			0x07

/* microchip specific cmds */
#define MCHP_ASERT_RST			0xd1
#define MCHP_DE_ASSERT_RST		0xd0
#define MCHP_ERASE				0xfc
#define MCHP_STATUS				0x00

/* ejtag control register bits ECR */
#define EJTAG_CTRL_TOF			(1 << 1)
#define EJTAG_CTRL_TIF			(1 << 2)
#define EJTAG_CTRL_BRKST		(1 << 3)
#define EJTAG_CTRL_DLOCK		(1 << 5)
#define EJTAG_CTRL_DRWN			(1 << 9)
#define EJTAG_CTRL_DERR			(1 << 10)
#define EJTAG_CTRL_DSTRT		(1 << 11)
#define EJTAG_CTRL_JTAGBRK		(1 << 12)
#define EJTAG_CTRL_DBGISA		(1 << 13)
#define EJTAG_CTRL_SETDEV		(1 << 14)
#define EJTAG_CTRL_PROBEN		(1 << 15)
#define EJTAG_CTRL_PRRST		(1 << 16)
#define EJTAG_CTRL_DMAACC		(1 << 17)
#define EJTAG_CTRL_PRACC		(1 << 18)
#define EJTAG_CTRL_PRNW			(1 << 19)
#define EJTAG_CTRL_PERRST		(1 << 20)
#define EJTAG_CTRL_SYNC			(1 << 23)
#define EJTAG_CTRL_DNM			(1 << 28)
#define EJTAG_CTRL_ROCC			(1 << 31)

/* Debug Register (CP0 Register 23, Select 0) */

#define EJTAG_DEBUG_DSS			(1 << 0)
#define EJTAG_DEBUG_DBP			(1 << 1)
#define EJTAG_DEBUG_DDBL		(1 << 2)
#define EJTAG_DEBUG_DDBS		(1 << 3)
#define EJTAG_DEBUG_DIB			(1 << 4)
#define EJTAG_DEBUG_DINT		(1 << 5)
#define EJTAG_DEBUG_OFFLINE		(1 << 7)
#define EJTAG_DEBUG_SST			(1 << 8)
#define EJTAG_DEBUG_NOSST		(1 << 9)
#define EJTAG_DEBUG_DDBLIMPR	(1 << 18)
#define EJTAG_DEBUG_DDBSIMPR	(1 << 19)
#define EJTAG_DEBUG_IEXI		(1 << 20)
#define EJTAG_DEBUG_DBUSEP		(1 << 21)
#define EJTAG_DEBUG_CACHEEP		(1 << 22)
#define EJTAG_DEBUG_MCHECKP		(1 << 23)
#define EJTAG_DEBUG_IBUSEP		(1 << 24)
#define EJTAG_DEBUG_COUNTDM		(1 << 25)
#define EJTAG_DEBUG_HALT		(1 << 26)
#define EJTAG_DEBUG_DOZE		(1 << 27)
#define EJTAG_DEBUG_LSNM		(1 << 28)
#define EJTAG_DEBUG_NODCR		(1 << 29)
#define EJTAG_DEBUG_DM			(1 << 30)
#define EJTAG_DEBUG_DBD			(1 << 31)

/* implementation MIPS register bits.
 * Bits marked with V20 or v2.0 mean that, this registers supported only
 * by EJTAG v2.0. Bits marked with Lexra or BMIPS are different from the
 * official EJATG.
 * NOTE: Lexra or BMIPS use EJTAG v2.0 */

#define EJTAG_IMP_HAS(x)			(ejtag_info->impcode & (x))
/* v2.0(Lexra) 29 - 1’b1 - Lexra Internal Trace Buffer implemented. This bit
 * overlaps with version bit of MIPS EJTAG specification. */
#define EJTAG_V26_IMP_R3K		(1 << 28)
/* v2.0 - 24:25 - 2’b00- No profiling support */
#define EJTAG_V26_IMP_DINT		(1 << 24)
#define EJTAG_V20_IMP_SDBBP		(1 << 23) /* 1’b1 - sdbbp is Special2 Opcode */
#define EJTAG_IMP_ASID8			(1 << 22)
#define EJTAG_IMP_ASID6			(1 << 21)
#define EJTAG_V20_IMP_COMPLEX_BREAK	(1 << 20) /* Complex Breaks supported*/
#define EJTAG_V20_IMP_EADDR_NO32BIT	(1 << 19) /* EJTAG_ADDR > 32 bits wide */
#define EJTAG_V20_IMP_DCACHE_COH	(1 << 18) /* DCache does keep DMA coherent */
#define EJTAG_V20_IMP_ICACHE_COH	(1 << 17) /* DCache does keep DMA coherent */
#define EJTAG_IMP_MIPS16		(1 << 16)
#define EJTAG_IMP_NODMA			(1 << 14)
/* v2.0 - 11:13 external PC trace. Trace PC Width. */
/* v2.0 - 8:10 external PC trace. PCST Width and DCLK Division Factor */
#define EJTAG_V20_IMP_NOPB		(1 << 7) /* no processor breaks */
#define EJTAG_V20_IMP_NODB		(1 << 6) /* no data breaks */
#define EJTAG_V20_IMP_NOIB		(1 << 5) /* no instruction breaks implemented */
/* v2.0 - 1:4 Number of Break Channels. */
#define EJTAG_V20_IMP_BCHANNELS_MASK	0xf
#define EJTAG_V20_IMP_BCHANNELS_SHIFT	1
#define EJTAG_IMP_MIPS64		(1 << 0)

/* Debug Control Register DCR */
#define EJTAG_DCR				0xFF300000
#define EJTAG_DCR_ENM			(1 << 29)
#define EJTAG_DCR_DB			(1 << 17)
#define EJTAG_DCR_IB			(1 << 16)
#define EJTAG_DCR_INTE			(1 << 4)
#define EJTAG_DCR_MP			(1 << 2)

/* breakpoint support */
/* EJTAG_V20_* was tested on Broadcom BCM7401
 * and may or will differ with other hardware. For example EZ4021-FC. */
#define EJTAG_V20_IBS			0xFF300004
#define EJTAG_V20_IBA0			0xFF300100
#define EJTAG_V20_IBC_OFFS		0x4	/* IBC Offset */
#define EJTAG_V20_IBM_OFFS		0x8
#define EJTAG_V20_IBAn_STEP		0x10	/* Offset for next channel */
#define EJTAG_V20_DBS			0xFF300008
#define EJTAG_V20_DBA0			0xFF300200
#define EJTAG_V20_DBC_OFFS		0x4
#define EJTAG_V20_DBM_OFFS		0x8
#define EJTAG_V20_DBV_OFFS		0xc
#define EJTAG_V20_DBAn_STEP		0x10

#define EJTAG_V25_IBS			0xFF301000
#define EJTAG_V25_IBA0			0xFF301100
#define EJTAG_V25_IBM_OFFS		0x8
#define EJTAG_V25_IBASID_OFFS		0x10
#define EJTAG_V25_IBC_OFFS		0x18
#define EJTAG_V25_IBAn_STEP		0x100
#define EJTAG_V25_DBS			0xFF302000
#define EJTAG_V25_DBA0			0xFF302100
#define EJTAG_V25_DBM_OFFS		0x8
#define EJTAG_V25_DBASID_OFFS		0x10
#define EJTAG_V25_DBC_OFFS		0x18
#define EJTAG_V25_DBV_OFFS		0x20
#define EJTAG_V25_DBAn_STEP		0x100

#define	EJTAG_DBCn_NOSB			(1 << 13)
#define	EJTAG_DBCn_NOLB			(1 << 12)
#define	EJTAG_DBCn_BLM_MASK		0xff
#define	EJTAG_DBCn_BLM_SHIFT	4
#define	EJTAG_DBCn_BE			(1 << 0)

#define EJTAG_VERSION_20		0
#define EJTAG_VERSION_25		1
#define EJTAG_VERSION_26		2
#define EJTAG_VERSION_31		3
#define EJTAG_VERSION_41		4
#define EJTAG_VERSION_51		5

/*
 * Additional defines for MIPS64 EJTAG
 */
#define EJTAG64_DCR			0xFFFFFFFFFF300000ull
#define EJTAG64_DCR_ENM			(1llu << 29)
#define EJTAG64_DCR_DB			(1llu << 17)
#define EJTAG64_DCR_IB			(1llu << 16)
#define EJTAG64_DCR_INTE		(1llu << 4)
#define EJTAG64_DCR_MP			(1llu << 2)
#define EJTAG64_V25_DBA0		0xFFFFFFFFFF302100ull
#define EJTAG64_V25_DBS			0xFFFFFFFFFF302000ull
#define EJTAG64_V25_IBA0		0xFFFFFFFFFF301100ull
#define EJTAG64_V25_IBS			0xFFFFFFFFFF301000ull

struct mips_ejtag {
	struct jtag_tap *tap;
	uint32_t impcode;
	uint32_t idcode;
	uint32_t ejtag_ctrl;
	int fast_access_save;
	uint32_t config_regs;	/* number of config registers read */
	uint32_t config[4];	/* cp0 config to config3 */

	uint32_t reg8;
	uint32_t reg9;
	unsigned scan_delay;
	int mode;
	uint32_t pa_ctrl;
	uint32_t pa_addr;
	unsigned int ejtag_version;
	uint32_t isa;
	uint32_t endianness;

	/* Memory-Mapped Registers. This addresses are not same on different
	 * EJTAG versions. */
	uint32_t debug_caps;
	uint32_t ejtag_ibs_addr;	/* Instruction Address Break Status */
	uint32_t ejtag_iba0_addr;	/* IAB channel 0 */
	uint32_t ejtag_ibc_offs;	/* IAB Control offset */
	uint32_t ejtag_ibm_offs;	/* IAB Mask offset */
	uint32_t ejtag_ibasid_offs;	/* IAB ASID (4Kc) */

	uint32_t ejtag_dbs_addr;	/* Data Address Break Status Register */
	uint32_t ejtag_dba0_addr;	/* DAB channel 0 */
	uint32_t ejtag_dbc_offs;	/* DAB Control offset */
	uint32_t ejtag_dbm_offs;	/* DAB Mask offset */
	uint32_t ejtag_dbv_offs;	/* DAB Value offset */
	uint32_t ejtag_dbasid_offs;	/* DAB ASID (4Kc) */

	uint32_t ejtag_iba_step_size;
	uint32_t ejtag_dba_step_size;	/* size of step till next *DBAn register. */
};

void mips_ejtag_set_instr(struct mips_ejtag *ejtag_info, uint32_t new_instr);
int mips_ejtag_enter_debug(struct mips_ejtag *ejtag_info);
int mips_ejtag_exit_debug(struct mips_ejtag *ejtag_info);
int mips64_ejtag_exit_debug(struct mips_ejtag *ejtag_info);
int mips_ejtag_get_idcode(struct mips_ejtag *ejtag_info);
void mips_ejtag_add_scan_96(struct mips_ejtag *ejtag_info,
			    uint32_t ctrl, uint32_t data, uint8_t *in_scan_buf);
int mips_ejtag_drscan_64(struct mips_ejtag *ejtag_info, uint64_t *data);
void mips_ejtag_drscan_32_out(struct mips_ejtag *ejtag_info, uint32_t data);
int mips_ejtag_drscan_32(struct mips_ejtag *ejtag_info, uint32_t *data);
void mips_ejtag_drscan_8_out(struct mips_ejtag *ejtag_info, uint8_t data);
int mips_ejtag_drscan_8(struct mips_ejtag *ejtag_info, uint8_t *data);
int mips_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, int write_t, uint32_t *data);
int mips64_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, bool write_t, uint64_t *data);

int mips_ejtag_init(struct mips_ejtag *ejtag_info);
int mips_ejtag_config_step(struct mips_ejtag *ejtag_info, int enable_step);
int mips64_ejtag_config_step(struct mips_ejtag *ejtag_info, bool enable_step);

static inline void mips_le_to_h_u32(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)arg) = le_to_h_u32(in);
}

static inline void mips_le_to_h_u64(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint64_t *)arg) = le_to_h_u64(in);
}

#endif /* OPENOCD_TARGET_MIPS_EJTAG_H */
