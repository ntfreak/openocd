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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include "nds32_reg.h"

static bool nds32_reg_init_done;
static struct nds32_reg_s nds32_regs[TOTAL_REG_NUM];
static const struct nds32_reg_exception_s nds32_ex_reg_values[] = {
	{IR0, 3, 0x3, 2},
	{IR0, 3, 0x3, 3},
	{IR1, 3, 0x3, 2},
	{IR1, 3, 0x3, 3},
	{IR2, 3, 0x3, 2},
	{IR2, 3, 0x3, 3},
	{MR3, 1, 0x7, 0},
	{MR3, 1, 0x7, 4},
	{MR3, 1, 0x7, 6},
	{MR3, 8, 0x7, 3},
	{0, 0, 0, 0},
};

static inline void nds32_reg_set(uint32_t number, const char *simple_mnemonic,
		const char *symbolic_mnemonic, uint32_t sr_index,
		enum nds32_reg_type_s type, uint8_t size)
{
	nds32_regs[number].simple_mnemonic = simple_mnemonic;
	nds32_regs[number].symbolic_mnemonic = symbolic_mnemonic;
	nds32_regs[number].sr_index = sr_index;
	nds32_regs[number].type = type;
	nds32_regs[number].size = size;
}

void nds32_reg_init(void)
{
	if (nds32_reg_init_done == true)
		return;

	nds32_reg_set(R0, "r0", "r0", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R1, "r1", "r1", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R2, "r2", "r2", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R3, "r3", "r3", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R4, "r4", "r4", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R5, "r5", "r5", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R6, "r6", "r6", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R7, "r7", "r7", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R8, "r8", "r8", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R9, "r9", "r9", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R10, "r10", "r10", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R11, "r11", "r11", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R12, "r12", "r12", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R13, "r13", "r13", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R14, "r14", "r14", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R15, "r15", "r15", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R16, "r16", "r16", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R17, "r17", "r17", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R18, "r18", "r18", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R19, "r19", "r19", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R20, "r20", "r20", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R21, "r21", "r21", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R22, "r22", "r22", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R23, "r23", "r23", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R24, "r24", "r24", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R25, "r25", "r25", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R26, "r26", "p0", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R27, "r27", "p1", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R28, "fp", "fp", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R29, "gp", "gp", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R30, "lp", "lp", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(R31, "sp", "sp", 0, NDS32_REG_TYPE_GPR, 32);
	nds32_reg_set(PC, "pc", "pc", 31, NDS32_REG_TYPE_SPR, 32);

	nds32_reg_set(D0LO, "d0lo", "d0lo", 0, NDS32_REG_TYPE_SPR, 32);
	nds32_reg_set(D0HI, "d0hi", "d0hi", 1, NDS32_REG_TYPE_SPR, 32);
	nds32_reg_set(D1LO, "d1lo", "d1lo", 2, NDS32_REG_TYPE_SPR, 32);
	nds32_reg_set(D1HI, "d1hi", "d1hi", 3, NDS32_REG_TYPE_SPR, 32);
	nds32_reg_set(ITB, "itb", "itb", 28, NDS32_REG_TYPE_SPR, 32);
	nds32_reg_set(IFC_LP, "ifc_lp", "ifc_lp", 29, NDS32_REG_TYPE_SPR, 32);

	nds32_reg_set(CR0, "cr0", "CPU_VER", SRIDX(0, 0, 0), NDS32_REG_TYPE_CR, 32);
	nds32_reg_set(CR1, "cr1", "ICM_CFG", SRIDX(0, 1, 0), NDS32_REG_TYPE_CR, 32);
	nds32_reg_set(CR2, "cr2", "DCM_CFG", SRIDX(0, 2, 0), NDS32_REG_TYPE_CR, 32);
	nds32_reg_set(CR3, "cr3", "MMU_CFG", SRIDX(0, 3, 0), NDS32_REG_TYPE_CR, 32);
	nds32_reg_set(CR4, "cr4", "MSC_CFG", SRIDX(0, 4, 0), NDS32_REG_TYPE_CR, 32);
	nds32_reg_set(CR5, "cr5", "CORE_ID", SRIDX(0, 0, 1), NDS32_REG_TYPE_CR, 32);
	nds32_reg_set(CR6, "cr6", "FUCOP_EXIST", SRIDX(0, 5, 0), NDS32_REG_TYPE_CR, 32);

	nds32_reg_set(IR0, "ir0", "PSW", SRIDX(1, 0, 0), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR1, "ir1", "IPSW", SRIDX(1, 0, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR2, "ir2", "P_IPSW", SRIDX(1, 0, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR3, "ir3", "IVB", SRIDX(1, 1, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR4, "ir4", "EVA", SRIDX(1, 2, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR5, "ir5", "P_EVA", SRIDX(1, 2, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR6, "ir6", "ITYPE", SRIDX(1, 3, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR7, "ir7", "P_ITYPE", SRIDX(1, 3, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR8, "ir8", "MERR", SRIDX(1, 4, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR9, "ir9", "IPC", SRIDX(1, 5, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR10, "ir10", "P_IPC", SRIDX(1, 5, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR11, "ir11", "OIPC", SRIDX(1, 5, 3), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR12, "ir12", "P_P0", SRIDX(1, 6, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR13, "ir13", "P_P1", SRIDX(1, 7, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR14, "ir14", "INT_MASK", SRIDX(1, 8, 0), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR15, "ir15", "INT_PEND", SRIDX(1, 9, 0), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR16, "ir16", "", SRIDX(1, 10, 0), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR17, "ir17", "", SRIDX(1, 10, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR18, "ir18", "", SRIDX(1, 11, 0), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR19, "ir19", "", SRIDX(1, 1, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR20, "ir20", "", SRIDX(1, 10, 2), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR21, "ir21", "", SRIDX(1, 10, 3), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR22, "ir22", "", SRIDX(1, 10, 4), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR23, "ir23", "", SRIDX(1, 10, 5), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR24, "ir24", "", SRIDX(1, 10, 6), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR25, "ir25", "", SRIDX(1, 10, 7), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR26, "ir26", "", SRIDX(1, 8, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR27, "ir27", "", SRIDX(1, 9, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR28, "ir28", "", SRIDX(1, 11, 1), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR29, "ir29", "", SRIDX(1, 9, 4), NDS32_REG_TYPE_IR, 32);
	nds32_reg_set(IR30, "ir30", "", SRIDX(1, 1, 3), NDS32_REG_TYPE_IR, 32);

	nds32_reg_set(MR0, "mr0", "MMU_CTL", SRIDX(2, 0, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR1, "mr1", "L1_PPTB", SRIDX(2, 1, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR2, "mr2", "TLB_VPN", SRIDX(2, 2, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR3, "mr3", "TLB_DATA", SRIDX(2, 3, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR4, "mr4", "TLB_MISC", SRIDX(2, 4, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR5, "mr5", "VLPT_IDX", SRIDX(2, 5, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR6, "mr6", "ILMB", SRIDX(2, 6, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR7, "mr7", "DLMB", SRIDX(2, 7, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR8, "mr8", "CACHE_CTL", SRIDX(2, 8, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR9, "mr9", "HSMP_SADDR", SRIDX(2, 9, 0), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR10, "mr10", "HSMP_EADDR", SRIDX(2, 9, 1), NDS32_REG_TYPE_MR, 32);
	nds32_reg_set(MR11, "mr11", "", SRIDX(2, 0, 1), NDS32_REG_TYPE_MR, 32);

	nds32_reg_set(DR0, "dr0", "BPC0", SRIDX(3, 0, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR1, "dr1", "BPA0", SRIDX(3, 1, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR2, "dr2", "BPAM0", SRIDX(3, 2, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR3, "dr3", "BPV0", SRIDX(3, 3, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR4, "dr4", "BPCID0", SRIDX(3, 4, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR5, "dr5", "BPC1", SRIDX(3, 0, 1), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR6, "dr6", "BPA1", SRIDX(3, 1, 1), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR7, "dr7", "BPAM1", SRIDX(3, 2, 1), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR8, "dr8", "BPV1", SRIDX(3, 3, 1), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR9, "dr9", "BPCID1", SRIDX(3, 4, 1), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR10, "dr10", "BPC2", SRIDX(3, 0, 2), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR11, "dr11", "BPA2", SRIDX(3, 1, 2), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR12, "dr12", "BPAM2", SRIDX(3, 2, 2), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR13, "dr13", "BPV2", SRIDX(3, 3, 2), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR14, "dr14", "BPCID2", SRIDX(3, 4, 2), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR15, "dr15", "BPC3", SRIDX(3, 0, 3), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR16, "dr16", "BPA3", SRIDX(3, 1, 3), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR17, "dr17", "BPAM3", SRIDX(3, 2, 3), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR18, "dr18", "BPV3", SRIDX(3, 3, 3), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR19, "dr19", "BPCID3", SRIDX(3, 4, 3), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR20, "dr20", "BPC4", SRIDX(3, 0, 4), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR21, "dr21", "BPA4", SRIDX(3, 1, 4), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR22, "dr22", "BPAM4", SRIDX(3, 2, 4), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR23, "dr23", "BPV4", SRIDX(3, 3, 4), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR24, "dr24", "BPCID4", SRIDX(3, 4, 4), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR25, "dr25", "BPC5", SRIDX(3, 0, 5), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR26, "dr26", "BPA5", SRIDX(3, 1, 5), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR27, "dr27", "BPAM5", SRIDX(3, 2, 5), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR28, "dr28", "BPV5", SRIDX(3, 3, 5), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR29, "dr29", "BPCID5", SRIDX(3, 4, 5), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR30, "dr30", "BPC6", SRIDX(3, 0, 6), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR31, "dr31", "BPA6", SRIDX(3, 1, 6), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR32, "dr32", "BPAM6", SRIDX(3, 2, 6), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR33, "dr33", "BPV6", SRIDX(3, 3, 6), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR34, "dr34", "BPCID6", SRIDX(3, 4, 6), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR35, "dr35", "BPC7", SRIDX(3, 0, 7), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR36, "dr36", "BPA7", SRIDX(3, 1, 7), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR37, "dr37", "BPAM7", SRIDX(3, 2, 7), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR38, "dr38", "BPV7", SRIDX(3, 3, 7), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR39, "dr39", "BPCID7", SRIDX(3, 4, 7), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR40, "dr40", "EDM_CFG", SRIDX(3, 5, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR41, "dr41", "EDMSW", SRIDX(3, 6, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR42, "dr42", "EDM_CTL", SRIDX(3, 7, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR43, "dr43", "EDM_DTR", SRIDX(3, 8, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR44, "dr44", "BPMTC", SRIDX(3, 9, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR45, "dr45", "DIMBR", SRIDX(3, 10, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR46, "dr46", "TECR0", SRIDX(3, 14, 0), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR47, "dr47", "TECR1", SRIDX(3, 14, 1), NDS32_REG_TYPE_DR, 32);
	nds32_reg_set(DR48, "dr48", "", SRIDX(3, 11, 0), NDS32_REG_TYPE_DR, 32);

	nds32_reg_set(PFR0, "pfr0", "PFMC0", SRIDX(4, 0, 0), NDS32_REG_TYPE_PFR, 32);
	nds32_reg_set(PFR1, "pfr1", "PFMC1", SRIDX(4, 0, 1), NDS32_REG_TYPE_PFR, 32);
	nds32_reg_set(PFR2, "pfr2", "PFMC2", SRIDX(4, 0, 2), NDS32_REG_TYPE_PFR, 32);
	nds32_reg_set(PFR3, "pfr3", "PFM_CTL", SRIDX(4, 1, 0), NDS32_REG_TYPE_PFR, 32);

	nds32_reg_set(DMAR0, "dmar0", "DMA_CFG", SRIDX(5, 0, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR1, "dmar1", "DMA_GCSW", SRIDX(5, 1, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR2, "dmar2", "DMA_CHNSEL", SRIDX(5, 2, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR3, "dmar3", "DMA_ACT", SRIDX(5, 3, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR4, "dmar4", "DMA_SETUP", SRIDX(5, 4, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR5, "dmar5", "DMA_ISADDR", SRIDX(5, 5, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR6, "dmar6", "DMA_ESADDR", SRIDX(5, 6, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR7, "dmar7", "DMA_TCNT", SRIDX(5, 7, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR8, "dmar8", "DMA_STATUS", SRIDX(5, 8, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR9, "dmar9", "DMA_2DSET", SRIDX(5, 9, 0), NDS32_REG_TYPE_DMAR, 32);
	nds32_reg_set(DMAR10, "dmar10", "DMA_2DSCTL", SRIDX(5, 9, 1), NDS32_REG_TYPE_DMAR, 32);

	nds32_reg_set(RACR, "racr", "PRUSR_ACC_CTL", SRIDX(4, 4, 0), NDS32_REG_TYPE_RACR, 32);
	nds32_reg_set(FUCPR, "fucpr", "FUCOP_CTL", SRIDX(4, 5, 0), NDS32_REG_TYPE_RACR, 32);

	nds32_reg_set(IDR0, "idr0", "SDZ_CTL", SRIDX(2, 15, 0), NDS32_REG_TYPE_IDR, 32);
	nds32_reg_set(IDR1, "idr1", "MISC_CTL", SRIDX(2, 15, 1), NDS32_REG_TYPE_IDR, 32);

	nds32_reg_set(SECUR0, "secur0", "", SRIDX(6, 0, 0), NDS32_REG_TYPE_SECURE, 32);

	nds32_reg_set(D0L24, "D0L24", "D0L24", 0x10, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(D1L24, "D1L24", "D1L24", 0x11, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I0, "I0", "I0", 0x0, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I1, "I1", "I1", 0x1, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I2, "I2", "I2", 0x2, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I3, "I3", "I3", 0x3, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I4, "I4", "I4", 0x4, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I5, "I5", "I5", 0x5, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I6, "I6", "I6", 0x6, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(I7, "I7", "I7", 0x7, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(M1, "M1", "M1", 0x9, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(M2, "M2", "M2", 0xA, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(M3, "M3", "M3", 0xB, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(M5, "M5", "M5", 0xD, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(M6, "M6", "M6", 0xE, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(M7, "M7", "M7", 0xF, NDS32_REG_TYPE_AUMR, 32);

	nds32_reg_set(MOD, "MOD", "MOD", 0x8, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(LBE, "LBE", "LBE", 0x18, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(LE, "LE", "LE", 0x19, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(LC, "LC", "LC", 0x1A, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(ADM_VBASE, "ADM_VBASE", "ADM_VBASE", 0x1B, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(SHFT_CTL0, "SHFT_CTL0", "SHFT_CTL0", 0x12, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(SHFT_CTL1, "SHFT_CTL1", "SHFT_CTL1", 0x13, NDS32_REG_TYPE_AUMR, 32);

	nds32_reg_set(CB_CTL, "CB_CTL", "CB_CTL", 0x1F, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBB0, "CBB0", "CBB0", 0x0, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBB1, "CBB1", "CBB1", 0x1, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBB2, "CBB2", "CBB2", 0x2, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBB3, "CBB3", "CBB3", 0x3, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBE0, "CBE0", "CBE0", 0x4, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBE1, "CBE1", "CBE1", 0x5, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBE2, "CBE2", "CBE2", 0x6, NDS32_REG_TYPE_AUMR, 32);
	nds32_reg_set(CBE3, "CBE3", "CBE3", 0x7, NDS32_REG_TYPE_AUMR, 32);

	nds32_reg_set(FPCSR, "fpcsr", "FPCSR", 0x7, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FPCFG, "fpcfg", "FPCFG", 0x7, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS0, "fs0", "FS0", 0, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS1, "fs1", "FS1", 1, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS2, "fs2", "FS2", 2, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS3, "fs3", "FS3", 3, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS4, "fs4", "FS4", 4, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS5, "fs5", "FS5", 5, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS6, "fs6", "FS6", 6, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS7, "fs7", "FS7", 7, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS8, "fs8", "FS8", 8, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS9, "fs9", "FS9", 9, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS10, "fs10", "FS10", 10, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS11, "fs11", "FS11", 11, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS12, "fs12", "FS12", 12, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS13, "fs13", "FS13", 13, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS14, "fs14", "FS14", 14, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS15, "fs15", "FS15", 15, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS16, "fs16", "FS16", 16, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS17, "fs17", "FS17", 17, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS18, "fs18", "FS18", 18, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS19, "fs19", "FS19", 19, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS20, "fs20", "FS20", 20, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS21, "fs21", "FS21", 21, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS22, "fs22", "FS22", 22, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS23, "fs23", "FS23", 23, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS24, "fs24", "FS24", 24, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS25, "fs25", "FS25", 25, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS26, "fs26", "FS26", 26, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS27, "fs27", "FS27", 27, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS28, "fs28", "FS28", 28, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS29, "fs29", "FS29", 29, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS30, "fs30", "FS30", 30, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FS31, "fs31", "FS31", 31, NDS32_REG_TYPE_FPU, 32);
	nds32_reg_set(FD0, "fd0", "FD0", 0, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD1, "fd1", "FD1", 1, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD2, "fd2", "FD2", 2, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD3, "fd3", "FD3", 3, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD4, "fd4", "FD4", 4, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD5, "fd5", "FD5", 5, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD6, "fd6", "FD6", 6, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD7, "fd7", "FD7", 7, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD8, "fd8", "FD8", 8, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD9, "fd9", "FD9", 9, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD10, "fd10", "FD10", 10, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD11, "fd11", "FD11", 11, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD12, "fd12", "FD12", 12, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD13, "fd13", "FD13", 13, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD14, "fd14", "FD14", 14, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD15, "fd15", "FD15", 15, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD16, "fd16", "FD16", 16, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD17, "fd17", "FD17", 17, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD18, "fd18", "FD18", 18, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD19, "fd19", "FD19", 19, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD20, "fd20", "FD20", 20, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD21, "fd21", "FD21", 21, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD22, "fd22", "FD22", 22, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD23, "fd23", "FD23", 23, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD24, "fd24", "FD24", 24, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD25, "fd25", "FD25", 25, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD26, "fd26", "FD26", 26, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD27, "fd27", "FD27", 27, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD28, "fd28", "FD28", 28, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD29, "fd29", "FD29", 29, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD30, "fd30", "FD30", 30, NDS32_REG_TYPE_FPU, 64);
	nds32_reg_set(FD31, "fd31", "FD31", 31, NDS32_REG_TYPE_FPU, 64);

	nds32_reg_init_done = true;
}

uint32_t nds32_reg_sr_index(uint32_t number)
{
	return nds32_regs[number].sr_index;
}

enum nds32_reg_type_s nds32_reg_type(uint32_t number)
{
	return nds32_regs[number].type;
}

uint8_t nds32_reg_size(uint32_t number)
{
	return nds32_regs[number].size;
}

const char *nds32_reg_simple_name(uint32_t number)
{
	return nds32_regs[number].simple_mnemonic;
}

const char *nds32_reg_symbolic_name(uint32_t number)
{
	return nds32_regs[number].symbolic_mnemonic;
}

bool nds32_reg_exception(uint32_t number, uint32_t value)
{
	int i;
	const struct nds32_reg_exception_s *ex_reg_value;
	uint32_t field_value;

	i = 0;
	while (nds32_ex_reg_values[i].reg_num != 0) {
		ex_reg_value = nds32_ex_reg_values + i;

		if (ex_reg_value->reg_num == number) {
			field_value = (value >> ex_reg_value->ex_value_bit_pos) &
				ex_reg_value->ex_value_mask;
			if (field_value == ex_reg_value->ex_value) {
				LOG_WARNING("It will generate exceptions as setting %" PRIu32 " to %s",
						value, nds32_regs[number].simple_mnemonic);
				return true;
			}
		}

		i++;
	}

	return false;
}
