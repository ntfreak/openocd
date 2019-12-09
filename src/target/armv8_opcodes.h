/*
 * Copyright (C) 2015 by pierrr kuo
 * vichy.kuo@gmail.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef OPENOCD_TARGET_ARMV8_OPCODES_H
#define OPENOCD_TARGET_ARMV8_OPCODES_H

#include "arm_opcodes.h"

#define SYSTEM_CUREL_MASK		0xC0
#define SYSTEM_CUREL_SHIFT		6
#define SYSTEM_CUREL_EL0		0x0
#define SYSTEM_CUREL_EL1		0x1
#define SYSTEM_CUREL_EL2		0x2
#define SYSTEM_CUREL_EL3		0x3
#define SYSTEM_CUREL_NONCH		0xF
#define SYSTEM_AARCH64			0x1

#define SYSTEM_AAR64_MODE_EL0t	0x0
#define SYSTEM_AAR64_MODE_EL1t	0x4
#define SYSTEM_AAR64_MODE_EL1h	0x5
#define SYSTEM_AAR64_MODE_EL2t	0x8
#define SYSTEM_AAR64_MODE_EL2h	0x9
#define SYSTEM_AAR64_MODE_EL3t	0xC
#define SYSTEM_AAR64_MODE_EL3h	0xd

#define SYSTEM_DAIF			0b1101101000010001
#define SYSTEM_DAIF_MASK		0x3C0
#define SYSTEM_DAIF_SHIFT		6

#define SYSTEM_ELR_EL1			0b1100001000000001
#define SYSTEM_ELR_EL2			0b1110001000000001
#define SYSTEM_ELR_EL3			0b1111001000000001

#define SYSTEM_SCTLR_EL1	0b1100000010000000
#define SYSTEM_SCTLR_EL2	0b1110000010000000
#define SYSTEM_SCTLR_EL3	0b1111000010000000

#define SYSTEM_FPCR			0b1101101000100000
#define SYSTEM_FPSR			0b1101101000100001
#define SYSTEM_DAIF			0b1101101000010001
#define SYSTEM_NZCV			0b1101101000010000
#define SYSTEM_SP_EL0			0b1100001000001000
#define SYSTEM_SP_EL1			0b1110001000001000
#define SYSTEM_SP_EL2			0b1111001000001000
#define SYSTEM_SP_SEL			0b1100001000010000
#define SYSTEM_SPSR_ABT			0b1110001000011001
#define SYSTEM_SPSR_FIQ			0b1110001000011011
#define SYSTEM_SPSR_IRQ			0b1110001000011000
#define SYSTEM_SPSR_UND			0b1110001000011010

#define SYSTEM_SPSR_EL1			0b1100001000000000
#define SYSTEM_SPSR_EL2			0b1110001000000000
#define SYSTEM_SPSR_EL3			0b1111001000000000

#define SYSTEM_ISR_EL1			0b1100011000001000

#define SYSTEM_DBG_DSPSR_EL0    0b1101101000101000
#define SYSTEM_DBG_DLR_EL0		0b1101101000101001
#define SYSTEM_DBG_DTRRX_EL0	0b1001100000101000
#define SYSTEM_DBG_DTRTX_EL0	0b1001100000101000
#define SYSTEM_DBG_DBGDTR_EL0	0b1001100000100000

#define SYSTEM_CCSIDR			0b1100100000000000
#define SYSTEM_CLIDR			0b1100100000000001
#define SYSTEM_CSSELR			0b1101000000000000
#define SYSTEM_CTYPE			0b1101100000000001
#define SYSTEM_CTR				0b1101100000000001

#define SYSTEM_DCCISW			0b0100001111110010
#define SYSTEM_DCCSW			0b0100001111010010
#define SYSTEM_ICIVAU			0b0101101110101001
#define SYSTEM_DCCVAU			0b0101101111011001
#define SYSTEM_DCCIVAC			0b0101101111110001

#define SYSTEM_MPIDR			0b1100000000000101

#define SYSTEM_TCR_EL1			0b1100000100000010
#define SYSTEM_TCR_EL2			0b1110000100000010
#define SYSTEM_TCR_EL3			0b1111000100000010

#define SYSTEM_TTBR0_EL1		0b1100000100000000
#define SYSTEM_TTBR0_EL2		0b1110000100000000
#define SYSTEM_TTBR0_EL3		0b1111000100000000
#define SYSTEM_TTBR1_EL1		0b1100000100000001

/* ARMv8 address translation */
#define SYSTEM_PAR_EL1			0b1100001110100000
#define SYSTEM_ATS12E0R			0b0110001111000110
#define SYSTEM_ATS12E1R			0b0110001111000100
#define SYSTEM_ATS1E2R			0b0110001111000000
#define SYSTEM_ATS1E3R			0b0111001111000000

/* fault status and fault address */
#define SYSTEM_FAR_EL1			0b1100001100000000
#define SYSTEM_FAR_EL2			0b1110001100000000
#define SYSTEM_FAR_EL3			0b1111001100000000
#define SYSTEM_ESR_EL1			0b1100001010010000
#define SYSTEM_ESR_EL2			0b1110001010010000
#define SYSTEM_ESR_EL3			0b1111001010010000

#define ARMV8_MRS_DSPSR(Rt)	(0xd53b4500 | (Rt))
#define ARMV8_MSR_DSPSR(Rt)	(0xd51b4500 | (Rt))
#define ARMV8_MRS_DLR(Rt)	(0xd53b4520 | (Rt))
#define ARMV8_MSR_DLR(Rt)	(0xd51b4520 | (Rt))

/* T32 instruction to access coprocessor registers */
#define ARMV8_MCR_T1(cp, CRn, opc1, CRm, opc2, Rt) ARMV4_5_MCR(cp, opc1, Rt, CRn, CRm, opc2)
#define ARMV8_MRC_T1(cp, CRn, opc1, CRm, opc2, Rt) ARMV4_5_MRC(cp, opc1, Rt, CRn, CRm, opc2)

/* T32 instructions to access DSPSR and DLR */
#define ARMV8_MRC_DSPSR(Rt) ARMV8_MRC_T1(15, 4, 3, 5, 0, Rt)
#define ARMV8_MCR_DSPSR(Rt) ARMV8_MCR_T1(15, 4, 3, 5, 0, Rt)
#define ARMV8_MRC_DLR(Rt)	ARMV8_MRC_T1(15, 4, 3, 5, 1, Rt)
#define ARMV8_MCR_DLR(Rt)	ARMV8_MCR_T1(15, 4, 3, 5, 1, Rt)

#define ARMV8_DCPS1(IM)		(0xd4a00001 | (((IM) & 0xFFFF) << 5))
#define ARMV8_DCPS2(IM)		(0xd4a00002 | (((IM) & 0xFFFF) << 5))
#define ARMV8_DCPS3(IM)		(0xd4a00003 | (((IM) & 0xFFFF) << 5))
#define ARMV8_DCPS(EL, IM)	(0xd4a00000 | (((IM) & 0xFFFF) << 5) | EL)
#define ARMV8_DCPS_T1(EL)	(0xf78f8000 | EL)
#define ARMV8_DRPS		0xd6bf03e0
#define ARMV8_ERET_T1		0xf3de8f00

#define ARMV8_DSB_SY				0xd5033F9F
#define ARMV8_DSB_SY_T1				0xf3bf8f4f
#define ARMV8_ISB				0xd5033fdf
#define ARMV8_ISB_SY_T1				0xf3bf8f6f

#define ARMV8_MRS(System, Rt)	(0xd5300000 | ((System) << 5) | (Rt))
/* ARM V8 Move to system register. */
#define ARMV8_MSR_GP(System, Rt) \
	(0xd5100000 | ((System) << 5) | (Rt))
/* ARM V8 Move immediate to process state field. */
#define ARMV8_MSR_IM(Op1, CRm, Op2) \
	(0xd500401f | ((Op1) << 16)  | ((CRm) << 8) | ((Op2) << 5))

#define ARMV8_MRS_T1(R, M1, Rd, M) (0xF3E08020 | (R << 20) | (M1 << 16) | (Rd << 8) | (M << 4))
#define ARMV8_MRS_xPSR_T1(R, Rd) (0xF3EF8000 | (R << 20) | (Rd << 8))
#define ARMV8_MSR_GP_T1(R, M1, Rd, M) (0xF3808020 | (R << 20) | (M1 << 8) | (Rd << 16) | (M << 4))
#define ARMV8_MSR_GP_xPSR_T1(R, Rn, mask) (0xF3808000 | (R << 20) | (Rn << 16) | (mask << 8))

#define ARMV8_BKPT(Im) (0xD4200000 | ((Im & 0xffff) << 5))
#define ARMV8_HLT(Im) (0x0D4400000 | ((Im & 0xffff) << 5))
#define ARMV8_HLT_A1(Im) (0xE1000070 | ((Im & 0xFFF0) << 4) | (Im & 0xF))
#define ARMV8_HLT_T1(Im) (0xba80 | (Im & 0x3f))

#define ARMV8_MOVFSP_64(Rt) ((1 << 31) | 0x11000000 | (0x1f << 5) | (Rt))
#define ARMV8_MOVTSP_64(Rt) ((1 << 31) | 0x11000000 | (Rt << 5) | (0x1F))
#define ARMV8_MOVFSP_32(Rt) (0x11000000 | (0x1f << 5) | (Rt))
#define ARMV8_MOVTSP_32(Rt) (0x11000000 | (Rt << 5) | (0x1F))

#define ARMV8_LDRB_IP(Rd, Rn) (0x38401400 | (Rn << 5) | Rd)
#define ARMV8_LDRH_IP(Rd, Rn) (0x78402400 | (Rn << 5) | Rd)
#define ARMV8_LDRW_IP(Rd, Rn) (0xb8404400 | (Rn << 5) | Rd)

#define ARMV8_LDRB_IP_T3(Rd, Rn) (0xf8100b01 | (Rn << 16) | (Rd << 12))
#define ARMV8_LDRH_IP_T3(Rd, Rn) (0xf8300b02 | (Rn << 16) | (Rd << 12))
#define ARMV8_LDRW_IP_T3(Rd, Rn) (0xf8500b04 | (Rn << 16) | (Rd << 12))

#define ARMV8_STRB_IP(Rd, Rn) (0x38001400 | (Rn << 5) | Rd)
#define ARMV8_STRH_IP(Rd, Rn) (0x78002400 | (Rn << 5) | Rd)
#define ARMV8_STRW_IP(Rd, Rn) (0xb8004400 | (Rn << 5) | Rd)

#define ARMV8_STRB_IP_T3(Rd, Rn) (0xf8000b01 | (Rn << 16) | (Rd << 12))
#define ARMV8_STRH_IP_T3(Rd, Rn) (0xf8200b02 | (Rn << 16) | (Rd << 12))
#define ARMV8_STRW_IP_T3(Rd, Rn) (0xf8400b04 | (Rn << 16) | (Rd << 12))

#define ARMV8_MOV_GPR_VFP(Rd, Rn, Index) (0x4e083c00 | (Index << 20) | (Rn << 5) | Rd)
#define ARMV8_MOV_VFP_GPR(Rd, Rn, Index) (0x4e081c00 | (Index << 20) | (Rn << 5) | Rd)

#define ARMV8_MRS_FPCR(Rt)	(0xd53b4400 | (Rt))
#define ARMV8_MRS_FPSR(Rt)	(0xd53b4420 | (Rt))
#define ARMV8_MSR_FPCR(Rt)	(0xd51b4400 | (Rt))
#define ARMV8_MSR_FPSR(Rt)	(0xd51b4420 | (Rt))

#define ARMV8_SYS(System, Rt) (0xD5080000 | ((System) << 5) | Rt)

enum armv8_opcode {
	READ_REG_CTR,
	READ_REG_CLIDR,
	READ_REG_CSSELR,
	READ_REG_CCSIDR,
	WRITE_REG_CSSELR,
	READ_REG_MPIDR,
	READ_REG_DTRRX,
	WRITE_REG_DTRTX,
	WRITE_REG_DSPSR,
	READ_REG_DSPSR,
	ARMV8_OPC_DSB_SY,
	ARMV8_OPC_DCPS,
	ARMV8_OPC_DRPS,
	ARMV8_OPC_ISB_SY,
	ARMV8_OPC_DCCISW,
	ARMV8_OPC_DCCIVAC,
	ARMV8_OPC_ICIVAU,
	ARMV8_OPC_HLT,
	ARMV8_OPC_STRB_IP,
	ARMV8_OPC_STRH_IP,
	ARMV8_OPC_STRW_IP,
	ARMV8_OPC_LDRB_IP,
	ARMV8_OPC_LDRH_IP,
	ARMV8_OPC_LDRW_IP,
	ARMV8_OPC_NUM,
};

extern uint32_t armv8_opcode(struct armv8_common *armv8, enum armv8_opcode);
extern void armv8_select_opcodes(struct armv8_common *armv8, bool state_is_aarch64);

#endif /* OPENOCD_TARGET_ARMV8_OPCODES_H */
