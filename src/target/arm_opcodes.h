/*
 * Copyright (C) 2005 by Dominic Rath
 * Dominic.Rath@gmx.de
 *
 * Copyright (C) 2006 by Magnus Lundin
 * lundin@mlu.mine.nu
 *
 * Copyright (C) 2008 by Spencer Oliver
 * spen@spen-soft.co.uk
 *
 * Copyright (C) 2009 by Øyvind Harboe
 * oyvind.harboe@zylin.com
 *
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OPENOCD_TARGET_ARM_OPCODES_H
#define OPENOCD_TARGET_ARM_OPCODES_H

/**
 * @file
 * Macros used to generate various ARM or Thumb opcodes.
 */

/* ARM mode instructions */

/* Store multiple increment after
 * Rn: base register
 * List: for each bit in list: store register
 * S: in privileged mode: store user-mode registers
 * W = 1: update the base register. W = 0: leave the base register untouched
 */
#define ARMV4_5_STMIA(Rn, List, S, W) \
	(0xe8800000 | ((S) << 22) | ((W) << 21) | ((Rn) << 16) | (List))

/* Load multiple increment after
 * Rn: base register
 * List: for each bit in list: store register
 * S: in privileged mode: store user-mode registers
 * W = 1: update the base register. W = 0: leave the base register untouched
 */
#define ARMV4_5_LDMIA(Rn, List, S, W) \
	(0xe8900000 | ((S) << 22) | ((W) << 21) | ((Rn) << 16) | (List))

/* MOV r8, r8 */
#define ARMV4_5_NOP					(0xe1a08008)

/* Move PSR to general purpose register
 * R = 1: SPSR R = 0: CPSR
 * Rn: target register
 */
#define ARMV4_5_MRS(Rn, R)	(0xe10f0000 | ((R) << 22) | ((Rn) << 12))

/* Store register
 * Rd: register to store
 * Rn: base register
 */
#define ARMV4_5_STR(Rd, Rn)	(0xe5800000 | ((Rd) << 12) | ((Rn) << 16))

/* Load register
 * Rd: register to load
 * Rn: base register
 */
#define ARMV4_5_LDR(Rd, Rn)	(0xe5900000 | ((Rd) << 12) | ((Rn) << 16))

/* Move general purpose register to PSR
 * R = 1: SPSR R = 0: CPSR
 * Field: Field mask
 * 1: control field 2: extension field 4: status field 8: flags field
 * Rm: source register
 */
#define ARMV4_5_MSR_GP(Rm, Field, R) \
	(0xe120f000 | (Rm) | ((Field) << 16) | ((R) << 22))
#define ARMV4_5_MSR_IM(Im, Rotate, Field, R) \
	(0xe320f000 | (Im)  | ((Rotate) << 8) | ((Field) << 16) | ((R) << 22))

/* Load Register Word Immediate Post-Index
 * Rd: register to load
 * Rn: base register
 */
#define ARMV4_5_LDRW_IP(Rd, Rn)	(0xe4900004 | ((Rd) << 12) | ((Rn) << 16))

/* Load Register Halfword Immediate Post-Index
 * Rd: register to load
 * Rn: base register
 */
#define ARMV4_5_LDRH_IP(Rd, Rn)	(0xe0d000b2 | ((Rd) << 12) | ((Rn) << 16))

/* Load Register Byte Immediate Post-Index
 * Rd: register to load
 * Rn: base register
 */
#define ARMV4_5_LDRB_IP(Rd, Rn)	(0xe4d00001 | ((Rd) << 12) | ((Rn) << 16))

/* Store register Word Immediate Post-Index
 * Rd: register to store
 * Rn: base register
 */
#define ARMV4_5_STRW_IP(Rd, Rn)	(0xe4800004 | ((Rd) << 12) | ((Rn) << 16))

/* Store register Halfword Immediate Post-Index
 * Rd: register to store
 * Rn: base register
 */
#define ARMV4_5_STRH_IP(Rd, Rn)	(0xe0c000b2 | ((Rd) << 12) | ((Rn) << 16))

/* Store register Byte Immediate Post-Index
 * Rd: register to store
 * Rn: base register
 */
#define ARMV4_5_STRB_IP(Rd, Rn)	(0xe4c00001 | ((Rd) << 12) | ((Rn) << 16))

/* Branch (and Link)
 * Im: Branch target (left-shifted by 2 bits, added to PC)
 * L: 1: branch and link 0: branch only
 */
#define ARMV4_5_B(Im, L) (0xea000000 | (Im) | ((L) << 24))

/* Branch and exchange (ARM state)
 * Rm: register holding branch target address
 */
#define ARMV4_5_BX(Rm) (0xe12fff10 | (Rm))

/* Copies two words from two ARM core registers
 * into a doubleword extension register, or
 * from a doubleword extension register to two ARM core registers.
 * See Armv7-A arch reference manual section A8.8.345
 * Rt:   Arm core register 1
 * Rt2:  Arm core register 2
 * Vm:   The doubleword extension register
 * M:    m = UInt(M:Vm);
 * op:   to_arm_registers = (op == ‘1’);
 */
#define ARMV4_5_VMOV(op, Rt2, Rt, M, Vm) \
	(0xec400b10 | ((op) << 20) | ((Rt2) << 16) | \
	((Rt) << 12) | ((M) << 5) | (Vm))

/* Moves the value of the FPSCR to an ARM core register
 * Rt: Arm core register
 */
#define ARMV4_5_VMRS(Rt) (0xeef10a10 | ((Rt) << 12))

/* Moves the value of an ARM core register to the FPSCR.
 * Rt: Arm core register
 */
#define ARMV4_5_VMSR(Rt) (0xeee10a10 | ((Rt) << 12))

/* Store data from coprocessor to consecutive memory
 * See Armv7-A arch doc section A8.6.187
 * P:    1=index mode (offset from Rn)
 * U:    1=add, 0=subtract  Rn address with imm
 * D:    Opcode D encoding
 * W:    write back the offset start address to the Rn register
 * CP:   Coprocessor number (4 bits)
 * CRd:  Coprocessor source register (4 bits)
 * Rn:   Base register for memory address (4 bits)
 * imm:  Immediate value (0 - 1020, must be divisible by 4)
 */
#define ARMV4_5_STC(P, U, D, W, CP, CRd, Rn, imm) \
	(0xec000000 | ((P) << 24) | ((U) << 23) | ((D) << 22) | \
	((W) << 21) | ((Rn) << 16) | ((CRd) << 12) | ((CP) << 8) | ((imm)>>2))

/* Loads data from consecutive memory to coprocessor
 * See Armv7-A arch doc section A8.6.51
 * P:    1=index mode (offset from Rn)
 * U:    1=add, 0=subtract  Rn address with imm
 * D:    Opcode D encoding
 * W:    write back the offset start address to the Rn register
 * CP:   Coprocessor number (4 bits)
 * CRd:  Coprocessor dest register (4 bits)
 * Rn:   Base register for memory address (4 bits)
 * imm:  Immediate value (0 - 1020, must be divisible by 4)
 */
#define ARMV4_5_LDC(P, U, D, W, CP, CRd, Rn, imm) \
	(0xec100000 | ((P) << 24) | ((U) << 23) | ((D) << 22) | \
	((W) << 21) | ((Rn) << 16) | ((CRd) << 12) | ((CP) << 8) | ((imm) >> 2))

/* Move to ARM register from coprocessor
 * CP: Coprocessor number
 * op1: Coprocessor opcode
 * Rd: destination register
 * CRn: first coprocessor operand
 * CRm: second coprocessor operand
 * op2: Second coprocessor opcode
 */
#define ARMV4_5_MRC(CP, op1, Rd, CRn, CRm, op2) \
	(0xee100010 | (CRm) | ((op2) << 5) | ((CP) << 8) \
	| ((Rd) << 12) | ((CRn) << 16) | ((op1) << 21))

/* Move to coprocessor from ARM register
 * CP: Coprocessor number
 * op1: Coprocessor opcode
 * Rd: destination register
 * CRn: first coprocessor operand
 * CRm: second coprocessor operand
 * op2: Second coprocessor opcode
 */
#define ARMV4_5_MCR(CP, op1, Rd, CRn, CRm, op2) \
	(0xee000010 | (CRm) | ((op2) << 5) | ((CP) << 8) \
	| ((Rd) << 12) | ((CRn) << 16) | ((op1) << 21))

/* Breakpoint instruction (ARMv5)
 * Im: 16-bit immediate
 */
#define ARMV5_BKPT(Im) (0xe1200070 | ((Im & 0xfff0) << 4) | (Im & 0xf))


/* Thumb mode instructions
 *
 * NOTE: these 16-bit opcodes fill both halves of a word with the same
 * value.  The reason for this is that when we need to execute Thumb
 * opcodes on ARM7/ARM9 cores (to switch to ARM state on debug entry),
 * we must shift 32 bits to the bus using scan chain 1 ... if we write
 * both halves, we don't need to track which half matters.  On ARMv6 and
 * ARMv7 we don't execute Thumb instructions in debug mode; the ITR
 * register does not accept Thumb (or Thumb2) opcodes.
 */

/* Store register (Thumb mode)
 * Rd: source register
 * Rn: base register
 */
#define ARMV4_5_T_STR(Rd, Rn) \
	((0x6000 | (Rd) | ((Rn) << 3)) | \
	((0x6000 | (Rd) | ((Rn) << 3)) << 16))

/* Load register (Thumb state)
 * Rd: destination register
 * Rn: base register
 */
#define ARMV4_5_T_LDR(Rd, Rn) \
	((0x6800 | ((Rn) << 3) | (Rd)) \
	| ((0x6800 | ((Rn) << 3) | (Rd)) << 16))

/* Load multiple (Thumb state)
 * Rn: base register
 * List: for each bit in list: store register
 */
#define ARMV4_5_T_LDMIA(Rn, List) \
	((0xc800 | ((Rn) << 8) | (List)) \
	| ((0xc800 | ((Rn) << 8) | (List)) << 16))

/* Load register with PC relative addressing
 * Rd: register to load
 */
#define ARMV4_5_T_LDR_PCREL(Rd) \
	((0x4800 | ((Rd) << 8)) \
	| ((0x4800 | ((Rd) << 8)) << 16))

/* Move hi register (Thumb mode)
 * Rd: destination register
 * Rm: source register
 */
#define ARMV4_5_T_MOV(Rd, Rm) \
	((0x4600 | ((Rd) & 0x7) | (((Rd) & 0x8) << 4) | \
		(((Rm) & 0x7) << 3) | (((Rm) & 0x8) << 3)) \
	| ((0x4600 | ((Rd) & 0x7) | (((Rd) & 0x8) << 4) | \
		(((Rm) & 0x7) << 3) | (((Rm) & 0x8) << 3)) << 16))

/* No operation (Thumb mode)
 * NOTE:  this is "MOV r8, r8" ... Thumb2 adds two
 * architected NOPs, 16-bit and 32-bit.
 */
#define ARMV4_5_T_NOP	(0x46c0 | (0x46c0 << 16))

/* Move immediate to register (Thumb state)
 * Rd: destination register
 * Im: 8-bit immediate value
 */
#define ARMV4_5_T_MOV_IM(Rd, Im) \
	((0x2000 | ((Rd) << 8) | (Im)) \
	| ((0x2000 | ((Rd) << 8) | (Im)) << 16))

/* Branch and Exchange
 * Rm: register containing branch target
 */
#define ARMV4_5_T_BX(Rm) \
	((0x4700 | ((Rm) << 3)) \
	| ((0x4700 | ((Rm) << 3)) << 16))

/* Branch (Thumb state)
 * Imm: Branch target
 */
#define ARMV4_5_T_B(Imm) \
	((0xe000 | (Imm)) \
	| ((0xe000 | (Imm)) << 16))

/* Breakpoint instruction (ARMv5) (Thumb state)
 * Im: 8-bit immediate
 */
#define ARMV5_T_BKPT(Im) \
	((0xbe00 | (Im)) \
	| ((0xbe00 | (Im)) << 16))

/* Move to Register from Special Register
 *	32 bit Thumb2 instruction
 * Rd: destination register
 * SYSm: source special register
 */
#define ARM_T2_MRS(Rd, SYSm) \
	((0xF3EF) | ((0x8000 | (Rd << 8) | SYSm) << 16))

/* Move from Register from Special Register
 *	32 bit Thumb2 instruction
 * Rd: source register
 * SYSm: destination special register
 */
#define ARM_T2_MSR(SYSm, Rn) \
	((0xF380 | (Rn << 8)) | ((0x8800 | SYSm) << 16))

/* Change Processor State.
 *	16 bit Thumb2 instruction
 * Rd: source register
 * IF: A_FLAG and/or I_FLAG and/or F_FLAG
 */
#define A_FLAG 4
#define I_FLAG 2
#define F_FLAG 1
#define ARM_T2_CPSID(IF) \
	((0xB660 | (1 << 8) | ((IF)&0x3)) \
	| ((0xB660 | (1 << 8) | ((IF)&0x3)) << 16))
#define ARM_T2_CPSIE(IF) \
	((0xB660 | (0 << 8) | ((IF)&0x3)) \
	| ((0xB660 | (0 << 8) | ((IF)&0x3)) << 16))

#endif /* OPENOCD_TARGET_ARM_OPCODES_H */
