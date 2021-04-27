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
 * rn: base register
 * list: for each bit in list: store register
 * s: in privileged mode: store user-mode registers
 * w = 1: update the base register. w = 0: leave the base register untouched
 */
#define ARMV4_5_STMIA(rn, list, s, w) \
	(0xe8800000 | ((s) << 22) | ((w) << 21) | ((rn) << 16) | (list))

/* Load multiple increment after
 * rn: base register
 * list: for each bit in list: store register
 * s: in privileged mode: store user-mode registers
 * w = 1: update the base register. w = 0: leave the base register untouched
 */
#define ARMV4_5_LDMIA(rn, list, s, w) \
	(0xe8900000 | ((s) << 22) | ((w) << 21) | ((rn) << 16) | (list))

/* MOV r8, r8 */
#define ARMV4_5_NOP					(0xe1a08008)

/* Move PSR to general purpose register
 * r = 1: SPSR r = 0: CPSR
 * rn: target register
 */
#define ARMV4_5_MRS(rn, r)	(0xe10f0000 | ((r) << 22) | ((rn) << 12))

/* Store register
 * rd: register to store
 * rn: base register
 */
#define ARMV4_5_STR(rd, rn)	(0xe5800000 | ((rd) << 12) | ((rn) << 16))

/* Load register
 * rd: register to load
 * rn: base register
 */
#define ARMV4_5_LDR(rd, rn)	(0xe5900000 | ((rd) << 12) | ((rn) << 16))

/* Move general purpose register to PSR
 * r = 1: SPSR r = 0: CPSR
 * field: Field mask
 * 1: control field 2: extension field 4: status field 8: flags field
 * rm: source register
 */
#define ARMV4_5_MSR_GP(rm, field, r) \
	(0xe120f000 | (rm) | ((field) << 16) | ((r) << 22))
#define ARMV4_5_MSR_IM(im, rotate, field, r) \
	(0xe320f000 | (im)  | ((rotate) << 8) | ((field) << 16) | ((r) << 22))

/* Load Register Word Immediate Post-Index
 * rd: register to load
 * rn: base register
 */
#define ARMV4_5_LDRW_IP(rd, rn)	(0xe4900004 | ((rd) << 12) | ((rn) << 16))

/* Load Register Halfword Immediate Post-Index
 * rd: register to load
 * rn: base register
 */
#define ARMV4_5_LDRH_IP(rd, rn)	(0xe0d000b2 | ((rd) << 12) | ((rn) << 16))

/* Load Register Byte Immediate Post-Index
 * rd: register to load
 * rn: base register
 */
#define ARMV4_5_LDRB_IP(rd, rn)	(0xe4d00001 | ((rd) << 12) | ((rn) << 16))

/* Store register Word Immediate Post-Index
 * rd: register to store
 * rn: base register
 */
#define ARMV4_5_STRW_IP(rd, rn)	(0xe4800004 | ((rd) << 12) | ((rn) << 16))

/* Store register Halfword Immediate Post-Index
 * rd: register to store
 * rn: base register
 */
#define ARMV4_5_STRH_IP(rd, rn)	(0xe0c000b2 | ((rd) << 12) | ((rn) << 16))

/* Store register Byte Immediate Post-Index
 * rd: register to store
 * rn: base register
 */
#define ARMV4_5_STRB_IP(rd, rn)	(0xe4c00001 | ((rd) << 12) | ((rn) << 16))

/* Branch (and Link)
 * im: Branch target (left-shifted by 2 bits, added to PC)
 * l: 1: branch and link 0: branch only
 */
#define ARMV4_5_B(im, l) (0xea000000 | (im) | ((l) << 24))

/* Branch and exchange (ARM state)
 * rm: register holding branch target address
 */
#define ARMV4_5_BX(rm) (0xe12fff10 | (rm))

/* Copies two words from two ARM core registers
 * into a doubleword extension register, or
 * from a doubleword extension register to two ARM core registers.
 * See Armv7-A arch reference manual section A8.8.345
 * rt:   Arm core register 1
 * rt2:  Arm core register 2
 * vm:   The doubleword extension register
 * m:    m = UInt(m:vm);
 * op:   to_arm_registers = (op == ‘1’);
 */
#define ARMV4_5_VMOV(op, rt2, rt, m, vm) \
	(0xec400b10 | ((op) << 20) | ((rt2) << 16) | \
	((rt) << 12) | ((m) << 5) | (vm))

/* Moves the value of the FPSCR to an ARM core register
 * rt: Arm core register
 */
#define ARMV4_5_VMRS(rt) (0xeef10a10 | ((rt) << 12))

/* Moves the value of an ARM core register to the FPSCR.
 * rt: Arm core register
 */
#define ARMV4_5_VMSR(rt) (0xeee10a10 | ((rt) << 12))

/* Store data from coprocessor to consecutive memory
 * See Armv7-A arch doc section A8.6.187
 * p:    1=index mode (offset from rn)
 * u:    1=add, 0=subtract  rn address with imm
 * d:    Opcode D encoding
 * w:    write back the offset start address to the rn register
 * cp:   Coprocessor number (4 bits)
 * crd:  Coprocessor source register (4 bits)
 * rn:   Base register for memory address (4 bits)
 * imm:  Immediate value (0 - 1020, must be divisible by 4)
 */
#define ARMV4_5_STC(p, u, d, w, cp, crd, rn, imm) \
	(0xec000000 | ((p) << 24) | ((u) << 23) | ((d) << 22) | \
	((w) << 21) | ((rn) << 16) | ((crd) << 12) | ((cp) << 8) | ((imm)>>2))

/* Loads data from consecutive memory to coprocessor
 * See Armv7-A arch doc section A8.6.51
 * p:    1=index mode (offset from rn)
 * u:    1=add, 0=subtract  rn address with imm
 * d:    Opcode D encoding
 * w:    write back the offset start address to the rn register
 * cp:   Coprocessor number (4 bits)
 * crd:  Coprocessor dest register (4 bits)
 * rn:   Base register for memory address (4 bits)
 * imm:  Immediate value (0 - 1020, must be divisible by 4)
 */
#define ARMV4_5_LDC(p, u, d, w, cp, crd, rn, imm) \
	(0xec100000 | ((p) << 24) | ((u) << 23) | ((d) << 22) | \
	((w) << 21) | ((rn) << 16) | ((crd) << 12) | ((cp) << 8) | ((imm) >> 2))

/* Move to ARM register from coprocessor
 * cp: Coprocessor number
 * op1: Coprocessor opcode
 * rd: destination register
 * crn: first coprocessor operand
 * crm: second coprocessor operand
 * op2: Second coprocessor opcode
 */
#define ARMV4_5_MRC(cp, op1, rd, crn, crm, op2) \
	(0xee100010 | (crm) | ((op2) << 5) | ((cp) << 8) \
	| ((rd) << 12) | ((crn) << 16) | ((op1) << 21))

/* Move to coprocessor from ARM register
 * cp: Coprocessor number
 * op1: Coprocessor opcode
 * rd: destination register
 * crn: first coprocessor operand
 * crm: second coprocessor operand
 * op2: Second coprocessor opcode
 */
#define ARMV4_5_MCR(cp, op1, rd, crn, crm, op2) \
	(0xee000010 | (crm) | ((op2) << 5) | ((cp) << 8) \
	| ((rd) << 12) | ((crn) << 16) | ((op1) << 21))

/* Breakpoint instruction (ARMv5)
 * im: 16-bit immediate
 */
#define ARMV5_BKPT(im) (0xe1200070 | ((im & 0xfff0) << 4) | (im & 0xf))


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
 * rd: source register
 * rn: base register
 */
#define ARMV4_5_T_STR(rd, rn) \
	((0x6000 | (rd) | ((rn) << 3)) | \
	((0x6000 | (rd) | ((rn) << 3)) << 16))

/* Load register (Thumb state)
 * rd: destination register
 * rn: base register
 */
#define ARMV4_5_T_LDR(rd, rn) \
	((0x6800 | ((rn) << 3) | (rd)) \
	| ((0x6800 | ((rn) << 3) | (rd)) << 16))

/* Load multiple (Thumb state)
 * rn: base register
 * list: for each bit in list: store register
 */
#define ARMV4_5_T_LDMIA(rn, list) \
	((0xc800 | ((rn) << 8) | (list)) \
	| ((0xc800 | ((rn) << 8) | (list)) << 16))

/* Load register with PC relative addressing
 * rd: register to load
 */
#define ARMV4_5_T_LDR_PCREL(rd) \
	((0x4800 | ((rd) << 8)) \
	| ((0x4800 | ((rd) << 8)) << 16))

/* Move hi register (Thumb mode)
 * rd: destination register
 * rm: source register
 */
#define ARMV4_5_T_MOV(rd, rm) \
	((0x4600 | ((rd) & 0x7) | (((rd) & 0x8) << 4) | \
		(((rm) & 0x7) << 3) | (((rm) & 0x8) << 3)) \
	| ((0x4600 | ((rd) & 0x7) | (((rd) & 0x8) << 4) | \
		(((rm) & 0x7) << 3) | (((rm) & 0x8) << 3)) << 16))

/* No operation (Thumb mode)
 * NOTE:  this is "MOV r8, r8" ... Thumb2 adds two
 * architected NOPs, 16-bit and 32-bit.
 */
#define ARMV4_5_T_NOP	(0x46c0 | (0x46c0 << 16))

/* Move immediate to register (Thumb state)
 * rd: destination register
 * im: 8-bit immediate value
 */
#define ARMV4_5_T_MOV_IM(rd, im) \
	((0x2000 | ((rd) << 8) | (im)) \
	| ((0x2000 | ((rd) << 8) | (im)) << 16))

/* Branch and Exchange
 * rm: register containing branch target
 */
#define ARMV4_5_T_BX(rm) \
	((0x4700 | ((rm) << 3)) \
	| ((0x4700 | ((rm) << 3)) << 16))

/* Branch (Thumb state)
 * imm: Branch target
 */
#define ARMV4_5_T_B(imm) \
	((0xe000 | (imm)) \
	| ((0xe000 | (imm)) << 16))

/* Breakpoint instruction (ARMv5) (Thumb state)
 * Im: 8-bit immediate
 */
#define ARMV5_T_BKPT(im) \
	((0xbe00 | (im)) \
	| ((0xbe00 | (im)) << 16))

/* Move to Register from Special Register
 *	32 bit Thumb2 instruction
 * rd: destination register
 * sysm: source special register
 */
#define ARM_T2_MRS(rd, sysm) \
	((0xF3EF) | ((0x8000 | (rd << 8) | sysm) << 16))

/* Move from Register from Special Register
 *	32 bit Thumb2 instruction
 * rd: source register
 * sysm: destination special register
 */
#define ARM_T2_MSR(sysm, rn) \
	((0xF380 | (rn << 8)) | ((0x8800 | sysm) << 16))

/* Change Processor State.
 *	16 bit Thumb2 instruction
 * rd: source register
 * IF: A_FLAG and/or I_FLAG and/or F_FLAG
 */
#define A_FLAG 4
#define I_FLAG 2
#define F_FLAG 1
#define ARM_T2_CPSID(_if) \
	((0xB660 | (1 << 8) | ((_if)&0x3)) \
	| ((0xB660 | (1 << 8) | ((_if)&0x3)) << 16))
#define ARM_T2_CPSIE(_if) \
	((0xB660 | (0 << 8) | ((_if)&0x3)) \
	| ((0xB660 | (0 << 8) | ((_if)&0x3)) << 16))

#endif /* OPENOCD_TARGET_ARM_OPCODES_H */
