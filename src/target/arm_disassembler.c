/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2009 by David Brownell                                  *
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

#include "target.h"
#include "arm_disassembler.h"
#include <helper/log.h>

#if HAVE_CAPSTONE
#include <capstone/capstone.h>
#endif

/*
 * This disassembler supports two main functions for OpenOCD:
 *
 *  - Various "disassemble" commands.  OpenOCD can serve as a
 *    machine-language debugger, without help from GDB.
 *
 *  - Single stepping.  Not all ARM cores support hardware single
 *    stepping.  To work without that support, the debugger must
 *    be able to decode instructions to find out where to put a
 *    "next instruction" breakpoint.
 *
 * In addition, interpretation of ETM trace data needs some of the
 * decoding mechanisms.
 *
 * At this writing (September 2009) neither function is complete.
 *
 *  - ARM decoding
 *     * Old-style syntax (not UAL) is generally used
 *     * VFP instructions are not understood (ARMv5 and later)
 *       except as coprocessor 10/11 operations
 *     * Most ARM instructions through ARMv6 are decoded, but some
 *       of the post-ARMv4 opcodes may not be handled yet
 *		CPS, SDIV, UDIV, LDREX*, STREX*, QASX, ...
 *     * NEON instructions are not understood (ARMv7-A)
 *
 *  - Thumb/Thumb2 decoding
 *     * UAL syntax should be consistently used
 *     * Any Thumb2 instructions used in Cortex-M3 (ARMv7-M) should
 *       be handled properly.  Accordingly, so should the subset
 *       used in Cortex-M0/M1; and "original" 16-bit Thumb from
 *       ARMv4T and ARMv5T.
 *     * Conditional effects of Thumb2 "IT" (if-then) instructions
 *       are not handled:  the affected instructions are not shown
 *       with their now-conditional suffixes.
 *     * Some ARMv6 and ARMv7-M Thumb2 instructions may not be
 *       handled (minimally for coprocessor access).
 *     * SIMD instructions, and some other Thumb2 instructions
 *       from ARMv7-A, are not understood.
 *
 *  - ThumbEE decoding
 *     * As a Thumb2 variant, the Thumb2 comments (above) apply.
 *     * Opcodes changed by ThumbEE mode are not handled; these
 *       instructions wrongly decode as LDM and STM.
 *
 *  - Jazelle decoding ...  no support whatsoever for Jazelle mode
 *    or decoding.  ARM encourages use of the more generic ThumbEE
 *    mode, instead of Jazelle mode, in current chips.
 *
 *  - Single-step/emulation ... spotty support, which is only weakly
 *    tested.  Thumb2 is not supported.  (Arguably a full simulator
 *    is not needed to support just single stepping.  Recognizing
 *    branch vs non-branch instructions suffices, except when the
 *    instruction faults and triggers a synchronous exception which
 *    can be intercepted using other means.)
 *
 * ARM DDI 0406B "ARM Architecture Reference Manual, ARM v7-A and
 * ARM v7-R edition" gives the most complete coverage of the various
 * generations of ARM instructions.  At this writing it is publicly
 * accessible to anyone willing to create an account at the ARM
 * web site; see http://www.arm.com/documentation/ for information.
 *
 * ARM DDI 0403C "ARMv7-M Architecture Reference Manual" provides
 * more details relevant to the Thumb2-only processors (such as
 * the Cortex-M implementations).
 */

/* textual representation of the condition field
 * ALways (default) is omitted (empty string) */
static const char *arm_condition_strings[] = {
	"EQ", "NE", "CS", "CC", "MI", "PL", "VS", "VC", "HI", "LS", "GE", "LT", "GT", "LE", "", "NV"
};

/* make up for C's missing ROR */
static uint32_t ror(uint32_t value, int places)
{
	return (value >> places) | (value << (32 - places));
}

static int evaluate_unknown(uint32_t opcode,
			    uint32_t address, struct arm_instruction *instruction)
{
	instruction->type = ARM_UNDEFINED_INSTRUCTION;
	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
			"\tUNDEFINED INSTRUCTION", address, opcode);
	return ERROR_OK;
}

static int evaluate_pld(uint32_t opcode,
			uint32_t address, struct arm_instruction *instruction)
{
	/* PLD */
	if ((opcode & 0x0d30f000) == 0x0510f000) {
		uint8_t Rn;
		uint8_t U;
		unsigned offset;

		instruction->type = ARM_PLD;
		Rn = (opcode & 0xf0000) >> 16;
		U = (opcode & 0x00800000) >> 23;
		if (Rn == 0xf) {
			/* literal */
			offset = opcode & 0x0fff;
			snprintf(instruction->text, 128,
				 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD %s%d",
				 address, opcode, U ? "" : "-", offset);
		} else {
			uint8_t I, R;

			I = (opcode & 0x02000000) >> 25;
			R = (opcode & 0x00400000) >> 22;

			if (I) {
				/* register PLD{W} [<Rn>,+/-<Rm>{, <shift>}] */
				offset = (opcode & 0x0F80) >> 7;
				uint8_t Rm;
				Rm = opcode & 0xf;

				if (offset == 0) {
					/* No shift */
					snprintf(instruction->text, 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD%s [r%d, %sr%d]",
						 address, opcode, R ? "" : "W", Rn, U ? "" : "-", Rm);

				} else {
					uint8_t shift;
					shift = (opcode & 0x60) >> 5;

					if (shift == 0x0) {
						/* LSL */
						snprintf(instruction->text, 128,
							 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD%s [r%d, %sr%d, LSL #0x%x)",
							 address, opcode, R ? "" : "W", Rn, U ? "" : "-", Rm, offset);
					} else if (shift == 0x1) {
						/* LSR */
						snprintf(instruction->text, 128,
							 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD%s [r%d, %sr%d, LSR #0x%x)",
							 address, opcode, R ? "" : "W", Rn, U ? "" : "-", Rm, offset);
					} else if (shift == 0x2) {
						/* ASR */
						snprintf(instruction->text, 128,
							 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD%s [r%d, %sr%d, ASR #0x%x)",
							 address, opcode, R ? "" : "W", Rn, U ? "" : "-", Rm, offset);
					} else if (shift == 0x3) {
						/* ROR */
						snprintf(instruction->text, 128,
							 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD%s [r%d, %sr%d, ROR #0x%x)",
							 address, opcode, R ? "" : "W", Rn, U ? "" : "-", Rm, offset);
					}
				}
			} else {
				/* immediate PLD{W} [<Rn>, #+/-<imm12>] */
				offset = opcode & 0x0fff;
				if (offset == 0) {
					snprintf(instruction->text, 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD%s [r%d]",
						 address, opcode, R ? "" : "W", Rn);
				} else {
					snprintf(instruction->text, 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tPLD%s [r%d, #%s%d]",
						 address, opcode, R ? "" : "W", Rn, U ? "" : "-", offset);
				}
			}
		}
		return ERROR_OK;
	}
	/* DSB */
	if ((opcode & 0x07f000f0) == 0x05700040) {
		instruction->type = ARM_DSB;

		char *opt;
		switch (opcode & 0x0000000f) {
		case 0xf:
			opt = "SY";
			break;
		case 0xe:
			opt = "ST";
			break;
		case 0xb:
			opt = "ISH";
			break;
		case 0xa:
			opt = "ISHST";
			break;
		case 0x7:
			opt = "NSH";
			break;
		case 0x6:
			opt = "NSHST";
			break;
		case 0x3:
			opt = "OSH";
			break;
		case 0x2:
			opt = "OSHST";
			break;
		default:
			opt = "UNK";
		}

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tDSB %s",
				address, opcode, opt);

		return ERROR_OK;
	}
	/* ISB */
	if ((opcode & 0x07f000f0) == 0x05700060) {
		instruction->type = ARM_ISB;

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tISB %s",
				address, opcode,
				((opcode & 0x0000000f) == 0xf) ? "SY" : "UNK");

		return ERROR_OK;
	}
	return evaluate_unknown(opcode, address, instruction);
}

static int evaluate_srs(uint32_t opcode,
			uint32_t address, struct arm_instruction *instruction)
{
	const char *wback = (opcode & (1 << 21)) ? "!" : "";
	const char *mode = "";

	switch ((opcode >> 23) & 0x3) {
		case 0:
			mode = "DA";
			break;
		case 1:
			/* "IA" is default */
			break;
		case 2:
			mode = "DB";
			break;
		case 3:
			mode = "IB";
			break;
	}

	switch (opcode & 0x0e500000) {
		case 0x08400000:
			snprintf(instruction->text, 128, "0x%8.8" PRIx32
				"\t0x%8.8" PRIx32
				"\tSRS%s\tSP%s, #%d",
				address, opcode,
				mode, wback,
				(unsigned)(opcode & 0x1f));
			break;
		case 0x08100000:
			snprintf(instruction->text, 128, "0x%8.8" PRIx32
				"\t0x%8.8" PRIx32
				"\tRFE%s\tr%d%s",
				address, opcode,
				mode,
				(unsigned)((opcode >> 16) & 0xf), wback);
			break;
		default:
			return evaluate_unknown(opcode, address, instruction);
	}
	return ERROR_OK;
}

static int evaluate_swi(uint32_t opcode,
			uint32_t address, struct arm_instruction *instruction)
{
	instruction->type = ARM_SWI;

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tSVC %#6.6" PRIx32,
			address, opcode, (opcode & 0xffffff));

	return ERROR_OK;
}

static int evaluate_blx_imm(uint32_t opcode,
			    uint32_t address, struct arm_instruction *instruction)
{
	int offset;
	uint32_t immediate;
	uint32_t target_address;

	instruction->type = ARM_BLX;
	immediate = opcode & 0x00ffffff;

	/* sign extend 24-bit immediate */
	if (immediate & 0x00800000)
		offset = 0xff000000 | immediate;
	else
		offset = immediate;

	/* shift two bits left */
	offset <<= 2;

	/* odd/event halfword */
	if (opcode & 0x01000000)
		offset |= 0x2;

	target_address = address + 8 + offset;

	snprintf(instruction->text,
			128,
			"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tBLX 0x%8.8" PRIx32 "",
			address,
			opcode,
			target_address);

	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;

	return ERROR_OK;
}

static int evaluate_b_bl(uint32_t opcode,
			 uint32_t address, struct arm_instruction *instruction)
{
	uint8_t L;
	uint32_t immediate;
	int offset;
	uint32_t target_address;

	immediate = opcode & 0x00ffffff;
	L = (opcode & 0x01000000) >> 24;

	/* sign extend 24-bit immediate */
	if (immediate & 0x00800000)
		offset = 0xff000000 | immediate;
	else
		offset = immediate;

	/* shift two bits left */
	offset <<= 2;

	target_address = address + 8 + offset;

	if (L)
		instruction->type = ARM_BL;
	else
		instruction->type = ARM_B;

	snprintf(instruction->text,
			128,
			"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tB%s%s 0x%8.8" PRIx32,
			address,
			opcode,
			(L) ? "L" : "",
			COND(opcode),
			target_address);

	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;

	return ERROR_OK;
}

/* Coprocessor load/store and double register transfers
 * both normal and extended instruction space (condition field b1111) */
static int evaluate_ldc_stc_mcrr_mrrc(uint32_t opcode,
				      uint32_t address, struct arm_instruction *instruction)
{
	uint8_t cp_num = (opcode & 0xf00) >> 8;

	/* MCRR or MRRC */
	if (((opcode & 0x0ff00000) == 0x0c400000) || ((opcode & 0x0ff00000) == 0x0c500000)) {
		uint8_t cp_opcode, Rd, Rn, CRm;
		char *mnemonic;

		cp_opcode = (opcode & 0xf0) >> 4;
		Rd = (opcode & 0xf000) >> 12;
		Rn = (opcode & 0xf0000) >> 16;
		CRm = (opcode & 0xf);

		/* MCRR */
		if ((opcode & 0x0ff00000) == 0x0c400000) {
			instruction->type = ARM_MCRR;
			mnemonic = "MCRR";
		} else if ((opcode & 0x0ff00000) == 0x0c500000) {
			/* MRRC */
			instruction->type = ARM_MRRC;
			mnemonic = "MRRC";
		} else {
			LOG_ERROR("Unknown instruction");
			return ERROR_FAIL;
		}

		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
				"\t%s%s%s p%i, %x, r%i, r%i, c%i",
				address, opcode, mnemonic,
				((opcode & 0xf0000000) == 0xf0000000)
				? "2" : COND(opcode),
				COND(opcode), cp_num, cp_opcode, Rd, Rn, CRm);
	} else {/* LDC or STC */
		uint8_t CRd, Rn, offset;
		uint8_t U;
		char *mnemonic;
		char addressing_mode[32];

		CRd = (opcode & 0xf000) >> 12;
		Rn = (opcode & 0xf0000) >> 16;
		offset = (opcode & 0xff) << 2;

		/* load/store */
		if (opcode & 0x00100000) {
			instruction->type = ARM_LDC;
			mnemonic = "LDC";
		} else {
			instruction->type = ARM_STC;
			mnemonic = "STC";
		}

		U = (opcode & 0x00800000) >> 23;

		/* addressing modes */
		if ((opcode & 0x01200000) == 0x01000000)/* offset */
			snprintf(addressing_mode, 32, "[r%i, #%s%d]",
					Rn, U ? "" : "-", offset);
		else if ((opcode & 0x01200000) == 0x01200000)	/* pre-indexed */
			snprintf(addressing_mode, 32, "[r%i, #%s%d]!",
					Rn, U ? "" : "-", offset);
		else if ((opcode & 0x01200000) == 0x00200000)	/* post-indexed */
			snprintf(addressing_mode, 32, "[r%i], #%s%d",
					Rn, U ? "" : "-", offset);
		else if ((opcode & 0x01200000) == 0x00000000)	/* unindexed */
			snprintf(addressing_mode, 32, "[r%i], {%d}",
					Rn, offset >> 2);

		snprintf(instruction->text, 128, "0x%8.8" PRIx32
				"\t0x%8.8" PRIx32
				"\t%s%s%s p%i, c%i, %s",
				address, opcode, mnemonic,
				((opcode & 0xf0000000) == 0xf0000000)
				? "2" : COND(opcode),
				(opcode & (1 << 22)) ? "L" : "",
				cp_num, CRd, addressing_mode);
	}

	return ERROR_OK;
}

/* Coprocessor data processing instructions
 * Coprocessor register transfer instructions
 * both normal and extended instruction space (condition field b1111) */
static int evaluate_cdp_mcr_mrc(uint32_t opcode,
				uint32_t address, struct arm_instruction *instruction)
{
	const char *cond;
	char *mnemonic;
	uint8_t cp_num, opcode_1, CRd_Rd, CRn, CRm, opcode_2;

	cond = ((opcode & 0xf0000000) == 0xf0000000) ? "2" : COND(opcode);
	cp_num = (opcode & 0xf00) >> 8;
	CRd_Rd = (opcode & 0xf000) >> 12;
	CRn = (opcode & 0xf0000) >> 16;
	CRm = (opcode & 0xf);
	opcode_2 = (opcode & 0xe0) >> 5;

	/* CDP or MRC/MCR */
	if (opcode & 0x00000010) {	/* bit 4 set -> MRC/MCR */
		if (opcode & 0x00100000) {	/* bit 20 set -> MRC */
			instruction->type = ARM_MRC;
			mnemonic = "MRC";
		} else {/* bit 20 not set -> MCR */
			instruction->type = ARM_MCR;
			mnemonic = "MCR";
		}

		opcode_1 = (opcode & 0x00e00000) >> 21;

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s p%i, 0x%2.2x, r%i, c%i, c%i, 0x%2.2x",
				address,
				opcode,
				mnemonic,
				cond,
				cp_num,
				opcode_1,
				CRd_Rd,
				CRn,
				CRm,
				opcode_2);
	} else {/* bit 4 not set -> CDP */
		instruction->type = ARM_CDP;
		mnemonic = "CDP";

		opcode_1 = (opcode & 0x00f00000) >> 20;

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s p%i, 0x%2.2x, c%i, c%i, c%i, 0x%2.2x",
				address,
				opcode,
				mnemonic,
				cond,
				cp_num,
				opcode_1,
				CRd_Rd,
				CRn,
				CRm,
				opcode_2);
	}

	return ERROR_OK;
}

/* Load/store instructions */
static int evaluate_load_store(uint32_t opcode,
			       uint32_t address, struct arm_instruction *instruction)
{
	uint8_t I, P, U, B, W, L;
	uint8_t Rn, Rd;
	char *operation;/* "LDR" or "STR" */
	char *suffix;	/* "", "B", "T", "BT" */
	char offset[32];

	/* examine flags */
	I = (opcode & 0x02000000) >> 25;
	P = (opcode & 0x01000000) >> 24;
	U = (opcode & 0x00800000) >> 23;
	B = (opcode & 0x00400000) >> 22;
	W = (opcode & 0x00200000) >> 21;
	L = (opcode & 0x00100000) >> 20;

	/* target register */
	Rd = (opcode & 0xf000) >> 12;

	/* base register */
	Rn = (opcode & 0xf0000) >> 16;

	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.U = U;

	/* determine operation */
	if (L)
		operation = "LDR";
	else
		operation = "STR";

	/* determine instruction type and suffix */
	if (B) {
		if ((P == 0) && (W == 1)) {
			if (L)
				instruction->type = ARM_LDRBT;
			else
				instruction->type = ARM_STRBT;
			suffix = "BT";
		} else {
			if (L)
				instruction->type = ARM_LDRB;
			else
				instruction->type = ARM_STRB;
			suffix = "B";
		}
	} else {
		if ((P == 0) && (W == 1)) {
			if (L)
				instruction->type = ARM_LDRT;
			else
				instruction->type = ARM_STRT;
			suffix = "T";
		} else {
			if (L)
				instruction->type = ARM_LDR;
			else
				instruction->type = ARM_STR;
			suffix = "";
		}
	}

	if (!I) {	/* #+-<offset_12> */
		uint32_t offset_12 = (opcode & 0xfff);
		if (offset_12)
			snprintf(offset, 32, ", #%s0x%" PRIx32 "", (U) ? "" : "-", offset_12);
		else
			snprintf(offset, 32, "%s", "");

		instruction->info.load_store.offset_mode = 0;
		instruction->info.load_store.offset.offset = offset_12;
	} else {/* either +-<Rm> or +-<Rm>, <shift>, #<shift_imm> */
		uint8_t shift_imm, shift;
		uint8_t Rm;

		shift_imm = (opcode & 0xf80) >> 7;
		shift = (opcode & 0x60) >> 5;
		Rm = (opcode & 0xf);

		/* LSR encodes a shift by 32 bit as 0x0 */
		if ((shift == 0x1) && (shift_imm == 0x0))
			shift_imm = 0x20;

		/* ASR encodes a shift by 32 bit as 0x0 */
		if ((shift == 0x2) && (shift_imm == 0x0))
			shift_imm = 0x20;

		/* ROR by 32 bit is actually a RRX */
		if ((shift == 0x3) && (shift_imm == 0x0))
			shift = 0x4;

		instruction->info.load_store.offset_mode = 1;
		instruction->info.load_store.offset.reg.Rm = Rm;
		instruction->info.load_store.offset.reg.shift = shift;
		instruction->info.load_store.offset.reg.shift_imm = shift_imm;

		if ((shift_imm == 0x0) && (shift == 0x0))	/* +-<Rm> */
			snprintf(offset, 32, ", %sr%i", (U) ? "" : "-", Rm);
		else {	/* +-<Rm>, <Shift>, #<shift_imm> */
			switch (shift) {
				case 0x0:		/* LSL */
					snprintf(offset, 32, ", %sr%i, LSL #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x1:		/* LSR */
					snprintf(offset, 32, ", %sr%i, LSR #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x2:		/* ASR */
					snprintf(offset, 32, ", %sr%i, ASR #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x3:		/* ROR */
					snprintf(offset, 32, ", %sr%i, ROR #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x4:		/* RRX */
					snprintf(offset, 32, ", %sr%i, RRX", (U) ? "" : "-", Rm);
					break;
			}
		}
	}

	if (P == 1) {
		if (W == 0) {	/* offset */
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, [r%i%s]",
					address,
					opcode,
					operation,
					COND(opcode),
					suffix,
					Rd,
					Rn,
					offset);

			instruction->info.load_store.index_mode = 0;
		} else {/* pre-indexed */
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, [r%i%s]!",
					address,
					opcode,
					operation,
					COND(opcode),
					suffix,
					Rd,
					Rn,
					offset);

			instruction->info.load_store.index_mode = 1;
		}
	} else {/* post-indexed */
		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, [r%i]%s",
				address,
				opcode,
				operation,
				COND(opcode),
				suffix,
				Rd,
				Rn,
				offset);

		instruction->info.load_store.index_mode = 2;
	}

	return ERROR_OK;
}

static int evaluate_extend(uint32_t opcode, uint32_t address, char *cp)
{
	unsigned rm = (opcode >> 0) & 0xf;
	unsigned rd = (opcode >> 12) & 0xf;
	unsigned rn = (opcode >> 16) & 0xf;
	char *type, *rot;

	switch ((opcode >> 24) & 0x3) {
		case 0:
			type = "B16";
			break;
		case 1:
			sprintf(cp, "UNDEFINED");
			return ARM_UNDEFINED_INSTRUCTION;
		case 2:
			type = "B";
			break;
		default:
			type = "H";
			break;
	}

	switch ((opcode >> 10) & 0x3) {
		case 0:
			rot = "";
			break;
		case 1:
			rot = ", ROR #8";
			break;
		case 2:
			rot = ", ROR #16";
			break;
		default:
			rot = ", ROR #24";
			break;
	}

	if (rn == 0xf) {
		sprintf(cp, "%cXT%s%s\tr%d, r%d%s",
				(opcode & (1 << 22)) ? 'U' : 'S',
				type, COND(opcode),
				rd, rm, rot);
		return ARM_MOV;
	} else {
		sprintf(cp, "%cXTA%s%s\tr%d, r%d, r%d%s",
				(opcode & (1 << 22)) ? 'U' : 'S',
				type, COND(opcode),
				rd, rn, rm, rot);
		return ARM_ADD;
	}
}

static int evaluate_p_add_sub(uint32_t opcode, uint32_t address, char *cp)
{
	char *prefix;
	char *op;
	int type;

	switch ((opcode >> 20) & 0x7) {
		case 1:
			prefix = "S";
			break;
		case 2:
			prefix = "Q";
			break;
		case 3:
			prefix = "SH";
			break;
		case 5:
			prefix = "U";
			break;
		case 6:
			prefix = "UQ";
			break;
		case 7:
			prefix = "UH";
			break;
		default:
			goto undef;
	}

	switch ((opcode >> 5) & 0x7) {
		case 0:
			op = "ADD16";
			type = ARM_ADD;
			break;
		case 1:
			op = "ADDSUBX";
			type = ARM_ADD;
			break;
		case 2:
			op = "SUBADDX";
			type = ARM_SUB;
			break;
		case 3:
			op = "SUB16";
			type = ARM_SUB;
			break;
		case 4:
			op = "ADD8";
			type = ARM_ADD;
			break;
		case 7:
			op = "SUB8";
			type = ARM_SUB;
			break;
		default:
			goto undef;
	}

	sprintf(cp, "%s%s%s\tr%d, r%d, r%d", prefix, op, COND(opcode),
			(int) (opcode >> 12) & 0xf,
			(int) (opcode >> 16) & 0xf,
			(int) (opcode >> 0) & 0xf);
	return type;

undef:
	/* these opcodes might be used someday */
	sprintf(cp, "UNDEFINED");
	return ARM_UNDEFINED_INSTRUCTION;
}

/* ARMv6 and later support "media" instructions (includes SIMD) */
static int evaluate_media(uint32_t opcode, uint32_t address,
			  struct arm_instruction *instruction)
{
	char *cp = instruction->text;
	char *mnemonic = NULL;

	sprintf(cp,
			"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t",
			address, opcode);
	cp = strchr(cp, 0);

	/* parallel add/subtract */
	if ((opcode & 0x01800000) == 0x00000000) {
		instruction->type = evaluate_p_add_sub(opcode, address, cp);
		return ERROR_OK;
	}

	/* halfword pack */
	if ((opcode & 0x01f00020) == 0x00800000) {
		char *type, *shift;
		unsigned imm = (unsigned) (opcode >> 7) & 0x1f;

		if (opcode & (1 << 6)) {
			type = "TB";
			shift = "ASR";
			if (imm == 0)
				imm = 32;
		} else {
			type = "BT";
			shift = "LSL";
		}
		sprintf(cp, "PKH%s%s\tr%d, r%d, r%d, %s #%d",
				type, COND(opcode),
				(int) (opcode >> 12) & 0xf,
				(int) (opcode >> 16) & 0xf,
				(int) (opcode >> 0) & 0xf,
				shift, imm);
		return ERROR_OK;
	}

	/* word saturate */
	if ((opcode & 0x01a00020) == 0x00a00000) {
		char *shift;
		unsigned imm = (unsigned) (opcode >> 7) & 0x1f;

		if (opcode & (1 << 6)) {
			shift = "ASR";
			if (imm == 0)
				imm = 32;
		} else
			shift = "LSL";

		sprintf(cp, "%cSAT%s\tr%d, #%d, r%d, %s #%d",
				(opcode & (1 << 22)) ? 'U' : 'S',
				COND(opcode),
				(int) (opcode >> 12) & 0xf,
				(int) (opcode >> 16) & 0x1f,
				(int) (opcode >> 0) & 0xf,
				shift, imm);
		return ERROR_OK;
	}

	/* sign extension */
	if ((opcode & 0x018000f0) == 0x00800070) {
		instruction->type = evaluate_extend(opcode, address, cp);
		return ERROR_OK;
	}

	/* multiplies */
	if ((opcode & 0x01f00080) == 0x01000000) {
		unsigned rn = (opcode >> 12) & 0xf;

		if (rn != 0xf)
			sprintf(cp, "SML%cD%s%s\tr%d, r%d, r%d, r%d",
					(opcode & (1 << 6)) ? 'S' : 'A',
					(opcode & (1 << 5)) ? "X" : "",
					COND(opcode),
					(int) (opcode >> 16) & 0xf,
					(int) (opcode >> 0) & 0xf,
					(int) (opcode >> 8) & 0xf,
					rn);
		else
			sprintf(cp, "SMU%cD%s%s\tr%d, r%d, r%d",
					(opcode & (1 << 6)) ? 'S' : 'A',
					(opcode & (1 << 5)) ? "X" : "",
					COND(opcode),
					(int) (opcode >> 16) & 0xf,
					(int) (opcode >> 0) & 0xf,
					(int) (opcode >> 8) & 0xf);
		return ERROR_OK;
	}
	if ((opcode & 0x01f00000) == 0x01400000) {
		sprintf(cp, "SML%cLD%s%s\tr%d, r%d, r%d, r%d",
				(opcode & (1 << 6)) ? 'S' : 'A',
				(opcode & (1 << 5)) ? "X" : "",
				COND(opcode),
				(int) (opcode >> 12) & 0xf,
				(int) (opcode >> 16) & 0xf,
				(int) (opcode >> 0) & 0xf,
				(int) (opcode >> 8) & 0xf);
		return ERROR_OK;
	}
	if ((opcode & 0x01f00000) == 0x01500000) {
		unsigned rn = (opcode >> 12) & 0xf;

		switch (opcode & 0xc0) {
			case 3:
				if (rn == 0xf)
					goto undef;
			/* FALL THROUGH */
			case 0:
				break;
			default:
				goto undef;
		}

		if (rn != 0xf)
			sprintf(cp, "SMML%c%s%s\tr%d, r%d, r%d, r%d",
					(opcode & (1 << 6)) ? 'S' : 'A',
					(opcode & (1 << 5)) ? "R" : "",
					COND(opcode),
					(int) (opcode >> 16) & 0xf,
					(int) (opcode >> 0) & 0xf,
					(int) (opcode >> 8) & 0xf,
					rn);
		else
			sprintf(cp, "SMMUL%s%s\tr%d, r%d, r%d",
					(opcode & (1 << 5)) ? "R" : "",
					COND(opcode),
					(int) (opcode >> 16) & 0xf,
					(int) (opcode >> 0) & 0xf,
					(int) (opcode >> 8) & 0xf);
		return ERROR_OK;
	}

	/* simple matches against the remaining decode bits */
	switch (opcode & 0x01f000f0) {
		case 0x00a00030:
		case 0x00e00030:
			/* parallel halfword saturate */
			sprintf(cp, "%cSAT16%s\tr%d, #%d, r%d",
				(opcode & (1 << 22)) ? 'U' : 'S',
				COND(opcode),
				(int) (opcode >> 12) & 0xf,
				(int) (opcode >> 16) & 0xf,
				(int) (opcode >> 0) & 0xf);
			return ERROR_OK;
		case 0x00b00030:
			mnemonic = "REV";
			break;
		case 0x00b000b0:
			mnemonic = "REV16";
			break;
		case 0x00f000b0:
			mnemonic = "REVSH";
			break;
		case 0x008000b0:
			/* select bytes */
			sprintf(cp, "SEL%s\tr%d, r%d, r%d", COND(opcode),
				(int) (opcode >> 12) & 0xf,
				(int) (opcode >> 16) & 0xf,
				(int) (opcode >> 0) & 0xf);
			return ERROR_OK;
		case 0x01800010:
			/* unsigned sum of absolute differences */
			if (((opcode >> 12) & 0xf) == 0xf)
				sprintf(cp, "USAD8%s\tr%d, r%d, r%d", COND(opcode),
						(int) (opcode >> 16) & 0xf,
						(int) (opcode >> 0) & 0xf,
						(int) (opcode >> 8) & 0xf);
			else
				sprintf(cp, "USADA8%s\tr%d, r%d, r%d, r%d", COND(opcode),
						(int) (opcode >> 16) & 0xf,
						(int) (opcode >> 0) & 0xf,
						(int) (opcode >> 8) & 0xf,
						(int) (opcode >> 12) & 0xf);
			return ERROR_OK;
	}
	if (mnemonic) {
		unsigned rm = (opcode >> 0) & 0xf;
		unsigned rd = (opcode >> 12) & 0xf;

		sprintf(cp, "%s%s\tr%d, r%d", mnemonic, COND(opcode), rm, rd);
		return ERROR_OK;
	}

undef:
	/* these opcodes might be used someday */
	sprintf(cp, "UNDEFINED");
	return ERROR_OK;
}

/* Miscellaneous load/store instructions */
static int evaluate_misc_load_store(uint32_t opcode,
				    uint32_t address, struct arm_instruction *instruction)
{
	uint8_t P, U, I, W, L, S, H;
	uint8_t Rn, Rd;
	char *operation;/* "LDR" or "STR" */
	char *suffix;	/* "H", "SB", "SH", "D" */
	char offset[32];

	/* examine flags */
	P = (opcode & 0x01000000) >> 24;
	U = (opcode & 0x00800000) >> 23;
	I = (opcode & 0x00400000) >> 22;
	W = (opcode & 0x00200000) >> 21;
	L = (opcode & 0x00100000) >> 20;
	S = (opcode & 0x00000040) >> 6;
	H = (opcode & 0x00000020) >> 5;

	/* target register */
	Rd = (opcode & 0xf000) >> 12;

	/* base register */
	Rn = (opcode & 0xf0000) >> 16;

	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.U = U;

	/* determine instruction type and suffix */
	if (S) {/* signed */
		if (L) {/* load */
			if (H) {
				operation = "LDR";
				instruction->type = ARM_LDRSH;
				suffix = "SH";
			} else {
				operation = "LDR";
				instruction->type = ARM_LDRSB;
				suffix = "SB";
			}
		} else {/* there are no signed stores, so this is used to encode double-register
			 *load/stores */
			suffix = "D";
			if (H) {
				operation = "STR";
				instruction->type = ARM_STRD;
			} else {
				operation = "LDR";
				instruction->type = ARM_LDRD;
			}
		}
	} else {/* unsigned */
		suffix = "H";
		if (L) {/* load */
			operation = "LDR";
			instruction->type = ARM_LDRH;
		} else {/* store */
			operation = "STR";
			instruction->type = ARM_STRH;
		}
	}

	if (I) {/* Immediate offset/index (#+-<offset_8>)*/
		uint32_t offset_8 = ((opcode & 0xf00) >> 4) | (opcode & 0xf);
		snprintf(offset, 32, "#%s0x%" PRIx32 "", (U) ? "" : "-", offset_8);

		instruction->info.load_store.offset_mode = 0;
		instruction->info.load_store.offset.offset = offset_8;
	} else {/* Register offset/index (+-<Rm>) */
		uint8_t Rm;
		Rm = (opcode & 0xf);
		snprintf(offset, 32, "%sr%i", (U) ? "" : "-", Rm);

		instruction->info.load_store.offset_mode = 1;
		instruction->info.load_store.offset.reg.Rm = Rm;
		instruction->info.load_store.offset.reg.shift = 0x0;
		instruction->info.load_store.offset.reg.shift_imm = 0x0;
	}

	if (P == 1) {
		if (W == 0) {	/* offset */
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, [r%i, %s]",
					address,
					opcode,
					operation,
					COND(opcode),
					suffix,
					Rd,
					Rn,
					offset);

			instruction->info.load_store.index_mode = 0;
		} else {/* pre-indexed */
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, [r%i, %s]!",
					address,
					opcode,
					operation,
					COND(opcode),
					suffix,
					Rd,
					Rn,
					offset);

			instruction->info.load_store.index_mode = 1;
		}
	} else {/* post-indexed */
		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, [r%i], %s",
				address,
				opcode,
				operation,
				COND(opcode),
				suffix,
				Rd,
				Rn,
				offset);

		instruction->info.load_store.index_mode = 2;
	}

	return ERROR_OK;
}

/* Load/store multiples instructions */
static int evaluate_ldm_stm(uint32_t opcode,
			    uint32_t address, struct arm_instruction *instruction)
{
	uint8_t P, U, S, W, L, Rn;
	uint32_t register_list;
	char *addressing_mode;
	char *mnemonic;
	char reg_list[69];
	char *reg_list_p;
	int i;
	int first_reg = 1;

	P = (opcode & 0x01000000) >> 24;
	U = (opcode & 0x00800000) >> 23;
	S = (opcode & 0x00400000) >> 22;
	W = (opcode & 0x00200000) >> 21;
	L = (opcode & 0x00100000) >> 20;
	register_list = (opcode & 0xffff);
	Rn = (opcode & 0xf0000) >> 16;

	instruction->info.load_store_multiple.Rn = Rn;
	instruction->info.load_store_multiple.register_list = register_list;
	instruction->info.load_store_multiple.S = S;
	instruction->info.load_store_multiple.W = W;

	if (L) {
		instruction->type = ARM_LDM;
		mnemonic = "LDM";
	} else {
		instruction->type = ARM_STM;
		mnemonic = "STM";
	}

	if (P) {
		if (U) {
			instruction->info.load_store_multiple.addressing_mode = 1;
			addressing_mode = "IB";
		} else {
			instruction->info.load_store_multiple.addressing_mode = 3;
			addressing_mode = "DB";
		}
	} else {
		if (U) {
			instruction->info.load_store_multiple.addressing_mode = 0;
			/* "IA" is the default in UAL syntax */
			addressing_mode = "";
		} else {
			instruction->info.load_store_multiple.addressing_mode = 2;
			addressing_mode = "DA";
		}
	}

	reg_list_p = reg_list;
	for (i = 0; i <= 15; i++) {
		if ((register_list >> i) & 1) {
			if (first_reg) {
				first_reg = 0;
				reg_list_p += snprintf(reg_list_p,
							(reg_list + 69 - reg_list_p),
							"r%i",
							i);
			} else
				reg_list_p += snprintf(reg_list_p,
							(reg_list + 69 - reg_list_p),
							", r%i",
							i);
		}
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
			"\t%s%s%s r%i%s, {%s}%s",
			address, opcode,
			mnemonic, addressing_mode, COND(opcode),
			Rn, (W) ? "!" : "", reg_list, (S) ? "^" : "");

	return ERROR_OK;
}

/* Multiplies, extra load/stores */
static int evaluate_mul_and_extra_ld_st(uint32_t opcode,
					uint32_t address, struct arm_instruction *instruction)
{
	/* Multiply (accumulate) (long) and Swap/swap byte */
	if ((opcode & 0x000000f0) == 0x00000090) {
		/* Multiply (accumulate) */
		if ((opcode & 0x0f800000) == 0x00000000) {
			uint8_t Rm, Rs, Rn, Rd, S;
			Rm = opcode & 0xf;
			Rs = (opcode & 0xf00) >> 8;
			Rn = (opcode & 0xf000) >> 12;
			Rd = (opcode & 0xf0000) >> 16;
			S = (opcode & 0x00100000) >> 20;

			/* examine A bit (accumulate) */
			if (opcode & 0x00200000) {
				instruction->type = ARM_MLA;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tMLA%s%s r%i, r%i, r%i, r%i",
						address,
						opcode,
						COND(opcode),
						(S) ? "S" : "",
						Rd,
						Rm,
						Rs,
						Rn);
			} else {
				instruction->type = ARM_MUL;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tMUL%s%s r%i, r%i, r%i",
						address,
						opcode,
						COND(opcode),
						(S) ? "S" : "",
						Rd,
						Rm,
						Rs);
			}

			return ERROR_OK;
		}

		/* Multiply (accumulate) long */
		if ((opcode & 0x0f800000) == 0x00800000) {
			char *mnemonic = NULL;
			uint8_t Rm, Rs, RdHi, RdLow, S;
			Rm = opcode & 0xf;
			Rs = (opcode & 0xf00) >> 8;
			RdHi = (opcode & 0xf000) >> 12;
			RdLow = (opcode & 0xf0000) >> 16;
			S = (opcode & 0x00100000) >> 20;

			switch ((opcode & 0x00600000) >> 21) {
				case 0x0:
					instruction->type = ARM_UMULL;
					mnemonic = "UMULL";
					break;
				case 0x1:
					instruction->type = ARM_UMLAL;
					mnemonic = "UMLAL";
					break;
				case 0x2:
					instruction->type = ARM_SMULL;
					mnemonic = "SMULL";
					break;
				case 0x3:
					instruction->type = ARM_SMLAL;
					mnemonic = "SMLAL";
					break;
			}

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, r%i, r%i, r%i",
					address,
					opcode,
					mnemonic,
					COND(opcode),
					(S) ? "S" : "",
					RdLow,
					RdHi,
					Rm,
					Rs);

			return ERROR_OK;
		}

		/* Swap/swap byte */
		if ((opcode & 0x0f800000) == 0x01000000) {
			uint8_t Rm, Rd, Rn;
			Rm = opcode & 0xf;
			Rd = (opcode & 0xf000) >> 12;
			Rn = (opcode & 0xf0000) >> 16;

			/* examine B flag */
			instruction->type = (opcode & 0x00400000) ? ARM_SWPB : ARM_SWP;

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s r%i, r%i, [r%i]",
					address,
					opcode,
					(opcode & 0x00400000) ? "SWPB" : "SWP",
					COND(opcode),
					Rd,
					Rm,
					Rn);
			return ERROR_OK;
		}

	}

	return evaluate_misc_load_store(opcode, address, instruction);
}

static int evaluate_mrs_msr(uint32_t opcode,
			    uint32_t address, struct arm_instruction *instruction)
{
	int R = (opcode & 0x00400000) >> 22;
	char *PSR = (R) ? "SPSR" : "CPSR";

	/* Move register to status register (MSR) */
	if (opcode & 0x00200000) {
		instruction->type = ARM_MSR;

		/* immediate variant */
		if (opcode & 0x02000000) {
			uint8_t immediate = (opcode & 0xff);
			uint8_t rotate = (opcode & 0xf00);

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tMSR%s %s_%s%s%s%s, 0x%8.8" PRIx32,
					address,
					opcode,
					COND(opcode),
					PSR,
					(opcode & 0x10000) ? "c" : "",
					(opcode & 0x20000) ? "x" : "",
					(opcode & 0x40000) ? "s" : "",
					(opcode & 0x80000) ? "f" : "",
					ror(immediate, (rotate * 2))
					);
		} else {/* register variant */
			uint8_t Rm = opcode & 0xf;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tMSR%s %s_%s%s%s%s, r%i",
					address,
					opcode,
					COND(opcode),
					PSR,
					(opcode & 0x10000) ? "c" : "",
					(opcode & 0x20000) ? "x" : "",
					(opcode & 0x40000) ? "s" : "",
					(opcode & 0x80000) ? "f" : "",
					Rm
					);
		}

	} else {/* Move status register to register (MRS) */
		uint8_t Rd;

		instruction->type = ARM_MRS;
		Rd = (opcode & 0x0000f000) >> 12;

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tMRS%s r%i, %s",
				address,
				opcode,
				COND(opcode),
				Rd,
				PSR);
	}

	return ERROR_OK;
}

/* Miscellaneous instructions */
static int evaluate_misc_instr(uint32_t opcode,
			       uint32_t address, struct arm_instruction *instruction)
{
	/* MRS/MSR */
	if ((opcode & 0x000000f0) == 0x00000000)
		evaluate_mrs_msr(opcode, address, instruction);

	/* BX */
	if ((opcode & 0x006000f0) == 0x00200010) {
		uint8_t Rm;
		instruction->type = ARM_BX;
		Rm = opcode & 0xf;

		snprintf(instruction->text, 128, "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tBX%s r%i",
				address, opcode, COND(opcode), Rm);

		instruction->info.b_bl_bx_blx.reg_operand = Rm;
		instruction->info.b_bl_bx_blx.target_address = -1;
	}

	/* BXJ - "Jazelle" support (ARMv5-J) */
	if ((opcode & 0x006000f0) == 0x00200020) {
		uint8_t Rm;
		instruction->type = ARM_BX;
		Rm = opcode & 0xf;

		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tBXJ%s r%i",
				address, opcode, COND(opcode), Rm);

		instruction->info.b_bl_bx_blx.reg_operand = Rm;
		instruction->info.b_bl_bx_blx.target_address = -1;
	}

	/* CLZ */
	if ((opcode & 0x006000f0) == 0x00600010) {
		uint8_t Rm, Rd;
		instruction->type = ARM_CLZ;
		Rm = opcode & 0xf;
		Rd = (opcode & 0xf000) >> 12;

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tCLZ%s r%i, r%i",
				address,
				opcode,
				COND(opcode),
				Rd,
				Rm);
	}

	/* BLX(2) */
	if ((opcode & 0x006000f0) == 0x00200030) {
		uint8_t Rm;
		instruction->type = ARM_BLX;
		Rm = opcode & 0xf;

		snprintf(instruction->text, 128, "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tBLX%s r%i",
				address, opcode, COND(opcode), Rm);

		instruction->info.b_bl_bx_blx.reg_operand = Rm;
		instruction->info.b_bl_bx_blx.target_address = -1;
	}

	/* Enhanced DSP add/subtracts */
	if ((opcode & 0x0000000f0) == 0x00000050) {
		uint8_t Rm, Rd, Rn;
		char *mnemonic = NULL;
		Rm = opcode & 0xf;
		Rd = (opcode & 0xf000) >> 12;
		Rn = (opcode & 0xf0000) >> 16;

		switch ((opcode & 0x00600000) >> 21) {
			case 0x0:
				instruction->type = ARM_QADD;
				mnemonic = "QADD";
				break;
			case 0x1:
				instruction->type = ARM_QSUB;
				mnemonic = "QSUB";
				break;
			case 0x2:
				instruction->type = ARM_QDADD;
				mnemonic = "QDADD";
				break;
			case 0x3:
				instruction->type = ARM_QDSUB;
				mnemonic = "QDSUB";
				break;
		}

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s r%i, r%i, r%i",
				address,
				opcode,
				mnemonic,
				COND(opcode),
				Rd,
				Rm,
				Rn);
	}

	/* exception return */
	if ((opcode & 0x0000000f0) == 0x00000060) {
		if (((opcode & 0x600000) >> 21) == 3)
			instruction->type = ARM_ERET;
		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tERET",
				address,
				opcode);
	}

	/* exception generate instructions */
	if ((opcode & 0x0000000f0) == 0x00000070) {
		uint32_t immediate = 0;
		char *mnemonic = NULL;

		switch ((opcode & 0x600000) >> 21) {
			case 0x1:
				instruction->type = ARM_BKPT;
				mnemonic = "BRKT";
				immediate = ((opcode & 0x000fff00) >> 4) | (opcode & 0xf);
				break;
			case 0x2:
				instruction->type = ARM_HVC;
				mnemonic = "HVC";
				immediate = ((opcode & 0x000fff00) >> 4) | (opcode & 0xf);
				break;
			case 0x3:
				instruction->type = ARM_SMC;
				mnemonic = "SMC";
				immediate = (opcode & 0xf);
				break;
		}

		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s 0x%4.4" PRIx32 "",
				address,
				opcode,
				mnemonic,
				immediate);
	}

	/* Enhanced DSP multiplies */
	if ((opcode & 0x000000090) == 0x00000080) {
		int x = (opcode & 0x20) >> 5;
		int y = (opcode & 0x40) >> 6;

		/* SMLA < x><y> */
		if ((opcode & 0x00600000) == 0x00000000) {
			uint8_t Rd, Rm, Rs, Rn;
			instruction->type = ARM_SMLAxy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;
			Rn = (opcode & 0xf000) >> 12;

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tSMLA%s%s%s r%i, r%i, r%i, r%i",
					address,
					opcode,
					(x) ? "T" : "B",
					(y) ? "T" : "B",
					COND(opcode),
					Rd,
					Rm,
					Rs,
					Rn);
		}

		/* SMLAL < x><y> */
		if ((opcode & 0x00600000) == 0x00400000) {
			uint8_t RdLow, RdHi, Rm, Rs;
			instruction->type = ARM_SMLAxy;
			RdHi = (opcode & 0xf0000) >> 16;
			RdLow = (opcode & 0xf000) >> 12;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tSMLA%s%s%s r%i, r%i, r%i, r%i",
					address,
					opcode,
					(x) ? "T" : "B",
					(y) ? "T" : "B",
					COND(opcode),
					RdLow,
					RdHi,
					Rm,
					Rs);
		}

		/* SMLAW < y> */
		if (((opcode & 0x00600000) == 0x00200000) && (x == 0)) {
			uint8_t Rd, Rm, Rs, Rn;
			instruction->type = ARM_SMLAWy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;
			Rn = (opcode & 0xf000) >> 12;

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tSMLAW%s%s r%i, r%i, r%i, r%i",
					address,
					opcode,
					(y) ? "T" : "B",
					COND(opcode),
					Rd,
					Rm,
					Rs,
					Rn);
		}

		/* SMUL < x><y> */
		if ((opcode & 0x00600000) == 0x00600000) {
			uint8_t Rd, Rm, Rs;
			instruction->type = ARM_SMULxy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tSMULW%s%s%s r%i, r%i, r%i",
					address,
					opcode,
					(x) ? "T" : "B",
					(y) ? "T" : "B",
					COND(opcode),
					Rd,
					Rm,
					Rs);
		}

		/* SMULW < y> */
		if (((opcode & 0x00600000) == 0x00200000) && (x == 1)) {
			uint8_t Rd, Rm, Rs;
			instruction->type = ARM_SMULWy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;

			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tSMULW%s%s r%i, r%i, r%i",
					address,
					opcode,
					(y) ? "T" : "B",
					COND(opcode),
					Rd,
					Rm,
					Rs);
		}
	}

	return ERROR_OK;
}

static int evaluate_mov_imm(uint32_t opcode,
			      uint32_t address, struct arm_instruction *instruction)
{
	uint16_t immediate;
	uint8_t Rd;
	bool T;

	Rd = (opcode & 0xf000) >> 12;
	T = opcode & 0x00400000;
	immediate = (opcode & 0xf0000) >> 4 | (opcode & 0xfff);

	instruction->type = ARM_MOV;
	instruction->info.data_proc.Rd = Rd;

	snprintf(instruction->text,
		 128,
		 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tMOV%s%s r%i, #0x%" PRIx16,
		 address,
		 opcode,
		 T ? "T" : "W",
		 COND(opcode),
		 Rd,
		 immediate);

	return ERROR_OK;
}

static int evaluate_data_proc(uint32_t opcode,
			      uint32_t address, struct arm_instruction *instruction)
{
	uint8_t I, op, S, Rn, Rd;
	char *mnemonic = NULL;
	char shifter_operand[32];

	I = (opcode & 0x02000000) >> 25;
	op = (opcode & 0x01e00000) >> 21;
	S = (opcode & 0x00100000) >> 20;

	Rd = (opcode & 0xf000) >> 12;
	Rn = (opcode & 0xf0000) >> 16;

	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rn;
	instruction->info.data_proc.S = S;

	switch (op) {
		case 0x0:
			instruction->type = ARM_AND;
			mnemonic = "AND";
			break;
		case 0x1:
			instruction->type = ARM_EOR;
			mnemonic = "EOR";
			break;
		case 0x2:
			instruction->type = ARM_SUB;
			mnemonic = "SUB";
			break;
		case 0x3:
			instruction->type = ARM_RSB;
			mnemonic = "RSB";
			break;
		case 0x4:
			instruction->type = ARM_ADD;
			mnemonic = "ADD";
			break;
		case 0x5:
			instruction->type = ARM_ADC;
			mnemonic = "ADC";
			break;
		case 0x6:
			instruction->type = ARM_SBC;
			mnemonic = "SBC";
			break;
		case 0x7:
			instruction->type = ARM_RSC;
			mnemonic = "RSC";
			break;
		case 0x8:
			instruction->type = ARM_TST;
			mnemonic = "TST";
			break;
		case 0x9:
			instruction->type = ARM_TEQ;
			mnemonic = "TEQ";
			break;
		case 0xa:
			instruction->type = ARM_CMP;
			mnemonic = "CMP";
			break;
		case 0xb:
			instruction->type = ARM_CMN;
			mnemonic = "CMN";
			break;
		case 0xc:
			instruction->type = ARM_ORR;
			mnemonic = "ORR";
			break;
		case 0xd:
			instruction->type = ARM_MOV;
			mnemonic = "MOV";
			break;
		case 0xe:
			instruction->type = ARM_BIC;
			mnemonic = "BIC";
			break;
		case 0xf:
			instruction->type = ARM_MVN;
			mnemonic = "MVN";
			break;
	}

	if (I) {/* immediate shifter operand (#<immediate>)*/
		uint8_t immed_8 = opcode & 0xff;
		uint8_t rotate_imm = (opcode & 0xf00) >> 8;
		uint32_t immediate;

		immediate = ror(immed_8, rotate_imm * 2);

		snprintf(shifter_operand, 32, "#0x%" PRIx32 "", immediate);

		instruction->info.data_proc.variant = 0;
		instruction->info.data_proc.shifter_operand.immediate.immediate = immediate;
	} else {/* register-based shifter operand */
		uint8_t shift, Rm;
		shift = (opcode & 0x60) >> 5;
		Rm = (opcode & 0xf);

		if ((opcode & 0x10) != 0x10) {	/* Immediate shifts ("<Rm>" or "<Rm>, <shift>
						 *#<shift_immediate>") */
			uint8_t shift_imm;
			shift_imm = (opcode & 0xf80) >> 7;

			instruction->info.data_proc.variant = 1;
			instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm;
			instruction->info.data_proc.shifter_operand.immediate_shift.shift_imm =
				shift_imm;
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = shift;

			/* LSR encodes a shift by 32 bit as 0x0 */
			if ((shift == 0x1) && (shift_imm == 0x0))
				shift_imm = 0x20;

			/* ASR encodes a shift by 32 bit as 0x0 */
			if ((shift == 0x2) && (shift_imm == 0x0))
				shift_imm = 0x20;

			/* ROR by 32 bit is actually a RRX */
			if ((shift == 0x3) && (shift_imm == 0x0))
				shift = 0x4;

			if ((shift_imm == 0x0) && (shift == 0x0))
				snprintf(shifter_operand, 32, "r%i", Rm);
			else {
				if (shift == 0x0)	/* LSL */
					snprintf(shifter_operand,
							32,
							"r%i, LSL #0x%x",
							Rm,
							shift_imm);
				else if (shift == 0x1)	/* LSR */
					snprintf(shifter_operand,
							32,
							"r%i, LSR #0x%x",
							Rm,
							shift_imm);
				else if (shift == 0x2)	/* ASR */
					snprintf(shifter_operand,
							32,
							"r%i, ASR #0x%x",
							Rm,
							shift_imm);
				else if (shift == 0x3)	/* ROR */
					snprintf(shifter_operand,
							32,
							"r%i, ROR #0x%x",
							Rm,
							shift_imm);
				else if (shift == 0x4)	/* RRX */
					snprintf(shifter_operand, 32, "r%i, RRX", Rm);
			}
		} else {/* Register shifts ("<Rm>, <shift> <Rs>") */
			uint8_t Rs = (opcode & 0xf00) >> 8;

			instruction->info.data_proc.variant = 2;
			instruction->info.data_proc.shifter_operand.register_shift.Rm = Rm;
			instruction->info.data_proc.shifter_operand.register_shift.Rs = Rs;
			instruction->info.data_proc.shifter_operand.register_shift.shift = shift;

			if (shift == 0x0)	/* LSL */
				snprintf(shifter_operand, 32, "r%i, LSL r%i", Rm, Rs);
			else if (shift == 0x1)	/* LSR */
				snprintf(shifter_operand, 32, "r%i, LSR r%i", Rm, Rs);
			else if (shift == 0x2)	/* ASR */
				snprintf(shifter_operand, 32, "r%i, ASR r%i", Rm, Rs);
			else if (shift == 0x3)	/* ROR */
				snprintf(shifter_operand, 32, "r%i, ROR r%i", Rm, Rs);
		}
	}

	if ((op < 0x8) || (op == 0xc) || (op == 0xe)) {	/* <opcode3>{<cond>}{S} <Rd>, <Rn>,
							 *<shifter_operand> */
		snprintf(instruction->text,
				128,
				"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, r%i, %s",
				address,
				opcode,
				mnemonic,
				COND(opcode),
				(S) ? "S" : "",
				Rd,
				Rn,
				shifter_operand);
	} else if ((op == 0xd) || (op == 0xf)) {	/* <opcode1>{<cond>}{S} <Rd>,
							 *<shifter_operand> */
		if (opcode == 0xe1a00000)	/* print MOV r0,r0 as NOP */
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tNOP",
					address,
					opcode);
		else
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s%s r%i, %s",
					address,
					opcode,
					mnemonic,
					COND(opcode),
					(S) ? "S" : "",
					Rd,
					shifter_operand);
	} else {/* <opcode2>{<cond>} <Rn>, <shifter_operand> */
		snprintf(instruction->text, 128, "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\t%s%s r%i, %s",
				address, opcode, mnemonic, COND(opcode),
				Rn, shifter_operand);
	}

	return ERROR_OK;
}

int arm_evaluate_opcode(uint32_t opcode, uint32_t address,
			struct arm_instruction *instruction)
{
	/* clear fields, to avoid confusion */
	memset(instruction, 0, sizeof(struct arm_instruction));
	instruction->opcode = opcode;
	instruction->instruction_size = 4;

	/* catch opcodes with condition field [31:28] = b1111 */
	if ((opcode & 0xf0000000) == 0xf0000000) {
		/* Undefined instruction (or ARMv5E cache preload PLD) */
		if ((opcode & 0x08000000) == 0x00000000)
			return evaluate_pld(opcode, address, instruction);

		/* Undefined instruction (or ARMv6+ SRS/RFE) */
		if ((opcode & 0x0e000000) == 0x08000000)
			return evaluate_srs(opcode, address, instruction);

		/* Branch and branch with link and change to Thumb */
		if ((opcode & 0x0e000000) == 0x0a000000)
			return evaluate_blx_imm(opcode, address, instruction);

		/* Extended coprocessor opcode space (ARMv5 and higher)
		 * Coprocessor load/store and double register transfers */
		if ((opcode & 0x0e000000) == 0x0c000000)
			return evaluate_ldc_stc_mcrr_mrrc(opcode, address, instruction);

		/* Coprocessor data processing */
		if ((opcode & 0x0f000100) == 0x0c000000)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);

		/* Coprocessor register transfers */
		if ((opcode & 0x0f000010) == 0x0c000010)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);

		/* Undefined instruction */
		if ((opcode & 0x0f000000) == 0x0f000000) {
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_OK;
		}
	}

	/* catch opcodes with [27:25] = b000 */
	if ((opcode & 0x0e000000) == 0x00000000) {
		/* Multiplies, extra load/stores */
		if ((opcode & 0x00000090) == 0x00000090)
			return evaluate_mul_and_extra_ld_st(opcode, address, instruction);

		/* Miscellaneous instructions */
		if ((opcode & 0x0f900000) == 0x01000000)
			return evaluate_misc_instr(opcode, address, instruction);

		return evaluate_data_proc(opcode, address, instruction);
	}

	/* catch opcodes with [27:25] = b001 */
	if ((opcode & 0x0e000000) == 0x02000000) {
		/* 16-bit immediate load */
		if ((opcode & 0x0fb00000) == 0x03000000)
			return evaluate_mov_imm(opcode, address, instruction);

		/* Move immediate to status register */
		if ((opcode & 0x0fb00000) == 0x03200000)
			return evaluate_mrs_msr(opcode, address, instruction);

		return evaluate_data_proc(opcode, address, instruction);

	}

	/* catch opcodes with [27:25] = b010 */
	if ((opcode & 0x0e000000) == 0x04000000) {
		/* Load/store immediate offset */
		return evaluate_load_store(opcode, address, instruction);
	}

	/* catch opcodes with [27:25] = b011 */
	if ((opcode & 0x0e000000) == 0x06000000) {
		/* Load/store register offset */
		if ((opcode & 0x00000010) == 0x00000000)
			return evaluate_load_store(opcode, address, instruction);

		/* Architecturally Undefined instruction
		 * ... don't expect these to ever be used
		 */
		if ((opcode & 0x07f000f0) == 0x07f000f0) {
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEF",
					address, opcode);
			return ERROR_OK;
		}

		/* "media" instructions */
		return evaluate_media(opcode, address, instruction);
	}

	/* catch opcodes with [27:25] = b100 */
	if ((opcode & 0x0e000000) == 0x08000000) {
		/* Load/store multiple */
		return evaluate_ldm_stm(opcode, address, instruction);
	}

	/* catch opcodes with [27:25] = b101 */
	if ((opcode & 0x0e000000) == 0x0a000000) {
		/* Branch and branch with link */
		return evaluate_b_bl(opcode, address, instruction);
	}

	/* catch opcodes with [27:25] = b110 */
	if ((opcode & 0x0e000000) == 0x0c000000) {
		/* Coprocessor load/store and double register transfers */
		return evaluate_ldc_stc_mcrr_mrrc(opcode, address, instruction);
	}

	/* catch opcodes with [27:25] = b111 */
	if ((opcode & 0x0e000000) == 0x0e000000) {
		/* Software interrupt */
		if ((opcode & 0x0f000000) == 0x0f000000)
			return evaluate_swi(opcode, address, instruction);

		/* Coprocessor data processing */
		if ((opcode & 0x0f000010) == 0x0e000000)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);

		/* Coprocessor register transfers */
		if ((opcode & 0x0f000010) == 0x0e000010)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);
	}

	LOG_ERROR("ARM: should never reach this point (opcode=%08x)",
			(unsigned) opcode);
	return -1;
}

static int evaluate_b_bl_blx_thumb(uint16_t opcode,
				   uint32_t address, struct arm_instruction *instruction)
{
	uint32_t offset = opcode & 0x7ff;
	uint32_t opc = (opcode >> 11) & 0x3;
	uint32_t target_address;
	char *mnemonic = NULL;

	/* sign extend 11-bit offset */
	if (((opc == 0) || (opc == 2)) && (offset & 0x00000400))
		offset = 0xfffff800 | offset;

	target_address = address + 4 + (offset << 1);

	switch (opc) {
		/* unconditional branch */
		case 0:
			instruction->type = ARM_B;
			mnemonic = "B";
			break;
		/* BLX suffix */
		case 1:
			instruction->type = ARM_BLX;
			mnemonic = "BLX";
			target_address &= 0xfffffffc;
			break;
		/* BL/BLX prefix */
		case 2:
			instruction->type = ARM_UNKNOWN_INSTRUCTION;
			mnemonic = "prefix";
			target_address = offset << 12;
			break;
		/* BL suffix */
		case 3:
			instruction->type = ARM_BL;
			mnemonic = "BL";
			break;
	}

	/* TODO: deal correctly with dual opcode (prefixed) BL/BLX;
	 * these are effectively 32-bit instructions even in Thumb1.  For
	 * disassembly, it's simplest to always use the Thumb2 decoder.
	 *
	 * But some cores will evidently handle them as two instructions,
	 * where exceptions may occur between the two.  The ETMv3.2+ ID
	 * register has a bit which exposes this behavior.
	 */

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s\t%#8.8" PRIx32,
			address, opcode, mnemonic, target_address);

	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;

	return ERROR_OK;
}

static int evaluate_add_sub_thumb(uint16_t opcode,
				  uint32_t address, struct arm_instruction *instruction)
{
	uint8_t Rd = (opcode >> 0) & 0x7;
	uint8_t Rn = (opcode >> 3) & 0x7;
	uint8_t Rm_imm = (opcode >> 6) & 0x7;
	uint32_t opc = opcode & (1 << 9);
	uint32_t reg_imm  = opcode & (1 << 10);
	char *mnemonic;

	if (opc) {
		instruction->type = ARM_SUB;
		mnemonic = "SUBS";
	} else {
		/* REVISIT:  if reg_imm == 0, display as "MOVS" */
		instruction->type = ARM_ADD;
		mnemonic = "ADDS";
	}

	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rn;
	instruction->info.data_proc.S = 1;

	if (reg_imm) {
		instruction->info.data_proc.variant = 0;/*immediate*/
		instruction->info.data_proc.shifter_operand.immediate.immediate = Rm_imm;
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tr%i, r%i, #%d",
				address, opcode, mnemonic, Rd, Rn, Rm_imm);
	} else {
		instruction->info.data_proc.variant = 1;/*immediate shift*/
		instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm_imm;
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tr%i, r%i, r%i",
				address, opcode, mnemonic, Rd, Rn, Rm_imm);
	}

	return ERROR_OK;
}

static int evaluate_shift_imm_thumb(uint16_t opcode,
				    uint32_t address, struct arm_instruction *instruction)
{
	uint8_t Rd = (opcode >> 0) & 0x7;
	uint8_t Rm = (opcode >> 3) & 0x7;
	uint8_t imm = (opcode >> 6) & 0x1f;
	uint8_t opc = (opcode >> 11) & 0x3;
	char *mnemonic = NULL;

	switch (opc) {
		case 0:
			instruction->type = ARM_MOV;
			mnemonic = "LSLS";
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = 0;
			break;
		case 1:
			instruction->type = ARM_MOV;
			mnemonic = "LSRS";
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = 1;
			break;
		case 2:
			instruction->type = ARM_MOV;
			mnemonic = "ASRS";
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = 2;
			break;
	}

	if ((imm == 0) && (opc != 0))
		imm = 32;

	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = -1;
	instruction->info.data_proc.S = 1;

	instruction->info.data_proc.variant = 1;/*immediate_shift*/
	instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm;
	instruction->info.data_proc.shifter_operand.immediate_shift.shift_imm = imm;

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tr%i, r%i, #%#2.2x",
			address, opcode, mnemonic, Rd, Rm, imm);

	return ERROR_OK;
}

static int evaluate_data_proc_imm_thumb(uint16_t opcode,
					uint32_t address, struct arm_instruction *instruction)
{
	uint8_t imm = opcode & 0xff;
	uint8_t Rd = (opcode >> 8) & 0x7;
	uint32_t opc = (opcode >> 11) & 0x3;
	char *mnemonic = NULL;

	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rd;
	instruction->info.data_proc.S = 1;
	instruction->info.data_proc.variant = 0;/*immediate*/
	instruction->info.data_proc.shifter_operand.immediate.immediate = imm;

	switch (opc) {
		case 0:
			instruction->type = ARM_MOV;
			mnemonic = "MOVS";
			instruction->info.data_proc.Rn = -1;
			break;
		case 1:
			instruction->type = ARM_CMP;
			mnemonic = "CMP";
			instruction->info.data_proc.Rd = -1;
			break;
		case 2:
			instruction->type = ARM_ADD;
			mnemonic = "ADDS";
			break;
		case 3:
			instruction->type = ARM_SUB;
			mnemonic = "SUBS";
			break;
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tr%i, #%#2.2x",
			address, opcode, mnemonic, Rd, imm);

	return ERROR_OK;
}

static int evaluate_data_proc_thumb(uint16_t opcode,
				    uint32_t address, struct arm_instruction *instruction)
{
	uint8_t high_reg, op, Rm, Rd, H1, H2;
	char *mnemonic = NULL;
	bool nop = false;

	high_reg = (opcode & 0x0400) >> 10;
	op = (opcode & 0x03C0) >> 6;

	Rd = (opcode & 0x0007);
	Rm = (opcode & 0x0038) >> 3;
	H1 = (opcode & 0x0080) >> 7;
	H2 = (opcode & 0x0040) >> 6;

	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rd;
	instruction->info.data_proc.S = (!high_reg || (instruction->type == ARM_CMP));
	instruction->info.data_proc.variant = 1	/*immediate shift*/;
	instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm;

	if (high_reg) {
		Rd |= H1 << 3;
		Rm |= H2 << 3;
		op >>= 2;

		switch (op) {
			case 0x0:
				instruction->type = ARM_ADD;
				mnemonic = "ADD";
				break;
			case 0x1:
				instruction->type = ARM_CMP;
				mnemonic = "CMP";
				break;
			case 0x2:
				instruction->type = ARM_MOV;
				mnemonic = "MOV";
				if (Rd == Rm)
					nop = true;
				break;
			case 0x3:
				if ((opcode & 0x7) == 0x0) {
					instruction->info.b_bl_bx_blx.reg_operand = Rm;
					if (H1) {
						instruction->type = ARM_BLX;
						snprintf(instruction->text, 128,
								"0x%8.8" PRIx32
								"  0x%4.4x    \tBLX\tr%i",
								address, opcode, Rm);
					} else {
						instruction->type = ARM_BX;
						snprintf(instruction->text, 128,
								"0x%8.8" PRIx32
								"  0x%4.4x    \tBX\tr%i",
								address, opcode, Rm);
					}
				} else {
					instruction->type = ARM_UNDEFINED_INSTRUCTION;
					snprintf(instruction->text, 128,
							"0x%8.8" PRIx32
							"  0x%4.4x    \t"
							"UNDEFINED INSTRUCTION",
							address, opcode);
				}
				return ERROR_OK;
		}
	} else {
		switch (op) {
			case 0x0:
				instruction->type = ARM_AND;
				mnemonic = "ANDS";
				break;
			case 0x1:
				instruction->type = ARM_EOR;
				mnemonic = "EORS";
				break;
			case 0x2:
				instruction->type = ARM_MOV;
				mnemonic = "LSLS";
				instruction->info.data_proc.variant = 2	/*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 0;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x3:
				instruction->type = ARM_MOV;
				mnemonic = "LSRS";
				instruction->info.data_proc.variant = 2	/*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 1;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x4:
				instruction->type = ARM_MOV;
				mnemonic = "ASRS";
				instruction->info.data_proc.variant = 2	/*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 2;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x5:
				instruction->type = ARM_ADC;
				mnemonic = "ADCS";
				break;
			case 0x6:
				instruction->type = ARM_SBC;
				mnemonic = "SBCS";
				break;
			case 0x7:
				instruction->type = ARM_MOV;
				mnemonic = "RORS";
				instruction->info.data_proc.variant = 2	/*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 3;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x8:
				instruction->type = ARM_TST;
				mnemonic = "TST";
				break;
			case 0x9:
				instruction->type = ARM_RSB;
				mnemonic = "RSBS";
				instruction->info.data_proc.variant = 0	/*immediate*/;
				instruction->info.data_proc.shifter_operand.immediate.immediate = 0;
				instruction->info.data_proc.Rn = Rm;
				break;
			case 0xA:
				instruction->type = ARM_CMP;
				mnemonic = "CMP";
				break;
			case 0xB:
				instruction->type = ARM_CMN;
				mnemonic = "CMN";
				break;
			case 0xC:
				instruction->type = ARM_ORR;
				mnemonic = "ORRS";
				break;
			case 0xD:
				instruction->type = ARM_MUL;
				mnemonic = "MULS";
				break;
			case 0xE:
				instruction->type = ARM_BIC;
				mnemonic = "BICS";
				break;
			case 0xF:
				instruction->type = ARM_MVN;
				mnemonic = "MVNS";
				break;
		}
	}

	if (nop)
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \tNOP\t\t\t"
				"; (%s r%i, r%i)",
				address, opcode, mnemonic, Rd, Rm);
	else
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tr%i, r%i",
				address, opcode, mnemonic, Rd, Rm);

	return ERROR_OK;
}

/* PC-relative data addressing is word-aligned even with Thumb */
static inline uint32_t thumb_alignpc4(uint32_t addr)
{
	return (addr + 4) & ~3;
}

static int evaluate_load_literal_thumb(uint16_t opcode,
				       uint32_t address, struct arm_instruction *instruction)
{
	uint32_t immediate;
	uint8_t Rd = (opcode >> 8) & 0x7;

	instruction->type = ARM_LDR;
	immediate = opcode & 0x000000ff;
	immediate *= 4;

	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = 15 /*PC*/;
	instruction->info.load_store.index_mode = 0;	/*offset*/
	instruction->info.load_store.offset_mode = 0;	/*immediate*/
	instruction->info.load_store.offset.offset = immediate;

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t"
			"LDR\tr%i, [pc, #%#" PRIx32 "]\t; %#8.8" PRIx32,
			address, opcode, Rd, immediate,
			thumb_alignpc4(address) + immediate);

	return ERROR_OK;
}

static int evaluate_load_store_reg_thumb(uint16_t opcode,
					 uint32_t address, struct arm_instruction *instruction)
{
	uint8_t Rd = (opcode >> 0) & 0x7;
	uint8_t Rn = (opcode >> 3) & 0x7;
	uint8_t Rm = (opcode >> 6) & 0x7;
	uint8_t opc = (opcode >> 9) & 0x7;
	char *mnemonic = NULL;

	switch (opc) {
		case 0:
			instruction->type = ARM_STR;
			mnemonic = "STR";
			break;
		case 1:
			instruction->type = ARM_STRH;
			mnemonic = "STRH";
			break;
		case 2:
			instruction->type = ARM_STRB;
			mnemonic = "STRB";
			break;
		case 3:
			instruction->type = ARM_LDRSB;
			mnemonic = "LDRSB";
			break;
		case 4:
			instruction->type = ARM_LDR;
			mnemonic = "LDR";
			break;
		case 5:
			instruction->type = ARM_LDRH;
			mnemonic = "LDRH";
			break;
		case 6:
			instruction->type = ARM_LDRB;
			mnemonic = "LDRB";
			break;
		case 7:
			instruction->type = ARM_LDRSH;
			mnemonic = "LDRSH";
			break;
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tr%i, [r%i, r%i]",
			address, opcode, mnemonic, Rd, Rn, Rm);

	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.index_mode = 0;	/*offset*/
	instruction->info.load_store.offset_mode = 1;	/*register*/
	instruction->info.load_store.offset.reg.Rm = Rm;

	return ERROR_OK;
}

static int evaluate_load_store_imm_thumb(uint16_t opcode,
					 uint32_t address, struct arm_instruction *instruction)
{
	uint32_t offset = (opcode >> 6) & 0x1f;
	uint8_t Rd = (opcode >> 0) & 0x7;
	uint8_t Rn = (opcode >> 3) & 0x7;
	uint32_t L = opcode & (1 << 11);
	uint32_t B = opcode & (1 << 12);
	char *mnemonic;
	char suffix = ' ';
	uint32_t shift = 2;

	if (L) {
		instruction->type = ARM_LDR;
		mnemonic = "LDR";
	} else {
		instruction->type = ARM_STR;
		mnemonic = "STR";
	}

	if ((opcode&0xF000) == 0x8000) {
		suffix = 'H';
		shift = 1;
	} else if (B) {
		suffix = 'B';
		shift = 0;
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s%c\tr%i, [r%i, #%#" PRIx32 "]",
			address, opcode, mnemonic, suffix, Rd, Rn, offset << shift);

	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.index_mode = 0;	/*offset*/
	instruction->info.load_store.offset_mode = 0;	/*immediate*/
	instruction->info.load_store.offset.offset = offset << shift;

	return ERROR_OK;
}

static int evaluate_load_store_stack_thumb(uint16_t opcode,
					   uint32_t address, struct arm_instruction *instruction)
{
	uint32_t offset = opcode  & 0xff;
	uint8_t Rd = (opcode >> 8) & 0x7;
	uint32_t L = opcode & (1 << 11);
	char *mnemonic;

	if (L) {
		instruction->type = ARM_LDR;
		mnemonic = "LDR";
	} else {
		instruction->type = ARM_STR;
		mnemonic = "STR";
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tr%i, [SP, #%#" PRIx32 "]",
			address, opcode, mnemonic, Rd, offset*4);

	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = 13 /*SP*/;
	instruction->info.load_store.index_mode = 0;	/*offset*/
	instruction->info.load_store.offset_mode = 0;	/*immediate*/
	instruction->info.load_store.offset.offset = offset*4;

	return ERROR_OK;
}

static int evaluate_add_sp_pc_thumb(uint16_t opcode,
				    uint32_t address, struct arm_instruction *instruction)
{
	uint32_t imm = opcode  & 0xff;
	uint8_t Rd = (opcode >> 8) & 0x7;
	uint8_t Rn;
	uint32_t SP = opcode & (1 << 11);
	const char *reg_name;

	instruction->type = ARM_ADD;

	if (SP) {
		reg_name = "SP";
		Rn = 13;
	} else {
		reg_name = "PC";
		Rn = 15;
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x  \tADD\tr%i, %s, #%#" PRIx32,
			address, opcode, Rd, reg_name, imm * 4);

	instruction->info.data_proc.variant = 0	/* immediate */;
	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rn;
	instruction->info.data_proc.shifter_operand.immediate.immediate = imm*4;

	return ERROR_OK;
}

static int evaluate_adjust_stack_thumb(uint16_t opcode,
				       uint32_t address, struct arm_instruction *instruction)
{
	uint32_t imm = opcode  & 0x7f;
	uint8_t opc = opcode & (1 << 7);
	char *mnemonic;


	if (opc) {
		instruction->type = ARM_SUB;
		mnemonic = "SUB";
	} else {
		instruction->type = ARM_ADD;
		mnemonic = "ADD";
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s\tSP, #%#" PRIx32,
			address, opcode, mnemonic, imm*4);

	instruction->info.data_proc.variant = 0	/* immediate */;
	instruction->info.data_proc.Rd = 13 /*SP*/;
	instruction->info.data_proc.Rn = 13 /*SP*/;
	instruction->info.data_proc.shifter_operand.immediate.immediate = imm*4;

	return ERROR_OK;
}

static int evaluate_breakpoint_thumb(uint16_t opcode,
				     uint32_t address, struct arm_instruction *instruction)
{
	uint32_t imm = opcode  & 0xff;

	instruction->type = ARM_BKPT;

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x  \tBKPT\t%#2.2" PRIx32 "",
			address, opcode, imm);

	return ERROR_OK;
}

static int evaluate_load_store_multiple_thumb(uint16_t opcode,
					      uint32_t address, struct arm_instruction *instruction)
{
	uint32_t reg_list = opcode  & 0xff;
	uint32_t L = opcode & (1 << 11);
	uint32_t R = opcode & (1 << 8);
	uint8_t Rn = (opcode >> 8) & 7;
	uint8_t addr_mode = 0 /* IA */;
	char reg_names[40];
	char *reg_names_p;
	char *mnemonic;
	char ptr_name[7] = "";
	int i;

	/* REVISIT:  in ThumbEE mode, there are no LDM or STM instructions.
	 * The STMIA and LDMIA opcodes are used for other instructions.
	 */

	if ((opcode & 0xf000) == 0xc000) {	/* generic load/store multiple */
		char *wback = "!";

		if (L) {
			instruction->type = ARM_LDM;
			mnemonic = "LDM";
			if (opcode & (1 << Rn))
				wback = "";
		} else {
			instruction->type = ARM_STM;
			mnemonic = "STM";
		}
		snprintf(ptr_name, sizeof(ptr_name), "r%i%s, ", Rn, wback);
	} else {/* push/pop */
		Rn = 13;/* SP */
		if (L) {
			instruction->type = ARM_LDM;
			mnemonic = "POP";
			if (R)
				reg_list |= (1 << 15) /*PC*/;
		} else {
			instruction->type = ARM_STM;
			mnemonic = "PUSH";
			addr_mode = 3;	/*DB*/
			if (R)
				reg_list |= (1 << 14) /*LR*/;
		}
	}

	reg_names_p = reg_names;
	for (i = 0; i <= 15; i++) {
		if (reg_list & (1 << i))
			reg_names_p += snprintf(reg_names_p,
						(reg_names + 40 - reg_names_p),
						"r%i, ",
						i);
	}
	if (reg_names_p > reg_names)
		reg_names_p[-2] = '\0';
	else	/* invalid op : no registers */
		reg_names[0] = '\0';

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x  \t%s\t%s{%s}",
			address, opcode, mnemonic, ptr_name, reg_names);

	instruction->info.load_store_multiple.register_list = reg_list;
	instruction->info.load_store_multiple.Rn = Rn;
	instruction->info.load_store_multiple.addressing_mode = addr_mode;

	return ERROR_OK;
}

static int evaluate_cond_branch_thumb(uint16_t opcode,
				      uint32_t address, struct arm_instruction *instruction)
{
	uint32_t offset = opcode  & 0xff;
	uint8_t cond = (opcode >> 8) & 0xf;
	uint32_t target_address;

	if (cond == 0xf) {
		instruction->type = ARM_SWI;
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \tSVC\t%#2.2" PRIx32,
				address, opcode, offset);
		return ERROR_OK;
	} else if (cond == 0xe) {
		instruction->type = ARM_UNDEFINED_INSTRUCTION;
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \tUNDEFINED INSTRUCTION",
				address, opcode);
		return ERROR_OK;
	}

	/* sign extend 8-bit offset */
	if (offset & 0x00000080)
		offset = 0xffffff00 | offset;

	target_address = address + 4 + (offset << 1);

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \tB%s\t%#8.8" PRIx32,
			address, opcode,
			arm_condition_strings[cond], target_address);

	instruction->type = ARM_B;
	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;

	return ERROR_OK;
}

static int evaluate_cb_thumb(uint16_t opcode, uint32_t address,
			     struct arm_instruction *instruction)
{
	unsigned offset;

	/* added in Thumb2 */
	offset = (opcode >> 3) & 0x1f;
	offset |= (opcode & 0x0200) >> 4;

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \tCB%sZ\tr%d, %#8.8" PRIx32,
			address, opcode,
			(opcode & 0x0800) ? "N" : "",
			opcode & 0x7, address + 4 + (offset << 1));

	return ERROR_OK;
}

static int evaluate_extend_thumb(uint16_t opcode, uint32_t address,
				 struct arm_instruction *instruction)
{
	/* added in ARMv6 */
	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%cXT%c\tr%d, r%d",
			address, opcode,
			(opcode & 0x0080) ? 'U' : 'S',
			(opcode & 0x0040) ? 'B' : 'H',
			opcode & 0x7, (opcode >> 3) & 0x7);

	return ERROR_OK;
}

static int evaluate_cps_thumb(uint16_t opcode, uint32_t address,
			      struct arm_instruction *instruction)
{
	/* added in ARMv6 */
	if ((opcode & 0x0ff0) == 0x0650)
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \tSETEND %s",
				address, opcode,
				(opcode & 0x80) ? "BE" : "LE");
	else	/* ASSUME (opcode & 0x0fe0) == 0x0660 */
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \tCPSI%c\t%s%s%s",
				address, opcode,
				(opcode & 0x0010) ? 'D' : 'E',
				(opcode & 0x0004) ? "A" : "",
				(opcode & 0x0002) ? "I" : "",
				(opcode & 0x0001) ? "F" : "");

	return ERROR_OK;
}

static int evaluate_byterev_thumb(uint16_t opcode, uint32_t address,
				  struct arm_instruction *instruction)
{
	char *suffix;

	/* added in ARMv6 */
	switch ((opcode >> 6) & 3) {
		case 0:
			suffix = "";
			break;
		case 1:
			suffix = "16";
			break;
		default:
			suffix = "SH";
			break;
	}
	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \tREV%s\tr%d, r%d",
			address, opcode, suffix,
			opcode & 0x7, (opcode >> 3) & 0x7);

	return ERROR_OK;
}

static int evaluate_hint_thumb(uint16_t opcode, uint32_t address,
			       struct arm_instruction *instruction)
{
	char *hint;

	switch ((opcode >> 4) & 0x0f) {
		case 0:
			hint = "NOP";
			break;
		case 1:
			hint = "YIELD";
			break;
		case 2:
			hint = "WFE";
			break;
		case 3:
			hint = "WFI";
			break;
		case 4:
			hint = "SEV";
			break;
		default:
			hint = "HINT (UNRECOGNIZED)";
			break;
	}

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \t%s",
			address, opcode, hint);

	return ERROR_OK;
}

static int evaluate_ifthen_thumb(uint16_t opcode, uint32_t address,
				 struct arm_instruction *instruction)
{
	unsigned cond = (opcode >> 4) & 0x0f;
	char *x = "", *y = "", *z = "";

	if (opcode & 0x01)
		z = (opcode & 0x02) ? "T" : "E";
	if (opcode & 0x03)
		y = (opcode & 0x04) ? "T" : "E";
	if (opcode & 0x07)
		x = (opcode & 0x08) ? "T" : "E";

	snprintf(instruction->text, 128,
			"0x%8.8" PRIx32 "  0x%4.4x    \tIT%s%s%s\t%s",
			address, opcode,
			x, y, z, arm_condition_strings[cond]);

	/* NOTE:  strictly speaking, the next 1-4 instructions should
	 * now be displayed with the relevant conditional suffix...
	 */

	return ERROR_OK;
}

int thumb_evaluate_opcode(uint16_t opcode, uint32_t address, struct arm_instruction *instruction)
{
	/* clear fields, to avoid confusion */
	memset(instruction, 0, sizeof(struct arm_instruction));
	instruction->opcode = opcode;
	instruction->instruction_size = 2;

	if ((opcode & 0xe000) == 0x0000) {
		/* add/subtract register or immediate */
		if ((opcode & 0x1800) == 0x1800)
			return evaluate_add_sub_thumb(opcode, address, instruction);
		/* shift by immediate */
		else
			return evaluate_shift_imm_thumb(opcode, address, instruction);
	}

	/* Add/subtract/compare/move immediate */
	if ((opcode & 0xe000) == 0x2000)
		return evaluate_data_proc_imm_thumb(opcode, address, instruction);

	/* Data processing instructions */
	if ((opcode & 0xf800) == 0x4000)
		return evaluate_data_proc_thumb(opcode, address, instruction);

	/* Load from literal pool */
	if ((opcode & 0xf800) == 0x4800)
		return evaluate_load_literal_thumb(opcode, address, instruction);

	/* Load/Store register offset */
	if ((opcode & 0xf000) == 0x5000)
		return evaluate_load_store_reg_thumb(opcode, address, instruction);

	/* Load/Store immediate offset */
	if (((opcode & 0xe000) == 0x6000)
			|| ((opcode & 0xf000) == 0x8000))
		return evaluate_load_store_imm_thumb(opcode, address, instruction);

	/* Load/Store from/to stack */
	if ((opcode & 0xf000) == 0x9000)
		return evaluate_load_store_stack_thumb(opcode, address, instruction);

	/* Add to SP/PC */
	if ((opcode & 0xf000) == 0xa000)
		return evaluate_add_sp_pc_thumb(opcode, address, instruction);

	/* Misc */
	if ((opcode & 0xf000) == 0xb000) {
		switch ((opcode >> 8) & 0x0f) {
			case 0x0:
				return evaluate_adjust_stack_thumb(opcode, address, instruction);
			case 0x1:
			case 0x3:
			case 0x9:
			case 0xb:
				return evaluate_cb_thumb(opcode, address, instruction);
			case 0x2:
				return evaluate_extend_thumb(opcode, address, instruction);
			case 0x4:
			case 0x5:
			case 0xc:
			case 0xd:
				return evaluate_load_store_multiple_thumb(opcode, address,
					instruction);
			case 0x6:
				return evaluate_cps_thumb(opcode, address, instruction);
			case 0xa:
				if ((opcode & 0x00c0) == 0x0080)
					break;
				return evaluate_byterev_thumb(opcode, address, instruction);
			case 0xe:
				return evaluate_breakpoint_thumb(opcode, address, instruction);
			case 0xf:
				if (opcode & 0x000f)
					return evaluate_ifthen_thumb(opcode, address,
							instruction);
				else
					return evaluate_hint_thumb(opcode, address,
							instruction);
		}

		instruction->type = ARM_UNDEFINED_INSTRUCTION;
		snprintf(instruction->text, 128,
				"0x%8.8" PRIx32 "  0x%4.4x    \tUNDEFINED INSTRUCTION",
				address, opcode);
		return ERROR_OK;
	}

	/* Load/Store multiple */
	if ((opcode & 0xf000) == 0xc000)
		return evaluate_load_store_multiple_thumb(opcode, address, instruction);

	/* Conditional branch + SWI */
	if ((opcode & 0xf000) == 0xd000)
		return evaluate_cond_branch_thumb(opcode, address, instruction);

	if ((opcode & 0xe000) == 0xe000) {
		/* Undefined instructions */
		if ((opcode & 0xf801) == 0xe801) {
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128,
					"0x%8.8" PRIx32 "  0x%8.8x\t"
					"UNDEFINED INSTRUCTION",
					address, opcode);
			return ERROR_OK;
		} else	/* Branch to offset */
			return evaluate_b_bl_blx_thumb(opcode, address, instruction);
	}

	LOG_ERROR("Thumb: should never reach this point (opcode=%04x)", opcode);
	return -1;
}

int arm_access_size(struct arm_instruction *instruction)
{
	if ((instruction->type == ARM_LDRB)
	    || (instruction->type == ARM_LDRBT)
	    || (instruction->type == ARM_LDRSB)
	    || (instruction->type == ARM_STRB)
	    || (instruction->type == ARM_STRBT))
		return 1;
	else if ((instruction->type == ARM_LDRH)
		 || (instruction->type == ARM_LDRSH)
		 || (instruction->type == ARM_STRH))
		return 2;
	else if ((instruction->type == ARM_LDR)
		 || (instruction->type == ARM_LDRT)
		 || (instruction->type == ARM_STR)
		 || (instruction->type == ARM_STRT))
		return 4;
	else if ((instruction->type == ARM_LDRD)
		 || (instruction->type == ARM_STRD))
		return 8;
	else {
		LOG_ERROR("BUG: instruction type %i isn't a load/store instruction",
				instruction->type);
		return 0;
	}
}

#if HAVE_CAPSTONE
static void print_opcode(struct command_invocation *cmd, const cs_insn *insn)
{
	uint32_t opcode = 0;

	memcpy(&opcode, insn->bytes, insn->size);

	if (insn->size == 4) {
		uint16_t opcode_high = opcode >> 16;
		opcode = opcode & 0xffff;

		command_print(cmd, "0x%08" PRIx64"  %04x %04x\t%s%s%s",
			insn->address, opcode, opcode_high, insn->mnemonic,
			insn->op_str[0] ? "\t" : "", insn->op_str);
	} else {
		command_print(cmd, "0x%08" PRIx64"  %04x\t%s%s%s",
			insn->address, opcode, insn->mnemonic,
			insn->op_str[0] ? "\t" : "", insn->op_str);
	}
}

int arm_disassemble(struct command_invocation *cmd, struct target *target,
		target_addr_t address, size_t count, bool thumb_mode)
{
	csh handle;
	int ret;
	cs_insn *insn;
	cs_mode mode;

	if (!cs_support(CS_ARCH_ARM)) {
		LOG_ERROR("ARM architecture not supported by capstone");
		return ERROR_FAIL;
	}

	mode = CS_MODE_LITTLE_ENDIAN;

	if (thumb_mode)
		mode |= CS_MODE_THUMB;

	ret = cs_open(CS_ARCH_ARM, mode, &handle);

	if (ret != CS_ERR_OK) {
		LOG_ERROR("cs_open() failed: %s", cs_strerror(ret));
		return ERROR_FAIL;
	}

	ret = cs_option(handle, CS_OPT_SKIPDATA, CS_OPT_ON);

	if (ret != CS_ERR_OK) {
		LOG_ERROR("cs_option() failed: %s", cs_strerror(ret));
		cs_close(&handle);
		return ERROR_FAIL;
	}

	insn = cs_malloc(handle);

	if (!insn) {
		LOG_ERROR("cs_malloc() failed\n");
		cs_close(&handle);
		return ERROR_FAIL;
	}

	while (count > 0) {
	    uint8_t buffer[4];

		ret = target_read_buffer(target, address, sizeof(buffer), buffer);

		if (ret != ERROR_OK) {
			cs_free(insn, 1);
			cs_close(&handle);
			return ret;
		}

		size_t size = sizeof(buffer);
		const uint8_t *tmp = buffer;

		ret = cs_disasm_iter(handle, &tmp, &size, &address, insn);

		if (!ret) {
			LOG_ERROR("cs_disasm_iter() failed: %s",
				cs_strerror(cs_errno(handle)));
			cs_free(insn, 1);
			cs_close(&handle);
			return ERROR_FAIL;
		}

		print_opcode(cmd, insn);
		count--;
	}

	cs_free(insn, 1);
	cs_close(&handle);

	return ERROR_OK;
}
#endif /* HAVE_CAPSTONE */
