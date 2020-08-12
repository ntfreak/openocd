/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef OPENOCD_TARGET_ARM_DISASSEMBLER_H
#define OPENOCD_TARGET_ARM_DISASSEMBLER_H

enum arm_instruction_type {
	ARM_UNKNOWN_INSTRUCTION,

	/* Branch instructions */
	ARM_B,
	ARM_BL,
	ARM_BX,
	ARM_BLX,

	/* Data processing instructions */
	ARM_AND,
	ARM_EOR,
	ARM_SUB,
	ARM_RSB,
	ARM_ADD,
	ARM_ADC,
	ARM_SBC,
	ARM_RSC,
	ARM_TST,
	ARM_TEQ,
	ARM_CMP,
	ARM_CMN,
	ARM_ORR,
	ARM_MOV,
	ARM_BIC,
	ARM_MVN,

	/* Load/store instructions */
	ARM_LDR,
	ARM_LDRB,
	ARM_LDRT,
	ARM_LDRBT,

	ARM_LDRH,
	ARM_LDRSB,
	ARM_LDRSH,

	ARM_LDM,

	ARM_STR,
	ARM_STRB,
	ARM_STRT,
	ARM_STRBT,

	ARM_STRH,

	ARM_STM,

	/* Status register access instructions */
	ARM_MRS,
	ARM_MSR,

	/* Multiply instructions */
	ARM_MUL,
	ARM_MLA,
	ARM_SMULL,
	ARM_SMLAL,
	ARM_UMULL,
	ARM_UMLAL,

	/* Miscellaneous instructions */
	ARM_CLZ,

	/* Exception return instructions */
	ARM_ERET,

	/* Exception generating instructions */
	ARM_BKPT,
	ARM_SWI,
	ARM_HVC,
	ARM_SMC,

	/* Coprocessor instructions */
	ARM_CDP,
	ARM_LDC,
	ARM_STC,
	ARM_MCR,
	ARM_MRC,

	/* Semaphore instructions */
	ARM_SWP,
	ARM_SWPB,

	/* Enhanced DSP extensions */
	ARM_MCRR,
	ARM_MRRC,
	ARM_PLD,
	ARM_DSB,
	ARM_ISB,
	ARM_QADD,
	ARM_QDADD,
	ARM_QSUB,
	ARM_QDSUB,
	ARM_SMLAxy,
	ARM_SMLALxy,
	ARM_SMLAWy,
	ARM_SMULxy,
	ARM_SMULWy,
	ARM_LDRD,
	ARM_STRD,

	ARM_UNDEFINED_INSTRUCTION = 0xffffffff,
};

struct arm_b_bl_bx_blx_instr {
	int reg_operand;
	uint32_t target_address;
};

union arm_shifter_operand {
	struct {
		uint32_t immediate;
	} immediate;
	struct {
		uint8_t Rm;
		uint8_t shift; /* 0: LSL, 1: LSR, 2: ASR, 3: ROR, 4: RRX */
		uint8_t shift_imm;
	} immediate_shift;
	struct {
		uint8_t Rm;
		uint8_t shift;
		uint8_t Rs;
	} register_shift;
};

struct arm_data_proc_instr {
	int variant; /* 0: immediate, 1: immediate_shift, 2: register_shift */
	uint8_t S;
	uint8_t Rn;
	uint8_t Rd;
	union arm_shifter_operand shifter_operand;
};

struct arm_load_store_instr {
	uint8_t Rd;
	uint8_t Rn;
	uint8_t U;
	int index_mode; /* 0: offset, 1: pre-indexed, 2: post-indexed */
	int offset_mode; /* 0: immediate, 1: (scaled) register */
	union {
		uint32_t offset;
		struct {
			uint8_t Rm;
			uint8_t shift; /* 0: LSL, 1: LSR, 2: ASR, 3: ROR, 4: RRX */
			uint8_t shift_imm;
		} reg;
	} offset;
};

struct arm_load_store_multiple_instr {
	uint8_t Rn;
	uint32_t register_list;
	uint8_t addressing_mode; /* 0: IA, 1: IB, 2: DA, 3: DB */
	uint8_t S;
	uint8_t W;
};

struct arm_instruction {
	enum arm_instruction_type type;
	char text[128];
	uint32_t opcode;

	/* return value ... Thumb-2 sizes vary */
	unsigned instruction_size;

	union {
		struct arm_b_bl_bx_blx_instr b_bl_bx_blx;
		struct arm_data_proc_instr data_proc;
		struct arm_load_store_instr load_store;
		struct arm_load_store_multiple_instr load_store_multiple;
	} info;

};

int arm_evaluate_opcode(uint32_t opcode, uint32_t address,
		struct arm_instruction *instruction);
int thumb_evaluate_opcode(uint16_t opcode, uint32_t address,
		struct arm_instruction *instruction);
int arm_access_size(struct arm_instruction *instruction);
#if HAVE_CAPSTONE
int arm_disassemble(struct command_invocation *cmd, struct target *target,
		target_addr_t address, size_t count, bool thumb_mode);
#endif

#define COND(opcode) (arm_condition_strings[(opcode & 0xf0000000) >> 28])

#endif /* OPENOCD_TARGET_ARM_DISASSEMBLER_H */
