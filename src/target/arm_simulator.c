/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Hongtao Zheng                                   *
 *   hontor@126.com                                                        *
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

#include "arm.h"
#include "armv4_5.h"
#include "arm_disassembler.h"
#include "arm_simulator.h"
#include <helper/binarybuffer.h>
#include "register.h"
#include <helper/log.h>

static uint32_t arm_shift(uint8_t shift, uint32_t rm,
	uint32_t shift_amount, uint8_t *carry)
{
	uint32_t return_value = 0;
	shift_amount &= 0xff;

	if (shift == 0x0) {	/* LSL */
		if ((shift_amount > 0) && (shift_amount <= 32)) {
			return_value = rm << shift_amount;
			*carry = rm >> (32 - shift_amount);
		} else if (shift_amount > 32) {
			return_value = 0x0;
			*carry = 0x0;
		} else /* (shift_amount == 0) */
			return_value = rm;
	} else if (shift == 0x1) {	/* LSR */
		if ((shift_amount > 0) && (shift_amount <= 32)) {
			return_value = rm >> shift_amount;
			*carry = (rm >> (shift_amount - 1)) & 1;
		} else if (shift_amount > 32) {
			return_value = 0x0;
			*carry = 0x0;
		} else /* (shift_amount == 0) */
			return_value = rm;
	} else if (shift == 0x2) {	/* ASR */
		if ((shift_amount > 0) && (shift_amount <= 32)) {
			/* C right shifts of unsigned values are guaranteed to
			 * be logical (shift in zeroes); simulate an arithmetic
			 * shift (shift in signed-bit) by adding the sign bit
			 * manually
			 */
			return_value = rm >> shift_amount;
			if (rm & 0x80000000)
				return_value |= 0xffffffff << (32 - shift_amount);
		} else if (shift_amount > 32) {
			if (rm & 0x80000000) {
				return_value = 0xffffffff;
				*carry = 0x1;
			} else {
				return_value = 0x0;
				*carry = 0x0;
			}
		} else /* (shift_amount == 0) */
			return_value = rm;
	} else if (shift == 0x3) {	/* ROR */
		if (shift_amount == 0)
			return_value = rm;
		else {
			shift_amount = shift_amount % 32;
			return_value = (rm >> shift_amount) | (rm << (32 - shift_amount));
			*carry = (return_value >> 31) & 0x1;
		}
	} else if (shift == 0x4) {	/* RRX */
		return_value = rm >> 1;
		if (*carry)
			rm |= 0x80000000;
		*carry = rm & 0x1;
	}

	return return_value;
}


static uint32_t arm_shifter_operand(struct arm_sim_interface *sim,
	int variant, union arm_shifter_operand shifter_operand,
	uint8_t *shifter_carry_out)
{
	uint32_t return_value;
	int instruction_size;

	if (sim->get_state(sim) == ARM_STATE_ARM)
		instruction_size = 4;
	else
		instruction_size = 2;

	*shifter_carry_out = sim->get_cpsr(sim, 29, 1);

	if (variant == 0) /* 32-bit immediate */
		return_value = shifter_operand.immediate.immediate;
	else if (variant == 1) {/* immediate shift */
		uint32_t rm = sim->get_reg_mode(sim, shifter_operand.immediate_shift.rm);

		/* adjust RM in case the PC is being read */
		if (shifter_operand.immediate_shift.rm == 15)
			rm += 2 * instruction_size;

		return_value = arm_shift(shifter_operand.immediate_shift.shift,
				rm, shifter_operand.immediate_shift.shift_imm,
				shifter_carry_out);
	} else if (variant == 2) {	/* register shift */
		uint32_t rm = sim->get_reg_mode(sim, shifter_operand.register_shift.rm);
		uint32_t rs = sim->get_reg_mode(sim, shifter_operand.register_shift.rs);

		/* adjust RM in case the PC is being read */
		if (shifter_operand.register_shift.rm == 15)
			rm += 2 * instruction_size;

		return_value = arm_shift(shifter_operand.immediate_shift.shift,
				rm, rs, shifter_carry_out);
	} else {
		LOG_ERROR("BUG: shifter_operand.variant not 0, 1 or 2");
		return_value = 0xffffffff;
	}

	return return_value;
}

static int pass_condition(uint32_t cpsr, uint32_t opcode)
{
	switch ((opcode & 0xf0000000) >> 28) {
		case 0x0:	/* EQ */
			if (cpsr & 0x40000000)
				return 1;
			else
				return 0;
		case 0x1:	/* NE */
			if (!(cpsr & 0x40000000))
				return 1;
			else
				return 0;
		case 0x2:	/* CS */
			if (cpsr & 0x20000000)
				return 1;
			else
				return 0;
		case 0x3:	/* CC */
			if (!(cpsr & 0x20000000))
				return 1;
			else
				return 0;
		case 0x4:	/* MI */
			if (cpsr & 0x80000000)
				return 1;
			else
				return 0;
		case 0x5:	/* PL */
			if (!(cpsr & 0x80000000))
				return 1;
			else
				return 0;
		case 0x6:	/* VS */
			if (cpsr & 0x10000000)
				return 1;
			else
				return 0;
		case 0x7:	/* VC */
			if (!(cpsr & 0x10000000))
				return 1;
			else
				return 0;
		case 0x8:	/* HI */
			if ((cpsr & 0x20000000) && !(cpsr & 0x40000000))
				return 1;
			else
				return 0;
		case 0x9:	/* LS */
			if (!(cpsr & 0x20000000) || (cpsr & 0x40000000))
				return 1;
			else
				return 0;
		case 0xa:	/* GE */
			if (((cpsr & 0x80000000) && (cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && !(cpsr & 0x10000000)))
				return 1;
			else
				return 0;
		case 0xb:	/* LT */
			if (((cpsr & 0x80000000) && !(cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && (cpsr & 0x10000000)))
				return 1;
			else
				return 0;
		case 0xc:	/* GT */
			if (!(cpsr & 0x40000000) &&
				(((cpsr & 0x80000000) && (cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && !(cpsr & 0x10000000))))
				return 1;
			else
				return 0;
		case 0xd:	/* LE */
			if ((cpsr & 0x40000000) ||
				((cpsr & 0x80000000) && !(cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && (cpsr & 0x10000000)))
				return 1;
			else
				return 0;
		case 0xe:
		case 0xf:
			return 1;

	}

	LOG_ERROR("BUG: should never get here");
	return 0;
}

static int thumb_pass_branch_condition(uint32_t cpsr, uint16_t opcode)
{
	return pass_condition(cpsr, (opcode & 0x0f00) << 20);
}

/* simulate a single step (if possible)
 * if the dry_run_pc argument is provided, no state is changed,
 * but the new pc is stored in the variable pointed at by the argument
 */
static int arm_simulate_step_core(struct target *target,
	uint32_t *dry_run_pc, struct arm_sim_interface *sim)
{
	uint32_t current_pc = sim->get_reg(sim, 15);
	struct arm_instruction instruction;
	int instruction_size;
	int retval = ERROR_OK;

	if (sim->get_state(sim) == ARM_STATE_ARM) {
		uint32_t opcode;

		/* get current instruction, and identify it */
		retval = target_read_u32(target, current_pc, &opcode);
		if (retval != ERROR_OK)
			return retval;
		retval = arm_evaluate_opcode(opcode, current_pc, &instruction);
		if (retval != ERROR_OK)
			return retval;
		instruction_size = 4;

		/* check condition code (for all instructions) */
		if (!pass_condition(sim->get_cpsr(sim, 0, 32), opcode)) {
			if (dry_run_pc)
				*dry_run_pc = current_pc + instruction_size;
			else
				sim->set_reg(sim, 15, current_pc + instruction_size);

			return ERROR_OK;
		}
	} else {
		uint16_t opcode;

		retval = target_read_u16(target, current_pc, &opcode);
		if (retval != ERROR_OK)
			return retval;
		retval = thumb_evaluate_opcode(opcode, current_pc, &instruction);
		if (retval != ERROR_OK)
			return retval;
		instruction_size = 2;

		/* check condition code (only for branch (1) instructions) */
		if ((opcode & 0xf000) == 0xd000
			&& !thumb_pass_branch_condition(
			sim->get_cpsr(sim, 0, 32), opcode)) {
			if (dry_run_pc)
				*dry_run_pc = current_pc + instruction_size;
			else
				sim->set_reg(sim, 15, current_pc + instruction_size);

			return ERROR_OK;
		}

		/* Deal with 32-bit BL/BLX */
		if ((opcode & 0xf800) == 0xf000) {
			uint32_t high = instruction.info.b_bl_bx_blx.target_address;
			retval = target_read_u16(target, current_pc+2, &opcode);
			if (retval != ERROR_OK)
				return retval;
			retval = thumb_evaluate_opcode(opcode, current_pc, &instruction);
			if (retval != ERROR_OK)
				return retval;
			instruction.info.b_bl_bx_blx.target_address += high;
		}
	}

	/* examine instruction type */

	/* branch instructions */
	if ((instruction.type >= ARM_B) && (instruction.type <= ARM_BLX)) {
		uint32_t target_address;

		if (instruction.info.b_bl_bx_blx.reg_operand == -1)
			target_address = instruction.info.b_bl_bx_blx.target_address;
		else {
			target_address = sim->get_reg_mode(sim,
					instruction.info.b_bl_bx_blx.reg_operand);
			if (instruction.info.b_bl_bx_blx.reg_operand == 15)
				target_address += 2 * instruction_size;
		}

		if (dry_run_pc) {
			*dry_run_pc = target_address & ~1;
			return ERROR_OK;
		} else {
			if (instruction.type == ARM_B)
				sim->set_reg(sim, 15, target_address);
			else if (instruction.type == ARM_BL) {
				uint32_t old_pc = sim->get_reg(sim, 15);
				int t = (sim->get_state(sim) == ARM_STATE_THUMB);
				sim->set_reg_mode(sim, 14, old_pc + 4 + t);
				sim->set_reg(sim, 15, target_address);
			} else if (instruction.type == ARM_BX) {
				if (target_address & 0x1)
					sim->set_state(sim, ARM_STATE_THUMB);
				else
					sim->set_state(sim, ARM_STATE_ARM);
				sim->set_reg(sim, 15, target_address & 0xfffffffe);
			} else if (instruction.type == ARM_BLX) {
				uint32_t old_pc = sim->get_reg(sim, 15);
				int t = (sim->get_state(sim) == ARM_STATE_THUMB);
				sim->set_reg_mode(sim, 14, old_pc + 4 + t);

				if (target_address & 0x1)
					sim->set_state(sim, ARM_STATE_THUMB);
				else
					sim->set_state(sim, ARM_STATE_ARM);
				sim->set_reg(sim, 15, target_address & 0xfffffffe);
			}

			return ERROR_OK;
		}
	}
	/* data processing instructions, except compare instructions (CMP, CMN, TST, TEQ) */
	else if (((instruction.type >= ARM_AND) && (instruction.type <= ARM_RSC))
		|| ((instruction.type >= ARM_ORR) && (instruction.type <= ARM_MVN))) {
		uint32_t rd, rn, shifter_operand;
		uint8_t c = sim->get_cpsr(sim, 29, 1);
		uint8_t carry_out;

		rd = 0x0;
		/* ARM_MOV and ARM_MVN does not use Rn */
		if ((instruction.type != ARM_MOV) && (instruction.type != ARM_MVN))
			rn = sim->get_reg_mode(sim, instruction.info.data_proc.rn);
		else
			rn = 0;

		shifter_operand = arm_shifter_operand(sim,
				instruction.info.data_proc.variant,
				instruction.info.data_proc.shifter_operand,
				&carry_out);

		/* adjust Rn in case the PC is being read */
		if (instruction.info.data_proc.rn == 15)
			rn += 2 * instruction_size;

		if (instruction.type == ARM_AND)
			rd = rn & shifter_operand;
		else if (instruction.type == ARM_EOR)
			rd = rn ^ shifter_operand;
		else if (instruction.type == ARM_SUB)
			rd = rn - shifter_operand;
		else if (instruction.type == ARM_RSB)
			rd = shifter_operand - rn;
		else if (instruction.type == ARM_ADD)
			rd = rn + shifter_operand;
		else if (instruction.type == ARM_ADC)
			rd = rn + shifter_operand + (c & 1);
		else if (instruction.type == ARM_SBC)
			rd = rn - shifter_operand - (c & 1) ? 0 : 1;
		else if (instruction.type == ARM_RSC)
			rd = shifter_operand - rn - (c & 1) ? 0 : 1;
		else if (instruction.type == ARM_ORR)
			rd = rn | shifter_operand;
		else if (instruction.type == ARM_BIC)
			rd = rn & ~(shifter_operand);
		else if (instruction.type == ARM_MOV)
			rd = shifter_operand;
		else if (instruction.type == ARM_MVN)
			rd = ~shifter_operand;
		else
			LOG_WARNING("unhandled instruction type");

		if (dry_run_pc) {
			if (instruction.info.data_proc.rd == 15)
				*dry_run_pc = rd & ~1;
			else
				*dry_run_pc = current_pc + instruction_size;

			return ERROR_OK;
		} else {
			if (instruction.info.data_proc.rd == 15) {
				sim->set_reg_mode(sim, 15, rd & ~1);
				if (rd & 1)
					sim->set_state(sim, ARM_STATE_THUMB);
				else
					sim->set_state(sim, ARM_STATE_ARM);
				return ERROR_OK;
			}
			sim->set_reg_mode(sim, instruction.info.data_proc.rd, rd);
			LOG_WARNING("no updating of flags yet");
		}
	}
	/* compare instructions (CMP, CMN, TST, TEQ) */
	else if ((instruction.type >= ARM_TST) && (instruction.type <= ARM_CMN)) {
		if (dry_run_pc) {
			*dry_run_pc = current_pc + instruction_size;
			return ERROR_OK;
		} else
			LOG_WARNING("no updating of flags yet");
	}
	/* load register instructions */
	else if ((instruction.type >= ARM_LDR) && (instruction.type <= ARM_LDRSH)) {
		uint32_t load_address = 0, modified_address = 0, load_value = 0;
		uint32_t rn = sim->get_reg_mode(sim, instruction.info.load_store.rn);

		/* adjust Rn in case the PC is being read */
		if (instruction.info.load_store.rn == 15)
			rn += 2 * instruction_size;

		if (instruction.info.load_store.offset_mode == 0) {
			if (instruction.info.load_store.u)
				modified_address = rn + instruction.info.load_store.offset.offset;
			else
				modified_address = rn - instruction.info.load_store.offset.offset;
		} else if (instruction.info.load_store.offset_mode == 1) {
			uint32_t offset;
			uint32_t rm = sim->get_reg_mode(sim,
					instruction.info.load_store.offset.reg.rm);
			uint8_t shift = instruction.info.load_store.offset.reg.shift;
			uint8_t shift_imm = instruction.info.load_store.offset.reg.shift_imm;
			uint8_t carry = sim->get_cpsr(sim, 29, 1);

			offset = arm_shift(shift, rm, shift_imm, &carry);

			if (instruction.info.load_store.u)
				modified_address = rn + offset;
			else
				modified_address = rn - offset;
		} else
			LOG_ERROR("BUG: offset_mode neither 0 (offset) nor 1 (scaled register)");

		if (instruction.info.load_store.index_mode == 0) {
			/* offset mode
			 * we load from the modified address, but don't change
			 * the base address register
			 */
			load_address = modified_address;
			modified_address = rn;
		} else if (instruction.info.load_store.index_mode == 1) {
			/* pre-indexed mode
			 * we load from the modified address, and write it
			 * back to the base address register
			 */
			load_address = modified_address;
		} else if (instruction.info.load_store.index_mode == 2) {
			/* post-indexed mode
			 * we load from the unmodified address, and write the
			 * modified address back
			 */
			load_address = rn;
		}

		if ((!dry_run_pc) || (instruction.info.load_store.rd == 15)) {
			retval = target_read_u32(target, load_address, &load_value);
			if (retval != ERROR_OK)
				return retval;
		}

		if (dry_run_pc) {
			if (instruction.info.load_store.rd == 15)
				*dry_run_pc = load_value & ~1;
			else
				*dry_run_pc = current_pc + instruction_size;
			return ERROR_OK;
		} else {
			if ((instruction.info.load_store.index_mode == 1) ||
				(instruction.info.load_store.index_mode == 2))
				sim->set_reg_mode(sim,
					instruction.info.load_store.rn,
					modified_address);

			if (instruction.info.load_store.rd == 15) {
				sim->set_reg_mode(sim, 15, load_value & ~1);
				if (load_value & 1)
					sim->set_state(sim, ARM_STATE_THUMB);
				else
					sim->set_state(sim, ARM_STATE_ARM);
				return ERROR_OK;
			}
			sim->set_reg_mode(sim, instruction.info.load_store.rd, load_value);
		}
	}
	/* load multiple instruction */
	else if (instruction.type == ARM_LDM) {
		int i;
		uint32_t rn = sim->get_reg_mode(sim, instruction.info.load_store_multiple.rn);
		uint32_t load_values[16];
		int bits_set = 0;

		for (i = 0; i < 16; i++) {
			if (instruction.info.load_store_multiple.register_list & (1 << i))
				bits_set++;
		}

		switch (instruction.info.load_store_multiple.addressing_mode) {
			case 0:	/* Increment after */
				/* rn = rn; */
				break;
			case 1:	/* Increment before */
				rn = rn + 4;
				break;
			case 2:	/* Decrement after */
				rn = rn - (bits_set * 4) + 4;
				break;
			case 3:	/* Decrement before */
				rn = rn - (bits_set * 4);
				break;
		}

		for (i = 0; i < 16; i++) {
			if (instruction.info.load_store_multiple.register_list & (1 << i)) {
				if ((!dry_run_pc) || (i == 15))
					target_read_u32(target, rn, &load_values[i]);
				rn += 4;
			}
		}

		if (dry_run_pc) {
			if (instruction.info.load_store_multiple.register_list & 0x8000) {
				*dry_run_pc = load_values[15] & ~1;
				return ERROR_OK;
			}
		} else {
			int update_cpsr = 0;

			if (instruction.info.load_store_multiple.s) {
				if (instruction.info.load_store_multiple.register_list & 0x8000)
					update_cpsr = 1;
			}

			for (i = 0; i < 16; i++) {
				if (instruction.info.load_store_multiple.register_list & (1 << i)) {
					if (i == 15) {
						uint32_t val = load_values[i];
						sim->set_reg_mode(sim, i, val & ~1);
						if (val & 1)
							sim->set_state(sim, ARM_STATE_THUMB);
						else
							sim->set_state(sim, ARM_STATE_ARM);
					} else
						sim->set_reg_mode(sim, i, load_values[i]);
				}
			}

			if (update_cpsr) {
				uint32_t spsr = sim->get_reg_mode(sim, 16);
				sim->set_reg(sim, ARMV4_5_CPSR, spsr);
			}

			/* base register writeback */
			if (instruction.info.load_store_multiple.w)
				sim->set_reg_mode(sim, instruction.info.load_store_multiple.rn, rn);


			if (instruction.info.load_store_multiple.register_list & 0x8000)
				return ERROR_OK;
		}
	}
	/* store multiple instruction */
	else if (instruction.type == ARM_STM) {
		int i;

		if (dry_run_pc) {
			/* STM wont affect PC (advance by instruction size */
		} else {
			uint32_t rn = sim->get_reg_mode(sim,
					instruction.info.load_store_multiple.rn);
			int bits_set = 0;

			for (i = 0; i < 16; i++) {
				if (instruction.info.load_store_multiple.register_list & (1 << i))
					bits_set++;
			}

			switch (instruction.info.load_store_multiple.addressing_mode) {
				case 0:	/* Increment after */
					/* rn = rn; */
					break;
				case 1:	/* Increment before */
					rn = rn + 4;
					break;
				case 2:	/* Decrement after */
					rn = rn - (bits_set * 4) + 4;
					break;
				case 3:	/* Decrement before */
					rn = rn - (bits_set * 4);
					break;
			}

			for (i = 0; i < 16; i++) {
				if (instruction.info.load_store_multiple.register_list & (1 << i)) {
					target_write_u32(target, rn, sim->get_reg_mode(sim, i));
					rn += 4;
				}
			}

			/* base register writeback */
			if (instruction.info.load_store_multiple.w)
				sim->set_reg_mode(sim,
					instruction.info.load_store_multiple.rn, rn);

		}
	} else if (!dry_run_pc) {
		/* the instruction wasn't handled, but we're supposed to simulate it
		 */
		LOG_ERROR("Unimplemented instruction, could not simulate it.");
		return ERROR_FAIL;
	}

	if (dry_run_pc) {
		*dry_run_pc = current_pc + instruction_size;
		return ERROR_OK;
	} else {
		sim->set_reg(sim, 15, current_pc + instruction_size);
		return ERROR_OK;
	}

}

static uint32_t armv4_5_get_reg(struct arm_sim_interface *sim, int reg)
{
	struct arm *arm = (struct arm *)sim->user_data;

	return buf_get_u32(arm->core_cache->reg_list[reg].value, 0, 32);
}

static void armv4_5_set_reg(struct arm_sim_interface *sim, int reg, uint32_t value)
{
	struct arm *arm = (struct arm *)sim->user_data;

	buf_set_u32(arm->core_cache->reg_list[reg].value, 0, 32, value);
}

static uint32_t armv4_5_get_reg_mode(struct arm_sim_interface *sim, int reg)
{
	struct arm *arm = (struct arm *)sim->user_data;

	return buf_get_u32(ARMV4_5_CORE_REG_MODE(arm->core_cache,
			arm->core_mode, reg).value, 0, 32);
}

static void armv4_5_set_reg_mode(struct arm_sim_interface *sim, int reg, uint32_t value)
{
	struct arm *arm = (struct arm *)sim->user_data;

	buf_set_u32(ARMV4_5_CORE_REG_MODE(arm->core_cache,
		arm->core_mode, reg).value, 0, 32, value);
}

static uint32_t armv4_5_get_cpsr(struct arm_sim_interface *sim, int pos, int bits)
{
	struct arm *arm = (struct arm *)sim->user_data;

	return buf_get_u32(arm->cpsr->value, pos, bits);
}

static enum arm_state armv4_5_get_state(struct arm_sim_interface *sim)
{
	struct arm *arm = (struct arm *)sim->user_data;

	return arm->core_state;
}

static void armv4_5_set_state(struct arm_sim_interface *sim, enum arm_state mode)
{
	struct arm *arm = (struct arm *)sim->user_data;

	arm->core_state = mode;
}

static enum arm_mode armv4_5_get_mode(struct arm_sim_interface *sim)
{
	struct arm *arm = (struct arm *)sim->user_data;

	return arm->core_mode;
}

int arm_simulate_step(struct target *target, uint32_t *dry_run_pc)
{
	struct arm *arm = target_to_arm(target);
	struct arm_sim_interface sim;

	sim.user_data = arm;
	sim.get_reg = &armv4_5_get_reg;
	sim.set_reg = &armv4_5_set_reg;
	sim.get_reg_mode = &armv4_5_get_reg_mode;
	sim.set_reg_mode = &armv4_5_set_reg_mode;
	sim.get_cpsr = &armv4_5_get_cpsr;
	sim.get_mode = &armv4_5_get_mode;
	sim.get_state = &armv4_5_get_state;
	sim.set_state = &armv4_5_set_state;

	return arm_simulate_step_core(target, dry_run_pc, &sim);
}
