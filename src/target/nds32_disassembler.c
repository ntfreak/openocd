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
#include <target/target.h>
#include "nds32_disassembler.h"

static const int enable4_bits[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

int nds32_read_opcode(struct nds32 *nds32, uint32_t address, uint32_t *value)
{
	struct target *target = nds32->target;
	uint8_t value_buf[4];

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	int retval = target_read_buffer(target, address, 4, value_buf);

	if (retval == ERROR_OK) {
		/* instructions are always big-endian */
		*value = be_to_h_u32(value_buf);

		LOG_DEBUG("address: 0x%8.8" PRIx32 ", value: 0x%8.8" PRIx32 "",
				address,
				*value);
	} else {
		*value = 0x0;
		LOG_DEBUG("address: 0x%8.8" PRIx32 " failed",
				address);
	}

	return retval;
}

static int nds32_parse_type_0(uint32_t opcode, int32_t *imm)
{
	*imm = opcode & 0x1FFFFFF;

	return ERROR_OK;
}

static int nds32_parse_type_1(uint32_t opcode, uint8_t *rt, int32_t *imm)
{
	*rt = (opcode >> 20) & 0x1F;
	*imm = opcode & 0xFFFFF;

	return ERROR_OK;
}

static int nds32_parse_type_2(uint32_t opcode, uint8_t *rt, uint8_t *ra, int32_t *imm)
{
	*rt = (opcode >> 20) & 0x1F;
	*ra = (opcode >> 15) & 0x1F;
	*imm = opcode & 0x7FFF;

	return ERROR_OK;
}

static int nds32_parse_type_3(uint32_t opcode, uint8_t *rt, uint8_t *ra,
		uint8_t *rb, int32_t *imm)
{
	*rt = (opcode >> 20) & 0x1F;
	*ra = (opcode >> 15) & 0x1F;
	*rb = (opcode >> 10) & 0x1F;
	*imm = opcode & 0x3FF;

	return ERROR_OK;
}

static int nds32_parse_type_4(uint32_t opcode, uint8_t *rt, uint8_t *ra,
		uint8_t *rb, uint8_t *rd, uint8_t *sub_opc)
{
	*rt = (opcode >> 20) & 0x1F;
	*ra = (opcode >> 15) & 0x1F;
	*rb = (opcode >> 10) & 0x1F;
	*rd = (opcode >> 5) & 0x1F;
	*sub_opc = opcode & 0x1F;

	return ERROR_OK;
}

/* LBI, LHI, LWI, LBI.bi, LHI.bi, LWI.bi */
static int nds32_parse_group_0_insn(struct nds32 *nds32, uint32_t opcode,
		uint32_t address,
		struct nds32_instruction *instruction)
{
	uint8_t opc_6;

	opc_6 = instruction->info.opc_6;

	switch (opc_6 & 0x7) {
		case 0: /* LBI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLBI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 1: /* LHI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 16; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLHI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 2: /* LWI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 15; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLWI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 4: /* LBI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLBI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 5: /* LHI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 16; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLHI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 6: /* LWI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 15; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLWI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32 "",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_1_insn(struct nds32 *nds32, uint32_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	uint8_t opc_6;

	opc_6 = instruction->info.opc_6;

	switch (opc_6 & 0x7) {
		case 0: /* SBI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSBI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 1: /* SHI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 16; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSHI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 2: /* SWI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 15; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSWI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 4: /* SBI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSBI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 5: /* SHI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 16; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSHI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 6: /* SWI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 15; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSWI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_2_insn(struct nds32 *nds32, uint32_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	uint8_t opc_6;

	opc_6 = instruction->info.opc_6;

	switch (opc_6 & 0x7) {
		case 0: /* LBSI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLBSI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 1: /* LHSI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 16; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLHSI\t$r%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 3: { /* DPREFI */
				uint8_t sub_type;
				nds32_parse_type_2(opcode, &sub_type, &(instruction->info.ra),
						&(instruction->info.imm));
				instruction->info.sub_opc = sub_type & 0xF;
				instruction->type = NDS32_INSN_MISC;
				if (sub_type & 0x10) { /* DPREFI.d */
					/* sign-extend */
					instruction->info.imm = (instruction->info.imm << 17) >> 14;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tDPREFI.d\t%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
							address,
							opcode, instruction->info.sub_opc,
							instruction->info.ra, instruction->info.imm);
				} else { /* DPREFI.w */
					/* sign-extend */
					instruction->info.imm = (instruction->info.imm << 17) >> 15;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tDPREFI.w\t%" PRIu8 ",[$r%" PRIu8 "+#%" PRId32 "]",
							address,
							opcode, instruction->info.sub_opc,
							instruction->info.ra, instruction->info.imm);
				}
			}
			break;
		case 4: /* LBSI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLBSI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 5: /* LHSI.bi */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 16; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLHSI.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 6: /* LBGP */
			nds32_parse_type_1(opcode, &(instruction->info.rt), &(instruction->info.imm));
			instruction->type = NDS32_INSN_LOAD_STORE;
			if ((instruction->info.imm >> 19) & 0x1) { /* LBSI.gp */
				instruction->info.imm = (instruction->info.imm << 13) >> 13;
				nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
				instruction->access_start += instruction->info.imm;
				instruction->access_end = instruction->access_start + 1;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tLBSI.gp\t$r%" PRIu8 ",[#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			} else { /* LBI.gp */
				instruction->info.imm = (instruction->info.imm << 13) >> 13;
				nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
				instruction->access_start += instruction->info.imm;
				instruction->access_end = instruction->access_start + 1;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tLBI.gp\t$r%" PRIu8 ",[#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			}
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_mem(struct nds32 *nds32, uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	uint32_t sub_opcode = opcode & 0x3F;
	uint32_t val_ra, val_rb;
	switch (sub_opcode >> 3) {
		case 0:
			switch (sub_opcode & 0x7) {
				case 0: /* LB */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLB\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 1: /* LH */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 2;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLH\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 2: /* LW */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLW\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 4: /* LB.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLB.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt,
							instruction->info.ra, instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 5: /* LH.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 2;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLH.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 6: /* LW.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLW.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
			}
			break;
		case 1:
			switch (sub_opcode & 0x7) {
				case 0: /* SB */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSB\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt,
							instruction->info.ra, instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 1: /* SH */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 2;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSH\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 2: /* SW */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSW\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt,
							instruction->info.ra, instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 4: /* SB.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSB.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 5: /* SH.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 2;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSH.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 6: /* SW.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSW.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
			}
			break;
		case 2:
			switch (sub_opcode & 0x7) {
				case 0: /* LBS */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLBS\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt,
							instruction->info.ra, instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 1: /* LHS */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 2;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLHS\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 3: /* DPREF */
					nds32_parse_type_3(opcode, &(instruction->info.sub_opc),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_MISC;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tDPREF\t#%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<#%" PRId32 ")]",
							address,
							opcode, instruction->info.sub_opc,
							instruction->info.ra, instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 4: /* LBS.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLBS.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 5: /* LHS.bi */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra,
							&(instruction->access_start));
					instruction->access_end = instruction->access_start + 2;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLHS.bi\t$r%" PRIu8 ",[$r%" PRIu8 "],($r%" PRIu8 "<<%" PRId32 ")",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
			}
			break;
		case 3:
			switch (sub_opcode & 0x7) {
				case 0: /* LLW */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLLW\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 1: /* SCW */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSCW\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
			}
			break;
		case 4:
			switch (sub_opcode & 0x7) {
				case 0: /* LBUP */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLBUP\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 2: /* LWUP */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tLWUP\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
			}
			break;
		case 5:
			switch (sub_opcode & 0x7) {
				case 0: /* SBUP */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 1;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSBUP\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
				case 2: /* SWUP */
					nds32_parse_type_3(opcode, &(instruction->info.rt),
							&(instruction->info.ra),
							&(instruction->info.rb), &(instruction->info.imm));
					instruction->type = NDS32_INSN_LOAD_STORE;
					nds32_get_mapped_reg(nds32, instruction->info.ra, &val_ra);
					nds32_get_mapped_reg(nds32, instruction->info.rb, &val_rb);
					instruction->access_start = val_ra +
						(val_rb << ((instruction->info.imm >> 8) & 0x3));
					instruction->access_end = instruction->access_start + 4;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tSWUP\t$r%" PRIu8 ",[$r%" PRIu8 "+($r%" PRIu8 "<<%" PRId32 ")]",
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.rb,
							(instruction->info.imm >> 8) & 0x3);
					break;
			}
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_calculate_lsmw_access_range(struct nds32 *nds32,
		struct nds32_instruction *instruction)
{
	uint8_t ba;
	uint8_t id;
	uint8_t enable4;

	enable4 = (instruction->info.imm >> 6) & 0xF;
	ba = (instruction->info.imm >> 4) & 0x1;
	id = (instruction->info.imm >> 3) & 0x1;

	if (ba) {
		nds32_get_mapped_reg(nds32, instruction->info.ra, &(instruction->access_start));
		if (id) { /* decrease */
			/* access_end is the (last_element+1), so no need to minus 4 */
			/* instruction->access_end -= 4; */
			instruction->access_end = instruction->access_start;
		} else { /* increase */
			instruction->access_start += 4;
		}
	} else {
		nds32_get_mapped_reg(nds32, instruction->info.ra, &(instruction->access_start));
		instruction->access_end = instruction->access_start - 4;
	}

	if (id) { /* decrease */
		instruction->access_start = instruction->access_end -
			4 * (instruction->info.rd - instruction->info.rb + 1);
		instruction->access_start -= (4 * enable4_bits[enable4]);
	} else { /* increase */
		instruction->access_end = instruction->access_start +
			4 * (instruction->info.rd - instruction->info.rb + 1);
		instruction->access_end += (4 * enable4_bits[enable4]);
	}

	return ERROR_OK;
}

static int nds32_parse_lsmw(struct nds32 *nds32, uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	if (opcode & 0x20) { /* SMW, SMWA, SMWZB */
		switch (opcode & 0x3) {
			/* TODO */
			case 0: /* SMW */
				/* use rd as re */
				nds32_parse_type_3(opcode, &(instruction->info.rb),
						&(instruction->info.ra),
						&(instruction->info.rd), &(instruction->info.imm));
				instruction->type = NDS32_INSN_LOAD_STORE;
				nds32_calculate_lsmw_access_range(nds32, instruction);
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSMW\t$r%" PRIu8 ",[$r%" PRIu8 "],$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rb, instruction->info.ra,
						instruction->info.rd,
						(instruction->info.imm >> 6) & 0xF);
				break;
			case 1: /* SMWA */
				nds32_parse_type_3(opcode, &(instruction->info.rb),
						&(instruction->info.ra),
						&(instruction->info.rd), &(instruction->info.imm));
				instruction->type = NDS32_INSN_LOAD_STORE;
				nds32_calculate_lsmw_access_range(nds32, instruction);
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSMWA\t$r%" PRIu8 ",[$r%" PRIu8 "],$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rb, instruction->info.ra,
						instruction->info.rd,
						(instruction->info.imm >> 6) & 0xF);
				break;
			case 2: /* SMWZB */
				nds32_parse_type_3(opcode, &(instruction->info.rb),
						&(instruction->info.ra),
						&(instruction->info.rd), &(instruction->info.imm));
				instruction->type = NDS32_INSN_LOAD_STORE;
				/* TODO: calculate access_start/access_end */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSMWZB\t$r%" PRIu8 ",[$r%" PRIu8 "],$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rb, instruction->info.ra,
						instruction->info.rd,
						(instruction->info.imm >> 6) & 0xF);
				break;
			default:
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
						address,
						opcode);
				return ERROR_FAIL;
		}
	} else { /* LMW, LMWA, LMWZB */
		switch (opcode & 0x3) {
			case 0: /* LMW */
				nds32_parse_type_3(opcode, &(instruction->info.rb),
						&(instruction->info.ra),
						&(instruction->info.rd), &(instruction->info.imm));
				instruction->type = NDS32_INSN_LOAD_STORE;
				nds32_calculate_lsmw_access_range(nds32, instruction);
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tLMW\t$r%" PRIu8 ",[$r%" PRIu8 "],$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rb, instruction->info.ra,
						instruction->info.rd,
						(instruction->info.imm >> 6) & 0xF);
				break;
			case 1: /* LMWA */
				nds32_parse_type_3(opcode, &(instruction->info.rb),
						&(instruction->info.ra),
						&(instruction->info.rd), &(instruction->info.imm));
				instruction->type = NDS32_INSN_LOAD_STORE;
				nds32_calculate_lsmw_access_range(nds32, instruction);
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tLMWA\t$r%" PRIu8 ",[$r%" PRIu8 "],$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rb, instruction->info.ra,
						instruction->info.rd,
						(instruction->info.imm >> 6) & 0xF);
				break;
			case 2: /* LMWZB */
				nds32_parse_type_3(opcode, &(instruction->info.rb),
						&(instruction->info.ra),
						&(instruction->info.rd), &(instruction->info.imm));
				instruction->type = NDS32_INSN_LOAD_STORE;
				/* TODO: calculate access_start/access_end */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tLMWZB\t$r%" PRIu8 ",[$r%" PRIu8 "],$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rb, instruction->info.ra,
						instruction->info.rd,
						(instruction->info.imm >> 6) & 0xF);
				break;
			default:
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
						address,
						opcode);
				return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int nds32_parse_hwgp(struct nds32 *nds32, uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	switch ((opcode >> 18) & 0x3) {
		case 0: /* LHI.gp */
			nds32_parse_type_1(opcode, &(instruction->info.rt), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 14) >> 13; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLHI.gp\t$r%" PRIu8 ",[#%" PRId32"]",
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 1: /* LHSI.gp */
			nds32_parse_type_1(opcode, &(instruction->info.rt), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 14) >> 13; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tLHSI.gp\t$r%" PRIu8 ",[#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 2: /* SHI.gp */
			nds32_parse_type_1(opcode, &(instruction->info.rt), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 14) >> 13; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSHI.gp\t$r%" PRIu8 ",[#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 3:
			instruction->type = NDS32_INSN_LOAD_STORE;
			if ((opcode >> 17) & 0x1) { /* SWI.gp */
				nds32_parse_type_1(opcode, &(instruction->info.rt),
						&(instruction->info.imm));
				/* sign-extend */
				instruction->info.imm = (instruction->info.imm << 15) >> 13;
				nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
				instruction->access_start += instruction->info.imm;
				instruction->access_end = instruction->access_start + 4;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSWI.gp\t$r%" PRIu8 ",[#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			} else { /* LWI.gp */
				nds32_parse_type_1(opcode, &(instruction->info.rt),
						&(instruction->info.imm));
				/* sign-extend */
				instruction->info.imm = (instruction->info.imm << 15) >> 13;
				nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
				instruction->access_start += instruction->info.imm;
				instruction->access_end = instruction->access_start + 4;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tLWI.gp\t$r%" PRIu8 ",[#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			}

			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_sbgp(struct nds32 *nds32, uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	switch ((opcode >> 19) & 0x1) {
		case 0: /* SBI.gp */
			nds32_parse_type_1(opcode, &(instruction->info.rt), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 13) >> 13; /* sign-extend */
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, R29, &(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSBI.gp\t$r%" PRIu8 ",[#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 1: /* ADDI.gp */
			nds32_parse_type_1(opcode, &(instruction->info.rt), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 13) >> 13; /* sign-extend */
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tADDI.gp\t$r%" PRIu8 ",#%" PRId32 "",
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_3_insn(struct nds32 *nds32, uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	uint8_t opc_6;

	opc_6 = instruction->info.opc_6;

	switch (opc_6 & 0x7) {
		case 4: /* MEM */
			nds32_parse_mem(nds32, opcode, address, instruction);
			break;
		case 5: /* LSMW */
			nds32_parse_lsmw(nds32, opcode, address, instruction);
			break;
		case 6: /* HWGP */
			nds32_parse_hwgp(nds32, opcode, address, instruction);
			break;
		case 7: /* SBGP */
			nds32_parse_sbgp(nds32, opcode, address, instruction);
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_alu_1(uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	switch (opcode & 0x1F) {
		case 0: /* ADD */
			nds32_parse_type_3(opcode, &(instruction->info.rt), &(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tADD_SLLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tADD\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			break;
		case 1: /* SUB */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSUB_SLLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSUB\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 "",
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			break;
		case 2: /* AND */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tAND_SLLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tAND\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 "",
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			break;
		case 3: /* XOR */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tXOR_SLLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tXOR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			break;
		case 4: /* OR */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tOR_SLLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tOR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			break;
		case 5: /* NOR */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tNOR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.rb);
			break;
		case 6: /* SLT */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSLT\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.rb);
			break;
		case 7: /* SLTS */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSLTS\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.rb);
			break;
		case 8: { /* SLLI */
				uint8_t imm;
				int32_t sub_op;
				nds32_parse_type_3(opcode, &(instruction->info.rt),
						&(instruction->info.ra),
						&imm, &sub_op);
				instruction->info.imm = imm;
				instruction->type = NDS32_INSN_DATA_PROC;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSLLI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			}
			break;
		case 9: { /* SRLI */
				uint8_t imm;
				int32_t sub_op;
				nds32_parse_type_3(opcode, &(instruction->info.rt),
						&(instruction->info.ra),
						&imm, &sub_op);
				instruction->info.imm = imm;
				instruction->type = NDS32_INSN_DATA_PROC;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSRLI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			}
			break;
		case 10: { /* SRAI */
				 uint8_t imm;
				 int32_t sub_op;
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &imm, &sub_op);
				 instruction->info.imm = imm;
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSRAI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.imm);
			 }
			 break;
		case 11: { /* ROTRI */
				 uint8_t imm;
				 int32_t sub_op;
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &imm, &sub_op);
				 instruction->info.imm = imm;
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tROTRI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.imm);
			 }
			 break;
		case 12: { /* SLL */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSLL\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 13: { /* SRL */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSRL\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 14: { /* SRA */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSRA\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 15: { /* ROTR */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tROTR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 16: { /* SEB */
				 nds32_parse_type_2(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSEB\t$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra);
			 }
			 break;
		case 17: { /* SEH */
				 nds32_parse_type_2(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSEH\t$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra);
			 }
			 break;
		case 18: /* BITC */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tBITC\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.rb);
			 break;
		case 19: { /* ZEH */
				 nds32_parse_type_2(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tZEH\t$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra);
			 }
			 break;
		case 20: { /* WSBH */
				 nds32_parse_type_2(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tWSBH\t$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra);
			 }
			 break;
		case 21: /* OR_SRLI */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tOR_SRLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tOR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			 break;
		case 22: { /* DIVSR */
				 nds32_parse_type_4(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.rd),
						 &(instruction->info.sub_opc));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tDIVSR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb,
						 instruction->info.rd);
			 }
			 break;
		case 23: { /* DIVR */
				 nds32_parse_type_4(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.rd),
						 &(instruction->info.sub_opc));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tDIVR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb,
						 instruction->info.rd);
			 }
			 break;
		case 24: { /* SVA */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSVA\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 25: { /* SVS */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tSVS\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 26: { /* CMOVZ */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_MISC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tCMOVZ\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 27: { /* CMOVN */
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_MISC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tCMOVN\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 28: /* ADD_SRLI */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tADD_SRLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tADD\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			 break;
		case 29: /* SUB_SRLI */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSUB_SRLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tSUB\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			 break;
		case 30: /* AND_SRLI */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tAND_SRLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tAND\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			 break;
		case 31: /* XOR_SRLI */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			instruction->info.imm = (instruction->info.imm >> 5) & 0x1F;
			if (instruction->info.imm)
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tXOR_SRLI\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8 ",%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb,
						instruction->info.imm);
			else
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tXOR\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			 break;
		default:
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					 address,
					 opcode);
			 return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_alu_2(uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	switch (opcode & 0x3F) {
		case 0: /* MAX */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tMAX\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.rb);
			break;
		case 1: /* MIN */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tMIN\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.rb);
			break;
		case 2: /* AVE */
			nds32_parse_type_3(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.rb), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tAVE\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.rb);
			break;
		case 3: /* ABS */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tAVE\t$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra);
			break;
		case 4: { /* CLIPS */
				uint8_t imm;
				nds32_parse_type_3(opcode, &(instruction->info.rt),
						&(instruction->info.ra),
						&imm, &(instruction->info.imm));
				instruction->info.imm = imm;
				instruction->type = NDS32_INSN_DATA_PROC;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tCLIPS\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			}
			break;
		case 5: { /* CLIP */
				uint8_t imm;
				nds32_parse_type_3(opcode, &(instruction->info.rt),
						&(instruction->info.ra),
						&imm, &(instruction->info.imm));
				instruction->info.imm = imm;
				instruction->type = NDS32_INSN_DATA_PROC;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tCLIP\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			}
			break;
		case 6: /* CLO */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tCLO\t$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra);
			break;
		case 7: /* CLZ */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra),
					&(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tCLZ\t$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra);
			break;
		case 8: { /* BSET */
				uint8_t imm;
				nds32_parse_type_3(opcode, &(instruction->info.rt),
						&(instruction->info.ra),
						&imm, &(instruction->info.imm));
				instruction->info.imm = imm;
				instruction->type = NDS32_INSN_DATA_PROC;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tBSET\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			}
			break;
		case 9: { /* BCLR */
				uint8_t imm;
				nds32_parse_type_3(opcode, &(instruction->info.rt),
						&(instruction->info.ra),
						&imm, &(instruction->info.imm));
				instruction->info.imm = imm;
				instruction->type = NDS32_INSN_DATA_PROC;
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tBCLR\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			}
			break;
		case 10: { /* BTGL */
				 uint8_t imm;
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &imm, &(instruction->info.imm));
				 instruction->info.imm = imm;
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tBTGL\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.imm);
			 }
			 break;
		case 11: { /* BTST */
				 uint8_t imm;
				 nds32_parse_type_3(opcode, &(instruction->info.rt),
						 &(instruction->info.ra),
						 &imm, &(instruction->info.imm));
				 instruction->info.imm = imm;
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tBTST\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						 address,
						 opcode, instruction->info.rt, instruction->info.ra,
						 instruction->info.imm);
			 }
			 break;
		case 12: /* BSE */
			 nds32_parse_type_3(opcode, &(instruction->info.rt),
					 &(instruction->info.ra),
					 &(instruction->info.rb), &(instruction->info.imm));
			 instruction->type = NDS32_INSN_DATA_PROC;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tBSE\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					 address,
					 opcode, instruction->info.rt, instruction->info.ra,
					 instruction->info.rb);
			 break;
		case 13: /* BSP */
			 nds32_parse_type_3(opcode, &(instruction->info.rt),
					 &(instruction->info.ra),
					 &(instruction->info.rb), &(instruction->info.imm));
			 instruction->type = NDS32_INSN_DATA_PROC;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tBSP\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					 address,
					 opcode, instruction->info.rt, instruction->info.ra,
					 instruction->info.rb);
			 break;
		case 14: /* FFB */
			 nds32_parse_type_3(opcode, &(instruction->info.rt),
					 &(instruction->info.ra),
					 &(instruction->info.rb), &(instruction->info.imm));
			 instruction->type = NDS32_INSN_DATA_PROC;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tFFB\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					 address,
					 opcode, instruction->info.rt, instruction->info.ra,
					 instruction->info.rb);
			 break;
		case 15: /* FFMISM */
			 nds32_parse_type_3(opcode, &(instruction->info.rt),
					 &(instruction->info.ra),
					 &(instruction->info.rb), &(instruction->info.imm));
			 instruction->type = NDS32_INSN_DATA_PROC;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tFFMISM\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					 address,
					 opcode, instruction->info.rt, instruction->info.ra,
					 instruction->info.rb);
			 break;
		case 23: /* FFZMISM */
			 nds32_parse_type_3(opcode, &(instruction->info.rt),
					 &(instruction->info.ra),
					 &(instruction->info.rb), &(instruction->info.imm));
			 instruction->type = NDS32_INSN_DATA_PROC;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tFFZMISM\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					 address,
					 opcode, instruction->info.rt, instruction->info.ra,
					 instruction->info.rb);
			 break;
		case 32: /* MFUSR */
			 nds32_parse_type_1(opcode, &(instruction->info.rt),
					 &(instruction->info.imm));
			 instruction->type = NDS32_INSN_RESOURCE_ACCESS;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tMFUSR\t$r%" PRIu8 ",#%" PRId32,
					 address,
					 opcode, instruction->info.rt,
					 (instruction->info.imm >> 10) & 0x3FF);
			 break;
		case 33: /* MTUSR */
			 nds32_parse_type_1(opcode, &(instruction->info.rt),
					 &(instruction->info.imm));
			 instruction->type = NDS32_INSN_RESOURCE_ACCESS;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tMTUSR\t$r%" PRIu8 ",#%" PRId32,
					 address,
					 opcode, instruction->info.rt,
					 (instruction->info.imm >> 10) & 0x3FF);
			 break;
		case 36: /* MUL */
			 nds32_parse_type_3(opcode, &(instruction->info.rt),
					 &(instruction->info.ra),
					 &(instruction->info.rb), &(instruction->info.imm));
			 instruction->type = NDS32_INSN_DATA_PROC;
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					 "\tMUL\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
					 address,
					 opcode, instruction->info.rt, instruction->info.ra,
					 instruction->info.rb);
			 break;
		case 40: { /* MULTS64 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val,
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMULTS64\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 41: { /* MULT64 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val,
						 &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMULT64\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 42: { /* MADDS64 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMADDS64\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 43: { /* MADD64 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMADD64\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 44: { /* MSUBS64 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMSUBS64\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 45: { /* MSUB64 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMSUB64\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 46: { /* DIVS */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tDIVS\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 47: { /* DIV */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tDIV\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 49: { /* MULT32 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMULT32\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 51: { /* MADD32 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMADD32\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		case 53: { /* MSUB32 */
				 uint8_t dt_val;
				 nds32_parse_type_3(opcode, &dt_val, &(instruction->info.ra),
						 &(instruction->info.rb), &(instruction->info.imm));
				 instruction->type = NDS32_INSN_DATA_PROC;
				 snprintf(instruction->text,
						 128,
						 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						 "\tMSUB32\t$D%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						 address,
						 opcode, (uint8_t)((dt_val >> 1) & 0x1), instruction->info.ra,
						 instruction->info.rb);
			 }
			 break;
		default:
			 snprintf(instruction->text,
					 128,
					 "0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					 address,
					 opcode);
			 return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_4_insn(struct nds32 *nds32, uint32_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	uint8_t opc_6;

	opc_6 = instruction->info.opc_6;

	switch (opc_6 & 0x7) {
		case 0: /* ALU_1 */
			nds32_parse_alu_1(opcode, address, instruction);
			break;
		case 1: /* ALU_2 */
			nds32_parse_alu_2(opcode, address, instruction);
			break;
		case 2: /* MOVI */
			nds32_parse_type_1(opcode, &(instruction->info.rt),
					&(instruction->info.imm));
			/* sign-extend */
			instruction->info.imm = (instruction->info.imm << 12) >> 12;
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tMOVI\t$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 3: /* SETHI */
			nds32_parse_type_1(opcode, &(instruction->info.rt),
					&(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSETHI\t$r%" PRIu8 ",0x%8.8" PRIx32,
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 4: /* JI */
			nds32_parse_type_0(opcode, &(instruction->info.imm));
			/* sign-extend */
			instruction->info.imm = (instruction->info.imm << 8) >> 8;
			instruction->type = NDS32_INSN_JUMP_BRANCH;
			if ((instruction->info.imm >> 24) & 0x1) { /* JAL */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tJAL\t#%" PRId32,
						address,
						opcode, instruction->info.imm);
			} else { /* J */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
						"\tJ\t#%" PRId32,
						address,
						opcode, instruction->info.imm);
			}
			break;
		case 5: { /* JREG */
				int32_t imm;
				nds32_parse_type_0(opcode, &imm);
				instruction->info.rb = (imm >> 10) & 0x1F;
				instruction->type = NDS32_INSN_JUMP_BRANCH;
				switch (imm & 0x1F) {
					/* TODO */
					case 0: /* JR */
						if (imm & 0x20) { /* RET */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
									"\tRET\t$r%" PRIu8,
									address,
									opcode, instruction->info.rb);
						} else { /* JR */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
									"\tJR\t$r%" PRIu8,
									address,
									opcode, instruction->info.rb);
						}
						break;
					case 1: /* JRAL */
						instruction->info.rt = (imm >> 20) & 0x1F;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tJRAL\t$r%" PRIu8 ",$r%" PRIu8,
								address,
								opcode, instruction->info.rt, instruction->info.rb);
						break;
					case 2: /* JRNEZ */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tJRNEZ\t$r%" PRIu8,
								address,
								opcode, instruction->info.rb);
						break;
					case 3: /* JRALNEZ */
						instruction->info.rt = (imm >> 20) & 0x1F;
						if (instruction->info.rt == R30)
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
									"\tJRALNEZ\t$r%" PRIu8,
									address,
									opcode, instruction->info.rb);
						else
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
									"\tJRALNEZ\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode,
									instruction->info.rt,
									instruction->info.rb);
						break;
				}
			}
			break;
		case 6: { /* BR1 */
				int32_t imm;

				nds32_parse_type_0(opcode, &imm);
				instruction->type = NDS32_INSN_JUMP_BRANCH;
				if ((imm >> 14) & 0x1) { /* BNE */
					nds32_parse_type_2(opcode, &(instruction->info.rt),
							&(instruction->info.ra), &(instruction->info.imm));
					/* sign-extend */
					instruction->info.imm = (instruction->info.imm << 18) >> 18;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tBNE\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
							address,
							opcode, instruction->info.rt, instruction->info.ra,
							instruction->info.imm);
				} else { /* BEQ */
					nds32_parse_type_2(opcode, &(instruction->info.rt),
							&(instruction->info.ra), &(instruction->info.imm));
					/* sign-extend */
					instruction->info.imm = (instruction->info.imm << 18) >> 18;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
							"\tBEQ\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
							address,
							opcode, instruction->info.rt,
							instruction->info.ra,
							instruction->info.imm);
				}
			}
			break;
		case 7: { /* BR2 */
				int32_t imm;

				nds32_parse_type_0(opcode, &imm);
				instruction->type = NDS32_INSN_JUMP_BRANCH;
				switch ((imm >> 16) & 0xF) {
					case 2: /* BEQZ */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBEQZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 3: /* BNEZ */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBNEZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 4: /* BGEZ */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBGEZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 5: /* BLTZ */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBLTZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 6: /* BGTZ */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBGTZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 7: /* BLEZ */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBLEZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 12: /* BGEZAL */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBGEZAL\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 13: /* BLTZAL */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->info.imm = (instruction->info.imm << 16) >> 16;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBLTZAL\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
				}
			}
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_5_insn(struct nds32 *nds32, uint32_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	uint8_t opc_6;

	opc_6 = instruction->info.opc_6;

	switch (opc_6 & 0x7) {
		case 0: /* ADDI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tADDI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 1: /* SUBRI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSUBRI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 2: /* ANDI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tANDI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 3: /* XORI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tXORI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 4: /* ORI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tORI\t$r%" PRIu8 ",$r%" PRIu8 ",0x%8.8" PRIx32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 6: /* SLTI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSLTI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 7: /* SLTSI */
			nds32_parse_type_2(opcode, &(instruction->info.rt),
					&(instruction->info.ra), &(instruction->info.imm));
			instruction->info.imm = (instruction->info.imm << 17) >> 17; /* sign-extend */
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
					"\tSLTSI\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_6_insn(struct nds32 *nds32, uint32_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	uint8_t opc_6;

	opc_6 = instruction->info.opc_6;

	switch (opc_6 & 0x7) {
		case 2: { /* MISC */
				int32_t imm;
				uint8_t sub_opc;

				nds32_parse_type_0(opcode, &imm);

				sub_opc = imm & 0x1F;
				switch (sub_opc) {
					case 0: /* STANDBY */
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tSTANDBY\t#%" PRIu32,
								address,
								opcode, (opcode >> 5) & 0x3);
						break;
					case 1: /* CCTL */
						/* TODO */
						nds32_parse_type_2(opcode, &(instruction->info.rt),
								&(instruction->info.ra), &(instruction->info.imm));
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tCCTL",
								address,
								opcode);
						break;
					case 2: /* MFSR */
						nds32_parse_type_1(opcode, &(instruction->info.rt),
								&(instruction->info.imm));
						instruction->type = NDS32_INSN_RESOURCE_ACCESS;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tMFSR\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.rt,
								(instruction->info.imm >> 10) & 0x3FF);
						break;
					case 3: /* MTSR */
						nds32_parse_type_1(opcode, &(instruction->info.ra),
								&(instruction->info.imm));
						instruction->type = NDS32_INSN_RESOURCE_ACCESS;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tMTSR\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.ra,
								(instruction->info.imm >> 10) & 0x3FF);
						break;
					case 4: /* IRET */
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tIRET",
								address,
								opcode);
						break;
					case 5: /* TRAP */
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tTRAP\t#%" PRId32,
								address,
								opcode, (imm >> 5) & 0x7FFF);
						break;
					case 6: /* TEQZ */
						nds32_parse_type_1(opcode, &(instruction->info.ra),
								&(instruction->info.imm));
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tTEQZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.ra,
								(instruction->info.imm >> 5) & 0x7FFF);
						break;
					case 7: /* TNEZ */
						nds32_parse_type_1(opcode, &(instruction->info.ra),
								&(instruction->info.imm));
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tTNEZ\t$r%" PRIu8 ",#%" PRId32,
								address,
								opcode, instruction->info.ra,
								(instruction->info.imm >> 5) & 0x7FFF);
						break;
					case 8: /* DSB */
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tDSB",
								address,
								opcode);
						break;
					case 9: /* ISB */
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tISB",
								address,
								opcode);
						break;
					case 10: /* BREAK */
						instruction->type = NDS32_INSN_MISC;
						instruction->info.sub_opc = imm & 0x1F;
						instruction->info.imm = (imm >> 5) & 0x7FFF;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tBREAK\t#%" PRId32,
								address,
								opcode, instruction->info.imm);
						break;
					case 11: /* SYSCALL */
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tSYSCALL\t#%" PRId32,
								address,
								opcode, (imm >> 5) & 0x7FFF);
						break;
					case 12: /* MSYNC */
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tMSYNC\t#%" PRId32,
								address,
								opcode, (imm >> 5) & 0x7);
						break;
					case 13: /* ISYNC */
						nds32_parse_type_1(opcode, &(instruction->info.ra),
								&(instruction->info.imm));
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32
								"\tISYNC\t$r%" PRIu8,
								address,
								opcode, instruction->info.ra);
						break;
					case 14: /* TLBOP */
						/* TODO */
						nds32_parse_type_2(opcode, &(instruction->info.rt),
								&(instruction->info.ra), &(instruction->info.imm));
						instruction->type = NDS32_INSN_RESOURCE_ACCESS;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tTLBOP",
								address,
								opcode);
						break;
				}

				break;
			}
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static uint32_t field_mask[9] = {
	0x0,
	0x1,
	0x3,
	0x7,
	0xF,
	0x1F,
	0x3F,
	0x7F,
	0xFF,
};

static uint8_t nds32_extract_field_8u(uint16_t opcode, uint32_t start, uint32_t length)
{
	if (0 < length && length < 9)
		return (opcode >> start) & field_mask[length];

	return 0;
}

static int nds32_parse_group_0_insn_16(struct nds32 *nds32, uint16_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	switch ((opcode >> 10) & 0x7) {
		case 0: /* MOV55 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 5);
			instruction->info.ra = nds32_extract_field_8u(opcode, 0, 5);
			instruction->type = NDS32_INSN_MISC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tMOV55\t$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra);
			break;
		case 1: /* MOVI55 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 5);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5);
			instruction->info.imm = (instruction->info.imm << 27) >> 27;
			instruction->type = NDS32_INSN_MISC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tMOVI55\t$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 2: /* ADD45, SUB45 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 4);
			instruction->info.rb = nds32_extract_field_8u(opcode, 0, 5);
			instruction->type = NDS32_INSN_DATA_PROC;
			if (nds32_extract_field_8u(opcode, 9, 1) == 0) { /* ADD45 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tADD45\t$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.rb);
			} else { /* SUB45 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSUB45\t$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.rb);
			}

			break;
		case 3: /* ADDI45, SUBI45 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 4);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5);
			instruction->type = NDS32_INSN_DATA_PROC;
			if (nds32_extract_field_8u(opcode, 9, 1) == 0) { /* ADDI45 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tADDI45\t$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			} else { /* SUBI45 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSUBI45\t$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			}
			break;
		case 4: /* SRAI45, SRLI45 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 4);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5);
			instruction->type = NDS32_INSN_DATA_PROC;
			if (nds32_extract_field_8u(opcode, 9, 1) == 0) { /* SRAI45 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSRAI45\t$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			} else { /* SRLI45 */
				if ((instruction->info.rt == 0) && (instruction->info.imm == 0)) {
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16 "\t\tNOP",
							address,
							opcode);
				} else {
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tSRLI45\t$r%" PRIu8 ",#%" PRId32,
							address,
							opcode, instruction->info.rt, instruction->info.imm);
				}
			}
			break;
		case 5:
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->type = NDS32_INSN_DATA_PROC;
			if (nds32_extract_field_8u(opcode, 9, 1) == 0) { /* SLLI333 */
				instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3);
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSLLI333\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			} else {
				instruction->info.sub_opc = nds32_extract_field_8u(opcode, 0, 3);
				switch (instruction->info.sub_opc) {
					case 0: /* ZEB33 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tZEB33\t$r%" PRIu8 ",$r%" PRIu8,
								address,
								opcode, instruction->info.rt, instruction->info.ra);
						break;
					case 1: /* ZEH33 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tZEH33\t$r%" PRIu8 ",$r%" PRIu8,
								address,
								opcode, instruction->info.rt, instruction->info.ra);
						break;
					case 2: /* SEB33 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tSEB33\t$r%" PRIu8 ",$r%" PRIu8,
								address,
								opcode, instruction->info.rt, instruction->info.ra);
						break;
					case 3: /* SEH33 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tSEH33\t$r%" PRIu8 ",$r%" PRIu8,
								address,
								opcode, instruction->info.rt, instruction->info.ra);
						break;
					case 4: /* XLSB33 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tXLSB33\t$r%" PRIu8 ",$r%" PRIu8,
								address,
								opcode, instruction->info.rt, instruction->info.ra);
						break;
					case 5: /* XLLB33 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tXLLB33\t$r%" PRIu8 ",$r%" PRIu8,
								address,
								opcode, instruction->info.rt, instruction->info.ra);
						break;
					case 6: /* BMSKI33 */
						instruction->info.ra = 0;
						instruction->info.imm = nds32_extract_field_8u(opcode, 3, 3);
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tBMSKI33\t$r%" PRIu8 ",$r%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					case 7: /* FEXTI33 */
						instruction->info.ra = 0;
						instruction->info.imm = nds32_extract_field_8u(opcode, 3, 3);
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tFEXTI33\t$r%" PRIu8 ",$r%" PRId32,
								address,
								opcode, instruction->info.rt, instruction->info.imm);
						break;
					default:
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx16
								"\tUNDEFINED INSTRUCTION",
								address,
								opcode);
						return ERROR_FAIL;
				}
			}
			break;
		case 6: /* ADD333, SUB333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.rb = nds32_extract_field_8u(opcode, 0, 3);
			instruction->type = NDS32_INSN_DATA_PROC;
			if (nds32_extract_field_8u(opcode, 9, 1) == 0) { /* ADD333 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tADD333\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			} else { /* SUB333 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSUB333\t$r%" PRIu8 ",$r%" PRIu8 ",$r%" PRIu8,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.rb);
			}
			break;
		case 7: /* ADDI333, SUBI333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3);
			instruction->type = NDS32_INSN_DATA_PROC;
			if (nds32_extract_field_8u(opcode, 9, 1) == 0) { /* ADDI333 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tADDI333\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			} else { /* SUBI333 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSUBI333\t$r%" PRIu8 ",$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.ra,
						instruction->info.imm);
			}
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx16 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_1_insn_16(struct nds32 *nds32, uint16_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	switch ((opcode >> 9) & 0xF) {
		case 0: /* LWI333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3) << 2;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tLWI333\t$r%" PRIu8 ",[$r%" PRIu8 "+(#%" PRId32 ")]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 1: /* LWI333.BI */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3);
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tLWI333.BI\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm << 2);
			break;
		case 2: /* LHI333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3) << 1;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tLHI333\t$r%" PRIu8 ",[$r%" PRIu8 "+(#%" PRId32 ")]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 3: /* LBI333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3);
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tLBI333\t$r%" PRIu8 ",[$r%" PRIu8 "+(#%" PRId32 ")]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 4: /* SWI333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3) << 2;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tSWI333\t$r%" PRIu8 ",[$r%" PRIu8 "+(#%" PRId32 ")]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 5: /* SWI333.BI */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3) << 2;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tSWI333.BI\t$r%" PRIu8 ",[$r%" PRIu8 "],#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 6: /* SHI333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3) << 1;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 2;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tSHI333\t$r%" PRIu8 ",[$r%" PRIu8 "+(#%" PRId32 ")]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 7: /* SBI333 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 3);
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 1;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tSHI333\t$r%" PRIu8 ",[$r%" PRIu8 "+(#%" PRId32 ")]",
					address,
					opcode, instruction->info.rt, instruction->info.ra,
					instruction->info.imm);
			break;
		case 8: /* ADDRI36.SP */
			instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 6) << 2;
			instruction->type = NDS32_INSN_DATA_PROC;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tADDRI36.SP\t$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 9: /* LWI45.FE */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 4);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5);
			instruction->info.imm -= 32;
			instruction->info.imm <<= 2;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, R8, &(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tLWI45.FE\t$r%" PRIu8 ",[#%" PRId32 "]",
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 10: /* LWI450 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 4);
			instruction->info.ra = nds32_extract_field_8u(opcode, 0, 5);
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tLWI450\t$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra);
			break;
		case 11: /* SWI450 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 5, 4);
			instruction->info.ra = nds32_extract_field_8u(opcode, 0, 5);
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, instruction->info.ra,
					&(instruction->access_start));
			instruction->access_end = instruction->access_start + 4;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tSWI450\t$r%" PRIu8 ",$r%" PRIu8,
					address,
					opcode, instruction->info.rt, instruction->info.ra);
			break;
		case 12:
		case 13:
		case 14:
		case 15: /* LWI37, SWI37 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 8, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 7) << 2;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, R28, &(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 4;
			if (nds32_extract_field_8u(opcode, 7, 1) == 0) { /* LWI37 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tLWI37\t$r%" PRIu8 ",[fp+#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			} else { /* SWI37 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSWI37\t$r%" PRIu8 ",[fp+#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			}
			break;
		default: /* ERROR */
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx16 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int nds32_parse_group_2_insn_16(struct nds32 *nds32, uint16_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	switch ((opcode >> 11) & 0x3) {
		case 0: /* BEQZ38 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 8, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 8);
			instruction->info.imm = (instruction->info.imm << 24) >> 24;
			instruction->type = NDS32_INSN_JUMP_BRANCH;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tBEQZ38\t$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 1: /* BNEZ38 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 8, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 8);
			instruction->info.imm = (instruction->info.imm << 24) >> 24;
			instruction->type = NDS32_INSN_JUMP_BRANCH;
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
					"\t\tBNEZ38\t$r%" PRIu8 ",#%" PRId32,
					address,
					opcode, instruction->info.rt, instruction->info.imm);
			break;
		case 2: /* BEQS38,J8 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 8, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 8);
			instruction->info.imm = (instruction->info.imm << 24) >> 24;
			instruction->type = NDS32_INSN_JUMP_BRANCH;
			if (instruction->info.rt == 5) { /* J8 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tJ8\t#%" PRId32,
						address,
						opcode, instruction->info.imm);
			} else { /* BEQS38 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tBEQS38\t$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			}
			break;
		case 3: /* BNES38, JR5, RET5, JRAL5 */
			instruction->info.rt = nds32_extract_field_8u(opcode, 8, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 8);
			instruction->info.imm = (instruction->info.imm << 24) >> 24;
			instruction->type = NDS32_INSN_JUMP_BRANCH;
			if (instruction->info.rt == 5) {
				instruction->info.imm = 0;
				instruction->info.rb = nds32_extract_field_8u(opcode, 0, 5);
				switch (nds32_extract_field_8u(opcode, 5, 3)) {
					case 0: /* JR5 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tJR5\t$r%" PRIu8,
								address,
								opcode, instruction->info.rb);
						break;
					case 1: /* JRAL5 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tJRAL5\t$r%" PRIu8,
								address,
								opcode, instruction->info.rb);
						break;
					case 2: /* EX9.IT */
						instruction->info.rb = 0;
						instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5);
						/* TODO: implement real instruction semantics */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tEX9.IT\t#%" PRId32,
								address,
								opcode, instruction->info.imm);
						break;
					case 4: /* RET5 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tRET5\t$r%" PRIu8,
								address,
								opcode, instruction->info.rb);
						break;
					case 5: /* ADD5.PC */
						instruction->info.rt = 0;
						instruction->info.rt = nds32_extract_field_8u(opcode, 0, 5);
						instruction->type = NDS32_INSN_DATA_PROC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tADD5.PC\t$r%" PRIu8,
								address,
								opcode, instruction->info.rt);
						break;
					default:
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%8.8" PRIx16
								"\tUNDEFINED INSTRUCTION",
								address,
								opcode);
						return ERROR_FAIL;
				}
			} else { /* BNES38 */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tBNES38\t$r%" PRIu8 ",#%" PRId32,
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			}
			break;
	}

	return ERROR_OK;
}

static int nds32_parse_group_3_insn_16(struct nds32 *nds32, uint16_t opcode,
		uint32_t address, struct nds32_instruction *instruction)
{
	switch ((opcode >> 11) & 0x3) {
		case 0:
			switch ((opcode >> 9) & 0x3) {
				case 0: /* SLTS45 */
					instruction->info.ra = nds32_extract_field_8u(opcode, 5, 4);
					instruction->info.rb = nds32_extract_field_8u(opcode, 0, 5);
					instruction->type = NDS32_INSN_DATA_PROC;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tSLTS45\t$r%" PRIu8 ",$r%" PRIu8,
							address,
							opcode, instruction->info.ra, instruction->info.rb);
					break;
				case 1: /* SLT45 */
					instruction->info.ra = nds32_extract_field_8u(opcode, 5, 4);
					instruction->info.rb = nds32_extract_field_8u(opcode, 0, 5);
					instruction->type = NDS32_INSN_DATA_PROC;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tSLT45\t$r%" PRIu8 ",$r%" PRIu8,
							address,
							opcode, instruction->info.ra, instruction->info.rb);
					break;
				case 2: /* SLTSI45 */
					instruction->info.ra = nds32_extract_field_8u(opcode, 5, 4);
					instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5);
					instruction->type = NDS32_INSN_DATA_PROC;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tSLTSI45\t$r%" PRIu8 ",#%" PRId32,
							address,
							opcode, instruction->info.ra, instruction->info.imm);
					break;
				case 3: /* SLTI45 */
					instruction->info.ra = nds32_extract_field_8u(opcode, 5, 4);
					instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5);
					instruction->type = NDS32_INSN_DATA_PROC;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tSLTI45\t$r%" PRIu8 ",#%" PRId32,
							address,
							opcode, instruction->info.ra, instruction->info.imm);
					break;
			}
			break;
		case 1:
			switch ((opcode >> 9) & 0x3) {
				case 0:
					instruction->info.imm = nds32_extract_field_8u(opcode, 0, 8);
					instruction->info.imm = (instruction->info.imm << 24) >> 24;
					instruction->type = NDS32_INSN_JUMP_BRANCH;
					if (nds32_extract_field_8u(opcode, 8, 1) == 0) { /* BEQZS8 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tBEQZS8\t#%" PRId32,
								address,
								opcode, instruction->info.imm);
					} else { /* BNEZS8 */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tBNEZS8\t#%" PRId32,
								address,
								opcode, instruction->info.imm);
					}
					break;
				case 1: /* BREAK16 */
					if (((opcode >> 5) & 0xF) == 0) {
						instruction->type = NDS32_INSN_MISC;
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tBREAK16\t#%" PRId16,
								address,
								opcode, (int16_t)(opcode & 0x1F));
					} else { /* EX9.IT */
						instruction->type = NDS32_INSN_MISC;
						/* TODO: implement real instruction semantics */
						snprintf(instruction->text,
								128,
								"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
								"\t\tEX9.IT\t#%" PRId16,
								address,
								opcode, (int16_t)(opcode & 0x1FF));
					}
					break;
				case 2: /* ADDI10S */
				case 3:
					instruction->info.imm = opcode & 0x3FF;
					instruction->info.imm = (instruction->info.imm << 22) >> 22;
					instruction->type = NDS32_INSN_DATA_PROC;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tADDI10.SP\t#%" PRId32,
							address,
							opcode, instruction->info.imm);
					break;
			}
			break;
		case 2:
			instruction->info.rt = nds32_extract_field_8u(opcode, 8, 3);
			instruction->info.imm = nds32_extract_field_8u(opcode, 0, 7) << 2;
			instruction->type = NDS32_INSN_LOAD_STORE;
			nds32_get_mapped_reg(nds32, R31, &(instruction->access_start));
			instruction->access_start += instruction->info.imm;
			instruction->access_end = instruction->access_start + 4;
			if (nds32_extract_field_8u(opcode, 7, 1) == 0) { /* LWI37.SP */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tLWI37.SP\t$r%" PRIu8 ",[+#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			} else { /* SWI37.SP */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
						"\t\tSWI37.SP\t$r%" PRIu8 ",[+#%" PRId32 "]",
						address,
						opcode, instruction->info.rt, instruction->info.imm);
			}
			break;
		case 3:
			switch ((opcode >> 9) & 0x3) {
				case 0: /* IFCALL9 */
					instruction->info.imm = opcode & 0x1FF;
					instruction->type = NDS32_INSN_JUMP_BRANCH;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tIFCALL9\t#%" PRId32 "",
							address,
							opcode, instruction->info.imm);
					break;
				case 1: /* MOVPI45 */
					instruction->info.imm = nds32_extract_field_8u(opcode, 0, 5) + 16;
					instruction->info.rt = nds32_extract_field_8u(opcode, 5, 4);
					instruction->type = NDS32_INSN_MISC;
					snprintf(instruction->text,
							128,
							"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
							"\t\tMOVPI45\t$r%" PRIu8 ",#%" PRId32 "",
							address,
							opcode, instruction->info.rt, instruction->info.imm);
					break;
				case 2: /* PUSH25, POP25, MOVD44 */
					switch ((opcode >> 7) & 0x3) {
						case 0: /* PUSH25 */
							{
								uint8_t re;
								uint8_t gpr_count;

								instruction->type = NDS32_INSN_LOAD_STORE;
								instruction->info.imm =
									nds32_extract_field_8u(opcode, 0, 5) << 3;
								re = nds32_extract_field_8u(opcode, 5, 2);

								if (re == 0)
									re = 6;
								else if (re == 1)
									re = 8;
								else if (re == 2)
									re = 10;
								else if (re == 3)
									re = 14;

								instruction->info.rd = re;
								/* GPRs list: R6 ~ Re and fp, gp, lp */
								gpr_count = 3 + (re - 5);

								nds32_get_mapped_reg(nds32, R31,
										&(instruction->access_end));
								instruction->access_start =
									instruction->access_end - (gpr_count * 4);

								snprintf(instruction->text,
										128,
										"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
										"\t\tPUSH25\t$r%" PRIu8 ",#%" PRId32,
										address,
										opcode, instruction->info.rd,
										instruction->info.imm);
							}
							break;
						case 1: /* POP25 */
							{
								uint8_t re;
								uint8_t gpr_count;

								instruction->type = NDS32_INSN_LOAD_STORE;
								instruction->info.imm =
									nds32_extract_field_8u(opcode, 0, 5) << 3;
								re = nds32_extract_field_8u(opcode, 5, 2);

								if (re == 0)
									re = 6;
								else if (re == 1)
									re = 8;
								else if (re == 2)
									re = 10;
								else if (re == 3)
									re = 14;

								instruction->info.rd = re;
								/* GPRs list: R6 ~ Re and fp, gp, lp */
								gpr_count = 3 + (re - 5);

								nds32_get_mapped_reg(nds32, R31,
										&(instruction->access_start));
								instruction->access_start += instruction->info.imm;
								instruction->access_end =
									instruction->access_start + (gpr_count * 4);

								snprintf(instruction->text,
										128,
										"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
										"\t\tPOP25\t$r%" PRIu8 ",#%" PRId32,
										address,
										opcode, instruction->info.rd,
										instruction->info.imm);
							}
							break;
						case 2: /* MOVD44 */
						case 3:
							instruction->info.ra =
								nds32_extract_field_8u(opcode, 0, 4) * 2;
							instruction->info.rt =
								nds32_extract_field_8u(opcode, 4, 4) * 2;
							instruction->type = NDS32_INSN_MISC;
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
									"\t\tMOVD44\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode, instruction->info.rt, instruction->info.ra);
							break;
					}
					break;
				case 3: /* NEG33, NOT33, MUL33, XOR33, AND33, OR33 */
					instruction->info.ra = nds32_extract_field_8u(opcode, 3, 3);
					instruction->info.rt = nds32_extract_field_8u(opcode, 6, 3);
					instruction->type = NDS32_INSN_DATA_PROC;
					switch (opcode & 0x7) {
						case 2: /* NEG33 */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
									"\t\tNEG33\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode, instruction->info.rt, instruction->info.ra);
							break;
						case 3: /* NOT33 */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
									"\t\tNOT33\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode, instruction->info.rt, instruction->info.ra);
							break;
						case 4: /* MUL33 */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
									"\t\tMUL33\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode, instruction->info.rt, instruction->info.ra);
							break;
						case 5: /* XOR33 */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
									"\t\tXOR33\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode, instruction->info.rt, instruction->info.ra);
							break;
						case 6: /* AND33 */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
									"\t\tAND33\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode, instruction->info.rt, instruction->info.ra);
							break;
						case 7: /* OR33 */
							snprintf(instruction->text,
									128,
									"0x%8.8" PRIx32 "\t0x%4.4" PRIx16
									"\t\tOR33\t$r%" PRIu8 ",$r%" PRIu8,
									address,
									opcode, instruction->info.rt, instruction->info.ra);
							break;
					}
					break;
			}
			break;
		default:
			snprintf(instruction->text,
					128,
					"0x%8.8" PRIx32 "\t0x%8.8" PRIx16 "\tUNDEFINED INSTRUCTION",
					address,
					opcode);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

int nds32_evaluate_opcode(struct nds32 *nds32, uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction)
{
	int retval = ERROR_OK;

	/* clear fields, to avoid confusion */
	memset(instruction, 0, sizeof(struct nds32_instruction));

	if (opcode >> 31) {
		/* 16 bits instruction */
		instruction->instruction_size = 2;
		opcode = (opcode >> 16) & 0xFFFF;
		instruction->opcode = opcode;

		switch ((opcode >> 13) & 0x3) {
			case 0:
				retval = nds32_parse_group_0_insn_16(nds32, opcode, address, instruction);
				break;
			case 1:
				retval = nds32_parse_group_1_insn_16(nds32, opcode, address, instruction);
				break;
			case 2:
				retval = nds32_parse_group_2_insn_16(nds32, opcode, address, instruction);
				break;
			case 3:
				retval = nds32_parse_group_3_insn_16(nds32, opcode, address, instruction);
				break;
			default:
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
						address,
						opcode);
				return ERROR_FAIL;
		}
	} else {
		/* 32 bits instruction */
		instruction->instruction_size = 4;
		instruction->opcode = opcode;

		uint8_t opc_6;
		opc_6 = opcode >> 25;
		instruction->info.opc_6 = opc_6;

		switch ((opc_6 >> 3) & 0x7) {
			case 0: /* LBI, LHI, LWI, LBI.bi, LHI.bi, LWI.bi */
				retval = nds32_parse_group_0_insn(nds32, opcode, address, instruction);
				break;
			case 1: /* SBI, SHI, SWI, SBI.bi, SHI.bi, SWI.bi */
				retval = nds32_parse_group_1_insn(nds32, opcode, address, instruction);
				break;
			case 2: /* LBSI, LHSI, DPREFI, LBSI.bi, LHSI.bi, LBGP */
				retval = nds32_parse_group_2_insn(nds32, opcode, address, instruction);
				break;
			case 3: /* MEM, LSMW, HWGP, SBGP */
				retval = nds32_parse_group_3_insn(nds32, opcode, address, instruction);
				break;
			case 4: /* ALU_1, ALU_2, MOVI, SETHI, JI, JREG, BR1, BR2 */
				retval = nds32_parse_group_4_insn(nds32, opcode, address, instruction);
				break;
			case 5: /* ADDI, SUBRI, ANDI, XORI, ORI, SLTI, SLTSI */
				retval = nds32_parse_group_5_insn(nds32, opcode, address, instruction);
				break;
			case 6: /* MISC */
				retval = nds32_parse_group_6_insn(nds32, opcode, address, instruction);
				break;
			default: /* ERROR */
				snprintf(instruction->text,
						128,
						"0x%8.8" PRIx32 "\t0x%8.8" PRIx32 "\tUNDEFINED INSTRUCTION",
						address,
						opcode);
				return ERROR_FAIL;
		}
	}

	return retval;
}
