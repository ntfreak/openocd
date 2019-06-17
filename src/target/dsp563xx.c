/***************************************************************************
 *   Copyright (C) 2009-2011 by Mathias Kuester                            *
 *   mkdorg@users.sourceforge.net                                          *
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

#include <jim.h>

#include "target.h"
#include "breakpoints.h"
#include "target_type.h"
#include "algorithm.h"
#include "register.h"
#include "dsp563xx.h"
#include "dsp563xx_once.h"

#define ASM_REG_W_R0    0x60F400
#define ASM_REG_W_R1    0x61F400
#define ASM_REG_W_R2    0x62F400
#define ASM_REG_W_R3    0x63F400
#define ASM_REG_W_R4    0x64F400
#define ASM_REG_W_R5    0x65F400
#define ASM_REG_W_R6    0x66F400
#define ASM_REG_W_R7    0x67F400

#define ASM_REG_W_N0    0x70F400
#define ASM_REG_W_N1    0x71F400
#define ASM_REG_W_N2    0x72F400
#define ASM_REG_W_N3    0x73F400
#define ASM_REG_W_N4    0x74F400
#define ASM_REG_W_N5    0x75F400
#define ASM_REG_W_N6    0x76F400
#define ASM_REG_W_N7    0x77F400

#define ASM_REG_W_M0    0x05F420
#define ASM_REG_W_M1    0x05F421
#define ASM_REG_W_M2    0x05F422
#define ASM_REG_W_M3    0x05F423
#define ASM_REG_W_M4    0x05F424
#define ASM_REG_W_M5    0x05F425
#define ASM_REG_W_M6    0x05F426
#define ASM_REG_W_M7    0x05F427

#define ASM_REG_W_X0    0x44F400
#define ASM_REG_W_X1    0x45F400

#define ASM_REG_W_Y0    0x46F400
#define ASM_REG_W_Y1    0x47F400

#define ASM_REG_W_A0    0x50F400
#define ASM_REG_W_A1    0x54F400
#define ASM_REG_W_A2    0x52F400

#define ASM_REG_W_B0    0x51F400
#define ASM_REG_W_B1    0x55F400
#define ASM_REG_W_B2    0x53F400

#define ASM_REG_W_VBA   0x05F430
#define ASM_REG_W_OMR   0x05F43A
#define ASM_REG_W_EP    0x05F42A
#define ASM_REG_W_SC    0x05F431
#define ASM_REG_W_SZ    0x05F438
#define ASM_REG_W_SR    0x05F439
#define ASM_REG_W_SP    0x05F43B
#define ASM_REG_W_SSH   0x05F43C
#define ASM_REG_W_SSL   0x05F43D
#define ASM_REG_W_LA    0x05F43E
#define ASM_REG_W_LC    0x05F43F
#define ASM_REG_W_PC    0x000000
#define ASM_REG_W_IPRC  0xFFFFFF
#define ASM_REG_W_IPRP  0xFFFFFE

#define ASM_REG_W_BCR   0xFFFFFB
#define ASM_REG_W_DCR   0xFFFFFA
#define ASM_REG_W_AAR0  0xFFFFF9
#define ASM_REG_W_AAR1  0xFFFFF8
#define ASM_REG_W_AAR2  0xFFFFF7
#define ASM_REG_W_AAR3  0xFFFFF6

/*
 * OBCR Register bit definitions
 */
#define OBCR_b0_and_b1            ((0x0) << 10)
#define OBCR_b0_or_b1             ((0x1) << 10)
#define OBCR_b1_after_b0          ((0x2) << 10)
#define OBCR_b0_after_b1          ((0x3) << 10)

#define OBCR_BP_DISABLED          (0x0)
#define OBCR_BP_MEM_P             (0x1)
#define OBCR_BP_MEM_X             (0x2)
#define OBCR_BP_MEM_Y             (0x3)
#define OBCR_BP_ON_READ           ((0x2) << 0)
#define OBCR_BP_ON_WRITE          ((0x1) << 0)
#define OBCR_BP_CC_NOT_EQUAL      ((0x0) << 2)
#define OBCR_BP_CC_EQUAL          ((0x1) << 2)
#define OBCR_BP_CC_LESS_THAN      ((0x2) << 2)
#define OBCR_BP_CC_GREATER_THAN   ((0x3) << 2)

#define OBCR_BP_0(x)              ((x)<<2)
#define OBCR_BP_1(x)              ((x)<<6)


enum once_reg_idx {
	ONCE_REG_IDX_OSCR = 0,
	ONCE_REG_IDX_OMBC = 1,
	ONCE_REG_IDX_OBCR = 2,
	ONCE_REG_IDX_OMLR0 = 3,
	ONCE_REG_IDX_OMLR1 = 4,
	ONCE_REG_IDX_OGDBR = 5,
	ONCE_REG_IDX_OPDBR = 6,
	ONCE_REG_IDX_OPILR = 7,
	ONCE_REG_IDX_PDB = 8,
	ONCE_REG_IDX_OTC = 9,
	ONCE_REG_IDX_OPABFR = 10,
	ONCE_REG_IDX_OPABDR = 11,
	ONCE_REG_IDX_OPABEX = 12,
	ONCE_REG_IDX_OPABF0 = 13,
	ONCE_REG_IDX_OPABF1 = 14,
	ONCE_REG_IDX_OPABF2 = 15,
	ONCE_REG_IDX_OPABF3 = 16,
	ONCE_REG_IDX_OPABF4 = 17,
	ONCE_REG_IDX_OPABF5 = 18,
	ONCE_REG_IDX_OPABF6 = 19,
	ONCE_REG_IDX_OPABF7 = 20,
	ONCE_REG_IDX_OPABF8 = 21,
	ONCE_REG_IDX_OPABF9 = 22,
	ONCE_REG_IDX_OPABF10 = 23,
	ONCE_REG_IDX_OPABF11 = 24,
};

static struct once_reg once_regs[] = {
	{ONCE_REG_IDX_OSCR,    DSP563XX_ONCE_OSCR,    24, "OSCR",    0},
	{ONCE_REG_IDX_OMBC,    DSP563XX_ONCE_OMBC,    24, "OMBC",    0},
	{ONCE_REG_IDX_OBCR,    DSP563XX_ONCE_OBCR,    24, "OBCR",    0},
	{ONCE_REG_IDX_OMLR0,   DSP563XX_ONCE_OMLR0,   24, "OMLR0",   0},
	{ONCE_REG_IDX_OMLR1,   DSP563XX_ONCE_OMLR1,   24, "OMLR1",   0},
	{ONCE_REG_IDX_OGDBR,   DSP563XX_ONCE_OGDBR,   24, "OGDBR",   0},
	{ONCE_REG_IDX_OPDBR,   DSP563XX_ONCE_OPDBR,   24, "OPDBR",   0},
	{ONCE_REG_IDX_OPILR,   DSP563XX_ONCE_OPILR,   24, "OPILR",   0},
	{ONCE_REG_IDX_PDB,     DSP563XX_ONCE_PDBGOTO, 24, "PDB",     0},
	{ONCE_REG_IDX_OTC,     DSP563XX_ONCE_OTC,     24, "OTC",     0},
	{ONCE_REG_IDX_OPABFR,  DSP563XX_ONCE_OPABFR,  24, "OPABFR",  0},
	{ONCE_REG_IDX_OPABDR,  DSP563XX_ONCE_OPABDR,  24, "OPABDR",  0},
	{ONCE_REG_IDX_OPABEX,  DSP563XX_ONCE_OPABEX,  24, "OPABEX",  0},
	{ONCE_REG_IDX_OPABF0,  DSP563XX_ONCE_OPABF11, 25, "OPABF0",  0},
	{ONCE_REG_IDX_OPABF1,  DSP563XX_ONCE_OPABF11, 25, "OPABF1",  0},
	{ONCE_REG_IDX_OPABF2,  DSP563XX_ONCE_OPABF11, 25, "OPABF2",  0},
	{ONCE_REG_IDX_OPABF3,  DSP563XX_ONCE_OPABF11, 25, "OPABF3",  0},
	{ONCE_REG_IDX_OPABF4,  DSP563XX_ONCE_OPABF11, 25, "OPABF4",  0},
	{ONCE_REG_IDX_OPABF5,  DSP563XX_ONCE_OPABF11, 25, "OPABF5",  0},
	{ONCE_REG_IDX_OPABF6,  DSP563XX_ONCE_OPABF11, 25, "OPABF6",  0},
	{ONCE_REG_IDX_OPABF7,  DSP563XX_ONCE_OPABF11, 25, "OPABF7",  0},
	{ONCE_REG_IDX_OPABF8,  DSP563XX_ONCE_OPABF11, 25, "OPABF8",  0},
	{ONCE_REG_IDX_OPABF9,  DSP563XX_ONCE_OPABF11, 25, "OPABF9",  0},
	{ONCE_REG_IDX_OPABF10, DSP563XX_ONCE_OPABF11, 25, "OPABF10", 0},
	{ONCE_REG_IDX_OPABF11, DSP563XX_ONCE_OPABF11, 25, "OPABF11", 0},
/*      {25,0x1f,24,"NRSEL",0}, */
};

enum dsp563xx_reg_idx {
	DSP563XX_REG_IDX_R0 = 0,
	DSP563XX_REG_IDX_R1 = 1,
	DSP563XX_REG_IDX_R2 = 2,
	DSP563XX_REG_IDX_R3 = 3,
	DSP563XX_REG_IDX_R4 = 4,
	DSP563XX_REG_IDX_R5 = 5,
	DSP563XX_REG_IDX_R6 = 6,
	DSP563XX_REG_IDX_R7 = 7,
	DSP563XX_REG_IDX_N0 = 8,
	DSP563XX_REG_IDX_N1 = 9,
	DSP563XX_REG_IDX_N2 = 10,
	DSP563XX_REG_IDX_N3 = 11,
	DSP563XX_REG_IDX_N4 = 12,
	DSP563XX_REG_IDX_N5 = 13,
	DSP563XX_REG_IDX_N6 = 14,
	DSP563XX_REG_IDX_N7 = 15,
	DSP563XX_REG_IDX_M0 = 16,
	DSP563XX_REG_IDX_M1 = 17,
	DSP563XX_REG_IDX_M2 = 18,
	DSP563XX_REG_IDX_M3 = 19,
	DSP563XX_REG_IDX_M4 = 20,
	DSP563XX_REG_IDX_M5 = 21,
	DSP563XX_REG_IDX_M6 = 22,
	DSP563XX_REG_IDX_M7 = 23,
	DSP563XX_REG_IDX_X0 = 24,
	DSP563XX_REG_IDX_X1 = 25,
	DSP563XX_REG_IDX_Y0 = 26,
	DSP563XX_REG_IDX_Y1 = 27,
	DSP563XX_REG_IDX_A0 = 28,
	DSP563XX_REG_IDX_A1 = 29,
	DSP563XX_REG_IDX_A2 = 30,
	DSP563XX_REG_IDX_B0 = 31,
	DSP563XX_REG_IDX_B1 = 32,
	DSP563XX_REG_IDX_B2 = 33,
	DSP563XX_REG_IDX_SSH = 34,
	DSP563XX_REG_IDX_SSL = 35,
	DSP563XX_REG_IDX_SP = 36,
	DSP563XX_REG_IDX_EP = 37,
	DSP563XX_REG_IDX_SZ = 38,
	DSP563XX_REG_IDX_SC = 39,
	DSP563XX_REG_IDX_PC = 40,
	DSP563XX_REG_IDX_SR = 41,
	DSP563XX_REG_IDX_OMR = 42,
	DSP563XX_REG_IDX_LA = 43,
	DSP563XX_REG_IDX_LC = 44,
	DSP563XX_REG_IDX_VBA = 45,
	DSP563XX_REG_IDX_IPRC = 46,
	DSP563XX_REG_IDX_IPRP = 47,
	DSP563XX_REG_IDX_BCR = 48,
	DSP563XX_REG_IDX_DCR = 49,
	DSP563XX_REG_IDX_AAR0 = 50,
	DSP563XX_REG_IDX_AAR1 = 51,
	DSP563XX_REG_IDX_AAR2 = 52,
	DSP563XX_REG_IDX_AAR3 = 53,
};

static const struct {
	unsigned id;
	const char *name;
	unsigned bits;
	/* effective addressing mode encoding */
	uint8_t eame;
	uint32_t instr_mask;
} dsp563xx_regs[] = {
	/* *INDENT-OFF* */
	/* address registers */
	{DSP563XX_REG_IDX_R0, "r0", 24, 0x10, ASM_REG_W_R0},
	{DSP563XX_REG_IDX_R1, "r1", 24, 0x11, ASM_REG_W_R1},
	{DSP563XX_REG_IDX_R2, "r2", 24, 0x12, ASM_REG_W_R2},
	{DSP563XX_REG_IDX_R3, "r3", 24, 0x13, ASM_REG_W_R3},
	{DSP563XX_REG_IDX_R4, "r4", 24, 0x14, ASM_REG_W_R4},
	{DSP563XX_REG_IDX_R5, "r5", 24, 0x15, ASM_REG_W_R5},
	{DSP563XX_REG_IDX_R6, "r6", 24, 0x16, ASM_REG_W_R6},
	{DSP563XX_REG_IDX_R7, "r7", 24, 0x17, ASM_REG_W_R7},
	/* offset registers */
	{DSP563XX_REG_IDX_N0, "n0", 24, 0x18, ASM_REG_W_N0},
	{DSP563XX_REG_IDX_N1, "n1", 24, 0x19, ASM_REG_W_N1},
	{DSP563XX_REG_IDX_N2, "n2", 24, 0x1a, ASM_REG_W_N2},
	{DSP563XX_REG_IDX_N3, "n3", 24, 0x1b, ASM_REG_W_N3},
	{DSP563XX_REG_IDX_N4, "n4", 24, 0x1c, ASM_REG_W_N4},
	{DSP563XX_REG_IDX_N5, "n5", 24, 0x1d, ASM_REG_W_N5},
	{DSP563XX_REG_IDX_N6, "n6", 24, 0x1e, ASM_REG_W_N6},
	{DSP563XX_REG_IDX_N7, "n7", 24, 0x1f, ASM_REG_W_N7},
	/* modifier registers */
	{DSP563XX_REG_IDX_M0, "m0", 24, 0x20, ASM_REG_W_M0},
	{DSP563XX_REG_IDX_M1, "m1", 24, 0x21, ASM_REG_W_M1},
	{DSP563XX_REG_IDX_M2, "m2", 24, 0x22, ASM_REG_W_M2},
	{DSP563XX_REG_IDX_M3, "m3", 24, 0x23, ASM_REG_W_M3},
	{DSP563XX_REG_IDX_M4, "m4", 24, 0x24, ASM_REG_W_M4},
	{DSP563XX_REG_IDX_M5, "m5", 24, 0x25, ASM_REG_W_M5},
	{DSP563XX_REG_IDX_M6, "m6", 24, 0x26, ASM_REG_W_M6},
	{DSP563XX_REG_IDX_M7, "m7", 24, 0x27, ASM_REG_W_M7},
	/* data alu input register */
	{DSP563XX_REG_IDX_X0, "x0", 24, 0x04, ASM_REG_W_X0},
	{DSP563XX_REG_IDX_X1, "x1", 24, 0x05, ASM_REG_W_X1},
	{DSP563XX_REG_IDX_Y0, "y0", 24, 0x06, ASM_REG_W_Y0},
	{DSP563XX_REG_IDX_Y1, "y1", 24, 0x07, ASM_REG_W_Y1},
	/* data alu accumulator register */
	{DSP563XX_REG_IDX_A0, "a0", 24, 0x08, ASM_REG_W_A0},
	{DSP563XX_REG_IDX_A1, "a1", 24, 0x0c, ASM_REG_W_A1},
	{DSP563XX_REG_IDX_A2, "a2", 8, 0x0a, ASM_REG_W_A2},
	{DSP563XX_REG_IDX_B0, "b0", 24, 0x09, ASM_REG_W_B0},
	{DSP563XX_REG_IDX_B1, "b1", 24, 0x0d, ASM_REG_W_B1},
	{DSP563XX_REG_IDX_B2, "b2", 8, 0x0b, ASM_REG_W_B2},
	/* stack */
	{DSP563XX_REG_IDX_SSH, "ssh", 24, 0x3c, ASM_REG_W_SSH},
	{DSP563XX_REG_IDX_SSL, "ssl", 24, 0x3d, ASM_REG_W_SSL},
	{DSP563XX_REG_IDX_SP, "sp", 24, 0x3b, ASM_REG_W_SP},
	{DSP563XX_REG_IDX_EP, "ep", 24, 0x2a, ASM_REG_W_EP},
	{DSP563XX_REG_IDX_SZ, "sz", 24, 0x38, ASM_REG_W_SZ},
	{DSP563XX_REG_IDX_SC, "sc", 24, 0x31, ASM_REG_W_SC},
	/* system */
	{DSP563XX_REG_IDX_PC, "pc", 24, 0x00, ASM_REG_W_PC},
	{DSP563XX_REG_IDX_SR, "sr", 24, 0x39, ASM_REG_W_SR},
	{DSP563XX_REG_IDX_OMR, "omr", 24, 0x3a, ASM_REG_W_OMR},
	{DSP563XX_REG_IDX_LA, "la", 24, 0x3e, ASM_REG_W_LA},
	{DSP563XX_REG_IDX_LC, "lc", 24, 0x3f, ASM_REG_W_LC},
	/* interrupt */
	{DSP563XX_REG_IDX_VBA, "vba", 24, 0x30, ASM_REG_W_VBA},
	{DSP563XX_REG_IDX_IPRC, "iprc", 24, 0x00, ASM_REG_W_IPRC},
	{DSP563XX_REG_IDX_IPRP, "iprp", 24, 0x00, ASM_REG_W_IPRP},
	/* port a */
	{DSP563XX_REG_IDX_BCR, "bcr", 24, 0x00, ASM_REG_W_BCR},
	{DSP563XX_REG_IDX_DCR, "dcr", 24, 0x00, ASM_REG_W_DCR},
	{DSP563XX_REG_IDX_AAR0, "aar0", 24, 0x00, ASM_REG_W_AAR0},
	{DSP563XX_REG_IDX_AAR1, "aar1", 24, 0x00, ASM_REG_W_AAR1},
	{DSP563XX_REG_IDX_AAR2, "aar2", 24, 0x00, ASM_REG_W_AAR2},
	{DSP563XX_REG_IDX_AAR3, "aar3", 24, 0x00, ASM_REG_W_AAR3},
	/* *INDENT-ON* */
};

enum memory_type {
	MEM_X = 0,
	MEM_Y = 1,
	MEM_P = 2,
	MEM_L = 3,
};

enum watchpoint_condition {
	EQUAL,
	NOT_EQUAL,
	GREATER,
	LESS_THAN
};

#define INSTR_JUMP      0x0AF080
/* Effective Addressing Mode Encoding */
#define EAME_R0         0x10
/* instrcution encoder */
/* movep
 * s - peripheral space X/Y (X=0,Y=1)
 * w - write/read
 * d - source/destination register
 * p - IO short address
 */
#define INSTR_MOVEP_REG_HIO(s, w, d, p) (0x084000 | \
	((s & 1) << 16) | ((w & 1) << 15) | ((d & 0x3f) << 8) | (p & 0x3f))

/* the gdb register list is send in this order */
static const uint8_t gdb_reg_list_idx[] = {
	DSP563XX_REG_IDX_X1, DSP563XX_REG_IDX_X0, DSP563XX_REG_IDX_Y1, DSP563XX_REG_IDX_Y0,
	DSP563XX_REG_IDX_A2, DSP563XX_REG_IDX_A1, DSP563XX_REG_IDX_A0, DSP563XX_REG_IDX_B2,
	DSP563XX_REG_IDX_B1, DSP563XX_REG_IDX_B0, DSP563XX_REG_IDX_PC, DSP563XX_REG_IDX_SR,
	DSP563XX_REG_IDX_OMR, DSP563XX_REG_IDX_LA, DSP563XX_REG_IDX_LC, DSP563XX_REG_IDX_SSH,
	DSP563XX_REG_IDX_SSL, DSP563XX_REG_IDX_SP, DSP563XX_REG_IDX_EP, DSP563XX_REG_IDX_SZ,
	DSP563XX_REG_IDX_SC, DSP563XX_REG_IDX_VBA, DSP563XX_REG_IDX_IPRC, DSP563XX_REG_IDX_IPRP,
	DSP563XX_REG_IDX_BCR, DSP563XX_REG_IDX_DCR, DSP563XX_REG_IDX_AAR0, DSP563XX_REG_IDX_AAR1,
	DSP563XX_REG_IDX_AAR2, DSP563XX_REG_IDX_AAR3, DSP563XX_REG_IDX_R0, DSP563XX_REG_IDX_R1,
	DSP563XX_REG_IDX_R2, DSP563XX_REG_IDX_R3, DSP563XX_REG_IDX_R4, DSP563XX_REG_IDX_R5,
	DSP563XX_REG_IDX_R6, DSP563XX_REG_IDX_R7, DSP563XX_REG_IDX_N0, DSP563XX_REG_IDX_N1,
	DSP563XX_REG_IDX_N2, DSP563XX_REG_IDX_N3, DSP563XX_REG_IDX_N4, DSP563XX_REG_IDX_N5,
	DSP563XX_REG_IDX_N6, DSP563XX_REG_IDX_N7, DSP563XX_REG_IDX_M0, DSP563XX_REG_IDX_M1,
	DSP563XX_REG_IDX_M2, DSP563XX_REG_IDX_M3, DSP563XX_REG_IDX_M4, DSP563XX_REG_IDX_M5,
	DSP563XX_REG_IDX_M6, DSP563XX_REG_IDX_M7,
};

static int dsp563xx_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	int i;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	*reg_list_size = DSP563XX_NUMCOREREGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++)
		(*reg_list)[i] = &dsp563xx->core_cache->reg_list[gdb_reg_list_idx[i]];

	return ERROR_OK;

}

static int dsp563xx_read_core_reg(struct target *target, int num)
{
	uint32_t reg_value;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if ((num < 0) || (num >= DSP563XX_NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = dsp563xx->core_regs[num];
	buf_set_u32(dsp563xx->core_cache->reg_list[num].value, 0, 32, reg_value);
	dsp563xx->core_cache->reg_list[num].valid = true;
	dsp563xx->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int dsp563xx_write_core_reg(struct target *target, int num)
{
	uint32_t reg_value;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if ((num < 0) || (num >= DSP563XX_NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = buf_get_u32(dsp563xx->core_cache->reg_list[num].value, 0, 32);
	dsp563xx->core_regs[num] = reg_value;
	dsp563xx->core_cache->reg_list[num].valid = true;
	dsp563xx->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int dsp563xx_get_core_reg(struct reg *reg)
{
	struct dsp563xx_core_reg *dsp563xx_reg = reg->arch_info;
	struct target *target = dsp563xx_reg->target;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	LOG_DEBUG("%s", __func__);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	return dsp563xx->read_core_reg(target, dsp563xx_reg->num);
}

static int dsp563xx_set_core_reg(struct reg *reg, uint8_t *buf)
{
	LOG_DEBUG("%s", __func__);

	struct dsp563xx_core_reg *dsp563xx_reg = reg->arch_info;
	struct target *target = dsp563xx_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static const struct reg_arch_type dsp563xx_reg_type = {
	.get = dsp563xx_get_core_reg,
	.set = dsp563xx_set_core_reg,
};

static void dsp563xx_build_reg_cache(struct target *target)
{
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(DSP563XX_NUMCOREREGS, sizeof(struct reg));
	struct dsp563xx_core_reg *arch_info = malloc(
			sizeof(struct dsp563xx_core_reg) * DSP563XX_NUMCOREREGS);
	int i;

	/* Build the process context cache */
	cache->name = "dsp563xx registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = DSP563XX_NUMCOREREGS;
	(*cache_p) = cache;
	dsp563xx->core_cache = cache;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++) {
		arch_info[i].num = dsp563xx_regs[i].id;
		arch_info[i].name = dsp563xx_regs[i].name;
		arch_info[i].size = dsp563xx_regs[i].bits;
		arch_info[i].eame = dsp563xx_regs[i].eame;
		arch_info[i].instr_mask = dsp563xx_regs[i].instr_mask;
		arch_info[i].target = target;
		arch_info[i].dsp563xx_common = dsp563xx;
		reg_list[i].name = dsp563xx_regs[i].name;
		reg_list[i].size = 32;	/* dsp563xx_regs[i].bits; */
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].exist = true;
		reg_list[i].type = &dsp563xx_reg_type;
		reg_list[i].arch_info = &arch_info[i];
	}
}

static int dsp563xx_read_register(struct target *target, int num, int force);
static int dsp563xx_write_register(struct target *target, int num, int force);

static int dsp563xx_reg_read_high_io(struct target *target, uint32_t instr_mask, uint32_t *data)
{
	int err;
	uint32_t instr;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].valid)
		dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_R0);

	/* move source memory to r0 */
	instr = INSTR_MOVEP_REG_HIO(MEM_X, 0, EAME_R0, instr_mask);
	err = dsp563xx_once_execute_sw_ir(target->tap, 0, instr);
	if (err != ERROR_OK)
		return err;
	/* move r0 to debug register */
	instr = INSTR_MOVEP_REG_HIO(MEM_X, 1, EAME_R0, 0xfffffc);
	err = dsp563xx_once_execute_sw_ir(target->tap, 1, instr);
	if (err != ERROR_OK)
		return err;
	/* read debug register */
	err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OGDBR, data);
	if (err != ERROR_OK)
		return err;
	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].dirty = true;

	return ERROR_OK;
}

static int dsp563xx_reg_write_high_io(struct target *target, uint32_t instr_mask, uint32_t data)
{
	int err;
	uint32_t instr;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].valid)
		dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_R0);

	/* move data to r0 */
	err = dsp563xx_once_execute_dw_ir(target->tap, 0, 0x60F400, data);
	if (err != ERROR_OK)
		return err;
	/* move r0 to destination memory */
	instr = INSTR_MOVEP_REG_HIO(MEM_X, 1, EAME_R0, instr_mask);
	err = dsp563xx_once_execute_sw_ir(target->tap, 1, instr);
	if (err != ERROR_OK)
		return err;

	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].dirty = true;

	return ERROR_OK;
}

static int dsp563xx_reg_read(struct target *target, uint32_t eame, uint32_t *data)
{
	int err;
	uint32_t instr;

	instr = INSTR_MOVEP_REG_HIO(MEM_X, 1, eame, 0xfffffc);
	err = dsp563xx_once_execute_sw_ir(target->tap, 0, instr);
	if (err != ERROR_OK)
		return err;
	/* nop */
	err = dsp563xx_once_execute_sw_ir(target->tap, 1, 0x000000);
	if (err != ERROR_OK)
		return err;
	/* read debug register */
	return dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OGDBR, data);
}

static int dsp563xx_reg_write(struct target *target, uint32_t instr_mask, uint32_t data)
{
	int err;

	err = dsp563xx_once_execute_dw_ir(target->tap, 0, instr_mask, data);
	if (err != ERROR_OK)
		return err;
	/* nop */
	return dsp563xx_once_execute_sw_ir(target->tap, 1, 0x000000);
}

static int dsp563xx_reg_pc_read(struct target *target)
{
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	/* pc was changed, nothing todo */
	if (dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_PC].dirty)
		return ERROR_OK;

	/* conditional branch check */
	if (once_regs[ONCE_REG_IDX_OPABDR].reg == once_regs[ONCE_REG_IDX_OPABEX].reg) {
		if ((once_regs[ONCE_REG_IDX_OPABF11].reg & 1) == 0) {
			LOG_DEBUG("%s conditional branch not supported yet (0x%" PRIx32 " 0x%" PRIx32 " 0x%" PRIx32 ")",
				__func__,
				(once_regs[ONCE_REG_IDX_OPABF11].reg >> 1),
				once_regs[ONCE_REG_IDX_OPABDR].reg,
				once_regs[ONCE_REG_IDX_OPABEX].reg);

			/* TODO: use disassembly to set correct pc offset
			 * read 2 words from OPABF11 and disasm the instruction
			 */
			dsp563xx->core_regs[DSP563XX_REG_IDX_PC] =
				(once_regs[ONCE_REG_IDX_OPABF11].reg >> 1) & 0x00FFFFFF;
		} else {
			if (once_regs[ONCE_REG_IDX_OPABEX].reg ==
				once_regs[ONCE_REG_IDX_OPABFR].reg)
				dsp563xx->core_regs[DSP563XX_REG_IDX_PC] =
					once_regs[ONCE_REG_IDX_OPABEX].reg;
			else
				dsp563xx->core_regs[DSP563XX_REG_IDX_PC] =
					once_regs[ONCE_REG_IDX_OPABEX].reg - 1;
		}
	} else
		dsp563xx->core_regs[DSP563XX_REG_IDX_PC] = once_regs[ONCE_REG_IDX_OPABEX].reg;

	dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_PC);

	return ERROR_OK;
}

static int dsp563xx_reg_ssh_read(struct target *target)
{
	int err;
	uint32_t sp;
	struct dsp563xx_core_reg *arch_info;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_SSH].arch_info;

	/* get a valid stack pointer */
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_SP, 0);
	if (err != ERROR_OK)
		return err;
	sp = dsp563xx->core_regs[DSP563XX_REG_IDX_SP];
	err = dsp563xx_write_register(target, DSP563XX_REG_IDX_SP, 0);
	if (err != ERROR_OK)
		return err;

	/* get a valid stack count */
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_SC, 0);
	if (err != ERROR_OK)
		return err;

	err = dsp563xx_write_register(target, DSP563XX_REG_IDX_SC, 0);
	if (err != ERROR_OK)
		return err;

	/* get a valid extended pointer */
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_EP, 0);
	if (err != ERROR_OK)
		return err;

	err = dsp563xx_write_register(target, DSP563XX_REG_IDX_EP, 0);
	if (err != ERROR_OK)
		return err;

	if (!sp)
		sp = 0x00FFFFFF;
	else {
		err = dsp563xx_reg_read(target, arch_info->eame, &sp);
		if (err != ERROR_OK)
			return err;

		err = dsp563xx_write_register(target, DSP563XX_REG_IDX_SC, 1);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_write_register(target, DSP563XX_REG_IDX_SP, 1);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_write_register(target, DSP563XX_REG_IDX_EP, 1);
		if (err != ERROR_OK)
			return err;
	}

	dsp563xx->core_regs[DSP563XX_REG_IDX_SSH] = sp;
	dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_SSH);

	return ERROR_OK;
}

static int dsp563xx_reg_ssh_write(struct target *target)
{
	int err;
	uint32_t sp;
	struct dsp563xx_core_reg *arch_info;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_SSH].arch_info;

	/* get a valid stack pointer */
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_SP, 0);
	if (err != ERROR_OK)
		return err;
	sp = dsp563xx->core_regs[DSP563XX_REG_IDX_SP];

	if (sp) {
		sp--;
		/* write new stackpointer */
		dsp563xx->core_regs[DSP563XX_REG_IDX_SP] = sp;
		err = dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_SP);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_write_register(target, DSP563XX_REG_IDX_SP, 1);
		if (err != ERROR_OK)
			return err;

		err = dsp563xx_reg_write(target, arch_info->instr_mask,
				dsp563xx->core_regs[DSP563XX_REG_IDX_SSH]);
		if (err != ERROR_OK)
			return err;

		err = dsp563xx_read_register(target, DSP563XX_REG_IDX_SP, 1);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_read_register(target, DSP563XX_REG_IDX_SSH, 1);
		if (err != ERROR_OK)
			return err;
	}

	return ERROR_OK;
}

static int dsp563xx_reg_ssl_read(struct target *target)
{
	int err;
	uint32_t sp;
	struct dsp563xx_core_reg *arch_info;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_SSL].arch_info;

	/* get a valid stack pointer */
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_SP, 0);
	if (err != ERROR_OK)
		return err;
	sp = dsp563xx->core_regs[DSP563XX_REG_IDX_SP];

	if (!sp)
		sp = 0x00FFFFFF;
	else {
		err = dsp563xx_reg_read(target, arch_info->eame, &sp);
		if (err != ERROR_OK)
			return err;
	}

	dsp563xx->core_regs[DSP563XX_REG_IDX_SSL] = sp;
	dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_SSL);

	return ERROR_OK;
}

static int dsp563xx_read_register(struct target *target, int num, int force)
{
	int err = ERROR_OK;
	uint32_t data = 0;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	if (force)
		dsp563xx->core_cache->reg_list[num].valid = false;

	if (!dsp563xx->core_cache->reg_list[num].valid) {
		arch_info = dsp563xx->core_cache->reg_list[num].arch_info;

		switch (arch_info->num) {
			case DSP563XX_REG_IDX_SSH:
				err = dsp563xx_reg_ssh_read(target);
				break;
			case DSP563XX_REG_IDX_SSL:
				err = dsp563xx_reg_ssl_read(target);
				break;
			case DSP563XX_REG_IDX_PC:
				err = dsp563xx_reg_pc_read(target);
				break;
			case DSP563XX_REG_IDX_IPRC:
			case DSP563XX_REG_IDX_IPRP:
			case DSP563XX_REG_IDX_BCR:
			case DSP563XX_REG_IDX_DCR:
			case DSP563XX_REG_IDX_AAR0:
			case DSP563XX_REG_IDX_AAR1:
			case DSP563XX_REG_IDX_AAR2:
			case DSP563XX_REG_IDX_AAR3:
				err = dsp563xx_reg_read_high_io(target,
						arch_info->instr_mask, &data);
				if (err == ERROR_OK) {
					dsp563xx->core_regs[num] = data;
					dsp563xx->read_core_reg(target, num);
				}
				break;
			default:
				err = dsp563xx_reg_read(target, arch_info->eame, &data);
				if (err == ERROR_OK) {
					dsp563xx->core_regs[num] = data;
					dsp563xx->read_core_reg(target, num);
				}
				break;
		}
	}

	return err;
}

static int dsp563xx_write_register(struct target *target, int num, int force)
{
	int err = ERROR_OK;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	if (force)
		dsp563xx->core_cache->reg_list[num].dirty = true;

	if (dsp563xx->core_cache->reg_list[num].dirty) {
		arch_info = dsp563xx->core_cache->reg_list[num].arch_info;

		dsp563xx->write_core_reg(target, num);

		switch (arch_info->num) {
			case DSP563XX_REG_IDX_SSH:
				err = dsp563xx_reg_ssh_write(target);
				break;
			case DSP563XX_REG_IDX_PC:
				/* pc is updated on resume, no need to write it here */
				break;
			case DSP563XX_REG_IDX_IPRC:
			case DSP563XX_REG_IDX_IPRP:
			case DSP563XX_REG_IDX_BCR:
			case DSP563XX_REG_IDX_DCR:
			case DSP563XX_REG_IDX_AAR0:
			case DSP563XX_REG_IDX_AAR1:
			case DSP563XX_REG_IDX_AAR2:
			case DSP563XX_REG_IDX_AAR3:
				err = dsp563xx_reg_write_high_io(target,
					arch_info->instr_mask,
					dsp563xx->core_regs[num]);
				break;
			default:
				err = dsp563xx_reg_write(target,
					arch_info->instr_mask,
					dsp563xx->core_regs[num]);

				if ((err == ERROR_OK) && (arch_info->num == DSP563XX_REG_IDX_SP)) {
					dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_SSH].valid =
						0;
					dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_SSL].valid =
						0;
				}

				break;
		}
	}

	return err;
}

static int dsp563xx_save_context(struct target *target)
{
	int i, err = ERROR_OK;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++) {
		err = dsp563xx_read_register(target, i, 0);
		if (err != ERROR_OK)
			break;
	}

	return err;
}

static int dsp563xx_restore_context(struct target *target)
{
	int i, err = ERROR_OK;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++) {
		err = dsp563xx_write_register(target, i, 0);
		if (err != ERROR_OK)
			break;
	}

	return err;
}

static void dsp563xx_invalidate_x_context(struct target *target,
	uint32_t addr_start,
	uint32_t addr_end)
{
	int i;
	struct dsp563xx_core_reg *arch_info;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (addr_start > ASM_REG_W_IPRC)
		return;
	if (addr_start < ASM_REG_W_AAR3)
		return;

	for (i = DSP563XX_REG_IDX_IPRC; i < DSP563XX_NUMCOREREGS; i++) {
		arch_info = dsp563xx->core_cache->reg_list[i].arch_info;

		if ((arch_info->instr_mask >= addr_start) &&
			(arch_info->instr_mask <= addr_end)) {
			dsp563xx->core_cache->reg_list[i].valid = false;
			dsp563xx->core_cache->reg_list[i].dirty = false;
		}
	}
}

static int dsp563xx_target_create(struct target *target, Jim_Interp *interp)
{
	struct dsp563xx_common *dsp563xx = calloc(1, sizeof(struct dsp563xx_common));

	if (!dsp563xx)
		return ERROR_COMMAND_SYNTAX_ERROR;

	dsp563xx->jtag_info.tap = target->tap;
	target->arch_info = dsp563xx;
	dsp563xx->read_core_reg = dsp563xx_read_core_reg;
	dsp563xx->write_core_reg = dsp563xx_write_core_reg;

	return ERROR_OK;
}

static int dsp563xx_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_DEBUG("%s", __func__);

	dsp563xx_build_reg_cache(target);
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	dsp563xx->hardware_breakpoints_cleared = 0;
	dsp563xx->hardware_breakpoint[0].used = BPU_NONE;

	return ERROR_OK;
}

static int dsp563xx_examine(struct target *target)
{
	uint32_t chip;

	if (target->tap->hasidcode == false) {
		LOG_ERROR("no IDCODE present on device");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!target_was_examined(target)) {
		target_set_examined(target);

		/* examine core and chip derivate number */
		chip = (target->tap->idcode>>12) & 0x3ff;
		/* core number 0 means DSP563XX */
		if (((chip>>5)&0x1f) == 0)
			chip += 300;

		LOG_INFO("DSP56%03" PRId32 " device found", chip);

		/* Clear all breakpoints */
		dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OBCR, 0);
	}

	return ERROR_OK;
}

static int dsp563xx_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

#define DSP563XX_SR_SA (1<<17)
#define DSP563XX_SR_SC (1<<13)

static int dsp563xx_debug_once_init(struct target *target)
{
	return dsp563xx_once_read_register(target->tap, 1, once_regs, DSP563XX_NUMONCEREGS);
}

static int dsp563xx_debug_init(struct target *target)
{
	int err;
	uint32_t sr;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	err = dsp563xx_debug_once_init(target);
	if (err != ERROR_OK)
		return err;

	arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_SR].arch_info;

	/* check 24bit mode */
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_SR, 0);
	if (err != ERROR_OK)
		return err;

	sr = dsp563xx->core_regs[DSP563XX_REG_IDX_SR];

	if (sr & (DSP563XX_SR_SA | DSP563XX_SR_SC)) {
		sr &= ~(DSP563XX_SR_SA | DSP563XX_SR_SC);

		err = dsp563xx_once_execute_dw_ir(target->tap, 1, arch_info->instr_mask, sr);
		if (err != ERROR_OK)
			return err;
		dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_SR].dirty = true;
	}

	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_N0, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_N1, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_M0, 0);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_read_register(target, DSP563XX_REG_IDX_M1, 0);
	if (err != ERROR_OK)
		return err;

	if (dsp563xx->core_regs[DSP563XX_REG_IDX_N0] != 0x000000) {
		arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_N0].arch_info;
		err = dsp563xx_reg_write(target, arch_info->instr_mask, 0x000000);
		if (err != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_N0].dirty = true;

	if (dsp563xx->core_regs[DSP563XX_REG_IDX_N1] != 0x000000) {
		arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_N1].arch_info;
		err = dsp563xx_reg_write(target, arch_info->instr_mask, 0x000000);
		if (err != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_N1].dirty = true;

	if (dsp563xx->core_regs[DSP563XX_REG_IDX_M0] != 0xffffff) {
		arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_M0].arch_info;
		err = dsp563xx_reg_write(target, arch_info->instr_mask, 0xffffff);
		if (err != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_M0].dirty = true;

	if (dsp563xx->core_regs[DSP563XX_REG_IDX_M1] != 0xffffff) {
		arch_info = dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_M1].arch_info;
		err = dsp563xx_reg_write(target, arch_info->instr_mask, 0xffffff);
		if (err != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_M1].dirty = true;

	err = dsp563xx_save_context(target);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int dsp563xx_jtag_debug_request(struct target *target)
{
	return dsp563xx_once_request_debug(target->tap, target->state == TARGET_RESET);
}

static int dsp563xx_poll(struct target *target)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	uint32_t once_status = 0;
	int state;

	state = dsp563xx_once_target_status(target->tap);

	if (state == TARGET_UNKNOWN) {
		target->state = state;
		LOG_ERROR("jtag status contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OSCR, &once_status);
	if (err != ERROR_OK)
		return err;

	if ((once_status & DSP563XX_ONCE_OSCR_DEBUG_M) == DSP563XX_ONCE_OSCR_DEBUG_M) {
		if (target->state != TARGET_HALTED) {
			target->state = TARGET_HALTED;

			err = dsp563xx_debug_init(target);
			if (err != ERROR_OK)
				return err;

			if (once_status & (DSP563XX_ONCE_OSCR_MBO|DSP563XX_ONCE_OSCR_SWO))
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			else
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);

			LOG_DEBUG("target->state: %s (%" PRIx32 ")", target_state_name(target), once_status);
			LOG_INFO("halted: PC: 0x%" PRIx32, dsp563xx->core_regs[DSP563XX_REG_IDX_PC]);
		}
	}

	if (!dsp563xx->hardware_breakpoints_cleared) {
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OBCR, 0);
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OMLR0, 0);
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OMLR1, 0);
		dsp563xx->hardware_breakpoints_cleared = 1;
	}

	return ERROR_OK;
}

static int dsp563xx_halt(struct target *target)
{
	int err;

	LOG_DEBUG("%s", __func__);

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	err = dsp563xx_jtag_debug_request(target);
	if (err != ERROR_OK)
		return err;

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int dsp563xx_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	/* check if pc was changed and resume want to execute the next address
	 * if pc was changed from gdb or other interface we will
	 * jump to this address and don't execute the next address
	 * this will not affect the resume command with an address argument
	 * because current is set to zero then
	 */
	if (current && dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_PC].dirty) {
		dsp563xx_write_core_reg(target, DSP563XX_REG_IDX_PC);
		address = dsp563xx->core_regs[DSP563XX_REG_IDX_PC];
		current = 0;
	}

	LOG_DEBUG("%s %08X %08X", __func__, current, (unsigned) address);

	err = dsp563xx_restore_context(target);
	if (err != ERROR_OK)
		return err;
	register_cache_invalidate(dsp563xx->core_cache);

	if (current) {
		/* restore pipeline registers and go */
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR,
				once_regs[ONCE_REG_IDX_OPILR].reg);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR |
				DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO,
				once_regs[ONCE_REG_IDX_OPDBR].reg);
		if (err != ERROR_OK)
			return err;
	} else {
		/* set to go register and jump */
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR,	INSTR_JUMP);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_PDBGOTO |
				DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO, address);
		if (err != ERROR_OK)
			return err;
	}

	target->state = TARGET_RUNNING;

	target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);

	return ERROR_OK;
}

static int dsp563xx_step_ex(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints,
	int steps)
{
	int err;
	uint32_t once_status;
	uint32_t dr_in, cnt;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("target was not halted");
		return ERROR_OK;
	}

	/* check if pc was changed and step want to execute the next address
	 * if pc was changed from gdb or other interface we will
	 * jump to this address and don't execute the next address
	 * this will not affect the step command with an address argument
	 * because current is set to zero then
	 */
	if (current && dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_PC].dirty) {
		dsp563xx_write_core_reg(target, DSP563XX_REG_IDX_PC);
		address = dsp563xx->core_regs[DSP563XX_REG_IDX_PC];
		current = 0;
	}

	LOG_DEBUG("%s %08X %08X", __func__, current, (unsigned) address);

	err = dsp563xx_jtag_debug_request(target);
	if (err != ERROR_OK)
		return err;
	err = dsp563xx_restore_context(target);
	if (err != ERROR_OK)
		return err;

	/* reset trace mode */
	err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OSCR, 0x000000);
	if (err != ERROR_OK)
		return err;
	/* enable trace mode */
	err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OSCR, DSP563XX_ONCE_OSCR_TME);
	if (err != ERROR_OK)
		return err;

	cnt = steps;

	/* on JUMP we need one extra cycle */
	if (!current)
		cnt++;

	/* load step counter with N-1 */
	err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OTC, cnt);
	if (err != ERROR_OK)
		return err;

	if (current) {
		/* restore pipeline registers and go */
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR,
				once_regs[ONCE_REG_IDX_OPILR].reg);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR |
				DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO,
				once_regs[ONCE_REG_IDX_OPDBR].reg);
		if (err != ERROR_OK)
			return err;
	} else {
		/* set to go register and jump */
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR, INSTR_JUMP);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_PDBGOTO |
				DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO,
				address);
		if (err != ERROR_OK)
			return err;
	}

	while (1) {
		err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OSCR, &once_status);
		if (err != ERROR_OK)
			return err;

		if (once_status & DSP563XX_ONCE_OSCR_TO) {
			err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABFR, &dr_in);
			if (err != ERROR_OK)
				return err;
			LOG_DEBUG("fetch: %08X", (unsigned) dr_in&0x00ffffff);
			err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABDR, &dr_in);
			if (err != ERROR_OK)
				return err;
			LOG_DEBUG("decode: %08X", (unsigned) dr_in&0x00ffffff);
			err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABEX, &dr_in);
			if (err != ERROR_OK)
				return err;
			LOG_DEBUG("execute: %08X", (unsigned) dr_in&0x00ffffff);

			/* reset trace mode */
			err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OSCR, 0x000000);
			if (err != ERROR_OK)
				return err;

			register_cache_invalidate(dsp563xx->core_cache);
			err = dsp563xx_debug_init(target);
			if (err != ERROR_OK)
				return err;

			break;
		}
	}

	return ERROR_OK;
}

static int dsp563xx_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	err = dsp563xx_step_ex(target, current, address, handle_breakpoints, 0);
	if (err != ERROR_OK)
		return err;

	target->debug_reason = DBG_REASON_SINGLESTEP;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	LOG_INFO("halted: PC: 0x%" PRIx32, dsp563xx->core_regs[DSP563XX_REG_IDX_PC]);

	return err;
}

static int dsp563xx_assert_reset(struct target *target)
{
	int retval = 0;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_HAS_SRST) {
		/* default to asserting srst */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
			jtag_add_reset(1, 1);
		else
			jtag_add_reset(0, 1);
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(5000);

	/* registers are now invalid */
	register_cache_invalidate(dsp563xx->core_cache);

	if (target->reset_halt) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
	}

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int dsp563xx_deassert_reset(struct target *target)
{
	int err;

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	err = dsp563xx_poll(target);
	if (err != ERROR_OK)
		return err;

	if (target->reset_halt) {
		if (target->state == TARGET_HALTED) {
			/* after a reset the cpu jmp to the
			 * reset vector and need 2 cycles to fill
			 * the cache (fetch,decode,excecute)
			 */
			err = dsp563xx_step_ex(target, 1, 0, 1, 1);
			if (err != ERROR_OK)
				return err;
		}
	} else
		target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int dsp563xx_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	int timeout_ms, void *arch_info)
{
	int i;
	int retval = ERROR_OK;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;
		retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size, mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN)
			continue;

		struct reg *reg = register_get_by_name(dsp563xx->core_cache,
				reg_params[i].reg_name,
				0);

		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			continue;
		}

		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
				reg_params[i].reg_name);
			continue;
		}

		retval = dsp563xx_set_core_reg(reg, reg_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	/* exec */
	retval = target_resume(target, 0, entry_point, 1, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT)
			retval = target_read_buffer(target,
					mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {

			struct reg *reg = register_get_by_name(dsp563xx->core_cache,
					reg_params[i].reg_name,
					0);
			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				continue;
			}

			if (reg->size != reg_params[i].size) {
				LOG_ERROR(
					"BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
				continue;
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	return ERROR_OK;
}

/* global command context from openocd.c */
extern struct command_context *global_cmd_ctx;

static int dsp563xx_get_default_memory(void)
{
	Jim_Interp *interp;
	Jim_Obj *memspace;
	char *c;

	if (!global_cmd_ctx)
		return MEM_P;

	interp = global_cmd_ctx->interp;

	if (!interp)
		return MEM_P;

	memspace = Jim_GetGlobalVariableStr(interp, "memspace", JIM_NONE);

	if (!memspace)
		return MEM_P;

	c = (char *)Jim_GetString(memspace, NULL);

	if (!c)
		return MEM_P;

	switch (c[0]) {
		case '1':
			return MEM_X;
		case '2':
			return MEM_Y;
		case '3':
			return MEM_L;
		default:
			break;
	}

	return MEM_P;
}

static int dsp563xx_read_memory_core(struct target *target,
	int mem_type,
	uint32_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	uint32_t i, x;
	uint32_t data, move_cmd = 0;
	uint8_t *b;

	LOG_DEBUG(
		"memtype: %d address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		mem_type,
		address,
		size,
		count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	switch (mem_type) {
		case MEM_X:
			/* TODO: mark effected queued registers */
			move_cmd = 0x61d800;
			break;
		case MEM_Y:
			move_cmd = 0x69d800;
			break;
		case MEM_P:
			move_cmd = 0x07d891;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].valid)
		dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_R0);
	/* we use r1 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R1].valid)
		dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_R1);

	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].dirty = true;
	/* r1 is no longer valid on target */
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R1].dirty = true;

	x = count;
	b = buffer;

	err = dsp563xx_once_execute_dw_ir(target->tap, 1, 0x60F400, address);
	if (err != ERROR_OK)
		return err;

	for (i = 0; i < x; i++) {
		err = dsp563xx_once_execute_sw_ir(target->tap, 0, move_cmd);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_once_execute_sw_ir(target->tap, 0, 0x08D13C);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_once_reg_read(target->tap, 0,
				DSP563XX_ONCE_OGDBR, (uint32_t *)(void *)b);
		if (err != ERROR_OK)
			return err;
		b += 4;
	}

	/* flush the jtag queue */
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	/* walk over the buffer and fix target endianness */
	b = buffer;

	for (i = 0; i < x; i++) {
		data = buf_get_u32(b, 0, 32) & 0x00FFFFFF;
/*		LOG_DEBUG("R: %08X", *((uint32_t*)b)); */
		target_buffer_set_u32(target, b, data);
		b += 4;
	}

	return ERROR_OK;
}

static int dsp563xx_read_memory(struct target *target,
	int mem_type,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer)
{
	int err;
	uint32_t i, i1;
	uint8_t *buffer_y, *buffer_x;

	/* if size equals zero we are called from target read memory
	 * and have to handle the parameter here */
	if ((size == 0) && (count != 0)) {
		size = count % 4;

		if (size)
			LOG_DEBUG("size is not aligned to 4 byte");

		count = (count - size) / 4;
		size = 4;
	}

	/* we only support 4 byte aligned data */
	if ((size != 4) || (!count))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (mem_type != MEM_L)
		return dsp563xx_read_memory_core(target, mem_type, address, size, count, buffer);

	buffer_y = malloc(size * count);
	if (!buffer_y)
		return ERROR_COMMAND_SYNTAX_ERROR;

	buffer_x = malloc(size * count);
	if (!buffer_x) {
		free(buffer_y);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	err = dsp563xx_read_memory_core(target, MEM_Y, address, size, count / 2, buffer_y);

	if (err != ERROR_OK) {
		free(buffer_y);
		free(buffer_x);
		return err;
	}

	err = dsp563xx_read_memory_core(target, MEM_X, address, size, count / 2, buffer_x);

	if (err != ERROR_OK) {
		free(buffer_y);
		free(buffer_x);
		return err;
	}

	for (i = 0, i1 = 0; i < count; i += 2, i1++) {
		buf_set_u32(buffer + i*sizeof(uint32_t), 0, 32,
			buf_get_u32(buffer_y + i1 * sizeof(uint32_t), 0, 32));
			buf_set_u32(buffer + (i + 1) * sizeof(uint32_t), 0, 32,
			buf_get_u32(buffer_x + i1 * sizeof(uint32_t), 0, 32));
	}

	free(buffer_y);
	free(buffer_x);

	return ERROR_OK;
}

static int dsp563xx_read_memory_default(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer)
{

	return dsp563xx_read_memory(target,
			dsp563xx_get_default_memory(), address, size, count, buffer);
}

static int dsp563xx_read_buffer_default(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint8_t *buffer)
{

	return dsp563xx_read_memory(target, dsp563xx_get_default_memory(), address, size, 0,
			buffer);
}

static int dsp563xx_write_memory_core(struct target *target,
	int mem_type,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	uint32_t i, x;
	uint32_t data, move_cmd = 0;
	const uint8_t *b;

	LOG_DEBUG(
		"memtype: %d address: 0x%8.8" TARGET_PRIxADDR ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		mem_type,
		address,
		size,
		count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	switch (mem_type) {
		case MEM_X:
			/* invalidate affected x registers */
			dsp563xx_invalidate_x_context(target, address, address + count - 1);
			move_cmd = 0x615800;
			break;
		case MEM_Y:
			move_cmd = 0x695800;
			break;
		case MEM_P:
			move_cmd = 0x075891;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].valid)
		dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_R0);
	/* we use r1 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R1].valid)
		dsp563xx->read_core_reg(target, DSP563XX_REG_IDX_R1);

	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R0].dirty = true;
	/* r1 is no longer valid on target */
	dsp563xx->core_cache->reg_list[DSP563XX_REG_IDX_R1].dirty = true;

	x = count;
	b = buffer;

	err = dsp563xx_once_execute_dw_ir(target->tap, 1, 0x60F400, address);
	if (err != ERROR_OK)
		return err;

	for (i = 0; i < x; i++) {
		data = target_buffer_get_u32(target, b);

/*		LOG_DEBUG("W: %08X", data); */

		data &= 0x00ffffff;

		err = dsp563xx_once_execute_dw_ir(target->tap, 0, 0x61F400, data);
		if (err != ERROR_OK)
			return err;
		err = dsp563xx_once_execute_sw_ir(target->tap, 0, move_cmd);
		if (err != ERROR_OK)
			return err;
		b += 4;
	}

	/* flush the jtag queue */
	err = jtag_execute_queue();
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int dsp563xx_write_memory(struct target *target,
	int mem_type,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer)
{
	int err;
	uint32_t i, i1;
	uint8_t *buffer_y, *buffer_x;

	/* if size equals zero we are called from target write memory
	 * and have to handle the parameter here */
	if ((size == 0) && (count != 0)) {
		size = count % 4;

		if (size)
			LOG_DEBUG("size is not aligned to 4 byte");

		count = (count - size) / 4;
		size = 4;
	}

	/* we only support 4 byte aligned data */
	if ((size != 4) || (!count))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (mem_type != MEM_L)
		return dsp563xx_write_memory_core(target, mem_type, address, size, count, buffer);

	buffer_y = malloc(size * count);
	if (!buffer_y)
		return ERROR_COMMAND_SYNTAX_ERROR;

	buffer_x = malloc(size * count);
	if (!buffer_x) {
		free(buffer_y);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (i = 0, i1 = 0; i < count; i += 2, i1++) {
		buf_set_u32(buffer_y + i1 * sizeof(uint32_t), 0, 32,
		buf_get_u32(buffer + i * sizeof(uint32_t), 0, 32));
		buf_set_u32(buffer_x + i1 * sizeof(uint32_t), 0, 32,
		buf_get_u32(buffer + (i + 1) * sizeof(uint32_t), 0, 32));
	}

	err = dsp563xx_write_memory_core(target, MEM_Y, address, size, count / 2, buffer_y);

	if (err != ERROR_OK) {
		free(buffer_y);
		free(buffer_x);
		return err;
	}

	err = dsp563xx_write_memory_core(target, MEM_X, address, size, count / 2, buffer_x);

	if (err != ERROR_OK) {
		free(buffer_y);
		free(buffer_x);
		return err;
	}

	free(buffer_y);
	free(buffer_x);

	return ERROR_OK;
}

static int dsp563xx_write_memory_default(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer)
{
	return dsp563xx_write_memory(target,
			dsp563xx_get_default_memory(), address, size, count, buffer);
}

static int dsp563xx_write_buffer_default(struct target *target,
	target_addr_t address,
	uint32_t size,
	const uint8_t *buffer)
{
	return dsp563xx_write_memory(target, dsp563xx_get_default_memory(), address, size, 0,
			buffer);
}

/*
 * Exit with error here, because we support watchpoints over a custom command.
 * This is because the DSP has separate X,Y,P memspace which is not compatible to the
 * traditional watchpoint logic.
 */
static int dsp563xx_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

/*
 * @see dsp563xx_add_watchpoint
 */
static int dsp563xx_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

static int dsp563xx_add_custom_watchpoint(struct target *target, uint32_t address, uint32_t memType,
		enum watchpoint_rw rw, enum watchpoint_condition cond)
{
	int err = ERROR_OK;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	bool wasRunning = false;
	/* Only set breakpoint when halted */
	if (target->state != TARGET_HALTED) {
		dsp563xx_halt(target);
		wasRunning = true;
	}

	if (dsp563xx->hardware_breakpoint[0].used) {
		LOG_ERROR("Cannot add watchpoint. Hardware resource already used.");
		err = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	uint32_t obcr_value = 0;
	if	(err == ERROR_OK) {
		obcr_value |= OBCR_b0_or_b1;
		switch (memType) {
			case MEM_X:
				obcr_value |= OBCR_BP_MEM_X;
				break;
			case MEM_Y:
				obcr_value |= OBCR_BP_MEM_Y;
				break;
			case MEM_P:
				obcr_value |= OBCR_BP_MEM_P;
				break;
			default:
				LOG_ERROR("Unknown memType parameter (%" PRIu32 ")", memType);
				err = ERROR_TARGET_INVALID;
		}
	}

	if (err == ERROR_OK) {
		switch (rw) {
			case WPT_READ:
				obcr_value |= OBCR_BP_0(OBCR_BP_ON_READ);
				break;
			case WPT_WRITE:
				obcr_value |= OBCR_BP_0(OBCR_BP_ON_WRITE);
				break;
			case WPT_ACCESS:
				obcr_value |= OBCR_BP_0(OBCR_BP_ON_READ|OBCR_BP_ON_WRITE);
				break;
			default:
				LOG_ERROR("Unsupported write mode (%d)", rw);
				err = ERROR_TARGET_INVALID;
		}
	}

	if (err == ERROR_OK) {
		switch (cond) {
			case EQUAL:
				obcr_value |= OBCR_BP_0(OBCR_BP_CC_EQUAL);
				break;
			case NOT_EQUAL:
				obcr_value |= OBCR_BP_0(OBCR_BP_CC_NOT_EQUAL);
				break;
			case LESS_THAN:
				obcr_value |= OBCR_BP_0(OBCR_BP_CC_LESS_THAN);
				break;
			case GREATER:
				obcr_value |= OBCR_BP_0(OBCR_BP_CC_GREATER_THAN);
				break;
			default:
				LOG_ERROR("Unsupported condition code (%d)", cond);
				err = ERROR_TARGET_INVALID;
		}
	}

	if (err == ERROR_OK)
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OMLR0, address);

	if (err == ERROR_OK)
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OMLR1, 0x0);

	if (err == ERROR_OK)
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OBCR, obcr_value);

	if (err == ERROR_OK) {
		/* You should write the memory breakpoint counter to 0 */
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OMBC, 0);
	}

	if (err == ERROR_OK) {
		/* You should write the memory breakpoint counter to 0 */
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OTC, 0);
	}

	if (err == ERROR_OK)
		dsp563xx->hardware_breakpoint[0].used = BPU_WATCHPOINT;

	if (err == ERROR_OK && wasRunning) {
		/* Resume from current PC */
		err = dsp563xx_resume(target, 1, 0x0, 0, 0);
	}

	return err;
}

static int dsp563xx_remove_custom_watchpoint(struct target *target)
{
	int err = ERROR_OK;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (dsp563xx->hardware_breakpoint[0].used != BPU_WATCHPOINT) {
		LOG_ERROR("Cannot remove watchpoint, as no watchpoint is currently configured!");
		err = ERROR_TARGET_INVALID;
	}

	if (err == ERROR_OK) {
		/* Clear watchpoint by clearing OBCR. */
		err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OBCR, 0);
	}

	if (err == ERROR_OK)
		dsp563xx->hardware_breakpoint[0].used = BPU_NONE;

	return err;
}

COMMAND_HANDLER(dsp563xx_add_watchpoint_command)
{
	int err = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	uint32_t mem_type = 0;
	switch (CMD_NAME[2]) {
		case 'x':
			mem_type = MEM_X;
			break;
		case 'y':
			mem_type = MEM_Y;
			break;
		case 'p':
			mem_type = MEM_P;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t address = 0;
	if (CMD_ARGC > 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], address);

	enum watchpoint_condition cond;
	switch (CMD_ARGV[0][0]) {
		case '>':
			cond = GREATER;
			break;
		case '<':
			cond = LESS_THAN;
			break;
		case '=':
			cond = EQUAL;
			break;
		case '!':
			cond = NOT_EQUAL;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	enum watchpoint_rw rw;
	switch (CMD_ARGV[1][0]) {
		case 'r':
			rw = WPT_READ;
			break;
		case 'w':
			rw = WPT_WRITE;
			break;
		case 'a':
			rw = WPT_ACCESS;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	err = dsp563xx_add_custom_watchpoint(target, address, mem_type, rw, cond);

	return err;
}

/* Adding a breakpoint using the once breakpoint logic.
 * Note that this mechanism is a true hw breakpoint and is share between the watchpoint logic.
 * This means, you can only have one breakpoint/watchpoint at any time.
 */
static int dsp563xx_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	return dsp563xx_add_custom_watchpoint(target, breakpoint->address, MEM_P, WPT_READ, EQUAL);
}

static int dsp563xx_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	return dsp563xx_remove_custom_watchpoint(target);
}

COMMAND_HANDLER(dsp563xx_remove_watchpoint_command)
{
	struct target *target = get_current_target(CMD_CTX);

	return dsp563xx_remove_custom_watchpoint(target);
}

COMMAND_HANDLER(dsp563xx_mem_command)
{
	struct target *target = get_current_target(CMD_CTX);
	int err = ERROR_OK;
	int read_mem;
	uint32_t address = 0;
	uint32_t count = 1, i;
	uint32_t pattern = 0;
	uint32_t mem_type;
	uint8_t *buffer, *b;

	switch (CMD_NAME[1]) {
		case 'w':
			read_mem = 0;
			break;
		case 'd':
			read_mem = 1;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	switch (CMD_NAME[3]) {
		case 'x':
			mem_type = MEM_X;
			break;
		case 'y':
			mem_type = MEM_Y;
			break;
		case 'p':
			mem_type = MEM_P;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC > 0)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

	if (read_mem == 0) {
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], pattern);
		if (CMD_ARGC > 2)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);
	}

	if (read_mem == 1) {
		if (CMD_ARGC < 1)
			return ERROR_COMMAND_SYNTAX_ERROR;
		if (CMD_ARGC > 1)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], count);
	}

	buffer = calloc(count, sizeof(uint32_t));

	if (read_mem == 1) {
		err = dsp563xx_read_memory(target, mem_type, address, sizeof(uint32_t),
				count, buffer);
		if (err == ERROR_OK)
			target_handle_md_output(CMD, target, address, sizeof(uint32_t), count, buffer);

	} else {
		b = buffer;

		for (i = 0; i < count; i++) {
			target_buffer_set_u32(target, b, pattern);
			b += 4;
		}

		err = dsp563xx_write_memory(target,
				mem_type,
				address,
				sizeof(uint32_t),
				count,
				buffer);
	}

	free(buffer);

	return err;
}

static const struct command_registration dsp563xx_command_handlers[] = {
	{
		.name = "mwwx",
		.handler = dsp563xx_mem_command,
		.mode = COMMAND_EXEC,
		.help = "write x memory words",
		.usage = "address value [count]",
	},
	{
		.name = "mwwy",
		.handler = dsp563xx_mem_command,
		.mode = COMMAND_EXEC,
		.help = "write y memory words",
		.usage = "address value [count]",
	},
	{
		.name = "mwwp",
		.handler = dsp563xx_mem_command,
		.mode = COMMAND_EXEC,
		.help = "write p memory words",
		.usage = "address value [count]",
	},
	{
		.name = "mdwx",
		.handler = dsp563xx_mem_command,
		.mode = COMMAND_EXEC,
		.help = "display x memory words",
		.usage = "address [count]",
	},
	{
		.name = "mdwy",
		.handler = dsp563xx_mem_command,
		.mode = COMMAND_EXEC,
		.help = "display y memory words",
		.usage = "address [count]",
	},
	{
		.name = "mdwp",
		.handler = dsp563xx_mem_command,
		.mode = COMMAND_EXEC,
		.help = "display p memory words",
		.usage = "address [count]",
	},
  /*
   * Watchpoint commands
   */
	{
		.name = "wpp",
		.handler = dsp563xx_add_watchpoint_command,
		.mode = COMMAND_EXEC,
		.help = "Create p memspace watchpoint",
		.usage = "(>|<|=|!) (r|w|a) address",
	},
	{
		.name = "wpx",
		.handler = dsp563xx_add_watchpoint_command,
		.mode = COMMAND_EXEC,
		.help = "Create x memspace watchpoint",
		.usage = "(>|<|=|!) (r|w|a) address",
	},
	{
		.name = "wpy",
		.handler = dsp563xx_add_watchpoint_command,
		.mode = COMMAND_EXEC,
		.help = "Create y memspace watchpoint",
		.usage = "(>|<|=|!) (r|w|a) address",
	},
	{
		.name = "rwpc",
		.handler = dsp563xx_remove_watchpoint_command,
		.mode = COMMAND_EXEC,
		.help = "remove watchpoint custom",
		.usage = " ",
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for DSP563XX targets. */
struct target_type dsp563xx_target = {
	.name = "dsp563xx",

	.poll = dsp563xx_poll,
	.arch_state = dsp563xx_arch_state,

	.get_gdb_reg_list = dsp563xx_get_gdb_reg_list,

	.halt = dsp563xx_halt,
	.resume = dsp563xx_resume,
	.step = dsp563xx_step,

	.assert_reset = dsp563xx_assert_reset,
	.deassert_reset = dsp563xx_deassert_reset,

	.read_memory = dsp563xx_read_memory_default,
	.write_memory = dsp563xx_write_memory_default,

	.read_buffer = dsp563xx_read_buffer_default,
	.write_buffer = dsp563xx_write_buffer_default,

	.run_algorithm = dsp563xx_run_algorithm,

	.add_breakpoint = dsp563xx_add_breakpoint,
	.remove_breakpoint = dsp563xx_remove_breakpoint,
	.add_watchpoint = dsp563xx_add_watchpoint,
	.remove_watchpoint = dsp563xx_remove_watchpoint,

	.commands = dsp563xx_command_handlers,
	.target_create = dsp563xx_target_create,
	.init_target = dsp563xx_init_target,
	.examine = dsp563xx_examine,
};
