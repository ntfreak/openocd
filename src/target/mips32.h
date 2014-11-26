/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef MIPS32_H
#define MIPS32_H

#include "target.h"
#include "mips32_pracc.h"

#define MIPS32_COMMON_MAGIC		0xB320B320

/**
 * Memory segments (32bit kernel mode addresses)
 * These are the traditional names used in the 32-bit universe.
 */
#define KUSEG			0x00000000
#define KSEG0			0x80000000
#define KSEG1			0xa0000000
#define KSEG2			0xc0000000
#define KSEG3			0xe0000000

/** Returns the kernel segment base of a given address */
#define KSEGX(a)		((a) & 0xe0000000)

/** CP0 CONFIG regites fields */
#define MIPS32_CONFIG0_KU_SHIFT 25
#define MIPS32_CONFIG0_KU_MASK (0x7 << MIPS32_CONFIG0_KU_SHIFT)

#define MIPS32_CONFIG0_K0_SHIFT 0
#define MIPS32_CONFIG0_K0_MASK (0x7 << MIPS32_CONFIG0_K0_SHIFT)

#define MIPS32_CONFIG0_K23_SHIFT 28
#define MIPS32_CONFIG0_K23_MASK (0x7 << MIPS32_CONFIG0_K23_SHIFT)

#define MIPS32_CONFIG0_AR_SHIFT 10
#define MIPS32_CONFIG0_AR_MASK (0x7 << MIPS32_CONFIG0_AR_SHIFT)

#define MIPS32_CONFIG1_DL_SHIFT 10
#define MIPS32_CONFIG1_DL_MASK (0x7 << MIPS32_CONFIG1_DL_SHIFT)

#define MIPS32_ARCH_REL1 0x0
#define MIPS32_ARCH_REL2 0x1

#define MIPS32_SCAN_DELAY_LEGACY_MODE 2000000

/* offsets into mips32 core register cache */
enum {
	MIPS32_PC = 37,
	MIPS32NUMCOREREGS
};

#define MIPS32NUMCP0REGS 38
#define MIPS32NUMDSPREGS 7

static const struct {
	unsigned reg;
	unsigned sel;
	const char *name;
} mips32_cp0_regs[MIPS32NUMCP0REGS] = {
	{ 4, 2, "userlocal"},
	{ 7, 0, "hwrena"},
	{ 8, 0, "badvaddr"},
	{ 9, 0, "count"},
	{11, 0, "compare"},
	{12, 0, "status"},
	{12, 1, "intctl"},
	{12, 2, "srsctl"},
	{12, 4, "view_ipl"},
	{13, 0, "cause"},
	{13, 5, "nestedexc"},
	{14, 0, "epc"},
	{14, 2, "nestedepc"},
	{15, 0, "prid"},
	{15, 1, "ebase"},
	{15, 2, "cdmmbase"},
	{16, 0, "config"},
	{16, 1, "config1"},
	{16, 2, "config2"},
	{16, 3, "config3"},
	{16, 4, "config4"},
	{16, 5, "config5"},
	{16, 7, "config7"},
	{17, 0, "lladdr"},
	{23, 0, "debug"},
	{23, 1, "tracecontrol"},
	{23, 2, "tracecontrol2"},
	{23, 3, "usertracedata1"},
	{23, 4, "tracebpc"},
	{24, 0, "depc"},
	{24, 3, "usertracedata2"},
	{25, 0, "perfctl0"},
	{25, 1, "perfcnt0"},
	{25, 2, "perfctl1"},
	{25, 3, "perfcnt1"},
	{26, 0, "errctl"},
	{30, 0, "errorepc"},
	{31, 0, "desave"},
};
enum mips32_isa_mode {
	MIPS32_ISA_MIPS32 = 0,
	MIPS32_ISA_MIPS16E = 1,
};

enum micro_mips_enabled {
	MIPS32_ONLY = 0,
	MICRO_MIPS_ONLY = 1,
	MICRO_MIPS32_16_ONRESET_MIPS32 = 2,
	MICRO_MIPS32_16_ONRESET_MIPS16 = 3,
};

enum dsp {
	DSP_NOT_IMP = 0,
	DSP_IMP = 1,
};

enum dsp_rev {
	DSP_REV1 = 0,
	DSP_REV2 = 1,
};

struct mips32_comparator {
	int used;
	uint32_t bp_value;
	uint32_t reg_address;
};

struct mips32_common {
	uint32_t common_magic;
	void *arch_info;
	struct reg_cache *core_cache;
	struct mips_ejtag ejtag_info;
	uint32_t core_regs[MIPS32NUMCOREREGS];
	enum mips32_isa_mode isa_mode;
	enum micro_mips_enabled mmips;
	enum dsp dsp_implemented;
	enum dsp_rev dsp_rev;

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	int bp_scanned;
	int num_inst_bpoints;
	int num_data_bpoints;
	int num_inst_bpoints_avail;
	int num_data_bpoints_avail;
	struct mips32_comparator *inst_break_list;
	struct mips32_comparator *data_break_list;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, int num);
	int (*write_core_reg)(struct target *target, int num);
};

static inline struct mips32_common *
target_to_mips32(struct target *target)
{
	return target->arch_info;
}

struct mips32_core_reg {
	uint32_t num;
	struct target *target;
	struct mips32_common *mips32_common;
};

struct mips32_algorithm {
	int common_magic;
	enum mips32_isa_mode isa_mode;
};

/*
 * MIPS32r2 Config3 Register (CP0 Register 16, Select 3)
 */
#define CFG3_M			0x80000000		/* Config4 implemented */
#define CFG3_ISAONEXC	0x00010000		/* ISA mode on exception entry */
#define CFG3_ISA_MODE	0x0000C000		/* ISA mode */
#define CFG3_DSP_REV	0x00000800		/* DSP Rev */
#define CFG3_DSPP		0x00000400		/* DSP ASE present */
#define CFG3_LPA		0x00000080		/* Large physical addresses */
#define CFG3_VEIC		0x00000040		/* Vectored external i/u controller */
#define CFG3_VI			0x00000020		/* Vectored i/us */
#define CFG3_SP			0x00000010		/* Small page support */
#define CFG3_MT			0x00000004		/* MT ASE present */
#define CFG3_SM			0x00000002		/* SmartMIPS ASE */
#define CFG3_TL			0x00000001		/* Trace Logic */

#define MIPS32_OP_ADDIU 0x21
#define MIPS32_OP_ANDI	0x0C
#define MIPS32_OP_BEQ	0x04
#define MIPS32_OP_BGTZ	0x07
#define MIPS32_OP_BNE	0x05
#define MIPS32_OP_ADDI	0x08
#define MIPS32_OP_AND	0x24
#define MIPS32_OP_CACHE	0x2F
#define MIPS32_OP_COP0	0x10
#define MIPS32_OP_EXT	0x1F
#define MIPS32_OP_J	0x02
#define MIPS32_OP_JR	0x08
#define MIPS32_OP_LUI	0x0F
#define MIPS32_OP_LW	0x23
#define MIPS32_OP_LBU	0x24
#define MIPS32_OP_LHU	0x25
#define MIPS32_OP_MFHI	0x10
#define MIPS32_OP_MTHI	0x11
#define MIPS32_OP_MFLO	0x12
#define MIPS32_OP_MUL	0x2
#define MIPS32_OP_MTLO	0x13
#define MIPS32_OP_RDHWR 0x3B
#define MIPS32_OP_SB	0x28
#define MIPS32_OP_SH	0x29
#define MIPS32_OP_SW	0x2B
#define MIPS32_OP_ORI	0x0D
#define MIPS32_OP_XORI	0x0E
#define MIPS32_OP_XOR	0x26
#define MIPS32_OP_SLTU  0x2B
#define MIPS32_OP_SLLV	0x04
#define MIPS32_OP_SRL	0x03
#define MIPS32_OP_SYNCI	0x1F

#define MIPS32_OP_REGIMM	0x01
#define MIPS32_OP_SDBBP		0x3F
#define MIPS32_OP_SPECIAL	0x00
#define MIPS32_OP_SPECIAL2	0x07
#define MIPS32_OP_SPECIAL3	0x1F

#define MIPS32_COP0_MF	0x00
#define MIPS32_COP0_MT	0x04

#define MIPS32_R_INST(opcode, rs, rt, rd, shamt, funct) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | ((rd) << 11) | ((shamt) << 6) | (funct))
#define MIPS32_I_INST(opcode, rs, rt, immd) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | (immd))
#define MIPS32_J_INST(opcode, addr)	(((opcode) << 26) | (addr))

#define MIPS32_NOP						0
#define MIPS32_ADD(dst, src, tar)		MIPS32_R_INST(0, src, tar, dst, 0, 32)
#define MIPS32_ADDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ADDI, src, tar, val)
#define MIPS32_ADDIU(tar, src, val)		MIPS32_I_INST(9, src, tar, val)
#define MIPS32_ADDU(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_ADDIU)
#define MIPS32_AND(reg, off, val)		MIPS32_R_INST(0, off, val, reg, 0, MIPS32_OP_AND)
#define MIPS32_ANDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ANDI, src, tar, val)
#define MIPS32_B(off)					MIPS32_BEQ(0, 0, off)
#define MIPS32_BEQ(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BEQ, src, tar, off)
#define MIPS32_BGTZ(reg, off)			MIPS32_I_INST(MIPS32_OP_BGTZ, reg, 0, off)
#define MIPS32_BNE(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BNE, src, tar, off)
#define MIPS32_CACHE(op, off, base)		MIPS32_I_INST(MIPS32_OP_CACHE, base, op, off)
#define MIPS32_EXT(dst, src, shf, sz)	MIPS32_R_INST(MIPS32_OP_EXT, src, dst, (sz-1), shf, 0)
#define MIPS32_J(tar)				MIPS32_J_INST(MIPS32_OP_J, tar)
#define MIPS32_JR(reg)					MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_JR)
#define MIPS32_MFC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MF, gpr, cpr, 0, sel)
#define MIPS32_MOVE(dst, src)			MIPS32_R_INST(17, 16, 0, src, dst, 6)
#define MIPS32_MTC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MT, gpr, cpr, 0, sel)
#define MIPS32_LBU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LBU, base, reg, off)
#define MIPS32_LHU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LHU, base, reg, off)
#define MIPS32_LUI(reg, val)			MIPS32_I_INST(MIPS32_OP_LUI, 0, reg, val)
#define MIPS32_LW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LW, base, reg, off)
#define MIPS32_MFLO(reg)				MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFLO)
#define MIPS32_MFHI(reg)				MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFHI)
#define MIPS32_MTLO(reg)				MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTLO)
#define MIPS32_MTHI(reg)				MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTHI)
#define MIPS32_MUL(dst, src, t)			MIPS32_R_INST(28, src, t, dst, 0, MIPS32_OP_MUL)
#define MIPS32_OR(dst, src, val)		MIPS32_R_INST(0, src, val, dst, 0, 37)
#define MIPS32_ORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ORI, src, tar, val)
#define MIPS32_XORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_XORI, src, tar, val)
#define MIPS32_RDHWR(tar, dst)			MIPS32_R_INST(MIPS32_OP_SPECIAL3, 0, tar, dst, 0, MIPS32_OP_RDHWR)
#define MIPS32_SB(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SB, base, reg, off)
#define MIPS32_SH(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SH, base, reg, off)
#define MIPS32_SW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SW, base, reg, off)
#define MIPS32_XOR(reg, val1, val2)		MIPS32_R_INST(0, val1, val2, reg, 0, MIPS32_OP_XOR)
#define MIPS32_SRL(reg, src, off)		MIPS32_R_INST(0, 0, src, reg, off, MIPS32_OP_SRL)
#define MIPS32_SLTU(dst, tar, src)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_SLTU)
#define MIPS32_SLLV(dst, tar, src)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_SLLV)
#define MIPS32_SYNCI(off, base)			MIPS32_I_INST(MIPS32_OP_REGIMM, base, MIPS32_OP_SYNCI, off)

#define MIPS32_SYNC			0xF
#define MIPS32_SYNCI_STEP	0x1	/* reg num od address step size to be used with synci instruction */

/**
 * Cache operations definietions
 * Operation field is 5 bits long :
 * 1) bits 1..0 hold cache type
 * 2) bits 4..2 hold operation code
 */
#define MIPS32_CACHE_D_HIT_WRITEBACK ((0x1 << 0) | (0x6 << 2))
#define MIPS32_CACHE_I_HIT_INVALIDATE ((0x0 << 0) | (0x4 << 2))

/* ejtag specific instructions */
#define MIPS32_DRET					0x4200001F
#define MIPS32_SDBBP				0x7000003F	/* MIPS32_J_INST(MIPS32_OP_SPECIAL2, MIPS32_OP_SDBBP) */
#define MIPS16_SDBBP				0xE801
#define MICRO_MIPS32_SDBBP			0x000046C0
#define MICRO_MIPS_SDBBP			0x46C0
#define MIPS32_DSP_ENABLE	0x1000000

#define MICRO_MIPS_OP_MFHI	0x007C
#define MICRO_MIPS_OP_MFLO	0x107C

#define MICRO_MIPS_OP_MTHI	0x207C
#define MICRO_MIPS_OP_MTLO	0x307C

#define MICRO_DSP_I_INST(rs, ac, opcode) \
	((0 << 21) | ((rs) << 16) | ((ac) << 14) | (opcode))

#define MICRO_DSP_MFLO(reg, ac)	MICRO_DSP_I_INST(reg, ac, MICRO_MIPS_OP_MFLO)
#define MICRO_DSP_MFHI(reg, ac)	MICRO_DSP_I_INST(reg, ac, MICRO_MIPS_OP_MFHI)
#define MICRO_DSP_MTLO(reg, ac)	MICRO_DSP_I_INST(reg, ac, MICRO_MIPS_OP_MTLO)
#define MICRO_DSP_MTHI(reg, ac)	MICRO_DSP_I_INST(reg, ac, MICRO_MIPS_OP_MTHI)


#define MICRO_MIPS_OP_RDDSP		0x067C
#define MICRO_MIPS_OP_WRDSP		0x167C

#define MICRO_DSP_R_INST(rt, mask, opcode) \
	((0 << 26) | ((rt) << 21) | ((mask) << 14) | (opcode))

#define MICRO_DSP_RDDSP(rt, mask)	MICRO_DSP_R_INST(rt, mask, MICRO_MIPS_OP_RDDSP)
#define MICRO_DSP_WRDSP(rt, mask)	MICRO_DSP_R_INST(rt, mask, MICRO_MIPS_OP_WRDSP)

extern const struct command_registration mips32_command_handlers[];

int mips32_arch_state(struct target *target);

int mips32_init_arch_info(struct target *target,
		struct mips32_common *mips32, struct jtag_tap *tap);

int mips32_restore_context(struct target *target);
int mips32_save_context(struct target *target);

struct reg_cache *mips32_build_reg_cache(struct target *target);

int mips32_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		uint32_t entry_point, uint32_t exit_point,
		int timeout_ms, void *arch_info);

int mips32_configure_break_unit(struct target *target);

int mips32_enable_interrupts(struct target *target, int enable);

int mips32_examine(struct target *target);

int mips32_register_commands(struct command_context *cmd_ctx);

int mips32_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);
int mips32_checksum_memory(struct target *target, uint32_t address,
		uint32_t count, uint32_t *checksum);
int mips32_blank_check_memory(struct target *target,
		uint32_t address, uint32_t count, uint32_t *blank);

int mips32_cp0_command(struct command_invocation *cmd);
int mips32_scan_delay_command(struct command_invocation *cmd);
#endif	/*MIPS32_H*/
