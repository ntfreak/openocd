/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Support for processors implementing MIPS64 instruction set
 *
 * Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>
 * Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>
 * Copyright (C) 2014-2019 by Peter Mamonov <pmamonov@gmail.com>
 *
 * Based on the work of:
 *     Copyright (C) 2008 by Spencer Oliver
 *     Copyright (C) 2008 by David T.L. Wong
 *     Copyright (C) 2010 by Konstantin Kostyukhin, Nikolay Shmyrev
 */

#ifndef OPENOCD_TARGET_MIPS64_H
#define OPENOCD_TARGET_MIPS64_H

#include "target.h"
#include "register.h"
#include "mips64_pracc.h"

#define MIPS64_COMMON_MAGIC		0xB640B640

/* MIPS64 CP0 registers */
#define MIPS64_C0_INDEX		0
#define MIPS64_C0_RANDOM	1
#define MIPS64_C0_ENTRYLO0	2
#define MIPS64_C0_ENTRYLO1	3
#define MIPS64_C0_CONTEXT	4
#define MIPS64_C0_PAGEMASK	5
#define MIPS64_C0_WIRED		6
#define MIPS64_C0_BADVADDR	8
#define MIPS64_C0_COUNT		9
#define MIPS64_C0_ENTRYHI	10
#define MIPS64_C0_COMPARE	11
#define MIPS64_C0_STATUS	12
#define MIPS64_C0_CAUSE		13
#define MIPS64_C0_EPC		14
#define MIPS64_C0_PRID		15
#define MIPS64_C0_CONFIG	16
#define MIPS64_C0_LLA		17
#define MIPS64_C0_WATCHLO	18
#define MIPS64_C0_WATCHHI	19
#define MIPS64_C0_XCONTEXT	20
#define MIPS64_C0_MEMCTRL	22
#define MIPS64_C0_DEBUG		23
#define MIPS64_C0_DEPC		24
#define MIPS64_C0_PERFCOUNT	25
#define MIPS64_C0_ECC		26
#define MIPS64_C0_CACHERR	27
#define MIPS64_C0_TAGLO		28
#define MIPS64_C0_TAGHI		29
#define MIPS64_C0_DATAHI	29
#define MIPS64_C0_EEPC		30

/* MIPS64 CP1 registers */
#define MIPS64_C1_FIR		0
#define MIPS64_C1_FCONFIG	24
#define MIPS64_C1_FCSR		31
#define MIPS64_C1_FCCR		25
#define MIPS64_C1_FEXR		26
#define MIPS64_C1_FENR		28

/* offsets into mips64 register cache */
#define MIPS64_NUM_CORE_REGS	34
#define MIPS64_NUM_C0_REGS	34
#define MIPS64_NUM_FP_REGS	38

#define MIPS64_NUM_REGS		(MIPS64_NUM_CORE_REGS + \
				 MIPS64_NUM_C0_REGS + \
				 MIPS64_NUM_FP_REGS)

#define MIPS64_NUM_CORE_C0_REGS	(MIPS64_NUM_CORE_REGS + MIPS64_NUM_C0_REGS)

#define MIPS64_PC		MIPS64_NUM_CORE_REGS

struct mips64_comparator {
	bool used;
	uint64_t bp_value;
	uint64_t reg_address;
};

struct mips64_common {
	uint32_t common_magic;
	void *arch_info;
	struct reg_cache *core_cache;
	struct mips_ejtag ejtag_info;
	uint64_t core_regs[MIPS64_NUM_REGS];

	struct working_area *fast_data_area;

	bool bp_scanned;
	int num_inst_bpoints;
	int num_data_bpoints;
	int num_inst_bpoints_avail;
	int num_data_bpoints_avail;
	struct mips64_comparator *inst_break_list;
	struct mips64_comparator *data_break_list;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, int num);
	int (*write_core_reg)(struct target *target, int num);

	bool mips64mode32;
};

struct mips64_core_reg {
	uint32_t num;
	struct target *target;
	struct mips64_common *mips64_common;
	uint8_t value[8];
	struct reg_feature feature;
	struct reg_data_type reg_data_type;
};

#define MIPS64_OP_SRL	0x02
#define MIPS64_OP_BEQ	0x04
#define MIPS64_OP_BNE	0x05
#define MIPS64_OP_ADDI	0x08
#define MIPS64_OP_ANDI	0x0c
#define MIPS64_OP_DADDI	0x18
#define MIPS64_OP_DADDIU	0x19
#define MIPS64_OP_AND	0x24
#define MIPS64_OP_LUI	0x0F
#define MIPS64_OP_LW	0x23
#define MIPS64_OP_LD	0x37
#define MIPS64_OP_LBU	0x24
#define MIPS64_OP_LHU	0x25
#define MIPS64_OP_MFHI	0x10
#define MIPS64_OP_MTHI	0x11
#define MIPS64_OP_MFLO	0x12
#define MIPS64_OP_MTLO	0x13
#define MIPS64_OP_SB	0x28
#define MIPS64_OP_SH	0x29
#define MIPS64_OP_SW	0x2B
#define MIPS64_OP_SD	0x3F
#define MIPS64_OP_ORI	0x0D
#define MIPS64_OP_JR	0x08

#define MIPS64_OP_COP0	0x10
#define MIPS64_OP_COP1	0x11
#define MIPS64_OP_COP2	0x12

#define MIPS64_COP_MF	0x00
#define MIPS64_COP_DMF	0x01
#define MIPS64_COP_MT	0x04
#define MIPS64_COP_DMT	0x05
#define MIPS64_COP_CF	0x02
#define MIPS64_COP_CT	0x06

#define MIPS64_R_INST(opcode, rs, rt, rd, shamt, funct) \
(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | ((rd) << 11) | ((shamt) << 6) | (funct))
#define MIPS64_I_INST(opcode, rs, rt, immd)	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | (immd))
#define MIPS64_J_INST(opcode, addr)	(((opcode) << 26) | (addr))

#define MIPS64_NOP			0
#define MIPS64_ADDI(tar, src, val)	MIPS64_I_INST(MIPS64_OP_ADDI, src, tar, val)
#define MIPS64_DADDI(tar, src, val)	MIPS64_I_INST(MIPS64_OP_DADDI, src, tar, val)
#define MIPS64_DADDIU(tar, src, val)	MIPS64_I_INST(MIPS64_OP_DADDIU, src, tar, val)
#define MIPS64_AND(reg, off, val)	MIPS64_R_INST(0, off, val, reg, 0, MIPS64_OP_AND)
#define MIPS64_ANDI(d, s, im)		MIPS64_I_INST(MIPS64_OP_ANDI, s, d, im)
#define MIPS64_SRL(d, w, sh)		MIPS64_R_INST(0, 0, w, d, sh, MIPS64_OP_SRL)
#define MIPS64_B(off)			MIPS64_BEQ(0, 0, off)
#define MIPS64_BEQ(src, tar, off)	MIPS64_I_INST(MIPS64_OP_BEQ, src, tar, off)
#define MIPS64_BNE(src, tar, off)	MIPS64_I_INST(MIPS64_OP_BNE, src, tar, off)
#define MIPS64_MFC0(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP0, MIPS64_COP_MF, gpr, cpr, 0, sel)
#define MIPS64_DMFC0(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP0, MIPS64_COP_DMF, gpr, cpr, 0, sel)
#define MIPS64_MTC0(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP0, MIPS64_COP_MT, gpr, cpr, 0, sel)
#define MIPS64_DMTC0(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP0, MIPS64_COP_DMT, gpr, cpr, 0, sel)
#define MIPS64_MFC1(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP1, MIPS64_COP_MF, gpr, cpr, 0, 0)
#define MIPS64_DMFC1(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP1, MIPS64_COP_DMF, gpr, cpr, 0, 0)
#define MIPS64_MTC1(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP1, MIPS64_COP_MT, gpr, cpr, 0, 0)
#define MIPS64_DMTC1(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP1, MIPS64_COP_DMT, gpr, cpr, 0, 0)
#define MIPS64_MFC2(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP2, MIPS64_COP_MF, gpr, cpr, 0, sel)
#define MIPS64_MTC2(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP2, MIPS64_COP_MT, gpr, cpr, 0, sel)
#define MIPS64_CFC1(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP1, MIPS64_COP_CF, gpr, cpr, 0, 0)
#define MIPS64_CTC1(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP1, MIPS64_COP_CT, gpr, cpr, 0, 0)
#define MIPS64_CFC2(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP2, MIPS64_COP_CF, gpr, cpr, 0, sel)
#define MIPS64_CTC2(gpr, cpr, sel)	MIPS64_R_INST(MIPS64_OP_COP2, MIPS64_COP_CT, gpr, cpr, 0, sel)
#define MIPS64_LBU(reg, off, base)	MIPS64_I_INST(MIPS64_OP_LBU, base, reg, off)
#define MIPS64_LHU(reg, off, base)	MIPS64_I_INST(MIPS64_OP_LHU, base, reg, off)
#define MIPS64_LUI(reg, val)		MIPS64_I_INST(MIPS64_OP_LUI, 0, reg, val)
#define MIPS64_LW(reg, off, base)	MIPS64_I_INST(MIPS64_OP_LW, base, reg, off)
#define MIPS64_LD(reg, off, base)	MIPS64_I_INST(MIPS64_OP_LD, base, reg, off)
#define MIPS64_MFLO(reg)		MIPS64_R_INST(0, 0, 0, reg, 0, MIPS64_OP_MFLO)
#define MIPS64_MFHI(reg)		MIPS64_R_INST(0, 0, 0, reg, 0, MIPS64_OP_MFHI)
#define MIPS64_MTLO(reg)		MIPS64_R_INST(0, reg, 0, 0, 0, MIPS64_OP_MTLO)
#define MIPS64_MTHI(reg)		MIPS64_R_INST(0, reg, 0, 0, 0, MIPS64_OP_MTHI)
#define MIPS64_ORI(src, tar, val)	MIPS64_I_INST(MIPS64_OP_ORI, src, tar, val)
#define MIPS64_SB(reg, off, base)	MIPS64_I_INST(MIPS64_OP_SB, base, reg, off)
#define MIPS64_SH(reg, off, base)	MIPS64_I_INST(MIPS64_OP_SH, base, reg, off)
#define MIPS64_SW(reg, off, base)	MIPS64_I_INST(MIPS64_OP_SW, base, reg, off)
#define MIPS64_SD(reg, off, base)	MIPS64_I_INST(MIPS64_OP_SD, base, reg, off)
#define MIPS64_CACHE(op, reg, off)	(47 << 26 | (reg) << 21 | (op) << 16 | (off))
#define MIPS64_SYNCI(reg, off)		(1 << 26 | (reg) << 21 | 0x1f << 16 | (off))
#define MIPS64_JR(reg)			MIPS64_R_INST(0, reg, 0, 0, 0, MIPS64_OP_JR)

/* ejtag specific instructions */
#define MIPS64_DRET			0x4200001F
#define MIPS64_SDBBP			0x7000003F
#define MIPS64_SDBBP_LE			0x3f000007
#define MIPS64_SDBBP_SIZE		4
#define MIPS16_SDBBP_SIZE		2

#define MIPS64_SYNC			0x0000000F

int mips64_arch_state(struct target *target);
int mips64_init_arch_info(struct target *target, struct mips64_common *mips64, struct jtag_tap *tap);
int mips64_restore_context(struct target *target);
int mips64_save_context(struct target *target);
int mips64_build_reg_cache(struct target *target);
int mips64_run_algorithm(struct target *target, int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	int timeout_ms, void *arch_info);
int mips64_configure_break_unit(struct target *target);
int mips64_enable_interrupts(struct target *target, bool enable);
int mips64_examine(struct target *target);

int mips64_register_commands(struct command_context *cmd_ctx);
int mips64_invalidate_core_regs(struct target *target);
int mips64_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class);

#endif	/* OPENOCD_TARGET_MIPS64_H */
