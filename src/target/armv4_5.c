/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by Oyvind Harboe                                   *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   <ilg@livius.net>                                                      *
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
#include "arm_jtag.h"
#include "breakpoints.h"
#include "arm_disassembler.h"
#include <helper/binarybuffer.h>
#include "algorithm.h"
#include "register.h"
#include "semihosting_common.h"

/* offsets into armv4_5 core register cache */
enum {
/*	ARMV4_5_CPSR = 31, */
	ARMV4_5_SPSR_FIQ = 32,
	ARMV4_5_SPSR_IRQ = 33,
	ARMV4_5_SPSR_SVC = 34,
	ARMV4_5_SPSR_ABT = 35,
	ARMV4_5_SPSR_UND = 36,
	ARM_SPSR_MON = 41,
	ARM_SPSR_HYP = 43,
};

static const uint8_t arm_usr_indices[17] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, ARMV4_5_CPSR,
};

static const uint8_t arm_fiq_indices[8] = {
	16, 17, 18, 19, 20, 21, 22, ARMV4_5_SPSR_FIQ,
};

static const uint8_t arm_irq_indices[3] = {
	23, 24, ARMV4_5_SPSR_IRQ,
};

static const uint8_t arm_svc_indices[3] = {
	25, 26, ARMV4_5_SPSR_SVC,
};

static const uint8_t arm_abt_indices[3] = {
	27, 28, ARMV4_5_SPSR_ABT,
};

static const uint8_t arm_und_indices[3] = {
	29, 30, ARMV4_5_SPSR_UND,
};

static const uint8_t arm_mon_indices[3] = {
	39, 40, ARM_SPSR_MON,
};

static const uint8_t arm_hyp_indices[2] = {
	42, ARM_SPSR_HYP,
};

static const struct {
	const char *name;
	unsigned short psr;
	/* For user and system modes, these list indices for all registers.
	 * otherwise they're just indices for the shadow registers and SPSR.
	 */
	unsigned short n_indices;
	const uint8_t *indices;
} arm_mode_data[] = {
	/* Seven modes are standard from ARM7 on. "System" and "User" share
	 * the same registers; other modes shadow from 3 to 8 registers.
	 */
	{
		.name = "User",
		.psr = ARM_MODE_USR,
		.n_indices = ARRAY_SIZE(arm_usr_indices),
		.indices = arm_usr_indices,
	},
	{
		.name = "FIQ",
		.psr = ARM_MODE_FIQ,
		.n_indices = ARRAY_SIZE(arm_fiq_indices),
		.indices = arm_fiq_indices,
	},
	{
		.name = "Supervisor",
		.psr = ARM_MODE_SVC,
		.n_indices = ARRAY_SIZE(arm_svc_indices),
		.indices = arm_svc_indices,
	},
	{
		.name = "Abort",
		.psr = ARM_MODE_ABT,
		.n_indices = ARRAY_SIZE(arm_abt_indices),
		.indices = arm_abt_indices,
	},
	{
		.name = "IRQ",
		.psr = ARM_MODE_IRQ,
		.n_indices = ARRAY_SIZE(arm_irq_indices),
		.indices = arm_irq_indices,
	},
	{
		.name = "Undefined instruction",
		.psr = ARM_MODE_UND,
		.n_indices = ARRAY_SIZE(arm_und_indices),
		.indices = arm_und_indices,
	},
	{
		.name = "System",
		.psr = ARM_MODE_SYS,
		.n_indices = ARRAY_SIZE(arm_usr_indices),
		.indices = arm_usr_indices,
	},
	/* TrustZone "Security Extensions" add a secure monitor mode.
	 * This is distinct from a "debug monitor" which can support
	 * non-halting debug, in conjunction with some debuggers.
	 */
	{
		.name = "Secure Monitor",
		.psr = ARM_MODE_MON,
		.n_indices = ARRAY_SIZE(arm_mon_indices),
		.indices = arm_mon_indices,
	},
	{
		.name = "Secure Monitor ARM1176JZF-S",
		.psr = ARM_MODE_1176_MON,
		.n_indices = ARRAY_SIZE(arm_mon_indices),
		.indices = arm_mon_indices,
	},

	/* These special modes are currently only supported
	 * by ARMv6M and ARMv7M profiles */
	{
		.name = "Thread",
		.psr = ARM_MODE_THREAD,
	},
	{
		.name = "Thread (User)",
		.psr = ARM_MODE_USER_THREAD,
	},
	{
		.name = "Handler",
		.psr = ARM_MODE_HANDLER,
	},

	/* armv7-a with virtualization extension */
	{
		.name = "Hypervisor",
		.psr = ARM_MODE_HYP,
		.n_indices = ARRAY_SIZE(arm_hyp_indices),
		.indices = arm_hyp_indices,
	},
};

/** Map PSR mode bits to the name of an ARM processor operating mode. */
const char *arm_mode_name(unsigned psr_mode)
{
	for (unsigned i = 0; i < ARRAY_SIZE(arm_mode_data); i++) {
		if (arm_mode_data[i].psr == psr_mode)
			return arm_mode_data[i].name;
	}
	LOG_ERROR("unrecognized psr mode: %#02x", psr_mode);
	return "UNRECOGNIZED";
}

/** Return true iff the parameter denotes a valid ARM processor mode. */
bool is_arm_mode(unsigned psr_mode)
{
	for (unsigned i = 0; i < ARRAY_SIZE(arm_mode_data); i++) {
		if (arm_mode_data[i].psr == psr_mode)
			return true;
	}
	return false;
}

/** Map PSR mode bits to linear number indexing armv4_5_core_reg_map */
int arm_mode_to_number(enum arm_mode mode)
{
	switch (mode) {
		case ARM_MODE_ANY:
		/* map MODE_ANY to user mode */
		case ARM_MODE_USR:
			return 0;
		case ARM_MODE_FIQ:
			return 1;
		case ARM_MODE_IRQ:
			return 2;
		case ARM_MODE_SVC:
			return 3;
		case ARM_MODE_ABT:
			return 4;
		case ARM_MODE_UND:
			return 5;
		case ARM_MODE_SYS:
			return 6;
		case ARM_MODE_MON:
		case ARM_MODE_1176_MON:
			return 7;
		case ARM_MODE_HYP:
			return 8;
		default:
			LOG_ERROR("invalid mode value encountered %d", mode);
			return -1;
	}
}

/** Map linear number indexing armv4_5_core_reg_map to PSR mode bits. */
enum arm_mode armv4_5_number_to_mode(int number)
{
	switch (number) {
		case 0:
			return ARM_MODE_USR;
		case 1:
			return ARM_MODE_FIQ;
		case 2:
			return ARM_MODE_IRQ;
		case 3:
			return ARM_MODE_SVC;
		case 4:
			return ARM_MODE_ABT;
		case 5:
			return ARM_MODE_UND;
		case 6:
			return ARM_MODE_SYS;
		case 7:
			return ARM_MODE_MON;
		case 8:
			return ARM_MODE_HYP;
		default:
			LOG_ERROR("mode index out of bounds %d", number);
			return ARM_MODE_ANY;
	}
}

static const char *arm_state_strings[] = {
	"ARM", "Thumb", "Jazelle", "ThumbEE",
};

/* Templates for ARM core registers.
 *
 * NOTE:  offsets in this table are coupled to the arm_mode_data
 * table above, the armv4_5_core_reg_map array below, and also to
 * the ARMV4_5_CPSR symbol (which should vanish after ARM11 updates).
 */
static const struct {
	/* The name is used for e.g. the "regs" command. */
	const char *name;

	/* The {cookie, mode} tuple uniquely identifies one register.
	 * In a given mode, cookies 0..15 map to registers R0..R15,
	 * with R13..R15 usually called SP, LR, PC.
	 *
	 * MODE_ANY is used as *input* to the mapping, and indicates
	 * various special cases (sigh) and errors.
	 *
	 * Cookie 16 is (currently) confusing, since it indicates
	 * CPSR -or- SPSR depending on whether 'mode' is MODE_ANY.
	 * (Exception modes have both CPSR and SPSR registers ...)
	 */
	unsigned cookie;
	unsigned gdb_index;
	enum arm_mode mode;
} arm_core_regs[] = {
	/* IMPORTANT:  we guarantee that the first eight cached registers
	 * correspond to r0..r7, and the fifteenth to PC, so that callers
	 * don't need to map them.
	 */
	[0] = { .name = "r0", .cookie = 0, .mode = ARM_MODE_ANY, .gdb_index = 0, },
	[1] = { .name = "r1", .cookie = 1, .mode = ARM_MODE_ANY, .gdb_index = 1, },
	[2] = { .name = "r2", .cookie = 2, .mode = ARM_MODE_ANY, .gdb_index = 2, },
	[3] = { .name = "r3", .cookie = 3, .mode = ARM_MODE_ANY, .gdb_index = 3, },
	[4] = { .name = "r4", .cookie = 4, .mode = ARM_MODE_ANY, .gdb_index = 4, },
	[5] = { .name = "r5", .cookie = 5, .mode = ARM_MODE_ANY, .gdb_index = 5, },
	[6] = { .name = "r6", .cookie = 6, .mode = ARM_MODE_ANY, .gdb_index = 6, },
	[7] = { .name = "r7", .cookie = 7, .mode = ARM_MODE_ANY, .gdb_index = 7, },

	/* NOTE: regs 8..12 might be shadowed by FIQ ... flagging
	 * them as MODE_ANY creates special cases.  (ANY means
	 * "not mapped" elsewhere; here it's "everything but FIQ".)
	 */
	[8] = { .name = "r8", .cookie = 8, .mode = ARM_MODE_ANY, .gdb_index = 8, },
	[9] = { .name = "r9", .cookie = 9, .mode = ARM_MODE_ANY, .gdb_index = 9, },
	[10] = { .name = "r10", .cookie = 10, .mode = ARM_MODE_ANY, .gdb_index = 10, },
	[11] = { .name = "r11", .cookie = 11, .mode = ARM_MODE_ANY, .gdb_index = 11, },
	[12] = { .name = "r12", .cookie = 12, .mode = ARM_MODE_ANY, .gdb_index = 12, },

	/* Historical GDB mapping of indices:
	 *  - 13-14 are sp and lr, but banked counterparts are used
	 *  - 16-24 are left for deprecated 8 FPA + 1 FPS
	 *  - 25 is the cpsr
	 */

	/* NOTE all MODE_USR registers are equivalent to MODE_SYS ones */
	[13] = { .name = "sp_usr", .cookie = 13, .mode = ARM_MODE_USR, .gdb_index = 26, },
	[14] = { .name = "lr_usr", .cookie = 14, .mode = ARM_MODE_USR, .gdb_index = 27, },

	/* guaranteed to be at index 15 */
	[15] = { .name = "pc", .cookie = 15, .mode = ARM_MODE_ANY, .gdb_index = 15, },
	[16] = { .name = "r8_fiq", .cookie = 8, .mode = ARM_MODE_FIQ, .gdb_index = 28, },
	[17] = { .name = "r9_fiq", .cookie = 9, .mode = ARM_MODE_FIQ, .gdb_index = 29, },
	[18] = { .name = "r10_fiq", .cookie = 10, .mode = ARM_MODE_FIQ, .gdb_index = 30, },
	[19] = { .name = "r11_fiq", .cookie = 11, .mode = ARM_MODE_FIQ, .gdb_index = 31, },
	[20] = { .name = "r12_fiq", .cookie = 12, .mode = ARM_MODE_FIQ, .gdb_index = 32, },

	[21] = { .name = "sp_fiq", .cookie = 13, .mode = ARM_MODE_FIQ, .gdb_index = 33, },
	[22] = { .name = "lr_fiq", .cookie = 14, .mode = ARM_MODE_FIQ, .gdb_index = 34, },

	[23] = { .name = "sp_irq", .cookie = 13, .mode = ARM_MODE_IRQ, .gdb_index = 35, },
	[24] = { .name = "lr_irq", .cookie = 14, .mode = ARM_MODE_IRQ, .gdb_index = 36, },

	[25] = { .name = "sp_svc", .cookie = 13, .mode = ARM_MODE_SVC, .gdb_index = 37, },
	[26] = { .name = "lr_svc", .cookie = 14, .mode = ARM_MODE_SVC, .gdb_index = 38, },

	[27] = { .name = "sp_abt", .cookie = 13, .mode = ARM_MODE_ABT, .gdb_index = 39, },
	[28] = { .name = "lr_abt", .cookie = 14, .mode = ARM_MODE_ABT, .gdb_index = 40, },

	[29] = { .name = "sp_und", .cookie = 13, .mode = ARM_MODE_UND, .gdb_index = 41, },
	[30] = { .name = "lr_und", .cookie = 14, .mode = ARM_MODE_UND, .gdb_index = 42, },

	[31] = { .name = "cpsr", .cookie = 16, .mode = ARM_MODE_ANY, .gdb_index = 25, },
	[32] = { .name = "spsr_fiq", .cookie = 16, .mode = ARM_MODE_FIQ, .gdb_index = 43, },
	[33] = { .name = "spsr_irq", .cookie = 16, .mode = ARM_MODE_IRQ, .gdb_index = 44, },
	[34] = { .name = "spsr_svc", .cookie = 16, .mode = ARM_MODE_SVC, .gdb_index = 45, },
	[35] = { .name = "spsr_abt", .cookie = 16, .mode = ARM_MODE_ABT, .gdb_index = 46, },
	[36] = { .name = "spsr_und", .cookie = 16, .mode = ARM_MODE_UND, .gdb_index = 47, },

	/* These are only used for GDB target description, banked registers are accessed instead */
	[37] = { .name = "sp", .cookie = 13, .mode = ARM_MODE_ANY, .gdb_index = 13, },
	[38] = { .name = "lr", .cookie = 14, .mode = ARM_MODE_ANY, .gdb_index = 14, },

	/* These exist only when the Security Extension (TrustZone) is present */
	[39] = { .name = "sp_mon", .cookie = 13, .mode = ARM_MODE_MON, .gdb_index = 48, },
	[40] = { .name = "lr_mon", .cookie = 14, .mode = ARM_MODE_MON, .gdb_index = 49, },
	[41] = { .name = "spsr_mon", .cookie = 16, .mode = ARM_MODE_MON, .gdb_index = 50, },

	/* These exist only when the Virtualization Extensions is present */
	[42] = { .name = "sp_hyp", .cookie = 13, .mode = ARM_MODE_HYP, .gdb_index = 51, },
	[43] = { .name = "spsr_hyp", .cookie = 16, .mode = ARM_MODE_HYP, .gdb_index = 52, },
};

static const struct {
	unsigned int id;
	const char *name;
	uint32_t bits;
	enum arm_mode mode;
	enum reg_type type;
	const char *group;
	const char *feature;
} arm_vfp_v3_regs[] = {
	{ ARM_VFP_V3_D0,  "d0",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D1,  "d1",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D2,  "d2",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D3,  "d3",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D4,  "d4",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D5,  "d5",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D6,  "d6",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D7,  "d7",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D8,  "d8",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D9,  "d9",  64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D10, "d10", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D11, "d11", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D12, "d12", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D13, "d13", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D14, "d14", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D15, "d15", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D16, "d16", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D17, "d17", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D18, "d18", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D19, "d19", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D20, "d20", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D21, "d21", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D22, "d22", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D23, "d23", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D24, "d24", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D25, "d25", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D26, "d26", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D27, "d27", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D28, "d28", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D29, "d29", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D30, "d30", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_D31, "d31", 64, ARM_MODE_ANY, REG_TYPE_IEEE_DOUBLE, NULL, "org.gnu.gdb.arm.vfp"},
	{ ARM_VFP_V3_FPSCR, "fpscr", 32, ARM_MODE_ANY, REG_TYPE_INT, "float", "org.gnu.gdb.arm.vfp"},
};

/* map core mode (USR, FIQ, ...) and register number to
 * indices into the register cache
 */
const int armv4_5_core_reg_map[9][17] = {
	{	/* USR */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31
	},
	{	/* FIQ (8 shadows of USR, vs normal 3) */
		0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 15, 32
	},
	{	/* IRQ */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 23, 24, 15, 33
	},
	{	/* SVC */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 25, 26, 15, 34
	},
	{	/* ABT */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 27, 28, 15, 35
	},
	{	/* UND */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 29, 30, 15, 36
	},
	{	/* SYS (same registers as USR) */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31
	},
	{	/* MON */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 39, 40, 15, 41,
	},
	{	/* HYP */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 42, 14, 15, 43,
	}
};

/**
 * Configures host-side ARM records to reflect the specified CPSR.
 * Later, code can use arm_reg_current() to map register numbers
 * according to how they are exposed by this mode.
 */
void arm_set_cpsr(struct arm *arm, uint32_t cpsr)
{
	enum arm_mode mode = cpsr & 0x1f;
	int num;

	/* NOTE:  this may be called very early, before the register
	 * cache is set up.  We can't defend against many errors, in
	 * particular against CPSRs that aren't valid *here* ...
	 */
	if (arm->cpsr) {
		buf_set_u32(arm->cpsr->value, 0, 32, cpsr);
		arm->cpsr->valid = true;
		arm->cpsr->dirty = false;
	}

	arm->core_mode = mode;

	/* mode_to_number() warned; set up a somewhat-sane mapping */
	num = arm_mode_to_number(mode);
	if (num < 0) {
		mode = ARM_MODE_USR;
		num = 0;
	}

	arm->map = &armv4_5_core_reg_map[num][0];
	arm->spsr = (mode == ARM_MODE_USR || mode == ARM_MODE_SYS)
		? NULL
		: arm->core_cache->reg_list + arm->map[16];

	/* Older ARMs won't have the J bit */
	enum arm_state state;

	if (cpsr & (1 << 5)) {	/* T */
		if (cpsr & (1 << 24)) {	/* J */
			LOG_WARNING("ThumbEE -- incomplete support");
			state = ARM_STATE_THUMB_EE;
		} else
			state = ARM_STATE_THUMB;
	} else {
		if (cpsr & (1 << 24)) {	/* J */
			LOG_ERROR("Jazelle state handling is BROKEN!");
			state = ARM_STATE_JAZELLE;
		} else
			state = ARM_STATE_ARM;
	}
	arm->core_state = state;

	LOG_DEBUG("set CPSR %#8.8x: %s mode, %s state", (unsigned) cpsr,
		arm_mode_name(mode),
		arm_state_strings[arm->core_state]);
}

/**
 * Returns handle to the register currently mapped to a given number.
 * Someone must have called arm_set_cpsr() before.
 *
 * \param arm This core's state and registers are used.
 * \param regnum From 0..15 corresponding to R0..R14 and PC.
 *	Note that R0..R7 don't require mapping; you may access those
 *	as the first eight entries in the register cache.  Likewise
 *	R15 (PC) doesn't need mapping; you may also access it directly.
 *	However, R8..R14, and SPSR (arm->spsr) *must* be mapped.
 *	CPSR (arm->cpsr) is also not mapped.
 */
struct reg *arm_reg_current(struct arm *arm, unsigned regnum)
{
	struct reg *r;

	if (regnum > 16)
		return NULL;

	if (!arm->map) {
		LOG_ERROR("Register map is not available yet, the target is not fully initialised");
		r = arm->core_cache->reg_list + regnum;
	} else
		r = arm->core_cache->reg_list + arm->map[regnum];

	/* e.g. invalid CPSR said "secure monitor" mode on a core
	 * that doesn't support it...
	 */
	if (!r) {
		LOG_ERROR("Invalid CPSR mode");
		r = arm->core_cache->reg_list + regnum;
	}

	return r;
}

static const uint8_t arm_gdb_dummy_fp_value[12];

static struct reg_feature arm_gdb_dummy_fp_features = {
	.name = "net.sourceforge.openocd.fake_fpa"
};

/**
 * Dummy FPA registers are required to support GDB on ARM.
 * Register packets require eight obsolete FPA register values.
 * Modern ARM cores use Vector Floating Point (VFP), if they
 * have any floating point support.  VFP is not FPA-compatible.
 */
struct reg arm_gdb_dummy_fp_reg = {
	.name = "GDB dummy FPA register",
	.value = (uint8_t *) arm_gdb_dummy_fp_value,
	.valid = true,
	.size = 96,
	.exist = false,
	.number = 16,
	.feature = &arm_gdb_dummy_fp_features,
	.group = "fake_fpa",
};

static const uint8_t arm_gdb_dummy_fps_value[4];

/**
 * Dummy FPA status registers are required to support GDB on ARM.
 * Register packets require an obsolete FPA status register.
 */
struct reg arm_gdb_dummy_fps_reg = {
	.name = "GDB dummy FPA status register",
	.value = (uint8_t *) arm_gdb_dummy_fps_value,
	.valid = true,
	.size = 32,
	.exist = false,
	.number = 24,
	.feature = &arm_gdb_dummy_fp_features,
	.group = "fake_fpa",
};

static void arm_gdb_dummy_init(void) __attribute__ ((constructor));

static void arm_gdb_dummy_init(void)
{
	register_init_dummy(&arm_gdb_dummy_fp_reg);
	register_init_dummy(&arm_gdb_dummy_fps_reg);
}

static int armv4_5_get_core_reg(struct reg *reg)
{
	int retval;
	struct arm_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = reg_arch_info->arm->read_core_reg(target, reg,
			reg_arch_info->num, reg_arch_info->mode);
	if (retval == ERROR_OK) {
		reg->valid = true;
		reg->dirty = false;
	}

	return retval;
}

static int armv4_5_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct arm_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct arm *armv4_5_target = target_to_arm(target);
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Except for CPSR, the "reg" command exposes a writeback model
	 * for the register cache.
	 */
	if (reg == armv4_5_target->cpsr) {
		arm_set_cpsr(armv4_5_target, value);

		/* Older cores need help to be in ARM mode during halt
		 * mode debug, so we clear the J and T bits if we flush.
		 * For newer cores (v6/v7a/v7r) we don't need that, but
		 * it won't hurt since CPSR is always flushed anyway.
		 */
		if (armv4_5_target->core_mode !=
			(enum arm_mode)(value & 0x1f)) {
			LOG_DEBUG("changing ARM core mode to '%s'",
				arm_mode_name(value & 0x1f));
			value &= ~((1 << 24) | (1 << 5));
			uint8_t t[4];
			buf_set_u32(t, 0, 32, value);
			armv4_5_target->write_core_reg(target, reg,
				16, ARM_MODE_ANY, t);
		}
	} else {
		buf_set_u32(reg->value, 0, 32, value);
		if (reg->size == 64) {
			value = buf_get_u32(buf + 4, 0, 32);
			buf_set_u32(reg->value + 4, 0, 32, value);
		}
		reg->valid = true;
	}
	reg->dirty = true;

	return ERROR_OK;
}

static const struct reg_arch_type arm_reg_type = {
	.get = armv4_5_get_core_reg,
	.set = armv4_5_set_core_reg,
};

struct reg_cache *arm_build_reg_cache(struct target *target, struct arm *arm)
{
	int num_regs = ARRAY_SIZE(arm_core_regs);
	int num_core_regs = num_regs;
	if (arm->arm_vfp_version == ARM_VFP_V3)
		num_regs += ARRAY_SIZE(arm_vfp_v3_regs);

	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct arm_reg *reg_arch_info = calloc(num_regs, sizeof(struct arm_reg));
	int i;

	if (!cache || !reg_list || !reg_arch_info) {
		free(cache);
		free(reg_list);
		free(reg_arch_info);
		return NULL;
	}

	cache->name = "ARM registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = 0;

	for (i = 0; i < num_core_regs; i++) {
		/* Skip registers this core doesn't expose */
		if (arm_core_regs[i].mode == ARM_MODE_MON
			&& arm->core_type != ARM_CORE_TYPE_SEC_EXT
			&& arm->core_type != ARM_CORE_TYPE_VIRT_EXT)
			continue;
		if (arm_core_regs[i].mode == ARM_MODE_HYP
			&& arm->core_type != ARM_CORE_TYPE_VIRT_EXT)
			continue;

		/* REVISIT handle Cortex-M, which only shadows R13/SP */

		reg_arch_info[i].num = arm_core_regs[i].cookie;
		reg_arch_info[i].mode = arm_core_regs[i].mode;
		reg_arch_info[i].target = target;
		reg_arch_info[i].arm = arm;

		reg_list[i].name = arm_core_regs[i].name;
		reg_list[i].number = arm_core_regs[i].gdb_index;
		reg_list[i].size = 32;
		reg_list[i].value = reg_arch_info[i].value;
		reg_list[i].type = &arm_reg_type;
		reg_list[i].arch_info = &reg_arch_info[i];
		reg_list[i].exist = true;

		/* This really depends on the calling convention in use */
		reg_list[i].caller_save = false;

		/* Registers data type, as used by GDB target description */
		reg_list[i].reg_data_type = malloc(sizeof(struct reg_data_type));
		switch (arm_core_regs[i].cookie) {
		case 13:
			reg_list[i].reg_data_type->type = REG_TYPE_DATA_PTR;
			break;
		case 14:
		case 15:
			reg_list[i].reg_data_type->type = REG_TYPE_CODE_PTR;
		    break;
		default:
			reg_list[i].reg_data_type->type = REG_TYPE_UINT32;
		    break;
		}

		/* let GDB shows banked registers only in "info all-reg" */
		reg_list[i].feature = malloc(sizeof(struct reg_feature));
		if (reg_list[i].number <= 15 || reg_list[i].number == 25) {
			reg_list[i].feature->name = "org.gnu.gdb.arm.core";
			reg_list[i].group = "general";
		} else {
			reg_list[i].feature->name = "net.sourceforge.openocd.banked";
			reg_list[i].group = "banked";
		}

		cache->num_regs++;
	}

	int j;
	for (i = num_core_regs, j = 0; i < num_regs; i++, j++) {
		reg_arch_info[i].num = arm_vfp_v3_regs[j].id;
		reg_arch_info[i].mode = arm_vfp_v3_regs[j].mode;
		reg_arch_info[i].target = target;
		reg_arch_info[i].arm = arm;

		reg_list[i].name = arm_vfp_v3_regs[j].name;
		reg_list[i].number = arm_vfp_v3_regs[j].id;
		reg_list[i].size = arm_vfp_v3_regs[j].bits;
		reg_list[i].value = reg_arch_info[i].value;
		reg_list[i].type = &arm_reg_type;
		reg_list[i].arch_info = &reg_arch_info[i];
		reg_list[i].exist = true;

		reg_list[i].caller_save = false;

		reg_list[i].reg_data_type = malloc(sizeof(struct reg_data_type));
		reg_list[i].reg_data_type->type = arm_vfp_v3_regs[j].type;

		reg_list[i].feature = malloc(sizeof(struct reg_feature));
		reg_list[i].feature->name = arm_vfp_v3_regs[j].feature;

		reg_list[i].group = arm_vfp_v3_regs[j].group;

		cache->num_regs++;
	}

	arm->pc = reg_list + 15;
	arm->cpsr = reg_list + ARMV4_5_CPSR;
	arm->core_cache = cache;

	return cache;
}

void arm_free_reg_cache(struct arm *arm)
{
	if (!arm || !arm->core_cache)
		return;

	struct reg_cache *cache = arm->core_cache;

	for (unsigned int i = 0; i < cache->num_regs; i++) {
		struct reg *reg = &cache->reg_list[i];

		free(reg->feature);
		free(reg->reg_data_type);
	}

	free(cache->reg_list[0].arch_info);
	free(cache->reg_list);
	free(cache);

	arm->core_cache = NULL;
}

int arm_arch_state(struct target *target)
{
	struct arm *arm = target_to_arm(target);

	if (arm->common_magic != ARM_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-ARM target");
		return ERROR_FAIL;
	}

	/* avoid filling log waiting for fileio reply */
	if (target->semihosting && target->semihosting->hit_fileio)
		return ERROR_OK;

	LOG_USER("target halted in %s state due to %s, current mode: %s\n"
		"cpsr: 0x%8.8" PRIx32 " pc: 0x%8.8" PRIx32 "%s%s",
		arm_state_strings[arm->core_state],
		debug_reason_name(target),
		arm_mode_name(arm->core_mode),
		buf_get_u32(arm->cpsr->value, 0, 32),
		buf_get_u32(arm->pc->value, 0, 32),
		(target->semihosting && target->semihosting->is_active) ? ", semihosting" : "",
		(target->semihosting && target->semihosting->is_fileio) ? " fileio" : "");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_armv4_5_reg_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct reg *regs;

	if (!is_arm(arm)) {
		command_print(CMD, "current target isn't an ARM");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "error: target must be halted for register accesses");
		return ERROR_FAIL;
	}

	if (arm->core_type != ARM_CORE_TYPE_STD) {
		command_print(CMD,
			"Microcontroller Profile not supported - use standard reg cmd");
		return ERROR_OK;
	}

	if (!is_arm_mode(arm->core_mode)) {
		LOG_ERROR("not a valid arm core mode - communication failure?");
		return ERROR_FAIL;
	}

	if (!arm->full_context) {
		command_print(CMD, "error: target doesn't support %s",
			CMD_NAME);
		return ERROR_FAIL;
	}

	regs = arm->core_cache->reg_list;

	for (unsigned mode = 0; mode < ARRAY_SIZE(arm_mode_data); mode++) {
		const char *name;
		char *sep = "\n";
		char *shadow = "";

		if (!arm_mode_data[mode].n_indices)
			continue;

		/* label this bank of registers (or shadows) */
		switch (arm_mode_data[mode].psr) {
			case ARM_MODE_SYS:
				continue;
			case ARM_MODE_USR:
				name = "System and User";
				sep = "";
				break;
			case ARM_MODE_HYP:
				if (arm->core_type != ARM_CORE_TYPE_VIRT_EXT)
					continue;
			/* FALLTHROUGH */
			case ARM_MODE_MON:
			case ARM_MODE_1176_MON:
				if (arm->core_type != ARM_CORE_TYPE_SEC_EXT
					&& arm->core_type != ARM_CORE_TYPE_VIRT_EXT)
					continue;
			/* FALLTHROUGH */
			default:
				name = arm_mode_data[mode].name;
				shadow = "shadow ";
				break;
		}
		command_print(CMD, "%s%s mode %sregisters",
			sep, name, shadow);

		/* display N rows of up to 4 registers each */
		for (unsigned i = 0; i < arm_mode_data[mode].n_indices; ) {
			char output[80];
			int output_len = 0;

			for (unsigned j = 0; j < 4; j++, i++) {
				uint32_t value;
				struct reg *reg = regs;

				if (i >= arm_mode_data[mode].n_indices)
					break;

				reg += arm_mode_data[mode].indices[i];

				/* REVISIT be smarter about faults... */
				if (!reg->valid)
					arm->full_context(target);

				value = buf_get_u32(reg->value, 0, 32);
				output_len += snprintf(output + output_len,
						sizeof(output) - output_len,
						"%8s: %8.8" PRIx32 " ",
						reg->name, value);
			}
			command_print(CMD, "%s", output);
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_armv4_5_core_state_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);

	if (!is_arm(arm)) {
		command_print(CMD, "current target isn't an ARM");
		return ERROR_FAIL;
	}

	if (arm->core_type == ARM_CORE_TYPE_M_PROFILE) {
		/* armv7m not supported */
		command_print(CMD, "Unsupported Command");
		return ERROR_OK;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "arm") == 0)
			arm->core_state = ARM_STATE_ARM;
		if (strcmp(CMD_ARGV[0], "thumb") == 0)
			arm->core_state = ARM_STATE_THUMB;
	}

	command_print(CMD, "core state: %s", arm_state_strings[arm->core_state]);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_arm_disassemble_command)
{
#if HAVE_CAPSTONE
	struct target *target = get_current_target(CMD_CTX);

	if (target == NULL) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct arm *arm = target_to_arm(target);
	target_addr_t address;
	unsigned int count = 1;
	bool thumb = false;

	if (!is_arm(arm)) {
		command_print(CMD, "current target isn't an ARM");
		return ERROR_FAIL;
	}

	if (arm->core_type == ARM_CORE_TYPE_M_PROFILE) {
		/* armv7m is always thumb mode */
		thumb = true;
	}

	switch (CMD_ARGC) {
		case 3:
			if (strcmp(CMD_ARGV[2], "thumb") != 0)
				return ERROR_COMMAND_SYNTAX_ERROR;
			thumb = true;
		/* FALL THROUGH */
		case 2:
			COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], count);
		/* FALL THROUGH */
		case 1:
			COMMAND_PARSE_ADDRESS(CMD_ARGV[0], address);
			if (address & 0x01) {
				if (!thumb) {
					command_print(CMD, "Disassemble as Thumb");
					thumb = true;
				}
				address &= ~1;
			}
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return arm_disassemble(CMD, target, address, count, thumb);
#else
	command_print(CMD, "capstone disassembly framework required");
	return ERROR_FAIL;
#endif
}

static int jim_mcrmrc(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct command_context *context;
	struct target *target;
	struct arm *arm;
	int retval;

	context = current_command_context(interp);
	assert(context != NULL);

	target = get_current_target(context);
	if (target == NULL) {
		LOG_ERROR("%s: no current target", __func__);
		return JIM_ERR;
	}
	if (!target_was_examined(target)) {
		LOG_ERROR("%s: not yet examined", target_name(target));
		return JIM_ERR;
	}
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		LOG_ERROR("%s: not an ARM", target_name(target));
		return JIM_ERR;
	}

	if ((argc < 6) || (argc > 7)) {
		/* FIXME use the command name to verify # params... */
		LOG_ERROR("%s: wrong number of arguments", __func__);
		return JIM_ERR;
	}

	int cpnum;
	uint32_t op1;
	uint32_t op2;
	uint32_t CRn;
	uint32_t CRm;
	uint32_t value;
	long l;

	/* NOTE:  parameter sequence matches ARM instruction set usage:
	 *	MCR	pNUM, op1, rX, CRn, CRm, op2	; write CP from rX
	 *	MRC	pNUM, op1, rX, CRn, CRm, op2	; read CP into rX
	 * The "rX" is necessarily omitted; it uses Tcl mechanisms.
	 */
	retval = Jim_GetLong(interp, argv[1], &l);
	if (retval != JIM_OK)
		return retval;
	if (l & ~0xf) {
		LOG_ERROR("%s: %s %d out of range", __func__,
			"coprocessor", (int) l);
		return JIM_ERR;
	}
	cpnum = l;

	retval = Jim_GetLong(interp, argv[2], &l);
	if (retval != JIM_OK)
		return retval;
	if (l & ~0x7) {
		LOG_ERROR("%s: %s %d out of range", __func__,
			"op1", (int) l);
		return JIM_ERR;
	}
	op1 = l;

	retval = Jim_GetLong(interp, argv[3], &l);
	if (retval != JIM_OK)
		return retval;
	if (l & ~0xf) {
		LOG_ERROR("%s: %s %d out of range", __func__,
			"CRn", (int) l);
		return JIM_ERR;
	}
	CRn = l;

	retval = Jim_GetLong(interp, argv[4], &l);
	if (retval != JIM_OK)
		return retval;
	if (l & ~0xf) {
		LOG_ERROR("%s: %s %d out of range", __func__,
			"CRm", (int) l);
		return JIM_ERR;
	}
	CRm = l;

	retval = Jim_GetLong(interp, argv[5], &l);
	if (retval != JIM_OK)
		return retval;
	if (l & ~0x7) {
		LOG_ERROR("%s: %s %d out of range", __func__,
			"op2", (int) l);
		return JIM_ERR;
	}
	op2 = l;

	value = 0;

	/* FIXME don't assume "mrc" vs "mcr" from the number of params;
	 * that could easily be a typo!  Check both...
	 *
	 * FIXME change the call syntax here ... simplest to just pass
	 * the MRC() or MCR() instruction to be executed.  That will also
	 * let us support the "mrc2" and "mcr2" opcodes (toggling one bit)
	 * if that's ever needed.
	 */
	if (argc == 7) {
		retval = Jim_GetLong(interp, argv[6], &l);
		if (retval != JIM_OK)
			return retval;
		value = l;

		/* NOTE: parameters reordered! */
		/* ARMV4_5_MCR(cpnum, op1, 0, CRn, CRm, op2) */
		retval = arm->mcr(target, cpnum, op1, op2, CRn, CRm, value);
		if (retval != ERROR_OK)
			return JIM_ERR;
	} else {
		/* NOTE: parameters reordered! */
		/* ARMV4_5_MRC(cpnum, op1, 0, CRn, CRm, op2) */
		retval = arm->mrc(target, cpnum, op1, op2, CRn, CRm, &value);
		if (retval != ERROR_OK)
			return JIM_ERR;

		Jim_SetResult(interp, Jim_NewIntObj(interp, value));
	}

	return JIM_OK;
}

extern const struct command_registration semihosting_common_handlers[];

static const struct command_registration arm_exec_command_handlers[] = {
	{
		.name = "reg",
		.handler = handle_armv4_5_reg_command,
		.mode = COMMAND_EXEC,
		.help = "display ARM core registers",
		.usage = "",
	},
	{
		.name = "core_state",
		.handler = handle_armv4_5_core_state_command,
		.mode = COMMAND_EXEC,
		.usage = "['arm'|'thumb']",
		.help = "display/change ARM core state",
	},
	{
		.name = "disassemble",
		.handler = handle_arm_disassemble_command,
		.mode = COMMAND_EXEC,
		.usage = "address [count ['thumb']]",
		.help = "disassemble instructions ",
	},
	{
		.name = "mcr",
		.mode = COMMAND_EXEC,
		.jim_handler = &jim_mcrmrc,
		.help = "write coprocessor register",
		.usage = "cpnum op1 CRn CRm op2 value",
	},
	{
		.name = "mrc",
		.mode = COMMAND_EXEC,
		.jim_handler = &jim_mcrmrc,
		.help = "read coprocessor register",
		.usage = "cpnum op1 CRn CRm op2",
	},
	{
		.chain = semihosting_common_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
const struct command_registration arm_command_handlers[] = {
	{
		.name = "arm",
		.mode = COMMAND_ANY,
		.help = "ARM command group",
		.usage = "",
		.chain = arm_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/*
 * gdb for arm targets (e.g. arm-none-eabi-gdb) supports several variants
 * of arm architecture. You can list them using the autocompletion of gdb
 * command prompt by typing "set architecture " and then press TAB key.
 * The default, selected automatically, is "arm".
 * Let's use the default value, here, to make gdb-multiarch behave in the
 * same way as a gdb for arm. This can be changed later on. User can still
 * set the specific architecture variant with the gdb command.
 */
const char *arm_get_gdb_arch(struct target *target)
{
	return "arm";
}

int arm_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	struct arm *arm = target_to_arm(target);
	unsigned int i;

	if (!is_arm_mode(arm->core_mode)) {
		LOG_ERROR("not a valid arm core mode - communication failure?");
		return ERROR_FAIL;
	}

	switch (reg_class) {
	case REG_CLASS_GENERAL:
		*reg_list_size = 26;
		*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

		for (i = 0; i < 16; i++)
				(*reg_list)[i] = arm_reg_current(arm, i);

		/* For GDB compatibility, take FPA registers size into account and zero-fill it*/
		for (i = 16; i < 24; i++)
				(*reg_list)[i] = &arm_gdb_dummy_fp_reg;
		(*reg_list)[24] = &arm_gdb_dummy_fps_reg;

		(*reg_list)[25] = arm->cpsr;

		return ERROR_OK;

	case REG_CLASS_ALL:
		switch (arm->core_type) {
			case ARM_CORE_TYPE_SEC_EXT:
				*reg_list_size = 51;
				break;
			case ARM_CORE_TYPE_VIRT_EXT:
				*reg_list_size = 53;
				break;
			default:
				*reg_list_size = 48;
		}
		unsigned int list_size_core = *reg_list_size;
		if (arm->arm_vfp_version == ARM_VFP_V3)
			*reg_list_size += 33;

		*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

		for (i = 0; i < 16; i++)
				(*reg_list)[i] = arm_reg_current(arm, i);

		for (i = 13; i < ARRAY_SIZE(arm_core_regs); i++) {
				int reg_index = arm->core_cache->reg_list[i].number;

				if (arm_core_regs[i].mode == ARM_MODE_MON
					&& arm->core_type != ARM_CORE_TYPE_SEC_EXT
					&& arm->core_type != ARM_CORE_TYPE_VIRT_EXT)
					continue;
				if (arm_core_regs[i].mode == ARM_MODE_HYP
					&& arm->core_type != ARM_CORE_TYPE_VIRT_EXT)
					continue;
				(*reg_list)[reg_index] = &(arm->core_cache->reg_list[i]);
		}

		/* When we supply the target description, there is no need for fake FPA */
		for (i = 16; i < 24; i++) {
				(*reg_list)[i] = &arm_gdb_dummy_fp_reg;
				(*reg_list)[i]->size = 0;
		}
		(*reg_list)[24] = &arm_gdb_dummy_fps_reg;
		(*reg_list)[24]->size = 0;

		if (arm->arm_vfp_version == ARM_VFP_V3) {
			unsigned int num_core_regs = ARRAY_SIZE(arm_core_regs);
			for (i = 0; i < 33; i++)
				(*reg_list)[list_size_core + i] = &(arm->core_cache->reg_list[num_core_regs + i]);
		}

		return ERROR_OK;

	default:
		LOG_ERROR("not a valid register class type in query.");
		return ERROR_FAIL;
	}
}

/* wait for execution to complete and check exit point */
static int armv4_5_run_algorithm_completion(struct target *target,
	uint32_t exit_point,
	int timeout_ms,
	void *arch_info)
{
	int retval;
	struct arm *arm = target_to_arm(target);

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	if (retval != ERROR_OK)
		return retval;
	if (target->state != TARGET_HALTED) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;
		return ERROR_TARGET_TIMEOUT;
	}

	/* fast exit: ARMv5+ code can use BKPT */
	if (exit_point && buf_get_u32(arm->pc->value, 0, 32) != exit_point) {
		LOG_WARNING(
			"target reentered debug state, but not at the desired exit point: 0x%4.4" PRIx32 "",
			buf_get_u32(arm->pc->value, 0, 32));
		return ERROR_TARGET_TIMEOUT;
	}

	return ERROR_OK;
}

int armv4_5_run_algorithm_inner(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	int timeout_ms, void *arch_info,
	int (*run_it)(struct target *target, uint32_t exit_point,
	int timeout_ms, void *arch_info))
{
	struct arm *arm = target_to_arm(target);
	struct arm_algorithm *arm_algorithm_info = arch_info;
	enum arm_state core_state = arm->core_state;
	uint32_t context[17];
	uint32_t cpsr;
	int exit_breakpoint_size = 0;
	int i;
	int retval = ERROR_OK;

	LOG_DEBUG("Running algorithm");

	if (arm_algorithm_info->common_magic != ARM_COMMON_MAGIC) {
		LOG_ERROR("current target isn't an ARMV4/5 target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!is_arm_mode(arm->core_mode)) {
		LOG_ERROR("not a valid arm core mode - communication failure?");
		return ERROR_FAIL;
	}

	/* armv5 and later can terminate with BKPT instruction; less overhead */
	if (!exit_point && arm->is_armv4) {
		LOG_ERROR("ARMv4 target needs HW breakpoint location");
		return ERROR_FAIL;
	}

	/* save r0..pc, cpsr-or-spsr, and then cpsr-for-sure;
	 * they'll be restored later.
	 */
	for (i = 0; i <= 16; i++) {
		struct reg *r;

		r = &ARMV4_5_CORE_REG_MODE(arm->core_cache,
				arm_algorithm_info->core_mode, i);
		if (!r->valid)
			arm->read_core_reg(target, r, i,
				arm_algorithm_info->core_mode);
		context[i] = buf_get_u32(r->value, 0, 32);
	}
	cpsr = buf_get_u32(arm->cpsr->value, 0, 32);

	for (i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;
		retval = target_write_buffer(target, mem_params[i].address, mem_params[i].size,
				mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN)
			continue;

		struct reg *reg = register_get_by_name(arm->core_cache, reg_params[i].reg_name, 0);
		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
				reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		retval = armv4_5_set_core_reg(reg, reg_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	arm->core_state = arm_algorithm_info->core_state;
	if (arm->core_state == ARM_STATE_ARM)
		exit_breakpoint_size = 4;
	else if (arm->core_state == ARM_STATE_THUMB)
		exit_breakpoint_size = 2;
	else {
		LOG_ERROR("BUG: can't execute algorithms when not in ARM or Thumb state");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (arm_algorithm_info->core_mode != ARM_MODE_ANY) {
		LOG_DEBUG("setting core_mode: 0x%2.2x",
			arm_algorithm_info->core_mode);
		buf_set_u32(arm->cpsr->value, 0, 5,
			arm_algorithm_info->core_mode);
		arm->cpsr->dirty = true;
		arm->cpsr->valid = true;
	}

	/* terminate using a hardware or (ARMv5+) software breakpoint */
	if (exit_point) {
		retval = breakpoint_add(target, exit_point,
				exit_breakpoint_size, BKPT_HARD);
		if (retval != ERROR_OK) {
			LOG_ERROR("can't add HW breakpoint to terminate algorithm");
			return ERROR_TARGET_FAILURE;
		}
	}

	retval = target_resume(target, 0, entry_point, 1, 1);
	if (retval != ERROR_OK)
		return retval;
	retval = run_it(target, exit_point, timeout_ms, arch_info);

	if (exit_point)
		breakpoint_remove(target, exit_point);

	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			int retvaltemp = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retvaltemp != ERROR_OK)
				retval = retvaltemp;
		}
	}

	for (i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {

			struct reg *reg = register_get_by_name(arm->core_cache,
					reg_params[i].reg_name,
					0);
			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				retval = ERROR_COMMAND_SYNTAX_ERROR;
				continue;
			}

			if (reg->size != reg_params[i].size) {
				LOG_ERROR(
					"BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
				retval = ERROR_COMMAND_SYNTAX_ERROR;
				continue;
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	/* restore everything we saved before (17 or 18 registers) */
	for (i = 0; i <= 16; i++) {
		uint32_t regvalue;
		regvalue = buf_get_u32(ARMV4_5_CORE_REG_MODE(arm->core_cache,
				arm_algorithm_info->core_mode, i).value, 0, 32);
		if (regvalue != context[i]) {
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32 "",
				ARMV4_5_CORE_REG_MODE(arm->core_cache,
				arm_algorithm_info->core_mode, i).name, context[i]);
			buf_set_u32(ARMV4_5_CORE_REG_MODE(arm->core_cache,
				arm_algorithm_info->core_mode, i).value, 0, 32, context[i]);
			ARMV4_5_CORE_REG_MODE(arm->core_cache, arm_algorithm_info->core_mode,
				i).valid = true;
			ARMV4_5_CORE_REG_MODE(arm->core_cache, arm_algorithm_info->core_mode,
				i).dirty = true;
		}
	}

	arm_set_cpsr(arm, cpsr);
	arm->cpsr->dirty = true;

	arm->core_state = core_state;

	return retval;
}

int armv4_5_run_algorithm(struct target *target,
	int num_mem_params,
	struct mem_param *mem_params,
	int num_reg_params,
	struct reg_param *reg_params,
	target_addr_t entry_point,
	target_addr_t exit_point,
	int timeout_ms,
	void *arch_info)
{
	return armv4_5_run_algorithm_inner(target,
			num_mem_params,
			mem_params,
			num_reg_params,
			reg_params,
			(uint32_t)entry_point,
			(uint32_t)exit_point,
			timeout_ms,
			arch_info,
			armv4_5_run_algorithm_completion);
}

/**
 * Runs ARM code in the target to calculate a CRC32 checksum.
 *
 */
int arm_checksum_memory(struct target *target,
	target_addr_t address, uint32_t count, uint32_t *checksum)
{
	struct working_area *crc_algorithm;
	struct arm_algorithm arm_algo;
	struct arm *arm = target_to_arm(target);
	struct reg_param reg_params[2];
	int retval;
	uint32_t i;
	uint32_t exit_var = 0;

	static const uint8_t arm_crc_code_le[] = {
#include "../../contrib/loaders/checksum/armv4_5_crc.inc"
	};

	assert(sizeof(arm_crc_code_le) % 4 == 0);

	retval = target_alloc_working_area(target,
			sizeof(arm_crc_code_le), &crc_algorithm);
	if (retval != ERROR_OK)
		return retval;

	/* convert code into a buffer in target endianness */
	for (i = 0; i < ARRAY_SIZE(arm_crc_code_le) / 4; i++) {
		retval = target_write_u32(target,
				crc_algorithm->address + i * sizeof(uint32_t),
				le_to_h_u32(&arm_crc_code_le[i * 4]));
		if (retval != ERROR_OK)
			goto cleanup;
	}

	arm_algo.common_magic = ARM_COMMON_MAGIC;
	arm_algo.core_mode = ARM_MODE_SVC;
	arm_algo.core_state = ARM_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, address);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	/* 20 second timeout/megabyte */
	int timeout = 20000 * (1 + (count / (1024 * 1024)));

	/* armv4 must exit using a hardware breakpoint */
	if (arm->is_armv4)
		exit_var = crc_algorithm->address + sizeof(arm_crc_code_le) - 8;

	retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
			crc_algorithm->address,
			exit_var,
			timeout, &arm_algo);

	if (retval == ERROR_OK)
		*checksum = buf_get_u32(reg_params[0].value, 0, 32);
	else
		LOG_ERROR("error executing ARM crc algorithm");

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

cleanup:
	target_free_working_area(target, crc_algorithm);

	return retval;
}

/**
 * Runs ARM code in the target to check whether a memory block holds
 * all ones.  NOR flash which has been erased, and thus may be written,
 * holds all ones.
 *
 */
int arm_blank_check_memory(struct target *target,
	struct target_memory_check_block *blocks, int num_blocks, uint8_t erased_value)
{
	struct working_area *check_algorithm;
	struct reg_param reg_params[3];
	struct arm_algorithm arm_algo;
	struct arm *arm = target_to_arm(target);
	int retval;
	uint32_t i;
	uint32_t exit_var = 0;

	static const uint8_t check_code_le[] = {
#include "../../contrib/loaders/erase_check/armv4_5_erase_check.inc"
	};

	assert(sizeof(check_code_le) % 4 == 0);

	if (erased_value != 0xff) {
		LOG_ERROR("Erase value 0x%02" PRIx8 " not yet supported for ARMv4/v5 targets",
			erased_value);
		return ERROR_FAIL;
	}

	/* make sure we have a working area */
	retval = target_alloc_working_area(target,
			sizeof(check_code_le), &check_algorithm);
	if (retval != ERROR_OK)
		return retval;

	/* convert code into a buffer in target endianness */
	for (i = 0; i < ARRAY_SIZE(check_code_le) / 4; i++) {
		retval = target_write_u32(target,
				check_algorithm->address
				+ i * sizeof(uint32_t),
				le_to_h_u32(&check_code_le[i * 4]));
		if (retval != ERROR_OK)
			goto cleanup;
	}

	arm_algo.common_magic = ARM_COMMON_MAGIC;
	arm_algo.core_mode = ARM_MODE_SVC;
	arm_algo.core_state = ARM_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, blocks[0].address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, blocks[0].size);

	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, erased_value);

	/* armv4 must exit using a hardware breakpoint */
	if (arm->is_armv4)
		exit_var = check_algorithm->address + sizeof(check_code_le) - 4;

	retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
			check_algorithm->address,
			exit_var,
			10000, &arm_algo);

	if (retval == ERROR_OK)
		blocks[0].result = buf_get_u32(reg_params[2].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

cleanup:
	target_free_working_area(target, check_algorithm);

	if (retval != ERROR_OK)
		return retval;

	return 1;       /* only one block has been checked */
}

static int arm_full_context(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	unsigned num_regs = arm->core_cache->num_regs;
	struct reg *reg = arm->core_cache->reg_list;
	int retval = ERROR_OK;

	for (; num_regs && retval == ERROR_OK; num_regs--, reg++) {
		if (reg->valid)
			continue;
		retval = armv4_5_get_core_reg(reg);
	}
	return retval;
}

static int arm_default_mrc(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2,
	uint32_t CRn, uint32_t CRm,
	uint32_t *value)
{
	LOG_ERROR("%s doesn't implement MRC", target_type_name(target));
	return ERROR_FAIL;
}

static int arm_default_mcr(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2,
	uint32_t CRn, uint32_t CRm,
	uint32_t value)
{
	LOG_ERROR("%s doesn't implement MCR", target_type_name(target));
	return ERROR_FAIL;
}

int arm_init_arch_info(struct target *target, struct arm *arm)
{
	target->arch_info = arm;
	arm->target = target;

	arm->common_magic = ARM_COMMON_MAGIC;

	/* core_type may be overridden by subtype logic */
	if (arm->core_type != ARM_CORE_TYPE_M_PROFILE) {
		arm->core_type = ARM_CORE_TYPE_STD;
		arm_set_cpsr(arm, ARM_MODE_USR);
	}

	/* default full_context() has no core-specific optimizations */
	if (!arm->full_context && arm->read_core_reg)
		arm->full_context = arm_full_context;

	if (!arm->mrc)
		arm->mrc = arm_default_mrc;
	if (!arm->mcr)
		arm->mcr = arm_default_mcr;

	return ERROR_OK;
}
