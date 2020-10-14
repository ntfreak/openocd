/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#ifndef OPENOCD_TARGET_ARMV7M_H
#define OPENOCD_TARGET_ARMV7M_H

#include "arm_adi_v5.h"
#include "arm.h"
#include "armv7m_trace.h"

extern const int armv7m_psp_reg_map[];
extern const int armv7m_msp_reg_map[];

const char *armv7m_exception_string(int number);

/* Cortex-M DCRSR.REGSEL selectors */
enum {
	ARMV7M_REGSEL_R0,
	ARMV7M_REGSEL_R1,
	ARMV7M_REGSEL_R2,
	ARMV7M_REGSEL_R3,

	ARMV7M_REGSEL_R4,
	ARMV7M_REGSEL_R5,
	ARMV7M_REGSEL_R6,
	ARMV7M_REGSEL_R7,

	ARMV7M_REGSEL_R8,
	ARMV7M_REGSEL_R9,
	ARMV7M_REGSEL_R10,
	ARMV7M_REGSEL_R11,

	ARMV7M_REGSEL_R12,
	ARMV7M_REGSEL_R13,
	ARMV7M_REGSEL_R14,
	ARMV7M_REGSEL_PC = 15,

	ARMV7M_REGSEL_xPSR = 16,
	ARMV7M_REGSEL_MSP,
	ARMV7M_REGSEL_PSP,

	ARMV7M_REGSEL_PMSK_BPRI_FLTMSK_CTRL = 0x14,
	ARMV7M_REGSEL_FPSCR = 0x21,

	/* 32bit Floating-point registers */
	ARMV7M_REGSEL_S0 = 0x40,
	ARMV7M_REGSEL_S1,
	ARMV7M_REGSEL_S2,
	ARMV7M_REGSEL_S3,
	ARMV7M_REGSEL_S4,
	ARMV7M_REGSEL_S5,
	ARMV7M_REGSEL_S6,
	ARMV7M_REGSEL_S7,
	ARMV7M_REGSEL_S8,
	ARMV7M_REGSEL_S9,
	ARMV7M_REGSEL_S10,
	ARMV7M_REGSEL_S11,
	ARMV7M_REGSEL_S12,
	ARMV7M_REGSEL_S13,
	ARMV7M_REGSEL_S14,
	ARMV7M_REGSEL_S15,
	ARMV7M_REGSEL_S16,
	ARMV7M_REGSEL_S17,
	ARMV7M_REGSEL_S18,
	ARMV7M_REGSEL_S19,
	ARMV7M_REGSEL_S20,
	ARMV7M_REGSEL_S21,
	ARMV7M_REGSEL_S22,
	ARMV7M_REGSEL_S23,
	ARMV7M_REGSEL_S24,
	ARMV7M_REGSEL_S25,
	ARMV7M_REGSEL_S26,
	ARMV7M_REGSEL_S27,
	ARMV7M_REGSEL_S28,
	ARMV7M_REGSEL_S29,
	ARMV7M_REGSEL_S30,
	ARMV7M_REGSEL_S31,
};

/* offsets into armv7m core register cache */
enum {
	/* for convenience, the first set of indices match
	 * the Cortex-M DCRSR.REGSEL selectors
	 */
	ARMV7M_R0 = ARMV7M_REGSEL_R0,
	ARMV7M_R1 = ARMV7M_REGSEL_R1,
	ARMV7M_R2 = ARMV7M_REGSEL_R2,
	ARMV7M_R3 = ARMV7M_REGSEL_R3,

	ARMV7M_R4 = ARMV7M_REGSEL_R4,
	ARMV7M_R5 = ARMV7M_REGSEL_R5,
	ARMV7M_R6 = ARMV7M_REGSEL_R6,
	ARMV7M_R7 = ARMV7M_REGSEL_R7,

	ARMV7M_R8 = ARMV7M_REGSEL_R8,
	ARMV7M_R9 = ARMV7M_REGSEL_R9,
	ARMV7M_R10 = ARMV7M_REGSEL_R10,
	ARMV7M_R11 = ARMV7M_REGSEL_R11,

	ARMV7M_R12 = ARMV7M_REGSEL_R12,
	ARMV7M_R13 = ARMV7M_REGSEL_R13,
	ARMV7M_R14 = ARMV7M_REGSEL_R14,
	ARMV7M_PC = ARMV7M_REGSEL_PC,

	ARMV7M_xPSR = ARMV7M_REGSEL_xPSR,
	ARMV7M_MSP = ARMV7M_REGSEL_MSP,
	ARMV7M_PSP = ARMV7M_REGSEL_PSP,

	/* following indices are arbitrary, do not match DCRSR.REGSEL selectors */

	/* working register for packing/unpacking special regs, hidden from gdb */
	ARMV7M_PMSK_BPRI_FLTMSK_CTRL,

	/* WARNING: If you use armv7m_write_core_reg() on one of 4 following
	 * special registers, the new data go to ARMV7M_PMSK_BPRI_FLTMSK_CTRL
	 * cache only and are not flushed to CPU HW register.
	 * To trigger write to CPU HW register, add
	 *		armv7m_write_core_reg(,,ARMV7M_PMSK_BPRI_FLTMSK_CTRL,);
	 */
	ARMV7M_PRIMASK,
	ARMV7M_BASEPRI,
	ARMV7M_FAULTMASK,
	ARMV7M_CONTROL,

	/* 64bit Floating-point registers */
	ARMV7M_D0,
	ARMV7M_D1,
	ARMV7M_D2,
	ARMV7M_D3,
	ARMV7M_D4,
	ARMV7M_D5,
	ARMV7M_D6,
	ARMV7M_D7,
	ARMV7M_D8,
	ARMV7M_D9,
	ARMV7M_D10,
	ARMV7M_D11,
	ARMV7M_D12,
	ARMV7M_D13,
	ARMV7M_D14,
	ARMV7M_D15,

	/* Floating-point status register */
	ARMV7M_FPSCR,

	ARMV7M_LAST_REG,
};

enum {
	FP_NONE = 0,
	FPv4_SP,
	FPv5_SP,
	FPv5_DP,
};

#define ARMV7M_NUM_CORE_REGS (ARMV7M_xPSR + 1)
#define ARMV7M_NUM_CORE_REGS_NOFP (ARMV7M_CONTROL + 1)

#define ARMV7M_COMMON_MAGIC 0x2A452A45

struct armv7m_common {
	struct arm	arm;

	int common_magic;
	int exception_number;

	/* AP this processor is connected to in the DAP */
	struct adiv5_ap *debug_ap;

	int fp_feature;
	uint32_t demcr;

	/* stlink is a high level adapter, does not support all functions */
	bool stlink;

	struct armv7m_trace_config trace_config;

	/* Direct processor core register read and writes */
	int (*load_core_reg_u32)(struct target *target, uint32_t regsel, uint32_t *value);
	int (*store_core_reg_u32)(struct target *target, uint32_t regsel, uint32_t value);

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv7m_common *
target_to_armv7m(struct target *target)
{
	return container_of(target->arch_info, struct armv7m_common, arm);
}

static inline bool is_armv7m(const struct armv7m_common *armv7m)
{
	return armv7m->common_magic == ARMV7M_COMMON_MAGIC;
}

struct armv7m_algorithm {
	int common_magic;

	enum arm_mode core_mode;

	uint32_t context[ARMV7M_LAST_REG]; /* ARMV7M_NUM_REGS */
};

struct reg_cache *armv7m_build_reg_cache(struct target *target);
void armv7m_free_reg_cache(struct target *target);

enum armv7m_mode armv7m_number_to_mode(int number);
int armv7m_mode_to_number(enum armv7m_mode mode);

int armv7m_arch_state(struct target *target);
int armv7m_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);

int armv7m_init_arch_info(struct target *target, struct armv7m_common *armv7m);

int armv7m_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t entry_point, target_addr_t exit_point,
		int timeout_ms, void *arch_info);

int armv7m_start_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t entry_point, target_addr_t exit_point,
		void *arch_info);

int armv7m_wait_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t exit_point, int timeout_ms,
		void *arch_info);

int armv7m_invalidate_core_regs(struct target *target);

int armv7m_restore_context(struct target *target);

int armv7m_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count, uint32_t *checksum);
int armv7m_blank_check_memory(struct target *target,
		struct target_memory_check_block *blocks, int num_blocks, uint8_t erased_value);

int armv7m_maybe_skip_bkpt_inst(struct target *target, bool *inst_found);

extern const struct command_registration armv7m_command_handlers[];

#endif /* OPENOCD_TARGET_ARMV7M_H */
