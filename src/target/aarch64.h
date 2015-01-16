/***************************************************************************
 *   Copyright (C) 2015 by David Ung                                       *
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

#ifndef AARCH64_H
#define AARCH64_H

#include "armv8.h"

#define AARCH64_COMMON_MAGIC 0x411fc082

#define CPUDBG_CPUID	0xD00
#define CPUDBG_CTYPR	0xD04
#define CPUDBG_TTYPR	0xD0C
#define ID_AA64DFR0_EL1	0xD28
#define CPUDBG_LOCKACCESS 0xFB0
#define CPUDBG_LOCKSTATUS 0xFB4

#define BRP_NORMAL 0
#define BRP_CONTEXT 1

#define AARCH64_PADDRDBG_CPU_SHIFT 13

struct aarch64_brp {
	int used;
	int type;
	uint32_t value;
	uint32_t control;
	uint8_t BRPn;
};

struct aarch64_common {
	int common_magic;
	struct arm_jtag jtag_info;

	/* Context information */
	uint32_t cpudbg_dscr;

	uint32_t system_control_reg;
	uint32_t system_control_reg_curr;

	enum arm_mode curr_mode;


	/* Breakpoint register pairs */
	int brp_num_context;
	int brp_num;
	int brp_num_available;
	struct aarch64_brp *brp_list;

	/* Use aarch64_read_regs_through_mem for fast register reads */
	int fast_reg_read;

	struct armv8_common armv8_common;

};

static inline struct aarch64_common *
target_to_aarch64(struct target *target)
{
	return container_of(target->arch_info, struct aarch64_common, armv8_common.arm);
}

#endif /* AARCH64_H */
