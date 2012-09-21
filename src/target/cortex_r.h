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
 *   Copyright (C) 2009 by Dirk Behme                                      *
 *   dirk.behme@gmail.com - copy from cortex_m3                            *
 *                                                                         *
 *   Copyright (C) 2011 by Google Inc.                                     *
 *   aschultz@google.com - Copy from cortex_a8                             *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef CORTEX_R4_H
#define CORTEX_R4_H

#include "armv7r.h"

#define CORTEX_R4_COMMON_MAGIC 0x411fc082

#define CPUDBG_CPUID	0xD00
#define CPUDBG_CTYPR	0xD04
#define CPUDBG_TTYPR	0xD0C
#define CPUDBG_LOCKACCESS 0xFB0
#define CPUDBG_LOCKSTATUS 0xFB4

#define BRP_NORMAL 0
#define BRP_CONTEXT 1

#define CORTEX_R4_PADDRDBG_CPU_SHIFT 13

struct cortex_r4_brp {
	int used;
	int type;
	uint32_t value;
	uint32_t control;
	uint8_t BRPn;
};

struct cortex_r4_common {
	int common_magic;
	struct arm_jtag jtag_info;

	/* Context information */
	uint32_t cpudbg_dscr;

	/* Saved cp15 registers */
	uint32_t cp15_control_reg;
	/* latest cp15 register value written and cpsr processor mode */
	uint32_t cp15_control_reg_curr;
	enum arm_mode curr_mode;


	/* Breakpoint register pairs */
	int brp_num_context;
	int brp_num;
	int brp_num_available;
	struct cortex_r4_brp *brp_list;

	/* Use cortex_r4_read_regs_through_mem for fast register reads */
	int fast_reg_read;

	struct armv7r_common armv7r_common;

};

static inline struct cortex_r4_common *
target_to_cortex_r4(struct target *target)
{
	return container_of(target->arch_info, struct cortex_r4_common, armv7r_common.arm);
}

#endif /* CORTEX_R4_H */
