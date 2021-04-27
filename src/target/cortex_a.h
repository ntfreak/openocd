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

#ifndef OPENOCD_TARGET_CORTEX_A_H
#define OPENOCD_TARGET_CORTEX_A_H

#include "armv7a.h"

#define CORTEX_A_COMMON_MAGIC 0x411fc082
#define CORTEX_A15_COMMON_MAGIC 0x413fc0f1

#define CORTEX_A5_PARTNUM 0xc05
#define CORTEX_A7_PARTNUM 0xc07
#define CORTEX_A8_PARTNUM 0xc08
#define CORTEX_A9_PARTNUM 0xc09
#define CORTEX_A15_PARTNUM 0xc0f
#define CORTEX_A_MIDR_PARTNUM_MASK 0x0000fff0
#define CORTEX_A_MIDR_PARTNUM_SHIFT 4

#define CPUDBG_CPUID	0xD00
#define CPUDBG_CTYPR	0xD04
#define CPUDBG_TTYPR	0xD0C
#define CPUDBG_LOCKACCESS 0xFB0
#define CPUDBG_LOCKSTATUS 0xFB4
#define CPUDBG_OSLAR_LK_MASK (1 << 1)

#define BRP_NORMAL 0
#define BRP_CONTEXT 1

#define CORTEX_A_PADDRDBG_CPU_SHIFT 13

enum cortex_a_isrmasking_mode {
	CORTEX_A_ISRMASK_OFF,
	CORTEX_A_ISRMASK_ON,
};

enum cortex_a_dacrfixup_mode {
	CORTEX_A_DACRFIXUP_OFF,
	CORTEX_A_DACRFIXUP_ON
};

struct cortex_a_brp {
	bool used;
	int type;
	uint32_t value;
	uint32_t control;
	uint8_t brpn;
};

struct cortex_a_wrp {
	bool used;
	uint32_t value;
	uint32_t control;
	uint8_t wrpn;
};

struct cortex_a_common {
	int common_magic;

	/* Context information */
	uint32_t cpudbg_dscr;

	/* Saved cp15 registers */
	uint32_t cp15_control_reg;
	/* latest cp15 register value written and cpsr processor mode */
	uint32_t cp15_control_reg_curr;
	/* auxiliary control reg */
	uint32_t cp15_aux_control_reg;
	/* DACR */
	uint32_t cp15_dacr_reg;
	enum arm_mode curr_mode;

	/* Breakpoint register pairs */
	int brp_num_context;
	int brp_num;
	int brp_num_available;
	struct cortex_a_brp *brp_list;
	int wrp_num;
	int wrp_num_available;
	struct cortex_a_wrp *wrp_list;

	uint32_t cpuid;
	uint32_t didr;

	enum cortex_a_isrmasking_mode isrmasking_mode;
	enum cortex_a_dacrfixup_mode dacrfixup_mode;

	struct armv7a_common armv7a_common;

};

static inline struct cortex_a_common *
target_to_cortex_a(struct target *target)
{
	return container_of(target->arch_info, struct cortex_a_common, armv7a_common.arm);
}

#endif /* OPENOCD_TARGET_CORTEX_A_H */
