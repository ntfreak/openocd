/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef OPENOCD_TARGET_ARM926EJS_H
#define OPENOCD_TARGET_ARM926EJS_H

#include "arm9tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM926EJS_COMMON_MAGIC 0xa926a926

struct arm926ejs_common {
	struct arm7_9_common arm7_9_common;
	uint32_t common_magic;
	struct armv4_5_mmu_common armv4_5_mmu;
	int (*read_cp15)(struct target *target, uint32_t op1, uint32_t op2,
			uint32_t crn, uint32_t crm, uint32_t *value);
	int (*write_cp15)(struct target *target, uint32_t op1, uint32_t op2,
			uint32_t crn, uint32_t crm, uint32_t value);
	uint32_t cp15_control_reg;
	uint32_t d_fsr;
	uint32_t i_fsr;
	uint32_t d_far;
};

static inline struct arm926ejs_common *target_to_arm926(struct target *target)
{
	return container_of(target->arch_info, struct arm926ejs_common, arm7_9_common.arm);
}

int arm926ejs_init_arch_info(struct target *target,
		struct arm926ejs_common *arm926ejs, struct jtag_tap *tap);
int arm926ejs_arch_state(struct target *target);
int arm926ejs_write_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
int arm926ejs_soft_reset_halt(struct target *target);

extern const struct command_registration arm926ejs_command_handlers[];

#endif /* OPENOCD_TARGET_ARM926EJS_H */
