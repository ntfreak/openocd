/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef OPENOCD_TARGET_ARM9TDMI_H
#define OPENOCD_TARGET_ARM9TDMI_H

#include "embeddedice.h"

int arm9tdmi_init_target(struct command_context *cmd_ctx,
		struct target *target);
void arm9tdmi_deinit_target(struct target *target);
int arm9tdmi_init_arch_info(struct target *target,
		struct arm7_9_common *arm7_9, struct jtag_tap *tap);
extern const struct command_registration arm9tdmi_command_handlers[];

int arm9tdmi_clock_out(struct arm_jtag *jtag_info,
		uint32_t instr, uint32_t out, uint32_t *in, int sysspeed);
int arm9tdmi_clock_data_in(struct arm_jtag *jtag_info, uint32_t *in);
int arm9tdmi_clock_data_in_endianness(struct arm_jtag *jtag_info,
		void *in, int size, int be);
void arm9tdmi_read_core_regs(struct target *target,
		uint32_t mask, uint32_t *core_regs[16]);
void arm9tdmi_write_core_regs(struct target *target,
		uint32_t mask, uint32_t core_regs[16]);

int arm9tdmi_examine_debug_reason(struct target *target);

void arm9tdmi_load_word_regs(struct target *target, uint32_t mask);
void arm9tdmi_load_hword_reg(struct target *target, int num);
void arm9tdmi_load_byte_reg(struct target *target, int num);
void arm9tdmi_store_word_regs(struct target *target, uint32_t mask);
void arm9tdmi_store_hword_reg(struct target *target, int num);
void arm9tdmi_store_byte_reg(struct target *target, int num);

void arm9tdmi_branch_resume(struct target *target);
void arm9tdmi_enable_single_step(struct target *target, uint32_t next_pc);
void arm9tdmi_disable_single_step(struct target *target);

#endif /* OPENOCD_TARGET_ARM9TDMI_H */
