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

#ifndef OPENOCD_TARGET_DSP563XX_H
#define OPENOCD_TARGET_DSP563XX_H

#include <jtag/jtag.h>
#include <target/dsp563xx_once.h>

#define DSP563XX_NUMCOREREGS	54
#define DSP563XX_NUMONCEREGS	25

struct mcu_jtag {
	struct jtag_tap *tap;
};

enum breakpoint_usage {
	BPU_NONE = 0,
	BPU_BREAKPOINT,
	BPU_WATCHPOINT
};

struct hardware_breakpoint {
	enum breakpoint_usage used;
};

struct dsp563xx_common {
	struct mcu_jtag jtag_info;
	struct reg_cache *core_cache;
	uint32_t core_regs[DSP563XX_NUMCOREREGS];
	struct once_reg once_regs[DSP563XX_NUMONCEREGS];

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, int num);
	int (*write_core_reg)(struct target *target, int num);

	struct hardware_breakpoint hardware_breakpoint[1];

	/*Were the hardware breakpoints cleared on startup?*/
	int hardware_breakpoints_cleared;
};

struct dsp563xx_core_reg {
	uint32_t num;
	const char *name;
	uint32_t size;
	uint8_t eame;
	uint32_t instr_mask;
	struct target *target;
	struct dsp563xx_common *dsp563xx_common;
};

static inline struct dsp563xx_common *target_to_dsp563xx(struct target *target)
{
	return target->arch_info;
}

#endif /* OPENOCD_TARGET_DSP563XX_H */
