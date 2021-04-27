/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
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

#ifndef OPENOCD_TARGET_AVR32_AP7K_H
#define OPENOCD_TARGET_AVR32_AP7K_H

struct target;

#define AP7K_COMMON_MAGIC	0x4150374b
struct avr32_ap7k_common {
	int common_magic;
	struct avr32_jtag jtag;
	struct reg_cache *core_cache;
	uint32_t core_regs[AVR32NUMCOREREGS];
};

static inline struct avr32_ap7k_common *
target_to_ap7k(struct target *target)
{
	return (struct avr32_ap7k_common *)target->arch_info;
}

struct avr32_core_reg {
	uint32_t num;
	struct target *target;
	struct avr32_ap7k_common *avr32_common;
};

#endif /* OPENOCD_TARGET_AVR32_AP7K_H */
