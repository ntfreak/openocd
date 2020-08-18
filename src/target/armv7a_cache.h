/***************************************************************************
 *   Copyright (C) 2015 Oleksij Rempel                                     *
 *   linux@rempel-privat.de                                                *
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

#ifndef OPENOCD_TARGET_ARM7A_CACHE_H
#define OPENOCD_TARGET_ARM7A_CACHE_H

#include "arm_jtag.h"
#include "armv7a_cache_l2x.h"

int armv7a_l1_d_cache_clean_virt(struct target *target, uint32_t virt,
					unsigned int size);
int armv7a_l1_d_cache_inval_virt(struct target *target, uint32_t virt,
					uint32_t size);
int armv7a_l1_d_cache_flush_virt(struct target *target, uint32_t virt,
					unsigned int size);
int armv7a_l1_i_cache_inval_all(struct target *target);
int armv7a_l1_i_cache_inval_virt(struct target *target, uint32_t virt,
					uint32_t size);
int armv7a_cache_auto_flush_on_write(struct target *target, uint32_t virt,
					uint32_t size);
int armv7a_cache_auto_flush_all_data(struct target *target);
int armv7a_cache_flush_virt(struct target *target, uint32_t virt,
				uint32_t size);
extern const struct command_registration arm7a_cache_command_handlers[];

/* CLIDR cache types */
#define CACHE_LEVEL_HAS_UNIFIED_CACHE	0x4
#define CACHE_LEVEL_HAS_D_CACHE		0x2
#define CACHE_LEVEL_HAS_I_CACHE		0x1

#endif /* OPENOCD_TARGET_ARM7A_CACHE_H */
