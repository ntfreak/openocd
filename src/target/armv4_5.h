/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009 by Øyvind Harboe                                   *
 *   oyvind.harboe@zylin.com                                               *
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

#ifndef OPENOCD_TARGET_ARMV4_5_H
#define OPENOCD_TARGET_ARMV4_5_H

/* This stuff "knows" that its callers aren't talking
 * to microcontroller profile (current Cortex-M) parts.
 * We want to phase it out so core code can be shared.
 */

/* OBSOLETE, DO NOT USE IN NEW CODE!  The "number" of an arm_mode is an
 * index into the armv4_5_core_reg_map array.  Its remaining users are
 * remnants which could as easily walk * the register cache directly as
 * use the expensive ARMV4_5_CORE_REG_MODE() macro.
 */
int arm_mode_to_number(enum arm_mode mode);
enum arm_mode armv4_5_number_to_mode(int number);

extern const int armv4_5_core_reg_map[9][17];

#define ARMV4_5_CORE_REG_MODE(cache, mode, num) \
		(cache->reg_list[armv4_5_core_reg_map[arm_mode_to_number(mode)][num]])

/* offset into armv4_5 core register cache -- OBSOLETE, DO NOT USE! */
enum { ARMV4_5_CPSR = 31, };

#endif /* OPENOCD_TARGET_ARMV4_5_H */
