/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#ifndef OPENOCD_RTOS_RTOS_STANDARD_STACKINGS_H
#define OPENOCD_RTOS_RTOS_STANDARD_STACKINGS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"

extern const struct rtos_register_stacking rtos_standard_cortex_m3_stacking;
extern const struct rtos_register_stacking rtos_standard_cortex_m4f_stacking;
extern const struct rtos_register_stacking rtos_standard_cortex_m4f_fpu_stacking;
extern const struct rtos_register_stacking rtos_standard_cortex_r4_stacking;
extern const struct rtos_register_stacking rtos_standard_nds32_n1068_stacking;
int64_t rtos_generic_stack_align8(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	int64_t stack_ptr);
int64_t rtos_cortex_m_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	int64_t stack_ptr, size_t xpsr_offset);

#endif /* OPENOCD_RTOS_RTOS_STANDARD_STACKINGS_H */
