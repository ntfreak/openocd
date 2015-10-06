/***************************************************************************
 *   Copyright (C) 2015 by Daniel Krebs                                    *
 *   Daniel Krebs - github@daniel-krebs.net                                *
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
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "target/armv7m.h"

/* see thread_arch.c */
static const struct stack_register_offset rtos_riot_Cortex_M0_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ 0x24, 32 },		/* r0	*/
	{ 0x28, 32 },		/* r1	*/
	{ 0x2c, 32 },		/* r2	*/
	{ 0x30, 32 },		/* r3	*/
	{ 0x14, 32 },		/* r4	*/
	{ 0x18, 32 },		/* r5	*/
	{ 0x1c, 32 },		/* r6	*/
	{ 0x20, 32 },		/* r7	*/
	{ 0x04, 32 },		/* r8	*/
	{ 0x08, 32 },		/* r9	*/
	{ 0x0c, 32 },		/* r10	*/
	{ 0x10, 32 },		/* r11	*/
	{ 0x34, 32 },		/* r12	*/
	{ -2,	32 },		/* sp	*/
	{ 0x38, 32 },		/* lr	*/
	{ 0x3c, 32 },		/* pc	*/
	{ 0x40, 32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_riot_Cortex_M0_stacking = {
	0x44,					/* stack_registers_size */
	-1,						/* stack_growth_direction */
	ARMV7M_NUM_CORE_REGS,	/* num_output_registers */
	8,						/* stack_alignment */
	rtos_riot_Cortex_M0_stack_offsets	/* register_offsets */
};

/* see thread_arch.c */
static const struct stack_register_offset rtos_riot_Cortex_M34_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ 0x24, 32 },		/* r0	*/
	{ 0x28, 32 },		/* r1	*/
	{ 0x2c, 32 },		/* r2	*/
	{ 0x30, 32 },		/* r3	*/
	{ 0x04, 32 },		/* r4	*/
	{ 0x08, 32 },		/* r5	*/
	{ 0x0c, 32 },		/* r6	*/
	{ 0x10, 32 },		/* r7	*/
	{ 0x14, 32 },		/* r8	*/
	{ 0x18, 32 },		/* r9	*/
	{ 0x1c, 32 },		/* r10	*/
	{ 0x20, 32 },		/* r11	*/
	{ 0x34, 32 },		/* r12	*/
	{ -2,	32 },		/* sp	*/
	{ 0x38, 32 },		/* lr	*/
	{ 0x3c, 32 },		/* pc	*/
	{ 0x40, 32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_riot_Cortex_M34_stacking = {
	0x44,					/* stack_registers_size */
	-1,						/* stack_growth_direction */
	ARMV7M_NUM_CORE_REGS,	/* num_output_registers */
	8,						/* stack_alignment */
	rtos_riot_Cortex_M34_stack_offsets	 /* register_offsets */
};
