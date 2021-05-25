/***************************************************************************
 *   Copyright (C) 2017 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/types.h>
#include <rtos/rtos.h>
#include <rtos/rtos_standard_stackings.h>
#include <target/armv7m.h>
#include <target/esirisc.h>

static const struct stack_register_offset rtos_ucos_iii_cortex_m_stack_offsets[] = {
	{ ARMV7M_R0,   0x20, 32 },	/* r0   */
	{ ARMV7M_R1,   0x24, 32 },	/* r1   */
	{ ARMV7M_R2,   0x28, 32 },	/* r2   */
	{ ARMV7M_R3,   0x2c, 32 },	/* r3   */
	{ ARMV7M_R4,   0x00, 32 },	/* r4   */
	{ ARMV7M_R5,   0x04, 32 },	/* r5   */
	{ ARMV7M_R6,   0x08, 32 },	/* r6   */
	{ ARMV7M_R7,   0x0c, 32 },	/* r7   */
	{ ARMV7M_R8,   0x10, 32 },	/* r8   */
	{ ARMV7M_R9,   0x14, 32 },	/* r9   */
	{ ARMV7M_R10,  0x18, 32 },	/* r10  */
	{ ARMV7M_R11,  0x1c, 32 },	/* r11  */
	{ ARMV7M_R12,  0x30, 32 },	/* r12  */
	{ ARMV7M_R13,  -2,   32 },	/* sp   */
	{ ARMV7M_R14,  0x34, 32 },	/* lr   */
	{ ARMV7M_PC,   0x38, 32 },	/* pc   */
	{ ARMV7M_xPSR, 0x3c, 32 },	/* xPSR */
};

static const struct stack_register_offset rtos_ucos_iii_esi_risc_stack_offsets[] = {
	{ ESIRISC_SP,  -2,   32 },	/* sp   */
	{ ESIRISC_RA,  0x48, 32 },	/* ra   */
	{ ESIRISC_R2,  0x44, 32 },	/* r2   */
	{ ESIRISC_R3,  0x40, 32 },	/* r3   */
	{ ESIRISC_R4,  0x3c, 32 },	/* r4   */
	{ ESIRISC_R5,  0x38, 32 },	/* r5   */
	{ ESIRISC_R6,  0x34, 32 },	/* r6   */
	{ ESIRISC_R7,  0x30, 32 },	/* r7   */
	{ ESIRISC_R8,  0x2c, 32 },	/* r8   */
	{ ESIRISC_R9,  0x28, 32 },	/* r9   */
	{ ESIRISC_R10, 0x24, 32 },	/* r10  */
	{ ESIRISC_R11, 0x20, 32 },	/* r11  */
	{ ESIRISC_R12, 0x1c, 32 },	/* r12  */
	{ ESIRISC_R13, 0x18, 32 },	/* r13  */
	{ ESIRISC_R14, 0x14, 32 },	/* r14  */
	{ ESIRISC_R15, 0x10, 32 },	/* r15  */
	{ ESIRISC_PC,  0x04, 32 },	/* PC   */
	{ ESIRISC_CAS, 0x08, 32 },	/* CAS  */
};

const struct rtos_register_stacking rtos_ucos_iii_cortex_m_stacking = {
	0x40,												/* stack_registers_size */
	-1,													/* stack_growth_direction */
	ARRAY_SIZE(rtos_ucos_iii_cortex_m_stack_offsets),	/* num_output_registers */
	rtos_generic_stack_align8,							/* stack_alignment */
	rtos_ucos_iii_cortex_m_stack_offsets				/* register_offsets */
};

const struct rtos_register_stacking rtos_ucos_iii_esi_risc_stacking = {
	0x4c,												/* stack_registers_size */
	-1,													/* stack_growth_direction */
	ARRAY_SIZE(rtos_ucos_iii_esi_risc_stack_offsets),	/* num_output_registers */
	NULL,												/* stack_alignment */
	rtos_ucos_iii_esi_risc_stack_offsets				/* register_offsets */
};
