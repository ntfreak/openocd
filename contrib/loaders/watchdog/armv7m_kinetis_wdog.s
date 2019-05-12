/***************************************************************************
 *   Copyright (C) 2015 Tomas Vanek                                        *
 *   vanekt@fbl.cz                                                         *
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
 *   Free Software Foundation, Inc.                                        *
 ***************************************************************************/

/*
	Disable watchdog for Kinetis Kx and KVx
	Parameters:
		r0 ... WDOG base (in)

	Used instruction set should work on both Cortex-M4 and M0+
*/

	.text
	.syntax unified
        .cpu cortex-m0
	.thumb

/* WDOG registers offsets */
WDOG_STCTRLH	= 0
WDOG_UNLOCK	= 0x0e

WDOG_KEY1	= 0xc520
WDOG_KEY2	= 0xd928

	.thumb_func
start:
/* WDOG_UNLOCK = 0xC520 */
	ldr     r2, =WDOG_KEY1
	strh    r2, [r0, WDOG_UNLOCK]
/* WDOG_UNLOCK = 0xD928 */
	ldr     r2, =WDOG_KEY2
	strh    r2, [r0, WDOG_UNLOCK]
/* WDOG_STCTRLH clear bit 0 */
	movs	r4, #1
	ldrh    r2, [r0, WDOG_STCTRLH]
	bics	r2, r4
	strh    r2, [r0, WDOG_STCTRLH]
/* OpenOCD checks exit point address. Jump to the very end. */
	b	done

	.pool

/* Avoid padding at .text segment end. Otherwise exit point check fails. */
	.skip	( . - start + 2) & 2, 0
done:
	bkpt    #0

	.end
