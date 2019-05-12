/***************************************************************************
 *   Copyright (C) 2017 Tomas Vanek                                        *
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
	Disable watchdog, 32-bit version for newer Kinetis
	Parameters:
		r0 ... WDOG32 base (in)

	Used instruction set should work on both Cortex-M4 and M0+
*/

	.text
	.syntax unified
        .cpu cortex-m0
	.thumb

/* WDOG registers offsets */
WDOG_CS		= 0
WDOG_CNT	= 4
WDOG_TOVAL	= 8

WDOG_KEY 	= 0xd928c520

	.thumb_func
start:
/* test WDOG_CS bit CMD32EN */
	ldr     r2, [r0, WDOG_CS]
	ldr     r3, =0x2000
	tst     r2, r3
	ldr     r3, =WDOG_KEY
	beq     cmd16

/* WDOG_CNT = key */
	str     r3, [r0, WDOG_CNT]
	b       unlocked

cmd16:
/* WDOG_CNT = key, halfword by halfword */
	strh    r3, [r0, WDOG_CNT]
	lsrs    r3, r3, #16
	strh    r3, [r0, WDOG_CNT]

/* WDOG_CS: clear EN bit 7, set UPDATE bit 5 */
unlocked:
	movs    r4, #0x80
	bics    r2, r4
	movs    r4, #0x20
	orrs    r2, r4
	str     r2, [r0, WDOG_CS]
/* All active WDOG registers have to be updated, set dummy timeout */
/* WDOG_TOVAL = 0x400 */
	ldr     r3, =0x400
	str     r3, [r0, WDOG_TOVAL]
/* OpenOCD checks exit point address. Jump to the very end. */
	b       done

	.pool

/* Avoid padding at .text segment end. Otherwise exit point check fails. */
	.skip	( . - start + 2) & 2, 0
done:
	bkpt    #0

	.end
