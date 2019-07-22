/***************************************************************************
 *   Copyright (C) 2014 Angus Gratton                                      *
 *   gus@projectgus.com                                                    *
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

	.text
	.syntax unified
	.thumb

/*
 * Params :
 * r0 = byte count
 * r1 = buffer start
 * r2 = buffer end
 * r3 = target address
 * r6 = watchdog refresh value
 * r7 = watchdog refresh register address
 */

	.thumb_func
	.global _start
_start:
wait_fifo:
	str	r6, [r7, #0]
	ldr	r5, [r1, #0]
	cmp	r5, #0
	beq.n	exit
	ldr	r4, [r1, #4]
	cmp	r4, r5
	beq.n	wait_fifo
	ldmia	r4!, {r5}
	stmia	r3!, {r5}
	cmp	r4, r2
	bcc.n   no_wrap
	mov	r4, r1
	adds	r4, #8
no_wrap:
	str	r4, [r1, #4]
	subs	r0, #4
	bne.n   wait_fifo
exit:
	bkpt	#0x00

	.pool
