/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2011 Clement Burin des Roziers                          *
 *   clement.burin-des-roziers@hikob.com                                   *
 *                                                                         *
 *   Copyright (C) 2017 Armin van der Togt                                 *
 *   armin@otheruse.nl                                                     *
 *   Copyright (C) 2019 Artur Troian                                       *
 *   troian dot ap at gmail dot com                                        *
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
	.cpu cortex-m0
	.thumb

/*
 * Params :
 * r0 = target address
 * r1 = source address
 * r2 = count (32bit words)
 * r3 = flash base
 */

	.thumb_func
	.global _start
_start:
    movs    r6, #0

check_bounds:
    cmp     r2, #0
    bne     store_word
    bkpt    0x0000

end_loop:
    b       end_loop

store_word:
    movs    r4, #15
    lsls    r5, r6, #2
    ldmia   r1!, {r7}
    stmia   r0!, {r7}
    adds    r5, r6, #1
    movs    r6, r5
    tst     r5, r4
    bne     store_word
    movs    r5, r3
copy_loop:
    movs    r4, #1
    ldr     r7, [r5, #24]
    tst     r7, r4
    bne     copy_loop
    ldr     r7, [r5, #24]
    subs    r2, #16
    orrs    r7, r4
    str     r7, [r5, #24]
    b       check_bounds
