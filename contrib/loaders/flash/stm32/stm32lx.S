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
 *                                                                         *
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
 * r0 = source address (return status)
 *      0 - OK
 *      non zero value from SR on error
 * r1 = target address
 * r2 = count (32bit words)
 * r3 = flash base
 * r4 = page size
 */

#define FLASH_PECR_OFFSET    0x04 /* offset of PECR register in FLASH struct */
#define FLASH_SR_OFFSET      0x18 /* offset of SR register in FLASH struct */
#define FLASH_SR_MASK_BSY    0x01
#define FLASH_SR_MASK_EOP    0x02
#define FLASH_SR_MASK_READY  0x08
#define FLASH_SR_MASK_ERRS   0x33F00

/* FLASH controller expected to finish programming in around 3.2ms (~51200 cycles on 16MHz clock)
 * Defining it to 220000 cycles gives reasonable amount of time check
 */
#define BSY_TIMEOUT          220000


    .thumb_func
    .global _start
_start:
    lsrs    r4, r4, #1
    subs    r4, r4, #1

check_bounds:
    cmp     r2, #0
    bne     store_word
    movs    r0, #0
    b       halt

finish_timeout:
    movs    r0, #1
    b       halt

halt:
    bkpt    0x0000

store_word:
    ldmia   r0!, {r5}
    stmia   r1!, {r5}
    adds    r7, r7, #1
    subs    r2, r2, #1

    movs    r5, r1
    ands    r5, r5, r4
    cmp     r5, #0
    beq     hp_finalize /* half-page has been programmed */

    cmp     r5, #1
    beq     long_delay
    movs    r5, #1
    b       latch_delay /* wait 6 cycles after 6nd word */

long_delay:
    movs    r5, #58     /* wait 63 cycles between first 2 words */

latch_delay:
    subs    r5, r5, #1
    cmp     r5, #0
    bne     latch_delay
    b       store_word

hp_finalize:
    ldr     r7, =BSY_TIMEOUT
bsy_loop:
    subs    r7, #1
    cmp     r7, #0
    beq     finish_timeout
    ldr     r6, [r3, #FLASH_SR_OFFSET]
    movs    r5, #FLASH_SR_MASK_BSY
    ands    r5, r5, r6
    cmp     r5, #0
    bne     bsy_loop

/* clear EOP flag */
    ldr     r6, [r3, #FLASH_SR_OFFSET]
    movs    r5, #FLASH_SR_MASK_EOP
    eors    r6, r6, r5
    str     r6, [r3, #FLASH_SR_OFFSET]

/* check for controller errors */
    ldr     r6, [r3, #FLASH_SR_OFFSET]
    ldr     r5, =FLASH_SR_MASK_ERRS
    ands    r5, r5, r6
    cmp     r5, #0
    b       check_bounds
    /* store errors mask for return result */
    mov     r0, r5
    b       halt
