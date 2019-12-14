/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2015 Uwe Bonnes                                         *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
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
	.cpu cortex-m4
	.thumb

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (64bit words)
 * r4 = flash base
 *
 * Clobbered:
 * r5   - rp
 * r6/7 - temp (64-bit)
 * r8   - wp, tmp
 */

#define STM32_FLASH_CR_OFFSET	0x14	/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x10	/* offset of SR register in FLASH struct */

#define STM32_PROG	 0x1	/* PG */

	.thumb_func
	.global _start
_start:
wait_fifo:
	ldr 	r8, [r0, #0]	/* read wp */
	cmp 	r8, #0		/* abort if wp == 0 */
	beq 	exit
	ldr 	r5, [r0, #4]	/* read rp */
	subs	r6, r8, r5	/* number of bytes available for read in r6*/
	itt 	mi              /* if wrapped around*/
	addmi	r6, r1          /* add size of buffer */
	submi	r6, r0
	cmp 	r6, #8		/* wait until 8 bytes are available */
	bcc 	wait_fifo

	ldr	r6, =STM32_PROG
	str	r6, [r4, #STM32_FLASH_CR_OFFSET]
	ldrd 	r6, [r5], #0x08	/* read one word from src, increment ptr */
	strd 	r6, [r2], #0x08	/* write one word to dst, increment ptr */
	dsb
busy:
	ldr 	r6, [r4, #STM32_FLASH_SR_OFFSET]
	tst 	r6, #0x10000	/* BSY (bit16) == 1 => operation in progress */
	bne 	busy		/* wait more... */
	tst	r6, #0xfa	/* PGSERR | SIZERR | PGAERR | WRPERR | PROGERR | OPERR */
	bne	error		/* fail... */

	cmp 	r5, r1		/* wrap rp at end of buffer */
	it  	cs
	addcs	r5, r0, #8	/* skip loader args */
	str 	r5, [r0, #4]	/* store rp */
	subs	r3, r3, #1	/* decrement dword count */
	cbz 	r3, exit	/* loop if not done */
	b	wait_fifo
error:
	movs	r1, #0
	str	r1, [r0, #4]	/* set rp = 0 on error */
exit:
	mov	r0, r6		/* return status in r0 */
	bkpt	#0x00

	.pool

