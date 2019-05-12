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
 *   Copyright (C) 2018 Andreas Bolsch                                     *
 *   andreas.bolsch@mni.thm.de                                             *
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
 * r0 = workarea start, status (out)
 * r1 = workarea end + 1
 * r2 = target address
 * r3 = count (64bit words)
 * r4 = flash status register
 * r5 = flash control register
 *
 * Clobbered:
 * r6/7 - temp (64-bit)
 */

#include "../../../../src/flash/nor/stm32l4x.h"

	.thumb_func
	.global _start

_start:
	mov		r8, r3						/* copy dword count */
wait_fifo:
	ldr		r6, [r0, #0]				/* read wp */
	cmp		r6, #0						/* if wp == 0,  */
	beq		exit						/* then abort */
	ldr		r3, [r0, #4]				/* read rp */
	subs	r6, r6, r3					/* number of bytes available for read in r6 */
	bpl		fifo_stat					/* if not wrapped around, skip */
	adds	r6, r6, r1					/* add end of buffer */
	subs	r6, r6, r0					/* sub start of buffer */
fifo_stat:
	cmp		r6, #8						/* wait until at least one dword available */
	bcc		wait_fifo

	movs	r6, #FLASH_PG				/* flash program enable */
	str		r6, [r5]					/* write to FLASH_CR, start operation */
	ldmia	r3!, {r6, r7}				/* read one dword from src, increment ptr */
	stmia	r2!, {r6, r7}				/* write one dword to dst, increment ptr */
	dsb
	ldr		r7, =FLASH_BSY				/* FLASH_BSY mask */
busy:
	ldr 	r6, [r4]					/* get FLASH_SR register */
	tst		r6, r7						/* BSY == 1 => operation in progress */
	bne 	busy						/* if still set, wait more ... */
	movs	r7, #FLASH_ERROR			/* all error bits */
	tst		r6, r7						/* check for any error bit */
	bne		error						/* fail ... */

	cmp		r3, r1						/* rp at end of buffer? */
	bcc		upd_rp						/* if no, then skip */
	subs	r3, r3, r1					/* sub end of buffer */
	adds	r3, r3, r0					/* add start of buffer */
	adds	r3, r3, #8					/* skip wp and rp */
upd_rp:
	str		r3, [r0, #4]				/* store rp */
	mov		r7, r8						/* get dword count */
	subs	r7, r7, #1					/* decrement dword count */
	mov		r8, r7						/* save dword count */
	beq		exit						/* exit if done */
	b		wait_fifo

	.pool

error:
	movs	r3, #0
	str		r3, [r0, #4]				/* set rp = 0 on error */
exit:
	mov		r0, r6						/* return status in r0 */
	movs	r6, #0						/* flash program disable */
	str		r6, [r5]					/* write to FLASH_CR */
	movs	r6, #FLASH_ERROR			/* all error bits */
	str		r6, [r4]					/* write to FLASH_CR to clear errors */
	bkpt	#0x00
