/***************************************************************************
 *   Copyright (C) 2017 by STMicroelectronics                              *
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
 * Code limitations:
 * The workarea must have size multiple of 4 bytes, since R/W
 * operations are all at 32 bits.
 * The workarea must be big enough to contain 32 bytes of data,
 * thus the minimum size is (rp, wp, data) = 4 + 4 + 32 = 40 bytes.
 * To benefit from concurrent host write-to-buffer and target
 * write-to-flash, the workarea must be way bigger than the minimum.
 */

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (256 bit words)
 * r4 = flash reg base
 *
 * Clobbered:
 * r5 - rp
 * r6 - wp, status, tmp
 * r7 - loop index, tmp
 */

#define STM32_FLASH_CR_OFFSET	0x0C	/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x10	/* offset of SR register in FLASH struct */
#define STM32_CR_PROG			0x00000032	/* PSIZE64 | PG */
#define STM32_SR_QW_MASK		0x00000004	/* QW */
#define STM32_SR_ERROR_MASK		0x07ee0000	/* DBECCERR | SNECCERR | RDSERR | RDPERR | OPERR
											   | INCERR | STRBERR | PGSERR | WRPERR */

	.thumb_func
	.global _start
_start:
	ldr		r5, [r0, #4]		/* read rp */

wait_fifo:
	ldr		r6, [r0, #0]		/* read wp */
	cbz		r6, exit			/* abort if wp == 0, status = 0 */
	subs	r6, r6, r5			/* number of bytes available for read in r6 */
	ittt	mi					/* if wrapped around */
	addmi	r6, r1				/* add size of buffer */
	submi	r6, r0
	submi	r6, #8
	cmp		r6, #32				/* wait until 32 bytes are available */
	bcc		wait_fifo

	mov		r6, #STM32_CR_PROG
	str		r6, [r4, #STM32_FLASH_CR_OFFSET]

	mov		r7, #8				/* program by 8 words = 32 bytes */
write_flash:
	dsb
	ldr		r6, [r5], #0x04		/* read one word from src, increment ptr */
	str		r6, [r2], #0x04		/* write one word to dst, increment ptr */
	dsb
	cmp		r5, r1				/* if rp >= end of buffer ... */
	it		cs
	addcs	r5, r0, #8			/* ... then wrap at buffer start */
	subs	r7, r7, #1			/* decrement loop index */
	bne		write_flash			/* loop if not done */

busy:
	ldr		r6, [r4, #STM32_FLASH_SR_OFFSET]
	tst		r6, #STM32_SR_QW_MASK
	bne		busy				/* operation in progress, wait ... */

	ldr		r7, =STM32_SR_ERROR_MASK
	tst		r6, r7
	bne		error				/* fail... */

	str		r5, [r0, #4]		/* store rp */
	subs	r3, r3, #1			/* decrement count */
	bne		wait_fifo			/* loop if not done */
	b		exit

error:
	movs	r7, #0
	str		r7, [r0, #4]		/* set rp = 0 on error */

exit:
	mov		r0, r6				/* return status in r0 */
	bkpt	#0x00

	.pool

