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
 * The workarea must be big enough to contain rp, wp and data, thus the minimum
 * workarea size is: min_wa_size = sizeof(rp, wp, data) = 4 + 4 + sizeof(data).
 *  - for 0x450 devices: sizeof(data) = 32 bytes, thus min_wa_size = 40 bytes.
 *  - for 0x480 devices: sizeof(data) = 16 bytes, thus min_wa_size = 24 bytes.
 * To benefit from concurrent host write-to-buffer and target
 * write-to-flash, the workarea must be way bigger than the minimum.
 *
 * To avoid confusions the write word size is got from .block_size member of
 * struct stm32h7x_part_info defined in stm32h7x.c
*/

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (of write words)
 * r4 = size of write word
 * r5 = flash reg base
 *
 * Clobbered:
 * r6 - rp
 * r7 - wp, status, tmp
 * r8 - loop index, tmp
 */

#define STM32_FLASH_CR_OFFSET	0x0C	/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x10	/* offset of SR register in FLASH struct */
#define STM32_CR_PROG			0x00000002	/* PG */
#define STM32_SR_QW_MASK		0x00000004	/* QW */
#define STM32_SR_ERROR_MASK		0x07ee0000	/* DBECCERR | SNECCERR | RDSERR | RDPERR | OPERR
											   | INCERR | STRBERR | PGSERR | WRPERR */

	.thumb_func
	.global _start
_start:
	ldr		r6, [r0, #4]		/* read rp */

wait_fifo:
	ldr		r7, [r0, #0]		/* read wp */
	cbz		r7, exit			/* abort if wp == 0, status = 0 */
	subs	r7, r7, r6			/* number of bytes available for read in r7 */
	ittt	mi					/* if wrapped around */
	addmi	r7, r1				/* add size of buffer */
	submi	r7, r0
	submi	r7, #8
	cmp		r7, r4				/* wait until data buffer is full */
	bcc		wait_fifo

	mov		r7, #STM32_CR_PROG
	str		r7, [r5, #STM32_FLASH_CR_OFFSET]

	mov		r8, #4
	udiv	r8, r4, r8			/* number of words is size of write word divided by 4*/
write_flash:
	dsb
	ldr		r7, [r6], #0x04		/* read one word from src, increment ptr */
	str		r7, [r2], #0x04		/* write one word to dst, increment ptr */
	dsb
	cmp		r6, r1				/* if rp >= end of buffer ... */
	it		cs
	addcs	r6, r0, #8			/* ... then wrap at buffer start */
	subs	r8, r8, #1			/* decrement loop index */
	bne		write_flash			/* loop if not done */

busy:
	ldr		r7, [r5, #STM32_FLASH_SR_OFFSET]
	tst		r7, #STM32_SR_QW_MASK
	bne		busy				/* operation in progress, wait ... */

	ldr		r8, =STM32_SR_ERROR_MASK
	tst		r7, r8
	bne		error				/* fail... */

	str		r6, [r0, #4]		/* store rp */
	subs	r3, r3, #1			/* decrement count */
	bne		wait_fifo			/* loop if not done */
	b		exit

error:
	movs	r8, #0
	str		r8, [r0, #4]		/* set rp = 0 on error */

exit:
	mov		r0, r7				/* return status in r0 */
	bkpt	#0x00

	.pool
