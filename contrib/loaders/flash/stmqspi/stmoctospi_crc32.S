/***************************************************************************
 *   Copyright (C) 2019 by Andreas Bolsch                                  *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

/* Params:
 * r0 - total count (bytes), crc32 (out)
 * r1 - flash page size
 * r2 - address offset into flash
 * r3 - OCTOSPI io_base

 * Clobbered:
 * r4 - tmp
 * r5 - address of OCTOSPI_DR
 * r6 - address of OCTOSPI_CCR
 * r7 - tmp
 */

#include "../../../../src/flash/nor/stmqspi.h"

#define OCTOSPI_CCR_CCR					(OCTOSPI_CCR - OCTOSPI_CCR)
#define OCTOSPI_TCR_CCR					(OCTOSPI_TCR - OCTOSPI_CCR)
#define OCTOSPI_IR_CCR					(OCTOSPI_IR - OCTOSPI_CCR)

	.macro	octospi_abort
	movs	r5, #(1<<SPI_ABORT)			/* abort bit mask */
	ldr		r7, [r3, #OCTOSPI_CR]		/* get OCTOSPI CR register */
	orrs	r7, r7, r5					/* set abort bit */
	str		r7, [r3, #OCTOSPI_CR]		/* store new CR register */
	.endm

	.macro	wait_busy
0:
	ldr		r7, [r3, #OCTOSPI_SR]		/* load status */
	lsrs	r7, r7, #(SPI_BUSY+1)		/* shift BUSY into C */
	bcs		0b							/* loop until BUSY cleared */
	movs	r7, #(1<<SPI_TCF)			/* TCF bitmask */
	str		r7, [r3, #OCTOSPI_FCR]		/* clear TCF flag */
	.endm

start:
	subs	r0, r0, #1					/* decrement count for DLR */
	subs	r1, r1, #1					/* page size mask and for DLR */
	movs	r4, #0x00					/* initialize crc */
	mvns	r4, r4						/* to 0xFFFFFFFF */
start_read:
	octospi_abort						/* start in clean state */
	movs	r5, #OCTOSPI_DR				/* load OCTOSPI_DR address offset */
	adds	r5, r5, r3					/* address of OCTOSPI_DR */
	movs	r6, #OCTOSPI_CCR-OCTOSPI_DR	/* load OCTOSPI_CCR address offset */
	adds	r6, r6, r5					/* address of OCTOSPI_CCR */
	wait_busy
	ldr		r7, cr_page_read			/* indirect read mode */
	str		r7, [r3, #OCTOSPI_CR]		/* set mode */
	mov		r7, r2						/* get current start address */
	orrs	r7, r7, r1					/* end of current page */
	subs	r7, r7, r2					/* count-1 to end of page */
	cmp		r7, r0						/* if this count <= remaining */
	bls		write_dlr					/* then read to end of page */
	mov		r7, r0						/* else read all remaining */
write_dlr:
	str		r7, [r3, #OCTOSPI_DLR]		/* size-1 in DLR register */
	ldr		r7, ccr_page_read			/* CCR for read */
	str		r7, [r6, #OCTOSPI_CCR_CCR]	/* initiate transfer */
	ldr		r7, tcr_page_read			/* TCR for read */
	str		r7, [r6, #OCTOSPI_TCR_CCR]	/* instruction */
	ldr		r7, ir_page_read			/* IR for read */
	str		r7, [r6, #OCTOSPI_IR_CCR]	/* instruction */
	str		r2, [r3, #OCTOSPI_AR]		/* store SPI start address */
	ldr		r6, =0x04C11DB7				/* CRC32 polynomial */
read_loop:
	ldrb	r7, [r5]					/* read next byte from DR */
	lsls	r7, r7, #24					/* shift into msb */
	eors	r4, r4, r7
	.rept	8							/* unrolled bit loop */
	asrs	r7, r4, #31					/* copy bit 31 into bits 0 to 31 */
	ands	r7, r7, r6					/* r7 neg. -> CRC32XOR, pos. -> 0x0 */
	lsls	r4, r4, #1					/* shift result */
	eors	r4, r4, r7					/* eor by CRC32XOR or 0x0 */
	.endr
	adds	r2, r2, #1					/* increment address */
	subs	r0, r0, #1					/* decrement (count-1) */
	bmi		exit						/* stop if no data left */
	tst		r2, r1						/* page end ? */
	bne		read_loop					/* if not, then next byte */
page_end:
	bal		start_read					/* then next page */
	.pool

exit:
	mvns	r0, r4						/* invert to get final result */
	octospi_abort						/* to idle state */
	.align	2							/* align to word, bkpt is 4 words */
	bkpt	#0							/* before code end for exit_point */
	.align	2							/* align to word */

cr_page_read:
	.space	4							/* OCTOSPI_CR value for read command */
ccr_page_read:
	.space	4							/* OCTOSPI_CCR value for read command */
tcr_page_read:
	.space	4							/* OCTOSPI_TCR value for read command */
ir_page_read:
	.space	4							/* OCTOSPI_IR value for read command */
