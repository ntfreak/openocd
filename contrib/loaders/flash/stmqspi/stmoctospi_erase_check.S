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
 * r0 - sector count
 * r1 - QSPI io_base

 * Clobbered:
 * r2 - r7 tmp */

#include "../../../../src/flash/nor/stmqspi.h"

#define OCTOSPI_CCR_CCR					(OCTOSPI_CCR - OCTOSPI_CCR)
#define OCTOSPI_TCR_CCR					(OCTOSPI_TCR - OCTOSPI_CCR)
#define OCTOSPI_IR_CCR					(OCTOSPI_IR - OCTOSPI_CCR)

	.macro	octospi_abort
	movs	r5, #(1<<SPI_ABORT)			/* abort bit mask */
	ldr		r7, [r1, #OCTOSPI_CR]		/* get OCTOSPI_CR register */
	orrs	r7, r7, r5					/* set abort bit */
	str		r7, [r1, #OCTOSPI_CR]		/* store new CR register */
	.endm

	.macro	wait_busy
0:
	ldr		r7, [r1, #OCTOSPI_SR]		/* load status */
	lsrs	r7, r7, #(SPI_BUSY+1)		/* shift BUSY into C */
	bcs		0b							/* loop until BUSY cleared */
	movs	r7, #(1<<SPI_TCF)			/* TCF bitmask */
	str		r7, [r1, #OCTOSPI_FCR]		/* clear TCF flag */
	.endm

start:
	adr		r2, buffer					/* pointer to start of buffer */
	movs	r3, #OCTOSPI_DR				/* load OCTOSPI_DR address offset */
	adds	r3, r3, r1					/* address of OCTOSPI_DR */
sector_start:
	octospi_abort						/* start in clean state */
	movs	r6, #OCTOSPI_CCR-OCTOSPI_DR	/* load OCTOSPI_CCR address offset */
	adds	r6, r6, r3					/* address of OCTOSPI_CCR */
	wait_busy
	ldr		r7, cr_page_read			/* indirect read mode */
	str		r7, [r1, #OCTOSPI_CR]		/* set mode */
	ldmia	r2!, {r4, r5}				/* load address offset, length */
	subs	r2, r2, #4					/* point to length */
	subs	r5, r5, #1					/* decrement sector length for DLR */
	str		r5, [r1, #OCTOSPI_DLR]		/* size-1 in DLR register */
	ldr		r7, ccr_page_read			/* CCR for read */
	str		r7, [r6, #OCTOSPI_CCR_CCR]	/* initiate transfer */
	ldr		r7, tcr_page_read			/* TCR for read */
	str		r7, [r6, #OCTOSPI_TCR_CCR]	/* instruction */
	ldr		r7, ir_page_read			/* IR for read */
	str		r7, [r6, #OCTOSPI_IR_CCR]	/* instruction */
	str		r4, [r1, #OCTOSPI_AR]		/* store SPI start address */
	ldr		r6, [r2, #4]				/* load initial value */
read_loop:
	ldrb	r4, [r3, #0]				/* read next byte from DR */
	movs	r7, #0xFF					/* fill bits 8-15 */
	lsls	r7, r7, #8					/* with ones */
	orrs	r4, r4, r7					/* copy ones to left of read byte */
	ands	r6, r6, r4					/* and read byte to result */
	lsls	r4, r4, #8					/* shift result into higher byte */
	orrs	r6, r6, r4					/* or read byte to result */
	subs	r5, r5, #1					/* decrement byte (count-1) */
	bpl		read_loop					/* again if sector not completed */
	adds	r5, r5, #1					/* increment count due to the -1 */
	stmia	r2!, {r5, r6}				/* save final count and result for sector */
	subs	r0, r0, #1					/* decrement sector count */
	bne		sector_start				/* next sector? */
	octospi_abort						/* to idle state */

exit:
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

	.equ buffer, .

