/***************************************************************************
 *   Copyright (C) 2018 by Andreas Bolsch                                  *
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
 * r0 - total count (bytes), remaining bytes (out, 0 means successful)
 * r1 - flash page size
 * r2 - address offset into flash
 * r3 - OCTOSPI io_base
 * r8 - fifo start
 * r9 - fifo end + 1

 * Clobbered:
 * r4 - rp
 * r5 - address of OCTOSPI_DR
 * r6 - address of OCTOSPI_CCR
 * r7 - tmp
 * r10 - single 0x0 / dual 0x1
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
	ldr		r4, rp						/* load rp */
	ldr		r7, [r3, #OCTOSPI_CR]		/* get OCTOSPI_CR register */
	lsls	r7, r7, #(31-SPI_DUAL_FLASH)	/* clear higher order bits */
	lsrs	r7, r7, #31					/* DUAL_FLASH bit into bit 0 */
	mov		r10, r7						/* save in r10 */
wip_loop:
	octospi_abort						/* start in clean state */
	movs	r5, #OCTOSPI_DR				/* load OCTOSPI_DR address offset */
	adds	r5, r5, r3					/* address of OCTOSPI_DR */
	movs	r6, #OCTOSPI_CCR-OCTOSPI_DR	/* load OCTOSPI_CCR address offset */
	adds	r6, r6, r5					/* address of OCTOSPI_CCR */
	wait_busy
	ldr		r7, cr_read_status			/* indirect read mode */
	str		r7, [r3, #OCTOSPI_CR]		/* set mode */
	mov		r7, r10						/* get dual bit */
	str		r7, [r3, #OCTOSPI_DLR]		/* one or two (for dual) bytes */
	ldr		r7, ccr_read_status			/* CCR for status read */
	str		r7, [r6, #OCTOSPI_CCR_CCR]	/* initiate status read */
	ldr		r7, tcr_read_status			/* TCR for status read */
	str		r7, [r6, #OCTOSPI_TCR_CCR]	/* instruction */
	ldr		r7, ir_read_status			/* IR for status read */
	str		r7, [r6, #OCTOSPI_IR_CCR]	/* instruction */
	movs	r7, #0						/* dummy address */
	str		r7, [r3, #OCTOSPI_AR]		/* into AR (for 8-line mode) */
	ldrb	r7, [r5]					/* get first status register */
	lsrs	r7, r7, #(SPIFLASH_BSY+1)	/* if first flash busy, */
	bcs		wip_loop					/* then poll again */
	mov		r7, r10						/* get dual bit */
	tst		r7, r7						/* dual mode ? */
	beq		write_enable				/* not dual, then ok */
	ldrb	r7, [r5]					/* get second status register */
	lsrs	r7, r7, #(SPIFLASH_BSY+1)	/* if second flash busy, */
	bcs		wip_loop					/* then poll again */
write_enable:
	tst		r0, r0						/* test residual count */
	bmi		exit						/* if negative, then finished */
	wait_busy
	ldr		r7, cr_write_enable			/* indirect write mode */
	str		r7, [r3, #OCTOSPI_CR]		/* set mode */
	ldr		r7, ccr_write_enable		/* CCR for write enable */
	str		r7, [r6, #OCTOSPI_CCR_CCR]	/* initiate write enable */
	ldr		r7, tcr_write_enable		/* TCR for write enable */
	str		r7, [r6, #OCTOSPI_TCR_CCR]	/* write enable instruction */
	ldr		r7, ir_write_enable			/* IR for write enable */
	str		r7, [r6, #OCTOSPI_IR_CCR]	/* instruction */
	movs	r7, #0						/* silicon bug in L5? dummy write */
	str		r7, [r3, #OCTOSPI_AR]		/* into AR resolves issue */
	wait_busy
	ldr		r7, cr_read_status			/* indirect read mode */
	str		r7, [r3, #OCTOSPI_CR]		/* set mode */
	mov		r7, r10						/* get dual count */
	str		r7, [r3, #OCTOSPI_DLR]		/* one or two (for dual) bytes */
	ldr		r7, ccr_read_status			/* CCR for status read */
	str		r7, [r6, #OCTOSPI_CCR_CCR]	/* initiate status read */
	ldr		r7, tcr_read_status			/* TCR for status read */
	str		r7, [r6, #OCTOSPI_TCR_CCR]	/* instruction */
	ldr		r7, ir_read_status			/* IR for status read */
	str		r7, [r6, #OCTOSPI_IR_CCR]	/* instruction */
	movs	r7, #0						/* dummy address */
	str		r7, [r3, #OCTOSPI_AR]		/* into AR (for 8-line mode) */
	ldrb	r7, [r5]					/* get first status register */
	lsrs	r7, r7, #(SPIFLASH_WE+1)	/* if first flash not */
	bcc		error						/* write enabled, then error */
	mov		r7, r10						/* get dual bit */
	tst		r7, r7						/* dual mode ? */
	beq		start_write					/* not dual, then ok */
	ldrb	r7, [r5]					/* get second status register */
	lsrs	r7, r7, #(SPIFLASH_WE+1)	/* if second flash not */
	bcc		error						/* write enabled, then error */
start_write:
	wait_busy
	ldr		r7, cr_page_write			/* indirect write mode */
	str		r7, [r3, #OCTOSPI_CR]		/* set mode */
	mov		r7, r2						/* get current start address */
	orrs	r7, r7, r1					/* end of current page */
	subs	r7, r7, r2					/* count-1 to end of page */
	cmp		r7, r0						/* if this count <= remaining */
	bls		write_dlr					/* then write to end of page */
	mov		r7, r0						/* else write all remaining */
write_dlr:
	str		r7, [r3, #OCTOSPI_DLR]		/* size-1 in DLR register */
	ldr		r7, ccr_page_write			/* CCR for page write */
	str		r7, [r6, #OCTOSPI_CCR_CCR]	/* initiate transfer */
	ldr		r7, tcr_page_write			/* TCR for page write */
	str		r7, [r6, #OCTOSPI_TCR_CCR]	/* instruction */
	ldr		r7, ir_page_write			/* IR for page write */
	str		r7, [r6, #OCTOSPI_IR_CCR]	/* instruction */
	str		r2, [r3, #OCTOSPI_AR]		/* store SPI start address */
write_loop:
	ldr		r7, wp						/* get wp */
	cmp		r7, #0						/* if wp equals 0 */
	beq		exit						/* then abort */
	cmp		r4, r7						/* check if fifo empty */
	beq		write_loop					/* wait until not empty */
	ldrb	r7, [r4, #0]				/* read next byte */
	strb	r7, [r5]					/* write next byte to DR */
	adds	r4, r4, #1					/* increment internal rp */
	cmp		r4, r9						/* internal rp beyond end? */
	blo		upd_write					/* if no, then ok */
	mov		r4, r8						/* else wrap around */
upd_write:
	adr		r7, rp						/* get address of rp */
	str		r4, [r7]					/* store updated rp */
	adds	r2, r2, #1					/* increment address */
	subs	r0, r0, #1					/* decrement (count-1) */
	bmi		page_end					/* stop if no data left */
	tst		r2, r1						/* page end ? */
	bne		write_loop					/* if not, then next byte */
page_end:
	ldr		r7, [r3, #OCTOSPI_SR]		/* load status */
	lsrs	r7, r7, #(SPI_TCF+1)		/* shift TCF into C */
	bcc		page_end					/* loop until TCF set */
	bal		wip_loop					/* then next page */

error:
	movs	r0, #0						/* return 0xFFFFFFFF */
	subs	r0, r0, #2					/* for error */
exit:
	adds	r0, r0, #1					/* increment count due to the -1 */
	octospi_abort						/* to idle state */
	.align	2							/* align to word, bkpt is 4 words */
	bkpt	#0							/* before code end for exit_point */
	.align	2							/* align to word */

cr_read_status:
	.space	4							/* OCTOSPI_CR value for READ_STATUS command */
ccr_read_status:
	.space	4							/* OCTOSPI_CCR value for READ_STATUS command */
tcr_read_status:
	.space	4							/* OCTOSPI_TCR value for READ_STATUS command */
ir_read_status:
	.space	4							/* OCTOSPI_IR value for READ_STATUS command */

cr_write_enable:
	.space	4							/* OCTOSPI_CR value for WRITE_ENABLE command */
ccr_write_enable:
	.space	4							/* OCTOSPI_CCR value for WRITE_ENABLE command */
tcr_write_enable:
	.space	4							/* OCTOSPI_TCR value for WRITE_ENABLE command */
ir_write_enable:
	.space	4							/* OCTOSPI_IR value for WRITE_ENABLE command */

cr_page_write:
	.space	4							/* OCTOSPI_CR value for PAGE_PROG command */
ccr_page_write:
	.space	4							/* OCTOSPI_CCR value for PAGE_PROG command */
tcr_page_write:
	.space	4							/* OCTOSPI_TCR value for PAGE_PROG command */
ir_page_write:
	.space	4							/* OCTOSPI_IR value for PAGE_PROG command */

	.equ wp, .							/* wp, uint32_t */
	.equ rp, wp + 4						/* rp, uint32_t */
	.equ buffer, rp + 4					/* buffer follows right away */
