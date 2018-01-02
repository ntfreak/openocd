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
 * r0 - sector count
 * r1 - flash page size
 * r2 - unused
 * r3 - number of address bytes required
 *
 * Clobbered:
 * r4 - r7, r10 - r12 tmp */

#include "cmspi.s"

start:
	mov		r8, r0						/* save sector count */
	adr		r0, buffer					/* pointer to start of buffer */
	mov		r9, r0						/* save pointer */
	setup_regs_i2c
sector_start:
	mov		r5, r9						/* get pointer to sector info */
	ldmia	r5!, {r6, r7}				/* load address offset and length */
	mov		r12, r6						/* save address offset*/
	mov		r11, r7						/* save sector length */
start_read:
	ldr		r4, cmd						/* load I2C address (bits 0 to 7) */
	send_cmd_addr_i2c error1			/* send dummy write with address */
	bl		i2c_stop_cond				/* end of dummy write */
	ldr		r4, cmd						/* load I2C address (bits 0 to 7) */
	prepare_i2c_cmd						/* insert extra address bits */
	adds	r4, r4, #1					/* set R/W bit for read */
	bl		i2c_start_cond				/* generate start condition */
	bl 		shift_out_byte_i2c			/* start read */
	bcs		error1						/* error exit if NACK */
read_loop:
	movs	r4, #0xFF					/* release SDA for entire byte time */
	lsls	r4, r4, #1					/* insert ACK bit to the right of data */
	movs	r5, #1						/* used as NACK bit */
	mov		r7, r12						/* get address offset */
	adds	r7, r7, #1					/* increment address */
	mov		r12, r7						/* save address offset */
	mov		r6, r10						/* get page size mask */
	tst		r6, r7						/* page end ? */
	bne		read_ack_page				/* if not last byte, keep ACK */
	orrs	r4, r4, r5					/* last byte, hence NACK */
read_ack_page:
	mov		r6, r11						/* get count */
	subs	r6, r6, #1					/* decrement count */
	mov		r11, r6						/* save count */
	bne		read_ack_end				/* if not last byte, keep ACK */
	orrs	r4, r4, r5					/* last byte, hence NACK */
read_ack_end:
	bl		shift_in_start_i2c			/* read byte from flash */
	movs	r7, #0xFF					/* fill bits 8-15 */
	lsls	r7, #8						/* with ones */
	orrs	r4, r4, r7					/* copy ones to left of read byte */
	mov		r7, r9						/* pointer to result */
	ldr		r6, [r7, #8]				/* get previous result */
	ands	r6, r6, r4					/* and read byte to result */
	lsls	r4, r4, #8					/* shift result into higher byte */
	orrs	r6, r6, r4					/* or read byte to result */
	str		r6, [r7, #8]				/* save updated result */
	mov		r6, r11						/* get count */
	movs	r6, r6						/* test count */
	beq		sector_end					/* stop if sector completed */
	mov		r6, r10						/* get sector size mask */
	mov		r7, r12						/* get address offset */
	tst		r6, r7						/* page/sector end ? */
	bne		read_loop					/* if not, then next byte */
page_end:
	bl		i2c_stop_cond				/* finish this page read */
	b		start_read					/* then next page */

sector_end:
	bl		i2c_stop_cond				/* finish read */
	mov		r7, r9						/* pointer to result */
	mov		r6, r11						/* get remaining count */
	str		r6, [r7, #4]				/* store remaining (zero) count */
	adds	r7, r7, #12					/* three words */
	mov		r9, r7						/* save update pointer */
	mov		r7, r8						/* get sector count */
	subs	r7, r7, #1					/* decrement count */
	mov		r8, r7						/* save updated count */
	bne		sector_start				/* next sector if not finished */
	b		exit1

exit1:
	b 		exit
error1:
	b		error

	shift_out_byte_i2c

error:
	movs	r0, #0						/* set count to $FFFFFFFF */
	subs	r0, r0, #2					/* as error indication */
	mov		r11, r0
	b		exit

	i2c_start_cond
	i2c_stop_cond

exit:
	bl		i2c_stop_cond				/* finish read */
	mov		r0, r11						/* get count */
	adds	r0, r0, #1					/* correct count */
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
	.equ	buffer, .
