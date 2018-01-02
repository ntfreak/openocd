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
 * r0 - total count (bytes), status (out)
 * r1 - flash sector size
 * r2 - address offset into flash
 * r3 - number of address bytes required
 * r8 - fifo start
 * r9 - fifo end + 1
 *
 * Clobbered:
 * r4 - r7, r10 - r12 tmp */

#include "cmspi.s"

start:
	setup_regs_i2c						/* initialize registers */
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
	ldr		r6, wp						/* get wp */
	adds	r6, r6, #1					/* increment wp */
	cmp		r6, r9						/* wp beyond end? */
	blo		read_cont1					/* if no, then ok */
	mov		r6, r8						/* else wrap around */
read_cont1:
	ldr		r7, rp						/* get rp */
	cmp		r7, #0						/* if rp equal 0 */
	beq		exit1						/* then abort */
	cmp		r6, r7						/* check if fifo full */
	beq		read_loop					/* wait until not full */
	movs	r4, #0xFF					/* release SDA for entire byte time */
	lsls	r4, r4, #1					/* insert ACK bit to the right of data */
	movs	r5, #1						/* used as NACK bit */
	mov		r7, r12						/* get address offset */
	adds	r7, r7, #1					/* increment address */
	mov		r12, r7						/* save address offset */
	mov		r6, r10						/* get sector size mask */
	tst		r6, r7						/* page/sector end ? */
	bne		read_ack_page				/* if not last byte, keep ACK */
	orrs	r4, r4, r5					/* last byte, hence NACK */
read_ack_page:
	mov		r6, r11						/* get count */
	subs	r6, r6, #1					/* decrement count */
	mov		r11, r6						/* save count */
	bpl		read_ack_end				/* if not last byte, keep ACK */
	orrs	r4, r4, r5					/* last byte, hence NACK */
read_ack_end:
	bl		shift_in_start_i2c			/* read byte from flash */
	ldr		r6, wp						/* get wp */
	strb	r4, [r6]					/* write next byte */
	adds	r6, r6, #1					/* increment wp */
	cmp		r6, r9						/* wp beyond end? */
	blo		read_cont2					/* if no, then ok */
	mov		r6, r8						/* else wrap around */
read_cont2:
	adr		r7, wp						/* get address of wp */
	str		r6, [r7]					/* save updated wp */
	mov		r6, r11						/* get count */
	movs	r6, r6						/* test count */
	bmi		exit1						/* stop if no data left */
	mov		r6, r10						/* get sector size mask */
	mov		r7, r12						/* get address offset */
	tst		r6, r7						/* page/sector end ? */
	bne		read_loop					/* if not, then next byte */
page_end:
	bl		i2c_stop_cond				/* finish this read */
	b		start_read					/* then next page/sector */

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
	mov		r2, r12						/* restore offset */
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
	.equ wp, .							/* wp, uint32_t */
	.equ rp, wp + 4						/* rp, uint32_t */
	.equ buffer, rp + 4					/* buffer follows right away */
