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
 * r1 - flash sector size
 * r2 - address offset into flash
 * r3 - number of address bytes required
 *
 * Clobbered:
 * r4 - r12 tmp */

#include "cmspi.s"

start:
	setup_regs_i2c						/* initialize registers */
	movs	r4, #0x00					/* initialize crc */
	mvns	r4, r4						/* to 0xFFFFFFFF */
	mov		r8, r4						/* save crc */
	ldr		r4, =0x04C11DB7				/* CRC32 polynomial */
	mov		r9, r4						/* save polynomial */
start_read:
	ldr		r4, cmd						/* load I2C address (bits 0 to 7) */
	send_cmd_addr_i2c exit1				/* send dummy write with address */
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
	mov		r6, r8						/* get current crc */
	mov		r7, r9						/* get polynomial */
	lsls	r4, r4, #24					/* shift new byte into msb */
	eors	r6, r6, r4
	.rept	8							/* unrolled bit loop */
	asrs	r4, r6, #31					/* copy bit 31 into bits 0 to 31 */
	ands	r4, r4, r7					/* r4 neg. -> CRC32XOR, pos. -> 0x0 */
	lsls	r6, r6, #1					/* shift result */
	eors	r6, r6, r4					/* eor by CRC32XOR or 0x0 */
	.endr
	mov		r8, r6						/* save updated crc */
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
	.pool

error1:
exit1:
	b 		exit

	shift_out_byte_i2c
	i2c_start_cond
	i2c_stop_cond

exit:
	bl		i2c_stop_cond				/* finish read */
	mov		r0, r8						/* get computed crc */
	mvns	r0, r0						/* invert to get final result */
	mov		r2, r12						/* restore offset */
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
