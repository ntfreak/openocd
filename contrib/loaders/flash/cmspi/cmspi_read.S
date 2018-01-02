/***************************************************************************
 *   Copyright (C) 2017 - 2018 by Andreas Bolsch                           *
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
	setup_regs_spi						/* initialize registers */
start_read:
	set_to_output						/* set IO0-IO3 to output */
	ldr		r4, cmd						/* load READ cmd (in bits 0 to 7) */
	send_cmd_addr spi					/* send cmd and address */
	set_to_input						/* set IO0-IO3 to input */
	prepare_shift_in_spi				/* load shift count MISO into C */
read_loop:
	shift_in_byte_spi					/* read byte from flash */
	ldr		r6, wp						/* get wp */
	strb	r4, [r6]					/* write next byte */
	adds	r6, r6, #1					/* increment wp */
	cmp		r6, r9						/* wp beyond end? */
	blo		read_wait					/* if no, then ok */
	mov		r6, r8						/* else wrap around */
read_wait:
	ldr		r7, rp						/* get rp */
	cmp		r7, #0						/* if rp equal 0 */
	beq		exit						/* then abort */
	cmp		r6, r7						/* check if fifo full */
	beq		read_wait					/* wait until not full */
	adr		r7, wp						/* get address of wp */
	str		r6, [r7]					/* save updated wp */
	mov		r7, r12						/* get address offset */
	adds	r7, r7, #1					/* increment address */
	mov		r12, r7						/* save address offset */
	mov		r6, r11						/* get count */
	subs	r6, r6, #1					/* decrement count */
	mov		r11, r6						/* save count */
	bmi		exit						/* stop if no data left */
	mov		r6, r10						/* get page size mask */
	tst		r6, r7						/* page end ? */
	bne		read_loop					/* if not, then next byte */
page_end:
	bl		deselect					/* finish this sector read */
	b		start_read					/* then next sector */

	deselect
	toggle_dir
	shift_out_byte_spi

exit:
	bl		deselect					/* finish this read cmd */
	mov		r0, r11						/* get count */
	adds	r0, r0, #1					/* correct count */
	mov		r2, r12						/* restore offset */
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
	.equ wp, .							/* wp, uint32_t */
	.equ rp, wp + 4						/* rp, uint32_t */
	.equ buffer, rp + 4					/* buffer follows right away */
