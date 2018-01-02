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
	setup_regs_dpi
sector_start:
	mov		r5, r9						/* get pointer to sector info */
	ldmia	r5!, {r6, r7}				/* load address offset and length */
	mov		r12, r6						/* save address offset*/
	mov		r11, r7						/* save sector length */
start_read:
	set_to_output						/* set IO0-IO3 to output */
	ldr		r4, cmd						/* load READ cmd (in bits 0 to 7) */
	send_cmd_addr dpi					/* send cmd and address */
	set_to_input						/* set IO0-IO3 to input */
	dummy_clocks						/* insert required dummy clocks */
read_loop:
	bl		shift_in_byte_dpi			/* read byte from flash */
	movs	r7, #0xFF					/* fill bits 8-15 */
	lsls	r7, #8						/* with ones */
	orrs	r4, r4, r7					/* copy ones to left of read byte */
	mov		r7, r9						/* pointer to result */
	ldr		r6, [r7, #8]				/* get previous result */
	ands	r6, r6, r4					/* and read byte to result */
	lsls	r4, r4, #8					/* shift result into higher byte */
	orrs	r6, r6, r4					/* or read byte to result */
	str		r6, [r7, #8]				/* save updated result */
	mov		r7, r12						/* get address offset */
	adds	r7, r7, #1					/* increment address */
	mov		r12, r7						/* save address offset */
	mov		r6, r11						/* get count */
	subs	r6, r6, #1					/* decrement count */
	mov		r11, r6						/* save count */
	beq		sector_end					/* stop if sector completed */
	mov		r6, r10						/* get page size mask */
	tst		r6, r7						/* page end ? */
	bne		read_loop					/* if not, then next byte */
page_end:
	bl		deselect					/* finish this sector read */
	b		start_read					/* then next page */

sector_end:
	bl		deselect					/* finish this sector read */
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

	deselect
	toggle_dir
	shift_in_byte_dpi
exit1:
	b exit
	shift_out_byte_dpi

exit:
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
	.equ	buffer, .
