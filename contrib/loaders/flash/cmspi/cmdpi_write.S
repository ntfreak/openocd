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
 * r1 - flash page size
 * r2 - address offset into flash
 * r3 - number of address bytes required
 * r8 - fifo start
 * r9 - fifo end + 1
 *
 * Clobbered:
 * r4 - r7, r10 - r12 tmp */

#include "cmspi.s"

start:
	setup_regs_dpi						/* initialize registers */
wip_loop:
	set_to_output						/* set IO0-IO3 to output */
	movs	r4, #SPIFLASH_READ_STATUS	/* read status reg cmd */
	bl		shift_out_byte_dpi			/* send command */
	set_to_input						/* set IO0-IO3 to input */
	bl		shift_in_byte_dpi			/* read status byte */
	bl		deselect					/* end cmd */
	lsrs	r4, r4, #(SPIFLASH_BSY+1)	/* if flash busy, */
	bcs		wip_loop					/* then poll again */
	mov		r7, r11						/* get residual count */
	tst		r7, r7						/* test residual count */
	bpl		start_write					/* if negative, then finished */
	b		exit
start_write:
	set_to_output						/* set IO0-IO3 to output */
	movs	r4, #SPIFLASH_WRITE_ENABLE	/* write enable cmd */
	bl		shift_out_byte_dpi			/* send cmd */
	bl		deselect					/* end cmd */
	movs	r4, #SPIFLASH_READ_STATUS	/* read status reg cmd */
	bl		shift_out_byte_dpi			/* send command */
	set_to_input						/* set IO0-IO3 to input */
	bl		shift_in_byte_dpi			/* read status byte */
	bl		deselect					/* end cmd */
	lsrs	r4, r4, #(SPIFLASH_WE+1)	/* if write enabled, */
	bcs		write_enabled				/* then continue  */
	b		error						/* else error exit */
write_enabled:
	set_to_output						/* set IO0-IO3 to output */
	ldr		r4, cmd						/* load PPROG cmd (in bits 0 to 7) */
	send_cmd_addr dpi					/* send cmd and address */
write_loop:
	ldr		r7, wp						/* get wp */
	cmp		r7, #0						/* if wp equal 0 */
	beq		exit1						/* then abort */
	ldr		r6, rp						/* get rp */
	cmp		r6, r7						/* check if fifo empty */
	beq		write_loop					/* wait till not empty */
	ldrb	r4, [r6, #0]				/* read next byte */
	bl		shift_out_next_dpi			/* send byte to flash */
	ldr		r6, rp						/* get rp */
	adds	r6, r6, #1					/* increment internal rp */
	cmp		r6, r9						/* internal rp beyond end? */
	blo		write_loop1					/* if no, then ok */
	mov		r6, r8						/* else wrap around */
write_loop1:
	adr		r7, rp						/* get address of rp */
	str		r6, [r7]					/* save updated rp */
	mov		r7, r12						/* get address offset */
	adds	r7, r7, #1					/* increment address */
	mov		r12, r7						/* save address offset */
	mov		r6, r11						/* get count */
	subs	r6, r6, #1					/* decrement count */
	mov		r11, r6						/* save updated count */
	bmi		page_end					/* stop if no data left */
	mov		r6, r10						/* get page size mask */
	tst		r6, r7						/* page end ? */
	bne		write_loop					/* if not, then next byte */
page_end:
	set_to_input						/* set IO0-IO3 to input */
	bl		deselect					/* finish this page write */
	b		wip_loop					/* then next page */

	deselect
	toggle_dir
	shift_in_byte_dpi
exit1:
	b		exit
	shift_out_byte_dpi

error:
	movs	r0, #0						/* set count to $FFFFFFFF */
	subs	r0, r0, #2					/* as error indication */
	mov		r11, r0
exit:
	mov		r0, r11						/* get count */
	adds	r0, r0, #1					/* correct count */
	mov		r1, r12						/* get address offset */
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
	.equ wp, .							/* wp, uint32_t */
	.equ rp, wp + 4						/* rp, uint32_t */
	.equ buffer, rp + 4					/* buffer follows right away */
