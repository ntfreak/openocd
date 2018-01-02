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
 *
 * Clobbered:
 * r0 - r10 */

#include "../../../../src/flash/nor/cmspi.h"
#include "cmspi.s"

start:
	setup_regs_i2c						/* initialize registers */
	movs	r4, #I2C_FIRST - 1			/* first legal address (with R/W = 0) - 1 */
	adr		r2, scan_info				/* pointer to output buffer */
	mov		r9, r2						/* save pointer */
	movs	r3, #1						/* stop mark for first block (32 addresses) */
	mov		r10, r3						/* save current value */
addr_loop:
	adds	r4, r4, #1					/* increment address */
	mov		r8, r4						/* save current address */
	bl		i2c_start_cond				/* send I2C start condition */
	bl		shift_out_byte_i2c			/* send I2C address byte, ACK into C */
	mov		r3, r10						/* get current result */
	adcs	r3, r3, r3					/* shift left and insert C into bit 0 */
	mov		r10, r3						/* save new result */
	bcc		stop_cond					/* address block not completed ? */
	mov		r2, r9						/* else get pointer */
	stmia	r2!, {r3}					/* save completed result */
	mov		r9, r2						/* save pointer */
	movs	r3, #1						/* stop mark for next block */
	mov		r10, r3						/* save current value */
stop_cond:
	bl		i2c_stop_cond				/* send I2C stop condition */
	movs	r4, #9						/* resync I2C bus by 9 times */
resync_loop:
	bl		i2c_start_cond				/* issuing start condition */
	bl		i2c_stop_cond				/* followed by stop condition */
	subs	r4, r4, #1					/* all iterations done? */
	bne		resync_loop					/* if not, repeat */
	ldr		r2, cmd						/* get dummy (bits 8 to 15) */
	lsrs	r2, r2, #((dummy-cmd)<<3)	/* shift into lowest byte */
	uxtb	r2, r2						/* clear upper bytes */
delay1:
	movs	r3, #100					/* very long delay  */
delay2:
	subs	r3, r3, #1					/* between successive tests */
	bne		delay2						/* i. e. long bus free time */
	subs	r2, r2, #1					/* decrement loop counter */
	bne		delay1						/* delay passed? */
	mov		r4, r8						/* fetch current address */
	cmp 	r4, #I2C_LAST				/* last addr already done ? */
	bne		addr_loop					/* if not, continue with next addr */
	mov		r3, r10						/* get partial result */
	cmp		r3, #1						/* only stop mark? */
	beq		end_full					/* then all done */
end_loop:
	lsls	r3, r3, #1					/* shift left until */
	bcc		end_loop					/* stop mark reached */
	mov		r2, r9						/* fetch pointer */
	str		r3, [r2]					/* save final result */
end_full:
	b		exit

	shift_out_byte_i2c
	i2c_start_cond
	i2c_stop_cond

exit:
	.align	2							/* align to word */
	bkpt	#0							/* exit point */

	param_block
	.equ	scan_info, .
