/***************************************************************************
 *   Copyright (C) 2016 by Maxim Integrated                                *
 *   Kevin Gillespie <kevin.gillespie@maximintegrated.com                  *
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
.cpu cortex-m3
.thumb
.thumb_func

/*
 * Params :
 * r0 = workarea start
 * r1 = workarea end
 * r2 = target address
 * r3 = count (number of writes)
 * r4 = pFLASH_CTRL_BASE
 * r5 = 128-bit write

 * Clobbered:
 * r6 = FLASHWRITECMD
 * r7 - rp
 * r8 - wp, tmp
 */

write:

	/* write in 32-bit units by default */
	ldr     r6, [r4, #0x08] 	/* FLSH_CN */
	orr     r6, r6, #0x10   	/* Width 32 bits */
	str     r6, [r4, #0x08] 	/* FLSH_CN */

	tst     r5, #1          	/* Check 32-bit write options */
	beq     wait_fifo

	/* Enable 128-bit write */
	ldr     r6, [r4, #0x08] 	/* FLSH_CN */
	bic     r6, r6, #0x10   	/* Width 128 bits */
	str     r6, [r4, #0x08] 	/* FLSH_CN */

wait_fifo:
	ldr 	r8, [r0, #0x00]		/* read wp */
	cmp 	r8, #0x00			/* abort if wp == 0 */
	beq 	exit
	ldr 	r7, [r0, #0x04]		/* read rp */
	cmp 	r7, r8				/* wait until rp != wp */
	beq 	wait_fifo

mainloop:

	str		r2, [r4, #0x00]	   	/* FLSH_ADDR - write address */
	add		r2, r2, #0x04      	/* increment target address */
	tst     r5, #1              /* Check 32-bit write options */
	beq     inc32
	add     r2, r2, #0x0C       /* additional 12 if 128-bit write*/

inc32:
	ldr		r8, [r7], #0x04
	str		r8, [r4, #0x30]  	/* FLSH_DATA0 - write data */
	cmp     r7, r1              /* wrap rp at end of buffer */
	it      cs
	addcs   r7, r0, #0x08       /* skip loader args */
	str     r7, [r0, #0x04]     /* store rp */

	tst     r5, #1              /* Check 32-bit write options */
	beq     write32

wait_fifo0:
	ldr     r8, [r0, #0x00]     /* read wp */
	cmp     r8, #0x00           /* abort if wp == 0 */
	beq     exit
	ldr     r7, [r0, #0x04]     /* read rp */
	cmp     r7, r8              /* wait until rp != wp */
	beq     wait_fifo0

	ldr     r8, [r7], #0x04
	str     r8, [r4,  #0x34]    /* FLSH_DATA1 - write data */
	cmp     r7, r1              /* wrap rp at end of buffer */
	it      cs
	addcs   r7, r0, #0x08   	/* skip loader args */
	str     r7, [r0, #0x04] 	/* store rp */

wait_fifo1:
	ldr     r8, [r0, #0x00] 	/* read wp */
	cmp     r8, #0x00       	/* abort if wp == 0 */
	beq     exit
	ldr     r7, [r0, #0x04] 	/* read rp */
	cmp     r7, r8          	/* wait until rp != wp */
	beq     wait_fifo1

	ldr     r8, [r7], #0x04
	str     r8, [r4,  #0x38] 	/* FLSH_DATA2 - write data */
	cmp     r7, r1          	/* wrap rp at end of buffer */
	it      cs
	addcs   r7, r0, #0x08   	/* skip loader args */
	str     r7, [r0, #0x04] 	/* store rp */

wait_fifo2:
	ldr     r8, [r0, #0x00] 	/* read wp */
	cmp     r8, #0x00       	/* abort if wp == 0 */
	beq     exit
	ldr     r7, [r0, #0x04] 	/* read rp */
	cmp     r7, r8          	/* wait until rp != wp */
	beq     wait_fifo2

	ldr     r8, [r7], #0x04
	str     r8, [r4,  #0x3C] 	/* FLSH_DATA3 - write data */
	cmp     r7, r1          	/* wrap rp at end of buffer */
	it      cs
	addcs   r7, r0, #0x08   	/* skip loader args */
	str     r7, [r0, #0x04]	 	/* store rp */

write32:
	ldr		r6, [r4, #0x08]		/* FLSH_CN */
	orr		r6, r6, #0x01   	/* WE */
	str		r6, [r4, #0x08]		/* FLSH_CN - enable write */

busy:
	ldr		r8, [r4, #0x08]		/* FLSH_CN */
	tst		r8, #0x07
	bne		busy

	subs	r3, r3, #0x01		/* decrement write count */
	cbz 	r3, exit			/* loop if not done */
	b		wait_fifo

exit:
	/* restore flash settings */
	ldr     r6, [r4, #0x08] 	/* FLSH_CN */
	orr     r6, r6, #0x10   	/* Width 32 bits */
	str     r6, [r4, #0x08] 	/* FLSH_CN */
	bkpt
