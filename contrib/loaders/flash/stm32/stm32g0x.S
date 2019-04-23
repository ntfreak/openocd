/***************************************************************************
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
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
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0plus
	.thumb

	/* Params:
	 * r0 - flash base (in), status (out)
	 * r1 - count (double word - 64bit)
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - tmp
	 */

#define STM32_FLASH_SR_OFFSET 0x10 /* offset of SR register from flash reg base */

	.thumb_func
	.global _start
_start:
wait_fifo:
	ldr 	r6, [r2, #0]	/* load R6 with 32-bit word from workarea start address */
	cmp 	r6, #0			  /* is word zero? */
	beq 	exit          /* if so, exit */
	ldr 	r5, [r2, #4]	/* load R5 with next word from workarea */
	cmp 	r5, r6			  /* are the two words the same? */
	beq 	wait_fifo     /* if so, goto wait_fifo */
	ldr	  r6, [r5]	    /* load R6 with word from R5 */
	str	  r6, [r4]      /* store word in R6 to target address */
	ldr	  r6, [r5, #4]  /* load R6 with word from R5 */
	str	  r6, [r4, #4]  /* store word in R6 to target address */
	adds	r5, #8        /* add 8 to R5 */
	adds	r4, #8        /* add 8 to R4 */
busy:
	ldr 	r6, [r0, #STM32_FLASH_SR_OFFSET]	/* wait until BSY flag is reset */
	movs	r7, #0x10000  /* set R7 = 0x10000 */
	tst 	r6, r7        /* AND SR reg with 0x10000 */
	bne 	busy          /* if bit is set, go to busy and check again */
	movs	r7, #0xFA	    /* set R7 = 0xFA */ /* check the error bits */
	tst 	r6, r7        /* AND SR reg with 0x3FA */
	bne 	error         /* if error bits set, goto error */
	cmp 	r5, r3			  /* is R5 at end of workarea? */ /* wrap rp at end of buffer */
	bcc	  no_wrap       /* if R5 < R3 */
	mov	  r5, r2        /* set R5 to start of ring buffer */
	adds	r5, #8        /* skip over rp and wp pointers */
no_wrap:
	str 	r5, [r2, #4]	/* store rp */
	subs	r1, r1, #1		/* decrement doubleword count */
	cmp   r1, #0
	beq   exit		/* loop if not done */
	b	    wait_fifo
error:
	movs	r0, #0
	str 	r0, [r2, #4]	/* set rp = 0 on error */
exit:
	mov		r0, r6			/* return status in r0 */
	bkpt	#0
