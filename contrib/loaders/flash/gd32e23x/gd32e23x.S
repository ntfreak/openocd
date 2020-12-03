	.text
	.syntax unified
	.cpu cortex-m23
	.thumb

	/* Params:
	 * r0 - flash base (in), status (out)
	 * r1 - count (word-32bit)
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - tmp
	 */

	.thumb_func
	.global _start
_start:
wait_fifo:
	ldr 	r6, [r2, #0]	/* read wp */
	cmp 	r6, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r5, [r2, #4]	/* read rp */
	cmp 	r5, r6			/* wait until rp != wp */
	beq 	wait_fifo
	ldr 	r6, [r5]	/* "*target_address++ = *rp++" */
	str 	r6, [r4]
	adds	r5, #4
	adds	r4, #4
busy:
	ldr 	r6, [r0, #0x0c]	/* wait until BSY flag is reset */
	movs	r7, #1
	tst 	r6, r7
	bne 	busy
	movs	r7, #0x14		/* check the error bits */
	tst 	r6, r7
	bne 	error
	cmp 	r5, r3			/* wrap rp at end of buffer */
	bcc	no_wrap
	mov	r5, r2
	adds	r5, #8
no_wrap:
	str 	r5, [r2, #4]	/* store rp */
	subs	r1, r1, #1		/* decrement halfword count */
	cmp     r1, #0
	beq     exit		/* loop if not done */
	b	wait_fifo
error:
	movs	r0, #0
	str 	r0, [r2, #4]	/* set rp = 0 on error */
exit:
	mov		r0, r6			/* return status in r0 */
	bkpt	#0
