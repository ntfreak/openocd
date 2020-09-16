.text
	/* Params:
	 * a0 - flash base (in), status (out)
	 * a1 - count (halfword-16bit)
	 * a2 - workarea start
	 * a3 - workarea end
	 * a4 - target address
	 * a5 - tmp
	 * Clobbered:
	 * s5 - rp
	 * s6 - wp, tmp
	 * t7 - tmp
	 */
.global _start
_start:
	j	wait_fifo
_exit:
	ebreak
wait_fifo:
	lw 	s6, 0(a2)		/* read wp */
	beqz 	s6,	exit		/* abort if wp == 0 */
	lw 	s5, 4(a2)		/* read rp */
	sub 	a5,	s5, s6			
	beqz	a5, wait_fifo		/* wait until rp != wp */	
	lhu	s6, 0(s5)		/* copy data from RAM to flash */
	sh	s6, 0(a4)
	addi	s5,	s5, 2		/* "*target_address++ = *rp++" */
	addi	a4,	a4, 2
busy:
	lw 	s7, 0x0c(a0)		/* wait until BSY flag is reset */
	andi	a5, s7,	1
	bnez 	a5, busy
	andi	a5, s7, 0x14			/* check the error bits */
	bnez 	a5, error
	bgtu 	a3, s5,	no_wrap		/* wrap rp at end of buffer */
	mv	s5, a2
	add	s5, s5, 8
no_wrap:
	sw 	s5, 4(a2)			/* store rp */
	addi	a1, a1, -1		/* decrement halfword count */
	beqz  a1, exit			/* loop if not done */
	j	wait_fifo
error:
	mv	a0, x0
	sw 	a0, 4(a2)		/* set rp = 0 on error */
exit:
	mv	a0, s7			/* return status in a0 */
	j	_exit
