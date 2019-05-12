/* Sample initialization file */
	
	.extern	main
	.extern	exit
	
/* .text is used instead of .section .text so it works with arm-aout too.  */
	.text
	.code 32
	.align 	0

	.global	_mainCRTStartup
	.global	_start
	.global	start
start:
_start:
_mainCRTStartup:

/* Start by setting up a stack */
	/*  Set up the stack pointer to end of bss */
	ldr	r3, .LC2
	mov 	sp, r3

	sub	sl, sp, #512	/* Still assumes 512 bytes below sl */

	mov 	a2, #0		/* Second arg: fill value */
	mov	fp, a2		/* Null frame pointer */
	mov	r7, a2		/* Null frame pointer for Thumb */
	
	ldr	a1, .LC1	/* First arg: start of memory block */
	ldr	a3, .LC2	/* Second arg: end of memory block */
	sub	a3, a3, a1	/* Third arg: length of block */
	
	mov	r0, #0		/*  no arguments  */
	mov	r1, #0		/*  no argv either */

	bl	main
	bl	exit		/* Should not return */

	/* For Thumb, constants must be after the code since only 
	positive offsets are supported for PC relative addresses. */
	
	.align 0
.LC1:
	.word	__bss_start__
.LC2:
	.word	__bss_end__
