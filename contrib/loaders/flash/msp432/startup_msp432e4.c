/******************************************************************************
*
* Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <stdint.h>

/* Entry point for the application. */
extern int main();

/* Reserve space for the system stack. */
extern uint32_t __stack_top;

typedef void(*pFunc)(void);

/* Interrupt handler prototypes */
void default_handler(void);
void reset_handler(void);

/*
 * The vector table.  Note that the proper constructs must be placed on this to
 * ensure that it ends up at physical address 0x0000.0000 or at the start of
 * the program if located at a start address other than 0.
 */
void (* const intr_vectors[])(void) __attribute__((section(".intvecs"))) = {
	(pFunc)&__stack_top, /* The initial stack pointer */
	reset_handler,       /* The reset handler         */
	default_handler,     /* The NMI handler           */
	default_handler,     /* The hard fault handler    */
	default_handler,     /* The MPU fault handler     */
	default_handler,     /* The bus fault handler     */
	default_handler,     /* The usage fault handler   */
	0,                   /* Reserved                  */
	0,                   /* Reserved                  */
	0,                   /* Reserved                  */
	0,                   /* Reserved                  */
	default_handler,     /* SVCall handler            */
	default_handler,     /* Debug monitor handler     */
	0,                   /* Reserved                  */
	default_handler,     /* The PendSV handler        */
	default_handler      /* The SysTick handler       */
};

/*
 * The following are constructs created by the linker, indicating where
 * the "data" and "bss" segments reside in memory.  The initializers for
 * the "data" segment resides immediately following the "text" segment.
 */
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

/*
 * This is the code that gets called when the processor first starts execution
 * following a reset event.  Only the absolutely necessary set is performed,
 * after which the application supplied entry() routine is called.  Any fancy
 * actions (such as making decisions based on the reset cause register, and
 * resetting the bits in that register) are left solely in the hands of the
 * application.
 */
__attribute__((section(".reset"))) __attribute__((naked))
void reset_handler(void)
{
	/* Set stack pointer */
	__asm("    MOVW.W  r0, #0x1700\n"
		  "    MOVT.W  r0, #0x2000\n"
		  "    mov     sp, r0\n");

	/* Zero fill the bss segment. */
	__asm("    ldr     r0, =__bss_start__\n"
		  "    ldr     r1, =__bss_end__\n"
		  "    mov     r2, #0\n"
		  "    .thumb_func\n"
		  "zero_loop:\n"
		  "    cmp     r0, r1\n"
		  "    it      lt\n"
		  "    strlt   r2, [r0], #4\n"
		  "    blt     zero_loop");

	/* Call the application's entry point. */
	main();
}

/*
 * This is the code that gets called when the processor receives an unexpected
 * interrupt.  This simply enters an infinite loop, preserving the system state
 * for examination by a debugger.
 */
void default_handler(void)
{
	/* Enter an infinite loop. */
	while (1)
		;
}
