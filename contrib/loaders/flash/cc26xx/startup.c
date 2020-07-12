/******************************************************************************
*
* Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

/******************************************************************************
*
* The entry point for the application startup code.
*
******************************************************************************/
extern int main(void);

/******************************************************************************
*
* Reserve space for the system stack.
*
******************************************************************************/
__attribute__ ((section(".stack")))
static uint32_t stack[100];
const uint32_t stack_pntr = (uint32_t)stack + sizeof(stack);

/******************************************************************************
*
* The following are constructs created by the linker indicating where
* the "bss" and "ebss" segments reside in memory.
*
******************************************************************************/
extern uint32_t _bss;
extern uint32_t _ebss;

/******************************************************************************
*
* This is the entry point that handles setting the stack within the allowed
* workspace, initializing the .bss segment, and then jumping to main.
*
******************************************************************************/
__attribute__ ((section(".entry")))
void entry(void)
{
	/* Workaround for ITT instructions. */
	__asm("		NOP");
	__asm("		NOP");
	__asm("		NOP");
	__asm("		NOP");

	/* Initialize stack pointer */
	__asm("		ldr		sp, =stack_pntr");

	/* Zero fill the bss segment. */
	__asm("		ldr     r0, =_bss\n"
		  "		ldr     r1, =_ebss\n"
		  "		mov     r2, #0\n"
		  "		.thumb_func\n"
		  "	zero_loop:\n"
		  "		cmp		r0, r1\n"
		  "		it		lt\n"
		  "		strlt	r2, [r0], #4\n"
		  "		blt		zero_loop");

	/* Call the application's entry point. */
	main();

	/* If we ever return, enter an infinite loop */
	while (1)
		;
}
