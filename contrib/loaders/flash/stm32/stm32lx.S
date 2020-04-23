/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2011 Clement Burin des Roziers                          *
 *   clement.burin-des-roziers@hikob.com                                   *
 *                                                                         *
 *   Copyright (C) 2017 Armin van der Togt                                 *
 *   armin@otheruse.nl                                                     *
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
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

/*
Parameters
	r0 - destination address
	r1 - source address
	r2 - half pages
	r3 - bytes per half page
	r4 - flash base
Variables
	r0 - destination write pointer
	r1 - source read pointer
	r2 - source limit address
	r3 - bytes per half page
	r4 - flash base
	r5 - pages left in current half page
	r6 - temporary r/w
*/

/* offsets of registers from flash reg base */
#define STM32_FLASH_SR_OFFSET 0x18

	.thumb_func
	.global _start
_start:
	// r2 = source + half pages * bytes per half page
	muls r2, r2, r3
	add r2, r1, r2
	// Go to compare
	b test_done
write_half_page:
	// initialize pages left in current half page
	mov r5, r3
write_word:
	// load word from address in r1 and increase r1 by 4
	ldmia r1!, {r6}
	// store word to address in r0 and increase r0 by 4
	stmia r0!, {r6}
	// check for end of half page
	subs r5, r5, #4
	bne write_word
wait_busy:
	// read status register into r6, loop while bottom bit is set
	ldr r6, [r4, #STM32_FLASH_SR_OFFSET]
	lsls r6, r6, #31
	bne wait_busy
test_done:
	// compare r1 and r2, loop if not equal
	cmp	r1, r2
	bne	write_half_page

	// Set breakpoint to exit
	bkpt #0x00
