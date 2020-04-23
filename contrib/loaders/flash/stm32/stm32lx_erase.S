/***************************************************************************
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

/*
Parameters
	r0 - destination base address
	r1 - pages
	r2 - bytes per page
	r3 - flash base
Variables
	r0 - destination erase pointer
	r1 - destination limit pointer
	r2 - bytes per page
	r3 - flash base
	r4 - temporary r/w
	r5 - erase command
	r6 - 0
*/

/* offsets of registers from flash reg base */
#define STM32_FLASH_PECR		0x04
#define STM32_FLASH_SR_OFFSET	0x18

	.thumb_func
	.global _start
_start:
	movs	r5, #0x41							/* set bits 3 and 9 (program and erase) */
	lsls	r5, 3
	movs	r6, #0								/* zero for the erase command */
	muls	r1, r1, r2							/* calculate limit = base + pages * pagesize */
	add		r1, r1, r0
	b		test_done							/* check loop condition */
arm_erase:
	str		r5, [r3, #STM32_FLASH_PECR]
wait_busy1:
	ldr		r4, [r3, #STM32_FLASH_SR_OFFSET]
	lsls	r4, r4, #31
	bne		wait_busy1
	str		r6, [r0]							/* start the erase */
	add		r0, r0, r2							/* calc next erase address */
wait_busy2:
	ldr		r4, [r3, #STM32_FLASH_SR_OFFSET]
	lsls	r4, r4, #31
	bne		wait_busy2
test_done:
	cmp		r0, r1
	bne		arm_erase

exit:
	bkpt	#0x00
