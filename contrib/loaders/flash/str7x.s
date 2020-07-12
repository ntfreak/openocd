/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

	.text
	.arm
	.arch armv4t

	.section .init
/*
	r0 source address
	r1 address
	r2 FLASH_CR0
	r3 dword count
	r4 result
	r5 busy mask
*/

write:
	mov		r4, #0x10000000			/* set DWPG bit */
	str		r4, [r2, #0x0]			/* FLASH_CR0 */
	str		r1, [r2, #0x10]			/* FLASH_AR */
	ldr		r4, [r0], #4			/* load data */
	str		r4, [r2, #0x8]			/* FLASH_DR0 */
	ldr		r4, [r0], #4			/* load data */
	str		r4, [r2, #0xc]			/* FLASH_DR1 */
	mov		r4, #0x90000000			/* set DWPG and WMS bits */
	str		r4, [r2, #0x0]			/* FLASH_CR0 */
busy:
	ldr		r4, [r2, #0x0]			/* FLASH_CR0 */
	tst		r4, r5
	bne		busy
	ldr		r4, [r2, #0x14]			/* FLASH_ER */
	tst		r4, #0xff				/* do we have errors */
	tsteq	r4, #0x100			/* write protection set */
	bne		exit
	add		r1, r1, #0x8			/* next 8 bytes */
	subs	r3, r3, #1				/* decrement dword count */
	bne		write
exit:
	b		exit

	.end
