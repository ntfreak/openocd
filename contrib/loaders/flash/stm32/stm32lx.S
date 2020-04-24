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
	r0 - destination address
	r1 - source address
	r2 - count
*/

	.thumb_func
	.global _start
_start:
	// r2 = source + count * 4
	lsls	r2, r2, #2
	adds	r2, r1, r2
	// Go to compare
	b	test_done
write_word:
	// load word from address in r1 and increase r1 by 4
	ldmia r1!, {r3}
	// store word to address in r0 and increase r0 by 4
	stmia r0!, {r3}
test_done:
	// compare r1 and r2
	cmp	r1, r2
	// loop if not equal
	bne	write_word

	// Set breakpoint to exit
	bkpt	#0x00
