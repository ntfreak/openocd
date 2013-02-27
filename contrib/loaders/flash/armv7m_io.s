/***************************************************************************
 *   Copyright (C) 2013 by Henrik Nilsson                                  *
 *   henrik.nilsson@bytequest.se                                           *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

	.text
	.syntax unified
	.arch armv7-m
	.thumb
	.thumb_func

	.align 4

read:
	ldrb	r3, [r1]
	strb	r3, [r0], #1
	subs	r2, r2, #1
	bne		read

done_read:
	bkpt #0

	.align 4

write:
	ldrb	r3, [r1], #1
	strb	r3, [r0]
	subs	r2, r2, #1
	bne		write

done_write:
	bkpt #0

	.end

