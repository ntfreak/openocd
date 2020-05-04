/***************************************************************************
 *   Copyright (C) 2011 by James K. Larson                                 *
 *   jlarson@pacifier.com                                                  *
 *                                                                         *
 *   Copyright (C) 2013 Cosmin Gorgovan                                    *
 *   cosmin [at] linux-geek [dot] org                                      *
 *                                                                         *
 *   Copyright (C) 2014 Pawel Si                                           *
 *   stawel+openocd@gmail.com                                              *
 *                                                                         *
 *   Copyright (C) 2015 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   Copyright (C) 2017 Zale Yu                                            *
 *   CYYU@nuvoton.com                                                      *
 *                                                                         *
 *   Copyright (C) 2020 Linumiz                                            *
 *   saravanan@linumiz.com                                                 *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

	/* Params:
	 * r0 - workarea buffer / result
	 * r1 - target address
	 * r2 - wordcount
	 * Clobbered:
	 * r4 - tmp
	 * r5 - tmp
	 * r6 - tmp
	 * r7 - tmp
	 */

	.thumb_func
	.global _start
.L1:
	/* for(register uint32_t i=0;i<wcount;i++) */
	mov    r4, r0
	mov    r3, #0
.L2:
	sub    r5, r1, r0
	add    r7, r4, r7
	cmp    r3, r2
	beq    .L7
.L4:
	/* NUMICRO_FLASH_ISPADR = faddr; */
	ldr	r6, .NUMICRO_FLASH_ISPADR
	str	r7, [r6]
	/* NUMICRO_FLASH_ISPDAT = *pLW; */
	ldmia	r4!, {r7}
	ldr	r5, .NUMICRO_FLASH_ISPDAT
	str	r7, [r5]
	/* faddr += 4; */
	/* pLW++; */
	/*  Trigger write action  */
	/* NUMICRO_FLASH_ISPTRG = ISPTRG_ISPGO; */
	ldr	r5, .NUMICRO_FLASH_ISPTRG
	mov	r6, #1
	str	r6, [r5]
.L3:
	/* while((NUMICRO_FLASH_ISPTRG & ISPTRG_ISPGO) == ISPTRG_ISPGO){}; */
	ldr	r7, [r5]
	lsl	r7, r7, #31
	bmi	.L3

	add	r3, r3, #1
	b	.L2
.L7:
	/* return (NUMICRO_FLASH_ISPCON & ISPCON_ISPFF); */
	ldr	r3, .NUMICRO_FLASH_ISPCON
	ldr	r0, [r3]
	mov	r1, #64
	and	r0, r1
.L9:
	bkpt    #0

.NUMICRO_FLASH_ISPCON:
	.word	0x4000C000
.NUMICRO_FLASH_ISPADR:
	.word	0x4000C004
.NUMICRO_FLASH_ISPDAT:
	.word	0x4000C008
.NUMICRO_FLASH_ISPTRG:
	.word	0x4000C010
