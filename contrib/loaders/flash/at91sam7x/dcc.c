/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
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
#include "dcc.h"


/* debug channel read (debugger->MCU) */
uint32 dcc_rd(void)
{
	volatile uint32 dcc_reg;

	do {
		asm volatile ("mrc p14, 0, %0, C0, C0" : "=r" (dcc_reg) :);
	} while ((dcc_reg&1) == 0);

	asm volatile ("mrc p14, 0, %0, C1, C0" : "=r" (dcc_reg) :);
	return dcc_reg;
}


/* debug channel write (MCU->debugger) */
int dcc_wr(uint32 data)
{
	volatile uint32 dcc_reg;

	do {
		asm volatile ("mrc p14, 0, %0, C0, C0" : "=r" (dcc_reg) :);
		/* operation controlled by master, cancel operation
			 upon reception of data for immediate response */
		if (dcc_reg&1) return -1;
	} while (dcc_reg&2);

	asm volatile ("mcr p14, 0, %0, C1, C0" : : "r" (data));
	return 0;
}
