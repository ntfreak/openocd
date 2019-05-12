/***************************************************************************
 *   Copyright (C) 2009 by Alan Carvalho de Assis       		           *
 *   acassis@gmail.com                                                     *
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

void delay()
{
	int i;
	for (i = 0; i < 500000; i++);
}

/* MAIN ARM FUNTION */
int main (void)
{
	int i;
        volatile unsigned char *ledoff = ((volatile unsigned char *)0xD4000008);
        volatile unsigned char *ledon = ((volatile unsigned char *)0xD400000C);

	for (i = 0; i < 10000; i++)
    	{
		*ledon = 0x30;
		delay();
		*ledoff = 0x30;
		delay();
    	} /* FOR */

} /* MAIN */

__gccmain()
{
} /* GCCMAIN */


void exit(int exit_code)
{
  while (1);
} /* EXIT */


atexit()
{
  while (1);
} /* ATEXIT */
