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
        volatile unsigned char *led = ((volatile unsigned char *)0xB6020000);

	while (1)
    	{
		*led = 0xFF;
		delay();
		*led = 0x00;
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
