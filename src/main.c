/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "openocd.h"
#include "helper/system.h"

/* This is the main entry for developer PC hosted OpenOCD.
 *
 * OpenOCD can also be used as a library that is linked with
 * another application(not mainstream yet, but possible), e.g.
 * w/as an embedded application.
 *
 * Those applications will have their own main() implementation
 * and use bits and pieces from openocd.c. */

int main(int argc, char *argv[])
{
	/* disable buffering otherwise piping to logs causes problems work */
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	return openocd_main(argc, argv);
}
