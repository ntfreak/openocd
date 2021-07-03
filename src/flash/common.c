/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
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

#include "common.h"
#include <helper/log.h>

unsigned get_flash_name_index(const char *name)
{
	const char *name_index = strrchr(name, '.');
	if (!name_index)
		return 0;
	if (name_index[1] < '0' || name_index[1] > '9')
		return ~0U;
	unsigned requested;
	int retval = parse_uint(name_index + 1, &requested);
	/* detect parsing error by forcing past end of bank list */
	return (retval == ERROR_OK) ? requested : ~0U;
}

bool flash_driver_name_matches(const char *name, const char *expected)
{
	unsigned blen = strlen(name);
	/* only match up to the length of the driver name... */
	if (strncmp(name, expected, blen) != 0)
		return false;

	/* ...then check that name terminates at this spot. */
	return expected[blen] == '.' || expected[blen] == '\0';
}
