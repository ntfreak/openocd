/***************************************************************************
 *   Copyright (C) 2008 Lou Deluxe                                         *
 *   lou.openocd012@fixit.nospammail.net                                   *
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

#ifndef OPENOCD_JTAG_DRIVERS_RLINK_H
#define OPENOCD_JTAG_DRIVERS_RLINK_H

#include "helper/types.h"
struct rlink_speed_table {
	uint8_t const *dtc;
	uint16_t dtc_size;
	uint16_t khz;
	uint8_t prescaler;
};

extern const struct rlink_speed_table rlink_speed_table[];
extern const size_t rlink_speed_table_size;

#endif /* OPENOCD_JTAG_DRIVERS_RLINK_H */
