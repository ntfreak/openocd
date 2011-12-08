/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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
#ifndef MINIDRIVER_IMP_H
#define MINIDRIVER_IMP_H

#include <jtag/jtag_minidriver.h>

static inline void interface_jtag_add_scan_check_alloc(struct scan_field *field)
{
	/* We're executing this synchronously, so try to use local storage. */

	/* FIX! we must have a way to tell if we're synchronous or not so as
	 * to tell if we can use intmp or not. intmp is faster, but broken
	 * for the asynchronous case. Minidrives *can* be asynchronous, such as
	 * JTAG over TCP/IP
	 */
	if (field->num_bits > 0)
	{
		unsigned num_bytes = DIV_ROUND_UP(field->num_bits, 8);
		field->in_value = (uint8_t *)malloc(num_bytes);
		field->allocated = 1;
	}
	else
		field->in_value = field->intmp;
}

static inline void interface_jtag_alloc_in_value32(struct scan_field *field)
{
	interface_jtag_add_scan_check_alloc(field);
}

static inline void jtag_add_dr_out(struct jtag_tap* tap,
		int num_fields, const int* num_bits, const uint32_t* value,
		tap_state_t end_state)
{
	cmd_queue_cur_state = end_state;

	interface_jtag_add_dr_out(tap,
			num_fields, num_bits, value,
			end_state);
}

#define jtag_add_callback(callback, in) interface_jtag_add_callback(callback, in)

#define jtag_add_callback4(callback, in, data1, data2, data3) interface_jtag_add_callback4(callback, in, data1, data2, data3)



#endif // MINIDRIVER_IMP_H
