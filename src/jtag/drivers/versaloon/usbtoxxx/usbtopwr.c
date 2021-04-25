/***************************************************************************
 *   Copyright (C) 2009 - 2010 by Simon Qian <SimonQian@SimonQian.com>     *
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

#include <string.h>

#include "../versaloon_include.h"
#include "../versaloon.h"
#include "../versaloon_internal.h"
#include "usbtoxxx.h"
#include "usbtoxxx_internal.h"

RESULT usbtopwr_init(uint8_t interface_index)
{
	return usbtoxxx_init_command(USB_TO_POWER, interface_index);
}

RESULT usbtopwr_fini(uint8_t interface_index)
{
	return usbtoxxx_fini_command(USB_TO_POWER, interface_index);
}

RESULT usbtopwr_config(uint8_t interface_index)
{
#if PARAM_CHECK
	if (interface_index > 7) {
		LOG_BUG(ERRMSG_INVALID_INTERFACE_NUM, interface_index);
		return ERROR_FAIL;
	}
#endif

	return usbtoxxx_conf_command(USB_TO_POWER, interface_index, NULL, 0);
}

RESULT usbtopwr_output(uint8_t interface_index, uint16_t millivolt)
{
#if PARAM_CHECK
	if (interface_index > 7) {
		LOG_BUG(ERRMSG_INVALID_INTERFACE_NUM, interface_index);
		return ERROR_FAIL;
	}
#endif

	return usbtoxxx_out_command(USB_TO_POWER, interface_index, (uint8_t *)&millivolt,
		2, 0);
}
