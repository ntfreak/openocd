/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_tcl.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>

static int hl_layout_open(struct hl_interface_s *adapter)
{
	int res;

	LOG_DEBUG("hl_layout_open");

	adapter->handle = NULL;

	res = adapter->layout->api->open(&adapter->param, &adapter->handle);

	if (res != ERROR_OK) {
		LOG_DEBUG("failed");
		return res;
	}

	return ERROR_OK;
}

static int hl_layout_close(struct hl_interface_s *adapter)
{
	return ERROR_OK;
}

static const struct hl_layout hl_layouts[] = {
#if BUILD_HLADAPTER_STLINK
	{
	 .name = "stlink",
	 .open = hl_layout_open,
	 .close = hl_layout_close,
	 .api = &stlink_usb_layout_api,
	 },
#endif
#if BUILD_HLADAPTER_ICDI
	{
	 .name = "ti-icdi",
	 .open = hl_layout_open,
	 .close = hl_layout_close,
	 .api = &icdi_usb_layout_api,
	},
#endif
#if BUILD_HLADAPTER_NULINK
	{
	 .name = "nulink",
	 .open = hl_layout_open,
	 .close = hl_layout_close,
	 .api = &nulink_usb_layout_api,
	},
#endif
	{.name = NULL, /* END OF TABLE */ },
};

/** */
const struct hl_layout *hl_layout_get_list(void)
{
	return hl_layouts;
}

int hl_layout_init(struct hl_interface_s *adapter)
{
	LOG_DEBUG("hl_layout_init");

	if (adapter->layout == NULL) {
		LOG_ERROR("no layout specified");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}
