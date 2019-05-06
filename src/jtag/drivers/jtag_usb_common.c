/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#include <helper/log.h>

#include "jtag_usb_common.h"

static char *jtag_usb_location;
/*
 * 1 char: bus
 * 2 * 7 chars: max 7 ports
 * 1 char: test for overflow
 * ------
 * 16 chars
 */
#define JTAG_USB_MAX_LOCATION_LENGHT	16

void jtag_usb_set_location(const char *location)
{
	if (strnlen(location, JTAG_USB_MAX_LOCATION_LENGHT) ==
	    JTAG_USB_MAX_LOCATION_LENGHT)
		LOG_WARNING("usb location string is too long!!\n");

	if (jtag_usb_location)
		free(jtag_usb_location);

	jtag_usb_location = strndup(location, JTAG_USB_MAX_LOCATION_LENGHT);
}

const char *jtag_usb_get_location(void)
{
	return jtag_usb_location;
}

bool jtag_usb_location_equal(uint8_t dev_bus, uint8_t *port_path,
			     size_t path_len)
{
	size_t path_step, string_lenght;
	char *ptr, *loc;
	bool equal = false;

	/* strtok need non const char */
	loc = strndup(jtag_usb_get_location(), JTAG_USB_MAX_LOCATION_LENGHT);
	string_lenght = strnlen(loc, JTAG_USB_MAX_LOCATION_LENGHT);

	ptr = strtok(loc, "-");
	if (ptr == NULL) {
		LOG_WARNING("no '-' in usb path\n");
		goto done;
	}

	string_lenght -= 1;
	/* check bus mismatch */
	if (atoi(ptr) != dev_bus)
		goto done;

	path_step = 0;
	while (path_step < path_len) {
		ptr = strtok(NULL, ".");

		/* no more tokens in path */
		if (ptr == NULL)
			break;

		/* path mismatch at some step */
		if (path_step < path_len && atoi(ptr) != port_path[path_step])
			break;

		path_step++;
		string_lenght -= 2;
	};

	/* walked the full path, all elements match */
	if (path_step == path_len && !string_lenght)
		equal = true;
	else
		LOG_WARNING("excluded by device path option: %s\n",
			    jtag_usb_get_location());

done:
	free(loc);
	return equal;
}
