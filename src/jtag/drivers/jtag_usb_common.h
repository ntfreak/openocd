/*
 * SPDX-License-Identifier: GPL-2.0+
 * Copyright (c) 2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#ifndef OPENOCD_JTAG_USB_COMMON_H
#define OPENOCD_JTAG_USB_COMMON_H

#include <helper/replacements.h>
#include <helper/types.h>

void jtag_usb_set_location(const char *location);
const char *jtag_usb_get_location(void);
bool jtag_usb_location_equal(uint8_t dev_bus, uint8_t *port_path,
			     size_t path_len);

#endif /* OPENOCD_JTAG_USB_COMMON_H */
