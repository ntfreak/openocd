/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 *                                                                         *
 *   Copyright (C) 2011 by Mauro Gamba <maurillo71@gmail.com>              *
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

#ifndef JTAG_LIBUSB_COMMON_H
#define JTAG_LIBUSB_COMMON_H

#include <usb.h>

#define jtag_libusb_device			usb_device
#define jtag_libusb_device_handle		usb_dev_handle
#define jtag_libusb_device_descriptor		usb_device_descriptor
#define jtag_libusb_interface			usb_interface
#define jtag_libusb_interface_descriptor	usb_interface_descriptor
#define jtag_libusb_endpoint_descriptor		usb_endpoint_descriptor
#define jtag_libusb_config_descriptor		usb_config_descriptor

#define jtag_libusb_reset_device(dev)		usb_reset(dev)
#define jtag_libusb_get_device(devh)		usb_device(devh)

/* make some defines compatible to libusb1 */
#define USB_TYPE_VENDOR		LIBUSB_REQUEST_TYPE_VENDOR 
#define USB_RECIP_DEVICE	LIBUSB_RECIPIENT_DEVICE
#define USB_ENDPOINT_OUT	LIBUSB_ENDPOINT_OUT
#define USB_ENDPOINT_IN		LIBUSB_ENDPOINT_IN

static inline int jtag_libusb_claim_interface(jtag_libusb_device_handle *devh,
				       int iface)
{
	return usb_claim_interface(devh, iface);
};

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		struct jtag_libusb_device_handle **out);
void jtag_libusb_close(jtag_libusb_device_handle *dev);
int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev,
		uint8_t requestType, uint8_t request, uint16_t wValue,
		uint16_t wIndex, char *bytes,	uint16_t size, unsigned int timeout);
int jtag_libusb_bulk_write(struct jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout);
int jtag_libusb_bulk_read(struct jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout);
int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration);
int jtag_libusb_get_endpoints(struct jtag_libusb_device *udev,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep);

#endif /* JTAG_USB_COMMON_H */
