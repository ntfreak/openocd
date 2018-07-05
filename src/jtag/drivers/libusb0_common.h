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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_LIBUSB0_COMMON_H
#define OPENOCD_JTAG_DRIVERS_LIBUSB0_COMMON_H

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
#define LIBUSB_REQUEST_TYPE_VENDOR		USB_TYPE_VENDOR
#define LIBUSB_RECIPIENT_DEVICE			USB_RECIP_DEVICE
#define LIBUSB_ENDPOINT_OUT				USB_ENDPOINT_OUT
#define LIBUSB_ENDPOINT_IN				USB_ENDPOINT_IN
#define LIBUSB_TRANSFER_TYPE_BULK		USB_ENDPOINT_TYPE_BULK

static inline int jtag_libusb_claim_interface(jtag_libusb_device_handle *devh,
				       int iface)
{
	return usb_claim_interface(devh, iface);
};

static inline int jtag_libusb_release_interface(jtag_libusb_device_handle *devh,
				       int iface)
{
	return usb_release_interface(devh, iface);
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *serial,
		struct jtag_libusb_device_handle **out);
void jtag_libusb_close(jtag_libusb_device_handle *dev);
int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev,
		uint8_t requestType, uint8_t request, uint16_t wValue,
		uint16_t wIndex, char *bytes,	uint16_t size, unsigned int timeout);
int jtag_libusb_bulk_write(struct jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout, int *transferred);
int jtag_libusb_bulk_read(struct jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout, int *transferred);
int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration);
int jtag_libusb_choose_interface(struct jtag_libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type);
int jtag_libusb_get_pid(struct jtag_libusb_device *dev, uint16_t *pid);

#endif /* OPENOCD_JTAG_DRIVERS_LIBUSB0_COMMON_H */
