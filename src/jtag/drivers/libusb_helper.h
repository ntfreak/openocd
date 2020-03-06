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

#ifndef OPENOCD_JTAG_DRIVERS_LIBUSB_HELPER_H
#define OPENOCD_JTAG_DRIVERS_LIBUSB_HELPER_H

#include <libusb.h>

/* this callback should return a non NULL value only when the serial could not
 * be retrieved by the standard 'libusb_get_string_descriptor_ascii' */
typedef char * (*adapter_get_alternate_serial_fn)(libusb_device_handle *device,
		struct libusb_device_descriptor *dev_desc);

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *serial,
		struct libusb_device_handle **out,
		adapter_get_alternate_serial_fn adapter_get_alternate_serial);
void jtag_libusb_close(struct libusb_device_handle *dev);
int jtag_libusb_control_transfer(struct libusb_device_handle *dev,
		uint8_t requestType, uint8_t request, uint16_t wValue,
		uint16_t wIndex, char *bytes,	uint16_t size, unsigned int timeout);
int jtag_libusb_bulk_write(struct libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout, int *transferred);
int jtag_libusb_bulk_read(struct libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout, int *transferred);
int jtag_libusb_set_configuration(struct libusb_device_handle *devh,
		int configuration);
/**
 * Find the first interface optionally matching class, subclass and
 * protocol and claim it.
 * @param devh _libusb_ device handle.
 * @param usb_read_ep A pointer to a variable where the _IN_ endpoint
 *	number will be stored.
 * @param usb_write_ep A pointer to a variable where the _OUT_ endpoint
 *	number will be stored.
 * @param bclass `bInterfaceClass` to match, or -1 to ignore this field.
 * @param subclass `bInterfaceSubClass` to match, or -1 to ignore this field.
 * @param protocol `bInterfaceProtocol` to match, or -1 to ignore this field.
 * @param trans_type `bmAttributes Bits 0..1 Transfer type` to match, or -1 to ignore this field.
 * @returns Returns ERROR_OK on success, ERROR_FAIL otherwise.
 */
int jtag_libusb_choose_interface(struct libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type);
int jtag_libusb_get_pid(struct libusb_device *dev, uint16_t *pid);

#endif /* OPENOCD_JTAG_DRIVERS_LIBUSB_HELPER_H */
