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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "log.h"
#include "libusb0_common.h"

static bool jtag_libusb_match(struct jtag_libusb_device *dev,
		const uint16_t vids[], const uint16_t pids[])
{
	for (unsigned i = 0; vids[i]; i++) {
		if (dev->descriptor.idVendor == vids[i] &&
			dev->descriptor.idProduct == pids[i]) {
			return true;
		}
	}
	return false;
}

/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(usb_dev_handle *device, uint8_t str_index,
									const char *string)
{
	int retval;
	bool matched;
	char desc_string[256+1]; /* Max size of string descriptor */

	if (str_index == 0)
		return false;

	retval = usb_get_string_simple(device, str_index,
			desc_string, sizeof(desc_string)-1);
	if (retval < 0) {
		LOG_ERROR("usb_get_string_simple() failed with %d", retval);
		return false;
	}

	/* Null terminate descriptor string in case it needs to be logged. */
	desc_string[sizeof(desc_string)-1] = '\0';

	matched = strncmp(string, desc_string, sizeof(desc_string)) == 0;
	if (!matched)
		LOG_DEBUG("Device serial number '%s' doesn't match requested serial '%s'",
			desc_string, string);
	return matched;
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *serial,
		struct jtag_libusb_device_handle **out)
{
	int retval = ERROR_FAIL;
	bool serial_mismatch = false;
	struct jtag_libusb_device_handle *libusb_handle;
	usb_init();

	usb_find_busses();
	usb_find_devices();

	struct usb_bus *busses = usb_get_busses();
	for (struct usb_bus *bus = busses; bus; bus = bus->next) {
		for (struct usb_device *dev = bus->devices;
				dev; dev = dev->next) {
			if (!jtag_libusb_match(dev, vids, pids))
				continue;

			libusb_handle = usb_open(dev);
			if (NULL == libusb_handle) {
				LOG_ERROR("usb_open() failed with %s", usb_strerror());
				continue;
			}

			/* Device must be open to use libusb_get_string_descriptor_ascii. */
			if (serial != NULL &&
					!string_descriptor_equal(libusb_handle, dev->descriptor.iSerialNumber, serial)) {
				serial_mismatch = true;
				usb_close(libusb_handle);
				continue;
			}
			*out = libusb_handle;
			retval = ERROR_OK;
			serial_mismatch = false;
			break;
		}
	}

	if (serial_mismatch)
		LOG_INFO("No device matches the serial string");

	return retval;
}

void jtag_libusb_close(jtag_libusb_device_handle *dev)
{
	/* Close device */
	usb_close(dev);
}

int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev, uint8_t requestType,
		uint8_t request, uint16_t wValue, uint16_t wIndex, char *bytes,
		uint16_t size, unsigned int timeout)
{
	int transferred = 0;

	transferred = usb_control_msg(dev, requestType, request, wValue, wIndex,
				bytes, size, timeout);

	if (transferred < 0)
		transferred = 0;

	return transferred;
}

int jtag_libusb_bulk_write(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout, int *transferred)
{
	 *transferred = usb_bulk_write(dev, ep, bytes, size, timeout);
	 return ERROR_OK;
}

int jtag_libusb_bulk_read(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout, int *transferred)
{
	*transferred = usb_bulk_read(dev, ep, bytes, size, timeout);
	return ERROR_OK;
}

int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);

	return usb_set_configuration(devh,
			udev->config[configuration].bConfigurationValue);
}

int jtag_libusb_choose_interface(struct jtag_libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	struct usb_interface *iface = udev->config->interface;
	struct usb_interface_descriptor *desc = iface->altsetting;

	*usb_read_ep = *usb_write_ep = 0;

	for (int i = 0; i < desc->bNumEndpoints; i++) {
		if ((bclass > 0 && desc->bInterfaceClass != bclass) ||
		    (subclass > 0 && desc->bInterfaceSubClass != subclass) ||
		    (protocol > 0 && desc->bInterfaceProtocol != protocol) ||
		    (trans_type > 0 && (desc->endpoint[i].bmAttributes & 0x3) != trans_type))
			continue;

		uint8_t epnum = desc->endpoint[i].bEndpointAddress;
		bool is_input = epnum & 0x80;
		LOG_DEBUG("usb ep %s %02x", is_input ? "in" : "out", epnum);
		if (is_input)
			*usb_read_ep = epnum;
		else
			*usb_write_ep = epnum;

		if (*usb_read_ep && *usb_write_ep) {
			LOG_DEBUG("Claiming interface %d", (int)desc->bInterfaceNumber);
			usb_claim_interface(devh, (int)desc->bInterfaceNumber);
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

int jtag_libusb_get_pid(struct jtag_libusb_device *dev, uint16_t *pid)
{
	if (!dev)
		return ERROR_FAIL;

	*pid = dev->descriptor.idProduct;
	return ERROR_OK;
}
