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
#include <jtag/drivers/jtag_usb_common.h>
#include "libusb_helper.h"
#include "log.h"

/*
 * comment from libusb:
 * As per the USB 3.0 specs, the current maximum limit for the depth is 7.
 */
#define MAX_USB_PORTS	7

static struct libusb_context *jtag_libusb_context; /**< Libusb context **/
static libusb_device **devs; /**< The usb device list **/

static int jtag_libusb_error(int err)
{
	switch (err) {
	case LIBUSB_SUCCESS:
		return ERROR_OK;
	case LIBUSB_ERROR_TIMEOUT:
		return ERROR_TIMEOUT_REACHED;
	case LIBUSB_ERROR_IO:
	case LIBUSB_ERROR_INVALID_PARAM:
	case LIBUSB_ERROR_ACCESS:
	case LIBUSB_ERROR_NO_DEVICE:
	case LIBUSB_ERROR_NOT_FOUND:
	case LIBUSB_ERROR_BUSY:
	case LIBUSB_ERROR_OVERFLOW:
	case LIBUSB_ERROR_PIPE:
	case LIBUSB_ERROR_INTERRUPTED:
	case LIBUSB_ERROR_NO_MEM:
	case LIBUSB_ERROR_NOT_SUPPORTED:
	case LIBUSB_ERROR_OTHER:
		return ERROR_FAIL;
	default:
		return ERROR_FAIL;
	}
}

static bool jtag_libusb_match_ids(struct libusb_device_descriptor *dev_desc,
		const uint16_t vids[], const uint16_t pids[])
{
	for (unsigned i = 0; vids[i]; i++) {
		if (dev_desc->idVendor == vids[i] &&
			dev_desc->idProduct == pids[i]) {
			return true;
		}
	}
	return false;
}

#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
static bool jtag_libusb_location_equal(libusb_device *device)
{
	uint8_t port_path[MAX_USB_PORTS];
	uint8_t dev_bus;
	int path_len;

	path_len = libusb_get_port_numbers(device, port_path, MAX_USB_PORTS);
	if (path_len == LIBUSB_ERROR_OVERFLOW) {
		LOG_WARNING("cannot determine path to usb device! (more than %i ports in path)\n",
			MAX_USB_PORTS);
		return false;
	}
	dev_bus = libusb_get_bus_number(device);

	return jtag_usb_location_equal(dev_bus, port_path, path_len);
}
#else /* HAVE_LIBUSB_GET_PORT_NUMBERS */
static bool jtag_libusb_location_equal(libusb_device *device)
{
	return true;
}
#endif /* HAVE_LIBUSB_GET_PORT_NUMBERS */


/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(libusb_device_handle *device, uint8_t str_index,
									const char *string)
{
	int retval;
	bool matched;
	char desc_string[256+1]; /* Max size of string descriptor */

	if (str_index == 0)
		return false;

	retval = libusb_get_string_descriptor_ascii(device, str_index,
			(unsigned char *)desc_string, sizeof(desc_string)-1);
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %d", retval);
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

static bool jtag_libusb_match_serial(libusb_device_handle *device,
		struct libusb_device_descriptor *dev_desc, const char *serial,
		adapter_get_alternate_serial_fn adapter_get_alternate_serial)
{
	if (string_descriptor_equal(device, dev_desc->iSerialNumber, serial))
		return true;

	/* check the alternate serial helper */
	if (!adapter_get_alternate_serial)
		return false;

	/* get the alternate serial */
	char *alternate_serial = adapter_get_alternate_serial(device, dev_desc);

	/* check possible failures */
	if (alternate_serial == NULL)
		return false;

	/* then compare and free the alternate serial */
	bool match = false;
	if (strcmp(serial, alternate_serial) == 0)
		match = true;
	else
		LOG_DEBUG("Device alternate serial number '%s' doesn't match requested serial '%s'",
				alternate_serial, serial);

	free(alternate_serial);
	return match;
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *serial,
		struct libusb_device_handle **out,
		adapter_get_alternate_serial_fn adapter_get_alternate_serial)
{
	int cnt, idx, errCode;
	int retval = ERROR_FAIL;
	bool serial_mismatch = false;
	struct libusb_device_handle *libusb_handle = NULL;

	if (libusb_init(&jtag_libusb_context) < 0)
		return ERROR_FAIL;

	cnt = libusb_get_device_list(jtag_libusb_context, &devs);

	for (idx = 0; idx < cnt; idx++) {
		struct libusb_device_descriptor dev_desc;

		if (libusb_get_device_descriptor(devs[idx], &dev_desc) != 0)
			continue;

		if (!jtag_libusb_match_ids(&dev_desc, vids, pids))
			continue;

		if (jtag_usb_get_location() && !jtag_libusb_location_equal(devs[idx]))
			continue;

		errCode = libusb_open(devs[idx], &libusb_handle);

		if (errCode) {
			LOG_ERROR("libusb_open() failed with %s",
				  libusb_error_name(errCode));
			continue;
		}

		/* Device must be open to use libusb_get_string_descriptor_ascii. */
		if (serial != NULL &&
				!jtag_libusb_match_serial(libusb_handle, &dev_desc, serial, adapter_get_alternate_serial)) {
			serial_mismatch = true;
			libusb_close(libusb_handle);
			continue;
		}

		/* Success. */
		*out = libusb_handle;
		retval = ERROR_OK;
		serial_mismatch = false;
		break;
	}
	if (cnt >= 0)
		libusb_free_device_list(devs, 1);

	if (serial_mismatch)
		LOG_INFO("No device matches the serial string");

	if (retval != ERROR_OK)
		libusb_exit(jtag_libusb_context);

	return retval;
}

void jtag_libusb_close(struct libusb_device_handle *dev)
{
	/* Close device */
	libusb_close(dev);

	libusb_exit(jtag_libusb_context);
}

int jtag_libusb_control_transfer(struct libusb_device_handle *dev, uint8_t requestType,
		uint8_t request, uint16_t wValue, uint16_t wIndex, char *bytes,
		uint16_t size, unsigned int timeout)
{
	int transferred = 0;

	transferred = libusb_control_transfer(dev, requestType, request, wValue, wIndex,
				(unsigned char *)bytes, size, timeout);

	if (transferred < 0)
		transferred = 0;

	return transferred;
}

int jtag_libusb_bulk_write(struct libusb_device_handle *dev, int ep, char *bytes,
			   int size, int timeout, int *transferred)
{
	int ret;

	*transferred = 0;

	ret = libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
				   transferred, timeout);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_bulk_write error: %s", libusb_error_name(ret));
		return jtag_libusb_error(ret);
	}

	return ERROR_OK;
}

int jtag_libusb_bulk_read(struct libusb_device_handle *dev, int ep, char *bytes,
			  int size, int timeout, int *transferred)
{
	int ret;

	*transferred = 0;

	ret = libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
				   transferred, timeout);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_bulk_read error: %s", libusb_error_name(ret));
		return jtag_libusb_error(ret);
	}

	return ERROR_OK;
}

int jtag_libusb_set_configuration(struct libusb_device_handle *devh,
		int configuration)
{
	struct libusb_device *udev = libusb_get_device(devh);
	int retCode = -99;

	struct libusb_config_descriptor *config = NULL;
	int current_config = -1;

	retCode = libusb_get_configuration(devh, &current_config);
	if (retCode != 0)
		return retCode;

	retCode = libusb_get_config_descriptor(udev, configuration, &config);
	if (retCode != 0 || config == NULL)
		return retCode;

	/* Only change the configuration if it is not already set to the
	   same one. Otherwise this issues a lightweight reset and hangs
	   LPC-Link2 with JLink firmware. */
	if (current_config != config->bConfigurationValue)
		retCode = libusb_set_configuration(devh, config->bConfigurationValue);

	libusb_free_config_descriptor(config);

	return retCode;
}

int jtag_libusb_choose_interface(struct libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type)
{
	struct libusb_device *udev = libusb_get_device(devh);
	const struct libusb_interface *inter;
	const struct libusb_interface_descriptor *interdesc;
	const struct libusb_endpoint_descriptor *epdesc;
	struct libusb_config_descriptor *config;

	*usb_read_ep = *usb_write_ep = 0;

	libusb_get_config_descriptor(udev, 0, &config);
	for (int i = 0; i < (int)config->bNumInterfaces; i++) {
		inter = &config->interface[i];

		interdesc = &inter->altsetting[0];
		for (int k = 0;
		     k < (int)interdesc->bNumEndpoints; k++) {
			if ((bclass > 0 && interdesc->bInterfaceClass != bclass) ||
			    (subclass > 0 && interdesc->bInterfaceSubClass != subclass) ||
			    (protocol > 0 && interdesc->bInterfaceProtocol != protocol))
				continue;

			epdesc = &interdesc->endpoint[k];
			if (trans_type > 0 && (epdesc->bmAttributes & 0x3) != trans_type)
				continue;

			uint8_t epnum = epdesc->bEndpointAddress;
			bool is_input = epnum & 0x80;
			LOG_DEBUG("usb ep %s %02x",
				  is_input ? "in" : "out", epnum);

			if (is_input)
				*usb_read_ep = epnum;
			else
				*usb_write_ep = epnum;

			if (*usb_read_ep && *usb_write_ep) {
				LOG_DEBUG("Claiming interface %d", (int)interdesc->bInterfaceNumber);
				libusb_claim_interface(devh, (int)interdesc->bInterfaceNumber);
				libusb_free_config_descriptor(config);
				return ERROR_OK;
			}
		}
	}
	libusb_free_config_descriptor(config);

	return ERROR_FAIL;
}

int jtag_libusb_get_pid(struct libusb_device *dev, uint16_t *pid)
{
	struct libusb_device_descriptor dev_desc;

	if (libusb_get_device_descriptor(dev, &dev_desc) == 0) {
		*pid = dev_desc.idProduct;

		return ERROR_OK;
	}

	return ERROR_FAIL;
}
