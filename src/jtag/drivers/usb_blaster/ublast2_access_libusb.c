/*
 *   Driver for USB-JTAG, Altera USB-Blaster II and compatibles
 *
 *   Copyright (C) 2013 Franck Jullien franck.jullien@gmail.com
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <libusb_helper.h>
#include <target/image.h>

#include "ublast_access.h"

#define USBBLASTER_CTRL_READ_REV	0x94
#define USBBLASTER_CTRL_LOAD_FIRM	0xA0
#define USBBLASTER_EPOUT		4
#define USBBLASTER_EPIN			8

#define EZUSB_CPUCS			0xe600
#define CPU_RESET			1

/** Maximum size of a single firmware section. Entire EZ-USB code space = 16kB */
#define SECTION_BUFFERSIZE		16384

static int ublast2_libusb_read(struct ublast_lowlevel *low, uint8_t *buf,
			      unsigned size, uint32_t *bytes_read)
{
	int ret, tmp = 0;

	ret = jtag_libusb_bulk_read(low->libusb_dev,
					    USBBLASTER_EPIN |
					    LIBUSB_ENDPOINT_IN,
					    (char *)buf,
					    size,
					    100, &tmp);
	*bytes_read = tmp;

	return ret;
}

static int ublast2_libusb_write(struct ublast_lowlevel *low, uint8_t *buf,
			       int size, uint32_t *bytes_written)
{
	int ret, tmp = 0;

	ret = jtag_libusb_bulk_write(low->libusb_dev,
						USBBLASTER_EPOUT |
						LIBUSB_ENDPOINT_OUT,
						(char *)buf,
						size,
						100, &tmp);
	*bytes_written = tmp;

	return ret;

}

static int ublast2_write_firmware_section(struct libusb_device_handle *libusb_dev,
				   struct image *firmware_image, int section_index)
{
	uint16_t chunk_size;
	uint8_t data[SECTION_BUFFERSIZE];
	uint8_t *data_ptr = data;
	size_t size_read;

	uint16_t size = (uint16_t)firmware_image->sections[section_index].size;
	uint16_t addr = (uint16_t)firmware_image->sections[section_index].base_address;

	LOG_DEBUG("section %02i at addr 0x%04x (size 0x%04x)", section_index, addr,
		size);

	/* Copy section contents to local buffer */
	int ret = image_read_section(firmware_image, section_index, 0, size, data,
			&size_read);

	if ((ret != ERROR_OK) || (size_read != size)) {
		/* Propagating the return code would return '0' (misleadingly indicating
		 * successful execution of the function) if only the size check fails. */
		return ERROR_FAIL;
	}

	uint16_t bytes_remaining = size;

	/* Send section data in chunks of up to 64 bytes to ULINK */
	while (bytes_remaining > 0) {
		if (bytes_remaining > 64)
			chunk_size = 64;
		else
			chunk_size = bytes_remaining;

		jtag_libusb_control_transfer(libusb_dev,
					     LIBUSB_REQUEST_TYPE_VENDOR |
					     LIBUSB_ENDPOINT_OUT,
					     USBBLASTER_CTRL_LOAD_FIRM,
					     addr,
					     0,
					     (char *)data_ptr,
					     chunk_size,
					     100);

		bytes_remaining -= chunk_size;
		addr += chunk_size;
		data_ptr += chunk_size;
	}

	return ERROR_OK;
}

static int load_usb_blaster_firmware(struct libusb_device_handle *libusb_dev,
				     struct ublast_lowlevel *low)
{
	struct image ublast2_firmware_image;

	if (!low->firmware_path) {
		LOG_ERROR("No firmware path specified");
		return ERROR_FAIL;
	}

	ublast2_firmware_image.base_address = 0;
	ublast2_firmware_image.base_address_set = false;

	int ret = image_open(&ublast2_firmware_image, low->firmware_path, "ihex");
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not load firmware image");
		return ret;
	}

	/** A host loader program must write 0x01 to the CPUCS register
	 * to put the CPU into RESET, load all or part of the EZUSB
	 * RAM with firmware, then reload the CPUCS register
	 * with ‘0’ to take the CPU out of RESET. The CPUCS register
	 * (at 0xE600) is the only EZ-USB register that can be written
	 * using the Firmware Download command.
	 */

	char value = CPU_RESET;
	jtag_libusb_control_transfer(libusb_dev,
				     LIBUSB_REQUEST_TYPE_VENDOR |
				     LIBUSB_ENDPOINT_OUT,
				     USBBLASTER_CTRL_LOAD_FIRM,
				     EZUSB_CPUCS,
				     0,
				     &value,
				     1,
				     100);

	/* Download all sections in the image to ULINK */
	for (unsigned int i = 0; i < ublast2_firmware_image.num_sections; i++) {
		ret = ublast2_write_firmware_section(libusb_dev,
						     &ublast2_firmware_image, i);
		if (ret != ERROR_OK) {
			LOG_ERROR("Error while downloading the firmware");
			return ret;
		}
	}

	value = !CPU_RESET;
	jtag_libusb_control_transfer(libusb_dev,
				     LIBUSB_REQUEST_TYPE_VENDOR |
				     LIBUSB_ENDPOINT_OUT,
				     USBBLASTER_CTRL_LOAD_FIRM,
				     EZUSB_CPUCS,
				     0,
				     &value,
				     1,
				     100);

	image_close(&ublast2_firmware_image);

	return ERROR_OK;
}

static int ublast2_libusb_init(struct ublast_lowlevel *low)
{
	const uint16_t vids[] = { low->ublast_vid_uninit, 0 };
	const uint16_t pids[] = { low->ublast_pid_uninit, 0 };
	struct libusb_device_handle *temp;
	bool renumeration = false;
	int ret;

	if (jtag_libusb_open(vids, pids, NULL, &temp, NULL) == ERROR_OK) {
		LOG_INFO("Altera USB-Blaster II (uninitialized) found");
		LOG_INFO("Loading firmware...");
		ret = load_usb_blaster_firmware(temp, low);
		jtag_libusb_close(temp);
		if (ret != ERROR_OK)
			return ret;
		renumeration = true;
	}

	const uint16_t vids_renum[] = { low->ublast_vid, 0 };
	const uint16_t pids_renum[] = { low->ublast_pid, 0 };

	if (renumeration == false) {
		if (jtag_libusb_open(vids_renum, pids_renum, NULL,
				&low->libusb_dev, NULL) != ERROR_OK) {
			LOG_ERROR("Altera USB-Blaster II not found");
			return ERROR_FAIL;
		}
	} else {
		int retry = 10;
		while (jtag_libusb_open(vids_renum, pids_renum, NULL,
				&low->libusb_dev, NULL) != ERROR_OK && retry--) {
			usleep(1000000);
			LOG_INFO("Waiting for reenumerate...");
		}

		if (!retry) {
			LOG_ERROR("Altera USB-Blaster II not found");
			return ERROR_FAIL;
		}
	}

	char buffer[5];
	jtag_libusb_control_transfer(low->libusb_dev,
				     LIBUSB_REQUEST_TYPE_VENDOR |
				     LIBUSB_ENDPOINT_IN,
				     USBBLASTER_CTRL_READ_REV,
				     0,
				     0,
				     buffer,
				     5,
				     100);

	LOG_INFO("Altera USB-Blaster II found (Firm. rev. = %s)", buffer);

	return ERROR_OK;
}

static int ublast2_libusb_quit(struct ublast_lowlevel *low)
{
	jtag_libusb_close(low->libusb_dev);
	return ERROR_OK;
};

static struct ublast_lowlevel low = {
	.open = ublast2_libusb_init,
	.close = ublast2_libusb_quit,
	.read = ublast2_libusb_read,
	.write = ublast2_libusb_write,
	.flags = COPY_TDO_BUFFER,
};

struct ublast_lowlevel *ublast2_register_libusb(void)
{
	return &low;
}
