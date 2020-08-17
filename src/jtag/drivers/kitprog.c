/***************************************************************************
 *   Copyright (C) 2007 by Juergen Stuber <juergen@jstuber.net>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Jean-Christophe PLAGNIOL-VIILARD                *
 *   plagnioj@jcrosoft.com                                                 *
 *                                                                         *
 *   Copyright (C) 2015 by Marc Schink                                     *
 *   openocd-dev@marcschink.de                                             *
 *                                                                         *
 *   Copyright (C) 2015 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2015-2017 by Forest Crossman                            *
 *   cyrozap@gmail.com                                                     *
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

#include <stdint.h>

#include <hidapi.h>

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>

#include "libusb_helper.h"

#define VID 0x04b4
#define PID 0xf139

#define BULK_EP_IN  1
#define BULK_EP_OUT 2

#define CONTROL_TYPE_READ  0x01
#define CONTROL_TYPE_WRITE 0x02

#define CONTROL_COMMAND_PROGRAM 0x07

#define CONTROL_MODE_POLL_PROGRAMMER_STATUS  0x01
#define CONTROL_MODE_RESET_TARGET            0x04
#define CONTROL_MODE_SET_PROGRAMMER_PROTOCOL 0x40
#define CONTROL_MODE_SYNCHRONIZE_TRANSFER    0x41
#define CONTROL_MODE_ACQUIRE_SWD_TARGET      0x42
#define CONTROL_MODE_SEND_SWD_SEQUENCE       0x43

#define PROTOCOL_JTAG 0x00
#define PROTOCOL_SWD  0x01

#define DEVICE_PSOC4   0x00
#define DEVICE_PSOC3   0x01
#define DEVICE_UNKNOWN 0x02
#define DEVICE_PSOC5   0x03

#define ACQUIRE_MODE_RESET       0x00
#define ACQUIRE_MODE_POWER_CYCLE 0x01

#define SEQUENCE_LINE_RESET  0x00
#define SEQUENCE_JTAG_TO_SWD 0x01

#define PROGRAMMER_NOK_NACK 0x00
#define PROGRAMMER_OK_ACK   0x01

#define HID_TYPE_WRITE 0x00
#define HID_TYPE_READ  0x01
#define HID_TYPE_START 0x02

#define HID_COMMAND_POWER      0x80
#define HID_COMMAND_VERSION    0x81
#define HID_COMMAND_RESET      0x82
#define HID_COMMAND_CONFIGURE  0x8f
#define HID_COMMAND_BOOTLOADER 0xa0

/* 512 bytes seems to work reliably */
#define SWD_MAX_BUFFER_LENGTH 512

struct kitprog {
	hid_device *hid_handle;
	struct libusb_device_handle *usb_handle;
	uint16_t packet_size;
	uint16_t packet_index;
	uint8_t *packet_buffer;
	char *serial;
	uint8_t hardware_version;
	uint8_t minor_version;
	uint8_t major_version;
	uint16_t millivolts;

	bool supports_jtag_to_swd;
};

struct pending_transfer_result {
	uint8_t cmd;
	uint32_t data;
	void *buffer;
};

static char *kitprog_serial;
static bool kitprog_init_acquire_psoc;

static int pending_transfer_count, pending_queue_len;
static struct pending_transfer_result *pending_transfers;

static int queued_retval;

static struct kitprog *kitprog_handle;

static int kitprog_usb_open(void);
static void kitprog_usb_close(void);

static int kitprog_hid_command(uint8_t *command, size_t command_length,
		uint8_t *data, size_t data_length);
static int kitprog_get_version(void);
static int kitprog_get_millivolts(void);
static int kitprog_get_info(void);
static int kitprog_set_protocol(uint8_t protocol);
static int kitprog_get_status(void);
static int kitprog_set_unknown(void);
static int kitprog_acquire_psoc(uint8_t psoc_type, uint8_t acquire_mode,
		uint8_t max_attempts);
static int kitprog_reset_target(void);
static int kitprog_swd_sync(void);
static int kitprog_swd_seq(uint8_t seq_type);

static int kitprog_generic_acquire(void);

static int kitprog_swd_run_queue(void);
static void kitprog_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data);
static int kitprog_swd_switch_seq(enum swd_special_seq seq);


static inline int mm_to_version(uint8_t major, uint8_t minor)
{
	return (major << 8) | minor;
}

static int kitprog_init(void)
{
	int retval;

	kitprog_handle = malloc(sizeof(struct kitprog));
	if (kitprog_handle == NULL) {
		LOG_ERROR("Failed to allocate memory");
		return ERROR_FAIL;
	}

	if (kitprog_usb_open() != ERROR_OK) {
		LOG_ERROR("Can't find a KitProg device! Please check device connections and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* Get the current KitProg version and target voltage */
	if (kitprog_get_info() != ERROR_OK)
		return ERROR_FAIL;

	/* Compatibility check */
	kitprog_handle->supports_jtag_to_swd = true;
	int kitprog_version = mm_to_version(kitprog_handle->major_version, kitprog_handle->minor_version);
	if (kitprog_version < mm_to_version(2, 14)) {
		LOG_WARNING("KitProg firmware versions below v2.14 do not support sending JTAG to SWD sequences. These sequences will be substituted with SWD line resets.");
		kitprog_handle->supports_jtag_to_swd = false;
	}

	/* I have no idea what this does */
	if (kitprog_set_unknown() != ERROR_OK)
		return ERROR_FAIL;

	/* SWD won't work unless we do this */
	if (kitprog_swd_sync() != ERROR_OK)
		return ERROR_FAIL;

	/* Set the protocol to SWD */
	if (kitprog_set_protocol(PROTOCOL_SWD) != ERROR_OK)
		return ERROR_FAIL;

	/* Reset the SWD bus */
	if (kitprog_swd_seq(SEQUENCE_LINE_RESET) != ERROR_OK)
		return ERROR_FAIL;

	if (kitprog_init_acquire_psoc) {
		/* Try to acquire any device that will respond */
		retval = kitprog_generic_acquire();
		if (retval != ERROR_OK) {
			LOG_ERROR("No PSoC devices found");
			return retval;
		}
	}

	/* Allocate packet buffers and queues */
	kitprog_handle->packet_size = SWD_MAX_BUFFER_LENGTH;
	kitprog_handle->packet_buffer = malloc(SWD_MAX_BUFFER_LENGTH);
	if (kitprog_handle->packet_buffer == NULL) {
		LOG_ERROR("Failed to allocate memory for the packet buffer");
		return ERROR_FAIL;
	}

	pending_queue_len = SWD_MAX_BUFFER_LENGTH / 5;
	pending_transfers = malloc(pending_queue_len * sizeof(*pending_transfers));
	if (pending_transfers == NULL) {
		LOG_ERROR("Failed to allocate memory for the SWD transfer queue");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_quit(void)
{
	kitprog_usb_close();

	free(kitprog_handle->packet_buffer);
	free(kitprog_handle->serial);
	free(kitprog_handle);
	free(kitprog_serial);
	free(pending_transfers);

	return ERROR_OK;
}

/*************** kitprog usb functions *********************/

static int kitprog_get_usb_serial(void)
{
	int retval;
	const uint8_t str_index = 128; /* This seems to be a constant */
	char desc_string[256+1]; /* Max size of string descriptor */

	retval = libusb_get_string_descriptor_ascii(kitprog_handle->usb_handle,
			str_index, (unsigned char *)desc_string, sizeof(desc_string)-1);
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %d", retval);
		return ERROR_FAIL;
	}

	/* Null terminate descriptor string */
	desc_string[retval] = '\0';

	/* Allocate memory for the serial number */
	kitprog_handle->serial = calloc(retval + 1, sizeof(char));
	if (kitprog_handle->serial == NULL) {
		LOG_ERROR("Failed to allocate memory for the serial number");
		return ERROR_FAIL;
	}

	/* Store the serial number */
	strncpy(kitprog_handle->serial, desc_string, retval + 1);

	return ERROR_OK;
}

static int kitprog_usb_open(void)
{
	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };

	if (jtag_libusb_open(vids, pids, kitprog_serial,
			&kitprog_handle->usb_handle, NULL) != ERROR_OK) {
		LOG_ERROR("Failed to open or find the device");
		return ERROR_FAIL;
	}

	/* Get the serial number for the device */
	if (kitprog_get_usb_serial() != ERROR_OK)
		LOG_WARNING("Failed to get KitProg serial number");

	/* Convert the ASCII serial number into a (wchar_t *) */
	size_t len = strlen(kitprog_handle->serial);
	wchar_t *hid_serial = calloc(len + 1, sizeof(wchar_t));
	if (hid_serial == NULL) {
		LOG_ERROR("Failed to allocate memory for the serial number");
		return ERROR_FAIL;
	}
	if (mbstowcs(hid_serial, kitprog_handle->serial, len + 1) == (size_t)-1) {
		free(hid_serial);
		LOG_ERROR("Failed to convert serial number");
		return ERROR_FAIL;
	}

	/* Use HID for the KitBridge interface */
	kitprog_handle->hid_handle = hid_open(VID, PID, hid_serial);
	free(hid_serial);
	if (kitprog_handle->hid_handle == NULL) {
		LOG_ERROR("Failed to open KitBridge (HID) interface");
		return ERROR_FAIL;
	}

	/* Claim the KitProg Programmer (bulk transfer) interface */
	if (libusb_claim_interface(kitprog_handle->usb_handle, 1) != ERROR_OK) {
		LOG_ERROR("Failed to claim KitProg Programmer (bulk transfer) interface");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void kitprog_usb_close(void)
{
	if (kitprog_handle->hid_handle != NULL) {
		hid_close(kitprog_handle->hid_handle);
		hid_exit();
	}

	jtag_libusb_close(kitprog_handle->usb_handle);
}

/*************** kitprog lowlevel functions *********************/

static int kitprog_hid_command(uint8_t *command, size_t command_length,
		uint8_t *data, size_t data_length)
{
	int ret;

	ret = hid_write(kitprog_handle->hid_handle, command, command_length);
	if (ret < 0) {
		LOG_DEBUG("HID write returned %i", ret);
		return ERROR_FAIL;
	}

	ret = hid_read(kitprog_handle->hid_handle, data, data_length);
	if (ret < 0) {
		LOG_DEBUG("HID read returned %i", ret);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_get_version(void)
{
	int ret;

	unsigned char command[3] = {HID_TYPE_START | HID_TYPE_WRITE, 0x00, HID_COMMAND_VERSION};
	unsigned char data[64];

	ret = kitprog_hid_command(command, sizeof(command), data, sizeof(data));
	if (ret != ERROR_OK)
		return ret;

	kitprog_handle->hardware_version = data[1];
	kitprog_handle->minor_version = data[2];
	kitprog_handle->major_version = data[3];

	return ERROR_OK;
}

static int kitprog_get_millivolts(void)
{
	int ret;

	unsigned char command[3] = {HID_TYPE_START | HID_TYPE_READ, 0x00, HID_COMMAND_POWER};
	unsigned char data[64];

	ret = kitprog_hid_command(command, sizeof(command), data, sizeof(data));
	if (ret != ERROR_OK)
		return ret;

	kitprog_handle->millivolts = (data[4] << 8) | data[3];

	return ERROR_OK;
}

static int kitprog_get_info(void)
{
	/* Get the device version information */
	if (kitprog_get_version() == ERROR_OK) {
		LOG_INFO("KitProg v%u.%02u",
			kitprog_handle->major_version, kitprog_handle->minor_version);
		LOG_INFO("Hardware version: %u",
			kitprog_handle->hardware_version);
	} else {
		LOG_ERROR("Failed to get KitProg version");
		return ERROR_FAIL;
	}

	/* Get the current reported target voltage */
	if (kitprog_get_millivolts() == ERROR_OK) {
		LOG_INFO("VTARG = %u.%03u V",
			kitprog_handle->millivolts / 1000, kitprog_handle->millivolts % 1000);
	} else {
		LOG_ERROR("Failed to get target voltage");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_set_protocol(uint8_t protocol)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_SET_PROGRAMMER_PROTOCOL << 8) | CONTROL_COMMAND_PROGRAM,
		protocol, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_get_status(void)
{
	int transferred = 0;
	char status = PROGRAMMER_NOK_NACK;

	/* Try a maximum of three times */
	for (int i = 0; (i < 3) && (transferred == 0); i++) {
		transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			CONTROL_TYPE_READ,
			(CONTROL_MODE_POLL_PROGRAMMER_STATUS << 8) | CONTROL_COMMAND_PROGRAM,
			0, &status, 1, 0);
		jtag_sleep(1000);
	}

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_set_unknown(void)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(0x03 << 8) | 0x04,
		0, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_acquire_psoc(uint8_t psoc_type, uint8_t acquire_mode,
		uint8_t max_attempts)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_ACQUIRE_SWD_TARGET << 8) | CONTROL_COMMAND_PROGRAM,
		(max_attempts << 8) | (acquire_mode << 4) | psoc_type, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_reset_target(void)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_RESET_TARGET << 8) | CONTROL_COMMAND_PROGRAM,
		0, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_swd_sync(void)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_SYNCHRONIZE_TRANSFER << 8) | CONTROL_COMMAND_PROGRAM,
		0, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_swd_seq(uint8_t seq_type)
{
	int transferred;
	char status = PROGRAMMER_NOK_NACK;

	transferred = jtag_libusb_control_transfer(kitprog_handle->usb_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		CONTROL_TYPE_WRITE,
		(CONTROL_MODE_SEND_SWD_SEQUENCE << 8) | CONTROL_COMMAND_PROGRAM,
		seq_type, &status, 1, 0);

	if (transferred == 0) {
		LOG_DEBUG("Zero bytes transferred");
		return ERROR_FAIL;
	}

	if (status != PROGRAMMER_OK_ACK) {
		LOG_DEBUG("Programmer did not respond OK");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_generic_acquire(void)
{
	const uint8_t devices[] = {DEVICE_PSOC4, DEVICE_PSOC3, DEVICE_PSOC5};

	int retval;
	int acquire_count = 0;

	/* Due to the way the SWD port is shared between the Test Controller (TC)
	 * and the Cortex-M3 DAP on the PSoC 5LP, the TC is the default SWD target
	 * after power is applied. To access the DAP, the PSoC 5LP requires at least
	 * one acquisition sequence to be run (which switches the SWD mux from the
	 * TC to the DAP). However, after the mux is switched, the Cortex-M3 will be
	 * held in reset until a series of registers are written to (see section 5.2
	 * of the PSoC 5LP Device Programming Specifications for details).
	 *
	 * Instead of writing the registers in this function, we just do what the
	 * Cypress tools do and run the acquisition sequence a second time. This
	 * will take the Cortex-M3 out of reset and enable debugging.
	 */
	for (int i = 0; i < 2; i++) {
		for (uint8_t j = 0; j < sizeof(devices) && acquire_count == i; j++) {
			retval = kitprog_acquire_psoc(devices[j], ACQUIRE_MODE_RESET, 3);
			if (retval != ERROR_OK) {
				LOG_DEBUG("Acquisition function failed for device 0x%02x.", devices[j]);
				return retval;
			}

			if (kitprog_get_status() == ERROR_OK)
				acquire_count++;
		}

		jtag_sleep(10);
	}

	if (acquire_count < 2)
		return ERROR_FAIL;

	return ERROR_OK;
}

/*************** swd wrapper functions *********************/

static int kitprog_swd_init(void)
{
	return ERROR_OK;
}

static void kitprog_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	kitprog_swd_queue_cmd(cmd, NULL, value);
}

static void kitprog_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	kitprog_swd_queue_cmd(cmd, value, 0);
}

/*************** swd lowlevel functions ********************/

static int kitprog_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
		case JTAG_TO_SWD:
			if (kitprog_handle->supports_jtag_to_swd) {
				LOG_DEBUG("JTAG to SWD");
				if (kitprog_swd_seq(SEQUENCE_JTAG_TO_SWD) != ERROR_OK)
					return ERROR_FAIL;
				break;
			} else {
				LOG_DEBUG("JTAG to SWD not supported");
				/* Fall through to fix target reset issue */
			}
			/* fallthrough */
		case LINE_RESET:
			LOG_DEBUG("SWD line reset");
			if (kitprog_swd_seq(SEQUENCE_LINE_RESET) != ERROR_OK)
				return ERROR_FAIL;
			break;
		default:
			LOG_ERROR("Sequence %d not supported.", seq);
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int kitprog_swd_run_queue(void)
{
	int ret;

	size_t read_count = 0;
	size_t read_index = 0;
	size_t write_count = 0;
	uint8_t *buffer = kitprog_handle->packet_buffer;

	do {
		LOG_DEBUG_IO("Executing %d queued transactions", pending_transfer_count);

		if (queued_retval != ERROR_OK) {
			LOG_DEBUG("Skipping due to previous errors: %d", queued_retval);
			break;
		}

		if (!pending_transfer_count)
			break;

		for (int i = 0; i < pending_transfer_count; i++) {
			uint8_t cmd = pending_transfers[i].cmd;
			uint32_t data = pending_transfers[i].data;

			/* When proper WAIT handling is implemented in the
			 * common SWD framework, this kludge can be
			 * removed. However, this might lead to minor
			 * performance degradation as the adapter wouldn't be
			 * able to automatically retry anything (because ARM
			 * has forgotten to implement sticky error flags
			 * clearing). See also comments regarding
			 * cmsis_dap_cmd_DAP_TFER_Configure() and
			 * cmsis_dap_cmd_DAP_SWD_Configure() in
			 * cmsis_dap_init().
			 */
			if (!(cmd & SWD_CMD_RnW) &&
				!(cmd & SWD_CMD_APnDP) &&
				(cmd & SWD_CMD_A32) >> 1 == DP_CTRL_STAT &&
				(data & CORUNDETECT)) {
				LOG_DEBUG("refusing to enable sticky overrun detection");
				data &= ~CORUNDETECT;
			}

			LOG_DEBUG_IO("%s %s reg %x %"PRIx32,
					cmd & SWD_CMD_APnDP ? "AP" : "DP",
					cmd & SWD_CMD_RnW ? "read" : "write",
				  (cmd & SWD_CMD_A32) >> 1, data);

			buffer[write_count++] = (cmd | SWD_CMD_START | SWD_CMD_PARK) & ~SWD_CMD_STOP;
			read_count++;
			if (!(cmd & SWD_CMD_RnW)) {
				buffer[write_count++] = (data) & 0xff;
				buffer[write_count++] = (data >> 8) & 0xff;
				buffer[write_count++] = (data >> 16) & 0xff;
				buffer[write_count++] = (data >> 24) & 0xff;
			} else {
				read_count += 4;
			}
		}

		if (jtag_libusb_bulk_write(kitprog_handle->usb_handle,
					   BULK_EP_OUT, (char *)buffer,
					   write_count, 0, &ret)) {
			LOG_ERROR("Bulk write failed");
			queued_retval = ERROR_FAIL;
			break;
		} else {
			queued_retval = ERROR_OK;
		}

		/* KitProg firmware does not send a zero length packet
		 * after the bulk-in transmission of a length divisible by bulk packet
		 * size (64 bytes) as required by the USB specification.
		 * Therefore libusb would wait for continuation of transmission.
		 * Workaround: Limit bulk read size to expected number of bytes
		 * for problematic transfer sizes. Otherwise use the maximum buffer
		 * size here because the KitProg sometimes doesn't like bulk reads
		 * of fewer than 62 bytes. (?!?!)
		 */
		size_t read_count_workaround = SWD_MAX_BUFFER_LENGTH;
		if (read_count % 64 == 0)
			read_count_workaround = read_count;

		if (jtag_libusb_bulk_read(kitprog_handle->usb_handle,
				BULK_EP_IN | LIBUSB_ENDPOINT_IN, (char *)buffer,
				read_count_workaround, 1000, &ret)) {
			LOG_ERROR("Bulk read failed");
			queued_retval = ERROR_FAIL;
			break;
		} else {
			/* Handle garbage data by offsetting the initial read index */
			if ((unsigned int)ret > read_count)
				read_index = ret - read_count;
			queued_retval = ERROR_OK;
		}

		for (int i = 0; i < pending_transfer_count; i++) {
			if (pending_transfers[i].cmd & SWD_CMD_RnW) {
				uint32_t data = le_to_h_u32(&buffer[read_index]);

				LOG_DEBUG_IO("Read result: %"PRIx32, data);

				if (pending_transfers[i].buffer)
					*(uint32_t *)pending_transfers[i].buffer = data;

				read_index += 4;
			}

			uint8_t ack = buffer[read_index] & 0x07;
			if (ack != SWD_ACK_OK || (buffer[read_index] & 0x08)) {
				LOG_DEBUG("SWD ack not OK: %d %s", i,
					  ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK");
				queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
				break;
			}
			read_index++;
		}
	} while (0);

	pending_transfer_count = 0;
	int retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

static void kitprog_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data)
{
	if (pending_transfer_count == pending_queue_len) {
		/* Not enough room in the queue. Run the queue. */
		queued_retval = kitprog_swd_run_queue();
	}

	if (queued_retval != ERROR_OK)
		return;

	pending_transfers[pending_transfer_count].data = data;
	pending_transfers[pending_transfer_count].cmd = cmd;
	if (cmd & SWD_CMD_RnW) {
		/* Queue a read transaction */
		pending_transfers[pending_transfer_count].buffer = dst;
	}
	pending_transfer_count++;
}

/*************** jtag lowlevel functions ********************/

static int kitprog_reset(int trst, int srst)
{
	int retval = ERROR_OK;

	if (trst == 1) {
		LOG_ERROR("KitProg: Interface has no TRST");
		return ERROR_FAIL;
	}

	if (srst == 1) {
		retval = kitprog_reset_target();
		/* Since the previous command also disables SWCLK output, we need to send an
		 * SWD bus reset command to re-enable it. For some reason, running
		 * kitprog_swd_seq() immediately after kitprog_reset_target() won't
		 * actually fix this. Instead, kitprog_swd_seq() will be run once OpenOCD
		 * tries to send a JTAG-to-SWD sequence, which should happen during
		 * swd_check_reconnect (see the JTAG_TO_SWD case in kitprog_swd_switch_seq).
		 */
	}

	if (retval != ERROR_OK)
		LOG_ERROR("KitProg: Interface reset failed");
	return retval;
}

COMMAND_HANDLER(kitprog_handle_info_command)
{
	int retval = kitprog_get_info();

	return retval;
}


COMMAND_HANDLER(kitprog_handle_acquire_psoc_command)
{
	int retval = kitprog_generic_acquire();

	return retval;
}

COMMAND_HANDLER(kitprog_handle_serial_command)
{
	if (CMD_ARGC == 1) {
		kitprog_serial = strdup(CMD_ARGV[0]);
		if (kitprog_serial == NULL) {
			LOG_ERROR("Failed to allocate memory for the serial number");
			return ERROR_FAIL;
		}
	} else {
		LOG_ERROR("expected exactly one argument to kitprog_serial <serial-number>");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(kitprog_handle_init_acquire_psoc_command)
{
	kitprog_init_acquire_psoc = true;

	return ERROR_OK;
}

static const struct command_registration kitprog_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &kitprog_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "show KitProg info",
	},
	{
		.name = "acquire_psoc",
		.handler = &kitprog_handle_acquire_psoc_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "try to acquire a PSoC",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration kitprog_command_handlers[] = {
	{
		.name = "kitprog",
		.mode = COMMAND_ANY,
		.help = "perform KitProg management",
		.usage = "<cmd>",
		.chain = kitprog_subcommand_handlers,
	},
	{
		.name = "kitprog_serial",
		.handler = &kitprog_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the adapter",
		.usage = "serial_string",
	},
	{
		.name = "kitprog_init_acquire_psoc",
		.handler = &kitprog_handle_init_acquire_psoc_command,
		.mode = COMMAND_CONFIG,
		.help = "try to acquire a PSoC during init",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver kitprog_swd = {
	.init = kitprog_swd_init,
	.switch_seq = kitprog_swd_switch_seq,
	.read_reg = kitprog_swd_read_reg,
	.write_reg = kitprog_swd_write_reg,
	.run = kitprog_swd_run_queue,
};

static const char * const kitprog_transports[] = { "swd", NULL };

struct adapter_driver kitprog_adapter_driver = {
	.name = "kitprog",
	.transports = kitprog_transports,
	.commands = kitprog_command_handlers,

	.init = kitprog_init,
	.quit = kitprog_quit,
	.reset = kitprog_reset,

	.swd_ops = &kitprog_swd,
};
