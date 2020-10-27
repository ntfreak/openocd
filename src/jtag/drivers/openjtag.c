/*******************************************************************************
 *   Driver for OpenJTAG Project (www.openjtag.org)                            *
 *   Compatible with libftdi and ftd2xx drivers.                               *
 *                                                                             *
 *   Cypress CY7C65215 support                                                 *
 *   Copyright (C) 2015 Vianney le Clément de Saint-Marcq, Essensium NV        *
 *                      <vianney.leclement@essensium.com>                      *
 *                                                                             *
 *   Copyright (C) 2010 by Ivan Meleca <mileca@gmail.com>                      *
 *                                                                             *
 *   Copyright (C) 2013 by Ryan Corbin, GlueLogix Inc. <corbin.ryan@gmail.com> *
 *   Updated to work with OpenOCD v0.7.0. Fixed libftdi read speed issue.      *
 *                                                                             *
 *   Based on usb_blaster.c                                                    *
 *   Copyright (C) 2009 Catalin Patulea                                        *
 *   Copyright (C) 2006 Kolja Waschk                                           *
 *                                                                             *
 *   And jlink.c                                                               *
 *   Copyright (C) 2008 by Spencer Oliver                                      *
 *   spen@spen-soft.co.uk                                                      *
 *                                                                             *
 *   This program is free software; you can redistribute it and/or modify      *
 *   it under the terms of the GNU General Public License as published by      *
 *   the Free Software Foundation; either version 2 of the License, or         *
 *   (at your option) any later version.                                       *
 *                                                                             *
 *   This program is distributed in the hope that it will be useful,           *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *   GNU General Public License for more details.                              *
 *                                                                             *
 *   You should have received a copy of the GNU General Public License         *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
 ***************************************************************************/

/***************************************************************************
 * Version 1.0  Tested on a MCBSTM32 board using a Cortex-M3 (stm32f103x), *
 *              GDB and Eclipse under Linux (Ubuntu 10.04)                 *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "libusb_helper.h"

static enum {
	OPENJTAG_VARIANT_STANDARD,
	OPENJTAG_VARIANT_CY7C65215,
} openjtag_variant = OPENJTAG_VARIANT_STANDARD;

static const char * const openjtag_variant_names[] = {
	"standard",
	"cy7c65215",
	NULL
};

/*
 * OpenJTAG-OpenOCD state conversion
 */
typedef enum openjtag_tap_state {
	OPENJTAG_TAP_INVALID    = -1,
	OPENJTAG_TAP_RESET  = 0,
	OPENJTAG_TAP_IDLE   = 1,
	OPENJTAG_TAP_SELECT_DR  = 2,
	OPENJTAG_TAP_CAPTURE_DR = 3,
	OPENJTAG_TAP_SHIFT_DR   = 4,
	OPENJTAG_TAP_EXIT1_DR   = 5,
	OPENJTAG_TAP_PAUSE_DR   = 6,
	OPENJTAG_TAP_EXIT2_DR   = 7,
	OPENJTAG_TAP_UPDATE_DR  = 8,
	OPENJTAG_TAP_SELECT_IR  = 9,
	OPENJTAG_TAP_CAPURE_IR  = 10,
	OPENJTAG_TAP_SHIFT_IR   = 11,
	OPENJTAG_TAP_EXIT1_IR   = 12,
	OPENJTAG_TAP_PAUSE_IR   = 13,
	OPENJTAG_TAP_EXIT2_IR   = 14,
	OPENJTAG_TAP_UPDATE_IR  = 15,
} openjtag_tap_state_t;

/* OPENJTAG access library includes */
#include "libftdi_helper.h"

/* OpenJTAG vid/pid */
static uint16_t openjtag_vid = 0x0403;
static uint16_t openjtag_pid = 0x6001;

static char *openjtag_device_desc;

static struct ftdi_context ftdic;

#define OPENJTAG_BUFFER_SIZE        504
#define OPENJTAG_MAX_PENDING_RESULTS    256

struct openjtag_scan_result {
	uint32_t bits;          /* Length in bits*/
	struct scan_command *command;   /* Corresponding scan command */
	uint8_t *buffer;
};

/* USB RX/TX buffers */
static int usb_tx_buf_offs;
static uint8_t usb_tx_buf[OPENJTAG_BUFFER_SIZE];
static uint32_t usb_rx_buf_len;
static uint8_t usb_rx_buf[OPENJTAG_BUFFER_SIZE];

/* Pending readings */
static struct openjtag_scan_result openjtag_scan_result_buffer[OPENJTAG_MAX_PENDING_RESULTS];
static int openjtag_scan_result_count;

static struct libusb_device_handle *usbh;

/* CY7C65215 model only */
#define CY7C65215_JTAG_REQUEST  0x40  /* bmRequestType: vendor host-to-device */
#define CY7C65215_JTAG_ENABLE   0xD0  /* bRequest: enable JTAG */
#define CY7C65215_JTAG_DISABLE  0xD1  /* bRequest: disable JTAG */
#define CY7C65215_JTAG_READ     0xD2  /* bRequest: read buffer */
#define CY7C65215_JTAG_WRITE    0xD3  /* bRequest: write buffer */

#define CY7C65215_USB_TIMEOUT   100

static const uint16_t cy7c65215_vids[] = {0x04b4, 0};
static const uint16_t cy7c65215_pids[] = {0x0007, 0};

#define CY7C65215_JTAG_CLASS     0xff
#define CY7C65215_JTAG_SUBCLASS  0x04

static unsigned int ep_in, ep_out;

#ifdef _DEBUG_USB_COMMS_

#define DEBUG_TYPE_READ     0
#define DEBUG_TYPE_WRITE    1
#define DEBUG_TYPE_OCD_READ 2
#define DEBUG_TYPE_BUFFER   3

#define LINE_LEN  16
static void openjtag_debug_buffer(uint8_t *buffer, int length, uint8_t type)
{
	char line[128];
	char s[4];
	int i;
	int j;

	switch (type) {
		case DEBUG_TYPE_READ:
			sprintf(line, "USB READ %d bytes", length);
			break;
		case DEBUG_TYPE_WRITE:
			sprintf(line, "USB WRITE %d bytes", length);
			break;
		case DEBUG_TYPE_OCD_READ:
			sprintf(line, "TO OpenOCD %d bytes", length);
			break;
		case DEBUG_TYPE_BUFFER:
			sprintf(line, "Buffer %d bytes", length);
			break;
	}

	LOG_DEBUG("%s", line);

	for (i = 0; i < length; i += LINE_LEN) {
		switch (type) {
			case DEBUG_TYPE_READ:
				sprintf(line, "USB READ: %04x", i);
				break;
			case DEBUG_TYPE_WRITE:
				sprintf(line, "USB WRITE: %04x", i);
				break;
			case DEBUG_TYPE_OCD_READ:
				sprintf(line, "TO OpenOCD: %04x", i);
				break;
			case DEBUG_TYPE_BUFFER:
				sprintf(line, "BUFFER: %04x", i);
				break;
		}

		for (j = i; j < i + LINE_LEN && j < length; j++) {
			sprintf(s, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}

}

#endif

static int8_t openjtag_get_tap_state(int8_t state)
{

	switch (state) {
		case TAP_DREXIT2:   return OPENJTAG_TAP_EXIT2_DR;
		case TAP_DREXIT1:   return OPENJTAG_TAP_EXIT1_DR;
		case TAP_DRSHIFT:   return OPENJTAG_TAP_SHIFT_DR;
		case TAP_DRPAUSE:   return OPENJTAG_TAP_PAUSE_DR;
		case TAP_IRSELECT:  return OPENJTAG_TAP_SELECT_IR;
		case TAP_DRUPDATE:  return OPENJTAG_TAP_UPDATE_DR;
		case TAP_DRCAPTURE: return OPENJTAG_TAP_CAPTURE_DR;
		case TAP_DRSELECT:  return OPENJTAG_TAP_SELECT_DR;
		case TAP_IREXIT2:   return OPENJTAG_TAP_EXIT2_IR;
		case TAP_IREXIT1:   return OPENJTAG_TAP_EXIT1_IR;
		case TAP_IRSHIFT:   return OPENJTAG_TAP_SHIFT_IR;
		case TAP_IRPAUSE:   return OPENJTAG_TAP_PAUSE_IR;
		case TAP_IDLE:      return OPENJTAG_TAP_IDLE;
		case TAP_IRUPDATE:  return OPENJTAG_TAP_UPDATE_IR;
		case TAP_IRCAPTURE: return OPENJTAG_TAP_CAPURE_IR;
		case TAP_RESET:     return OPENJTAG_TAP_RESET;
		case TAP_INVALID:
		default:            return OPENJTAG_TAP_INVALID;
	}
}

static int openjtag_buf_write_standard(
	uint8_t *buf, int size, uint32_t *bytes_written)
{
	int retval;
#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, size, DEBUG_TYPE_WRITE);
#endif

	retval = ftdi_write_data(&ftdic, buf, size);
	if (retval < 0) {
		*bytes_written = 0;
		LOG_ERROR("ftdi_write_data: %s", ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	*bytes_written = retval;

	return ERROR_OK;
}

static int openjtag_buf_write_cy7c65215(
	uint8_t *buf, int size, uint32_t *bytes_written)
{
	int ret;

#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, size, DEBUG_TYPE_WRITE);
#endif

	if (size == 0) {
		*bytes_written = 0;
		return ERROR_OK;
	}

	ret = jtag_libusb_control_transfer(usbh, CY7C65215_JTAG_REQUEST,
									   CY7C65215_JTAG_WRITE, size, 0,
									   NULL, 0, CY7C65215_USB_TIMEOUT);
	if (ret < 0) {
		LOG_ERROR("vendor command failed, error %d", ret);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (jtag_libusb_bulk_write(usbh, ep_out, (char *)buf, size,
				   CY7C65215_USB_TIMEOUT, &ret)) {
		LOG_ERROR("bulk write failed, error");
		return ERROR_JTAG_DEVICE_ERROR;
	}
	*bytes_written = ret;

	return ERROR_OK;
}

static int openjtag_buf_write(
	uint8_t *buf, int size, uint32_t *bytes_written)
{
	switch (openjtag_variant) {
	case OPENJTAG_VARIANT_CY7C65215:
		return openjtag_buf_write_cy7c65215(buf, size, bytes_written);
	default:
		return openjtag_buf_write_standard(buf, size, bytes_written);
	}
}

static int openjtag_buf_read_standard(
	uint8_t *buf, uint32_t qty, uint32_t *bytes_read)
{

	int retval;
	int timeout = 5;

	*bytes_read = 0;

	while ((*bytes_read < qty) && timeout--) {
		retval = ftdi_read_data(&ftdic, buf + *bytes_read,
				qty - *bytes_read);
		if (retval < 0) {
			*bytes_read = 0;
			LOG_DEBUG_IO("ftdi_read_data: %s",
					ftdi_get_error_string(&ftdic));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += retval;
	}

#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, *bytes_read, DEBUG_TYPE_READ);
#endif

	return ERROR_OK;
}

static int openjtag_buf_read_cy7c65215(
	uint8_t *buf, uint32_t qty, uint32_t *bytes_read)
{
	int ret;

	if (qty == 0) {
		*bytes_read = 0;
		goto out;
	}

	ret = jtag_libusb_control_transfer(usbh, CY7C65215_JTAG_REQUEST,
									   CY7C65215_JTAG_READ, qty, 0,
									   NULL, 0, CY7C65215_USB_TIMEOUT);
	if (ret < 0) {
		LOG_ERROR("vendor command failed, error %d", ret);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (jtag_libusb_bulk_read(usbh, ep_in, (char *)buf, qty,
				  CY7C65215_USB_TIMEOUT, &ret)) {
		LOG_ERROR("bulk read failed, error");
		return ERROR_JTAG_DEVICE_ERROR;
	}
	*bytes_read = ret;

out:
#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, *bytes_read, DEBUG_TYPE_READ);
#endif

	return ERROR_OK;
}

static int openjtag_buf_read(uint8_t *buf, uint32_t qty, uint32_t *bytes_read)
{
	switch (openjtag_variant) {
	case OPENJTAG_VARIANT_CY7C65215:
		return openjtag_buf_read_cy7c65215(buf, qty, bytes_read);
	default:
		return openjtag_buf_read_standard(buf, qty, bytes_read);
	}
}

static int openjtag_sendcommand(uint8_t cmd)
{
	uint32_t written;
	return openjtag_buf_write(&cmd, 1, &written);
}

static int openjtag_speed(int speed)
{
	int clockcmd;
	switch (speed) {
		case 48000:
			clockcmd = 0x00;
			break;
		case 24000:
			clockcmd = 0x20;
			break;
		case 12000:
			clockcmd = 0x40;
			break;
		case 6000:
			clockcmd = 0x60;
			break;
		case 3000:
			clockcmd = 0x80;
			break;
		case 1500:
			clockcmd = 0xA0;
			break;
		case 750:
			clockcmd = 0xC0;
			break;
		case 375:
			clockcmd = 0xE0;
			break;
		default:
			clockcmd = 0xE0;
			LOG_WARNING("adapter speed not recognized, reverting to 375 kHz");
			break;
	}
	openjtag_sendcommand(clockcmd);

	return ERROR_OK;
}

static int openjtag_init_standard(void)
{
	uint8_t latency_timer;

	/* Open by device description */
	if (openjtag_device_desc == NULL) {
		LOG_WARNING("no openjtag device description specified, "
				"using default 'Open JTAG Project'");
		openjtag_device_desc = "Open JTAG Project";
	}

	if (ftdi_init(&ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	/* context, vendor id, product id, description, serial id */
	if (ftdi_usb_open_desc(&ftdic, openjtag_vid, openjtag_pid, openjtag_device_desc, NULL) < 0) {
		LOG_ERROR("unable to open ftdi device: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_usb_reset(&ftdic) < 0) {
		LOG_ERROR("unable to reset ftdi device");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_latency_timer(&ftdic, 2) < 0) {
		LOG_ERROR("unable to set latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_get_latency_timer(&ftdic, &latency_timer) < 0) {
		LOG_ERROR("unable to get latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("current latency timer: %u", latency_timer);

	ftdi_disable_bitbang(&ftdic);
	/* was (3000000 / 4) with a comment about a bug in libftdi when using high baudrate */
	if (ftdi_set_baudrate(&ftdic, 3000000) < 0) {
		LOG_ERROR("Can't set baud rate to max: %s",
			ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (ftdi_tcioflush(&ftdic) < 0) {
		LOG_ERROR("ftdi flush: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int openjtag_init_cy7c65215(void)
{
	int ret;

	usbh = NULL;
	ret = jtag_libusb_open(cy7c65215_vids, cy7c65215_pids, NULL, &usbh, NULL);
	if (ret != ERROR_OK) {
		LOG_ERROR("unable to open cy7c65215 device");
		goto err;
	}

	ret = jtag_libusb_choose_interface(usbh, &ep_in, &ep_out,
									   CY7C65215_JTAG_CLASS,
									   CY7C65215_JTAG_SUBCLASS, -1, LIBUSB_TRANSFER_TYPE_BULK);
	if (ret != ERROR_OK) {
		LOG_ERROR("unable to claim JTAG interface");
		goto err;
	}

	ret = jtag_libusb_control_transfer(usbh,
									   CY7C65215_JTAG_REQUEST,
									   CY7C65215_JTAG_ENABLE,
									   0, 0, NULL, 0, CY7C65215_USB_TIMEOUT);
	if (ret < 0) {
		LOG_ERROR("could not enable JTAG module");
		goto err;
	}

	return ERROR_OK;

err:
	if (usbh != NULL)
		jtag_libusb_close(usbh);
	return ERROR_JTAG_INIT_FAILED;
}

static int openjtag_init(void)
{
	int ret;

	usb_tx_buf_offs = 0;
	usb_rx_buf_len = 0;
	openjtag_scan_result_count = 0;

	switch (openjtag_variant) {
	case OPENJTAG_VARIANT_CY7C65215:
		ret = openjtag_init_cy7c65215();
		break;
	default:
		ret = openjtag_init_standard();
	}
	if (ret != ERROR_OK)
		return ret;

	openjtag_speed(375); /* Start at slowest adapter speed */
	openjtag_sendcommand(0x75); /* MSB */

	return ERROR_OK;
}

static int openjtag_quit_standard(void)
{
	ftdi_usb_close(&ftdic);
	ftdi_deinit(&ftdic);

	return ERROR_OK;
}

static int openjtag_quit_cy7c65215(void)
{
	int ret;

	ret = jtag_libusb_control_transfer(usbh,
									   CY7C65215_JTAG_REQUEST,
									   CY7C65215_JTAG_DISABLE,
									   0, 0, NULL, 0, CY7C65215_USB_TIMEOUT);
	if (ret < 0)
		LOG_WARNING("could not disable JTAG module");

	jtag_libusb_close(usbh);

	return ERROR_OK;
}

static int openjtag_quit(void)
{
	switch (openjtag_variant) {
	case OPENJTAG_VARIANT_CY7C65215:
		return openjtag_quit_cy7c65215();
	default:
		return openjtag_quit_standard();
	}
}

static void openjtag_write_tap_buffer(void)
{
	uint32_t written;

	openjtag_buf_write(usb_tx_buf, usb_tx_buf_offs, &written);
	openjtag_buf_read(usb_rx_buf, usb_tx_buf_offs, &usb_rx_buf_len);

	usb_tx_buf_offs = 0;
}

static int openjtag_execute_tap_queue(void)
{
	openjtag_write_tap_buffer();

	int res_count = 0;

	if (openjtag_scan_result_count && usb_rx_buf_len) {

		int count;
		int rx_offs = 0;
		int len;

		/* for every pending result */
		while (res_count < openjtag_scan_result_count) {

			/* get sent bits */
			len = openjtag_scan_result_buffer[res_count].bits;

			count = 0;

			uint8_t *buffer = openjtag_scan_result_buffer[res_count].buffer;

			while (len > 0) {
				if (len <= 8 && openjtag_variant != OPENJTAG_VARIANT_CY7C65215) {
					LOG_DEBUG_IO("bits < 8 buf = 0x%X, will be 0x%X",
						usb_rx_buf[rx_offs], usb_rx_buf[rx_offs] >> (8 - len));
					buffer[count] = usb_rx_buf[rx_offs] >> (8 - len);
					len = 0;
				} else {
					buffer[count] = usb_rx_buf[rx_offs];
					len -= 8;
				}

				rx_offs++;
				count++;
			}

#ifdef _DEBUG_USB_COMMS_
			openjtag_debug_buffer(buffer,
				DIV_ROUND_UP(openjtag_scan_result_buffer[res_count].bits, 8), DEBUG_TYPE_OCD_READ);
#endif
			jtag_read_buffer(buffer, openjtag_scan_result_buffer[res_count].command);

			free(openjtag_scan_result_buffer[res_count].buffer);

			res_count++;
		}
	}

	openjtag_scan_result_count = 0;

	return ERROR_OK;
}

static void openjtag_add_byte(char buf)
{

	if (usb_tx_buf_offs == OPENJTAG_BUFFER_SIZE) {
		LOG_DEBUG_IO("Forcing execute_tap_queue");
		LOG_DEBUG_IO("TX Buff offs=%d", usb_tx_buf_offs);
		openjtag_execute_tap_queue();
	}

	usb_tx_buf[usb_tx_buf_offs] = buf;
	usb_tx_buf_offs++;
}

static void openjtag_add_scan(uint8_t *buffer, int length, struct scan_command *scan_cmd)
{

	/* Ensure space to send long chains */
	/* We add two byte for each eight (or less) bits, one for command, one for data */
	if (usb_tx_buf_offs + (DIV_ROUND_UP(length, 8) * 2) >= OPENJTAG_BUFFER_SIZE) {
		LOG_DEBUG_IO("Forcing execute_tap_queue from scan");
		LOG_DEBUG_IO("TX Buff offs=%d len=%d", usb_tx_buf_offs, DIV_ROUND_UP(length, 8) * 2);
		openjtag_execute_tap_queue();
	}

	openjtag_scan_result_buffer[openjtag_scan_result_count].bits = length;
	openjtag_scan_result_buffer[openjtag_scan_result_count].command = scan_cmd;
	openjtag_scan_result_buffer[openjtag_scan_result_count].buffer = buffer;

	uint8_t command;
	uint8_t bits;
	int count = 0;
	while (length) {

		/* write command */
		command = 6;

		/* last bits? */
		if (length <= 8) {
			/* tms high */
			command |= (1 << 4);

			/* bits to transfer */
			bits = (length - 1);
			command |= bits << 5;
			length = 0;
		} else {
			/* whole byte */

			/* bits to transfer */
			command |= (7 << 5);
			length -= 8;
		}

		openjtag_add_byte(command);
		openjtag_add_byte(buffer[count]);
		count++;
	}

	openjtag_scan_result_count++;
}

static void openjtag_execute_reset(struct jtag_command *cmd)
{

	LOG_DEBUG_IO("reset trst: %i srst %i",
			cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	uint8_t buf = 0x00;

	if (cmd->cmd.reset->trst) {
		buf = 0x03;
	} else {
		buf |= 0x04;
		buf |= 0x05 << 4;
	}

	openjtag_add_byte(buf);
}

static void openjtag_execute_sleep(struct jtag_command *cmd)
{
	jtag_sleep(cmd->cmd.sleep->us);
}

static void openjtag_set_state(uint8_t openocd_state)
{
	uint8_t state = openjtag_get_tap_state(openocd_state);

	uint8_t buf = 0;
	buf = 0x01;
	buf |= state << 4;

	openjtag_add_byte(buf);
}

static void openjtag_execute_statemove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("state move to %i", cmd->cmd.statemove->end_state);

	tap_set_end_state(cmd->cmd.statemove->end_state);

	openjtag_set_state(cmd->cmd.statemove->end_state);

	tap_set_state(tap_get_end_state());
}


static void openjtag_execute_scan(struct jtag_command *cmd)
{

	int scan_size, old_state;
	uint8_t *buffer;

	LOG_DEBUG_IO("scan ends in %s", tap_state_name(cmd->cmd.scan->end_state));

	/* get scan info */
	tap_set_end_state(cmd->cmd.scan->end_state);
	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);

#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buffer, (scan_size + 7) / 8, DEBUG_TYPE_BUFFER);
#endif
	/* set state */
	old_state = tap_get_end_state();
	openjtag_set_state(cmd->cmd.scan->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
	tap_set_state(cmd->cmd.scan->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
	tap_set_end_state(old_state);

	openjtag_add_scan(buffer, scan_size, cmd->cmd.scan);

	openjtag_set_state(cmd->cmd.scan->ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);
	tap_set_state(cmd->cmd.scan->ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state()) {
		openjtag_set_state(tap_get_end_state());
		tap_set_state(tap_get_end_state());
	}
}

static void openjtag_execute_runtest(struct jtag_command *cmd)
{

	tap_state_t end_state = cmd->cmd.runtest->end_state;
	tap_set_end_state(end_state);

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		openjtag_set_state(TAP_IDLE);
		tap_set_state(TAP_IDLE);
	}

	if (cmd->cmd.runtest->num_cycles > 16)
		LOG_WARNING("num_cycles > 16 on run test");

	if (openjtag_variant != OPENJTAG_VARIANT_CY7C65215 ||
		cmd->cmd.runtest->num_cycles) {
		uint8_t command;
		command = 7;
		command |= ((cmd->cmd.runtest->num_cycles - 1) & 0x0F) << 4;

		openjtag_add_byte(command);
	}

	tap_set_end_state(end_state);
	if (tap_get_end_state() != tap_get_state()) {
		openjtag_set_state(end_state);
		tap_set_state(end_state);
	}
}

static void openjtag_execute_command(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("openjtag_execute_command %i", cmd->type);
	switch (cmd->type) {
	case JTAG_RESET:
			openjtag_execute_reset(cmd);
			break;
	case JTAG_SLEEP:
			openjtag_execute_sleep(cmd);
			break;
	case JTAG_TLR_RESET:
			openjtag_execute_statemove(cmd);
			break;
	case JTAG_SCAN:
			openjtag_execute_scan(cmd);
			break;
	case JTAG_RUNTEST:
			openjtag_execute_runtest(cmd);
			break;
	case JTAG_PATHMOVE:
		/* jlink_execute_pathmove(cmd); break; */
	default:
		LOG_ERROR("BUG: unknown Open JTAG command type encountered");
		exit(-1);
	}
}

static int openjtag_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		openjtag_execute_command(cmd);
		cmd = cmd->next;
	}

	return openjtag_execute_tap_queue();
}

static int openjtag_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}

static int openjtag_khz(int khz, int *jtag_speed)
{

	if (khz >= 48000)
		*jtag_speed = 48000;
	else if (khz >= 24000)
		*jtag_speed = 24000;
	else if (khz >= 12000)
		*jtag_speed = 12000;
	else if (khz >= 6000)
		*jtag_speed = 6000;
	else if (khz >= 3000)
		*jtag_speed = 3000;
	else if (khz >= 1500)
		*jtag_speed = 1500;
	else if (khz >= 750)
		*jtag_speed = 750;
	else
		*jtag_speed = 375;

	return ERROR_OK;
}

COMMAND_HANDLER(openjtag_handle_device_desc_command)
{
	if (CMD_ARGC == 1)
		openjtag_device_desc = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("require exactly one argument to "
				  "openjtag_device_desc <description>");
	return ERROR_OK;
}

COMMAND_HANDLER(openjtag_handle_variant_command)
{
	if (CMD_ARGC == 1) {
		const char * const *name = openjtag_variant_names;
		int variant = 0;
		for (; *name; name++, variant++) {
			if (strcasecmp(CMD_ARGV[0], *name) == 0) {
				openjtag_variant = variant;
				return ERROR_OK;
			}
		}
		LOG_ERROR("unknown openjtag variant '%s'", CMD_ARGV[0]);
	} else {
		LOG_ERROR("require exactly one argument to "
				"openjtag_variant <variant>");
	}
	return ERROR_OK;
}

static const struct command_registration openjtag_command_handlers[] = {
	{
		.name = "openjtag_device_desc",
		.handler = openjtag_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the OpenJTAG",
		.usage = "description-string",
	},
	{
		.name = "openjtag_variant",
		.handler = openjtag_handle_variant_command,
		.mode = COMMAND_CONFIG,
		.help = "set the OpenJTAG variant",
		.usage = "variant-string",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface openjtag_interface = {
	.execute_queue = openjtag_execute_queue,
};

struct adapter_driver openjtag_adapter_driver = {
	.name = "openjtag",
	.transports = jtag_only,
	.commands = openjtag_command_handlers,

	.init = openjtag_init,
	.quit = openjtag_quit,
	.speed = openjtag_speed,
	.khz = openjtag_khz,
	.speed_div = openjtag_speed_div,

	.jtag_ops = &openjtag_interface,
};
