/***************************************************************************
 *   Copyright (C) 2007 by Benedikt Sauter                                 *
 *   sauter@ixbat.de                                                       *
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

/*
 * This file is based on Dominic Rath's amt_jtagaccel.c.
 *
 * usbprog is a free programming adapter. You can easily install
 * different firmware versions from an "online pool" over USB.
 * The adapter can be used for programming and debugging AVR and ARM
 * processors, as USB to RS232 converter, as JTAG interface or as
 * simple I/O interface (5 lines).
 *
 * http://www.embedded-projects.net/usbprog
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "libusb_helper.h"

#define VID 0x1781
#define PID 0x0c63

/* Pins at usbprog */
#define TDO_BIT                 0
#define TDI_BIT                 3
#define TCK_BIT                 2
#define TMS_BIT                 1

static void usbprog_end_state(tap_state_t state);
static void usbprog_state_move(void);
static void usbprog_path_move(struct pathmove_command *cmd);
static void usbprog_runtest(int num_cycles);
static void usbprog_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size);

#define UNKNOWN_COMMAND 0x00
#define PORT_DIRECTION  0x01
#define PORT_SET                0x02
#define PORT_GET                0x03
#define PORT_SETBIT             0x04
#define PORT_GETBIT             0x05
#define WRITE_TDI               0x06
#define READ_TDO                0x07
#define WRITE_AND_READ  0x08
#define WRITE_TMS               0x09
#define WRITE_TMS_CHAIN 0x0A

struct usbprog_jtag {
	struct libusb_device_handle *usb_handle;
};

static struct usbprog_jtag *usbprog_jtag_handle;

static struct usbprog_jtag *usbprog_jtag_open(void);
/* static void usbprog_jtag_close(struct usbprog_jtag *usbprog_jtag); */
static void usbprog_jtag_init(struct usbprog_jtag *usbprog_jtag);
static unsigned char usbprog_jtag_message(struct usbprog_jtag *usbprog_jtag, char *msg, int msglen);

static void usbprog_jtag_read_tdo(struct usbprog_jtag *usbprog_jtag, char *buffer, int size);
static void usbprog_jtag_write_tdi(struct usbprog_jtag *usbprog_jtag, char *buffer, int size);
static void usbprog_jtag_write_and_read(struct usbprog_jtag *usbprog_jtag, char *buffer, int size);
static void usbprog_jtag_write_tms(struct usbprog_jtag *usbprog_jtag, char tms_scan);

static char tms_chain[64];
static int tms_chain_index;

static void usbprog_jtag_tms_collect(char tms_scan);
static void usbprog_jtag_tms_send(struct usbprog_jtag *usbprog_jtag);

static void usbprog_write(int tck, int tms, int tdi);
static void usbprog_reset(int trst, int srst);

static void usbprog_jtag_set_direction(struct usbprog_jtag *usbprog_jtag, unsigned char direction);
static void usbprog_jtag_write_slice(struct usbprog_jtag *usbprog_jtag, unsigned char value);
/* static unsigned char usbprog_jtag_get_port(struct usbprog_jtag *usbprog_jtag); */
static void usbprog_jtag_set_bit(struct usbprog_jtag *usbprog_jtag, int bit, int value);
/* static int usbprog_jtag_get_bit(struct usbprog_jtag *usbprog_jtag, int bit); */

static int usbprog_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	while (cmd) {
		switch (cmd->type) {
		case JTAG_RESET:
			LOG_DEBUG_IO("reset trst: %i srst %i",
					cmd->cmd.reset->trst,
					cmd->cmd.reset->srst);
			if (cmd->cmd.reset->trst == 1)
				tap_set_state(TAP_RESET);
			usbprog_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			LOG_DEBUG_IO("runtest %i cycles, end in %i",
					cmd->cmd.runtest->num_cycles,
					cmd->cmd.runtest->end_state);
			usbprog_end_state(cmd->cmd.runtest->end_state);
			usbprog_runtest(cmd->cmd.runtest->num_cycles);
			break;
		case JTAG_TLR_RESET:
			LOG_DEBUG_IO("statemove end in %i", cmd->cmd.statemove->end_state);
			usbprog_end_state(cmd->cmd.statemove->end_state);
			usbprog_state_move();
			break;
		case JTAG_PATHMOVE:
			LOG_DEBUG_IO("pathmove: %i states, end in %i",
					cmd->cmd.pathmove->num_states,
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
			usbprog_path_move(cmd->cmd.pathmove);
			break;
		case JTAG_SCAN:
			LOG_DEBUG_IO("scan end in %i", cmd->cmd.scan->end_state);
			usbprog_end_state(cmd->cmd.scan->end_state);
			scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
			type = jtag_scan_type(cmd->cmd.scan);
			usbprog_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
			if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
				return ERROR_JTAG_QUEUE_FAILED;
			free(buffer);
			break;
		case JTAG_SLEEP:
			LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
		}

		cmd = cmd->next;
	}

	return ERROR_OK;
}

static int usbprog_init(void)
{
	usbprog_jtag_handle = usbprog_jtag_open();

	tms_chain_index = 0;
	if (usbprog_jtag_handle == 0) {
		LOG_ERROR("Can't find USB JTAG Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("USB JTAG Interface ready!");

	usbprog_jtag_init(usbprog_jtag_handle);
	usbprog_reset(0, 0);
	usbprog_write(0, 0, 0);

	return ERROR_OK;
}

static int usbprog_quit(void)
{
	return ERROR_OK;
}

/*************** jtag execute commands **********************/
static void usbprog_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void usbprog_state_move(void)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());

	usbprog_jtag_write_tms(usbprog_jtag_handle, (char)tms_scan);

	tap_set_state(tap_get_end_state());
}

static void usbprog_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;

	/* There may be queued transitions, and before following a specified
	   path, we must flush those queued transitions */
	usbprog_jtag_tms_send(usbprog_jtag_handle);

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count]) {
			/* LOG_INFO("1"); */
			usbprog_write(0, 0, 0);
			usbprog_write(1, 0, 0);
		} else if (tap_state_transition(tap_get_state(),
				   true) == cmd->path[state_count]) {
			/* LOG_INFO("2"); */
			usbprog_write(0, 1, 0);
			usbprog_write(1, 1, 0);
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	tap_set_end_state(tap_get_state());
}

static void usbprog_runtest(int num_cycles)
{
	int i;

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		usbprog_end_state(TAP_IDLE);
		usbprog_state_move();
	}

	/* execute num_cycles */
	if (num_cycles > 0) {
		usbprog_jtag_tms_send(usbprog_jtag_handle);
		usbprog_write(0, 0, 0);
	} else {
		usbprog_jtag_tms_send(usbprog_jtag_handle);
		/* LOG_INFO("NUM CYCLES %i",num_cycles); */
	}

	for (i = 0; i < num_cycles; i++) {
		usbprog_write(1, 0, 0);
		usbprog_write(0, 0, 0);
	}

	LOG_DEBUG_IO("runtest: cur_state %s end_state %s", tap_state_name(
			tap_get_state()), tap_state_name(tap_get_end_state()));

	/* finish in end_state */
	/*
	usbprog_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		usbprog_state_move();
	*/
}

static void usbprog_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();

	if (ir_scan)
		usbprog_end_state(TAP_IRSHIFT);
	else
		usbprog_end_state(TAP_DRSHIFT);

	/* Only move if we're not already there */
	if (tap_get_state() != tap_get_end_state())
		usbprog_state_move();

	usbprog_end_state(saved_end_state);

	usbprog_jtag_tms_send(usbprog_jtag_handle);

	void (*f)(struct usbprog_jtag *usbprog_jtag, char *buffer_local, int size);
	switch (type) {
		case SCAN_OUT:
			f = &usbprog_jtag_write_tdi;
			break;
		case SCAN_IN:
			f = &usbprog_jtag_read_tdo;
			break;
		case SCAN_IO:
			f = &usbprog_jtag_write_and_read;
			break;
		default:
			LOG_ERROR("unknown scan type: %i", type);
			exit(-1);
	}
	f(usbprog_jtag_handle, (char *)buffer, scan_size);

	/* The adapter does the transition to PAUSE internally */
	if (ir_scan)
		tap_set_state(TAP_IRPAUSE);
	else
		tap_set_state(TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
		usbprog_state_move();
}

/*************** jtag wrapper functions *********************/

static void usbprog_write(int tck, int tms, int tdi)
{
	unsigned char output_value = 0x00;

	if (tms)
		output_value |= (1 << TMS_BIT);
	if (tdi)
		output_value |= (1 << TDI_BIT);
	if (tck)
		output_value |= (1 << TCK_BIT);

	usbprog_jtag_write_slice(usbprog_jtag_handle, output_value);
}

/* (1) assert or (0) deassert reset lines */
static void usbprog_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (trst)
		usbprog_jtag_set_bit(usbprog_jtag_handle, 5, 0);
	else
		usbprog_jtag_set_bit(usbprog_jtag_handle, 5, 1);

	if (srst)
		usbprog_jtag_set_bit(usbprog_jtag_handle, 4, 0);
	else
		usbprog_jtag_set_bit(usbprog_jtag_handle, 4, 1);
}

/*************** jtag lowlevel functions ********************/

struct usb_bus *busses;

struct usbprog_jtag *usbprog_jtag_open(void)
{
	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };
	struct libusb_device_handle *dev;

	if (jtag_libusb_open(vids, pids, NULL, &dev, NULL) != ERROR_OK)
		return NULL;

	struct usbprog_jtag *tmp = malloc(sizeof(struct usbprog_jtag));
	tmp->usb_handle = dev;

	libusb_set_configuration(dev, 1);
	libusb_claim_interface(dev, 0);
	libusb_set_interface_alt_setting(dev, 0, 0);

	return tmp;
}

#if 0
static void usbprog_jtag_close(struct usbprog_jtag *usbprog_jtag)
{
	libusb_close(usbprog_jtag->usb_handle);
	free(usbprog_jtag);
}
#endif

static unsigned char usbprog_jtag_message(struct usbprog_jtag *usbprog_jtag, char *msg, int msglen)
{
	int transferred;

	int res = jtag_libusb_bulk_write(usbprog_jtag->usb_handle, 3, msg, msglen, 100, &transferred);
	if ((msg[0] == 2) || (msg[0] == 1) || (msg[0] == 4) || (msg[0] == 0) ||
	    (msg[0] == 6) || (msg[0] == 0x0A) || (msg[0] == 9))
		return 1;
	if (res == ERROR_OK && transferred == msglen) {
		/* LOG_INFO("HALLLLOOO %i",(int)msg[0]); */
		res = jtag_libusb_bulk_read(usbprog_jtag->usb_handle, 0x82, msg, 2, 100, &transferred);
		if (res == ERROR_OK && transferred > 0)
			return (unsigned char)msg[1];
		else
			return -1;
	} else
		return -1;
	return 0;
}

static void usbprog_jtag_init(struct usbprog_jtag *usbprog_jtag)
{
	usbprog_jtag_set_direction(usbprog_jtag, 0xFE);
}

static void usbprog_jtag_write_and_read(struct usbprog_jtag *usbprog_jtag, char *buffer, int size)
{
	char tmp[64];	/* fastest packet size for usb controller */
	int send_bits, bufindex = 0, fillindex = 0, i, loops;

	char swap;
	/* 61 byte can be transferred (488 bit) */

	while (size > 0) {
		if (size > 488) {
			send_bits = 488;
			size = size - 488;
			loops = 61;
		} else {
			send_bits = size;
			loops = size / 8;
			loops++;
			size = 0;
		}
		tmp[0] = WRITE_AND_READ;
		tmp[1] = (char)(send_bits >> 8);	/* high */
		tmp[2] = (char)(send_bits);			/* low */

		for (i = 0; i < loops; i++) {
			tmp[3 + i] = buffer[bufindex];
			bufindex++;
		}

		int transferred;
		int res = jtag_libusb_bulk_write(usbprog_jtag->usb_handle, 3, tmp, 64, 1000, &transferred);
		if (res == ERROR_OK && transferred == 64) {
			/* LOG_INFO("HALLLLOOO2 %i",(int)tmp[0]); */
			usleep(1);
			int timeout = 0;
			while (jtag_libusb_bulk_read(usbprog_jtag->usb_handle, 0x82, tmp, 64, 1000, &transferred) != ERROR_OK) {
				timeout++;
				if (timeout > 10)
					break;
			}

			for (i = 0; i < loops; i++) {
				swap = tmp[3 + i];
				buffer[fillindex++] = swap;
			}
		}
	}
}

static void usbprog_jtag_read_tdo(struct usbprog_jtag *usbprog_jtag, char *buffer, int size)
{
	char tmp[64];	/* fastest packet size for usb controller */
	int send_bits, fillindex = 0, i, loops;

	char swap;
	/* 61 byte can be transferred (488 bit) */

	while (size > 0) {
		if (size > 488) {
			send_bits = 488;
			size = size - 488;
			loops = 61;
		} else {
			send_bits = size;
			loops = size / 8;
			loops++;
			size = 0;
		}
		tmp[0] = WRITE_AND_READ;
		tmp[1] = (char)(send_bits >> 8);	/* high */
		tmp[2] = (char)(send_bits);			/* low */

		int transferred;
		jtag_libusb_bulk_write(usbprog_jtag->usb_handle, 3, tmp, 3, 1000, &transferred);

		/* LOG_INFO("HALLLLOOO3 %i",(int)tmp[0]); */
		int timeout = 0;
		usleep(1);
		while (jtag_libusb_bulk_read(usbprog_jtag->usb_handle, 0x82, tmp, 64, 10, &transferred) != ERROR_OK) {
			timeout++;
			if (timeout > 10)
				break;
		}

		for (i = 0; i < loops; i++) {
			swap = tmp[3 + i];
			buffer[fillindex++] = swap;
		}
	}
}

static void usbprog_jtag_write_tdi(struct usbprog_jtag *usbprog_jtag, char *buffer, int size)
{
	char tmp[64];	/* fastest packet size for usb controller */
	int send_bits, bufindex = 0, i, loops;

	/* 61 byte can be transferred (488 bit) */
	while (size > 0) {
		if (size > 488) {
			send_bits = 488;
			size = size - 488;
			loops = 61;
		} else {
			send_bits = size;
			loops = size/8;
			/* if (loops == 0) */
			loops++;
			size = 0;
		}
		tmp[0] = WRITE_TDI;
		tmp[1] = (char)(send_bits >> 8);	/* high */
		tmp[2] = (char)(send_bits);			/* low */

		for (i = 0; i < loops; i++) {
			tmp[3 + i] = buffer[bufindex];
			bufindex++;
		}
		int transferred;
		jtag_libusb_bulk_write(usbprog_jtag->usb_handle, 3, tmp, 64, 1000, &transferred);
	}
}

static void usbprog_jtag_write_tms(struct usbprog_jtag *usbprog_jtag, char tms_scan)
{
	usbprog_jtag_tms_collect(tms_scan);
}

static void usbprog_jtag_set_direction(struct usbprog_jtag *usbprog_jtag, unsigned char direction)
{
	char tmp[2];
	tmp[0] = PORT_DIRECTION;
	tmp[1] = (char)direction;
	usbprog_jtag_message(usbprog_jtag, tmp, 2);
}

static void usbprog_jtag_write_slice(struct usbprog_jtag *usbprog_jtag, unsigned char value)
{
	char tmp[2];
	tmp[0] = PORT_SET;
	tmp[1] = (char)value;
	usbprog_jtag_message(usbprog_jtag, tmp, 2);
}

#if 0
static unsigned char usbprog_jtag_get_port(struct usbprog_jtag *usbprog_jtag)
{
	char tmp[2];
	tmp[0] = PORT_GET;
	tmp[1] = 0x00;
	return usbprog_jtag_message(usbprog_jtag, tmp, 2);
}
#endif

static void usbprog_jtag_set_bit(struct usbprog_jtag *usbprog_jtag, int bit, int value)
{
	char tmp[3];
	tmp[0] = PORT_SETBIT;
	tmp[1] = (char)bit;
	if (value == 1)
		tmp[2] = 0x01;
	else
		tmp[2] = 0x00;
	usbprog_jtag_message(usbprog_jtag, tmp, 3);
}

#if 0
static int usbprog_jtag_get_bit(struct usbprog_jtag *usbprog_jtag, int bit)
{
	char tmp[2];
	tmp[0] = PORT_GETBIT;
	tmp[1] = (char)bit;

	if (usbprog_jtag_message(usbprog_jtag, tmp, 2) > 0)
		return 1;
	else
		return 0;
}
#endif

static void usbprog_jtag_tms_collect(char tms_scan)
{
	tms_chain[tms_chain_index] = tms_scan;
	tms_chain_index++;
}

static void usbprog_jtag_tms_send(struct usbprog_jtag *usbprog_jtag)
{
	/* LOG_INFO("TMS SEND"); */
	if (tms_chain_index > 0) {
		char tmp[tms_chain_index + 2];
		tmp[0] = WRITE_TMS_CHAIN;
		tmp[1] = (char)(tms_chain_index);
		for (int i = 0; i < tms_chain_index + 1; i++)
			tmp[2 + i] = tms_chain[i];
		int transferred;
		jtag_libusb_bulk_write(usbprog_jtag->usb_handle, 3, tmp, tms_chain_index + 2, 1000, &transferred);
		tms_chain_index = 0;
	}
}

static struct jtag_interface usbprog_interface = {
	.execute_queue = usbprog_execute_queue,
};

struct adapter_driver usbprog_adapter_driver = {
	.name = "usbprog",
	.transports = jtag_only,

	.init = usbprog_init,
	.quit = usbprog_quit,

	.jtag_ops = &usbprog_interface,
};
