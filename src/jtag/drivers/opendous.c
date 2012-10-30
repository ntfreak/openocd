/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2009 by Cahya Wirawan <cahya@gmx.at>                    *
 *   Based on opendous driver by Vladimir Fonov                            *
 *                                                                         *
 *   Copyright (C) 2009 by Vladimir Fonov <vladimir.fonov@gmai.com>        *
 *   Based on J-link driver by  Juergen Stuber                             *
 *                                                                         *
 *   Copyright (C) 2007 by Juergen Stuber <juergen@jstuber.net>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "libusb_common.h"
#include <string.h>
#include <sys/timeb.h>
#include <time.h>

#define ESTICK_VID 0x1781
#define ESTICK_PID 0xC0C0

#define OPENDOUS_VID 0x03EB
#define OPENDOUS_PID 0x204F

/* pid could be specified at runtime */
static uint16_t vids[] = { OPENDOUS_VID, ESTICK_VID, 0 };
static uint16_t pids[] = { OPENDOUS_PID, ESTICK_PID, 0 };

#define OPENDOUS_WRITE_ENDPOINT   0x02
#define OPENDOUS_READ_ENDPOINT    0x81

static unsigned int opendous_hw_jtag_version = 1;

#define OPENDOUS_USB_TIMEOUT      1000

#define OPENDOUS_USB_BUFFER_SIZE  360
#define OPENDOUS_IN_BUFFER_SIZE   (OPENDOUS_USB_BUFFER_SIZE)
#define OPENDOUS_OUT_BUFFER_SIZE  (OPENDOUS_USB_BUFFER_SIZE)

/* Global USB buffers */
static uint8_t usb_in_buffer[OPENDOUS_IN_BUFFER_SIZE];
static uint8_t usb_out_buffer[OPENDOUS_OUT_BUFFER_SIZE];

/* Constants for OPENDOUS command */

#define OPENDOUS_MAX_SPEED			66
#define OPENDOUS_MAX_TAP_TRANSMIT	350	/* even number is easier to handle */
#define OPENDOUS_MAX_INPUT_DATA		(OPENDOUS_MAX_TAP_TRANSMIT*4)

#define OPENDOUS_TAP_BUFFER_SIZE 65536

#define MAX_PENDING_SCAN_RESULTS (OPENDOUS_MAX_INPUT_DATA)

/* JTAG usb commands */
#define JTAG_CMD_TAP_OUTPUT		0x0
#define JTAG_CMD_SET_TRST		0x1
#define JTAG_CMD_SET_SRST		0x2
#define JTAG_CMD_READ_INPUT		0x3
#define JTAG_CMD_TAP_OUTPUT_EMU	0x4
#define JTAG_CMD_SET_DELAY		0x5
#define JTAG_CMD_SET_SRST_TRST	0x6
#define JTAG_CMD_READ_CONFIG	0x7

/* External interface functions */
static int opendous_execute_queue(void);
static int opendous_speed(int speed);
static int opendous_speed_div(int speed, int *khz);
static int opendous_khz(int khz, int *jtag_speed);
static int opendous_init(void);
static int opendous_quit(void);

/* Queue command functions */
static void opendous_end_state(tap_state_t state);
static void opendous_state_move(void);
static void opendous_path_move(int num_states, tap_state_t *path);
static void opendous_runtest(int num_cycles);
static void opendous_scan(int ir_scan, enum scan_type type, uint8_t *buffer,
		int scan_size, struct scan_command *command);
static void opendous_reset(int trst, int srst);
static void opendous_simple_command(uint8_t command, uint8_t _data);
static int opendous_get_status(void);

/* opendous tap buffer functions */
static void opendous_tap_init(void);
static int opendous_tap_execute(void);
static void opendous_tap_ensure_space(int scans, int bits);
static void opendous_tap_append_step(int tms, int tdi);
static void opendous_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command);

/* opendous lowlevel functions */
struct opendous_jtag {
	struct jtag_libusb_device_handle *usb_handle;
};

static struct opendous_jtag *opendous_usb_open(void);
static void opendous_usb_close(struct opendous_jtag *opendous_jtag);
static int opendous_usb_message(struct opendous_jtag *opendous_jtag, int out_length, int in_length);
static int opendous_usb_write(struct opendous_jtag *opendous_jtag, int out_length);
static int opendous_usb_read(struct opendous_jtag *opendous_jtag);

/* helper functions */
int opendous_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
char time_str[50];
static void opendous_debug_buffer(uint8_t *buffer, int length);
char *opendous_get_time(char *);
#endif

static struct opendous_jtag *opendous_jtag_handle;

/***************************************************************************/
/* External interface implementation */

COMMAND_HANDLER(opendous_handle_opendous_info_command)
{
	if (opendous_get_version_info() == ERROR_OK) {
		/* attempt to get status */
		opendous_get_status();
	}

	return ERROR_OK;
}

COMMAND_HANDLER(opendous_handle_opendous_hw_jtag_command)
{
	switch (CMD_ARGC) {
		case 0:
			command_print(CMD_CTX, "opendous hw jtag  %i", opendous_hw_jtag_version);
			break;

		case 1: {
			int request_version = atoi(CMD_ARGV[0]);
			switch (request_version) {
				case 2:
				case 3:
					opendous_hw_jtag_version = request_version;
					break;

			default:
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			break;
		}

		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static const struct command_registration opendous_command_handlers[] = {
	{
		.name = "opendous_info",
		.handler = &opendous_handle_opendous_info_command,
		.mode = COMMAND_EXEC,
		.help = "show opendous info",
	},
	{
		.name = "opendous_hw_jtag",
		.handler = &opendous_handle_opendous_hw_jtag_command,
		.mode = COMMAND_EXEC,
		.help = "access opendous HW JTAG command version",
		.usage = "[2|3]",
	},
	COMMAND_REGISTRATION_DONE
};

const struct adapter_driver opendous_driver = {
	.name = "opendous",
	.commands = opendous_command_handlers,
	.execute_queue = opendous_execute_queue,
	.speed = opendous_speed,
	.speed_div = opendous_speed_div,
	.khz = opendous_khz,
	.init = opendous_init,
	.quit = opendous_quit,
};

static int opendous_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	while (cmd != NULL) {
		switch (cmd->type) {
			case JTAG_RUNTEST:
				DEBUG_JTAG_IO("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, \
					cmd->cmd.runtest->end_state);

				if (cmd->cmd.runtest->end_state != -1)
					opendous_end_state(cmd->cmd.runtest->end_state);
				opendous_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_TLR_RESET:
				DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

				if (cmd->cmd.statemove->end_state != -1)
					opendous_end_state(cmd->cmd.statemove->end_state);
				opendous_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %i", \
					cmd->cmd.pathmove->num_states, \
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

				opendous_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

				if (cmd->cmd.scan->end_state != -1)
					opendous_end_state(cmd->cmd.scan->end_state);

				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
				opendous_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
				type = jtag_scan_type(cmd->cmd.scan);
				opendous_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size, cmd->cmd.scan);
				break;

			case JTAG_RESET:
				DEBUG_JTAG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

				opendous_tap_execute();

				if (cmd->cmd.reset->trst == 1)
					tap_set_state(TAP_RESET);
				opendous_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_SLEEP:
				DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
				opendous_tap_execute();
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}
	return opendous_tap_execute();
}

/* Sets speed in kHz. */
static int opendous_speed(int speed)
{
	if (speed <= OPENDOUS_MAX_SPEED) {
		/* one day... */
		return ERROR_OK;
	} else
		LOG_INFO("Requested speed %dkHz exceeds maximum of %dkHz, ignored", speed, OPENDOUS_MAX_SPEED);

	return ERROR_OK;
}

static int opendous_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}

static int opendous_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	/* TODO: convert this into delay value for opendous */

	return ERROR_OK;
}

static int opendous_init(void)
{
	int check_cnt;

	opendous_jtag_handle = opendous_usb_open();

	if (opendous_jtag_handle == 0) {
		LOG_ERROR("Cannot find opendous Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	check_cnt = 0;
	while (check_cnt < 3) {
		if (opendous_get_version_info() == ERROR_OK) {
			/* attempt to get status */
			opendous_get_status();
			break;
		}

		check_cnt++;
	}

	LOG_INFO("opendous JTAG Interface ready");

	opendous_reset(0, 0);
	opendous_tap_init();

	return ERROR_OK;
}

static int opendous_quit(void)
{
	opendous_usb_close(opendous_jtag_handle);
	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

void opendous_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/* Goes to the end state. */
void opendous_state_move(void)
{
	int i;
	int tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_scan_bits; i++) {
		tms = (tms_scan >> i) & 1;
		opendous_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

void opendous_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			opendous_tap_append_step(0, 0);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			opendous_tap_append_step(1, 0);
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
					tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

void opendous_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		opendous_end_state(TAP_IDLE);
		opendous_state_move();
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
		opendous_tap_append_step(0, 0);

	/* finish in end_state */
	opendous_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		opendous_state_move();
}

void opendous_scan(int ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	opendous_tap_ensure_space(1, scan_size + 8);

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	opendous_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	opendous_state_move();
	opendous_end_state(saved_end_state);

	/* Scan */
	opendous_tap_append_scan(scan_size, buffer, command);

	/* We are in Exit1, go to Pause */
	opendous_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
		opendous_state_move();
}

void opendous_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	/* Signals are active low */
#if 0
	if (srst == 0)
		opendous_simple_command(JTAG_CMD_SET_SRST, 1);
	else if (srst == 1)
		opendous_simple_command(JTAG_CMD_SET_SRST, 0);

	if (trst == 0)
		opendous_simple_command(JTAG_CMD_SET_TRST, 1);
	else if (trst == 1)
		opendous_simple_command(JTAG_CMD_SET_TRST, 0);
#endif

	srst = srst ? 0 : 1;
	trst = trst ? 0 : 2;
	opendous_simple_command(JTAG_CMD_SET_SRST_TRST, srst | trst);
}

void opendous_simple_command(uint8_t command, uint8_t _data)
{
	int result;

	DEBUG_JTAG_IO("0x%02x 0x%02x", command, _data);

	usb_out_buffer[0] = 2;
	usb_out_buffer[1] = 0;
	usb_out_buffer[2] = command;
	usb_out_buffer[3] = _data;

	result = opendous_usb_message(opendous_jtag_handle, 4, 1);
	if (result != 1)
		LOG_ERROR("opendous command 0x%02x failed (%d)", command, result);
}

int opendous_get_status(void)
{
	return ERROR_OK;
}

int opendous_get_version_info(void)
{
	return ERROR_OK;
}

/***************************************************************************/
/* Estick tap functions */

static int tap_length;
static uint8_t tms_buffer[OPENDOUS_TAP_BUFFER_SIZE];
static uint8_t tdo_buffer[OPENDOUS_TAP_BUFFER_SIZE];

struct pending_scan_result {
	int first;	/* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
};

static int pending_scan_results_length;
static struct pending_scan_result pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static int last_tms;

void opendous_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

void opendous_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;

	if (scans > available_scans)
		opendous_tap_execute();
}

void opendous_tap_append_step(int tms, int tdi)
{
	last_tms = tms;
	unsigned char _tms = tms ? 1 : 0;
	unsigned char _tdi = tdi ? 1 : 0;

	int tap_index =  tap_length / 4;
	int bits  = (tap_length % 4) * 2;

	if (tap_length < OPENDOUS_TAP_BUFFER_SIZE) {
		if (!bits)
			tms_buffer[tap_index] = 0;

		tms_buffer[tap_index] |= (_tdi << bits)|(_tms << (bits + 1)) ;
		tap_length++;
	} else
		LOG_ERROR("opendous_tap_append_step, overflow");
}

void opendous_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command)
{
	DEBUG_JTAG_IO("append scan, length = %d", length);

	struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++)
		opendous_tap_append_step((i < length-1 ? 0 : 1), (buffer[i / 8] >> (i % 8)) & 1);
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
int opendous_tap_execute(void)
{
	int byte_length;
	int i, j;
	int result;

#ifdef _DEBUG_USB_COMMS_
	int byte_length_out;
#endif

	if (tap_length > 0) {

		/* memset(tdo_buffer,0,OPENDOUS_TAP_BUFFER_SIZE); */
		/* LOG_INFO("OPENDOUS tap execute %d",tap_length); */
		byte_length = (tap_length + 3) / 4;

#ifdef _DEBUG_USB_COMMS_
		byte_length_out = (tap_length + 7) / 8;
		LOG_DEBUG("opendous is sending %d bytes", byte_length);
#endif

		for (j = 0, i = 0; j <  byte_length;) {

			int receive;
			int transmit = byte_length - j;
			if (transmit > OPENDOUS_MAX_TAP_TRANSMIT) {
				transmit = OPENDOUS_MAX_TAP_TRANSMIT;
				receive = (OPENDOUS_MAX_TAP_TRANSMIT) / 2;
				usb_out_buffer[2] = JTAG_CMD_TAP_OUTPUT;
			} else {
				usb_out_buffer[2] = JTAG_CMD_TAP_OUTPUT | ((tap_length % 4) << 4);
				receive = (transmit + 1) / 2;
			}
			usb_out_buffer[0] = (transmit + 1) & 0xff;
			usb_out_buffer[1] = ((transmit + 1) >> 8) & 0xff;

			memmove(usb_out_buffer + 3, tms_buffer + j, transmit);
			result = opendous_usb_message(opendous_jtag_handle, 3 + transmit, receive);
			if (result != receive) {
				LOG_ERROR("opendous_tap_execute, wrong result %d, expected %d", result, receive);
				return ERROR_JTAG_QUEUE_FAILED;
			}

			memmove(tdo_buffer + i, usb_in_buffer, receive);
			i += receive;
			j += transmit;
		}

#ifdef _DEBUG_USB_COMMS_
		LOG_DEBUG("opendous tap result %d", byte_length_out);
		opendous_debug_buffer(tdo_buffer, byte_length_out);
#endif

		/* LOG_INFO("eStick tap execute %d",tap_length); */
		for (i = 0; i < pending_scan_results_length; i++) {

			struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[i];
			uint8_t *buffer = pending_scan_result->buffer;
			int length = pending_scan_result->length;
			int first = pending_scan_result->first;
			struct scan_command *command = pending_scan_result->command;

			/* Copy to buffer */
			buf_set_buf(tdo_buffer, first, buffer, 0, length);

			DEBUG_JTAG_IO("pending scan result, length = %d", length);

#ifdef _DEBUG_USB_COMMS_
			opendous_debug_buffer(buffer, byte_length_out);
#endif

			if (jtag_read_buffer(buffer, command) != ERROR_OK) {
				opendous_tap_init();
				return ERROR_JTAG_QUEUE_FAILED;
			}

			if (pending_scan_result->buffer != NULL)
				free(pending_scan_result->buffer);
		}

		opendous_tap_init();
	}

	return ERROR_OK;
}

/*****************************************************************************/
/* Estick USB low-level functions */

struct opendous_jtag *opendous_usb_open(void)
{
	struct opendous_jtag *result;

	struct jtag_libusb_device_handle *devh;
	if (jtag_libusb_open(vids, pids, &devh) != ERROR_OK)
		return NULL;

	jtag_libusb_set_configuration(devh, 0);
	jtag_libusb_claim_interface(devh, 0);

	result = (struct opendous_jtag *) malloc(sizeof(struct opendous_jtag));
	result->usb_handle = devh;
	return result;
}

void opendous_usb_close(struct opendous_jtag *opendous_jtag)
{
	jtag_libusb_close(opendous_jtag->usb_handle);
	free(opendous_jtag);
}

/* Send a message and receive the reply. */
int opendous_usb_message(struct opendous_jtag *opendous_jtag, int out_length, int in_length)
{
	int result;

	result = opendous_usb_write(opendous_jtag, out_length);
	if (result == out_length) {
		result = opendous_usb_read(opendous_jtag);
		if (result == in_length)
			return result;
		else {
			LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)", in_length, result);
			return -1;
		}
	} else {
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)", out_length, result);
		return -1;
	}
}

/* Write data from out_buffer to USB. */
int opendous_usb_write(struct opendous_jtag *opendous_jtag, int out_length)
{
	int result;

	if (out_length > OPENDOUS_OUT_BUFFER_SIZE) {
		LOG_ERROR("opendous_jtag_write illegal out_length=%d (max=%d)", out_length, OPENDOUS_OUT_BUFFER_SIZE);
		return -1;
	}

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB write begin", opendous_get_time(time_str));
#endif
	result = jtag_libusb_bulk_write(opendous_jtag->usb_handle, OPENDOUS_WRITE_ENDPOINT, \
		(char *)usb_out_buffer, out_length, OPENDOUS_USB_TIMEOUT);
#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB write end: %d bytes", opendous_get_time(time_str), result);
#endif

	DEBUG_JTAG_IO("opendous_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	opendous_debug_buffer(usb_out_buffer, out_length);
#endif
	return result;
}

/* Read data from USB into in_buffer. */
int opendous_usb_read(struct opendous_jtag *opendous_jtag)
{
#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB read begin", opendous_get_time(time_str));
#endif
	int result = jtag_libusb_bulk_read(opendous_jtag->usb_handle, OPENDOUS_READ_ENDPOINT,
		(char *)usb_in_buffer, OPENDOUS_IN_BUFFER_SIZE, OPENDOUS_USB_TIMEOUT);
#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB read end: %d bytes", opendous_get_time(time_str), result);
#endif
	DEBUG_JTAG_IO("opendous_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	opendous_debug_buffer(usb_in_buffer, result);
#endif
	return result;
}

#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

void opendous_debug_buffer(uint8_t *buffer, int length)
{
	char line[81];
	char s[4];
	int i;
	int j;

	for (i = 0; i < length; i += BYTES_PER_LINE) {
		snprintf(line, 5, "%04x", i);
		for (j = i; j < i + BYTES_PER_LINE && j < length; j++) {
			snprintf(s, 4, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}
}

char *opendous_get_time(char *str)
{
	struct timeb timebuffer;
	char *timeline;

	ftime(&timebuffer);
	timeline = ctime(&(timebuffer.time));
	snprintf(str, 49, "%.8s.%hu", &timeline[11], timebuffer.millitm);
	return str;
}
#endif
