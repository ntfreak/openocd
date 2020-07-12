/***************************************************************************
 *   Copyright (C) 2010 by Michal Demin                                    *
 *   based on usbprog.c and arm-jtag-ew.c                                  *
 *   Several fixes by R. Diez in 2013.                                     *
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

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>

#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#undef DEBUG_SERIAL
/*#define DEBUG_SERIAL */
static int buspirate_execute_queue(void);
static int buspirate_init(void);
static int buspirate_quit(void);
static int buspirate_reset(int trst, int srst);

static void buspirate_end_state(tap_state_t state);
static void buspirate_state_move(void);
static void buspirate_path_move(int num_states, tap_state_t *path);
static void buspirate_runtest(int num_cycles);
static void buspirate_scan(bool ir_scan, enum scan_type type,
	uint8_t *buffer, int scan_size, struct scan_command *command);
static void buspirate_stableclocks(int num_cycles);

#define CMD_UNKNOWN       0x00
#define CMD_PORT_MODE     0x01
#define CMD_FEATURE       0x02
#define CMD_READ_ADCS     0x03
/*#define CMD_TAP_SHIFT     0x04 // old protocol */
#define CMD_TAP_SHIFT     0x05
#define CMD_ENTER_RWIRE   0x05
#define CMD_ENTER_OOCD    0x06
#define CMD_UART_SPEED    0x07
#define CMD_JTAG_SPEED    0x08
#define CMD_RAW_PERIPH    0x40
#define CMD_RAW_SPEED     0x60
#define CMD_RAW_MODE      0x80

/* raw-wire mode configuration */
#define CMD_RAW_CONFIG_HIZ 0x00
#define CMD_RAW_CONFIG_3V3 0x08
#define CMD_RAW_CONFIG_2W  0x00
#define CMD_RAW_CONFIG_3W  0x04
#define CMD_RAW_CONFIG_MSB 0x00
#define CMD_RAW_CONFIG_LSB 0x02

/* Not all OSes have this speed defined */
#if !defined(B1000000)
#define  B1000000 0010010
#endif

enum {
	MODE_HIZ = 0,
	MODE_JTAG = 1,		/* push-pull outputs */
	MODE_JTAG_OD = 2,	/* open-drain outputs */
};

enum {
	FEATURE_LED = 0x01,
	FEATURE_VREG = 0x02,
	FEATURE_TRST = 0x04,
	FEATURE_SRST = 0x08,
	FEATURE_PULLUP = 0x10
};

enum {
	ACTION_DISABLE = 0,
	ACTION_ENABLE = 1
};

enum {
	SERIAL_NORMAL = 0,
	SERIAL_FAST = 1
};

enum {
	SPEED_RAW_5_KHZ   = 0x0,
	SPEED_RAW_50_KHZ  = 0x1,
	SPEED_RAW_100_KHZ = 0x2,
	SPEED_RAW_400_KHZ = 0x3
};

/* SWD mode specific */
static bool swd_mode;
static int  queued_retval;
static char swd_features;

static const cc_t SHORT_TIMEOUT  = 1; /* Must be at least 1. */
static const cc_t NORMAL_TIMEOUT = 10;

static int buspirate_fd = -1;
static int buspirate_pinmode = MODE_JTAG_OD;
static int buspirate_baudrate = SERIAL_NORMAL;
static int buspirate_vreg;
static int buspirate_pullup;
static char *buspirate_port;

static enum tap_state last_tap_state = TAP_RESET;

/* SWD interface */
static int buspirate_swd_init(void);
static void buspirate_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk);
static void buspirate_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk);
static int buspirate_swd_switch_seq(enum swd_special_seq seq);
static int buspirate_swd_run_queue(void);

/* TAP interface */
static void buspirate_tap_init(void);
static int buspirate_tap_execute(void);
static void buspirate_tap_append(int tms, int tdi);
static void buspirate_tap_append_scan(int length, uint8_t *buffer,
		struct scan_command *command);
static void buspirate_tap_make_space(int scan, int bits);

static void buspirate_set_feature(int, char, char);
static void buspirate_set_mode(int, char);
static void buspirate_set_speed(int, char);

/* low level interface */
static void buspirate_bbio_enable(int);
static void buspirate_jtag_reset(int);
static unsigned char buspirate_jtag_command(int, uint8_t *, int);
static void buspirate_jtag_set_speed(int, char);
static void buspirate_jtag_set_mode(int, char);
static void buspirate_jtag_set_feature(int, char, char);
static void buspirate_jtag_get_adcs(int);

/* low level two-wire interface */
static void buspirate_swd_set_speed(int, char);
static void buspirate_swd_set_feature(int, char, char);
static void buspirate_swd_set_mode(int, char);

/* low level HW communication interface */
static int buspirate_serial_open(char *port);
static int buspirate_serial_setspeed(int fd, char speed, cc_t timeout);
static int buspirate_serial_write(int fd, uint8_t *buf, int size);
static int buspirate_serial_read(int fd, uint8_t *buf, int size);
static void buspirate_serial_close(int fd);
static void buspirate_print_buffer(uint8_t *buf, int size);

static int buspirate_execute_queue(void)
{
	/* currently processed command */
	struct jtag_command *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	while (cmd) {
		switch (cmd->type) {
		case JTAG_RUNTEST:
			LOG_DEBUG_IO("runtest %i cycles, end in %s",
				cmd->cmd.runtest->num_cycles,
				tap_state_name(cmd->cmd.runtest
					->end_state));
			buspirate_end_state(cmd->cmd.runtest
					->end_state);
			buspirate_runtest(cmd->cmd.runtest
					->num_cycles);
			break;
		case JTAG_TLR_RESET:
			LOG_DEBUG_IO("statemove end in %s",
				tap_state_name(cmd->cmd.statemove
						->end_state));
			buspirate_end_state(cmd->cmd.statemove
					->end_state);
			buspirate_state_move();
			break;
		case JTAG_PATHMOVE:
			LOG_DEBUG_IO("pathmove: %i states, end in %s",
				cmd->cmd.pathmove->num_states,
				tap_state_name(cmd->cmd.pathmove
					->path[cmd->cmd.pathmove
						->num_states - 1]));
			buspirate_path_move(cmd->cmd.pathmove
					->num_states,
					cmd->cmd.pathmove->path);
			break;
		case JTAG_SCAN:
			LOG_DEBUG_IO("scan end in %s",
				tap_state_name(cmd->cmd.scan
					->end_state));

			buspirate_end_state(cmd->cmd.scan
					->end_state);

			scan_size = jtag_build_buffer(cmd->cmd.scan,
					&buffer);
			type = jtag_scan_type(cmd->cmd.scan);
			buspirate_scan(cmd->cmd.scan->ir_scan, type,
				buffer, scan_size, cmd->cmd.scan);

			break;
		case JTAG_SLEEP:
			LOG_DEBUG_IO("sleep %i", cmd->cmd.sleep->us);
			buspirate_tap_execute();
			jtag_sleep(cmd->cmd.sleep->us);
				break;
		case JTAG_STABLECLOCKS:
			LOG_DEBUG_IO("stable clock %i cycles", cmd->cmd.stableclocks->num_cycles);
			buspirate_stableclocks(cmd->cmd.stableclocks->num_cycles);
				break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
		}

		cmd = cmd->next;
	}

	return buspirate_tap_execute();
}


/* Returns true if successful, false if error. */

static bool read_and_discard_all_data(const int fd)
{
	/* LOG_INFO("Discarding any stale data from a previous connection..."); */

	bool was_msg_already_printed = false;

	for ( ; ; ) {
		uint8_t buffer[1024];  /* Any size will do, it's a trade-off between stack size and performance. */

		const ssize_t read_count = read(fd, buffer, sizeof(buffer));

		if (read_count == 0) {
			/* This is the "end of file" or "connection closed at the other end" condition. */
			return true;
		}

		if (read_count > 0) {
			if (!was_msg_already_printed)	{
				LOG_INFO("Some stale data from a previous connection was discarded.");
				was_msg_already_printed = true;
			}

			continue;
		}

		assert(read_count == -1);  /* According to the specification. */

		const int errno_code = errno;

		if (errno_code == EINTR)
			continue;

		if (errno_code == EAGAIN ||
			errno_code == EWOULDBLOCK) {
			/* We know that the file descriptor has been opened with O_NONBLOCK or O_NDELAY,
			   and these codes mean that there is no data to read at present. */
			return true;
		}

		/* Some other error has occurred. */
		return false;
	}
}


static int buspirate_init(void)
{
	if (buspirate_port == NULL) {
		LOG_ERROR("You need to specify the serial port!");
		return ERROR_JTAG_INIT_FAILED;
	}

	buspirate_fd = buspirate_serial_open(buspirate_port);
	if (buspirate_fd == -1) {
		LOG_ERROR("Could not open serial port");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* The Operating System or the device itself may deliver stale data from the last connection,
	   so discard all available bytes right after the new connection has been established.
	   After all, we are implementing here a master/slave protocol, so the slave should have nothing
	   to say until the master sends the first command.

	   In the past, there was a tcflush() call in buspirate_serial_setspeed(), but that
	   was not enough. I guess you must actively read from the serial port to trigger any
	   data collection from the device and/or lower USB layers. If you disable the serial port
	   read timeout (if you set SHORT_TIMEOUT to 0), then the discarding does not work any more.

	   Note that we are lowering the serial port timeout for this first read operation,
	   otherwise the normal initialisation would be delayed for too long. */

	if (-1 == buspirate_serial_setspeed(buspirate_fd, SERIAL_NORMAL, SHORT_TIMEOUT)) {
		LOG_ERROR("Error configuring the serial port.");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (!read_and_discard_all_data(buspirate_fd)) {
		LOG_ERROR("Error while attempting to discard any stale data right after establishing the connection.");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (-1 == buspirate_serial_setspeed(buspirate_fd, SERIAL_NORMAL, NORMAL_TIMEOUT)) {
		LOG_ERROR("Error configuring the serial port.");
		return ERROR_JTAG_INIT_FAILED;
	}

	buspirate_bbio_enable(buspirate_fd);

	if (swd_mode || buspirate_baudrate != SERIAL_NORMAL)
		buspirate_set_speed(buspirate_fd, SERIAL_FAST);

	LOG_INFO("Buspirate %s Interface ready!", swd_mode ? "SWD" : "JTAG");

	if (!swd_mode)
		buspirate_tap_init();

	buspirate_set_mode(buspirate_fd, buspirate_pinmode);
	buspirate_set_feature(buspirate_fd, FEATURE_VREG,
		(buspirate_vreg == 1) ? ACTION_ENABLE : ACTION_DISABLE);
	buspirate_set_feature(buspirate_fd, FEATURE_PULLUP,
		(buspirate_pullup == 1) ? ACTION_ENABLE : ACTION_DISABLE);
	buspirate_reset(0, 0);

	return ERROR_OK;
}

static int buspirate_quit(void)
{
	LOG_INFO("Shutting down buspirate.");
	buspirate_set_mode(buspirate_fd, MODE_HIZ);
	buspirate_set_speed(buspirate_fd, SERIAL_NORMAL);

	buspirate_jtag_reset(buspirate_fd);

	buspirate_serial_close(buspirate_fd);

	if (buspirate_port) {
		free(buspirate_port);
		buspirate_port = NULL;
	}
	return ERROR_OK;
}

/* openocd command interface */
COMMAND_HANDLER(buspirate_handle_adc_command)
{
	if (buspirate_fd == -1)
		return ERROR_OK;

	/* unavailable in SWD mode */
	if (swd_mode)
		return ERROR_OK;

	/* send the command */
	buspirate_jtag_get_adcs(buspirate_fd);

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_vreg_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (atoi(CMD_ARGV[0]) == 1)
		buspirate_vreg = 1;
	else if (atoi(CMD_ARGV[0]) == 0)
		buspirate_vreg = 0;
	else
		LOG_ERROR("usage: buspirate_vreg <1|0>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_pullup_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (atoi(CMD_ARGV[0]) == 1)
		buspirate_pullup = 1;
	else if (atoi(CMD_ARGV[0]) == 0)
		buspirate_pullup = 0;
	else
		LOG_ERROR("usage: buspirate_pullup <1|0>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_led_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (atoi(CMD_ARGV[0]) == 1) {
		/* enable led */
		buspirate_set_feature(buspirate_fd, FEATURE_LED,
				ACTION_ENABLE);
	} else if (atoi(CMD_ARGV[0]) == 0) {
		/* disable led */
		buspirate_set_feature(buspirate_fd, FEATURE_LED,
				ACTION_DISABLE);
	} else {
		LOG_ERROR("usage: buspirate_led <1|0>");
	}

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_mode_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGV[0][0] == 'n')
		buspirate_pinmode = MODE_JTAG;
	else if (CMD_ARGV[0][0] == 'o')
		buspirate_pinmode = MODE_JTAG_OD;
	else
		LOG_ERROR("usage: buspirate_mode <normal|open-drain>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_speed_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGV[0][0] == 'n')
		buspirate_baudrate = SERIAL_NORMAL;
	else if (CMD_ARGV[0][0] == 'f')
		buspirate_baudrate = SERIAL_FAST;
	else
		LOG_ERROR("usage: buspirate_speed <normal|fast>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_port_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (buspirate_port == NULL)
		buspirate_port = strdup(CMD_ARGV[0]);

	return ERROR_OK;

}

static const struct command_registration buspirate_command_handlers[] = {
	{
		.name = "buspirate_adc",
		.handler = &buspirate_handle_adc_command,
		.mode = COMMAND_EXEC,
		.help = "reads voltages on adc pins",
		.usage = "",
	},
	{
		.name = "buspirate_vreg",
		.usage = "<1|0>",
		.handler = &buspirate_handle_vreg_command,
		.mode = COMMAND_CONFIG,
		.help = "changes the state of voltage regulators",
	},
	{
		.name = "buspirate_pullup",
		.usage = "<1|0>",
		.handler = &buspirate_handle_pullup_command,
		.mode = COMMAND_CONFIG,
		.help = "changes the state of pullup",
	},
	{
		.name = "buspirate_led",
		.usage = "<1|0>",
		.handler = &buspirate_handle_led_command,
		.mode = COMMAND_EXEC,
		.help = "changes the state of led",
	},
	{
		.name = "buspirate_speed",
		.usage = "<normal|fast>",
		.handler = &buspirate_handle_speed_command,
		.mode = COMMAND_CONFIG,
		.help = "speed of the interface",
	},
	{
		.name = "buspirate_mode",
		.usage = "<normal|open-drain>",
		.handler = &buspirate_handle_mode_command,
		.mode = COMMAND_CONFIG,
		.help = "pin mode of the interface",
	},
	{
		.name = "buspirate_port",
		.usage = "/dev/ttyUSB0",
		.handler = &buspirate_handle_port_command,
		.mode =	COMMAND_CONFIG,
		.help = "name of the serial port to open",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver buspirate_swd = {
	.init = buspirate_swd_init,
	.switch_seq = buspirate_swd_switch_seq,
	.read_reg = buspirate_swd_read_reg,
	.write_reg = buspirate_swd_write_reg,
	.run = buspirate_swd_run_queue,
};

static const char * const buspirate_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface buspirate_interface = {
	.execute_queue = buspirate_execute_queue,
};

struct adapter_driver buspirate_adapter_driver = {
	.name = "buspirate",
	.transports = buspirate_transports,
	.commands = buspirate_command_handlers,

	.init = buspirate_init,
	.quit = buspirate_quit,
	.reset = buspirate_reset,

	.jtag_ops = &buspirate_interface,
	.swd_ops = &buspirate_swd,
};

/*************** jtag execute commands **********************/
static void buspirate_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void buspirate_state_move(void)
{
	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
			tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(),
			tap_get_end_state());

	for (i = 0; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		buspirate_tap_append(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

static void buspirate_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) == path[i]) {
			buspirate_tap_append(0, 0);
		} else if (tap_state_transition(tap_get_state(), true)
				== path[i]) {
			buspirate_tap_append(1, 0);
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid "
				"TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static void buspirate_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		buspirate_end_state(TAP_IDLE);
		buspirate_state_move();
	}

	for (i = 0; i < num_cycles; i++)
		buspirate_tap_append(0, 0);

	LOG_DEBUG_IO("runtest: cur_state %s end_state %s",
			tap_state_name(tap_get_state()),
			tap_state_name(tap_get_end_state()));

	/* finish in end_state */
	buspirate_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		buspirate_state_move();
}

static void buspirate_scan(bool ir_scan, enum scan_type type,
	uint8_t *buffer, int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	buspirate_tap_make_space(1, scan_size+8);
	/* is 8 correct ? (2 moves = 16) */

	saved_end_state = tap_get_end_state();

	buspirate_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	/* Only move if we're not already there */
	if (tap_get_state() != tap_get_end_state())
		buspirate_state_move();

	buspirate_tap_append_scan(scan_size, buffer, command);

	/* move to PAUSE */
	buspirate_tap_append(0, 0);

	/* restore the saved state */
	buspirate_end_state(saved_end_state);
	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
		buspirate_state_move();
}

static void buspirate_stableclocks(int num_cycles)
{
	int i;
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);

	buspirate_tap_make_space(0, num_cycles);

	for (i = 0; i < num_cycles; i++)
		buspirate_tap_append(tms, 0);
}

/************************* TAP related stuff **********/

/* This buffer size matches the maximum CMD_TAP_SHIFT bit length in the Bus Pirate firmware,
   look for constant 0x2000 in OpenOCD.c . */
#define BUSPIRATE_BUFFER_SIZE 1024

/* The old value of 32 scans was not enough to achieve near 100% utilisation ratio
   for the current BUSPIRATE_BUFFER_SIZE value of 1024.
   With 128 scans I am getting full USB 2.0 high speed packets (512 bytes long) when
   using the JtagDue firmware on the Arduino Due instead of the Bus Pirate, which
   amounts approximately to a 10% overall speed gain. Bigger packets should also
   benefit the Bus Pirate, but the speed difference is much smaller.
   Unfortunately, each 512-byte packet is followed by a 329-byte one, which is not ideal.
   However, increasing BUSPIRATE_BUFFER_SIZE for the benefit of the JtagDue would
   make it incompatible with the Bus Pirate firmware. */
#define BUSPIRATE_MAX_PENDING_SCANS 128

static uint8_t tms_chain[BUSPIRATE_BUFFER_SIZE]; /* send */
static uint8_t tdi_chain[BUSPIRATE_BUFFER_SIZE]; /* send */
static int tap_chain_index;

struct pending_scan_result /* this was stolen from arm-jtag-ew */
{
	int first; /* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
};

static struct pending_scan_result
tap_pending_scans[BUSPIRATE_MAX_PENDING_SCANS];
static int tap_pending_scans_num;

static void buspirate_tap_init(void)
{
	tap_chain_index = 0;
	tap_pending_scans_num = 0;
}

static int buspirate_tap_execute(void)
{
	static const int CMD_TAP_SHIFT_HEADER_LEN = 3;

	uint8_t tmp[4096];
	uint8_t *in_buf;
	int i;
	int fill_index = 0;
	int ret;
	int bytes_to_send;

	if (tap_chain_index <= 0)
		return ERROR_OK;

	LOG_DEBUG("executing tap num bits = %i scans = %i",
			tap_chain_index, tap_pending_scans_num);

	bytes_to_send = DIV_ROUND_UP(tap_chain_index, 8);

	tmp[0] = CMD_TAP_SHIFT; /* this command expects number of bits */
	tmp[1] = tap_chain_index >> 8;  /* high */
	tmp[2] = tap_chain_index;  /* low */

	fill_index = CMD_TAP_SHIFT_HEADER_LEN;
	for (i = 0; i < bytes_to_send; i++) {
		tmp[fill_index] = tdi_chain[i];
		fill_index++;
		tmp[fill_index] = tms_chain[i];
		fill_index++;
	}

	/* jlink.c calls the routine below, which may be useful for debugging purposes.
	   For example, enabling this allows you to compare the log outputs from jlink.c
	   and from this module for JTAG development or troubleshooting purposes. */
	if (false) {
		last_tap_state = jtag_debug_state_machine(tms_chain, tdi_chain,
												  tap_chain_index, last_tap_state);
	}

	ret = buspirate_serial_write(buspirate_fd, tmp, CMD_TAP_SHIFT_HEADER_LEN + bytes_to_send*2);
	if (ret != bytes_to_send*2+CMD_TAP_SHIFT_HEADER_LEN) {
		LOG_ERROR("error writing :(");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	ret = buspirate_serial_read(buspirate_fd, tmp, bytes_to_send + CMD_TAP_SHIFT_HEADER_LEN);
	if (ret != bytes_to_send + CMD_TAP_SHIFT_HEADER_LEN) {
		LOG_ERROR("error reading");
		return ERROR_FAIL;
	}
	in_buf = (uint8_t *)(&tmp[CMD_TAP_SHIFT_HEADER_LEN]);

	/* parse the scans */
	for (i = 0; i < tap_pending_scans_num; i++) {
		uint8_t *buffer = tap_pending_scans[i].buffer;
		int length = tap_pending_scans[i].length;
		int first = tap_pending_scans[i].first;
		struct scan_command *command = tap_pending_scans[i].command;

		/* copy bits from buffer */
		buf_set_buf(in_buf, first, buffer, 0, length);

		/* return buffer to higher level */
		if (jtag_read_buffer(buffer, command) != ERROR_OK) {
			buspirate_tap_init();
			return ERROR_JTAG_QUEUE_FAILED;
		}

		free(buffer);
	}
	buspirate_tap_init();
	return ERROR_OK;
}

static void buspirate_tap_make_space(int scans, int bits)
{
	int have_scans = BUSPIRATE_MAX_PENDING_SCANS - tap_pending_scans_num;
	int have_bits = BUSPIRATE_BUFFER_SIZE * 8 - tap_chain_index;

	if ((have_scans < scans) || (have_bits < bits))
		buspirate_tap_execute();
}

static void buspirate_tap_append(int tms, int tdi)
{
	int chain_index;

	buspirate_tap_make_space(0, 1);
	chain_index = tap_chain_index / 8;

	if (chain_index < BUSPIRATE_BUFFER_SIZE) {
		int bit_index = tap_chain_index % 8;
		uint8_t bit = 1 << bit_index;

		if (0 == bit_index) {
			/* Let's say that the TAP shift operation wants to shift 9 bits,
			   so we will be sending to the Bus Pirate a bit count of 9 but still
			   full 16 bits (2 bytes) of shift data.
			   If we don't clear all bits at this point, the last 7 bits will contain
			   random data from the last buffer contents, which is not pleasant to the eye.
			   Besides, the Bus Pirate (or some clone) may want to assert in debug builds
			   that, after consuming all significant data bits, the rest of them are zero.
			   Therefore, for aesthetic and for assert purposes, we clear all bits below. */
			tms_chain[chain_index] = 0;
			tdi_chain[chain_index] = 0;
		}

		if (tms)
			tms_chain[chain_index] |= bit;
		else
			tms_chain[chain_index] &= ~bit;

		if (tdi)
			tdi_chain[chain_index] |= bit;
		else
			tdi_chain[chain_index] &= ~bit;

		tap_chain_index++;
	} else {
		LOG_ERROR("tap_chain overflow, bad things will happen");
		/* Exit abruptly, like jlink.c does. After a buffer overflow we don't want
		   to carry on, as data will be corrupt. Another option would be to return
		   some error code at this point. */
		exit(-1);
	}
}

static void buspirate_tap_append_scan(int length, uint8_t *buffer,
		struct scan_command *command)
{
	int i;
	tap_pending_scans[tap_pending_scans_num].length = length;
	tap_pending_scans[tap_pending_scans_num].buffer = buffer;
	tap_pending_scans[tap_pending_scans_num].command = command;
	tap_pending_scans[tap_pending_scans_num].first = tap_chain_index;

	for (i = 0; i < length; i++) {
		int tms = (i < length-1 ? 0 : 1);
		int tdi = (buffer[i/8] >> (i%8)) & 1;
		buspirate_tap_append(tms, tdi);
	}
	tap_pending_scans_num++;
}

/*************** wrapper functions *********************/

/* (1) assert or (0) deassert reset lines */
static int buspirate_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (trst)
		buspirate_set_feature(buspirate_fd, FEATURE_TRST, ACTION_DISABLE);
	else
		buspirate_set_feature(buspirate_fd, FEATURE_TRST, ACTION_ENABLE);

	if (srst)
		buspirate_set_feature(buspirate_fd, FEATURE_SRST, ACTION_DISABLE);
	else
		buspirate_set_feature(buspirate_fd, FEATURE_SRST, ACTION_ENABLE);

	return ERROR_OK;
}

static void buspirate_set_feature(int fd, char feat, char action)
{
	if (swd_mode)
		buspirate_swd_set_feature(fd, feat, action);
	else
		buspirate_jtag_set_feature(fd, feat, action);
}

static void buspirate_set_mode(int fd, char mode)
{
	if (swd_mode)
		buspirate_swd_set_mode(fd, mode);
	else
		buspirate_jtag_set_mode(fd, mode);
}

static void buspirate_set_speed(int fd, char speed)
{
	if (swd_mode)
		buspirate_swd_set_speed(fd, speed);
	else
		buspirate_jtag_set_speed(fd, speed);
}


/*************** swd lowlevel functions ********************/

static void buspirate_swd_set_speed(int fd, char speed)
{
	int  ret;
	uint8_t tmp[1];

	LOG_DEBUG("Buspirate speed setting in SWD mode defaults to 400 kHz");

	/* speed settings */
	tmp[0] = CMD_RAW_SPEED | SPEED_RAW_400_KHZ;
	buspirate_serial_write(fd, tmp, 1);
	ret = buspirate_serial_read(fd, tmp, 1);
	if (ret != 1) {
		LOG_ERROR("Buspirate did not answer correctly");
		exit(-1);
	}
	if (tmp[0] != 1) {
		LOG_ERROR("Buspirate did not reply as expected to the speed change command");
		exit(-1);
	}
}

static void buspirate_swd_set_mode(int fd, char mode)
{
	int ret;
	uint8_t tmp[1];

	/* raw-wire mode configuration */
	if (mode == MODE_HIZ)
		tmp[0] = CMD_RAW_MODE | CMD_RAW_CONFIG_LSB;
	else
		tmp[0] = CMD_RAW_MODE | CMD_RAW_CONFIG_LSB | CMD_RAW_CONFIG_3V3;

	buspirate_serial_write(fd, tmp, 1);
	ret = buspirate_serial_read(fd, tmp, 1);
	if (ret != 1) {
		LOG_ERROR("Buspirate did not answer correctly");
		exit(-1);
	}
	if (tmp[0] != 1) {
		LOG_ERROR("Buspirate did not reply as expected to the configure command");
		exit(-1);
	}
}

static void buspirate_swd_set_feature(int fd, char feat, char action)
{
	int  ret;
	uint8_t tmp[1];

	switch (feat) {
		case FEATURE_TRST:
			LOG_DEBUG("Buspirate TRST feature not available in SWD mode");
			return;
		case FEATURE_LED:
			LOG_ERROR("Buspirate LED feature not available in SWD mode");
			return;
		case FEATURE_SRST:
			swd_features = (action == ACTION_ENABLE) ? swd_features | 0x02 : swd_features & 0x0D;
			break;
		case FEATURE_PULLUP:
			swd_features = (action == ACTION_ENABLE) ? swd_features | 0x04 : swd_features & 0x0B;
			break;
		case FEATURE_VREG:
			swd_features = (action == ACTION_ENABLE) ? swd_features | 0x08 : swd_features & 0x07;
			break;
		default:
			LOG_DEBUG("Buspirate unknown feature %d", feat);
			return;
	}

	tmp[0] = CMD_RAW_PERIPH | swd_features;
	buspirate_serial_write(fd, tmp, 1);
	ret = buspirate_serial_read(fd, tmp, 1);
	if (ret != 1) {
		LOG_DEBUG("Buspirate feature %d not supported in SWD mode", feat);
	} else if (tmp[0] != 1) {
		LOG_ERROR("Buspirate did not reply as expected to the configure command");
		exit(-1);
	}
}

/*************** jtag lowlevel functions ********************/
static void buspirate_bbio_enable(int fd)
{
	int ret;
	char command;
	const char *mode_answers[2] = { "OCD1", "RAW1" };
	const char *correct_ans     = NULL;
	uint8_t tmp[21] = { [0 ... 20] = 0x00 };
	int done = 0;
	int cmd_sent = 0;

	if (swd_mode) {
		command     = CMD_ENTER_RWIRE;
		correct_ans = mode_answers[1];
	} else {
		command     = CMD_ENTER_OOCD;
		correct_ans = mode_answers[0];
	}

	LOG_DEBUG("Entering binary mode, that is %s", correct_ans);
	buspirate_serial_write(fd, tmp, 20);
	usleep(10000);

	/* reads 1 to n "BBIO1"s and one "OCD1" or "RAW1" */
	while (!done) {
		ret = buspirate_serial_read(fd, tmp, 4);
		if (ret != 4) {
			LOG_ERROR("Buspirate error. Is binary"
				"/OpenOCD support enabled?");
			exit(-1);
		}
		if (strncmp((char *)tmp, "BBIO", 4) == 0) {
			ret = buspirate_serial_read(fd, tmp, 1);
			if (ret != 1) {
				LOG_ERROR("Buspirate did not answer correctly! "
					"Do you have correct firmware?");
				exit(-1);
			}
			if (tmp[0] != '1') {
				LOG_ERROR("Unsupported binary protocol");
				exit(-1);
			}
			if (cmd_sent == 0) {
				cmd_sent = 1;
				tmp[0] = command;
				ret = buspirate_serial_write(fd, tmp, 1);
				if (ret != 1) {
					LOG_ERROR("error reading");
					exit(-1);
				}
			}
		} else if (strncmp((char *)tmp, correct_ans, 4) == 0)
			done = 1;
		else {
			LOG_ERROR("Buspirate did not answer correctly! "
				"Do you have correct firmware?");
			exit(-1);
		}
	}

}

static void buspirate_jtag_reset(int fd)
{
	uint8_t tmp[5];

	tmp[0] = 0x00; /* exit OCD1 mode */
	buspirate_serial_write(fd, tmp, 1);
	usleep(10000);
	/* We ignore the return value here on purpose, nothing we can do */
	buspirate_serial_read(fd, tmp, 5);
	if (strncmp((char *)tmp, "BBIO1", 5) == 0) {
		tmp[0] = 0x0F; /*  reset BP */
		buspirate_serial_write(fd, tmp, 1);
	} else
		LOG_ERROR("Unable to restart buspirate!");
}

static void buspirate_jtag_set_speed(int fd, char speed)
{
	int ret;
	uint8_t tmp[2];
	uint8_t ack[2];

	ack[0] = 0xAA;
	ack[1] = 0x55;

	tmp[0] = CMD_UART_SPEED;
	tmp[1] = speed;
	buspirate_jtag_command(fd, tmp, 2);

	/* here the adapter changes speed, we need follow */
	if (-1 == buspirate_serial_setspeed(fd, speed, NORMAL_TIMEOUT)) {
		LOG_ERROR("Error configuring the serial port.");
		exit(-1);
	}

	buspirate_serial_write(fd, ack, 2);
	ret = buspirate_serial_read(fd, tmp, 2);
	if (ret != 2) {
		LOG_ERROR("Buspirate did not ack speed change");
		exit(-1);
	}
	if ((tmp[0] != CMD_UART_SPEED) || (tmp[1] != speed)) {
		LOG_ERROR("Buspirate did not reply as expected to the speed change command");
		exit(-1);
	}
	LOG_INFO("Buspirate switched to %s mode",
		(speed == SERIAL_NORMAL) ? "normal" : "FAST");
}


static void buspirate_jtag_set_mode(int fd, char mode)
{
	uint8_t tmp[2];
	tmp[0] = CMD_PORT_MODE;
	tmp[1] = mode;
	buspirate_jtag_command(fd, tmp, 2);
}

static void buspirate_jtag_set_feature(int fd, char feat, char action)
{
	uint8_t tmp[3];
	tmp[0] = CMD_FEATURE;
	tmp[1] = feat;   /* what */
	tmp[2] = action; /* action */
	buspirate_jtag_command(fd, tmp, 3);
}

static void buspirate_jtag_get_adcs(int fd)
{
	uint8_t tmp[10];
	uint16_t a, b, c, d;
	tmp[0] = CMD_READ_ADCS;
	buspirate_jtag_command(fd, tmp, 1);
	a = tmp[2] << 8 | tmp[3];
	b = tmp[4] << 8 | tmp[5];
	c = tmp[6] << 8 | tmp[7];
	d = tmp[8] << 8 | tmp[9];

	LOG_INFO("ADC: ADC_Pin = %.02f VPullup = %.02f V33 = %.02f "
		"V50 = %.02f",
		((float)a)/155.1515, ((float)b)/155.1515,
		((float)c)/155.1515, ((float)d)/155.1515);
}

static unsigned char buspirate_jtag_command(int fd,
		uint8_t *cmd, int cmdlen)
{
	int res;
	int len = 0;

	res = buspirate_serial_write(fd, cmd, cmdlen);

	if ((cmd[0] == CMD_UART_SPEED)
				|| (cmd[0] == CMD_PORT_MODE)
				|| (cmd[0] == CMD_FEATURE)
				|| (cmd[0] == CMD_JTAG_SPEED))
		return 1;

	if (res == cmdlen) {
		switch (cmd[0]) {
		case CMD_READ_ADCS:
			len = 10; /* 2*sizeof(char)+4*sizeof(uint16_t) */
			break;
		case CMD_TAP_SHIFT:
			len = cmdlen;
			break;
		default:
			LOG_INFO("Wrong !");
		}
		res =  buspirate_serial_read(fd, cmd, len);
		if (res > 0)
			return (unsigned char)cmd[1];
		else
			return -1;
	} else
		return -1;
	return 0;
}

/* low level serial port */
/* TODO add support for WIN32 and others ! */
static int buspirate_serial_open(char *port)
{
	int fd;
	fd = open(buspirate_port, O_RDWR | O_NOCTTY | O_NDELAY);
	return fd;
}


/* Returns -1 on error. */

static int buspirate_serial_setspeed(int fd, char speed, cc_t timeout)
{
	struct termios t_opt;
	speed_t baud = (speed == SERIAL_FAST) ? B1000000 : B115200;

	/* set the serial port parameters */
	fcntl(fd, F_SETFL, 0);
	if (0 != tcgetattr(fd, &t_opt))
		return -1;

	if (0 != cfsetispeed(&t_opt, baud))
		return -1;

	if (0 != cfsetospeed(&t_opt, baud))
		return -1;

	t_opt.c_cflag |= (CLOCAL | CREAD);
	t_opt.c_cflag &= ~PARENB;
	t_opt.c_cflag &= ~CSTOPB;
	t_opt.c_cflag &= ~CSIZE;
	t_opt.c_cflag |= CS8;
	t_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* The serial port may have been configured for human interaction with
	   the Bus Pirate console, but OpenOCD is going to use a binary protocol,
	   so make sure to turn off any CR/LF translation and the like. */
	t_opt.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

	t_opt.c_oflag &= ~OPOST;
	t_opt.c_cc[VMIN] = 0;
	t_opt.c_cc[VTIME] = timeout;

	/* Note that, in the past, TCSANOW was used below instead of TCSADRAIN,
	   and CMD_UART_SPEED did not work properly then, at least with
	   the Bus Pirate v3.5 (USB). */
	if (0 != tcsetattr(fd, TCSADRAIN, &t_opt)) {
		/* According to the Linux documentation, this is actually not enough
		   to detect errors, you need to call tcgetattr() and check that
		   all changes have been performed successfully. */
		return -1;
	}

	return 0;
}

static int buspirate_serial_write(int fd, uint8_t *buf, int size)
{
	int ret = 0;

	ret = write(fd, buf, size);

	LOG_DEBUG("size = %d ret = %d", size, ret);
	buspirate_print_buffer(buf, size);

	if (ret != size)
		LOG_ERROR("Error sending data");

	return ret;
}

static int buspirate_serial_read(int fd, uint8_t *buf, int size)
{
	int len = 0;
	int ret = 0;
	int timeout = 0;

	while (len < size) {
		ret = read(fd, buf+len, size-len);
		if (ret == -1)
			return -1;

		if (ret == 0) {
			timeout++;

			if (timeout >= 10)
				break;

			continue;
		}

		len += ret;
	}

	LOG_DEBUG("should have read = %d actual size = %d", size, len);
	buspirate_print_buffer(buf, len);

	if (len != size)
		LOG_ERROR("Error reading data");

	return len;
}

static void buspirate_serial_close(int fd)
{
	close(fd);
}

#define LINE_SIZE      81
#define BYTES_PER_LINE 16
static void buspirate_print_buffer(uint8_t *buf, int size)
{
	char line[LINE_SIZE];
	char tmp[10];
	int offset = 0;

	line[0] = 0;
	while (offset < size) {
		snprintf(tmp, 5, "%02x ", (uint8_t)buf[offset]);
		offset++;

		strcat(line, tmp);

		if (offset % BYTES_PER_LINE == 0) {
			LOG_DEBUG("%s", line);
			line[0] = 0;
		}
	}

	if (line[0] != 0)
		LOG_DEBUG("%s", line);
}

/************************* SWD related stuff **********/

static int buspirate_swd_init(void)
{
	LOG_INFO("Buspirate SWD mode enabled");
	swd_mode = true;

	return ERROR_OK;
}

static int buspirate_swd_switch_seq(enum swd_special_seq seq)
{
	const uint8_t *sequence;
	int sequence_len;
	uint32_t no_bytes, sequence_offset;

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		sequence = swd_seq_line_reset;
		sequence_len = DIV_ROUND_UP(swd_seq_line_reset_len, 8);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		sequence = swd_seq_jtag_to_swd;
		sequence_len = DIV_ROUND_UP(swd_seq_jtag_to_swd_len, 8);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		sequence = swd_seq_swd_to_jtag;
		sequence_len = DIV_ROUND_UP(swd_seq_swd_to_jtag_len, 8);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	no_bytes = sequence_len;
	sequence_offset = 0;

	while (no_bytes) {
		uint8_t tmp[17];
		uint32_t to_send;

		to_send = no_bytes > 16 ? 16 : no_bytes;

		tmp[0] = 0x10 + ((to_send - 1) & 0x0F);
		memcpy(tmp + 1, &sequence[sequence_offset], to_send);

		buspirate_serial_write(buspirate_fd, tmp, to_send + 1);
		buspirate_serial_read(buspirate_fd, tmp, to_send + 1);

		no_bytes -= to_send;
		sequence_offset += to_send;
	}

	return ERROR_OK;
}

static uint8_t buspirate_swd_write_header(uint8_t cmd)
{
	uint8_t tmp[8];
	int  to_send;

	tmp[0] = 0x10; /* bus pirate: send 1 byte */
	tmp[1] = cmd;  /* swd cmd */
	tmp[2] = 0x07; /* ack __x */
	tmp[3] = 0x07; /* ack _x_ */
	tmp[4] = 0x07; /* ack x__ */
	tmp[5] = 0x07; /* write mode trn_1 */
	tmp[6] = 0x07; /* write mode trn_2 */

	to_send = ((cmd & SWD_CMD_RnW) == 0) ? 7 : 5;
	buspirate_serial_write(buspirate_fd, tmp, to_send);

	/* read ack */
	buspirate_serial_read(buspirate_fd, tmp, 2); /* drop pirate command ret vals */
	buspirate_serial_read(buspirate_fd, tmp, to_send - 2); /* ack bits */

	return tmp[2] << 2 | tmp[1] << 1 | tmp[0];
}

static void buspirate_swd_idle_clocks(uint32_t no_bits)
{
	uint32_t no_bytes;
	uint8_t tmp[20];

	no_bytes = (no_bits + 7) / 8;
	memset(tmp + 1, 0x00, sizeof(tmp) - 1);

	/* unfortunately bus pirate misbehaves when clocks are sent in parts
	 * so we need to limit at 128 clock cycles
	 */
	if (no_bytes > 16)
		no_bytes = 16;

	while (no_bytes) {
		uint8_t to_send = no_bytes > 16 ? 16 : no_bytes;
		tmp[0] = 0x10 + ((to_send - 1) & 0x0F);

		buspirate_serial_write(buspirate_fd, tmp, to_send + 1);
		buspirate_serial_read(buspirate_fd, tmp, to_send + 1);

		no_bytes -= to_send;
	}
}

static void buspirate_swd_clear_sticky_errors(void)
{
	buspirate_swd_write_reg(swd_cmd(false,  false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
}

static void buspirate_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	uint8_t tmp[16];

	LOG_DEBUG("buspirate_swd_read_reg");
	assert(cmd & SWD_CMD_RnW);

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skip buspirate_swd_read_reg because queued_retval=%d", queued_retval);
		return;
	}

	cmd |= SWD_CMD_START | SWD_CMD_PARK;
	uint8_t ack = buspirate_swd_write_header(cmd);

	/* do a read transaction */
	tmp[0] = 0x06; /* 4 data bytes */
	tmp[1] = 0x06;
	tmp[2] = 0x06;
	tmp[3] = 0x06;
	tmp[4] = 0x07; /* parity bit */
	tmp[5] = 0x21; /* 2 turnaround clocks */

	buspirate_serial_write(buspirate_fd, tmp, 6);
	buspirate_serial_read(buspirate_fd, tmp, 6);

	/* store the data and parity */
	uint32_t data = (uint8_t) tmp[0];
	data |= (uint8_t) tmp[1] << 8;
	data |= (uint8_t) tmp[2] << 16;
	data |= (uint8_t) tmp[3] << 24;
	int parity = tmp[4] ? 0x01 : 0x00;

	LOG_DEBUG("%s %s %s reg %X = %08"PRIx32,
			ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			cmd & SWD_CMD_APnDP ? "AP" : "DP",
			cmd & SWD_CMD_RnW ? "read" : "write",
			(cmd & SWD_CMD_A32) >> 1,
			data);

	switch (ack) {
	 case SWD_ACK_OK:
		if (parity != parity_u32(data)) {
			LOG_DEBUG("Read data parity mismatch %x %x", parity, parity_u32(data));
			queued_retval = ERROR_FAIL;
			return;
		}
		if (value)
			*value = data;
		if (cmd & SWD_CMD_APnDP)
			buspirate_swd_idle_clocks(ap_delay_clk);
		return;
	 case SWD_ACK_WAIT:
		LOG_DEBUG("SWD_ACK_WAIT");
		buspirate_swd_clear_sticky_errors();
		return;
	 case SWD_ACK_FAULT:
		LOG_DEBUG("SWD_ACK_FAULT");
		queued_retval = ack;
		return;
	 default:
		LOG_DEBUG("No valid acknowledge: ack=%d", ack);
		queued_retval = ack;
		return;
	}
}

static void buspirate_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	uint8_t tmp[16];

	LOG_DEBUG("buspirate_swd_write_reg");
	assert(!(cmd & SWD_CMD_RnW));

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skip buspirate_swd_write_reg because queued_retval=%d", queued_retval);
		return;
	}

	cmd |= SWD_CMD_START | SWD_CMD_PARK;
	uint8_t ack = buspirate_swd_write_header(cmd);

	/* do a write transaction */
	tmp[0] = 0x10 + ((4 + 1 - 1) & 0xF); /* bus pirate: send 4+1 bytes */
	buf_set_u32((uint8_t *) tmp + 1, 0, 32, value);
	/* write sequence ends with parity bit and 7 idle ticks */
	tmp[5] = parity_u32(value) ? 0x01 : 0x00;

	buspirate_serial_write(buspirate_fd, tmp, 6);
	buspirate_serial_read(buspirate_fd, tmp, 6);

	LOG_DEBUG("%s %s %s reg %X = %08"PRIx32,
			ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
			cmd & SWD_CMD_APnDP ? "AP" : "DP",
			cmd & SWD_CMD_RnW ? "read" : "write",
			(cmd & SWD_CMD_A32) >> 1,
			value);

	switch (ack) {
	 case SWD_ACK_OK:
		if (cmd & SWD_CMD_APnDP)
			buspirate_swd_idle_clocks(ap_delay_clk);
		return;
	 case SWD_ACK_WAIT:
		LOG_DEBUG("SWD_ACK_WAIT");
		buspirate_swd_clear_sticky_errors();
		return;
	 case SWD_ACK_FAULT:
		LOG_DEBUG("SWD_ACK_FAULT");
		queued_retval = ack;
		return;
	 default:
		LOG_DEBUG("No valid acknowledge: ack=%d", ack);
		queued_retval = ack;
		return;
	}
}

static int buspirate_swd_run_queue(void)
{
	LOG_DEBUG("buspirate_swd_run_queue");
	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	buspirate_swd_idle_clocks(8);

	int retval = queued_retval;
	queued_retval = ERROR_OK;
	LOG_DEBUG("SWD queue return value: %02x", retval);
	return retval;
}


