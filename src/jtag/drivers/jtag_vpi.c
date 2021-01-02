/*
 * JTAG to VPI driver
 *
 * Copyright (C) 2013 Franck Jullien, <elec4fun@gmail.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif

#ifndef _WIN32
#include <netinet/tcp.h>
#endif

#include <string.h>

#define NO_TAP_SHIFT	0
#define TAP_SHIFT	1

#define SERVER_ADDRESS	"127.0.0.1"
#define SERVER_PORT	5555

#define	XFERT_MAX_SIZE		512

#define CMD_RESET		0
#define CMD_TMS_SEQ		1
#define CMD_SCAN_CHAIN		2
#define CMD_SCAN_CHAIN_FLIP_TMS	3
#define CMD_STOP_SIMU		4

/* jtag_vpi server port and address to connect to */
static int server_port = SERVER_PORT;
static char *server_address;

/* Send CMD_STOP_SIMU to server when OpenOCD exits? */
static bool stop_sim_on_exit;

static int sockfd;
static struct sockaddr_in serv_addr;

/* One jtag_vpi "packet" as sent over a TCP channel. */
struct vpi_cmd {
	union {
		uint32_t cmd;
		unsigned char cmd_buf[4];
	};
	unsigned char buffer_out[XFERT_MAX_SIZE];
	unsigned char buffer_in[XFERT_MAX_SIZE];
	union {
		uint32_t length;
		unsigned char length_buf[4];
	};
	union {
		uint32_t nb_bits;
		unsigned char nb_bits_buf[4];
	};
};

static char *jtag_vpi_cmd_to_str(int cmd_num)
{
	switch (cmd_num) {
	case CMD_RESET:
		return "CMD_RESET";
	case CMD_TMS_SEQ:
		return "CMD_TMS_SEQ";
	case CMD_SCAN_CHAIN:
		return "CMD_SCAN_CHAIN";
	case CMD_SCAN_CHAIN_FLIP_TMS:
		return "CMD_SCAN_CHAIN_FLIP_TMS";
	case CMD_STOP_SIMU:
		return "CMD_STOP_SIMU";
	default:
		return "<unknown>";
	}
}

static int jtag_vpi_send_cmd(struct vpi_cmd *vpi)
{
	int retval;

	/* Optional low-level JTAG debug */
	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		if (vpi->nb_bits > 0) {
			/* command with a non-empty data payload */
			char *char_buf = buf_to_hex_str(vpi->buffer_out,
					(vpi->nb_bits > DEBUG_JTAG_IOZ)
						? DEBUG_JTAG_IOZ
						: vpi->nb_bits);
			LOG_DEBUG_IO("sending JTAG VPI cmd: cmd=%s, "
					"length=%" PRIu32 ", "
					"nb_bits=%" PRIu32 ", "
					"buf_out=0x%s%s",
					jtag_vpi_cmd_to_str(vpi->cmd),
					vpi->length,
					vpi->nb_bits,
					char_buf,
					(vpi->nb_bits > DEBUG_JTAG_IOZ) ? "(...)" : "");
			free(char_buf);
		} else {
			/* command without data payload */
			LOG_DEBUG_IO("sending JTAG VPI cmd: cmd=%s, "
					"length=%" PRIu32 ", "
					"nb_bits=%" PRIu32,
					jtag_vpi_cmd_to_str(vpi->cmd),
					vpi->length,
					vpi->nb_bits);
		}
	}

	/* Use little endian when transmitting/receiving jtag_vpi cmds.
	   The choice of little endian goes against usual networking conventions
	   but is intentional to remain compatible with most older OpenOCD builds
	   (i.e. builds on little-endian platforms). */
	h_u32_to_le(vpi->cmd_buf, vpi->cmd);
	h_u32_to_le(vpi->length_buf, vpi->length);
	h_u32_to_le(vpi->nb_bits_buf, vpi->nb_bits);

retry_write:
	retval = write_socket(sockfd, vpi, sizeof(struct vpi_cmd));

	if (retval < 0) {
		/* Account for the case when socket write is interrupted. */
#ifdef _WIN32
		int wsa_err = WSAGetLastError();
		if (wsa_err == WSAEINTR)
			goto retry_write;
#else
		if (errno == EINTR)
			goto retry_write;
#endif
		/* Otherwise this is an error using the socket, most likely fatal
		   for the connection. B*/
		log_socket_error("jtag_vpi xmit");
		/* TODO: Clean way how adapter drivers can report fatal errors
		   to upper layers of OpenOCD and let it perform an orderly shutdown? */
		exit(-1);
	} else if (retval < (int)sizeof(struct vpi_cmd)) {
		/* This means we could not send all data, which is most likely fatal
		   for the jtag_vpi connection (the underlying TCP connection likely not
		   usable anymore) */
		LOG_ERROR("Could not send all data through jtag_vpi connection.");
		exit(-1);
	}

	/* Otherwise the packet has been sent successfully. */
	return ERROR_OK;
}

static int jtag_vpi_receive_cmd(struct vpi_cmd *vpi)
{
	unsigned bytes_buffered = 0;
	while (bytes_buffered < sizeof(struct vpi_cmd)) {
		int bytes_to_receive = sizeof(struct vpi_cmd) - bytes_buffered;
		int retval = read_socket(sockfd, ((char *)vpi) + bytes_buffered, bytes_to_receive);
		if (retval < 0) {
#ifdef _WIN32
			int wsa_err = WSAGetLastError();
			if (wsa_err == WSAEINTR) {
				/* socket read interrupted by WSACancelBlockingCall() */
				continue;
			}
#else
			if (errno == EINTR) {
				/* socket read interrupted by a signal */
				continue;
			}
#endif
			/* Otherwise, this is an error when accessing the socket. */
			log_socket_error("jtag_vpi recv");
			exit(-1);
		} else if (retval == 0) {
			/* Connection closed by the other side */
			LOG_ERROR("Connection prematurely closed by jtag_vpi server.");
			exit(-1);
		}
		/* Otherwise, we have successfully received some data */
		bytes_buffered += retval;
	}

	/* Use little endian when transmitting/receiving jtag_vpi cmds. */
	vpi->cmd = le_to_h_u32(vpi->cmd_buf);
	vpi->length = le_to_h_u32(vpi->length_buf);
	vpi->nb_bits = le_to_h_u32(vpi->nb_bits_buf);

	return ERROR_OK;
}

/**
 * jtag_vpi_reset - ask to reset the JTAG device
 * @param trst 1 if TRST is to be asserted
 * @param srst 1 if SRST is to be asserted
 */
static int jtag_vpi_reset(int trst, int srst)
{
	struct vpi_cmd vpi;
	memset(&vpi, 0, sizeof(struct vpi_cmd));

	vpi.cmd = CMD_RESET;
	vpi.length = 0;
	return jtag_vpi_send_cmd(&vpi);
}

/**
 * jtag_vpi_tms_seq - ask a TMS sequence transition to JTAG
 * @param bits TMS bits to be written (bit0, bit1 .. bitN)
 * @param nb_bits number of TMS bits (between 1 and 8)
 *
 * Write a series of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=\<new_state>, TDI=\<???>
 *  - writing out TCK=1, TMS=\<new_state>, TDI=\<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */
static int jtag_vpi_tms_seq(const uint8_t *bits, int nb_bits)
{
	struct vpi_cmd vpi;
	int nb_bytes;

	memset(&vpi, 0, sizeof(struct vpi_cmd));
	nb_bytes = DIV_ROUND_UP(nb_bits, 8);

	vpi.cmd = CMD_TMS_SEQ;
	memcpy(vpi.buffer_out, bits, nb_bytes);
	vpi.length = nb_bytes;
	vpi.nb_bits = nb_bits;

	return jtag_vpi_send_cmd(&vpi);
}

/**
 * jtag_vpi_path_move - ask a TMS sequence transition to JTAG
 * @param cmd path transition
 *
 * Write a series of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=\<new_state>, TDI=\<???>
 *  - writing out TCK=1, TMS=\<new_state>, TDI=\<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */

static int jtag_vpi_path_move(struct pathmove_command *cmd)
{
	uint8_t trans[DIV_ROUND_UP(cmd->num_states, 8)];

	memset(trans, 0, DIV_ROUND_UP(cmd->num_states, 8));

	for (int i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			buf_set_u32(trans, i, 1, 1);
		tap_set_state(cmd->path[i]);
	}

	return jtag_vpi_tms_seq(trans, cmd->num_states);
}

/**
 * jtag_vpi_tms - ask a tms command
 * @param cmd tms command
 */
static int jtag_vpi_tms(struct tms_command *cmd)
{
	return jtag_vpi_tms_seq(cmd->bits, cmd->num_bits);
}

static int jtag_vpi_state_move(tap_state_t state)
{
	if (tap_get_state() == state)
		return ERROR_OK;

	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), state);
	int tms_len = tap_get_tms_path_len(tap_get_state(), state);

	int retval = jtag_vpi_tms_seq(&tms_scan, tms_len);
	if (retval != ERROR_OK)
		return retval;

	tap_set_state(state);

	return ERROR_OK;
}

static int jtag_vpi_queue_tdi_xfer(uint8_t *bits, int nb_bits, int tap_shift)
{
	struct vpi_cmd vpi;
	int nb_bytes = DIV_ROUND_UP(nb_bits, 8);

	memset(&vpi, 0, sizeof(struct vpi_cmd));

	vpi.cmd = tap_shift ? CMD_SCAN_CHAIN_FLIP_TMS : CMD_SCAN_CHAIN;

	if (bits)
		memcpy(vpi.buffer_out, bits, nb_bytes);
	else
		memset(vpi.buffer_out, 0xff, nb_bytes);

	vpi.length = nb_bytes;
	vpi.nb_bits = nb_bits;

	int retval = jtag_vpi_send_cmd(&vpi);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_vpi_receive_cmd(&vpi);
	if (retval != ERROR_OK)
		return retval;

	/* Optional low-level JTAG debug */
	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		char *char_buf = buf_to_hex_str(vpi.buffer_in,
				(nb_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : nb_bits);
		LOG_DEBUG_IO("recvd JTAG VPI data: nb_bits=%d, buf_in=0x%s%s",
			nb_bits, char_buf, (nb_bits > DEBUG_JTAG_IOZ) ? "(...)" : "");
		free(char_buf);
	}

	if (bits)
		memcpy(bits, vpi.buffer_in, nb_bytes);

	return ERROR_OK;
}

/**
 * jtag_vpi_queue_tdi - short description
 * @param bits bits to be queued on TDI (or NULL if 0 are to be queued)
 * @param nb_bits number of bits
 * @param tap_shift
 */
static int jtag_vpi_queue_tdi(uint8_t *bits, int nb_bits, int tap_shift)
{
	int nb_xfer = DIV_ROUND_UP(nb_bits, XFERT_MAX_SIZE * 8);
	int retval;

	while (nb_xfer) {
		if (nb_xfer ==  1) {
			retval = jtag_vpi_queue_tdi_xfer(bits, nb_bits, tap_shift);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = jtag_vpi_queue_tdi_xfer(bits, XFERT_MAX_SIZE * 8, NO_TAP_SHIFT);
			if (retval != ERROR_OK)
				return retval;
			nb_bits -= XFERT_MAX_SIZE * 8;
			if (bits)
				bits += XFERT_MAX_SIZE;
		}

		nb_xfer--;
	}

	return ERROR_OK;
}

/**
 * jtag_vpi_clock_tms - clock a TMS transition
 * @param tms the TMS to be sent
 *
 * Triggers a TMS transition (ie. one JTAG TAP state move).
 */
static int jtag_vpi_clock_tms(int tms)
{
	const uint8_t tms_0 = 0;
	const uint8_t tms_1 = 1;

	return jtag_vpi_tms_seq(tms ? &tms_1 : &tms_0, 1);
}

/**
 * jtag_vpi_scan - launches a DR-scan or IR-scan
 * @param cmd the command to launch
 *
 * Launch a JTAG IR-scan or DR-scan
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read/write error occurred.
 */
static int jtag_vpi_scan(struct scan_command *cmd)
{
	int scan_bits;
	uint8_t *buf = NULL;
	int retval = ERROR_OK;

	scan_bits = jtag_build_buffer(cmd, &buf);

	if (cmd->ir_scan) {
		retval = jtag_vpi_state_move(TAP_IRSHIFT);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = jtag_vpi_state_move(TAP_DRSHIFT);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cmd->end_state == TAP_DRSHIFT) {
		retval = jtag_vpi_queue_tdi(buf, scan_bits, NO_TAP_SHIFT);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = jtag_vpi_queue_tdi(buf, scan_bits, TAP_SHIFT);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cmd->end_state != TAP_DRSHIFT) {
		/*
		 * As our JTAG is in an unstable state (IREXIT1 or DREXIT1), move it
		 * forward to a stable IRPAUSE or DRPAUSE.
		 */
		retval = jtag_vpi_clock_tms(0);
		if (retval != ERROR_OK)
			return retval;

		if (cmd->ir_scan)
			tap_set_state(TAP_IRPAUSE);
		else
			tap_set_state(TAP_DRPAUSE);
	}

	retval = jtag_read_buffer(buf, cmd);
	if (retval != ERROR_OK)
		return retval;

	free(buf);

	if (cmd->end_state != TAP_DRSHIFT) {
		retval = jtag_vpi_state_move(cmd->end_state);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int jtag_vpi_runtest(int cycles, tap_state_t state)
{
	int retval;

	retval = jtag_vpi_state_move(TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_vpi_queue_tdi(NULL, cycles, NO_TAP_SHIFT);
	if (retval != ERROR_OK)
		return retval;

	return jtag_vpi_state_move(state);
}

static int jtag_vpi_stableclocks(int cycles)
{
	uint8_t tms_bits[4];
	int cycles_remain = cycles;
	int nb_bits;
	int retval;
	const int CYCLES_ONE_BATCH = sizeof(tms_bits) * 8;

	assert(cycles >= 0);

	/* use TMS=1 in TAP RESET state, TMS=0 in all other stable states */
	memset(&tms_bits, (tap_get_state() == TAP_RESET) ? 0xff : 0x00, sizeof(tms_bits));

	/* send the TMS bits */
	while (cycles_remain > 0) {
		nb_bits = (cycles_remain < CYCLES_ONE_BATCH) ? cycles_remain : CYCLES_ONE_BATCH;
		retval = jtag_vpi_tms_seq(tms_bits, nb_bits);
		if (retval != ERROR_OK)
			return retval;
		cycles_remain -= nb_bits;
	}

	return ERROR_OK;
}

static int jtag_vpi_execute_queue(void)
{
	struct jtag_command *cmd;
	int retval = ERROR_OK;

	for (cmd = jtag_command_queue; retval == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			retval = jtag_vpi_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			retval = jtag_vpi_runtest(cmd->cmd.runtest->num_cycles,
						  cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			retval = jtag_vpi_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			retval = jtag_vpi_state_move(cmd->cmd.statemove->end_state);
			break;
		case JTAG_PATHMOVE:
			retval = jtag_vpi_path_move(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			retval = jtag_vpi_tms(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			retval = jtag_vpi_scan(cmd->cmd.scan);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X",
				  cmd->type);
			retval = ERROR_FAIL;
			break;
		}
	}

	return retval;
}

static int jtag_vpi_init(void)
{
	int flag = 1;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		LOG_ERROR("Could not create socket");
		return ERROR_FAIL;
	}

	memset(&serv_addr, 0, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(server_port);

	if (!server_address)
		server_address = strdup(SERVER_ADDRESS);

	serv_addr.sin_addr.s_addr = inet_addr(server_address);

	if (serv_addr.sin_addr.s_addr == INADDR_NONE) {
		LOG_ERROR("inet_addr error occurred");
		return ERROR_FAIL;
	}

	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		close(sockfd);
		LOG_ERROR("Can't connect to %s : %u", server_address, server_port);
		return ERROR_COMMAND_CLOSE_CONNECTION;
	}

	if (serv_addr.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
		/* This increases performance dramatically for local
		 * connections, which is the most likely arrangement
		 * for a VPI connection. */
		setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
	}

	LOG_INFO("Connection to %s : %u succeed", server_address, server_port);

	return ERROR_OK;
}

static int jtag_vpi_stop_simulation(void)
{
	struct vpi_cmd cmd;
	memset(&cmd, 0, sizeof(struct vpi_cmd));
	cmd.length = 0;
	cmd.nb_bits = 0;
	cmd.cmd = CMD_STOP_SIMU;
	return jtag_vpi_send_cmd(&cmd);
}

static int jtag_vpi_quit(void)
{
	if (stop_sim_on_exit) {
		if (jtag_vpi_stop_simulation() != ERROR_OK)
			LOG_WARNING("jtag_vpi: failed to send \"stop simulation\" command");
	}
	if (close_socket(sockfd) != 0) {
		LOG_WARNING("jtag_vpi: could not close jtag_vpi client socket");
		log_socket_error("jtag_vpi");
	}
	free(server_address);
	return ERROR_OK;
}

COMMAND_HANDLER(jtag_vpi_set_port)
{
	if (CMD_ARGC == 0)
		LOG_WARNING("You need to set a port number");
	else
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], server_port);

	LOG_INFO("Set server port to %u", server_port);

	return ERROR_OK;
}

COMMAND_HANDLER(jtag_vpi_set_address)
{
	free(server_address);

	if (CMD_ARGC == 0) {
		LOG_WARNING("You need to set an address");
		server_address = strdup(SERVER_ADDRESS);
	} else
		server_address = strdup(CMD_ARGV[0]);

	LOG_INFO("Set server address to %s", server_address);

	return ERROR_OK;
}

COMMAND_HANDLER(jtag_vpi_stop_sim_on_exit_handler)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("jtag_vpi_stop_sim_on_exit expects 1 argument (on|off)");
		return ERROR_COMMAND_SYNTAX_ERROR;
	} else {
		COMMAND_PARSE_ON_OFF(CMD_ARGV[0], stop_sim_on_exit);
	}
	return ERROR_OK;
}

static const struct command_registration jtag_vpi_command_handlers[] = {
	{
		.name = "jtag_vpi_set_port",
		.handler = &jtag_vpi_set_port,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the VPI server",
		.usage = "tcp_port_num",
	},
	{
		.name = "jtag_vpi_set_address",
		.handler = &jtag_vpi_set_address,
		.mode = COMMAND_CONFIG,
		.help = "set the address of the VPI server",
		.usage = "ipv4_addr",
	},
	{
		.name = "jtag_vpi_stop_sim_on_exit",
		.handler = &jtag_vpi_stop_sim_on_exit_handler,
		.mode = COMMAND_CONFIG,
		.help = "Configure if simulation stop command shall be sent "
			"before OpenOCD exits (default: off)",
		.usage = "<on|off>",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface jtag_vpi_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = jtag_vpi_execute_queue,
};

struct adapter_driver jtag_vpi_adapter_driver = {
	.name = "jtag_vpi",
	.transports = jtag_only,
	.commands = jtag_vpi_command_handlers,

	.init = jtag_vpi_init,
	.quit = jtag_vpi_quit,

	.jtag_ops = &jtag_vpi_interface,
};
