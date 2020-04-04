/*
 * JTAG to DPI driver
 *
 * Copyright (C) 2013 Franck Jullien, <elec4fun@gmail.com>
 *
 * Copyright (C) 2019-2020, Ampere Computing LLC
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

#define SERVER_ADDRESS	"127.0.0.1"
#define SERVER_PORT	5555

int server_port = SERVER_PORT;
char *server_address;

int sockfd;
struct sockaddr_in serv_addr;

static uint8_t *last_ir_buf;
static int last_ir_num_bits;

static int write_sock(char *buf, size_t len)
{
	if (write(sockfd, buf, len) != (ssize_t)len) {
		perror("write");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int read_sock(char *buf, size_t len)
{
	int r;
	int junk;	/* if pointer is NULL, dump the data */
	r = read(sockfd, buf ? buf : (char *) &junk, len);
	if (r != (ssize_t)len) {
		perror("read()");
		/*printf("read() returned %d\n", r); */
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

/**
 * jtag_dpi_reset - ask to reset the JTAG device
 * @trst: 1 if TRST is to be asserted
 * @srst: 1 if SRST is to be asserted
 */
static int jtag_dpi_reset(int trst, int srst)
{
	char *buf = "reset\n";
	return write_sock(buf, strlen(buf));
}

/**
 * jtag_dpi_scan - launches a DR-scan or IR-scan
 * @cmd: the command to launch
 *
 * Launch a JTAG IR-scan or DR-scan
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read/write error occured.
 */
static int jtag_dpi_scan(struct scan_command *cmd)
{
	char buf[20];
	uint8_t *data_buf;
	int num_bits, bytes, ret;

	num_bits = jtag_build_buffer(cmd, &data_buf);
	bytes = DIV_ROUND_UP(num_bits, 8);
	if (cmd->ir_scan) {
		if (last_ir_buf)
			free(last_ir_buf);
		last_ir_buf = calloc(bytes, 1);
		memcpy(last_ir_buf, data_buf, bytes);
		last_ir_num_bits = num_bits;
	}
	snprintf(buf, sizeof(buf), "%s %u\n", cmd->ir_scan ? "ib" : "db", num_bits);
	ret = write_sock(buf, strlen(buf)) || write_sock((char *)data_buf, bytes) ||
				read_sock((char *)data_buf, bytes);
	if (ret)
		goto out;
	ret |= jtag_read_buffer(data_buf, cmd);
out:
	free(data_buf);
	return ERROR_OK;
}

static int jtag_dpi_runtest(int cycles, tap_state_t state)
{
	char buf[20];
	uint8_t *data_buf = last_ir_buf, *junk;
	int num_bits = last_ir_num_bits, bytes, ret;
	ret = ERROR_OK;

	if (!data_buf || !num_bits)
		return ERROR_FAIL;
	bytes = DIV_ROUND_UP(num_bits, 8);
	junk = malloc(bytes);
	if (!junk)
		return ERROR_FAIL;
	snprintf(buf, sizeof(buf), "%s %u\n", "ib", num_bits);
	while (cycles > 0) {
		ret = write_sock(buf, strlen(buf)) || write_sock((char *)data_buf, bytes) ||
			read_sock((char *)junk, bytes);
		cycles -= num_bits + 6;
	}
	free(junk);
	return ret;

}

static int jtag_dpi_stableclocks(int cycles)
{
	return jtag_dpi_runtest(cycles, TAP_IDLE);
}

static int jtag_dpi_execute_queue(void)
{
	struct jtag_command *cmd;
	int retval = ERROR_OK;

	for (cmd = jtag_command_queue; retval == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			retval = jtag_dpi_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			/* wait cmd->cmd.runtest->num_cycles */
			retval = jtag_dpi_runtest(cmd->cmd.runtest->num_cycles,
						  cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			/* wait cmd->cmd.stableclocks->num_cycles */
			retval = jtag_dpi_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			/* retval = jtag_dpi_state_move(cmd->cmd.statemove->end_state); */
			break;
		case JTAG_PATHMOVE:
			/* unsupported */
			/* retval = jtag_dpi_path_move(cmd->cmd.pathmove); */
			break;
		case JTAG_TMS:
			/* unsupported */
			/* retval = jtag_dpi_tms(cmd->cmd.tms); */
			break;
		case JTAG_SLEEP:
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			retval = jtag_dpi_scan(cmd->cmd.scan);
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

static int jtag_dpi_init(void)
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
		LOG_ERROR("inet_addr error occured");
		return ERROR_FAIL;
	}

	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		close(sockfd);
		LOG_ERROR("Can't connect to %s : %u", server_address, server_port);
		return ERROR_COMMAND_CLOSE_CONNECTION;
	}
	if (serv_addr.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
		/* This increases performance drematically for local
		* connections, which is the most likely arrangement
		* for a DPI connection. */
		setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
	}

	LOG_INFO("Connection to %s : %u succeed", server_address, server_port);

	return ERROR_OK;
}

static int jtag_dpi_quit(void)
{
	free(server_address);
	return close(sockfd);
}

COMMAND_HANDLER(jtag_dpi_set_port)
{
	if (CMD_ARGC == 0)
		LOG_WARNING("You need to set a port number");
	else
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], server_port);

	LOG_INFO("Set server port to %u", server_port);

	return ERROR_OK;
}

COMMAND_HANDLER(jtag_dpi_set_address)
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

static const struct command_registration jtag_dpi_command_handlers[] = {
	{
		.name = "jtag_dpi_set_port",
		.handler = &jtag_dpi_set_port,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the DPI server",
		.usage = "description_string",
	},
	{
		.name = "jtag_dpi_set_address",
		.handler = &jtag_dpi_set_address,
		.mode = COMMAND_CONFIG,
		.help = "set the address of the DPI server",
		.usage = "description_string",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface jtag_dpi_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = jtag_dpi_execute_queue,
};

struct adapter_driver jtag_dpi_adapter_driver = {
	.name = "jtag_dpi",
	.transports = jtag_only,
	.commands = jtag_dpi_command_handlers,

	.init = jtag_dpi_init,
	.quit = jtag_dpi_quit,

	.jtag_ops = &jtag_dpi_interface,
};

