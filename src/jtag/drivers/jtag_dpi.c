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

static uint16_t server_port = SERVER_PORT;
static char *server_address;

static int sockfd;
static struct sockaddr_in serv_addr;

static uint8_t *last_ir_buf;
static int last_ir_num_bits;

static int write_sock(char *buf, size_t len)
{
	if (buf == NULL) {
		LOG_ERROR("%s: NULL 'buf' argument, file %s, line %d",
			__func__, __FILE__, __LINE__);
		return ERROR_FAIL;
	}
	if (write(sockfd, buf, len) != (ssize_t)len) {
		LOG_ERROR("%s: %s, file %s, line %d", __func__,
			strerror(errno), __FILE__, __LINE__);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int read_sock(char *buf, size_t len)
{
	if (buf == NULL) {
		LOG_ERROR("%s: NULL 'buf' argument, file %s, line %d",
			__func__, __FILE__, __LINE__);
		return ERROR_FAIL;
	}
	if (read(sockfd, buf, len) != (ssize_t)len) {
		LOG_ERROR("%s: %s, file %s, line %d", __func__,
			strerror(errno), __FILE__, __LINE__);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

/**
 * jtag_dpi_reset - ask to reset the JTAG device
 * @param trst 1 if TRST is to be asserted
 * @param srst 1 if SRST is to be asserted
 */
static int jtag_dpi_reset(int trst, int srst)
{
	char *buf = "reset\n";
	int ret = ERROR_OK;

	LOG_DEBUG_IO("JTAG DRIVER DEBUG: reset trst: %i srst %i", trst, srst);

	if (trst == 1) {
		/* reset the JTAG TAP controller */
		ret = write_sock(buf, strlen(buf));
		if (ret != ERROR_OK) {
			LOG_ERROR("write_sock() fail, file %s, line %d",
				__FILE__, __LINE__);
		}
	}

	if (srst == 1) {
		/* System target reset not supported */
		LOG_ERROR("DPI SRST not supported");
		ret = ERROR_FAIL;
	}

	return ret;
}

/**
 * jtag_dpi_scan - launches a DR-scan or IR-scan
 * @param cmd the command to launch
 *
 * Launch a JTAG IR-scan or DR-scan
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read/write error occurred.
 */
static int jtag_dpi_scan(struct scan_command *cmd)
{
	char buf[20];
	uint8_t *data_buf;
	int num_bits, bytes;
	int ret = ERROR_OK;

	num_bits = jtag_build_buffer(cmd, &data_buf);
	if (data_buf == NULL) {
		LOG_ERROR("jtag_build_buffer call failed, data_buf == NULL, "
			"file %s, line %d", __FILE__, __LINE__);
		return ERROR_FAIL;
	}

	bytes = DIV_ROUND_UP(num_bits, 8);
	if (cmd->ir_scan) {
		free(last_ir_buf);
		last_ir_buf = (uint8_t *)malloc(bytes * sizeof(uint8_t));
		if (last_ir_buf == NULL) {
			LOG_ERROR("%s: malloc fail, file %s, line %d",
				__func__, __FILE__, __LINE__);
			ret = ERROR_FAIL;
			goto out;
		}
		memcpy(last_ir_buf, data_buf, bytes);
		last_ir_num_bits = num_bits;
	}
	snprintf(buf, sizeof(buf), "%s %d\n", cmd->ir_scan ? "ib" : "db", num_bits);
	ret = write_sock(buf, strlen(buf));
	if (ret != ERROR_OK) {
		LOG_ERROR("write_sock() fail, file %s, line %d",
			__FILE__, __LINE__);
		goto out;
	}
	ret = write_sock((char *)data_buf, bytes);
	if (ret != ERROR_OK) {
		LOG_ERROR("write_sock() fail, file %s, line %d",
			__FILE__, __LINE__);
		goto out;
	}
	ret = read_sock((char *)data_buf, bytes);
	if (ret != ERROR_OK) {
		LOG_ERROR("read_sock() fail, file %s, line %d",
			__FILE__, __LINE__);
		goto out;
	}

	ret = jtag_read_buffer(data_buf, cmd);
	if (ret != ERROR_OK) {
		LOG_ERROR("jtag_read_buffer() fail, file %s, line %d",
			__FILE__, __LINE__);
		goto out;
	}

out:
	free(data_buf);
	return ret;
}

static int jtag_dpi_runtest(int cycles)
{
	char buf[20];
	uint8_t *data_buf = last_ir_buf, *read_scan;
	int num_bits = last_ir_num_bits, bytes;
	int ret = ERROR_OK;

	if (data_buf == NULL) {
		LOG_ERROR("%s: NULL 'data_buf' argument, file %s, line %d",
			__func__, __FILE__, __LINE__);
		return ERROR_FAIL;
	}
	if (num_bits <= 0) {
		LOG_ERROR("%s: 'num_bits' invalid value, file %s, line %d",
			__func__, __FILE__, __LINE__);
		return ERROR_FAIL;
	}

	bytes = DIV_ROUND_UP(num_bits, 8);
	read_scan = (uint8_t *)malloc(bytes * sizeof(uint8_t));
	if (read_scan == NULL) {
		LOG_ERROR("%s: malloc fail, file %s, line %d",
			__func__, __FILE__, __LINE__);
		return ERROR_FAIL;
	}
	snprintf(buf, sizeof(buf), "ib %d\n", num_bits);
	while (cycles > 0) {
		ret = write_sock(buf, strlen(buf));
		if (ret != ERROR_OK) {
			LOG_ERROR("write_sock() fail, file %s, line %d",
				__FILE__, __LINE__);
			goto out;
		}
		ret = write_sock((char *)data_buf, bytes);
		if (ret != ERROR_OK) {
			LOG_ERROR("write_sock() fail, file %s, line %d",
				__FILE__, __LINE__);
			goto out;
		}
		ret = read_sock((char *)read_scan, bytes);
		if (ret != ERROR_OK) {
			LOG_ERROR("read_sock() fail, file %s, line %d",
				__FILE__, __LINE__);
			goto out;
		}

		cycles -= num_bits + 6;
	}

out:
	free(read_scan);
	return ret;
}

static int jtag_dpi_stableclocks(int cycles)
{
	return jtag_dpi_runtest(cycles);
}

static int jtag_dpi_execute_queue(void)
{
	struct jtag_command *cmd;
	int ret = ERROR_OK;

	for (cmd = jtag_command_queue; ret == ERROR_OK && cmd != NULL;
	     cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RUNTEST:
			ret = jtag_dpi_runtest(cmd->cmd.runtest->num_cycles);
			break;
		case JTAG_STABLECLOCKS:
			ret = jtag_dpi_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			/* Enter Test-Logic-Reset state by asserting TRST */
			if (cmd->cmd.statemove->end_state == TAP_RESET)
				jtag_dpi_reset(1, 0);
			break;
		case JTAG_PATHMOVE:
			/* unsupported */
			break;
		case JTAG_TMS:
			/* unsupported */
			break;
		case JTAG_SLEEP:
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			ret = jtag_dpi_scan(cmd->cmd.scan);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X",
				  cmd->type);
			ret = ERROR_FAIL;
			break;
		}
	}

	return ret;
}

static int jtag_dpi_init(void)
{
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		LOG_ERROR("socket: %s, function %s, file %s, line %d",
			strerror(errno), __func__, __FILE__, __LINE__);
		return ERROR_FAIL;
	}

	memset(&serv_addr, 0, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(server_port);

	if (server_address == NULL) {
		server_address = strdup(SERVER_ADDRESS);
		if (server_address == NULL) {
			LOG_ERROR("%s: strdup fail, file %s, line %d",
				__func__, __FILE__, __LINE__);
			return ERROR_FAIL;
		}
	}

	serv_addr.sin_addr.s_addr = inet_addr(server_address);

	if (serv_addr.sin_addr.s_addr == INADDR_NONE) {
		LOG_ERROR("inet_addr error occured");
		return ERROR_FAIL;
	}

	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		close(sockfd);
		LOG_ERROR("Can't connect to %s : %" PRIu16, server_address, server_port);
		return ERROR_FAIL;
	}
	if (serv_addr.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
		/* This increases performance dramatically for local
		* connections, which is the most likely arrangement
		* for a DPI connection. */
		int flag = 1;
		setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
	}

	LOG_INFO("Connection to %s : %" PRIu16 " succeed", server_address, server_port);

	return ERROR_OK;
}

static int jtag_dpi_quit(void)
{
	free(server_address);
	server_address = NULL;

	return close(sockfd);
}

COMMAND_HANDLER(jtag_dpi_set_port)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (CMD_ARGC == 0)
		LOG_INFO("Using server port %" PRIu16, server_port);
	else {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], server_port);
		LOG_INFO("Set server port to %" PRIu16, server_port);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jtag_dpi_set_address)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (CMD_ARGC == 0) {
		if (server_address == NULL) {
			server_address = strdup(SERVER_ADDRESS);
			if (server_address == NULL) {
				LOG_ERROR("%s: strdup fail, file %s, line %d",
					__func__, __FILE__, __LINE__);
				return ERROR_FAIL;
			}
		}
		LOG_INFO("Using server address %s", server_address);
	} else {
		free(server_address);
		server_address = strdup(CMD_ARGV[0]);
		if (server_address == NULL) {
			LOG_ERROR("%s: strdup fail, file %s, line %d",
				__func__, __FILE__, __LINE__);
			return ERROR_FAIL;
		}
		LOG_INFO("Set server address to %s", server_address);
	}

	return ERROR_OK;
}

static const struct command_registration jtag_dpi_command_handlers[] = {
	{
		.name = "jtag_dpi_set_port",
		.handler = &jtag_dpi_set_port,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the DPI server",
		.usage = "[port]",
	},
	{
		.name = "jtag_dpi_set_address",
		.handler = &jtag_dpi_set_address,
		.mode = COMMAND_CONFIG,
		.help = "set the address of the DPI server",
		.usage = "[address]",
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
	.reset = jtag_dpi_reset,
	.jtag_ops = &jtag_dpi_interface,
};
