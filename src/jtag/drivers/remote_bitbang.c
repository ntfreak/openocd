/***************************************************************************
 *   Copyright (C) 2011 by Richard Uhler                                   *
 *   ruhler@mit.edu                                                        *
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

#ifndef _WIN32
#include <sys/un.h>
#include <netdb.h>
#endif
#include <jtag/interface.h>
#include "bitbang.h"

/* arbitrary limit on host name length: */
#define REMOTE_BITBANG_HOST_MAX 255

static char *remote_bitbang_host;
static char *remote_bitbang_port;

static FILE *remote_bitbang_file;
static int remote_bitbang_fd;

/* Circular buffer. When start == end, the buffer is empty. */
static char remote_bitbang_buf[64];
static unsigned remote_bitbang_start;
static unsigned remote_bitbang_end;

static int remote_bitbang_buf_full(void)
{
	return remote_bitbang_end ==
		((remote_bitbang_start + sizeof(remote_bitbang_buf) - 1) %
		 sizeof(remote_bitbang_buf));
}

/* Read any incoming data, placing it into the buffer. */
static int remote_bitbang_fill_buf(void)
{
	socket_nonblock(remote_bitbang_fd);
	while (!remote_bitbang_buf_full()) {
		unsigned contiguous_available_space;
		if (remote_bitbang_end >= remote_bitbang_start) {
			contiguous_available_space = sizeof(remote_bitbang_buf) -
				remote_bitbang_end;
			if (remote_bitbang_start == 0)
				contiguous_available_space -= 1;
		} else {
			contiguous_available_space = remote_bitbang_start -
				remote_bitbang_end - 1;
		}
		ssize_t count = read(remote_bitbang_fd,
				remote_bitbang_buf + remote_bitbang_end,
				contiguous_available_space);
		if (count > 0) {
			remote_bitbang_end += count;
			if (remote_bitbang_end == sizeof(remote_bitbang_buf))
				remote_bitbang_end = 0;
		} else if (count == 0) {
			return ERROR_OK;
		} else if (count < 0) {
			if (errno == EAGAIN) {
				return ERROR_OK;
			} else {
				LOG_ERROR("remote_bitbang_fill_buf: %s (%d)",
						strerror(errno), errno);
				return ERROR_FAIL;
			}
		}
	}

	return ERROR_OK;
}

static int remote_bitbang_putc(int c)
{
	if (EOF == fputc(c, remote_bitbang_file)) {
		LOG_ERROR("remote_bitbang_putc: %s", strerror(errno));
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int remote_bitbang_quit(void)
{
	if (EOF == fputc('Q', remote_bitbang_file)) {
		LOG_ERROR("fputs: %s", strerror(errno));
		return ERROR_FAIL;
	}

	if (EOF == fflush(remote_bitbang_file)) {
		LOG_ERROR("fflush: %s", strerror(errno));
		return ERROR_FAIL;
	}

	/* We only need to close one of the FILE*s, because they both use the same */
	/* underlying file descriptor. */
	if (EOF == fclose(remote_bitbang_file)) {
		LOG_ERROR("fclose: %s", strerror(errno));
		return ERROR_FAIL;
	}

	free(remote_bitbang_host);
	free(remote_bitbang_port);

	LOG_INFO("remote_bitbang interface quit");
	return ERROR_OK;
}

static bb_value_t char_to_int(int c)
{
	switch (c) {
		case '0':
			return BB_LOW;
		case '1':
			return BB_HIGH;
		default:
			remote_bitbang_quit();
			LOG_ERROR("remote_bitbang: invalid read response: %c(%i)", c, c);
			return BB_ERROR;
	}
}

/* Get the next read response. */
static bb_value_t remote_bitbang_rread(void)
{
	if (EOF == fflush(remote_bitbang_file)) {
		remote_bitbang_quit();
		LOG_ERROR("fflush: %s", strerror(errno));
		return BB_ERROR;
	}

	/* Enable blocking access. */
	socket_block(remote_bitbang_fd);
	char c;
	ssize_t count = read(remote_bitbang_fd, &c, 1);
	if (count == 1) {
		return char_to_int(c);
	} else {
		remote_bitbang_quit();
		LOG_ERROR("read: count=%d, error=%s", (int) count, strerror(errno));
		return BB_ERROR;
	}
}

static int remote_bitbang_sample(void)
{
	if (remote_bitbang_fill_buf() != ERROR_OK)
		return ERROR_FAIL;
	assert(!remote_bitbang_buf_full());
	return remote_bitbang_putc('R');
}

static bb_value_t remote_bitbang_read_sample(void)
{
	if (remote_bitbang_start != remote_bitbang_end) {
		int c = remote_bitbang_buf[remote_bitbang_start];
		remote_bitbang_start =
			(remote_bitbang_start + 1) % sizeof(remote_bitbang_buf);
		return char_to_int(c);
	}
	return remote_bitbang_rread();
}

static int remote_bitbang_write(int tck, int tms, int tdi)
{
	char c = '0' + ((tck ? 0x4 : 0x0) | (tms ? 0x2 : 0x0) | (tdi ? 0x1 : 0x0));
	return remote_bitbang_putc(c);
}

static int remote_bitbang_reset(int trst, int srst)
{
	char c = 'r' + ((trst ? 0x2 : 0x0) | (srst ? 0x1 : 0x0));
	return remote_bitbang_putc(c);
}

static int remote_bitbang_blink(int on)
{
	char c = on ? 'B' : 'b';
	return remote_bitbang_putc(c);
}

static struct bitbang_interface remote_bitbang_bitbang = {
	.buf_size = sizeof(remote_bitbang_buf) - 1,
	.sample = &remote_bitbang_sample,
	.read_sample = &remote_bitbang_read_sample,
	.write = &remote_bitbang_write,
	.blink = &remote_bitbang_blink,
};

static int remote_bitbang_init_tcp(void)
{
	struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
	struct addrinfo *result, *rp;
	int fd = 0;

	LOG_INFO("Connecting to %s:%s",
			remote_bitbang_host ? remote_bitbang_host : "localhost",
			remote_bitbang_port);

	/* Obtain address(es) matching host/port */
	int s = getaddrinfo(remote_bitbang_host, remote_bitbang_port, &hints, &result);
	if (s != 0) {
		LOG_ERROR("getaddrinfo: %s\n", gai_strerror(s));
		return ERROR_FAIL;
	}

	/* getaddrinfo() returns a list of address structures.
	 Try each address until we successfully connect(2).
	 If socket(2) (or connect(2)) fails, we (close the socket
	 and) try the next address. */

	for (rp = result; rp != NULL ; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (fd == -1)
			continue;

		if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1)
			break; /* Success */

		close(fd);
	}

	freeaddrinfo(result); /* No longer needed */

	if (rp == NULL) { /* No address succeeded */
		LOG_ERROR("Failed to connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	return fd;
}

static int remote_bitbang_init_unix(void)
{
	if (remote_bitbang_host == NULL) {
		LOG_ERROR("host/socket not specified");
		return ERROR_FAIL;
	}

	LOG_INFO("Connecting to unix socket %s", remote_bitbang_host);
	int fd = socket(PF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		LOG_ERROR("socket: %s", strerror(errno));
		return ERROR_FAIL;
	}

	struct sockaddr_un addr;
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, remote_bitbang_host, sizeof(addr.sun_path));
	addr.sun_path[sizeof(addr.sun_path)-1] = '\0';

	if (connect(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_un)) < 0) {
		LOG_ERROR("connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	return fd;
}

static int remote_bitbang_init(void)
{
	bitbang_interface = &remote_bitbang_bitbang;

	remote_bitbang_start = 0;
	remote_bitbang_end = 0;

	LOG_INFO("Initializing remote_bitbang driver");
	if (remote_bitbang_port == NULL)
		remote_bitbang_fd = remote_bitbang_init_unix();
	else
		remote_bitbang_fd = remote_bitbang_init_tcp();

	if (remote_bitbang_fd < 0)
		return remote_bitbang_fd;

	remote_bitbang_file = fdopen(remote_bitbang_fd, "w+");
	if (remote_bitbang_file == NULL) {
		LOG_ERROR("fdopen: failed to open write stream");
		close(remote_bitbang_fd);
		return ERROR_FAIL;
	}

	LOG_INFO("remote_bitbang driver initialized");
	return ERROR_OK;
}

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_port_command)
{
	if (CMD_ARGC == 1) {
		uint16_t port;
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], port);
		free(remote_bitbang_port);
		remote_bitbang_port = port == 0 ? NULL : strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_host_command)
{
	if (CMD_ARGC == 1) {
		free(remote_bitbang_host);
		remote_bitbang_host = strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const struct command_registration remote_bitbang_command_handlers[] = {
	{
		.name = "remote_bitbang_port",
		.handler = remote_bitbang_handle_remote_bitbang_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the port to use to connect to the remote jtag.\n"
			"  if 0 or unset, use unix sockets to connect to the remote jtag.",
		.usage = "port_number",
	},
	{
		.name = "remote_bitbang_host",
		.handler = remote_bitbang_handle_remote_bitbang_host_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the host to use to connect to the remote jtag.\n"
			"  if port is 0 or unset, this is the name of the unix socket to use.",
		.usage = "host_name",
	},
	COMMAND_REGISTRATION_DONE,
};

static struct jtag_interface remote_bitbang_interface = {
	.execute_queue = &bitbang_execute_queue,
};

struct adapter_driver remote_bitbang_adapter_driver = {
	.name = "remote_bitbang",
	.transports = jtag_only,
	.commands = remote_bitbang_command_handlers,

	.init = &remote_bitbang_init,
	.quit = &remote_bitbang_quit,
	.reset = &remote_bitbang_reset,

	.jtag_ops = &remote_bitbang_interface,
};
