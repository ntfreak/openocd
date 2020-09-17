/*
 * Copyright (C) 2016-2017 by Marc Schink <dev@zapb.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <rtt/rtt.h>

#include "server.h"
#include "rtt_server.h"

/**
 * @file
 *
 * RTT server.
 *
 * This server allows access to Real Time Transfer (RTT) channels via TCP
 * connections.
 */

struct rtt_service {
	unsigned int channel;
};

static int read_callback(unsigned int channel, const uint8_t *buffer,
		size_t length, void *user_data)
{
	int ret;
	struct connection *connection;
	size_t offset;

	connection = (struct connection *)user_data;
	offset = 0;

	while (offset < length) {
		ret = connection_write(connection, buffer + offset, length - offset);

		if (ret < 0) {
			LOG_ERROR("Failed to write data to socket.");
			return ERROR_FAIL;
		}

		offset += ret;
	}

	return ERROR_OK;
}

static int rtt_new_connection(struct connection *connection)
{
	int ret;
	struct rtt_service *service;

	service = connection->service->priv;

	LOG_DEBUG("rtt: New connection for channel %u", service->channel);

	ret = rtt_register_sink(service->channel, &read_callback, connection);

	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int rtt_connection_closed(struct connection *connection)
{
	struct rtt_service *service;

	service = (struct rtt_service *)connection->service->priv;
	rtt_unregister_sink(service->channel, &read_callback, connection);

	LOG_DEBUG("rtt: Connection for channel %u closed", service->channel);

	return ERROR_OK;
}

static int rtt_input(struct connection *connection)
{
	int bytes_read;
	unsigned char buffer[1024];
	struct rtt_service *service;
	size_t length;

	service = (struct rtt_service *)connection->service->priv;
	bytes_read = connection_read(connection, buffer, sizeof(buffer));

	if (!bytes_read)
		return ERROR_SERVER_REMOTE_CLOSED;
	else if (bytes_read < 0) {
		LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	length = bytes_read;
	rtt_write_channel(service->channel, buffer, &length);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_rtt_start_command)
{
	int ret;
	struct rtt_service *service;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	service = malloc(sizeof(struct rtt_service));

	if (!service)
		return ERROR_FAIL;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], service->channel);

	ret = add_service("rtt", CMD_ARGV[0], CONNECTION_LIMIT_UNLIMITED,
		rtt_new_connection, rtt_input, rtt_connection_closed, service, NULL);

	if (ret != ERROR_OK) {
		free(service);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_rtt_stop_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	remove_service("rtt", CMD_ARGV[0]);

	return ERROR_OK;
}

static const struct command_registration rtt_server_subcommand_handlers[] = {
	{
		.name = "start",
		.handler = handle_rtt_start_command,
		.mode = COMMAND_ANY,
		.help = "Start a RTT server",
		.usage = "<port> <channel>"
	},
	{
		.name = "stop",
		.handler = handle_rtt_stop_command,
		.mode = COMMAND_ANY,
		.help = "Stop a RTT server",
		.usage = "<port>"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration rtt_server_command_handlers[] = {
	{
		.name = "server",
		.mode = COMMAND_ANY,
		.help = "RTT server",
		.usage = "",
		.chain = rtt_server_subcommand_handlers
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration rtt_command_handlers[] = {
	{
		.name = "rtt",
		.mode = COMMAND_ANY,
		.help = "RTT",
		.usage = "",
		.chain = rtt_server_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

int rtt_server_register_commands(struct command_context *ctx)
{
	return register_commands(ctx, NULL, rtt_command_handlers);
}
