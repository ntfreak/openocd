/***************************************************************************
 *   Copyright (C) 2015  Paul Fertser <fercerpav@gmail.com>                *
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

#include <target/target.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>
#include <target/armv7m_trace.h>
#include <jtag/interface.h>

#define TRACE_BUF_SIZE	4096

static int armv7m_poll_trace(void *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint8_t buf[TRACE_BUF_SIZE];
	size_t size = sizeof(buf);
	int retval;

	retval = adapter_poll_trace(buf, &size);
	if (retval != ERROR_OK || !size)
		return retval;

	target_call_trace_callbacks(target, size, buf);

	switch (armv7m->trace_config.internal_channel) {
	case TRACE_INTERNAL_CHANNEL_FILE:
		if (armv7m->trace_config.trace_file != NULL) {
			if (fwrite(buf, 1, size, armv7m->trace_config.trace_file) == size)
				fflush(armv7m->trace_config.trace_file);
			else {
				LOG_ERROR("Error writing to the trace destination file");
				return ERROR_FAIL;
			}
		}
		break;
	case TRACE_INTERNAL_CHANNEL_TCP:
		if (armv7m->trace_config.trace_service != NULL) {
			/* broadcast to all service connections */
			struct connection *connection = armv7m->trace_config.trace_service->connections;
			retval = ERROR_OK;
			while (connection) {
				if (connection_write(connection, buf, size) != (int) size)
					retval = ERROR_FAIL;

				connection = connection->next;
			}

			if (retval != ERROR_OK) {
				LOG_ERROR("Error streaming the trace to TCP/IP port");
				return ERROR_FAIL;
			}
		}
		break;
	case TRACE_INTERNAL_CHANNEL_TCL_ONLY:
		/* nothing to do :
		 * the trace data is sent to TCL by calling the target_call_trace_callbacks
		 **/
		break;
	default:
		LOG_ERROR("unsupported trace internal channel");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int armv7m_trace_tpiu_config(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_trace_config *trace_config = &armv7m->trace_config;
	uint16_t prescaler;
	int retval;

	target_unregister_timer_callback(armv7m_poll_trace, target);

	retval = adapter_config_trace(trace_config->config_type == TRACE_CONFIG_TYPE_INTERNAL,
		trace_config->pin_protocol, trace_config->port_size,
		&trace_config->trace_freq, trace_config->traceclkin_freq, &prescaler);

	if (retval != ERROR_OK)
		return retval;

	if (trace_config->config_type == TRACE_CONFIG_TYPE_EXTERNAL) {
		prescaler = trace_config->traceclkin_freq / trace_config->trace_freq;

		if (trace_config->traceclkin_freq % trace_config->trace_freq) {
			prescaler++;

			int trace_freq = trace_config->traceclkin_freq / prescaler;
			LOG_INFO("Can not obtain %u trace port frequency from %u "
				"TRACECLKIN frequency, using %u instead",
				trace_config->trace_freq, trace_config->traceclkin_freq,
				trace_freq);

			trace_config->trace_freq = trace_freq;
		}
	}

	if (!trace_config->trace_freq) {
		LOG_ERROR("Trace port frequency is 0, can't enable TPIU");
		return ERROR_FAIL;
	}

	retval = target_write_u32(target, TPIU_CSPSR, 1 << trace_config->port_size);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, TPIU_ACPR, prescaler - 1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, TPIU_SPPR, trace_config->pin_protocol);
	if (retval != ERROR_OK)
		return retval;

	uint32_t ffcr;
	retval = target_read_u32(target, TPIU_FFCR, &ffcr);
	if (retval != ERROR_OK)
		return retval;
	if (trace_config->formatter)
		ffcr |= (1 << 1);
	else
		ffcr &= ~(1 << 1);
	retval = target_write_u32(target, TPIU_FFCR, ffcr);
	if (retval != ERROR_OK)
		return retval;

	if (trace_config->config_type == TRACE_CONFIG_TYPE_INTERNAL)
		target_register_timer_callback(armv7m_poll_trace, 1,
		TARGET_TIMER_TYPE_PERIODIC, target);

	target_call_event_callbacks(target, TARGET_EVENT_TRACE_CONFIG);

	return ERROR_OK;
}

int armv7m_trace_itm_config(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_trace_config *trace_config = &armv7m->trace_config;
	int retval;

	retval = target_write_u32(target, ITM_LAR, ITM_LAR_KEY);
	if (retval != ERROR_OK)
		return retval;

	/* Enable ITM, TXENA, set TraceBusID and other parameters */
	retval = target_write_u32(target, ITM_TCR, (1 << 0) | (1 << 3) |
				  (trace_config->itm_diff_timestamps << 1) |
				  (trace_config->itm_synchro_packets << 2) |
				  (trace_config->itm_async_timestamps << 4) |
				  (trace_config->itm_ts_prescale << 8) |
				  (trace_config->trace_bus_id << 16));
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < 8; i++) {
		retval = target_write_u32(target, ITM_TER0 + i * 4,
					  trace_config->itm_ter[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static void close_trace_channel(struct armv7m_common *armv7m)
{
	switch (armv7m->trace_config.internal_channel) {
	case TRACE_INTERNAL_CHANNEL_FILE:
		if (armv7m->trace_config.trace_file)
			fclose(armv7m->trace_config.trace_file);
		armv7m->trace_config.trace_file = NULL;
		break;
	case TRACE_INTERNAL_CHANNEL_TCP:
		if (armv7m->trace_config.trace_service)
			remove_service(armv7m->trace_config.trace_service->name, armv7m->trace_config.trace_service->port);
		armv7m->trace_config.trace_service = NULL;
		break;
	case TRACE_INTERNAL_CHANNEL_TCL_ONLY:
		/* nothing to do:
		 * the trace polling is disabled in the beginning of armv7m_trace_tpiu_config
		 **/
		break;
	default:
		LOG_ERROR("unsupported trace internal channel");
	}
}

static int trace_new_connection(struct connection *connection)
{
	/* nothing to do */
	return ERROR_OK;
}

static int trace_input(struct connection *connection)
{
	/* create a dummy buffer to check if the connection is still active */
	const int buf_len = 100;
	unsigned char buf[buf_len];
	int bytes_read = connection_read(connection, buf, buf_len);

	if (bytes_read == 0)
		return ERROR_SERVER_REMOTE_CLOSED;
	else if (bytes_read == -1) {
		LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	return ERROR_OK;
}

static int trace_connection_closed(struct connection *connection)
{
	/* nothing to do, no connection->priv to free */
	return ERROR_OK;
}

extern struct command_context *global_cmd_ctx;

int armv7m_trace_tpiu_exit(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);

	if (global_cmd_ctx->mode == COMMAND_CONFIG ||
		armv7m->trace_config.config_type == TRACE_CONFIG_TYPE_DISABLED)
		return ERROR_OK;

	close_trace_channel(armv7m);
	armv7m->trace_config.config_type = TRACE_CONFIG_TYPE_DISABLED;
	return armv7m_trace_tpiu_config(target);
}

COMMAND_HANDLER(handle_tpiu_config_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7m_common *armv7m = target_to_armv7m(target);

	unsigned int cmd_idx = 0;

	if (CMD_ARGC == cmd_idx)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (!strcmp(CMD_ARGV[cmd_idx], "disable")) {
		if (CMD_ARGC == cmd_idx + 1) {
			close_trace_channel(armv7m);

			armv7m->trace_config.config_type = TRACE_CONFIG_TYPE_DISABLED;
			if (CMD_CTX->mode == COMMAND_EXEC)
				return armv7m_trace_tpiu_config(target);
			else
				return ERROR_OK;
		}
	} else if (!strcmp(CMD_ARGV[cmd_idx], "external") ||
		   !strcmp(CMD_ARGV[cmd_idx], "internal")) {
		close_trace_channel(armv7m);

		armv7m->trace_config.config_type = TRACE_CONFIG_TYPE_EXTERNAL;
		if (!strcmp(CMD_ARGV[cmd_idx], "internal")) {
			cmd_idx++;
			if (CMD_ARGC == cmd_idx)
				return ERROR_COMMAND_SYNTAX_ERROR;

			armv7m->trace_config.config_type = TRACE_CONFIG_TYPE_INTERNAL;
			armv7m->trace_config.internal_channel = TRACE_INTERNAL_CHANNEL_TCL_ONLY;

			if (strcmp(CMD_ARGV[cmd_idx], "-") != 0) {
				if (CMD_ARGV[cmd_idx][0] == ':') {
					armv7m->trace_config.internal_channel = TRACE_INTERNAL_CHANNEL_TCP;

					int ret = add_service("armv7m_trace", &(CMD_ARGV[cmd_idx][1]),
							CONNECTION_LIMIT_UNLIMITED, trace_new_connection, trace_input,
							trace_connection_closed, NULL, &armv7m->trace_config.trace_service);
					if (ret != ERROR_OK) {
						LOG_ERROR("Can't configure trace TCP port");
						return ERROR_FAIL;
					}
				} else {
					armv7m->trace_config.internal_channel = TRACE_INTERNAL_CHANNEL_FILE;
					armv7m->trace_config.trace_file = fopen(CMD_ARGV[cmd_idx], "ab");
					if (!armv7m->trace_config.trace_file) {
						LOG_ERROR("Can't open trace destination file");
						return ERROR_FAIL;
					}
				}
			}
		}
		cmd_idx++;
		if (CMD_ARGC == cmd_idx)
			return ERROR_COMMAND_SYNTAX_ERROR;

		if (!strcmp(CMD_ARGV[cmd_idx], "sync")) {
			armv7m->trace_config.pin_protocol = TPIU_PIN_PROTOCOL_SYNC;

			cmd_idx++;
			if (CMD_ARGC == cmd_idx)
				return ERROR_COMMAND_SYNTAX_ERROR;

			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[cmd_idx], armv7m->trace_config.port_size);
		} else {
			if (!strcmp(CMD_ARGV[cmd_idx], "manchester"))
				armv7m->trace_config.pin_protocol = TPIU_PIN_PROTOCOL_ASYNC_MANCHESTER;
			else if (!strcmp(CMD_ARGV[cmd_idx], "uart"))
				armv7m->trace_config.pin_protocol = TPIU_PIN_PROTOCOL_ASYNC_UART;
			else
				return ERROR_COMMAND_SYNTAX_ERROR;

			cmd_idx++;
			if (CMD_ARGC == cmd_idx)
				return ERROR_COMMAND_SYNTAX_ERROR;

			COMMAND_PARSE_ON_OFF(CMD_ARGV[cmd_idx], armv7m->trace_config.formatter);
		}

		cmd_idx++;
		if (CMD_ARGC == cmd_idx)
			return ERROR_COMMAND_SYNTAX_ERROR;

		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[cmd_idx], armv7m->trace_config.traceclkin_freq);

		cmd_idx++;
		if (CMD_ARGC != cmd_idx) {
			COMMAND_PARSE_NUMBER(uint, CMD_ARGV[cmd_idx], armv7m->trace_config.trace_freq);
			cmd_idx++;
		} else {
			if (armv7m->trace_config.config_type != TRACE_CONFIG_TYPE_INTERNAL) {
				LOG_ERROR("Trace port frequency can't be omitted in external capture mode");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			armv7m->trace_config.trace_freq = 0;
		}

		if (CMD_ARGC == cmd_idx) {
			if (CMD_CTX->mode == COMMAND_EXEC)
				return armv7m_trace_tpiu_config(target);
			else
				return ERROR_OK;
		}
	}

	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(handle_itm_port_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	unsigned int reg_idx;
	uint8_t port;
	bool enable;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], port);
	COMMAND_PARSE_ON_OFF(CMD_ARGV[1], enable);
	reg_idx = port / 32;
	port = port % 32;
	if (enable)
		armv7m->trace_config.itm_ter[reg_idx] |= (1 << port);
	else
		armv7m->trace_config.itm_ter[reg_idx] &= ~(1 << port);

	if (CMD_CTX->mode == COMMAND_EXEC)
		return armv7m_trace_itm_config(target);
	else
		return ERROR_OK;
}

COMMAND_HANDLER(handle_itm_ports_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7m_common *armv7m = target_to_armv7m(target);
	bool enable;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], enable);
	memset(armv7m->trace_config.itm_ter, enable ? 0xff : 0,
	       sizeof(armv7m->trace_config.itm_ter));

	if (CMD_CTX->mode == COMMAND_EXEC)
		return armv7m_trace_itm_config(target);
	else
		return ERROR_OK;
}

static const struct command_registration tpiu_command_handlers[] = {
	{
		.name = "config",
		.handler = handle_tpiu_config_command,
		.mode = COMMAND_ANY,
		.help = "Configure TPIU features",
		.usage = "(disable | "
		"((external | internal (<filename> | <:port> | -)) "
		"(sync <port width> | ((manchester | uart) <formatter enable>)) "
		"<TRACECLKIN freq> [<trace freq>]))",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration itm_command_handlers[] = {
	{
		.name = "port",
		.handler = handle_itm_port_command,
		.mode = COMMAND_ANY,
		.help = "Enable or disable ITM stimulus port",
		.usage = "<port> (0|1|on|off)",
	},
	{
		.name = "ports",
		.handler = handle_itm_ports_command,
		.mode = COMMAND_ANY,
		.help = "Enable or disable all ITM stimulus ports",
		.usage = "(0|1|on|off)",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration armv7m_trace_command_handlers[] = {
	{
		.name = "tpiu",
		.mode = COMMAND_ANY,
		.help = "tpiu command group",
		.usage = "",
		.chain = tpiu_command_handlers,
	},
	{
		.name = "itm",
		.mode = COMMAND_ANY,
		.help = "itm command group",
		.usage = "",
		.chain = itm_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
