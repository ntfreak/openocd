/***************************************************************************
 *   Copyright (C) 2014 by Franck Jullien                                  *
 *   franck.jullien@gmail.com                                              *
 *                                                                         *
 *   Based on ./src/server/telnet_server.c                                 *
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

#include <server/telnet_server.h>

#include "or1k_tap.h"
#include "or1k_du.h"
#include "jsp_server.h"

static char *jsp_port;

/**A skim of the relevant RFCs suggests that if my application simply sent the
 * characters IAC DONT LINEMODE (\377\376\042) as soon as the client connects,
 * the client should be forced into character mode. However it doesn't make any difference.
 */

static const char * const negotiate =
	"\xFF\xFB\x03"			/* IAC WILL Suppress Go Ahead */
	"\xFF\xFB\x01"			/* IAC WILL Echo */
	"\xFF\xFD\x03"			/* IAC DO Suppress Go Ahead */
	"\xFF\xFE\x01";			/* IAC DON'T Echo */

/* The only way we can detect that the socket is closed is the first time
 * we write to it, we will fail. Subsequent write operations will
 * succeed. Shudder!
 */
static int telnet_write(struct connection *connection, const void *data, int len)
{
	struct telnet_connection *t_con = connection->priv;
	if (t_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	if (connection_write(connection, data, len) == len)
		return ERROR_OK;
	t_con->closed = 1;
	return ERROR_SERVER_REMOTE_CLOSED;
}

static int jsp_poll_read(void *priv)
{
	struct jsp_service *jsp_service = (struct jsp_service *)priv;
	unsigned char out_buffer[10];
	unsigned char in_buffer[10];
	int out_len = 0;
	int in_len;

	if (!jsp_service->connection)
		return ERROR_FAIL;

	memset(out_buffer, 0, 10);

	or1k_adv_jtag_jsp_xfer(jsp_service->jtag_info, &out_len, out_buffer, &in_len, in_buffer);
	if (in_len)
		telnet_write(jsp_service->connection, in_buffer, in_len);

	return ERROR_OK;
}

static int jsp_new_connection(struct connection *connection)
{
	struct telnet_connection *telnet_connection = malloc(sizeof(struct telnet_connection));
	struct jsp_service *jsp_service = connection->service->priv;

	connection->priv = telnet_connection;

	/* initialize telnet connection information */
	telnet_connection->closed = 0;
	telnet_connection->line_size = 0;
	telnet_connection->line_cursor = 0;
	telnet_connection->state = TELNET_STATE_DATA;

	/* negotiate telnet options */
	telnet_write(connection, negotiate, strlen(negotiate));

	/* print connection banner */
	if (jsp_service->banner) {
		telnet_write(connection, jsp_service->banner, strlen(jsp_service->banner));
		telnet_write(connection, "\r\n", 2);
	}

	jsp_service->connection = connection;

	int retval = target_register_timer_callback(&jsp_poll_read, 1,
		TARGET_TIMER_TYPE_PERIODIC, jsp_service);
	if (ERROR_OK != retval)
		return retval;

	return ERROR_OK;
}

static int jsp_input(struct connection *connection)
{
	int bytes_read;
	unsigned char buffer[TELNET_BUFFER_SIZE];
	unsigned char *buf_p;
	struct telnet_connection *t_con = connection->priv;
	struct jsp_service *jsp_service = connection->service->priv;

	bytes_read = connection_read(connection, buffer, TELNET_BUFFER_SIZE);

	if (bytes_read == 0)
		return ERROR_SERVER_REMOTE_CLOSED;
	else if (bytes_read == -1) {
		LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	buf_p = buffer;
	while (bytes_read) {
		switch (t_con->state) {
			case TELNET_STATE_DATA:
				if (*buf_p == 0xff)
					t_con->state = TELNET_STATE_IAC;
				else {
					int out_len = 1;
					int in_len;
					unsigned char in_buffer[10];
					or1k_adv_jtag_jsp_xfer(jsp_service->jtag_info,
							       &out_len, buf_p, &in_len,
							       in_buffer);
					if (in_len)
						telnet_write(connection,
							     in_buffer, in_len);
				}
				break;
			case TELNET_STATE_IAC:
				switch (*buf_p) {
				case 0xfe:
					t_con->state = TELNET_STATE_DONT;
					break;
				case 0xfd:
					t_con->state = TELNET_STATE_DO;
					break;
				case 0xfc:
					t_con->state = TELNET_STATE_WONT;
					break;
				case 0xfb:
					t_con->state = TELNET_STATE_WILL;
					break;
				}
				break;
			case TELNET_STATE_SB:
				break;
			case TELNET_STATE_SE:
				break;
			case TELNET_STATE_WILL:
			case TELNET_STATE_WONT:
			case TELNET_STATE_DO:
			case TELNET_STATE_DONT:
				t_con->state = TELNET_STATE_DATA;
				break;
			default:
				LOG_ERROR("unknown telnet state");
				exit(-1);
		}

		bytes_read--;
		buf_p++;
	}

	return ERROR_OK;
}

static int jsp_connection_closed(struct connection *connection)
{
	struct jsp_service *jsp_service = connection->service->priv;

	int retval = target_unregister_timer_callback(&jsp_poll_read, jsp_service);
	if (ERROR_OK != retval)
		return retval;

	free(connection->priv);
	connection->priv = NULL;
	return ERROR_OK;
}

int jsp_init(struct or1k_jtag *jtag_info, char *banner)
{
	struct jsp_service *jsp_service = malloc(sizeof(struct jsp_service));
	jsp_service->banner = banner;
	jsp_service->jtag_info = jtag_info;

	return add_service("jsp",
		jsp_port,
		1,
		jsp_new_connection,
		jsp_input,
		jsp_connection_closed,
		jsp_service,
		NULL);
}

COMMAND_HANDLER(handle_jsp_port_command)
{
	return CALL_COMMAND_HANDLER(server_pipe_command, &jsp_port);
}

static const struct command_registration jsp_command_handlers[] = {
	{
		.name = "jsp_port",
		.handler = handle_jsp_port_command,
		.mode = COMMAND_ANY,
		.help = "Specify port on which to listen "
			"for incoming JSP telnet connections.",
		.usage = "[port_num]",
	},
	COMMAND_REGISTRATION_DONE
};

int jsp_register_commands(struct command_context *cmd_ctx)
{
	jsp_port = strdup("7777");
	return register_commands(cmd_ctx, NULL, jsp_command_handlers);
}

void jsp_service_free(void)
{
	free(jsp_port);
}
