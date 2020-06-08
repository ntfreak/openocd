/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "telnet_server.h"
#include <target/target_request.h>
#include <helper/configuration.h>

static char *telnet_port;

static char *negotiate =
	"\xFF\xFB\x03"			/* IAC WILL Suppress Go Ahead */
	"\xFF\xFB\x01"			/* IAC WILL Echo */
	"\xFF\xFD\x03"			/* IAC DO Suppress Go Ahead */
	"\xFF\xFE\x01";			/* IAC DON'T Echo */

#define CTRL(c) (c - '@')
#define TELNET_HISTORY	".openocd_history"

/* The only way we can detect that the socket is closed is the first time
 * we write to it, we will fail. Subsequent write operations will
 * succeed. Shudder!
 */
static int telnet_write(struct connection *connection, const void *data,
	int len)
{
	struct telnet_connection *t_con = connection->priv;
	if (t_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	if (connection_write(connection, data, len) == len)
		return ERROR_OK;
	t_con->closed = true;
	return ERROR_SERVER_REMOTE_CLOSED;
}

static int telnet_prompt(struct connection *connection)
{
	struct telnet_connection *t_con = connection->priv;

	return telnet_write(connection, t_con->prompt, strlen(t_con->prompt));
}

static int telnet_outputline(struct connection *connection, const char *line)
{
	int len;

	/* process lines in buffer */
	while (*line) {
		char *line_end = strchr(line, '\n');

		if (line_end)
			len = line_end-line;
		else
			len = strlen(line);

		telnet_write(connection, line, len);
		if (line_end) {
			telnet_write(connection, "\r\n", 2);
			line += len + 1;
		} else
			line += len;
	}

	return ERROR_OK;
}

static int telnet_output(struct command_context *cmd_ctx, const char *line)
{
	struct connection *connection = cmd_ctx->output_handler_priv;

	return telnet_outputline(connection, line);
}

static void telnet_log_callback(void *priv, const char *file, unsigned line,
	const char *function, const char *string)
{
	struct connection *connection = priv;
	struct telnet_connection *t_con = connection->priv;
	size_t i;
	size_t tmp;

	/* If the prompt is not visible, simply output the message. */
	if (!t_con->prompt_visible) {
		telnet_outputline(connection, string);
		return;
	}

	/* Clear the command line. */
	tmp = strlen(t_con->prompt) + t_con->line_size;

	for (i = 0; i < tmp; i += 16)
		telnet_write(connection, "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b",
			MIN(tmp - i, 16));

	for (i = 0; i < tmp; i += 16)
		telnet_write(connection, "                ", MIN(tmp - i, 16));

	for (i = 0; i < tmp; i += 16)
		telnet_write(connection, "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b",
			MIN(tmp - i, 16));

	telnet_outputline(connection, string);

	/* Put the command line to its previous state. */
	telnet_prompt(connection);
	telnet_write(connection, t_con->line, t_con->line_size);

	for (i = t_con->line_cursor; i < t_con->line_size; i++)
		telnet_write(connection, "\b", 1);
}

static void telnet_load_history(struct telnet_connection *t_con)
{
	FILE *histfp;
	char buffer[TELNET_BUFFER_SIZE];
	int i = 0;

	char *history = get_home_dir(TELNET_HISTORY);

	if (history == NULL) {
		LOG_INFO("unable to get user home directory, telnet history will be disabled");
		return;
	}

	histfp = fopen(history, "rb");

	if (histfp) {

		while (fgets(buffer, sizeof(buffer), histfp) != NULL) {

			char *p = strchr(buffer, '\n');
			if (p)
				*p = '\0';
			if (buffer[0] && i < TELNET_LINE_HISTORY_SIZE)
				t_con->history[i++] = strdup(buffer);
		}

		t_con->next_history = i;
		t_con->next_history %= TELNET_LINE_HISTORY_SIZE;
		/* try to set to last entry - 1, that way we skip over any exit/shutdown cmds */
		t_con->current_history = t_con->next_history > 0 ? i - 1 : 0;
		fclose(histfp);
	}

	free(history);
}

static void telnet_save_history(struct telnet_connection *t_con)
{
	FILE *histfp;
	int i;
	int num;

	char *history = get_home_dir(TELNET_HISTORY);

	if (history == NULL) {
		LOG_INFO("unable to get user home directory, telnet history will be disabled");
		return;
	}

	histfp = fopen(history, "wb");

	if (histfp) {

		num = TELNET_LINE_HISTORY_SIZE;
		i = t_con->current_history + 1;
		i %= TELNET_LINE_HISTORY_SIZE;

		while (t_con->history[i] == NULL && num > 0) {
			i++;
			i %= TELNET_LINE_HISTORY_SIZE;
			num--;
		}

		if (num > 0) {
			for (; num > 0; num--) {
				fprintf(histfp, "%s\n", t_con->history[i]);
				i++;
				i %= TELNET_LINE_HISTORY_SIZE;
			}
		}
		fclose(histfp);
	}

	free(history);
}

static int telnet_new_connection(struct connection *connection)
{
	struct telnet_connection *telnet_connection;
	struct telnet_service *telnet_service = connection->service->priv;
	int i;

	telnet_connection = malloc(sizeof(struct telnet_connection));

	if (!telnet_connection) {
		LOG_ERROR("Failed to allocate telnet connection.");
		return ERROR_FAIL;
	}

	connection->priv = telnet_connection;

	/* initialize telnet connection information */
	telnet_connection->closed = false;
	telnet_connection->line_size = 0;
	telnet_connection->line_cursor = 0;
	telnet_connection->prompt = strdup("> ");
	telnet_connection->prompt_visible = true;
	telnet_connection->state = TELNET_STATE_DATA;

	/* output goes through telnet connection */
	command_set_output_handler(connection->cmd_ctx, telnet_output, connection);

	/* negotiate telnet options */
	telnet_write(connection, negotiate, strlen(negotiate));

	/* print connection banner */
	if (telnet_service->banner) {
		telnet_write(connection, telnet_service->banner, strlen(telnet_service->banner));
		telnet_write(connection, "\r\n", 2);
	}

	/* the prompt is always placed at the line beginning */
	telnet_write(connection, "\r", 1);
	telnet_prompt(connection);

	/* initialize history */
	for (i = 0; i < TELNET_LINE_HISTORY_SIZE; i++)
		telnet_connection->history[i] = NULL;
	telnet_connection->next_history = 0;
	telnet_connection->current_history = 0;
	telnet_load_history(telnet_connection);

	log_add_callback(telnet_log_callback, connection);

	return ERROR_OK;
}

static void telnet_clear_line(struct connection *connection,
	struct telnet_connection *t_con)
{
	/* move to end of line */
	if (t_con->line_cursor < t_con->line_size)
		telnet_write(connection,
			t_con->line + t_con->line_cursor,
			t_con->line_size - t_con->line_cursor);

	/* backspace, overwrite with space, backspace */
	while (t_con->line_size > 0) {
		telnet_write(connection, "\b \b", 3);
		t_con->line_size--;
	}
	t_con->line_cursor = 0;
}

static void telnet_history_go(struct connection *connection, int idx)
{
	struct telnet_connection *t_con = connection->priv;

	if (t_con->history[idx]) {
		telnet_clear_line(connection, t_con);
		t_con->line_size = strlen(t_con->history[idx]);
		t_con->line_cursor = t_con->line_size;
		memcpy(t_con->line, t_con->history[idx], t_con->line_size);
		telnet_write(connection, t_con->line, t_con->line_size);
		t_con->current_history = idx;
	}
	t_con->state = TELNET_STATE_DATA;
}

static void telnet_history_up(struct connection *connection)
{
	struct telnet_connection *t_con = connection->priv;

	size_t last_history = (t_con->current_history > 0) ?
				t_con->current_history - 1 :
				TELNET_LINE_HISTORY_SIZE-1;
	telnet_history_go(connection, last_history);
}

static void telnet_history_down(struct connection *connection)
{
	struct telnet_connection *t_con = connection->priv;
	size_t next_history;

	next_history = (t_con->current_history + 1) % TELNET_LINE_HISTORY_SIZE;
	telnet_history_go(connection, next_history);
}

static int telnet_history_print(struct connection *connection)
{
	struct telnet_connection *tc;

	tc = connection->priv;

	for (size_t i = 1; i < TELNET_LINE_HISTORY_SIZE; i++) {
		char *line;

		/*
		 * The tc->next_history line contains empty string (unless NULL), thus
		 * it is not printed.
		 */
		line = tc->history[(tc->next_history + i) % TELNET_LINE_HISTORY_SIZE];

		if (line) {
			telnet_write(connection, line, strlen(line));
			telnet_write(connection, "\r\n\x00", 3);
		}
	}

	tc->line_size = 0;
	tc->line_cursor = 0;

	/* The prompt is always placed at the line beginning. */
	telnet_write(connection, "\r", 1);

	return telnet_prompt(connection);
}

static void telnet_move_cursor(struct connection *connection, size_t pos)
{
	struct telnet_connection *tc;
	size_t tmp;

	tc = connection->priv;

	if (pos < tc->line_cursor) {
		tmp = tc->line_cursor - pos;

		for (size_t i = 0; i < tmp; i += 16)
			telnet_write(connection, "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b",
				MIN(tmp - i, 16));
	} else {
		tmp = pos - tc->line_cursor;

		for (size_t i = 0; i < tmp; i += 16)
			telnet_write(connection, tc->line + tc->line_cursor + i,
				MIN(tmp - i, 16));
	}

	tc->line_cursor = pos;
}

static int telnet_input(struct connection *connection)
{
	int bytes_read;
	unsigned char buffer[TELNET_BUFFER_SIZE];
	unsigned char *buf_p;
	struct telnet_connection *t_con = connection->priv;
	struct command_context *command_context = connection->cmd_ctx;

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
					if (isprint(*buf_p)) {	/* printable character */
						/* watch buffer size leaving one spare character for
						 * string null termination */
						if (t_con->line_size == TELNET_LINE_MAX_SIZE-1) {
							/* output audible bell if buffer is full
							 * "\a" does not work, at least on windows */
							telnet_write(connection, "\x07", 1);
						} else if (t_con->line_cursor == t_con->line_size) {
							telnet_write(connection, buf_p, 1);
							t_con->line[t_con->line_size++] = *buf_p;
							t_con->line_cursor++;
						} else {
							size_t i;
							memmove(t_con->line + t_con->line_cursor + 1,
									t_con->line + t_con->line_cursor,
									t_con->line_size - t_con->line_cursor);
							t_con->line[t_con->line_cursor] = *buf_p;
							t_con->line_size++;
							telnet_write(connection,
									t_con->line + t_con->line_cursor,
									t_con->line_size - t_con->line_cursor);
							t_con->line_cursor++;
							for (i = t_con->line_cursor; i < t_con->line_size; i++)
								telnet_write(connection, "\b", 1);
						}
					} else {	/* non-printable */
						if (*buf_p == 0x1b) {	/* escape */
							t_con->state = TELNET_STATE_ESCAPE;
							t_con->last_escape = '\x00';
						} else if ((*buf_p == 0xd) || (*buf_p == 0xa)) {	/* CR/LF */
							int retval;

							/* skip over combinations with CR/LF and NUL characters */
							if ((bytes_read > 1) && ((*(buf_p + 1) == 0xa) ||
									(*(buf_p + 1) == 0xd))) {
								buf_p++;
								bytes_read--;
							}
							if ((bytes_read > 1) && (*(buf_p + 1) == 0)) {
								buf_p++;
								bytes_read--;
							}
							t_con->line[t_con->line_size] = 0;

							telnet_write(connection, "\r\n\x00", 3);

							if (strcmp(t_con->line, "history") == 0) {
								retval = telnet_history_print(connection);

								if (retval != ERROR_OK)
									return retval;

								continue;
							}

							/* save only non-blank not repeating lines in the history */
							char *prev_line = t_con->history[(t_con->current_history > 0) ?
									t_con->current_history - 1 : TELNET_LINE_HISTORY_SIZE-1];
							if (*t_con->line && (prev_line == NULL ||
									strcmp(t_con->line, prev_line))) {
								/* if the history slot is already taken, free it */
								free(t_con->history[t_con->next_history]);

								/* add line to history */
								t_con->history[t_con->next_history] = strdup(t_con->line);

								/* wrap history at TELNET_LINE_HISTORY_SIZE */
								t_con->next_history = (t_con->next_history + 1) %
										TELNET_LINE_HISTORY_SIZE;

								/* current history line starts at the new entry */
								t_con->current_history =
										t_con->next_history;

								free(t_con->history[t_con->current_history]);
								t_con->history[t_con->current_history] = strdup("");
							}

							t_con->line_size = 0;

							/* to suppress prompt in log callback during command execution */
							t_con->prompt_visible = false;

							if (strcmp(t_con->line, "shutdown") == 0)
								telnet_save_history(t_con);

							retval = command_run_line(command_context, t_con->line);

							t_con->line_cursor = 0;
							t_con->prompt_visible = true;

							if (retval == ERROR_COMMAND_CLOSE_CONNECTION)
								return ERROR_SERVER_REMOTE_CLOSED;

							/* the prompt is always * placed at the line beginning */
							telnet_write(connection, "\r", 1);

							retval = telnet_prompt(connection);
							if (retval == ERROR_SERVER_REMOTE_CLOSED)
								return ERROR_SERVER_REMOTE_CLOSED;

						} else if ((*buf_p == 0x7f) || (*buf_p == 0x8)) {	/* delete character */
							if (t_con->line_cursor > 0) {
								if (t_con->line_cursor != t_con->line_size) {
									size_t i;
									telnet_write(connection, "\b", 1);
									t_con->line_cursor--;
									t_con->line_size--;
									memmove(t_con->line + t_con->line_cursor,
											t_con->line + t_con->line_cursor + 1,
											t_con->line_size -
											t_con->line_cursor);

									telnet_write(connection,
											t_con->line + t_con->line_cursor,
											t_con->line_size -
											t_con->line_cursor);
									telnet_write(connection, " \b", 2);
									for (i = t_con->line_cursor; i < t_con->line_size; i++)
										telnet_write(connection, "\b", 1);
								} else {
									t_con->line_size--;
									t_con->line_cursor--;
									/* back space: move the 'printer' head one char
									 * back, overwrite with space, move back again */
									telnet_write(connection, "\b \b", 3);
								}
							}
						} else if (*buf_p == 0x15) /* clear line */
							telnet_clear_line(connection, t_con);
						else if (*buf_p == CTRL('B')) {	/* cursor left */
							if (t_con->line_cursor > 0) {
								telnet_write(connection, "\b", 1);
								t_con->line_cursor--;
							}
							t_con->state = TELNET_STATE_DATA;
						} else if (*buf_p == CTRL('F')) {	/* cursor right */
							if (t_con->line_cursor < t_con->line_size)
								telnet_write(connection, t_con->line + t_con->line_cursor++, 1);
							t_con->state = TELNET_STATE_DATA;
						} else if (*buf_p == CTRL('P'))		/* cursor up */
							telnet_history_up(connection);
						else if (*buf_p == CTRL('N'))		/* cursor down */
							telnet_history_down(connection);
						else if (*buf_p == CTRL('A'))
							telnet_move_cursor(connection, 0);
						else if (*buf_p == CTRL('E'))
							telnet_move_cursor(connection, t_con->line_size);
						else if (*buf_p == CTRL('K')) {         /* kill line to end */
							if (t_con->line_cursor < t_con->line_size) {
								/* overwrite with space, until end of line, move back */
								for (size_t i = t_con->line_cursor; i < t_con->line_size; i++)
									telnet_write(connection, " ", 1);
								for (size_t i = t_con->line_cursor; i < t_con->line_size; i++)
									telnet_write(connection, "\b", 1);
								t_con->line[t_con->line_cursor] = '\0';
								t_con->line_size = t_con->line_cursor;
							}
						} else
							LOG_DEBUG("unhandled nonprintable: %2.2x", *buf_p);
					}
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
			case TELNET_STATE_ESCAPE:
				if (t_con->last_escape == '[') {
					if (*buf_p == 'D') {	/* cursor left */
						if (t_con->line_cursor > 0) {
							telnet_write(connection, "\b", 1);
							t_con->line_cursor--;
						}
						t_con->state = TELNET_STATE_DATA;
					} else if (*buf_p == 'C') {	/* cursor right */
						if (t_con->line_cursor < t_con->line_size)
							telnet_write(connection,
									t_con->line + t_con->line_cursor++, 1);
						t_con->state = TELNET_STATE_DATA;
					} else if (*buf_p == 'A') {	/* cursor up */
						telnet_history_up(connection);
					} else if (*buf_p == 'B') {	/* cursor down */
						telnet_history_down(connection);
					} else if (*buf_p == '3')
						t_con->last_escape = *buf_p;
					else
						t_con->state = TELNET_STATE_DATA;
				} else if (t_con->last_escape == '3') {
					/* Remove character */
					if (*buf_p == '~') {
						if (t_con->line_cursor < t_con->line_size) {
							size_t i;
							t_con->line_size--;
							/* remove char from line buffer */
							memmove(t_con->line + t_con->line_cursor,
									t_con->line + t_con->line_cursor + 1,
									t_con->line_size - t_con->line_cursor);

							/* print remainder of buffer */
							telnet_write(connection, t_con->line + t_con->line_cursor,
									t_con->line_size - t_con->line_cursor);
							/* overwrite last char with whitespace */
							telnet_write(connection, " \b", 2);

							/* move back to cursor position*/
							for (i = t_con->line_cursor; i < t_con->line_size; i++)
								telnet_write(connection, "\b", 1);
						}

						t_con->state = TELNET_STATE_DATA;
					} else
						t_con->state = TELNET_STATE_DATA;
				} else if (t_con->last_escape == '\x00') {
					if (*buf_p == '[')
						t_con->last_escape = *buf_p;
					else
						t_con->state = TELNET_STATE_DATA;
				} else {
					LOG_ERROR("BUG: unexpected value in t_con->last_escape");
					t_con->state = TELNET_STATE_DATA;
				}

				break;
			default:
				LOG_ERROR("unknown telnet state");
				return ERROR_FAIL;
		}

		bytes_read--;
		buf_p++;
	}

	return ERROR_OK;
}

static int telnet_connection_closed(struct connection *connection)
{
	struct telnet_connection *t_con = connection->priv;
	int i;

	log_remove_callback(telnet_log_callback, connection);

	free(t_con->prompt);
	t_con->prompt = NULL;

	/* save telnet history */
	telnet_save_history(t_con);

	for (i = 0; i < TELNET_LINE_HISTORY_SIZE; i++) {
		free(t_con->history[i]);
		t_con->history[i] = NULL;
	}

	/* if this connection registered a debug-message receiver delete it */
	delete_debug_msg_receiver(connection->cmd_ctx, NULL);

	free(connection->priv);
	connection->priv = NULL;

	return ERROR_OK;
}

int telnet_init(char *banner)
{
	if (strcmp(telnet_port, "disabled") == 0) {
		LOG_INFO("telnet server disabled");
		return ERROR_OK;
	}

	struct telnet_service *telnet_service =
		malloc(sizeof(struct telnet_service));

	if (!telnet_service) {
		LOG_ERROR("Failed to allocate telnet service.");
		return ERROR_FAIL;
	}

	telnet_service->banner = banner;

	int ret = add_service("telnet", telnet_port, CONNECTION_LIMIT_UNLIMITED,
		telnet_new_connection, telnet_input, telnet_connection_closed,
		telnet_service, NULL);

	if (ret != ERROR_OK) {
		free(telnet_service);
		return ret;
	}

	return ERROR_OK;
}

/* daemon configuration command telnet_port */
COMMAND_HANDLER(handle_telnet_port_command)
{
	return CALL_COMMAND_HANDLER(server_pipe_command, &telnet_port);
}

COMMAND_HANDLER(handle_exit_command)
{
	return ERROR_COMMAND_CLOSE_CONNECTION;
}

static const struct command_registration telnet_command_handlers[] = {
	{
		.name = "exit",
		.handler = handle_exit_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "exit telnet session",
	},
	{
		.name = "telnet_port",
		.handler = handle_telnet_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Specify port on which to listen "
			"for incoming telnet connections.  "
			"Read help on 'gdb_port'.",
		.usage = "[port_num]",
	},
	COMMAND_REGISTRATION_DONE
};

int telnet_register_commands(struct command_context *cmd_ctx)
{
	telnet_port = strdup("4444");
	return register_commands(cmd_ctx, NULL, telnet_command_handlers);
}

void telnet_service_free(void)
{
	free(telnet_port);
}
