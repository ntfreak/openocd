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
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011                                     *
 *   michel.jaouen@stericsson.com : smp minimum support                    *
 *                                                                         *
 *   Copyright (C) 2013 by Franck Jullien                                  *
 *   elec4fun@gmail.com : target description file support                  *
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

#include <target/breakpoints.h>
#include <target/target_request.h>
#include <target/register.h>
#include "server.h"
#include <flash/nor/core.h>
#include "gdb_server.h"
#include <target/image.h>
#include <jtag/jtag.h>
#include "rtos/rtos.h"
#include "target/smp.h"

/**
 * @file
 * GDB server implementation.
 *
 * This implements the GDB Remote Serial Protocol, over TCP connections,
 * giving GDB access to the JTAG or other hardware debugging facilities
 * found in most modern embedded processors.
 */

/* private connection data for GDB */
struct gdb_connection {
	char buffer[GDB_BUFFER_SIZE];
	char *buf_p;
	int buf_cnt;
	int ctrl_c;
	enum target_state frontend_state;
	struct image *vflash_image;
	int closed;
	int busy;
	int noack_mode;
	/* set flag to true if you want the next stepi to return immediately.
	 * allowing GDB to pick up a fresh set of register values from the target
	 * without modifying the target state. */
	bool sync;
	/* We delay reporting memory write errors until next step/continue or memory
	 * write. This improves performance of gdb load significantly as the GDB packet
	 * can be replied immediately and a new GDB packet will be ready without delay
	 * (ca. 10% or so...). */
	bool mem_write_error;
	/* with extended-remote it seems we need to better emulate attach/detach.
	 * what this means is we reply with a W stop reply after a kill packet,
	 * normally we reply with a S reply via gdb_last_signal_packet.
	 * as a side note this behaviour only effects gdb > 6.8 */
	bool attached;
};

#if 0
#define _DEBUG_GDB_IO_
#endif

static struct gdb_connection *current_gdb_connection;

static int gdb_breakpoint_override;
static enum breakpoint_type gdb_breakpoint_override_type;

static int gdb_error(struct connection *connection, int retval);
static const char *gdb_port;
static const char *gdb_port_next;
static const char DIGITS[16] = "0123456789abcdef";

static void gdb_log_callback(void *priv, const char *file, unsigned line,
		const char *function, const char *string);

/* number of gdb connections, mainly to suppress gdb related debugging spam
 * in helper/log.c when no gdb connections are actually active */
int gdb_actual_connections;

/* set if we are sending a memory map to gdb
 * via qXfer:memory-map:read packet */
/* enabled by default*/
static int gdb_use_memory_map = 1;
/* enabled by default*/
static int gdb_flash_program = 1;

/* if set, data aborts cause an error to be reported in memory read packets
 * see the code in gdb_read_memory_packet() for further explanations.
 * Disabled by default.
 */
static int gdb_report_data_abort;

static int gdb_last_signal(struct target *target)
{
	switch (target->debug_reason) {
		case DBG_REASON_DBGRQ:
			return 0x2;		/* SIGINT */
		case DBG_REASON_BREAKPOINT:
		case DBG_REASON_WATCHPOINT:
		case DBG_REASON_WPTANDBKPT:
			return 0x05;	/* SIGTRAP */
		case DBG_REASON_SINGLESTEP:
			return 0x05;	/* SIGTRAP */
		case DBG_REASON_NOTHALTED:
			return 0x0;		/* no signal... shouldn't happen */
		default:
			LOG_USER("undefined debug reason %d - target needs reset",
					target->debug_reason);
			return 0x0;
	}
}

static int check_pending(struct connection *connection,
		int timeout_s, int *got_data)
{
	/* a non-blocking socket will block if there is 0 bytes available on the socket,
	 * but return with as many bytes as are available immediately
	 */
	struct timeval tv;
	fd_set read_fds;
	struct gdb_connection *gdb_con = connection->priv;
	int t;
	if (got_data == NULL)
		got_data = &t;
	*got_data = 0;

	if (gdb_con->buf_cnt > 0) {
		*got_data = 1;
		return ERROR_OK;
	}

	FD_ZERO(&read_fds);
	FD_SET(connection->fd, &read_fds);

	tv.tv_sec = timeout_s;
	tv.tv_usec = 0;
	if (socket_select(connection->fd + 1, &read_fds, NULL, NULL, &tv) == 0) {
		/* This can typically be because a "monitor" command took too long
		 * before printing any progress messages
		 */
		if (timeout_s > 0)
			return ERROR_GDB_TIMEOUT;
		else
			return ERROR_OK;
	}
	*got_data = FD_ISSET(connection->fd, &read_fds) != 0;
	return ERROR_OK;
}

static int gdb_get_char_inner(struct connection *connection, int *next_char)
{
	struct gdb_connection *gdb_con = connection->priv;
	int retval = ERROR_OK;

#ifdef _DEBUG_GDB_IO_
	char *debug_buffer;
#endif
	for (;; ) {
		if (connection->service->type != CONNECTION_TCP)
			gdb_con->buf_cnt = read(connection->fd, gdb_con->buffer, GDB_BUFFER_SIZE);
		else {
			retval = check_pending(connection, 1, NULL);
			if (retval != ERROR_OK)
				return retval;
			gdb_con->buf_cnt = read_socket(connection->fd,
					gdb_con->buffer,
					GDB_BUFFER_SIZE);
		}

		if (gdb_con->buf_cnt > 0)
			break;
		if (gdb_con->buf_cnt == 0) {
			gdb_con->closed = 1;
			return ERROR_SERVER_REMOTE_CLOSED;
		}

#ifdef _WIN32
		errno = WSAGetLastError();

		switch (errno) {
			case WSAEWOULDBLOCK:
				usleep(1000);
				break;
			case WSAECONNABORTED:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			case WSAECONNRESET:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				LOG_ERROR("read: %d", errno);
				exit(-1);
		}
#else
		switch (errno) {
			case EAGAIN:
				usleep(1000);
				break;
			case ECONNABORTED:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			case ECONNRESET:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				LOG_ERROR("read: %s", strerror(errno));
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
		}
#endif
	}

#ifdef _DEBUG_GDB_IO_
	debug_buffer = strndup(gdb_con->buffer, gdb_con->buf_cnt);
	LOG_DEBUG("received '%s'", debug_buffer);
	free(debug_buffer);
#endif

	gdb_con->buf_p = gdb_con->buffer;
	gdb_con->buf_cnt--;
	*next_char = *(gdb_con->buf_p++);
	if (gdb_con->buf_cnt > 0)
		connection->input_pending = 1;
	else
		connection->input_pending = 0;
#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

	return retval;
}

/**
 * The cool thing about this fn is that it allows buf_p and buf_cnt to be
 * held in registers in the inner loop.
 *
 * For small caches and embedded systems this is important!
 */
static inline int gdb_get_char_fast(struct connection *connection,
		int *next_char, char **buf_p, int *buf_cnt)
{
	int retval = ERROR_OK;

	if ((*buf_cnt)-- > 0) {
		*next_char = **buf_p;
		(*buf_p)++;
		if (*buf_cnt > 0)
			connection->input_pending = 1;
		else
			connection->input_pending = 0;

#ifdef _DEBUG_GDB_IO_
		LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

		return ERROR_OK;
	}

	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->buf_p = *buf_p;
	gdb_con->buf_cnt = *buf_cnt;
	retval = gdb_get_char_inner(connection, next_char);
	*buf_p = gdb_con->buf_p;
	*buf_cnt = gdb_con->buf_cnt;

	return retval;
}

static int gdb_get_char(struct connection *connection, int *next_char)
{
	struct gdb_connection *gdb_con = connection->priv;
	return gdb_get_char_fast(connection, next_char, &gdb_con->buf_p, &gdb_con->buf_cnt);
}

static int gdb_putback_char(struct connection *connection, int last_char)
{
	struct gdb_connection *gdb_con = connection->priv;

	if (gdb_con->buf_p > gdb_con->buffer) {
		*(--gdb_con->buf_p) = last_char;
		gdb_con->buf_cnt++;
	} else
		LOG_ERROR("BUG: couldn't put character back");

	return ERROR_OK;
}

/* The only way we can detect that the socket is closed is the first time
 * we write to it, we will fail. Subsequent write operations will
 * succeed. Shudder! */
static int gdb_write(struct connection *connection, void *data, int len)
{
	struct gdb_connection *gdb_con = connection->priv;
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	if (connection_write(connection, data, len) == len)
		return ERROR_OK;
	gdb_con->closed = 1;
	return ERROR_SERVER_REMOTE_CLOSED;
}

static int gdb_put_packet_inner(struct connection *connection,
		char *buffer, int len)
{
	int i;
	unsigned char my_checksum = 0;
#ifdef _DEBUG_GDB_IO_
	char *debug_buffer;
#endif
	int reply;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;

	for (i = 0; i < len; i++)
		my_checksum += buffer[i];

#ifdef _DEBUG_GDB_IO_
	/*
	 * At this point we should have nothing in the input queue from GDB,
	 * however sometimes '-' is sent even though we've already received
	 * an ACK (+) for everything we've sent off.
	 */
	int gotdata;
	for (;; ) {
		retval = check_pending(connection, 0, &gotdata);
		if (retval != ERROR_OK)
			return retval;
		if (!gotdata)
			break;
		retval = gdb_get_char(connection, &reply);
		if (retval != ERROR_OK)
			return retval;
		if (reply == '$') {
			/* fix a problem with some IAR tools */
			gdb_putback_char(connection, reply);
			LOG_DEBUG("Unexpected start of new packet");
			break;
		}

		LOG_WARNING("Discard unexpected char %c", reply);
	}
#endif

	while (1) {
#ifdef _DEBUG_GDB_IO_
		debug_buffer = strndup(buffer, len);
		LOG_DEBUG("sending packet '$%s#%2.2x'", debug_buffer, my_checksum);
		free(debug_buffer);
#endif

		char local_buffer[1024];
		local_buffer[0] = '$';
		if ((size_t)len + 4 <= sizeof(local_buffer)) {
			/* performance gain on smaller packets by only a single call to gdb_write() */
			memcpy(local_buffer + 1, buffer, len++);
			local_buffer[len++] = '#';
			local_buffer[len++] = DIGITS[(my_checksum >> 4) & 0xf];
			local_buffer[len++] = DIGITS[my_checksum & 0xf];
			retval = gdb_write(connection, local_buffer, len);
			if (retval != ERROR_OK)
				return retval;
		} else {
			/* larger packets are transmitted directly from caller supplied buffer
			 * by several calls to gdb_write() to avoid dynamic allocation */
			local_buffer[1] = '#';
			local_buffer[2] = DIGITS[(my_checksum >> 4) & 0xf];
			local_buffer[3] = DIGITS[my_checksum & 0xf];
			retval = gdb_write(connection, local_buffer, 1);
			if (retval != ERROR_OK)
				return retval;
			retval = gdb_write(connection, buffer, len);
			if (retval != ERROR_OK)
				return retval;
			retval = gdb_write(connection, local_buffer + 1, 3);
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->noack_mode)
			break;

		retval = gdb_get_char(connection, &reply);
		if (retval != ERROR_OK)
			return retval;

		if (reply == '+')
			break;
		else if (reply == '-') {
			/* Stop sending output packets for now */
			log_remove_callback(gdb_log_callback, connection);
			LOG_WARNING("negative reply, retrying");
		} else if (reply == 0x3) {
			gdb_con->ctrl_c = 1;
			retval = gdb_get_char(connection, &reply);
			if (retval != ERROR_OK)
				return retval;
			if (reply == '+')
				break;
			else if (reply == '-') {
				/* Stop sending output packets for now */
				log_remove_callback(gdb_log_callback, connection);
				LOG_WARNING("negative reply, retrying");
			} else if (reply == '$') {
				LOG_ERROR("GDB missing ack(1) - assumed good");
				gdb_putback_char(connection, reply);
				return ERROR_OK;
			} else {
				LOG_ERROR("unknown character(1) 0x%2.2x in reply, dropping connection", reply);
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			}
		} else if (reply == '$') {
			LOG_ERROR("GDB missing ack(2) - assumed good");
			gdb_putback_char(connection, reply);
			return ERROR_OK;
		} else {
			LOG_ERROR("unknown character(2) 0x%2.2x in reply, dropping connection",
				reply);
			gdb_con->closed = 1;
			return ERROR_SERVER_REMOTE_CLOSED;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

int gdb_put_packet(struct connection *connection, char *buffer, int len)
{
	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->busy = 1;
	int retval = gdb_put_packet_inner(connection, buffer, len);
	gdb_con->busy = 0;

	/* we sent some data, reset timer for keep alive messages */
	kept_alive();

	return retval;
}

static inline int fetch_packet(struct connection *connection,
		int *checksum_ok, int noack, int *len, char *buffer)
{
	unsigned char my_checksum = 0;
	char checksum[3];
	int character;
	int retval = ERROR_OK;

	struct gdb_connection *gdb_con = connection->priv;
	my_checksum = 0;
	int count = 0;
	count = 0;

	/* move this over into local variables to use registers and give the
	 * more freedom to optimize */
	char *buf_p = gdb_con->buf_p;
	int buf_cnt = gdb_con->buf_cnt;

	for (;; ) {
		/* The common case is that we have an entire packet with no escape chars.
		 * We need to leave at least 2 bytes in the buffer to have
		 * gdb_get_char() update various bits and bobs correctly.
		 */
		if ((buf_cnt > 2) && ((buf_cnt + count) < *len)) {
			/* The compiler will struggle a bit with constant propagation and
			 * aliasing, so we help it by showing that these values do not
			 * change inside the loop
			 */
			int i;
			char *buf = buf_p;
			int run = buf_cnt - 2;
			i = 0;
			int done = 0;
			while (i < run) {
				character = *buf++;
				i++;
				if (character == '#') {
					/* Danger! character can be '#' when esc is
					 * used so we need an explicit boolean for done here. */
					done = 1;
					break;
				}

				if (character == '}') {
					/* data transmitted in binary mode (X packet)
					 * uses 0x7d as escape character */
					my_checksum += character & 0xff;
					character = *buf++;
					i++;
					my_checksum += character & 0xff;
					buffer[count++] = (character ^ 0x20) & 0xff;
				} else {
					my_checksum += character & 0xff;
					buffer[count++] = character & 0xff;
				}
			}
			buf_p += i;
			buf_cnt -= i;
			if (done)
				break;
		}
		if (count > *len) {
			LOG_ERROR("packet buffer too small");
			retval = ERROR_GDB_BUFFER_TOO_SMALL;
			break;
		}

		retval = gdb_get_char_fast(connection, &character, &buf_p, &buf_cnt);
		if (retval != ERROR_OK)
			break;

		if (character == '#')
			break;

		if (character == '}') {
			/* data transmitted in binary mode (X packet)
			 * uses 0x7d as escape character */
			my_checksum += character & 0xff;

			retval = gdb_get_char_fast(connection, &character, &buf_p, &buf_cnt);
			if (retval != ERROR_OK)
				break;

			my_checksum += character & 0xff;
			buffer[count++] = (character ^ 0x20) & 0xff;
		} else {
			my_checksum += character & 0xff;
			buffer[count++] = character & 0xff;
		}
	}

	gdb_con->buf_p = buf_p;
	gdb_con->buf_cnt = buf_cnt;

	if (retval != ERROR_OK)
		return retval;

	*len = count;

	retval = gdb_get_char(connection, &character);
	if (retval != ERROR_OK)
		return retval;
	checksum[0] = character;
	retval = gdb_get_char(connection, &character);
	if (retval != ERROR_OK)
		return retval;
	checksum[1] = character;
	checksum[2] = 0;

	if (!noack)
		*checksum_ok = (my_checksum == strtoul(checksum, NULL, 16));

	return ERROR_OK;
}

static int gdb_get_packet_inner(struct connection *connection,
		char *buffer, int *len)
{
	int character;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;

	while (1) {
		do {
			retval = gdb_get_char(connection, &character);
			if (retval != ERROR_OK)
				return retval;

#ifdef _DEBUG_GDB_IO_
			LOG_DEBUG("character: '%c'", character);
#endif

			switch (character) {
				case '$':
					break;
				case '+':
					/* gdb sends a dummy ack '+' at every remote connect - see
					 * remote_start_remote (remote.c)
					 * in case anyone tries to debug why they receive this
					 * warning every time */
					LOG_WARNING("acknowledgment received, but no packet pending");
					break;
				case '-':
					LOG_WARNING("negative acknowledgment, but no packet pending");
					break;
				case 0x3:
					gdb_con->ctrl_c = 1;
					*len = 0;
					return ERROR_OK;
				default:
					LOG_WARNING("ignoring character 0x%x", character);
					break;
			}
		} while (character != '$');

		int checksum_ok = 0;
		/* explicit code expansion here to get faster inlined code in -O3 by not
		 * calculating checksum */
		if (gdb_con->noack_mode) {
			retval = fetch_packet(connection, &checksum_ok, 1, len, buffer);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = fetch_packet(connection, &checksum_ok, 0, len, buffer);
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->noack_mode) {
			/* checksum is not checked in noack mode */
			break;
		}
		if (checksum_ok) {
			retval = gdb_write(connection, "+", 1);
			if (retval != ERROR_OK)
				return retval;
			break;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

static int gdb_get_packet(struct connection *connection, char *buffer, int *len)
{
	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->busy = 1;
	int retval = gdb_get_packet_inner(connection, buffer, len);
	gdb_con->busy = 0;
	return retval;
}

static int gdb_output_con(struct connection *connection, const char *line)
{
	char *hex_buffer;
	int i, bin_size;

	bin_size = strlen(line);

	hex_buffer = malloc(bin_size*2 + 2);
	if (hex_buffer == NULL)
		return ERROR_GDB_BUFFER_TOO_SMALL;

	hex_buffer[0] = 'O';
	for (i = 0; i < bin_size; i++)
		snprintf(hex_buffer + 1 + i*2, 3, "%2.2x", line[i]);
	hex_buffer[bin_size*2 + 1] = 0;

	int retval = gdb_put_packet(connection, hex_buffer, bin_size*2 + 1);

	free(hex_buffer);
	return retval;
}

static int gdb_output(struct command_context *context, const char *line)
{
	/* this will be dumped to the log and also sent as an O packet if possible */
	LOG_USER_N("%s", line);
	return ERROR_OK;
}

static void gdb_frontend_halted(struct target *target, struct connection *connection)
{
	struct gdb_connection *gdb_connection = connection->priv;

	/* In the GDB protocol when we are stepping or continuing execution,
	 * we have a lingering reply. Upon receiving a halted event
	 * when we have that lingering packet, we reply to the original
	 * step or continue packet.
	 *
	 * Executing monitor commands can bring the target in and
	 * out of the running state so we'll see lots of TARGET_EVENT_XXX
	 * that are to be ignored.
	 */
	if (gdb_connection->frontend_state == TARGET_RUNNING) {
		char sig_reply[4];
		int signal_var;

		/* stop forwarding log packets! */
		log_remove_callback(gdb_log_callback, connection);

		if (gdb_connection->ctrl_c) {
			signal_var = 0x2;
			gdb_connection->ctrl_c = 0;
		} else
			signal_var = gdb_last_signal(target);

		snprintf(sig_reply, 4, "T%2.2x", signal_var);
		gdb_put_packet(connection, sig_reply, 3);
		gdb_connection->frontend_state = TARGET_HALTED;
		rtos_update_threads(target);
	}
}

static int gdb_target_callback_event_handler(struct target *target,
		enum target_event event, void *priv)
{
	int retval;
	struct connection *connection = priv;

	target_handle_event(target, event);
	switch (event) {
		case TARGET_EVENT_GDB_HALT:
			gdb_frontend_halted(target, connection);
			break;
		case TARGET_EVENT_HALTED:
			target_call_event_callbacks(target, TARGET_EVENT_GDB_END);
			break;
		case TARGET_EVENT_GDB_FLASH_ERASE_START:
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;
			break;
		default:
			break;
	}

	return ERROR_OK;
}

static int gdb_new_connection(struct connection *connection)
{
	struct gdb_connection *gdb_connection = malloc(sizeof(struct gdb_connection));
	struct gdb_service *gdb_service = connection->service->priv;
	int retval;
	int initial_ack;

	connection->priv = gdb_connection;

	/* initialize gdb connection information */
	gdb_connection->buf_p = gdb_connection->buffer;
	gdb_connection->buf_cnt = 0;
	gdb_connection->ctrl_c = 0;
	gdb_connection->frontend_state = TARGET_HALTED;
	gdb_connection->vflash_image = NULL;
	gdb_connection->closed = 0;
	gdb_connection->busy = 0;
	gdb_connection->noack_mode = 0;
	gdb_connection->sync = true;
	gdb_connection->mem_write_error = false;
	gdb_connection->attached = true;

	/* send ACK to GDB for debug request */
	gdb_write(connection, "+", 1);

	/* output goes through gdb connection */
	command_set_output_handler(connection->cmd_ctx, gdb_output, connection);

	/* we must remove all breakpoints registered to the target as a previous
	 * GDB session could leave dangling breakpoints if e.g. communication
	 * timed out.
	 */
	breakpoint_clear_target(gdb_service->target);
	watchpoint_clear_target(gdb_service->target);

	/* clean previous rtos session if supported*/
	if ((gdb_service->target->rtos) && (gdb_service->target->rtos->type->clean))
		gdb_service->target->rtos->type->clean(gdb_service->target);

	/* remove the initial ACK from the incoming buffer */
	retval = gdb_get_char(connection, &initial_ack);
	if (retval != ERROR_OK)
		return retval;

	/* FIX!!!??? would we actually ever receive a + here???
	 * Not observed.
	 */
	if (initial_ack != '+')
		gdb_putback_char(connection, initial_ack);
	target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_ATTACH);

	if (gdb_use_memory_map) {
		/* Connect must fail if the memory map can't be set up correctly.
		 *
		 * This will cause an auto_probe to be invoked, which is either
		 * a no-op or it will fail when the target isn't ready(e.g. not halted).
		 */
		int i;
		for (i = 0; i < flash_get_bank_count(); i++) {
			struct flash_bank *p;
			retval = get_flash_bank_by_num(i, &p);
			if (retval != ERROR_OK) {
				LOG_ERROR("Connect failed. Consider setting up a gdb-attach event for the target " \
						"to prepare target for GDB connect, or use 'gdb_memory_map disable'.");
				return retval;
			}
		}
	}

	gdb_actual_connections++;
	LOG_DEBUG("New GDB Connection: %d, Target %s, state: %s",
			gdb_actual_connections,
			target_name(gdb_service->target),
			target_state_name(gdb_service->target));

	/* DANGER! If we fail subsequently, we must remove this handler,
	 * otherwise we occasionally see crashes as the timer can invoke the
	 * callback fn.
	 *
	 * register callback to be informed about target events */
	target_register_event_callback(gdb_target_callback_event_handler, connection);

	return ERROR_OK;
}

static int gdb_connection_closed(struct connection *connection)
{
	struct gdb_service *gdb_service = connection->service->priv;
	struct gdb_connection *gdb_connection = connection->priv;

	/* we're done forwarding messages. Tear down callback before
	 * cleaning up connection.
	 */
	log_remove_callback(gdb_log_callback, connection);

	gdb_actual_connections--;
	LOG_DEBUG("GDB Close, Target: %s, state: %s, gdb_actual_connections=%d",
		target_name(gdb_service->target),
		target_state_name(gdb_service->target),
		gdb_actual_connections);

	/* see if an image built with vFlash commands is left */
	if (gdb_connection->vflash_image) {
		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;
	}

	/* if this connection registered a debug-message receiver delete it */
	delete_debug_msg_receiver(connection->cmd_ctx, gdb_service->target);

	if (connection->priv) {
		free(connection->priv);
		connection->priv = NULL;
	} else
		LOG_ERROR("BUG: connection->priv == NULL");

	target_unregister_event_callback(gdb_target_callback_event_handler, connection);

	target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_END);

	target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_DETACH);

	return ERROR_OK;
}

static void gdb_send_error(struct connection *connection, uint8_t the_error)
{
	char err[4];
	snprintf(err, 4, "E%2.2X", the_error);
	gdb_put_packet(connection, err, 3);
}

static int gdb_last_signal_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	struct gdb_connection *gdb_con = connection->priv;
	char sig_reply[4];
	int signal_var;

	if (!gdb_con->attached) {
		/* if we are here we have received a kill packet
		 * reply W stop reply otherwise gdb gets very unhappy */
		gdb_put_packet(connection, "W00", 3);
		return ERROR_OK;
	}

	signal_var = gdb_last_signal(target);

	snprintf(sig_reply, 4, "S%2.2x", signal_var);
	gdb_put_packet(connection, sig_reply, 3);

	return ERROR_OK;
}

static int gdb_reg_pos(struct target *target, int pos, int len)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return pos;
	else
		return len - 1 - pos;
}

/* Convert register to string of bytes. NB! The # of bits in the
 * register might be non-divisible by 8(a byte), in which
 * case an entire byte is shown.
 *
 * NB! the format on the wire is the target endianness
 *
 * The format of reg->value is little endian
 *
 */
static void gdb_str_to_target(struct target *target,
		char *tstr, struct reg *reg)
{
	int i;

	uint8_t *buf;
	int buf_len;
	buf = reg->value;
	buf_len = DIV_ROUND_UP(reg->size, 8);

	for (i = 0; i < buf_len; i++) {
		int j = gdb_reg_pos(target, i, buf_len);
		tstr[i*2]   = DIGITS[(buf[j]>>4) & 0xf];
		tstr[i*2 + 1] = DIGITS[buf[j]&0xf];
	}
}

static int hextoint(int c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	c = toupper(c);
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	LOG_ERROR("BUG: invalid register value %08x", c);
	return 0;
}

/* copy over in register buffer */
static void gdb_target_to_reg(struct target *target,
		char *tstr, int str_len, uint8_t *bin)
{
	if (str_len % 2) {
		LOG_ERROR("BUG: gdb value with uneven number of characters encountered");
		exit(-1);
	}

	int i;
	for (i = 0; i < str_len; i += 2) {
		uint8_t t = hextoint(tstr[i]) << 4;
		t |= hextoint(tstr[i + 1]);

		int j = gdb_reg_pos(target, i/2, str_len/2);
		bin[j] = t;
	}
}

static int gdb_get_registers_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	struct reg **reg_list;
	int reg_list_size;
	int retval;
	int reg_packet_size = 0;
	char *reg_packet;
	char *reg_packet_p;
	int i;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if ((target->rtos != NULL) && (ERROR_OK == rtos_get_gdb_reg_list(connection)))
		return ERROR_OK;

	retval = target_get_gdb_reg_list(target, &reg_list, &reg_list_size);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	for (i = 0; i < reg_list_size; i++)
		reg_packet_size += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;

	assert(reg_packet_size > 0);

	reg_packet = malloc(reg_packet_size);
	reg_packet_p = reg_packet;

	for (i = 0; i < reg_list_size; i++) {
		if (!reg_list[i]->valid)
			reg_list[i]->type->get(reg_list[i]);
		gdb_str_to_target(target, reg_packet_p, reg_list[i]);
		reg_packet_p += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;
	}

#ifdef _DEBUG_GDB_IO_
	{
		char *reg_packet_p_debug;
		reg_packet_p_debug = strndup(reg_packet, reg_packet_size);
		LOG_DEBUG("reg_packet: %s", reg_packet_p_debug);
		free(reg_packet_p_debug);
	}
#endif

	gdb_put_packet(connection, reg_packet, reg_packet_size);
	free(reg_packet);

	free(reg_list);

	return ERROR_OK;
}

static int gdb_set_registers_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int i;
	struct reg **reg_list;
	int reg_list_size;
	int retval;
	char *packet_p;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	/* skip command character */
	packet++;
	packet_size--;

	if (packet_size % 2) {
		LOG_WARNING("GDB set_registers packet with uneven characters received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	retval = target_get_gdb_reg_list(target, &reg_list, &reg_list_size);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	packet_p = packet;
	for (i = 0; i < reg_list_size; i++) {
		uint8_t *bin_buf;
		int chars = (DIV_ROUND_UP(reg_list[i]->size, 8) * 2);

		if (packet_p + chars > packet + packet_size)
			LOG_ERROR("BUG: register packet is too small for registers");

		bin_buf = malloc(DIV_ROUND_UP(reg_list[i]->size, 8));
		gdb_target_to_reg(target, packet_p, chars, bin_buf);

		reg_list[i]->type->set(reg_list[i], bin_buf);

		/* advance packet pointer */
		packet_p += chars;

		free(bin_buf);
	}

	/* free struct reg *reg_list[] array allocated by get_gdb_reg_list */
	free(reg_list);

	gdb_put_packet(connection, "OK", 2);

	return ERROR_OK;
}

static int gdb_get_register_packet(struct connection *connection,
	char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *reg_packet;
	int reg_num = strtoul(packet + 1, NULL, 16);
	struct reg **reg_list;
	int reg_list_size;
	int retval;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	retval = target_get_gdb_reg_list(target, &reg_list, &reg_list_size);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	if (reg_list_size <= reg_num) {
		LOG_ERROR("gdb requested a non-existing register");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (!reg_list[reg_num]->valid)
		reg_list[reg_num]->type->get(reg_list[reg_num]);

	reg_packet = malloc(DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2);

	gdb_str_to_target(target, reg_packet, reg_list[reg_num]);

	gdb_put_packet(connection, reg_packet, DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2);

	free(reg_list);
	free(reg_packet);

	return ERROR_OK;
}

static int gdb_set_register_packet(struct connection *connection,
	char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	uint8_t *bin_buf;
	int reg_num = strtoul(packet + 1, &separator, 16);
	struct reg **reg_list;
	int reg_list_size;
	int retval;

	LOG_DEBUG("-");

	retval = target_get_gdb_reg_list(target, &reg_list, &reg_list_size);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	if (reg_list_size <= reg_num) {
		LOG_ERROR("gdb requested a non-existing register");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (*separator != '=') {
		LOG_ERROR("GDB 'set register packet', but no '=' following the register number");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	/* convert from GDB-string (target-endian) to hex-string (big-endian) */
	bin_buf = malloc(DIV_ROUND_UP(reg_list[reg_num]->size, 8));
	int chars = (DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2);

	if ((unsigned int)chars != strlen(separator + 1)) {
		LOG_ERROR("gdb sent a packet with wrong register size");
		free(bin_buf);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	gdb_target_to_reg(target, separator + 1, chars, bin_buf);

	reg_list[reg_num]->type->set(reg_list[reg_num], bin_buf);

	gdb_put_packet(connection, "OK", 2);

	free(bin_buf);
	free(reg_list);

	return ERROR_OK;
}

/* No attempt is made to translate the "retval" to
 * GDB speak. This has to be done at the calling
 * site as no mapping really exists.
 */
static int gdb_error(struct connection *connection, int retval)
{
	LOG_DEBUG("Reporting %i to GDB as generic error", retval);
	gdb_send_error(connection, EFAULT);
	return ERROR_OK;
}

/* We don't have to worry about the default 2 second timeout for GDB packets,
 * because GDB breaks up large memory reads into smaller reads.
 *
 * 8191 bytes by the looks of it. Why 8191 bytes instead of 8192?????
 */
static int gdb_read_memory_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	uint32_t addr = 0;
	uint32_t len = 0;

	uint8_t *buffer;
	char *hex_buffer;

	int retval = ERROR_OK;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete read memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, NULL, 16);

	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%8.8" PRIx32 ", len: 0x%8.8" PRIx32 "", addr, len);

	retval = target_read_buffer(target, addr, len, buffer);

	if ((retval != ERROR_OK) && !gdb_report_data_abort) {
		/* TODO : Here we have to lie and send back all zero's lest stack traces won't work.
		 * At some point this might be fixed in GDB, in which case this code can be removed.
		 *
		 * OpenOCD developers are acutely aware of this problem, but there is nothing
		 * gained by involving the user in this problem that hopefully will get resolved
		 * eventually
		 *
		 * http://sourceware.org/cgi-bin/gnatsweb.pl? \
		 * cmd = view%20audit-trail&database = gdb&pr = 2395
		 *
		 * For now, the default is to fix up things to make current GDB versions work.
		 * This can be overwritten using the gdb_report_data_abort <'enable'|'disable'> command.
		 */
		memset(buffer, 0, len);
		retval = ERROR_OK;
	}

	if (retval == ERROR_OK) {
		hex_buffer = malloc(len * 2 + 1);

		uint32_t i;
		for (i = 0; i < len; i++) {
			uint8_t t = buffer[i];
			hex_buffer[2 * i] = DIGITS[(t >> 4) & 0xf];
			hex_buffer[2 * i + 1] = DIGITS[t & 0xf];
		}

		gdb_put_packet(connection, hex_buffer, len * 2);

		free(hex_buffer);
	} else
		retval = gdb_error(connection, retval);

	free(buffer);

	return retval;
}

static int gdb_write_memory_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	uint32_t addr = 0;
	uint32_t len = 0;

	uint8_t *buffer;

	uint32_t i;
	int retval;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, &separator, 16);

	if (*(separator++) != ':') {
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%8.8" PRIx32 ", len: 0x%8.8" PRIx32 "", addr, len);

	for (i = 0; i < len; i++) {
		uint32_t tmp;
		sscanf(separator + 2*i, "%2" SCNx32, &tmp);
		buffer[i] = tmp;
	}

	retval = target_write_buffer(target, addr, len, buffer);

	if (retval == ERROR_OK)
		gdb_put_packet(connection, "OK", 2);
	else
		retval = gdb_error(connection, retval);

	free(buffer);

	return retval;
}

static int gdb_write_memory_binary_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	uint32_t addr = 0;
	uint32_t len = 0;

	int retval = ERROR_OK;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, &separator, 16);

	if (*(separator++) != ':') {
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	struct gdb_connection *gdb_connection = connection->priv;

	if (gdb_connection->mem_write_error) {
		retval = ERROR_FAIL;
		/* now that we have reported the memory write error, we can clear the condition */
		gdb_connection->mem_write_error = false;
	}

	/* By replying the packet *immediately* GDB will send us a new packet
	 * while we write the last one to the target.
	 */
	if (retval == ERROR_OK)
		gdb_put_packet(connection, "OK", 2);
	else {
		retval = gdb_error(connection, retval);
		if (retval != ERROR_OK)
			return retval;
	}

	if (len) {
		LOG_DEBUG("addr: 0x%8.8" PRIx32 ", len: 0x%8.8" PRIx32 "", addr, len);

		retval = target_write_buffer(target, addr, len, (uint8_t *)separator);
		if (retval != ERROR_OK)
			gdb_connection->mem_write_error = true;
	}

	return ERROR_OK;
}

static int gdb_step_continue_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int current = 0;
	uint32_t address = 0x0;
	int retval = ERROR_OK;

	LOG_DEBUG("-");

	if (packet_size > 1) {
		packet[packet_size] = 0;
		address = strtoul(packet + 1, NULL, 16);
	} else
		current = 1;

	if (packet[0] == 'c') {
		LOG_DEBUG("continue");
		/* resume at current address, don't handle breakpoints, not debugging */
		retval = target_resume(target, current, address, 0, 0);
	} else if (packet[0] == 's') {
		LOG_DEBUG("step");
		/* step at current or address, don't handle breakpoints */
		retval = target_step(target, current, address, 0);
	}
	return retval;
}

static int gdb_breakpoint_watchpoint_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int type;
	enum breakpoint_type bp_type = BKPT_SOFT /* dummy init to avoid warning */;
	enum watchpoint_rw wp_type = WPT_READ /* dummy init to avoid warning */;
	uint32_t address;
	uint32_t size;
	char *separator;
	int retval;

	LOG_DEBUG("-");

	type = strtoul(packet + 1, &separator, 16);

	if (type == 0)	/* memory breakpoint */
		bp_type = BKPT_SOFT;
	else if (type == 1)	/* hardware breakpoint */
		bp_type = BKPT_HARD;
	else if (type == 2)	/* write watchpoint */
		wp_type = WPT_WRITE;
	else if (type == 3)	/* read watchpoint */
		wp_type = WPT_READ;
	else if (type == 4)	/* access watchpoint */
		wp_type = WPT_ACCESS;
	else {
		LOG_ERROR("invalid gdb watch/breakpoint type(%d), dropping connection", type);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (gdb_breakpoint_override && ((bp_type == BKPT_SOFT) || (bp_type == BKPT_HARD)))
		bp_type = gdb_breakpoint_override_type;

	if (*separator != ',') {
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	address = strtoul(separator + 1, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	size = strtoul(separator + 1, &separator, 16);

	switch (type) {
		case 0:
		case 1:
			if (packet[0] == 'Z') {
				retval = breakpoint_add(target, address, size, bp_type);
				if (retval != ERROR_OK) {
					retval = gdb_error(connection, retval);
					if (retval != ERROR_OK)
						return retval;
				} else
					gdb_put_packet(connection, "OK", 2);
			} else {
				breakpoint_remove(target, address);
				gdb_put_packet(connection, "OK", 2);
			}
			break;
		case 2:
		case 3:
		case 4:
		{
			if (packet[0] == 'Z') {
				retval = watchpoint_add(target, address, size, wp_type, 0, 0xffffffffu);
				if (retval != ERROR_OK) {
					retval = gdb_error(connection, retval);
					if (retval != ERROR_OK)
						return retval;
				} else
					gdb_put_packet(connection, "OK", 2);
			} else {
				watchpoint_remove(target, address);
				gdb_put_packet(connection, "OK", 2);
			}
			break;
		}
		default:
			break;
	}

	return ERROR_OK;
}

/* print out a string and allocate more space as needed,
 * mainly used for XML at this point
 */
static void xml_printf(int *retval, char **xml, int *pos, int *size,
		const char *fmt, ...)
{
	if (*retval != ERROR_OK)
		return;
	int first = 1;

	for (;; ) {
		if ((*xml == NULL) || (!first)) {
			/* start by 0 to exercise all the code paths.
			 * Need minimum 2 bytes to fit 1 char and 0 terminator. */

			*size = *size * 2 + 2;
			char *t = *xml;
			*xml = realloc(*xml, *size);
			if (*xml == NULL) {
				if (t)
					free(t);
				*retval = ERROR_SERVER_REMOTE_CLOSED;
				return;
			}
		}

		va_list ap;
		int ret;
		va_start(ap, fmt);
		ret = vsnprintf(*xml + *pos, *size - *pos, fmt, ap);
		va_end(ap);
		if ((ret > 0) && ((ret + 1) < *size - *pos)) {
			*pos += ret;
			return;
		}
		/* there was just enough or not enough space, allocate more. */
		first = 0;
	}
}

static int decode_xfer_read(char *buf, char **annex, int *ofs, unsigned int *len)
{
	char *separator;

	/* Extract and NUL-terminate the annex. */
	*annex = buf;
	while (*buf && *buf != ':')
		buf++;
	if (*buf == '\0')
		return -1;
	*buf++ = 0;

	/* After the read marker and annex, qXfer looks like a
	 * traditional 'm' packet. */

	*ofs = strtoul(buf, &separator, 16);

	if (*separator != ',')
		return -1;

	*len = strtoul(separator + 1, NULL, 16);

	return 0;
}

static int compare_bank(const void *a, const void *b)
{
	struct flash_bank *b1, *b2;
	b1 = *((struct flash_bank **)a);
	b2 = *((struct flash_bank **)b);

	if (b1->base == b2->base)
		return 0;
	else if (b1->base > b2->base)
		return 1;
	else
		return -1;
}

static int gdb_memory_map(struct connection *connection,
		char *packet, int packet_size)
{
	/* We get away with only specifying flash here. Regions that are not
	 * specified are treated as if we provided no memory map(if not we
	 * could detect the holes and mark them as RAM).
	 * Normally we only execute this code once, but no big deal if we
	 * have to regenerate it a couple of times.
	 */

	struct target *target = get_target_from_connection(connection);
	struct flash_bank *p;
	char *xml = NULL;
	int size = 0;
	int pos = 0;
	int retval = ERROR_OK;
	struct flash_bank **banks;
	int offset;
	int length;
	char *separator;
	uint32_t ram_start = 0;
	int i;
	int target_flash_banks = 0;

	/* skip command character */
	packet += 23;

	offset = strtoul(packet, &separator, 16);
	length = strtoul(separator + 1, &separator, 16);

	xml_printf(&retval, &xml, &pos, &size, "<memory-map>\n");

	/* Sort banks in ascending order.  We need to report non-flash
	 * memory as ram (or rather read/write) by default for GDB, since
	 * it has no concept of non-cacheable read/write memory (i/o etc).
	 *
	 * FIXME Most non-flash addresses are *NOT* RAM!  Don't lie.
	 * Current versions of GDB assume unlisted addresses are RAM...
	 */
	banks = malloc(sizeof(struct flash_bank *)*flash_get_bank_count());

	for (i = 0; i < flash_get_bank_count(); i++) {
		retval = get_flash_bank_by_num(i, &p);
		if (retval != ERROR_OK) {
			free(banks);
			gdb_error(connection, retval);
			return retval;
		}
		if (p->target == target)
			banks[target_flash_banks++] = p;
	}

	qsort(banks, target_flash_banks, sizeof(struct flash_bank *),
		compare_bank);

	for (i = 0; i < target_flash_banks; i++) {
		int j;
		unsigned sector_size = 0;
		uint32_t start;

		p = banks[i];
		start = p->base;

		if (ram_start < p->base)
			xml_printf(&retval, &xml, &pos, &size,
				"<memory type=\"ram\" start=\"0x%x\" "
				"length=\"0x%x\"/>\n",
				ram_start, p->base - ram_start);

		/* Report adjacent groups of same-size sectors.  So for
		 * example top boot CFI flash will list an initial region
		 * with several large sectors (maybe 128KB) and several
		 * smaller ones at the end (maybe 32KB).  STR7 will have
		 * regions with 8KB, 32KB, and 64KB sectors; etc.
		 */
		for (j = 0; j < p->num_sectors; j++) {
			unsigned group_len;

			/* Maybe start a new group of sectors. */
			if (sector_size == 0) {
				start = p->base + p->sectors[j].offset;
				xml_printf(&retval, &xml, &pos, &size,
					"<memory type=\"flash\" "
					"start=\"0x%x\" ",
					start);
				sector_size = p->sectors[j].size;
			}

			/* Does this finish a group of sectors?
			 * If not, continue an already-started group.
			 */
			if (j == p->num_sectors - 1)
				group_len = (p->base + p->size) - start;
			else if (p->sectors[j + 1].size != sector_size)
				group_len = p->base + p->sectors[j + 1].offset
					- start;
			else
				continue;

			xml_printf(&retval, &xml, &pos, &size,
				"length=\"0x%x\">\n"
				"<property name=\"blocksize\">"
				"0x%x</property>\n"
				"</memory>\n",
				group_len,
				sector_size);
			sector_size = 0;
		}

		ram_start = p->base + p->size;
	}

	if (ram_start != 0)
		xml_printf(&retval, &xml, &pos, &size,
			"<memory type=\"ram\" start=\"0x%x\" "
			"length=\"0x%x\"/>\n",
			ram_start, 0-ram_start);
	/* ELSE a flash chip could be at the very end of the 32 bit address
	 * space, in which case ram_start will be precisely 0
	 */

	free(banks);
	banks = NULL;

	xml_printf(&retval, &xml, &pos, &size, "</memory-map>\n");

	if (retval != ERROR_OK) {
		gdb_error(connection, retval);
		return retval;
	}

	if (offset + length > pos)
		length = pos - offset;

	char *t = malloc(length + 1);
	t[0] = 'l';
	memcpy(t + 1, xml + offset, length);
	gdb_put_packet(connection, t, length + 1);

	free(t);
	free(xml);
	return ERROR_OK;
}

static int prepare_file_chunks(struct target *target, const char *filename,
			       void *buffer, int *len)
{
	struct fileio fileio;
	size_t read_bytes;
	int filesize;
	char *filebuffer;

	filebuffer = (char *)buffer;

	int retval = fileio_open(&fileio, filename, FILEIO_READ, FILEIO_BINARY);
	if (retval != ERROR_OK) {
		target->remaining_xfer = -1;
		return retval;
	}

	retval = fileio_size(&fileio, &filesize);
	if (retval != ERROR_OK)
		goto error;

	/* If there is no pending transfert, set the number of xfer to come. */
	if (target->remaining_xfer == -1)
		target->remaining_xfer = DIV_ROUND_UP(filesize, QXFER_CHUNK_SIZE);

	memset(filebuffer, 0, QXFER_CHUNK_SIZE + 1);

	/* This is not the last chunk of data, so prepare a 'm' packet. */
	if (target->remaining_xfer > 1) {

		filebuffer[0] = 'm';

		retval = fileio_seek(&fileio, (DIV_ROUND_UP(filesize, QXFER_CHUNK_SIZE)
					- target->remaining_xfer) * QXFER_CHUNK_SIZE);
		if (retval != ERROR_OK)
			goto error;

		retval = fileio_read(&fileio, QXFER_CHUNK_SIZE, &filebuffer[1], &read_bytes);
		if (retval != ERROR_OK)
			goto error;

		target->remaining_xfer--;

		*len = QXFER_CHUNK_SIZE + 1;

	/* This is the last chunk of data, so prepare a 'l' packet. */
	} else {

		filebuffer[0] = 'l';

		retval = fileio_seek(&fileio, filesize - (filesize % QXFER_CHUNK_SIZE));
		if (retval != ERROR_OK)
			goto error;

		retval = fileio_read(&fileio, filesize % QXFER_CHUNK_SIZE,
					&filebuffer[1], &read_bytes);
		if (retval != ERROR_OK)
			goto error;

		target->remaining_xfer = -1;

		*len = (filesize % QXFER_CHUNK_SIZE) + 1;
	}

	fileio_close(&fileio);
	return ERROR_OK;

error:
	target->remaining_xfer = -1;
	fileio_close(&fileio);
	return retval;
}

static int gdb_query_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct command_context *cmd_ctx = connection->cmd_ctx;
	struct gdb_connection *gdb_connection = connection->priv;
	struct target *target = get_target_from_connection(connection);

	if (strncmp(packet, "qRcmd,", 6) == 0) {
		if (packet_size > 6) {
			char *cmd;
			int i;
			cmd = malloc((packet_size - 6)/2 + 1);
			for (i = 0; i < (packet_size - 6)/2; i++) {
				uint32_t tmp;
				sscanf(packet + 6 + 2*i, "%2" SCNx32, &tmp);
				cmd[i] = tmp;
			}
			cmd[(packet_size - 6)/2] = 0x0;

			/* We want to print all debug output to GDB connection */
			log_add_callback(gdb_log_callback, connection);
			target_call_timer_callbacks_now();
			/* some commands need to know the GDB connection, make note of current
			 * GDB connection. */
			current_gdb_connection = gdb_connection;
			command_run_line(cmd_ctx, cmd);
			current_gdb_connection = NULL;
			target_call_timer_callbacks_now();
			log_remove_callback(gdb_log_callback, connection);
			free(cmd);
		}
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	} else if (strncmp(packet, "qCRC:", 5) == 0) {
		if (packet_size > 5) {
			int retval;
			char gdb_reply[10];
			char *separator;
			uint32_t checksum;
			uint32_t addr = 0;
			uint32_t len = 0;

			/* skip command character */
			packet += 5;

			addr = strtoul(packet, &separator, 16);

			if (*separator != ',') {
				LOG_ERROR("incomplete read memory packet received, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			}

			len = strtoul(separator + 1, NULL, 16);

			retval = target_checksum_memory(target, addr, len, &checksum);

			if (retval == ERROR_OK) {
				snprintf(gdb_reply, 10, "C%8.8" PRIx32 "", checksum);
				gdb_put_packet(connection, gdb_reply, 9);
			} else {
				retval = gdb_error(connection, retval);
				if (retval != ERROR_OK)
					return retval;
			}

			return ERROR_OK;
		}
	} else if (strncmp(packet, "qSupported", 10) == 0) {
		/* we currently support packet size and qXfer:memory-map:read (if enabled)
		 * disable qXfer:features:read for the moment */
		int retval = ERROR_OK;
		char *buffer = NULL;
		int pos = 0;
		int size = 0;

		xml_printf(&retval,
			&buffer,
			&pos,
			&size,
			"PacketSize=%x;qXfer:memory-map:read%c;qXfer:features:read%c;QStartNoAckMode+",
			(GDB_BUFFER_SIZE - 1),
			((gdb_use_memory_map == 1) && (flash_get_bank_count() > 0)) ? '+' : '-',
			(target->gdb_tdesc_path) ? '+' : '-');

		if (retval != ERROR_OK) {
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		gdb_put_packet(connection, buffer, strlen(buffer));
		free(buffer);

		return ERROR_OK;
	} else if ((strncmp(packet, "qXfer:memory-map:read::", 23) == 0)
		   && (flash_get_bank_count() > 0))
		return gdb_memory_map(connection, packet, packet_size);
	else if (strncmp(packet, "qXfer:features:read:", 20) == 0) {
		int retval = ERROR_OK;
		char filebuffer[QXFER_CHUNK_SIZE + 1];
		int offset;
		unsigned int length;
		char *annex;
		int len = 0;
		char *tdesc_filename;

		/* skip command character */
		packet += 20;

		if (decode_xfer_read(packet, &annex, &offset, &length) < 0)
			goto error;

		if (strcmp(annex, "target.xml") != 0)
			goto error;

		if (!strcmp(target->gdb_tdesc_path, "auto")) {

			tdesc_filename = malloc(strlen(target->cmd_name) + 5);
			if (tdesc_filename == NULL)
				goto error;
			snprintf(tdesc_filename, strlen(target->cmd_name) + 5,
				 "%s.xml", target->cmd_name);

			if (fileio_exist(tdesc_filename) != FILE_EXIST) {
				retval = target_generate_tdesc_file(target, tdesc_filename);
				if (retval != ERROR_OK) {
					free(tdesc_filename);
					goto error;
				}
				target->gdb_tdesc_path = realloc(target->gdb_tdesc_path, strlen(tdesc_filename));
				strcpy(target->gdb_tdesc_path, tdesc_filename);
			}
		} else {
			if (target->gdb_tdesc_path && strcmp(target->gdb_tdesc_path, ""))
				tdesc_filename = strdup(target->gdb_tdesc_path);
			else
				goto error;
		}

		retval = prepare_file_chunks(target, tdesc_filename, filebuffer, &len);

		free(tdesc_filename);

		if (retval != ERROR_OK)
			goto error;

		gdb_put_packet(connection, filebuffer, len);
		return ERROR_OK;

error:
		gdb_send_error(connection, 01);
		return ERROR_OK;

	} else if (strncmp(packet, "QStartNoAckMode", 15) == 0) {
		gdb_connection->noack_mode = 1;
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

static int gdb_v_packet(struct connection *connection,
		char *packet, int packet_size)
{
	struct gdb_connection *gdb_connection = connection->priv;
	struct gdb_service *gdb_service = connection->service->priv;
	int result;

	/* if flash programming disabled - send a empty reply */

	if (gdb_flash_program == 0) {
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	}

	if (strncmp(packet, "vFlashErase:", 12) == 0) {
		unsigned long addr;
		unsigned long length;

		char *parse = packet + 12;
		if (*parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		addr = strtoul(parse, &parse, 16);

		if (*(parse++) != ',' || *parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		length = strtoul(parse, &parse, 16);

		if (*parse != '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		/* assume all sectors need erasing - stops any problems
		 * when flash_write is called multiple times */
		flash_set_dirty();

		/* perform any target specific operations before the erase */
		target_call_event_callbacks(gdb_service->target,
			TARGET_EVENT_GDB_FLASH_ERASE_START);

		/* vFlashErase:addr,length messages require region start and
		 * end to be "block" aligned ... if padding is ever needed,
		 * GDB will have become dangerously confused.
		 */
		result = flash_erase_address_range(gdb_service->target,
				false, addr, length);

		/* perform any target specific operations after the erase */
		target_call_event_callbacks(gdb_service->target,
			TARGET_EVENT_GDB_FLASH_ERASE_END);

		/* perform erase */
		if (result != ERROR_OK) {
			/* GDB doesn't evaluate the actual error number returned,
			 * treat a failed erase as an I/O error
			 */
			gdb_send_error(connection, EIO);
			LOG_ERROR("flash_erase returned %i", result);
		} else
			gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	if (strncmp(packet, "vFlashWrite:", 12) == 0) {
		int retval;
		unsigned long addr;
		unsigned long length;
		char *parse = packet + 12;

		if (*parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}
		addr = strtoul(parse, &parse, 16);
		if (*(parse++) != ':') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}
		length = packet_size - (parse - packet);

		/* create a new image if there isn't already one */
		if (gdb_connection->vflash_image == NULL) {
			gdb_connection->vflash_image = malloc(sizeof(struct image));
			image_open(gdb_connection->vflash_image, "", "build");
		}

		/* create new section with content from packet buffer */
		retval = image_add_section(gdb_connection->vflash_image,
				addr, length, 0x0, (uint8_t *)parse);
		if (retval != ERROR_OK)
			return retval;

		gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	if (strncmp(packet, "vFlashDone", 10) == 0) {
		uint32_t written;

		/* process the flashing buffer. No need to erase as GDB
		 * always issues a vFlashErase first. */
		target_call_event_callbacks(gdb_service->target,
				TARGET_EVENT_GDB_FLASH_WRITE_START);
		result = flash_write(gdb_service->target, gdb_connection->vflash_image, &written, 0);
		target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_FLASH_WRITE_END);
		if (result != ERROR_OK) {
			if (result == ERROR_FLASH_DST_OUT_OF_BANK)
				gdb_put_packet(connection, "E.memtype", 9);
			else
				gdb_send_error(connection, EIO);
		} else {
			LOG_DEBUG("wrote %u bytes from vFlash image to flash", (unsigned)written);
			gdb_put_packet(connection, "OK", 2);
		}

		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;

		return ERROR_OK;
	}

	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

static int gdb_detach(struct connection *connection)
{
	struct gdb_service *gdb_service = connection->service->priv;

	target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_DETACH);

	return gdb_put_packet(connection, "OK", 2);
}

static void gdb_log_callback(void *priv, const char *file, unsigned line,
		const char *function, const char *string)
{
	struct connection *connection = priv;
	struct gdb_connection *gdb_con = connection->priv;

	if (gdb_con->busy) {
		/* do not reply this using the O packet */
		return;
	}

	gdb_output_con(connection, string);
}

static void gdb_sig_halted(struct connection *connection)
{
	char sig_reply[4];
	snprintf(sig_reply, 4, "T%2.2x", 2);
	gdb_put_packet(connection, sig_reply, 3);
}

static int gdb_input_inner(struct connection *connection)
{
	/* Do not allocate this on the stack */
	static char gdb_packet_buffer[GDB_BUFFER_SIZE];

	struct gdb_service *gdb_service = connection->service->priv;
	struct target *target = gdb_service->target;
	char *packet = gdb_packet_buffer;
	int packet_size;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;
	static int extended_protocol;

	/* drain input buffer. If one of the packets fail, then an error
	 * packet is replied, if applicable.
	 *
	 * This loop will terminate and the error code is returned.
	 *
	 * The calling fn will check if this error is something that
	 * can be recovered from, or if the connection must be closed.
	 *
	 * If the error is recoverable, this fn is called again to
	 * drain the rest of the buffer.
	 */
	do {
		packet_size = GDB_BUFFER_SIZE-1;
		retval = gdb_get_packet(connection, packet, &packet_size);
		if (retval != ERROR_OK)
			return retval;

		/* terminate with zero */
		packet[packet_size] = 0;

		if (LOG_LEVEL_IS(LOG_LVL_DEBUG)) {
			if (packet[0] == 'X') {
				/* binary packets spew junk into the debug log stream */
				char buf[50];
				int x;
				for (x = 0; (x < 49) && (packet[x] != ':'); x++)
					buf[x] = packet[x];
				buf[x] = 0;
				LOG_DEBUG("received packet: '%s:<binary-data>'", buf);
			} else
				LOG_DEBUG("received packet: '%s'", packet);
		}

		if (packet_size > 0) {
			retval = ERROR_OK;
			switch (packet[0]) {
				case 'T':	/* Is thread alive? */
					gdb_thread_packet(connection, packet, packet_size);
					break;
				case 'H':	/* Set current thread ( 'c' for step and continue,
							 * 'g' for all other operations ) */
					gdb_thread_packet(connection, packet, packet_size);
					break;
				case 'q':
				case 'Q':
					retval = gdb_thread_packet(connection, packet, packet_size);
					if (retval == GDB_THREAD_PACKET_NOT_CONSUMED)
						retval = gdb_query_packet(connection, packet, packet_size);
					break;
				case 'g':
					retval = gdb_get_registers_packet(connection, packet, packet_size);
					break;
				case 'G':
					retval = gdb_set_registers_packet(connection, packet, packet_size);
					break;
				case 'p':
					retval = gdb_get_register_packet(connection, packet, packet_size);
					break;
				case 'P':
					retval = gdb_set_register_packet(connection, packet, packet_size);
					break;
				case 'm':
					retval = gdb_read_memory_packet(connection, packet, packet_size);
					break;
				case 'M':
					retval = gdb_write_memory_packet(connection, packet, packet_size);
					break;
				case 'z':
				case 'Z':
					retval = gdb_breakpoint_watchpoint_packet(connection, packet, packet_size);
					break;
				case '?':
					gdb_last_signal_packet(connection, packet, packet_size);
					break;
				case 'c':
				case 's':
				{
					gdb_thread_packet(connection, packet, packet_size);
					log_add_callback(gdb_log_callback, connection);

					if (gdb_con->mem_write_error) {
						LOG_ERROR("Memory write failure!");

						/* now that we have reported the memory write error,
						 * we can clear the condition */
						gdb_con->mem_write_error = false;
					}

					bool nostep = false;
					bool already_running = false;
					if (target->state == TARGET_RUNNING) {
						LOG_WARNING("WARNING! The target is already running. "
								"All changes GDB did to registers will be discarded! "
								"Waiting for target to halt.");
						already_running = true;
					} else if (target->state != TARGET_HALTED) {
						LOG_WARNING("The target is not in the halted nor running stated, " \
								"stepi/continue ignored.");
						nostep = true;
					} else if ((packet[0] == 's') && gdb_con->sync) {
						/* Hmm..... when you issue a continue in GDB, then a "stepi" is
						 * sent by GDB first to OpenOCD, thus defeating the check to
						 * make only the single stepping have the sync feature...
						 */
						nostep = true;
						LOG_WARNING("stepi ignored. GDB will now fetch the register state " \
								"from the target.");
					}
					gdb_con->sync = false;

					if (!already_running && nostep) {
						/* Either the target isn't in the halted state, then we can't
						 * step/continue. This might be early setup, etc.
						 *
						 * Or we want to allow GDB to pick up a fresh set of
						 * register values without modifying the target state.
						 *
						 */
						gdb_sig_halted(connection);

						/* stop forwarding log packets! */
						log_remove_callback(gdb_log_callback, connection);
					} else {
						/* We're running/stepping, in which case we can
						 * forward log output until the target is halted
						 */
						gdb_con->frontend_state = TARGET_RUNNING;
						target_call_event_callbacks(target, TARGET_EVENT_GDB_START);

						if (!already_running) {
							/* Here we don't want packet processing to stop even if this fails,
							 * so we use a local variable instead of retval. */
							retval = gdb_step_continue_packet(connection, packet, packet_size);
							if (retval != ERROR_OK) {
								/* we'll never receive a halted
								 * condition... issue a false one..
								 */
								gdb_frontend_halted(target, connection);
							}
						}
					}
				}
				break;
				case 'v':
					retval = gdb_v_packet(connection, packet, packet_size);
					break;
				case 'D':
					retval = gdb_detach(connection);
					extended_protocol = 0;
					break;
				case 'X':
					retval = gdb_write_memory_binary_packet(connection, packet, packet_size);
					if (retval != ERROR_OK)
						return retval;
					break;
				case 'k':
					if (extended_protocol != 0) {
						gdb_con->attached = false;
						break;
					}
					gdb_put_packet(connection, "OK", 2);
					return ERROR_SERVER_REMOTE_CLOSED;
				case '!':
					/* handle extended remote protocol */
					extended_protocol = 1;
					gdb_put_packet(connection, "OK", 2);
					break;
				case 'R':
					/* handle extended restart packet */
					breakpoint_clear_target(gdb_service->target);
					watchpoint_clear_target(gdb_service->target);
					command_run_linef(connection->cmd_ctx, "ocd_gdb_restart %s",
							target_name(target));
					/* set connection as attached after reset */
					gdb_con->attached = true;
					/*  info rtos parts */
					gdb_thread_packet(connection, packet, packet_size);
					break;

				case 'j':
					/* packet supported only by smp target i.e cortex_a.c*/
					/* handle smp packet replying coreid played to gbd */
					gdb_read_smp_packet(connection, packet, packet_size);
					break;

				case 'J':
					/* packet supported only by smp target i.e cortex_a.c */
					/* handle smp packet setting coreid to be played at next
					 * resume to gdb */
					gdb_write_smp_packet(connection, packet, packet_size);
					break;

				default:
					/* ignore unknown packets */
					LOG_DEBUG("ignoring 0x%2.2x packet", packet[0]);
					gdb_put_packet(connection, NULL, 0);
					break;
			}

			/* if a packet handler returned an error, exit input loop */
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->ctrl_c) {
			if (target->state == TARGET_RUNNING) {
				retval = target_halt(target);
				if (retval != ERROR_OK)
					target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);
				gdb_con->ctrl_c = 0;
			} else {
				LOG_INFO("The target is not running when halt was requested, stopping GDB.");
				target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);
			}
		}

	} while (gdb_con->buf_cnt > 0);

	return ERROR_OK;
}

static int gdb_input(struct connection *connection)
{
	int retval = gdb_input_inner(connection);
	struct gdb_connection *gdb_con = connection->priv;
	if (retval == ERROR_SERVER_REMOTE_CLOSED)
		return retval;

	/* logging does not propagate the error, yet can set the gdb_con->closed flag */
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	/* we'll recover from any other errors(e.g. temporary timeouts, etc.) */
	return ERROR_OK;
}

static int gdb_target_start(struct target *target, const char *port)
{
	struct gdb_service *gdb_service;
	int ret;
	gdb_service = malloc(sizeof(struct gdb_service));

	if (NULL == gdb_service)
		return -ENOMEM;

	gdb_service->target = target;
	gdb_service->core[0] = -1;
	gdb_service->core[1] = -1;
	target->gdb_service = gdb_service;

	ret = add_service("gdb",
			port, 1, &gdb_new_connection, &gdb_input,
			&gdb_connection_closed, gdb_service);
	/* initialialize all targets gdb service with the same pointer */
	{
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			if (curr != target)
				curr->gdb_service = gdb_service;
			head = head->next;
		}
	}
	return ret;
}

static int gdb_target_add_one(struct target *target)
{
	/*  one gdb instance per smp list */
	if ((target->smp) && (target->gdb_service))
		return ERROR_OK;
	int retval = gdb_target_start(target, gdb_port_next);
	if (retval == ERROR_OK) {
		long portnumber;
		/* If we can parse the port number
		 * then we increment the port number for the next target.
		 */
		char *end;
		portnumber = strtol(gdb_port_next, &end, 0);
		if (!*end) {
			if (parse_long(gdb_port_next, &portnumber) == ERROR_OK) {
				free((void *)gdb_port_next);
				gdb_port_next = alloc_printf("%d", portnumber+1);
			}
		}
	}
	return retval;
}

int gdb_target_add_all(struct target *target)
{
	if (NULL == target) {
		LOG_WARNING("gdb services need one or more targets defined");
		return ERROR_OK;
	}

	while (NULL != target) {
		int retval = gdb_target_add_one(target);
		if (ERROR_OK != retval)
			return retval;

		target = target->next;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_sync_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (current_gdb_connection == NULL) {
		command_print(CMD_CTX,
			"gdb_sync command can only be run from within gdb using \"monitor gdb_sync\"");
		return ERROR_FAIL;
	}

	current_gdb_connection->sync = true;

	return ERROR_OK;
}

/* daemon configuration command gdb_port */
COMMAND_HANDLER(handle_gdb_port_command)
{
	int retval = CALL_COMMAND_HANDLER(server_pipe_command, &gdb_port);
	if (ERROR_OK == retval) {
		free((void *)gdb_port_next);
		gdb_port_next = strdup(gdb_port);
	}
	return retval;
}

COMMAND_HANDLER(handle_gdb_memory_map_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_use_memory_map);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_flash_program_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_flash_program);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_report_data_abort_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_report_data_abort);
	return ERROR_OK;
}

/* gdb_breakpoint_override */
COMMAND_HANDLER(handle_gdb_breakpoint_override_command)
{
	if (CMD_ARGC == 0) {
		/* nothing */
	} else if (CMD_ARGC == 1) {
		gdb_breakpoint_override = 1;
		if (strcmp(CMD_ARGV[0], "hard") == 0)
			gdb_breakpoint_override_type = BKPT_HARD;
		else if (strcmp(CMD_ARGV[0], "soft") == 0)
			gdb_breakpoint_override_type = BKPT_SOFT;
		else if (strcmp(CMD_ARGV[0], "disable") == 0)
			gdb_breakpoint_override = 0;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (gdb_breakpoint_override)
		LOG_USER("force %s breakpoints",
			(gdb_breakpoint_override_type == BKPT_HARD) ? "hard" : "soft");
	else
		LOG_USER("breakpoint type is not overridden");

	return ERROR_OK;
}

static const struct command_registration gdb_command_handlers[] = {
	{
		.name = "gdb_sync",
		.handler = handle_gdb_sync_command,
		.mode = COMMAND_ANY,
		.help = "next stepi will return immediately allowing "
			"GDB to fetch register state without affecting "
			"target state",
		.usage = ""
	},
	{
		.name = "gdb_port",
		.handler = handle_gdb_port_command,
		.mode = COMMAND_ANY,
		.help = "Normally gdb listens to a TCP/IP port. Each subsequent GDB "
			"server listens for the next port number after the "
			"base port number specified. "
			"No arguments reports GDB port. \"pipe\" means listen to stdin "
			"output to stdout, an integer is base port number, \"disable\" disables "
			"port. Any other string is are interpreted as named pipe to listen to. "
			"Output pipe is the same name as input pipe, but with 'o' appended.",
		.usage = "[port_num]",
	},
	{
		.name = "gdb_memory_map",
		.handler = handle_gdb_memory_map_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable memory map",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_flash_program",
		.handler = handle_gdb_flash_program_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable flash program",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_report_data_abort",
		.handler = handle_gdb_report_data_abort_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable reporting data aborts",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "gdb_breakpoint_override",
		.handler = handle_gdb_breakpoint_override_command,
		.mode = COMMAND_ANY,
		.help = "Display or specify type of breakpoint "
			"to be used by gdb 'break' commands.",
		.usage = "('hard'|'soft'|'disable')"
	},
	COMMAND_REGISTRATION_DONE
};

int gdb_register_commands(struct command_context *cmd_ctx)
{
	gdb_port = strdup("3333");
	gdb_port_next = strdup("3333");
	return register_commands(cmd_ctx, NULL, gdb_command_handlers);
}
