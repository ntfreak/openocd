/***************************************************************************
 *   Copyright (C) 2020 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include <server/server.h>
#include <target/target.h>
#include "ipdbg.h"
#include <helper/time_support.h>

#define IPDBG_MIN_NUM_OF_OPTIONS 4
#define IPDBG_MAX_NUM_OF_OPTIONS 14

/* private connection data for IPDBG */
struct ipdbg_fifo {
	size_t count;
	size_t rd_idx;
	char buffer[IPDBG_BUFFER_SIZE];
};

struct ipdbg_connection {
	struct ipdbg_fifo dn_fifo;
	struct ipdbg_fifo up_fifo;
};

struct ipdbg_service {
	struct ipdbg_hub *hub;
	struct ipdbg_service *next;
	uint16_t port;
	struct ipdbg_connection connection;
	uint8_t tool;
};

struct virtual_ir_info {
	uint32_t instruction;
	uint32_t length;
	uint32_t value;
};

struct ipdbg_hub {
	uint32_t user_instruction;
	uint32_t max_tools;
	uint32_t active_connections;
	uint32_t active_services;
	uint32_t valid_mask;
	uint32_t xoff_mask;
	uint32_t tool_mask;
	uint32_t last_dn_tool;
	struct ipdbg_hub *next;
	struct jtag_tap *tap;
	struct connection **connections;
	uint8_t data_register_length;
	uint8_t dn_xoff;
	struct virtual_ir_info *virtual_ir;
};

static struct ipdbg_hub *ipdbg_first_hub;

static struct ipdbg_service *ipdbg_first_service;

static void init_fifo(struct ipdbg_fifo *fifo)
{
	fifo->count = 0;
	fifo->rd_idx = 0;
}

static bool fifo_is_empty(struct ipdbg_fifo *fifo)
{
	return fifo->count == 0;
}

static bool fifo_is_full(struct ipdbg_fifo *fifo)
{
	return fifo->count == IPDBG_BUFFER_SIZE;
}

static void zero_rd_idx(struct ipdbg_fifo *fifo)
{
	if (fifo->rd_idx == 0)
		return;

	size_t ri = fifo->rd_idx;
	for (size_t idx = 0 ; idx < fifo->count ; ++idx)
		fifo->buffer[idx] = fifo->buffer[ri++];
	fifo->rd_idx = 0;
}

static void append_to_fifo(struct ipdbg_fifo *fifo, char data)
{
	if (fifo_is_full(fifo))
		return;

	zero_rd_idx(fifo);
	fifo->buffer[fifo->count++] = data;
}

static char get_from_fifo(struct ipdbg_fifo *fifo)
{
	if (fifo_is_empty(fifo))
		return 0;

	fifo->count--;
	return fifo->buffer[fifo->rd_idx++];
}

static void move_buffer_to_connection(struct connection *conn, struct ipdbg_fifo *fifo)
{
	if (fifo_is_empty(fifo))
		return;

	zero_rd_idx(fifo);
	connection_write(conn, fifo->buffer, fifo->count);
	fifo->count = 0;
}

static int move_connection_to_fifo(struct connection *conn, struct ipdbg_fifo *fifo)
{
	if (fifo_is_full(fifo))
		return 1;

	zero_rd_idx(fifo);
	int bytes_read = connection_read(conn, fifo->buffer + fifo->count, IPDBG_BUFFER_SIZE - fifo->count);
	if (bytes_read > 0)
		fifo->count += bytes_read;

	return bytes_read;
}

static int max_tools_from_data_register_length(uint8_t data_register_length)
{
	int max_tools = 1;
	data_register_length -= 10; /* 8 bit payload, 1 xoff-flag, 1 valid-flag; remaining bits used to select tool*/
	while (data_register_length--)
		max_tools *= 2;

	return max_tools - 1; /* last tool is used to reset JtagCDC */
}

static struct ipdbg_service *find_ipdbg_service(struct ipdbg_hub *hub, uint8_t tool)
{
	struct ipdbg_service *service;
	for (service = ipdbg_first_service ; service ; service = service->next) {
		if (service->hub == hub && service->tool == tool)
			break;
	}
	return service;
}

static void add_ipdbg_service(struct ipdbg_service *service)
{
	struct ipdbg_service *iservice;
	if (ipdbg_first_service != NULL) {
		for (iservice = ipdbg_first_service ; iservice->next; iservice = iservice->next)
			;
		iservice->next = service;
	} else
		ipdbg_first_service = service;
}

static int create_ipdbg_service(struct ipdbg_hub *hub, uint8_t tool, struct ipdbg_service **service, uint16_t port)
{
	*service = malloc(sizeof(struct ipdbg_service));
	if (*service == NULL)
		return -ENOMEM;

	(*service)->hub = hub;
	(*service)->tool = tool;
	(*service)->next = NULL;
	(*service)->port = port;

	return ERROR_OK;
}

static int remove_ipdbg_service(struct ipdbg_service *service)
{
	if (ipdbg_first_service == NULL)
		return ERROR_FAIL;

	if (service == ipdbg_first_service) {
		ipdbg_first_service = ipdbg_first_service->next;
		return ERROR_OK;
	}

	for (struct ipdbg_service *iservice = ipdbg_first_service ; iservice->next ; iservice = iservice->next) {
		if (service == iservice->next) {
			iservice->next = service->next;
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
}

static struct ipdbg_hub *find_hub(struct jtag_tap *tap, uint32_t user_instruction, struct virtual_ir_info *virtual_ir)
{
	struct ipdbg_hub *hub = NULL;
	for (hub = ipdbg_first_hub ; hub ; hub = hub->next) {
		if (hub->tap == tap && hub->user_instruction == user_instruction) {
			if ((!virtual_ir && !hub->virtual_ir) ||
				 (virtual_ir && hub->virtual_ir &&
				  virtual_ir->instruction == hub->virtual_ir->instruction &&
				  virtual_ir->length == hub->virtual_ir->length &&
				  virtual_ir->value == hub->virtual_ir->value)) {
				break;
			}
		}
	}
	return hub;
}

static void add_hub(struct ipdbg_hub *hub)
{
	struct ipdbg_hub *ihub;
	if (ipdbg_first_hub != NULL) {
		for (ihub = ipdbg_first_hub ; ihub->next; ihub = ihub->next);
		ihub->next = hub;
	} else
		ipdbg_first_hub = hub;
}

static int create_hub(struct jtag_tap *tap, uint32_t user_instruction, uint8_t data_register_length,
					  struct virtual_ir_info *virtual_ir, struct ipdbg_hub **hub)
{
	*hub = NULL;
	struct ipdbg_hub *new_hub = malloc(sizeof(struct ipdbg_hub));
	if (new_hub == NULL) {
		free(virtual_ir);
		return -ENOMEM;
	}

	new_hub->tap                  = tap;
	new_hub->user_instruction     = user_instruction;
	new_hub->data_register_length = data_register_length;
	new_hub->max_tools            = max_tools_from_data_register_length(data_register_length) ;
	new_hub->valid_mask           = 1 << (data_register_length - 1);
	new_hub->xoff_mask            = 1 << (data_register_length - 2);
	new_hub->tool_mask            = (new_hub->xoff_mask - 1) >> 8;
	new_hub->last_dn_tool         = new_hub->tool_mask;
	new_hub->dn_xoff              = 0;
	new_hub->active_connections   = 0;
	new_hub->active_services      = 0;
	new_hub->virtual_ir           = virtual_ir;
	new_hub->next                 = NULL;
	new_hub->connections          = malloc(new_hub->max_tools * sizeof(struct connection *));
	if (new_hub->connections == NULL) {
		free(virtual_ir);
		free(new_hub);
		return -ENOMEM;
	}
	memset(new_hub->connections, 0, new_hub->max_tools * sizeof(struct connection *));

	*hub = new_hub;
	return ERROR_OK;
}

static void free_hub(struct ipdbg_hub *hub)
{
	if (hub == NULL)
		return;
	free(hub->connections);
	free(hub->virtual_ir);
	free(hub);
}

static int remove_hub(struct ipdbg_hub *hub)
{
	if (ipdbg_first_hub == NULL)
		return ERROR_FAIL;
	if (hub == ipdbg_first_hub) {
		ipdbg_first_hub = ipdbg_first_hub->next;
		return ERROR_OK;
	}

	for (struct ipdbg_hub *ihub = ipdbg_first_hub ; ihub->next ; ihub = ihub->next) {
		if (hub == ihub->next) {
			ihub->next = hub->next;
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int shift_instr(struct ipdbg_hub *hub, uint32_t instr)
{
	if (hub == NULL)
		return ERROR_FAIL;

	struct jtag_tap *tap = hub->tap;
	if (tap == NULL)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != instr) {
		uint8_t *ir_out_val = calloc(DIV_ROUND_UP(tap->ir_length, 8), 1);
		buf_set_u32(ir_out_val, 0, tap->ir_length, instr);

		struct scan_field fields = {
			.check_mask = NULL,
			.check_value = NULL,
			.in_value = NULL,
			.num_bits = tap->ir_length,
			.out_value = ir_out_val
		};
		jtag_add_ir_scan(tap, &fields, TAP_IDLE);
		int retval = jtag_execute_queue();

		free(ir_out_val);

		return retval;
	}

	return ERROR_OK;
}

static int shift_vir(struct ipdbg_hub *hub)
{
	if (hub == NULL)
		return ERROR_FAIL;

	if (!hub->virtual_ir)
		return ERROR_OK;

	if (shift_instr(hub, hub->virtual_ir->instruction) != ERROR_OK)
		return ERROR_FAIL;

	struct jtag_tap *tap = hub->tap;
	if (tap == NULL)
		return ERROR_FAIL;

	uint8_t *dr_out_val = calloc(DIV_ROUND_UP(hub->virtual_ir->length, 8), 1);
	buf_set_u32(dr_out_val, 0, hub->virtual_ir->length, hub->virtual_ir->value);

	struct scan_field fields = {
		.check_mask = NULL,
		.check_value = NULL,
		.in_value = NULL,
		.num_bits = hub->virtual_ir->length,
		.out_value = dr_out_val
	};
	jtag_add_dr_scan(tap, 1, &fields, TAP_IDLE);
	int retval = jtag_execute_queue();

	free(dr_out_val);

	return retval;
}

static int ipdbg_shift_data(struct ipdbg_hub *hub, uint32_t dn_data, uint32_t *up_data)
{
	if (hub == NULL)
		return ERROR_FAIL;

	struct jtag_tap *tap = hub->tap;
	if (tap == NULL)
		return ERROR_FAIL;

	uint8_t *dr_out_val = calloc(DIV_ROUND_UP(hub->data_register_length, 8), 1);
	buf_set_u32(dr_out_val, 0, hub->data_register_length, dn_data);
	uint8_t *dr_in_val = up_data ? calloc(DIV_ROUND_UP(hub->data_register_length, 8), 1) : NULL;

	struct scan_field fields = {
		.check_mask = NULL,
		.check_value = NULL,
		.in_value = dr_in_val,
		.num_bits = hub->data_register_length,
		.out_value = dr_out_val
	};
	jtag_add_dr_scan(tap, 1, &fields, TAP_IDLE);
	int retval = jtag_execute_queue();

	if (up_data && retval == ERROR_OK)
		*up_data = buf_get_u32(dr_in_val, 0, hub->data_register_length);

	free(dr_out_val);
	free(dr_in_val);

	return retval;
}

static int distribute_data_from_hub(struct ipdbg_hub *hub, uint32_t up)
{
	const bool valid_up_data = up & hub->valid_mask;
	if (!valid_up_data)
		return 0;

	const size_t tool = (up >> 8) & hub->tool_mask;
	if (tool == hub->tool_mask) {
		const uint8_t xon_cmd = up & 0x00ff;
		hub->dn_xoff &= ~xon_cmd;
		printf("received xon cmd: %d\n", xon_cmd);
	} else {
		struct connection *conn = hub->connections[tool];
		if (conn) {
			struct ipdbg_connection *connection = conn->priv;
			if (fifo_is_full(&(connection->up_fifo)))
				move_buffer_to_connection(conn, &(connection->up_fifo));

			append_to_fifo(&(connection->up_fifo), up);
		}
	}
	return 1;
}

static int polling_callback(void *priv)
{
	struct ipdbg_hub *hub = priv;

	int ret = shift_vir(hub);
	if (ret != ERROR_OK)
		return ret;

	ret = shift_instr(hub, hub->user_instruction);
	if (ret != ERROR_OK)
		return ret;

	/* transfer dn buffers to jtag-hub */
	unsigned empty_up_transfers = 0;
	for (size_t tool = 0 ; tool < hub->max_tools ; ++tool) {
		struct connection *conn = hub->connections[tool];
		if (conn && conn->priv) {
			struct ipdbg_connection *connection = conn->priv;
			while (((hub->dn_xoff & (1ul << tool)) == 0) && !fifo_is_empty(&(connection->dn_fifo))) {
				uint32_t dn = hub->valid_mask |
							  ((tool & hub->tool_mask) << 8) |
							  (0x00fful & get_from_fifo(&(connection->dn_fifo)));
				uint32_t up = 0;
				ret = ipdbg_shift_data(hub, dn, &up);
				if (ret != ERROR_OK)
					return ret;

				if (distribute_data_from_hub(hub, up))
					empty_up_transfers = 0;
				else
					++empty_up_transfers;

				if ((up & hub->xoff_mask) && (hub->last_dn_tool != hub->max_tools)) {
					hub->dn_xoff |= (1ul << hub->last_dn_tool);
					log_printf_lf(LOG_LVL_INFO, __FILE__, __LINE__, __func__,
								"tool %d sent xoff", hub->last_dn_tool);
				}

				hub->last_dn_tool = tool;
			}
		}
	}

	/* some transfers to get data from jtag-hub in case there is no dn data */
	while (empty_up_transfers++ < hub->max_tools) {
		uint32_t dn = 0;
		uint32_t up = 0;
		int retval = ipdbg_shift_data(hub, dn, &up);
		if (retval != ERROR_OK)
			return ret;
		distribute_data_from_hub(hub, up);
	}

	for (size_t tool = 0 ; tool < hub->max_tools ; ++tool) {
		struct connection *conn = hub->connections[tool];
		if (conn && conn->priv) {
			struct ipdbg_connection *connection = conn->priv;

			if (!fifo_is_empty(&(connection->up_fifo)))
				move_buffer_to_connection(conn, &(connection->up_fifo));
		}
	}

	return ERROR_OK;
}

static int start_polling(struct ipdbg_service *service, struct connection *connection)
{
	struct ipdbg_hub *hub = service->hub;
	hub->connections[service->tool] = connection;
	hub->active_connections++;
	if (hub->active_connections == 1) {
		const uint32_t resetHub = hub->valid_mask | ((hub->max_tools) << 8);

		int ret = shift_vir(hub);
		if (ret != ERROR_OK)
			return ret;

		ret = shift_instr(hub, hub->user_instruction);
		if (ret != ERROR_OK)
			return ret;

		ret = ipdbg_shift_data(hub, resetHub, NULL);
		hub->last_dn_tool = hub->tool_mask;
		hub->dn_xoff = 0;
		if (ret != ERROR_OK)
			return ret;

		log_printf_lf(LOG_LVL_INFO, __FILE__, __LINE__, __func__,
			"IPDBG start_polling");

		const int time_ms = 20;
		const int periodic = 1;
		return target_register_timer_callback(polling_callback, time_ms, periodic, hub);
	}
	return ERROR_OK;
}

static int stop_polling(struct ipdbg_service *service, struct connection *connection)
{
	struct ipdbg_hub *hub = service->hub;
	hub->connections[service->tool] = NULL;
	hub->active_connections--;
	if (hub->active_connections == 0) {
		log_printf_lf(LOG_LVL_INFO, __FILE__, __LINE__, __func__,
			"IPDBG stop_polling");

		return target_unregister_timer_callback(polling_callback, hub);
	}

	return ERROR_OK;
}

static int on_new_connection(struct connection *connection)
{
	struct ipdbg_service *service = connection->service->priv;
	connection->priv = &(service->connection);
	/* initialize ipdbg connection information */
	init_fifo(&(service->connection.up_fifo));
	init_fifo(&(service->connection.dn_fifo));

	start_polling(service, connection);

	log_printf_lf(LOG_LVL_INFO, __FILE__, __LINE__, __func__,
		"New IPDBG Connection");

	return ERROR_OK;
}

static int on_connection_input(struct connection *connection)
{
	struct ipdbg_connection *conn = connection->priv;
	int bytes_read = move_connection_to_fifo(connection, &(conn->dn_fifo));

	if (bytes_read == 0)
		return ERROR_SERVER_REMOTE_CLOSED;
	else if (bytes_read == -1) {
		LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	/* we'll recover from any other errors(e.g. temporary timeouts, etc.) */
	return ERROR_OK;
}

static int on_connection_closed(struct connection *connection)
{
	log_printf_lf(LOG_LVL_INFO, __FILE__, __LINE__, __func__,
		"Closed IPDBG Connection ");

	return stop_polling(connection->service->priv, connection);
}

static int ipdbg_start(uint16_t port, struct jtag_tap *tap, uint32_t user_instruction,
					uint8_t data_register_length, struct virtual_ir_info *virtual_ir, uint8_t tool)
{
	LOG_DEBUG("starting ipdbg service on port %d for tool %d", port, tool);

	struct ipdbg_hub *hub = find_hub(tap, user_instruction, virtual_ir);
	if (hub) {
		free(virtual_ir);
		if (hub->data_register_length != data_register_length) {
			LOG_DEBUG("hub must have the same data_register_length for all tools");
			return ERROR_FAIL;
		}
	} else {
		int retval = create_hub(tap, user_instruction, data_register_length, virtual_ir, &hub);
		if (retval != ERROR_OK) {
			free(virtual_ir);
			return retval;
		}
	}

	struct ipdbg_service *service = NULL;
	int retval = create_ipdbg_service(hub, tool, &service, port);

	if (retval != ERROR_OK || service == NULL) {
		if (hub->active_services == 0 && hub->active_connections == 0)
			free_hub(hub);
		return -ENOMEM;
	}

	char port_str_buffer[6];
	snprintf(port_str_buffer, 6, "%u", port);
	retval = add_service("ipdbg", port_str_buffer, 1, &on_new_connection,
		&on_connection_input, &on_connection_closed, service, NULL);
	if (retval == ERROR_OK) {
		add_ipdbg_service(service);
		if (hub->active_services == 0 && hub->active_connections == 0)
			add_hub(hub);
		hub->active_services++;
	} else {
		if (hub->active_services == 0 && hub->active_connections == 0)
			free_hub(hub);
		free(service);
	}

	return retval;
}

static int ipdbg_stop(struct jtag_tap *tap, uint32_t user_instruction, struct virtual_ir_info *virtual_ir, uint8_t tool)
{
	struct ipdbg_hub *hub = find_hub(tap, user_instruction, virtual_ir);
	free(virtual_ir);
	if (hub == NULL)
		return ERROR_FAIL;

	struct ipdbg_service *service = find_ipdbg_service(hub, tool);
	if (service == NULL)
		return ERROR_FAIL;

	remove_ipdbg_service(service);
	char port_str_buffer[6];
	snprintf(port_str_buffer, 6, "%u", service->port);
	int retval = remove_service("ipdbg", port_str_buffer);
	if (retval == ERROR_OK) {
		hub->active_services--;
		if (hub->active_connections == 0 && hub->active_services == 0) {
			remove_hub(hub);
			free_hub(hub);
		}
	} else
		LOG_ERROR("BUG: remove_service failed");

	return retval;
}

COMMAND_HANDLER(handle_ipdbg_command)
{
	struct jtag_tap *tap = NULL;
	uint16_t port = 4242;
	uint8_t tool = 1;
	uint32_t user_instruction = 0x00;
	uint8_t data_register_length = 13;
	bool start = true;
	bool hub_configured = false;
	bool has_virtual_ir = false;
	uint32_t virtual_ir_instruction = 0x00e;
	uint32_t virtual_ir_length = 5;
	uint32_t virtual_ir_value = 0x11;
	struct virtual_ir_info *virtual_ir = NULL;

	if ((CMD_ARGC < IPDBG_MIN_NUM_OF_OPTIONS) || (CMD_ARGC > IPDBG_MAX_NUM_OF_OPTIONS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned int i = 0; i < CMD_ARGC; ++i) {
		if (strcmp(CMD_ARGV[i], "-tap") == 0) {
			if (i+1 < CMD_ARGC && CMD_ARGV[i+1][0] != '-') {
				tap = jtag_tap_by_string(CMD_ARGV[i+1]);
				if (!tap) {
					command_print(CMD, "Tap %s unknown", CMD_ARGV[i+1]);
					return ERROR_FAIL;
				}
				++i;
			} else {
				command_print(CMD, "no TAP given");
				return ERROR_FAIL;
			}
		} else if (strcmp(CMD_ARGV[i], "-hub") == 0) {
			if (i+1 < CMD_ARGC && CMD_ARGV[i+1][0] != '-') {
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i+1], user_instruction);
				if (i+2 < CMD_ARGC && CMD_ARGV[i+2][0] != '-') {
						COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i+2], data_register_length);
						if (data_register_length < 11 || data_register_length > 32) {
							command_print(CMD, "length of \"user\"-data register must be at least 11 and at most 32.");
							return ERROR_FAIL;
						}
						++i;
				}
				hub_configured = true;
				++i;
			} else {
				command_print(CMD, "no ir_value to select hub given");
				return ERROR_FAIL;
			}
		} else if (strcmp(CMD_ARGV[i], "-vir") == 0) {
			if (i+1 < CMD_ARGC && CMD_ARGV[i+1][0] != '-') {
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i+1], virtual_ir_value);
				++i;
			}
			if (i+1 < CMD_ARGC && CMD_ARGV[i+1][0] != '-') {
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i+1], virtual_ir_length);
				++i;
			}
			if (i+1 < CMD_ARGC && CMD_ARGV[i+1][0] != '-') {
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i+1], virtual_ir_instruction);
				++i;
			}
			has_virtual_ir = true;
		} else if (strcmp(CMD_ARGV[i], "-port") == 0) {
			if (i+1 < CMD_ARGC && CMD_ARGV[i+1][0] != '-') {
				COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i+1], port);
				++i;
			} else {
				command_print(CMD, "no port number given");
				return ERROR_FAIL;
			}
		} else if (strcmp(CMD_ARGV[i], "-tool") == 0) {
			if (i+1 < CMD_ARGC && CMD_ARGV[i+1][0] != '-') {
				COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i+1], tool);
				++i;
			} else {
				command_print(CMD, "no tool given");
				return ERROR_FAIL;
			}
		} else if (strcmp(CMD_ARGV[i], "-stop") == 0) {
			start = false;
		} else if (strcmp(CMD_ARGV[i], "-start") == 0) {
			start = true;
		} else {
			command_print(CMD, "Unknown argument: %s", CMD_ARGV[i]);
			return ERROR_FAIL;
		}
	}

	if (!tap) {
		command_print(CMD, "no valid tap selected");
		return ERROR_FAIL;
	}

	if (!hub_configured) {
		command_print(CMD, "hub not configured correctly");
		return ERROR_FAIL;
	}

	if (tool >= max_tools_from_data_register_length(data_register_length)) {
		command_print(CMD, "Tool: %d is invalid", tool);
		return ERROR_FAIL;
	}

	if (has_virtual_ir) {
		virtual_ir = malloc(sizeof(struct virtual_ir_info));
		if (virtual_ir == NULL)
			return -ENOMEM;
		virtual_ir->instruction = virtual_ir_instruction;
		virtual_ir->length      = virtual_ir_length;
		virtual_ir->value       = virtual_ir_value;
	}

	if (start)
		return ipdbg_start(port, tap, user_instruction, data_register_length, virtual_ir, tool);
	else
		return ipdbg_stop(tap, user_instruction, virtual_ir, tool);
}

static const struct command_registration ipdbg_command_handlers[] = {
	{
		.name = "ipdbg",
		.handler = handle_ipdbg_command,
		.mode = COMMAND_EXEC,
		.help = "Starts or stops an IPDBG JTAG-Host server.",
		.usage = "[-start|-stop] -tap device.tap -hub ir_value [dr_length]"
				 " [-port number] [-tool number] [-vir [vir_value [length [instr_code]]]]",
	},
	COMMAND_REGISTRATION_DONE
};

int ipdbg_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, ipdbg_command_handlers);
}
