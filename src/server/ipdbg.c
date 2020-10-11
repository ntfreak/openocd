/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright (C) 2020 by Daniel Anselmi <danselmi@gmx.ch> */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/bits.h>
#include <helper/time_support.h>
#include <jtag/jtag.h>
#include <server/server.h>
#include <target/target.h>

#include "ipdbg.h"

#define IPDBG_BUFFER_SIZE 16384
#define IPDBG_MIN_NUM_OF_OPTIONS 4
#define IPDBG_MAX_NUM_OF_OPTIONS 14
#define IPDBG_MIN_DR_LENGTH 11
#define IPDBG_MAX_DR_LENGTH 13
#define IPDBG_TCP_PORT_STR_MAX_LENGTH 6

/* private connection data for IPDBG */
struct ipdbg_fifo {
	size_t count;
	size_t rd_idx;
	char buffer[IPDBG_BUFFER_SIZE];
};

struct ipdbg_connection {
	struct ipdbg_fifo dn_fifo;
	struct ipdbg_fifo up_fifo;
	bool closed;
};

struct ipdbg_service {
	struct ipdbg_hub *hub;
	struct ipdbg_service *next;
	uint16_t port;
	struct ipdbg_connection connection;
	uint8_t tool;
};

struct ipdbg_virtual_ir_info {
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
	struct ipdbg_virtual_ir_info *virtual_ir;
};

static struct ipdbg_hub *ipdbg_first_hub;

static struct ipdbg_service *ipdbg_first_service;

static void ipdbg_init_fifo(struct ipdbg_fifo *fifo)
{
	fifo->count = 0;
	fifo->rd_idx = 0;
}

static bool ipdbg_fifo_is_empty(struct ipdbg_fifo *fifo)
{
	return fifo->count == 0;
}

static bool ipdbg_fifo_is_full(struct ipdbg_fifo *fifo)
{
	return fifo->count == IPDBG_BUFFER_SIZE;
}

static void ipdbg_zero_rd_idx(struct ipdbg_fifo *fifo)
{
	if (fifo->rd_idx == 0)
		return;

	size_t ri = fifo->rd_idx;
	for (size_t idx = 0 ; idx < fifo->count ; ++idx)
		fifo->buffer[idx] = fifo->buffer[ri++];
	fifo->rd_idx = 0;
}

static void ipdbg_append_to_fifo(struct ipdbg_fifo *fifo, char data)
{
	if (ipdbg_fifo_is_full(fifo))
		return;

	ipdbg_zero_rd_idx(fifo);
	fifo->buffer[fifo->count++] = data;
}

static char ipdbg_get_from_fifo(struct ipdbg_fifo *fifo)
{
	if (ipdbg_fifo_is_empty(fifo))
		return 0;

	fifo->count--;
	return fifo->buffer[fifo->rd_idx++];
}

static int ipdbg_move_buffer_to_connection(struct connection *conn, struct ipdbg_fifo *fifo)
{
	if (ipdbg_fifo_is_empty(fifo))
		return ERROR_OK;

	struct ipdbg_connection *connection = conn->priv;
	if (connection->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	ipdbg_zero_rd_idx(fifo);
	size_t bytes_written = connection_write(conn, fifo->buffer, fifo->count);
	if (bytes_written != fifo->count) {
		LOG_ERROR("error during write: %zu != %zu", bytes_written, fifo->count);
		connection->closed = true;
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	fifo->count -= bytes_written;

	return ERROR_OK;
}

static int ipdbg_max_tools_from_data_register_length(uint8_t data_register_length)
{
	int max_tools = 1;
	data_register_length -= 10; /* 8 bit payload, 1 xoff-flag, 1 valid-flag; remaining bits used to select tool*/
	while (data_register_length--)
		max_tools *= 2;

	/* last tool is used to reset JtagCDC and transfer "XON" to host*/
	return max_tools - 1;
}

static struct ipdbg_service *ipdbg_find_service(struct ipdbg_hub *hub, uint8_t tool)
{
	struct ipdbg_service *service;
	for (service = ipdbg_first_service ; service ; service = service->next) {
		if (service->hub == hub && service->tool == tool)
			break;
	}
	return service;
}

static void ipdbg_add_service(struct ipdbg_service *service)
{
	struct ipdbg_service *iservice;
	if (ipdbg_first_service) {
		for (iservice = ipdbg_first_service ; iservice->next; iservice = iservice->next)
			;
		iservice->next = service;
	} else
		ipdbg_first_service = service;
}

static int ipdbg_create_service(struct ipdbg_hub *hub, uint8_t tool, struct ipdbg_service **service, uint16_t port)
{
	*service = calloc(1, sizeof(struct ipdbg_service));
	if (!*service) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	(*service)->hub = hub;
	(*service)->tool = tool;
	(*service)->port = port;

	return ERROR_OK;
}

static int ipdbg_remove_service(struct ipdbg_service *service)
{
	if (!ipdbg_first_service)
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

static struct ipdbg_hub *ipdbg_find_hub(struct jtag_tap *tap,
				uint32_t user_instruction, struct ipdbg_virtual_ir_info *virtual_ir)
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

static void ipdbg_add_hub(struct ipdbg_hub *hub)
{
	struct ipdbg_hub *ihub;
	if (ipdbg_first_hub) {
		for (ihub = ipdbg_first_hub ; ihub->next; ihub = ihub->next)
			;
		ihub->next = hub;
	} else
		ipdbg_first_hub = hub;
}

static int ipdbg_create_hub(struct jtag_tap *tap, uint32_t user_instruction, uint8_t data_register_length,
					  struct ipdbg_virtual_ir_info *virtual_ir, struct ipdbg_hub **hub)
{
	*hub = NULL;
	struct ipdbg_hub *new_hub = calloc(1, sizeof(struct ipdbg_hub));
	if (!new_hub) {
		free(virtual_ir);
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	new_hub->max_tools = ipdbg_max_tools_from_data_register_length(data_register_length);
	new_hub->connections = calloc(new_hub->max_tools, sizeof(struct connection *));
	if (!new_hub->connections) {
		free(virtual_ir);
		free(new_hub);
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	new_hub->tap                  = tap;
	new_hub->user_instruction     = user_instruction;
	new_hub->data_register_length = data_register_length;
	new_hub->valid_mask           = BIT(data_register_length - 1);
	new_hub->xoff_mask            = BIT(data_register_length - 2);
	new_hub->tool_mask            = (new_hub->xoff_mask - 1) >> 8;
	new_hub->last_dn_tool         = new_hub->tool_mask;
	new_hub->virtual_ir           = virtual_ir;

	*hub = new_hub;

	return ERROR_OK;
}

static void ipdbg_free_hub(struct ipdbg_hub *hub)
{
	if (!hub)
		return;
	free(hub->connections);
	free(hub->virtual_ir);
	free(hub);
}

static int ipdbg_remove_hub(struct ipdbg_hub *hub)
{
	if (!ipdbg_first_hub)
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

static void ipdbg_init_scan_field(struct scan_field *fields, uint8_t *in_value, int num_bits, const uint8_t *out_value)
{
	fields->check_mask = NULL;
	fields->check_value = NULL;
	fields->in_value = in_value;
	fields->num_bits = num_bits;
	fields->out_value = out_value;
}

static int ipdbg_shift_instr(struct ipdbg_hub *hub, uint32_t instr)
{
	if (!hub)
		return ERROR_FAIL;

	struct jtag_tap *tap = hub->tap;
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) == instr) {
		/* there is already the requested instruction in the ir */
		return ERROR_OK;
	}

	uint8_t *ir_out_val = calloc(DIV_ROUND_UP(tap->ir_length, 8), 1);
	buf_set_u32(ir_out_val, 0, tap->ir_length, instr);

	struct scan_field fields;
	ipdbg_init_scan_field(&fields, NULL, tap->ir_length, ir_out_val);
	jtag_add_ir_scan(tap, &fields, TAP_IDLE);
	int retval = jtag_execute_queue();

	free(ir_out_val);

	return retval;
}

static int ipdbg_shift_vir(struct ipdbg_hub *hub)
{
	if (!hub)
		return ERROR_FAIL;

	if (!hub->virtual_ir)
		return ERROR_OK;

	int retval = ipdbg_shift_instr(hub, hub->virtual_ir->instruction);
	if (retval != ERROR_OK)
		return retval;

	struct jtag_tap *tap = hub->tap;
	if (!tap)
		return ERROR_FAIL;

	uint8_t *dr_out_val = calloc(DIV_ROUND_UP(hub->virtual_ir->length, 8), 1);
	buf_set_u32(dr_out_val, 0, hub->virtual_ir->length, hub->virtual_ir->value);

	struct scan_field fields;
	ipdbg_init_scan_field(&fields, NULL, hub->virtual_ir->length, dr_out_val);
	jtag_add_dr_scan(tap, 1, &fields, TAP_IDLE);
	retval = jtag_execute_queue();

	free(dr_out_val);

	return retval;
}

static int ipdbg_shift_data(struct ipdbg_hub *hub, uint32_t dn_data, uint32_t *up_data)
{
	if (!hub)
		return ERROR_FAIL;

	struct jtag_tap *tap = hub->tap;
	if (!tap)
		return ERROR_FAIL;

	uint8_t *dr_out_val = calloc(DIV_ROUND_UP(hub->data_register_length, 8), 1);
	buf_set_u32(dr_out_val, 0, hub->data_register_length, dn_data);
	uint8_t *dr_in_val = up_data ? calloc(DIV_ROUND_UP(hub->data_register_length, 8), 1) : NULL;

	struct scan_field fields;
	ipdbg_init_scan_field(&fields, dr_in_val, hub->data_register_length, dr_out_val);
	jtag_add_dr_scan(tap, 1, &fields, TAP_IDLE);
	int retval = jtag_execute_queue();

	if (up_data && retval == ERROR_OK)
		*up_data = buf_get_u32(dr_in_val, 0, hub->data_register_length);

	free(dr_out_val);
	free(dr_in_val);

	return retval;
}

static int ipdbg_distribute_data_from_hub(struct ipdbg_hub *hub, uint32_t up)
{
	const bool valid_up_data = up & hub->valid_mask;
	if (!valid_up_data)
		return ERROR_OK;

	const size_t tool = (up >> 8) & hub->tool_mask;
	if (tool == hub->tool_mask) {
		const uint8_t xon_cmd = up & 0x00ff;
		hub->dn_xoff &= ~xon_cmd;
		LOG_INFO("received xon cmd: %d\n", xon_cmd);
		return ERROR_OK;
	}

	struct connection *conn = hub->connections[tool];
	if (conn) {
		struct ipdbg_connection *connection = conn->priv;
		if (ipdbg_fifo_is_full(&connection->up_fifo)) {
			int retval = ipdbg_move_buffer_to_connection(conn, &connection->up_fifo);
			if (retval != ERROR_OK)
				return retval;
		}
		ipdbg_append_to_fifo(&connection->up_fifo, up);
	}
	return ERROR_OK;
}

static int ipdbg_jtag_transfer_byte(struct ipdbg_hub *hub, size_t tool, struct ipdbg_connection *connection)
{
	uint32_t dn = hub->valid_mask | ((tool & hub->tool_mask) << 8) |
				(0x00fful & ipdbg_get_from_fifo(&connection->dn_fifo));
	uint32_t up = 0;
	int ret = ipdbg_shift_data(hub, dn, &up);
	if (ret != ERROR_OK)
		return ret;

	ret = ipdbg_distribute_data_from_hub(hub, up);
	if (ret != ERROR_OK)
		return ret;

	if ((up & hub->xoff_mask) && (hub->last_dn_tool != hub->max_tools)) {
		hub->dn_xoff |= BIT(hub->last_dn_tool);
		LOG_INFO("tool %d sent xoff", hub->last_dn_tool);
	}

	hub->last_dn_tool = tool;

	return ERROR_OK;
}

static int ipdbg_polling_callback(void *priv)
{
	struct ipdbg_hub *hub = priv;

	int ret = ipdbg_shift_vir(hub);
	if (ret != ERROR_OK)
		return ret;

	ret = ipdbg_shift_instr(hub, hub->user_instruction);
	if (ret != ERROR_OK)
		return ret;

	/* transfer dn buffers to jtag-hub */
	unsigned int num_transfers = 0;
	for (size_t tool = 0 ; tool < hub->max_tools ; ++tool) {
		struct connection *conn = hub->connections[tool];
		if (conn && conn->priv) {
			struct ipdbg_connection *connection = conn->priv;
			while (((hub->dn_xoff & BIT(tool)) == 0) && !ipdbg_fifo_is_empty(&connection->dn_fifo)) {
				ret = ipdbg_jtag_transfer_byte(hub, tool, connection);
				if (ret != ERROR_OK)
					return ret;
				++num_transfers;
			}
		}
	}

	/* some transfers to get data from jtag-hub in case there is no dn data */
	while (num_transfers++ < hub->max_tools) {
		uint32_t dn = 0;
		uint32_t up = 0;

		int retval = ipdbg_shift_data(hub, dn, &up);
		if (retval != ERROR_OK)
			return ret;

		retval = ipdbg_distribute_data_from_hub(hub, up);
		if (retval != ERROR_OK)
			return ret;
	}

	/* write from up fifos to sockets */
	for (size_t tool = 0 ; tool < hub->max_tools ; ++tool) {
		struct connection *conn = hub->connections[tool];
		if (conn && conn->priv) {
			struct ipdbg_connection *connection = conn->priv;
			int retval = ipdbg_move_buffer_to_connection(conn, &connection->up_fifo);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	return ERROR_OK;
}

static int ipdbg_start_polling(struct ipdbg_service *service, struct connection *connection)
{
	struct ipdbg_hub *hub = service->hub;
	hub->connections[service->tool] = connection;
	hub->active_connections++;
	if (hub->active_connections > 1) {
		/* hub is already initialized */
		return ERROR_OK;
	}

	const uint32_t reset_hub = hub->valid_mask | ((hub->max_tools) << 8);

	int ret = ipdbg_shift_vir(hub);
	if (ret != ERROR_OK)
		return ret;

	ret = ipdbg_shift_instr(hub, hub->user_instruction);
	if (ret != ERROR_OK)
		return ret;

	ret = ipdbg_shift_data(hub, reset_hub, NULL);
	hub->last_dn_tool = hub->tool_mask;
	hub->dn_xoff = 0;
	if (ret != ERROR_OK)
		return ret;

	LOG_INFO("IPDBG start_polling");

	const int time_ms = 20;
	const int periodic = 1;
	return target_register_timer_callback(ipdbg_polling_callback, time_ms, periodic, hub);
}

static int ipdbg_stop_polling(struct ipdbg_service *service)
{
	struct ipdbg_hub *hub = service->hub;
	hub->connections[service->tool] = NULL;
	hub->active_connections--;
	if (hub->active_connections == 0) {
		LOG_INFO("IPDBG stop_polling");

		return target_unregister_timer_callback(ipdbg_polling_callback, hub);
	}

	return ERROR_OK;
}

static int ipdbg_on_new_connection(struct connection *connection)
{
	struct ipdbg_service *service = connection->service->priv;
	connection->priv = &service->connection;
	/* initialize ipdbg connection information */
	ipdbg_init_fifo(&service->connection.up_fifo);
	ipdbg_init_fifo(&service->connection.dn_fifo);

	int retval = ipdbg_start_polling(service, connection);
	if (retval != ERROR_OK) {
		LOG_ERROR("BUG: ipdbg_start_polling failed");
		return retval;
	}

	struct ipdbg_connection *conn = connection->priv;
	conn->closed = false;

	LOG_INFO("New IPDBG Connection");

	return ERROR_OK;
}

static int ipdbg_on_connection_input(struct connection *connection)
{
	struct ipdbg_connection *conn = connection->priv;
	struct ipdbg_fifo *fifo = &conn->dn_fifo;

	if (ipdbg_fifo_is_full(fifo))
		return ERROR_OK;

	ipdbg_zero_rd_idx(fifo);
	int bytes_read = connection_read(connection, fifo->buffer + fifo->count, IPDBG_BUFFER_SIZE - fifo->count);
	if (bytes_read <= 0) {
		if (bytes_read < 0)
			LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	fifo->count += bytes_read;

	return ERROR_OK;
}

static int ipdbg_on_connection_closed(struct connection *connection)
{
	struct ipdbg_connection *conn = connection->priv;
	conn->closed = true;
	LOG_INFO("Closed IPDBG Connection");

	return ipdbg_stop_polling(connection->service->priv);
}

static int ipdbg_start(uint16_t port, struct jtag_tap *tap, uint32_t user_instruction,
					uint8_t data_register_length, struct ipdbg_virtual_ir_info *virtual_ir, uint8_t tool)
{
	LOG_INFO("starting ipdbg service on port %d for tool %d", port, tool);

	struct ipdbg_hub *hub = ipdbg_find_hub(tap, user_instruction, virtual_ir);
	if (hub) {
		free(virtual_ir);
		if (hub->data_register_length != data_register_length) {
			LOG_DEBUG("hub must have the same data_register_length for all tools");
			return ERROR_FAIL;
		}
	} else {
		int retval = ipdbg_create_hub(tap, user_instruction, data_register_length, virtual_ir, &hub);
		if (retval != ERROR_OK) {
			free(virtual_ir);
			return retval;
		}
	}

	struct ipdbg_service *service = NULL;
	int retval = ipdbg_create_service(hub, tool, &service, port);

	if (retval != ERROR_OK || !service) {
		if (hub->active_services == 0 && hub->active_connections == 0)
			ipdbg_free_hub(hub);
		return ERROR_FAIL;
	}

	char port_str_buffer[IPDBG_TCP_PORT_STR_MAX_LENGTH];
	snprintf(port_str_buffer, IPDBG_TCP_PORT_STR_MAX_LENGTH, "%u", port);
	retval = add_service("ipdbg", port_str_buffer, 1, &ipdbg_on_new_connection,
		&ipdbg_on_connection_input, &ipdbg_on_connection_closed, service);
	if (retval == ERROR_OK) {
		ipdbg_add_service(service);
		if (hub->active_services == 0 && hub->active_connections == 0)
			ipdbg_add_hub(hub);
		hub->active_services++;
	} else {
		if (hub->active_services == 0 && hub->active_connections == 0)
			ipdbg_free_hub(hub);
		free(service);
	}

	return retval;
}

static int ipdbg_stop(struct jtag_tap *tap, uint32_t user_instruction,
			struct ipdbg_virtual_ir_info *virtual_ir, uint8_t tool)
{
	struct ipdbg_hub *hub = ipdbg_find_hub(tap, user_instruction, virtual_ir);
	free(virtual_ir);
	if (!hub)
		return ERROR_FAIL;

	struct ipdbg_service *service = ipdbg_find_service(hub, tool);
	if (!service)
		return ERROR_FAIL;

	int retval = ipdbg_remove_service(service);
	if (retval != ERROR_OK) {
		LOG_ERROR("BUG: ipdbg_remove_service failed");
		return retval;
	}

	char port_str_buffer[IPDBG_TCP_PORT_STR_MAX_LENGTH];
	snprintf(port_str_buffer, IPDBG_TCP_PORT_STR_MAX_LENGTH, "%u", service->port);
	retval = remove_service("ipdbg", port_str_buffer);
	/* The ipdbg_service structure is freed by server.c:remove_service().
	   There the "priv" pointer is freed.*/
	if (retval != ERROR_OK) {
		LOG_ERROR("BUG: remove_service failed");
		return retval;
	}
	hub->active_services--;
	if (hub->active_connections == 0 && hub->active_services == 0) {
		retval = ipdbg_remove_hub(hub);
		if (retval != ERROR_OK) {
			LOG_ERROR("BUG: ipdbg_remove_hub failed");
			return retval;
		}
		ipdbg_free_hub(hub);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_ipdbg_command)
{
	struct jtag_tap *tap = NULL;
	uint16_t port = 4242;
	uint8_t tool = 1;
	uint32_t user_instruction = 0x00;
	uint8_t data_register_length = IPDBG_MAX_DR_LENGTH;
	bool start = true;
	bool hub_configured = false;
	bool has_virtual_ir = false;
	uint32_t virtual_ir_instruction = 0x00e;
	uint32_t virtual_ir_length = 5;
	uint32_t virtual_ir_value = 0x11;
	struct ipdbg_virtual_ir_info *virtual_ir = NULL;

	if ((CMD_ARGC < IPDBG_MIN_NUM_OF_OPTIONS) || (CMD_ARGC > IPDBG_MAX_NUM_OF_OPTIONS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned int i = 0; i < CMD_ARGC; ++i) {
		if (strcmp(CMD_ARGV[i], "-tap") == 0) {
			if (i + 1 >= CMD_ARGC || CMD_ARGV[i + 1][0] == '-') {
				command_print(CMD, "no TAP given");
				return ERROR_FAIL;
			}
			tap = jtag_tap_by_string(CMD_ARGV[i + 1]);
			if (!tap) {
				command_print(CMD, "Tap %s unknown", CMD_ARGV[i + 1]);
				return ERROR_FAIL;
			}
			++i;
		} else if (strcmp(CMD_ARGV[i], "-hub") == 0) {
			COMMAND_PARSE_ADDITIONAL_NUMBER(u32, i, user_instruction, "ir_value to select hub");
			hub_configured = true;
			COMMAND_PARSE_OPTIONAL_NUMBER(u8, i, data_register_length);
			if (data_register_length < IPDBG_MIN_DR_LENGTH ||
				data_register_length > IPDBG_MAX_DR_LENGTH) {
				command_print(CMD, "length of \"user\"-data register must be at least %d and at most %d.",
							IPDBG_MIN_DR_LENGTH, IPDBG_MAX_DR_LENGTH);
				return ERROR_FAIL;
			}
		} else if (strcmp(CMD_ARGV[i], "-vir") == 0) {
			COMMAND_PARSE_OPTIONAL_NUMBER(u32, i, virtual_ir_value);
			COMMAND_PARSE_OPTIONAL_NUMBER(u32, i, virtual_ir_length);
			COMMAND_PARSE_OPTIONAL_NUMBER(u32, i, virtual_ir_instruction);
			has_virtual_ir = true;
		} else if (strcmp(CMD_ARGV[i], "-port") == 0) {
			COMMAND_PARSE_ADDITIONAL_NUMBER(u16, i, port, "port number");
		} else if (strcmp(CMD_ARGV[i], "-tool") == 0) {
			COMMAND_PARSE_ADDITIONAL_NUMBER(u8, i, tool, "tool");
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

	if (tool >= ipdbg_max_tools_from_data_register_length(data_register_length)) {
		command_print(CMD, "Tool: %d is invalid", tool);
		return ERROR_FAIL;
	}

	if (has_virtual_ir) {
		virtual_ir = calloc(1, sizeof(struct ipdbg_virtual_ir_info));
		if (!virtual_ir) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
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
