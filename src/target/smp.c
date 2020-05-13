/***************************************************************************
 *                                                                         *
 * Copyright (C) ST-Ericsson SA 2011                                       *
 * Author: Michel Jaouen <michel.jaouen@stericsson.com> for ST-Ericsson.   *
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

#include "server/server.h"

#include "target/target.h"

#include "server/gdb_server.h"
#include "smp.h"
#include "helper/binarybuffer.h"

/*  implementation of new packet in gdb interface for smp feature          */
/*                                                                         */
/*   j : smp  status request                                               */
/*   J : smp  set request                                                  */
/*                                                                         */
/*   jc :read core id displayed by gdb connection                          */
/*   reply XXXXXXXX core id is int32_t , 8 hex digits                      */
/*                                                                         */
/*   Reply ENN error not supported (target not smp)                        */
/*                                                                         */
/*   JcXX  set core id displayed at next gdb continue                      */
/*   maximum 8 bytes described core id int32_t (8 hex digits)              */
/*  (core id -1 , reserved for returning to normal continue mode) */
/*  Reply ENN error not supported(target not smp,core id out of range)     */
/*  Reply OK : for success                                                 */
/*                                                                         */
/*  handling of this packet within gdb can be done by the creation         */
/*  internal variable by mean of function allocate_computed_value          */
/*  set $_core 1 => Jc01 packet is sent                                    */
/*  print $_core => jc packet is sent and result is affected in $          */
/*  Another way to test this packet is the usage of maintenance packet     */
/*  maint packet Jc01                                                      */
/*  maint packet jc                                                        */

/* packet j :smp status request */
int gdb_read_smp_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int retval = ERROR_OK;
	if (target->smp) {
		if (strncmp(packet, "jc", 2) == 0) {
			const uint32_t len = sizeof(target->gdb_service->core[0]);
			char hex_buffer[len * 2 + 1];
			uint8_t buffer[len];
			buf_set_u32(buffer, 0, len * 8, target->gdb_service->core[0]);
			size_t pkt_len = hexify(hex_buffer, buffer, sizeof(buffer),
				sizeof(hex_buffer));

			retval = gdb_put_packet(connection, hex_buffer, pkt_len);
		}
	} else
		retval = gdb_put_packet(connection, "E01", 3);
	return retval;
}

/* J :  smp set request */
int gdb_write_smp_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	int coreid = 0;
	int retval = ERROR_OK;

	/* skip command character */
	if (target->smp) {
		if (strncmp(packet, "Jc", 2) == 0) {
			packet += 2;
			coreid = strtoul(packet, &separator, 16);
			target->gdb_service->core[1] = coreid;
			retval = gdb_put_packet(connection, "OK", 2);
		}
	} else
		retval = gdb_put_packet(connection, "E01", 3);

	return retval;
}

COMMAND_HANDLER(default_handle_smp_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct target_list *head;

	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!CMD_ARGC) {
		command_print(CMD, "%s", target->smp ? "on" : "off");
		return ERROR_OK;
	}

	if (!strcmp(CMD_ARGV[0], "on")) {
		foreach_smp_target(head, target->head)
			head->target->smp = 1;

		return ERROR_OK;
	}

	if (!strcmp(CMD_ARGV[0], "off")) {
		foreach_smp_target(head, target->head)
			head->target->smp = 0;

		/* fixes the target display to the debugger */
		if (target->head)
			target->gdb_service->target = target;

		return ERROR_OK;
	}

	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(handle_smp_gdb_command)
{
	struct target *target = get_current_target(CMD_CTX);
	int retval = ERROR_OK;
	struct target_list *head;
	head = target->head;
	if (head != (struct target_list *)NULL) {
		if (CMD_ARGC == 1) {
			int coreid = 0;
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], coreid);
			if (ERROR_OK != retval)
				return retval;
			target->gdb_service->core[1] = coreid;

		}
		command_print(CMD, "gdb coreid  %" PRId32 " -> %" PRId32, target->gdb_service->core[0]
			, target->gdb_service->core[1]);
	}
	return ERROR_OK;
}

const struct command_registration smp_command_handlers[] = {
	{
		.name = "smp",
		.handler = default_handle_smp_command,
		.mode = COMMAND_EXEC,
		.help = "smp handling",
		.usage = "[on|off]",
	},
	{
		.name = "smp_gdb",
		.handler = handle_smp_gdb_command,
		.mode = COMMAND_EXEC,
		.help = "display/fix current core played to gdb",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};
