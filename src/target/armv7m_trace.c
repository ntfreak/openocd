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

	armv7m->trace_config.itm_deferred_config = true;
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

	armv7m->trace_config.itm_deferred_config = true;
	return ERROR_OK;
}

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
		.name = "itm",
		.mode = COMMAND_ANY,
		.help = "itm command group",
		.usage = "",
		.chain = itm_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
