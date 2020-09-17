/*
 * Copyright (C) 2019-2020 by Marc Schink <dev@zapb.de>
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

#include <helper/log.h>
#include <target/rtt.h>

#include "rtt.h"

#define CHANNEL_NAME_SIZE	128

COMMAND_HANDLER(handle_rtt_setup_command)
{
struct rtt_source source;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	source.find_cb = &target_rtt_find_control_block;
	source.read_cb = &target_rtt_read_control_block;
	source.start = &target_rtt_start;
	source.stop = &target_rtt_stop;
	source.read = &target_rtt_read_callback;
	source.write = &target_rtt_write_callback;
	source.read_channel_info = &target_rtt_read_channel_info;

	target_addr_t address;
	uint32_t size;

	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);

	rtt_register_source(source, get_current_target(CMD_CTX));

	if (rtt_setup(address, size, CMD_ARGV[2]) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_rtt_start_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!rtt_configured()) {
		command_print(CMD, "RTT is not configured");
		return ERROR_FAIL;
	}

	return rtt_start();
}

COMMAND_HANDLER(handle_rtt_stop_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return rtt_stop();
}

COMMAND_HANDLER(handle_rtt_polling_interval_command)
{
	if (CMD_ARGC == 0) {
		int ret;
		unsigned int interval;

		ret = rtt_get_polling_interval(&interval);

		if (ret != ERROR_OK) {
			command_print(CMD, "Failed to get polling interval");
			return ret;
		}

		command_print(CMD, "%u ms", interval);
	} else if (CMD_ARGC == 1) {
		int ret;
		unsigned int interval;

		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], interval);
		ret = rtt_set_polling_interval(interval);

		if (ret != ERROR_OK) {
			command_print(CMD, "Failed to set polling interval");
			return ret;
		}
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_rtt_channels_command)
{
	int ret;
	char channel_name[CHANNEL_NAME_SIZE];
	const struct rtt_control *ctrl;
	struct rtt_channel_info info;

	if (!rtt_found_cb()) {
		command_print(CMD, "rtt: Control block not available");
		return ERROR_FAIL;
	}

	ctrl = rtt_get_control();

	command_print(CMD, "Channels: up=%u, down=%u", ctrl->num_up_channels,
		ctrl->num_down_channels);

	command_print(CMD, "Up-channels:");

	info.name = channel_name;
	info.name_length = sizeof(channel_name);

	for (unsigned int i = 0; i < ctrl->num_up_channels; i++) {
		ret = rtt_read_channel_info(i, RTT_CHANNEL_TYPE_UP, &info);

		if (ret != ERROR_OK)
			return ret;

		if (!info.size)
			continue;

		command_print(CMD, "%u: %s %u %u", i, info.name, info.size,
			info.flags);
	}

	command_print(CMD, "Down-channels:");

	for (unsigned int i = 0; i < ctrl->num_down_channels; i++) {
		ret = rtt_read_channel_info(i, RTT_CHANNEL_TYPE_DOWN, &info);

		if (ret != ERROR_OK)
			return ret;

		if (!info.size)
			continue;

		command_print(CMD, "%u: %s %u %u", i, info.name, info.size,
			info.flags);
	}

	return ERROR_OK;
}

static int jim_channel_list(Jim_Interp *interp, int argc,
	Jim_Obj * const *argv)
{
	Jim_Obj *list;
	Jim_Obj *channel_list;
	char channel_name[CHANNEL_NAME_SIZE];
	const struct rtt_control *ctrl;
	struct rtt_channel_info info;

	if (!rtt_found_cb()) {
		Jim_SetResultFormatted(interp, "rtt: Control block not available");
		return ERROR_FAIL;
	}

	ctrl = rtt_get_control();

	info.name = channel_name;
	info.name_length = sizeof(channel_name);

	list = Jim_NewListObj(interp, NULL, 0);
	channel_list = Jim_NewListObj(interp, NULL, 0);

	for (unsigned int i = 0; i < ctrl->num_up_channels; i++) {
		int ret;
		Jim_Obj *tmp;

		ret = rtt_read_channel_info(i, RTT_CHANNEL_TYPE_UP, &info);

		if (ret != ERROR_OK)
			return ret;

		if (!info.size)
			continue;

		tmp = Jim_NewListObj(interp, NULL, 0);

		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			"name", -1));
		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			info.name, -1));

		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			"size", -1));
		Jim_ListAppendElement(interp, tmp, Jim_NewIntObj(interp,
			info.size));

		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			"flags", -1));
		Jim_ListAppendElement(interp, tmp, Jim_NewIntObj(interp,
			info.flags));

		Jim_ListAppendElement(interp, channel_list, tmp);
	}

	Jim_ListAppendElement(interp, list, channel_list);

	channel_list = Jim_NewListObj(interp, NULL, 0);

	for (unsigned int i = 0; i < ctrl->num_down_channels; i++) {
		int ret;
		Jim_Obj *tmp;

		ret = rtt_read_channel_info(i, RTT_CHANNEL_TYPE_DOWN, &info);

		if (ret != ERROR_OK)
			return ret;

		if (!info.size)
			continue;

		tmp = Jim_NewListObj(interp, NULL, 0);

		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			"name", -1));
		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			info.name, -1));

		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			"size", -1));
		Jim_ListAppendElement(interp, tmp, Jim_NewIntObj(interp,
			info.size));

		Jim_ListAppendElement(interp, tmp, Jim_NewStringObj(interp,
			"flags", -1));
		Jim_ListAppendElement(interp, tmp, Jim_NewIntObj(interp,
			info.flags));

		Jim_ListAppendElement(interp, channel_list, tmp);
	}

	Jim_ListAppendElement(interp, list, channel_list);
	Jim_SetResult(interp, list);

	return JIM_OK;
}

static const struct command_registration rtt_subcommand_handlers[] = {
	{
		.name = "setup",
		.handler = handle_rtt_setup_command,
		.mode = COMMAND_ANY,
		.help = "setup RTT",
		.usage = "<address> <size> <ID>"
	},
	{
		.name = "start",
		.handler = handle_rtt_start_command,
		.mode = COMMAND_EXEC,
		.help = "start RTT",
		.usage = ""
	},
	{
		.name = "stop",
		.handler = handle_rtt_stop_command,
		.mode = COMMAND_EXEC,
		.help = "stop RTT",
		.usage = ""
	},
	{
		.name = "polling_interval",
		.handler = handle_rtt_polling_interval_command,
		.mode = COMMAND_EXEC,
		.help = "show or set polling interval in ms",
		.usage = "[interval]"
	},
	{
		.name = "channels",
		.handler = handle_rtt_channels_command,
		.mode = COMMAND_EXEC,
		.help = "list available channels",
		.usage = ""
	},
	{
		.name = "channellist",
		.jim_handler = jim_channel_list,
		.mode = COMMAND_EXEC,
		.help = "list available channels",
		.usage = ""
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration rtt_target_command_handlers[] = {
	{
		.name = "rtt",
		.mode = COMMAND_EXEC,
		.help = "RTT target commands",
		.usage = "",
		.chain = rtt_subcommand_handlers
	},
	COMMAND_REGISTRATION_DONE
};
