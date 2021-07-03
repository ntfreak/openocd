/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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
#include <config.h>
#endif
#include <helper/log.h>

COMMAND_HANDLER(handle_foo_command)
{
	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

	const char *msg = "<unchanged>";
	if (CMD_ARGC == 2) {
		bool enable;
		COMMAND_PARSE_ENABLE(CMD_ARGV[1], enable);
		msg = enable ? "enable" : "disable";
	}

	LOG_INFO("%s: address=0x%8.8" PRIx32 " enabled=%s", CMD_NAME, address, msg);
	return ERROR_OK;
}

static bool foo_flag;

COMMAND_HANDLER(handle_flag_command)
{
	return CALL_COMMAND_HANDLER(handle_command_parse_bool,
		&foo_flag, "foo flag");
}

static const struct command_registration foo_command_handlers[] = {
	{
		.name = "bar",
		.handler = &handle_foo_command,
		.mode = COMMAND_ANY,
		.usage = "address ['enable'|'disable']",
		.help = "an example command",
	},
	{
		.name = "baz",
		.handler = &handle_foo_command,
		.mode = COMMAND_ANY,
		.usage = "address ['enable'|'disable']",
		.help = "a sample command",
	},
	{
		.name = "flag",
		.handler = &handle_flag_command,
		.mode = COMMAND_ANY,
		.usage = "[on|off]",
		.help = "set a flag",
	},
	COMMAND_REGISTRATION_DONE
};

static COMMAND_HELPER(handle_hello_args, const char **sep, const char **name)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (1 == CMD_ARGC) {
		*sep = " ";
		*name = CMD_ARGV[0];
	} else
		*sep = *name = "";

	return ERROR_OK;
}
COMMAND_HANDLER(handle_hello_command)
{
	const char *sep, *name;
	int retval = CALL_COMMAND_HANDLER(handle_hello_args, &sep, &name);
	if (retval == ERROR_OK)
		command_print(CMD, "Greetings%s%s!", sep, name);
	return retval;
}

const struct command_registration hello_command_handlers[] = {
	{
		.name = "hello",
		.handler = handle_hello_command,
		.mode = COMMAND_ANY,
		.help = "prints a warm welcome",
		.usage = "[name]",
	},
	{
		.name = "foo",
		.mode = COMMAND_ANY,
		.help = "example command handler skeleton",
		.chain = foo_command_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};
