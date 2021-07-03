/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "pld.h"
#include <helper/log.h>
#include <helper/replacements.h>
#include <helper/time_support.h>


/* pld drivers
 */
extern struct pld_driver virtex2_pld;

static struct pld_driver *pld_drivers[] = {
	&virtex2_pld,
	NULL,
};

static struct pld_device *pld_devices;

struct pld_device *get_pld_device_by_num(int num)
{
	struct pld_device *p;
	int i = 0;

	for (p = pld_devices; p; p = p->next) {
		if (i++ == num)
			return p;
	}

	return NULL;
}

/* pld device <driver> [driver_options ...]
 */
COMMAND_HANDLER(handle_pld_device_command)
{
	int i;
	int found = 0;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (i = 0; pld_drivers[i]; i++) {
		if (strcmp(CMD_ARGV[0], pld_drivers[i]->name) == 0) {
			struct pld_device *p, *c;

			/* register pld specific commands */
			int retval;
			if (pld_drivers[i]->commands) {
				retval = register_commands(CMD_CTX, NULL,
						pld_drivers[i]->commands);
				if (retval != ERROR_OK) {
					LOG_ERROR("couldn't register '%s' commands", CMD_ARGV[0]);
					return ERROR_FAIL;
				}
			}

			c = malloc(sizeof(struct pld_device));
			c->driver = pld_drivers[i];
			c->next = NULL;

			retval = CALL_COMMAND_HANDLER(
					pld_drivers[i]->pld_device_command, c);
			if (retval != ERROR_OK) {
				LOG_ERROR("'%s' driver rejected pld device",
					CMD_ARGV[0]);
				free(c);
				return ERROR_OK;
			}

			/* put pld device in linked list */
			if (pld_devices) {
				/* find last pld device */
				for (p = pld_devices; p && p->next; p = p->next)
					;
				if (p)
					p->next = c;
			} else
				pld_devices = c;

			found = 1;
		}
	}

	/* no matching pld driver found */
	if (!found) {
		LOG_ERROR("pld driver '%s' not found", CMD_ARGV[0]);
		exit(-1);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_pld_devices_command)
{
	struct pld_device *p;
	int i = 0;

	if (!pld_devices) {
		command_print(CMD, "no pld devices configured");
		return ERROR_OK;
	}

	for (p = pld_devices; p; p = p->next)
		command_print(CMD, "#%i: %s", i++, p->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_pld_load_command)
{
	int retval;
	struct timeval start, end, duration;
	struct pld_device *p;

	gettimeofday(&start, NULL);

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned dev_id;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	p = get_pld_device_by_num(dev_id);
	if (!p) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	retval = p->driver->load(p, CMD_ARGV[1]);
	if (retval != ERROR_OK) {
		command_print(CMD, "failed loading file %s to pld device %u",
			CMD_ARGV[1], dev_id);
		switch (retval) {
		}
		return retval;
	} else {
		gettimeofday(&end, NULL);
		timeval_subtract(&duration, &end, &start);

		command_print(CMD, "loaded file %s to pld device %u in %jis %jius",
			CMD_ARGV[1], dev_id,
			(intmax_t)duration.tv_sec, (intmax_t)duration.tv_usec);
	}

	return ERROR_OK;
}

static const struct command_registration pld_exec_command_handlers[] = {
	{
		.name = "devices",
		.handler = handle_pld_devices_command,
		.mode = COMMAND_EXEC,
		.help = "list configured pld devices",
		.usage = "",
	},
	{
		.name = "load",
		.handler = handle_pld_load_command,
		.mode = COMMAND_EXEC,
		.help = "load configuration file into PLD",
		.usage = "pld_num filename",
	},
	COMMAND_REGISTRATION_DONE
};

static int pld_init(struct command_context *cmd_ctx)
{
	if (!pld_devices)
		return ERROR_OK;

	return register_commands(cmd_ctx, "pld", pld_exec_command_handlers);
}

COMMAND_HANDLER(handle_pld_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool pld_initialized;
	if (pld_initialized) {
		LOG_INFO("'pld init' has already been called");
		return ERROR_OK;
	}
	pld_initialized = true;

	LOG_DEBUG("Initializing PLDs...");
	return pld_init(CMD_CTX);
}

static const struct command_registration pld_config_command_handlers[] = {
	{
		.name = "device",
		.mode = COMMAND_CONFIG,
		.handler = handle_pld_device_command,
		.help = "configure a PLD device",
		.usage = "driver_name [driver_args ... ]",
	},
	{
		.name = "init",
		.mode = COMMAND_CONFIG,
		.handler = handle_pld_init_command,
		.help = "initialize PLD devices",
		.usage = ""
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration pld_command_handler[] = {
	{
		.name = "pld",
		.mode = COMMAND_ANY,
		.help = "programmable logic device commands",
		.usage = "",
		.chain = pld_config_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
int pld_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, pld_command_handler);
}
