/*
 * Copyright (c) 2010 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/** @file
 * Infrastructure for specifying and managing the transport protocol
 * used in a given debug or programming session.
 *
 * Examples of "debug-capable" transports are JTAG or SWD.
 * Additionally, JTAG supports boundary scan testing.
 *
 * Examples of "programming-capable" transports include SPI or UART;
 * those are used (often mediated by a ROM bootloader) for ISP style
 * programming, to perform an initial load of code into flash, or
 * sometimes into SRAM.  Target code could use "variant" options to
 * decide how to use such protocols.  For example, Cortex-M3 cores
 * from TI/Luminary and from NXP use different protocols for for
 * UART or SPI based firmware loading.
 *
 * As a rule, there are protocols layered on top of the transport.
 * For example, different chip families use JTAG in different ways
 * for debugging.  Also, each family that supports programming over
 * a UART link for initial firmware loading tends to define its own
 * messaging and error handling.
 */

#include <helper/log.h>
#include <helper/replacements.h>
#include <transport/transport.h>

extern struct command_context *global_cmd_ctx;

/*-----------------------------------------------------------------------*/

/*
 * Infrastructure internals
 */

/** List of transports known to OpenOCD. */
static struct transport *transport_list;

/**
 * NULL-terminated Vector of names of transports which the
 * currently selected debug adapter supports.  This is declared
 * by the time that adapter is fully set up.
 */
static const char * const *allowed_transports;

/** * The transport being used for the current OpenOCD session.  */
static struct transport *session;

static int transport_select(struct command_context *ctx, const char *name)
{
	/* name may only identify a known transport;
	 * caller guarantees session's transport isn't yet set.*/
	for (struct transport *t = transport_list; t; t = t->next) {
		if (strcmp(t->name, name) == 0) {
			int retval = t->select(ctx);
			/* select() registers commands specific to this
			 * transport, and may also reset the link, e.g.
			 * forcing it to JTAG or SWD mode.
			 */
			if (retval == ERROR_OK)
				session = t;
			else
				LOG_ERROR("Error selecting '%s' as transport", t->name);
			return retval;
		}
	}

	LOG_ERROR("No transport named '%s' is available.", name);
	return ERROR_FAIL;
}

/**
 * Called by debug adapter drivers, or affiliated Tcl config scripts,
 * to declare the set of transports supported by an adapter.  When
 * there is only one member of that set, it is automatically selected.
 */
int allow_transports(struct command_context *ctx, const char * const *vector)
{
	/* NOTE:  caller is required to provide only a list
	 * of *valid* transport names
	 *
	 * REVISIT should we validate that?  and insist there's
	 * at least one non-NULL element in that list?
	 *
	 * ... allow removals, e.g. external strapping prevents use
	 * of one transport; C code should be definitive about what
	 * can be used when all goes well.
	 */
	if (allowed_transports || session) {
		LOG_ERROR("Can't modify the set of allowed transports.");
		return ERROR_FAIL;
	}

	allowed_transports = vector;

	/* autoselect if there's no choice ... */
	if (!vector[1]) {
		LOG_INFO("only one transport option; autoselect '%s'", vector[0]);
		return transport_select(ctx, vector[0]);
	}

	return ERROR_OK;
}

/**
 * Registers a transport.  There are general purpose transports
 * (such as JTAG), as well as relatively proprietary ones which are
 * specific to a given chip (or chip family).
 *
 * Code implementing a transport needs to register it before it can
 * be selected and then activated.  This is a dynamic process, so
 * that chips (and families) can define transports as needed (without
 * needing error-prone static tables).
 *
 * @param new_transport the transport being registered.  On a
 * successful return, this memory is owned by the transport framework.
 *
 * @returns ERROR_OK on success, else a fault code.
 */
int transport_register(struct transport *new_transport)
{
	struct transport *t;

	for (t = transport_list; t; t = t->next) {
		if (strcmp(t->name, new_transport->name) == 0) {
			LOG_ERROR("transport name already used");
			return ERROR_FAIL;
		}
	}

	if (!new_transport->select || !new_transport->init)
		LOG_ERROR("invalid transport %s", new_transport->name);

	/* splice this into the list */
	new_transport->next = transport_list;
	transport_list = new_transport;
	LOG_DEBUG("register '%s'", new_transport->name);

	return ERROR_OK;
}

/**
 * Returns the transport currently being used by this debug or
 * programming session.
 *
 * @returns handle to the read-only transport entity.
 */
struct transport *get_current_transport(void)
{
	/* REVISIT -- constify */
	return session;
}

/*-----------------------------------------------------------------------*/

/*
 * Infrastructure for Tcl interface to transports.
 */

/**
 * Makes and stores a copy of a set of transports passed as
 * parameters to a command.
 *
 * @param vector where the resulting copy is stored, as an argv-style
 *	NULL-terminated vector.
 */
COMMAND_HELPER(transport_list_parse, char ***vector)
{
	char **argv;
	unsigned n = CMD_ARGC;
	unsigned j = 0;

	*vector = NULL;

	if (n < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* our return vector must be NULL terminated */
	argv = calloc(n + 1, sizeof(char *));
	if (!argv)
		return ERROR_FAIL;

	for (unsigned i = 0; i < n; i++) {
		struct transport *t;

		for (t = transport_list; t; t = t->next) {
			if (strcmp(t->name, CMD_ARGV[i]) != 0)
				continue;
			argv[j++] = strdup(CMD_ARGV[i]);
			break;
		}
		if (!t) {
			LOG_ERROR("no such transport '%s'", CMD_ARGV[i]);
			goto fail;
		}
	}

	*vector = argv;
	return ERROR_OK;

fail:
	for (unsigned i = 0; i < n; i++)
		free(argv[i]);
	free(argv);
	return ERROR_FAIL;
}

COMMAND_HANDLER(handle_transport_init)
{
	LOG_DEBUG("%s", __func__);
	if (!session) {
		LOG_ERROR("session transport was not selected. Use 'transport select <transport>'");

		/* no session transport configured, print transports then fail */
		LOG_ERROR("Transports available:");
		const char * const *vector = allowed_transports;
		while (*vector) {
			LOG_ERROR("%s", *vector);
			vector++;
		}
		return ERROR_FAIL;
	}

	return session->init(CMD_CTX);
}

COMMAND_HANDLER(handle_transport_list)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "The following transports are available:");

	for (struct transport *t = transport_list; t; t = t->next)
		command_print(CMD, "\t%s", t->name);

	return ERROR_OK;
}

/**
 * Implements the Tcl "transport select" command, choosing the
 * transport to be used in this debug session from among the
 * set supported by the debug adapter being used.  Return value
 * is scriptable (allowing "if swd then..." etc).
 */
static int jim_transport_select(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	int res;
	switch (argc) {
		case 1:	/* autoselect if necessary, then return/display current config */
			if (!session) {
				if (!allowed_transports) {
					LOG_ERROR("Debug adapter does not support any transports? Check config file order.");
					return JIM_ERR;
				}
				LOG_INFO("auto-selecting first available session transport \"%s\". "
					 "To override use 'transport select <transport>'.", allowed_transports[0]);
				res = transport_select(global_cmd_ctx, allowed_transports[0]);
				if (res != JIM_OK)
					return res;
			}
			Jim_SetResultString(interp, session->name, -1);
			return JIM_OK;
		case 2:	/* assign */
			if (session) {
				if (!strcmp(session->name, argv[1]->bytes)) {
					LOG_WARNING("Transport \"%s\" was already selected", session->name);
					Jim_SetResultString(interp, session->name, -1);
					return JIM_OK;
				} else {
					LOG_ERROR("Can't change session's transport after the initial selection was made");
					return JIM_ERR;
				}
			}

			/* Is this transport supported by our debug adapter?
			 * Example, "JTAG-only" means SWD is not supported.
			 *
			 * NOTE:  requires adapter to have been set up, with
			 * transports declared via C.
			 */
			if (!allowed_transports) {
				LOG_ERROR("Debug adapter doesn't support any transports?");
				return JIM_ERR;
			}

			for (unsigned i = 0; allowed_transports[i]; i++) {

				if (strcmp(allowed_transports[i], argv[1]->bytes) == 0) {
					if (transport_select(global_cmd_ctx, argv[1]->bytes) == ERROR_OK) {
						Jim_SetResultString(interp, session->name, -1);
						return JIM_OK;
					}
					return JIM_ERR;
				}
			}

			LOG_ERROR("Debug adapter doesn't support '%s' transport", argv[1]->bytes);
			return JIM_ERR;
		default:
			Jim_WrongNumArgs(interp, 1, argv, "[too many parameters]");
			return JIM_ERR;
	}
}

static const struct command_registration transport_commands[] = {
	{
		.name = "init",
		.handler = handle_transport_init,
		/* this would be COMMAND_CONFIG ... except that
		 * it needs to trigger event handlers that may
		 * require COMMAND_EXEC ...
		 */
		.mode = COMMAND_ANY,
		.help = "Initialize this session's transport",
		.usage = ""
	},
	{
		.name = "list",
		.handler = handle_transport_list,
		.mode = COMMAND_ANY,
		.help = "list all built-in transports",
		.usage = ""
	},
	{
		.name = "select",
		.jim_handler = jim_transport_select,
		.mode = COMMAND_ANY,
		.help = "Select this session's transport",
		.usage = "[transport_name]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration transport_group[] = {
	{
		.name = "transport",
		.mode = COMMAND_ANY,
		.help = "Transport command group",
		.chain = transport_commands,
		.usage = ""
	},
	COMMAND_REGISTRATION_DONE
};

int transport_register_commands(struct command_context *ctx)
{
	return register_commands(ctx, NULL, transport_group);
}
