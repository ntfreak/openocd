/*
 * Copyright (C) 2019, STMicroelectronics - All Rights Reserved
 * Author(s): Antonio Borneo <borneo.antonio@gmail.com> for STMicroelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file
 * Utilities to support in-circuit debuggers that provide APIs to access
 * directly ARM DAP, hiding the access to the underlining transport used
 * for the physical connection (either JTAG or SWD).
 * E.g. STMicroelectronics ST-Link/V2 (from version V2J24) and STLINK-V3.
 *
 * Single-DAP support only.
 *
 * For details, see "ARM IHI 0031A"
 * ARM Debug Interface v5 Architecture Specification
 *
 * FIXME: in JTAG mode, trst is not managed
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/tcl.h>
#include <transport/transport.h>

COMMAND_HANDLER(dapdirect_jtag_empty_command)
{
	LOG_DEBUG("dapdirect_jtag_empty_command(\"%s\")", CMD_NAME);

	return ERROR_OK;
}

COMMAND_HANDLER(dapdirect_jtag_reset_command)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	/*
	 * in case the adapter has not already handled asserting srst
	 * we will attempt it again
	 */
	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING) {
			adapter_assert_reset();
			return ERROR_OK;
		}
		LOG_WARNING("\'srst_nogate\' reset_config option is required");
	}
	adapter_deassert_reset();
	return ERROR_OK;
}

static const struct command_registration dapdirect_jtag_subcommand_handlers[] = {
	{
		.name = "newtap",
		.mode = COMMAND_CONFIG,
		.jim_handler = jim_jtag_newtap,
		.help = "declare a new TAP"
	},
	{
		.name = "init",
		.mode = COMMAND_ANY,
		.handler = dapdirect_jtag_empty_command,
		.usage = ""
	},
	{
		.name = "arp_init",
		.mode = COMMAND_ANY,
		.handler = dapdirect_jtag_empty_command,
		.usage = ""
	},
	{
		.name = "arp_init-reset",
		.mode = COMMAND_ANY,
		.handler = dapdirect_jtag_reset_command,
		.usage = ""
	},
	{
		.name = "tapisenabled",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
	},
	{
		.name = "tapenable",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
	},
	{
		.name = "tapdisable",
		.mode = COMMAND_EXEC,
		.handler = dapdirect_jtag_empty_command,
		.usage = "",
	},
	{
		.name = "configure",
		.mode = COMMAND_ANY,
		.handler = dapdirect_jtag_empty_command,
		.usage = "",
	},
	{
		.name = "cget",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_configure,
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.handler = dapdirect_jtag_empty_command,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dapdirect_jtag_handlers[] = {
	{
		.name = "jtag",
		.mode = COMMAND_ANY,
		.chain = dapdirect_jtag_subcommand_handlers,
		.usage = "",
	},
	{
		.name = "jtag_ntrst_delay",
		.mode = COMMAND_ANY,
		.handler = dapdirect_jtag_empty_command,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dapdirect_swd_subcommand_handlers[] = {
	{
		.name = "newdap",
		.mode = COMMAND_CONFIG,
		.jim_handler = jim_jtag_newtap,
		.help = "declare a new SWD DAP",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dapdirect_swd_handlers[] = {
	{
		.name = "swd",
		.mode = COMMAND_ANY,
		.help = "SWD command group",
		.usage = "",
		.chain = dapdirect_swd_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static int dapdirect_jtag_select(struct command_context *ctx)
{
	LOG_DEBUG("dapdirect_jtag_select()");

	return register_commands(ctx, NULL, dapdirect_jtag_handlers);
}

static int dapdirect_swd_select(struct command_context *ctx)
{
	LOG_DEBUG("dapdirect_swd_select()");

	return register_commands(ctx, NULL, dapdirect_swd_handlers);
}

static int dapdirect_init(struct command_context *ctx)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	LOG_DEBUG("dapdirect_init()");

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			adapter_assert_reset();
		else
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
	} else
		adapter_deassert_reset();

	return ERROR_OK;
}

static struct transport dapdirect_jtag_transport = {
	.name = "dapdirect_jtag",
	.select = dapdirect_jtag_select,
	.init = dapdirect_init,
};

static struct transport dapdirect_swd_transport = {
	.name = "dapdirect_swd",
	.select = dapdirect_swd_select,
	.init = dapdirect_init,
};

static void dapdirect_constructor(void) __attribute__((constructor));
static void dapdirect_constructor(void)
{
	transport_register(&dapdirect_jtag_transport);
	transport_register(&dapdirect_swd_transport);
}

/**
 * Returns true if the current debug session
 * is using JTAG as its transport.
 */
bool transport_is_dapdirect_jtag(void)
{
	return get_current_transport() == &dapdirect_jtag_transport;
}

/**
 * Returns true if the current debug session
 * is using SWD as its transport.
 */
bool transport_is_dapdirect_swd(void)
{
	return get_current_transport() == &dapdirect_swd_transport;
}
