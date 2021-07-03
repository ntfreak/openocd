/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/tcl.h>
#include <transport/transport.h>
#include <target/target.h>
#include <jtag/aice/aice_interface.h>
#include <jtag/aice/aice_transport.h>
#include <string.h>

/* */
static int jim_newtap_expected_id(struct jim_nvp *n, struct jim_getopt_info *goi,
		struct jtag_tap *tap)
{
	jim_wide w;
	int e = jim_getopt_wide(goi, &w);
	if (e != JIM_OK) {
		Jim_SetResultFormatted(goi->interp, "option: %s bad parameter",
				n->name);
		return e;
	}

	unsigned expected_len = sizeof(uint32_t) * tap->expected_ids_cnt;
	uint32_t *new_expected_ids = malloc(expected_len + sizeof(uint32_t));
	if (!new_expected_ids) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	assert(tap->expected_ids);
	memcpy(new_expected_ids, tap->expected_ids, expected_len);

	new_expected_ids[tap->expected_ids_cnt] = w;

	free(tap->expected_ids);
	tap->expected_ids = new_expected_ids;
	tap->expected_ids_cnt++;

	return JIM_OK;
}

#define NTAP_OPT_EXPECTED_ID 0

/* */
static int jim_aice_newtap_cmd(struct jim_getopt_info *goi)
{
	struct jtag_tap *tap;
	int x;
	int e;
	struct jim_nvp *n;
	char *cp;
	const struct jim_nvp opts[] = {
		{.name = "-expected-id", .value = NTAP_OPT_EXPECTED_ID},
		{.name = NULL, .value = -1},
	};

	tap = calloc(1, sizeof(struct jtag_tap));
	if (!tap) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	/*
	 * we expect CHIP + TAP + OPTIONS
	 * */
	if (goi->argc < 3) {
		Jim_SetResultFormatted(goi->interp,
				"Missing CHIP TAP OPTIONS ....");
		free(tap);
		return JIM_ERR;
	}

	const char *tmp;
	jim_getopt_string(goi, &tmp, NULL);
	tap->chip = strdup(tmp);

	jim_getopt_string(goi, &tmp, NULL);
	tap->tapname = strdup(tmp);

	/* name + dot + name + null */
	x = strlen(tap->chip) + 1 + strlen(tap->tapname) + 1;
	cp = malloc(x);
	sprintf(cp, "%s.%s", tap->chip, tap->tapname);
	tap->dotted_name = cp;

	LOG_DEBUG("Creating New Tap, Chip: %s, Tap: %s, Dotted: %s, %d params",
			tap->chip, tap->tapname, tap->dotted_name, goi->argc);

	while (goi->argc) {
		e = jim_getopt_nvp(goi, opts, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(goi, opts, 0);
			free(cp);
			free(tap);
			return e;
		}
		LOG_DEBUG("Processing option: %s", n->name);
		switch (n->value) {
			case NTAP_OPT_EXPECTED_ID:
				e = jim_newtap_expected_id(n, goi, tap);
				if (e != JIM_OK) {
					free(cp);
					free(tap);
					return e;
				}
				break;
		}		/* switch (n->value) */
	}			/* while (goi->argc) */

	/* default is enabled-after-reset */
	tap->enabled = !tap->disabled_after_reset;

	jtag_tap_init(tap);
	return JIM_OK;
}

/* */
static int jim_aice_newtap(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc - 1, argv + 1);
	return jim_aice_newtap_cmd(&goi);
}

/* */
COMMAND_HANDLER(handle_aice_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool jtag_initialized;
	if (jtag_initialized) {
		LOG_INFO("'jtag init' has already been called");
		return ERROR_OK;
	}
	jtag_initialized = true;

	LOG_DEBUG("Initializing jtag devices...");
	return jtag_init(CMD_CTX);
}

COMMAND_HANDLER(handle_scan_chain_command)
{
	struct jtag_tap *tap;
	char expected_id[12];

	aice_scan_jtag_chain();
	tap = jtag_all_taps();
	command_print(CMD,
		"   TapName             Enabled  IdCode     Expected   IrLen IrCap IrMask");
	command_print(CMD,
		"-- ------------------- -------- ---------- ---------- ----- ----- ------");

	while (tap) {
		uint32_t expected, expected_mask, ii;

		snprintf(expected_id, sizeof(expected_id), "0x%08x",
			(unsigned)((tap->expected_ids_cnt > 0)
				   ? tap->expected_ids[0]
				   : 0));
		if (tap->ignore_version)
			expected_id[2] = '*';

		expected = buf_get_u32(tap->expected, 0, tap->ir_length);
		expected_mask = buf_get_u32(tap->expected_mask, 0, tap->ir_length);

		command_print(CMD,
			"%2d %-18s     %c     0x%08x %s %5d 0x%02x  0x%02x",
			tap->abs_chain_position,
			tap->dotted_name,
			tap->enabled ? 'Y' : 'n',
			(unsigned int)(tap->idcode),
			expected_id,
			(unsigned int)(tap->ir_length),
			(unsigned int)(expected),
			(unsigned int)(expected_mask));

		for (ii = 1; ii < tap->expected_ids_cnt; ii++) {
			snprintf(expected_id, sizeof(expected_id), "0x%08x",
				(unsigned) tap->expected_ids[ii]);
			if (tap->ignore_version)
				expected_id[2] = '*';

			command_print(CMD,
				"                                           %s",
				expected_id);
		}

		tap = tap->next_tap;
	}

	return ERROR_OK;
}

static int jim_aice_arp_init(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	LOG_DEBUG("No implement: jim_aice_arp_init");

	return JIM_OK;
}

/* */
static int aice_init_reset(struct command_context *cmd_ctx)
{
	LOG_DEBUG("Initializing with hard TRST+SRST reset");

	int retval;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	jtag_add_reset(1, 0);	/* TAP_RESET */
	if (jtag_reset_config & RESET_HAS_SRST) {
		jtag_add_reset(1, 1);
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) == 0)
			jtag_add_reset(0, 1);
	}
	jtag_add_reset(0, 0);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/* */
static int jim_aice_arp_init_reset(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	int e = ERROR_OK;
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc - 1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv - 1, "(no params)");
		return JIM_ERR;
	}
	struct command_context *context = current_command_context(interp);
	e = aice_init_reset(context);

	if (e != ERROR_OK) {
		Jim_Obj *obj = Jim_NewIntObj(goi.interp, e);
		Jim_SetResultFormatted(goi.interp, "error: %#s", obj);
		return JIM_ERR;
	}
	return JIM_OK;
}

static int jim_aice_names(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc - 1, argv + 1);
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
		return JIM_ERR;
	}
	Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
	struct jtag_tap *tap;

	for (tap = jtag_all_taps(); tap; tap = tap->next_tap)
		Jim_ListAppendElement(goi.interp,
				Jim_GetResult(goi.interp),
				Jim_NewStringObj(goi.interp,
					tap->dotted_name, -1));

	return JIM_OK;
}

/* */
static const struct command_registration
aice_transport_jtag_subcommand_handlers[] = {
	{
		.name = "init",
		.mode = COMMAND_ANY,
		.handler = handle_aice_init_command,
		.help = "initialize jtag scan chain",
		.usage = ""
	},
	{
		.name = "arp_init",
		.mode = COMMAND_ANY,
		.jim_handler = jim_aice_arp_init,
		.help = "Validates JTAG scan chain against the list of "
			"declared TAPs.",
	},
	{
		.name = "arp_init-reset",
		.mode = COMMAND_ANY,
		.jim_handler = jim_aice_arp_init_reset,
		.help = "Uses TRST and SRST to try resetting everything on "
			"the JTAG scan chain, then performs 'jtag arp_init'."
	},
	{
		.name = "newtap",
		.mode = COMMAND_CONFIG,
		.jim_handler = jim_aice_newtap,
		.help = "Create a new TAP instance named basename.tap_type, "
			"and appends it to the scan chain.",
		.usage = "basename tap_type ['-expected_id' number]"
	},
	{
		.name = "tapisenabled",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
		.help = "Returns a Tcl boolean (0/1) indicating whether "
			"the TAP is enabled (1) or not (0).",
		.usage = "tap_name",
	},
	{
		.name = "tapenable",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
		.help = "Try to enable the specified TAP using the "
			"'tap-enable' TAP event.",
		.usage = "tap_name",
	},
	{
		.name = "tapdisable",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
		.help = "Try to disable the specified TAP using the "
			"'tap-disable' TAP event.",
		.usage = "tap_name",
	},
	{
		.name = "configure",
		.mode = COMMAND_ANY,
		.jim_handler = jim_jtag_configure,
		.help = "Provide a Tcl handler for the specified "
			"TAP event.",
		.usage = "tap_name '-event' event_name handler",
	},
	{
		.name = "cget",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_configure,
		.help = "Return any Tcl handler for the specified "
			"TAP event.",
		.usage = "tap_name '-event' event_name",
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.jim_handler = jim_aice_names,
		.help = "Returns list of all JTAG tap names.",
	},
	{
		.name = "scan_chain",
		.handler = handle_scan_chain_command,
		.mode = COMMAND_ANY,
		.help = "print current scan chain configuration",
		.usage = ""
	},

	COMMAND_REGISTRATION_DONE
};

/* */
static const struct command_registration aice_transport_command_handlers[] = {
	{
		.name = "jtag",
		.mode = COMMAND_ANY,
		.usage = "",
		.chain = aice_transport_jtag_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE

};

/* */
static int aice_transport_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL,
			aice_transport_command_handlers);
}

/* */
static int aice_transport_init(struct command_context *cmd_ctx)
{
	LOG_DEBUG("aice_transport_init");
	struct target *t = get_current_target(cmd_ctx);
	struct transport *transport;

	if (!t) {
		LOG_ERROR("no current target");
		return ERROR_FAIL;
	}

	transport = get_current_transport();

	if (!transport) {
		LOG_ERROR("no transport selected");
		return ERROR_FAIL;
	}

	LOG_DEBUG("current transport %s", transport->name);

	return aice_init_targets();
}

/* */
static int aice_transport_select(struct command_context *ctx)
{
	LOG_DEBUG("aice_transport_select");

	int retval;

	retval = aice_transport_register_commands(ctx);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static struct transport aice_jtag_transport = {
	.name = "aice_jtag",
	.select = aice_transport_select,
	.init = aice_transport_init,
};

const char *aice_transports[] = { "aice_jtag", NULL };

static void aice_constructor(void) __attribute__((constructor));
static void aice_constructor(void)
{
	transport_register(&aice_jtag_transport);
}
