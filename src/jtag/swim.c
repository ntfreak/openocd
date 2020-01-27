/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2020 by Antonio Borneo <borneo.antonio@gmail.com
 *
 * SWIM (Single Wire Interface Module) is a low-pin-count debug protocol
 * used by STMicroelectronics MCU family STM8 and documented in UM470
 * https://www.st.com/resource/en/user_manual/cd00173911.pdf
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "interface.h"
#include "swim.h"
#include <helper/command.h>
#include <transport/transport.h>

extern struct adapter_driver *adapter_driver;

int swim_system_reset(void)
{
	assert(adapter_driver->swim_ops);

	return adapter_driver->swim_ops->srst();
}

int swim_read_mem(uint32_t addr, uint32_t size, uint32_t count,
				  uint8_t *buffer)
{
	assert(adapter_driver->swim_ops);

	return adapter_driver->swim_ops->read_mem(addr, size, count, buffer);
}

int swim_write_mem(uint32_t addr, uint32_t size, uint32_t count,
				   const uint8_t *buffer)
{
	assert(adapter_driver->swim_ops);

	return adapter_driver->swim_ops->write_mem(addr, size, count, buffer);
}

int swim_reconnect(void)
{
	assert(adapter_driver->swim_ops);

	return adapter_driver->swim_ops->reconnect();
}

COMMAND_HANDLER(handle_swim_newtap_command)
{
	struct jtag_tap *tap;

	/*
	 * only need "basename" and "tap_type", but for backward compatibility
	 * ignore extra parameters
	 */
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	tap = calloc(1, sizeof(*tap));
	if (!tap) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	tap->chip = strdup(CMD_ARGV[0]);
	tap->tapname = strdup(CMD_ARGV[1]);
	tap->dotted_name = alloc_printf("%s.%s", CMD_ARGV[0], CMD_ARGV[1]);
	if (!tap->chip || !tap->tapname || !tap->dotted_name) {
		LOG_ERROR("Out of memory");
		free(tap->dotted_name);
		free(tap->tapname);
		free(tap->chip);
		free(tap);
		return ERROR_FAIL;
	}

	LOG_DEBUG("Creating new SWIM \"tap\", Chip: %s, Tap: %s, Dotted: %s",
			  tap->chip, tap->tapname, tap->dotted_name);

	/* default is enabled-after-reset */
	tap->enabled = true;

	jtag_tap_init(tap);
	return ERROR_OK;
}

static const struct command_registration swim_transport_subcommand_handlers[] = {
	{
		.name = "newtap",
		.handler = handle_swim_newtap_command,
		.mode = COMMAND_CONFIG,
		.help = "Create a new TAP instance named basename.tap_type, "
				"and appends it to the scan chain.",
		.usage = "basename tap_type",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration swim_transport_command_handlers[] = {
	{
		.name = "swim",
		.mode = COMMAND_ANY,
		.help = "perform swim adapter actions",
		.usage = "",
		.chain = swim_transport_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static int swim_transport_select(struct command_context *cmd_ctx)
{
	LOG_DEBUG(__func__);

	return register_commands(cmd_ctx, NULL, swim_transport_command_handlers);
}

static int swim_transport_init(struct command_context *cmd_ctx)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	LOG_DEBUG(__func__);

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			adapter_assert_reset();
		else
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
	} else
		adapter_deassert_reset();

	return ERROR_OK;
}

static struct transport swim_transport = {
	.name = "swim",
	.select = swim_transport_select,
	.init = swim_transport_init,
};

static void swim_constructor(void) __attribute__ ((constructor));
static void swim_constructor(void)
{
	transport_register(&swim_transport);
}

bool transport_is_swim(void)
{
	return get_current_transport() == &swim_transport;
}
