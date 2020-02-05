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

#include <jtag/interface.h>
#include <jtag/commands.h>
#include <transport/transport.h>
#include <target/target.h>
#include <jtag/aice/aice_transport.h>
#include "aice_usb.h"

#define AICE_KHZ_TO_SPEED_MAP_SIZE	16
static const int aice_khz_to_speed_map[AICE_KHZ_TO_SPEED_MAP_SIZE] = {
	30000,
	15000,
	7500,
	3750,
	1875,
	937,
	468,
	234,
	48000,
	24000,
	12000,
	6000,
	3000,
	1500,
	750,
	375,
};

static const struct aice_port *aice_port;
static struct aice_port_param_s param;
static uint32_t retry_times;
static uint32_t count_to_check_dbger;

/***************************************************************************/
/* External interface implementation */
static uint32_t aice_target_id_codes[AICE_MAX_NUM_CORE];
static uint8_t aice_num_of_target_id_codes;

/***************************************************************************/
/* AICE operations */
int aice_init_targets(void)
{
	int res;
	struct target *target;
	struct aice_port_s *aice;

	LOG_DEBUG("aice_init_targets");

	if (aice_num_of_target_id_codes == 0) {
		res = aice_port->api->idcode(aice_target_id_codes, &aice_num_of_target_id_codes);
		if (res != ERROR_OK) {
			LOG_ERROR("<-- TARGET ERROR! Failed to identify AndesCore "
					"JTAG Manufacture ID in the JTAG scan chain. "
					"Failed to access EDM registers. -->");
			return res;
		}
	}

	for (target = all_targets; target; target = target->next) {
		target->tap->idcode = aice_target_id_codes[target->tap->abs_chain_position];

		unsigned ii, limit = target->tap->expected_ids_cnt;
		int found = 0;

		for (ii = 0; ii < limit; ii++) {
			uint32_t expected = target->tap->expected_ids[ii];

			/* treat "-expected-id 0" as a "don't-warn" wildcard */
			if (!expected || (target->tap->idcode == expected)) {
				found = 1;
				break;
			}
		}

		if (found == 0) {
			LOG_ERROR
				("aice_init_targets: target not found: idcode: %" PRIx32,
				 target->tap->idcode);
			return ERROR_FAIL;
		}

		aice = calloc(1, sizeof(struct aice_port_s));
		aice->port = aice_port;
		aice->coreid = target->tap->abs_chain_position;

		target->tap->priv = aice;
		target->tap->hasidcode = 1;
	}

	return ERROR_OK;
}

/***************************************************************************/
/* End of External interface implementation */

/* initial aice
 * 1. open usb
 * 2. get/show version number
 * 3. reset
 */
static int aice_init(void)
{
	if (ERROR_OK != aice_port->api->open(&param)) {
		LOG_ERROR("Cannot find AICE Interface! Please check "
				"connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	aice_port->api->set_retry_times(retry_times);
	aice_port->api->set_count_to_check_dbger(count_to_check_dbger);

	LOG_INFO("AICE JTAG Interface ready");

	return ERROR_OK;
}

/* cleanup aice resource
 * close usb
 */
static int aice_quit(void)
{
	aice_port->api->close();
	return ERROR_OK;
}

static int aice_execute_reset(struct jtag_command *cmd)
{
	static int last_trst;
	int retval = ERROR_OK;

	LOG_DEBUG_IO("reset trst: %d", cmd->cmd.reset->trst);

	if (cmd->cmd.reset->trst != last_trst) {
		if (cmd->cmd.reset->trst)
			retval = aice_port->api->reset();

		last_trst = cmd->cmd.reset->trst;
	}

	return retval;
}

static int aice_execute_command(struct jtag_command *cmd)
{
	int retval;

	switch (cmd->type) {
		case JTAG_RESET:
			retval = aice_execute_reset(cmd);
			break;
		default:
			retval = ERROR_OK;
			break;
	}
	return retval;
}

/* aice has no need to implement jtag execution model
*/
static int aice_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int retval;

	retval = ERROR_OK;

	while (cmd) {
		if (aice_execute_command(cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;

		cmd = cmd->next;
	}

	return retval;
}

/* set jtag frequency(base frequency/frequency divider) to your jtag adapter */
static int aice_speed(int speed)
{
	return aice_port->api->set_jtag_clock(speed);
}

/* convert jtag adapter frequency(base frequency/frequency divider) to
 * human readable KHz value */
static int aice_speed_div(int speed, int *khz)
{
	*khz = aice_khz_to_speed_map[speed];

	return ERROR_OK;
}

/* convert human readable KHz value to jtag adapter frequency
 * (base frequency/frequency divider) */
static int aice_khz(int khz, int *jtag_speed)
{
	int i;
	for (i = 0 ; i < AICE_KHZ_TO_SPEED_MAP_SIZE ; i++) {
		if (khz == aice_khz_to_speed_map[i]) {
			if (8 <= i)
				*jtag_speed = i | AICE_TCK_CONTROL_TCK3048;
			else
				*jtag_speed = i;
			break;
		}
	}

	if (i == AICE_KHZ_TO_SPEED_MAP_SIZE) {
		LOG_INFO("No support the jtag clock: %d", khz);
		LOG_INFO("Supported jtag clocks are:");

		for (i = 0 ; i < AICE_KHZ_TO_SPEED_MAP_SIZE ; i++)
			LOG_INFO("* %d", aice_khz_to_speed_map[i]);

		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int aice_scan_jtag_chain(void)
{
	LOG_DEBUG("=== %s ===", __func__);
	uint8_t num_of_idcode = 0;
	struct target *target;

	int res = aice_port->api->idcode(aice_target_id_codes, &num_of_idcode);
	if (res != ERROR_OK) {
		LOG_ERROR("<-- TARGET ERROR! Failed to identify AndesCore "
					"JTAG Manufacture ID in the JTAG scan chain. "
					"Failed to access EDM registers. -->");
		return res;
	}

	for (uint32_t i = 0; i < num_of_idcode; i++)
		LOG_DEBUG("id_codes[%d] = 0x%x", i, aice_target_id_codes[i]);

	/* Update tap idcode */
	for (target = all_targets; target; target = target->next)
		target->tap->idcode = aice_target_id_codes[target->tap->abs_chain_position];

	return ERROR_OK;
}

/***************************************************************************/
/* Command handlers */
COMMAND_HANDLER(aice_handle_aice_info_command)
{
	LOG_DEBUG("aice_handle_aice_info_command");

	command_print(CMD, "Description: %s", param.device_desc);
	command_print(CMD, "Serial number: %s", param.serial);
	if (strncmp(aice_port->name, "aice_pipe", 9) == 0)
		command_print(CMD, "Adapter: %s", param.adapter_name);

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_port_command)
{
	LOG_DEBUG("aice_handle_aice_port_command");

	if (CMD_ARGC != 1) {
		LOG_ERROR("Need exactly one argument to 'aice port'");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (const struct aice_port *l = aice_port_get_list(); l->name; l++) {
		if (strcmp(l->name, CMD_ARGV[0]) == 0) {
			aice_port = l;
			return ERROR_OK;
		}
	}

	LOG_ERROR("No AICE port '%s' found", CMD_ARGV[0]);
	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_desc_command)
{
	LOG_DEBUG("aice_handle_aice_desc_command");

	if (CMD_ARGC == 1)
		param.device_desc = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("expected exactly one argument to aice desc <description>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_serial_command)
{
	LOG_DEBUG("aice_handle_aice_serial_command");

	if (CMD_ARGC == 1)
		param.serial = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("expected exactly one argument to aice serial <serial-number>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_vid_pid_command)
{
	LOG_DEBUG("aice_handle_aice_vid_pid_command");

	if (CMD_ARGC != 2) {
		LOG_WARNING("ignoring extra IDs in aice vid_pid (maximum is 1 pair)");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], param.vid);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], param.pid);

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_adapter_command)
{
	LOG_DEBUG("aice_handle_aice_adapter_command");

	if (CMD_ARGC == 1)
		param.adapter_name = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("expected exactly one argument to aice adapter <adapter-name>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_retry_times_command)
{
	LOG_DEBUG("aice_handle_aice_retry_times_command");

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], retry_times);
	else
		LOG_ERROR("expected exactly one argument to aice retry_times <num_of_retry>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_count_to_check_dbger_command)
{
	LOG_DEBUG("aice_handle_aice_count_to_check_dbger_command");

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], count_to_check_dbger);
	else
		LOG_ERROR("expected exactly one argument to aice count_to_check_dbger "
				"<count_of_checking>");

	return ERROR_OK;
}

COMMAND_HANDLER(aice_handle_aice_custom_srst_script_command)
{
	LOG_DEBUG("aice_handle_aice_custom_srst_script_command");

	if (CMD_ARGC > 0) {
		aice_port->api->set_custom_srst_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_custom_trst_script_command)
{
	LOG_DEBUG("aice_handle_aice_custom_trst_script_command");

	if (CMD_ARGC > 0) {
		aice_port->api->set_custom_trst_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_custom_restart_script_command)
{
	LOG_DEBUG("aice_handle_aice_custom_restart_script_command");

	if (CMD_ARGC > 0) {
		aice_port->api->set_custom_restart_script(CMD_ARGV[0]);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

COMMAND_HANDLER(aice_handle_aice_reset_command)
{
	LOG_DEBUG("aice_handle_aice_reset_command");

	return aice_port->api->reset();
}


static const struct command_registration aice_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &aice_handle_aice_info_command,
		.mode = COMMAND_EXEC,
		.help = "show aice info",
		.usage = "aice info",
	},
	{
		.name = "port",
		.handler = &aice_handle_aice_port_command,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the AICE",
		.usage = "aice port ['aice_pipe'|'aice_usb']",
	},
	{
		.name = "desc",
		.handler = &aice_handle_aice_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the aice device description",
		.usage = "aice desc [desciption string]",
	},
	{
		.name = "serial",
		.handler = &aice_handle_aice_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the AICE device",
		.usage = "aice serial [serial string]",
	},
	{
		.name = "vid_pid",
		.handler = &aice_handle_aice_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor and product ID of the AICE device",
		.usage = "aice vid_pid (vid pid)*",
	},
	{
		.name = "adapter",
		.handler = &aice_handle_aice_adapter_command,
		.mode = COMMAND_CONFIG,
		.help = "set the file name of adapter",
		.usage = "aice adapter [adapter name]",
	},
	{
		.name = "retry_times",
		.handler = &aice_handle_aice_retry_times_command,
		.mode = COMMAND_CONFIG,
		.help = "set retry times as AICE timeout",
		.usage = "aice retry_times num_of_retry",
	},
	{
		.name = "count_to_check_dbger",
		.handler = &aice_handle_aice_count_to_check_dbger_command,
		.mode = COMMAND_CONFIG,
		.help = "set retry times as checking $DBGER status",
		.usage = "aice count_to_check_dbger count_of_checking",
	},
	{
		.name = "custom_srst_script",
		.handler = &aice_handle_aice_custom_srst_script_command,
		.mode = COMMAND_CONFIG,
		.usage = "custom_srst_script script_file_name",
		.help = "set custom srst script",
	},
	{
		.name = "custom_trst_script",
		.handler = &aice_handle_aice_custom_trst_script_command,
		.mode = COMMAND_CONFIG,
		.usage = "custom_trst_script script_file_name",
		.help = "set custom trst script",
	},
	{
		.name = "custom_restart_script",
		.handler = &aice_handle_aice_custom_restart_script_command,
		.mode = COMMAND_CONFIG,
		.usage = "custom_restart_script script_file_name",
		.help = "set custom restart script",
	},
	{
		.name = "reset",
		.handler = &aice_handle_aice_reset_command,
		.mode = COMMAND_EXEC,
		.usage = "aice reset",
		.help = "reset AICE",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration aice_command_handlers[] = {
	{
		.name = "aice",
		.mode = COMMAND_ANY,
		.help = "perform aice management",
		.usage = "aice [subcommand]",
		.chain = aice_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
/***************************************************************************/
/* End of Command handlers */

static struct jtag_interface aice_interface = {
	.execute_queue = aice_execute_queue,
};

struct adapter_driver aice_adapter_driver = {
	.name = "aice",
	.transports = aice_transports,
	.commands = aice_command_handlers,

	.init = aice_init,
	.quit = aice_quit,
	.speed = aice_speed,		/* set interface speed */
	.khz = aice_khz,		/* convert khz to interface speed value */
	.speed_div = aice_speed_div,	/* return readable value */

	.jtag_ops = &aice_interface,
};
