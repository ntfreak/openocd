/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *                                                                         *
 *   Copyright (C) 2020-2021, Ampere Computing LLC                         *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdint.h>
#include "target/arm_adi_v5.h"
#include "target/arm.h"
#include "helper/list.h"
#include "helper/command.h"
#include "transport/transport.h"
#include "jtag/interface.h"

static LIST_HEAD(all_dap);

extern const struct dap_ops swd_dap_ops;
extern const struct dap_ops jtag_dp_ops;
extern struct adapter_driver *adapter_driver;

/* DAP command support */
struct arm_dap_object {
	struct list_head lh;
	struct adiv5_dap dap;
	char *name;
	const struct swd_driver *swd;
};

static void dap_instance_init(struct adiv5_dap *dap)
{
	int i;
	/* Set up with safe defaults */
	for (i = 0; i <= DP_APSEL_MAX; i++) {
		dap->ap[i].dap = dap;
		dap->ap[i].ap_num = i;
		/* by default init base address used for adiv6 at 16-bit granularity */
		dap->ap[i].base_addr = i << 16;
		/* memaccess_tck max is 255 */
		dap->ap[i].memaccess_tck = 255;
		/* Number of bits for tar autoincrement, impl. dep. at least 10 */
		dap->ap[i].tar_autoincr_block = (1<<10);
		/* default CSW value */
		if (!dap->ap[i].csw_default)
			dap->ap[i].csw_default = CSW_AHB_DEFAULT;
		dap->ap[i].cfg_reg = ADI_BAD_CFG; /* mem_ap configuration reg (large physical addr, etc. */
	}
	INIT_LIST_HEAD(&dap->cmd_journal);
	INIT_LIST_HEAD(&dap->cmd_pool);
}

const char *adiv5_dap_name(struct adiv5_dap *self)
{
	struct arm_dap_object *obj = container_of(self, struct arm_dap_object, dap);
	return obj->name;
}

const struct swd_driver *adiv5_dap_swd_driver(struct adiv5_dap *self)
{
	struct arm_dap_object *obj = container_of(self, struct arm_dap_object, dap);
	return obj->swd;
}

struct adiv5_dap *adiv5_get_dap(struct arm_dap_object *obj)
{
	return &obj->dap;
}
struct adiv5_dap *dap_instance_by_jim_obj(Jim_Interp *interp, Jim_Obj *o)
{
	struct arm_dap_object *obj = NULL;
	const char *name;
	bool found = false;

	name = Jim_GetString(o, NULL);

	list_for_each_entry(obj, &all_dap, lh) {
		if (!strcmp(name, obj->name)) {
			found = true;
			break;
		}
	}

	if (found)
		return &obj->dap;
	return NULL;
}

static int dap_init_all(void)
{
	struct arm_dap_object *obj;
	int retval;
	uint32_t dpidr;

	LOG_DEBUG("Initializing all DAPs ...");

	list_for_each_entry(obj, &all_dap, lh) {
		struct adiv5_dap *dap = &obj->dap;

		/* with hla, dap is just a dummy */
		if (transport_is_hla())
			continue;

		/* skip taps that are disabled */
		if (!dap->tap->enabled)
			continue;

		if (transport_is_swd()) {
			dap->ops = &swd_dap_ops;
			obj->swd = adapter_driver->swd_ops;
		} else if (transport_is_dapdirect_swd()) {
			dap->ops = adapter_driver->dap_swd_ops;
		} else if (transport_is_dapdirect_jtag()) {
			dap->ops = adapter_driver->dap_jtag_ops;
		} else
			dap->ops = &jtag_dp_ops;

		dap_instance_init(dap);

		if (dap->adi_version == 6) {
			dap->adi_ap_reg_offset = ADIV6_REG_DELTA;
			LOG_INFO("DAP %s configured to use ADIv6 protocol by user cfg file", jtag_tap_name(dap->tap));
			retval = dap->ops->connect(dap);
			if (retval != ERROR_OK)
				return retval;
		} else if (dap->adi_version != 5) {
			/***************************************************/
			/* User did not specify ADIv5 or ADIv6 override    */
			/* so read DPIDR and switch ADI version if need be.*/
			/* Note the initial read is via an ADIv6 connection*/
			/* since that connection can handle all ADIv5 ACK  */
			/* responses. An ADIv5 connection will not         */
			/* recognize an ADIv6 ACK response of '4' (OK)     */
			/***************************************************/
			dap->adi_ap_reg_offset = ADIV6_REG_DELTA;
			dap->adi_version = 6;
			retval = dap->ops->connect(dap);
			if (retval != ERROR_OK)
				return retval;
			retval = dap->ops->queue_dp_read(dap, DP_DPIDR, &dpidr);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR failed...");
				return retval;
			}
			retval = dap_run(dap);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR failed...");
				return retval;
			}

			if (((dpidr & 0x0000F000) >> 12) < 3) {
				LOG_INFO("DAP %s DPIDR indicates ADIv5 protocol is being used", jtag_tap_name(dap->tap));
				dap->adi_version = 5;
				dap->adi_ap_reg_offset = ADIV5_REG_DELTA;	/* MEM AP register address delta to apply */
				retval = dap->ops->connect(dap);
				if (retval != ERROR_OK)
					return retval;
			} else {
				/* target is using an ADI v6 DAP which has already been set up */
				LOG_INFO("DAP %s DPIDR indicates ADIv6 protocol is being used", jtag_tap_name(dap->tap));
			}
		} else {
			/**************************************************/
			/* User configuration wants to force use of ADI-v5*/
			/* This may be required on DPv0 parts that do not */
			/* have a DPIDR register value indicating ADI-v5  */
			/**************************************************/
			LOG_INFO("DAP %s configured to use ADIv5 protocol by user cfg file", jtag_tap_name(dap->tap));
			dap->adi_ap_reg_offset = ADIV5_REG_DELTA;	/* MEM AP register address delta to apply */
			retval = dap->ops->connect(dap);
			if (retval != ERROR_OK)
				return retval;
		}
		/* see if address size of ROM Table is greater than 32-bits */
		if (dap->adi_version == 6) {
			retval = dap->ops->queue_dp_read(dap, DP_DPIDR1, &dpidr);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR1 failed...");
				return retval;
			}
			retval = dap_run(dap);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR1 failed...");
				return retval;
			}
			dap->asize = dpidr & 0x0000007F;
		} else
			dap->asize = 32;  /* ADIv5 only supports one select reg */
	}

	return ERROR_OK;
}

int dap_cleanup_all(void)
{
	struct arm_dap_object *obj, *tmp;
	struct adiv5_dap *dap;

	list_for_each_entry_safe(obj, tmp, &all_dap, lh) {
		dap = &obj->dap;
		if (dap->ops && dap->ops->quit)
			dap->ops->quit(dap);

		free(obj->name);
		free(obj);
	}

	return ERROR_OK;
}

enum dap_cfg_param {
	CFG_CHAIN_POSITION,
	CFG_IGNORE_SYSPWRUPACK,
	CFG_ADIV6,
	CFG_ADIV5,
};

static const Jim_Nvp nvp_config_opts[] = {
	{ .name = "-chain-position",   .value = CFG_CHAIN_POSITION },
	{ .name = "-ignore-syspwrupack", .value = CFG_IGNORE_SYSPWRUPACK },
	{ .name = "-adiv6",   .value = CFG_ADIV6 },
	{ .name = "-adiv5",   .value = CFG_ADIV5 },
	{ .name = NULL, .value = -1 }
};

static int dap_configure(Jim_GetOptInfo *goi, struct arm_dap_object *dap)
{
	struct jtag_tap *tap = NULL;
	Jim_Nvp *n;
	int e;

	/* parse config or cget options ... */
	while (goi->argc > 0) {
		Jim_SetEmptyResult(goi->interp);

		e = Jim_GetOpt_Nvp(goi, nvp_config_opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, nvp_config_opts, 0);
			return e;
		}
		switch (n->value) {
		case CFG_CHAIN_POSITION: {
			Jim_Obj *o_t;
			e = Jim_GetOpt_Obj(goi, &o_t);
			if (e != JIM_OK)
				return e;
			tap = jtag_tap_by_jim_obj(goi->interp, o_t);
			if (tap == NULL) {
				Jim_SetResultString(goi->interp, "-chain-position is invalid", -1);
				return JIM_ERR;
			}
			/* loop for more */
			break;
		}
		case CFG_IGNORE_SYSPWRUPACK:
			dap->dap.ignore_syspwrupack = true;
			break;
		case CFG_ADIV6:
			dap->dap.adi_version = 6;
			break;
		case CFG_ADIV5:
			dap->dap.adi_version = 5;
			break;
		default:
			break;
		}
	}

	if (tap == NULL) {
		Jim_SetResultString(goi->interp, "-chain-position required when creating DAP", -1);
		return JIM_ERR;
	}

	dap_instance_init(&dap->dap);
	dap->dap.tap = tap;

	return JIM_OK;
}

static int dap_create(Jim_GetOptInfo *goi)
{
	struct command_context *cmd_ctx;
	static struct arm_dap_object *dap;
	Jim_Obj *new_cmd;
	Jim_Cmd *cmd;
	const char *cp;
	int e;

	cmd_ctx = current_command_context(goi->interp);
	assert(cmd_ctx != NULL);

	if (goi->argc < 3) {
		Jim_WrongNumArgs(goi->interp, 1, goi->argv, "?name? ..options...");
		return JIM_ERR;
	}
	/* COMMAND */
	Jim_GetOpt_Obj(goi, &new_cmd);
	/* does this command exist? */
	cmd = Jim_GetCommand(goi->interp, new_cmd, JIM_ERRMSG);
	if (cmd) {
		cp = Jim_GetString(new_cmd, NULL);
		Jim_SetResultFormatted(goi->interp, "Command: %s Exists", cp);
		return JIM_ERR;
	}

	/* Create it */
	dap = calloc(1, sizeof(struct arm_dap_object));
	if (dap == NULL)
		return JIM_ERR;

	e = dap_configure(goi, dap);
	if (e != JIM_OK) {
		free(dap);
		return e;
	}

	cp = Jim_GetString(new_cmd, NULL);
	dap->name = strdup(cp);

	struct command_registration dap_commands[] = {
		{
			.name = cp,
			.mode = COMMAND_ANY,
			.help = "dap instance command group",
			.usage = "",
			.chain = dap_instance_commands,
		},
		COMMAND_REGISTRATION_DONE
	};

	/* don't expose the instance commands when using hla */
	if (transport_is_hla())
		dap_commands[0].chain = NULL;

	e = register_commands(cmd_ctx, NULL, dap_commands);
	if (ERROR_OK != e)
		return JIM_ERR;

	struct command *c = command_find_in_context(cmd_ctx, cp);
	assert(c);
	command_set_handler_data(c, dap);

	list_add_tail(&dap->lh, &all_dap);

	return (ERROR_OK == e) ? JIM_OK : JIM_ERR;
}

static int jim_dap_create(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);
	if (goi.argc < 2) {
		Jim_WrongNumArgs(goi.interp, goi.argc, goi.argv,
			"<name> [<dap_options> ...]");
		return JIM_ERR;
	}
	return dap_create(&goi);
}

static int jim_dap_names(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct arm_dap_object *obj;

	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "Too many parameters");
		return JIM_ERR;
	}
	Jim_SetResult(interp, Jim_NewListObj(interp, NULL, 0));
	list_for_each_entry(obj, &all_dap, lh) {
		Jim_ListAppendElement(interp, Jim_GetResult(interp),
			Jim_NewStringObj(interp, obj->name, -1));
	}
	return JIM_OK;
}

COMMAND_HANDLER(handle_dap_init)
{
	return dap_init_all();
}

COMMAND_HANDLER(handle_dap_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;
	uint32_t apsel;

	if (dap == NULL) {
		LOG_ERROR("DAP instance not available. Probably a HLA target...");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	switch (CMD_ARGC) {
		case 0:
			apsel = dap->apsel;
			break;
		case 1:
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
			if (apsel > DP_APSEL_MAX)
				return ERROR_COMMAND_SYNTAX_ERROR;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return dap_info_command(CMD, &dap->ap[apsel]);
}

static const struct command_registration dap_subcommand_handlers[] = {
	{
		.name = "create",
		.mode = COMMAND_ANY,
		.jim_handler = jim_dap_create,
		.usage = "name '-chain-position' name",
		.help = "Creates a new DAP instance",
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.jim_handler = jim_dap_names,
		.usage = "",
		.help = "Lists all registered DAP instances by name",
	},
	{
		.name = "init",
		.mode = COMMAND_ANY,
		.handler = handle_dap_init,
		.usage = "",
		.help = "Initialize all registered DAP instances"
	},
	{
		.name = "info",
		.handler = handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "display ROM table for MEM-AP of current target "
		"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dap_commands[] = {
	{
		.name = "dap",
		.mode = COMMAND_CONFIG,
		.help = "DAP commands",
		.chain = dap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int dap_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, dap_commands);
}
