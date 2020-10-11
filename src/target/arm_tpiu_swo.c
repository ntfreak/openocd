/* SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * @file
 * This file implements support for the ARM CoreSight components Trace Port
 * Interface Unit (TPIU) and Serial Wire Output (SWO). It also supports the
 * CoreSight TPIU-Lite and the special TPIU version present with Cortex-M3
 * and Cortex-M4 (that includes SWO).
 */

/*
 * Relevant specifications from ARM include:
 *
 * CoreSight(tm) Components Technical Reference Manual           ARM DDI 0314H
 * CoreSight(tm) TPIU-Lite Technical Reference Manual            ARM DDI 0317A
 * Cortex(tm)-M3 Technical Reference Manual                      ARM DDI 0337G
 * Cortex(tm)-M4 Technical Reference Manual                      ARM DDI 0439B
 * CoreSight(tm) SoC-400 Technical Reference Manual              ARM DDI 0480F
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <jim.h>

#include <helper/bits.h>
#include <helper/command.h>
#include <helper/jim-nvp.h>
#include <helper/list.h>
#include <helper/log.h>
#include <helper/types.h>
#include <jtag/interface.h>
#include <target/arm_adi_v5.h>
#include <target/target.h>
#include <transport/transport.h>
#include "arm_tpiu_swo.h"

/* default for Cortex-M3 and Cortex-M4 specific TPIU */
#define TPIU_SWO_DEFAULT_BASE           0xE0040000

#define TPIU_SSPSR_OFFSET               0x000
#define TPIU_CSPSR_OFFSET               0x004
#define TPIU_ACPR_OFFSET                0x010
#define TPIU_SPPR_OFFSET                0x0F0
#define TPIU_FFSR_OFFSET                0x300
#define TPIU_FFCR_OFFSET                0x304
#define TPIU_FSCR_OFFSET                0x308
#define TPIU_DEVID_OFFSET               0xfc8

#define TPIU_ACPR_MAX_PRESCALER         0x1fff
#define TPIU_SPPR_PROTOCOL_SYNC         0x0 /**< synchronous trace output */
#define TPIU_SPPR_PROTOCOL_MANCHESTER   0x1 /**< asynchronous output with NRZ coding */
#define TPIU_SPPR_PROTOCOL_UART         0x2 /**< asynchronous output with Manchester coding */
#define TPIU_DEVID_NOSUPPORT_SYNC       BIT(9)
#define TPIU_DEVID_SUPPORT_MANCHESTER   BIT(10)
#define TPIU_DEVID_SUPPORT_UART         BIT(11)

enum arm_tpiu_swo_event {
	TPIU_SWO_EVENT_PRE_ENABLE,
	TPIU_SWO_EVENT_POST_ENABLE,
	TPIU_SWO_EVENT_PRE_DISABLE,
	TPIU_SWO_EVENT_POST_DISABLE,
};

static const Jim_Nvp nvp_arm_tpiu_swo_event[] = {
	{ .value = TPIU_SWO_EVENT_PRE_ENABLE,   .name = "pre-enable" },
	{ .value = TPIU_SWO_EVENT_POST_ENABLE,  .name = "post-enable" },
	{ .value = TPIU_SWO_EVENT_PRE_DISABLE,  .name = "pre-disable" },
	{ .value = TPIU_SWO_EVENT_POST_DISABLE, .name = "post-disable" },
};

struct arm_tpiu_swo_event_action {
	enum arm_tpiu_swo_event event;
	Jim_Interp *interp;
	Jim_Obj *body;
	struct arm_tpiu_swo_event_action *next;
};

struct arm_tpiu_swo_object {
	struct list_head lh;
	struct adiv5_mem_ap_spot spot;
	char *name;
	struct arm_tpiu_swo_event_action *event_action;
	bool enabled;
	bool en_capture;
	/** Handle to output trace data in INTERNAL capture mode */
	/** Synchronous output port width */
	uint32_t port_width;
	FILE *file;
	/** output mode */
	unsigned int pin_protocol;
	/** Enable formatter */
	bool en_formatter;
	/** frequency of TRACECLKIN (usually matches HCLK) */
	unsigned int traceclkin_freq;
	/** SWO pin frequency */
	unsigned int swo_pin_freq;
	/** where to dump the captured output trace data */
	char *out_filename;
};

static LIST_HEAD(all_tpiu_swo);

#define ARM_TPIU_SWO_TRACE_BUF_SIZE	4096

static int arm_tpiu_swo_poll_trace(void *priv)
{
	struct arm_tpiu_swo_object *obj = priv;
	uint8_t buf[ARM_TPIU_SWO_TRACE_BUF_SIZE];
	size_t size = sizeof(buf);

	int retval = adapter_poll_trace(buf, &size);
	if (retval != ERROR_OK || !size)
		return retval;

	target_call_trace_callbacks(/*target*/NULL, size, buf);

	if (obj->file) {
		if (fwrite(buf, 1, size, obj->file) == size) {
			fflush(obj->file);
		} else {
			LOG_ERROR("Error writing to the SWO trace destination file");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static void arm_tpiu_swo_close_file(struct arm_tpiu_swo_object *obj)
{
	if (!obj->file)
		return;
	fclose(obj->file);
	obj->file = NULL;
}

int arm_tpiu_swo_cleanup_all(void)
{
	struct arm_tpiu_swo_object *obj, *tmp;

	list_for_each_entry_safe(obj, tmp, &all_tpiu_swo, lh) {
		struct arm_tpiu_swo_event_action *ea = obj->event_action;

		while (ea) {
			struct arm_tpiu_swo_event_action *next = ea->next;
			Jim_DecrRefCount(ea->interp, ea->body);
			free(ea);
			ea = next;
		}
		if (obj->en_capture)
			target_unregister_timer_callback(arm_tpiu_swo_poll_trace, obj);
		arm_tpiu_swo_close_file(obj);
		free(obj->name);
		free(obj->out_filename);
		free(obj);
	}

	return ERROR_OK;
}

static void arm_tpiu_swo_handle_event(struct arm_tpiu_swo_object *obj, enum arm_tpiu_swo_event event)
{
	for (struct arm_tpiu_swo_event_action *ea = obj->event_action; ea; ea = ea->next) {
		if (ea->event != event)
			continue;

		LOG_DEBUG("TPIU/SWO: %s event: %s (%d) action : %s",
			obj->name,
			Jim_Nvp_value2name_simple(nvp_arm_tpiu_swo_event, event)->name,
			event,
			Jim_GetString(ea->body, NULL));

		/* prevent event execution to change current target */
		struct command_context *cmd_ctx = current_command_context(ea->interp);
		struct target *saved_target = cmd_ctx->current_target;
		int retval = Jim_EvalObj(ea->interp, ea->body);
		cmd_ctx->current_target = saved_target;

		if (retval == JIM_RETURN)
			retval = ea->interp->returnCode;
		if (retval == JIM_OK || retval == ERROR_COMMAND_CLOSE_CONNECTION)
			return;

		Jim_MakeErrorMessage(ea->interp);
		LOG_USER("Error executing event %s on TPIU/SWO %s:\n%s",
			Jim_Nvp_value2name_simple(nvp_arm_tpiu_swo_event, event)->name,
			obj->name,
			Jim_GetString(Jim_GetResult(ea->interp), NULL));
		/* clean both error code and stacktrace before return */
		Jim_Eval(ea->interp, "error \"\" \"\"");
		return;
	}
}

COMMAND_HANDLER(handle_arm_tpiu_swo_event_list)
{
	struct arm_tpiu_swo_object *obj = CMD_DATA;

	command_print(CMD, "Event actions for TPIU/SWO %s\n", obj->name);
	command_print(CMD, "%-25s | Body", "Event");
	command_print(CMD, "------------------------- | "
			"----------------------------------------");

	for (struct arm_tpiu_swo_event_action *ea = obj->event_action; ea; ea = ea->next) {
		Jim_Nvp *opt = Jim_Nvp_value2name_simple(nvp_arm_tpiu_swo_event, ea->event);
		command_print(CMD, "%-25s | %s",
				opt->name, Jim_GetString(ea->body, NULL));
	}
	command_print(CMD, "***END***");
	return ERROR_OK;
}

enum arm_tpiu_swo_cfg_param {
	CFG_PORT_WIDTH,
	CFG_PROTOCOL,
	CFG_FORMATTER,
	CFG_TRACECLKIN,
	CFG_BITRATE,
	CFG_OUTFILE,
	CFG_EVENT,
};

static const Jim_Nvp nvp_arm_tpiu_swo_config_opts[] = {
	{ .name = "-port-width",    .value = CFG_PORT_WIDTH },
	{ .name = "-protocol",      .value = CFG_PROTOCOL },
	{ .name = "-formatter",     .value = CFG_FORMATTER },
	{ .name = "-traceclk",      .value = CFG_TRACECLKIN },
	{ .name = "-pin-freq",      .value = CFG_BITRATE },
	{ .name = "-output",        .value = CFG_OUTFILE },
	{ .name = "-event",         .value = CFG_EVENT },
	/* handled by mem_ap_spot, added for Jim_GetOpt_NvpUnknown() */
	{ .name = "-dap",           .value = -1 },
	{ .name = "-ap-num",        .value = -1 },
	{ .name = "-baseaddr",      .value = -1 },
	{ .name = NULL,             .value = -1 },
};

static const Jim_Nvp nvp_arm_tpiu_swo_protocol_opts[] = {
	{ .name = "sync",           .value = TPIU_SPPR_PROTOCOL_SYNC },
	{ .name = "uart",           .value = TPIU_SPPR_PROTOCOL_UART },
	{ .name = "manchester",     .value = TPIU_SPPR_PROTOCOL_MANCHESTER },
	{ .name = NULL,             .value = -1 },
};

static const Jim_Nvp nvp_arm_tpiu_swo_bool_opts[] = {
	{ .name = "on",             .value = 1 },
	{ .name = "yes",            .value = 1 },
	{ .name = "1",              .value = 1 },
	{ .name = "true",           .value = 1 },
	{ .name = "off",            .value = 0 },
	{ .name = "no",             .value = 0 },
	{ .name = "0",              .value = 0 },
	{ .name = "false",          .value = 0 },
	{ .name = NULL,             .value = -1 },
};

static int arm_tpiu_swo_configure(Jim_GetOptInfo *goi, struct arm_tpiu_swo_object *obj)
{
	assert(obj != NULL);

	if (goi->isconfigure && obj->enabled) {
		Jim_SetResultString(goi->interp, "Configuration not allowed on TPIU/SWO enabled!", -1);
		return JIM_ERR;
	}

	/* parse config or cget options ... */
	while (goi->argc > 0) {
		Jim_SetEmptyResult(goi->interp);

		int e = adiv5_jim_mem_ap_spot_configure(&obj->spot, goi);
		if (e == JIM_OK)
			continue;
		if (e == JIM_ERR)
			return e;

		Jim_Nvp *n;
		e = Jim_GetOpt_Nvp(goi, nvp_arm_tpiu_swo_config_opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, nvp_arm_tpiu_swo_config_opts, 0);
			return e;
		}

		switch (n->value) {
		case CFG_PORT_WIDTH:
			if (goi->isconfigure) {
				jim_wide port_width;
				e = Jim_GetOpt_Wide(goi, &port_width);
				if (e != JIM_OK)
					return e;
				if (port_width < 1 || port_width > 32) {
					Jim_SetResultString(goi->interp, "Invalid port width!", -1);
					return JIM_ERR;
				}
				obj->port_width = (uint32_t)port_width;
			} else {
				if (goi->argc)
					goto err_no_params;
				Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, obj->port_width));
			}
			break;
		case CFG_PROTOCOL:
			if (goi->isconfigure) {
				Jim_Nvp *p;
				e = Jim_GetOpt_Nvp(goi, nvp_arm_tpiu_swo_protocol_opts, &p);
				if (e != JIM_OK)
					return e;
				obj->pin_protocol = p->value;
			} else {
				if (goi->argc)
					goto err_no_params;
				Jim_Nvp *p;
				e = Jim_Nvp_value2name(goi->interp, nvp_arm_tpiu_swo_protocol_opts, obj->pin_protocol, &p);
				if (e != JIM_OK) {
					Jim_SetResultString(goi->interp, "protocol error", -1);
					return JIM_ERR;
				}
				Jim_SetResult(goi->interp, Jim_NewStringObj(goi->interp, p->name, -1));
			}
			break;
		case CFG_FORMATTER:
			if (goi->isconfigure) {
				Jim_Nvp *p;
				e = Jim_GetOpt_Nvp(goi, nvp_arm_tpiu_swo_bool_opts, &p);
				if (e != JIM_OK)
					return e;
				obj->en_formatter = p->value;
			} else {
				if (goi->argc)
					goto err_no_params;
				Jim_Nvp *p;
				e = Jim_Nvp_value2name(goi->interp, nvp_arm_tpiu_swo_bool_opts, obj->en_formatter, &p);
				if (e != JIM_OK) {
					Jim_SetResultString(goi->interp, "formatter error", -1);
					return JIM_ERR;
				}
				Jim_SetResult(goi->interp, Jim_NewStringObj(goi->interp, p->name, -1));
			}
			break;
		case CFG_TRACECLKIN:
			if (goi->isconfigure) {
				jim_wide clk;
				e = Jim_GetOpt_Wide(goi, &clk);
				if (e != JIM_OK)
					return e;
				obj->traceclkin_freq = clk;
			} else {
				if (goi->argc)
					goto err_no_params;
				Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, obj->traceclkin_freq));
			}
			break;
		case CFG_BITRATE:
			if (goi->isconfigure) {
				jim_wide clk;
				e = Jim_GetOpt_Wide(goi, &clk);
				if (e != JIM_OK)
					return e;
				obj->swo_pin_freq = clk;
			} else {
				if (goi->argc)
					goto err_no_params;
				Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, obj->swo_pin_freq));
			}
			break;
		case CFG_OUTFILE:
			if (goi->isconfigure) {
				const char *s;
				e = Jim_GetOpt_String(goi, &s, NULL);
				if (e != JIM_OK)
					return e;
				free(obj->out_filename);
				obj->out_filename = strdup(s);
				if (!obj->out_filename) {
					LOG_ERROR("Out of memory");
					return JIM_ERR;
				}
			} else {
				if (goi->argc)
					goto err_no_params;
				if (obj->out_filename)
					Jim_SetResult(goi->interp, Jim_NewStringObj(goi->interp, obj->out_filename, -1));
			}
			break;
		case CFG_EVENT:
			if (goi->isconfigure) {
				if (goi->argc < 2) {
					Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name? ?EVENT-BODY?");
					return JIM_ERR;
				}
			} else {
				if (goi->argc != 1) {
					Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name?");
					return JIM_ERR;
				}
			}

			{
				Jim_Nvp *p;
				Jim_Obj *o;
				struct arm_tpiu_swo_event_action *ea = obj->event_action;

				e = Jim_GetOpt_Nvp(goi, nvp_arm_tpiu_swo_event, &p);
				if (e != JIM_OK) {
					Jim_GetOpt_NvpUnknown(goi, nvp_arm_tpiu_swo_event, 1);
					return e;
				}

				while (ea) {
					/* replace existing? */
					if (ea->event == (enum arm_tpiu_swo_event)p->value)
						break;
					ea = ea->next;
				}

				if (goi->isconfigure) {
					if (!ea) {
						ea = calloc(1, sizeof(*ea));
						if (!ea) {
							LOG_ERROR("Out of memory");
							return JIM_ERR;
						}
						ea->next = obj->event_action;
						obj->event_action = ea;
					}
					if (ea->body)
						Jim_DecrRefCount(ea->interp, ea->body);
					ea->event = p->value;
					ea->interp = goi->interp;
					Jim_GetOpt_Obj(goi, &o);
					ea->body = Jim_DuplicateObj(goi->interp, o);
					Jim_IncrRefCount(ea->body);
				} else {
					if (ea)
						Jim_SetResult(goi->interp, Jim_DuplicateObj(goi->interp, ea->body));
				}
			}
			break;
		}
	}

	return JIM_OK;

err_no_params:
	Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "NO PARAMS");
	return JIM_ERR;
}

static int jim_arm_tpiu_swo_configure(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	Jim_GetOptInfo goi;

	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);
	goi.isconfigure = !strcmp(Jim_GetString(argv[0], NULL), "configure");
	if (goi.argc < 1) {
		Jim_WrongNumArgs(goi.interp, goi.argc, goi.argv,
			"missing: -option ...");
		return JIM_ERR;
	}
	struct arm_tpiu_swo_object *obj = Jim_CmdPrivData(interp);
	return arm_tpiu_swo_configure(&goi, obj);
}

static int wrap_write_u32(struct target *target, struct adiv5_ap *tpiu_ap,
		target_addr_t address, uint32_t value)
{
	if (transport_is_hla())
		return target_write_u32(target, address, value);
	else
		return mem_ap_write_u32(tpiu_ap, address, value);
}

static int wrap_read_u32(struct target *target, struct adiv5_ap *tpiu_ap,
		target_addr_t address, uint32_t *value)
{
	if (transport_is_hla())
		return target_read_u32(target, address, value);
	else
		return mem_ap_read_u32(tpiu_ap, address, value);
}

static int jim_arm_tpiu_swo_enable(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct arm_tpiu_swo_object *obj = Jim_CmdPrivData(interp);
	struct command_context *cmd_ctx = current_command_context(interp);
	struct target *target = get_current_target(cmd_ctx);
	struct adiv5_ap *tpiu_ap = dap_ap(obj->spot.dap, obj->spot.ap_num);
	uint32_t value;
	int retval;

	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "Too many parameters");
		return JIM_ERR;
	}

	if (obj->enabled)
		return JIM_OK;

	if (transport_is_hla() && obj->spot.ap_num > 0) {
		LOG_ERROR("Invalid access port %d. Only AP#0 allowed with hla transport", obj->spot.ap_num);
		return JIM_ERR;
	}

	if (!obj->traceclkin_freq) {
		LOG_ERROR("Trace clock-in frequency not set");
		return JIM_ERR;
	}

	if (obj->pin_protocol == TPIU_SPPR_PROTOCOL_MANCHESTER || obj->pin_protocol == TPIU_SPPR_PROTOCOL_UART)
		if (!obj->swo_pin_freq) {
			LOG_ERROR("SWO pin frequency not set");
			return JIM_ERR;
		}

	/* trigger the event before any attempt to R/W in the TPIU/SWO */
	arm_tpiu_swo_handle_event(obj, TPIU_SWO_EVENT_PRE_ENABLE);

	retval = wrap_read_u32(target, tpiu_ap, obj->spot.base + TPIU_DEVID_OFFSET, &value);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to read %s", obj->name);
		return JIM_ERR;
	}
	switch (obj->pin_protocol) {
	case TPIU_SPPR_PROTOCOL_SYNC:
		value = !(value & TPIU_DEVID_NOSUPPORT_SYNC);
		break;
	case TPIU_SPPR_PROTOCOL_UART:
		value &= TPIU_DEVID_SUPPORT_UART;
		break;
	case TPIU_SPPR_PROTOCOL_MANCHESTER:
		value &= TPIU_DEVID_SUPPORT_MANCHESTER;
		break;
	default:
		value = 0;
	}
	if (!value) {
		Jim_Nvp *p;
		Jim_Nvp_value2name(interp, nvp_arm_tpiu_swo_protocol_opts, obj->pin_protocol, &p);
		LOG_ERROR("%s does not support protocol %s", obj->name, p->name);
		return JIM_ERR;
	}

	if (obj->pin_protocol == TPIU_SPPR_PROTOCOL_SYNC) {
		retval = wrap_read_u32(target, tpiu_ap, obj->spot.base + TPIU_SSPSR_OFFSET, &value);
		if (!(value & BIT(obj->port_width - 1))) {
			LOG_ERROR("TPIU does not support port-width of %d bits", obj->port_width);
			return JIM_ERR;
		}
	}

	uint16_t prescaler = 1; /* dummy value */
	unsigned int swo_pin_freq = obj->swo_pin_freq; /* could be replaced */

	if (obj->out_filename && strcmp(obj->out_filename, "external") && obj->out_filename[0]) {
		if (strcmp(obj->out_filename, "-")) {
			obj->file = fopen(obj->out_filename, "ab");
			if (!obj->file) {
				LOG_ERROR("Can't open trace destination file \"%s\"", obj->out_filename);
				return JIM_ERR;
			}
		}

		retval = adapter_config_trace(true, obj->pin_protocol, obj->port_width,
			&swo_pin_freq, obj->traceclkin_freq, &prescaler);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to start adapter's SWO");
			arm_tpiu_swo_close_file(obj);
			return JIM_ERR;
		}

		if (obj->swo_pin_freq != swo_pin_freq)
			LOG_INFO("SWO pin data rate adjusted by adapter to %d Hz", swo_pin_freq);
		obj->swo_pin_freq = swo_pin_freq;

		target_register_timer_callback(arm_tpiu_swo_poll_trace, 1,
			TARGET_TIMER_TYPE_PERIODIC, obj);

		obj->en_capture = true;
	} else if (obj->pin_protocol == TPIU_SPPR_PROTOCOL_MANCHESTER || obj->pin_protocol == TPIU_SPPR_PROTOCOL_UART) {
		prescaler = (obj->traceclkin_freq + obj->swo_pin_freq / 2) / obj->swo_pin_freq;
		if (prescaler > TPIU_ACPR_MAX_PRESCALER)
			prescaler = TPIU_ACPR_MAX_PRESCALER;
		swo_pin_freq = obj->traceclkin_freq / prescaler;

		if (obj->swo_pin_freq != swo_pin_freq)
			LOG_INFO("SWO pin data rate adjusted to %d Hz", swo_pin_freq);
		obj->swo_pin_freq = swo_pin_freq;
	}

	retval = wrap_write_u32(target, tpiu_ap, obj->spot.base + TPIU_CSPSR_OFFSET, BIT(obj->port_width - 1));
	if (retval != ERROR_OK)
		goto error_exit;

	retval = wrap_write_u32(target, tpiu_ap, obj->spot.base + TPIU_ACPR_OFFSET, prescaler - 1);
	if (retval != ERROR_OK)
		goto error_exit;

	retval = wrap_write_u32(target, tpiu_ap, obj->spot.base + TPIU_SPPR_OFFSET, obj->pin_protocol);
	if (retval != ERROR_OK)
		goto error_exit;

	retval = wrap_read_u32(target, tpiu_ap, obj->spot.base + TPIU_FFCR_OFFSET, &value);
	if (retval != ERROR_OK)
		goto error_exit;
	if (obj->en_formatter)
		value |= BIT(1);
	else
		value &= ~BIT(1);
	retval = wrap_write_u32(target, tpiu_ap, obj->spot.base + TPIU_FFCR_OFFSET, value);
	if (retval != ERROR_OK)
		goto error_exit;

	arm_tpiu_swo_handle_event(obj, TPIU_SWO_EVENT_POST_ENABLE);

	obj->enabled = true;
	return JIM_OK;

error_exit:
	LOG_ERROR("Error!");
	if (obj->en_capture)
		target_unregister_timer_callback(arm_tpiu_swo_poll_trace, obj);
	obj->en_capture = false;

	arm_tpiu_swo_close_file(obj);
	return JIM_ERR;
}

static int jim_arm_tpiu_swo_disable(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct arm_tpiu_swo_object *obj = Jim_CmdPrivData(interp);

	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "Too many parameters");
		return JIM_ERR;
	}

	if (!obj->enabled)
		return JIM_OK;
	obj->enabled = false;

	arm_tpiu_swo_handle_event(obj, TPIU_SWO_EVENT_PRE_DISABLE);

	arm_tpiu_swo_close_file(obj);

	if (obj->en_capture) {
		obj->en_capture = false;
		target_unregister_timer_callback(arm_tpiu_swo_poll_trace, obj);

		int retval = adapter_config_trace(0, 0, 0, NULL, 0, NULL);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to stop adapter's SWO");
			return JIM_ERR;
		}
	}

	arm_tpiu_swo_handle_event(obj, TPIU_SWO_EVENT_POST_DISABLE);
	return JIM_OK;
}

static const struct command_registration arm_tpiu_swo_instance_command_handlers[] = {
	{
		.name = "configure",
		.mode = COMMAND_ANY,
		.jim_handler = jim_arm_tpiu_swo_configure,
		.help  = "configure a new TPIU/SWO for use",
		.usage = "[attribute value ...]",
	},
	{
		.name = "cget",
		.mode = COMMAND_ANY,
		.jim_handler = jim_arm_tpiu_swo_configure,
		.help  = "returns the specified TPIU/SWO attribute",
		.usage = "attribute",
	},
	{
		.name = "eventlist",
		.mode = COMMAND_EXEC,
		.handler = handle_arm_tpiu_swo_event_list,
		.help = "displays a table of events defined for this TPIU/SWO",
		.usage = "",
	},
	{
		.name = "enable",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_arm_tpiu_swo_enable,
		.usage = "",
		.help = "Enables the TPIU/SWO output",
	},
	{
		.name = "disable",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_arm_tpiu_swo_disable,
		.usage = "",
		.help = "Disables the TPIU/SWO output",
	},
	COMMAND_REGISTRATION_DONE
};

static int arm_tpiu_swo_create(Jim_Interp *interp, struct arm_tpiu_swo_object *obj)
{
	struct command_context *cmd_ctx;
	Jim_Cmd *cmd;
	int e;

	cmd_ctx = current_command_context(interp);
	assert(cmd_ctx != NULL);

	/* does this command exist? */
	cmd = Jim_GetCommand(interp, Jim_NewStringObj(interp, obj->name, -1), JIM_ERRMSG);
	if (cmd) {
		Jim_SetResultFormatted(interp, "Command: %s Exists", obj->name);
		return JIM_ERR;
	}

	/* now - create the new tpiu/swo name command */
	const struct command_registration obj_commands[] = {
		{
			.name = obj->name,
			.mode = COMMAND_ANY,
			.help = "tpiu/swo instance command group",
			.usage = "",
			.chain = arm_tpiu_swo_instance_command_handlers,
		},
		COMMAND_REGISTRATION_DONE
	};
	e = register_commands(cmd_ctx, NULL, obj_commands);
	if (ERROR_OK != e)
		return JIM_ERR;

	struct command *c = command_find_in_context(cmd_ctx, obj->name);
	assert(c);
	command_set_handler_data(c, obj);

	list_add_tail(&obj->lh, &all_tpiu_swo);

	return JIM_OK;
}

static int jim_arm_tpiu_swo_create(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);
	if (goi.argc < 1) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv, "?name? ..options...");
		return JIM_ERR;
	}

	struct arm_tpiu_swo_object *obj = calloc(1, sizeof(struct arm_tpiu_swo_object));
	if (!obj) {
		LOG_ERROR("Out of memory");
		return JIM_ERR;
	}
	adiv5_mem_ap_spot_init(&obj->spot);
	obj->spot.base = TPIU_SWO_DEFAULT_BASE;
	obj->port_width = 1;

	Jim_Obj *n;
	Jim_GetOpt_Obj(&goi, &n);
	obj->name = strdup(Jim_GetString(n, NULL));
	if (!obj->name) {
		LOG_ERROR("Out of memory");
		free(obj);
		return JIM_ERR;
	}

	/* Do the rest as "configure" options */
	goi.isconfigure = 1;
	int e = arm_tpiu_swo_configure(&goi, obj);
	if (e != JIM_OK)
		goto err_exit;

	if (!obj->spot.dap || obj->spot.ap_num == DP_APSEL_INVALID) {
		Jim_SetResultString(goi.interp, "-dap and -ap-num required when creating TPIU", -1);
		goto err_exit;
	}

	e = arm_tpiu_swo_create(goi.interp, obj);
	if (e != JIM_OK)
		goto err_exit;

	return JIM_OK;

err_exit:
	free(obj->name);
	free(obj->out_filename);
	free(obj);
	return JIM_ERR;
}

static int jim_arm_tpiu_swo_names(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct arm_tpiu_swo_object *obj;

	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "Too many parameters");
		return JIM_ERR;
	}
	Jim_SetResult(interp, Jim_NewListObj(interp, NULL, 0));
	list_for_each_entry(obj, &all_tpiu_swo, lh) {
		Jim_ListAppendElement(interp, Jim_GetResult(interp),
			Jim_NewStringObj(interp, obj->name, -1));
	}
	return JIM_OK;
}

static const struct command_registration arm_tpiu_swo_subcommand_handlers[] = {
	{
		.name = "create",
		.mode = COMMAND_ANY,
		.jim_handler = jim_arm_tpiu_swo_create,
		.usage = "name [-dap dap] [-ap-num num] [-address baseaddr]",
		.help = "Creates a new TPIU or SWO object",
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.jim_handler = jim_arm_tpiu_swo_names,
		.usage = "",
		.help = "Lists all registered TPIU and SWO objects by name",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration arm_tpiu_swo_command_handlers[] = {
	{
		.name = "tpiu",
		.chain = arm_tpiu_swo_subcommand_handlers,
		.usage = "",
		.help = "tpiu command group",
	},
	{
		.name = "swo",
		.chain = arm_tpiu_swo_subcommand_handlers,
		.usage = "",
		.help = "swo command group",
	},
	COMMAND_REGISTRATION_DONE
};

int arm_tpiu_swo_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, arm_tpiu_swo_command_handlers);
}
