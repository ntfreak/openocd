/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#include <helper/log.h>
#include <helper/binarybuffer.h>

#include "target.h"
#include "target_request.h"
#include "target_type.h"
#include "trace.h"

static bool got_message;

bool target_got_message(void)
{
	bool t = got_message;
	got_message = false;
	return t;
}

static int charmsg_mode;

static int target_asciimsg(struct target *target, uint32_t length)
{
	char *msg = malloc(DIV_ROUND_UP(length + 1, 4) * 4);
	struct debug_msg_receiver *c = target->dbgmsg;

	target->type->target_request_data(target, DIV_ROUND_UP(length, 4), (uint8_t *)msg);
	msg[length] = 0;

	LOG_DEBUG("%s", msg);

	while (c) {
		command_output_text(c->cmd_ctx, msg);
		c = c->next;
	}

	return ERROR_OK;
}

static int target_charmsg(struct target *target, uint8_t msg)
{
	LOG_USER_N("%c", msg);

	return ERROR_OK;
}

static int target_hexmsg(struct target *target, int size, uint32_t length)
{
	uint8_t *data = malloc(DIV_ROUND_UP(length * size, 4) * 4);
	char line[128];
	int line_len;
	struct debug_msg_receiver *c = target->dbgmsg;
	uint32_t i;

	LOG_DEBUG("size: %i, length: %i", (int)size, (int)length);

	target->type->target_request_data(target, DIV_ROUND_UP(length * size, 4), (uint8_t *)data);

	line_len = 0;
	for (i = 0; i < length; i++) {
		switch (size) {
			case 4:
				line_len += snprintf(line + line_len, 128 - line_len, "%8.8" PRIx32 " ", le_to_h_u32(data + (4*i)));
				break;
			case 2:
				line_len += snprintf(line + line_len, 128 - line_len, "%4.4x ", le_to_h_u16(data + (2*i)));
				break;
			case 1:
				line_len += snprintf(line + line_len, 128 - line_len, "%2.2x ", data[i]);
				break;
		}

		if ((i%8 == 7) || (i == length - 1)) {
			LOG_DEBUG("%s", line);

			while (c) {
				command_output_text(c->cmd_ctx, line);
				c = c->next;
			}
			c = target->dbgmsg;
			line_len = 0;
		}
	}

	free(data);

	return ERROR_OK;
}

/* handle requests from the target received by a target specific
 * side-band channel (e.g. ARM7/9 DCC)
 */
int target_request(struct target *target, uint32_t request)
{
	target_req_cmd_t target_req_cmd = request & 0xff;

	assert(target->type->target_request_data);

	/* Record that we got a target message for back-off algorithm */
	got_message = true;

	if (charmsg_mode) {
		target_charmsg(target, target_req_cmd);
		return ERROR_OK;
	}

	switch (target_req_cmd) {
		case TARGET_REQ_TRACEMSG:
			trace_point(target, (request & 0xffffff00) >> 8);
			break;
		case TARGET_REQ_DEBUGMSG:
			if (((request & 0xff00) >> 8) == 0)
				target_asciimsg(target, (request & 0xffff0000) >> 16);
			else
				target_hexmsg(target, (request & 0xff00) >> 8, (request & 0xffff0000) >> 16);
			break;
		case TARGET_REQ_DEBUGCHAR:
			target_charmsg(target, (request & 0x00ff0000) >> 16);
			break;
/*		case TARGET_REQ_SEMIHOSTING:
 *			break;
 */
		default:
			LOG_ERROR("unknown target request: %2.2x", target_req_cmd);
			break;
	}

	return ERROR_OK;
}

static int add_debug_msg_receiver(struct command_context *cmd_ctx, struct target *target)
{
	struct debug_msg_receiver **p = &target->dbgmsg;

	if (!target)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* see if there's already a list */
	if (*p) {
		/* find end of linked list */
		while ((*p)->next)
			p = &((*p)->next);
		p = &((*p)->next);
	}

	/* add new debug message receiver */
	(*p) = malloc(sizeof(struct debug_msg_receiver));
	(*p)->cmd_ctx = cmd_ctx;
	(*p)->next = NULL;

	/* enable callback */
	target->dbg_msg_enabled = 1;

	return ERROR_OK;
}

static struct debug_msg_receiver *find_debug_msg_receiver(struct command_context *cmd_ctx,
		struct target *target)
{
	int do_all_targets = 0;

	/* if no target has been specified search all of them */
	if (!target) {
		/* if no targets haven been specified */
		if (!all_targets)
			return NULL;

		target = all_targets;
		do_all_targets = 1;
	}

	/* so we target != null */
	struct debug_msg_receiver **p = &target->dbgmsg;
	do {
		while (*p) {
			if ((*p)->cmd_ctx == cmd_ctx)
				return *p;
			p = &((*p)->next);
		}

		target = target->next;
	} while (target && do_all_targets);

	return NULL;
}

int delete_debug_msg_receiver(struct command_context *cmd_ctx, struct target *target)
{
	struct debug_msg_receiver **p;
	struct debug_msg_receiver *c;
	int do_all_targets = 0;

	/* if no target has been specified search all of them */
	if (!target) {
		/* if no targets haven been specified */
		if (!all_targets)
			return ERROR_OK;

		target = all_targets;
		do_all_targets = 1;
	}

	do {
		p = &target->dbgmsg;
		c = *p;
		while (c) {
			struct debug_msg_receiver *next = c->next;
			if (c->cmd_ctx == cmd_ctx) {
				*p = next;
				free(c);
				if (!*p) {
					/* disable callback */
					target->dbg_msg_enabled = 0;
				}
				return ERROR_OK;
			} else
				p = &(c->next);
			c = next;
		}

		target = target->next;
	} while (target && do_all_targets);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_target_request_debugmsgs_command)
{
	struct target *target = get_current_target(CMD_CTX);

	int receiving = 0;

	if (!target->type->target_request_data) {
		LOG_ERROR("Target %s does not support target requests", target_name(target));
		return ERROR_OK;
	}

	/* see if receiver is already registered */
	if (find_debug_msg_receiver(CMD_CTX, target))
		receiving = 1;

	if (CMD_ARGC > 0) {
		if (!strcmp(CMD_ARGV[0], "enable") || !strcmp(CMD_ARGV[0], "charmsg")) {
			/* don't register if this command context is already receiving */
			if (!receiving) {
				receiving = 1;
				add_debug_msg_receiver(CMD_CTX, target);
			}
			charmsg_mode = !strcmp(CMD_ARGV[0], "charmsg");
		} else if (!strcmp(CMD_ARGV[0], "disable")) {
			/* no need to delete a receiver if none is registered */
			if (receiving) {
				receiving = 0;
				delete_debug_msg_receiver(CMD_CTX, target);
			}
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "receiving debug messages from current target %s",
			(receiving) ? (charmsg_mode ? "charmsg" : "enabled") : "disabled");
	return ERROR_OK;
}

static const struct command_registration target_req_exec_command_handlers[] = {
	{
		.name = "debugmsgs",
		.handler = handle_target_request_debugmsgs_command,
		.mode = COMMAND_EXEC,
		.help = "display and/or modify reception of debug messages from target",
		.usage = "['enable'|'charmsg'|'disable']",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration target_req_command_handlers[] = {
	{
		.name = "target_request",
		.mode = COMMAND_ANY,
		.help = "target request command group",
		.usage = "",
		.chain = target_req_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int target_request_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, target_req_command_handlers);
}
