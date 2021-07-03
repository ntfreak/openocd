/***************************************************************************
 *   Copyright (C) 2009 - 2010 by Simon Qian <SimonQian@SimonQian.com>     *
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

#include <string.h>

#include "../versaloon_include.h"
#include "../versaloon.h"
#include "../versaloon_internal.h"
#include "usbtoxxx.h"
#include "usbtoxxx_internal.h"

#define N_A "n/a"

static const char *types_name[96] = {
	"usbtousart", "usbtospi", "usbtoi2c", "usbtogpio", "usbtocan", "usbtopwm",
	"usbtoadc", "usbtodac",
	"usbtomicrowire", "usbtoswim", "usbtodusi", N_A, N_A, N_A, "usbtopower", "usbtodelay",
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A,
	N_A, N_A, N_A, N_A, N_A, N_A, N_A,
	"usbtojtagll", "usbtojtaghl", "usbtoissp", "usbtoc2", "usbtosbw",
	"usbtolpcicp", "usbtoswd", "usbtojtagraw",
	"usbtobdm", N_A, N_A, N_A, N_A, N_A, N_A, N_A,
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, N_A,
	"usbtomsp430jtag", N_A, N_A, N_A, N_A, N_A, N_A, N_A,
	"usbtopower", "usbtodelay", "usbtopoll", N_A, N_A, N_A, N_A, N_A,
	N_A, N_A, N_A, N_A, N_A, N_A, N_A, "usbtoall"
};

uint8_t usbtoxxx_abilities[USB_TO_XXX_ABILITIES_LEN];

#define usbtoxxx_get_type_name(type)	\
	types_name[((type) - VERSALOON_USB_TO_XXX_CMD_START) \
		   % ARRAY_SIZE(types_name)]

static uint8_t type_pre;
static uint16_t usbtoxxx_buffer_index;
static uint16_t usbtoxxx_current_cmd_index;
static uint8_t *usbtoxxx_buffer;

static uint16_t collect_index;
static uint8_t collect_cmd;
static uint8_t poll_nesting;

struct usbtoxxx_context_t {
	uint8_t type_pre;
	uint8_t *usbtoxxx_buffer;
	uint16_t usbtoxxx_current_cmd_index;
	uint16_t usbtoxxx_buffer_index;
	uint16_t versaloon_pending_idx;
};
static struct usbtoxxx_context_t poll_context;

static void usbtoxxx_save_context(struct usbtoxxx_context_t *c)
{
	c->type_pre = type_pre;
	c->usbtoxxx_buffer = usbtoxxx_buffer;
	c->usbtoxxx_buffer_index = usbtoxxx_buffer_index;
	c->usbtoxxx_current_cmd_index = usbtoxxx_current_cmd_index;
	c->versaloon_pending_idx = versaloon_pending_idx;
}

static void usbtoxxx_pop_context(struct usbtoxxx_context_t *c)
{
	type_pre = c->type_pre;
	usbtoxxx_buffer = c->usbtoxxx_buffer;
	usbtoxxx_buffer_index = c->usbtoxxx_buffer_index;
	usbtoxxx_current_cmd_index = c->usbtoxxx_current_cmd_index;
	versaloon_pending_idx = c->versaloon_pending_idx;
}

static RESULT usbtoxxx_validate_current_command_type(void)
{
	if (type_pre > 0) {
		/* not the first command */
		if (!usbtoxxx_buffer) {
			LOG_BUG(ERRMSG_INVALID_BUFFER, TO_STR(usbtoxxx_buffer));
			return ERRCODE_INVALID_BUFFER;
		}

		usbtoxxx_buffer[0] = type_pre;
		SET_LE_U16(&usbtoxxx_buffer[1], usbtoxxx_current_cmd_index);

		usbtoxxx_buffer_index += usbtoxxx_current_cmd_index;
	} else {
		/* first command */
		usbtoxxx_buffer_index = 3;
	}

	/* prepare for next command */
	usbtoxxx_current_cmd_index = 3;
	usbtoxxx_buffer = versaloon_buf + usbtoxxx_buffer_index;

	collect_index = 0;
	collect_cmd = 0;

	return ERROR_OK;
}

RESULT usbtoxxx_execute_command(void)
{
	uint16_t i;
	uint16_t inlen;
	RESULT result = ERROR_OK;

	if (poll_nesting) {
		LOG_BUG(ERRMSG_INVALID_USAGE, "USB_TO_POLL");
		versaloon_free_want_pos();
		return ERROR_FAIL;
	}

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		versaloon_free_want_pos();
		return ERRCODE_FAILURE_OPERATION;
	}
	if (usbtoxxx_buffer_index == 3) {
		versaloon_free_want_pos();
		return ERROR_OK;
	}

	versaloon_buf[0] = USB_TO_ALL;
	SET_LE_U16(&versaloon_buf[1], usbtoxxx_buffer_index);

	if (versaloon_send_command(usbtoxxx_buffer_index, &inlen) != ERROR_OK) {
		versaloon_free_want_pos();
		return ERROR_FAIL;
	}

	/* process return data */
	usbtoxxx_buffer_index = 0;
	for (i = 0; i < versaloon_pending_idx; i++) {
		/* check result */
		if ((i == 0) || !((versaloon_pending[i].collect)
				  && (versaloon_pending[i - 1].collect)
				  && (versaloon_pending[i].cmd
				      == versaloon_pending[i - 1].cmd))) {
			if (USB_TO_XXX_CMD_NOT_SUPPORT
					== versaloon_buf[usbtoxxx_buffer_index]) {
				LOG_ERROR(ERRMSG_NOT_SUPPORT_BY,
					usbtoxxx_get_type_name(versaloon_pending[i].type),
					"current dongle");
				result = ERROR_FAIL;
				break;
			} else if (versaloon_buf[usbtoxxx_buffer_index] != USB_TO_XXX_OK) {
				LOG_ERROR("%s command 0x%02x failed with 0x%02x",
					usbtoxxx_get_type_name(versaloon_pending[i].type),
					versaloon_pending[i].cmd,
					versaloon_buf[usbtoxxx_buffer_index]);
				result = ERROR_FAIL;
				break;
			}
			usbtoxxx_buffer_index++;
		}

		/* get result data */
		if (versaloon_pending[i].pos) {
			uint8_t processed = 0;

			if (versaloon_pending[i].callback) {
				versaloon_pending[i].callback(&versaloon_pending[i],
					versaloon_buf + usbtoxxx_buffer_index, &processed);
			}
			if (!processed) {
				struct versaloon_want_pos_t *tmp;

				tmp = versaloon_pending[i].pos;
				while (tmp) {
					if ((tmp->buff) && (tmp->size > 0)) {
						memcpy(tmp->buff,
							versaloon_buf + usbtoxxx_buffer_index
							+ tmp->offset,
							tmp->size);
					}
					struct versaloon_want_pos_t *free_tmp;
					free_tmp = tmp;
					tmp = tmp->next;
					free(free_tmp);
				}
				versaloon_pending[i].pos = NULL;
			}
		} else if ((versaloon_pending[i].want_data_size > 0)
				&& (versaloon_pending[i].data_buffer)) {
			uint8_t processed = 0;

			if (versaloon_pending[i].callback) {
				versaloon_pending[i].callback(&versaloon_pending[i],
					versaloon_buf + usbtoxxx_buffer_index, &processed);
			}
			if (!processed) {
				memcpy(versaloon_pending[i].data_buffer,
					versaloon_buf + usbtoxxx_buffer_index
					+ versaloon_pending[i].want_data_pos,
					versaloon_pending[i].want_data_size);
			}
		}
		usbtoxxx_buffer_index += versaloon_pending[i].actual_data_size;
		if (usbtoxxx_buffer_index > inlen) {
			LOG_BUG("%s command 0x%02x process error",
				usbtoxxx_get_type_name(versaloon_pending[i].type),
				versaloon_pending[i].cmd);
			result = ERROR_FAIL;
			break;
		}
	}

	/* data is not the right size */
	if (inlen != usbtoxxx_buffer_index) {
		LOG_ERROR(ERRMSG_INVALID_TARGET, "length of return data");
		result = ERROR_FAIL;
	}

	if (versaloon_pending_idx > 0)
		versaloon_pending_idx = 0;
	else {
		/* no receive data, avoid collision */
		sleep_ms(10);
	}

	type_pre = 0;
	collect_cmd = 0;
	collect_index = 0;
	versaloon_free_want_pos();
	return result;
}

RESULT usbtoxxx_init(void)
{
	versaloon_pending_idx = 0;

	if ((usbtoinfo_get_abilities(usbtoxxx_abilities) != ERROR_OK) ||
			(usbtoxxx_execute_command() != ERROR_OK))
		return ERROR_FAIL;
	LOG_INFO("USB_TO_XXX abilities: 0x%08X:0x%08X:0x%08X",
		GET_LE_U32(&usbtoxxx_abilities[0]),
		GET_LE_U32(&usbtoxxx_abilities[4]),
		GET_LE_U32(&usbtoxxx_abilities[8]));
	return ERROR_OK;
}

RESULT usbtoxxx_fini(void)
{
	usbtoxxx_buffer = NULL;
	type_pre = 0;
	return ERROR_OK;
}

bool usbtoxxx_interface_supported(uint8_t cmd)
{
	if ((cmd < VERSALOON_USB_TO_XXX_CMD_START) ||
			(cmd > VERSALOON_USB_TO_XXX_CMD_END))
		return false;

	cmd -= VERSALOON_USB_TO_XXX_CMD_START;
	return (usbtoxxx_abilities[cmd  / 8] & (1 << (cmd % 8))) > 0;
}

static RESULT usbtoxxx_ensure_buffer_size(uint16_t cmdlen)
{
	/* check free space, commit if not enough */
	if (((usbtoxxx_buffer_index + usbtoxxx_current_cmd_index + cmdlen)
			>= versaloon_buf_size)
			|| (versaloon_pending_idx >= VERSALOON_MAX_PENDING_NUMBER)) {
		struct usbtoxxx_context_t context_tmp;
		uint8_t poll_nesting_tmp = 0;

		memset(&context_tmp, 0, sizeof(context_tmp));
		if (poll_nesting) {
			if (poll_context.type_pre == 0) {
				LOG_BUG("USB_TO_POLL toooooo long");
				return ERROR_OK;
			}

			usbtoxxx_save_context(&context_tmp);
			usbtoxxx_pop_context(&poll_context);
			poll_nesting_tmp = poll_nesting;
			poll_nesting = 0;
		}

		if (usbtoxxx_execute_command() != ERROR_OK)
			return ERROR_FAIL;

		if (poll_nesting_tmp) {
			uint16_t newlen, oldlen;

			newlen = context_tmp.versaloon_pending_idx
				- poll_context.versaloon_pending_idx;
			memcpy(&versaloon_pending[0],
				&versaloon_pending[poll_context.versaloon_pending_idx],
				sizeof(versaloon_pending[0]) * newlen);
			context_tmp.versaloon_pending_idx = newlen;
			oldlen = poll_context.usbtoxxx_buffer_index
				+ poll_context.usbtoxxx_current_cmd_index;
			newlen = context_tmp.usbtoxxx_buffer_index
				+ context_tmp.usbtoxxx_current_cmd_index;
			memcpy(versaloon_buf + 3, versaloon_buf + oldlen, newlen - oldlen);
			oldlen -= 3;
			context_tmp.usbtoxxx_buffer -= oldlen;
			context_tmp.usbtoxxx_buffer_index -= oldlen;
			usbtoxxx_pop_context(&context_tmp);
			poll_nesting = poll_nesting_tmp;
		}
	}
	return ERROR_OK;
}

RESULT usbtoxxx_add_command(uint8_t type, uint8_t cmd, uint8_t *cmdbuf,
	uint16_t cmdlen, uint16_t retlen, uint8_t *wantbuf,
	uint16_t wantpos, uint16_t wantlen, uint8_t collect)
{
	uint16_t len_tmp;

	/* 3 more bytes by usbtoxxx_validate_current_command_type */
	/* 3 more bytes when ((0 == collect_index) || (collect_cmd != cmd)) */
	if (usbtoxxx_ensure_buffer_size(cmdlen + 6) != ERROR_OK)
		return ERROR_FAIL;

	if ((type_pre != type) || (!usbtoxxx_buffer)) {
		if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
			LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
			return ERRCODE_FAILURE_OPERATION;
		}
		type_pre = type;
	}

	if ((collect_index == 0) || (collect_cmd != cmd)) {
		usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = cmd;

		if (collect) {
			collect_index = usbtoxxx_current_cmd_index;
			collect_cmd = cmd;
		} else {
			collect_index = 0;
			collect_cmd = 0;
		}
		SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], cmdlen);
		usbtoxxx_current_cmd_index += 2;
	} else {
		len_tmp = GET_LE_U16(&usbtoxxx_buffer[collect_index]) + cmdlen;
		SET_LE_U16(&usbtoxxx_buffer[collect_index], len_tmp);
	}

	if (cmdbuf) {
		memcpy(usbtoxxx_buffer + usbtoxxx_current_cmd_index, cmdbuf, cmdlen);
		usbtoxxx_current_cmd_index += cmdlen;
	}

	return versaloon_add_pending(type, cmd, retlen, wantpos, wantlen,
		wantbuf, collect);
}

RESULT usbtoinfo_get_abilities(uint8_t abilities[USB_TO_XXX_ABILITIES_LEN])
{
	if (usbtoxxx_ensure_buffer_size(3) != ERROR_OK)
		return ERROR_FAIL;

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		return ERRCODE_FAILURE_OPERATION;
	}
	type_pre = USB_TO_INFO;

	return versaloon_add_pending(USB_TO_INFO, 0, USB_TO_XXX_ABILITIES_LEN, 0,
		USB_TO_XXX_ABILITIES_LEN, abilities, 0);
}

RESULT usbtopoll_start(uint16_t retry_cnt, uint16_t interval_us)
{
	if (usbtoxxx_ensure_buffer_size(3 + 5) != ERROR_OK)
		return ERROR_FAIL;
	if (!poll_nesting)
		usbtoxxx_save_context(&poll_context);

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		return ERRCODE_FAILURE_OPERATION;
	}
	poll_nesting++;
	type_pre = USB_TO_POLL;

	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = USB_TO_POLL_START;
	SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], retry_cnt);
	usbtoxxx_current_cmd_index += 2;
	SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], interval_us);
	usbtoxxx_current_cmd_index += 2;

	return versaloon_add_pending(USB_TO_POLL, 0, 0, 0, 0, NULL, 0);
}

RESULT usbtopoll_end(void)
{
	if (!poll_nesting) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "check poll nesting");
		return ERRCODE_FAILURE_OPERATION;
	}
	if (usbtoxxx_ensure_buffer_size(3 + 1) != ERROR_OK)
		return ERROR_FAIL;

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		return ERRCODE_FAILURE_OPERATION;
	}

	poll_nesting--;
	type_pre = USB_TO_POLL;

	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = USB_TO_POLL_END;

	return versaloon_add_pending(USB_TO_POLL, 0, 0, 0, 0, NULL, 0);
}

RESULT usbtopoll_checkok(uint8_t equ, uint16_t offset, uint8_t size,
	uint32_t mask, uint32_t value)
{
	uint8_t i;

	if (size > 4) {
		LOG_BUG(ERRMSG_INVALID_PARAMETER, __func__);
		return ERRCODE_INVALID_PARAMETER;
	}
	if (!poll_nesting) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "check poll nesting");
		return ERRCODE_FAILURE_OPERATION;
	}
	if (usbtoxxx_ensure_buffer_size(3 + 4 + 2 * size) != ERROR_OK)
		return ERROR_FAIL;

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		return ERRCODE_FAILURE_OPERATION;
	}

	type_pre = USB_TO_POLL;

	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = USB_TO_POLL_CHECKOK;
	SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], offset);
	usbtoxxx_current_cmd_index += 2;
	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = size;
	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = equ;
	for (i = 0; i < size; i++)
		usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = (mask >> (8 * i)) & 0xFF;
	for (i = 0; i < size; i++)
		usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = (value >> (8 * i)) & 0xFF;

	return ERROR_OK;
}

RESULT usbtopoll_checkfail(uint8_t equ, uint16_t offset, uint8_t size,
	uint32_t mask, uint32_t value)
{
	uint8_t i;

	if (size > 4) {
		LOG_BUG(ERRMSG_INVALID_PARAMETER, __func__);
		return ERRCODE_INVALID_PARAMETER;
	}
	if (!poll_nesting) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "check poll nesting");
		return ERRCODE_FAILURE_OPERATION;
	}
	if (usbtoxxx_ensure_buffer_size(3 + 4 + 2 * size) != ERROR_OK)
		return ERROR_FAIL;

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		return ERRCODE_FAILURE_OPERATION;
	}

	type_pre = USB_TO_POLL;

	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = USB_TO_POLL_CHECKFAIL;
	SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], offset);
	usbtoxxx_current_cmd_index += 2;
	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = size;
	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = equ;
	for (i = 0; i < size; i++)
		usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = (mask >> (8 * i)) & 0xFF;
	for (i = 0; i < size; i++)
		usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = (value >> (8 * i)) & 0xFF;

	return ERROR_OK;
}

RESULT usbtopoll_verifybuff(uint16_t offset, uint16_t size, uint8_t *buff)
{
	if (!poll_nesting) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "check poll nesting");
		return ERRCODE_FAILURE_OPERATION;
	}
	if (usbtoxxx_ensure_buffer_size(3 + 5 + size) != ERROR_OK)
		return ERROR_FAIL;

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		return ERRCODE_FAILURE_OPERATION;
	}

	type_pre = USB_TO_POLL;

	usbtoxxx_buffer[usbtoxxx_current_cmd_index++] = USB_TO_POLL_VERIFYBUFF;
	SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], offset);
	usbtoxxx_current_cmd_index += 2;
	SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], size);
	usbtoxxx_current_cmd_index += 2;
	memcpy(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], buff, size);
	usbtoxxx_current_cmd_index += size;

	return ERROR_OK;
}

RESULT usbtodelay_delay(uint16_t dly)
{
	if (usbtoxxx_ensure_buffer_size(3 + 2) != ERROR_OK)
		return ERROR_FAIL;

	if (usbtoxxx_validate_current_command_type() != ERROR_OK) {
		LOG_BUG(ERRMSG_FAILURE_OPERATION, "validate previous commands");
		return ERRCODE_FAILURE_OPERATION;
	}
	type_pre = USB_TO_DELAY;

	SET_LE_U16(&usbtoxxx_buffer[usbtoxxx_current_cmd_index], dly);
	usbtoxxx_current_cmd_index += 2;

	return versaloon_add_pending(USB_TO_DELAY, 0, 0, 0, 0, NULL, 0);
}

RESULT usbtodelay_delayms(uint16_t ms)
{
	return usbtodelay_delay(ms | 0x8000);
}

RESULT usbtodelay_delayus(uint16_t us)
{
	return usbtodelay_delay(us & 0x7FFF);
}
