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

#include <stdio.h>
#include <string.h>
#include <libusb.h>

#include "versaloon_include.h"
#include "versaloon.h"
#include "versaloon_internal.h"
#include "usbtoxxx/usbtoxxx.h"

uint8_t *versaloon_buf;
uint8_t *versaloon_cmd_buf;
uint16_t versaloon_buf_size;

struct versaloon_pending_t versaloon_pending[VERSALOON_MAX_PENDING_NUMBER];
uint16_t versaloon_pending_idx;

libusb_device_handle *versaloon_usb_device_handle;
static uint32_t versaloon_usb_to = VERSALOON_TIMEOUT;

static RESULT versaloon_init(void);
static RESULT versaloon_fini(void);
static RESULT versaloon_get_target_voltage(uint16_t *voltage);
static RESULT versaloon_set_target_voltage(uint16_t voltage);
static RESULT versaloon_delay_ms(uint16_t ms);
static RESULT versaloon_delay_us(uint16_t us);

struct versaloon_interface_t versaloon_interface = {
	.init				= versaloon_init,
	.fini				= versaloon_fini,
	{	/* adaptors */
		{	/* target_voltage */
			.get		= versaloon_get_target_voltage,
			.set		= versaloon_set_target_voltage,
		},
		{	/* gpio */
			.init		= usbtogpio_init,
			.fini		= usbtogpio_fini,
			.config		= usbtogpio_config,
			.out		= usbtogpio_out,
			.in			= usbtogpio_in,
		},
		{	/* delay */
			.delayms	= versaloon_delay_ms,
			.delayus	= versaloon_delay_us,
		},
		{	/* swd */
			.init		= usbtoswd_init,
			.fini		= usbtoswd_fini,
			.config		= usbtoswd_config,
			.seqout		= usbtoswd_seqout,
			.seqin		= usbtoswd_seqin,
			.transact	= usbtoswd_transact,
		},
		{	/* jtag_raw */
			.init		= usbtojtagraw_init,
			.fini		= usbtojtagraw_fini,
			.config		= usbtojtagraw_config,
			.execute	= usbtojtagraw_execute,
		},
		.peripheral_commit = usbtoxxx_execute_command,
	},
	{	/* usb_setting */
		.vid			= VERSALOON_VID,
		.pid			= VERSALOON_PID,
		.ep_out			= VERSALOON_OUTP,
		.ep_in			= VERSALOON_INP,
		.interface		= VERSALOON_IFACE,
		.serialstring	= NULL,
		.buf_size		= 256,
	}
};

/* programmer_cmd */
static uint32_t versaloon_pending_id;
static versaloon_callback_t versaloon_callback;
static void *versaloon_extra_data;
static struct versaloon_want_pos_t *versaloon_want_pos;

void versaloon_set_pending_id(uint32_t id)
{
	versaloon_pending_id = id;
}
void versaloon_set_callback(versaloon_callback_t callback)
{
	versaloon_callback = callback;
}
void versaloon_set_extra_data(void *p)
{
	versaloon_extra_data = p;
}

void versaloon_free_want_pos(void)
{
	uint16_t i;
	struct versaloon_want_pos_t *tmp, *free_tmp;

	tmp = versaloon_want_pos;
	while (tmp != NULL) {
		free_tmp = tmp;
		tmp = tmp->next;
		free(free_tmp);
	}
	versaloon_want_pos = NULL;

	for (i = 0; i < dimof(versaloon_pending); i++) {
		tmp = versaloon_pending[i].pos;
		while (tmp != NULL) {
			free_tmp = tmp;
			tmp = tmp->next;
			free(free_tmp);
		}
		versaloon_pending[i].pos = NULL;
	}
}

RESULT versaloon_add_want_pos(uint16_t offset, uint16_t size, uint8_t *buff)
{
	struct versaloon_want_pos_t *new_pos = NULL;

	new_pos = malloc(sizeof(*new_pos));
	if (NULL == new_pos) {
		LOG_ERROR(ERRMSG_NOT_ENOUGH_MEMORY);
		return ERRCODE_NOT_ENOUGH_MEMORY;
	}
	new_pos->offset = offset;
	new_pos->size = size;
	new_pos->buff = buff;
	new_pos->next = NULL;

	if (NULL == versaloon_want_pos)
		versaloon_want_pos = new_pos;
	else {
		struct versaloon_want_pos_t *tmp = versaloon_want_pos;

		while (tmp->next != NULL)
			tmp = tmp->next;
		tmp->next = new_pos;
	}

	return ERROR_OK;
}

RESULT versaloon_add_pending(uint8_t type, uint8_t cmd, uint16_t actual_szie,
	uint16_t want_pos, uint16_t want_size, uint8_t *buffer, uint8_t collect)
{
#if PARAM_CHECK
	if (versaloon_pending_idx >= VERSALOON_MAX_PENDING_NUMBER) {
		LOG_BUG(ERRMSG_INVALID_INDEX, versaloon_pending_idx,
			"versaloon pending data");
		return ERROR_FAIL;
	}
#endif

	versaloon_pending[versaloon_pending_idx].type = type;
	versaloon_pending[versaloon_pending_idx].cmd = cmd;
	versaloon_pending[versaloon_pending_idx].actual_data_size = actual_szie;
	versaloon_pending[versaloon_pending_idx].want_data_pos = want_pos;
	versaloon_pending[versaloon_pending_idx].want_data_size = want_size;
	versaloon_pending[versaloon_pending_idx].data_buffer = buffer;
	versaloon_pending[versaloon_pending_idx].collect = collect;
	versaloon_pending[versaloon_pending_idx].id = versaloon_pending_id;
	versaloon_pending_id = 0;
	versaloon_pending[versaloon_pending_idx].extra_data = versaloon_extra_data;
	versaloon_extra_data = NULL;
	versaloon_pending[versaloon_pending_idx].callback = versaloon_callback;
	versaloon_callback = NULL;
	versaloon_pending[versaloon_pending_idx].pos = versaloon_want_pos;
	versaloon_want_pos = NULL;
	versaloon_pending_idx++;

	return ERROR_OK;
}

RESULT versaloon_send_command(uint16_t out_len, uint16_t *inlen)
{
	int ret;
	int transferred;

#if PARAM_CHECK
	if (NULL == versaloon_buf) {
		LOG_BUG(ERRMSG_INVALID_BUFFER, TO_STR(versaloon_buf));
		return ERRCODE_INVALID_BUFFER;
	}
	if ((0 == out_len) || (out_len > versaloon_interface.usb_setting.buf_size)) {
		LOG_BUG(ERRMSG_INVALID_PARAMETER, __func__);
		return ERRCODE_INVALID_PARAMETER;
	}
#endif

	ret = libusb_bulk_transfer(versaloon_usb_device_handle,
			versaloon_interface.usb_setting.ep_out,
			versaloon_buf, out_len, &transferred, versaloon_usb_to);
	if (0 != ret || transferred != out_len) {
		LOG_ERROR(ERRMSG_FAILURE_OPERATION, "send usb data");
		return ERRCODE_FAILURE_OPERATION;
	}

	if (inlen != NULL) {
		ret = libusb_bulk_transfer(versaloon_usb_device_handle,
			versaloon_interface.usb_setting.ep_in,
			versaloon_buf, versaloon_interface.usb_setting.buf_size,
			&transferred, versaloon_usb_to);
		if (0 == ret) {
			*inlen = (uint16_t)transferred;
			return ERROR_OK;
		} else {
			LOG_ERROR(ERRMSG_FAILURE_OPERATION, "receive usb data");
			return ERROR_FAIL;
		}
	} else
		return ERROR_OK;
}

#define VERSALOON_RETRY_CNT 10
static RESULT versaloon_init(void)
{
	uint16_t ret = 0;
	uint8_t retry;
	uint32_t timeout_tmp;

	/* malloc temporary buffer */
	versaloon_buf = malloc(versaloon_interface.usb_setting.buf_size);
	if (NULL == versaloon_buf) {
		LOG_ERROR(ERRMSG_NOT_ENOUGH_MEMORY);
		return ERRCODE_NOT_ENOUGH_MEMORY;
	}

	/* connect to versaloon */
	timeout_tmp = versaloon_usb_to;
	/* not output error message when connecting */
	/* 100ms delay when connect */
	versaloon_usb_to = 100;
	for (retry = 0; retry < VERSALOON_RETRY_CNT; retry++) {
		versaloon_buf[0] = VERSALOON_GET_INFO;
		if ((ERROR_OK == versaloon_send_command(1, &ret)) && (ret >= 3))
			break;
	}
	versaloon_usb_to = timeout_tmp;
	if (VERSALOON_RETRY_CNT == retry) {
		versaloon_fini();
		LOG_ERROR(ERRMSG_FAILURE_OPERATION, "communicate with versaloon");
		return ERRCODE_FAILURE_OPERATION;
	}

	versaloon_buf[ret] = 0;
	versaloon_buf_size = versaloon_buf[0] + (versaloon_buf[1] << 8);
	versaloon_interface.usb_setting.buf_size = versaloon_buf_size;
	LOG_INFO("%s", versaloon_buf + 2);

	/* free temporary buffer */
	free(versaloon_buf);
	versaloon_buf = NULL;

	versaloon_buf = malloc(versaloon_interface.usb_setting.buf_size);
	if (NULL == versaloon_buf) {
		versaloon_fini();
		LOG_ERROR(ERRMSG_NOT_ENOUGH_MEMORY);
		return ERRCODE_NOT_ENOUGH_MEMORY;
	}
	versaloon_cmd_buf = malloc(versaloon_interface.usb_setting.buf_size - 3);
	if (NULL == versaloon_cmd_buf) {
		versaloon_fini();
		LOG_ERROR(ERRMSG_NOT_ENOUGH_MEMORY);
		return ERRCODE_NOT_ENOUGH_MEMORY;
	}
	if (ERROR_OK != usbtoxxx_init()) {
		LOG_ERROR(ERRMSG_FAILURE_OPERATION, "initialize usbtoxxx");
		return ERROR_FAIL;
	}
	return versaloon_get_target_voltage(&ret);
}

static RESULT versaloon_fini(void)
{
	if (versaloon_usb_device_handle != NULL) {
		usbtoxxx_fini();
		versaloon_free_want_pos();

		versaloon_usb_device_handle = NULL;

		free(versaloon_buf);
		versaloon_buf = NULL;

		free(versaloon_cmd_buf);
		versaloon_cmd_buf = NULL;
	}

	return ERROR_OK;
}

static RESULT versaloon_set_target_voltage(uint16_t voltage)
{
	usbtopwr_init(0);
	usbtopwr_config(0);
	usbtopwr_output(0, voltage);
	usbtopwr_fini(0);

	return usbtoxxx_execute_command();
}

static RESULT versaloon_get_target_voltage(uint16_t *voltage)
{
	uint16_t inlen;

#if PARAM_CHECK
	if (NULL == versaloon_buf) {
		LOG_BUG(ERRMSG_INVALID_BUFFER, TO_STR(versaloon_buf));
		return ERRCODE_INVALID_BUFFER;
	}
	if (NULL == voltage) {
		LOG_BUG(ERRMSG_INVALID_PARAMETER, __func__);
		return ERRCODE_INVALID_PARAMETER;
	}
#endif

	versaloon_buf[0] = VERSALOON_GET_TVCC;

	if ((ERROR_OK != versaloon_send_command(1, &inlen)) || (inlen != 2)) {
		LOG_ERROR(ERRMSG_FAILURE_OPERATION, "communicate with versaloon");
		return ERRCODE_FAILURE_OPERATION;
	} else {
		*voltage = versaloon_buf[0] + (versaloon_buf[1] << 8);
		return ERROR_OK;
	}
}

static RESULT versaloon_delay_ms(uint16_t ms)
{
	return usbtodelay_delay(ms | 0x8000);
}

static RESULT versaloon_delay_us(uint16_t us)
{
	return usbtodelay_delay(us & 0x7FFF);
}
