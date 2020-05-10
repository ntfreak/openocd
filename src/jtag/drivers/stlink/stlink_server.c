/***************************************************************************
 *   Copyright (C) 2020 by Tarek Bochkati                                  *
 *   Tarek Bochkati <tarek.bouchkati@gmail.com>                            *
 *                                                                         *
 *   SWIM contributions by Ake Rehnman                                     *
 *   Copyright (C) 2017  Ake Rehnman                                       *
 *   ake.rehnman(at)gmail.com                                              *
 *                                                                         *
 *   Copyright (C) 2011-2012 by Mathias Kuester                            *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This code is based on https://github.com/texane/stlink                *
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

#include "stlink.h"
#include <helper/log.h>

#if WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/uio.h>
#include <netinet/tcp.h>
#endif

/** */
static int stlink_server_send_cmd(void *handle, const uint8_t *cmd, int cmd_size,
		uint8_t *data, int data_size, bool check_tcp_status)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_server_priv_s *itf_priv = h->itf->priv;

	assert(handle != NULL);

	/* send the TCP command */
	int sent_size = send(itf_priv->fd, (char *) cmd, cmd_size, 0);
	if (sent_size != cmd_size) {
		LOG_ERROR("failed to send USB CMD");
		if (sent_size == -1)
			LOG_DEBUG("socket send error: %s (errno %d)", strerror(errno), errno);
		else
			LOG_DEBUG("sent size %d (expected %d)", sent_size, cmd_size);
		return ERROR_FAIL;
	}

	keep_alive();

	/* read the TCP response */
	int received_size = recv(itf_priv->fd, (char *) data, data_size, 0);
	if (received_size != data_size) {
		LOG_ERROR("failed to receive USB CMD response");
		if (received_size == -1)
			LOG_DEBUG("socket recv error: %s (errno %d)", strerror(errno), errno);
		else
			LOG_DEBUG("received size %d (expected %d)", received_size, data_size);
		return ERROR_FAIL;
	}

	if (check_tcp_status) {
		uint32_t tcp_ss = le_to_h_u32(data);
		if (tcp_ss != STLINK_TCP_SS_OK) {
			LOG_ERROR("TCP error status 0x%X", tcp_ss);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/** */
static int stlink_server_xfer(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_server_priv_s *itf_priv = h->itf->priv;

	uint8_t cmdbuf[STLINK_TCP_SEND_BUFFER_SIZE];
	uint8_t databuf[STLINK_TCP_RECV_BUFFER_SIZE];
	int cmd_size = STLINK_TCP_USB_CMD_SIZE;
	int data_size = STLINK_TCP_SS_SIZE;

	assert(handle != NULL);

	/* prepare the TCP command */
	cmdbuf[0] = STLINK_TCP_CMD_SEND_USB_CMD;
	memset(&cmdbuf[1], 0, 3); /* reserved for alignment and future use, must be zero */
	h_u32_to_le(&cmdbuf[4], itf_priv->connect_id);
	memcpy(&cmdbuf[8], h->cmdbuf, 16);
	cmdbuf[24] = h->direction;
	memset(&cmdbuf[25], 0, 3);  /* reserved for alignment and future use, must be zero */

	h_u32_to_le(&cmdbuf[28], size);

	/*
	 * if the xfer is a write request (tx_ep)
	 *  > then buf content will be copied
	 * into &cmdbuf[32].
	 * else : the xfer is a read or trace read request (rx_ep or trace_ep)
	 *  > the buf content will be filled from &databuf[4].
	 *
	 * note : if h->direction is trace_ep, h->cmdbuf is zeros.
	 */

	if (h->direction == h->tx_ep) { /* STLINK_TCP_REQUEST_WRITE */
		cmd_size += size;
		if (cmd_size > STLINK_TCP_SEND_BUFFER_SIZE) {
			LOG_ERROR("STLINK_TCP command buffer overflow");
			return ERROR_FAIL;
		}
		memcpy(&cmdbuf[32], buf, size);
	} else { /* STLINK_TCP_REQUEST_READ or STLINK_TCP_REQUEST_READ_SWO */
		data_size += size;
		if (data_size > STLINK_TCP_RECV_BUFFER_SIZE) {
			LOG_ERROR("STLINK_TCP data buffer overflow");
			return ERROR_FAIL;
		}
	}

	int ret = stlink_server_send_cmd(h, cmdbuf, cmd_size, databuf, data_size, true);
	if (ret != ERROR_OK)
		return ret;

	if (h->direction != h->tx_ep)
		memcpy((uint8_t *) buf, &databuf[STLINK_TCP_SS_SIZE], size);

	return ERROR_OK;
}

/** */
static int stlink_server_close(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_server_priv_s *itf_priv = h->itf->priv;

	int ret = ERROR_OK;
	uint8_t cmdbuf[STLINK_TCP_SEND_BUFFER_SIZE];
	uint8_t databuf[STLINK_TCP_RECV_BUFFER_SIZE];

	if (h && itf_priv->connected) {
		if (itf_priv->connect_id) {
			stlink_usb_exit_mode(h);

			/* close the stlink */
			cmdbuf[0] = STLINK_TCP_CMD_CLOSE_DEV;
			memset(&cmdbuf[1], 0, 4); /* reserved */
			h_u32_to_le(&cmdbuf[4], itf_priv->connect_id);
			ret = stlink_server_send_cmd(h, cmdbuf, 8, databuf, 4, true);
			if (ret != ERROR_OK)
				LOG_ERROR("cannot close the STLINK");
		}

		if (close_socket(itf_priv->fd) != 0) {
			LOG_ERROR("error closing the socket");
			LOG_DEBUG("close error: %s (errno %d)", strerror(errno), errno);
		}
	}

	return ret;
}

/** */
static int stlink_server_open(void *handle, struct hl_interface_param_s *param)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_server_priv_s *itf_priv = h->itf->priv;
	int ret;
	uint8_t cmdbuf[STLINK_TCP_SEND_BUFFER_SIZE];
	uint8_t databuf[STLINK_TCP_RECV_BUFFER_SIZE];

	/* SWIM is not supported using stlink-server */
	if (h->st_mode ==  STLINK_MODE_DEBUG_SWIM) {
		LOG_ERROR("stlink-server does not support SWIM mode");
		return ERROR_FAIL;
	}

	/* configure directions */
	h->rx_ep = STLINK_TCP_REQUEST_READ;
	h->tx_ep = STLINK_TCP_REQUEST_WRITE;
	h->trace_ep = STLINK_TCP_REQUEST_READ_SWO;

	itf_priv->fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	itf_priv->connected = false;
	itf_priv->device_id = 0;
	itf_priv->connect_id = 0;

	struct sockaddr_in serv;
	memset(&serv, 0, sizeof(struct sockaddr_in));
	serv.sin_family = AF_INET;
	serv.sin_port = htons(param->stlink_server_port);
	serv.sin_addr.s_addr = inet_addr("127.0.0.1");

	LOG_DEBUG("socket : %x", itf_priv->fd);

	int res;

	int flag = 1;
	res = setsockopt(itf_priv->fd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(flag));
	if (res == -1) {
		LOG_ERROR("cannot set sock option 'TCP_NODELEAY', errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	int a = 49152;
	res = setsockopt(itf_priv->fd, SOL_SOCKET, SO_RCVBUF, (char *) &a, sizeof(a));
	if (res == -1) {
		LOG_ERROR("cannot set sock option 'SO_RCVBUF', errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	res = setsockopt(itf_priv->fd, SOL_SOCKET, SO_SNDBUF, (char *) &a, sizeof(a));
	if (res == -1) {
		LOG_ERROR("cannot set sock option 'SO_SNDBUF', errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	if (connect(itf_priv->fd, (const struct sockaddr *) &serv, sizeof(serv)) == -1) {
		LOG_ERROR("cannot connect to stlink server");
		return ERROR_FAIL;
	}

	itf_priv->connected = true;

	LOG_INFO("connected to stlink-server");

	/* print stlink-server version */
	cmdbuf[0] = STLINK_TCP_CMD_GET_SERVER_VERSION;
	cmdbuf[1] = OPENOCD_STLINK_TCP_API_VERSION;
	memset(&cmdbuf[2], 0, 2); /* reserved */
	ret = stlink_server_send_cmd(h, cmdbuf, 4, databuf, 16, false);
	if (ret != ERROR_OK) {
		LOG_ERROR("cannot get the stlink-server version");
		return ERROR_FAIL;
	}

	uint32_t api_ver = le_to_h_u32(&databuf[0]);
	uint32_t ver_major = le_to_h_u32(&databuf[4]);
	uint32_t ver_minor = le_to_h_u32(&databuf[8]);
	uint32_t ver_build = le_to_h_u32(&databuf[12]);
	LOG_INFO("stlink-server API v%d, version %d.%d.%d",
			api_ver, ver_major, ver_minor, ver_build);

	/* refresh stlink list (re-enumerate) */
	cmdbuf[0] = STLINK_TCP_CMD_REFRESH_DEVICE_LIST;
	cmdbuf[1] = 0; /* don't clear the list, just refresh it */
	ret = stlink_server_send_cmd(h, cmdbuf, 2, databuf, 4, true);
	if (ret != ERROR_OK)
		return ret;

	/* get the number of connected stlinks */
	cmdbuf[0] = STLINK_TCP_CMD_GET_NB_DEV;
	ret = stlink_server_send_cmd(h, cmdbuf, 1, databuf, 4, false);
	if (ret != ERROR_OK)
		return ret;

	uint32_t connected_stlinks = le_to_h_u32(databuf);

	if (connected_stlinks == 0) {
		LOG_ERROR("no ST-LINK detected");
		return ERROR_FAIL;
	}

	LOG_DEBUG("%d ST-LINK detected", connected_stlinks);

	/* list all connected ST-Link and seek for the requested vid:pid and serial */
	uint8_t serial[STLINK_TCP_SERIAL_SIZE+1] = {0};
	uint8_t stlink_used;
	bool stlink_id_matched = false;
	bool stlink_serial_matched = (param->serial == NULL);

	for (uint32_t stlink_id = 0; stlink_id < connected_stlinks; stlink_id++) {
		/* get the stlink info */
		cmdbuf[0] = STLINK_TCP_CMD_GET_DEV_INFO;
		cmdbuf[1] = (uint8_t) stlink_id;
		memset(&cmdbuf[2], 0, 2); /* reserved */
		h_u32_to_le(&cmdbuf[4], 41); /* size of TDeviceInfo2 */
		ret = stlink_server_send_cmd(h, cmdbuf, 8, databuf, 45, true);
		if (ret != ERROR_OK)
			return ret;

		itf_priv->device_id = le_to_h_u32(&databuf[4]);
		memcpy(serial, &databuf[8], STLINK_TCP_SERIAL_SIZE);
		h->vid = le_to_h_u16(&databuf[40]);
		h->pid = le_to_h_u16(&databuf[42]);
		stlink_used = databuf[44];

		/* check the vid:pid */
		for (int i = 0; param->vid[i]; i++) {
			if (param->vid[i] == h->vid && param->pid[i] == h->pid) {
				stlink_id_matched = true;
				break;
			}
		}

		if (!stlink_id_matched)
			continue;

		/* check the serial if specified */
		if (param->serial)
			stlink_serial_matched = strcmp(param->serial, (const char *) serial) == 0;

		if (!stlink_serial_matched)
			LOG_DEBUG("Device serial number '%s' doesn't match requested serial '%s'",
					(const char *) serial, param->serial);
		else /* exit the search loop if there is match */
			break;
	}

	if (!stlink_id_matched) {
		LOG_ERROR("ST-LINK open failed (vid/pid mismatch)");
		return ERROR_FAIL;
	}

	if (!stlink_serial_matched) {
		LOG_ERROR("ST-LINK open failed (serial mismatch)");
		return ERROR_FAIL;
	}

	/* check if device is 'exclusively' used by another application */
	if (stlink_used) {
		LOG_ERROR("the selected device is already used");
		return ERROR_FAIL;
	}

	LOG_DEBUG("transport: vid: 0x%04x pid: 0x%04x serial: %s", h->vid, h->pid, serial);

	/* now let's open the stlink */
	cmdbuf[0] = STLINK_TCP_CMD_OPEN_DEV;
	memset(&cmdbuf[1], 0, 4); /* reserved */
	h_u32_to_le(&cmdbuf[4], itf_priv->device_id);
	ret = stlink_server_send_cmd(h, cmdbuf, 8, databuf, 8, true);
	if (ret != ERROR_OK)
		return ret;

	itf_priv->connect_id = le_to_h_u32(&databuf[4]);

	/* get stlink version */
	ret = stlink_usb_version(h);
	if (ret != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/** */
struct stlink_interface_s stlink_server_itf = {
	.open = stlink_server_open,
	.close = stlink_server_close,
	.xfer = stlink_server_xfer,
	.read = stlink_server_xfer,
};
