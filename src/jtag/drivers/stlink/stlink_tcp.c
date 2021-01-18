/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020 by Antonio Borneo <borneo.antonio@gmail.com>
 * Copyright (C) 2020 by Tarek BOCHKATI <tarek.bouchkati@gmail.com>
 * Copyright (C) 2017 by Ake Rehnman <ake.rehnman@gmail.com>
 * Copyright (C) 2012 by Mathias Kuester <tarek.bouchkati@gmail.com>
 * Copyright (C) 2012 by Spencer Oliver <spen@spen-soft.co.uk>
 *
 * This code is based on https://github.com/texane/stlink
 */

#include "stlink.h"

#include <helper/system.h>

#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif

#ifdef HAVE_NETINET_TCP_H
#include <netinet/tcp.h>
#endif

static int stlink_tcp_send_cmd(void *handle, int send_size, int recv_size, bool check_tcp_status)
{
	struct stlink_handle_s *h = handle;

	assert(handle != NULL);

	/* send the TCP command */
	int sent_size = send(h->tcp_backend_priv.fd, (void *)h->tcp_backend_priv.send_buf, send_size, 0);
	if (sent_size != send_size) {
		LOG_ERROR("failed to send USB CMD");
		if (sent_size == -1)
			LOG_DEBUG("socket send error: %s (errno %d)", strerror(errno), errno);
		else
			LOG_DEBUG("sent size %d (expected %d)", sent_size, send_size);
		return ERROR_FAIL;
	}

	keep_alive();

	/* read the TCP response */
	int received_size = recv(h->tcp_backend_priv.fd, (void *)h->tcp_backend_priv.recv_buf, recv_size, 0);
	if (received_size != recv_size) {
		LOG_ERROR("failed to receive USB CMD response");
		if (received_size == -1)
			LOG_DEBUG("socket recv error: %s (errno %d)", strerror(errno), errno);
		else
			LOG_DEBUG("received size %d (expected %d)", received_size, recv_size);
		return ERROR_FAIL;
	}

	if (check_tcp_status) {
		uint32_t tcp_ss = le_to_h_u32(h->tcp_backend_priv.recv_buf);
		if (tcp_ss != STLINK_TCP_SS_OK) {
			LOG_ERROR("TCP error status 0x%X", tcp_ss);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/** */
static int stlink_tcp_xfer_noerrcheck(void *handle, const uint8_t *buf, int size)
{
	struct stlink_handle_s *h = handle;

	int send_size = STLINK_TCP_USB_CMD_SIZE;
	int recv_size = STLINK_TCP_SS_SIZE;

	assert(handle != NULL);

	/* prepare the TCP command */
	h->tcp_backend_priv.send_buf[0] = STLINK_TCP_CMD_SEND_USB_CMD;
	memset(&h->tcp_backend_priv.send_buf[1], 0, 3); /* reserved for alignment and future use, must be zero */
	h_u32_to_le(&h->tcp_backend_priv.send_buf[4], h->tcp_backend_priv.connect_id);
	/* tcp_backend_priv.send_buf[8..23] already contains the constructed stlink command */
	h->tcp_backend_priv.send_buf[24] = h->direction;
	memset(&h->tcp_backend_priv.send_buf[25], 0, 3);  /* reserved for alignment and future use, must be zero */

	h_u32_to_le(&h->tcp_backend_priv.send_buf[28], size);

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
		send_size += size;
		if (send_size > STLINK_TCP_SEND_BUFFER_SIZE) {
			LOG_ERROR("STLINK_TCP command buffer overflow");
			return ERROR_FAIL;
		}
		memcpy(&h->tcp_backend_priv.send_buf[32], buf, size);
	} else { /* STLINK_TCP_REQUEST_READ or STLINK_TCP_REQUEST_READ_SWO */
		recv_size += size;
		if (recv_size > STLINK_TCP_RECV_BUFFER_SIZE) {
			LOG_ERROR("STLINK_TCP data buffer overflow");
			return ERROR_FAIL;
		}
	}

	int ret = stlink_tcp_send_cmd(h, send_size, recv_size, true);
	if (ret != ERROR_OK)
		return ret;

	if (h->direction != h->tx_ep) {
		/* the read data is located in tcp_backend_priv.recv_buf[4] */
		/* most of the case it will be copying the data from tcp_backend_priv.recv_buf[4]
		 * to handle->cmd_buff which are the same, so let's avoid unnecessary copying */
		if (buf != &h->tcp_backend_priv.recv_buf[4])
			memcpy((uint8_t *)buf, &h->tcp_backend_priv.recv_buf[4], size);
	}

	return ERROR_OK;
}

/** */
static int stlink_tcp_read_trace(void *handle, const uint8_t *buf, int size)
{
	struct stlink_handle_s *h = handle;

	stlink_init_buffer(h, h->trace_ep, 0);
	return stlink_tcp_xfer_noerrcheck(handle, buf, size);
}

/** */
static int stlink_tcp_open(void *handle, struct hl_interface_param_s *param)
{
	struct stlink_handle_s *h = handle;
	int ret;

	/* SWIM is not supported using stlink-server */
	if (h->st_mode ==  STLINK_MODE_DEBUG_SWIM) {
		LOG_ERROR("stlink-server does not support SWIM mode");
		return ERROR_FAIL;
	}

	h->tcp_backend_priv.send_buf = malloc(STLINK_TCP_SEND_BUFFER_SIZE);
	h->tcp_backend_priv.recv_buf = malloc(STLINK_TCP_RECV_BUFFER_SIZE);

	if (h->tcp_backend_priv.send_buf == NULL || h->tcp_backend_priv.recv_buf == NULL)
		return ERROR_FAIL;

	h->cmdbuf = &h->tcp_backend_priv.send_buf[8];
	h->databuf = &h->tcp_backend_priv.recv_buf[4];

	/* configure directions */
	h->rx_ep = STLINK_TCP_REQUEST_READ;
	h->tx_ep = STLINK_TCP_REQUEST_WRITE;
	h->trace_ep = STLINK_TCP_REQUEST_READ_SWO;

	h->tcp_backend_priv.fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	h->tcp_backend_priv.connected = false;
	h->tcp_backend_priv.device_id = 0;
	h->tcp_backend_priv.connect_id = 0;

	if (h->tcp_backend_priv.fd == -1) {
		LOG_ERROR("error creating the socket, errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	struct sockaddr_in serv;
	memset(&serv, 0, sizeof(struct sockaddr_in));
	serv.sin_family = AF_INET;
	serv.sin_port = htons(param->stlink_tcp_port);
	serv.sin_addr.s_addr = inet_addr("127.0.0.1");

	LOG_DEBUG("socket : %x", h->tcp_backend_priv.fd);

	int optval = 1;
	if (setsockopt(h->tcp_backend_priv.fd, IPPROTO_TCP, TCP_NODELAY, (const void *)&optval, sizeof(int)) == -1) {
		LOG_ERROR("cannot set sock option 'TCP_NODELAY', errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	optval = STLINK_TCP_RECV_BUFFER_SIZE;
	if (setsockopt(h->tcp_backend_priv.fd, SOL_SOCKET, SO_RCVBUF, (const void *)&optval, sizeof(int)) == -1) {
		LOG_ERROR("cannot set sock option 'SO_RCVBUF', errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	optval = STLINK_TCP_SEND_BUFFER_SIZE;
	if (setsockopt(h->tcp_backend_priv.fd, SOL_SOCKET, SO_SNDBUF, (const void *)&optval, sizeof(int)) == -1) {
		LOG_ERROR("cannot set sock option 'SO_SNDBUF', errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	if (connect(h->tcp_backend_priv.fd, (const struct sockaddr *)&serv, sizeof(serv)) == -1) {
		LOG_ERROR("cannot connect to stlink server, errno: %s", strerror(errno));
		return ERROR_FAIL;
	}

	h->tcp_backend_priv.connected = true;

	LOG_INFO("connected to stlink-server");

	/* print stlink-server version */
	h->tcp_backend_priv.send_buf[0] = STLINK_TCP_CMD_GET_SERVER_VERSION;
	h->tcp_backend_priv.send_buf[1] = OPENOCD_STLINK_TCP_API_VERSION;
	memset(&h->tcp_backend_priv.send_buf[2], 0, 2); /* reserved */
	ret = stlink_tcp_send_cmd(h, 4, 16, false);
	if (ret != ERROR_OK) {
		LOG_ERROR("cannot get the stlink-server version");
		return ERROR_FAIL;
	}

	uint32_t api_ver = le_to_h_u32(&h->tcp_backend_priv.recv_buf[0]);
	uint32_t ver_major = le_to_h_u32(&h->tcp_backend_priv.recv_buf[4]);
	uint32_t ver_minor = le_to_h_u32(&h->tcp_backend_priv.recv_buf[8]);
	uint32_t ver_build = le_to_h_u32(&h->tcp_backend_priv.recv_buf[12]);
	LOG_INFO("stlink-server API v%d, version %d.%d.%d",
			api_ver, ver_major, ver_minor, ver_build);

	/* in stlink-server API v1 sending more than 1428 bytes will cause stlink-server
	 * to crash in windows: select a safe default value (1K) */
	if (api_ver < 2)
		h->max_mem_packet = (1 << 10);

	/* refresh stlink list (re-enumerate) */
	h->tcp_backend_priv.send_buf[0] = STLINK_TCP_CMD_REFRESH_DEVICE_LIST;
	h->tcp_backend_priv.send_buf[1] = 0; /* don't clear the list, just refresh it */
	ret = stlink_tcp_send_cmd(h, 2, 4, true);
	if (ret != ERROR_OK)
		return ret;

	/* get the number of connected stlinks */
	h->tcp_backend_priv.send_buf[0] = STLINK_TCP_CMD_GET_NB_DEV;
	ret = stlink_tcp_send_cmd(h, 1, 4, false);
	if (ret != ERROR_OK)
		return ret;

	uint32_t connected_stlinks = le_to_h_u32(h->tcp_backend_priv.recv_buf);

	if (connected_stlinks == 0) {
		LOG_ERROR("no ST-LINK detected");
		return ERROR_FAIL;
	}

	LOG_DEBUG("%d ST-LINK detected", connected_stlinks);

	if (connected_stlinks > 255) {
		LOG_WARNING("STLink server cannot handle more than 255 ST-LINK connected");
		connected_stlinks = 255;
	}

	/* list all connected ST-Link and seek for the requested vid:pid and serial */
	char serial[STLINK_TCP_SERIAL_SIZE + 1] = {0};
	uint8_t stlink_used;
	bool stlink_id_matched = false;
	bool stlink_serial_matched = (param->serial == NULL);

	for (uint32_t stlink_id = 0; stlink_id < connected_stlinks; stlink_id++) {
		/* get the stlink info */
		h->tcp_backend_priv.send_buf[0] = STLINK_TCP_CMD_GET_DEV_INFO;
		h->tcp_backend_priv.send_buf[1] = (uint8_t)stlink_id;
		memset(&h->tcp_backend_priv.send_buf[2], 0, 2); /* reserved */
		h_u32_to_le(&h->tcp_backend_priv.send_buf[4], 41); /* size of TDeviceInfo2 */
		ret = stlink_tcp_send_cmd(h, 8, 45, true);
		if (ret != ERROR_OK)
			return ret;

		h->tcp_backend_priv.device_id = le_to_h_u32(&h->tcp_backend_priv.recv_buf[4]);
		memcpy(serial, &h->tcp_backend_priv.recv_buf[8], STLINK_TCP_SERIAL_SIZE);
		h->vid = le_to_h_u16(&h->tcp_backend_priv.recv_buf[40]);
		h->pid = le_to_h_u16(&h->tcp_backend_priv.recv_buf[42]);
		stlink_used = h->tcp_backend_priv.recv_buf[44];

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
		if (param->serial) {
			/* ST-Link server fixes the buggy serial returned by old ST-Link DFU
			 * for further details refer to stlink_usb_get_alternate_serial
			 * so if the user passes the buggy serial, we need to fix it before
			 * comparing with the serial returned by ST-Link server */
			if (strlen(param->serial) == STLINK_SERIAL_LEN / 2) {
				char fixed_serial[STLINK_SERIAL_LEN + 1];

				for (unsigned int i = 0; i < STLINK_SERIAL_LEN; i += 2)
					sprintf(fixed_serial + i, "%02X", param->serial[i / 2]);

				fixed_serial[STLINK_SERIAL_LEN] = '\0';

				stlink_serial_matched = strcmp(fixed_serial, serial) == 0;
			} else
				stlink_serial_matched = strcmp(param->serial, serial) == 0;
		}

		if (!stlink_serial_matched)
			LOG_DEBUG("Device serial number '%s' doesn't match requested serial '%s'",
					serial, param->serial);
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
	h->tcp_backend_priv.send_buf[0] = STLINK_TCP_CMD_OPEN_DEV;
	memset(&h->tcp_backend_priv.send_buf[1], 0, 4); /* reserved */
	h_u32_to_le(&h->tcp_backend_priv.send_buf[4], h->tcp_backend_priv.device_id);
	ret = stlink_tcp_send_cmd(h, 8, 8, true);
	if (ret != ERROR_OK)
		return ret;

	h->tcp_backend_priv.connect_id = le_to_h_u32(&h->tcp_backend_priv.recv_buf[4]);

	/* get stlink version */
	return stlink_version(h);
}

/** */
static int stlink_tcp_close(void *handle)
{
	struct stlink_handle_s *h = handle;

	if (!h)
		return ERROR_OK;

	int ret = ERROR_OK;
	if (h->tcp_backend_priv.connected) {
		if (h->tcp_backend_priv.connect_id) {
			stlink_exit_mode(h);

			/* close the stlink */
			h->tcp_backend_priv.send_buf[0] = STLINK_TCP_CMD_CLOSE_DEV;
			memset(&h->tcp_backend_priv.send_buf[1], 0, 4); /* reserved */
			h_u32_to_le(&h->tcp_backend_priv.send_buf[4], h->tcp_backend_priv.connect_id);
			ret = stlink_tcp_send_cmd(h, 8, 4, true);
			if (ret != ERROR_OK)
				LOG_ERROR("cannot close the STLINK");
		}

		if (close_socket(h->tcp_backend_priv.fd) != 0)
			LOG_ERROR("error closing the socket, errno: %s", strerror(errno));
	}

	free(h->tcp_backend_priv.send_buf);
	free(h->tcp_backend_priv.recv_buf);

	return ret;
}

struct stlink_backend_s stlink_tcp_backend = {
	.open = stlink_tcp_open,
	.close = stlink_tcp_close,
	.xfer_noerrcheck = stlink_tcp_xfer_noerrcheck,
	.read_trace = stlink_tcp_read_trace,
};
