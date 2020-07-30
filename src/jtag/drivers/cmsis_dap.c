/***************************************************************************
 *   Copyright (C) 2018 by Mickaël Thomas                                  *
 *   mickael9@gmail.com                                                    *
 *                                                                         *
 *   Copyright (C) 2016 by Maksym Hilliaka                                 *
 *   oter@frozen-team.com                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Phillip Pearson                                 *
 *   pp@myelin.co.nz                                                       *
 *                                                                         *
 *   Copyright (C) 2014 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2013 by mike brown                                      *
 *   mike@theshedworks.org.uk                                              *
 *                                                                         *
 *   Copyright (C) 2013 by Spencer Oliver                                  *
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

#include <transport/transport.h>
#include <jtag/swd.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/tcl.h>

#include "cmsis_dap.h"

static const struct cmsis_dap_backend *const cmsis_dap_backends[] = {
#if BUILD_CMSIS_DAP_USB == 1
	&cmsis_dap_usb_backend,
#endif

#if BUILD_CMSIS_DAP_HID == 1
	&cmsis_dap_hid_backend,
#endif
};

/* USB Config */

/* Known vid/pid pairs:
 * VID 0xc251: Keil Software
 * PID 0xf001: LPC-Link-II CMSIS_DAP
 * PID 0xf002: OPEN-SDA CMSIS_DAP (Freedom Board)
 * PID 0x2722: Keil ULINK2 CMSIS-DAP
 * PID 0x2750: Keil ULINKplus CMSIS-DAP
 *
 * VID 0x0d28: mbed Software
 * PID 0x0204: MBED CMSIS-DAP
 */

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t cmsis_dap_vid[MAX_USB_IDS + 1] = { 0 };
static uint16_t cmsis_dap_pid[MAX_USB_IDS + 1] = { 0 };
static char *cmsis_dap_serial;
static int cmsis_dap_backend = -1;
static bool swd_mode;

#define USB_TIMEOUT       1000

/* CMSIS-DAP General Commands */
#define CMD_DAP_INFO              0x00
#define CMD_DAP_LED               0x01
#define CMD_DAP_CONNECT           0x02
#define CMD_DAP_DISCONNECT        0x03
#define CMD_DAP_WRITE_ABORT       0x08
#define CMD_DAP_DELAY             0x09
#define CMD_DAP_RESET_TARGET      0x0A

/* CMD_INFO */
#define INFO_ID_VENDOR            0x01      /* string */
#define INFO_ID_PRODUCT           0x02      /* string */
#define INFO_ID_SERNUM            0x03      /* string */
#define INFO_ID_FW_VER            0x04      /* string */
#define INFO_ID_TD_VEND           0x05      /* string */
#define INFO_ID_TD_NAME           0x06      /* string */
#define INFO_ID_CAPS              0xf0      /* byte */
#define INFO_ID_PKT_CNT           0xfe      /* byte */
#define INFO_ID_PKT_SZ            0xff      /* short */

#define INFO_CAPS_SWD             0x01
#define INFO_CAPS_JTAG            0x02

/* CMD_LED */
#define LED_ID_CONNECT            0x00
#define LED_ID_RUN                0x01

#define LED_OFF                   0x00
#define LED_ON                    0x01

/* CMD_CONNECT */
#define CONNECT_DEFAULT           0x00
#define CONNECT_SWD               0x01
#define CONNECT_JTAG              0x02

/* CMSIS-DAP Common SWD/JTAG Commands */
#define CMD_DAP_DELAY             0x09
#define CMD_DAP_SWJ_PINS          0x10
#define CMD_DAP_SWJ_CLOCK         0x11
#define CMD_DAP_SWJ_SEQ           0x12

/*
 * PINS
 * Bit 0: SWCLK/TCK
 * Bit 1: SWDIO/TMS
 * Bit 2: TDI
 * Bit 3: TDO
 * Bit 5: nTRST
 * Bit 7: nRESET
 */

#define SWJ_PIN_TCK               (1<<0)
#define SWJ_PIN_TMS               (1<<1)
#define SWJ_PIN_TDI               (1<<2)
#define SWJ_PIN_TDO               (1<<3)
#define SWJ_PIN_TRST              (1<<5)
#define SWJ_PIN_SRST              (1<<7)

/* CMSIS-DAP SWD Commands */
#define CMD_DAP_SWD_CONFIGURE     0x13

/* CMSIS-DAP JTAG Commands */
#define CMD_DAP_JTAG_SEQ          0x14
#define CMD_DAP_JTAG_CONFIGURE    0x15
#define CMD_DAP_JTAG_IDCODE       0x16

/* CMSIS-DAP JTAG sequence info masks */
/* Number of bits to clock through (0 means 64) */
#define DAP_JTAG_SEQ_TCK          0x3F
/* TMS will be set during the sequence if this bit is set */
#define DAP_JTAG_SEQ_TMS          0x40
/* TDO output will be captured if this bit is set */
#define DAP_JTAG_SEQ_TDO          0x80


/* CMSIS-DAP Transfer Commands */
#define CMD_DAP_TFER_CONFIGURE    0x04
#define CMD_DAP_TFER              0x05
#define CMD_DAP_TFER_BLOCK        0x06
#define CMD_DAP_TFER_ABORT        0x07

/* DAP Status Code */
#define DAP_OK                    0
#define DAP_ERROR                 0xFF

/* CMSIS-DAP Vendor Commands
 * None as yet... */

static const char * const info_caps_str[] = {
	"SWD  Supported",
	"JTAG Supported"
};

/* max clock speed (kHz) */
#define DAP_MAX_CLOCK             5000


struct pending_transfer_result {
	uint8_t cmd;
	uint32_t data;
	void *buffer;
};

struct pending_request_block {
	struct pending_transfer_result *transfers;
	int transfer_count;
};

struct pending_scan_result {
	/** Offset in bytes in the CMD_DAP_JTAG_SEQ response buffer. */
	unsigned first;
	/** Number of bits to read. */
	unsigned length;
	/** Location to store the result */
	uint8_t *buffer;
	/** Offset in the destination buffer */
	unsigned buffer_offset;
};

/* Up to MIN(packet_count, MAX_PENDING_REQUESTS) requests may be issued
 * until the first response arrives */
#define MAX_PENDING_REQUESTS 3

/* Pending requests are organized as a FIFO - circular buffer */
/* Each block in FIFO can contain up to pending_queue_len transfers */
static int pending_queue_len;
static struct pending_request_block pending_fifo[MAX_PENDING_REQUESTS];
static int pending_fifo_put_idx, pending_fifo_get_idx;
static int pending_fifo_block_count;

/* pointers to buffers that will receive jtag scan results on the next flush */
#define MAX_PENDING_SCAN_RESULTS 256
static int pending_scan_result_count;
static struct pending_scan_result pending_scan_results[MAX_PENDING_SCAN_RESULTS];

/* queued JTAG sequences that will be executed on the next flush */
#define QUEUED_SEQ_BUF_LEN (cmsis_dap_handle->packet_size - 3)
static int queued_seq_count;
static int queued_seq_buf_end;
static int queued_seq_tdo_ptr;
static uint8_t queued_seq_buf[1024]; /* TODO: make dynamic / move into cmsis object */

static int queued_retval;

static uint8_t output_pins = SWJ_PIN_SRST | SWJ_PIN_TRST;

static struct cmsis_dap *cmsis_dap_handle;


static int cmsis_dap_open(void)
{
	const struct cmsis_dap_backend *backend = NULL;

	struct cmsis_dap *dap = malloc(sizeof(struct cmsis_dap));
	if (dap == NULL) {
		LOG_ERROR("unable to allocate memory");
		return ERROR_FAIL;
	}

	dap->caps = 0;
	dap->mode = 0;
	dap->packet_size = 0; /* initialized by backend */

	if (cmsis_dap_backend >= 0) {
		/* Use forced backend */
		backend = cmsis_dap_backends[cmsis_dap_backend];
		if (backend->open(dap, cmsis_dap_vid, cmsis_dap_pid, cmsis_dap_serial) != ERROR_OK)
			backend = NULL;
	} else {
		/* Try all backends */
		for (unsigned int i = 0; i < ARRAY_SIZE(cmsis_dap_backends); i++) {
			backend = cmsis_dap_backends[i];
			if (backend->open(dap, cmsis_dap_vid, cmsis_dap_pid, cmsis_dap_serial) == ERROR_OK)
				break;
			else
				backend = NULL;
		}
	}

	if (backend == NULL) {
		LOG_ERROR("unable to find a matching CMSIS-DAP device");
		free(dap);
		return ERROR_FAIL;
	}

	assert(dap->packet_size > 0);

	dap->backend = backend;
	dap->packet_buffer = malloc(dap->packet_size);

	if (dap->packet_buffer == NULL) {
		LOG_ERROR("unable to allocate memory");
		dap->backend->close(dap);
		free(dap);
		return ERROR_FAIL;
	}

	cmsis_dap_handle = dap;

	return ERROR_OK;
}

static void cmsis_dap_close(struct cmsis_dap *dap)
{
	if (dap->backend) {
		dap->backend->close(dap);
		dap->backend = NULL;
	}

	free(cmsis_dap_handle->packet_buffer);
	free(cmsis_dap_handle);
	cmsis_dap_handle = NULL;
	free(cmsis_dap_serial);
	cmsis_dap_serial = NULL;

	for (int i = 0; i < MAX_PENDING_REQUESTS; i++) {
		free(pending_fifo[i].transfers);
		pending_fifo[i].transfers = NULL;
	}
}

/* Send a message and receive the reply */
static int cmsis_dap_xfer(struct cmsis_dap *dap, int txlen)
{
	if (pending_fifo_block_count) {
		LOG_ERROR("pending %d blocks, flushing", pending_fifo_block_count);
		while (pending_fifo_block_count) {
			dap->backend->read(dap, 10);
			pending_fifo_block_count--;
		}
		pending_fifo_put_idx = 0;
		pending_fifo_get_idx = 0;
	}

	int retval = dap->backend->write(dap, txlen, USB_TIMEOUT);
	if (retval < 0)
		return retval;

	/* get reply */
	retval = dap->backend->read(dap, USB_TIMEOUT);
	if (retval < 0)
		return retval;

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_SWJ_Pins(uint8_t pins, uint8_t mask, uint32_t delay, uint8_t *input)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWJ_PINS;
	buffer[2] = pins;
	buffer[3] = mask;
	buffer[4] = delay & 0xff;
	buffer[5] = (delay >> 8) & 0xff;
	buffer[6] = (delay >> 16) & 0xff;
	buffer[7] = (delay >> 24) & 0xff;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 8);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_PINS failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (input)
		*input = buffer[1];

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_SWJ_Clock(uint32_t swj_clock)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	/* set clock in Hz */
	swj_clock *= 1000;
	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWJ_CLOCK;
	buffer[2] = swj_clock & 0xff;
	buffer[3] = (swj_clock >> 8) & 0xff;
	buffer[4] = (swj_clock >> 16) & 0xff;
	buffer[5] = (swj_clock >> 24) & 0xff;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 6);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_CLOCK failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/* clock a sequence of bits out on TMS, to change JTAG states */
static int cmsis_dap_cmd_DAP_SWJ_Sequence(uint8_t s_len, const uint8_t *sequence)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

#ifdef CMSIS_DAP_JTAG_DEBUG
	LOG_DEBUG("cmsis-dap TMS sequence: len=%d", s_len);
	for (int i = 0; i < DIV_ROUND_UP(s_len, 8); ++i)
		printf("%02X ", sequence[i]);

	printf("\n");
#endif

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWJ_SEQ;
	buffer[2] = s_len;
	bit_copy(&buffer[3], 0, sequence, 0, s_len);

	retval = cmsis_dap_xfer(cmsis_dap_handle, DIV_ROUND_UP(s_len, 8) + 3);

	if (retval != ERROR_OK || buffer[1] != DAP_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_Info(uint8_t info, uint8_t **data)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_INFO;
	buffer[2] = info;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 3);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_INFO failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	*data = &(buffer[1]);

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_LED(uint8_t led, uint8_t state)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_LED;
	buffer[2] = led;
	buffer[3] = state;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 4);

	if (retval != ERROR_OK || buffer[1] != 0x00) {
		LOG_ERROR("CMSIS-DAP command CMD_LED failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_Connect(uint8_t mode)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_CONNECT;
	buffer[2] = mode;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 3);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_CONNECT failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (buffer[1] != mode) {
		LOG_ERROR("CMSIS-DAP failed to connect in mode (%d)", mode);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_Disconnect(void)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_DISCONNECT;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 2);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DISCONNECT failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_TFER_Configure(uint8_t idle, uint16_t retry_count, uint16_t match_retry)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_TFER_CONFIGURE;
	buffer[2] = idle;
	buffer[3] = retry_count & 0xff;
	buffer[4] = (retry_count >> 8) & 0xff;
	buffer[5] = match_retry & 0xff;
	buffer[6] = (match_retry >> 8) & 0xff;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 7);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_TFER_Configure failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_DAP_SWD_Configure(uint8_t cfg)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_SWD_CONFIGURE;
	buffer[2] = cfg;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 3);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_SWD_Configure failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

#if 0
static int cmsis_dap_cmd_DAP_Delay(uint16_t delay_us)
{
	int retval;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_DELAY;
	buffer[2] = delay_us & 0xff;
	buffer[3] = (delay_us >> 8) & 0xff;
	retval = cmsis_dap_xfer(cmsis_dap_handle, 4);

	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_Delay failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}
#endif

static void cmsis_dap_swd_write_from_queue(struct cmsis_dap *dap)
{
	uint8_t *buffer = dap->packet_buffer;
	struct pending_request_block *block = &pending_fifo[pending_fifo_put_idx];

	LOG_DEBUG_IO("Executing %d queued transactions from FIFO index %d", block->transfer_count, pending_fifo_put_idx);

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skipping due to previous errors: %d", queued_retval);
		goto skip;
	}

	if (block->transfer_count == 0)
		goto skip;

	size_t idx = 0;
	buffer[idx++] = 0;	/* report number */
	buffer[idx++] = CMD_DAP_TFER;
	buffer[idx++] = 0x00;	/* DAP Index */
	buffer[idx++] = block->transfer_count;

	for (int i = 0; i < block->transfer_count; i++) {
		struct pending_transfer_result *transfer = &(block->transfers[i]);
		uint8_t cmd = transfer->cmd;
		uint32_t data = transfer->data;

		LOG_DEBUG_IO("%s %s reg %x %"PRIx32,
				cmd & SWD_CMD_APnDP ? "AP" : "DP",
				cmd & SWD_CMD_RnW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1, data);

		/* When proper WAIT handling is implemented in the
		 * common SWD framework, this kludge can be
		 * removed. However, this might lead to minor
		 * performance degradation as the adapter wouldn't be
		 * able to automatically retry anything (because ARM
		 * has forgotten to implement sticky error flags
		 * clearing). See also comments regarding
		 * cmsis_dap_cmd_DAP_TFER_Configure() and
		 * cmsis_dap_cmd_DAP_SWD_Configure() in
		 * cmsis_dap_init().
		 */
		if (!(cmd & SWD_CMD_RnW) &&
		    !(cmd & SWD_CMD_APnDP) &&
		    (cmd & SWD_CMD_A32) >> 1 == DP_CTRL_STAT &&
		    (data & CORUNDETECT)) {
			LOG_DEBUG("refusing to enable sticky overrun detection");
			data &= ~CORUNDETECT;
		}

		buffer[idx++] = (cmd >> 1) & 0x0f;
		if (!(cmd & SWD_CMD_RnW)) {
			buffer[idx++] = (data) & 0xff;
			buffer[idx++] = (data >> 8) & 0xff;
			buffer[idx++] = (data >> 16) & 0xff;
			buffer[idx++] = (data >> 24) & 0xff;
		}
	}

	int retval = dap->backend->write(dap, idx, USB_TIMEOUT);

	if (retval < 0) {
		queued_retval = retval;
		goto skip;
	} else {
		queued_retval = ERROR_OK;
	}

	pending_fifo_put_idx = (pending_fifo_put_idx + 1) % dap->packet_count;
	pending_fifo_block_count++;
	if (pending_fifo_block_count > dap->packet_count)
		LOG_ERROR("too much pending writes %d", pending_fifo_block_count);

	return;

skip:
	block->transfer_count = 0;
}

static void cmsis_dap_swd_read_process(struct cmsis_dap *dap, int timeout_ms)
{
	uint8_t *buffer = dap->packet_buffer;
	struct pending_request_block *block = &pending_fifo[pending_fifo_get_idx];

	if (pending_fifo_block_count == 0)
		LOG_ERROR("no pending write");

	/* get reply */
	int retval = dap->backend->read(dap, timeout_ms);
	if (retval == ERROR_TIMEOUT_REACHED && timeout_ms < USB_TIMEOUT)
		return;

	if (retval <= 0) {
		LOG_DEBUG("error reading data");
		queued_retval = ERROR_FAIL;
		goto skip;
	}

	if (buffer[2] & 0x08) {
		LOG_DEBUG("CMSIS-DAP Protocol Error @ %d (wrong parity)", buffer[1]);
		queued_retval = ERROR_FAIL;
		goto skip;
	}
	uint8_t ack = buffer[2] & 0x07;
	if (ack != SWD_ACK_OK) {
		LOG_DEBUG("SWD ack not OK @ %d %s", buffer[1],
			  ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK");
		queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
		goto skip;
	}

	if (block->transfer_count != buffer[1])
		LOG_ERROR("CMSIS-DAP transfer count mismatch: expected %d, got %d",
			  block->transfer_count, buffer[1]);

	LOG_DEBUG_IO("Received results of %d queued transactions FIFO index %d", buffer[1], pending_fifo_get_idx);
	size_t idx = 3;
	for (int i = 0; i < buffer[1]; i++) {
		struct pending_transfer_result *transfer = &(block->transfers[i]);
		if (transfer->cmd & SWD_CMD_RnW) {
			static uint32_t last_read;
			uint32_t data = le_to_h_u32(&buffer[idx]);
			uint32_t tmp = data;
			idx += 4;

			LOG_DEBUG_IO("Read result: %"PRIx32, data);

			/* Imitate posted AP reads */
			if ((transfer->cmd & SWD_CMD_APnDP) ||
			    ((transfer->cmd & SWD_CMD_A32) >> 1 == DP_RDBUFF)) {
				tmp = last_read;
				last_read = data;
			}

			if (transfer->buffer)
				*(uint32_t *)(transfer->buffer) = tmp;
		}
	}

skip:
	block->transfer_count = 0;
	pending_fifo_get_idx = (pending_fifo_get_idx + 1) % dap->packet_count;
	pending_fifo_block_count--;
}

static int cmsis_dap_swd_run_queue(void)
{
	if (pending_fifo_block_count)
		cmsis_dap_swd_read_process(cmsis_dap_handle, 0);

	cmsis_dap_swd_write_from_queue(cmsis_dap_handle);

	while (pending_fifo_block_count)
		cmsis_dap_swd_read_process(cmsis_dap_handle, USB_TIMEOUT);

	pending_fifo_put_idx = 0;
	pending_fifo_get_idx = 0;

	int retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

static void cmsis_dap_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data)
{
	if (pending_fifo[pending_fifo_put_idx].transfer_count == pending_queue_len) {
		if (pending_fifo_block_count)
			cmsis_dap_swd_read_process(cmsis_dap_handle, 0);

		/* Not enough room in the queue. Run the queue. */
		cmsis_dap_swd_write_from_queue(cmsis_dap_handle);

		if (pending_fifo_block_count >= cmsis_dap_handle->packet_count)
			cmsis_dap_swd_read_process(cmsis_dap_handle, USB_TIMEOUT);
	}

	if (queued_retval != ERROR_OK)
		return;

	struct pending_request_block *block = &pending_fifo[pending_fifo_put_idx];
	struct pending_transfer_result *transfer = &(block->transfers[block->transfer_count]);
	transfer->data = data;
	transfer->cmd = cmd;
	if (cmd & SWD_CMD_RnW) {
		/* Queue a read transaction */
		transfer->buffer = dst;
	}
	block->transfer_count++;
}

static void cmsis_dap_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	cmsis_dap_swd_queue_cmd(cmd, NULL, value);
}

static void cmsis_dap_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	cmsis_dap_swd_queue_cmd(cmd, value, 0);
}

static int cmsis_dap_get_serial_info(void)
{
	uint8_t *data;

	int retval = cmsis_dap_cmd_DAP_Info(INFO_ID_SERNUM, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0]) /* strlen */
		LOG_INFO("CMSIS-DAP: Serial# = %s", &data[1]);

	return ERROR_OK;
}

static int cmsis_dap_get_version_info(void)
{
	uint8_t *data;

	/* INFO_ID_FW_VER - string */
	int retval = cmsis_dap_cmd_DAP_Info(INFO_ID_FW_VER, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0]) /* strlen */
		LOG_INFO("CMSIS-DAP: FW Version = %s", &data[1]);

	return ERROR_OK;
}

static int cmsis_dap_get_caps_info(void)
{
	uint8_t *data;

	/* INFO_ID_CAPS - byte */
	int retval = cmsis_dap_cmd_DAP_Info(INFO_ID_CAPS, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] == 1) {
		uint8_t caps = data[1];

		cmsis_dap_handle->caps = caps;

		if (caps & INFO_CAPS_SWD)
			LOG_INFO("CMSIS-DAP: %s", info_caps_str[0]);
		if (caps & INFO_CAPS_JTAG)
			LOG_INFO("CMSIS-DAP: %s", info_caps_str[1]);
	}

	return ERROR_OK;
}

static int cmsis_dap_get_status(void)
{
	uint8_t d;

	int retval = cmsis_dap_cmd_DAP_SWJ_Pins(0, 0, 0, &d);

	if (retval == ERROR_OK) {
		LOG_INFO("SWCLK/TCK = %d SWDIO/TMS = %d TDI = %d TDO = %d nTRST = %d nRESET = %d",
			(d & SWJ_PIN_TCK) ? 1 : 0,
			(d & SWJ_PIN_TMS) ? 1 : 0,
			(d & SWJ_PIN_TDI) ? 1 : 0,
			(d & SWJ_PIN_TDO) ? 1 : 0,
			(d & SWJ_PIN_TRST) ? 1 : 0,
			(d & SWJ_PIN_SRST) ? 1 : 0);
	}

	return retval;
}

static int cmsis_dap_swd_switch_seq(enum swd_special_seq seq)
{
	const uint8_t *s;
	unsigned int s_len;
	int retval;

	if ((output_pins & (SWJ_PIN_SRST | SWJ_PIN_TRST)) == (SWJ_PIN_SRST | SWJ_PIN_TRST)) {
		/* Following workaround deasserts reset on most adapters.
		 * Do not reconnect if a reset line is active!
		 * Reconnecting would break connecting under reset. */

		/* First disconnect before connecting, Atmel EDBG needs it for SAMD/R/L/C */
		cmsis_dap_cmd_DAP_Disconnect();

		/* When we are reconnecting, DAP_Connect needs to be rerun, at
		 * least on Keil ULINK-ME */
		retval = cmsis_dap_cmd_DAP_Connect(CONNECT_SWD);
		if (retval != ERROR_OK)
			return retval;
	}

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		s = swd_seq_line_reset;
		s_len = swd_seq_line_reset_len;
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		s = swd_seq_jtag_to_swd;
		s_len = swd_seq_jtag_to_swd_len;
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		s = swd_seq_swd_to_jtag;
		s_len = swd_seq_swd_to_jtag_len;
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	retval = cmsis_dap_cmd_DAP_SWJ_Sequence(s_len, s);
	if (retval != ERROR_OK)
		return retval;

	/* Atmel EDBG needs renew clock setting after SWJ_Sequence
	 * otherwise default frequency is used */
	return cmsis_dap_cmd_DAP_SWJ_Clock(jtag_get_speed_khz());
}

static int cmsis_dap_swd_open(void)
{
	int retval;

	if (!(cmsis_dap_handle->caps & INFO_CAPS_SWD)) {
		LOG_ERROR("CMSIS-DAP: SWD not supported");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	retval = cmsis_dap_cmd_DAP_Connect(CONNECT_SWD);
	if (retval != ERROR_OK)
		return retval;

	/* Add more setup here.??... */

	LOG_INFO("CMSIS-DAP: Interface Initialised (SWD)");
	return ERROR_OK;
}

static int cmsis_dap_init(void)
{
	int retval;
	uint8_t *data;

	retval = cmsis_dap_open();
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_get_caps_info();
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_get_version_info();
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_get_serial_info();
	if (retval != ERROR_OK)
		return retval;

	if (swd_mode) {
		retval = cmsis_dap_swd_open();
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* Connect in JTAG mode */
		if (!(cmsis_dap_handle->caps & INFO_CAPS_JTAG)) {
			LOG_ERROR("CMSIS-DAP: JTAG not supported");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		retval = cmsis_dap_cmd_DAP_Connect(CONNECT_JTAG);
		if (retval != ERROR_OK)
			return retval;

		LOG_INFO("CMSIS-DAP: Interface Initialised (JTAG)");
	}

	/* Be conservative and suppress submitting multiple HID requests
	 * until we get packet count info from the adaptor */
	cmsis_dap_handle->packet_count = 1;
	pending_queue_len = 12;

	/* INFO_ID_PKT_SZ - short */
	retval = cmsis_dap_cmd_DAP_Info(INFO_ID_PKT_SZ, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] == 2) {  /* short */
		uint16_t pkt_sz = data[1] + (data[2] << 8);

		/* 4 bytes of command header + 5 bytes per register
		 * write. For bulk read sequences just 4 bytes are
		 * needed per transfer, so this is suboptimal. */
		pending_queue_len = (pkt_sz - 4) / 5;

		if (cmsis_dap_handle->packet_size != pkt_sz + 1) {
			/* reallocate buffer */
			cmsis_dap_handle->packet_size = pkt_sz + 1;
			cmsis_dap_handle->packet_buffer = realloc(cmsis_dap_handle->packet_buffer,
					cmsis_dap_handle->packet_size);
			if (cmsis_dap_handle->packet_buffer == NULL) {
				LOG_ERROR("unable to reallocate memory");
				return ERROR_FAIL;
			}
		}

		LOG_DEBUG("CMSIS-DAP: Packet Size = %" PRId16, pkt_sz);
	}

	/* INFO_ID_PKT_CNT - byte */
	retval = cmsis_dap_cmd_DAP_Info(INFO_ID_PKT_CNT, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] == 1) { /* byte */
		int pkt_cnt = data[1];
		if (pkt_cnt > 1)
			cmsis_dap_handle->packet_count = MIN(MAX_PENDING_REQUESTS, pkt_cnt);

		LOG_DEBUG("CMSIS-DAP: Packet Count = %d", pkt_cnt);
	}

	LOG_DEBUG("Allocating FIFO for %d pending packets", cmsis_dap_handle->packet_count);
	for (int i = 0; i < cmsis_dap_handle->packet_count; i++) {
		pending_fifo[i].transfers = malloc(pending_queue_len * sizeof(struct pending_transfer_result));
		if (!pending_fifo[i].transfers) {
			LOG_ERROR("Unable to allocate memory for CMSIS-DAP queue");
			return ERROR_FAIL;
		}
	}


	retval = cmsis_dap_get_status();
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* Now try to connect to the target
	 * TODO: This is all SWD only @ present */
	retval = cmsis_dap_cmd_DAP_SWJ_Clock(jtag_get_speed_khz());
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* Ask CMSIS-DAP to automatically retry on receiving WAIT for
	 * up to 64 times. This must be changed to 0 if sticky
	 * overrun detection is enabled. */
	retval = cmsis_dap_cmd_DAP_TFER_Configure(0, 64, 0);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	if (swd_mode) {
		/* Data Phase (bit 2) must be set to 1 if sticky overrun
		 * detection is enabled */
		retval = cmsis_dap_cmd_DAP_SWD_Configure(0);	/* 1 TRN, no Data Phase */
		if (retval != ERROR_OK)
			return ERROR_FAIL;
	}
	/* Both LEDs on */
	retval = cmsis_dap_cmd_DAP_LED(LED_ID_CONNECT, LED_ON);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	retval = cmsis_dap_cmd_DAP_LED(LED_ID_RUN, LED_ON);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* support connecting with srst asserted */
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING) {
			retval = cmsis_dap_cmd_DAP_SWJ_Pins(0, SWJ_PIN_SRST, 0, NULL);
			if (retval != ERROR_OK)
				return ERROR_FAIL;
			LOG_INFO("Connecting under reset");
		}
	}
	LOG_INFO("CMSIS-DAP: Interface ready");

	return ERROR_OK;
}

static int cmsis_dap_swd_init(void)
{
	swd_mode = true;
	return ERROR_OK;
}

static int cmsis_dap_quit(void)
{
	cmsis_dap_cmd_DAP_Disconnect();
	/* Both LEDs off */
	cmsis_dap_cmd_DAP_LED(LED_ID_RUN, LED_OFF);
	cmsis_dap_cmd_DAP_LED(LED_ID_CONNECT, LED_OFF);

	cmsis_dap_close(cmsis_dap_handle);

	return ERROR_OK;
}

static int cmsis_dap_reset(int trst, int srst)
{
	/* Set both TRST and SRST even if they're not enabled as
	 * there's no way to tristate them */

	output_pins = 0;
	if (!srst)
		output_pins |= SWJ_PIN_SRST;
	if (!trst)
		output_pins |= SWJ_PIN_TRST;

	int retval = cmsis_dap_cmd_DAP_SWJ_Pins(output_pins,
			SWJ_PIN_TRST | SWJ_PIN_SRST, 0, NULL);
	if (retval != ERROR_OK)
		LOG_ERROR("CMSIS-DAP: Interface reset failed");
	return retval;
}

static void cmsis_dap_execute_sleep(struct jtag_command *cmd)
{
#if 0
	int retval = cmsis_dap_cmd_DAP_Delay(cmd->cmd.sleep->us);
	if (retval != ERROR_OK)
#endif
		jtag_sleep(cmd->cmd.sleep->us);
}

/* Set TMS high for five TCK clocks, to move the TAP to the Test-Logic-Reset state */
static int cmsis_dap_execute_tlr_reset(struct jtag_command *cmd)
{
	LOG_INFO("cmsis-dap JTAG TLR_RESET");
	uint8_t seq = 0xff;
	int ret = cmsis_dap_cmd_DAP_SWJ_Sequence(8, &seq);
	if (ret == ERROR_OK)
		tap_set_state(TAP_RESET);
	return ret;
}

/* Set new end state */
static void cmsis_dap_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

#ifdef SPRINT_BINARY
static void sprint_binary(char *s, const uint8_t *buf, int offset, int len)
{
	if (!len)
		return;

	/*
	buf = { 0x18 } len=5 should result in: 11000
	buf = { 0xff 0x18 } len=13 should result in: 11111111 11000
	buf = { 0xc0 0x18 } offset=3 len=10 should result in: 11000 11000
		i=3 there means i/8 = 0 so c = 0xFF, and
	*/
	for (int i = offset; i < offset + len; ++i) {
		uint8_t c = buf[i / 8], mask = 1 << (i % 8);
		if ((i != offset) && !(i % 8))
			putchar(' ');
		*s++ = (c & mask) ? '1' : '0';
	}
	*s = 0;
}
#endif

#ifdef CMSIS_DAP_JTAG_DEBUG
static void debug_parse_cmsis_buf(const uint8_t *cmd, int cmdlen)
{
	/* cmd is a usb packet to go to the cmsis-dap interface */
	printf("cmsis-dap buffer (%d b): ", cmdlen);
	for (int i = 0; i < cmdlen; ++i)
		printf(" %02x", cmd[i]);
	printf("\n");
	switch (cmd[1]) {
		case CMD_DAP_JTAG_SEQ: {
			printf("cmsis-dap jtag sequence command %02x (n=%d)\n", cmd[1], cmd[2]);
			/*
			 * #2 = number of sequences
			 * #3 = sequence info 1
			 * #4...4+n_bytes-1 = sequence 1
			 * #4+n_bytes = sequence info 2
			 * #5+n_bytes = sequence 2 (single bit)
			 */
			int pos = 3;
			for (int seq = 0; seq < cmd[2]; ++seq) {
				uint8_t info = cmd[pos++];
				int len = info & DAP_JTAG_SEQ_TCK;
				if (len == 0)
					len = 64;
				printf("  sequence %d starting %d: info %02x (len=%d tms=%d read_tdo=%d): ",
					seq, pos, info, len, info & DAP_JTAG_SEQ_TMS, info & DAP_JTAG_SEQ_TDO);
				for (int i = 0; i < DIV_ROUND_UP(len, 8); ++i)
					printf(" %02x", cmd[pos+i]);
				pos += DIV_ROUND_UP(len, 8);
				printf("\n");
			}
			if (pos != cmdlen) {
				printf("BUFFER LENGTH MISMATCH looks like %d but %d specified", pos, cmdlen);
				exit(-1);
			}

			break;
		}
		default:
			LOG_DEBUG("unknown cmsis-dap command %02x", cmd[1]);
			break;
	}
}
#endif

static void cmsis_dap_flush(void)
{
	if (!queued_seq_count)
		return;

	LOG_DEBUG_IO("Flushing %d queued sequences (%d bytes) with %d pending scan results to capture",
		queued_seq_count, queued_seq_buf_end, pending_scan_result_count);

	/* prep CMSIS-DAP packet */
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;
	buffer[0] = 0;	/* report number */
	buffer[1] = CMD_DAP_JTAG_SEQ;
	buffer[2] = queued_seq_count;
	memcpy(buffer + 3, queued_seq_buf, queued_seq_buf_end);

#ifdef CMSIS_DAP_JTAG_DEBUG
	debug_parse_cmsis_buf(buffer, queued_seq_buf_end + 3);
#endif

	/* send command to USB device */
	int retval = cmsis_dap_xfer(cmsis_dap_handle, queued_seq_buf_end + 3);
	if (retval != ERROR_OK || buffer[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_JTAG_SEQ failed.");
		exit(-1);
	}

#ifdef CMSIS_DAP_JTAG_DEBUG
	LOG_DEBUG_IO("USB response buf:");
	for (int c = 0; c < queued_seq_buf_end + 3; ++c)
		printf("%02X ", buffer[c]);
	printf("\n");
#endif

	/* copy scan results into client buffers */
	for (int i = 0; i < pending_scan_result_count; ++i) {
		struct pending_scan_result *scan = &pending_scan_results[i];
		LOG_DEBUG_IO("Copying pending_scan_result %d/%d: %d bits from byte %d -> buffer + %d bits",
			i, pending_scan_result_count, scan->length, scan->first + 2, scan->buffer_offset);
#ifdef CMSIS_DAP_JTAG_DEBUG
		for (uint32_t b = 0; b < DIV_ROUND_UP(scan->length, 8); ++b)
			printf("%02X ", buffer[2+scan->first+b]);
		printf("\n");
#endif
		bit_copy(scan->buffer, scan->buffer_offset, buffer + 2 + scan->first, 0, scan->length);
	}

	/* reset */
	queued_seq_count = 0;
	queued_seq_buf_end = 0;
	queued_seq_tdo_ptr = 0;
	pending_scan_result_count = 0;
}

/* queue a sequence of bits to clock out TDI / in TDO, executing if the buffer is full.
 *
 * sequence=NULL means clock out zeros on TDI
 * tdo_buffer=NULL means don't capture TDO
 */
static void cmsis_dap_add_jtag_sequence(int s_len, const uint8_t *sequence, int s_offset,
					bool tms, uint8_t *tdo_buffer, int tdo_buffer_offset)
{
	LOG_DEBUG_IO("[at %d] %d bits, tms %s, seq offset %d, tdo buf %p, tdo offset %d",
		queued_seq_buf_end,
		s_len, tms ? "HIGH" : "LOW", s_offset, tdo_buffer, tdo_buffer_offset);

	if (s_len == 0)
		return;

	if (s_len > 64) {
		LOG_DEBUG_IO("START JTAG SEQ SPLIT");
		for (int offset = 0; offset < s_len; offset += 64) {
			int len = s_len - offset;
			if (len > 64)
				len = 64;
			LOG_DEBUG_IO("Splitting long jtag sequence: %d-bit chunk starting at offset %d", len, offset);
			cmsis_dap_add_jtag_sequence(
				len,
				sequence,
				s_offset + offset,
				tms,
				tdo_buffer,
				tdo_buffer == NULL ? 0 : (tdo_buffer_offset + offset)
				);
		}
		LOG_DEBUG_IO("END JTAG SEQ SPLIT");
		return;
	}

	int cmd_len = 1 + DIV_ROUND_UP(s_len, 8);
	if (queued_seq_count >= 255 || queued_seq_buf_end + cmd_len > QUEUED_SEQ_BUF_LEN)
		/* empty out the buffer */
		cmsis_dap_flush();

	++queued_seq_count;

	/* control byte */
	queued_seq_buf[queued_seq_buf_end] =
		(tms ? DAP_JTAG_SEQ_TMS : 0) |
		(tdo_buffer != NULL ? DAP_JTAG_SEQ_TDO : 0) |
		(s_len == 64 ? 0 : s_len);

	if (sequence != NULL)
		bit_copy(&queued_seq_buf[queued_seq_buf_end + 1], 0, sequence, s_offset, s_len);
	else
		memset(&queued_seq_buf[queued_seq_buf_end + 1], 0, DIV_ROUND_UP(s_len, 8));

	queued_seq_buf_end += cmd_len;

	if (tdo_buffer != NULL) {
		struct pending_scan_result *scan = &pending_scan_results[pending_scan_result_count++];
		scan->first = queued_seq_tdo_ptr;
		queued_seq_tdo_ptr += DIV_ROUND_UP(s_len, 8);
		scan->length = s_len;
		scan->buffer = tdo_buffer;
		scan->buffer_offset = tdo_buffer_offset;
	}
}

/* queue a sequence of bits to clock out TMS, executing if the buffer is full */
static void cmsis_dap_add_tms_sequence(const uint8_t *sequence, int s_len)
{
	LOG_DEBUG_IO("%d bits: %02X", s_len, *sequence);
	/* we use a series of CMD_DAP_JTAG_SEQ commands to toggle TMS,
	   because even though it seems ridiculously inefficient, it
	   allows us to combine TMS and scan sequences into the same
	   USB packet. */
	/* TODO: combine runs of the same tms value */
	for (int i = 0; i < s_len; ++i) {
		bool bit = (sequence[i / 8] & (1 << (i % 8))) != 0;
		cmsis_dap_add_jtag_sequence(1, NULL, 0, bit, NULL, 0);
	}
}

/* Move to the end state by queuing a sequence to clock into TMS */
static void cmsis_dap_state_move(void)
{
	uint8_t tms_scan;
	uint8_t tms_scan_bits;

	tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	LOG_DEBUG_IO("state move from %s to %s: %d clocks, %02X on tms",
		tap_state_name(tap_get_state()), tap_state_name(tap_get_end_state()),
		tms_scan_bits, tms_scan);
	cmsis_dap_add_tms_sequence(&tms_scan, tms_scan_bits);

	tap_set_state(tap_get_end_state());
}


/* Execute a JTAG scan operation by queueing TMS and TDI/TDO sequences */
static void cmsis_dap_execute_scan(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN",
		jtag_scan_type(cmd->cmd.scan));

	/* Make sure there are no trailing fields with num_bits == 0, or the logic below will fail. */
	while (cmd->cmd.scan->num_fields > 0
			&& cmd->cmd.scan->fields[cmd->cmd.scan->num_fields - 1].num_bits == 0) {
		cmd->cmd.scan->num_fields--;
		LOG_DEBUG("discarding trailing empty field");
	}

	if (cmd->cmd.scan->num_fields == 0) {
		LOG_DEBUG("empty scan, doing nothing");
		return;
	}

	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT) {
			cmsis_dap_end_state(TAP_IRSHIFT);
			cmsis_dap_state_move();
		}
	} else {
		if (tap_get_state() != TAP_DRSHIFT) {
			cmsis_dap_end_state(TAP_DRSHIFT);
			cmsis_dap_state_move();
		}
	}

	cmsis_dap_end_state(cmd->cmd.scan->end_state);

	struct scan_field *field = cmd->cmd.scan->fields;
	unsigned scan_size = 0;

	for (int i = 0; i < cmd->cmd.scan->num_fields; i++, field++) {
		scan_size += field->num_bits;
		LOG_DEBUG_IO("%s%s field %d/%d %d bits",
			field->in_value ? "in" : "",
			field->out_value ? "out" : "",
			i,
			cmd->cmd.scan->num_fields,
			field->num_bits);

		if (i == cmd->cmd.scan->num_fields - 1 && tap_get_state() != tap_get_end_state()) {
			LOG_DEBUG_IO("Last field and have to move out of SHIFT state");
			/* Last field, and we're leaving IRSHIFT/DRSHIFT. Clock last bit during tap
			 * movement. This last field can't have length zero, it was checked above. */
			cmsis_dap_add_jtag_sequence(
				field->num_bits - 1, /* number of bits to clock */
				field->out_value, /* output sequence */
				0, /* output offset */
				false, /* TMS low */
				field->in_value,
				0);

			/* Clock the last bit out, with TMS high */
			uint8_t last_bit = 0;
			if (field->out_value)
				bit_copy(&last_bit, 0, field->out_value, field->num_bits - 1, 1);
			cmsis_dap_add_jtag_sequence(
				1,
				&last_bit,
				0,
				true,
				field->in_value,
				field->num_bits - 1);
			tap_set_state(tap_state_transition(tap_get_state(), 1));

			/* Now clock one more cycle, with TMS low, to get us into a PAUSE state */
			cmsis_dap_add_jtag_sequence(
				1,
				&last_bit,
				0,
				false,
				NULL,
				0);
			tap_set_state(tap_state_transition(tap_get_state(), 0));
		} else {
			LOG_DEBUG_IO("Internal field, staying in SHIFT state afterwards");
			/* Clocking part of a sequence into DR or IR with TMS=0,
			   leaving TMS=0 at the end so we can continue later */
			cmsis_dap_add_jtag_sequence(
				field->num_bits,
				field->out_value,
				0,
				false,
				field->in_value,
				0);
		}
	}

	if (tap_get_state() != tap_get_end_state()) {
		cmsis_dap_end_state(tap_get_end_state());
		cmsis_dap_state_move();
	}

	LOG_DEBUG_IO("%s scan, %i bits, end in %s",
		(cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
		tap_state_name(tap_get_end_state()));
}

static void cmsis_dap_pathmove(int num_states, tap_state_t *path)
{
	int i;
	uint8_t tms0 = 0x00;
	uint8_t tms1 = 0xff;

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			cmsis_dap_add_tms_sequence(&tms0, 1);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			cmsis_dap_add_tms_sequence(&tms1, 1);
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				  tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	cmsis_dap_end_state(tap_get_state());
}

static void cmsis_dap_execute_pathmove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("pathmove: %i states, end in %i",
		      cmd->cmd.pathmove->num_states,
	       cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	cmsis_dap_pathmove(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
}

static void cmsis_dap_stableclocks(int num_cycles)
{
	int i;

	uint8_t tms = tap_get_state() == TAP_RESET;
	/* TODO: Perform optimizations? */
	/* Execute num_cycles. */
	for (i = 0; i < num_cycles; i++)
		cmsis_dap_add_tms_sequence(&tms, 1);
}

static void cmsis_dap_runtest(int num_cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();

	/* Only do a state_move when we're not already in IDLE. */
	if (tap_get_state() != TAP_IDLE) {
		cmsis_dap_end_state(TAP_IDLE);
		cmsis_dap_state_move();
	}
	cmsis_dap_stableclocks(num_cycles);

	/* Finish in end_state. */
	cmsis_dap_end_state(saved_end_state);

	if (tap_get_state() != tap_get_end_state())
		cmsis_dap_state_move();
}

static void cmsis_dap_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles,
		      cmd->cmd.runtest->end_state);

	cmsis_dap_end_state(cmd->cmd.runtest->end_state);
	cmsis_dap_runtest(cmd->cmd.runtest->num_cycles);
}

static void cmsis_dap_execute_stableclocks(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("stableclocks %i cycles", cmd->cmd.runtest->num_cycles);
	cmsis_dap_stableclocks(cmd->cmd.runtest->num_cycles);
}

static void cmsis_dap_execute_tms(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("TMS: %d bits", cmd->cmd.tms->num_bits);
	cmsis_dap_cmd_DAP_SWJ_Sequence(cmd->cmd.tms->num_bits, cmd->cmd.tms->bits);
}

/* TODO: Is there need to call cmsis_dap_flush() for the JTAG_PATHMOVE,
 * JTAG_RUNTEST, JTAG_STABLECLOCKS? */
static void cmsis_dap_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_SLEEP:
			cmsis_dap_flush();
			cmsis_dap_execute_sleep(cmd);
			break;
		case JTAG_TLR_RESET:
			cmsis_dap_flush();
			cmsis_dap_execute_tlr_reset(cmd);
			break;
		case JTAG_SCAN:
			cmsis_dap_execute_scan(cmd);
			break;
		case JTAG_PATHMOVE:
			cmsis_dap_execute_pathmove(cmd);
			break;
		case JTAG_RUNTEST:
			cmsis_dap_execute_runtest(cmd);
			break;
		case JTAG_STABLECLOCKS:
			cmsis_dap_execute_stableclocks(cmd);
			break;
		case JTAG_TMS:
			cmsis_dap_execute_tms(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X encountered", cmd->type);
			exit(-1);
	}
}

static int cmsis_dap_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		cmsis_dap_execute_command(cmd);
		cmd = cmd->next;
	}

	cmsis_dap_flush();

	return ERROR_OK;
}

static int cmsis_dap_speed(int speed)
{
	if (speed > DAP_MAX_CLOCK)
		LOG_INFO("High speed (adapter speed %d) may be limited by adapter firmware.", speed);

	if (speed == 0) {
		LOG_ERROR("RTCK not supported. Set nonzero \"adapter speed\".");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	return cmsis_dap_cmd_DAP_SWJ_Clock(speed);
}

static int cmsis_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int cmsis_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_info_command)
{
	if (cmsis_dap_get_version_info() == ERROR_OK)
		cmsis_dap_get_status();

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_cmd_command)
{
	int retval;
	unsigned i;
	uint8_t *buffer = cmsis_dap_handle->packet_buffer;

	buffer[0] = 0;	/* report number */

	for (i = 0; i < CMD_ARGC; i++)
		buffer[i + 1] = strtoul(CMD_ARGV[i], NULL, 16);

	retval = cmsis_dap_xfer(cmsis_dap_handle, CMD_ARGC + 1);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	LOG_INFO("Returned data %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " %02" PRIx8,
		buffer[1], buffer[2], buffer[3], buffer[4]);

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in cmsis_dap_vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete cmsis_dap_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], cmsis_dap_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], cmsis_dap_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * cmsis_dap_vid_pid.
	 */
	cmsis_dap_vid[i >> 1] = cmsis_dap_pid[i >> 1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_serial_command)
{
	if (CMD_ARGC == 1)
		cmsis_dap_serial = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("expected exactly one argument to cmsis_dap_serial <serial-number>");

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_backend_command)
{
	if (CMD_ARGC == 1) {
		if (strcmp(CMD_ARGV[0], "auto") == 0) {
			cmsis_dap_backend = -1; /* autoselect */
		} else {
			for (unsigned int i = 0; i < ARRAY_SIZE(cmsis_dap_backends); i++) {
				if (strcasecmp(cmsis_dap_backends[i]->name, CMD_ARGV[0]) == 0) {
					cmsis_dap_backend = i;
					return ERROR_OK;
				}
			}

			LOG_ERROR("invalid backend argument to cmsis_dap_backend <backend>");
		}
	} else {
		LOG_ERROR("expected exactly one argument to cmsis_dap_backend <backend>");
	}

	return ERROR_OK;
}

static const struct command_registration cmsis_dap_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &cmsis_dap_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "show cmsis-dap info",
	},
	{
		.name = "cmd",
		.handler = &cmsis_dap_handle_cmd_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "issue cmsis-dap command",
	},
	COMMAND_REGISTRATION_DONE
};


static const struct command_registration cmsis_dap_command_handlers[] = {
	{
		.name = "cmsis-dap",
		.mode = COMMAND_ANY,
		.help = "perform CMSIS-DAP management",
		.usage = "<cmd>",
		.chain = cmsis_dap_subcommand_handlers,
	},
	{
		.name = "cmsis_dap_vid_pid",
		.handler = &cmsis_dap_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the CMSIS-DAP device",
		.usage = "(vid pid)* ",
	},
	{
		.name = "cmsis_dap_serial",
		.handler = &cmsis_dap_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the adapter",
		.usage = "serial_string",
	},
	{
		.name = "cmsis_dap_backend",
		.handler = &cmsis_dap_handle_backend_command,
		.mode = COMMAND_CONFIG,
		.help = "set the communication backend to use (USB bulk or HID).",
		.usage = "(auto | usb_bulk | hid)",
	},
#if BUILD_CMSIS_DAP_USB
	{
		.name = "cmsis_dap_usb",
		.chain = cmsis_dap_usb_subcommand_handlers,
		.mode = COMMAND_ANY,
		.help = "USB bulk backend-specific commands",
		.usage = "<cmd>",
	},
#endif
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver cmsis_dap_swd_driver = {
	.init = cmsis_dap_swd_init,
	.switch_seq = cmsis_dap_swd_switch_seq,
	.read_reg = cmsis_dap_swd_read_reg,
	.write_reg = cmsis_dap_swd_write_reg,
	.run = cmsis_dap_swd_run_queue,
};

static const char * const cmsis_dap_transport[] = { "swd", "jtag", NULL };

static struct jtag_interface cmsis_dap_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = cmsis_dap_execute_queue,
};

struct adapter_driver cmsis_dap_adapter_driver = {
	.name = "cmsis-dap",
	.transports = cmsis_dap_transport,
	.commands = cmsis_dap_command_handlers,

	.init = cmsis_dap_init,
	.quit = cmsis_dap_quit,
	.reset = cmsis_dap_reset,
	.speed = cmsis_dap_speed,
	.khz = cmsis_dap_khz,
	.speed_div = cmsis_dap_speed_div,

	.jtag_ops = &cmsis_dap_interface,
	.swd_ops = &cmsis_dap_swd_driver,
};
