/***************************************************************************
 *   Copyright (C) 2012 by Jan Dakinevich                                  *
 *   jan.dakinevich@gmail.com                                              *
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
#	include "config.h"
#endif

#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <helper/command.h>
#include <jtag/interface.h>
#include "libusb_helper.h"

struct sequence {
	int len;
	void *tms;
	void *tdo;
	const void *tdi;
	struct sequence *next;
};

struct queue {
	struct sequence *head;
	struct sequence *tail;
};

static struct sequence *queue_add_tail(struct queue *queue, int len)
{
	if (len <= 0) {
		LOG_ERROR("BUG: sequences with zero length are not allowed");
		return NULL;
	}

	struct sequence *next;
	next = malloc(sizeof(*next));
	if (next) {
		next->tms = calloc(1, DIV_ROUND_UP(len, 8));
		if (next->tms) {
			next->len = len;
			next->tdo = NULL;
			next->tdi = NULL;
			next->next = NULL;

			if (!queue->head) {
				/* Queue is empty at the moment */
				queue->head = next;
			} else {
				/* Queue already contains at least one sequence */
				queue->tail->next = next;
			}

			queue->tail = next;
		} else {
			free(next);
			next = NULL;
		}
	}

	if (!next)
		LOG_ERROR("Not enough memory");

	return next;
}

static void queue_drop_head(struct queue *queue)
{
	struct sequence *head = queue->head->next; /* New head */
	free(queue->head->tms);
	free(queue->head);
	queue->head = head;
}

static void queue_free(struct queue *queue)
{
	if (queue) {
		while (queue->head)
			queue_drop_head(queue);

		free(queue);
	}
}

static struct queue *queue_alloc(void)
{
	struct queue *queue = malloc(sizeof(*queue));
	if (queue)
		queue->head = NULL;
	else
		LOG_ERROR("Not enough memory");

	return queue;
}

/* Size of usb communication buffer */
#define OSBDM_USB_BUFSIZE 64
/* Timeout for USB transfer, ms */
#define OSBDM_USB_TIMEOUT 1000
/* Write end point */
#define OSBDM_USB_EP_WRITE 0x01
/* Read end point */
#define OSBDM_USB_EP_READ 0x82

/* Initialize OSBDM device */
#define OSBDM_CMD_INIT 0x11
/* Execute special, not-BDM command. But only this
 * command is used for JTAG operation */
#define OSBDM_CMD_SPECIAL 0x27
/* Execute JTAG swap (tms/tdi -> tdo) */
#define OSBDM_CMD_SPECIAL_SWAP 0x05
/* Reset control */
#define OSBDM_CMD_SPECIAL_SRST 0x01
/* Maximum bit-length in one swap */
#define OSBDM_SWAP_MAX (((OSBDM_USB_BUFSIZE - 6) / 5) * 16)

/* Lists of valid VID/PID pairs
 */
static const uint16_t osbdm_vid[] = { 0x15a2, 0x15a2, 0x15a2, 0 };
static const uint16_t osbdm_pid[] = { 0x0042, 0x0058, 0x005e, 0 };

struct osbdm {
	struct libusb_device_handle *devh; /* USB handle */
	uint8_t buffer[OSBDM_USB_BUFSIZE]; /* Data to send and receive */
	int count; /* Count data to send and to read */
};

/* osbdm instance
 */
static struct osbdm osbdm_context;

static int osbdm_send_and_recv(struct osbdm *osbdm)
{
	/* Send request */
	int count, ret;

	ret = jtag_libusb_bulk_write(osbdm->devh, OSBDM_USB_EP_WRITE,
				     (char *)osbdm->buffer, osbdm->count,
				     OSBDM_USB_TIMEOUT, &count);
	if (ret || count != osbdm->count) {
		LOG_ERROR("OSBDM communication error: can't write");
		return ERROR_FAIL;
	}

	/* Save command code for next checking */
	uint8_t cmd_saved = osbdm->buffer[0];

	/* Reading answer */
	ret = jtag_libusb_bulk_read(osbdm->devh, OSBDM_USB_EP_READ,
				    (char *)osbdm->buffer, OSBDM_USB_BUFSIZE,
				    OSBDM_USB_TIMEOUT, &osbdm->count);
	/* Now perform basic checks for data sent by BDM device
	 */
	if (ret) {
		LOG_ERROR("OSBDM communication error: can't read");
		return ERROR_FAIL;
	}

	if (osbdm->count < 2) {
		LOG_ERROR("OSBDM communication error: reply too small");
		return ERROR_FAIL;
	}

	if (osbdm->count != osbdm->buffer[1])  {
		LOG_ERROR("OSBDM communication error: reply size mismatch");
		return ERROR_FAIL;
	}

	if (cmd_saved != osbdm->buffer[0]) {
		LOG_ERROR("OSBDM communication error: reply command mismatch");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int osbdm_srst(struct osbdm *osbdm, int srst)
{
	osbdm->count = 0;
	(void)memset(osbdm->buffer, 0, OSBDM_USB_BUFSIZE);

	/* Composing request
	 */
	osbdm->buffer[osbdm->count++] = OSBDM_CMD_SPECIAL; /* Command */
	osbdm->buffer[osbdm->count++] = OSBDM_CMD_SPECIAL_SRST; /* Subcommand */
	/* Length in bytes - not used */
	osbdm->buffer[osbdm->count++] = 0;
	osbdm->buffer[osbdm->count++] = 0;
	/* SRST state */
	osbdm->buffer[osbdm->count++] = (srst ? 0 : 0x08);

	/* Sending data
	 */
	if (osbdm_send_and_recv(osbdm) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int osbdm_swap(struct osbdm *osbdm, void *tms, void *tdi,
	void *tdo, int length)
{
	if (length > OSBDM_SWAP_MAX) {
		LOG_ERROR("BUG: bit sequence too long");
		return ERROR_FAIL;
	}

	if (length <= 0) {
		LOG_ERROR("BUG: bit sequence equal or less than 0");
		return ERROR_FAIL;
	}

	int swap_count = DIV_ROUND_UP(length, 16);

	/* cleanup */
	osbdm->count = 0;
	(void)memset(osbdm->buffer, 0, OSBDM_USB_BUFSIZE);

	/* Composing request
	 */

	osbdm->buffer[osbdm->count++] = OSBDM_CMD_SPECIAL; /* Command */
	osbdm->buffer[osbdm->count++] = OSBDM_CMD_SPECIAL_SWAP; /* Subcommand */
	/* Length in bytes - not used */
	osbdm->buffer[osbdm->count++] = 0;
	osbdm->buffer[osbdm->count++] = 0;
	/* Swap count */
	osbdm->buffer[osbdm->count++] = 0;
	osbdm->buffer[osbdm->count++] = (uint8_t)swap_count;

	for (int bit_idx = 0; bit_idx < length; ) {
		/* Bit count in swap */
		int bit_count = length - bit_idx;
		if (bit_count > 16)
			bit_count = 16;

		osbdm->buffer[osbdm->count++] = (uint8_t)bit_count;

		/* Copying TMS and TDI data to output buffer */
		uint32_t tms_data = buf_get_u32(tms, bit_idx, bit_count);
		uint32_t tdi_data = buf_get_u32(tdi, bit_idx, bit_count);
		osbdm->buffer[osbdm->count++] = (uint8_t)(tdi_data >> 8);
		osbdm->buffer[osbdm->count++] = (uint8_t)tdi_data;
		osbdm->buffer[osbdm->count++] = (uint8_t)(tms_data >> 8);
		osbdm->buffer[osbdm->count++] = (uint8_t)tms_data;

		/* Next bit offset */
		bit_idx += bit_count;
	}

	assert(osbdm->count <= OSBDM_USB_BUFSIZE);

	/* Sending data
	 */
	if (osbdm_send_and_recv(osbdm) != ERROR_OK)
		return ERROR_FAIL;

	/*	Extra check
	 */
	if (((osbdm->buffer[2] << 8) | osbdm->buffer[3]) != 2 * swap_count) {
		LOG_ERROR("OSBDM communication error: invalid swap command reply");
		return ERROR_FAIL;
	}

	/* Copy TDO response
	 */
	uint8_t *buffer = osbdm->buffer + 4;
	for (int bit_idx = 0; bit_idx < length; ) {
		int bit_count = length - bit_idx;
		if (bit_count > 16)
			bit_count = 16;

		/* Prepare data */
		uint32_t tdo_data = 0;
		tdo_data |= (*buffer++) << 8;
		tdo_data |= (*buffer++);
		tdo_data >>= (16 - bit_count);

		/* Copy TDO to return */
		buf_set_u32(tdo, bit_idx, bit_count, tdo_data);

		bit_idx += bit_count;
	}

	return ERROR_OK;
}

static int osbdm_flush(struct osbdm *osbdm, struct queue *queue)
{
	uint8_t tms[DIV_ROUND_UP(OSBDM_SWAP_MAX, 8)];
	uint8_t tdi[DIV_ROUND_UP(OSBDM_SWAP_MAX, 8)];
	uint8_t tdo[DIV_ROUND_UP(OSBDM_SWAP_MAX, 8)];

	int seq_back_len = 0;

	while (queue->head) {
		(void)memset(tms, 0, sizeof(tms));
		(void)memset(tdi, 0, sizeof(tdi));
		(void)memset(tdo, 0, sizeof(tdo));

		int seq_len;
		int swap_len;
		struct sequence *seq;

		/* Copy from queue to tms/tdi streams
		 */
		seq = queue->head;
		seq_len = seq_back_len;
		swap_len = 0;

		while (seq && swap_len != OSBDM_SWAP_MAX) {
			/* Count bit for copy at this iteration.
			 * len should fit into remaining space
			 * in tms/tdo bitstreams
			 */
			int len = seq->len - seq_len;
			if (len > OSBDM_SWAP_MAX - swap_len)
				len = OSBDM_SWAP_MAX - swap_len;

			/* Set tms data */
			buf_set_buf(seq->tms, seq_len, tms, swap_len, len);

			/* Set tdi data if they exists */
			if (seq->tdi)
				buf_set_buf(seq->tdi, seq_len, tdi, swap_len, len);

			swap_len += len;
			seq_len += len;
			if (seq_len == seq->len) {
				seq = seq->next; /* Move to next sequence */
				seq_len = 0;
			}
		}

		if (osbdm_swap(osbdm, tms, tdi, tdo, swap_len))
			return ERROR_FAIL;

		/* Copy from tdo stream to queue
		 */

		for (int swap_back_len = 0; swap_back_len < swap_len; ) {
			int len = queue->head->len - seq_back_len;
			if (len > swap_len - swap_back_len)
				len = swap_len - swap_back_len;

			if (queue->head->tdo)
				buf_set_buf(tdo, swap_back_len,	queue->head->tdo, seq_back_len, len);

			swap_back_len += len;
			seq_back_len += len;
			if (seq_back_len == queue->head->len) {
				queue_drop_head(queue);
				seq_back_len = 0;
			}
		}
	}

	return ERROR_OK;
}

/*	Basic operation for opening USB device */
static int osbdm_open(struct osbdm *osbdm)
{
	(void)memset(osbdm, 0, sizeof(*osbdm));
	if (jtag_libusb_open(osbdm_vid, osbdm_pid, NULL, &osbdm->devh, NULL) != ERROR_OK)
		return ERROR_FAIL;

	if (libusb_claim_interface(osbdm->devh, 0) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int osbdm_quit(void)
{
	jtag_libusb_close(osbdm_context.devh);
	return ERROR_OK;
}

static int osbdm_add_pathmove(
	struct queue *queue,
	tap_state_t *path,
	int num_states)
{
	assert(num_states <= 32);

	struct sequence *next = queue_add_tail(queue, num_states);
	if (!next) {
		LOG_ERROR("BUG: can't allocate bit sequence");
		return ERROR_FAIL;
	}

	uint32_t tms = 0;
	for (int i = 0; i < num_states; i++) {
		if (tap_state_transition(tap_get_state(), 1) == path[i]) {
			tms |= (1 << i);
		} else if (tap_state_transition(tap_get_state(), 0) == path[i]) {
			tms &= ~(1 << i); /* This line not so needed */
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP state transition",
				tap_state_name(tap_get_state()),
				tap_state_name(path[i]));
			return ERROR_FAIL;
		}

		tap_set_state(path[i]);
	}

	buf_set_u32(next->tms, 0, num_states, tms);
	tap_set_end_state(tap_get_state());

	return ERROR_OK;
}

static int osbdm_add_statemove(
	struct queue *queue,
	tap_state_t new_state,
	int skip_first)
{
	int len = 0;
	int tms = 0;

	tap_set_end_state(new_state);
	if (tap_get_end_state() == TAP_RESET) {
		/* Ignore current state */
		tms = 0xff;
		len = 5;
	} else if (tap_get_state() != tap_get_end_state()) {
		tms = tap_get_tms_path(tap_get_state(), new_state);
		len = tap_get_tms_path_len(tap_get_state(), new_state);
	}

	if (len && skip_first) {
		len--;
		tms >>= 1;
	}

	if (len) {
		struct sequence *next = queue_add_tail(queue, len);
		if (!next) {
			LOG_ERROR("BUG: can't allocate bit sequence");
			return ERROR_FAIL;
		}
		buf_set_u32(next->tms, 0, len, tms);
	}

	tap_set_state(tap_get_end_state());
	return ERROR_OK;
}

static int osbdm_add_stableclocks(
	struct queue *queue,
	int count)
{
	if (!tap_is_state_stable(tap_get_state())) {
		LOG_ERROR("BUG: current state (%s) is not stable",
			tap_state_name(tap_get_state()));
		return ERROR_FAIL;
	}

	struct sequence *next = queue_add_tail(queue, count);
	if (!next) {
		LOG_ERROR("BUG: can't allocate bit sequence");
		return ERROR_FAIL;
	}

	if (tap_get_state() == TAP_RESET)
		(void)memset(next->tms, 0xff, DIV_ROUND_UP(count, 8));

	return ERROR_OK;
}

static int osbdm_add_tms(
	struct queue *queue,
	const uint8_t *tms,
	int num_bits)
{
	struct sequence *next = queue_add_tail(queue, num_bits);
	if (!next) {
		LOG_ERROR("BUG: can't allocate bit sequence");
		return ERROR_FAIL;
	}
	buf_set_buf(tms, 0, next->tms, 0, num_bits);

	return ERROR_OK;
}

static int osbdm_add_scan(
	struct queue *queue,
	struct scan_field *fields,
	int num_fields,
	tap_state_t end_state,
	bool ir_scan)
{
	/* Move to desired shift state */
	if (ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT) {
			if (osbdm_add_statemove(queue, TAP_IRSHIFT, 0) != ERROR_OK)
				return ERROR_FAIL;
		}
	} else {
		if (tap_get_state() != TAP_DRSHIFT) {
			if (osbdm_add_statemove(queue, TAP_DRSHIFT, 0) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	/* Add scan */
	tap_set_end_state(end_state);
	for (int idx = 0; idx < num_fields; idx++) {
		struct sequence *next = queue_add_tail(queue, fields[idx].num_bits);
		if (!next) {
			LOG_ERROR("Can't allocate bit sequence");
			return ERROR_FAIL;
		}

		(void)memset(next->tms, 0, DIV_ROUND_UP(fields[idx].num_bits, 8));
		next->tdi = fields[idx].out_value;
		next->tdo = fields[idx].in_value;
	}

	/* Move to end state
	 */
	if (tap_get_state() != tap_get_end_state()) {
		/* Exit from IRSHIFT/DRSHIFT */
		buf_set_u32(queue->tail->tms, queue->tail->len - 1, 1, 1);

		/* Move with skip_first flag */
		if (osbdm_add_statemove(queue, tap_get_end_state(), 1) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int osbdm_add_runtest(
	struct queue *queue,
	int num_cycles,
	tap_state_t end_state)
{
	if (osbdm_add_statemove(queue, TAP_IDLE, 0) != ERROR_OK)
		return ERROR_FAIL;

	if (osbdm_add_stableclocks(queue, num_cycles) != ERROR_OK)
		return ERROR_FAIL;

	if (osbdm_add_statemove(queue, end_state, 0) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int osbdm_execute_command(
	struct osbdm *osbdm,
	struct queue *queue,
	struct jtag_command *cmd)
{
	int retval = ERROR_OK;

	switch (cmd->type) {
	case JTAG_RESET:
		if (cmd->cmd.reset->trst) {
			LOG_ERROR("BUG: nTRST signal is not supported");
			retval = ERROR_FAIL;
		} else {
			retval = osbdm_flush(osbdm, queue);
			if (retval == ERROR_OK)
				retval = osbdm_srst(osbdm, cmd->cmd.reset->srst);
		}
		break;

	case JTAG_PATHMOVE:
		retval = osbdm_add_pathmove(
			queue,
			cmd->cmd.pathmove->path,
			cmd->cmd.pathmove->num_states);
		break;

	case JTAG_TLR_RESET:
		retval = osbdm_add_statemove(
			queue,
			cmd->cmd.statemove->end_state,
			0);
		break;

	case JTAG_STABLECLOCKS:
		retval = osbdm_add_stableclocks(
			queue,
			cmd->cmd.stableclocks->num_cycles);
		break;

	case JTAG_TMS:
		retval = osbdm_add_tms(
			queue,
			cmd->cmd.tms->bits,
			cmd->cmd.tms->num_bits);
		break;

	case JTAG_SCAN:
		retval = osbdm_add_scan(
			queue,
			cmd->cmd.scan->fields,
			cmd->cmd.scan->num_fields,
			cmd->cmd.scan->end_state,
			cmd->cmd.scan->ir_scan);
		break;

	case JTAG_SLEEP:
		retval = osbdm_flush(osbdm, queue);
		if (retval == ERROR_OK)
			jtag_sleep(cmd->cmd.sleep->us);
		break;

	case JTAG_RUNTEST:
		retval = osbdm_add_runtest(
			queue,
			cmd->cmd.runtest->num_cycles,
			cmd->cmd.runtest->end_state);
		break;

	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered");
		retval = ERROR_FAIL;
		break;
	}

	return retval;
}

static int osbdm_execute_queue(void)
{
	int retval = ERROR_OK;

	struct queue *queue = queue_alloc();
	if (!queue) {
		LOG_ERROR("BUG: can't allocate bit queue");
		retval = ERROR_FAIL;
	} else {
		struct jtag_command *cmd = jtag_command_queue;

		while (retval == ERROR_OK && cmd) {
			retval = osbdm_execute_command(&osbdm_context, queue, cmd);
			cmd = cmd->next;
		}

		if (retval == ERROR_OK)
			retval = osbdm_flush(&osbdm_context, queue);

		queue_free(queue);
	}

	if (retval != ERROR_OK) {
		LOG_ERROR("FATAL: can't execute jtag command");
		exit(-1);
	}

	return retval;
}

static int osbdm_init(void)
{
	/* Open device */
	if (osbdm_open(&osbdm_context) != ERROR_OK) {
		LOG_ERROR("Can't open OSBDM device");
		return ERROR_FAIL;
	} else {
		/* Device successfully opened */
		LOG_DEBUG("OSBDM init");
	}

	/* Perform initialize command */
	osbdm_context.count = 0;
	osbdm_context.buffer[osbdm_context.count++] = OSBDM_CMD_INIT;
	if (osbdm_send_and_recv(&osbdm_context) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static struct jtag_interface osbdm_interface = {
	.execute_queue = osbdm_execute_queue,
};

struct adapter_driver osbdm_adapter_driver = {
	.name = "osbdm",
	.transports = jtag_only,

	.init = osbdm_init,
	.quit = osbdm_quit,

	.jtag_ops = &osbdm_interface,
};
