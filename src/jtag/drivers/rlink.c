/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 Rob Brown, Lou Deluxe                              *
 *   rob@cobbleware.com, lou.openocd012@fixit.nospammail.net               *
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

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/commands.h>
#include "rlink.h"
#include "rlink_st7.h"
#include "rlink_ep1_cmd.h"
#include "rlink_dtc_cmd.h"
#include "usb_common.h"

/* This feature is made useless by running the DTC all the time.  When automatic, the LED is on
 *whenever the DTC is running.  Otherwise, USB messages are sent to turn it on and off. */
#undef AUTOMATIC_BUSY_LED

/* This feature may require derating the speed due to reduced hold time. */
#undef USE_HARDWARE_SHIFTER_FOR_TMS

#define INTERFACE_NAME          "RLink"

#define USB_IDVENDOR            (0x138e)
#define USB_IDPRODUCT           (0x9000)

#define USB_EP1OUT_ADDR         (0x01)
#define USB_EP1OUT_SIZE         (16)
#define USB_EP1IN_ADDR          (USB_EP1OUT_ADDR | 0x80)
#define USB_EP1IN_SIZE          (USB_EP1OUT_SIZE)

#define USB_EP2OUT_ADDR         (0x02)
#define USB_EP2OUT_SIZE         (64)
#define USB_EP2IN_ADDR          (USB_EP2OUT_ADDR | 0x80)
#define USB_EP2IN_SIZE          (USB_EP2OUT_SIZE)
#define USB_EP2BANK_SIZE        (512)

#define USB_TIMEOUT_MS          (3 * 1000)

#define DTC_STATUS_POLL_BYTE    (ST7_USB_BUF_EP0OUT + 0xff)

#define ST7_PD_NBUSY_LED                ST7_PD0
#define ST7_PD_NRUN_LED                 ST7_PD1
/* low enables VPP at adapter header, high connects it to GND instead */
#define ST7_PD_VPP_SEL                  ST7_PD6
/* low: VPP = 12v, high: VPP <= 5v */
#define ST7_PD_VPP_SHDN                 ST7_PD7

/* These pins are connected together */
#define ST7_PE_ADAPTER_SENSE_IN         ST7_PE3
#define ST7_PE_ADAPTER_SENSE_OUT        ST7_PE4

/* Symbolic mapping between port pins and numbered IO lines */
#define ST7_PA_IO1      ST7_PA1
#define ST7_PA_IO2      ST7_PA2
#define ST7_PA_IO4      ST7_PA4
#define ST7_PA_IO8      ST7_PA6
#define ST7_PA_IO10     ST7_PA7
#define ST7_PB_IO5      ST7_PB5
#define ST7_PC_IO9      ST7_PC1
#define ST7_PC_IO3      ST7_PC2
#define ST7_PC_IO7      ST7_PC3
#define ST7_PE_IO6      ST7_PE5

/* Symbolic mapping between numbered IO lines and adapter signals */
#define ST7_PA_RTCK     ST7_PA_IO0
#define ST7_PA_NTRST    ST7_PA_IO1
#define ST7_PC_TDI      ST7_PC_IO3
#define ST7_PA_DBGRQ    ST7_PA_IO4
#define ST7_PB_NSRST    ST7_PB_IO5
#define ST7_PE_TMS      ST7_PE_IO6
#define ST7_PC_TCK      ST7_PC_IO7
#define ST7_PC_TDO      ST7_PC_IO9
#define ST7_PA_DBGACK   ST7_PA_IO10

static usb_dev_handle *pHDev;

/*
 * ep1 commands are up to USB_EP1OUT_SIZE bytes in length.
 * This function takes care of zeroing the unused bytes before sending the packet.
 * Any reply packet is not handled by this function.
 */
static int ep1_generic_commandl(usb_dev_handle *pHDev_param, size_t length, ...)
{
	uint8_t usb_buffer[USB_EP1OUT_SIZE];
	uint8_t *usb_buffer_p;
	va_list ap;
	int usb_ret;

	if (length > sizeof(usb_buffer))
		length = sizeof(usb_buffer);

	usb_buffer_p = usb_buffer;

	va_start(ap, length);
	while (length > 0) {
		*usb_buffer_p++ = va_arg(ap, int);
		length--;
	}

	memset(
		usb_buffer_p,
		0,
		sizeof(usb_buffer) - (usb_buffer_p - usb_buffer)
		);

	usb_ret = usb_bulk_write(
			pHDev_param,
			USB_EP1OUT_ADDR,
			(char *)usb_buffer, sizeof(usb_buffer),
			USB_TIMEOUT_MS
			);

	return usb_ret;
}

#if 0
static ssize_t ep1_memory_read(
	usb_dev_handle *pHDev, uint16_t addr,
	size_t length, uint8_t *buffer)
{
	uint8_t usb_buffer[USB_EP1OUT_SIZE];
	int usb_ret;
	size_t remain;
	ssize_t count;

	usb_buffer[0] = EP1_CMD_MEMORY_READ;
	memset(
		usb_buffer + 4,
		0,
		sizeof(usb_buffer) - 4
		);

	remain = length;
	count = 0;

	while (remain) {
		if (remain > sizeof(usb_buffer))
			length = sizeof(usb_buffer);
		else
			length = remain;

		usb_buffer[1] = addr >> 8;
		usb_buffer[2] = addr;
		usb_buffer[3] = length;

		usb_ret = usb_bulk_write(
				pHDev, USB_EP1OUT_ADDR,
				usb_buffer, sizeof(usb_buffer),
				USB_TIMEOUT_MS
				);

		if (usb_ret < sizeof(usb_buffer))
			break;

		usb_ret = usb_bulk_read(
				pHDev, USB_EP1IN_ADDR,
				buffer, length,
				USB_TIMEOUT_MS
				);

		if (usb_ret < length)
			break;

		addr += length;
		buffer += length;
		count += length;
		remain -= length;
	}

	return count;
}
#endif

static ssize_t ep1_memory_write(usb_dev_handle *pHDev_param, uint16_t addr,
	size_t length, uint8_t const *buffer)
{
	uint8_t usb_buffer[USB_EP1OUT_SIZE];
	int usb_ret;
	size_t remain;
	ssize_t count;

	usb_buffer[0] = EP1_CMD_MEMORY_WRITE;

	remain = length;
	count = 0;

	while (remain) {
		if (remain > (sizeof(usb_buffer) - 4))
			length = (sizeof(usb_buffer) - 4);
		else
			length = remain;

		usb_buffer[1] = addr >> 8;
		usb_buffer[2] = addr;
		usb_buffer[3] = length;
		memcpy(
			usb_buffer + 4,
			buffer,
			length
			);
		memset(
			usb_buffer + 4 + length,
			0,
			sizeof(usb_buffer) - 4 - length
			);

		usb_ret = usb_bulk_write(
				pHDev_param, USB_EP1OUT_ADDR,
				(char *)usb_buffer, sizeof(usb_buffer),
				USB_TIMEOUT_MS
				);

		if ((size_t)usb_ret < sizeof(usb_buffer))
			break;

		addr += length;
		buffer += length;
		count += length;
		remain -= length;
	}

	return count;
}


#if 0
static ssize_t ep1_memory_writel(usb_dev_handle *pHDev, uint16_t addr,
	size_t length, ...)
{
	uint8_t buffer[USB_EP1OUT_SIZE - 4];
	uint8_t *buffer_p;
	va_list ap;
	size_t remain;

	if (length > sizeof(buffer))
		length = sizeof(buffer);

	remain = length;
	buffer_p = buffer;

	va_start(ap, length);
	while (remain > 0) {
		*buffer_p++ = va_arg(ap, int);
		remain--;
	}

	return ep1_memory_write(pHDev, addr, length, buffer);
}
#endif

#define DTCLOAD_COMMENT         (0)
#define DTCLOAD_ENTRY           (1)
#define DTCLOAD_LOAD            (2)
#define DTCLOAD_RUN                     (3)
#define DTCLOAD_LUT_START       (4)
#define DTCLOAD_LUT                     (5)

#define DTC_LOAD_BUFFER         ST7_USB_BUF_EP2UIDO

/* This gets set by the DTC loader */
static uint8_t dtc_entry_download;

/* The buffer is specially formatted to represent a valid image to load into the DTC. */
static int dtc_load_from_buffer(usb_dev_handle *pHDev_param, const uint8_t *buffer,
		size_t length)
{
	struct header_s {
		uint8_t type;
		uint8_t length;
	};

	int usb_err;
	struct header_s *header;
	uint8_t lut_start = 0xc0;

	dtc_entry_download = 0;

	/* Stop the DTC before loading anything. */
	usb_err = ep1_generic_commandl(
			pHDev_param, 1,
			EP1_CMD_DTC_STOP
			);
	if (usb_err < 0)
		return usb_err;

	while (length) {
		if (length < sizeof(*header)) {
			LOG_ERROR("Malformed DTC image");
			exit(1);
		}

		header = (struct header_s *)buffer;
		buffer += sizeof(*header);
		length -= sizeof(*header);

		if (length < (size_t)header->length + 1) {
			LOG_ERROR("Malformed DTC image");
			exit(1);
		}

		switch (header->type) {
			case DTCLOAD_COMMENT:
				break;

			case DTCLOAD_ENTRY:
				/* store entry addresses somewhere */
				if (!strncmp("download", (char *)buffer + 1, 8))
					dtc_entry_download = buffer[0];
				break;

			case DTCLOAD_LOAD:
				/* Send the DTC program to ST7 RAM. */
				usb_err = ep1_memory_write(
						pHDev_param,
						DTC_LOAD_BUFFER,
						header->length + 1, buffer
					);
				if (usb_err < 0)
					return usb_err;

				/* Load it into the DTC. */
				usb_err = ep1_generic_commandl(
						pHDev_param, 3,
						EP1_CMD_DTC_LOAD,
						(DTC_LOAD_BUFFER >> 8),
						DTC_LOAD_BUFFER
					);
				if (usb_err < 0)
					return usb_err;

				break;

			case DTCLOAD_RUN:
				usb_err = ep1_generic_commandl(
						pHDev_param, 3,
						EP1_CMD_DTC_CALL,
						buffer[0],
						EP1_CMD_DTC_WAIT
					);
				if (usb_err < 0)
					return usb_err;

				break;

			case DTCLOAD_LUT_START:
				lut_start = buffer[0];
				break;

			case DTCLOAD_LUT:
				usb_err = ep1_memory_write(
						pHDev_param,
						ST7_USB_BUF_EP0OUT + lut_start,
						header->length + 1, buffer
					);
				if (usb_err < 0)
					return usb_err;
				break;

			default:
				LOG_ERROR("Invalid DTC image record type: 0x%02x", header->type);
				exit(1);
				break;
		}

		buffer += (header->length + 1);
		length -= (header->length + 1);
	}

	return 0;
}

/*
 * Start the DTC running in download mode (waiting for 512 byte command packets on ep2).
 */
static int dtc_start_download(void)
{
	int usb_err;
	uint8_t ep2txr;

	/* set up for download mode and make sure EP2 is set up to transmit */
	usb_err = ep1_generic_commandl(
			pHDev, 7,

			EP1_CMD_DTC_STOP,
			EP1_CMD_SET_UPLOAD,
			EP1_CMD_SET_DOWNLOAD,
			EP1_CMD_MEMORY_READ,	/* read EP2TXR for its data toggle */
			ST7_EP2TXR >> 8,
			ST7_EP2TXR,
			1
			);
	if (usb_err < 0)
		return usb_err;

	/* read back ep2txr */
	usb_err = usb_bulk_read(
			pHDev, USB_EP1IN_ADDR,
			(char *)&ep2txr, 1,
			USB_TIMEOUT_MS
			);
	if (usb_err < 0)
		return usb_err;

	usb_err = ep1_generic_commandl(
			pHDev, 13,

			EP1_CMD_MEMORY_WRITE,	/* preinitialize poll byte */
			DTC_STATUS_POLL_BYTE >> 8,
			DTC_STATUS_POLL_BYTE,
			1,
			0x00,
			EP1_CMD_MEMORY_WRITE,	/* set EP2IN to return data */
			ST7_EP2TXR >> 8,
			ST7_EP2TXR,
			1,
			(ep2txr & ST7_EP2TXR_DTOG_TX) | ST7_EP2TXR_STAT_VALID,
			EP1_CMD_DTC_CALL,	/* start running the DTC */
			dtc_entry_download,
			EP1_CMD_DTC_GET_CACHED_STATUS
			);
	if (usb_err < 0)
		return usb_err;

	/* wait for completion */
	usb_err = usb_bulk_read(
			pHDev, USB_EP1IN_ADDR,
			(char *)&ep2txr, 1,
			USB_TIMEOUT_MS
			);

	return usb_err;
}

static int dtc_run_download(
	usb_dev_handle *pHDev_param,
	uint8_t *command_buffer,
	int command_buffer_size,
	uint8_t *reply_buffer,
	int reply_buffer_size
	)
{
	char dtc_status;
	int usb_err;
	int i;

	LOG_DEBUG("%d/%d", command_buffer_size, reply_buffer_size);

	usb_err = usb_bulk_write(
			pHDev_param,
			USB_EP2OUT_ADDR,
			(char *)command_buffer, USB_EP2BANK_SIZE,
			USB_TIMEOUT_MS
			);
	if (usb_err < 0)
		return usb_err;


	/* Wait for DTC to finish running command buffer */
	for (i = 50;; ) {
		usb_err = ep1_generic_commandl(
				pHDev_param, 4,

				EP1_CMD_MEMORY_READ,
				DTC_STATUS_POLL_BYTE >> 8,
				DTC_STATUS_POLL_BYTE,
				1
				);
		if (usb_err < 0)
			return usb_err;

		usb_err = usb_bulk_read(
				pHDev_param,
				USB_EP1IN_ADDR,
				&dtc_status, 1,
				USB_TIMEOUT_MS
				);
		if (usb_err < 0)
			return usb_err;

		if (dtc_status & 0x01)
			break;

		if (!--i) {
			LOG_ERROR("too many retries waiting for DTC status");
			return -ETIMEDOUT;
		}
	}


	if (reply_buffer && reply_buffer_size) {
		usb_err = usb_bulk_read(
				pHDev_param,
				USB_EP2IN_ADDR,
				(char *)reply_buffer, reply_buffer_size,
				USB_TIMEOUT_MS
				);

		if (usb_err < reply_buffer_size) {
			LOG_ERROR("Read of endpoint 2 returned %d, expected %d",
				usb_err, reply_buffer_size
				);
			return usb_err;
		}
	}

	return usb_err;
}

/*
 * The dtc reply queue is a singly linked list that describes what to do
 * with the reply packet that comes from the DTC.  Only SCAN_IN and SCAN_IO generate
 * these entries.
 */

struct dtc_reply_queue_entry {
	struct dtc_reply_queue_entry *next;
	struct jtag_command *cmd;	/* the command that resulted in this entry */

	struct {
		uint8_t *buffer;		/* the scan buffer */
		int size;			/* size of the scan buffer in bits */
		int offset;			/* how many bits were already done before this? */
		int length;			/* how many bits are processed in this operation? */
		enum scan_type type;		/* SCAN_IN/SCAN_OUT/SCAN_IO */
	} scan;
};


/*
 * The dtc_queue consists of a buffer of pending commands and a reply queue.
 * rlink_scan and tap_state_run add to the command buffer and maybe to the reply queue.
 */

static struct {
	struct dtc_reply_queue_entry *rq_head;
	struct dtc_reply_queue_entry *rq_tail;
	uint32_t cmd_index;
	uint32_t reply_index;
	uint8_t cmd_buffer[USB_EP2BANK_SIZE];
} dtc_queue;

/*
 * The tap state queue is for accumulating TAP state changes without needlessly
 * flushing the dtc_queue.  When it fills or is run, it adds the accumulated bytes to
 * the dtc_queue.
 */

static struct {
	uint32_t length;
	uint32_t buffer;
} tap_state_queue;

static int dtc_queue_init(void)
{
	dtc_queue.rq_head = NULL;
	dtc_queue.rq_tail = NULL;
	dtc_queue.cmd_index = 0;
	dtc_queue.reply_index = 0;
	return 0;
}

static inline struct dtc_reply_queue_entry *dtc_queue_enqueue_reply(
	enum scan_type type, uint8_t *buffer, int size, int offset,
	int length, struct jtag_command *cmd)
{
	struct dtc_reply_queue_entry *rq_entry;

	rq_entry = malloc(sizeof(struct dtc_reply_queue_entry));
	if (rq_entry != NULL) {
		rq_entry->scan.type = type;
		rq_entry->scan.buffer = buffer;
		rq_entry->scan.size = size;
		rq_entry->scan.offset = offset;
		rq_entry->scan.length = length;
		rq_entry->cmd = cmd;
		rq_entry->next = NULL;

		if (dtc_queue.rq_head == NULL)
			dtc_queue.rq_head = rq_entry;
		else
			dtc_queue.rq_tail->next = rq_entry;

		dtc_queue.rq_tail = rq_entry;
	}

	return rq_entry;
}

/*
 * Running the queue means that any pending command buffer is run
 * and any reply data dealt with.  The command buffer is then cleared for subsequent processing.
 * The queue is automatically run by append when it is necessary to get space for the append.
 */

static int dtc_queue_run(void)
{
	struct dtc_reply_queue_entry *rq_p, *rq_next;
	int retval;
	int usb_err;
	int bit_cnt;
	int x;
	uint8_t *dtc_p, *tdo_p;
	uint8_t dtc_mask, tdo_mask;
	uint8_t reply_buffer[USB_EP2IN_SIZE];

	assert((dtc_queue.rq_head != 0) == (dtc_queue.reply_index > 0));
	assert(dtc_queue.cmd_index < USB_EP2BANK_SIZE);
	assert(dtc_queue.reply_index <= USB_EP2IN_SIZE);

	retval = ERROR_OK;

	if (dtc_queue.cmd_index < 1)
		return retval;

	dtc_queue.cmd_buffer[dtc_queue.cmd_index++] = DTC_CMD_STOP;

	usb_err = dtc_run_download(pHDev,
			dtc_queue.cmd_buffer, dtc_queue.cmd_index,
			reply_buffer, sizeof(reply_buffer)
			);
	if (usb_err < 0) {
		LOG_ERROR("dtc_run_download: %s", usb_strerror());
		exit(1);
	}

	if (dtc_queue.rq_head != NULL) {
		/* process the reply, which empties the reply queue and frees its entries */
		dtc_p = reply_buffer;

		/* The rigamarole with the masks and doing it bit-by-bit is due to the fact that the
		 *scan buffer is LSb-first and the DTC code is MSb-first for hardware reasons.   It
		 *was that or craft a function to do the reversal, and that wouldn't work with
		 *bit-stuffing (supplying extra bits to use mostly byte operations), or any other
		 *scheme which would throw the byte alignment off. */

		for (
			rq_p = dtc_queue.rq_head;
			rq_p != NULL;
			rq_p = rq_next
			) {
			tdo_p = rq_p->scan.buffer + (rq_p->scan.offset / 8);
			tdo_mask = 1 << (rq_p->scan.offset % 8);


			bit_cnt = rq_p->scan.length;
			if (bit_cnt >= 8) {
				/* bytes */

				dtc_mask = 1 << (8 - 1);

				for (
					;
					bit_cnt;
					bit_cnt--
					) {
					if (*dtc_p & dtc_mask)
						*tdo_p |= tdo_mask;
					else
						*tdo_p &= ~tdo_mask;

					dtc_mask >>= 1;
					if (dtc_mask == 0) {
						dtc_p++;
						dtc_mask = 1 << (8 - 1);
					}

					tdo_mask <<= 1;
					if (tdo_mask == 0) {
						tdo_p++;
						tdo_mask = 1;
					}
				}
			} else {
				/*  extra bits or last bit */

				x = *dtc_p++;
				if ((rq_p->scan.type == SCAN_IN) && (
						rq_p->scan.offset != rq_p->scan.size - 1
					)) {
					/* extra bits were sent as a full byte with padding on the
					 *end */
					dtc_mask = 1 << (8 - 1);
				} else
					dtc_mask = 1 << (bit_cnt - 1);

				for (
					;
					bit_cnt;
					bit_cnt--
					) {
					if (x & dtc_mask)
						*tdo_p |= tdo_mask;
					else
						*tdo_p &= ~tdo_mask;

					dtc_mask >>= 1;

					tdo_mask <<= 1;
					if (tdo_mask == 0) {
						tdo_p++;
						tdo_mask = 1;
					}

				}
			}

			if ((rq_p->scan.offset + rq_p->scan.length) >= rq_p->scan.size) {
				/* feed scan buffer back into openocd and free it */
				if (jtag_read_buffer(rq_p->scan.buffer,
						rq_p->cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				free(rq_p->scan.buffer);
			}

			rq_next = rq_p->next;
			free(rq_p);
		}
		dtc_queue.rq_head = NULL;
		dtc_queue.rq_tail = NULL;
	}

	/* reset state for new appends */
	dtc_queue.cmd_index = 0;
	dtc_queue.reply_index = 0;

	return retval;
}

/* runs the queue if it cannot take reserved_cmd bytes of command data
 * or reserved_reply bytes of reply data */
static int dtc_queue_run_if_full(int reserved_cmd, int reserved_reply)
{
	/* reserve one additional byte for the STOP cmd appended during run */
	if (dtc_queue.cmd_index + reserved_cmd + 1 > USB_EP2BANK_SIZE)
		return dtc_queue_run();

	if (dtc_queue.reply_index + reserved_reply > USB_EP2IN_SIZE)
		return dtc_queue_run();

	return ERROR_OK;
}

static int tap_state_queue_init(void)
{
	tap_state_queue.length = 0;
	tap_state_queue.buffer = 0;
	return 0;
}

static int tap_state_queue_run(void)
{
	int i;
	int bits;
	uint8_t byte_param;
	int retval;

	retval = 0;
	if (!tap_state_queue.length)
		return retval;
	bits = 1;
	byte_param = 0;
	for (i = tap_state_queue.length; i--; ) {

		byte_param <<= 1;
		if (tap_state_queue.buffer & 1)
			byte_param |= 1;
		if ((bits >= 8) || !i) {
			byte_param <<= (8 - bits);

			/* make sure there's room for two cmd bytes */
			dtc_queue_run_if_full(2, 0);

#ifdef USE_HARDWARE_SHIFTER_FOR_TMS
			if (bits == 8) {
				dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
					DTC_CMD_SHIFT_TMS_BYTES(1);
			} else {
#endif
			dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
				DTC_CMD_SHIFT_TMS_BITS(bits);
#ifdef USE_HARDWARE_SHIFTER_FOR_TMS
		}
#endif

			dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
				byte_param;

			byte_param = 0;
			bits = 1;
		} else
			bits++;

		tap_state_queue.buffer >>= 1;
	}
	retval = tap_state_queue_init();
	return retval;
}

static int tap_state_queue_append(uint8_t tms)
{
	int retval;

	if (tap_state_queue.length >= sizeof(tap_state_queue.buffer) * 8) {
		retval = tap_state_queue_run();
		if (retval != 0)
			return retval;
	}

	if (tms)
		tap_state_queue.buffer |= (1 << tap_state_queue.length);
	tap_state_queue.length++;

	return 0;
}

static void rlink_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void rlink_state_move(void)
{

	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		tap_state_queue_append(tms);
	}

	tap_set_state(tap_get_end_state());
}

static void rlink_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
			tms = 0;
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		tap_state_queue_append(tms);

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	tap_set_end_state(tap_get_state());
}

static void rlink_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in RTI */
	if (tap_get_state() != TAP_IDLE) {
		rlink_end_state(TAP_IDLE);
		rlink_state_move();
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
		tap_state_queue_append(0);

	/* finish in end_state */
	rlink_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		rlink_state_move();
}

/* (1) assert or (0) deassert reset lines */
static void rlink_reset(int trst, int srst)
{
	uint8_t bitmap;
	int usb_err;

	/* Read port A for bit op */
	usb_err = ep1_generic_commandl(
			pHDev, 4,
			EP1_CMD_MEMORY_READ,
			ST7_PADR >> 8,
			ST7_PADR,
			1
			);
	if (usb_err < 0) {
		LOG_ERROR("%s", usb_strerror());
		exit(1);
	}

	usb_err = usb_bulk_read(
			pHDev, USB_EP1IN_ADDR,
			(char *)&bitmap, 1,
			USB_TIMEOUT_MS
			);
	if (usb_err < 1) {
		LOG_ERROR("%s", usb_strerror());
		exit(1);
	}

	if (trst)
		bitmap &= ~ST7_PA_NTRST;
	else
		bitmap |= ST7_PA_NTRST;

	/* Write port A and read port B for bit op
	 * port B has no OR, and we want to emulate open drain on NSRST, so we initialize DR to 0
	 *and assert NSRST by setting DDR to 1. */
	usb_err = ep1_generic_commandl(
			pHDev, 9,
			EP1_CMD_MEMORY_WRITE,
			ST7_PADR >> 8,
			ST7_PADR,
			1,
			bitmap,
			EP1_CMD_MEMORY_READ,
			ST7_PBDDR >> 8,
			ST7_PBDDR,
			1
			);
	if (usb_err < 0) {
		LOG_ERROR("%s", usb_strerror());
		exit(1);
	}

	usb_err = usb_bulk_read(
			pHDev, USB_EP1IN_ADDR,
			(char *)&bitmap, 1,
			USB_TIMEOUT_MS
			);
	if (usb_err < 1) {
		LOG_ERROR("%s", usb_strerror());
		exit(1);
	}

	if (srst)
		bitmap |= ST7_PB_NSRST;
	else
		bitmap &= ~ST7_PB_NSRST;

	/* write port B and read dummy to ensure completion before returning */
	usb_err = ep1_generic_commandl(
			pHDev, 6,
			EP1_CMD_MEMORY_WRITE,
			ST7_PBDDR >> 8,
			ST7_PBDDR,
			1,
			bitmap,
			EP1_CMD_DTC_GET_CACHED_STATUS
			);
	if (usb_err < 0) {
		LOG_ERROR("%s", usb_strerror());
		exit(1);
	}

	usb_err = usb_bulk_read(
			pHDev, USB_EP1IN_ADDR,
			(char *)&bitmap, 1,
			USB_TIMEOUT_MS
			);
	if (usb_err < 1) {
		LOG_ERROR("%s", usb_strerror());
		exit(1);
	}
}

static int rlink_scan(struct jtag_command *cmd, enum scan_type type,
		uint8_t *buffer, int scan_size)
{
	bool ir_scan;
	tap_state_t saved_end_state;
	int byte_bits;
	int extra_bits;
	int chunk_bits;
	int chunk_bytes;
	int x;

	int tdi_bit_offset;
	uint8_t tdi_mask, *tdi_p;
	uint8_t dtc_mask;

	if (scan_size < 1) {
		LOG_ERROR("scan_size cannot be less than 1 bit");
		exit(1);
	}

	ir_scan = cmd->cmd.scan->ir_scan;

	/* Move to the proper state before starting to shift TDI/TDO. */
	if (!((!ir_scan && (tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		saved_end_state = tap_get_end_state();
		rlink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
		rlink_state_move();
		rlink_end_state(saved_end_state);
	}

	tap_state_queue_run();


#if 0
	printf("scan_size = %d, type = 0x%x\n", scan_size, type);
	{
		int i;

		/* clear unused bits in scan buffer for ease of debugging
		 * (it makes diffing output easier) */
		buffer[scan_size / 8] &= ((1 << ((scan_size - 1) % 8) + 1) - 1);

		printf("before scan:");
		for (i = 0; i < (scan_size + 7) / 8; i++)
			printf(" %02x", buffer[i]);
		printf("\n");
	}
#endif

	/* The number of bits that can be shifted as complete bytes */
	byte_bits = (int)(scan_size - 1) / 8 * 8;
	/* The number of bits left over, not counting the last bit */
	extra_bits = (scan_size - 1) - byte_bits;

	tdi_bit_offset = 0;
	tdi_p = buffer;
	tdi_mask = 1;

	if (extra_bits && (type == SCAN_OUT)) {
		/* Schedule any extra bits into the DTC command buffer, padding as needed
		 * For SCAN_OUT, this comes before the full bytes so the (leading) padding bits will
		 *fall off the end */

		/* make sure there's room for two cmd bytes */
		dtc_queue_run_if_full(2, 0);

		x = 0;
		dtc_mask = 1 << (extra_bits - 1);

		while (extra_bits--) {
			if (*tdi_p & tdi_mask)
				x |= dtc_mask;

			dtc_mask >>= 1;

			tdi_mask <<= 1;
			if (tdi_mask == 0) {
				tdi_p++;
				tdi_mask = 1;
			}
		}

		dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
			DTC_CMD_SHIFT_TDI_BYTES(1);

		dtc_queue.cmd_buffer[dtc_queue.cmd_index++] = x;
	}

	/* Loop scheduling full bytes into the DTC command buffer */
	while (byte_bits) {
		/* make sure there's room for one (for in scans) or two cmd bytes and
		 * at least one reply byte for in or inout scans*/
		dtc_queue_run_if_full(type == SCAN_IN ? 1 : 2, type != SCAN_OUT ? 1 : 0);

		chunk_bits = byte_bits;
		/* we can only use up to 16 bytes at a time */
		if (chunk_bits > (16 * 8))
			chunk_bits = (16 * 8);

		if (type != SCAN_IN) {
			/* how much is there room for, considering stop and byte op? */
			x = (sizeof(dtc_queue.cmd_buffer) - (dtc_queue.cmd_index + 1 + 1)) * 8;
			if (chunk_bits > x)
				chunk_bits = x;
		}

		if (type != SCAN_OUT) {
			/* how much is there room for in the reply buffer? */
			x = (USB_EP2IN_SIZE - dtc_queue.reply_index) * 8;
			if (chunk_bits > x)
				chunk_bits = x;
		}

		/* so the loop will end */
		byte_bits -= chunk_bits;

		if (type != SCAN_OUT) {
			if (dtc_queue_enqueue_reply(
					type, buffer, scan_size, tdi_bit_offset,
					chunk_bits,
					cmd
				) == NULL) {
				LOG_ERROR("enqueuing DTC reply entry: %s", strerror(errno));
				exit(1);
			}
			dtc_queue.reply_index += (chunk_bits + 7) / 8;

			tdi_bit_offset += chunk_bits;
		}

		/* chunk_bits is a multiple of 8, so there are no rounding issues. */
		chunk_bytes = chunk_bits / 8;

		switch (type) {
			case SCAN_IN:
				x = DTC_CMD_SHIFT_TDO_BYTES(chunk_bytes);
				break;
			case SCAN_OUT:
				x = DTC_CMD_SHIFT_TDI_BYTES(chunk_bytes);
				break;
			default:
				x = DTC_CMD_SHIFT_TDIO_BYTES(chunk_bytes);
				break;
		}
		dtc_queue.cmd_buffer[dtc_queue.cmd_index++] = x;

		if (type != SCAN_IN) {
			x = 0;
			dtc_mask = 1 << (8 - 1);

			while (chunk_bits--) {
				if (*tdi_p & tdi_mask)
					x |= dtc_mask;

				dtc_mask >>= 1;
				if (dtc_mask == 0) {
					dtc_queue.cmd_buffer[dtc_queue.cmd_index++] = x;
					x = 0;
					dtc_mask = 1 << (8 - 1);
				}

				tdi_mask <<= 1;
				if (tdi_mask == 0) {
					tdi_p++;
					tdi_mask = 1;
				}
			}
		}
	}

	if (extra_bits && (type != SCAN_OUT)) {
		/* Schedule any extra bits into the DTC command buffer */

		/* make sure there's room for one (for in scans) or two cmd bytes
		 * and one reply byte */
		dtc_queue_run_if_full(type == SCAN_IN ? 1 : 2, 1);

		if (dtc_queue_enqueue_reply(
				type, buffer, scan_size, tdi_bit_offset,
				extra_bits,
				cmd
			) == NULL) {
			LOG_ERROR("enqueuing DTC reply entry: %s", strerror(errno));
			exit(1);
		}

		dtc_queue.reply_index++;

		tdi_bit_offset += extra_bits;

		if (type == SCAN_IN) {
			dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
				DTC_CMD_SHIFT_TDO_BYTES(1);

		} else {
			dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
				DTC_CMD_SHIFT_TDIO_BITS(extra_bits);

			x = 0;
			dtc_mask = 1 << (8 - 1);

			while (extra_bits--) {
				if (*tdi_p & tdi_mask)
					x |= dtc_mask;

				dtc_mask >>= 1;

				tdi_mask <<= 1;
				if (tdi_mask == 0) {
					tdi_p++;
					tdi_mask = 1;
				}
			}

			dtc_queue.cmd_buffer[dtc_queue.cmd_index++] = x;
		}
	}

	/* Schedule the last bit into the DTC command buffer */

	/* make sure there's room for one cmd byte and one reply byte
	 * for in or inout scans*/
	dtc_queue_run_if_full(1, type == SCAN_OUT ? 0 : 1);

	if (type == SCAN_OUT) {
		dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
			DTC_CMD_SHIFT_TMS_TDI_BIT_PAIR(1, (*tdi_p & tdi_mask), 0);

	} else {
		if (dtc_queue_enqueue_reply(
				type, buffer, scan_size, tdi_bit_offset,
				1,
				cmd
				) == NULL) {
			LOG_ERROR("enqueuing DTC reply entry: %s", strerror(errno));
			exit(1);
		}

		dtc_queue.reply_index++;

		dtc_queue.cmd_buffer[dtc_queue.cmd_index++] =
			DTC_CMD_SHIFT_TMS_TDI_BIT_PAIR(1, (*tdi_p & tdi_mask), 1);
	}

	/* Move to pause state */
	tap_state_queue_append(0);
	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);
	if (tap_get_state() != tap_get_end_state())
		rlink_state_move();

	return 0;
}

static int rlink_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval, tmp_retval;

	/* return ERROR_OK, unless something goes wrong */
	retval = ERROR_OK;

#ifndef AUTOMATIC_BUSY_LED
	/* turn LED on */
	ep1_generic_commandl(pHDev, 2,
		EP1_CMD_SET_PORTD_LEDS,
		~(ST7_PD_NBUSY_LED)
		);
#endif

	while (cmd) {
		switch (cmd->type) {
			case JTAG_RUNTEST:
			case JTAG_TLR_RESET:
			case JTAG_PATHMOVE:
			case JTAG_SCAN:
				break;

			default:
				/* some events, such as resets, need a queue flush to ensure
				 *consistency */
				tap_state_queue_run();
				dtc_queue_run();
				break;
		}

		switch (cmd->type) {
			case JTAG_RESET:
				LOG_DEBUG_IO("reset trst: %i srst %i",
						cmd->cmd.reset->trst,
						cmd->cmd.reset->srst);
				if ((cmd->cmd.reset->trst == 1) ||
						(cmd->cmd.reset->srst &&
								(jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
					tap_set_state(TAP_RESET);
				rlink_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
				LOG_DEBUG_IO("runtest %i cycles, end in %i",
						cmd->cmd.runtest->num_cycles,
						cmd->cmd.runtest->end_state);
				if (cmd->cmd.runtest->end_state != -1)
					rlink_end_state(cmd->cmd.runtest->end_state);
				rlink_runtest(cmd->cmd.runtest->num_cycles);
				break;
			case JTAG_TLR_RESET:
				LOG_DEBUG_IO("statemove end in %i", cmd->cmd.statemove->end_state);
				if (cmd->cmd.statemove->end_state != -1)
					rlink_end_state(cmd->cmd.statemove->end_state);
				rlink_state_move();
				break;
			case JTAG_PATHMOVE:
				LOG_DEBUG_IO("pathmove: %i states, end in %i",
						cmd->cmd.pathmove->num_states,
						cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
				rlink_path_move(cmd->cmd.pathmove);
				break;
			case JTAG_SCAN:
				LOG_DEBUG_IO("%s scan end in %i",
						(cmd->cmd.scan->ir_scan) ? "IR" : "DR",
								cmd->cmd.scan->end_state);
				if (cmd->cmd.scan->end_state != -1)
					rlink_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				if (rlink_scan(cmd, type, buffer, scan_size) != ERROR_OK)
					retval = ERROR_FAIL;
				break;
			case JTAG_SLEEP:
				LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	/* Flush the DTC queue to make sure any pending reads have been done before exiting this
	 *function */
	tap_state_queue_run();
	tmp_retval = dtc_queue_run();
	if (tmp_retval != ERROR_OK)
		retval = tmp_retval;

#ifndef AUTOMATIC_BUSY_LED
	/* turn LED off */
	ep1_generic_commandl(pHDev, 2,
		EP1_CMD_SET_PORTD_LEDS,
		~0
		);
#endif

	return retval;
}

/* Using an unindexed table because it is infrequently accessed and it is short.  The table must be
 *in order of ascending speed (and descending prescaler), as it is scanned in reverse. */

static int rlink_speed(int speed)
{
	int i;

	if (speed == 0) {
		/* fastest speed */
		speed = rlink_speed_table[rlink_speed_table_size - 1].prescaler;
	}

	for (i = rlink_speed_table_size; i--; ) {
		if (rlink_speed_table[i].prescaler == speed) {
			if (dtc_load_from_buffer(pHDev, rlink_speed_table[i].dtc,
					rlink_speed_table[i].dtc_size) != 0) {
				LOG_ERROR(
					"An error occurred while trying to load DTC code for speed \"%d\".",
					speed);
				exit(1);
			}

			if (dtc_start_download() < 0) {
				LOG_ERROR("starting DTC: %s", usb_strerror());
				exit(1);
			}

			return ERROR_OK;
		}
	}

	LOG_ERROR("%d is not a supported speed", speed);
	return ERROR_FAIL;
}

static int rlink_speed_div(int speed, int *khz)
{
	int i;

	for (i = rlink_speed_table_size; i--; ) {
		if (rlink_speed_table[i].prescaler == speed) {
			*khz = rlink_speed_table[i].khz;
			return ERROR_OK;
		}
	}

	LOG_ERROR("%d is not a supported speed", speed);
	return ERROR_FAIL;
}

static int rlink_khz(int khz, int *speed)
{
	int i;

	if (khz == 0) {
		LOG_ERROR("RCLK not supported");
		return ERROR_FAIL;
	}

	for (i = rlink_speed_table_size; i--; ) {
		if (rlink_speed_table[i].khz <= khz) {
			*speed = rlink_speed_table[i].prescaler;
			return ERROR_OK;
		}
	}

	LOG_WARNING("The lowest supported JTAG speed is %d KHz", rlink_speed_table[0].khz);
	*speed = rlink_speed_table[0].prescaler;
	return ERROR_OK;
}

static int rlink_init(void)
{
	int i, j, retries;
	uint8_t reply_buffer[USB_EP1IN_SIZE];

	usb_init();
	const uint16_t vids[] = { USB_IDVENDOR, 0 };
	const uint16_t pids[] = { USB_IDPRODUCT, 0 };
	if (jtag_usb_open(vids, pids, &pHDev) != ERROR_OK)
		return ERROR_FAIL;

	struct usb_device *dev = usb_device(pHDev);
	if (dev->descriptor.bNumConfigurations > 1) {
		LOG_ERROR("Whoops! NumConfigurations is not 1, don't know what to do...");
		return ERROR_FAIL;
	}
	if (dev->config->bNumInterfaces > 1) {
		LOG_ERROR("Whoops! NumInterfaces is not 1, don't know what to do...");
		return ERROR_FAIL;
	}

	LOG_DEBUG("Opened device, pHDev = %p", pHDev);

	/* usb_set_configuration required under win32 */
	usb_set_configuration(pHDev, dev->config[0].bConfigurationValue);

	retries = 3;
	do {
		i = usb_claim_interface(pHDev, 0);
		if (i) {
			LOG_ERROR("usb_claim_interface: %s", usb_strerror());
#ifdef LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP
			j = usb_detach_kernel_driver_np(pHDev, 0);
			if (j)
				LOG_ERROR("detach kernel driver: %s", usb_strerror());
#endif
		} else {
			LOG_DEBUG("interface claimed!");
			break;
		}
	} while (--retries);

	if (i) {
		LOG_ERROR("Initialisation failed.");
		return ERROR_FAIL;
	}
	if (usb_set_altinterface(pHDev, 0) != 0) {
		LOG_ERROR("Failed to set interface.");
		return ERROR_FAIL;
	}

	/* The device starts out in an unknown state on open.  As such,
	 * result reads time out, and it's not even known whether the
	 * command was accepted.  So, for this first command, we issue
	 * it repeatedly until its response doesn't time out.  Also, if
	 * sending a command is going to time out, we find that out here.
	 *
	 * It must be possible to open the device in such a way that
	 * this special magic isn't needed, but, so far, it escapes us.
	 */
	for (i = 0; i < 5; i++) {
		j = ep1_generic_commandl(
				pHDev, 1,
				EP1_CMD_GET_FWREV
				);
		if (j < USB_EP1OUT_SIZE) {
			LOG_ERROR("USB write error: %s", usb_strerror());
			return ERROR_FAIL;
		}
		j = usb_bulk_read(
				pHDev, USB_EP1IN_ADDR,
				(char *)reply_buffer, sizeof(reply_buffer),
				200
				);
		if (j != -ETIMEDOUT)
			break;
	}

	if (j < (int)sizeof(reply_buffer)) {
		LOG_ERROR("USB read error: %s", usb_strerror());
		return ERROR_FAIL;
	}
	LOG_DEBUG(INTERFACE_NAME " firmware version: %d.%d.%d",
		reply_buffer[0],
		reply_buffer[1],
		reply_buffer[2]);

	if ((reply_buffer[0] != 0) || (reply_buffer[1] != 0) || (reply_buffer[2] != 3))
		LOG_WARNING(
			"The rlink device is not of the version that the developers have played with.  It may or may not work.");

	/* Probe port E for adapter presence */
	ep1_generic_commandl(
		pHDev, 16,
		EP1_CMD_MEMORY_WRITE,		/* Drive sense pin with 0 */
		ST7_PEDR >> 8,
		ST7_PEDR,
		3,
		0x00,							/* DR */
		ST7_PE_ADAPTER_SENSE_OUT,		/* DDR */
		ST7_PE_ADAPTER_SENSE_OUT,		/* OR */
		EP1_CMD_MEMORY_READ,		/* Read back */
		ST7_PEDR >> 8,
		ST7_PEDR,
		1,
		EP1_CMD_MEMORY_WRITE,		/* Drive sense pin with 1 */
		ST7_PEDR >> 8,
		ST7_PEDR,
		1,
		ST7_PE_ADAPTER_SENSE_OUT
		);

	usb_bulk_read(
		pHDev, USB_EP1IN_ADDR,
		(char *)reply_buffer, 1,
		USB_TIMEOUT_MS
		);

	if ((reply_buffer[0] & ST7_PE_ADAPTER_SENSE_IN) != 0)
		LOG_WARNING("target detection problem");

	ep1_generic_commandl(
		pHDev, 11,
		EP1_CMD_MEMORY_READ,		/* Read back */
		ST7_PEDR >> 8,
		ST7_PEDR,
		1,
		EP1_CMD_MEMORY_WRITE,		/* float port E */
		ST7_PEDR >> 8,
		ST7_PEDR,
		3,
		0x00,		/* DR */
		0x00,		/* DDR */
		0x00		/* OR */
		);

	usb_bulk_read(
		pHDev, USB_EP1IN_ADDR,
		(char *)reply_buffer, 1,
		USB_TIMEOUT_MS
		);


	if ((reply_buffer[0] & ST7_PE_ADAPTER_SENSE_IN) == 0)
		LOG_WARNING("target not plugged in");

	/* float ports A and B */
	ep1_generic_commandl(
		pHDev, 11,
		EP1_CMD_MEMORY_WRITE,
		ST7_PADDR >> 8,
		ST7_PADDR,
		2,
		0x00,
		0x00,
		EP1_CMD_MEMORY_WRITE,
		ST7_PBDDR >> 8,
		ST7_PBDDR,
		1,
		0x00
		);

	/* make sure DTC is stopped, set VPP control, set up ports A and B */
	ep1_generic_commandl(
		pHDev, 14,
		EP1_CMD_DTC_STOP,
		EP1_CMD_SET_PORTD_VPP,
		~(ST7_PD_VPP_SHDN),
		EP1_CMD_MEMORY_WRITE,
		ST7_PADR >> 8,
		ST7_PADR,
		2,
		((~(0)) & (ST7_PA_NTRST)),
		(ST7_PA_NTRST),
		/* port B has no OR, and we want to emulate open drain on NSRST, so we set DR to 0
		 *here and later assert NSRST by setting DDR bit to 1. */
		EP1_CMD_MEMORY_WRITE,
		ST7_PBDR >> 8,
		ST7_PBDR,
		1,
		0x00
		);

	/* set LED updating mode and make sure they're unlit */
	ep1_generic_commandl(
		pHDev, 3,
#ifdef AUTOMATIC_BUSY_LED
		EP1_CMD_LEDUE_BUSY,
#else
		EP1_CMD_LEDUE_NONE,
#endif
		EP1_CMD_SET_PORTD_LEDS,
		~0
		);

	tap_state_queue_init();
	dtc_queue_init();
	rlink_reset(0, 0);

	return ERROR_OK;
}

static int rlink_quit(void)
{
	/* stop DTC and make sure LEDs are off */
	ep1_generic_commandl(
		pHDev, 6,
		EP1_CMD_DTC_STOP,
		EP1_CMD_LEDUE_NONE,
		EP1_CMD_SET_PORTD_LEDS,
		~0,
		EP1_CMD_SET_PORTD_VPP,
		~0
		);

	usb_release_interface(pHDev, 0);
	usb_close(pHDev);

	return ERROR_OK;
}

static struct jtag_interface rlink_interface = {
	.execute_queue = rlink_execute_queue,
};

struct adapter_driver rlink_adapter_driver = {
	.name = "rlink",
	.transports = jtag_only,

	.init = rlink_init,
	.quit = rlink_quit,
	.speed = rlink_speed,
	.khz = rlink_khz,
	.speed_div = rlink_speed_div,

	.jtag_ops = &rlink_interface,
};
