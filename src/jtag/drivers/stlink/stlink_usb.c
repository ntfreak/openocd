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
#include "../libusb_helper.h"
#include <helper/binarybuffer.h>

#ifdef HAVE_LIBUSB1
#define USE_LIBUSB_ASYNCIO
#endif

#ifdef USE_LIBUSB_ASYNCIO

static LIBUSB_CALL void sync_transfer_cb(struct libusb_transfer *transfer)
{
	int *completed = transfer->user_data;
	*completed = 1;
	/* caller interprets result and frees transfer */
}


static void sync_transfer_wait_for_completion(struct libusb_transfer *transfer)
{
	int r, *completed = transfer->user_data;

	/* Assuming a single libusb context exists.  There no existing interface into this
	 * module to pass a libusb context.
	 */
	struct libusb_context *ctx = NULL;

	while (!*completed) {
		r = libusb_handle_events_completed(ctx, completed);
		if (r < 0) {
			if (r == LIBUSB_ERROR_INTERRUPTED)
				continue;
			libusb_cancel_transfer(transfer);
			continue;
		}
	}
}


static int transfer_error_status(const struct libusb_transfer *transfer)
{
	int r = 0;

	switch (transfer->status) {
		case LIBUSB_TRANSFER_COMPLETED:
			r = 0;
			break;
		case LIBUSB_TRANSFER_TIMED_OUT:
			r = LIBUSB_ERROR_TIMEOUT;
			break;
		case LIBUSB_TRANSFER_STALL:
			r = LIBUSB_ERROR_PIPE;
			break;
		case LIBUSB_TRANSFER_OVERFLOW:
			r = LIBUSB_ERROR_OVERFLOW;
			break;
		case LIBUSB_TRANSFER_NO_DEVICE:
			r = LIBUSB_ERROR_NO_DEVICE;
			break;
		case LIBUSB_TRANSFER_ERROR:
		case LIBUSB_TRANSFER_CANCELLED:
			r = LIBUSB_ERROR_IO;
			break;
		default:
			r = LIBUSB_ERROR_OTHER;
			break;
	}

	return r;
}

struct jtag_xfer {
	int ep;
	uint8_t *buf;
	size_t size;
	/* Internal */
	int retval;
	int completed;
	size_t transfer_size;
	struct libusb_transfer *transfer;
};

static int jtag_libusb_bulk_transfer_n(
		struct libusb_device_handle *dev_handle,
		struct jtag_xfer *transfers,
		size_t n_transfers,
		int timeout)
{
	int retval = 0;
	int returnval = ERROR_OK;


	for (size_t i = 0; i < n_transfers; ++i) {
		transfers[i].retval = 0;
		transfers[i].completed = 0;
		transfers[i].transfer_size = 0;
		transfers[i].transfer = libusb_alloc_transfer(0);

		if (transfers[i].transfer == NULL) {
			for (size_t j = 0; j < i; ++j)
				libusb_free_transfer(transfers[j].transfer);

			LOG_DEBUG("ERROR, failed to alloc usb transfers");
			for (size_t k = 0; k < n_transfers; ++k)
				transfers[k].retval = LIBUSB_ERROR_NO_MEM;
			return ERROR_FAIL;
		}
	}

	for (size_t i = 0; i < n_transfers; ++i) {
		libusb_fill_bulk_transfer(
				transfers[i].transfer,
				dev_handle,
				transfers[i].ep, transfers[i].buf, transfers[i].size,
				sync_transfer_cb, &transfers[i].completed, timeout);
		transfers[i].transfer->type = LIBUSB_TRANSFER_TYPE_BULK;

		retval = libusb_submit_transfer(transfers[i].transfer);
		if (retval < 0) {
			LOG_DEBUG("ERROR, failed to submit transfer %zu, error %d", i, retval);

			/* Probably no point continuing to submit transfers once a submission fails.
			 * As a result, tag all remaining transfers as errors.
			 */
			for (size_t j = i; j < n_transfers; ++j)
				transfers[j].retval = retval;

			returnval = ERROR_FAIL;
			break;
		}
	}

	/* Wait for every submitted USB transfer to complete.
	*/
	for (size_t i = 0; i < n_transfers; ++i) {
		if (transfers[i].retval == 0) {
			sync_transfer_wait_for_completion(transfers[i].transfer);

			retval = transfer_error_status(transfers[i].transfer);
			if (retval) {
				returnval = ERROR_FAIL;
				transfers[i].retval = retval;
				LOG_DEBUG("ERROR, transfer %zu failed, error %d", i, retval);
			} else {
				/* Assuming actual_length is only valid if there is no transfer error.
				 */
				transfers[i].transfer_size = transfers[i].transfer->actual_length;
			}
		}

		libusb_free_transfer(transfers[i].transfer);
		transfers[i].transfer = NULL;
	}

	return returnval;
}

static int stlink_usb_xfer_rw(void *handle, int cmdsize, const uint8_t *buf, int size)
{
	struct stlink_handle_s *h = handle;

	assert(handle != NULL);

	size_t n_transfers = 0;
	struct jtag_xfer transfers[2];

	memset(transfers, 0, sizeof(transfers));

	transfers[0].ep = h->tx_ep;
	transfers[0].buf = h->cmdbuf;
	transfers[0].size = cmdsize;

	++n_transfers;

	if (h->direction == h->tx_ep && size) {
		transfers[1].ep = h->tx_ep;
		transfers[1].buf = (uint8_t *)buf;
		transfers[1].size = size;

		++n_transfers;
	} else if (h->direction == h->rx_ep && size) {
		transfers[1].ep = h->rx_ep;
		transfers[1].buf = (uint8_t *)buf;
		transfers[1].size = size;

		++n_transfers;
	}

	return jtag_libusb_bulk_transfer_n(
			h->usb_backend_priv.fd,
			transfers,
			n_transfers,
			STLINK_WRITE_TIMEOUT);
}

#else

static int stlink_usb_xfer_rw(void *handle, int cmdsize, const uint8_t *buf, int size)
{
	struct stlink_handle_s *h = handle;
	int tr, ret;

	assert(handle != NULL);

	ret = jtag_libusb_bulk_write(h->usb_backend_priv.fd, h->tx_ep, (char *)h->cmdbuf,
				     cmdsize, STLINK_WRITE_TIMEOUT, &tr);
	if (ret || tr != cmdsize)
		return ERROR_FAIL;

	if (h->direction == h->tx_ep && size) {
		ret = jtag_libusb_bulk_write(h->usb_backend_priv.fd, h->tx_ep, (char *)buf,
					     size, STLINK_WRITE_TIMEOUT, &tr);
		if (ret || tr != size) {
			LOG_DEBUG("bulk write failed");
			return ERROR_FAIL;
		}
	} else if (h->direction == h->rx_ep && size) {
		ret = jtag_libusb_bulk_read(h->usb_backend_priv.fd, h->rx_ep, (char *)buf,
					    size, STLINK_READ_TIMEOUT, &tr);
		if (ret || tr != size) {
			LOG_DEBUG("bulk read failed");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

#endif


/** */
static int stlink_usb_xfer_v1_get_status(void *handle)
{
	struct stlink_handle_s *h = handle;
	int tr, ret;

	assert(handle != NULL);

	/* read status */
	memset(h->cmdbuf, 0, STLINK_SG_SIZE);

	ret = jtag_libusb_bulk_read(h->usb_backend_priv.fd, h->rx_ep, (char *)h->cmdbuf, 13,
				    STLINK_READ_TIMEOUT, &tr);
	if (ret || tr != 13)
		return ERROR_FAIL;

	uint32_t t1;

	t1 = buf_get_u32(h->cmdbuf, 0, 32);

	/* check for USBS */
	if (t1 != 0x53425355)
		return ERROR_FAIL;
	/*
	 * CSW status:
	 * 0 success
	 * 1 command failure
	 * 2 phase error
	 */
	if (h->cmdbuf[12] != 0)
		return ERROR_FAIL;

	return ERROR_OK;
}

/** */
static int stlink_usb_xfer_v1_get_sense(void *handle)
{
	int res;
	struct stlink_handle_s *h = handle;

	assert(handle != NULL);

	stlink_init_buffer(handle, h->rx_ep, 16);

	h->cmdbuf[h->cmdidx++] = REQUEST_SENSE;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = REQUEST_SENSE_LENGTH;

	res = stlink_usb_xfer_rw(handle, REQUEST_SENSE_LENGTH, h->databuf, 16);

	if (res != ERROR_OK)
		return res;

	if (stlink_usb_xfer_v1_get_status(handle) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/*
	transfers block in cmdbuf
	<size> indicates number of bytes in the following
	data phase.
	Ignore the (eventual) error code in the received packet.
*/
static int stlink_usb_xfer_noerrcheck(void *handle, const uint8_t *buf, int size)
{
	int err, cmdsize = STLINK_CMD_SIZE_V2;
	struct stlink_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.stlink == 1) {
		cmdsize = STLINK_SG_SIZE;
		/* put length in bCBWCBLength */
		h->cmdbuf[14] = h->cmdidx-15;
	}

	err = stlink_usb_xfer_rw(handle, cmdsize, buf, size);

	if (err != ERROR_OK)
		return err;

	if (h->version.stlink == 1) {
		if (stlink_usb_xfer_v1_get_status(handle) != ERROR_OK) {
			/* check csw status */
			if (h->cmdbuf[12] == 1) {
				LOG_DEBUG("get sense");
				if (stlink_usb_xfer_v1_get_sense(handle) != ERROR_OK)
					return ERROR_FAIL;
			}
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_read_trace(void *handle, const uint8_t *buf, int size)
{
	struct stlink_handle_s *h = handle;
	int tr, ret;

	ret = jtag_libusb_bulk_read(h->usb_backend_priv.fd, h->trace_ep, (char *)buf, size,
				    STLINK_READ_TIMEOUT, &tr);
	if (ret || tr != size) {
		LOG_ERROR("bulk trace read failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* Compute ST-Link serial number from the device descriptor
 * this function will help to work-around a bug in old ST-Link/V2 DFU
 * the buggy DFU returns an incorrect serial in the USB descriptor
 * example for the following serial "57FF72067265575742132067"
 *  - the correct descriptor serial is:
 *    0x32, 0x03, 0x35, 0x00, 0x37, 0x00, 0x46, 0x00, 0x46, 0x00, 0x37, 0x00, 0x32, 0x00 ...
 *    this contains the length (0x32 = 50), the type (0x3 = DT_STRING) and the serial in unicode format
 *    the serial part is: 0x0035, 0x0037, 0x0046, 0x0046, 0x0037, 0x0032 ... >>  57FF72 ...
 *    this format could be read correctly by 'libusb_get_string_descriptor_ascii'
 *    so this case is managed by libusb_helper::string_descriptor_equal
 *  - the buggy DFU is not doing any unicode conversion and returns a raw serial data in the descriptor
 *    0x1a, 0x03, 0x57, 0x00, 0xFF, 0x00, 0x72, 0x00 ...
 *            >>    57          FF          72       ...
 *    based on the length (0x1a = 26) we could easily decide if we have to fixup the serial
 *    and then we have just to convert the raw data into printable characters using sprintf
 */
static char *stlink_usb_get_alternate_serial(libusb_device_handle *device,
		struct libusb_device_descriptor *dev_desc)
{
	int usb_retval;
	unsigned char desc_serial[(STLINK_SERIAL_LEN + 1) * 2];

	if (dev_desc->iSerialNumber == 0)
		return NULL;

	/* get the LANGID from String Descriptor Zero */
	usb_retval = libusb_get_string_descriptor(device, 0, 0, desc_serial,
			sizeof(desc_serial));

	if (usb_retval < LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_string_descriptor() failed: %s(%d)",
				libusb_error_name(usb_retval), usb_retval);
		return NULL;
	} else if (usb_retval < 4) {
		/* the size should be least 4 bytes to contain a minimum of 1 supported LANGID */
		LOG_ERROR("could not get the LANGID");
		return NULL;
	}

	uint32_t langid = desc_serial[2] | (desc_serial[3] << 8);

	/* get the serial */
	usb_retval = libusb_get_string_descriptor(device, dev_desc->iSerialNumber,
			langid, desc_serial, sizeof(desc_serial));

	unsigned char len = desc_serial[0];

	if (usb_retval < LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_get_string_descriptor() failed: %s(%d)",
				libusb_error_name(usb_retval), usb_retval);
		return NULL;
	} else if (desc_serial[1] != LIBUSB_DT_STRING || len > usb_retval) {
		LOG_ERROR("invalid string in ST-LINK USB serial descriptor");
		return NULL;
	}

	if (len == ((STLINK_SERIAL_LEN + 1) * 2)) {
		/* good ST-Link adapter, this case is managed by
		 * libusb::libusb_get_string_descriptor_ascii */
		return NULL;
	} else if (len != ((STLINK_SERIAL_LEN / 2 + 1) * 2)) {
		LOG_ERROR("unexpected serial length (%d) in descriptor", len);
		return NULL;
	}

	/* else (len == 26) => buggy ST-Link */

	char *alternate_serial = malloc((STLINK_SERIAL_LEN + 1) * sizeof(char));
	if (alternate_serial == NULL)
		return NULL;

	for (unsigned int i = 0; i < STLINK_SERIAL_LEN; i += 2)
		sprintf(alternate_serial + i, "%02X", desc_serial[i + 2]);

	alternate_serial[STLINK_SERIAL_LEN] = '\0';

	return alternate_serial;
}

/** */
static int stlink_usb_open(void *handle, struct hl_interface_param_s *param)
{
	struct stlink_handle_s *h = handle;
	int err, retry_count = 1;

	h->cmdbuf = malloc(STLINK_SG_SIZE);
	h->databuf = malloc(STLINK_DATA_SIZE);

	if (h->cmdbuf == NULL || h->databuf == NULL)
		return ERROR_FAIL;

	/*
	  On certain host USB configurations(e.g. MacBook Air)
	  STLINKv2 dongle seems to have its FW in a funky state if,
	  after plugging it in, you try to use openocd with it more
	  then once (by launching and closing openocd). In cases like
	  that initial attempt to read the FW info via
	  stlink_usb_version will fail and the device has to be reset
	  in order to become operational.
	 */
	do {
		if (jtag_libusb_open(param->vid, param->pid, param->serial,
				&h->usb_backend_priv.fd, stlink_usb_get_alternate_serial) != ERROR_OK) {
			LOG_ERROR("open failed");
			return ERROR_FAIL;
		}

		jtag_libusb_set_configuration(h->usb_backend_priv.fd, 0);

		if (libusb_claim_interface(h->usb_backend_priv.fd, 0) != ERROR_OK) {
			LOG_DEBUG("claim interface failed");
			return ERROR_FAIL;
		}

		/* RX EP is common for all versions */
		h->rx_ep = STLINK_RX_EP;

		uint16_t pid;
		if (jtag_libusb_get_pid(libusb_get_device(h->usb_backend_priv.fd), &pid) != ERROR_OK) {
			LOG_DEBUG("libusb_get_pid failed");
			return ERROR_FAIL;
		}

		/* wrap version for first read */
		switch (pid) {
			case STLINK_V1_PID:
				h->version.stlink = 1;
				h->tx_ep = STLINK_TX_EP;
				break;
			case STLINK_V3_USBLOADER_PID:
			case STLINK_V3E_PID:
			case STLINK_V3S_PID:
			case STLINK_V3_2VCP_PID:
				h->version.stlink = 3;
				h->tx_ep = STLINK_V2_1_TX_EP;
				h->trace_ep = STLINK_V2_1_TRACE_EP;
				break;
			case STLINK_V2_1_PID:
			case STLINK_V2_1_NO_MSD_PID:
				h->version.stlink = 2;
				h->tx_ep = STLINK_V2_1_TX_EP;
				h->trace_ep = STLINK_V2_1_TRACE_EP;
				break;
			default:
			/* fall through - we assume V2 to be the default version*/
			case STLINK_V2_PID:
				h->version.stlink = 2;
				h->tx_ep = STLINK_TX_EP;
				h->trace_ep = STLINK_TRACE_EP;
				break;
		}

		/* get the device version */
		err = stlink_version(h);

		if (err == ERROR_OK) {
			break;
		} else if (h->version.stlink == 1 ||
			   retry_count == 0) {
			LOG_ERROR("read version failed");
			return ERROR_FAIL;
		} else {
			err = libusb_release_interface(h->usb_backend_priv.fd, 0);
			if (err != ERROR_OK) {
				LOG_ERROR("release interface failed");
				return ERROR_FAIL;
			}

			err = libusb_reset_device(h->usb_backend_priv.fd);
			if (err != ERROR_OK) {
				LOG_ERROR("reset device failed");
				return ERROR_FAIL;
			}

			jtag_libusb_close(h->usb_backend_priv.fd);
			/*
			  Give the device one second to settle down and
			  reenumerate.
			 */
			usleep(1 * 1000 * 1000);
			retry_count--;
		}
	} while (1);

	return ERROR_OK;
}

/** */
static int stlink_usb_close(void *handle)
{
	struct stlink_handle_s *h = handle;

	if (!h)
		return ERROR_OK;

	if (h->usb_backend_priv.fd) {
		stlink_exit_mode(h);
		/* do not check return code, it prevent
		us from closing jtag_libusb */
		jtag_libusb_close(h->usb_backend_priv.fd);
	}

	free(h->cmdbuf);
	free(h->databuf);

	return ERROR_OK;
}

struct stlink_backend_s stlink_usb_backend = {
	.open = stlink_usb_open,
	.close = stlink_usb_close,
	.xfer_noerrcheck = stlink_usb_xfer_noerrcheck,
	.read_trace = stlink_usb_read_trace,
};
