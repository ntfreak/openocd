/***************************************************************************
 *   Copyright (C) 2017 by Texas Instruments, Inc.                         *
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
#include <libusb.h>

/* XDS110 USB serial number length */
#define XDS110_SERIAL_LEN 8

/* XDS110 stand-alone probe voltage supply limits */
#define XDS110_MIN_VOLTAGE 1800
#define XDS110_MAX_VOLTAGE 3600

/* XDS110 stand-alone probe hardware ID */
#define XDS110_STAND_ALONE_ID 0x21

/* Firmware version that introduced OpenOCD support via block accesses */
#define OCD_FIRMWARE_VERSION 0x02030011
#define OCD_FIRMWARE_UPGRADE \
	"XDS110: upgrade to version 2.3.0.11+ for improved support"

/* Firmware version that introduced improved TCK performance */
#define FAST_TCK_FIRMWARE_VERSION 0x03000000

/* Firmware version that introduced 10 MHz and 12 MHz TCK support */
#define FAST_TCK_PLUS_FIRMWARE_VERSION 0x03000003

/***************************************************************************
 *   USB Connection Buffer Definitions                                     *
 ***************************************************************************/

/* Max USB packet size for up to USB 3.0 */
#define MAX_PACKET 1024

/*
 * Maximum data payload that can be handled in a single call
 * Limitation is the size of the buffers in the XDS110 firmware
 */
#define MAX_DATA_BLOCK 4096

#ifndef USB_PAYLOAD_SIZE
/* Largest data block plus parameters */
#define USB_PAYLOAD_SIZE (MAX_DATA_BLOCK + 60)
#endif
#define MAX_RESULT_QUEUE (MAX_DATA_BLOCK / 4)

/***************************************************************************
 *   XDS110 Firmware API Definitions                                       *
 ***************************************************************************/

/*
 * Default values controlling how the host communicates commands
 * with XDS110 firmware (automatic retry count and wait timeout)
 */
#define DEFAULT_ATTEMPTS (1)
#define DEFAULT_TIMEOUT  (4000)

/* XDS110 API error codes */
#define SC_ERR_NONE             0
#define SC_ERR_XDS110_FAIL   -261
#define SC_ERR_SWD_WAIT      -613
#define SC_ERR_SWD_FAULT     -614
#define SC_ERR_SWD_PROTOCOL  -615
#define SC_ERR_SWD_PARITY    -616
#define SC_ERR_SWD_DEVICE_ID -617

/* TCK frequency limits */
#define XDS110_MIN_TCK_SPEED  100 /* kHz */
#define XDS110_MAX_SLOW_TCK_SPEED 2500 /* kHz */
#define XDS110_MAX_FAST_TCK_SPEED 14000 /* kHz */
#define XDS110_DEFAULT_TCK_SPEED 2500 /* kHz */

/* Fixed TCK delay values for "Fast" TCK frequencies */
#define FAST_TCK_DELAY_14000_KHZ 0
#define FAST_TCK_DELAY_10000_KHZ 0xfffffffd
#define FAST_TCK_DELAY_12000_KHZ 0xfffffffe
#define FAST_TCK_DELAY_8500_KHZ 1
#define FAST_TCK_DELAY_5500_KHZ 2
/* For TCK frequencies below 5500 kHz, use calculated delay */

/* Scan mode on connect */
#define MODE_JTAG 1

/* XDS110 API JTAG state definitions */
#define XDS_JTAG_STATE_RESET       1
#define XDS_JTAG_STATE_IDLE        2
#define XDS_JTAG_STATE_SHIFT_DR    3
#define XDS_JTAG_STATE_SHIFT_IR    4
#define XDS_JTAG_STATE_PAUSE_DR    5
#define XDS_JTAG_STATE_PAUSE_IR    6
#define XDS_JTAG_STATE_EXIT1_DR    8
#define XDS_JTAG_STATE_EXIT1_IR    9
#define XDS_JTAG_STATE_EXIT2_DR   10
#define XDS_JTAG_STATE_EXIT2_IR   11
#define XDS_JTAG_STATE_SELECT_DR  12
#define XDS_JTAG_STATE_SELECT_IR  13
#define XDS_JTAG_STATE_UPDATE_DR  14
#define XDS_JTAG_STATE_UPDATE_IR  15
#define XDS_JTAG_STATE_CAPTURE_DR 16
#define XDS_JTAG_STATE_CAPTURE_IR 17

/* XDS110 API JTAG transit definitions */
#define XDS_JTAG_TRANSIT_QUICKEST    1
#define XDS_JTAG_TRANSIT_VIA_CAPTURE 2
#define XDS_JTAG_TRANSIT_VIA_IDLE    3

/* DAP register definitions as used by XDS110 APIs */

#define DAP_AP 0 /* DAP AP register type */
#define DAP_DP 1 /* DAP DP register type */

#define DAP_DP_IDCODE 0x0 /* DAP DP IDCODE register (read only) */
#define DAP_DP_ABORT  0x0 /* DAP DP ABORT register (write only) */
#define DAP_DP_STAT   0x4 /* DAP DP STAT register (for read only) */
#define DAP_DP_CTRL   0x4 /* DAP DP CTRL register (for write only) */
#define DAP_DP_ADDR   0x8 /* DAP DP SELECT register (legacy name) */
#define DAP_DP_RESEND 0x8 /* DAP DP RESEND register (read only) */
#define DAP_DP_SELECT 0x8 /* DAP DP SELECT register (write only) */
#define DAP_DP_RDBUFF 0xc /* DAP DP RDBUFF Read Buffer register */

#define DAP_AP_CSW  0x00 /* DAP AP Control Status Word */
#define DAP_AP_TAR  0x04 /* DAP AP Transfer Address */
#define DAP_AP_DRW  0x0C /* DAP AP Data Read/Write */
#define DAP_AP_BD0  0x10 /* DAP AP Banked Data 0 */
#define DAP_AP_BD1  0x14 /* DAP AP Banked Data 1 */
#define DAP_AP_BD2  0x18 /* DAP AP Banked Data 2 */
#define DAP_AP_BD3  0x1C /* DAP AP Banked Data 3 */
#define DAP_AP_RTBL 0xF8 /* DAP AP Debug ROM Table */
#define DAP_AP_IDR  0xFC /* DAP AP Identification Register */

/* Command packet definitions */

#define XDS_OUT_LEN 1 /* command (byte) */
#define XDS_IN_LEN  4 /* error code (int) */

/* XDS API Commands */
#define XDS_CONNECT      0x01 /* Connect JTAG connection */
#define XDS_DISCONNECT   0x02 /* Disconnect JTAG connection */
#define XDS_VERSION      0x03 /* Get firmware version and hardware ID */
#define XDS_SET_TCK      0x04 /* Set TCK delay (to set TCK frequency) */
#define XDS_SET_TRST     0x05 /* Assert or deassert nTRST signal */
#define XDS_CYCLE_TCK    0x07 /* Toggle TCK for a number of cycles */
#define XDS_GOTO_STATE   0x09 /* Go to requested JTAG state */
#define XDS_JTAG_SCAN    0x0c /* Send and receive JTAG scan */
#define XDS_SET_SRST     0x0e /* Assert or deassert nSRST signal */
#define CMAPI_CONNECT    0x0f /* CMAPI connect */
#define CMAPI_DISCONNECT 0x10 /* CMAPI disconnect */
#define CMAPI_ACQUIRE    0x11 /* CMAPI acquire */
#define CMAPI_RELEASE    0x12 /* CMAPI release */
#define CMAPI_REG_READ   0x15 /* CMAPI DAP register read */
#define CMAPI_REG_WRITE  0x16 /* CMAPI DAP register write */
#define SWD_CONNECT      0x17 /* Switch from JTAG to SWD connection */
#define SWD_DISCONNECT   0x18 /* Switch from SWD to JTAG connection */
#define CJTAG_CONNECT    0x2b /* Switch from JTAG to cJTAG connection */
#define CJTAG_DISCONNECT 0x2c /* Switch from cJTAG to JTAG connection */
#define XDS_SET_SUPPLY   0x32 /* Set up stand-alone probe upply voltage */
#define OCD_DAP_REQUEST  0x3a /* Handle block of DAP requests */
#define OCD_SCAN_REQUEST 0x3b /* Handle block of JTAG scan requests */
#define OCD_PATHMOVE     0x3c /* Handle PATHMOVE to navigate JTAG states */

#define CMD_IR_SCAN      1
#define CMD_DR_SCAN      2
#define CMD_RUNTEST      3
#define CMD_STABLECLOCKS 4

/* Array to convert from OpenOCD tap_state_t to XDS JTAG state */
static const uint32_t xds_jtag_state[] = {
	XDS_JTAG_STATE_EXIT2_DR,   /* TAP_DREXIT2   = 0x0 */
	XDS_JTAG_STATE_EXIT1_DR,   /* TAP_DREXIT1   = 0x1 */
	XDS_JTAG_STATE_SHIFT_DR,   /* TAP_DRSHIFT   = 0x2 */
	XDS_JTAG_STATE_PAUSE_DR,   /* TAP_DRPAUSE   = 0x3 */
	XDS_JTAG_STATE_SELECT_IR,  /* TAP_IRSELECT  = 0x4 */
	XDS_JTAG_STATE_UPDATE_DR,  /* TAP_DRUPDATE  = 0x5 */
	XDS_JTAG_STATE_CAPTURE_DR, /* TAP_DRCAPTURE = 0x6 */
	XDS_JTAG_STATE_SELECT_DR,  /* TAP_DRSELECT  = 0x7 */
	XDS_JTAG_STATE_EXIT2_IR,   /* TAP_IREXIT2   = 0x8 */
	XDS_JTAG_STATE_EXIT1_IR,   /* TAP_IREXIT1   = 0x9 */
	XDS_JTAG_STATE_SHIFT_IR,   /* TAP_IRSHIFT   = 0xa */
	XDS_JTAG_STATE_PAUSE_IR,   /* TAP_IRPAUSE   = 0xb */
	XDS_JTAG_STATE_IDLE,       /* TAP_IDLE      = 0xc */
	XDS_JTAG_STATE_UPDATE_IR,  /* TAP_IRUPDATE  = 0xd */
	XDS_JTAG_STATE_CAPTURE_IR, /* TAP_IRCAPTURE = 0xe */
	XDS_JTAG_STATE_RESET,      /* TAP_RESET     = 0xf */
};

struct scan_result {
	bool first;
	uint8_t *buffer;
	uint32_t num_bits;
};

struct xds110_info {
	/* USB connection handles and data buffers */
	libusb_context *ctx;
	libusb_device_handle *dev;
	unsigned char read_payload[USB_PAYLOAD_SIZE];
	unsigned char write_packet[3];
	unsigned char write_payload[USB_PAYLOAD_SIZE];
	/* Device vid/pid */
	uint16_t vid;
	uint16_t pid;
	/* Debug interface */
	uint8_t interface;
	uint8_t endpoint_in;
	uint8_t endpoint_out;
	/* Status flags */
	bool is_connected;
	bool is_cmapi_connected;
	bool is_cmapi_acquired;
	bool is_swd_mode;
	bool is_ap_dirty;
	/* DAP register caches */
	uint32_t select;
	uint32_t rdbuff;
	bool use_rdbuff;
	/* TCK speed and delay count*/
	uint32_t speed;
	uint32_t delay_count;
	/* XDS110 serial number */
	char serial[XDS110_SERIAL_LEN + 1];
	/* XDS110 voltage supply setting */
	uint32_t voltage;
	/* XDS110 firmware and hardware version */
	uint32_t firmware;
	uint16_t hardware;
	/* Transaction queues */
	unsigned char txn_requests[MAX_DATA_BLOCK];
	uint32_t *txn_dap_results[MAX_DATA_BLOCK / 4];
	struct scan_result txn_scan_results[MAX_DATA_BLOCK / 4];
	uint32_t txn_request_size;
	uint32_t txn_result_size;
	uint32_t txn_result_count;
};

static struct xds110_info xds110 = {
	.ctx = NULL,
	.dev = NULL,
	.vid = 0,
	.pid = 0,
	.interface = 0,
	.endpoint_in = 0,
	.endpoint_out = 0,
	.is_connected = false,
	.is_cmapi_connected = false,
	.is_cmapi_acquired = false,
	.is_swd_mode = false,
	.is_ap_dirty = false,
	.speed = XDS110_DEFAULT_TCK_SPEED,
	.delay_count = 0,
	.serial = {0},
	.voltage = 0,
	.firmware = 0,
	.hardware = 0,
	.txn_request_size = 0,
	.txn_result_size = 0,
	.txn_result_count = 0
};

static inline void xds110_set_u32(uint8_t *buffer, uint32_t value)
{
	buffer[3] = (value >> 24) & 0xff;
	buffer[2] = (value >> 16) & 0xff;
	buffer[1] = (value >> 8) & 0xff;
	buffer[0] = (value >> 0) & 0xff;
}

static inline void xds110_set_u16(uint8_t *buffer, uint16_t value)
{
	buffer[1] = (value >> 8) & 0xff;
	buffer[0] = (value >> 0) & 0xff;
}

static inline uint32_t xds110_get_u32(uint8_t *buffer)
{
	uint32_t value = (((uint32_t)buffer[3]) << 24) |
					 (((uint32_t)buffer[2]) << 16) |
					 (((uint32_t)buffer[1]) << 8)  |
					 (((uint32_t)buffer[0]) << 0);
	return value;
}

static inline uint16_t xds110_get_u16(uint8_t *buffer)
{
	uint16_t value = (((uint32_t)buffer[1]) << 8) |
					 (((uint32_t)buffer[0]) << 0);
	return value;
}

/***************************************************************************
 *   usb connection routines                                               *
 *                                                                         *
 *   The following functions handle connecting, reading, and writing to    *
 *   the XDS110 over USB using the libusb library.                         *
 ***************************************************************************/

static bool usb_connect(void)
{
	libusb_context *ctx  = NULL;
	libusb_device **list = NULL;
	libusb_device_handle *dev  = NULL;

	struct libusb_device_descriptor desc;

	/* The vid/pids of possible XDS110 configurations */
	uint16_t vids[] = { 0x0451, 0x0451, 0x1cbe };
	uint16_t pids[] = { 0xbef3, 0xbef4, 0x02a5 };
	/* Corresponding interface and endpoint numbers for configurations */
	uint8_t interfaces[] = { 2, 2, 0 };
	uint8_t endpoints_in[] = { 3, 3, 1 };
	uint8_t endpoints_out[] = { 2, 2, 1 };

	ssize_t count = 0;
	ssize_t i = 0;
	int result = 0;
	bool found = false;
	uint32_t device = 0;
	bool match = false;

	/* Initialize libusb context */
	result = libusb_init(&ctx);

	if (0 == result) {
		/* Get list of USB devices attached to system */
		count = libusb_get_device_list(ctx, &list);
		if (count <= 0) {
			result = -1;
			list = NULL;
		}
	}

	if (0 == result) {
		/* Scan through list of devices for any XDS110s */
		for (i = 0; i < count; i++) {
			/* Check for device vid/pid match */
			libusb_get_device_descriptor(list[i], &desc);
			match = false;
			for (device = 0; device < sizeof(vids)/sizeof(vids[0]); device++) {
				if (desc.idVendor == vids[device] &&
					desc.idProduct == pids[device]) {
					match = true;
					break;
				}
			}
			if (match) {
				result = libusb_open(list[i], &dev);
				if (0 == result) {
					const int max_data = 256;
					unsigned char data[max_data + 1];
					*data = '\0';

					/* May be the requested device if serial number matches */
					if (0 == xds110.serial[0]) {
						/* No serial number given; match first XDS110 found */
						found = true;
						break;
					} else {
						/* Get the device's serial number string */
						result = libusb_get_string_descriptor_ascii(dev,
									desc.iSerialNumber, data, max_data);
						if (0 < result &&
							0 == strcmp((char *)data, (char *)xds110.serial)) {
							found = true;
							break;
						}
					}

					/* If we fall though to here, we don't want this device */
					libusb_close(dev);
					dev = NULL;
				}
			}
		}
	}

	/*
	 * We can fall through the for() loop with two possible exit conditions:
	 * 1) found the right XDS110, and that device is open
	 * 2) didn't find the XDS110, and no devices are currently open
	 */

	if (NULL != list) {
		/* Free the device list, we're done with it */
		libusb_free_device_list(list, 1);
	}

	if (found) {
		/* Save the vid/pid of the device we're using */
		xds110.vid = vids[device];
		xds110.pid = pids[device];

		/* Save the debug interface and endpoints for the device */
		xds110.interface = interfaces[device];
		xds110.endpoint_in = endpoints_in[device] | LIBUSB_ENDPOINT_IN;
		xds110.endpoint_out = endpoints_out[device] | LIBUSB_ENDPOINT_OUT;

		/* Save the context and device handles */
		xds110.ctx = ctx;
		xds110.dev = dev;

		/* Set libusb to auto detach kernel */
		(void)libusb_set_auto_detach_kernel_driver(dev, 1);

		/* Claim the debug interface on the XDS110 */
		result = libusb_claim_interface(dev, xds110.interface);
	} else {
		/* Couldn't find an XDS110, flag the error */
		result = -1;
	}

	/* On an error, clean up what we can */
	if (0 != result) {
		if (NULL != dev) {
			/* Release the debug and data interface on the XDS110 */
			(void)libusb_release_interface(dev, xds110.interface);
			libusb_close(dev);
		}
		if (NULL != ctx)
			libusb_exit(ctx);
		xds110.ctx = NULL;
		xds110.dev = NULL;
	}

	/* Log the results */
	if (0 == result)
		LOG_INFO("XDS110: connected");
	else
		LOG_ERROR("XDS110: failed to connect");

	return (0 == result) ? true : false;
}

static void usb_disconnect(void)
{
	if (NULL != xds110.dev) {
		/* Release the debug and data interface on the XDS110 */
		(void)libusb_release_interface(xds110.dev, xds110.interface);
		libusb_close(xds110.dev);
		xds110.dev = NULL;
	}
	if (NULL != xds110.ctx) {
		libusb_exit(xds110.ctx);
		xds110.ctx = NULL;
	}

	LOG_INFO("XDS110: disconnected");
}

static bool usb_read(unsigned char *buffer, int size, int *bytes_read,
	int timeout)
{
	int result;

	if (NULL == xds110.dev || NULL == buffer || NULL == bytes_read)
		return false;

	/* Force a non-zero timeout to prevent blocking */
	if (0 == timeout)
		timeout = DEFAULT_TIMEOUT;

	result = libusb_bulk_transfer(xds110.dev, xds110.endpoint_in, buffer, size,
				bytes_read, timeout);

	return (0 == result) ? true : false;
}

static bool usb_write(unsigned char *buffer, int size, int *written)
{
	int bytes_written = 0;
	int result = LIBUSB_SUCCESS;
	int retries = 0;

	if (NULL == xds110.dev || NULL == buffer)
		return false;

	result = libusb_bulk_transfer(xds110.dev, xds110.endpoint_out, buffer,
				size, &bytes_written, 0);

	while (LIBUSB_ERROR_PIPE == result && retries < 3) {
		/* Try clearing the pipe stall and retry transfer */
		libusb_clear_halt(xds110.dev, xds110.endpoint_out);
		result = libusb_bulk_transfer(xds110.dev, xds110.endpoint_out, buffer,
					size, &bytes_written, 0);
		retries++;
	}

	if (NULL != written)
		*written = bytes_written;

	return (0 == result && size == bytes_written) ? true : false;
}

static bool usb_get_response(uint32_t *total_bytes_read, uint32_t timeout)
{
	static unsigned char buffer[MAX_PACKET];
	int bytes_read;
	uint16_t size;
	uint16_t count;
	bool success;

	size = 0;
	success = true;
	while (success) {
		success = usb_read(buffer, sizeof(buffer), &bytes_read, timeout);
		if (success) {
			/*
			 * Validate that this appears to be a good response packet
			 * First check it contains enough data for header and error
			 * code, plus the first character is the start character
			 */
			if (bytes_read >= 7 && '*' == buffer[0]) {
				/* Extract the payload size */
				size = xds110_get_u16(&buffer[1]);
				/* Sanity test on payload size */
				if (USB_PAYLOAD_SIZE >= size && 4 <= size) {
					/* Check we didn't get more data than expected */
					if ((bytes_read - 3) <= size) {
						/* Packet appears to be valid, move on */
						break;
					}
				}
			}
		}
		/*
		 * Somehow received an invalid packet, retry till we
		 * time out or a valid response packet is received
		 */
	}

	/* Abort now if we didn't receive a valid response */
	if (!success) {
		if (NULL != total_bytes_read)
			*total_bytes_read = 0;
		return false;
	}

	/* Build the return payload into xds110.read_payload */

	/* Copy over payload data from received buffer (skipping header) */
	count = 0;
	bytes_read -= 3;
	memcpy((void *)&xds110.read_payload[count], (void *)&buffer[3], bytes_read);
	count += bytes_read;
	/*
	 * Drop timeout to just 1/2 second. Once the XDS110 starts sending
	 * a response, the remaining packets should arrive in short order
	 */
	if (timeout > 500)
		timeout = 500; /* ms */

	/* If there's more data to retrieve, get it now */
	while ((count < size) && success) {
		success = usb_read(buffer, sizeof(buffer), &bytes_read, timeout);
		if (success) {
			if ((count + bytes_read) > size) {
				/* Read too much data, not a valid packet, abort */
				success = false;
			} else {
				/* Copy this data over to xds110.read_payload */
				memcpy((void *)&xds110.read_payload[count], (void *)buffer,
					bytes_read);
				count += bytes_read;
			}
		}
	}

	if (!success)
		count = 0;
	if (NULL != total_bytes_read)
		*total_bytes_read = count;

	return success;
}

static bool usb_send_command(uint16_t size)
{
	int written;
	bool success = true;

	/* Check the packet length */
	if (size > USB_PAYLOAD_SIZE)
		return false;

	/* Place the start character into the packet buffer */
	xds110.write_packet[0] = '*';

	/* Place the payload size into the packet buffer */
	xds110_set_u16(&xds110.write_packet[1], size);

	/* Adjust size to include header */
	size += 3;

	/* Send the data via the USB connection */
	success = usb_write(xds110.write_packet, (int)size, &written);

	/* Check if the correct number of bytes was written */
	if (written != (int)size)
		success = false;

	return success;
}

/***************************************************************************
 *   XDS110 firmware API routines                                          *
 *                                                                         *
 *   The following functions handle calling into the XDS110 firmware to    *
 *   perform requested debug actions.                                      *
 ***************************************************************************/

static bool xds_execute(uint32_t out_length, uint32_t in_length,
	uint32_t attempts, uint32_t timeout)
{
	bool done = false;
	bool success = true;
	int error = 0;
	uint32_t bytes_read = 0;

	if (NULL == xds110.dev)
		return false;

	while (!done && attempts > 0) {
		attempts--;

		/* Send command to XDS110 */
		success = usb_send_command(out_length);

		if (success) {
			/* Get response from XDS110 */
			success = usb_get_response(&bytes_read, timeout);
		}

		if (success) {
			/* Check for valid response from XDS code handling */
			if (bytes_read != in_length) {
				/* Unexpected amount of data returned */
				success = false;
				LOG_DEBUG("XDS110: command 0x%02x return %" PRIu32 " bytes, expected %" PRIu32,
					xds110.write_payload[0], bytes_read, in_length);
			} else {
				/* Extract error code from return packet */
				error = (int)xds110_get_u32(&xds110.read_payload[0]);
				done = true;
				if (SC_ERR_NONE != error)
					LOG_DEBUG("XDS110: command 0x%02x returned error %d",
						xds110.write_payload[0], error);
			}
		}
	}

	if (!success)
		error = SC_ERR_XDS110_FAIL;

	if (0 != error)
		success = false;

	return success;
}

static bool xds_connect(void)
{
	bool success;

	xds110.write_payload[0] = XDS_CONNECT;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool xds_disconnect(void)
{
	bool success;

	xds110.write_payload[0] = XDS_DISCONNECT;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool xds_version(uint32_t *firmware_id, uint16_t *hardware_id)
{
	uint8_t *fw_id_pntr = &xds110.read_payload[XDS_IN_LEN + 0]; /* 32-bits */
	uint8_t *hw_id_pntr = &xds110.read_payload[XDS_IN_LEN + 4]; /* 16-bits */

	bool success;

	xds110.write_payload[0] = XDS_VERSION;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN + 6, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	if (success) {
		if (NULL != firmware_id)
			*firmware_id = xds110_get_u32(fw_id_pntr);
		if (NULL != hardware_id)
			*hardware_id = xds110_get_u16(hw_id_pntr);
	}

	return success;
}

static bool xds_set_tck_delay(uint32_t delay)
{
	uint8_t *delay_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 32-bits */

	bool success;

	xds110.write_payload[0] = XDS_SET_TCK;

	xds110_set_u32(delay_pntr, delay);

	success = xds_execute(XDS_OUT_LEN + 4, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool xds_set_trst(uint8_t trst)
{
	uint8_t *trst_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 8-bits */

	bool success;

	xds110.write_payload[0] = XDS_SET_TRST;

	*trst_pntr = trst;

	success = xds_execute(XDS_OUT_LEN + 1, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool xds_cycle_tck(uint32_t count)
{
	uint8_t *count_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 32-bits */

	bool success;

	xds110.write_payload[0] = XDS_CYCLE_TCK;

	xds110_set_u32(count_pntr, count);

	success = xds_execute(XDS_OUT_LEN + 4, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool xds_goto_state(uint32_t state)
{
	uint8_t *state_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 32-bits */
	uint8_t *transit_pntr = &xds110.write_payload[XDS_OUT_LEN+4]; /* 32-bits */

	bool success;

	xds110.write_payload[0] = XDS_GOTO_STATE;

	xds110_set_u32(state_pntr, state);
	xds110_set_u32(transit_pntr, XDS_JTAG_TRANSIT_QUICKEST);

	success = xds_execute(XDS_OUT_LEN+8, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool xds_jtag_scan(uint32_t shift_state, uint16_t shift_bits,
	uint32_t end_state, uint8_t *data_out, uint8_t *data_in)
{
	uint8_t *bits_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 16-bits */
	uint8_t *path_pntr = &xds110.write_payload[XDS_OUT_LEN + 2]; /* 8-bits */
	uint8_t *trans1_pntr = &xds110.write_payload[XDS_OUT_LEN + 3]; /* 8-bits */
	uint8_t *end_pntr = &xds110.write_payload[XDS_OUT_LEN + 4]; /* 8-bits */
	uint8_t *trans2_pntr = &xds110.write_payload[XDS_OUT_LEN + 5]; /* 8-bits */
	uint8_t *pre_pntr = &xds110.write_payload[XDS_OUT_LEN + 6]; /* 16-bits */
	uint8_t *pos_pntr = &xds110.write_payload[XDS_OUT_LEN + 8]; /* 16-bits */
	uint8_t *delay_pntr = &xds110.write_payload[XDS_OUT_LEN + 10]; /* 16-bits */
	uint8_t *rep_pntr = &xds110.write_payload[XDS_OUT_LEN + 12]; /* 16-bits */
	uint8_t *out_pntr = &xds110.write_payload[XDS_OUT_LEN + 14]; /* 16-bits */
	uint8_t *in_pntr = &xds110.write_payload[XDS_OUT_LEN + 16]; /* 16-bits */
	uint8_t *data_out_pntr = &xds110.write_payload[XDS_OUT_LEN + 18];
	uint8_t *data_in_pntr = &xds110.read_payload[XDS_IN_LEN+0];

	uint16_t total_bytes = DIV_ROUND_UP(shift_bits, 8);

	bool success;

	xds110.write_payload[0] = XDS_JTAG_SCAN;

	xds110_set_u16(bits_pntr, shift_bits); /* bits to scan */
	*path_pntr = (uint8_t)(shift_state & 0xff); /* IR vs DR path */
	*trans1_pntr = (uint8_t)XDS_JTAG_TRANSIT_QUICKEST; /* start state route */
	*end_pntr = (uint8_t)(end_state & 0xff); /* JTAG state after scan */
	*trans2_pntr = (uint8_t)XDS_JTAG_TRANSIT_QUICKEST; /* end state route */
	xds110_set_u16(pre_pntr, 0); /* number of preamble bits */
	xds110_set_u16(pos_pntr, 0); /* number of postamble bits */
	xds110_set_u16(delay_pntr, 0); /* number of extra TCKs after scan */
	xds110_set_u16(rep_pntr, 1); /* number of repetitions */
	xds110_set_u16(out_pntr, total_bytes); /* out buffer offset (if repeats) */
	xds110_set_u16(in_pntr, total_bytes); /* in buffer offset (if repeats) */

	memcpy((void *)data_out_pntr, (void *)data_out, total_bytes);

	success = xds_execute(XDS_OUT_LEN + 18 + total_bytes,
		XDS_IN_LEN + total_bytes, DEFAULT_ATTEMPTS, DEFAULT_TIMEOUT);

	if (success)
		memcpy((void *)data_in, (void *)data_in_pntr, total_bytes);

	return success;
}

static bool xds_set_srst(uint8_t srst)
{
	uint8_t *srst_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 8-bits */

	bool success;

	xds110.write_payload[0] = XDS_SET_SRST;

	*srst_pntr = srst;

	success = xds_execute(XDS_OUT_LEN + 1, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool cmapi_connect(uint32_t *idcode)
{
	uint8_t *idcode_pntr = &xds110.read_payload[XDS_IN_LEN + 0]; /* 32-bits */

	bool success;

	xds110.write_payload[0] = CMAPI_CONNECT;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN+4, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	if (success) {
		if (NULL != idcode)
			*idcode = xds110_get_u32(idcode_pntr);
	}

	return success;
}

static bool cmapi_disconnect(void)
{
	bool success;

	xds110.write_payload[0] = CMAPI_DISCONNECT;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool cmapi_acquire(void)
{
	bool success;

	xds110.write_payload[0] = CMAPI_ACQUIRE;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool cmapi_release(void)
{
	bool success;

	xds110.write_payload[0] = CMAPI_RELEASE;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool cmapi_read_dap_reg(uint32_t type, uint32_t ap_num,
	uint32_t address, uint32_t *value)
{
	uint8_t *type_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 8-bits */
	uint8_t *ap_num_pntr = &xds110.write_payload[XDS_OUT_LEN + 1]; /* 8-bits */
	uint8_t *address_pntr = &xds110.write_payload[XDS_OUT_LEN + 2]; /* 8-bits */
	uint8_t *value_pntr = &xds110.read_payload[XDS_IN_LEN + 0]; /* 32-bits */

	bool success;

	xds110.write_payload[0] = CMAPI_REG_READ;

	*type_pntr = (uint8_t)(type & 0xff);
	*ap_num_pntr = (uint8_t)(ap_num & 0xff);
	*address_pntr = (uint8_t)(address & 0xff);

	success = xds_execute(XDS_OUT_LEN + 3, XDS_IN_LEN + 4, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	if (success) {
		if (NULL != value)
			*value = xds110_get_u32(value_pntr);
	}

	return success;
}

static bool cmapi_write_dap_reg(uint32_t type, uint32_t ap_num,
	uint32_t address, uint32_t *value)
{
	uint8_t *type_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 8-bits */
	uint8_t *ap_num_pntr = &xds110.write_payload[XDS_OUT_LEN + 1]; /* 8-bits */
	uint8_t *address_pntr = &xds110.write_payload[XDS_OUT_LEN + 2]; /* 8-bits */
	uint8_t *value_pntr = &xds110.write_payload[XDS_OUT_LEN + 3]; /* 32-bits */

	bool success;

	if (NULL == value)
		return false;

	xds110.write_payload[0] = CMAPI_REG_WRITE;

	*type_pntr = (uint8_t)(type & 0xff);
	*ap_num_pntr = (uint8_t)(ap_num & 0xff);
	*address_pntr = (uint8_t)(address & 0xff);
	xds110_set_u32(value_pntr, *value);

	success = xds_execute(XDS_OUT_LEN + 7, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool swd_connect(void)
{
	bool success;

	xds110.write_payload[0] = SWD_CONNECT;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool swd_disconnect(void)
{
	bool success;

	xds110.write_payload[0] = SWD_DISCONNECT;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool cjtag_connect(uint32_t format)
{
	uint8_t *format_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 32-bits */

	bool success;

	xds110.write_payload[0] = CJTAG_CONNECT;

	xds110_set_u32(format_pntr, format);

	success = xds_execute(XDS_OUT_LEN + 4, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool cjtag_disconnect(void)
{
	bool success;

	xds110.write_payload[0] = CJTAG_DISCONNECT;

	success = xds_execute(XDS_OUT_LEN, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool xds_set_supply(uint32_t voltage)
{
	uint8_t *volts_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 32-bits */
	uint8_t *source_pntr = &xds110.write_payload[XDS_OUT_LEN + 4]; /* 8-bits */

	bool success;

	xds110.write_payload[0] = XDS_SET_SUPPLY;

	xds110_set_u32(volts_pntr, voltage);
	*source_pntr = (uint8_t)(0 != voltage ? 1 : 0);

	success = xds_execute(XDS_OUT_LEN + 5, XDS_IN_LEN, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	return success;
}

static bool ocd_dap_request(uint8_t *dap_requests, uint32_t request_size,
	uint32_t *dap_results, uint32_t result_count)
{
	uint8_t *request_pntr = &xds110.write_payload[XDS_OUT_LEN + 0];
	uint8_t *result_pntr = &xds110.read_payload[XDS_IN_LEN + 0];

	bool success;

	if (NULL == dap_requests || NULL == dap_results)
		return false;

	xds110.write_payload[0] = OCD_DAP_REQUEST;

	memcpy((void *)request_pntr, (void *)dap_requests, request_size);

	success = xds_execute(XDS_OUT_LEN + request_size,
				XDS_IN_LEN + (result_count * 4), DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	if (success && (result_count > 0))
		memcpy((void *)dap_results, (void *)result_pntr, result_count * 4);

	return success;
}

static bool ocd_scan_request(uint8_t *scan_requests, uint32_t request_size,
	uint8_t *scan_results, uint32_t result_size)
{
	uint8_t *request_pntr = &xds110.write_payload[XDS_OUT_LEN + 0];
	uint8_t *result_pntr = &xds110.read_payload[XDS_IN_LEN + 0];

	bool success;

	if (NULL == scan_requests || NULL == scan_results)
		return false;

	xds110.write_payload[0] = OCD_SCAN_REQUEST;

	memcpy((void *)request_pntr, (void *)scan_requests, request_size);

	success = xds_execute(XDS_OUT_LEN + request_size,
				XDS_IN_LEN + result_size, DEFAULT_ATTEMPTS,
				DEFAULT_TIMEOUT);

	if (success && (result_size > 0))
		memcpy((void *)scan_results, (void *)result_pntr, result_size);

	return success;
}

static bool ocd_pathmove(uint32_t num_states, uint8_t *path)
{
	uint8_t *num_pntr = &xds110.write_payload[XDS_OUT_LEN + 0]; /* 32-bits */
	uint8_t *path_pntr = &xds110.write_payload[XDS_OUT_LEN + 4];

	bool success;

	if (NULL == path)
		return false;

	xds110.write_payload[0] = OCD_PATHMOVE;

	xds110_set_u32(num_pntr, num_states);

	memcpy((void *)path_pntr, (void *)path, num_states);

	success = xds_execute(XDS_OUT_LEN + 4 + num_states, XDS_IN_LEN,
				DEFAULT_ATTEMPTS, DEFAULT_TIMEOUT);

	return success;
}

/***************************************************************************
 *   swd driver interface                                                  *
 *                                                                         *
 *   The following functions provide SWD support to OpenOCD.               *
 ***************************************************************************/

static int xds110_swd_init(void)
{
	xds110.is_swd_mode = true;
	return ERROR_OK;
}

static int xds110_swd_switch_seq(enum swd_special_seq seq)
{
	uint32_t idcode;
	bool success;

	switch (seq) {
	case LINE_RESET:
		LOG_ERROR("Sequence SWD line reset (%d) not supported", seq);
		return ERROR_FAIL;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		xds110.is_swd_mode = false;
		xds110.is_cmapi_connected = false;
		xds110.is_cmapi_acquired = false;
		/* Run sequence to put target in SWD mode */
		success = swd_connect();
		/* Re-initialize CMAPI API for DAP access */
		if (success) {
			xds110.is_swd_mode = true;
			success = cmapi_connect(&idcode);
			if (success) {
				xds110.is_cmapi_connected = true;
				success = cmapi_acquire();
			}
		}
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		xds110.is_swd_mode = false;
		xds110.is_cmapi_connected = false;
		xds110.is_cmapi_acquired = false;
		/* Run sequence to put target in JTAG mode */
		success = swd_disconnect();
		if (success) {
			/* Re-initialize JTAG interface */
			success = cjtag_connect(MODE_JTAG);
		}
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	if (success)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static bool xds110_legacy_read_reg(uint8_t cmd, uint32_t *value)
{
	/* Make sure this is a read request */
	bool is_read_request = (0 != (SWD_CMD_RnW & cmd));
	/* Determine whether this is a DP or AP register access */
	uint32_t type = (0 != (SWD_CMD_APnDP & cmd)) ? DAP_AP : DAP_DP;
	/* Determine the AP number from cached SELECT value */
	uint32_t ap_num = (xds110.select & 0xff000000) >> 24;
	/* Extract register address from command */
	uint32_t address = ((cmd & SWD_CMD_A32) >> 1);
	/* Extract bank address from cached SELECT value */
	uint32_t bank = (xds110.select & 0x000000f0);

	uint32_t reg_value = 0;
	uint32_t temp_value = 0;

	bool success;

	if (!is_read_request)
		return false;

	if (DAP_AP == type) {
		/* Add bank address to register address for CMAPI call */
		address |= bank;
	}

	if (DAP_DP == type && DAP_DP_RDBUFF == address && xds110.use_rdbuff) {
		/* If RDBUFF is cached and this is a DP RDBUFF read, use the cache */
		reg_value = xds110.rdbuff;
		success = true;
	} else if (DAP_AP == type && DAP_AP_DRW == address && xds110.use_rdbuff) {
		/* If RDBUFF is cached and this is an AP DRW read, use the cache, */
		/* but still call into the firmware to get the next read. */
		reg_value = xds110.rdbuff;
		success = cmapi_read_dap_reg(type, ap_num, address, &temp_value);
	} else {
		success = cmapi_read_dap_reg(type, ap_num, address, &temp_value);
		if (success)
			reg_value = temp_value;
	}

	/* Mark that we have consumed or invalidated the RDBUFF cache */
	xds110.use_rdbuff = false;

	/* Handle result of read attempt */
	if (!success)
		LOG_ERROR("XDS110: failed to read DAP register");
	else if (NULL != value)
		*value = reg_value;

	if (success && DAP_AP == type) {
		/*
		 * On a successful DAP AP read, we actually have the value from RDBUFF,
		 * the firmware will have run the AP request and made the RDBUFF read
		 */
		xds110.use_rdbuff = true;
		xds110.rdbuff = temp_value;
	}

	return success;
}

static bool xds110_legacy_write_reg(uint8_t cmd, uint32_t value)
{
	/* Make sure this isn't a read request */
	bool is_read_request = (0 != (SWD_CMD_RnW & cmd));
	/* Determine whether this is a DP or AP register access */
	uint32_t type = (0 != (SWD_CMD_APnDP & cmd)) ? DAP_AP : DAP_DP;
	/* Determine the AP number from cached SELECT value */
	uint32_t ap_num = (xds110.select & 0xff000000) >> 24;
	/* Extract register address from command */
	uint32_t address = ((cmd & SWD_CMD_A32) >> 1);
	/* Extract bank address from cached SELECT value */
	uint32_t bank = (xds110.select & 0x000000f0);

	bool success;

	if (is_read_request)
		return false;

	/* Invalidate the RDBUFF cache */
	xds110.use_rdbuff = false;

	if (DAP_AP == type) {
		/* Add bank address to register address for CMAPI call */
		address |= bank;
		/* Any write to an AP register invalidates the firmware's cache */
		xds110.is_ap_dirty = true;
	} else if (DAP_DP_SELECT == address) {
		/* Any write to the SELECT register invalidates the firmware's cache */
		xds110.is_ap_dirty = true;
	}

	success = cmapi_write_dap_reg(type, ap_num, address, &value);

	if (!success) {
		LOG_ERROR("XDS110: failed to write DAP register");
	} else {
		/*
		 * If the debugger wrote to SELECT, cache the value
		 * to use to build the apNum and address values above
		 */
		if ((DAP_DP == type) && (DAP_DP_SELECT == address))
			xds110.select = value;
	}

	return success;
}

static int xds110_swd_run_queue(void)
{
	static uint32_t dap_results[MAX_RESULT_QUEUE];
	uint8_t cmd;
	uint32_t request;
	uint32_t result;
	uint32_t value;
	bool success = true;

	if (0 == xds110.txn_request_size)
		return ERROR_OK;

	/* Terminate request queue */
	xds110.txn_requests[xds110.txn_request_size++] = 0;

	if (xds110.firmware >= OCD_FIRMWARE_VERSION) {
		/* XDS110 firmware has the API to directly handle the queue */
		success = ocd_dap_request(xds110.txn_requests,
			xds110.txn_request_size, dap_results, xds110.txn_result_count);
	} else {
		/* Legacy firmware needs to handle queue via discrete DAP calls */
		request = 0;
		result = 0;
		while (xds110.txn_requests[request] != 0) {
			cmd = xds110.txn_requests[request++];
			if (0 == (SWD_CMD_RnW & cmd)) {
				/* DAP register write command */
				value  = (uint32_t)(xds110.txn_requests[request++]) <<  0;
				value |= (uint32_t)(xds110.txn_requests[request++]) <<  8;
				value |= (uint32_t)(xds110.txn_requests[request++]) << 16;
				value |= (uint32_t)(xds110.txn_requests[request++]) << 24;
				if (success)
					success = xds110_legacy_write_reg(cmd, value);
			} else {
				/* DAP register read command */
				value = 0;
				if (success)
					success = xds110_legacy_read_reg(cmd, &value);
				dap_results[result++] = value;
			}
		}
	}

	/* Transfer results into caller's buffers */
	for (result = 0; result < xds110.txn_result_count; result++)
		if (0 != xds110.txn_dap_results[result])
			*xds110.txn_dap_results[result] = dap_results[result];

	xds110.txn_request_size = 0;
	xds110.txn_result_size = 0;
	xds110.txn_result_count = 0;

	return (success) ? ERROR_OK : ERROR_FAIL;
}

static void xds110_swd_queue_cmd(uint8_t cmd, uint32_t *value)
{
	/* Check if this is a read or write request */
	bool is_read_request = (0 != (SWD_CMD_RnW & cmd));
	/* Determine whether this is a DP or AP register access */
	uint32_t type = (0 != (SWD_CMD_APnDP & cmd)) ? DAP_AP : DAP_DP;
	/* Extract register address from command */
	uint32_t address = ((cmd & SWD_CMD_A32) >> 1);
	uint32_t request_size = (is_read_request) ? 1 : 5;

	/* Check if new request would be too large to fit */
	if (((xds110.txn_request_size + request_size + 1) > MAX_DATA_BLOCK) ||
		((xds110.txn_result_count + 1) > MAX_RESULT_QUEUE))
		xds110_swd_run_queue();

	/* Set the START bit in cmd to ensure cmd is not zero */
	/* (a value of zero is used to terminate the buffer) */
	cmd |= SWD_CMD_START;

	/* Add request to queue; queue is built marshalled for XDS110 call */
	if (is_read_request) {
		/* Queue read request, save pointer to pass back result */
		xds110.txn_requests[xds110.txn_request_size++] = cmd;
		xds110.txn_dap_results[xds110.txn_result_count++] = value;
		xds110.txn_result_size += 4;
	} else {
		/* Check for and prevent sticky overrun detection */
		if (DAP_DP == type && DAP_DP_CTRL == address &&
			(*value & CORUNDETECT)) {
			LOG_DEBUG("XDS110: refusing to enable sticky overrun detection");
			*value &= ~CORUNDETECT;
		}
		/* Queue write request, add value directly to queue buffer */
		xds110.txn_requests[xds110.txn_request_size++] = cmd;
		xds110.txn_requests[xds110.txn_request_size++] = (*value >>  0) & 0xff;
		xds110.txn_requests[xds110.txn_request_size++] = (*value >>  8) & 0xff;
		xds110.txn_requests[xds110.txn_request_size++] = (*value >> 16) & 0xff;
		xds110.txn_requests[xds110.txn_request_size++] = (*value >> 24) & 0xff;
	}
}

static void xds110_swd_read_reg(uint8_t cmd, uint32_t *value,
	uint32_t ap_delay_clk)
{
	xds110_swd_queue_cmd(cmd, value);
}
static void xds110_swd_write_reg(uint8_t cmd, uint32_t value,
	uint32_t ap_delay_clk)
{
	xds110_swd_queue_cmd(cmd, &value);
}

/***************************************************************************
 *   jtag interface                                                        *
 *                                                                         *
 *   The following functions provide XDS110 interface to OpenOCD.          *
 ***************************************************************************/

static void xds110_show_info(void)
{
	uint32_t firmware = xds110.firmware;

	LOG_INFO("XDS110: vid/pid = %04x/%04x", xds110.vid, xds110.pid);
	LOG_INFO("XDS110: firmware version = %" PRIu32 ".%" PRIu32 ".%" PRIu32 ".%" PRIu32,
		(((firmware >> 28) & 0xf) * 10) + ((firmware >> 24) & 0xf),
		(((firmware >> 20) & 0xf) * 10) + ((firmware >> 16) & 0xf),
		(((firmware >> 12) & 0xf) * 10) + ((firmware >>  8) & 0xf),
		(((firmware >>  4) & 0xf) * 10) + ((firmware >>  0) & 0xf));
	LOG_INFO("XDS110: hardware version = 0x%04x", xds110.hardware);
	if (0 != xds110.serial[0])
		LOG_INFO("XDS110: serial number = %s", xds110.serial);
	if (xds110.is_swd_mode) {
		LOG_INFO("XDS110: connected to target via SWD");
		LOG_INFO("XDS110: SWCLK set to %" PRIu32 " kHz", xds110.speed);
	} else {
		LOG_INFO("XDS110: connected to target via JTAG");
		LOG_INFO("XDS110: TCK set to %" PRIu32 " kHz", xds110.speed);
	}

	/* Alert user that there's a better firmware to use */
	if (firmware < OCD_FIRMWARE_VERSION) {
		LOG_WARNING("XDS110: the firmware is not optimized for OpenOCD");
		LOG_WARNING(OCD_FIRMWARE_UPGRADE);
	}
}

static int xds110_quit(void)
{
	if (xds110.is_cmapi_acquired) {
		(void)cmapi_release();
		xds110.is_cmapi_acquired = false;
	}
	if (xds110.is_cmapi_connected) {
		(void)cmapi_disconnect();
		xds110.is_cmapi_connected = false;
	}
	if (xds110.is_connected) {
		if (xds110.is_swd_mode) {
			/* Switch out of SWD mode */
			(void)swd_disconnect();
		} else {
			/* Switch out of cJTAG mode */
			(void)cjtag_disconnect();
		}
		/* Tell firmware we're disconnecting */
		(void)xds_disconnect();
		xds110.is_connected = false;
	}
	/* Close down the USB connection to the XDS110 debug probe */
	usb_disconnect();

	return ERROR_OK;
}

static int xds110_init(void)
{
	bool success;

	/* Establish USB connection to the XDS110 debug probe */
	success = usb_connect();

	if (success) {
		/* Send connect message to XDS110 firmware */
		success = xds_connect();
		if (success)
			xds110.is_connected = true;
	}

	if (success) {
		uint32_t firmware;
		uint16_t hardware;

		/* Retrieve version IDs from firmware */
		/* Version numbers are stored in BCD format */
		success = xds_version(&firmware, &hardware);
		if (success) {
			/* Save the firmware and hardware version */
			xds110.firmware = firmware;
			xds110.hardware = hardware;
		}
	}

	if (success) {
		/* Set supply voltage for stand-alone probes */
		if (XDS110_STAND_ALONE_ID == xds110.hardware) {
			success = xds_set_supply(xds110.voltage);
			/* Allow time for target device to power up */
			/* (CC32xx takes up to 1300 ms before debug is enabled) */
			alive_sleep(1500);
		} else if (0 != xds110.voltage) {
			/* Voltage supply not a feature of embedded probes */
			LOG_WARNING(
				"XDS110: ignoring supply voltage, not supported on this probe");
		}
	}

	if (success) {
		success = xds_set_trst(0);
		if (success)
			success = xds_cycle_tck(50);
		if (success)
			success = xds_set_trst(1);
		if (success)
			success = xds_cycle_tck(50);
	}

	if (success) {
		if (xds110.is_swd_mode) {
			/* Switch to SWD if needed */
			success = swd_connect();
		} else {
			success = cjtag_connect(MODE_JTAG);
		}
	}

	if (success && xds110.is_swd_mode) {
		uint32_t idcode;

		/* Connect to CMAPI interface in XDS110 */
		success = cmapi_connect(&idcode);

		/* Acquire exclusive access to CMAPI interface */
		if (success) {
			xds110.is_cmapi_connected = true;
			success = cmapi_acquire();
			if (success)
				xds110.is_cmapi_acquired = true;
		}
	}

	if (!success)
		xds110_quit();

	if (success)
		xds110_show_info();

	return (success) ? ERROR_OK : ERROR_FAIL;
}

static void xds110_legacy_scan(uint32_t shift_state, uint32_t total_bits,
	uint32_t end_state, uint8_t *data_out, uint8_t *data_in)
{
	(void)xds_jtag_scan(shift_state, total_bits, end_state, data_out, data_in);
}

static void xds110_legacy_runtest(uint32_t clocks, uint32_t end_state)
{
	xds_goto_state(XDS_JTAG_STATE_IDLE);
	xds_cycle_tck(clocks);
	xds_goto_state(end_state);
}

static void xds110_legacy_stableclocks(uint32_t clocks)
{
	xds_cycle_tck(clocks);
}

static void xds110_flush(void)
{
	uint8_t command;
	uint32_t clocks;
	uint32_t shift_state;
	uint32_t end_state;
	uint32_t bits;
	uint32_t bytes;
	uint32_t request;
	uint32_t result;
	uint8_t *data_out;
	uint8_t data_in[MAX_DATA_BLOCK];
	uint8_t *data_pntr;

	if (0 == xds110.txn_request_size)
		return;

	/* Terminate request queue */
	xds110.txn_requests[xds110.txn_request_size++] = 0;

	if (xds110.firmware >= OCD_FIRMWARE_VERSION) {
		/* Updated firmware has the API to directly handle the queue */
		(void)ocd_scan_request(xds110.txn_requests, xds110.txn_request_size,
			data_in, xds110.txn_result_size);
	} else {
		/* Legacy firmware needs to handle queue via discrete JTAG calls */
		request = 0;
		result = 0;
		while (xds110.txn_requests[request] != 0) {
			command = xds110.txn_requests[request++];
			switch (command) {
				case CMD_IR_SCAN:
				case CMD_DR_SCAN:
					if (command == CMD_IR_SCAN)
						shift_state = XDS_JTAG_STATE_SHIFT_IR;
					else
						shift_state = XDS_JTAG_STATE_SHIFT_DR;
					end_state = (uint32_t)(xds110.txn_requests[request++]);
					bits  = (uint32_t)(xds110.txn_requests[request++]) << 0;
					bits |= (uint32_t)(xds110.txn_requests[request++]) << 8;
					data_out = &xds110.txn_requests[request];
					bytes = DIV_ROUND_UP(bits, 8);
					xds110_legacy_scan(shift_state, bits, end_state, data_out,
						&data_in[result]);
					result += bytes;
					request += bytes;
					break;
				case CMD_RUNTEST:
					clocks  = (uint32_t)(xds110.txn_requests[request++]) <<  0;
					clocks |= (uint32_t)(xds110.txn_requests[request++]) <<  8;
					clocks |= (uint32_t)(xds110.txn_requests[request++]) << 16;
					clocks |= (uint32_t)(xds110.txn_requests[request++]) << 24;
					end_state = (uint32_t)xds110.txn_requests[request++];
					xds110_legacy_runtest(clocks, end_state);
					break;
				case CMD_STABLECLOCKS:
					clocks  = (uint32_t)(xds110.txn_requests[request++]) <<  0;
					clocks |= (uint32_t)(xds110.txn_requests[request++]) <<  8;
					clocks |= (uint32_t)(xds110.txn_requests[request++]) << 16;
					clocks |= (uint32_t)(xds110.txn_requests[request++]) << 24;
					xds110_legacy_stableclocks(clocks);
					break;
				default:
					LOG_ERROR("BUG: unknown JTAG command type 0x%x encountered",
						command);
					exit(-1);
					break;
			}
		}
	}

	/* Transfer results into caller's buffers from data_in buffer */
	bits = 0; /* Bit offset into current scan result */
	data_pntr = data_in;
	for (result = 0; result < xds110.txn_result_count; result++) {
		if (xds110.txn_scan_results[result].first) {
			if (bits != 0) {
				bytes = DIV_ROUND_UP(bits, 8);
				data_pntr += bytes;
			}
			bits = 0;
		}
		if (xds110.txn_scan_results[result].buffer != 0)
			bit_copy(xds110.txn_scan_results[result].buffer, 0, data_pntr,
				bits, xds110.txn_scan_results[result].num_bits);
		bits += xds110.txn_scan_results[result].num_bits;
	}

	xds110.txn_request_size = 0;
	xds110.txn_result_size = 0;
	xds110.txn_result_count = 0;
}

static int xds110_reset(int trst, int srst)
{
	uint8_t value;
	bool success;
	int retval = ERROR_OK;

	if (trst != -1) {
		if (trst == 0) {
			/* Deassert nTRST (active low) */
			value = 1;
		} else {
			/* Assert nTRST (active low) */
			value = 0;
		}
		success = xds_set_trst(value);
		if (!success)
			retval = ERROR_FAIL;
	}

	if (srst != -1) {
		if (srst == 0) {
			/* Deassert nSRST (active low) */
			value = 1;
		} else {
			/* Assert nSRST (active low) */
			value = 0;
		}
		success = xds_set_srst(value);
		if (!success)
			retval = ERROR_FAIL;

		/* Toggle TCK to trigger HIB on CC13x/CC26x devices */
		if (success && !xds110.is_swd_mode) {
			/* Toggle TCK for about 50 ms */
			success = xds_cycle_tck(xds110.speed * 50);
		}

		if (!success)
			retval = ERROR_FAIL;
	}

	return retval;
}

static void xds110_execute_sleep(struct jtag_command *cmd)
{
	jtag_sleep(cmd->cmd.sleep->us);
}

static void xds110_execute_tlr_reset(struct jtag_command *cmd)
{
	(void)xds_goto_state(XDS_JTAG_STATE_RESET);
}

static void xds110_execute_pathmove(struct jtag_command *cmd)
{
	uint32_t i;
	uint32_t num_states;
	uint8_t *path;

	num_states = (uint32_t)cmd->cmd.pathmove->num_states;

	if (num_states == 0)
		return;

	path = (uint8_t *)malloc(num_states * sizeof(uint8_t));
	if (path == 0) {
		LOG_ERROR("XDS110: unable to allocate memory");
		return;
	}

	/* Convert requested path states into XDS API states */
	for (i = 0; i < num_states; i++)
		path[i] = (uint8_t)xds_jtag_state[cmd->cmd.pathmove->path[i]];

	if (xds110.firmware >= OCD_FIRMWARE_VERSION) {
		/* Updated firmware fully supports pathmove */
		(void)ocd_pathmove(num_states, path);
	} else {
		/* Notify user that legacy firmware simply cannot handle pathmove */
		LOG_ERROR("XDS110: the firmware does not support pathmove command");
		LOG_ERROR(OCD_FIRMWARE_UPGRADE);
		/* If pathmove is required, then debug is not possible */
		exit(-1);
	}

	free((void *)path);
}

static void xds110_queue_scan(struct jtag_command *cmd)
{
	int i;
	uint32_t offset;
	uint32_t total_fields;
	uint32_t total_bits;
	uint32_t total_bytes;
	uint8_t end_state;
	uint8_t *buffer;

	/* Calculate the total number of bits to scan */
	total_bits = 0;
	total_fields = 0;
	for (i = 0; i < cmd->cmd.scan->num_fields; i++) {
		total_fields++;
		total_bits += (uint32_t)cmd->cmd.scan->fields[i].num_bits;
	}

	if (total_bits == 0)
		return;

	total_bytes = DIV_ROUND_UP(total_bits, 8);

	/* Check if new request would be too large to fit */
	if (((xds110.txn_request_size + 1 + total_bytes + sizeof(end_state) + 1)
		> MAX_DATA_BLOCK) || ((xds110.txn_result_count + total_fields) >
		MAX_RESULT_QUEUE))
		xds110_flush();

	/* Check if this single request is too large to fit */
	if ((1 + total_bytes + sizeof(end_state) + 1) > MAX_DATA_BLOCK) {
		LOG_ERROR("BUG: JTAG scan request is too large to handle (%" PRIu32 " bits)",
			total_bits);
		/* Failing to run this scan mucks up debug on this target */
		exit(-1);
	}

	if (cmd->cmd.scan->ir_scan)
		xds110.txn_requests[xds110.txn_request_size++] = CMD_IR_SCAN;
	else
		xds110.txn_requests[xds110.txn_request_size++] = CMD_DR_SCAN;

	end_state = (uint8_t)xds_jtag_state[cmd->cmd.scan->end_state];
	xds110.txn_requests[xds110.txn_request_size++] = end_state;

	xds110.txn_requests[xds110.txn_request_size++] = (total_bits >> 0) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = (total_bits >> 8) & 0xff;

	/* Build request data by flattening fields into single buffer */
	/* also populate the results array to return the results when run */
	offset = 0;
	buffer = &xds110.txn_requests[xds110.txn_request_size];
	/* Clear data out buffer to default value of all zeros */
	memset((void *)buffer, 0x00, total_bytes);
	for (i = 0; i < cmd->cmd.scan->num_fields; i++) {
		if (cmd->cmd.scan->fields[i].out_value != 0) {
			/* Copy over data to scan out into request buffer */
			bit_copy(buffer, offset, cmd->cmd.scan->fields[i].out_value, 0,
				cmd->cmd.scan->fields[i].num_bits);
		}
		offset += cmd->cmd.scan->fields[i].num_bits;
		xds110.txn_scan_results[xds110.txn_result_count].first = (i == 0);
		xds110.txn_scan_results[xds110.txn_result_count].num_bits =
			cmd->cmd.scan->fields[i].num_bits;
		xds110.txn_scan_results[xds110.txn_result_count++].buffer =
			cmd->cmd.scan->fields[i].in_value;
	}
	xds110.txn_request_size += total_bytes;
	xds110.txn_result_size += total_bytes;
}

static void xds110_queue_runtest(struct jtag_command *cmd)
{
	uint32_t clocks = (uint32_t)cmd->cmd.stableclocks->num_cycles;
	uint8_t end_state = (uint8_t)xds_jtag_state[cmd->cmd.runtest->end_state];

	/* Check if new request would be too large to fit */
	if ((xds110.txn_request_size + 1 + sizeof(clocks) + sizeof(end_state) + 1)
		> MAX_DATA_BLOCK)
		xds110_flush();

	/* Queue request and cycle count directly to queue buffer */
	xds110.txn_requests[xds110.txn_request_size++] = CMD_RUNTEST;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >>  0) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >>  8) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >> 16) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >> 24) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = end_state;
}

static void xds110_queue_stableclocks(struct jtag_command *cmd)
{
	uint32_t clocks = (uint32_t)cmd->cmd.stableclocks->num_cycles;

	/* Check if new request would be too large to fit */
	if ((xds110.txn_request_size + 1 + sizeof(clocks) + 1) > MAX_DATA_BLOCK)
		xds110_flush();

	/* Queue request and cycle count directly to queue buffer */
	xds110.txn_requests[xds110.txn_request_size++] = CMD_STABLECLOCKS;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >>  0) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >>  8) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >> 16) & 0xff;
	xds110.txn_requests[xds110.txn_request_size++] = (clocks >> 24) & 0xff;
}

static void xds110_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_SLEEP:
			xds110_flush();
			xds110_execute_sleep(cmd);
			break;
		case JTAG_TLR_RESET:
			xds110_flush();
			xds110_execute_tlr_reset(cmd);
			break;
		case JTAG_PATHMOVE:
			xds110_flush();
			xds110_execute_pathmove(cmd);
			break;
		case JTAG_SCAN:
			xds110_queue_scan(cmd);
			break;
		case JTAG_RUNTEST:
			xds110_queue_runtest(cmd);
			break;
		case JTAG_STABLECLOCKS:
			xds110_queue_stableclocks(cmd);
			break;
		case JTAG_TMS:
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%x encountered",
				cmd->type);
			exit(-1);
	}
}

static int xds110_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		xds110_execute_command(cmd);
		cmd = cmd->next;
	}

	xds110_flush();

	return ERROR_OK;
}

static int xds110_speed(int speed)
{
	double freq_to_use;
	uint32_t delay_count;
	bool success;

	if (speed == 0) {
		LOG_INFO("XDS110: RTCK not supported");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	if (speed < XDS110_MIN_TCK_SPEED) {
		LOG_INFO("XDS110: increase speed request: %d kHz to %d kHz minimum",
			speed, XDS110_MIN_TCK_SPEED);
		speed = XDS110_MIN_TCK_SPEED;
	}

	/* Older XDS110 firmware had inefficient scan routines and could only */
	/* achieve a peak TCK frequency of about 2500 kHz */
	if (xds110.firmware < FAST_TCK_FIRMWARE_VERSION) {

		/* Check for request for top speed or higher */
		if (speed >= XDS110_MAX_SLOW_TCK_SPEED) {

			/* Inform user that speed was adjusted down to max possible */
			if (speed > XDS110_MAX_SLOW_TCK_SPEED) {
				LOG_INFO(
					"XDS110: reduce speed request: %d kHz to %d kHz maximum",
					speed, XDS110_MAX_SLOW_TCK_SPEED);
				speed = XDS110_MAX_SLOW_TCK_SPEED;
			}
			delay_count = 0;

		} else {

			const double XDS110_TCK_PULSE_INCREMENT = 66.0;
			freq_to_use = speed * 1000; /* Hz */
			delay_count = 0;

			/* Calculate the delay count value */
			double one_giga = 1000000000;
			/* Get the pulse duration for the max frequency supported in ns */
			double max_freq_pulse_duration = one_giga /
				(XDS110_MAX_SLOW_TCK_SPEED * 1000);

			/* Convert frequency to pulse duration */
			double freq_to_pulse_width_in_ns = one_giga / freq_to_use;

			/*
			* Start with the pulse duration for the maximum frequency. Keep
			* decrementing time added by each count value till the requested
			* frequency pulse is less than the calculated value.
			*/
			double current_value = max_freq_pulse_duration;

			while (current_value < freq_to_pulse_width_in_ns) {
				current_value += XDS110_TCK_PULSE_INCREMENT;
				++delay_count;
			}

			/*
			* Determine which delay count yields the best match.
			* The one obtained above or one less.
			*/
			if (delay_count) {
				double diff_freq_1 = freq_to_use -
					(one_giga / (max_freq_pulse_duration +
					(XDS110_TCK_PULSE_INCREMENT * delay_count)));
				double diff_freq_2 = (one_giga / (max_freq_pulse_duration +
					(XDS110_TCK_PULSE_INCREMENT * (delay_count - 1)))) -
					freq_to_use;

				/* One less count value yields a better match */
				if (diff_freq_1 > diff_freq_2)
					--delay_count;
			}
		}

	/* Newer firmware has reworked TCK routines that are much more efficient */
	/* and can now achieve a peak TCK frequency of 14000 kHz */
	} else {

		if (speed >= XDS110_MAX_FAST_TCK_SPEED) {
			if (speed > XDS110_MAX_FAST_TCK_SPEED) {
				LOG_INFO(
					"XDS110: reduce speed request: %d kHz to %d kHz maximum",
					speed, XDS110_MAX_FAST_TCK_SPEED);
				speed = XDS110_MAX_FAST_TCK_SPEED;
			}
			delay_count = 0;
		} else if (speed >= 12000 && xds110.firmware >=
			FAST_TCK_PLUS_FIRMWARE_VERSION) {
			delay_count = FAST_TCK_DELAY_12000_KHZ;
		} else if (speed >= 10000 && xds110.firmware >=
			FAST_TCK_PLUS_FIRMWARE_VERSION) {
			delay_count = FAST_TCK_DELAY_10000_KHZ;
		} else if (speed >= 8500) {
			delay_count = FAST_TCK_DELAY_8500_KHZ;
		} else if (speed >= 5500) {
			delay_count = FAST_TCK_DELAY_5500_KHZ;
		} else {
			/* Calculate the delay count to set the frequency */
			/* Formula determined by measuring the waveform on Saeleae logic */
			/* analyzer using known values for delay count */
			const double m = 17100000.0; /* slope */
			const double b = -1.02;      /* y-intercept */

			freq_to_use = speed * 1000; /* Hz */
			double period = 1.0/freq_to_use;
			double delay = m * period + b;

			if (delay < 1.0)
				delay_count = 1;
			else
				delay_count = (uint32_t)delay;
		}
	}

	/* Send the delay count to the XDS110 firmware */
	success = xds_set_tck_delay(delay_count);

	if (success) {
		xds110.delay_count = delay_count;
		xds110.speed = speed;
	}

	return (success) ? ERROR_OK : ERROR_FAIL;
}

static int xds110_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int xds110_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

COMMAND_HANDLER(xds110_handle_info_command)
{
	xds110_show_info();
	return ERROR_OK;
}

COMMAND_HANDLER(xds110_handle_serial_command)
{
	wchar_t serial[XDS110_SERIAL_LEN + 1];

	xds110.serial[0] = 0;

	if (CMD_ARGC == 1) {
		size_t len = mbstowcs(0, CMD_ARGV[0], 0);
		if (len > XDS110_SERIAL_LEN) {
			LOG_ERROR("XDS110: serial number is limited to %d characters",
				XDS110_SERIAL_LEN);
			return ERROR_FAIL;
		}
		if ((size_t)-1 == mbstowcs(serial, CMD_ARGV[0], len + 1)) {
			LOG_ERROR("XDS110: unable to convert serial number");
			return ERROR_FAIL;
		}

		for (uint32_t i = 0; i < len; i++)
			xds110.serial[i] = (char)serial[i];

		xds110.serial[len] = 0;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(xds110_handle_supply_voltage_command)
{
	uint32_t voltage = 0;

	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], voltage);
		if (voltage == 0 || (voltage >= XDS110_MIN_VOLTAGE && voltage
			<= XDS110_MAX_VOLTAGE)) {
			/* Requested voltage is in range */
			xds110.voltage = voltage;
		} else {
			LOG_ERROR("XDS110: voltage must be 0 or between %d and %d "
				"millivolts", XDS110_MIN_VOLTAGE, XDS110_MAX_VOLTAGE);
			return ERROR_FAIL;
		}
		xds110.voltage = voltage;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static const struct command_registration xds110_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &xds110_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "show XDS110 info",
		.usage = "",
	},
	{
		.name = "serial",
		.handler = &xds110_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the XDS110 probe serial number",
		.usage = "serial_string",
	},
	{
		.name = "supply",
		.handler = &xds110_handle_supply_voltage_command,
		.mode = COMMAND_CONFIG,
		.help = "set the XDS110 probe supply voltage",
		.usage = "voltage_in_millivolts",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration xds110_command_handlers[] = {
	{
		.name = "xds110",
		.mode = COMMAND_ANY,
		.help = "perform XDS110 management",
		.usage = "",
		.chain = xds110_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver xds110_swd_driver = {
	.init = xds110_swd_init,
	.switch_seq = xds110_swd_switch_seq,
	.read_reg = xds110_swd_read_reg,
	.write_reg = xds110_swd_write_reg,
	.run = xds110_swd_run_queue,
};

static const char * const xds110_transport[] = { "swd", "jtag", NULL };

static struct jtag_interface xds110_interface = {
	.execute_queue = xds110_execute_queue,
};

struct adapter_driver xds110_adapter_driver = {
	.name = "xds110",
	.transports = xds110_transport,
	.commands = xds110_command_handlers,

	.init = xds110_init,
	.quit = xds110_quit,
	.reset = xds110_reset,
	.speed = xds110_speed,
	.khz = xds110_khz,
	.speed_div = xds110_speed_div,

	.jtag_ops = &xds110_interface,
	.swd_ops = &xds110_swd_driver,
};
