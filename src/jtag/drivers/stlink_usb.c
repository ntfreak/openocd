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

/* project specific includes */
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <jtag/swim.h>
#include <target/target.h>
#include <transport/transport.h>

#include <target/cortex_m.h>


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

#include "libusb_helper.h"

#ifdef HAVE_LIBUSB1
#define USE_LIBUSB_ASYNCIO
#endif

#define STLINK_SERIAL_LEN 24

#define ENDPOINT_IN  0x80
#define ENDPOINT_OUT 0x00

#define STLINK_WRITE_TIMEOUT 1000
#define STLINK_READ_TIMEOUT 1000

#define STLINK_RX_EP          (1|ENDPOINT_IN)
#define STLINK_TX_EP          (2|ENDPOINT_OUT)
#define STLINK_TRACE_EP       (3|ENDPOINT_IN)

#define STLINK_V2_1_TX_EP     (1|ENDPOINT_OUT)
#define STLINK_V2_1_TRACE_EP  (2|ENDPOINT_IN)

#define STLINK_SG_SIZE        (31)
#define STLINK_DATA_SIZE      (4096)
#define STLINK_CMD_SIZE_V2    (16)
#define STLINK_CMD_SIZE_V1    (10)

#define STLINK_V1_PID         (0x3744)
#define STLINK_V2_PID         (0x3748)
#define STLINK_V2_1_PID       (0x374B)
#define STLINK_V2_1_NO_MSD_PID  (0x3752)
#define STLINK_V3_USBLOADER_PID (0x374D)
#define STLINK_V3E_PID          (0x374E)
#define STLINK_V3S_PID          (0x374F)
#define STLINK_V3_2VCP_PID      (0x3753)

/*
 * ST-Link/V1, ST-Link/V2 and ST-Link/V2.1 are full-speed USB devices and
 * this limits the bulk packet size and the 8bit read/writes to max 64 bytes.
 * STLINK-V3 is a high speed USB 2.0 and the limit is 512 bytes from FW V3J6.
 */
#define STLINK_MAX_RW8		(64)
#define STLINKV3_MAX_RW8	(512)

/* "WAIT" responses will be retried (with exponential backoff) at
 * most this many times before failing to caller.
 */
#define MAX_WAIT_RETRIES 8

enum stlink_jtag_api_version {
	STLINK_JTAG_API_V1 = 1,
	STLINK_JTAG_API_V2,
	STLINK_JTAG_API_V3,
};

enum stlink_mode {
	STLINK_MODE_UNKNOWN = 0,
	STLINK_MODE_DFU,
	STLINK_MODE_MASS,
	STLINK_MODE_DEBUG_JTAG,
	STLINK_MODE_DEBUG_SWD,
	STLINK_MODE_DEBUG_SWIM
};

/** */
struct stlink_usb_version {
	/** */
	int stlink;
	/** */
	int jtag;
	/** */
	int swim;
	/** jtag api version supported */
	enum stlink_jtag_api_version jtag_api;
	/** one bit for each feature supported. See macros STLINK_F_* */
	uint32_t flags;
};

struct stlink_usb_priv_s {
	/** */
	struct libusb_device_handle *fd;
	/** */
	struct libusb_transfer *trans;
};

struct stlink_tcp_priv_s {
	/** */
	int fd;
	/** */
	bool connected;
	/** */
	uint32_t device_id;
	/** */
	uint32_t connect_id;
};

struct stlink_interface_s {
	/** */
	void *priv;
	/** */
	int (*open)(void *handle, struct hl_interface_param_s *param);
	/** */
	int (*close)(void *handle);
	/** */
	int (*xfer_noerrcheck)(void *handle, const uint8_t *buf, int size);
	/** */
	int (*bulk_read)(void *handle, const uint8_t *buf, int size);
};

/** */
struct stlink_usb_handle_s {
	/** */
	struct stlink_interface_s *itf;
	/** */
	uint8_t rx_ep;
	/** */
	uint8_t tx_ep;
	/** */
	uint8_t trace_ep;
	/** */
	uint8_t cmdbuf[STLINK_SG_SIZE];
	/** */
	uint8_t cmdidx;
	/** */
	uint8_t direction;
	/** */
	uint8_t databuf[STLINK_DATA_SIZE];
	/** */
	uint32_t max_mem_packet;
	/** */
	enum stlink_mode st_mode;
	/** */
	struct stlink_usb_version version;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** */
	struct {
		/** whether SWO tracing is enabled or not */
		bool enabled;
		/** trace module source clock */
		uint32_t source_hz;
	} trace;
	/** reconnect is needed next time we try to query the
	 * status */
	bool reconnect_pending;
};

/** */
static inline int stlink_usb_open(void *handle, struct hl_interface_param_s *param)
{
	struct stlink_usb_handle_s *h = handle;
	return h->itf->open(handle, param);
}

/** */
static inline int stlink_usb_close(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	return h->itf->close(handle);
}
/** */
static inline int stlink_usb_xfer_noerrcheck(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;
	return h->itf->xfer_noerrcheck(handle, buf, size);
}

/** */
static inline int stlink_usb_bulk_read(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;
	return h->itf->bulk_read(handle, buf, size);
}

#define STLINK_SWIM_ERR_OK             0x00
#define STLINK_SWIM_BUSY               0x01
#define STLINK_DEBUG_ERR_OK            0x80
#define STLINK_DEBUG_ERR_FAULT         0x81
#define STLINK_SWD_AP_WAIT             0x10
#define STLINK_SWD_AP_FAULT            0x11
#define STLINK_SWD_AP_ERROR            0x12
#define STLINK_SWD_AP_PARITY_ERROR     0x13
#define STLINK_JTAG_GET_IDCODE_ERROR   0x09
#define STLINK_JTAG_WRITE_ERROR        0x0c
#define STLINK_JTAG_WRITE_VERIF_ERROR  0x0d
#define STLINK_SWD_DP_WAIT             0x14
#define STLINK_SWD_DP_FAULT            0x15
#define STLINK_SWD_DP_ERROR            0x16
#define STLINK_SWD_DP_PARITY_ERROR     0x17

#define STLINK_SWD_AP_WDATA_ERROR      0x18
#define STLINK_SWD_AP_STICKY_ERROR     0x19
#define STLINK_SWD_AP_STICKYORUN_ERROR 0x1a

#define STLINK_BAD_AP_ERROR            0x1d

#define STLINK_CORE_RUNNING            0x80
#define STLINK_CORE_HALTED             0x81
#define STLINK_CORE_STAT_UNKNOWN       -1

#define STLINK_GET_VERSION             0xF1
#define STLINK_DEBUG_COMMAND           0xF2
#define STLINK_DFU_COMMAND             0xF3
#define STLINK_SWIM_COMMAND            0xF4
#define STLINK_GET_CURRENT_MODE        0xF5
#define STLINK_GET_TARGET_VOLTAGE      0xF7

#define STLINK_DEV_DFU_MODE            0x00
#define STLINK_DEV_MASS_MODE           0x01
#define STLINK_DEV_DEBUG_MODE          0x02
#define STLINK_DEV_SWIM_MODE           0x03
#define STLINK_DEV_BOOTLOADER_MODE     0x04
#define STLINK_DEV_UNKNOWN_MODE        -1

#define STLINK_DFU_EXIT                0x07

/*
	STLINK_SWIM_ENTER_SEQ
	1.3ms low then 750Hz then 1.5kHz

	STLINK_SWIM_GEN_RST
	STM8 DM pulls reset pin low 50us

	STLINK_SWIM_SPEED
	uint8_t (0=low|1=high)

	STLINK_SWIM_WRITEMEM
	uint16_t length
	uint32_t address

	STLINK_SWIM_RESET
	send synchronization seq (16us low, response 64 clocks low)
*/
#define STLINK_SWIM_ENTER                  0x00
#define STLINK_SWIM_EXIT                   0x01
#define STLINK_SWIM_READ_CAP               0x02
#define STLINK_SWIM_SPEED                  0x03
#define STLINK_SWIM_ENTER_SEQ              0x04
#define STLINK_SWIM_GEN_RST                0x05
#define STLINK_SWIM_RESET                  0x06
#define STLINK_SWIM_ASSERT_RESET           0x07
#define STLINK_SWIM_DEASSERT_RESET         0x08
#define STLINK_SWIM_READSTATUS             0x09
#define STLINK_SWIM_WRITEMEM               0x0a
#define STLINK_SWIM_READMEM                0x0b
#define STLINK_SWIM_READBUF                0x0c

#define STLINK_DEBUG_GETSTATUS             0x01
#define STLINK_DEBUG_FORCEDEBUG            0x02
#define STLINK_DEBUG_APIV1_RESETSYS        0x03
#define STLINK_DEBUG_APIV1_READALLREGS     0x04
#define STLINK_DEBUG_APIV1_READREG         0x05
#define STLINK_DEBUG_APIV1_WRITEREG        0x06
#define STLINK_DEBUG_READMEM_32BIT         0x07
#define STLINK_DEBUG_WRITEMEM_32BIT        0x08
#define STLINK_DEBUG_RUNCORE               0x09
#define STLINK_DEBUG_STEPCORE              0x0a
#define STLINK_DEBUG_APIV1_SETFP           0x0b
#define STLINK_DEBUG_READMEM_8BIT          0x0c
#define STLINK_DEBUG_WRITEMEM_8BIT         0x0d
#define STLINK_DEBUG_APIV1_CLEARFP         0x0e
#define STLINK_DEBUG_APIV1_WRITEDEBUGREG   0x0f
#define STLINK_DEBUG_APIV1_SETWATCHPOINT   0x10

#define STLINK_DEBUG_ENTER_JTAG_RESET      0x00
#define STLINK_DEBUG_ENTER_SWD_NO_RESET    0xa3
#define STLINK_DEBUG_ENTER_JTAG_NO_RESET   0xa4

#define STLINK_DEBUG_APIV1_ENTER           0x20
#define STLINK_DEBUG_EXIT                  0x21
#define STLINK_DEBUG_READCOREID            0x22

#define STLINK_DEBUG_APIV2_ENTER           0x30
#define STLINK_DEBUG_APIV2_READ_IDCODES    0x31
#define STLINK_DEBUG_APIV2_RESETSYS        0x32
#define STLINK_DEBUG_APIV2_READREG         0x33
#define STLINK_DEBUG_APIV2_WRITEREG        0x34
#define STLINK_DEBUG_APIV2_WRITEDEBUGREG   0x35
#define STLINK_DEBUG_APIV2_READDEBUGREG    0x36

#define STLINK_DEBUG_APIV2_READALLREGS     0x3A
#define STLINK_DEBUG_APIV2_GETLASTRWSTATUS 0x3B
#define STLINK_DEBUG_APIV2_DRIVE_NRST      0x3C

#define STLINK_DEBUG_APIV2_GETLASTRWSTATUS2 0x3E

#define STLINK_DEBUG_APIV2_START_TRACE_RX  0x40
#define STLINK_DEBUG_APIV2_STOP_TRACE_RX   0x41
#define STLINK_DEBUG_APIV2_GET_TRACE_NB    0x42
#define STLINK_DEBUG_APIV2_SWD_SET_FREQ    0x43
#define STLINK_DEBUG_APIV2_JTAG_SET_FREQ   0x44
#define STLINK_DEBUG_APIV2_READ_DAP_REG    0x45
#define STLINK_DEBUG_APIV2_WRITE_DAP_REG   0x46
#define STLINK_DEBUG_APIV2_READMEM_16BIT   0x47
#define STLINK_DEBUG_APIV2_WRITEMEM_16BIT  0x48

#define STLINK_DEBUG_APIV2_INIT_AP         0x4B
#define STLINK_DEBUG_APIV2_CLOSE_AP_DBG    0x4C

#define STLINK_APIV3_SET_COM_FREQ           0x61
#define STLINK_APIV3_GET_COM_FREQ           0x62

#define STLINK_APIV3_GET_VERSION_EX         0xFB

#define STLINK_DEBUG_APIV2_DRIVE_NRST_LOW   0x00
#define STLINK_DEBUG_APIV2_DRIVE_NRST_HIGH  0x01
#define STLINK_DEBUG_APIV2_DRIVE_NRST_PULSE 0x02

#define STLINK_DEBUG_PORT_ACCESS            0xffff

#define STLINK_TRACE_SIZE               4096
#define STLINK_TRACE_MAX_HZ             2000000
#define STLINK_V3_TRACE_MAX_HZ          24000000

#define STLINK_V3_MAX_FREQ_NB               10

#define REQUEST_SENSE        0x03
#define REQUEST_SENSE_LENGTH 18

/* STLINK TCP commands */
#define STLINK_TCP_CMD_REFRESH_DEVICE_LIST   0x00
#define STLINK_TCP_CMD_GET_NB_DEV            0x01
#define STLINK_TCP_CMD_GET_DEV_INFO          0x02
#define STLINK_TCP_CMD_OPEN_DEV              0x03
#define STLINK_TCP_CMD_CLOSE_DEV             0x04
#define STLINK_TCP_CMD_SEND_USB_CMD          0x05
#define STLINK_TCP_CMD_GET_SERVER_VERSION    0x06
#define STLINK_TCP_CMD_GET_NB_OF_DEV_CLIENTS 0x07

/* STLINK TCP constants */
#define OPENOCD_STLINK_TCP_API_VERSION       1
#define STLINK_TCP_REQUEST_WRITE             0
#define STLINK_TCP_REQUEST_READ              1
#define STLINK_TCP_REQUEST_READ_SWO          3
#define STLINK_TCP_SS_SIZE                   4
#define STLINK_TCP_USB_CMD_SIZE              32
#define STLINK_TCP_SERIAL_SIZE               32
#define STLINK_TCP_SEND_BUFFER_SIZE          10240
#define STLINK_TCP_RECV_BUFFER_SIZE          10240

/* STLINK TCP command status */
#define STLINK_TCP_SS_OK                     0x00000001
#define STLINK_TCP_SS_MEMORY_PROBLEM         0x00001000
#define STLINK_TCP_SS_TIMEOUT                0x00001001
#define STLINK_TCP_SS_BAD_PARAMETER          0x00001002
#define STLINK_TCP_SS_OPEN_ERR               0x00001003
#define STLINK_TCP_SS_TRUNCATED_DATA         0x00001052
#define STLINK_TCP_SS_CMD_NOT_AVAILABLE      0x00001053
#define STLINK_TCP_SS_TCP_ERROR              0x00002001
#define STLINK_TCP_SS_TCP_CANT_CONNECT       0x00002002
#define STLINK_TCP_SS_WIN32_ERROR            0x00010000

/*
 * Map the relevant features, quirks and workaround for specific firmware
 * version of stlink
 */
#define STLINK_F_HAS_TRACE              BIT(0)
#define STLINK_F_HAS_SWD_SET_FREQ       BIT(1)
#define STLINK_F_HAS_JTAG_SET_FREQ      BIT(2)
#define STLINK_F_HAS_MEM_16BIT          BIT(3)
#define STLINK_F_HAS_GETLASTRWSTATUS2   BIT(4)
#define STLINK_F_HAS_DAP_REG            BIT(5)
#define STLINK_F_QUIRK_JTAG_DP_READ     BIT(6)
#define STLINK_F_HAS_AP_INIT            BIT(7)
#define STLINK_F_HAS_DPBANKSEL          BIT(8)
#define STLINK_F_HAS_RW8_512BYTES       BIT(9)
#define STLINK_F_FIX_CLOSE_AP           BIT(10)

/* aliases */
#define STLINK_F_HAS_TARGET_VOLT        STLINK_F_HAS_TRACE
#define STLINK_F_HAS_FPU_REG            STLINK_F_HAS_GETLASTRWSTATUS2

#define STLINK_REGSEL_IS_FPU(x)         ((x) > 0x1F)

struct speed_map {
	int speed;
	int speed_divisor;
};

/* SWD clock speed */
static const struct speed_map stlink_khz_to_speed_map_swd[] = {
	{4000, 0},
	{1800, 1}, /* default */
	{1200, 2},
	{950,  3},
	{480,  7},
	{240, 15},
	{125, 31},
	{100, 40},
	{50,  79},
	{25, 158},
	{15, 265},
	{5,  798}
};

/* JTAG clock speed */
static const struct speed_map stlink_khz_to_speed_map_jtag[] = {
	{9000,  4},
	{4500,  8},
	{2250, 16},
	{1125, 32}, /* default */
	{562,  64},
	{281, 128},
	{140, 256}
};

static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size);
static int stlink_swim_status(void *handle);
static void stlink_dump_speed_map(const struct speed_map *map, unsigned int map_size);
static int stlink_get_com_freq(void *handle, bool is_jtag, struct speed_map *map);
static int stlink_speed(void *handle, int khz, bool query);
static int stlink_usb_open_ap(void *handle, unsigned short apsel);

/** */
static unsigned int stlink_usb_block(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.flags & STLINK_F_HAS_RW8_512BYTES)
		return STLINKV3_MAX_RW8;
	else
		return STLINK_MAX_RW8;
}



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

#endif


/** */
static int stlink_usb_xfer_v1_get_status(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_usb_priv_s *itf_priv = h->itf->priv;
	int tr, ret;

	assert(handle != NULL);

	/* read status */
	memset(h->cmdbuf, 0, STLINK_SG_SIZE);

	ret = jtag_libusb_bulk_read(itf_priv->fd, h->rx_ep, (char *)h->cmdbuf, 13,
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

#ifdef USE_LIBUSB_ASYNCIO
static int stlink_usb_xfer_rw(void *handle, int cmdsize, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_usb_priv_s *itf_priv = h->itf->priv;

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
			itf_priv->fd,
			transfers,
			n_transfers,
			STLINK_WRITE_TIMEOUT);
}
#else
static int stlink_usb_xfer_rw(void *handle, int cmdsize, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_usb_priv_s *itf_priv = h->itf->priv;
	int tr, ret;

	assert(handle != NULL);

	ret = jtag_libusb_bulk_write(itf_priv->fd, h->tx_ep, (char *)h->cmdbuf,
				     cmdsize, STLINK_WRITE_TIMEOUT, &tr);
	if (ret || tr != cmdsize)
		return ERROR_FAIL;

	if (h->direction == h->tx_ep && size) {
		ret = jtag_libusb_bulk_write(itf_priv->fd, h->tx_ep, (char *)buf,
					     size, STLINK_WRITE_TIMEOUT, &tr);
		if (ret || tr != size) {
			LOG_DEBUG("bulk write failed");
			return ERROR_FAIL;
		}
	} else if (h->direction == h->rx_ep && size) {
		ret = jtag_libusb_bulk_read(itf_priv->fd, h->rx_ep, (char *)buf,
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
static int stlink_usb_xfer_v1_get_sense(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 16);

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

/** */
static int stlink_usb_usb_bulk_read(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_usb_priv_s *itf_priv = h->itf->priv;
	int tr, ret;

	ret = jtag_libusb_bulk_read(itf_priv->fd, h->direction, (char *)buf, size,
				    STLINK_READ_TIMEOUT, &tr);
	if (ret || tr != size) {
		LOG_ERROR("bulk trace read failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/*
	transfers block in cmdbuf
	<size> indicates number of bytes in the following
	data phase.
	Ignore the (eventual) error code in the received packet.
*/
static int stlink_usb_usb_xfer_noerrcheck(void *handle, const uint8_t *buf, int size)
{
	int err, cmdsize = STLINK_CMD_SIZE_V2;
	struct stlink_usb_handle_s *h = handle;

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


static int stlink_tcp_send_cmd(void *handle, const uint8_t *cmd, int cmd_size,
		uint8_t *data, int data_size, bool check_tcp_status)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_tcp_priv_s *itf_priv = h->itf->priv;

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
static int stlink_tcp_xfer_noerrcheck(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_tcp_priv_s *itf_priv = h->itf->priv;

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

	int ret = stlink_tcp_send_cmd(h, cmdbuf, cmd_size, databuf, data_size, true);
	if (ret != ERROR_OK)
		return ret;

	if (h->direction != h->tx_ep)
		memcpy((uint8_t *) buf, &databuf[STLINK_TCP_SS_SIZE], size);

	return ERROR_OK;
}

/**
    Converts an STLINK status code held in the first byte of a response
    to an openocd error, logs any error/wait status as debug output.
*/
static int stlink_usb_error_check(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->st_mode == STLINK_MODE_DEBUG_SWIM) {
		switch (h->databuf[0]) {
			case STLINK_SWIM_ERR_OK:
				return ERROR_OK;
			case STLINK_SWIM_BUSY:
				return ERROR_WAIT;
			default:
				LOG_DEBUG("unknown/unexpected STLINK status code 0x%x", h->databuf[0]);
				return ERROR_FAIL;
		}
	}

	/* TODO: no error checking yet on api V1 */
	if (h->version.jtag_api == STLINK_JTAG_API_V1)
		h->databuf[0] = STLINK_DEBUG_ERR_OK;

	switch (h->databuf[0]) {
		case STLINK_DEBUG_ERR_OK:
			return ERROR_OK;
		case STLINK_DEBUG_ERR_FAULT:
			LOG_DEBUG("SWD fault response (0x%x)", STLINK_DEBUG_ERR_FAULT);
			return ERROR_FAIL;
		case STLINK_SWD_AP_WAIT:
			LOG_DEBUG("wait status SWD_AP_WAIT (0x%x)", STLINK_SWD_AP_WAIT);
			return ERROR_WAIT;
		case STLINK_SWD_DP_WAIT:
			LOG_DEBUG("wait status SWD_DP_WAIT (0x%x)", STLINK_SWD_DP_WAIT);
			return ERROR_WAIT;
		case STLINK_JTAG_GET_IDCODE_ERROR:
			LOG_DEBUG("STLINK_JTAG_GET_IDCODE_ERROR");
			return ERROR_FAIL;
		case STLINK_JTAG_WRITE_ERROR:
			LOG_DEBUG("Write error");
			return ERROR_FAIL;
		case STLINK_JTAG_WRITE_VERIF_ERROR:
			LOG_DEBUG("Write verify error, ignoring");
			return ERROR_OK;
		case STLINK_SWD_AP_FAULT:
			/* git://git.ac6.fr/openocd commit 657e3e885b9ee10
			 * returns ERROR_OK with the comment:
			 * Change in error status when reading outside RAM.
			 * This fix allows CDT plugin to visualize memory.
			 */
			LOG_DEBUG("STLINK_SWD_AP_FAULT");
			return ERROR_FAIL;
		case STLINK_SWD_AP_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_AP_PARITY_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_PARITY_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_DP_FAULT:
			LOG_DEBUG("STLINK_SWD_DP_FAULT");
			return ERROR_FAIL;
		case STLINK_SWD_DP_ERROR:
			LOG_DEBUG("STLINK_SWD_DP_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_DP_PARITY_ERROR:
			LOG_DEBUG("STLINK_SWD_DP_PARITY_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_AP_WDATA_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_WDATA_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_AP_STICKY_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_STICKY_ERROR");
			return ERROR_FAIL;
		case STLINK_SWD_AP_STICKYORUN_ERROR:
			LOG_DEBUG("STLINK_SWD_AP_STICKYORUN_ERROR");
			return ERROR_FAIL;
		case STLINK_BAD_AP_ERROR:
			LOG_DEBUG("STLINK_BAD_AP_ERROR");
			return ERROR_FAIL;
		default:
			LOG_DEBUG("unknown/unexpected STLINK status code 0x%x", h->databuf[0]);
			return ERROR_FAIL;
	}
}

/*
 * Wrapper around stlink_usb_xfer_noerrcheck()
 * to check the error code in the received packet
 */
static int stlink_usb_xfer_errcheck(void *handle, const uint8_t *buf, int size)
{
	int retval;

	assert(size > 0);

	retval = stlink_usb_xfer_noerrcheck(handle, buf, size);
	if (retval != ERROR_OK)
		return retval;

	return stlink_usb_error_check(handle);
}

/** Issue an STLINK command via USB transfer, with retries on any wait status responses.

    Works for commands where the STLINK_DEBUG status is returned in the first
    byte of the response packet. For SWIM a SWIM_READSTATUS is requested instead.

    Returns an openocd result code.
*/
static int stlink_cmd_allow_retry(void *handle, const uint8_t *buf, int size)
{
	int retries = 0;
	int res;
	struct stlink_usb_handle_s *h = handle;

	while (1) {
		if ((h->st_mode != STLINK_MODE_DEBUG_SWIM) || !retries) {
			res = stlink_usb_xfer_noerrcheck(handle, buf, size);
			if (res != ERROR_OK)
				return res;
		}

		if (h->st_mode == STLINK_MODE_DEBUG_SWIM) {
			res = stlink_swim_status(handle);
			if (res != ERROR_OK)
				return res;
		}

		res = stlink_usb_error_check(handle);
		if (res == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			unsigned int delay_us = (1<<retries++) * 1000;
			LOG_DEBUG("stlink_cmd_allow_retry ERROR_WAIT, retry %d, delaying %u microseconds", retries, delay_us);
			usleep(delay_us);
			continue;
		}
		return res;
	}
}

/** */
static int stlink_usb_read_trace(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	assert(h->version.flags & STLINK_F_HAS_TRACE);

	stlink_usb_init_buffer(h, h->trace_ep, 0);

	return stlink_usb_bulk_read(handle, buf, size);
}

/*
	this function writes transfer length in
	the right place in the cb
*/
static void stlink_usb_set_cbw_transfer_datalength(void *handle, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	buf_set_u32(h->cmdbuf+8, 0, 32, size);
}

static void stlink_usb_xfer_v1_create_cmd(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	/* fill the send buffer */
	strcpy((char *)h->cmdbuf, "USBC");
	h->cmdidx += 4;
	/* csw tag not used */
	buf_set_u32(h->cmdbuf+h->cmdidx, 0, 32, 0);
	h->cmdidx += 4;
	/* cbw data transfer length (in the following data phase in or out) */
	buf_set_u32(h->cmdbuf+h->cmdidx, 0, 32, size);
	h->cmdidx += 4;
	/* cbw flags */
	h->cmdbuf[h->cmdidx++] = (direction == h->rx_ep ? ENDPOINT_IN : ENDPOINT_OUT);
	h->cmdbuf[h->cmdidx++] = 0; /* lun */
	/* cdb clength (is filled in at xfer) */
	h->cmdbuf[h->cmdidx++] = 0;
}

/** */
static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	h->direction = direction;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, STLINK_SG_SIZE);
	memset(h->databuf, 0, STLINK_DATA_SIZE);

	if (h->version.stlink == 1)
		stlink_usb_xfer_v1_create_cmd(handle, direction, size);
}

/** */
static int stlink_usb_version(void *handle)
{
	int res;
	uint32_t flags;
	uint16_t version;
	uint8_t v, x, y, jtag, swim, msd, bridge = 0;
	char v_str[5 * (1 + 3) + 1]; /* VvJjMmBbSs */
	char *p;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 6);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_VERSION;

	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 6);

	if (res != ERROR_OK)
		return res;

	version = be_to_h_u16(h->databuf);
	v = (version >> 12) & 0x0f;
	x = (version >> 6) & 0x3f;
	y = version & 0x3f;

	h->vid = le_to_h_u16(h->databuf + 2);
	h->pid = le_to_h_u16(h->databuf + 4);

	switch (h->pid) {
	case STLINK_V2_1_PID:
	case STLINK_V2_1_NO_MSD_PID:
		if ((x <= 22 && y == 7) || (x >= 25 && y >= 7 && y <= 12)) {
			/* MxSy : STM8 V2.1 - SWIM only */
			msd = x;
			swim = y;
			jtag = 0;
		} else {
			/* JxMy : STM32 V2.1 - JTAG/SWD only */
			jtag = x;
			msd = y;
			swim = 0;
		}
		break;
	default:
		jtag = x;
		swim = y;
		msd = 0;
		break;
	}

	/* STLINK-V3 requires a specific command */
	if (v == 3 && x == 0 && y == 0) {
		stlink_usb_init_buffer(handle, h->rx_ep, 16);

		h->cmdbuf[h->cmdidx++] = STLINK_APIV3_GET_VERSION_EX;

		res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 12);
		if (res != ERROR_OK)
			return res;

		v = h->databuf[0];
		swim = h->databuf[1];
		jtag = h->databuf[2];
		msd  = h->databuf[3];
		bridge = h->databuf[4];
		h->vid = le_to_h_u16(h->databuf + 8);
		h->pid = le_to_h_u16(h->databuf + 10);
	}

	h->version.stlink = v;
	h->version.jtag = jtag;
	h->version.swim = swim;

	flags = 0;
	switch (h->version.stlink) {
	case 1:
		/* ST-LINK/V1 from J11 switch to api-v2 (and support SWD) */
		if (h->version.jtag >= 11)
			h->version.jtag_api = STLINK_JTAG_API_V2;
		else
			h->version.jtag_api = STLINK_JTAG_API_V1;

		break;
	case 2:
		/* all ST-LINK/V2 and ST-Link/V2.1 use api-v2 */
		h->version.jtag_api = STLINK_JTAG_API_V2;

		/* API for trace from J13 */
		/* API for target voltage from J13 */
		if (h->version.jtag >= 13)
			flags |= STLINK_F_HAS_TRACE;

		/* preferred API to get last R/W status from J15 */
		if (h->version.jtag >= 15)
			flags |= STLINK_F_HAS_GETLASTRWSTATUS2;

		/* API to set SWD frequency from J22 */
		if (h->version.jtag >= 22)
			flags |= STLINK_F_HAS_SWD_SET_FREQ;

		/* API to set JTAG frequency from J24 */
		/* API to access DAP registers from J24 */
		if (h->version.jtag >= 24) {
			flags |= STLINK_F_HAS_JTAG_SET_FREQ;
			flags |= STLINK_F_HAS_DAP_REG;
		}

		/* Quirk for read DP in JTAG mode (V2 only) from J24, fixed in J32 */
		if (h->version.jtag >= 24 && h->version.jtag < 32)
			flags |= STLINK_F_QUIRK_JTAG_DP_READ;

		/* API to read/write memory at 16 bit from J26 */
		if (h->version.jtag >= 26)
			flags |= STLINK_F_HAS_MEM_16BIT;

		/* API required to init AP before any AP access from J28 */
		if (h->version.jtag >= 28)
			flags |= STLINK_F_HAS_AP_INIT;

		/* API required to return proper error code on close AP from J29 */
		if (h->version.jtag >= 29)
			flags |= STLINK_F_FIX_CLOSE_AP;

		/* Banked regs (DPv1 & DPv2) support from V2J32 */
		if (h->version.jtag >= 32)
			flags |= STLINK_F_HAS_DPBANKSEL;

		break;
	case 3:
		/* all STLINK-V3 use api-v3 */
		h->version.jtag_api = STLINK_JTAG_API_V3;

		/* STLINK-V3 is a superset of ST-LINK/V2 */

		/* API for trace */
		/* API for target voltage */
		flags |= STLINK_F_HAS_TRACE;

		/* preferred API to get last R/W status */
		flags |= STLINK_F_HAS_GETLASTRWSTATUS2;

		/* API to access DAP registers */
		flags |= STLINK_F_HAS_DAP_REG;

		/* API to read/write memory at 16 bit */
		flags |= STLINK_F_HAS_MEM_16BIT;

		/* API required to init AP before any AP access */
		flags |= STLINK_F_HAS_AP_INIT;

		/* API required to return proper error code on close AP */
		flags |= STLINK_F_FIX_CLOSE_AP;

		/* Banked regs (DPv1 & DPv2) support from V3J2 */
		if (h->version.jtag >= 2)
			flags |= STLINK_F_HAS_DPBANKSEL;

		/* 8bit read/write max packet size 512 bytes from V3J6 */
		if (h->version.jtag >= 6)
			flags |= STLINK_F_HAS_RW8_512BYTES;

		break;
	default:
		break;
	}
	h->version.flags = flags;

	p = v_str;
	p += sprintf(p, "V%d", v);
	if (jtag || !msd)
		p += sprintf(p, "J%d", jtag);
	if (msd)
		p += sprintf(p, "M%d", msd);
	if (bridge)
		p += sprintf(p, "B%d", bridge);
	if (swim || !msd)
		sprintf(p, "S%d", swim);

	LOG_INFO("STLINK %s (API v%d) VID:PID %04X:%04X",
		v_str,
		h->version.jtag_api,
		h->vid,
		h->pid);

	return ERROR_OK;
}

static int stlink_usb_check_voltage(void *handle, float *target_voltage)
{
	struct stlink_usb_handle_s *h = handle;
	uint32_t adc_results[2];

	/* no error message, simply quit with error */
	if (!(h->version.flags & STLINK_F_HAS_TARGET_VOLT))
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 8);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_TARGET_VOLTAGE;

	int result = stlink_usb_xfer_noerrcheck(handle, h->databuf, 8);

	if (result != ERROR_OK)
		return result;

	/* convert result */
	adc_results[0] = le_to_h_u32(h->databuf);
	adc_results[1] = le_to_h_u32(h->databuf + 4);

	*target_voltage = 0;

	if (adc_results[0])
		*target_voltage = 2 * ((float)adc_results[1]) * (float)(1.2 / adc_results[0]);

	LOG_INFO("Target voltage: %f", (double)*target_voltage);

	return ERROR_OK;
}

static int stlink_usb_set_swdclk(void *handle, uint16_t clk_divisor)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_SWD_SET_FREQ))
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_SWD_SET_FREQ;
	h_u16_to_le(h->cmdbuf+h->cmdidx, clk_divisor);
	h->cmdidx += 2;

	int result = stlink_cmd_allow_retry(handle, h->databuf, 2);

	if (result != ERROR_OK)
		return result;

	return ERROR_OK;
}

static int stlink_usb_set_jtagclk(void *handle, uint16_t clk_divisor)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_JTAG_SET_FREQ))
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_JTAG_SET_FREQ;
	h_u16_to_le(h->cmdbuf+h->cmdidx, clk_divisor);
	h->cmdidx += 2;

	int result = stlink_cmd_allow_retry(handle, h->databuf, 2);

	if (result != ERROR_OK)
		return result;

	return ERROR_OK;
}

/** */
static int stlink_usb_current_mode(void *handle, uint8_t *mode)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_CURRENT_MODE;

	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	*mode = h->databuf[0];

	return ERROR_OK;
}

/** */
static int stlink_usb_mode_enter(void *handle, enum stlink_mode type)
{
	int rx_size = 0;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* on api V2 we are able the read the latest command
	 * status
	 * TODO: we need the test on api V1 too
	 */
	if (h->version.jtag_api != STLINK_JTAG_API_V1)
		rx_size = 2;

	stlink_usb_init_buffer(handle, h->rx_ep, rx_size);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			if (h->version.jtag_api == STLINK_JTAG_API_V1)
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_ENTER;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_ENTER_JTAG_NO_RESET;
			break;
		case STLINK_MODE_DEBUG_SWD:
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			if (h->version.jtag_api == STLINK_JTAG_API_V1)
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_ENTER;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_ENTER_SWD_NO_RESET;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_ENTER;
			/* swim enter does not return any response or status */
			return stlink_usb_xfer_noerrcheck(handle, h->databuf, 0);
		case STLINK_MODE_DFU:
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	return stlink_cmd_allow_retry(handle, h->databuf, rx_size);
}

/** */
static int stlink_usb_mode_leave(void *handle, enum stlink_mode type)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* command with no reply, use a valid endpoint but zero size */
	stlink_usb_init_buffer(handle, h->rx_ep, 0);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
		case STLINK_MODE_DEBUG_SWD:
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_EXIT;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_EXIT;
			break;
		case STLINK_MODE_DFU:
			h->cmdbuf[h->cmdidx++] = STLINK_DFU_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_DFU_EXIT;
			break;
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 0);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

static int stlink_usb_assert_srst(void *handle, int srst);

static enum stlink_mode stlink_get_mode(enum hl_transports t)
{
	switch (t) {
	case HL_TRANSPORT_SWD:
		return STLINK_MODE_DEBUG_SWD;
	case HL_TRANSPORT_JTAG:
		return STLINK_MODE_DEBUG_JTAG;
	default:
		return STLINK_MODE_UNKNOWN;
	}
}

/** */
static int stlink_usb_exit_mode(void *handle)
{
	int res;
	uint8_t mode;
	enum stlink_mode emode;

	assert(handle != NULL);

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* try to exit current mode */
	switch (mode) {
		case STLINK_DEV_DFU_MODE:
			emode = STLINK_MODE_DFU;
			break;
		case STLINK_DEV_DEBUG_MODE:
			emode = STLINK_MODE_DEBUG_SWD;
			break;
		case STLINK_DEV_SWIM_MODE:
			emode = STLINK_MODE_DEBUG_SWIM;
			break;
		case STLINK_DEV_BOOTLOADER_MODE:
		case STLINK_DEV_MASS_MODE:
		default:
			emode = STLINK_MODE_UNKNOWN;
			break;
	}

	if (emode != STLINK_MODE_UNKNOWN)
		return stlink_usb_mode_leave(handle, emode);

	return ERROR_OK;
}

/** */
static int stlink_usb_init_mode(void *handle, bool connect_under_reset, int initial_interface_speed)
{
	int res;
	uint8_t mode;
	enum stlink_mode emode;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	res = stlink_usb_exit_mode(handle);
	if (res != ERROR_OK)
		return res;

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	/* we check the target voltage here as an aid to debugging connection problems.
	 * the stlink requires the target Vdd to be connected for reliable debugging.
	 * this cmd is supported in all modes except DFU
	 */
	if (mode != STLINK_DEV_DFU_MODE) {

		float target_voltage;

		/* check target voltage (if supported) */
		res = stlink_usb_check_voltage(h, &target_voltage);

		if (res != ERROR_OK) {
			if (res != ERROR_COMMAND_NOTFOUND)
				LOG_ERROR("voltage check failed");
			/* attempt to continue as it is not a catastrophic failure */
		} else {
			/* check for a sensible target voltage, operating range is 1.65-5.5v
			 * according to datasheet */
			if (target_voltage < 1.5)
				LOG_ERROR("target voltage may be too low for reliable debugging");
		}
	}

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* set selected mode */
	emode = h->st_mode;

	if (emode == STLINK_MODE_UNKNOWN) {
		LOG_ERROR("selected mode (transport) not supported");
		return ERROR_FAIL;
	}

	/* set the speed before entering the mode, as the chip discovery phase should be done at this speed too */
	if (emode == STLINK_MODE_DEBUG_JTAG) {
		if (h->version.flags & STLINK_F_HAS_JTAG_SET_FREQ) {
			stlink_dump_speed_map(stlink_khz_to_speed_map_jtag, ARRAY_SIZE(stlink_khz_to_speed_map_jtag));
			stlink_speed(h, initial_interface_speed, false);
		}
	} else if (emode == STLINK_MODE_DEBUG_SWD) {
		if (h->version.flags & STLINK_F_HAS_SWD_SET_FREQ) {
			stlink_dump_speed_map(stlink_khz_to_speed_map_swd, ARRAY_SIZE(stlink_khz_to_speed_map_swd));
			stlink_speed(h, initial_interface_speed, false);
		}
	}

	if (h->version.jtag_api == STLINK_JTAG_API_V3) {
		struct speed_map map[STLINK_V3_MAX_FREQ_NB];

		stlink_get_com_freq(h, (emode == STLINK_MODE_DEBUG_JTAG), map);
		stlink_dump_speed_map(map, ARRAY_SIZE(map));
		stlink_speed(h, initial_interface_speed, false);
	}

	/* preliminary SRST assert:
	 * We want SRST is asserted before activating debug signals (mode_enter).
	 * As the required mode has not been set, the adapter may not know what pin to use.
	 * Tested firmware STLINK v2 JTAG v29 API v2 SWIM v0 uses T_NRST pin by default
	 * Tested firmware STLINK v2 JTAG v27 API v2 SWIM v6 uses T_NRST pin by default
	 * after power on, SWIM_RST stays unchanged */
	if (connect_under_reset && emode != STLINK_MODE_DEBUG_SWIM)
		stlink_usb_assert_srst(handle, 0);
		/* do not check the return status here, we will
		   proceed and enter the desired mode below
		   and try asserting srst again. */

	res = stlink_usb_mode_enter(handle, emode);
	if (res != ERROR_OK)
		return res;

	/* assert SRST again: a little bit late but now the adapter knows for sure what pin to use */
	if (connect_under_reset) {
		res = stlink_usb_assert_srst(handle, 0);
		if (res != ERROR_OK)
			return res;
	}

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: 0x%02X", mode);

	return ERROR_OK;
}

/* request status from last swim request */
static int stlink_swim_status(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 4);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READSTATUS;
	/* error is checked by the caller */
	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 4);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}
/*
	the purpose of this function is unknown...
	capabilities? anyway for swim v6 it returns
	0001020600000000
*/
__attribute__((unused))
static int stlink_swim_cap(void *handle, uint8_t *cap)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 8);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READ_CAP;
	h->cmdbuf[h->cmdidx++] = 0x01;
	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 8);
	if (res != ERROR_OK)
		return res;
	memcpy(cap, h->databuf, 8);
	return ERROR_OK;
}

/*	debug dongle assert/deassert sreset line */
static int stlink_swim_assert_reset(void *handle, int reset)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	if (!reset)
		h->cmdbuf[h->cmdidx++] = STLINK_SWIM_ASSERT_RESET;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_SWIM_DEASSERT_RESET;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*
	send swim enter seq
	1.3ms low then 750Hz then 1.5kHz
*/
static int stlink_swim_enter(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_ENTER_SEQ;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*	switch high/low speed swim */
static int stlink_swim_speed(void *handle, int speed)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_SPEED;
	if (speed)
		h->cmdbuf[h->cmdidx++] = 1;
	else
		h->cmdbuf[h->cmdidx++] = 0;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*
	initiate srst from swim.
	nrst is pulled low for 50us.
*/
static int stlink_swim_generate_rst(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_GEN_RST;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

/*
	send resynchronize sequence
	swim is pulled low for 16us
	reply is 64 clks low
*/
static int stlink_swim_resync(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_RESET;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

static int stlink_swim_writebytes(void *handle, uint32_t addr, uint32_t len, const uint8_t *data)
{
	struct stlink_usb_handle_s *h = handle;
	int res;
	unsigned int i;
	unsigned int datalen = 0;
	int cmdsize = STLINK_CMD_SIZE_V2;

	if (len > STLINK_DATA_SIZE)
		return ERROR_FAIL;

	if (h->version.stlink == 1)
		cmdsize = STLINK_SG_SIZE;

	stlink_usb_init_buffer(handle, h->tx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_WRITEMEM;
	h_u16_to_be(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h_u32_to_be(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	for (i = 0; i < len; i++) {
		if (h->cmdidx == cmdsize)
			h->databuf[datalen++] = *(data++);
		else
			h->cmdbuf[h->cmdidx++] = *(data++);
	}
	if (h->version.stlink == 1)
		stlink_usb_set_cbw_transfer_datalength(handle, datalen);

	res = stlink_cmd_allow_retry(handle, h->databuf, datalen);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

static int stlink_swim_readbytes(void *handle, uint32_t addr, uint32_t len, uint8_t *data)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	if (len > STLINK_DATA_SIZE)
		return ERROR_FAIL;

	stlink_usb_init_buffer(handle, h->rx_ep, 0);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READMEM;
	h_u16_to_be(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;
	h_u32_to_be(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	res = stlink_cmd_allow_retry(handle, h->databuf, 0);
	if (res != ERROR_OK)
		return res;

	stlink_usb_init_buffer(handle, h->rx_ep, len);
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_SWIM_READBUF;
	res = stlink_usb_xfer_noerrcheck(handle, data, len);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_idcode(void *handle, uint32_t *idcode)
{
	int res, offset;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* there is no swim read core id cmd */
	if (h->st_mode == STLINK_MODE_DEBUG_SWIM) {
		*idcode = 0;
		return ERROR_OK;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 12);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->version.jtag_api == STLINK_JTAG_API_V1) {
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READCOREID;

		res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 4);
		offset = 0;
	} else {
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READ_IDCODES;

		res = stlink_usb_xfer_errcheck(handle, h->databuf, 12);
		offset = 4;
	}

	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->databuf + offset);

	LOG_DEBUG("IDCODE: 0x%08" PRIX32, *idcode);

	return ERROR_OK;
}

static int stlink_usb_v2_read_debug_reg(void *handle, uint32_t addr, uint32_t *val)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 8);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READDEBUGREG;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;

	res = stlink_cmd_allow_retry(handle, h->databuf, 8);
	if (res != ERROR_OK)
		return res;

	*val = le_to_h_u32(h->databuf + 4);
	return ERROR_OK;
}

static int stlink_usb_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->version.jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_WRITEDEBUGREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITEDEBUGREG;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u32_to_le(h->cmdbuf+h->cmdidx, val);
	h->cmdidx += 4;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_trace_read(void *handle, uint8_t *buf, size_t *size)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->trace.enabled && (h->version.flags & STLINK_F_HAS_TRACE)) {
		int res;

		stlink_usb_init_buffer(handle, h->rx_ep, 10);

		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_GET_TRACE_NB;

		res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 2);
		if (res != ERROR_OK)
			return res;

		size_t bytes_avail = le_to_h_u16(h->databuf);
		*size = bytes_avail < *size ? bytes_avail : *size - 1;

		if (*size > 0) {
			res = stlink_usb_read_trace(handle, buf, *size);
			if (res != ERROR_OK)
				return res;
			return ERROR_OK;
		}
	}
	*size = 0;
	return ERROR_OK;
}

static enum target_state stlink_usb_v2_get_status(void *handle)
{
	int result;
	uint32_t status;

	result = stlink_usb_v2_read_debug_reg(handle, DCB_DHCSR, &status);
	if  (result != ERROR_OK)
		return TARGET_UNKNOWN;

	if (status & S_HALT)
		return TARGET_HALTED;
	else if (status & S_RESET_ST)
		return TARGET_RESET;

	return TARGET_RUNNING;
}

/** */
static enum target_state stlink_usb_state(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->reconnect_pending) {
		LOG_INFO("Previous state query failed, trying to reconnect");
		res = stlink_usb_mode_enter(handle, h->st_mode);
		if (res != ERROR_OK)
			return TARGET_UNKNOWN;

		h->reconnect_pending = false;
	}

	if (h->version.jtag_api != STLINK_JTAG_API_V1) {
		res = stlink_usb_v2_get_status(handle);
		if (res == TARGET_UNKNOWN)
			h->reconnect_pending = true;
		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_GETSTATUS;

	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	if (h->databuf[0] == STLINK_CORE_RUNNING)
		return TARGET_RUNNING;
	if (h->databuf[0] == STLINK_CORE_HALTED)
		return TARGET_HALTED;

	h->reconnect_pending = true;

	return TARGET_UNKNOWN;
}

static int stlink_usb_assert_srst(void *handle, int srst)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->st_mode == STLINK_MODE_DEBUG_SWIM)
		return stlink_swim_assert_reset(handle, srst);

	if (h->version.stlink == 1)
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_DRIVE_NRST;
	h->cmdbuf[h->cmdidx++] = srst;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static void stlink_usb_trace_disable(void *handle)
{
	int res = ERROR_OK;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	assert(h->version.flags & STLINK_F_HAS_TRACE);

	LOG_DEBUG("Tracing: disable");

	stlink_usb_init_buffer(handle, h->rx_ep, 2);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_STOP_TRACE_RX;
	res = stlink_usb_xfer_errcheck(handle, h->databuf, 2);

	if (res == ERROR_OK)
		h->trace.enabled = false;
}


/** */
static int stlink_usb_trace_enable(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.flags & STLINK_F_HAS_TRACE) {
		stlink_usb_init_buffer(handle, h->rx_ep, 10);

		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_START_TRACE_RX;
		h_u16_to_le(h->cmdbuf+h->cmdidx, (uint16_t)STLINK_TRACE_SIZE);
		h->cmdidx += 2;
		h_u32_to_le(h->cmdbuf+h->cmdidx, h->trace.source_hz);
		h->cmdidx += 4;

		res = stlink_usb_xfer_errcheck(handle, h->databuf, 2);

		if (res == ERROR_OK)  {
			h->trace.enabled = true;
			LOG_DEBUG("Tracing: recording at %" PRIu32 "Hz", h->trace.source_hz);
		}
	} else {
		LOG_ERROR("Tracing is not supported by this version.");
		res = ERROR_FAIL;
	}

	return res;
}

/** */
static int stlink_usb_reset(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	int retval;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;

	if (h->version.jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_RESETSYS;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_RESETSYS;

	retval = stlink_cmd_allow_retry(handle, h->databuf, 2);
	if (retval != ERROR_OK)
		return retval;

	if (h->trace.enabled) {
		stlink_usb_trace_disable(h);
		return stlink_usb_trace_enable(h);
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_run(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.jtag_api != STLINK_JTAG_API_V1) {
		res = stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_DEBUGEN);

		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_RUNCORE;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_halt(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.jtag_api != STLINK_JTAG_API_V1) {
		res = stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);

		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_FORCEDEBUG;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_step(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.jtag_api != STLINK_JTAG_API_V1) {
		/* TODO: this emulates the v1 api, it should really use a similar auto mask isr
		 * that the Cortex-M3 currently does. */
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_MASKINTS|C_DEBUGEN);
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_STEP|C_MASKINTS|C_DEBUGEN);
		return stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_STEPCORE;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

/** */
static int stlink_usb_read_regs(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 88);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->version.jtag_api == STLINK_JTAG_API_V1) {

		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_READALLREGS;
		res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 84);
		/* regs data from offset 0 */
	} else {
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READALLREGS;
		res = stlink_usb_xfer_errcheck(handle, h->databuf, 88);
		/* status at offset 0, regs data from offset 4 */
	}

	return res;
}

/** */
static int stlink_usb_read_reg(void *handle, unsigned int regsel, uint32_t *val)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (STLINK_REGSEL_IS_FPU(regsel) && !(h->version.flags & STLINK_F_HAS_FPU_REG)) {
		res = stlink_usb_write_debug_reg(h, DCB_DCRSR, regsel & 0x7f);
		if (res != ERROR_OK)
			return res;

		/* FIXME: poll DHCSR.S_REGRDY before read DCRDR */
		return stlink_usb_v2_read_debug_reg(h, DCB_DCRDR, val);
	}

	stlink_usb_init_buffer(handle, h->rx_ep, h->version.jtag_api == STLINK_JTAG_API_V1 ? 4 : 8);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->version.jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_READREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READREG;
	h->cmdbuf[h->cmdidx++] = regsel;

	if (h->version.jtag_api == STLINK_JTAG_API_V1) {
		res = stlink_usb_xfer_noerrcheck(handle, h->databuf, 4);
		if (res != ERROR_OK)
			return res;
		*val = le_to_h_u32(h->databuf);
		return ERROR_OK;
	} else {
		res = stlink_cmd_allow_retry(handle, h->databuf, 8);
		if (res != ERROR_OK)
			return res;
		*val = le_to_h_u32(h->databuf + 4);
		return ERROR_OK;
	}
}

/** */
static int stlink_usb_write_reg(void *handle, unsigned int regsel, uint32_t val)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (STLINK_REGSEL_IS_FPU(regsel) && !(h->version.flags & STLINK_F_HAS_FPU_REG)) {
		int res = stlink_usb_write_debug_reg(h, DCB_DCRDR, val);
		if (res != ERROR_OK)
			return res;

		return stlink_usb_write_debug_reg(h, DCB_DCRSR, DCRSR_WnR | (regsel & 0x7f));
		/* FIXME: poll DHCSR.S_REGRDY after write DCRSR */
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->version.jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_WRITEREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITEREG;
	h->cmdbuf[h->cmdidx++] = regsel;
	h_u32_to_le(h->cmdbuf+h->cmdidx, val);
	h->cmdidx += 4;

	return stlink_cmd_allow_retry(handle, h->databuf, 2);
}

static int stlink_usb_get_rw_status(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.jtag_api == STLINK_JTAG_API_V1)
		return ERROR_OK;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->version.flags & STLINK_F_HAS_GETLASTRWSTATUS2) {
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_GETLASTRWSTATUS2;
		return stlink_usb_xfer_errcheck(handle, h->databuf, 12);
	} else {
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_GETLASTRWSTATUS;
		return stlink_usb_xfer_errcheck(handle, h->databuf, 2);
	}
}

/** */
static int stlink_usb_read_mem8(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	uint16_t read_len = len;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* max 8 bit read/write is 64 bytes or 512 bytes for v3 */
	if (len > stlink_usb_block(h)) {
		LOG_DEBUG("max buffer (%d) length exceeded", stlink_usb_block(h));
		return ERROR_FAIL;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, read_len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READMEM_8BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	/* we need to fix read length for single bytes */
	if (read_len == 1)
		read_len++;

	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, read_len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem8(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* max 8 bit read/write is 64 bytes or 512 bytes for v3 */
	if (len > stlink_usb_block(h)) {
		LOG_DEBUG("max buffer length (%d) exceeded", stlink_usb_block(h));
		return ERROR_FAIL;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_WRITEMEM_8BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer_noerrcheck(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_read_mem16(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_MEM_16BIT))
		return ERROR_COMMAND_NOTFOUND;

	/* data must be a multiple of 2 and half-word aligned */
	if (len % 2 || addr % 2) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READMEM_16BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem16(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_MEM_16BIT))
		return ERROR_COMMAND_NOTFOUND;

	/* data must be a multiple of 2 and half-word aligned */
	if (len % 2 || addr % 2) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITEMEM_16BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer_noerrcheck(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_read_mem32(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READMEM_32BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer_noerrcheck(handle, h->databuf, len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_WRITEMEM_32BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer_noerrcheck(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

static uint32_t stlink_max_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	uint32_t max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));
	if (max_tar_block == 0)
		max_tar_block = 4;
	return max_tar_block;
}

static int stlink_usb_read_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	int retries = 0;
	struct stlink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	/* switch to 8 bit if stlink does not support 16 bit memory read */
	if (size == 2 && !(h->version.flags & STLINK_F_HAS_MEM_16BIT))
		size = 1;

	while (count) {

		bytes_remaining = (size != 1) ?
				stlink_max_block_size(h->max_mem_packet, addr) : stlink_usb_block(h);

		if (count < bytes_remaining)
			bytes_remaining = count;

		/*
		 * all stlink support 8/32bit memory read/writes and only from
		 * stlink V2J26 there is support for 16 bit memory read/write.
		 * Honour 32 bit and, if possible, 16 bit too. Otherwise, handle
		 * as 8bit access.
		 */
		if (size != 1) {

			/* When in jtag mode the stlink uses the auto-increment functionality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the stlink is a hla adapter and so this
			 * needs implementing manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr & (size - 1)) {

				uint32_t head_bytes = size - (addr & (size - 1));
				retval = stlink_usb_read_mem8(handle, addr, head_bytes, buffer);
				if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
					usleep((1<<retries++) * 1000);
					continue;
				}
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining & (size - 1))
				retval = stlink_usb_read_mem(handle, addr, 1, bytes_remaining, buffer);
			else if (size == 2)
				retval = stlink_usb_read_mem16(handle, addr, bytes_remaining, buffer);
			else
				retval = stlink_usb_read_mem32(handle, addr, bytes_remaining, buffer);
		} else
			retval = stlink_usb_read_mem8(handle, addr, bytes_remaining, buffer);

		if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			usleep((1<<retries++) * 1000);
			continue;
		}
		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int stlink_usb_write_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	int retries = 0;
	struct stlink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	/* switch to 8 bit if stlink does not support 16 bit memory read */
	if (size == 2 && !(h->version.flags & STLINK_F_HAS_MEM_16BIT))
		size = 1;

	while (count) {

		bytes_remaining = (size != 1) ?
				stlink_max_block_size(h->max_mem_packet, addr) : stlink_usb_block(h);

		if (count < bytes_remaining)
			bytes_remaining = count;

		/*
		 * all stlink support 8/32bit memory read/writes and only from
		 * stlink V2J26 there is support for 16 bit memory read/write.
		 * Honour 32 bit and, if possible, 16 bit too. Otherwise, handle
		 * as 8bit access.
		 */
		if (size != 1) {

			/* When in jtag mode the stlink uses the auto-increment functionality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the stlink is a hla adapter and so this
			 * needs implementing manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr & (size - 1)) {

				uint32_t head_bytes = size - (addr & (size - 1));
				retval = stlink_usb_write_mem8(handle, addr, head_bytes, buffer);
				if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
					usleep((1<<retries++) * 1000);
					continue;
				}
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining & (size - 1))
				retval = stlink_usb_write_mem(handle, addr, 1, bytes_remaining, buffer);
			else if (size == 2)
				retval = stlink_usb_write_mem16(handle, addr, bytes_remaining, buffer);
			else
				retval = stlink_usb_write_mem32(handle, addr, bytes_remaining, buffer);

		} else
			retval = stlink_usb_write_mem8(handle, addr, bytes_remaining, buffer);
		if (retval == ERROR_WAIT && retries < MAX_WAIT_RETRIES) {
			usleep((1<<retries++) * 1000);
			continue;
		}
		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

/** */
static int stlink_usb_override_target(const char *targetname)
{
	return !strcmp(targetname, "cortex_m");
}

static int stlink_speed_swim(void *handle, int khz, bool query)
{
	int retval;

	/*
			we only have low and high speed...
			before changing speed the SWIM_CSR HS bit
			must be updated
	 */
	if (!query) {
		retval = stlink_swim_speed(handle, (khz < SWIM_FREQ_HIGH) ? 0 : 1);
		if (retval != ERROR_OK)
			LOG_ERROR("Unable to set adapter speed");
	}

	return (khz < SWIM_FREQ_HIGH) ? SWIM_FREQ_LOW : SWIM_FREQ_HIGH;
}

static int stlink_match_speed_map(const struct speed_map *map, unsigned int map_size, int khz, bool query)
{
	unsigned int i;
	int speed_index = -1;
	int speed_diff = INT_MAX;
	int last_valid_speed = -1;
	bool match = true;

	for (i = 0; i < map_size; i++) {
		if (!map[i].speed)
			continue;
		last_valid_speed = i;
		if (khz == map[i].speed) {
			speed_index = i;
			break;
		} else {
			int current_diff = khz - map[i].speed;
			/* get abs value for comparison */
			current_diff = (current_diff > 0) ? current_diff : -current_diff;
			if ((current_diff < speed_diff) && khz >= map[i].speed) {
				speed_diff = current_diff;
				speed_index = i;
			}
		}
	}

	if (speed_index == -1) {
		/* this will only be here if we cannot match the slow speed.
		 * use the slowest speed we support.*/
		speed_index = last_valid_speed;
		match = false;
	} else if (i == map_size)
		match = false;

	if (!match && query) {
		LOG_INFO("Unable to match requested speed %d kHz, using %d kHz",
				khz, map[speed_index].speed);
	}

	return speed_index;
}

static int stlink_speed_swd(void *handle, int khz, bool query)
{
	int speed_index;
	struct stlink_usb_handle_s *h = handle;

	/* old firmware cannot change it */
	if (!(h->version.flags & STLINK_F_HAS_SWD_SET_FREQ))
		return khz;

	speed_index = stlink_match_speed_map(stlink_khz_to_speed_map_swd,
		ARRAY_SIZE(stlink_khz_to_speed_map_swd), khz, query);

	if (!query) {
		int result = stlink_usb_set_swdclk(h, stlink_khz_to_speed_map_swd[speed_index].speed_divisor);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to set adapter speed");
			return khz;
		}
	}

	return stlink_khz_to_speed_map_swd[speed_index].speed;
}

static int stlink_speed_jtag(void *handle, int khz, bool query)
{
	int speed_index;
	struct stlink_usb_handle_s *h = handle;

	/* old firmware cannot change it */
	if (!(h->version.flags & STLINK_F_HAS_JTAG_SET_FREQ))
		return khz;

	speed_index = stlink_match_speed_map(stlink_khz_to_speed_map_jtag,
		ARRAY_SIZE(stlink_khz_to_speed_map_jtag), khz, query);

	if (!query) {
		int result = stlink_usb_set_jtagclk(h, stlink_khz_to_speed_map_jtag[speed_index].speed_divisor);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to set adapter speed");
			return khz;
		}
	}

	return stlink_khz_to_speed_map_jtag[speed_index].speed;
}

static void stlink_dump_speed_map(const struct speed_map *map, unsigned int map_size)
{
	unsigned int i;

	LOG_DEBUG("Supported clock speeds are:");
	for (i = 0; i < map_size; i++)
		if (map[i].speed)
			LOG_DEBUG("%d kHz", map[i].speed);
}

static int stlink_get_com_freq(void *handle, bool is_jtag, struct speed_map *map)
{
	struct stlink_usb_handle_s *h = handle;
	int i;

	if (h->version.jtag_api != STLINK_JTAG_API_V3) {
		LOG_ERROR("Unknown command");
		return 0;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 16);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_APIV3_GET_COM_FREQ;
	h->cmdbuf[h->cmdidx++] = is_jtag ? 1 : 0;

	int res = stlink_usb_xfer_errcheck(handle, h->databuf, 52);

	int size = h->databuf[8];

	if (size > STLINK_V3_MAX_FREQ_NB)
		size = STLINK_V3_MAX_FREQ_NB;

	for (i = 0; i < size; i++) {
		map[i].speed = le_to_h_u32(&h->databuf[12 + 4 * i]);
		map[i].speed_divisor = i;
	}

	/* set to zero all the next entries */
	for (i = size; i < STLINK_V3_MAX_FREQ_NB; i++)
		map[i].speed = 0;

	return res;
}

static int stlink_set_com_freq(void *handle, bool is_jtag, unsigned int frequency)
{
	struct stlink_usb_handle_s *h = handle;

	if (h->version.jtag_api != STLINK_JTAG_API_V3) {
		LOG_ERROR("Unknown command");
		return 0;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 16);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_APIV3_SET_COM_FREQ;
	h->cmdbuf[h->cmdidx++] = is_jtag ? 1 : 0;
	h->cmdbuf[h->cmdidx++] = 0;

	h_u32_to_le(&h->cmdbuf[4], frequency);

	return stlink_usb_xfer_errcheck(handle, h->databuf, 8);
}

static int stlink_speed_v3(void *handle, bool is_jtag, int khz, bool query)
{
	struct stlink_usb_handle_s *h = handle;
	int speed_index;
	struct speed_map map[STLINK_V3_MAX_FREQ_NB];

	stlink_get_com_freq(h, is_jtag, map);

	speed_index = stlink_match_speed_map(map, ARRAY_SIZE(map), khz, query);

	if (!query) {
		int result = stlink_set_com_freq(h, is_jtag, map[speed_index].speed);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to set adapter speed");
			return khz;
		}
	}
	return map[speed_index].speed;
}

static int stlink_speed(void *handle, int khz, bool query)
{
	struct stlink_usb_handle_s *h = handle;

	if (!handle)
		return khz;

	switch (h->st_mode) {
	case STLINK_MODE_DEBUG_SWIM:
		return stlink_speed_swim(handle, khz, query);
	case STLINK_MODE_DEBUG_SWD:
		if (h->version.jtag_api == STLINK_JTAG_API_V3)
			return stlink_speed_v3(handle, false, khz, query);
		else
			return stlink_speed_swd(handle, khz, query);
		break;
	case STLINK_MODE_DEBUG_JTAG:
		if (h->version.jtag_api == STLINK_JTAG_API_V3)
			return stlink_speed_v3(handle, true, khz, query);
		else
			return stlink_speed_jtag(handle, khz, query);
		break;
	default:
		break;
	}

	return khz;
}

/** */
static int stlink_usb_usb_close(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_usb_priv_s *itf_priv = h->itf->priv;

	if (h && itf_priv->fd) {
		stlink_usb_exit_mode(h);
		/* do not check return code, it prevent
		us from closing jtag_libusb */
		jtag_libusb_close(itf_priv->fd);
	}

	return ERROR_OK;
}

/** */
static int stlink_tcp_close(void *handle)
{
	struct stlink_usb_handle_s *h = handle;
	struct stlink_tcp_priv_s *itf_priv = h->itf->priv;

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
			ret = stlink_tcp_send_cmd(h, cmdbuf, 8, databuf, 4, true);
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
static int stlink_close(void *handle)
{
	if (handle != NULL) {
		struct stlink_usb_handle_s *h = handle;

		stlink_usb_close(handle);

		free(h->itf->priv);
		free(h);
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
static int stlink_usb_usb_open(void *handle, struct hl_interface_param_s *param)
{
	struct stlink_usb_handle_s *h = handle;
	int err, retry_count = 1;

	h->itf->priv = calloc(1, sizeof(struct stlink_usb_priv_s));
	if (h->itf->priv == NULL)
		return ERROR_FAIL;

	struct stlink_usb_priv_s *itf_priv = h->itf->priv;

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
				&itf_priv->fd, stlink_usb_get_alternate_serial) != ERROR_OK) {
			LOG_ERROR("open failed");
			return ERROR_FAIL;
		}

		jtag_libusb_set_configuration(itf_priv->fd, 0);

		if (libusb_claim_interface(itf_priv->fd, 0) != ERROR_OK) {
			LOG_DEBUG("claim interface failed");
			return ERROR_FAIL;
		}

		/* RX EP is common for all versions */
		h->rx_ep = STLINK_RX_EP;

		uint16_t pid;
		if (jtag_libusb_get_pid(libusb_get_device(itf_priv->fd), &pid) != ERROR_OK) {
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
		err = stlink_usb_version(h);

		if (err == ERROR_OK) {
			break;
		} else if (h->version.stlink == 1 ||
			   retry_count == 0) {
			LOG_ERROR("read version failed");
			return ERROR_FAIL;
		} else {
			err = libusb_release_interface(itf_priv->fd, 0);
			if (err != ERROR_OK) {
				LOG_ERROR("release interface failed");
				return ERROR_FAIL;
			}

			err = libusb_reset_device(itf_priv->fd);
			if (err != ERROR_OK) {
				LOG_ERROR("reset device failed");
				return ERROR_FAIL;
			}

			jtag_libusb_close(itf_priv->fd);
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
static int stlink_tcp_open(void *handle, struct hl_interface_param_s *param)
{
	struct stlink_usb_handle_s *h = handle;
	int ret;
	uint8_t cmdbuf[STLINK_TCP_SEND_BUFFER_SIZE];
	uint8_t databuf[STLINK_TCP_RECV_BUFFER_SIZE];

	h->itf->priv = calloc(1, sizeof(struct stlink_tcp_priv_s));
	if (h->itf->priv == NULL)
		return ERROR_FAIL;

	struct stlink_tcp_priv_s *itf_priv = h->itf->priv;

	/* SWIM is not supported using stlink-server */
	if (h->st_mode ==  STLINK_MODE_DEBUG_SWIM) {
		LOG_ERROR("stlink-server does not support SWIM mode");
		return ERROR_FAIL;
	}

	/* configure directions */
	h->rx_ep = STLINK_TCP_REQUEST_READ;
	h->tx_ep = STLINK_TCP_REQUEST_WRITE;
	h->trace_ep = STLINK_TCP_REQUEST_READ_SWO;

	/* setting max_mem_packet to 4K for Cortex-M3 and M4 will cause stlink-server
	 * to crash in windows : select a safe default value (1K) */
	h->max_mem_packet = (1 << 10);

	itf_priv->fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	itf_priv->connected = false;
	itf_priv->device_id = 0;
	itf_priv->connect_id = 0;

	struct sockaddr_in serv;
	memset(&serv, 0, sizeof(struct sockaddr_in));
	serv.sin_family = AF_INET;
	serv.sin_port = htons(param->stlink_tcp_port);
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
	ret = stlink_tcp_send_cmd(h, cmdbuf, 4, databuf, 16, false);
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
	ret = stlink_tcp_send_cmd(h, cmdbuf, 2, databuf, 4, true);
	if (ret != ERROR_OK)
		return ret;

	/* get the number of connected stlinks */
	cmdbuf[0] = STLINK_TCP_CMD_GET_NB_DEV;
	ret = stlink_tcp_send_cmd(h, cmdbuf, 1, databuf, 4, false);
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
		ret = stlink_tcp_send_cmd(h, cmdbuf, 8, databuf, 45, true);
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

				stlink_serial_matched = strcmp(fixed_serial, (const char *) serial) == 0;
			} else
				stlink_serial_matched = strcmp(param->serial, (const char *) serial) == 0;
		}

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
	ret = stlink_tcp_send_cmd(h, cmdbuf, 8, databuf, 8, true);
	if (ret != ERROR_OK)
		return ret;

	itf_priv->connect_id = le_to_h_u32(&databuf[4]);

	/* get stlink version */
	ret = stlink_usb_version(h);
	if (ret != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static struct stlink_interface_s stlink_usb_itf = {
	.open = stlink_usb_usb_open,
	.close = stlink_usb_usb_close,
	.xfer_noerrcheck = stlink_usb_usb_xfer_noerrcheck,
	.bulk_read = stlink_usb_usb_bulk_read,
};

static struct stlink_interface_s stlink_tcp_itf = {
	.open = stlink_tcp_open,
	.close = stlink_tcp_close,
	.xfer_noerrcheck = stlink_tcp_xfer_noerrcheck,
	.bulk_read = stlink_tcp_xfer_noerrcheck,
};

static int stlink_open(struct hl_interface_param_s *param, enum stlink_mode mode, void **fd)
{
	struct stlink_usb_handle_s *h;

	LOG_DEBUG("stlink_open");

	h = calloc(1, sizeof(struct stlink_usb_handle_s));

	if (h == 0) {
		LOG_DEBUG("malloc failed");
		return ERROR_FAIL;
	}

	h->st_mode = mode;

	for (unsigned i = 0; param->vid[i]; i++) {
		LOG_DEBUG("transport: %d vid: 0x%04x pid: 0x%04x serial: %s",
			  h->st_mode, param->vid[i], param->pid[i],
			  param->serial ? param->serial : "");
	}

	if (param->use_stlink_tcp)
		h->itf = &stlink_tcp_itf;
	else
		h->itf = &stlink_usb_itf;

	if (stlink_usb_open(h, param) != ERROR_OK)
		goto error_open;

	/* check if mode is supported */
	int err = ERROR_OK;

	switch (h->st_mode) {
		case STLINK_MODE_DEBUG_SWD:
			if (h->version.jtag_api == STLINK_JTAG_API_V1)
				err = ERROR_FAIL;
			/* fall-through */
		case STLINK_MODE_DEBUG_JTAG:
			if (h->version.jtag == 0)
				err = ERROR_FAIL;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			if (h->version.swim == 0)
				err = ERROR_FAIL;
			break;
		default:
			err = ERROR_FAIL;
			break;
	}

	if (err != ERROR_OK) {
		LOG_ERROR("mode (transport) not supported by device");
		goto error_open;
	}

	/* initialize the debug hardware */
	err = stlink_usb_init_mode(h, param->connect_under_reset, param->initial_interface_speed);

	if (err != ERROR_OK) {
		LOG_ERROR("init mode failed (unable to connect to the target)");
		goto error_open;
	}

	if (h->st_mode == STLINK_MODE_DEBUG_SWIM) {
		err = stlink_swim_enter(h);
		if (err != ERROR_OK) {
			LOG_ERROR("stlink_swim_enter_failed (unable to connect to the target)");
			goto error_open;
		}
		*fd = h;
		h->max_mem_packet = STLINK_DATA_SIZE;
		return ERROR_OK;
	}

	/* set max_mem_packet if it was not set by the low-level interface */
	if (h->max_mem_packet == 0) {
		/* get cpuid, so we can determine the max page size
		 * start with a safe default */
		h->max_mem_packet = (1 << 10);

		uint8_t buffer[4];
		stlink_usb_open_ap(h, 0);
		err = stlink_usb_read_mem32(h, CPUID, 4, buffer);
		if (err == ERROR_OK) {
			uint32_t cpuid = le_to_h_u32(buffer);
			int i = (cpuid >> 4) & 0xf;
			if (i == 4 || i == 3) {
				/* Cortex-M3/M4 has 4096 bytes autoincrement range */
				h->max_mem_packet = (1 << 12);
			}
		}

		LOG_DEBUG("Using TAR autoincrement: %" PRIu32, h->max_mem_packet);
	}

	*fd = h;

	return ERROR_OK;

error_open:
	stlink_close(h);
	return ERROR_FAIL;
}

static int stlink_usb_hl_open(struct hl_interface_param_s *param, void **fd)
{
	return stlink_open(param, stlink_get_mode(param->transport), fd);
}

static int stlink_config_trace(void *handle, bool enabled,
		enum tpiu_pin_protocol pin_protocol, uint32_t port_size,
		unsigned int *trace_freq, unsigned int traceclkin_freq,
		uint16_t *prescaler)
{
	struct stlink_usb_handle_s *h = handle;

	if (enabled && (!(h->version.flags & STLINK_F_HAS_TRACE) ||
			pin_protocol != TPIU_PIN_PROTOCOL_ASYNC_UART)) {
		LOG_ERROR("The attached ST-LINK version doesn't support this trace mode");
		return ERROR_FAIL;
	}

	unsigned int max_trace_freq = (h->version.stlink == 3) ?
			STLINK_V3_TRACE_MAX_HZ : STLINK_TRACE_MAX_HZ;

	/* Only concern ourselves with the frequency if the STlink is processing it. */
	if (enabled && *trace_freq > max_trace_freq) {
		LOG_ERROR("ST-LINK doesn't support SWO frequency higher than %u",
			  max_trace_freq);
		return ERROR_FAIL;
	}

	stlink_usb_trace_disable(h);

	if (!*trace_freq)
		*trace_freq = max_trace_freq;

	unsigned int presc = (traceclkin_freq + *trace_freq / 2) / *trace_freq;
	if (presc == 0 || presc > TPIU_ACPR_MAX_SWOSCALER + 1) {
		LOG_ERROR("SWO frequency is not suitable. Please choose a different "
			"frequency.");
		return ERROR_FAIL;
	}

	/* Probe's UART speed must be within 3% of the TPIU's SWO baud rate. */
	unsigned int max_deviation = (traceclkin_freq * 3) / 100;
	if (presc * *trace_freq < traceclkin_freq - max_deviation ||
			presc * *trace_freq > traceclkin_freq + max_deviation) {
		LOG_ERROR("SWO frequency is not suitable. Please choose a different "
			"frequency.");
		return ERROR_FAIL;
	}

	*prescaler = presc;

	if (!enabled)
		return ERROR_OK;

	h->trace.source_hz = *trace_freq;

	return stlink_usb_trace_enable(h);
}

/** */
static int stlink_usb_init_access_port(void *handle, unsigned char ap_num)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_AP_INIT))
		return ERROR_COMMAND_NOTFOUND;

	LOG_DEBUG_IO("init ap_num = %d", ap_num);
	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_INIT_AP;
	h->cmdbuf[h->cmdidx++] = ap_num;

	return stlink_usb_xfer_errcheck(handle, h->databuf, 2);
}

/** */
static int stlink_usb_close_access_port(void *handle, unsigned char ap_num)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_AP_INIT))
		return ERROR_COMMAND_NOTFOUND;

	LOG_DEBUG_IO("close ap_num = %d", ap_num);
	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_CLOSE_AP_DBG;
	h->cmdbuf[h->cmdidx++] = ap_num;

	/* ignore incorrectly returned error on bogus FW */
	if (h->version.flags & STLINK_F_FIX_CLOSE_AP)
		return stlink_usb_xfer_errcheck(handle, h->databuf, 2);
	else
		return stlink_usb_xfer_noerrcheck(handle, h->databuf, 2);

}

/** */
static int stlink_read_dap_register(void *handle, unsigned short dap_port,
			unsigned short addr, uint32_t *val)
{
	struct stlink_usb_handle_s *h = handle;
	int retval;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_DAP_REG))
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READ_DAP_REG;
	h_u16_to_le(&h->cmdbuf[2], dap_port);
	h_u16_to_le(&h->cmdbuf[4], addr);

	retval = stlink_usb_xfer_errcheck(handle, h->databuf, 8);
	*val = le_to_h_u32(h->databuf + 4);
	LOG_DEBUG_IO("dap_port_read = %d, addr =  0x%x, value = 0x%" PRIx32, dap_port, addr, *val);
	return retval;
}

/** */
static int stlink_write_dap_register(void *handle, unsigned short dap_port,
			unsigned short addr, uint32_t val)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (!(h->version.flags & STLINK_F_HAS_DAP_REG))
		return ERROR_COMMAND_NOTFOUND;

	LOG_DEBUG_IO("dap_write port = %d, addr = 0x%x, value = 0x%" PRIx32, dap_port, addr, val);
	stlink_usb_init_buffer(handle, h->rx_ep, 16);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITE_DAP_REG;
	h_u16_to_le(&h->cmdbuf[2], dap_port);
	h_u16_to_le(&h->cmdbuf[4], addr);
	h_u32_to_le(&h->cmdbuf[6], val);
	return stlink_usb_xfer_errcheck(handle, h->databuf, 2);
}

/** */
struct hl_layout_api_s stlink_usb_layout_api = {
	/** */
	.open = stlink_usb_hl_open,
	/** */
	.close = stlink_close,
	/** */
	.idcode = stlink_usb_idcode,
	/** */
	.state = stlink_usb_state,
	/** */
	.reset = stlink_usb_reset,
	/** */
	.assert_srst = stlink_usb_assert_srst,
	/** */
	.run = stlink_usb_run,
	/** */
	.halt = stlink_usb_halt,
	/** */
	.step = stlink_usb_step,
	/** */
	.read_regs = stlink_usb_read_regs,
	/** */
	.read_reg = stlink_usb_read_reg,
	/** */
	.write_reg = stlink_usb_write_reg,
	/** */
	.read_mem = stlink_usb_read_mem,
	/** */
	.write_mem = stlink_usb_write_mem,
	/** */
	.write_debug_reg = stlink_usb_write_debug_reg,
	/** */
	.override_target = stlink_usb_override_target,
	/** */
	.speed = stlink_speed,
	/** */
	.config_trace = stlink_config_trace,
	/** */
	.poll_trace = stlink_usb_trace_read,
};

/*****************************************************************************
 * DAP direct interface
 */

static struct stlink_usb_handle_s *stlink_dap_handle;
static struct hl_interface_param_s stlink_dap_param;
static DECLARE_BITMAP(opened_ap, DP_APSEL_MAX + 1);
static int stlink_dap_error = ERROR_OK;

static int stlink_dap_op_queue_dp_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data);

/** */
static int stlink_dap_record_error(int error)
{
	if (stlink_dap_error == ERROR_OK)
		stlink_dap_error = error;
	return ERROR_OK;
}

/** */
static int stlink_dap_get_and_clear_error(void)
{
	int retval = stlink_dap_error;
	stlink_dap_error = ERROR_OK;
	return retval;
}

static int stlink_usb_open_ap(void *handle, unsigned short apsel)
{
	struct stlink_usb_handle_s *h = handle;
	int retval;

	/* nothing to do on old versions */
	if (!(h->version.flags & STLINK_F_HAS_AP_INIT))
		return ERROR_OK;

	if (apsel > DP_APSEL_MAX)
		return ERROR_FAIL;

	if (test_bit(apsel, opened_ap))
		return ERROR_OK;

	retval = stlink_usb_init_access_port(h, apsel);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("AP %d enabled", apsel);
	set_bit(apsel, opened_ap);
	return ERROR_OK;
}

static int stlink_dap_open_ap(unsigned short apsel)
{
	return stlink_usb_open_ap(stlink_dap_handle, apsel);
}

/** */
static int stlink_dap_closeall_ap(void)
{
	int retval, apsel;

	/* nothing to do on old versions */
	if (!(stlink_dap_handle->version.flags & STLINK_F_HAS_AP_INIT))
		return ERROR_OK;

	for (apsel = 0; apsel <= DP_APSEL_MAX; apsel++) {
		if (!test_bit(apsel, opened_ap))
			continue;
		retval = stlink_usb_close_access_port(stlink_dap_handle, apsel);
		if (retval != ERROR_OK)
			return retval;
		clear_bit(apsel, opened_ap);
	}
	return ERROR_OK;
}

/** */
static int stlink_dap_reinit_interface(void)
{
	int retval;

	/*
	 * On JTAG only, it should be enough to call stlink_usb_reset(). But on
	 * some firmware version it does not work as expected, and there is no
	 * equivalent for SWD.
	 * At least for now, to reset the interface quit from JTAG/SWD mode then
	 * select the mode again.
	 */

	if (!stlink_dap_handle->reconnect_pending) {
		stlink_dap_handle->reconnect_pending = true;
		stlink_usb_mode_leave(stlink_dap_handle, stlink_dap_handle->st_mode);
	}

	retval = stlink_usb_mode_enter(stlink_dap_handle, stlink_dap_handle->st_mode);
	if (retval != ERROR_OK)
		return retval;

	stlink_dap_handle->reconnect_pending = false;
	/* on new FW, calling mode-leave closes all the opened AP; reopen them! */
	if (stlink_dap_handle->version.flags & STLINK_F_HAS_AP_INIT)
		for (int apsel = 0; apsel <= DP_APSEL_MAX; apsel++)
			if (test_bit(apsel, opened_ap)) {
				clear_bit(apsel, opened_ap);
				stlink_dap_open_ap(apsel);
			}
	return ERROR_OK;
}

/** */
static int stlink_dap_op_connect(struct adiv5_dap *dap)
{
	uint32_t idcode;
	int retval;

	LOG_INFO("stlink_dap_op_connect(%sconnect)", dap->do_reconnect ? "re" : "");

	/* Check if we should reset srst already when connecting, but not if reconnecting. */
	if (!dap->do_reconnect) {
		enum reset_types jtag_reset_config = jtag_get_reset_config();

		if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
			if (jtag_reset_config & RESET_SRST_NO_GATING)
				adapter_assert_reset();
			else
				LOG_WARNING("\'srst_nogate\' reset_config option is required");
		}
	}

	dap->do_reconnect = false;
	dap_invalidate_cache(dap);

	retval = dap_dp_init(dap);
	if (retval != ERROR_OK) {
		dap->do_reconnect = true;
		return retval;
	}

	retval = stlink_usb_idcode(stlink_dap_handle, &idcode);
	if (retval == ERROR_OK)
		LOG_INFO("%s %#8.8" PRIx32,
			(stlink_dap_handle->st_mode == STLINK_MODE_DEBUG_JTAG) ? "JTAG IDCODE" : "SWD DPIDR",
			idcode);
	else
		dap->do_reconnect = true;

	return retval;
}

/** */
static int stlink_dap_check_reconnect(struct adiv5_dap *dap)
{
	int retval;

	if (!dap->do_reconnect)
		return ERROR_OK;

	retval = stlink_dap_reinit_interface();
	if (retval != ERROR_OK)
		return retval;

	return stlink_dap_op_connect(dap);
}

/** */
static int stlink_dap_op_send_sequence(struct adiv5_dap *dap, enum swd_special_seq seq)
{
	/* Ignore the request */
	return ERROR_OK;
}

/** */
static int stlink_dap_op_queue_dp_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	uint32_t dummy;
	int retval;

	if (!(stlink_dap_handle->version.flags & STLINK_F_HAS_DPBANKSEL))
		if (reg & 0x000000F0) {
			LOG_ERROR("Banked DP registers not supported in current STLink FW");
			return ERROR_COMMAND_NOTFOUND;
		}

	retval = stlink_dap_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	data = data ? : &dummy;
	if (stlink_dap_handle->version.flags & STLINK_F_QUIRK_JTAG_DP_READ
		&& stlink_dap_handle->st_mode == STLINK_MODE_DEBUG_JTAG) {
		/* Quirk required in JTAG. Read RDBUFF to get the data */
		retval = stlink_read_dap_register(stlink_dap_handle,
					STLINK_DEBUG_PORT_ACCESS, reg, &dummy);
		if (retval == ERROR_OK)
			retval = stlink_read_dap_register(stlink_dap_handle,
						STLINK_DEBUG_PORT_ACCESS, DP_RDBUFF, data);
	} else {
		retval = stlink_read_dap_register(stlink_dap_handle,
					STLINK_DEBUG_PORT_ACCESS, reg, data);
	}

	return stlink_dap_record_error(retval);
}

/** */
static int stlink_dap_op_queue_dp_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	int retval;

	if (!(stlink_dap_handle->version.flags & STLINK_F_HAS_DPBANKSEL))
		if (reg & 0x000000F0) {
			LOG_ERROR("Banked DP registers not supported in current STLink FW");
			return ERROR_COMMAND_NOTFOUND;
		}

	if (reg == DP_SELECT && (data & DP_SELECT_DPBANK) != 0) {
		/* ignored if STLINK_F_HAS_DPBANKSEL, not properly managed otherwise */
		LOG_DEBUG("Ignoring DPBANKSEL while write SELECT");
		data &= ~DP_SELECT_DPBANK;
	}

	retval = stlink_dap_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	/* ST-Link does not like that we set CORUNDETECT */
	if (reg == DP_CTRL_STAT)
		data &= ~CORUNDETECT;

	retval = stlink_write_dap_register(stlink_dap_handle,
				STLINK_DEBUG_PORT_ACCESS, reg, data);
	return stlink_dap_record_error(retval);
}

/** */
static int stlink_dap_op_queue_ap_read(struct adiv5_ap *ap, unsigned reg,
		uint32_t *data)
{
	struct adiv5_dap *dap = ap->dap;
	uint32_t dummy;
	int retval;

	retval = stlink_dap_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	if (reg != AP_REG_IDR) {
		retval = stlink_dap_open_ap(ap->ap_num);
		if (retval != ERROR_OK)
			return retval;
	}
	data = data ? : &dummy;
	retval = stlink_read_dap_register(stlink_dap_handle, ap->ap_num, reg,
				 data);
	dap->stlink_flush_ap_write = false;
	return stlink_dap_record_error(retval);
}

/** */
static int stlink_dap_op_queue_ap_write(struct adiv5_ap *ap, unsigned reg,
		uint32_t data)
{
	struct adiv5_dap *dap = ap->dap;
	int retval;

	retval = stlink_dap_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = stlink_dap_open_ap(ap->ap_num);
	if (retval != ERROR_OK)
		return retval;

	retval = stlink_write_dap_register(stlink_dap_handle, ap->ap_num, reg,
				data);
	dap->stlink_flush_ap_write = true;
	return stlink_dap_record_error(retval);
}

/** */
static int stlink_dap_op_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	LOG_WARNING("stlink_dap_op_queue_ap_abort()");
	return ERROR_OK;
}

/** */
static int stlink_dap_op_run(struct adiv5_dap *dap)
{
	uint32_t ctrlstat, pwrmask;
	int retval, saved_retval;

	/* Here no LOG_DEBUG. This is called continuously! */

	/*
	 * ST-Link returns immediately after a DAP write, without waiting for it
	 * to complete.
	 * Run a dummy read to DP_RDBUFF, as suggested in
	 * http://infocenter.arm.com/help/topic/com.arm.doc.faqs/ka16363.html
	 */
	if (dap->stlink_flush_ap_write) {
		dap->stlink_flush_ap_write = false;
		retval = stlink_dap_op_queue_dp_read(dap, DP_RDBUFF, NULL);
		if (retval != ERROR_OK) {
			dap->do_reconnect = true;
			return retval;
		}
	}

	saved_retval = stlink_dap_get_and_clear_error();

	retval = stlink_dap_op_queue_dp_read(dap, DP_CTRL_STAT, &ctrlstat);
	if (retval != ERROR_OK) {
		dap->do_reconnect = true;
		return retval;
	}
	retval = stlink_dap_get_and_clear_error();
	if (retval != ERROR_OK) {
		LOG_ERROR("Fail reading CTRL/STAT register. Force reconnect");
		dap->do_reconnect = true;
		return retval;
	}

	if (ctrlstat & SSTICKYERR) {
		if (stlink_dap_handle->st_mode == STLINK_MODE_DEBUG_JTAG)
			retval = stlink_dap_op_queue_dp_write(dap, DP_CTRL_STAT,
					ctrlstat & (dap->dp_ctrl_stat | SSTICKYERR));
		else
			retval = stlink_dap_op_queue_dp_write(dap, DP_ABORT, STKERRCLR);
		if (retval != ERROR_OK) {
			dap->do_reconnect = true;
			return retval;
		}
		retval = stlink_dap_get_and_clear_error();
		if (retval != ERROR_OK) {
			dap->do_reconnect = true;
			return retval;
		}
	}

	/* check for power lost */
	pwrmask = dap->dp_ctrl_stat & (CDBGPWRUPREQ | CSYSPWRUPREQ);
	if ((ctrlstat & pwrmask) != pwrmask)
		dap->do_reconnect = true;

	return saved_retval;
}

/** */
static void stlink_dap_op_quit(struct adiv5_dap *dap)
{
	int retval;

	retval = stlink_dap_closeall_ap();
	if (retval != ERROR_OK)
		LOG_ERROR("Error closing APs");
}

static int stlink_swim_op_srst(void)
{
	return stlink_swim_generate_rst(stlink_dap_handle);
}

static int stlink_swim_op_read_mem(uint32_t addr, uint32_t size,
								   uint32_t count, uint8_t *buffer)
{
	int retval;
	uint32_t bytes_remaining;

	LOG_DEBUG_IO("read at 0x%08" PRIx32 " len %" PRIu32 "*0x%08" PRIx32, addr, size, count);
	count *= size;

	while (count) {
		bytes_remaining = (count > STLINK_DATA_SIZE) ? STLINK_DATA_SIZE : count;
		retval = stlink_swim_readbytes(stlink_dap_handle, addr, bytes_remaining, buffer);
		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return ERROR_OK;
}

static int stlink_swim_op_write_mem(uint32_t addr, uint32_t size,
									uint32_t count, const uint8_t *buffer)
{
	int retval;
	uint32_t bytes_remaining;

	LOG_DEBUG_IO("write at 0x%08" PRIx32 " len %" PRIu32 "*0x%08" PRIx32, addr, size, count);
	count *= size;

	while (count) {
		bytes_remaining = (count > STLINK_DATA_SIZE) ? STLINK_DATA_SIZE : count;
		retval = stlink_swim_writebytes(stlink_dap_handle, addr, bytes_remaining, buffer);
		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return ERROR_OK;
}

static int stlink_swim_op_reconnect(void)
{
	int retval;

	retval = stlink_usb_mode_enter(stlink_dap_handle, STLINK_MODE_DEBUG_SWIM);
	if (retval != ERROR_OK)
		return retval;

	return stlink_swim_resync(stlink_dap_handle);
}

static int stlink_dap_config_trace(bool enabled,
		enum tpiu_pin_protocol pin_protocol, uint32_t port_size,
		unsigned int *trace_freq, unsigned int traceclkin_freq,
		uint16_t *prescaler)
{
	return stlink_config_trace(stlink_dap_handle, enabled, pin_protocol,
							   port_size, trace_freq, traceclkin_freq,
							   prescaler);
}

static int stlink_dap_trace_read(uint8_t *buf, size_t *size)
{
	return stlink_usb_trace_read(stlink_dap_handle, buf, size);
}

/** */
COMMAND_HANDLER(stlink_dap_serial_command)
{
	LOG_DEBUG("stlink_dap_serial_command");

	if (CMD_ARGC != 1) {
		LOG_ERROR("Expected exactly one argument for \"st-link serial <serial-number>\".");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (stlink_dap_param.serial) {
		LOG_WARNING("Command \"st-link serial\" already used. Replacing previous value");
		free((void *)stlink_dap_param.serial);
	}

	stlink_dap_param.serial = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

/** */
COMMAND_HANDLER(stlink_dap_vid_pid)
{
	unsigned int i, max_usb_ids = HLA_MAX_USB_IDS;

	if (CMD_ARGC > max_usb_ids * 2) {
		LOG_WARNING("ignoring extra IDs in vid_pid "
			"(maximum is %d pairs)", max_usb_ids);
		CMD_ARGC = max_usb_ids * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete vid_pid configuration directive");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], stlink_dap_param.vid[i / 2]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], stlink_dap_param.pid[i / 2]);
	}

	/* null termination */
	stlink_dap_param.vid[i / 2] = stlink_dap_param.pid[i / 2] = 0;

	return ERROR_OK;
}

/** */
COMMAND_HANDLER(stlink_dap_interface_command)
{
	/* default values */
	bool use_stlink_tcp = false;
	uint16_t stlink_tcp_port = 7184;

	if (CMD_ARGC == 0 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (strcmp(CMD_ARGV[0], "usb") == 0 && CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else if (strcmp(CMD_ARGV[0], "tcp") == 0) {
		use_stlink_tcp = true;
		if (CMD_ARGC == 2)
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], stlink_tcp_port);
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	stlink_dap_param.use_stlink_tcp = use_stlink_tcp;
	stlink_dap_param.stlink_tcp_port = stlink_tcp_port;

	return ERROR_OK;
}

/** */
static const struct command_registration stlink_dap_subcommand_handlers[] = {
	{
		.name = "serial",
		.handler = stlink_dap_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the adapter",
		.usage = "<serial_number>",
	},
	{
		.name = "vid_pid",
		.handler = stlink_dap_vid_pid,
		.mode = COMMAND_CONFIG,
		.help = "USB VID and PID of the adapter",
		.usage = "(vid pid)+",
	},
	{
		.name = "interface",
		.handler = &stlink_dap_interface_command,
		.mode = COMMAND_CONFIG,
		.help = "select which ST-Link interface to use",
		.usage = "usb | tcp [port]",
	},
	COMMAND_REGISTRATION_DONE
};

/** */
static const struct command_registration stlink_dap_command_handlers[] = {
	{
		.name = "st-link",
		.mode = COMMAND_ANY,
		.help = "perform st-link management",
		.chain = stlink_dap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

/** */
static int stlink_dap_init(void)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	enum stlink_mode mode;
	int retval;

	LOG_DEBUG("stlink_dap_init()");

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			stlink_dap_param.connect_under_reset = true;
		else
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
	}

	if (transport_is_dapdirect_swd())
		mode = STLINK_MODE_DEBUG_SWD;
	else if (transport_is_dapdirect_jtag())
		mode = STLINK_MODE_DEBUG_JTAG;
	else if (transport_is_swim())
		mode = STLINK_MODE_DEBUG_SWIM;
	else {
		LOG_ERROR("Unsupported transport");
		return ERROR_FAIL;
	}

	retval = stlink_open(&stlink_dap_param, mode, (void **)&stlink_dap_handle);
	if (retval != ERROR_OK)
		return retval;

	if ((mode != STLINK_MODE_DEBUG_SWIM) &&
		!(stlink_dap_handle->version.flags & STLINK_F_HAS_DAP_REG)) {
		LOG_ERROR("ST-Link version does not support DAP direct transport");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

/** */
static int stlink_dap_quit(void)
{
	LOG_DEBUG("stlink_dap_quit()");

	free((void *)stlink_dap_param.serial);
	stlink_dap_param.serial = NULL;

	return stlink_close(stlink_dap_handle);
}

/** */
static int stlink_dap_reset(int req_trst, int req_srst)
{
	LOG_DEBUG("stlink_dap_reset(%d)", req_srst);
	return stlink_usb_assert_srst(stlink_dap_handle,
		req_srst ? STLINK_DEBUG_APIV2_DRIVE_NRST_LOW
				 : STLINK_DEBUG_APIV2_DRIVE_NRST_HIGH);
}

/** */
static int stlink_dap_speed(int speed)
{
	if (speed == 0) {
		LOG_ERROR("RTCK not supported. Set nonzero adapter_khz.");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	stlink_dap_param.initial_interface_speed = speed;
	stlink_speed(stlink_dap_handle, speed, false);
	return ERROR_OK;
}

/** */
static int stlink_dap_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_ERROR("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = stlink_speed(stlink_dap_handle, khz, true);
	return ERROR_OK;
}

/** */
static int stlink_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static const struct dap_ops stlink_dap_ops = {
	.connect = stlink_dap_op_connect,
	.send_sequence = stlink_dap_op_send_sequence,
	.queue_dp_read = stlink_dap_op_queue_dp_read,
	.queue_dp_write = stlink_dap_op_queue_dp_write,
	.queue_ap_read = stlink_dap_op_queue_ap_read,
	.queue_ap_write = stlink_dap_op_queue_ap_write,
	.queue_ap_abort = stlink_dap_op_queue_ap_abort,
	.run = stlink_dap_op_run,
	.sync = NULL, /* optional */
	.quit = stlink_dap_op_quit, /* optional */
};

static const struct swim_driver stlink_swim_ops = {
	.srst = stlink_swim_op_srst,
	.read_mem = stlink_swim_op_read_mem,
	.write_mem = stlink_swim_op_write_mem,
	.reconnect = stlink_swim_op_reconnect,
};

static const char *const stlink_dap_transport[] = { "dapdirect_swd", "dapdirect_jtag", "swim", NULL };

struct adapter_driver stlink_dap_adapter_driver = {
	.name = "st-link",
	.transports = stlink_dap_transport,
	.commands = stlink_dap_command_handlers,

	.init = stlink_dap_init,
	.quit = stlink_dap_quit,
	.reset = stlink_dap_reset,
	.speed = stlink_dap_speed,
	.khz = stlink_dap_khz,
	.speed_div = stlink_dap_speed_div,
	.config_trace = stlink_dap_config_trace,
	.poll_trace = stlink_dap_trace_read,

	.dap_jtag_ops = &stlink_dap_ops,
	.dap_swd_ops = &stlink_dap_ops,
	.swim_ops = &stlink_swim_ops,
};
