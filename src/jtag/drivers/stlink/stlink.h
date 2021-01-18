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

#ifndef OPENOCD_JTAG_DRIVERS_STLINK_STLINK_H
#define OPENOCD_JTAG_DRIVERS_STLINK_STLINK_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>

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
struct stlink_version_s {
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
	/** */
	uint8_t *send_buf;
	/** */
	uint8_t *recv_buf;
};

struct stlink_backend_s {
	/** */
	int (*open)(void *handle, struct hl_interface_param_s *param);
	/** */
	int (*close)(void *handle);
	/** */
	int (*xfer_noerrcheck)(void *handle, const uint8_t *buf, int size);
	/** */
	int (*read_trace)(void *handle, const uint8_t *buf, int size);
};

/** */
struct stlink_handle_s {
	/** */
	struct stlink_backend_s *backend;
	/** */
	union {
		struct stlink_usb_priv_s usb_backend_priv;
		struct stlink_tcp_priv_s tcp_backend_priv;
	};
	/** */
	uint8_t rx_ep;
	/** */
	uint8_t tx_ep;
	/** */
	uint8_t trace_ep;
	/** */
	uint8_t *cmdbuf;
	/** */
	uint8_t cmdidx;
	/** */
	uint8_t direction;
	/** */
	uint8_t *databuf;
	/** */
	uint32_t max_mem_packet;
	/** */
	enum stlink_mode st_mode;
	/** */
	struct stlink_version_s version;
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

/** shared functions */
void stlink_init_buffer(void *handle, uint8_t direction, uint32_t size);
int stlink_version(void *handle);
int stlink_exit_mode(void *handle);

#endif /* OPENOCD_JTAG_DRIVERS_STLINK_STLINK_H */
