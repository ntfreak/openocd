/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#ifndef OPENOCD_JTAG_AICE_AICE_USB_H
#define OPENOCD_JTAG_AICE_AICE_USB_H

#include "aice_port.h"

/* AICE USB timeout value */
#define AICE_USB_TIMEOUT				5000

/* AICE USB buffer size */
#define AICE_IN_BUFFER_SIZE				2048
#define AICE_OUT_BUFFER_SIZE			2048
#define AICE_IN_PACKETS_BUFFER_SIZE		2048
#define AICE_OUT_PACKETS_BUFFER_SIZE	2048
#define AICE_IN_BATCH_COMMAND_SIZE		512
#define AICE_OUT_BATCH_COMMAND_SIZE		512
#define AICE_IN_PACK_COMMAND_SIZE		2048
#define AICE_OUT_PACK_COMMAND_SIZE		2048

/* Constants for AICE command READ_CTRL */
#define AICE_READ_CTRL_GET_ICE_STATE		0x00
#define AICE_READ_CTRL_GET_HARDWARE_VERSION	0x01
#define AICE_READ_CTRL_GET_FPGA_VERSION		0x02
#define AICE_READ_CTRL_GET_FIRMWARE_VERSION	0x03
#define AICE_READ_CTRL_GET_JTAG_PIN_STATUS	0x04
#define AICE_READ_CTRL_BATCH_BUF_INFO		0x22
#define AICE_READ_CTRL_BATCH_STATUS			0x23
#define AICE_READ_CTRL_BATCH_BUF0_STATE		0x31
#define AICE_READ_CTRL_BATCH_BUF4_STATE		0x39
#define AICE_READ_CTRL_BATCH_BUF5_STATE		0x3b

/* Constants for AICE command WRITE_CTRL */
#define AICE_WRITE_CTRL_TCK_CONTROL				0x00
#define AICE_WRITE_CTRL_JTAG_PIN_CONTROL		0x01
#define AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS	0x02
#define AICE_WRITE_CTRL_RESERVED				0x03
#define AICE_WRITE_CTRL_JTAG_PIN_STATUS			0x04
#define AICE_WRITE_CTRL_CUSTOM_DELAY			0x0d
#define AICE_WRITE_CTRL_BATCH_CTRL				0x20
#define AICE_WRITE_CTRL_BATCH_ITERATION			0x21
#define AICE_WRITE_CTRL_BATCH_DIM_SIZE			0x22
#define AICE_WRITE_CTRL_BATCH_CMD_BUF0_CTRL		0x30
#define AICE_WRITE_CTRL_BATCH_DATA_BUF0_CTRL	0x38
#define AICE_WRITE_CTRL_BATCH_DATA_BUF1_CTRL	0x3a

#define AICE_BATCH_COMMAND_BUFFER_0			0x0
#define AICE_BATCH_COMMAND_BUFFER_1			0x1
#define AICE_BATCH_COMMAND_BUFFER_2			0x2
#define AICE_BATCH_COMMAND_BUFFER_3			0x3
#define AICE_BATCH_DATA_BUFFER_0			0x4
#define AICE_BATCH_DATA_BUFFER_1			0x5
#define AICE_BATCH_DATA_BUFFER_2			0x6
#define AICE_BATCH_DATA_BUFFER_3			0x7

/* Constants for AICE command WRITE_CTRL:TCK_CONTROL */
#define AICE_TCK_CONTROL_TCK3048		0x08
#define AICE_TCK_CONTROL_TCK_SCAN		0x10

/* Constants for AICE command WRITE_CTRL:JTAG_PIN_CONTROL */
#define AICE_JTAG_PIN_CONTROL_SRST		0x01
#define AICE_JTAG_PIN_CONTROL_TRST		0x02
#define AICE_JTAG_PIN_CONTROL_STOP		0x04
#define AICE_JTAG_PIN_CONTROL_RESTART	0x08

/* Constants for AICE command WRITE_CTRL:TCK_CONTROL */
#define AICE_TCK_CONTROL_TCK_SCAN		0x10

/* Custom SRST/DBGI/TRST */
#define AICE_CUSTOM_DELAY_SET_SRST		0x01
#define AICE_CUSTOM_DELAY_CLEAN_SRST	0x02
#define AICE_CUSTOM_DELAY_SET_DBGI		0x04
#define AICE_CUSTOM_DELAY_CLEAN_DBGI	0x08
#define AICE_CUSTOM_DELAY_SET_TRST		0x10
#define AICE_CUSTOM_DELAY_CLEAN_TRST	0x20

struct aice_usb_handler_s {
	unsigned int usb_read_ep;
	unsigned int usb_write_ep;
	struct libusb_device_handle *usb_handle;
};

struct cache_info {
	uint32_t set;
	uint32_t way;
	uint32_t line_size;

	uint32_t log2_set;
	uint32_t log2_line_size;
};

struct aice_nds32_info {
	uint32_t edm_version;
	uint32_t r0_backup;
	uint32_t r1_backup;
	uint32_t host_dtr_backup;
	uint32_t target_dtr_backup;
	uint32_t edmsw_backup;
	uint32_t edm_ctl_backup;
	bool debug_under_dex_on;
	bool dex_use_psw_on;
	bool host_dtr_valid;
	bool target_dtr_valid;
	enum nds_memory_access access_channel;
	enum nds_memory_select memory_select;
	enum aice_target_state_s core_state;
	bool cache_init;
	struct cache_info icache;
	struct cache_info dcache;
};

extern struct aice_port_api_s aice_usb_api;

int aice_read_ctrl(uint32_t address, uint32_t *data);
int aice_write_ctrl(uint32_t address, uint32_t data);

#endif /* OPENOCD_JTAG_AICE_AICE_USB_H */
