/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian <SimonQian@SimonQian.com>            *
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

#ifndef OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_H
#define OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_H

#include <libusb.h>

struct usart_status_t {
	uint32_t tx_buff_avail;
	uint32_t tx_buff_size;
	uint32_t rx_buff_avail;
	uint32_t rx_buff_size;
};

#include "usbtoxxx/usbtoxxx.h"

/* GPIO pins */
#define GPIO_SRST				(1 << 0)
#define GPIO_TRST				(1 << 1)
#define GPIO_USR1				(1 << 2)
#define GPIO_USR2				(1 << 3)
#define GPIO_TCK				(1 << 4)
#define GPIO_TDO				(1 << 5)
#define GPIO_TDI				(1 << 6)
#define GPIO_RTCK				(1 << 7)
#define GPIO_TMS				(1 << 8)

struct interface_gpio_t {
	RESULT(*init)(uint8_t interface_index);
	RESULT(*fini)(uint8_t interface_index);
	RESULT(*config)(uint8_t interface_index, uint32_t pin_mask, uint32_t io,
			uint32_t pull_en_mask, uint32_t input_pull_mask);
	RESULT(*out)(uint8_t interface_index, uint32_t pin_mask, uint32_t value);
	RESULT(*in)(uint8_t interface_index, uint32_t pin_mask, uint32_t *value);
};

struct interface_delay_t {
	RESULT(*delayms)(uint16_t ms);
	RESULT(*delayus)(uint16_t us);
};

struct interface_swd_t {
	RESULT(*init)(uint8_t interface_index);
	RESULT(*fini)(uint8_t interface_index);
	RESULT(*config)(uint8_t interface_index, uint8_t trn, uint16_t retry,
		uint16_t dly);
	RESULT(*seqout)(uint8_t interface_index, const uint8_t *data,
			uint16_t bitlen);
	RESULT(*seqin)(uint8_t interface_index, uint8_t *data, uint16_t bitlen);
	RESULT(*transact)(uint8_t interface_index, uint8_t request,
		uint32_t *data, uint8_t *ack);
};

struct interface_jtag_raw_t {
	RESULT(*init)(uint8_t interface_index);
	RESULT(*fini)(uint8_t interface_index);
	RESULT(*config)(uint8_t interface_index, uint32_t khz);
	RESULT(*execute)(uint8_t interface_index, uint8_t *tdi, uint8_t *tms,
		uint8_t *tdo, uint32_t bitlen);
};

struct interface_target_voltage_t {
	RESULT(*get)(uint16_t *voltage);
	RESULT(*set)(uint16_t voltage);
};

struct versaloon_adaptors_t {
	struct interface_target_voltage_t target_voltage;
	struct interface_gpio_t gpio;
	struct interface_delay_t delay;
	struct interface_swd_t swd;
	struct interface_jtag_raw_t jtag_raw;
	RESULT(*peripheral_commit)(void);
};

struct versaloon_usb_setting_t {
	uint16_t vid;
	uint16_t pid;
	uint8_t ep_out;
	uint8_t ep_in;
	uint8_t interface;
	char *serialstring;

	uint16_t buf_size;
};

struct versaloon_interface_t {
	RESULT(*init)(void);
	RESULT(*fini)(void);
	struct versaloon_adaptors_t adaptors;
	struct versaloon_usb_setting_t usb_setting;
};

extern struct versaloon_interface_t versaloon_interface;
extern struct libusb_device_handle *versaloon_usb_device_handle;

#endif /* OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_H */
