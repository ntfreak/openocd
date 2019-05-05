/***************************************************************************
 *   Copyright (C) 2009 - 2010 by Simon Qian <SimonQian@SimonQian.com>     *
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

#ifndef OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_INTERNAL_H
#define OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_INTERNAL_H

#define VERSALOON_PRODUCTSTRING_INDEX	2
#define VERSALOON_SERIALSTRING_INDEX	3

#define VERSALOON_PRODUCTSTRING			"Versaloon"

#define VERSALOON_VID					0x0483
#define VERSALOON_PID					0xA038
#define VERSALOON_INP					0x82
#define VERSALOON_OUTP					0x03
#define VERSALOON_IFACE					0x00

#define VERSALOON_FULL					1
#define VERSALOON_MINI					2
#define VERSALOON_NANO					3

#define VERSALOON_TIMEOUT				5000
#define VERSALOON_TIMEOUT_LONG			60000

/* USB Commands */
/* Common Commands */
#define VERSALOON_COMMON_CMD_START		0x00
#define VERSALOON_COMMON_CMD_END		0x0F

#define VERSALOON_GET_INFO				0x00
#define VERSALOON_GET_TVCC				0x01
#define VERSALOON_GET_HARDWARE			0x02
#define VERSALOON_GET_OFFLINE_SIZE		0x08
#define VERSALOON_ERASE_OFFLINE_DATA	0x09
#define VERSALOON_WRITE_OFFLINE_DATA	0x0A
#define VERSALOON_GET_OFFLINE_CHECKSUM	0x0B
#define VERSALOON_FW_UPDATE				0x0F
#define VERSALOON_FW_UPDATE_KEY			0xAA

/* MCU Command */
#define VERSALOON_MCU_CMD_START			0x10
#define VERSALOON_MCU_CMD_END			0x1F

/* USB_TO_XXX Command */
#define VERSALOON_USB_TO_XXX_CMD_START	0x20
#define VERSALOON_USB_TO_XXX_CMD_END	0x7F

/* VSLLink Command */
#define VERSALOON_VSLLINK_CMD_START		0x80
#define VERSALOON_VSLLINK_CMD_END		0xFF

/* Mass-product */
#define MP_OK							0x00
#define MP_FAIL							0x01

#define MP_ISSP							0x11

/* pending struct */
#define VERSALOON_MAX_PENDING_NUMBER	4096
typedef RESULT(*versaloon_callback_t)(void *, uint8_t *, uint8_t *);
struct versaloon_want_pos_t {
	uint16_t offset;
	uint16_t size;
	uint8_t *buff;
	struct versaloon_want_pos_t *next;
};
struct versaloon_pending_t {
	uint8_t type;
	uint8_t cmd;
	uint16_t want_data_pos;
	uint16_t want_data_size;
	uint16_t actual_data_size;
	uint8_t *data_buffer;
	uint8_t collect;
	uint32_t id;
	struct versaloon_want_pos_t *pos;
	void *extra_data;
	versaloon_callback_t callback;
};
extern struct versaloon_pending_t
	versaloon_pending[VERSALOON_MAX_PENDING_NUMBER];
extern uint16_t versaloon_pending_idx;
void versaloon_set_pending_id(uint32_t id);
void versaloon_set_callback(versaloon_callback_t callback);
void versaloon_set_extra_data(void *p);
RESULT versaloon_add_want_pos(uint16_t offset, uint16_t size, uint8_t *buff);
RESULT versaloon_add_pending(uint8_t type, uint8_t cmd, uint16_t actual_szie,
		uint16_t want_pos, uint16_t want_size, uint8_t *buffer, uint8_t collect);
void versaloon_free_want_pos(void);

RESULT versaloon_send_command(uint16_t out_len, uint16_t *inlen);
extern uint8_t *versaloon_buf;
extern uint8_t *versaloon_cmd_buf;
extern uint16_t versaloon_buf_size;

#endif /* OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_INTERNAL_H */
