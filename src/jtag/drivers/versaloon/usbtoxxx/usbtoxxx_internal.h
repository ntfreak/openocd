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

#ifndef OPENOCD_JTAG_DRIVERS_VERSALOON_USBTOXXX_USBTOXXX_INTERNAL_H
#define OPENOCD_JTAG_DRIVERS_VERSALOON_USBTOXXX_USBTOXXX_INTERNAL_H

/* USB_TO_XXX USB Commands */
/* Page0 */
#define USB_TO_USART                            (VERSALOON_USB_TO_XXX_CMD_START + 0x00)
#define USB_TO_SPI                              (VERSALOON_USB_TO_XXX_CMD_START + 0x01)
#define USB_TO_I2C                              (VERSALOON_USB_TO_XXX_CMD_START + 0x02)
#define USB_TO_GPIO                             (VERSALOON_USB_TO_XXX_CMD_START + 0x03)
#define USB_TO_CAN                              (VERSALOON_USB_TO_XXX_CMD_START + 0x04)
#define USB_TO_PWM                              (VERSALOON_USB_TO_XXX_CMD_START + 0x05)
#define USB_TO_ADC                              (VERSALOON_USB_TO_XXX_CMD_START + 0x06)
#define USB_TO_DAC                              (VERSALOON_USB_TO_XXX_CMD_START + 0x07)
#define USB_TO_MICROWIRE                        (VERSALOON_USB_TO_XXX_CMD_START + 0x08)
#define USB_TO_SWIM                             (VERSALOON_USB_TO_XXX_CMD_START + 0x09)
#define USB_TO_DUSI                             (VERSALOON_USB_TO_XXX_CMD_START + 0x0A)
/* Page1 */
#define USB_TO_JTAG_LL                          (VERSALOON_USB_TO_XXX_CMD_START + 0x20)
#define USB_TO_JTAG_HL                          (VERSALOON_USB_TO_XXX_CMD_START + 0x21)
#define USB_TO_ISSP                             (VERSALOON_USB_TO_XXX_CMD_START + 0x22)
#define USB_TO_C2                               (VERSALOON_USB_TO_XXX_CMD_START + 0x23)
#define USB_TO_SBW                              (VERSALOON_USB_TO_XXX_CMD_START + 0x24)
#define USB_TO_LPCICP                           (VERSALOON_USB_TO_XXX_CMD_START + 0x25)
#define USB_TO_SWD                              (VERSALOON_USB_TO_XXX_CMD_START + 0x26)
#define USB_TO_JTAG_RAW                         (VERSALOON_USB_TO_XXX_CMD_START + 0x27)
#define USB_TO_BDM                              (VERSALOON_USB_TO_XXX_CMD_START + 0x28)
#define USB_TO_MSP430_JTAG                      (VERSALOON_USB_TO_XXX_CMD_START + 0x38)
/* Page2 */
#define USB_TO_POWER                            (VERSALOON_USB_TO_XXX_CMD_START + 0x40)
#define USB_TO_DELAY                            (VERSALOON_USB_TO_XXX_CMD_START + 0x41)
#define USB_TO_POLL                             (VERSALOON_USB_TO_XXX_CMD_START + 0x42)
#define USB_TO_INFO                             (VERSALOON_USB_TO_XXX_CMD_START + 0x5E)
#define USB_TO_ALL                              (VERSALOON_USB_TO_XXX_CMD_START + 0x5F)

/* USB_TO_XXX Masks */
#define USB_TO_XXX_CMDMASK                      0xF8
#define USB_TO_XXX_CMDSHIFT                     3
#define USB_TO_XXX_IDXMASK                      0x07
/* USB_TO_XXX Sub Commands */
/* Common Sub Commands */
#define USB_TO_XXX_INIT                         (0x00 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_FINI                         (0x01 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_CONFIG                       (0x02 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_GETHWINFO                    (0x03 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_STATUS                       (0X04 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_IN_OUT                       (0x05 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_IN                           (0x06 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_OUT                          (0x07 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_POLL                         (0x08 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_SPECIAL                      (0x09 << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_RESET                        (0x0A << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_SYNC                         (0x0B << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_ENABLE                       (0x0C << USB_TO_XXX_CMDSHIFT)
#define USB_TO_XXX_DISABLE                      (0x0D << USB_TO_XXX_CMDSHIFT)
/* USB_TO_POLL */
#define USB_TO_POLL_START                       0x00
#define USB_TO_POLL_END                         0x01
#define USB_TO_POLL_CHECKOK                     0x02
#define USB_TO_POLL_CHECKFAIL                   0x03
#define USB_TO_POLL_VERIFYBUFF                  0x04

/* USB_TO_XXX Replies */
#define USB_TO_XXX_OK                           0x00
#define USB_TO_XXX_FAILED                       0x01
#define USB_TO_XXX_TIME_OUT                     0x02
#define USB_TO_XXX_INVALID_INDEX                0x03
#define USB_TO_XXX_INVALID_PARA                 0x04
#define USB_TO_XXX_INVALID_CMD                  0x05
#define USB_TO_XXX_CMD_NOT_SUPPORT              0x06

/* USB_TO_XXX */
RESULT usbtoxxx_add_pending(uint8_t type, uint8_t cmd, uint16_t
		actual_szie, uint16_t want_pos,
		uint16_t want_size, uint8_t *buffer);

RESULT usbtoxxx_add_command(uint8_t type, uint8_t cmd, uint8_t *cmdbuf,
		uint16_t cmdlen, uint16_t retlen,
		uint8_t *wantbuf, uint16_t wantpos,
		uint16_t wantlen, uint8_t collect);

#define usbtoxxx_init_command(type, port)							\
	usbtoxxx_add_command((type), (USB_TO_XXX_INIT | (port)), \
	NULL, 0, 0, NULL, 0, 0, 0)
#define usbtoxxx_fini_command(type, port)									\
	usbtoxxx_add_command((type), (USB_TO_XXX_FINI | (port)), \
	NULL, 0, 0, NULL, 0, 0, 0)
#define usbtoxxx_conf_command(type, port, cmdbuf, cmdlen)					\
	usbtoxxx_add_command((type), (USB_TO_XXX_CONFIG | (port)), \
	(cmdbuf), (cmdlen), 0, NULL, 0, 0, 0)
#define usbtoxxx_inout_command(type, port, cmdbuf, cmdlen, retlen, wantbuf, \
	wantpos, wantlen, c)						    \
	usbtoxxx_add_command((type), (USB_TO_XXX_IN_OUT | (port)), \
	(cmdbuf), (cmdlen), (retlen), (wantbuf), \
	(wantpos), (wantlen), (c))
#define usbtoxxx_in_command(type, port, cmdbuf, cmdlen, retlen, wantbuf, \
		wantpos, wantlen, c)						    \
	usbtoxxx_add_command((type), (USB_TO_XXX_IN | (port)), (cmdbuf), \
	(cmdlen), (retlen), (wantbuf), (wantpos), \
	(wantlen), (c))
#define usbtoxxx_out_command(type, port, cmdbuf, cmdlen, c)					\
	usbtoxxx_add_command((type), (USB_TO_XXX_OUT | (port)), (cmdbuf), \
	(cmdlen), 0, NULL, 0, 0, (c))
#define usbtoxxx_poll_command(type, port, cmdbuf, cmdlen, retbuf, retlen)	\
	usbtoxxx_add_command((type), (USB_TO_XXX_POLL | (port)), (cmdbuf), \
	(cmdlen), (retlen), (retbuf), 0, (retlen), 0)
#define usbtoxxx_status_command(type, port, retlen, wantbuf, wantpos, wantlen, c) \
	usbtoxxx_add_command((type), (USB_TO_XXX_STATUS | (port)), \
	NULL, 0, (retlen), (wantbuf), (wantpos), \
	(wantlen), (c))
#define usbtoxxx_special_command(type, port, cmdbuf, cmdlen, retlen, wantbuf, \
	wantpos, wantlen, c)						\
	usbtoxxx_add_command((type), (USB_TO_XXX_SPECIAL | (port)), \
	(cmdbuf), (cmdlen), retlen, wantbuf, \
	wantpos, wantlen, (c))
#define usbtoxxx_reset_command(type, port, cmdbuf, cmdlen)					\
	usbtoxxx_add_command((type), (USB_TO_XXX_RESET | (port)), \
	(cmdbuf), (cmdlen), 0, NULL, 0, 0, 0)
#define usbtoxxx_sync_command(type, port, cmdbuf, cmdlen, retlen, wantbuf)	\
	usbtoxxx_add_command((type), (USB_TO_XXX_SYNC | (port)), \
	(cmdbuf), (cmdlen), (retlen), (wantbuf), 0, \
	(retlen), 0)
#define usbtoxxx_enable_command(type, port, cmdbuf, cmdlen)					\
	usbtoxxx_add_command((type), (USB_TO_XXX_ENABLE | (port)), \
	(cmdbuf), (cmdlen), 0, NULL, 0, 0, 0)
#define usbtoxxx_disable_command(type, port, cmdbuf, cmdlen)				\
	usbtoxxx_add_command((type), (USB_TO_XXX_DISABLE | (port)), \
	(cmdbuf), (cmdlen), 0, NULL, 0, 0, 0)

/* USB_TO_SPI */
#define USB_TO_SPI_BAUDRATE_MSK		0x1F
#define USB_TO_SPI_CPOL_MSK			0x20
#define USB_TO_SPI_CPHA_MSK			0x40
#define USB_TO_SPI_MSB_FIRST		0x80

/* USB_TO_DUSI */
#define USB_TO_DUSI_BAUDRATE_MSK	0x1F
#define USB_TO_DUSI_CPOL_MSK		0x20
#define USB_TO_DUSI_CPHA_MSK		0x40
#define USB_TO_DUSI_MSB_FIRST		0x80

/* USB_TO_GPIO */
#define USB_TO_GPIO_DIR_MSK			0x01

#endif /* OPENOCD_JTAG_DRIVERS_VERSALOON_USBTOXXX_USBTOXXX_INTERNAL_H */
