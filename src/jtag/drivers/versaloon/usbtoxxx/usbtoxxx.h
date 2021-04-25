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

#ifndef OPENOCD_JTAG_DRIVERS_VERSALOON_USBTOXXX_USBTOXXX_H
#define OPENOCD_JTAG_DRIVERS_VERSALOON_USBTOXXX_USBTOXXX_H

RESULT usbtoxxx_init(void);
RESULT usbtoxxx_fini(void);
RESULT usbtoxxx_execute_command(void);

#define USB_TO_XXX_ABILITIES_LEN                        12
extern uint8_t usbtoxxx_abilities[USB_TO_XXX_ABILITIES_LEN];
bool usbtoxxx_interface_supported(uint8_t cmd);

/* USB_TO_INFO */
RESULT usbtoinfo_get_abilities(uint8_t abilities[USB_TO_XXX_ABILITIES_LEN]);

/* USB_TO_DELAY */
RESULT usbtodelay_delay(uint16_t dly);
RESULT usbtodelay_delayms(uint16_t ms);
RESULT usbtodelay_delayus(uint16_t us);

/* USB_TO_USART */
RESULT usbtousart_init(uint8_t interface_index);
RESULT usbtousart_fini(uint8_t interface_index);
RESULT usbtousart_config(uint8_t interface_index, uint32_t baudrate,
			 uint8_t datalength, uint8_t mode);
RESULT usbtousart_send(uint8_t interface_index, uint8_t *buf, uint16_t len);
RESULT usbtousart_receive(uint8_t interface_index, uint8_t *buf, uint16_t len);
RESULT usbtousart_status(uint8_t interface_index,
			 struct usart_status_t *status);

/* USB_TO_SPI */
RESULT usbtospi_init(uint8_t interface_index);
RESULT usbtospi_fini(uint8_t interface_index);
RESULT usbtospi_config(uint8_t interface_index, uint32_t khz, uint8_t mode);
RESULT usbtospi_io(uint8_t interface_index, uint8_t *out, uint8_t *in,
		   uint16_t bytelen);

/* USB_TO_GPIO */
RESULT usbtogpio_init(uint8_t interface_index);
RESULT usbtogpio_fini(uint8_t interface_index);
RESULT usbtogpio_config(uint8_t interface_index, uint32_t mask,
			uint32_t dir_mask, uint32_t pull_en_mask,
			uint32_t input_pull_mask);
RESULT usbtogpio_in(uint8_t interface_index, uint32_t mask, uint32_t *value);
RESULT usbtogpio_out(uint8_t interface_index, uint32_t mask, uint32_t value);

/* USB_TO_ISSP */
RESULT usbtoissp_init(uint8_t interface_index);
RESULT usbtoissp_fini(uint8_t interface_index);
RESULT usbtoissp_enter_program_mode(uint8_t interface_index, uint8_t mode);
RESULT usbtoissp_leave_program_mode(uint8_t interface_index, uint8_t mode);
RESULT usbtoissp_wait_and_poll(uint8_t interface_index);
RESULT usbtoissp_vector(uint8_t interface_index, uint8_t operate, uint8_t addr,
			uint8_t data, uint8_t *buf);

/* USB_TO_LPCICP */
RESULT usbtolpcicp_init(uint8_t interface_index);
RESULT usbtolpcicp_fini(uint8_t interface_index);
RESULT usbtolpcicp_config(uint8_t interface_index);
RESULT usbtolpcicp_enter_program_mode(uint8_t interface_index);
RESULT usbtolpcicp_in(uint8_t interface_index, uint8_t *buff, uint16_t len);
RESULT usbtolpcicp_out(uint8_t interface_index, uint8_t *buff, uint16_t len);
RESULT usbtolpcicp_poll_ready(uint8_t interface_index, uint8_t data,
			      uint8_t *ret, uint8_t setmask, uint8_t clearmask, uint16_t pollcnt);

/* USB_TO_JTAG_LL */
RESULT usbtojtagll_init(uint8_t interface_index);
RESULT usbtojtagll_fini(uint8_t interface_index);
RESULT usbtojtagll_config(uint8_t interface_index, uint32_t khz);
RESULT usbtojtagll_tms(uint8_t interface_index, uint8_t *tms, uint8_t bytelen);
RESULT usbtojtagll_tms_clocks(uint8_t interface_index, uint32_t bytelen,
			      uint8_t tms);
RESULT usbtojtagll_scan(uint8_t interface_index, uint8_t *data,
			uint16_t bitlen, uint8_t tms_before_valid,
			uint8_t tms_before, uint8_t tms_after0,
			uint8_t tms_after1);

/* USB_TO_JTAG_HL */
RESULT usbtojtaghl_init(uint8_t interface_index);
RESULT usbtojtaghl_fini(uint8_t interface_index);
RESULT usbtojtaghl_config(uint8_t interface_index, uint32_t khz, uint8_t ub,
			  uint8_t ua, uint16_t bb, uint16_t ba);
RESULT usbtojtaghl_ir(uint8_t interface_index, uint8_t *ir, uint16_t bitlen,
		      uint8_t idle, uint8_t want_ret);
RESULT usbtojtaghl_dr(uint8_t interface_index, uint8_t *dr, uint16_t bitlen,
		      uint8_t idle, uint8_t want_ret);
RESULT usbtojtaghl_tms(uint8_t interface_index, uint8_t *tms, uint16_t bitlen);
RESULT usbtojtaghl_runtest(uint8_t interface_index, uint32_t cycles);
RESULT usbtojtaghl_register_callback(uint8_t index, jtag_callback_t send_callback,
				     jtag_callback_t receive_callback);

/* USB_TO_JTAG_RAW */
RESULT usbtojtagraw_init(uint8_t interface_index);
RESULT usbtojtagraw_fini(uint8_t interface_index);
RESULT usbtojtagraw_config(uint8_t interface_index, uint32_t khz);
RESULT usbtojtagraw_execute(uint8_t interface_index, uint8_t *tdi,
			    uint8_t *tms, uint8_t *tdo, uint32_t bitlen);

/* USB_TO_C2 */
RESULT usbtoc2_init(uint8_t interface_index);
RESULT usbtoc2_fini(uint8_t interface_index);
RESULT usbtoc2_writeaddr(uint8_t interface_index, uint8_t addr);
RESULT usbtoc2_readaddr(uint8_t interface_index, uint8_t *data);
RESULT usbtoc2_writedata(uint8_t interface_index, uint8_t *buf, uint8_t len);
RESULT usbtoc2_readdata(uint8_t interface_index, uint8_t *buf, uint8_t len);

/* USB_TO_I2C */
RESULT usbtoi2c_init(uint8_t interface_index);
RESULT usbtoi2c_fini(uint8_t interface_index);
RESULT usbtoi2c_config(uint8_t interface_index, uint16_t khz,
		       uint16_t byte_interval, uint16_t max_dly);
RESULT usbtoi2c_read(uint8_t interface_index, uint16_t chip_addr,
		     uint8_t *data, uint16_t data_len, uint8_t stop,
		     bool nacklast);
RESULT usbtoi2c_write(uint8_t interface_index, uint16_t chip_addr,
		      uint8_t *data, uint16_t data_len, uint8_t stop);

/* USB_TO_MSP430_JTAG */
RESULT usbtomsp430jtag_init(uint8_t interface_index);
RESULT usbtomsp430jtag_fini(uint8_t interface_index);
RESULT usbtomsp430jtag_config(uint8_t interface_index, uint8_t has_test);
RESULT usbtomsp430jtag_ir(uint8_t interface_index, uint8_t *ir,
			  uint8_t want_ret);
RESULT usbtomsp430jtag_dr(uint8_t interface_index, uint32_t *dr,
			  uint8_t bitlen, uint8_t want_ret);
RESULT usbtomsp430jtag_tclk(uint8_t interface_index, uint8_t value);
RESULT usbtomsp430jtag_tclk_strobe(uint8_t interface_index, uint16_t cnt);
RESULT usbtomsp430jtag_reset(uint8_t interface_index);
RESULT usbtomsp430jtag_poll(uint8_t interface_index, uint32_t dr,
			    uint32_t mask, uint32_t value, uint8_t len,
			    uint16_t poll_cnt, uint8_t toggle_tclk);

/* USB_TO_MSP430_SBW */
RESULT usbtomsp430sbw_init(uint8_t interface_index);
RESULT usbtomsp430sbw_fini(uint8_t interface_index);
RESULT usbtomsp430sbw_config(uint8_t interface_index, uint8_t has_test);
RESULT usbtomsp430sbw_ir(uint8_t interface_index, uint8_t *ir,
			 uint8_t want_ret);
RESULT usbtomsp430sbw_dr(uint8_t interface_index, uint32_t *dr,
			 uint8_t bitlen, uint8_t want_ret);
RESULT usbtomsp430sbw_tclk(uint8_t interface_index, uint8_t value);
RESULT usbtomsp430sbw_tclk_strobe(uint8_t interface_index, uint16_t cnt);
RESULT usbtomsp430sbw_reset(uint8_t interface_index);
RESULT usbtomsp430sbw_poll(uint8_t interface_index, uint32_t dr, uint32_t mask,
			   uint32_t value, uint8_t len, uint16_t poll_cnt,
			   uint8_t toggle_tclk);

/* USB_TO_POWER */
RESULT usbtopwr_init(uint8_t interface_index);
RESULT usbtopwr_fini(uint8_t interface_index);
RESULT usbtopwr_config(uint8_t interface_index);
RESULT usbtopwr_output(uint8_t interface_index, uint16_t millivolt);

/* USB_TO_POLL */
RESULT usbtopoll_start(uint16_t retry_cnt, uint16_t interval_us);
RESULT usbtopoll_end(void);
RESULT usbtopoll_checkok(uint8_t equ, uint16_t offset, uint8_t size,
			 uint32_t mask, uint32_t value);
RESULT usbtopoll_checkfail(uint8_t equ, uint16_t offset, uint8_t size,
			   uint32_t mask, uint32_t value);
RESULT usbtopoll_verifybuff(uint16_t offset, uint16_t size, uint8_t *buff);

/* USB_TO_SWD */
RESULT usbtoswd_init(uint8_t interface_index);
RESULT usbtoswd_fini(uint8_t interface_index);
RESULT usbtoswd_config(uint8_t interface_index, uint8_t trn, uint16_t retry,
		       uint16_t dly);
RESULT usbtoswd_seqout(uint8_t interface_index, const uint8_t *data,
		       uint16_t bitlen);
RESULT usbtoswd_seqin(uint8_t interface_index, uint8_t *data, uint16_t bitlen);
RESULT usbtoswd_transact(uint8_t interface_index, uint8_t request,
			 uint32_t *data, uint8_t *ack);

/* USB_TO_SWIM */
RESULT usbtoswim_init(uint8_t interface_index);
RESULT usbtoswim_fini(uint8_t interface_index);
RESULT usbtoswim_config(uint8_t interface_index, uint8_t mhz, uint8_t cnt0,
			uint8_t cnt1);
RESULT usbtoswim_srst(uint8_t interface_index);
RESULT usbtoswim_wotf(uint8_t interface_index, uint8_t *data,
		      uint16_t bytelen, uint32_t addr);
RESULT usbtoswim_rotf(uint8_t interface_index, uint8_t *data,
		      uint16_t bytelen, uint32_t addr);
RESULT usbtoswim_sync(uint8_t interface_index, uint8_t mhz);
RESULT usbtoswim_enable(uint8_t interface_index);

/* USB_TO_BDM */
RESULT usbtobdm_init(uint8_t interface_index);
RESULT usbtobdm_fini(uint8_t interface_index);
RESULT usbtobdm_sync(uint8_t interface_index, uint16_t *khz);
RESULT usbtobdm_transact(uint8_t interface_index, uint8_t *out,
			 uint8_t outlen, uint8_t *in, uint8_t inlen, uint8_t delay, uint8_t ack);

/* USB_TO_DUSI */
RESULT usbtodusi_init(uint8_t interface_index);
RESULT usbtodusi_fini(uint8_t interface_index);
RESULT usbtodusi_config(uint8_t interface_index, uint32_t khz, uint8_t mode);
RESULT usbtodusi_io(uint8_t interface_index, uint8_t *mo, uint8_t *mi,
		    uint8_t *so, uint8_t *si, uint32_t bitlen);

/* USB_TO_MICROWIRE */
RESULT usbtomicrowire_init(uint8_t interface_index);
RESULT usbtomicrowire_fini(uint8_t interface_index);
RESULT usbtomicrowire_config(uint8_t interface_index, uint16_t khz,
			     uint8_t sel_polarity);
RESULT usbtomicrowire_transport(uint8_t interface_index,
				uint32_t opcode, uint8_t opcode_bitlen,
				uint32_t addr, uint8_t addr_bitlen,
				uint32_t data, uint8_t data_bitlen,
				uint8_t *reply, uint8_t reply_bitlen);
RESULT usbtomicrowire_poll(uint8_t interface_index, uint16_t interval_us,
			   uint16_t retry_cnt);

/* USB_TO_PWM */
RESULT usbtopwm_init(uint8_t interface_index);
RESULT usbtopwm_fini(uint8_t interface_index);
RESULT usbtopwm_config(uint8_t interface_index, uint16_t khz, uint8_t mode);
RESULT usbtopwm_out(uint8_t interface_index, uint16_t count, uint16_t *rate);
RESULT usbtopwm_in(uint8_t interface_index, uint16_t count, uint16_t *rate);

#endif /* OPENOCD_JTAG_DRIVERS_VERSALOON_USBTOXXX_USBTOXXX_H */
