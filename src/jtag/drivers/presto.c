/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
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

/**
 * @file
 * Holds driver for PRESTO programmer from ASIX.
 * http://tools.asix.net/prg_presto.htm
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#endif

#include <jtag/interface.h>
#include <helper/time_support.h>
#include "bitq.h"

/* PRESTO access library includes */
#include "libftdi_helper.h"

/* -------------------------------------------------------------------------- */

#define FT_DEVICE_NAME_LEN 64
#define FT_DEVICE_SERNUM_LEN 64

#define PRESTO_VID_PID 0x0403f1a0
#define PRESTO_VID (0x0403)
#define PRESTO_PID (0xf1a0)

#define BUFFER_SIZE (64*62)

struct presto {
	struct ftdi_context ftdic;
	int retval;

	char serial[FT_DEVICE_SERNUM_LEN];

	uint8_t buff_out[BUFFER_SIZE];
	int buff_out_pos;

	uint8_t buff_in[BUFFER_SIZE];
	int buff_in_exp;/* expected in buffer length */
	int buff_in_len;/* length of data received */
	int buff_in_pos;

	unsigned long total_out;
	unsigned long total_in;

	int jtag_tms;	/* last tms state */
	int jtag_tck;	/* last tck state */
	int jtag_rst;	/* last trst state */

	int jtag_tdi_data;
	int jtag_tdi_count;

	int jtag_speed;
};

static struct presto presto_state;
static struct presto *presto = &presto_state;

static uint8_t presto_init_seq[] = {
	0x80, 0xA0, 0xA8, 0xB0, 0xC0, 0xE0
};

static int presto_write(uint8_t *buf, uint32_t size)
{
	uint32_t ftbytes;
	presto->retval = ftdi_write_data(&presto->ftdic, buf, size);
	if (presto->retval < 0) {
		LOG_ERROR("ftdi_write_data: %s", ftdi_get_error_string(&presto->ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}
	ftbytes = presto->retval;

	if (ftbytes != size) {
		LOG_ERROR("couldn't write the requested number of bytes to PRESTO (%u < %u)",
			(unsigned)ftbytes, (unsigned)size);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int presto_read(uint8_t *buf, uint32_t size)
{
	uint32_t ftbytes = 0;

	struct timeval timeout, now;
	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, 1, 0);	/* one second timeout */

	while (ftbytes < size) {
		presto->retval = ftdi_read_data(&presto->ftdic, buf + ftbytes, size - ftbytes);
		if (presto->retval < 0) {
			LOG_ERROR("ftdi_read_data: %s", ftdi_get_error_string(&presto->ftdic));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		ftbytes += presto->retval;

		gettimeofday(&now, NULL);
		if (timeval_compare(&now, &timeout) > 0)
			break;
	}

	if (ftbytes != size) {
		/* this is just a warning, there might have been timeout when detecting PRESTO,
		 *which is not fatal */
		LOG_WARNING("couldn't read the requested number of bytes from PRESTO (%u < %u)",
			(unsigned)ftbytes, (unsigned)size);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int presto_open_libftdi(char *req_serial)
{
	uint8_t presto_data;

	LOG_DEBUG("searching for PRESTO using libftdi");

	/* initialize FTDI context structure */
	if (ftdi_init(&presto->ftdic) < 0) {
		LOG_ERROR("unable to init libftdi: %s", presto->ftdic.error_str);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* context, vendor id, product id */
	if (ftdi_usb_open_desc(&presto->ftdic, PRESTO_VID, PRESTO_PID, NULL, req_serial) < 0) {
		LOG_ERROR("unable to open PRESTO: %s", presto->ftdic.error_str);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (ftdi_usb_reset(&presto->ftdic) < 0) {
		LOG_ERROR("unable to reset PRESTO device");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (ftdi_set_latency_timer(&presto->ftdic, 1) < 0) {
		LOG_ERROR("unable to set latency timer");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (ftdi_tcioflush(&presto->ftdic) < 0) {
		LOG_ERROR("unable to flush PRESTO buffers");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	presto_data = 0xD0;
	if (presto_write(&presto_data, 1) != ERROR_OK) {
		LOG_ERROR("error writing to PRESTO");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (presto_read(&presto_data, 1) != ERROR_OK) {
		LOG_DEBUG("no response from PRESTO, retrying");

		if (ftdi_tcioflush(&presto->ftdic) < 0)
			return ERROR_JTAG_DEVICE_ERROR;

		presto_data = 0xD0;
		if (presto_write(&presto_data, 1) != ERROR_OK)
			return ERROR_JTAG_DEVICE_ERROR;

		if (presto_read(&presto_data, 1) != ERROR_OK) {
			LOG_ERROR("no response from PRESTO, giving up");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	if (presto_write(presto_init_seq, sizeof(presto_init_seq)) != ERROR_OK) {
		LOG_ERROR("error writing PRESTO init sequence");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int presto_open(char *req_serial)
{
	presto->buff_out_pos = 0;
	presto->buff_in_pos = 0;
	presto->buff_in_len = 0;
	presto->buff_in_exp = 0;

	presto->total_out = 0;
	presto->total_in = 0;

	presto->jtag_tms = 0;
	presto->jtag_tck = 0;
	presto->jtag_rst = 0;
	presto->jtag_tdi_data = 0;
	presto->jtag_tdi_count = 0;

	presto->jtag_speed = 0;

	return presto_open_libftdi(req_serial);
}

static int presto_close(void)
{

	int result = ERROR_OK;

	presto->retval = ftdi_write_data(&presto->ftdic, presto_init_seq, sizeof(presto_init_seq));
	if (presto->retval != sizeof(presto_init_seq))
		result = ERROR_JTAG_DEVICE_ERROR;

	presto->retval = ftdi_set_latency_timer(&presto->ftdic, 16);
	if (presto->retval < 0)
		result = ERROR_JTAG_DEVICE_ERROR;

	presto->retval = ftdi_usb_close(&presto->ftdic);
	if (presto->retval < 0)
		result = ERROR_JTAG_DEVICE_ERROR;
	else
		ftdi_deinit(&presto->ftdic);

	return result;
}

static int presto_flush(void)
{
	if (presto->buff_out_pos == 0)
		return ERROR_OK;

	if (presto->retval < 0) {
		LOG_DEBUG("error in previous communication, canceling I/O operation");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (presto_write(presto->buff_out, presto->buff_out_pos) != ERROR_OK) {
		presto->buff_out_pos = 0;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	presto->total_out += presto->buff_out_pos;
	presto->buff_out_pos = 0;

	if (presto->buff_in_exp == 0)
		return ERROR_OK;

	presto->buff_in_pos = 0;
	presto->buff_in_len = 0;

	if (presto_read(presto->buff_in, presto->buff_in_exp) != ERROR_OK) {
		presto->buff_in_exp = 0;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	presto->total_in += presto->buff_in_exp;
	presto->buff_in_len = presto->buff_in_exp;
	presto->buff_in_exp = 0;

	return ERROR_OK;
}

static int presto_sendbyte(int data)
{
	if (data == EOF)
		return presto_flush();

	if (presto->buff_out_pos < BUFFER_SIZE) {
		presto->buff_out[presto->buff_out_pos++] = (uint8_t)data;
		if (((data & 0xC0) == 0x40) || ((data & 0xD0) == 0xD0))
			presto->buff_in_exp++;
	} else
		return ERROR_JTAG_DEVICE_ERROR;

	/* libftdi does not do background read, be sure that USB IN buffer does not overflow (128
	 *bytes only!) */
	if (presto->buff_out_pos >= BUFFER_SIZE || presto->buff_in_exp == 128)
		return presto_flush();

	return ERROR_OK;
}

#if 0
static int presto_getbyte(void)
{
	if (presto->buff_in_pos < presto->buff_in_len)
		return presto->buff_in[presto->buff_in_pos++];

	if (presto->buff_in_exp == 0)
		return -1;

	if (presto_flush() != ERROR_OK)
		return -1;

	if (presto->buff_in_pos < presto->buff_in_len)
		return presto->buff_in[presto->buff_in_pos++];

	return -1;
}
#endif

/* -------------------------------------------------------------------------- */

static int presto_tdi_flush(void)
{
	if (presto->jtag_tdi_count == 0)
		return 0;

	if (presto->jtag_tck == 0) {
		LOG_ERROR("BUG: unexpected TAP condition, TCK low");
		return -1;
	}

	presto->jtag_tdi_data |= (presto->jtag_tdi_count - 1) << 4;
	presto_sendbyte(presto->jtag_tdi_data);
	presto->jtag_tdi_count = 0;
	presto->jtag_tdi_data = 0;

	return 0;
}

static int presto_tck_idle(void)
{
	if (presto->jtag_tck == 1) {
		presto_sendbyte(0xCA);
		presto->jtag_tck = 0;
	}

	return 0;
}

/* -------------------------------------------------------------------------- */

static int presto_bitq_out(int tms, int tdi, int tdo_req)
{
	int i;
	unsigned char cmd;

	if (presto->jtag_tck == 0)
		presto_sendbyte(0xA4);	/* LED indicator - JTAG active */
	else if (presto->jtag_speed == 0 && !tdo_req && tms == presto->jtag_tms) {
		presto->jtag_tdi_data |= (tdi != 0) << presto->jtag_tdi_count;

		if (++presto->jtag_tdi_count == 4)
			presto_tdi_flush();

		return 0;
	}

	presto_tdi_flush();

	cmd = tdi ? 0xCB : 0xCA;
	presto_sendbyte(cmd);

	if (tms != presto->jtag_tms) {
		presto_sendbyte((tms ? 0xEC : 0xE8) | (presto->jtag_rst ? 0x02 : 0));
		presto->jtag_tms = tms;
	}

	/* delay with TCK low */
	for (i = presto->jtag_speed; i > 1; i--)
		presto_sendbyte(cmd);

	cmd |= 0x04;
	presto_sendbyte(cmd | (tdo_req ? 0x10 : 0));

	/* delay with TCK high */
	for (i = presto->jtag_speed; i > 1; i--)
		presto_sendbyte(cmd);

	presto->jtag_tck = 1;

	return 0;
}

static int presto_bitq_flush(void)
{
	presto_tdi_flush();
	presto_tck_idle();

	presto_sendbyte(0xA0);	/* LED indicator - JTAG idle */

	return presto_flush();
}

static int presto_bitq_in_rdy(void)
{
	if (presto->buff_in_pos >= presto->buff_in_len)
		return 0;
	return presto->buff_in_len-presto->buff_in_pos;
}

static int presto_bitq_in(void)
{
	if (presto->buff_in_pos >= presto->buff_in_len)
		return -1;
	if (presto->buff_in[presto->buff_in_pos++]&0x08)
		return 1;
	return 0;
}

static int presto_bitq_sleep(unsigned long us)
{
	long waits;

	presto_tdi_flush();
	presto_tck_idle();

	if (us > 100000) {
		presto_bitq_flush();
		jtag_sleep(us);
		return 0;
	}

	waits = us / 170 + 2;
	while (waits--)
		presto_sendbyte(0x80);

	return 0;
}

static int presto_bitq_reset(int trst, int srst)
{
	presto_tdi_flush();
	presto_tck_idle();

	/* add a delay after possible TCK transition */
	presto_sendbyte(0x80);
	presto_sendbyte(0x80);

	presto->jtag_rst = trst || srst;
	presto_sendbyte((presto->jtag_rst ? 0xEA : 0xE8) | (presto->jtag_tms ? 0x04 : 0));

	return 0;
}

static struct bitq_interface presto_bitq = {
	.out = &presto_bitq_out,
	.flush = &presto_bitq_flush,
	.sleep = &presto_bitq_sleep,
	.reset = &presto_bitq_reset,
	.in_rdy = &presto_bitq_in_rdy,
	.in = &presto_bitq_in,
};

/* -------------------------------------------------------------------------- */

static int presto_adapter_khz(int khz, int *jtag_speed)
{
	if (khz < 0) {
		*jtag_speed = 0;
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (khz >= 3000)
		*jtag_speed = 0;
	else
		*jtag_speed = (1000 + khz-1)/khz;

	return 0;
}

static int presto_jtag_speed_div(int speed, int *khz)
{
	if ((speed < 0) || (speed > 1000)) {
		*khz = 0;
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (speed == 0)
		*khz = 3000;
	else
		*khz = 1000/speed;

	return 0;
}

static int presto_jtag_speed(int speed)
{
	int khz;

	if (presto_jtag_speed_div(speed, &khz))
		return ERROR_COMMAND_SYNTAX_ERROR;

	presto->jtag_speed = speed;

	if (khz%1000 == 0)
		LOG_INFO("setting speed to %d, max. TCK freq. is %d MHz", speed, khz/1000);
	else
		LOG_INFO("setting speed to %d, max. TCK freq. is %d kHz", speed, khz);

	return 0;
}

static char *presto_serial;

COMMAND_HANDLER(presto_handle_serial_command)
{
	if (CMD_ARGC == 1) {
		free(presto_serial);
		presto_serial = strdup(CMD_ARGV[0]);
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static const struct command_registration presto_command_handlers[] = {
	{
		.name = "presto_serial",
		.handler = presto_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure USB serial number of Presto device.",
		.usage = "serial_string",
	},
	COMMAND_REGISTRATION_DONE
};

static int presto_jtag_init(void)
{
	if (presto_open(presto_serial) != ERROR_OK) {
		presto_close();
		if (presto_serial != NULL)
			LOG_ERROR("Cannot open PRESTO, serial number '%s'", presto_serial);
		else
			LOG_ERROR("Cannot open PRESTO");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_INFO("PRESTO open, serial number '%s'", presto->serial);

	bitq_interface = &presto_bitq;
	return ERROR_OK;
}

static int presto_jtag_quit(void)
{
	bitq_cleanup();
	presto_close();
	LOG_INFO("PRESTO closed");

	free(presto_serial);
	presto_serial = NULL;

	return ERROR_OK;
}

static struct jtag_interface presto_interface = {
	.execute_queue = bitq_execute_queue,
};

struct adapter_driver presto_adapter_driver = {
	.name = "presto",
	.transports = jtag_only,
	.commands = presto_command_handlers,

	.init = presto_jtag_init,
	.quit = presto_jtag_quit,
	.speed = presto_jtag_speed,
	.khz = presto_adapter_khz,
	.speed_div = presto_jtag_speed_div,

	.jtag_ops = &presto_interface,
};
