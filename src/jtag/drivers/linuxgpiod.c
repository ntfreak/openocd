/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Bitbang driver for Linux GPIO descriptors through libgpiod
 * Copyright (C) 2020 Antonio Borneo <borneo.antonio@gmail.com>
 *
 * Largely based on sysfsgpio driver
 * Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au
 * Copyright (C) 2014 by Jean-Christian de Rivaz <jc@eclis.ch>
 * Copyright (C) 2014 by Paul Fertser <fercerpav@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gpiod.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

/* gpio numbers for each gpio. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;
static int swclk_gpio = -1;
static int swdio_gpio = -1;
static int led_gpio = -1;
static int gpiochip = -1;

static struct gpiod_chip *gpiod_chip;
static struct gpiod_line *gpiod_tck;
static struct gpiod_line *gpiod_tms;
static struct gpiod_line *gpiod_tdi;
static struct gpiod_line *gpiod_tdo;
static struct gpiod_line *gpiod_trst;
static struct gpiod_line *gpiod_swclk;
static struct gpiod_line *gpiod_swdio;
static struct gpiod_line *gpiod_srst;
static struct gpiod_line *gpiod_led;

static int last_swclk;
static int last_swdio;
static bool last_stored;
static bool swdio_input;

/* Bitbang interface read of TDO */
static bb_value_t linuxgpiod_read(void)
{
	int retval;

	retval = gpiod_line_get_value(gpiod_tdo);
	if (retval < 0) {
		LOG_WARNING("reading tdo failed");
		return 0;
	}

	return retval ? BB_HIGH : BB_LOW;
}

/*
 * Bitbang interface write of TCK, TMS, TDI
 *
 * Seeing as this is the only function where the outputs are changed,
 * we can cache the old value to avoid needlessly writing it.
 */
static int linuxgpiod_write(int tck, int tms, int tdi)
{
	static int last_tck;
	static int last_tms;
	static int last_tdi;

	static int first_time;

	int retval;

	if (!first_time) {
		last_tck = !tck;
		last_tms = !tms;
		last_tdi = !tdi;
		first_time = 1;
	}

	if (tdi != last_tdi) {
		retval = gpiod_line_set_value(gpiod_tdi, tdi);
		if (retval < 0)
			LOG_WARNING("writing tdi failed");
	}

	if (tms != last_tms) {
		retval = gpiod_line_set_value(gpiod_tms, tms);
		if (retval < 0)
			LOG_WARNING("writing tms failed");
	}

	/* write clk last */
	if (tck != last_tck) {
		retval = gpiod_line_set_value(gpiod_tck, tck);
		if (retval < 0)
			LOG_WARNING("writing tck failed");
	}

	last_tdi = tdi;
	last_tms = tms;
	last_tck = tck;

	return ERROR_OK;
}

static int linuxgpiod_swdio_read(void)
{
	int retval;

	retval = gpiod_line_get_value(gpiod_swdio);
	if (retval < 0) {
		LOG_WARNING("Fail read swdio");
		return 0;
	}

	return retval;
}

static void linuxgpiod_swdio_drive(bool is_output)
{
	int retval;

	/*
	 * FIXME: change direction requires release and re-require the line
	 * https://stackoverflow.com/questions/58735140/
	 * this would change in future libgpiod
	 */
	gpiod_line_release(gpiod_swdio);

	if (is_output) {
		retval = gpiod_line_request_output(gpiod_swdio, "OpenOCD", 1);
		if (retval < 0)
			LOG_WARNING("Fail request_output line swdio");
	} else {
		retval = gpiod_line_request_input(gpiod_swdio, "OpenOCD");
		if (retval < 0)
			LOG_WARNING("Fail request_input line swdio");
	}

	last_stored = false;
	swdio_input = !is_output;
}

static int linuxgpiod_swd_write(int swclk, int swdio)
{
	int retval;

	if (!swdio_input) {
		if (!last_stored || (swdio != last_swdio)) {
			retval = gpiod_line_set_value(gpiod_swdio, swdio);
			if (retval < 0)
				LOG_WARNING("Fail set swdio");
		}
	}

	/* write swclk last */
	if (!last_stored || (swclk != last_swclk)) {
		retval = gpiod_line_set_value(gpiod_swclk, swclk);
		if (retval < 0)
			LOG_WARNING("Fail set swclk");
	}

	last_swdio = swdio;
	last_swclk = swclk;
	last_stored = true;

	return ERROR_OK;
}

static int linuxgpiod_blink(int on)
{
	int retval;

	if (!gpiod_led)
		return ERROR_OK;

	retval = gpiod_line_set_value(gpiod_led, on);
	if (retval < 0)
		LOG_WARNING("Fail set led");
	return retval;
}

static struct bitbang_interface linuxgpiod_bitbang = {
	.read = linuxgpiod_read,
	.write = linuxgpiod_write,
	.swdio_read = linuxgpiod_swdio_read,
	.swdio_drive = linuxgpiod_swdio_drive,
	.swd_write = linuxgpiod_swd_write,
	.blink = linuxgpiod_blink,
};

/*
 * Bitbang interface to manipulate reset lines SRST and TRST
 *
 * (1) assert or (0) deassert reset lines
 */
static int linuxgpiod_reset(int trst, int srst)
{
	int retval1 = 0, retval2 = 0;

	LOG_DEBUG("linuxgpiod_reset");

	/* assume active low */
	if (gpiod_srst != NULL) {
		retval1 = gpiod_line_set_value(gpiod_srst, srst ? 0 : 1);
		if (retval1 < 0)
			LOG_WARNING("set srst value failed");
	}

	/* assume active low */
	if (gpiod_trst != NULL) {
		retval2 = gpiod_line_set_value(gpiod_trst, trst ? 0 : 1);
		if (retval2 < 0)
			LOG_WARNING("set trst value failed");
	}

	return ((retval1 < 0) || (retval2 < 0)) ? ERROR_FAIL : ERROR_OK;
}

/*
 * Helper function to determine if gpio number is valid
 *
 * Assume here that there will be less than 10000 gpios per gpiochip
 */
static bool is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio < 10000;
}

static bool linuxgpiod_jtag_mode_possible(void)
{
	if (!is_gpio_valid(tck_gpio))
		return false;
	if (!is_gpio_valid(tms_gpio))
		return false;
	if (!is_gpio_valid(tdi_gpio))
		return false;
	if (!is_gpio_valid(tdo_gpio))
		return false;
	return true;
}

static bool linuxgpiod_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return false;
	if (!is_gpio_valid(swdio_gpio))
		return false;
	return true;
}

static inline void helper_release(struct gpiod_line *line)
{
	if (line)
		gpiod_line_release(line);
}

static int linuxgpiod_quit(void)
{
	helper_release(gpiod_led);
	helper_release(gpiod_srst);
	helper_release(gpiod_swdio);
	helper_release(gpiod_swclk);
	helper_release(gpiod_trst);
	helper_release(gpiod_tms);
	helper_release(gpiod_tck);
	helper_release(gpiod_tdi);
	helper_release(gpiod_tdo);

	gpiod_chip_close(gpiod_chip);

	return ERROR_OK;
}

static struct gpiod_line *helper_get_input_line(const char *label, unsigned int offset)
{
	struct gpiod_line *line;
	int retval;

	line = gpiod_chip_get_line(gpiod_chip, offset);
	if (line == NULL) {
		LOG_ERROR("Error get line %s", label);
		return NULL;
	}

	retval = gpiod_line_request_input(line, "OpenOCD");
	if (retval < 0) {
		LOG_ERROR("Error request_input line %s", label);
		return NULL;
	}

	return line;
}

static struct gpiod_line *helper_get_output_line(const char *label, unsigned int offset, int val)
{
	struct gpiod_line *line;
	int retval;

	line = gpiod_chip_get_line(gpiod_chip, offset);
	if (line == NULL) {
		LOG_ERROR("Error get line %s", label);
		return NULL;
	}

	retval = gpiod_line_request_output(line, "OpenOCD", val);
	if (retval < 0) {
		LOG_ERROR("Error request_output line %s", label);
		return NULL;
	}

	return line;
}

static int linuxgpiod_init(void)
{
	LOG_INFO("Linux GPIOD JTAG/SWD bitbang driver");

	bitbang_interface = &linuxgpiod_bitbang;

	gpiod_chip = gpiod_chip_open_by_number(gpiochip);
	if (gpiod_chip == NULL) {
		LOG_ERROR("Cannot open LinuxGPIOD gpiochip %d", gpiochip);
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 * For SWD, SWCLK and SWDIO are configures as output high.
	 */

	if (transport_is_jtag()) {
		if (!linuxgpiod_jtag_mode_possible()) {
			LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
			goto out_error;
		}

		gpiod_tdo = helper_get_input_line("tdo", tdo_gpio);
		if (gpiod_tdo == NULL)
			goto out_error;

		gpiod_tdi = helper_get_output_line("tdi", tdi_gpio, 0);
		if (gpiod_tdi == NULL)
			goto out_error;

		gpiod_tck = helper_get_output_line("tck", tck_gpio, 0);
		if (gpiod_tck == NULL)
			goto out_error;

		gpiod_tms = helper_get_output_line("tms", tms_gpio, 1);
		if (gpiod_tms == NULL)
			goto out_error;

		if (is_gpio_valid(trst_gpio)) {
			gpiod_trst = helper_get_output_line("trst", trst_gpio, 1);
			if (gpiod_trst == NULL)
				goto out_error;
		}
	}

	if (transport_is_swd()) {
		if (!linuxgpiod_swd_mode_possible()) {
			LOG_ERROR("Require swclk and swdio gpio for SWD mode");
			goto out_error;
		}

		gpiod_swclk = helper_get_output_line("swclk", swclk_gpio, 1);
		if (gpiod_swclk == NULL)
			goto out_error;

		gpiod_swdio = helper_get_output_line("swdio", swdio_gpio, 1);
		if (gpiod_swdio == NULL)
			goto out_error;
	}

	if (is_gpio_valid(srst_gpio)) {
		gpiod_srst = helper_get_output_line("srst", srst_gpio, 1);
		if (gpiod_srst == NULL)
			goto out_error;
	}

	if (is_gpio_valid(led_gpio)) {
		gpiod_led = helper_get_output_line("led", led_gpio, 0);
		if (gpiod_led == NULL)
			goto out_error;
	}

	return ERROR_OK;

out_error:
	linuxgpiod_quit();

	return ERROR_JTAG_INIT_FAILED;
}

COMMAND_HANDLER(linuxgpiod_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"LinuxGPIOD nums: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "LinuxGPIOD num: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "LinuxGPIOD num: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "LinuxGPIOD num: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "LinuxGPIOD num: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "LinuxGPIOD num: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "LinuxGPIOD num: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"LinuxGPIOD nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "LinuxGPIOD num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "LinuxGPIOD num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_gpionum_led)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], led_gpio);

	command_print(CMD, "LinuxGPIOD num: led = %d", led_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(linuxgpiod_handle_gpiochip)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], gpiochip);

	command_print(CMD, "LinuxGPIOD gpiochip = %d", gpiochip);
	return ERROR_OK;
}

static const struct command_registration linuxgpiod_command_handlers[] = {
	{
		.name = "linuxgpiod_jtag_nums",
		.handler = linuxgpiod_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "tck tms tdi tdo",
	},
	{
		.name = "linuxgpiod_tck_num",
		.handler = linuxgpiod_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "tck",
	},
	{
		.name = "linuxgpiod_tms_num",
		.handler = linuxgpiod_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "tms",
	},
	{
		.name = "linuxgpiod_tdo_num",
		.handler = linuxgpiod_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "tdo",
	},
	{
		.name = "linuxgpiod_tdi_num",
		.handler = linuxgpiod_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "tdi",
	},
	{
		.name = "linuxgpiod_srst_num",
		.handler = linuxgpiod_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "srst",
	},
	{
		.name = "linuxgpiod_trst_num",
		.handler = linuxgpiod_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "trst",
	},
	{
		.name = "linuxgpiod_swd_nums",
		.handler = linuxgpiod_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "swclk swdio",
	},
	{
		.name = "linuxgpiod_swclk_num",
		.handler = linuxgpiod_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "swclk",
	},
	{
		.name = "linuxgpiod_swdio_num",
		.handler = linuxgpiod_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "swdio",
	},
	{
		.name = "linuxgpiod_led_num",
		.handler = linuxgpiod_handle_gpionum_led,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for LED.",
		.usage = "led",
	},
	{
		.name = "linuxgpiod_gpiochip",
		.handler = linuxgpiod_handle_gpiochip,
		.mode = COMMAND_CONFIG,
		.help = "number of the gpiochip.",
		.usage = "gpiochip",
	},
	COMMAND_REGISTRATION_DONE
};

static const char *const linuxgpiod_transport[] = { "swd", "jtag", NULL };

static struct jtag_interface linuxgpiod_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver linuxgpiod_adapter_driver = {
	.name = "linuxgpiod",
	.transports = linuxgpiod_transport,
	.commands = linuxgpiod_command_handlers,

	.init = linuxgpiod_init,
	.quit = linuxgpiod_quit,
	.reset = linuxgpiod_reset,

	.jtag_ops = &linuxgpiod_interface,
	.swd_ops = &bitbang_swd,
};
