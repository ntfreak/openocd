/***************************************************************************
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
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

/* 2014-12: Addition of the SWD protocol support is based on the initial work
 * on bcm2835gpio.c by Paul Fertser and modifications by Jean-Christian de Rivaz. */

/**
 * @file
 * This driver implements a bitbang jtag interface using gpio lines via
 * sysfs.
 * The aim of this driver implementation is use system GPIOs but avoid the
 * need for a additional kernel driver.
 * (Note memory mapped IO is another option, however it doesn't mix well with
 * the kernel gpiolib driver - which makes sense I guess.)
 *
 * A gpio is required for tck, tms, tdi and tdo. One or both of srst and trst
 * must be also be specified. The required jtag gpios are specified via the
 * sysfsgpio_jtag_nums command or the relevant sysfsgpio_XXX_num commands.
 * The srst and trst gpios are set via the sysfsgpio_srst_num and
 * sysfsgpio_trst_num respectively. GPIO numbering follows the kernel
 * convention of starting from 0.
 *
 * The gpios should not be in use by another entity, and must not be requested
 * by a kernel driver without also being exported by it (otherwise they can't
 * be exported by sysfs).
 *
 * The sysfs gpio interface can only manipulate one gpio at a time, so the
 * bitbang write handler remembers the last state for tck, tms, tdi to avoid
 * superfluous writes.
 * For speed the sysfs "value" entry is opened at init and held open.
 * This results in considerable gains over open-write-close (45s vs 900s)
 *
 * Further work could address:
 *  -srst and trst open drain/ push pull
 *  -configurable active high/low for srst & trst
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

/*
 * Helper func to determine if gpio number valid
 *
 * Assume here that there will be less than 10000 gpios on a system
 */
static bool is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio < 10000;
}

/*
 * Helper func to open, write to and close a file
 * name and valstr must be null terminated.
 *
 * Returns negative on failure.
 */
static int open_write_close(const char *name, const char *valstr)
{
	int ret;
	int fd = open(name, O_WRONLY);
	if (fd < 0)
		return fd;

	ret = write(fd, valstr, strlen(valstr));
	close(fd);

	return ret;
}

/*
 * Helper func to unexport gpio from sysfs
 */
static void unexport_sysfs_gpio(int gpio)
{
	char gpiostr[5];

	if (!is_gpio_valid(gpio))
		return;

	snprintf(gpiostr, sizeof(gpiostr), "%d", gpio);
	if (open_write_close("/sys/class/gpio/unexport", gpiostr) < 0)
		LOG_ERROR("Couldn't unexport gpio %d", gpio);
}

/*
 * Exports and sets up direction for gpio.
 * If the gpio is an output, it is initialized according to init_high,
 * otherwise it is ignored.
 *
 * If the gpio is already exported we just show a warning and continue; if
 * openocd happened to crash (or was killed by user) then the gpios will not
 * have been cleaned up.
 */
static int setup_sysfs_gpio(int gpio, int is_output, int init_high)
{
	struct timeval timeout, now;
	char buf[40];
	char gpiostr[5];
	int ret;

	if (!is_gpio_valid(gpio))
		return ERROR_OK;

	snprintf(gpiostr, sizeof(gpiostr), "%d", gpio);
	ret = open_write_close("/sys/class/gpio/export", gpiostr);
	if (ret < 0) {
		if (errno == EBUSY) {
			LOG_WARNING("gpio %d is already exported", gpio);
		} else {
			LOG_ERROR("Couldn't export gpio %d", gpio);
			LOG_ERROR("sysfsgpio: %s", strerror(errno));
			return ERROR_FAIL;
		}
	}

	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, 0, 500000);

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
	for (;;) {
		ret = open_write_close(buf, is_output ? (init_high ? "high" : "low") : "in");
		if (ret >= 0 || errno != EACCES)
			break;
		gettimeofday(&now, NULL);
		if (timeval_compare(&now, &timeout) >= 0)
			break;
		jtag_sleep(10000);
	}
	if (ret < 0) {
		LOG_ERROR("Couldn't set direction for gpio %d", gpio);
		LOG_ERROR("sysfsgpio: %s", strerror(errno));
		unexport_sysfs_gpio(gpio);
		return ERROR_FAIL;
	}

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
	for (;;) {
		ret = open(buf, O_RDWR | O_NONBLOCK | O_SYNC);
		if (ret >= 0 || errno != EACCES)
			break;
		gettimeofday(&now, NULL);
		if (timeval_compare(&now, &timeout) >= 0)
			break;
		jtag_sleep(10000);
	}
	if (ret < 0) {
		LOG_ERROR("Couldn't open value for gpio %d", gpio);
		LOG_ERROR("sysfsgpio: %s", strerror(errno));
		unexport_sysfs_gpio(gpio);
	}

	return ret;
}

/* gpio numbers for each gpio. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;
static int swclk_gpio = -1;
static int swdio_gpio = -1;

/*
 * file descriptors for /sys/class/gpio/gpioXX/value
 * Set up during init.
 */
static int tck_fd = -1;
static int tms_fd = -1;
static int tdi_fd = -1;
static int tdo_fd = -1;
static int trst_fd = -1;
static int srst_fd = -1;
static int swclk_fd = -1;
static int swdio_fd = -1;

static int last_swclk;
static int last_swdio;
static bool last_stored;
static bool swdio_input;

static void sysfsgpio_swdio_drive(bool is_output)
{
	char buf[40];
	int ret;

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", swdio_gpio);
	ret = open_write_close(buf, is_output ? "high" : "in");
	if (ret < 0) {
		LOG_ERROR("Couldn't set direction for gpio %d", swdio_gpio);
		LOG_ERROR("sysfsgpio: %s", strerror(errno));
	}

	last_stored = false;
	swdio_input = !is_output;
}

static int sysfsgpio_swdio_read(void)
{
	char buf[1];

	/* important to seek to signal sysfs of new read */
	lseek(swdio_fd, 0, SEEK_SET);
	int ret = read(swdio_fd, &buf, sizeof(buf));

	if (ret < 0) {
		LOG_WARNING("reading swdio failed");
		return 0;
	}

	return buf[0] != '0';
}

static int sysfsgpio_swd_write(int swclk, int swdio)
{
	const char one[] = "1";
	const char zero[] = "0";

	size_t bytes_written;

	if (!swdio_input) {
		if (!last_stored || (swdio != last_swdio)) {
			bytes_written = write(swdio_fd, swdio ? &one : &zero, 1);
			if (bytes_written != 1)
				LOG_WARNING("writing swdio failed");
		}
	}

	/* write swclk last */
	if (!last_stored || (swclk != last_swclk)) {
		bytes_written = write(swclk_fd, swclk ? &one : &zero, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing swclk failed");
	}

	last_swdio = swdio;
	last_swclk = swclk;
	last_stored = true;

	return ERROR_OK;
}

/*
 * Bitbang interface read of TDO
 *
 * The sysfs value will read back either '0' or '1'. The trick here is to call
 * lseek to bypass buffering in the sysfs kernel driver.
 */
static bb_value_t sysfsgpio_read(void)
{
	char buf[1];

	/* important to seek to signal sysfs of new read */
	lseek(tdo_fd, 0, SEEK_SET);
	int ret = read(tdo_fd, &buf, sizeof(buf));

	if (ret < 0) {
		LOG_WARNING("reading tdo failed");
		return 0;
	}

	return buf[0] == '0' ? BB_LOW : BB_HIGH;
}

/*
 * Bitbang interface write of TCK, TMS, TDI
 *
 * Seeing as this is the only function where the outputs are changed,
 * we can cache the old value to avoid needlessly writing it.
 */
static int sysfsgpio_write(int tck, int tms, int tdi)
{
	const char one[] = "1";
	const char zero[] = "0";

	static int last_tck;
	static int last_tms;
	static int last_tdi;

	static int first_time;
	size_t bytes_written;

	if (!first_time) {
		last_tck = !tck;
		last_tms = !tms;
		last_tdi = !tdi;
		first_time = 1;
	}

	if (tdi != last_tdi) {
		bytes_written = write(tdi_fd, tdi ? &one : &zero, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing tdi failed");
	}

	if (tms != last_tms) {
		bytes_written = write(tms_fd, tms ? &one : &zero, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing tms failed");
	}

	/* write clk last */
	if (tck != last_tck) {
		bytes_written = write(tck_fd, tck ? &one : &zero, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing tck failed");
	}

	last_tdi = tdi;
	last_tms = tms;
	last_tck = tck;

	return ERROR_OK;
}

/*
 * Bitbang interface to manipulate reset lines SRST and TRST
 *
 * (1) assert or (0) deassert reset lines
 */
static int sysfsgpio_reset(int trst, int srst)
{
	LOG_DEBUG("sysfsgpio_reset");
	const char one[] = "1";
	const char zero[] = "0";
	size_t bytes_written;

	/* assume active low */
	if (srst_fd >= 0) {
		bytes_written = write(srst_fd, srst ? &zero : &one, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing srst failed");
	}

	/* assume active low */
	if (trst_fd >= 0) {
		bytes_written = write(trst_fd, trst ? &zero : &one, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing trst failed");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionums)
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
			"SysfsGPIO nums: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "SysfsGPIO num: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "SysfsGPIO num: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "SysfsGPIO num: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "SysfsGPIO num: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "SysfsGPIO num: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "SysfsGPIO num: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"SysfsGPIO nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "SysfsGPIO num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(sysfsgpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "SysfsGPIO num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

static const struct command_registration sysfsgpio_command_handlers[] = {
	{
		.name = "sysfsgpio_jtag_nums",
		.handler = &sysfsgpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "sysfsgpio_tck_num",
		.handler = &sysfsgpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "sysfsgpio_tms_num",
		.handler = &sysfsgpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "sysfsgpio_tdo_num",
		.handler = &sysfsgpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "sysfsgpio_tdi_num",
		.handler = &sysfsgpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "sysfsgpio_srst_num",
		.handler = &sysfsgpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "sysfsgpio_trst_num",
		.handler = &sysfsgpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "sysfsgpio_swd_nums",
		.handler = &sysfsgpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "sysfsgpio_swclk_num",
		.handler = &sysfsgpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "sysfsgpio_swdio_num",
		.handler = &sysfsgpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	COMMAND_REGISTRATION_DONE
};

static int sysfsgpio_init(void);
static int sysfsgpio_quit(void);

static const char * const sysfsgpio_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface sysfsgpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver sysfsgpio_adapter_driver = {
	.name = "sysfsgpio",
	.transports = sysfsgpio_transports,
	.commands = sysfsgpio_command_handlers,

	.init = sysfsgpio_init,
	.quit = sysfsgpio_quit,
	.reset = sysfsgpio_reset,

	.jtag_ops = &sysfsgpio_interface,
	.swd_ops = &bitbang_swd,
};

static struct bitbang_interface sysfsgpio_bitbang = {
	.read = sysfsgpio_read,
	.write = sysfsgpio_write,
	.swdio_read = sysfsgpio_swdio_read,
	.swdio_drive = sysfsgpio_swdio_drive,
	.swd_write = sysfsgpio_swd_write,
	.blink = 0
};

/* helper func to close and cleanup files only if they were valid/ used */
static void cleanup_fd(int fd, int gpio)
{
	if (gpio >= 0) {
		if (fd >= 0)
			close(fd);

		unexport_sysfs_gpio(gpio);
	}
}

static void cleanup_all_fds(void)
{
	if (transport_is_jtag()) {
		cleanup_fd(tck_fd, tck_gpio);
		cleanup_fd(tms_fd, tms_gpio);
		cleanup_fd(tdi_fd, tdi_gpio);
		cleanup_fd(tdo_fd, tdo_gpio);
		cleanup_fd(trst_fd, trst_gpio);
	}
	if (transport_is_swd()) {
		cleanup_fd(swclk_fd, swclk_gpio);
		cleanup_fd(swdio_fd, swdio_gpio);
	}
	cleanup_fd(srst_fd, srst_gpio);
}

static bool sysfsgpio_jtag_mode_possible(void)
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

static bool sysfsgpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return false;
	if (!is_gpio_valid(swdio_gpio))
		return false;
	return true;
}

static int sysfsgpio_init(void)
{
	bitbang_interface = &sysfsgpio_bitbang;

	LOG_INFO("SysfsGPIO JTAG/SWD bitbang driver");

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 * For SWD, SWCLK and SWDIO are configures as output high.
	 */

	if (transport_is_jtag()) {
		if (!sysfsgpio_jtag_mode_possible()) {
			LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
			return ERROR_JTAG_INIT_FAILED;
		}

		tck_fd = setup_sysfs_gpio(tck_gpio, 1, 0);
		if (tck_fd < 0)
			goto out_error;

		tms_fd = setup_sysfs_gpio(tms_gpio, 1, 1);
		if (tms_fd < 0)
			goto out_error;

		tdi_fd = setup_sysfs_gpio(tdi_gpio, 1, 0);
		if (tdi_fd < 0)
			goto out_error;

		tdo_fd = setup_sysfs_gpio(tdo_gpio, 0, 0);
		if (tdo_fd < 0)
			goto out_error;

		/* assume active low*/
		if (trst_gpio >= 0) {
			trst_fd = setup_sysfs_gpio(trst_gpio, 1, 1);
			if (trst_fd < 0)
				goto out_error;
		}
	}

	if (transport_is_swd()) {
		if (!sysfsgpio_swd_mode_possible()) {
			LOG_ERROR("Require swclk and swdio gpio for SWD mode");
			return ERROR_JTAG_INIT_FAILED;
		}

		swclk_fd = setup_sysfs_gpio(swclk_gpio, 1, 0);
		if (swclk_fd < 0)
			goto out_error;

		swdio_fd = setup_sysfs_gpio(swdio_gpio, 1, 0);
		if (swdio_fd < 0)
			goto out_error;
	}

	/* assume active low*/
	if (srst_gpio >= 0) {
		srst_fd = setup_sysfs_gpio(srst_gpio, 1, 1);
		if (srst_fd < 0)
			goto out_error;
	}

	return ERROR_OK;

out_error:
	cleanup_all_fds();
	return ERROR_JTAG_INIT_FAILED;
}

static int sysfsgpio_quit(void)
{
	cleanup_all_fds();
	return ERROR_OK;
}
