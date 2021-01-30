/***************************************************************************
 *   Copyright (C) 2021 by Manuel Wick <manuel@matronix.de>                *
 *   Copyright (C) 2013 Paul Fertser <fercerpav@gmail.com>                 *
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

/*
  This is a test application to be used as a remote bitbang server for
  the OpenOCD remote_bitbang interface driver.

  To compile run:
  gcc -Wall -ansi -pedantic -std=c99 -o remote_bitbang_sysfsgpio remote_bitbang_sysfsgpio.c


  Usage example:

  On Raspberry Pi run:
  socat TCP6-LISTEN:7777,fork EXEC:"sudo ./remote_bitbang_sysfsgpio tck 11 tms 25 tdo 9 tdi 10"

  On host run:
  openocd -c "adapter driver remote_bitbang; remote_bitbang_host raspberrypi; remote_bitbang_port 7777" \
	  -f target/stm32f1x.cfg

  Or if you want to test UNIX sockets, run both on Raspberry Pi:
  socat UNIX-LISTEN:/tmp/remotebitbang-socket,fork EXEC:"sudo ./remote_bitbang_sysfsgpio tck 11 tms 25 tdo 9 tdi 10"
  openocd -c "adapter driver remote_bitbang; remote_bitbang_host /tmp/remotebitbang-socket" -f target/stm32f1x.cfg
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#define LOG_ERROR(...)		do {					\
		fprintf(stderr, __VA_ARGS__);				\
		fputc('\n', stderr);					\
	} while (0)
#define LOG_WARNING(...)	LOG_ERROR(__VA_ARGS__)

#define ERROR_OK	(-1)
#define ERROR_FAIL	(-2)
#define ERROR_JTAG_INIT_FAILED	ERROR_FAIL

/*
 * Helper func to determine if gpio number valid
 *
 * Assume here that there will be less than 1000 gpios on a system
 */
static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio < 1000;
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
	char gpiostr[4];

	if (!is_gpio_valid(gpio))
		return;

	snprintf(gpiostr, sizeof(gpiostr), "%d", gpio);
	if (open_write_close("/sys/class/gpio/unexport", gpiostr) < 0)
		LOG_ERROR("Couldn't unexport gpio %d", gpio);

	return;
}

/*
 * Exports and sets up direction for gpio.
 * If the gpio is an output, it is initialized according to init_high,
 * otherwise it is ignored.
 *
 * When open_rw is set, the file descriptor will be open as read and write,
 * e.g. for SWDIO (TMS) that is used as input and output.
 *
 * If the gpio is already exported we just show a warning and continue; if
 * openocd happened to crash (or was killed by user) then the gpios will not
 * have been cleaned up.
 */
static int setup_sysfs_gpio(int gpio, int is_output, int init_high, int open_rw)
{
	char buf[40];
	char gpiostr[4];
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
			perror("sysfsgpio: ");
			return ERROR_FAIL;
		}
	}

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
	ret = open_write_close(buf, is_output ? (init_high ? "high" : "low") : "in");
	if (ret < 0) {
		LOG_ERROR("Couldn't set direction for gpio %d", gpio);
		perror("sysfsgpio: ");
		unexport_sysfs_gpio(gpio);
		return ERROR_FAIL;
	}

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
	if (open_rw)
		ret = open(buf, O_RDWR | O_NONBLOCK | O_SYNC);
	else if (is_output)
		ret = open(buf, O_WRONLY | O_NONBLOCK | O_SYNC);
	else
		ret = open(buf, O_RDONLY | O_NONBLOCK | O_SYNC);

	if (ret < 0)
		unexport_sysfs_gpio(gpio);

	return ret;
}

/*
 * Change direction for gpio.
 */
static int change_dir_sysfs_gpio(int gpio, int is_output, int init_high)
{
	char buf[40];
	int ret;

	if (!is_gpio_valid(gpio))
		return ERROR_OK;

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
	ret = open_write_close(buf, is_output ? (init_high ? "high" : "low") : "in");
	if (ret < 0) {
		LOG_ERROR("Couldn't set direction for gpio %d", gpio);
		perror("sysfsgpio: ");
		unexport_sysfs_gpio(gpio);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* gpio numbers for each gpio. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;

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

/*
 * GPIO state of /sys/class/gpio/gpioXX/value
 */
static int last_tck = -1;
static int last_tms = -1;
static int last_tms_drive = -1;
static int last_tdi = -1;
static int last_initialized = -1;

/*
 * Bitbang interface read of TDO
 *
 * The sysfs value will read back either '0' or '1'. The trick here is to call
 * lseek to bypass buffering in the sysfs kernel driver.
 */
static int sysfsgpio_read(void)
{
	char buf[1];

	/* important to seek to signal sysfs of new read */
	lseek(tdo_fd, 0, SEEK_SET);
	int ret = read(tdo_fd, &buf, sizeof(buf));

	if (ret < 0) {
		LOG_WARNING("reading tdo failed");
		return 0;
	}

	return buf[0];
}

/*
 * Bitbang interface write of TCK, TMS, TDI
 *
 * Output states are changed here and in sysfsgpio_write_swd,
 * which are not used simultaneously, so we can cache the old
 * value to avoid needlessly writing it.
 */
static void sysfsgpio_write(int tck, int tms, int tdi)
{
	const char one[] = "1";
	const char zero[] = "0";

	size_t bytes_written;

	if (!last_initialized) {
		last_tck = !tck;
		last_tms = !tms;
		last_tdi = !tdi;
		last_initialized = 1;
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
}

/*
 * Bitbang interface to manipulate reset lines SRST and TRST
 *
 * (1) assert or (0) deassert reset lines
 */
static void sysfsgpio_reset(int trst, int srst)
{
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
}

/*
 * Bitbang interface set direction of SWDIO (TMS)
 */
static void sysfsgpio_swdio_drive(int is_output)
{
	int ret;

	if ((is_output != 0) && (last_tms == -1))
		last_tms = 0;

	ret = change_dir_sysfs_gpio(tms_gpio, (is_output != 0) ? 1 : 0, last_tms);
	if (ret != ERROR_OK)
		LOG_WARNING("Failed to change SWDIO (TMS) direction to output");
	else
		last_tms_drive = (is_output != 0) ? 1 : 0;
}

/*
 * Bitbang interface read of SWDIO (TMS)
 *
 * The sysfs value will read back either '0' or '1'. The trick here is to call
 * lseek to bypass buffering in the sysfs kernel driver.
 */
static int sysfsgpio_swdio_read(void)
{
	char buf[1];

	/* important to seek to signal sysfs of new read */
	lseek(tms_fd, 0, SEEK_SET);
	int ret = read(tms_fd, &buf, sizeof(buf));

	if (ret < 0) {
		LOG_WARNING("reading tdo failed");
		return 0;
	}

	return buf[0];
}

/*
 * Bitbang interface write of SWCLK (TCK) and SWDIO (TMS)
 *
 * Output states are changed here and in sysfsgpio_write, which
 * are not used simultaneously, so we can cache the old value
 * to avoid needlessly writing it.
 */
static void sysfsgpio_swd_write(int swclk, int swdio)
{
	const char one[] = "1";
	const char zero[] = "0";

	size_t bytes_written;

	if (!last_initialized) {
		last_tck = !swclk;
		last_tms = !swdio;
		last_initialized = 1;
	}

	if ((last_tms_drive == 1) && (swdio != last_tms)) {
		bytes_written = write(tms_fd, swdio ? &one : &zero, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing swdio (tms) failed");
	}

	/* write clk last */
	if (swclk != last_tck) {
		bytes_written = write(tck_fd, swclk ? &one : &zero, 1);
		if (bytes_written != 1)
			LOG_WARNING("writing swclk (tck) failed");
	}

	last_tms = swdio;
	last_tck = swclk;
}

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
	cleanup_fd(tck_fd, tck_gpio);
	cleanup_fd(tms_fd, tms_gpio);
	cleanup_fd(tdi_fd, tdi_gpio);
	cleanup_fd(tdo_fd, tdo_gpio);
	cleanup_fd(trst_fd, trst_gpio);
	cleanup_fd(srst_fd, srst_gpio);
}

static void process_remote_protocol(void)
{
	int c;
	while (1) {
		c = getchar();
		if (c == EOF || c == 'Q') /* Quit */
			break;
		else if (c == 'b' || c == 'B') /* Blink */
			continue;
		else if (c >= 'r' && c <= 'r' + 3) { /* Reset */
			char d = c - 'r';
			sysfsgpio_reset(!!(d & 2),
					(d & 1));
		} else if (c >= '0' && c <= '0' + 7) { /* Write */
			char d = c - '0';
			sysfsgpio_write(!!(d & 4),
					!!(d & 2),
					(d & 1));
		} else if (c == 'R')
			putchar(sysfsgpio_read());
		else if (c == 'c') /* SWDIO read */
			putchar(sysfsgpio_swdio_read());
		else if (c == 'o' || c == 'O') /* SWDIO drive */
			sysfsgpio_swdio_drive(c == 'o' ? 0 : 1);
		else if (c >= 'd' && c <= 'g') { /* SWD write */
			char d = c - 'd';
			sysfsgpio_swd_write((d & 2), (d & 1));
		}
		else
			LOG_ERROR("Unknown command '%c' received", c);
	}
}

int main(int argc, char *argv[])
{
	LOG_WARNING("SysfsGPIO remote_bitbang JTAG+SWD driver\n");

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "tck"))
			tck_gpio = atoi(argv[++i]);
		else if (!strcmp(argv[i], "tms"))
			tms_gpio = atoi(argv[++i]);
		else if (!strcmp(argv[i], "tdo"))
			tdo_gpio = atoi(argv[++i]);
		else if (!strcmp(argv[i], "tdi"))
			tdi_gpio = atoi(argv[++i]);
		else if (!strcmp(argv[i], "trst"))
			trst_gpio = atoi(argv[++i]);
		else if (!strcmp(argv[i], "srst"))
			srst_gpio = atoi(argv[++i]);
		else {
			LOG_ERROR("Usage:\n%s ((tck|tms|tdo|tdi|trst|srst) num)*", argv[0]);
			return -1;
		}
	}

	if (!(is_gpio_valid(tck_gpio)
			&& is_gpio_valid(tms_gpio)
			&& is_gpio_valid(tdi_gpio)
			&& is_gpio_valid(tdo_gpio))) {
		if (!is_gpio_valid(tck_gpio))
			LOG_ERROR("gpio num for tck is invalid");
		if (!is_gpio_valid(tms_gpio))
			LOG_ERROR("gpio num for tms is invalid");
		if (!is_gpio_valid(tdo_gpio))
			LOG_ERROR("gpio num for tdo is invalid");
		if (!is_gpio_valid(tdi_gpio))
			LOG_ERROR("gpio num for tdi is invalid");

		LOG_ERROR("Require tck, tms, tdi and tdo gpios to all be specified");
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	tck_fd = setup_sysfs_gpio(tck_gpio, 1, 0, 0);
	if (tck_fd < 0)
		goto out_error;

	tms_fd = setup_sysfs_gpio(tms_gpio, 1, 1, 1);
	if (tms_fd < 0)
		goto out_error;
	last_tms_drive = 0;

	tdi_fd = setup_sysfs_gpio(tdi_gpio, 1, 0, 0);
	if (tdi_fd < 0)
		goto out_error;

	tdo_fd = setup_sysfs_gpio(tdo_gpio, 0, 0, 0);
	if (tdo_fd < 0)
		goto out_error;

	/* assume active low */
	if (trst_gpio > 0) {
		trst_fd = setup_sysfs_gpio(trst_gpio, 1, 1, 0);
		if (trst_fd < 0)
			goto out_error;
	}

	/* assume active low */
	if (srst_gpio > 0) {
		srst_fd = setup_sysfs_gpio(srst_gpio, 1, 1, 0);
		if (srst_fd < 0)
			goto out_error;
	}

	last_initialized = 0;

	LOG_WARNING("SysfsGPIO nums: tck = %d, tms = %d, tdi = %d, tdo = %d",
		 tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);
	LOG_WARNING("SysfsGPIO num: srst = %d", srst_gpio);
	LOG_WARNING("SysfsGPIO num: trst = %d", trst_gpio);

	setvbuf(stdout, NULL, _IONBF, 0);
	process_remote_protocol();

	cleanup_all_fds();
	return 0;
out_error:
	cleanup_all_fds();
	return ERROR_JTAG_INIT_FAILED;
}
