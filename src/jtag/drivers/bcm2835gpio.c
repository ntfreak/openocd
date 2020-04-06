/***************************************************************************
 *   Copyright (C) 2013 by Paul Fertser, fercerpav@gmail.com               *
 *                                                                         *
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *   Based on at91rm9200.c (c) Anders Larsen                               *
 *   and RPi GPIO examples by Gert van Loo & Dom                           *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"

#include <sys/mman.h>

uint32_t bcm2835_peri_base = 0x20000000;
#define BCM2835_GPIO_BASE	(bcm2835_peri_base + 0x200000) /* GPIO controller */

#define BCM2835_PADS_GPIO_0_27		(bcm2835_peri_base + 0x100000)
#define BCM2835_PADS_GPIO_0_27_OFFSET	(0x2c / 4)

/* GPIO setup macros */
#define MODE_GPIO(g) (*(pio_base+((g)/10))>>(((g)%10)*3) & 7)
#define INP_GPIO(g) do { *(pio_base+((g)/10)) &= ~(7<<(((g)%10)*3)); } while (0)
#define SET_MODE_GPIO(g, m) do { /* clear the mode bits first, then set as necessary */ \
		INP_GPIO(g);						\
		*(pio_base+((g)/10)) |=  ((m)<<(((g)%10)*3)); } while (0)
#define OUT_GPIO(g) SET_MODE_GPIO(g, 1)

#define GPIO_SET (*(pio_base+7))  /* sets   bits which are 1, ignores bits which are 0 */
#define GPIO_CLR (*(pio_base+10)) /* clears bits which are 1, ignores bits which are 0 */
#define GPIO_LEV (*(pio_base+13)) /* current level of the pin */

static int dev_mem_fd;
static volatile uint32_t *pio_base;

static bb_value_t bcm2835gpio_read(void);
static int bcm2835gpio_write(int tck, int tms, int tdi);

static int bcm2835_swdio_read(void);
static void bcm2835_swdio_drive(bool is_output);

static int bcm2835gpio_init(void);
static int bcm2835gpio_quit(void);

static struct bitbang_interface bcm2835gpio_bitbang = {
	.read = bcm2835gpio_read,
	.write = bcm2835gpio_write,
	.swdio_read = bcm2835_swdio_read,
	.swdio_drive = bcm2835_swdio_drive,
	.blink = NULL
};

/* GPIO numbers for each signal. Negative values are invalid */
static int tck_gpio = -1;
static int tck_gpio_mode;
static int tms_gpio = -1;
static int tms_gpio_mode;
static int tdi_gpio = -1;
static int tdi_gpio_mode;
static int tdo_gpio = -1;
static int tdo_gpio_mode;
static int trst_gpio = -1;
static int trst_gpio_mode;
static int srst_gpio = -1;
static int srst_gpio_mode;
static int swclk_gpio = -1;
static int swclk_gpio_mode;
static int swdio_gpio = -1;
static int swdio_gpio_mode;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static unsigned int jtag_delay;

static bb_value_t bcm2835gpio_read(void)
{
	return (GPIO_LEV & 1<<tdo_gpio) ? BB_HIGH : BB_LOW;
}

static int bcm2835gpio_write(int tck, int tms, int tdi)
{
	uint32_t set = tck<<tck_gpio | tms<<tms_gpio | tdi<<tdi_gpio;
	uint32_t clear = !tck<<tck_gpio | !tms<<tms_gpio | !tdi<<tdi_gpio;

	GPIO_SET = set;
	GPIO_CLR = clear;

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int bcm2835gpio_swd_write(int tck, int tms, int tdi)
{
	uint32_t set = tck<<swclk_gpio | tdi<<swdio_gpio;
	uint32_t clear = !tck<<swclk_gpio | !tdi<<swdio_gpio;

	GPIO_SET = set;
	GPIO_CLR = clear;

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int bcm2835gpio_reset(int trst, int srst)
{
	uint32_t set = 0;
	uint32_t clear = 0;

	if (trst_gpio > 0) {
		set |= !trst<<trst_gpio;
		clear |= trst<<trst_gpio;
	}

	if (srst_gpio > 0) {
		set |= !srst<<srst_gpio;
		clear |= srst<<srst_gpio;
	}

	GPIO_SET = set;
	GPIO_CLR = clear;

	return ERROR_OK;
}

static void bcm2835_swdio_drive(bool is_output)
{
	if (is_output)
		OUT_GPIO(swdio_gpio);
	else
		INP_GPIO(swdio_gpio);
}

static int bcm2835_swdio_read(void)
{
	return !!(GPIO_LEV & 1 << swdio_gpio);
}

static int bcm2835gpio_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff/khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int bcm2835gpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int bcm2835gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio <= 53;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionums)
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
			"BCM2835 GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "BCM2835 GPIO config: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "BCM2835 GPIO config: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "BCM2835 GPIO config: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "BCM2835 GPIO config: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "BCM2835 GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "BCM2835 GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"BCM2835 GPIO nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "BCM2835 num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "BCM2835 num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "BCM2835 GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], bcm2835_peri_base);

	command_print(CMD, "BCM2835 GPIO: peripheral_base = 0x%08x",
				  bcm2835_peri_base);
	return ERROR_OK;
}

static const struct command_registration bcm2835gpio_command_handlers[] = {
	{
		.name = "bcm2835gpio_jtag_nums",
		.handler = &bcm2835gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "bcm2835gpio_tck_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "bcm2835gpio_tms_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "bcm2835gpio_tdo_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "bcm2835gpio_tdi_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "bcm2835gpio_swd_nums",
		.handler = &bcm2835gpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "bcm2835gpio_swclk_num",
		.handler = &bcm2835gpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "bcm2835gpio_swdio_num",
		.handler = &bcm2835gpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "bcm2835gpio_srst_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "bcm2835gpio_trst_num",
		.handler = &bcm2835gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "bcm2835gpio_speed_coeffs",
		.handler = &bcm2835gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "bcm2835gpio_peripheral_base",
		.handler = &bcm2835gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const char * const bcm2835_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface bcm2835gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver bcm2835gpio_adapter_driver = {
	.name = "bcm2835gpio",
	.transports = bcm2835_transports,
	.commands = bcm2835gpio_command_handlers,

	.init = bcm2835gpio_init,
	.quit = bcm2835gpio_quit,
	.reset = bcm2835gpio_reset,
	.speed = bcm2835gpio_speed,
	.khz = bcm2835gpio_khz,
	.speed_div = bcm2835gpio_speed_div,

	.jtag_ops = &bcm2835gpio_interface,
	.swd_ops = &bitbang_swd,
};

static bool bcm2835gpio_jtag_mode_possible(void)
{
	if (!is_gpio_valid(tck_gpio))
		return 0;
	if (!is_gpio_valid(tms_gpio))
		return 0;
	if (!is_gpio_valid(tdi_gpio))
		return 0;
	if (!is_gpio_valid(tdo_gpio))
		return 0;
	return 1;
}

static bool bcm2835gpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return 0;
	if (!is_gpio_valid(swdio_gpio))
		return 0;
	return 1;
}

static int bcm2835gpio_init(void)
{
	bitbang_interface = &bcm2835gpio_bitbang;

	LOG_INFO("BCM2835 GPIO JTAG/SWD bitbang driver");

	if (bcm2835gpio_jtag_mode_possible()) {
		if (bcm2835gpio_swd_mode_possible())
			LOG_INFO("JTAG and SWD modes enabled");
		else
			LOG_INFO("JTAG only mode enabled (specify swclk and swdio gpio to add SWD mode)");
	} else if (bcm2835gpio_swd_mode_possible()) {
		LOG_INFO("SWD only mode enabled (specify tck, tms, tdi and tdo gpios to add JTAG mode)");
	} else {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode and/or swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	dev_mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_DEBUG("Cannot open /dev/gpiomem, fallback to /dev/mem");
		dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	}
	if (dev_mem_fd < 0) {
		perror("open");
		return ERROR_JTAG_INIT_FAILED;
	}

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_GPIO_BASE);

	if (pio_base == MAP_FAILED) {
		perror("mmap");
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	static volatile uint32_t *pads_base;
	pads_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_PADS_GPIO_0_27);

	if (pads_base == MAP_FAILED) {
		perror("mmap");
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	/* set 4mA drive strength, slew rate limited, hysteresis on */
	pads_base[BCM2835_PADS_GPIO_0_27_OFFSET] = 0x5a000008 + 1;

	tdo_gpio_mode = MODE_GPIO(tdo_gpio);
	tdi_gpio_mode = MODE_GPIO(tdi_gpio);
	tck_gpio_mode = MODE_GPIO(tck_gpio);
	tms_gpio_mode = MODE_GPIO(tms_gpio);
	swclk_gpio_mode = MODE_GPIO(swclk_gpio);
	swdio_gpio_mode = MODE_GPIO(swdio_gpio);
	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	INP_GPIO(tdo_gpio);

	GPIO_CLR = 1<<tdi_gpio | 1<<tck_gpio | 1<<swdio_gpio | 1<<swclk_gpio;
	GPIO_SET = 1<<tms_gpio;

	OUT_GPIO(tdi_gpio);
	OUT_GPIO(tck_gpio);
	OUT_GPIO(tms_gpio);
	OUT_GPIO(swclk_gpio);
	OUT_GPIO(swdio_gpio);
	if (trst_gpio != -1) {
		trst_gpio_mode = MODE_GPIO(trst_gpio);
		GPIO_SET = 1 << trst_gpio;
		OUT_GPIO(trst_gpio);
	}
	if (srst_gpio != -1) {
		srst_gpio_mode = MODE_GPIO(srst_gpio);
		GPIO_SET = 1 << srst_gpio;
		OUT_GPIO(srst_gpio);
	}

	LOG_DEBUG("saved pinmux settings: tck %d tms %d tdi %d "
		  "tdo %d trst %d srst %d", tck_gpio_mode, tms_gpio_mode,
		  tdi_gpio_mode, tdo_gpio_mode, trst_gpio_mode, srst_gpio_mode);

	if (swd_mode) {
		bcm2835gpio_bitbang.write = bcm2835gpio_swd_write;
		bitbang_switch_to_swd();
	}

	return ERROR_OK;
}

static int bcm2835gpio_quit(void)
{
	SET_MODE_GPIO(tdo_gpio, tdo_gpio_mode);
	SET_MODE_GPIO(tdi_gpio, tdi_gpio_mode);
	SET_MODE_GPIO(tck_gpio, tck_gpio_mode);
	SET_MODE_GPIO(tms_gpio, tms_gpio_mode);
	SET_MODE_GPIO(swclk_gpio, swclk_gpio_mode);
	SET_MODE_GPIO(swdio_gpio, swdio_gpio_mode);
	if (trst_gpio != -1)
		SET_MODE_GPIO(trst_gpio, trst_gpio_mode);
	if (srst_gpio != -1)
		SET_MODE_GPIO(srst_gpio, srst_gpio_mode);

	return ERROR_OK;
}
