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
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

uint32_t mt7688_peri_base = 0x10000600;

/* GPIO setup macros */
#define GPIO_REG(g, s) *(pio_base+((g)/32)+((s)/4))
#define GPIO_PIN(g) (1<<((g)%32))
#define MODE_GPIO(g) !!(GPIO_REG(g, 0x0) & GPIO_PIN(g))
#define INP_GPIO(g) do { GPIO_REG(g, 0x0) &= ~GPIO_PIN(g); } while (0)
#define OUT_GPIO(g) do { GPIO_REG(g, 0x0) |= GPIO_PIN(g); } while(0)
#define SET_MODE_GPIO(g, m) do { if ((m) == 0) { \
			INP_GPIO(g); \
		} else { \
			OUT_GPIO(g); \
		} } while(0)

#define GPIO_GET(g) !!(GPIO_REG(g, 0x20) & GPIO_PIN(g))
#define GPIO_SET(g, v) do { if ((v) == 0) { \
			GPIO_REG(g, 0x40) = GPIO_PIN(g); \
		} else { \
			GPIO_REG(g, 0x30) = GPIO_PIN(g); \
		} } while(0)

static int dev_mem_fd;
static volatile uint32_t *mapped_base;
static volatile uint32_t *pio_base;

static bb_value_t mt7688gpio_read(void);
static int mt7688gpio_write(int tck, int tms, int tdi);

static int mt7688_swdio_read(void);
static void mt7688_swdio_drive(bool is_output);
static int mt7688gpio_swd_write(int swclk, int swdio);

static int mt7688gpio_init(void);
static int mt7688gpio_quit(void);

static struct bitbang_interface mt7688gpio_bitbang = {
	.read = mt7688gpio_read,
	.write = mt7688gpio_write,
	.swdio_read = mt7688_swdio_read,
	.swdio_drive = mt7688_swdio_drive,
	.swd_write = mt7688gpio_swd_write,
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

static bb_value_t mt7688gpio_read(void)
{
	return GPIO_GET(tdo_gpio) ? BB_HIGH : BB_LOW;
}

static int mt7688gpio_write(int tck, int tms, int tdi)
{
	GPIO_SET(tck_gpio, tck);
	GPIO_SET(tms_gpio, tms);
	GPIO_SET(tdi_gpio, tdi);

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int mt7688gpio_swd_write(int swclk, int swdio)
{
	GPIO_SET(swclk_gpio, swclk);
	GPIO_SET(swdio_gpio, swdio);

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int mt7688gpio_reset(int trst, int srst)
{
	GPIO_SET(trst_gpio, trst == 0);
	GPIO_SET(srst_gpio, srst == 0);

	return ERROR_OK;
}

static void mt7688_swdio_drive(bool is_output)
{
	if (is_output)
		OUT_GPIO(swdio_gpio);
	else
		INP_GPIO(swdio_gpio);
}

static int mt7688_swdio_read(void)
{
	return GPIO_GET(swdio_gpio);
}

static int mt7688gpio_khz(int khz, int *jtag_speed)
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

static int mt7688gpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int mt7688gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio <= 96;
}

COMMAND_HANDLER(mt7688gpio_handle_jtag_gpionums)
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
			"MT7688 GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "MT7688 GPIO config: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "MT7688 GPIO config: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "MT7688 GPIO config: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "MT7688 GPIO config: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "MT7688 GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "MT7688 GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"MT7688 GPIO nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "MT7688 num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "MT7688 num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "MT7688 GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(mt7688gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], mt7688_peri_base);

	command_print(CMD, "MT7688 GPIO: peripheral_base = 0x%08x",
				  mt7688_peri_base);
	return ERROR_OK;
}

static const struct command_registration mt7688gpio_command_handlers[] = {
	{
		.name = "mt7688gpio_jtag_nums",
		.handler = &mt7688gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "mt7688gpio_tck_num",
		.handler = &mt7688gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "mt7688gpio_tms_num",
		.handler = &mt7688gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "mt7688gpio_tdo_num",
		.handler = &mt7688gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "mt7688gpio_tdi_num",
		.handler = &mt7688gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "mt7688gpio_swd_nums",
		.handler = &mt7688gpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "mt7688gpio_swclk_num",
		.handler = &mt7688gpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "mt7688gpio_swdio_num",
		.handler = &mt7688gpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "mt7688gpio_srst_num",
		.handler = &mt7688gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "mt7688gpio_trst_num",
		.handler = &mt7688gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "mt7688gpio_speed_coeffs",
		.handler = &mt7688gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "mt7688gpio_peripheral_base",
		.handler = &mt7688gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const char * const mt7688_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface mt7688gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver mt7688gpio_adapter_driver = {
	.name = "mt7688gpio",
	.transports = mt7688_transports,
	.commands = mt7688gpio_command_handlers,

	.init = mt7688gpio_init,
	.quit = mt7688gpio_quit,
	.reset = mt7688gpio_reset,
	.speed = mt7688gpio_speed,
	.khz = mt7688gpio_khz,
	.speed_div = mt7688gpio_speed_div,

	.jtag_ops = &mt7688gpio_interface,
	.swd_ops = &bitbang_swd,
};

static bool mt7688gpio_jtag_mode_possible(void)
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

static bool mt7688gpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return 0;
	if (!is_gpio_valid(swdio_gpio))
		return 0;
	return 1;
}

static int mt7688gpio_init(void)
{
	bitbang_interface = &mt7688gpio_bitbang;

	LOG_INFO("MT7688 GPIO JTAG/SWD bitbang driver");

	if (transport_is_jtag() && !mt7688gpio_jtag_mode_possible()) {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (transport_is_swd() && !mt7688gpio_swd_mode_possible()) {
		LOG_ERROR("Require swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	dev_mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_DEBUG("Cannot open /dev/gpiomem, fallback to /dev/mem");
		dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	}
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	long pagesize = sysconf(_SC_PAGE_SIZE);
	uint32_t aligned = mt7688_peri_base & ~(pagesize-1);
	uint32_t *mapped_base = mmap(NULL, pagesize, PROT_READ | PROT_WRITE,
								 MAP_SHARED, dev_mem_fd, aligned);
	pio_base = mapped_base + ((mt7688_peri_base - aligned) / sizeof(*mapped_base));
	if (pio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	if (transport_is_jtag()) {
		tdo_gpio_mode = MODE_GPIO(tdo_gpio);
		tdi_gpio_mode = MODE_GPIO(tdi_gpio);
		tck_gpio_mode = MODE_GPIO(tck_gpio);
		tms_gpio_mode = MODE_GPIO(tms_gpio);

		INP_GPIO(tdo_gpio);

		GPIO_SET(tdi_gpio, 0);
		GPIO_SET(tck_gpio, 0);
		GPIO_SET(tms_gpio, 1);

		OUT_GPIO(tdi_gpio);
		OUT_GPIO(tck_gpio);
		OUT_GPIO(tms_gpio);

		if (trst_gpio != -1) {
			trst_gpio_mode = MODE_GPIO(trst_gpio);
			GPIO_SET(trst_gpio, 1);
			OUT_GPIO(trst_gpio);
		}
	}

	if (transport_is_swd()) {
		swclk_gpio_mode = MODE_GPIO(swclk_gpio);
		swdio_gpio_mode = MODE_GPIO(swdio_gpio);

		GPIO_SET(swdio_gpio, 0);
		GPIO_SET(swclk_gpio, 0);

		OUT_GPIO(swclk_gpio);
		OUT_GPIO(swdio_gpio);
	}

	if (srst_gpio != -1) {
		srst_gpio_mode = MODE_GPIO(srst_gpio);
		GPIO_SET(srst_gpio, 1);
		OUT_GPIO(srst_gpio);
	}

	LOG_DEBUG("saved pinmux settings: tck %d tms %d tdi %d "
		  "tdo %d trst %d srst %d", tck_gpio_mode, tms_gpio_mode,
		  tdi_gpio_mode, tdo_gpio_mode, trst_gpio_mode, srst_gpio_mode);

	return ERROR_OK;
}

static int mt7688gpio_quit(void)
{
	if (transport_is_jtag()) {
		SET_MODE_GPIO(tdo_gpio, tdo_gpio_mode);
		SET_MODE_GPIO(tdi_gpio, tdi_gpio_mode);
		SET_MODE_GPIO(tck_gpio, tck_gpio_mode);
		SET_MODE_GPIO(tms_gpio, tms_gpio_mode);
		if (trst_gpio != -1)
			SET_MODE_GPIO(trst_gpio, trst_gpio_mode);
	}

	if (transport_is_swd()) {
		SET_MODE_GPIO(swclk_gpio, swclk_gpio_mode);
		SET_MODE_GPIO(swdio_gpio, swdio_gpio_mode);
	}

	if (srst_gpio != -1)
		SET_MODE_GPIO(srst_gpio, srst_gpio_mode);

	munmap(mapped_base, sysconf(_SC_PAGE_SIZE));
	close(dev_mem_fd);

	return ERROR_OK;
}
