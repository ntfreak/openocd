/***************************************************************************
 *   Copyright (C) 2017 by Grzegorz Kostka, kostka.grzegorz@gmail.com      *
 *                                                                         *
 *   Based on bcm2835gpio.c                                                 *
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

#define IMX_GPIO_BASE 0x0209c000
#define IMX_GPIO_SIZE 0x00004000
#define IMX_GPIO_REGS_COUNT 8

static uint32_t imx_gpio_peri_base = IMX_GPIO_BASE;

struct imx_gpio_regs {
	uint32_t dr;
	uint32_t gdir;
	uint32_t psr;
	uint32_t icr1;
	uint32_t icr2;
	uint32_t imr;
	uint32_t isr;
	uint32_t edge_sel;
} __attribute__((aligned(IMX_GPIO_SIZE)));

static int dev_mem_fd;
static volatile struct imx_gpio_regs *pio_base;

/* GPIO setup functions */
static inline bool gpio_mode_get(int g)
{
	return pio_base[g / 32].gdir >> (g & 0x1F) & 1;
}

static inline void gpio_mode_input_set(int g)
{
	pio_base[g / 32].gdir &=  ~(1u << (g & 0x1F));
}

static inline void gpio_mode_output_set(int g)
{
	pio_base[g / 32].gdir |=  (1u << (g & 0x1F));
}

static inline void gpio_mode_set(int g, int m)
{
	(m) ? gpio_mode_output_set(g) : gpio_mode_input_set(g);
}

static inline void gpio_set(int g)
{
	pio_base[g / 32].dr |=  (1u << (g & 0x1F));
}

static inline void gpio_clear(int g)
{
	pio_base[g / 32].dr &=  ~(1u << (g & 0x1F));
}

static inline bool gpio_level(int g)
{
	return pio_base[g / 32].dr >> (g & 0x1F) & 1;
}

static bb_value_t imx_gpio_read(void);
static int imx_gpio_write(int tck, int tms, int tdi);

static int imx_gpio_swdio_read(void);
static void imx_gpio_swdio_drive(bool is_output);

static int imx_gpio_init(void);
static int imx_gpio_quit(void);

static struct bitbang_interface imx_gpio_bitbang = {
	.read = imx_gpio_read,
	.write = imx_gpio_write,
	.swdio_read = imx_gpio_swdio_read,
	.swdio_drive = imx_gpio_swdio_drive,
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

/* Transition delay coefficients. Tuned for IMX6UL 528MHz. Adjusted
 * experimentally for:10kHz, 100Khz, 500KHz. Speeds above 800Khz are impossible
 * to reach via memory mapped method (at least for IMX6UL@528MHz).
 * Measured mmap raw GPIO toggling speed on IMX6UL@528MHz: 1.3MHz.
 */
static int speed_coeff = 50000;
static int speed_offset = 100;
static unsigned int jtag_delay;

static bb_value_t imx_gpio_read(void)
{
	return gpio_level(tdo_gpio) ? BB_HIGH : BB_LOW;
}

static int imx_gpio_write(int tck, int tms, int tdi)
{
	tms ? gpio_set(tms_gpio) : gpio_clear(tms_gpio);
	tdi ? gpio_set(tdi_gpio) : gpio_clear(tdi_gpio);
	tck ? gpio_set(tck_gpio) : gpio_clear(tck_gpio);

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int imx_gpio_swd_write(int tck, int tms, int tdi)
{
	tdi ? gpio_set(swdio_gpio) : gpio_clear(swdio_gpio);
	tck ? gpio_set(swclk_gpio) : gpio_clear(swclk_gpio);

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int imx_gpio_reset(int trst, int srst)
{
	if (trst_gpio != -1)
		trst ? gpio_clear(trst_gpio) : gpio_set(trst_gpio);

	if (srst_gpio != -1)
		srst ? gpio_clear(srst_gpio) : gpio_set(srst_gpio);

	return ERROR_OK;
}

static void imx_gpio_swdio_drive(bool is_output)
{
	if (is_output)
		gpio_mode_output_set(swdio_gpio);
	else
		gpio_mode_input_set(swdio_gpio);
}

static int imx_gpio_swdio_read(void)
{
	return gpio_level(swdio_gpio);
}

static int imx_gpio_khz(int khz, int *jtag_speed)
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

static int imx_gpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int imx_gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio < 32 * IMX_GPIO_REGS_COUNT;
}

COMMAND_HANDLER(imx_gpio_handle_jtag_gpionums)
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
			"imx_gpio GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "imx_gpio GPIO config: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "imx_gpio GPIO config: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "imx_gpio GPIO config: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "imx_gpio GPIO config: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "imx_gpio GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "imx_gpio GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"imx_gpio GPIO nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "imx_gpio num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "imx_gpio num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "imx_gpio: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(imx_gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], imx_gpio_peri_base);

	command_print(CMD, "imx_gpio: peripheral_base = 0x%08x",
				  imx_gpio_peri_base);
	return ERROR_OK;
}

static const struct command_registration imx_gpio_command_handlers[] = {
	{
		.name = "imx_gpio_jtag_nums",
		.handler = &imx_gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "imx_gpio_tck_num",
		.handler = &imx_gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "imx_gpio_tms_num",
		.handler = &imx_gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "imx_gpio_tdo_num",
		.handler = &imx_gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "imx_gpio_tdi_num",
		.handler = &imx_gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "imx_gpio_swd_nums",
		.handler = &imx_gpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "imx_gpio_swclk_num",
		.handler = &imx_gpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "imx_gpio_swdio_num",
		.handler = &imx_gpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "imx_gpio_srst_num",
		.handler = &imx_gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "imx_gpio_trst_num",
		.handler = &imx_gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "imx_gpio_speed_coeffs",
		.handler = &imx_gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "imx_gpio_peripheral_base",
		.handler = &imx_gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (0x0209c000 for most IMX).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const char * const imx_gpio_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface imx_gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver imx_gpio_adapter_driver = {
	.name = "imx_gpio",
	.transports = imx_gpio_transports,
	.commands = imx_gpio_command_handlers,

	.init = imx_gpio_init,
	.quit = imx_gpio_quit,
	.reset = imx_gpio_reset,
	.speed = imx_gpio_speed,
	.khz = imx_gpio_khz,
	.speed_div = imx_gpio_speed_div,

	.jtag_ops = &imx_gpio_interface,
	.swd_ops = &bitbang_swd,
};

static bool imx_gpio_jtag_mode_possible(void)
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

static bool imx_gpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return 0;
	if (!is_gpio_valid(swdio_gpio))
		return 0;
	return 1;
}

static int imx_gpio_init(void)
{
	bitbang_interface = &imx_gpio_bitbang;

	LOG_INFO("imx_gpio GPIO JTAG/SWD bitbang driver");

	if (imx_gpio_jtag_mode_possible()) {
		if (imx_gpio_swd_mode_possible())
			LOG_INFO("JTAG and SWD modes enabled");
		else
			LOG_INFO("JTAG only mode enabled (specify swclk and swdio gpio to add SWD mode)");
	} else if (imx_gpio_swd_mode_possible()) {
		LOG_INFO("SWD only mode enabled (specify tck, tms, tdi and tdo gpios to add JTAG mode)");
	} else {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode and/or swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		perror("open");
		return ERROR_JTAG_INIT_FAILED;
	}


	LOG_INFO("imx_gpio mmap: pagesize: %u, regionsize: %u",
			(unsigned int) sysconf(_SC_PAGE_SIZE), IMX_GPIO_REGS_COUNT * IMX_GPIO_SIZE);
	pio_base = mmap(NULL, IMX_GPIO_REGS_COUNT * IMX_GPIO_SIZE,
				PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, imx_gpio_peri_base);

	if (pio_base == MAP_FAILED) {
		perror("mmap");
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	if (imx_gpio_jtag_mode_possible()) {
		tdo_gpio_mode = gpio_mode_get(tdo_gpio);
		tdi_gpio_mode = gpio_mode_get(tdi_gpio);
		tck_gpio_mode = gpio_mode_get(tck_gpio);
		tms_gpio_mode = gpio_mode_get(tms_gpio);

		gpio_clear(tdi_gpio);
		gpio_clear(tck_gpio);
		gpio_set(tms_gpio);

		gpio_mode_input_set(tdo_gpio);
		gpio_mode_output_set(tdi_gpio);
		gpio_mode_output_set(tck_gpio);
		gpio_mode_output_set(tms_gpio);
	}
	if (imx_gpio_swd_mode_possible()) {
		swclk_gpio_mode = gpio_mode_get(swclk_gpio);
		swdio_gpio_mode = gpio_mode_get(swdio_gpio);

		gpio_clear(swdio_gpio);
		gpio_clear(swclk_gpio);
		gpio_mode_output_set(swclk_gpio);
		gpio_mode_output_set(swdio_gpio);
	}
	if (trst_gpio != -1) {
		trst_gpio_mode = gpio_mode_get(trst_gpio);
		gpio_set(trst_gpio);
		gpio_mode_output_set(trst_gpio);
	}
	if (srst_gpio != -1) {
		srst_gpio_mode = gpio_mode_get(srst_gpio);
		gpio_set(srst_gpio);
		gpio_mode_output_set(srst_gpio);
	}

	LOG_DEBUG("saved pinmux settings: tck %d tms %d tdi %d "
		  "tdo %d trst %d srst %d", tck_gpio_mode, tms_gpio_mode,
		  tdi_gpio_mode, tdo_gpio_mode, trst_gpio_mode, srst_gpio_mode);

	if (swd_mode) {
		imx_gpio_bitbang.write = imx_gpio_swd_write;
		bitbang_switch_to_swd();
	}

	return ERROR_OK;
}

static int imx_gpio_quit(void)
{
	if (imx_gpio_jtag_mode_possible()) {
		gpio_mode_set(tdo_gpio, tdo_gpio_mode);
		gpio_mode_set(tdi_gpio, tdi_gpio_mode);
		gpio_mode_set(tck_gpio, tck_gpio_mode);
		gpio_mode_set(tms_gpio, tms_gpio_mode);
	}
	if (imx_gpio_swd_mode_possible()) {
		gpio_mode_set(swclk_gpio, swclk_gpio_mode);
		gpio_mode_set(swdio_gpio, swdio_gpio_mode);
	}
	if (trst_gpio != -1)
		gpio_mode_set(trst_gpio, trst_gpio_mode);
	if (srst_gpio != -1)
		gpio_mode_set(srst_gpio, srst_gpio_mode);

	return ERROR_OK;
}
