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

uint32_t allwinner_lo_base = 0x01c20800;
uint32_t allwinner_hi_base = 0x01f02c00;

/* GPIO setup macros */
#define GPIO_BANK(g) (g > 10*32 ? hi_pio_base : lo_pio_base)
#define GPIO_CONF(g) (*(GPIO_BANK(g)+(((g)/32)*9)+(((g)%32)/8)))
#define GPIO_DATA(g) (*(GPIO_BANK(g)+4+(((g)/32)*9)))  /* sets   bits which are 1, ignores bits which are 0 */
#define GPIO_PIN(g) (1 << ((g) % 32))
#define MODE_GPIO(g) (GPIO_CONF(g)>>(((g)%8)*4) & 7)
#define INP_GPIO(g) do { GPIO_CONF(g) &= ~(7<<(((g)%8)*4)); } while (0)
#define SET_MODE_GPIO(g, m) do { /* clear the mode bits first, then set as necessary */ \
		INP_GPIO(g); \
		GPIO_CONF(g) |= ((m)<<(((g)%8)*4)); } while (0)
#define OUT_GPIO(g) SET_MODE_GPIO(g, 1)
#define GPIO_GET(g) !!(GPIO_DATA(g) & GPIO_PIN(g))
#define GPIO_SET(g, v) do { if ((v) == 0) { \
			GPIO_DATA(g) &= ~GPIO_PIN(g); \
		} else { \
			GPIO_DATA(g) |= GPIO_PIN(g); \
		} } while(0)

static int dev_mem_fd;
static volatile uint32_t *hi_pio_base, *lo_pio_base;
static volatile uint32_t *hi_mapped = 0, *lo_mapped = 0;

static bb_value_t allwinner_gpio_read(void);
static int allwinner_gpio_write(int tck, int tms, int tdi);

static int allwinner_gpio_swdio_read(void);
static void allwinner_gpio_swdio_drive(bool is_output);
static int allwinner_gpio_swd_write(int swclk, int swdio);

static int allwinner_gpio_init(void);
static int allwinner_gpio_quit(void);

static struct bitbang_interface allwinner_gpio_bitbang = {
	.read = allwinner_gpio_read,
	.write = allwinner_gpio_write,
	.swdio_read = allwinner_gpio_swdio_read,
	.swdio_drive = allwinner_gpio_swdio_drive,
	.swd_write = allwinner_gpio_swd_write,
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

static bb_value_t allwinner_gpio_read(void)
{
	return GPIO_GET(tdo_gpio) ? BB_HIGH : BB_LOW;
}

static int allwinner_gpio_write(int tck, int tms, int tdi)
{
	GPIO_SET(tck_gpio, tck);
	GPIO_SET(tms_gpio, tms);
	GPIO_SET(tdi_gpio, tdi);

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int allwinner_gpio_swd_write(int swclk, int swdio)
{
	GPIO_SET(swclk_gpio, swclk);
	GPIO_SET(swdio_gpio, swdio);

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int allwinner_gpio_reset(int trst, int srst)
{
	GPIO_SET(trst_gpio, trst == 0);
	GPIO_SET(srst_gpio, srst == 0);

	return ERROR_OK;
}

static void allwinner_gpio_swdio_drive(bool is_output)
{
	if (is_output)
		OUT_GPIO(swdio_gpio);
	else
		INP_GPIO(swdio_gpio);
}

static int allwinner_gpio_swdio_read(void)
{
	return GPIO_GET(swdio_gpio);
}

static int allwinner_gpio_khz(int khz, int *jtag_speed)
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

static int allwinner_gpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int allwinner_gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio <= 14 * 32 + 31;
}

static int parse_pin(const char* data, int *out) {
	if (data == NULL) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (*data == 'p' || *data == 'P') {
		data++;
	}

	int port = 0;
	if (*data >= 'A' && *data <= 'A' + 14) {
		port = *data - 'A';
	} else if (*data >= 'a' && *data <= 'a' + 14) {
		port = *data - 'a';
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	data++;
	int pin = 0;
	int result = parse_int(data, &pin);
	if (ERROR_OK != result) {
		return result;
	}
	*out = port * 32 + pin;
	return ERROR_OK;
}

static void pin_name(int pin, char* buffer) {
	buffer[0] = 'P';
	buffer[1] = 'A' + (pin / 32);
	buffer[2] = '0' + (pin % 32 / 10);
	buffer[3] = '0' + (pin % 32 % 10);
	buffer[4] = 0;
}

COMMAND_HANDLER(allwinner_gpio_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	char tck[5], tms[5], tdi[5], tdo[5];
	pin_name(tck_gpio, tck);
	pin_name(tms_gpio, tms);
	pin_name(tdi_gpio, tdi);
	pin_name(tdo_gpio, tdo);
	command_print(CMD,
			"Allwinner GPIO config: tck = %s, tms = %s, tdi = %s, tdo = %s",
			tck, tms, tdi, tdo);

	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], tck_gpio);

	char pin[5];
	pin_name(tck_gpio, pin);
	command_print(CMD, "Allwinner GPIO config: tck = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], tms_gpio);

	char pin[5];
	pin_name(tms_gpio, pin);
	command_print(CMD, "Allwinner GPIO config: tms = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], tdo_gpio);

	char pin[5];
	pin_name(tdo_gpio, pin);
	command_print(CMD, "Allwinner GPIO config: tdo = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], tdi_gpio);

	char pin[5];
	pin_name(tdi_gpio, pin);
	command_print(CMD, "Allwinner GPIO config: tdi = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], srst_gpio);

	char pin[5];
	pin_name(srst_gpio, pin);
	command_print(CMD, "Allwinner GPIO config: srst = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], trst_gpio);

	char pin[5];
	pin_name(trst_gpio, pin);
	command_print(CMD, "Allwinner GPIO config: trst = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	char swclk[5], swdio[5];
	pin_name(swclk_gpio, swclk);
	pin_name(swdio_gpio, swdio);
	command_print(CMD,
			"Allwinner GPIO nums: swclk = %s, swdio = %s",
			swclk, swdio);

	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], swclk_gpio);

	char pin[5];
	pin_name(swclk_gpio, pin);
	command_print(CMD, "Allwinner num: swclk = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(pin, CMD_ARGV[0], swdio_gpio);

	char pin[5];
	pin_name(swdio_gpio, pin);
	command_print(CMD, "Allwinner num: swdio = %s", pin);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "Allwinner GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(allwinner_gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], allwinner_lo_base);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], allwinner_hi_base);
	}

	command_print(CMD, "Allwinner GPIO: peripheral_base low ports = 0x%08x, high ports = 0x%08x",
				  allwinner_lo_base, allwinner_hi_base);
	return ERROR_OK;
}

static const struct command_registration allwinner_gpio_command_handlers[] = {
	{
		.name = "allwinner_gpio_jtag_nums",
		.handler = &allwinner_gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "allwinner_gpio_tck_num",
		.handler = &allwinner_gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "allwinner_gpio_tms_num",
		.handler = &allwinner_gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "allwinner_gpio_tdo_num",
		.handler = &allwinner_gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "allwinner_gpio_tdi_num",
		.handler = &allwinner_gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "allwinner_gpio_swd_nums",
		.handler = &allwinner_gpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "allwinner_gpio_swclk_num",
		.handler = &allwinner_gpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "allwinner_gpio_swdio_num",
		.handler = &allwinner_gpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "allwinner_gpio_srst_num",
		.handler = &allwinner_gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "allwinner_gpio_trst_num",
		.handler = &allwinner_gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "allwinner_gpio_speed_coeffs",
		.handler = &allwinner_gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "allwinner_gpio_peripheral_base",
		.handler = &allwinner_gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (H2+/H3/H5 0x01c20800 0x01f02c00, H6/H616 0x0300b000 0x07022000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const char * const allwinner_gpio_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface allwinner_gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver allwinner_gpio_adapter_driver = {
	.name = "allwinner_gpio",
	.transports = allwinner_gpio_transports,
	.commands = allwinner_gpio_command_handlers,

	.init = allwinner_gpio_init,
	.quit = allwinner_gpio_quit,
	.reset = allwinner_gpio_reset,
	.speed = allwinner_gpio_speed,
	.khz = allwinner_gpio_khz,
	.speed_div = allwinner_gpio_speed_div,

	.jtag_ops = &allwinner_gpio_interface,
	.swd_ops = &bitbang_swd,
};

static bool allwinner_gpio_jtag_mode_possible(void)
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

static bool allwinner_gpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return 0;
	if (!is_gpio_valid(swdio_gpio))
		return 0;
	return 1;
}

static int allwinner_gpio_init(void)
{
	bitbang_interface = &allwinner_gpio_bitbang;

	LOG_INFO("Allwinner GPIO JTAG/SWD bitbang driver");

	if (transport_is_jtag() && !allwinner_gpio_jtag_mode_possible()) {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (transport_is_swd() && !allwinner_gpio_swd_mode_possible()) {
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
	uint32_t lo_aligned = allwinner_lo_base & ~(pagesize-1);
	lo_mapped = mmap(NULL, pagesize, PROT_READ | PROT_WRITE,
					 MAP_SHARED, dev_mem_fd, lo_aligned);
	lo_pio_base = lo_mapped + ((allwinner_lo_base - lo_aligned) / sizeof(*lo_mapped));

	if (lo_pio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	uint32_t hi_aligned = allwinner_hi_base & ~(pagesize - 1);
	hi_mapped = mmap(NULL, pagesize, PROT_READ | PROT_WRITE,
					 MAP_SHARED, dev_mem_fd, hi_aligned);
	hi_pio_base = hi_mapped + ((allwinner_hi_base - hi_aligned) / sizeof(*hi_mapped));

	if (hi_pio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		munmap((void*)lo_mapped, pagesize);
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

static int allwinner_gpio_quit(void)
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

	long pagesize = sysconf(_SC_PAGE_SIZE);
	munmap((void*)lo_mapped, pagesize);
	munmap((void*)hi_mapped, pagesize);
	close(dev_mem_fd);

	return ERROR_OK;
}
