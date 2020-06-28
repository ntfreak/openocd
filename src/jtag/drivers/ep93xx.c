/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#define TDO_BIT		1
#define TDI_BIT		2
#define TCK_BIT		4
#define TMS_BIT		8
#define TRST_BIT	16
#define SRST_BIT	32
#define VCC_BIT		64

#include <sys/mman.h>

static uint8_t output_value;
static int dev_mem_fd;
static void *gpio_controller;
static volatile uint8_t *gpio_data_register;
static volatile uint8_t *gpio_data_direction_register;

/* low level command set
 */
static bb_value_t ep93xx_read(void);
static int ep93xx_write(int tck, int tms, int tdi);
static int ep93xx_reset(int trst, int srst);

static int ep93xx_init(void);
static int ep93xx_quit(void);

struct timespec ep93xx_zzzz;

static struct jtag_interface ep93xx_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver ep93xx_adapter_driver = {
	.name = "ep93xx",
	.transports = jtag_only,

	.init = ep93xx_init,
	.quit = ep93xx_quit,
	.reset = ep93xx_reset,

	.jtag_ops = &ep93xx_interface,
};

static struct bitbang_interface ep93xx_bitbang = {
	.read = ep93xx_read,
	.write = ep93xx_write,
	.blink = 0,
};

static bb_value_t ep93xx_read(void)
{
	return (*gpio_data_register & TDO_BIT) ? BB_HIGH : BB_LOW;
}

static int ep93xx_write(int tck, int tms, int tdi)
{
	if (tck)
		output_value |= TCK_BIT;
	else
		output_value &= ~TCK_BIT;

	if (tms)
		output_value |= TMS_BIT;
	else
		output_value &= ~TMS_BIT;

	if (tdi)
		output_value |= TDI_BIT;
	else
		output_value &= ~TDI_BIT;

	*gpio_data_register = output_value;
	nanosleep(&ep93xx_zzzz, NULL);

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int ep93xx_reset(int trst, int srst)
{
	if (trst == 0)
		output_value |= TRST_BIT;
	else if (trst == 1)
		output_value &= ~TRST_BIT;

	if (srst == 0)
		output_value |= SRST_BIT;
	else if (srst == 1)
		output_value &= ~SRST_BIT;

	*gpio_data_register = output_value;
	nanosleep(&ep93xx_zzzz, NULL);

	return ERROR_OK;
}

static int set_gonk_mode(void)
{
	void *syscon;
	uint32_t devicecfg;

	syscon = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
			MAP_SHARED, dev_mem_fd, 0x80930000);
	if (syscon == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	devicecfg = *((volatile int *)(syscon + 0x80));
	*((volatile int *)(syscon + 0xc0)) = 0xaa;
	*((volatile int *)(syscon + 0x80)) = devicecfg | 0x08000000;

	munmap(syscon, 4096);

	return ERROR_OK;
}

static int ep93xx_init(void)
{
	int ret;

	bitbang_interface = &ep93xx_bitbang;

	ep93xx_zzzz.tv_sec = 0;
	ep93xx_zzzz.tv_nsec = 10000000;

	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	gpio_controller = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, 0x80840000);
	if (gpio_controller == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	ret = set_gonk_mode();
	if (ret != ERROR_OK) {
		munmap(gpio_controller, 4096);
		close(dev_mem_fd);
		return ret;
	}

#if 0
	/* Use GPIO port A.  */
	gpio_data_register = gpio_controller + 0x00;
	gpio_data_direction_register = gpio_controller + 0x10;


	/* Use GPIO port B.  */
	gpio_data_register = gpio_controller + 0x04;
	gpio_data_direction_register = gpio_controller + 0x14;

	/* Use GPIO port C.  */
	gpio_data_register = gpio_controller + 0x08;
	gpio_data_direction_register = gpio_controller + 0x18;

	/* Use GPIO port D.  */
	gpio_data_register = gpio_controller + 0x0c;
	gpio_data_direction_register = gpio_controller + 0x1c;
#endif

	/* Use GPIO port C.  */
	gpio_data_register = gpio_controller + 0x08;
	gpio_data_direction_register = gpio_controller + 0x18;

	LOG_INFO("gpio_data_register      = %p", gpio_data_register);
	LOG_INFO("gpio_data_direction_reg = %p", gpio_data_direction_register);
	/*
	 * Configure bit 0 (TDO) as an input, and bits 1-5 (TDI, TCK
	 * TMS, TRST, SRST) as outputs.  Drive TDI and TCK low, and
	 * TMS/TRST/SRST high.
	 */
	output_value = TMS_BIT | TRST_BIT | SRST_BIT | VCC_BIT;
	*gpio_data_register = output_value;
	nanosleep(&ep93xx_zzzz, NULL);

	/*
	 * Configure the direction register.  1 = output, 0 = input.
	 */
	*gpio_data_direction_register =
		TDI_BIT | TCK_BIT | TMS_BIT | TRST_BIT | SRST_BIT | VCC_BIT;

	nanosleep(&ep93xx_zzzz, NULL);
	return ERROR_OK;
}

static int ep93xx_quit(void)
{

	return ERROR_OK;
}
