/***************************************************************************
 *   Copyright (C) 2006 by Anders Larsen                                   *
 *   al@alarsen.net                                                        *
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

/* AT91RM9200 */
#define AT91C_BASE_SYS	(0xfffff000)

/* GPIO assignment */
#define PIOA	(0 << 7)
#define PIOB	(1 << 7)
#define PIOC	(2 << 7)
#define PIOD	(3 << 7)

#define PIO_PER		(0)		/* PIO enable */
#define PIO_OER		(4)		/* output enable */
#define PIO_ODR		(5)		/* output disable */
#define PIO_SODR	(12)		/* set output data */
#define PIO_CODR	(13)		/* clear output data */
#define PIO_PDSR	(15)		/* pin data status */
#define PIO_PPUER	(25)		/* pull-up enable */

#define NC	(0)			/* not connected */
#define P0	(1 << 0)
#define P1	(1 << 1)
#define P2	(1 << 2)
#define P3	(1 << 3)
#define P4	(1 << 4)
#define P5	(1 << 5)
#define P6	(1 << 6)
#define P7	(1 << 7)
#define P8	(1 << 8)
#define P9	(1 << 9)
#define P10	(1 << 10)
#define P11	(1 << 11)
#define P12	(1 << 12)
#define P13	(1 << 13)
#define P14	(1 << 14)
#define P15	(1 << 15)
#define P16	(1 << 16)
#define P17	(1 << 17)
#define P18	(1 << 18)
#define P19	(1 << 19)
#define P20	(1 << 20)
#define P21	(1 << 21)
#define P22	(1 << 22)
#define P23	(1 << 23)
#define P24	(1 << 24)
#define P25	(1 << 25)
#define P26	(1 << 26)
#define P27	(1 << 27)
#define P28	(1 << 28)
#define P29	(1 << 29)
#define P30	(1 << 30)
#define P31	(1 << 31)

struct device_t {
	const char *name;
	int TDO_PIO;	/* PIO holding TDO */
	uint32_t TDO_MASK;	/* TDO bitmask */
	int TRST_PIO;	/* PIO holding TRST */
	uint32_t TRST_MASK;	/* TRST bitmask */
	int TMS_PIO;	/* PIO holding TMS */
	uint32_t TMS_MASK;	/* TMS bitmask */
	int TCK_PIO;	/* PIO holding TCK */
	uint32_t TCK_MASK;	/* TCK bitmask */
	int TDI_PIO;	/* PIO holding TDI */
	uint32_t TDI_MASK;	/* TDI bitmask */
	int SRST_PIO;	/* PIO holding SRST */
	uint32_t SRST_MASK;	/* SRST bitmask */
};

static const struct device_t devices[] = {
	{ "rea_ecr", PIOD, P27, PIOA, NC, PIOD, P23, PIOD, P24, PIOD, P26, PIOC, P5 },
	{ .name = NULL },
};

/* configuration */
static char *at91rm9200_device;

/* interface variables
 */
static const struct device_t *device;
static int dev_mem_fd;
static void *sys_controller;
static uint32_t *pio_base;

/* low level command set
 */
static bb_value_t at91rm9200_read(void);
static int at91rm9200_write(int tck, int tms, int tdi);

static int at91rm9200_init(void);
static int at91rm9200_quit(void);

static struct bitbang_interface at91rm9200_bitbang = {
	.read = at91rm9200_read,
	.write = at91rm9200_write,
	.blink = 0
};

static bb_value_t at91rm9200_read(void)
{
	return (pio_base[device->TDO_PIO + PIO_PDSR] & device->TDO_MASK) ? BB_HIGH : BB_LOW;
}

static int at91rm9200_write(int tck, int tms, int tdi)
{
	if (tck)
		pio_base[device->TCK_PIO + PIO_SODR] = device->TCK_MASK;
	else
		pio_base[device->TCK_PIO + PIO_CODR] = device->TCK_MASK;

	if (tms)
		pio_base[device->TMS_PIO + PIO_SODR] = device->TMS_MASK;
	else
		pio_base[device->TMS_PIO + PIO_CODR] = device->TMS_MASK;

	if (tdi)
		pio_base[device->TDI_PIO + PIO_SODR] = device->TDI_MASK;
	else
		pio_base[device->TDI_PIO + PIO_CODR] = device->TDI_MASK;

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int at91rm9200_reset(int trst, int srst)
{
	if (trst == 0)
		pio_base[device->TRST_PIO + PIO_SODR] = device->TRST_MASK;
	else if (trst == 1)
		pio_base[device->TRST_PIO + PIO_CODR] = device->TRST_MASK;

	if (srst == 0)
		pio_base[device->SRST_PIO + PIO_SODR] = device->SRST_MASK;
	else if (srst == 1)
		pio_base[device->SRST_PIO + PIO_CODR] = device->SRST_MASK;

	return ERROR_OK;
}

COMMAND_HANDLER(at91rm9200_handle_device_command)
{
	if (CMD_ARGC == 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* only if the device name wasn't overwritten by cmdline */
	if (at91rm9200_device == 0) {
		at91rm9200_device = malloc(strlen(CMD_ARGV[0]) + sizeof(char));
		strcpy(at91rm9200_device, CMD_ARGV[0]);
	}

	return ERROR_OK;
}

static const struct command_registration at91rm9200_command_handlers[] = {
	{
		.name = "at91rm9200_device",
		.handler = &at91rm9200_handle_device_command,
		.mode = COMMAND_CONFIG,
		.help = "Set at91rm9200 device [default \"rea_ecr\"]",
		.usage = "<device>",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface at91rm9200_interface = {
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver at91rm9200_adapter_driver = {
	.name = "at91rm9200",
	.transports = jtag_only,
	.commands = at91rm9200_command_handlers,

	.init = at91rm9200_init,
	.quit = at91rm9200_quit,
	.reset = at91rm9200_reset,

	.jtag_ops = &at91rm9200_interface,
};

static int at91rm9200_init(void)
{
	const struct device_t *cur_device;

	cur_device = devices;

	if (!at91rm9200_device || at91rm9200_device[0] == 0) {
		at91rm9200_device = "rea_ecr";
		LOG_WARNING("No at91rm9200 device specified, using default 'rea_ecr'");
	}

	while (cur_device->name) {
		if (strcmp(cur_device->name, at91rm9200_device) == 0) {
			device = cur_device;
			break;
		}
		cur_device++;
	}

	if (!device) {
		LOG_ERROR("No matching device found for %s", at91rm9200_device);
		return ERROR_JTAG_INIT_FAILED;
	}

	bitbang_interface = &at91rm9200_bitbang;

	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	sys_controller = mmap(NULL, 4096, PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, AT91C_BASE_SYS);
	if (sys_controller == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}
	pio_base = (uint32_t *)sys_controller + 0x100;

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	pio_base[device->TDI_PIO + PIO_CODR] = device->TDI_MASK;
	pio_base[device->TDI_PIO + PIO_OER] = device->TDI_MASK;
	pio_base[device->TDI_PIO + PIO_PER] = device->TDI_MASK;
	pio_base[device->TCK_PIO + PIO_CODR] = device->TCK_MASK;
	pio_base[device->TCK_PIO + PIO_OER] = device->TCK_MASK;
	pio_base[device->TCK_PIO + PIO_PER] = device->TCK_MASK;
	pio_base[device->TMS_PIO + PIO_SODR] = device->TMS_MASK;
	pio_base[device->TMS_PIO + PIO_OER] = device->TMS_MASK;
	pio_base[device->TMS_PIO + PIO_PER] = device->TMS_MASK;
	pio_base[device->TRST_PIO + PIO_SODR] = device->TRST_MASK;
	pio_base[device->TRST_PIO + PIO_OER] = device->TRST_MASK;
	pio_base[device->TRST_PIO + PIO_PER] = device->TRST_MASK;
	pio_base[device->SRST_PIO + PIO_SODR] = device->SRST_MASK;
	pio_base[device->SRST_PIO + PIO_OER] = device->SRST_MASK;
	pio_base[device->SRST_PIO + PIO_PER] = device->SRST_MASK;
	pio_base[device->TDO_PIO + PIO_ODR] = device->TDO_MASK;
	pio_base[device->TDO_PIO + PIO_PPUER] = device->TDO_MASK;
	pio_base[device->TDO_PIO + PIO_PER] = device->TDO_MASK;

	return ERROR_OK;
}

static int at91rm9200_quit(void)
{

	return ERROR_OK;
}
