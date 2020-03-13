/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2002 Thomas Gleixner <tglx@linutronix.de>               *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *                                                                         *
 *   Partially based on drivers/mtd/nand_ids.c from Linux.                 *
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

#include "imp.h"

/* configured NAND devices and NAND Flash command handler */
struct nand_device *nand_devices;

void nand_device_add(struct nand_device *c)
{
	if (nand_devices) {
		struct nand_device *p = nand_devices;
		while (p && p->next)
			p = p->next;
		p->next = c;
	} else
		nand_devices = c;
}


/*	Chip ID list
 *
 *	Manufacturer, ID code, pagesize, chipsize in MegaByte, eraseblock size,
 *	options, name
 *
 *	Pagesize; 0, 256, 512
 *	0	get this information from the extended chip ID
 *	256	256 Byte page size
 *	512	512 Byte page size
 */
static struct nand_info nand_flash_ids[] = {
	/* Vendor Specific Entries */
	{ NAND_MFR_SAMSUNG,     0xD5, 8192, 2048, 0x100000, LP_OPTIONS,
	  "K9GAG08 2GB NAND 3.3V x8 MLC 2b/cell"},
	{ NAND_MFR_SAMSUNG,     0xD7, 8192, 4096, 0x100000, LP_OPTIONS,
	  "K9LBG08 4GB NAND 3.3V x8 MLC 2b/cell"},

	/* start "museum" IDs */
	{ 0x0,                  0x6e, 256, 1, 0x1000, 0,                "NAND 1MiB 5V 8-bit"},
	{ 0x0,                  0x64, 256, 2, 0x1000, 0,                "NAND 2MiB 5V 8-bit"},
	{ 0x0,                  0x6b, 512, 4, 0x2000, 0,                "NAND 4MiB 5V 8-bit"},
	{ 0x0,                  0xe8, 256, 1, 0x1000, 0,                "NAND 1MiB 3.3V 8-bit"},
	{ 0x0,                  0xec, 256, 1, 0x1000, 0,                "NAND 1MiB 3.3V 8-bit"},
	{ 0x0,                  0xea, 256, 2, 0x1000, 0,                "NAND 2MiB 3.3V 8-bit"},
	{ 0x0,                  0xd5, 512, 4, 0x2000, 0,                "NAND 4MiB 3.3V 8-bit"},
	{ 0x0,                  0xe3, 512, 4, 0x2000, 0,                "NAND 4MiB 3.3V 8-bit"},
	{ 0x0,                  0xe5, 512, 4, 0x2000, 0,                "NAND 4MiB 3.3V 8-bit"},
	{ 0x0,                  0xd6, 512, 8, 0x2000, 0,                "NAND 8MiB 3.3V 8-bit"},

	{ 0x0,                  0x39, 512, 8, 0x2000, 0,                "NAND 8MiB 1.8V 8-bit"},
	{ 0x0,                  0xe6, 512, 8, 0x2000, 0,                "NAND 8MiB 3.3V 8-bit"},
	{ 0x0,                  0x49, 512, 8, 0x2000, NAND_BUSWIDTH_16, "NAND 8MiB 1.8V 16-bit"},
	{ 0x0,                  0x59, 512, 8, 0x2000, NAND_BUSWIDTH_16, "NAND 8MiB 3.3V 16-bit"},
	/* end "museum" IDs */

	{ 0x0,                  0x33, 512, 16, 0x4000, 0,               "NAND 16MiB 1.8V 8-bit"},
	{ 0x0,                  0x73, 512, 16, 0x4000, 0,               "NAND 16MiB 3.3V 8-bit"},
	{ 0x0,                  0x43, 512, 16, 0x4000, NAND_BUSWIDTH_16, "NAND 16MiB 1.8V 16-bit"},
	{ 0x0,                  0x53, 512, 16, 0x4000, NAND_BUSWIDTH_16, "NAND 16MiB 3.3V 16-bit"},

	{ 0x0,                  0x35, 512, 32, 0x4000, 0,               "NAND 32MiB 1.8V 8-bit"},
	{ 0x0,                  0x75, 512, 32, 0x4000, 0,               "NAND 32MiB 3.3V 8-bit"},
	{ 0x0,                  0x45, 512, 32, 0x4000, NAND_BUSWIDTH_16, "NAND 32MiB 1.8V 16-bit"},
	{ 0x0,                  0x55, 512, 32, 0x4000, NAND_BUSWIDTH_16, "NAND 32MiB 3.3V 16-bit"},

	{ 0x0,                  0x36, 512, 64, 0x4000, 0,               "NAND 64MiB 1.8V 8-bit"},
	{ 0x0,                  0x76, 512, 64, 0x4000, 0,               "NAND 64MiB 3.3V 8-bit"},
	{ 0x0,                  0x46, 512, 64, 0x4000, NAND_BUSWIDTH_16, "NAND 64MiB 1.8V 16-bit"},
	{ 0x0,                  0x56, 512, 64, 0x4000, NAND_BUSWIDTH_16, "NAND 64MiB 3.3V 16-bit"},

	{ 0x0,                  0x78, 512, 128, 0x4000, 0,              "NAND 128MiB 1.8V 8-bit"},
	{ 0x0,                  0x39, 512, 128, 0x4000, 0,              "NAND 128MiB 1.8V 8-bit"},
	{ 0x0,                  0x79, 512, 128, 0x4000, 0,              "NAND 128MiB 3.3V 8-bit"},
	{ 0x0,                  0x72, 512, 128, 0x4000, NAND_BUSWIDTH_16, "NAND 128MiB 1.8V 16-bit"},
	{ 0x0,                  0x49, 512, 128, 0x4000, NAND_BUSWIDTH_16, "NAND 128MiB 1.8V 16-bit"},
	{ 0x0,                  0x74, 512, 128, 0x4000, NAND_BUSWIDTH_16, "NAND 128MiB 3.3V 16-bit"},
	{ 0x0,                  0x59, 512, 128, 0x4000, NAND_BUSWIDTH_16, "NAND 128MiB 3.3V 16-bit"},

	{ 0x0,                  0x71, 512, 256, 0x4000, 0,              "NAND 256MiB 3.3V 8-bit"},

	{ 0x0,                  0xA2, 0,  64, 0, LP_OPTIONS,            "NAND 64MiB 1.8V 8-bit"},
	{ 0x0,                  0xF2, 0,  64, 0, LP_OPTIONS,            "NAND 64MiB 3.3V 8-bit"},
	{ 0x0,                  0xB2, 0,  64, 0, LP_OPTIONS16,          "NAND 64MiB 1.8V 16-bit"},
	{ 0x0,                  0xC2, 0,  64, 0, LP_OPTIONS16,          "NAND 64MiB 3.3V 16-bit"},

	{ 0x0,                  0xA1, 0, 128, 0, LP_OPTIONS,            "NAND 128MiB 1.8V 8-bit"},
	{ 0x0,                  0xF1, 0, 128, 0, LP_OPTIONS,            "NAND 128MiB 3.3V 8-bit"},
	{ 0x0,                  0xB1, 0, 128, 0, LP_OPTIONS16,          "NAND 128MiB 1.8V 16-bit"},
	{ 0x0,                  0xC1, 0, 128, 0, LP_OPTIONS16,          "NAND 128MiB 3.3V 16-bit"},

	{ 0x0,                  0xAA, 0, 256, 0, LP_OPTIONS,            "NAND 256MiB 1.8V 8-bit"},
	{ 0x0,                  0xDA, 0, 256, 0, LP_OPTIONS,            "NAND 256MiB 3.3V 8-bit"},
	{ 0x0,                  0xBA, 0, 256, 0, LP_OPTIONS16,          "NAND 256MiB 1.8V 16-bit"},
	{ 0x0,                  0xCA, 0, 256, 0, LP_OPTIONS16,          "NAND 256MiB 3.3V 16-bit"},

	{ 0x0,                  0xAC, 0, 512, 0, LP_OPTIONS,            "NAND 512MiB 1.8V 8-bit"},
	{ 0x0,                  0xDC, 0, 512, 0, LP_OPTIONS,            "NAND 512MiB 3.3V 8-bit"},
	{ 0x0,                  0xBC, 0, 512, 0, LP_OPTIONS16,          "NAND 512MiB 1.8V 16-bit"},
	{ 0x0,                  0xCC, 0, 512, 0, LP_OPTIONS16,          "NAND 512MiB 3.3V 16-bit"},

	{ 0x0,                  0xA3, 0, 1024, 0, LP_OPTIONS,           "NAND 1GiB 1.8V 8-bit"},
	{ 0x0,                  0xD3, 0, 1024, 0, LP_OPTIONS,           "NAND 1GiB 3.3V 8-bit"},
	{ 0x0,                  0xB3, 0, 1024, 0, LP_OPTIONS16,         "NAND 1GiB 1.8V 16-bit"},
	{ 0x0,                  0xC3, 0, 1024, 0, LP_OPTIONS16,         "NAND 1GiB 3.3V 16-bit"},

	{ 0x0,                  0xA5, 0, 2048, 0, LP_OPTIONS,           "NAND 2GiB 1.8V 8-bit"},
	{ 0x0,                  0xD5, 0, 8192, 0, LP_OPTIONS,           "NAND 2GiB 3.3V 8-bit"},
	{ 0x0,                  0xB5, 0, 2048, 0, LP_OPTIONS16,         "NAND 2GiB 1.8V 16-bit"},
	{ 0x0,                  0xC5, 0, 2048, 0, LP_OPTIONS16,         "NAND 2GiB 3.3V 16-bit"},

	{ 0x0,                  0x48, 0, 2048, 0, LP_OPTIONS,           "NAND 2GiB 3.3V 8-bit"},

	{0, 0, 0, 0, 0, 0, NULL}
};

/* Manufacturer ID list
 */
static struct nand_manufacturer nand_manuf_ids[] = {
	{0x0, "unknown"},
	{NAND_MFR_TOSHIBA, "Toshiba"},
	{NAND_MFR_SAMSUNG, "Samsung"},
	{NAND_MFR_FUJITSU, "Fujitsu"},
	{NAND_MFR_NATIONAL, "National"},
	{NAND_MFR_RENESAS, "Renesas"},
	{NAND_MFR_STMICRO, "ST Micro"},
	{NAND_MFR_HYNIX, "Hynix"},
	{NAND_MFR_MICRON, "Micron"},
	{0x0, NULL},
};

/*
 * Define default oob placement schemes for large and small page devices
 */

#if 0
static struct nand_ecclayout nand_oob_8 = {
	.eccbytes = 3,
	.eccpos = {0, 1, 2},
	.oobfree = {
		{.offset = 3,
		 .length = 2},
		{.offset = 6,
		 .length = 2}
	}
};
#endif

/**
 * Returns the flash bank specified by @a name, which matches the
 * driver name and a suffix (option) specify the driver-specific
 * bank number. The suffix consists of the '.' and the driver-specific
 * bank number: when two davinci banks are defined, then 'davinci.1' refers
 * to the second (e.g. DM355EVM).
 */
static struct nand_device *get_nand_device_by_name(const char *name)
{
	unsigned requested = get_flash_name_index(name);
	unsigned found = 0;

	struct nand_device *nand;
	for (nand = nand_devices; NULL != nand; nand = nand->next) {
		if (strcmp(nand->name, name) == 0)
			return nand;
		if (!flash_driver_name_matches(nand->controller->name, name))
			continue;
		if (++found < requested)
			continue;
		return nand;
	}
	return NULL;
}

struct nand_device *get_nand_device_by_num(int num)
{
	struct nand_device *p;
	int i = 0;

	for (p = nand_devices; p; p = p->next) {
		if (i++ == num)
			return p;
	}

	return NULL;
}

COMMAND_HELPER(nand_command_get_device, unsigned name_index,
	struct nand_device **nand)
{
	const char *str = CMD_ARGV[name_index];
	*nand = get_nand_device_by_name(str);
	if (*nand)
		return ERROR_OK;

	unsigned num;
	COMMAND_PARSE_NUMBER(uint, str, num);
	*nand = get_nand_device_by_num(num);
	if (!*nand) {
		command_print(CMD, "NAND flash device '%s' not found", str);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

int nand_build_bbt(struct nand_device *nand, int first, int last)
{
	uint32_t page;
	int i;
	int pages_per_block = (nand->erase_size / nand->page_size);
	uint8_t oob[6];
	int ret;

	if ((first < 0) || (first >= nand->num_blocks))
		first = 0;

	if ((last >= nand->num_blocks) || (last == -1))
		last = nand->num_blocks - 1;

	page = first * pages_per_block;
	for (i = first; i <= last; i++) {
		ret = nand_read_page(nand, page, NULL, 0, oob, 6);
		if (ret != ERROR_OK)
			return ret;

		if (((nand->device->options & NAND_BUSWIDTH_16) && ((oob[0] & oob[1]) != 0xff))
				|| (((nand->page_size == 512) && (oob[5] != 0xff)) ||
				((nand->page_size == 2048) && (oob[0] != 0xff)))) {
			LOG_WARNING("bad block: %i", i);
			nand->blocks[i].is_bad = 1;
		} else
			nand->blocks[i].is_bad = 0;

		page += pages_per_block;
	}

	return ERROR_OK;
}

int nand_read_status(struct nand_device *nand, uint8_t *status)
{
	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;

	/* Send read status command */
	/* FIXME: errors returned from nand->controller are mostly ignored! */
	nand->controller->command(nand, NAND_CMD_STATUS);

	alive_sleep(1);

	/* read status */
	if (nand->device->options & NAND_BUSWIDTH_16) {
		uint16_t data;
		nand->controller->read_data(nand, &data);
		*status = data & 0xff;
	} else
		nand->controller->read_data(nand, status);

	return ERROR_OK;
}

static int nand_poll_ready(struct nand_device *nand, int timeout)
{
	uint8_t status;

	nand->controller->command(nand, NAND_CMD_STATUS);
	do {
		if (nand->device->options & NAND_BUSWIDTH_16) {
			uint16_t data;
			nand->controller->read_data(nand, &data);
			status = data & 0xff;
		} else
			nand->controller->read_data(nand, &status);
		if (status & NAND_STATUS_READY)
			break;
		alive_sleep(1);
	} while (timeout--);

	return (status & NAND_STATUS_READY) != 0;
}

int nand_probe(struct nand_device *nand)
{
	uint8_t manufacturer_id, device_id;
	uint8_t id_buff[6] = { 0 };	/* zero buff to silence false warning
					 * from clang static analyzer */
	int retval;
	int i;

	/* clear device data */
	nand->device = NULL;
	nand->manufacturer = NULL;

	/* clear device parameters */
	nand->bus_width = 0;
	nand->address_cycles = 0;
	nand->page_size = 0;
	nand->erase_size = 0;

	/* initialize controller (device parameters are zero, use controller default) */
	retval = nand->controller->init(nand);
	if (retval != ERROR_OK) {
		switch (retval) {
			case ERROR_NAND_OPERATION_FAILED:
				LOG_DEBUG("controller initialization failed");
				return ERROR_NAND_OPERATION_FAILED;
			case ERROR_NAND_OPERATION_NOT_SUPPORTED:
				LOG_ERROR(
				"BUG: controller reported that it doesn't support default parameters");
				return ERROR_NAND_OPERATION_FAILED;
			default:
				LOG_ERROR("BUG: unknown controller initialization failure");
				return ERROR_NAND_OPERATION_FAILED;
		}
	}

	nand->controller->command(nand, NAND_CMD_RESET);
	nand->controller->reset(nand);

	nand->controller->command(nand, NAND_CMD_READID);
	nand->controller->address(nand, 0x0);

	if (nand->bus_width == 8) {
		nand->controller->read_data(nand, &manufacturer_id);
		nand->controller->read_data(nand, &device_id);
	} else {
		uint16_t data_buf;
		nand->controller->read_data(nand, &data_buf);
		manufacturer_id = data_buf & 0xff;
		nand->controller->read_data(nand, &data_buf);
		device_id = data_buf & 0xff;
	}

	for (i = 0; nand_flash_ids[i].name; i++) {
		if (nand_flash_ids[i].id == device_id &&
				(nand_flash_ids[i].mfr_id == manufacturer_id ||
				nand_flash_ids[i].mfr_id == 0)) {
			nand->device = &nand_flash_ids[i];
			break;
		}
	}

	for (i = 0; nand_manuf_ids[i].name; i++) {
		if (nand_manuf_ids[i].id == manufacturer_id) {
			nand->manufacturer = &nand_manuf_ids[i];
			break;
		}
	}

	if (!nand->manufacturer) {
		nand->manufacturer = &nand_manuf_ids[0];
		nand->manufacturer->id = manufacturer_id;
	}

	if (!nand->device) {
		LOG_ERROR(
			"unknown NAND flash device found, manufacturer id: 0x%2.2x device id: 0x%2.2x",
			manufacturer_id,
			device_id);
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("found %s (%s)", nand->device->name, nand->manufacturer->name);

	/* initialize device parameters */

	/* bus width */
	if (nand->device->options & NAND_BUSWIDTH_16)
		nand->bus_width = 16;
	else
		nand->bus_width = 8;

	/* Do we need extended device probe information? */
	if (nand->device->page_size == 0 ||
			nand->device->erase_size == 0) {
		if (nand->bus_width == 8) {
			retval = nand->controller->read_data(nand, id_buff + 3);
			if (retval != ERROR_OK)
				return retval;

			retval = nand->controller->read_data(nand, id_buff + 4);
			if (retval != ERROR_OK)
				return retval;

			retval = nand->controller->read_data(nand, id_buff + 5);
			if (retval != ERROR_OK)
				return retval;

		} else {
			uint16_t data_buf;

			retval = nand->controller->read_data(nand, &data_buf);
			if (retval != ERROR_OK)
				return retval;
			id_buff[3] = data_buf;

			retval = nand->controller->read_data(nand, &data_buf);
			if (retval != ERROR_OK)
				return retval;
			id_buff[4] = data_buf;

			retval = nand->controller->read_data(nand, &data_buf);
			if (retval != ERROR_OK)
				return retval;
			id_buff[5] = data_buf >> 8;
		}
	}

	/* page size */
	if (nand->device->page_size == 0)
		nand->page_size = 1 << (10 + (id_buff[4] & 3));
	else if (nand->device->page_size == 256) {
		LOG_ERROR("NAND flashes with 256 byte pagesize are not supported");
		return ERROR_NAND_OPERATION_FAILED;
	} else
		nand->page_size = nand->device->page_size;

	/* number of address cycles */
	if (nand->page_size <= 512) {
		/* small page devices */
		if (nand->device->chip_size <= 32)
			nand->address_cycles = 3;
		else if (nand->device->chip_size <= 8*1024)
			nand->address_cycles = 4;
		else {
			LOG_ERROR("BUG: small page NAND device with more than 8 GiB encountered");
			nand->address_cycles = 5;
		}
	} else {
		/* large page devices */
		if (nand->device->chip_size <= 128)
			nand->address_cycles = 4;
		else if (nand->device->chip_size <= 32*1024)
			nand->address_cycles = 5;
		else {
			LOG_ERROR("BUG: large page NAND device with more than 32 GiB encountered");
			nand->address_cycles = 6;
		}
	}

	/* erase size */
	if (nand->device->erase_size == 0) {
		switch ((id_buff[4] >> 4) & 3) {
			case 0:
				nand->erase_size = 64 << 10;
				break;
			case 1:
				nand->erase_size = 128 << 10;
				break;
			case 2:
				nand->erase_size = 256 << 10;
				break;
			case 3:
				nand->erase_size = 512 << 10;
				break;
		}
	} else
		nand->erase_size = nand->device->erase_size;

	/* initialize controller, but leave parameters at the controllers default */
	retval = nand->controller->init(nand);
	if (retval != ERROR_OK) {
		switch (retval) {
			case ERROR_NAND_OPERATION_FAILED:
				LOG_DEBUG("controller initialization failed");
				return ERROR_NAND_OPERATION_FAILED;
			case ERROR_NAND_OPERATION_NOT_SUPPORTED:
				LOG_ERROR(
				"controller doesn't support requested parameters (buswidth: %i, address cycles: %i, page size: %i)",
				nand->bus_width,
				nand->address_cycles,
				nand->page_size);
				return ERROR_NAND_OPERATION_FAILED;
			default:
				LOG_ERROR("BUG: unknown controller initialization failure");
				return ERROR_NAND_OPERATION_FAILED;
		}
	}

	nand->num_blocks = (nand->device->chip_size * 1024) / (nand->erase_size / 1024);
	nand->blocks = malloc(sizeof(struct nand_block) * nand->num_blocks);

	for (i = 0; i < nand->num_blocks; i++) {
		nand->blocks[i].size = nand->erase_size;
		nand->blocks[i].offset = i * nand->erase_size;
		nand->blocks[i].is_erased = -1;
		nand->blocks[i].is_bad = -1;
	}

	return ERROR_OK;
}

int nand_erase(struct nand_device *nand, int first_block, int last_block)
{
	int i;
	uint32_t page;
	uint8_t status;
	int retval;

	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;

	if ((first_block < 0) || (last_block >= nand->num_blocks))
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* make sure we know if a block is bad before erasing it */
	for (i = first_block; i <= last_block; i++) {
		if (nand->blocks[i].is_bad == -1) {
			nand_build_bbt(nand, i, last_block);
			break;
		}
	}

	for (i = first_block; i <= last_block; i++) {
		/* Send erase setup command */
		nand->controller->command(nand, NAND_CMD_ERASE1);

		page = i * (nand->erase_size / nand->page_size);

		/* Send page address */
		if (nand->page_size <= 512) {
			/* row */
			nand->controller->address(nand, page & 0xff);
			nand->controller->address(nand, (page >> 8) & 0xff);

			/* 3rd cycle only on devices with more than 32 MiB */
			if (nand->address_cycles >= 4)
				nand->controller->address(nand, (page >> 16) & 0xff);

			/* 4th cycle only on devices with more than 8 GiB */
			if (nand->address_cycles >= 5)
				nand->controller->address(nand, (page >> 24) & 0xff);
		} else {
			/* row */
			nand->controller->address(nand, page & 0xff);
			nand->controller->address(nand, (page >> 8) & 0xff);

			/* 3rd cycle only on devices with more than 128 MiB */
			if (nand->address_cycles >= 5)
				nand->controller->address(nand, (page >> 16) & 0xff);
		}

		/* Send erase confirm command */
		nand->controller->command(nand, NAND_CMD_ERASE2);

		retval = nand->controller->nand_ready ?
			nand->controller->nand_ready(nand, 1000) :
			nand_poll_ready(nand, 1000);
		if (!retval) {
			LOG_ERROR("timeout waiting for NAND flash block erase to complete");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}

		retval = nand_read_status(nand, &status);
		if (retval != ERROR_OK) {
			LOG_ERROR("couldn't read status");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (status & 0x1) {
			LOG_ERROR("didn't erase %sblock %d; status: 0x%2.2x",
				(nand->blocks[i].is_bad == 1)
				? "bad " : "",
				i, status);
			/* continue; other blocks might still be erasable */
		}

		nand->blocks[i].is_erased = 1;
	}

	return ERROR_OK;
}

#if 0
static int nand_read_plain(struct nand_device *nand,
	uint32_t address,
	uint8_t *data,
	uint32_t data_size)
{
	uint8_t *page;

	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;

	if (address % nand->page_size) {
		LOG_ERROR("reads need to be page aligned");
		return ERROR_NAND_OPERATION_FAILED;
	}

	page = malloc(nand->page_size);

	while (data_size > 0) {
		uint32_t thisrun_size = (data_size > nand->page_size) ? nand->page_size : data_size;
		uint32_t page_address;


		page_address = address / nand->page_size;

		nand_read_page(nand, page_address, page, nand->page_size, NULL, 0);

		memcpy(data, page, thisrun_size);

		address += thisrun_size;
		data += thisrun_size;
		data_size -= thisrun_size;
	}

	free(page);

	return ERROR_OK;
}

static int nand_write_plain(struct nand_device *nand,
	uint32_t address,
	uint8_t *data,
	uint32_t data_size)
{
	uint8_t *page;

	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;

	if (address % nand->page_size) {
		LOG_ERROR("writes need to be page aligned");
		return ERROR_NAND_OPERATION_FAILED;
	}

	page = malloc(nand->page_size);

	while (data_size > 0) {
		uint32_t thisrun_size = (data_size > nand->page_size) ? nand->page_size : data_size;
		uint32_t page_address;

		memset(page, 0xff, nand->page_size);
		memcpy(page, data, thisrun_size);

		page_address = address / nand->page_size;

		nand_write_page(nand, page_address, page, nand->page_size, NULL, 0);

		address += thisrun_size;
		data += thisrun_size;
		data_size -= thisrun_size;
	}

	free(page);

	return ERROR_OK;
}
#endif

int nand_write_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	uint32_t block;

	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;

	block = page / (nand->erase_size / nand->page_size);
	if (nand->blocks[block].is_erased == 1)
		nand->blocks[block].is_erased = 0;

	if (nand->use_raw || nand->controller->write_page == NULL)
		return nand_write_page_raw(nand, page, data, data_size, oob, oob_size);
	else
		return nand->controller->write_page(nand, page, data, data_size, oob, oob_size);
}

int nand_read_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;

	if (nand->use_raw || nand->controller->read_page == NULL)
		return nand_read_page_raw(nand, page, data, data_size, oob, oob_size);
	else
		return nand->controller->read_page(nand, page, data, data_size, oob, oob_size);
}

int nand_page_command(struct nand_device *nand, uint32_t page,
	uint8_t cmd, bool oob_only)
{
	if (!nand->device)
		return ERROR_NAND_DEVICE_NOT_PROBED;

	if (oob_only && NAND_CMD_READ0 == cmd && nand->page_size <= 512)
		cmd = NAND_CMD_READOOB;

	nand->controller->command(nand, cmd);

	if (nand->page_size <= 512) {
		/* small page device */

		/* column (always 0, we start at the beginning of a page/OOB area) */
		nand->controller->address(nand, 0x0);

		/* row */
		nand->controller->address(nand, page & 0xff);
		nand->controller->address(nand, (page >> 8) & 0xff);

		/* 4th cycle only on devices with more than 32 MiB */
		if (nand->address_cycles >= 4)
			nand->controller->address(nand, (page >> 16) & 0xff);

		/* 5th cycle only on devices with more than 8 GiB */
		if (nand->address_cycles >= 5)
			nand->controller->address(nand, (page >> 24) & 0xff);
	} else {
		/* large page device */

		/* column (0 when we start at the beginning of a page,
		 * or 2048 for the beginning of OOB area)
		 */
		nand->controller->address(nand, 0x0);
		if (oob_only)
			nand->controller->address(nand, 0x8);
		else
			nand->controller->address(nand, 0x0);

		/* row */
		nand->controller->address(nand, page & 0xff);
		nand->controller->address(nand, (page >> 8) & 0xff);

		/* 5th cycle only on devices with more than 128 MiB */
		if (nand->address_cycles >= 5)
			nand->controller->address(nand, (page >> 16) & 0xff);

		/* large page devices need a start command if reading */
		if (NAND_CMD_READ0 == cmd)
			nand->controller->command(nand, NAND_CMD_READSTART);
	}

	if (nand->controller->nand_ready) {
		if (!nand->controller->nand_ready(nand, 100))
			return ERROR_NAND_OPERATION_TIMEOUT;
	} else {
		/* nand_poll_read() cannot be used during nand read */
		alive_sleep(1);
	}

	return ERROR_OK;
}

int nand_read_data_page(struct nand_device *nand, uint8_t *data, uint32_t size)
{
	int retval = ERROR_NAND_NO_BUFFER;

	if (nand->controller->read_block_data != NULL)
		retval = (nand->controller->read_block_data)(nand, data, size);

	if (ERROR_NAND_NO_BUFFER == retval) {
		uint32_t i;
		int incr = (nand->device->options & NAND_BUSWIDTH_16) ? 2 : 1;

		retval = ERROR_OK;
		for (i = 0; retval == ERROR_OK && i < size; i += incr) {
			retval = nand->controller->read_data(nand, data);
			data += incr;
		}
	}

	return retval;
}

int nand_read_page_raw(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	int retval;

	retval = nand_page_command(nand, page, NAND_CMD_READ0, !data);
	if (ERROR_OK != retval)
		return retval;

	if (data)
		nand_read_data_page(nand, data, data_size);

	if (oob)
		nand_read_data_page(nand, oob, oob_size);

	return ERROR_OK;
}

int nand_write_data_page(struct nand_device *nand, uint8_t *data, uint32_t size)
{
	int retval = ERROR_NAND_NO_BUFFER;

	if (nand->controller->write_block_data != NULL)
		retval = (nand->controller->write_block_data)(nand, data, size);

	if (ERROR_NAND_NO_BUFFER == retval) {
		bool is16bit = nand->device->options & NAND_BUSWIDTH_16;
		uint32_t incr = is16bit ? 2 : 1;
		uint16_t write_data;
		uint32_t i;

		for (i = 0; i < size; i += incr) {
			if (is16bit)
				write_data = le_to_h_u16(data);
			else
				write_data = *data;

			retval = nand->controller->write_data(nand, write_data);
			if (ERROR_OK != retval)
				break;

			data += incr;
		}
	}

	return retval;
}

int nand_write_finish(struct nand_device *nand)
{
	int retval;
	uint8_t status;

	nand->controller->command(nand, NAND_CMD_PAGEPROG);

	retval = nand->controller->nand_ready ?
		nand->controller->nand_ready(nand, 100) :
		nand_poll_ready(nand, 100);
	if (!retval)
		return ERROR_NAND_OPERATION_TIMEOUT;

	retval = nand_read_status(nand, &status);
	if (ERROR_OK != retval) {
		LOG_ERROR("couldn't read status");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (status & NAND_STATUS_FAIL) {
		LOG_ERROR("write operation didn't pass, status: 0x%2.2x",
			status);
		return ERROR_NAND_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int nand_write_page_raw(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	int retval;

	retval = nand_page_command(nand, page, NAND_CMD_SEQIN, !data);
	if (ERROR_OK != retval)
		return retval;

	if (data) {
		retval = nand_write_data_page(nand, data, data_size);
		if (ERROR_OK != retval) {
			LOG_ERROR("Unable to write data to NAND device");
			return retval;
		}
	}

	if (oob) {
		retval = nand_write_data_page(nand, oob, oob_size);
		if (ERROR_OK != retval) {
			LOG_ERROR("Unable to write OOB data to NAND device");
			return retval;
		}
	}

	return nand_write_finish(nand);
}
