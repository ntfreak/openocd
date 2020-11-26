/***************************************************************************
 *   Copyright (C) 2020 by Nuvoton Technology Corporation                  *
 *   Mulin Chao <mlchao@nuvoton.com>                                       *
 *   Wealian Liao <WHLIAO@nuvoton.com>                                     *
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
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/armv7m.h>

/* NPCX flash loader */
const uint8_t npcx_algo[] = {
#include "../../../contrib/loaders/flash/npcx/npcx_algo.inc"
};

#define FLASH_TIMEOUT_MS 8000

/* NPCX chip information */
#define NPCX_MONITOR_FLASH_ERASE_SIZE 0x1000
#define NPCX_FLASH_BASE_ADDR          0x64000000

/* NPCX flash loader information */
#define NPCX_FLASH_LOADER_PROGRAM_ADDR 0x200c0000
#define NPCX_FLASH_LOADER_PARAMS_ADDR 0x200c1000
#define NPCX_FLASH_LOADER_BUFFER_ADDR 0x200c1010
#define NPCX_FLASH_LOADER_BUFFER_SIZE NPCX_MONITOR_FLASH_ERASE_SIZE

/* NPCX flash loader data format */
#define NPCX_PARAMS_ADDRESS_OFFSET 0x0
#define NPCX_PARAMS_LENGTH_OFFSET  0x4
#define NPCX_PARAMS_COMMAND_OFFSET 0x8
#define NPCX_PARAMS_STATUS_OFFSET  0xC

/* flash loader trigger signal */
typedef enum {
	LOADER_WAIT    = 0x0,       /* Idle */
	LOADER_EXECUTE = 0xFFFFFFFF /* Execute Command */
} npcx_flash_handshake_t;

/* flash loader command */
typedef enum {
	CMD_NO_ACTION = 0, /* No action, default value */
	CMD_GET_FLASH_ID,  /* Get the internal flash ID */
	CMD_ERASE_SECTORS, /* Erase unprotected sectors */
	CMD_ERASE_ALL,     /* Erase all */
	CMD_PROGRAM,       /* Program data */
} flash_commands_t;

/* flash list */
typedef enum {
	FLASH_256KB = 0,
	FLASH_512KB = 1,
	FLASH_1MB = 2,
	FLASH_UNKNOWN,
} npcx_device_index_t;

struct npcx_bank {
	const char *family_name;
	uint32_t sector_length;
	bool probed;
	uint32_t flash;
	struct working_area *working_area;
	struct armv7m_algorithm armv7m_info;
	const uint8_t *algo_code;
	uint32_t algo_size;
	uint32_t algo_working_size;
	uint32_t buffer_addr;
	uint32_t params_addr;
};

struct npcx_algo_params {
	uint8_t address[4];
	uint8_t length[4];
	uint8_t command[4];
	uint8_t status[4];
};

struct npcx_flash_info {
	char *name;
	uint32_t id;
	uint32_t size;
};

static const struct npcx_flash_info flash_info[] = {
	[FLASH_256KB] = {
		.name = "256KB Flash",
		.id = 0xEF4012,
		.size = 256 * 1024,
	},
	[FLASH_512KB] = {
		.name = "512KB Flash",
		.id = 0xEF4013,
		.size = 512 * 1024,
	},
	[FLASH_1MB] = {
		.name = "1MB Flash",
		.id = 0xEF4014,
		.size = 1024 * 1024,
	},
	[FLASH_UNKNOWN] = {
		.name = "Unknown Flash",
		.size = 0xFFFFFFFF,
	},
};

static int npcx_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct npcx_bank *npcx_bank = bank->driver_priv;

	/* Check for working area to use for flash helper algorithm */
	if (npcx_bank->working_area)
		target_free_working_area(target, npcx_bank->working_area);

	int retval = target_alloc_working_area(target, npcx_bank->algo_working_size,
				&npcx_bank->working_area);
	if (retval != ERROR_OK)
		return retval;

	/* Confirm the defined working address is the area we need to use */
	if (npcx_bank->working_area->address != NPCX_FLASH_LOADER_PROGRAM_ADDR)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, NPCX_FLASH_LOADER_PROGRAM_ADDR,
				npcx_bank->algo_size, npcx_bank->algo_code);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: Failed to load flash helper algorithm",
			npcx_bank->family_name);
		target_free_working_area(target, npcx_bank->working_area);
		return retval;
	}

	/* Initialize the ARMv7 specific info to run the algorithm */
	npcx_bank->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	npcx_bank->armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Begin executing the flash helper algorithm */
	retval = target_start_algorithm(target, 0, NULL, 0, NULL,
				NPCX_FLASH_LOADER_PROGRAM_ADDR, 0,
				&npcx_bank->armv7m_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: Failed to start flash helper algorithm",
			npcx_bank->family_name);
		target_free_working_area(target, npcx_bank->working_area);
		return retval;
	}

	/*
	 * At this point, the algorithm is running on the target and
	 * ready to receive commands and data to flash the target
	 */

	return retval;
}

static int npcx_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct npcx_bank *npcx_bank = bank->driver_priv;

	/* Regardless of the algo's status, attempt to halt the target */
	(void)target_halt(target);

	/* Now confirm target halted and clean up from flash helper algorithm */
	int retval = target_wait_algorithm(target, 0, NULL, 0, NULL, 0,
					FLASH_TIMEOUT_MS, &npcx_bank->armv7m_info);

	target_free_working_area(target, npcx_bank->working_area);
	npcx_bank->working_area = NULL;

	return retval;
}

static int npcx_wait_algo_done(struct flash_bank *bank, uint32_t params_addr)
{
	struct target *target = bank->target;
	struct npcx_bank *npcx_bank = bank->driver_priv;
	uint32_t status_addr = params_addr + NPCX_PARAMS_STATUS_OFFSET;
	uint32_t status;
	int64_t start_ms = timeval_ms();

	do {
		int retval = target_read_u32(target, status_addr, &status);
		if (retval != ERROR_OK)
			return retval;

		int64_t elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
		if (elapsed_ms > FLASH_TIMEOUT_MS)
			break;
	} while (status == LOADER_EXECUTE);

	if (status != LOADER_WAIT) {
		LOG_ERROR("%s: Flash operation failed, 0x%" PRIx32,
				npcx_bank->family_name,
				status);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int npcx_get_flash_id(struct flash_bank *bank, uint32_t *flash_id)
{
	struct target *target = bank->target;
	struct npcx_bank *npcx_bank = bank->driver_priv;
	struct npcx_algo_params algo_params;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = npcx_init(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Set up algorithm parameters for get flash ID command */
	buf_set_u32(algo_params.command, 0, 32, CMD_GET_FLASH_ID);
	buf_set_u32(algo_params.status,  0, 32, LOADER_WAIT);

	/* Issue flash helper algorithm parameters for get flash ID */
	retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);

	buf_set_u32(algo_params.status,  0, 32, LOADER_EXECUTE);
	retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);

	/* If no error, wait for finishing */
	if (retval == ERROR_OK) {
		retval = npcx_wait_algo_done(bank, npcx_bank->params_addr);
		if (retval == ERROR_OK)
			target_read_u32(target, NPCX_FLASH_LOADER_BUFFER_ADDR, flash_id);
	}

	/* Regardless of errors, try to close down algo */
	(void)npcx_quit(bank);

	return retval;
}

static int npcx_get_flash(uint32_t flash_id)
{
	for (uint32_t i = 0; i < ARRAY_SIZE(flash_info) - 1; i++) {
		if (flash_info[i].id == flash_id)
			return i;
	}

	return FLASH_UNKNOWN;
}

static int npcx_probe(struct flash_bank *bank)
{
	struct npcx_bank *npcx_bank = bank->driver_priv;
	uint32_t sector_length = NPCX_MONITOR_FLASH_ERASE_SIZE;
	uint32_t flash_id;

	/* Set up appropriate flash helper algorithm */
	npcx_bank->algo_code = npcx_algo;
	npcx_bank->algo_size = sizeof(npcx_algo);
	npcx_bank->algo_working_size = NPCX_FLASH_LOADER_BUFFER_ADDR +
					NPCX_FLASH_LOADER_BUFFER_SIZE -
					NPCX_FLASH_LOADER_PROGRAM_ADDR;
	npcx_bank->buffer_addr = NPCX_FLASH_LOADER_BUFFER_ADDR;
	npcx_bank->params_addr = NPCX_FLASH_LOADER_PARAMS_ADDR;

	int retval = npcx_get_flash_id(bank, &flash_id);
	if (retval != ERROR_OK)
		return retval;

	npcx_bank->flash = npcx_get_flash(flash_id);

	int num_sectors = flash_info[npcx_bank->flash].size / sector_length;

	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	bank->base = NPCX_FLASH_BASE_ADDR;
	bank->num_sectors = num_sectors;
	bank->size = num_sectors * sector_length;
	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;
	npcx_bank->sector_length = sector_length;

	for (int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * sector_length;
		bank->sectors[i].size = sector_length;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	/* We've successfully determined the stats on the flash bank */
	npcx_bank->probed = true;

	/* If we fall through to here, then all went well */
	return ERROR_OK;
}

static int npcx_auto_probe(struct flash_bank *bank)
{
	struct npcx_bank *npcx_bank = bank->driver_priv;
	int retval = ERROR_OK;

	if (!npcx_bank->probed)
		retval = npcx_probe(bank);

	return retval;
}

FLASH_BANK_COMMAND_HANDLER(npcx_flash_bank_command)
{
	struct npcx_bank *npcx_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	npcx_bank = malloc(sizeof(struct npcx_bank));
	if (!npcx_bank) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	/* Initialize private flash information */
	memset(npcx_bank, 0x00, sizeof(struct npcx_bank));
	npcx_bank->family_name = "npcx";
	npcx_bank->sector_length = NPCX_MONITOR_FLASH_ERASE_SIZE;

	/* Finish initialization of bank */
	bank->driver_priv = npcx_bank;
	bank->next = NULL;

	return ERROR_OK;
}

static int npcx_chip_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct npcx_bank *npcx_bank = bank->driver_priv;
	struct npcx_algo_params algo_params;

	LOG_INFO("chip erase");

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Make sure we've probed the flash to get the device and size */
	int retval = npcx_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = npcx_init(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Set up algorithm parameters for chip erase command */
	buf_set_u32(algo_params.command, 0, 32, CMD_ERASE_ALL);
	buf_set_u32(algo_params.status,  0, 32, LOADER_WAIT);

	/* Set algorithm parameters */
	retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);

	/* Issue flash helper algorithm parameters for chip erase */
	buf_set_u32(algo_params.status,  0, 32, LOADER_EXECUTE);
	retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);

	/* If no error, wait for chip erase finish */
	if (retval == ERROR_OK)
		retval = npcx_wait_algo_done(bank, npcx_bank->params_addr);

	/* Regardless of errors, try to close down algo */
	(void)npcx_quit(bank);

	return retval;
}

static int npcx_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct npcx_bank *npcx_bank = bank->driver_priv;
	struct npcx_algo_params algo_params;

	LOG_INFO("erase first: %d, last: %d", first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1))) {
		/* Request chip erase */
		return npcx_chip_erase(bank);
	}

	uint32_t address = first * npcx_bank->sector_length;
	uint32_t length = (last - first + 1) * npcx_bank->sector_length;

	/* Make sure we've probed the flash to get the device and size */
	int retval = npcx_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = npcx_init(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Set up algorithm parameters for erase command */
	buf_set_u32(algo_params.address, 0, 32, address);
	buf_set_u32(algo_params.length,  0, 32, length);
	buf_set_u32(algo_params.command, 0, 32, CMD_ERASE_SECTORS);
	buf_set_u32(algo_params.status,  0, 32, LOADER_WAIT);

	/* Set algorithm parameters */
	retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);

	/* Issue flash helper algorithm parameters for erase */
	buf_set_u32(algo_params.status,  0, 32, LOADER_EXECUTE);
	retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);

	/* If no error, wait for erase to finish */
	if (retval == ERROR_OK)
		retval = npcx_wait_algo_done(bank, npcx_bank->params_addr);

	/* Regardless of errors, try to close down algo */
	(void)npcx_quit(bank);

	return retval;
}

static int npcx_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct npcx_bank *npcx_bank = bank->driver_priv;
	struct npcx_algo_params algo_params;

	LOG_INFO("write first: 0x%x, count: 0x%x", offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Make sure we've probed the flash to get the device and size */
	int retval = npcx_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = npcx_init(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Initialize algorithm parameters to default values */
	buf_set_u32(algo_params.command, 0, 32, CMD_PROGRAM);

	uint32_t address = offset;

	while (count > 0) {
		uint32_t size = (count > NPCX_FLASH_LOADER_BUFFER_SIZE) ?
							NPCX_FLASH_LOADER_BUFFER_SIZE : count;

		/* Put the data into buffer */
		retval = target_write_buffer(target, npcx_bank->buffer_addr,
					size, buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write data to target memory");
			break;
		}

		/* Update algo parameters for flash write */
		buf_set_u32(algo_params.address, 0, 32, address);
		buf_set_u32(algo_params.length,  0, 32, size);
		buf_set_u32(algo_params.status,  0, 32, LOADER_WAIT);

		/* Set algorithm parameters */
		retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);

		/* Issue flash helper algorithm parameters for flash write */
		buf_set_u32(algo_params.status,  0, 32, LOADER_EXECUTE);
		retval = target_write_buffer(target, npcx_bank->params_addr,
				sizeof(algo_params), (uint8_t *)&algo_params);
		if (retval != ERROR_OK)
			break;

		/* Wait for flash write finish */
		retval = npcx_wait_algo_done(bank, npcx_bank->params_addr);
		if (retval != ERROR_OK)
			break;

		count -= size;
		buffer += size;
		address += size;
	}

	/* Regardless of errors, try to close down algo */
	(void)npcx_quit(bank);

	return retval;
}

static int npcx_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct npcx_bank *npcx_bank = bank->driver_priv;
	int printed = 0;

	printed = snprintf(buf, buf_size, "%s flash: %s\n",
					npcx_bank->family_name,
					flash_info[npcx_bank->flash].name);

	if (printed >= buf_size)
		return ERROR_BUF_TOO_SMALL;

	return ERROR_OK;
}

const struct flash_driver npcx_flash = {
	.name = "npcx",
	.flash_bank_command = npcx_flash_bank_command,
	.erase = npcx_erase,
	.write = npcx_write,
	.read = default_flash_read,
	.probe = npcx_probe,
	.auto_probe = npcx_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = npcx_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
