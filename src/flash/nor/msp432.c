/***************************************************************************
 *   Copyright (C) 2018 by Texas Instruments, Inc.                         *
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
#include "msp432.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/image.h>

/* MSP432P4 hardware registers */
#define P4_FLASH_MAIN_SIZE_REG 0xE0043020
#define P4_FLASH_INFO_SIZE_REG 0xE0043024
#define P4_DEVICE_ID_REG       0x0020100C
#define P4_HARDWARE_REV_REG    0x00201010

/* MSP432E4 hardware registers */
#define E4_DID0_REG 0x400FE000
#define E4_DID1_REG 0x400FE004

#define FLASH_TIMEOUT 8000

#define SUPPORT_MESSAGE \
	"Your pre-production MSP432P401x silicon is not fully supported\n" \
	"You can find more information at www.ti.com/product/MSP432P401R"

struct msp432_bank {
	uint32_t device_id;
	uint32_t hardware_rev;
	int family_type;
	int device_type;
	uint32_t sector_length;
	bool probed_main;
	bool probed_info;
	bool unlock_bsl;
	struct working_area *working_area;
	struct armv7m_algorithm armv7m_info;
};

static int msp432_auto_probe(struct flash_bank *bank);

static int msp432_device_type(uint32_t family_type, uint32_t device_id,
	uint32_t hardware_rev)
{
	int device_type = MSP432_NO_TYPE;

	if (family_type == MSP432E4) {
		/* MSP432E4 device family */

		if (device_id == 0x180C0002) {
			if (hardware_rev == 0x102DC06E) {
				/* The 01Y variant */
				device_type = MSP432E401Y;
			} else if (hardware_rev == 0x1032E076) {
				/* The 11Y variant */
				device_type = MSP432E411Y;
			} else {
				/* Reasonable guess that this is a new variant */
				device_type = MSP432E4X_GUESS;
			}
		} else {
			/* Wild guess that this is an MSP432E4 */
			device_type = MSP432E4X_GUESS;
		}
	} else {
		/* MSP432P4 device family */

		/* Examine the device ID and hardware revision to get the device type */
		switch (device_id) {
			case 0xA000:
			case 0xA001:
			case 0xA002:
			case 0xA003:
			case 0xA004:
			case 0xA005:
				/* Device is definitely MSP432P401x, check hardware revision */
				if (hardware_rev == 0x41 || hardware_rev == 0x42) {
					/* Rev A or B of the silicon has been deprecated */
					device_type = MSP432P401X_DEPR;
				} else if (hardware_rev >= 0x43 && hardware_rev <= 0x49) {
					/* Current and future revisions of the MSP432P401x device */
					device_type = MSP432P401X;
				} else {
					/* Unknown or unanticipated hardware revision */
					device_type = MSP432P401X_GUESS;
				}
				break;
			case 0xA010:
			case 0xA012:
			case 0xA016:
			case 0xA019:
			case 0xA01F:
			case 0xA020:
			case 0xA022:
			case 0xA026:
			case 0xA029:
			case 0xA02F:
				/* Device is definitely MSP432P411x, check hardware revision */
				if (hardware_rev >= 0x41 && hardware_rev <= 0x49) {
					/* Current and future revisions of the MSP432P411x device */
					device_type = MSP432P411X;
				} else {
					/* Unknown or unanticipated hardware revision */
					device_type = MSP432P411X_GUESS;
				}
				break;
			case 0xFFFF:
				/* Device is very early silicon that has been deprecated */
				device_type = MSP432P401X_DEPR;
				break;
			default:
				if (device_id < 0xA010) {
					/* Wild guess that this is an MSP432P401x */
					device_type = MSP432P401X_GUESS;
				} else {
					/* Reasonable guess that this is a new variant */
					device_type = MSP432P411X_GUESS;
				}
				break;
		}
	}

	return device_type;
}

static const char *msp432_return_text(uint32_t return_code)
{
	switch (return_code) {
		case FLASH_BUSY:
			return "FLASH_BUSY";
		case FLASH_SUCCESS:
			return "FLASH_SUCCESS";
		case FLASH_ERROR:
			return "FLASH_ERROR";
		case FLASH_TIMEOUT_ERROR:
			return "FLASH_TIMEOUT_ERROR";
		case FLASH_VERIFY_ERROR:
			return "FLASH_VERIFY_WRONG";
		case FLASH_WRONG_COMMAND:
			return "FLASH_WRONG_COMMAND";
		case FLASH_POWER_ERROR:
			return "FLASH_POWER_ERROR";
		default:
			return "UNDEFINED_RETURN_CODE";
	}
}

static void msp432_init_params(struct msp432_algo_params *algo_params)
{
	buf_set_u32(algo_params->flash_command, 0, 32, FLASH_NO_COMMAND);
	buf_set_u32(algo_params->return_code, 0, 32, 0);
	buf_set_u32(algo_params->_reserved0, 0, 32, 0);
	buf_set_u32(algo_params->address, 0, 32, 0);
	buf_set_u32(algo_params->length, 0, 32, 0);
	buf_set_u32(algo_params->buffer1_status, 0, 32, BUFFER_INACTIVE);
	buf_set_u32(algo_params->buffer2_status, 0, 32, BUFFER_INACTIVE);
	buf_set_u32(algo_params->erase_param, 0, 32, FLASH_ERASE_MAIN);
	buf_set_u32(algo_params->unlock_bsl, 0, 32, FLASH_LOCK_BSL);
}

static int msp432_exec_cmd(struct target *target, struct msp432_algo_params
			*algo_params, uint32_t command)
{
	int retval;

	/* Make sure the given params do not include the command */
	buf_set_u32(algo_params->flash_command, 0, 32, FLASH_NO_COMMAND);
	buf_set_u32(algo_params->return_code, 0, 32, 0);
	buf_set_u32(algo_params->buffer1_status, 0, 32, BUFFER_INACTIVE);
	buf_set_u32(algo_params->buffer2_status, 0, 32, BUFFER_INACTIVE);

	/* Write out parameters to target memory */
	retval = target_write_buffer(target, ALGO_PARAMS_BASE_ADDR,
				sizeof(struct msp432_algo_params), (uint8_t *)algo_params);
	if (retval != ERROR_OK)
		return retval;

	/* Write out command to target memory */
	retval = target_write_u32(target, ALGO_FLASH_COMMAND_ADDR, command);

	return retval;
}

static int msp432_wait_return_code(struct target *target)
{
	uint32_t return_code = 0;
	long long start_ms;
	long long elapsed_ms;

	int retval = ERROR_OK;

	start_ms = timeval_ms();
	while ((return_code == 0) || (return_code == FLASH_BUSY)) {
		retval = target_read_u32(target, ALGO_RETURN_CODE_ADDR, &return_code);
		if (retval != ERROR_OK)
			return retval;

		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
		if (elapsed_ms > FLASH_TIMEOUT)
			break;
	};

	if (return_code != FLASH_SUCCESS) {
		LOG_ERROR("msp432: Flash operation failed: %s",
			msp432_return_text(return_code));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int msp432_wait_inactive(struct target *target, uint32_t buffer)
{
	uint32_t status_code = BUFFER_ACTIVE;
	uint32_t status_addr;
	long long start_ms;
	long long elapsed_ms;

	int retval;

	switch (buffer) {
		case 1: /* Buffer 1 */
			status_addr = ALGO_BUFFER1_STATUS_ADDR;
			break;
		case 2: /* Buffer 2 */
			status_addr = ALGO_BUFFER2_STATUS_ADDR;
			break;
		default:
			return ERROR_FAIL;
	}

	start_ms = timeval_ms();
	while (status_code != BUFFER_INACTIVE) {
		retval = target_read_u32(target, status_addr, &status_code);
		if (retval != ERROR_OK)
			return retval;

		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
		if (elapsed_ms > FLASH_TIMEOUT)
			break;
	};

	if (status_code != BUFFER_INACTIVE) {
		LOG_ERROR(
			"msp432: Flash operation failed: buffer not written to flash");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int msp432_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct msp432_bank *msp432_bank = bank->driver_priv;
	struct msp432_algo_params algo_params;
	struct reg_param reg_params[1];

	const uint8_t *loader_code;
	uint32_t loader_size;
	uint32_t algo_entry_addr;
	int retval;

	/* Make sure we've probed the flash to get the device and size */
	retval = msp432_auto_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Choose appropriate flash helper algorithm */
	switch (msp432_bank->device_type) {
		case MSP432P401X:
		case MSP432P401X_DEPR:
		case MSP432P401X_GUESS:
		default:
			loader_code = msp432p401x_algo;
			loader_size = sizeof(msp432p401x_algo);
			algo_entry_addr = P4_ALGO_ENTRY_ADDR;
			break;
		case MSP432P411X:
		case MSP432P411X_GUESS:
			loader_code = msp432p411x_algo;
			loader_size = sizeof(msp432p411x_algo);
			algo_entry_addr = P4_ALGO_ENTRY_ADDR;
			break;
		case MSP432E401Y:
		case MSP432E411Y:
		case MSP432E4X_GUESS:
			loader_code = msp432e4x_algo;
			loader_size = sizeof(msp432e4x_algo);
			algo_entry_addr = E4_ALGO_ENTRY_ADDR;
			break;
	}

	/* Issue warnings if this is a device we may not be able to flash */
	if (msp432_bank->device_type == MSP432P401X_GUESS ||
		msp432_bank->device_type == MSP432P411X_GUESS) {
		/* Explicit device type check failed. Report this. */
		LOG_WARNING(
			"msp432: Unrecognized MSP432P4 Device ID and Hardware "
			"Rev (%04" PRIX32 ", %02" PRIX32 ")", msp432_bank->device_id,
			msp432_bank->hardware_rev);
	} else if (msp432_bank->device_type == MSP432P401X_DEPR) {
		LOG_WARNING(
			"msp432: MSP432P401x pre-production device (deprecated "
			"silicon)\n" SUPPORT_MESSAGE);
	} else if (msp432_bank->device_type == MSP432E4X_GUESS) {
		/* Explicit device type check failed. Report this. */
		LOG_WARNING(
			"msp432: Unrecognized MSP432E4 DID0 and DID1 values "
			"(%08" PRIX32 ", %08" PRIX32 ")", msp432_bank->device_id,
			msp432_bank->hardware_rev);
	}

	/* Check for working area to use for flash helper algorithm */
	if (msp432_bank->working_area)
		target_free_working_area(target, msp432_bank->working_area);
	retval = target_alloc_working_area(target, ALGO_WORKING_SIZE,
				&msp432_bank->working_area);
	if (retval != ERROR_OK)
		return retval;

	/* Confirm the defined working address is the area we need to use */
	if (msp432_bank->working_area->address != ALGO_BASE_ADDR)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, ALGO_BASE_ADDR, loader_size,
				loader_code);
	if (retval != ERROR_OK)
		return retval;

	/* Initialize the ARMv7 specific info to run the algorithm */
	msp432_bank->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	msp432_bank->armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Initialize algorithm parameters to default values */
	msp432_init_params(&algo_params);

	/* Write out parameters to target memory */
	retval = target_write_buffer(target, ALGO_PARAMS_BASE_ADDR,
				sizeof(algo_params), (uint8_t *)&algo_params);
	if (retval != ERROR_OK)
		return retval;

	/* Initialize stack pointer for flash helper algorithm */
	init_reg_param(&reg_params[0], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, ALGO_STACK_POINTER_ADDR);

	/* Begin executing the flash helper algorithm */
	retval = target_start_algorithm(target, 0, 0, 1, reg_params,
				algo_entry_addr, 0, &msp432_bank->armv7m_info);
	destroy_reg_param(&reg_params[0]);
	if (retval != ERROR_OK) {
		LOG_ERROR("msp432: Failed to start flash helper algorithm");
		return retval;
	}

	/*
	 * At this point, the algorithm is running on the target and
	 * ready to receive commands and data to flash the target
	 */

	/* Issue the init command to the flash helper algorithm */
	retval = msp432_exec_cmd(target, &algo_params, FLASH_INIT);
	if (retval != ERROR_OK)
		return retval;

	retval = msp432_wait_return_code(target);

	return retval;
}

static int msp432_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct msp432_bank *msp432_bank = bank->driver_priv;
	struct msp432_algo_params algo_params;

	int retval;

	/* Initialize algorithm parameters to default values */
	msp432_init_params(&algo_params);

	/* Issue the exit command to the flash helper algorithm */
	retval = msp432_exec_cmd(target, &algo_params, FLASH_EXIT);
	if (retval != ERROR_OK)
		return retval;

	(void)msp432_wait_return_code(target);

	/* Regardless of the return code, attempt to halt the target */
	(void)target_halt(target);

	/* Now confirm target halted and clean up from flash helper algorithm */
	retval = target_wait_algorithm(target, 0, NULL, 0, NULL, 0, FLASH_TIMEOUT,
				&msp432_bank->armv7m_info);

	target_free_working_area(target, msp432_bank->working_area);
	msp432_bank->working_area = NULL;

	return retval;
}

static int msp432_mass_erase(struct flash_bank *bank, bool all)
{
	struct target *target = bank->target;
	struct msp432_bank *msp432_bank = bank->driver_priv;
	struct msp432_algo_params algo_params;

	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = msp432_init(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Initialize algorithm parameters to default values */
	msp432_init_params(&algo_params);
	if (all) {
		buf_set_u32(algo_params.erase_param, 0, 32,
			FLASH_ERASE_MAIN | FLASH_ERASE_INFO);
		if (msp432_bank->unlock_bsl)
			buf_set_u32(algo_params.unlock_bsl, 0, 32, FLASH_UNLOCK_BSL);
	}

	/* Issue the mass erase command to the flash helper algorithm */
	retval = msp432_exec_cmd(target, &algo_params, FLASH_MASS_ERASE);
	if (retval != ERROR_OK) {
		(void)msp432_quit(bank);
		return retval;
	}

	retval = msp432_wait_return_code(target);
	if (retval != ERROR_OK) {
		(void)msp432_quit(bank);
		return retval;
	}

	retval = msp432_quit(bank);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

COMMAND_HANDLER(msp432_mass_erase_command)
{
	struct flash_bank *bank;
	struct msp432_bank *msp432_bank;
	bool all;

	int retval;

	if (1 > CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	if (1 == CMD_ARGC) {
		all = false;
	} else if (2 == CMD_ARGC) {
		/* Check argument for how much to erase */
		if (strcmp(CMD_ARGV[1], "main") == 0)
			all = false;
		else if (strcmp(CMD_ARGV[1], "all") == 0)
			all = true;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	msp432_bank = bank->driver_priv;

	if (msp432_bank->family_type == MSP432E4) {
		/* MSP432E4 does not have main vs info regions, ignore "all" */
		all = false;
	}

	retval = msp432_mass_erase(bank, all);
	if (retval != ERROR_OK)
		return retval;

	if (msp432_bank->family_type == MSP432E4) {
		/* MSP432E4 does not have main vs info regions */
		LOG_INFO("msp432: Mass erase of flash is complete");
	} else {
		LOG_INFO("msp432: Mass erase of %s is complete",
			all ? "main + information flash" : "main flash");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(msp432_bsl_command)
{
	struct flash_bank *bank;
	struct msp432_bank *msp432_bank;

	int retval;

	if (1 > CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	msp432_bank = bank->driver_priv;

	if (msp432_bank->family_type == MSP432E4) {
		LOG_WARNING("msp432: MSP432E4 does not have a BSL region");
		return ERROR_OK;
	}

	if (2 == CMD_ARGC) {
		if (strcmp(CMD_ARGV[1], "lock") == 0)
			msp432_bank->unlock_bsl = false;
		else if (strcmp(CMD_ARGV[1], "unlock") == 0)
			msp432_bank->unlock_bsl = true;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	} else if (1 != CMD_ARGC) {
		/* Extra, unknown argument passed in */
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	LOG_INFO("msp432: BSL flash region is currently %slocked",
		msp432_bank->unlock_bsl ? "un" : "");

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(msp432_flash_bank_command)
{
	struct msp432_bank *msp432_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Create shared private struct for flash banks */
	msp432_bank = malloc(sizeof(struct msp432_bank));
	if (!msp432_bank)
		return ERROR_FAIL;

	/* Initialize private flash information */
	msp432_bank->device_id = 0;
	msp432_bank->hardware_rev = 0;
	msp432_bank->family_type = MSP432_NO_FAMILY;
	msp432_bank->device_type = MSP432_NO_TYPE;
	msp432_bank->sector_length = 0x1000;
	msp432_bank->probed_main = false;
	msp432_bank->probed_info = false;
	msp432_bank->unlock_bsl = false;
	msp432_bank->working_area = NULL;

	/* Finish up initial settings here */
	bank->driver_priv = msp432_bank;
	bank->base = FLASH_BASE;

	return ERROR_OK;
}

static int msp432_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct msp432_bank *msp432_bank = bank->driver_priv;
	struct msp432_algo_params algo_params;

	bool is_main = bank->base == FLASH_BASE;
	bool is_info = bank->base == P4_FLASH_INFO_BASE;

	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Do a mass erase if user requested all sectors of main flash */
	if (is_main && (first == 0) && (last == (bank->num_sectors - 1))) {
		/* Request mass erase of main flash */
		return msp432_mass_erase(bank, false);
	}

	retval = msp432_init(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Initialize algorithm parameters to default values */
	msp432_init_params(&algo_params);

	/* Adjust params if this is the info bank */
	if (is_info) {
		buf_set_u32(algo_params.erase_param, 0, 32, FLASH_ERASE_INFO);
		/* And flag if BSL is unlocked */
		if (msp432_bank->unlock_bsl)
			buf_set_u32(algo_params.unlock_bsl, 0, 32, FLASH_UNLOCK_BSL);
	}

	/* Erase requested sectors one by one */
	for (unsigned int i = first; i <= last; i++) {

		/* Skip TVL (read-only) sector of the info bank */
		if (is_info && 1 == i)
			continue;

		/* Skip BSL sectors of info bank if locked */
		if (is_info && (2 == i || 3 == i) &&
			!msp432_bank->unlock_bsl)
			continue;

		/* Convert sector number to starting address of sector */
		buf_set_u32(algo_params.address, 0, 32, bank->base +
			(i * msp432_bank->sector_length));

		/* Issue the sector erase command to the flash helper algorithm */
		retval = msp432_exec_cmd(target, &algo_params, FLASH_SECTOR_ERASE);
		if (retval != ERROR_OK) {
			(void)msp432_quit(bank);
			return retval;
		}

		retval = msp432_wait_return_code(target);
		if (retval != ERROR_OK) {
			(void)msp432_quit(bank);
			return retval;
		}
	}

	retval = msp432_quit(bank);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int msp432_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct msp432_bank *msp432_bank = bank->driver_priv;
	struct msp432_algo_params algo_params;
	uint32_t size;
	uint32_t data_ready = BUFFER_DATA_READY;
	long long start_ms;
	long long elapsed_ms;

	bool is_info = bank->base == P4_FLASH_INFO_BASE;

	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/*
	 * Block attempts to write to read-only sectors of flash
	 * The TVL region in sector 1 of the info flash is always read-only
	 * The BSL region in sectors 2 and 3 of the info flash may be unlocked
	 * The helper algorithm will hang on attempts to write to TVL
	 */
	if (is_info) {
		/* Set read-only start to TVL sector */
		uint32_t start = 0x1000;
		/* Set read-only end after BSL region if locked */
		uint32_t end = (msp432_bank->unlock_bsl) ? 0x2000 : 0x4000;
		/* Check if request includes anything in read-only sectors */
		if ((offset + count - 1) < start || offset >= end) {
			/* The request includes no bytes in read-only sectors */
			/* Fall out and process the request normally */
		} else {
			/* Send a request for anything before read-only sectors */
			if (offset < start) {
				uint32_t start_count = MIN(start - offset, count);
				retval = msp432_write(bank, buffer, offset, start_count);
				if (retval != ERROR_OK)
					return retval;
			}
			/* Send a request for anything after read-only sectors */
			if ((offset + count - 1) >= end) {
				uint32_t skip = end - offset;
				count -= skip;
				offset += skip;
				buffer += skip;
				return msp432_write(bank, buffer, offset, count);
			} else {
				/* Request is entirely in read-only sectors */
				return ERROR_OK;
			}
		}
	}

	retval = msp432_init(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Initialize algorithm parameters to default values */
	msp432_init_params(&algo_params);

	/* Set up parameters for requested flash write operation */
	buf_set_u32(algo_params.address, 0, 32, bank->base + offset);
	buf_set_u32(algo_params.length, 0, 32, count);

	/* Check if this is the info bank */
	if (is_info) {
		/* And flag if BSL is unlocked */
		if (msp432_bank->unlock_bsl)
			buf_set_u32(algo_params.unlock_bsl, 0, 32, FLASH_UNLOCK_BSL);
	}

	/* Set up flash helper algorithm to continuous flash mode */
	retval = msp432_exec_cmd(target, &algo_params, FLASH_CONTINUOUS);
	if (retval != ERROR_OK) {
		(void)msp432_quit(bank);
		return retval;
	}

	/* Write requested data, one buffer at a time */
	start_ms = timeval_ms();
	while (count > 0) {

		if (count > ALGO_BUFFER_SIZE)
			size = ALGO_BUFFER_SIZE;
		else
			size = count;

		/* Put next block of data to flash into buffer */
		retval = target_write_buffer(target, ALGO_BUFFER1_ADDR, size, buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write data to target memory");
			(void)msp432_quit(bank);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		/* Signal the flash helper algorithm that data is ready to flash */
		retval = target_write_u32(target, ALGO_BUFFER1_STATUS_ADDR,
					data_ready);
		if (retval != ERROR_OK) {
			(void)msp432_quit(bank);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		retval = msp432_wait_inactive(target, 1);
		if (retval != ERROR_OK) {
			(void)msp432_quit(bank);
			return retval;
		}

		count -= size;
		buffer += size;

		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
	}

	/* Confirm that the flash helper algorithm is finished */
	retval = msp432_wait_return_code(target);
	if (retval != ERROR_OK) {
		(void)msp432_quit(bank);
		return retval;
	}

	retval = msp432_quit(bank);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int msp432_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct msp432_bank *msp432_bank = bank->driver_priv;

	uint32_t device_id;
	uint32_t hardware_rev;

	uint32_t sector_length;
	uint32_t size;
	unsigned int num_sectors;

	bool is_main = bank->base == FLASH_BASE;
	bool is_info = bank->base == P4_FLASH_INFO_BASE;

	int retval;

	/* Check if this bank has already been successfully probed */
	if (is_main && msp432_bank->probed_main)
		return ERROR_OK;
	if (is_info && msp432_bank->probed_info)
		return ERROR_OK;

	/* Read the flash size register to determine this is a P4 or not */
	/* MSP432P4s will return the size of flash.  MSP432E4s will return zero */
	retval = target_read_u32(target, P4_FLASH_MAIN_SIZE_REG, &size);
	if (retval != ERROR_OK)
		return retval;

	if (size == 0) {
		/* This is likely an MSP432E4 */
		msp432_bank->family_type = MSP432E4;

		retval = target_read_u32(target, E4_DID0_REG, &device_id);
		if (retval != ERROR_OK)
			return retval;

		msp432_bank->device_id = device_id;

		retval = target_read_u32(target, E4_DID1_REG, &hardware_rev);
		if (retval != ERROR_OK)
			return retval;

		msp432_bank->hardware_rev = hardware_rev;
	} else {
		/* This is likely an MSP432P4 */
		msp432_bank->family_type = MSP432P4;

		retval = target_read_u32(target, P4_DEVICE_ID_REG, &device_id);
		if (retval != ERROR_OK)
			return retval;

		msp432_bank->device_id = device_id & 0xFFFF;

		retval = target_read_u32(target, P4_HARDWARE_REV_REG, &hardware_rev);
		if (retval != ERROR_OK)
			return retval;

		msp432_bank->hardware_rev = hardware_rev & 0xFF;
	}

	msp432_bank->device_type = msp432_device_type(msp432_bank->family_type,
		msp432_bank->device_id, msp432_bank->hardware_rev);

	if (msp432_bank->family_type == MSP432P4) {
		/* Set up MSP432P4 specific flash parameters */
		if (is_main) {
			retval = target_read_u32(target, P4_FLASH_MAIN_SIZE_REG, &size);
			if (retval != ERROR_OK)
				return retval;

			sector_length = P4_SECTOR_LENGTH;
			num_sectors = size / sector_length;
		} else if (is_info) {
			if (msp432_bank->device_type == MSP432P411X ||
				msp432_bank->device_type == MSP432P411X_GUESS) {
				/* MSP432P411x has an info size register, use that for size */
				retval = target_read_u32(target, P4_FLASH_INFO_SIZE_REG, &size);
				if (retval != ERROR_OK)
					return retval;
			} else {
				/* All other MSP432P401x devices have fixed info region size */
				size = 0x4000; /* 16 KB info region */
			}
			sector_length = P4_SECTOR_LENGTH;
			num_sectors = size / sector_length;
		} else {
			/* Invalid bank somehow */
			return ERROR_FAIL;
		}
	} else {
		/* Set up MSP432E4 specific flash parameters */
		if (is_main) {
			size = E4_FLASH_SIZE;
			sector_length = E4_SECTOR_LENGTH;
			num_sectors = size / sector_length;
		} else {
			/* Invalid bank somehow */
			return ERROR_FAIL;
		}
	}

	free(bank->sectors);
	bank->sectors = NULL;

	if (num_sectors > 0) {
		bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
		if (!bank->sectors)
			return ERROR_FAIL;
	}

	bank->size = size;
	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;
	bank->num_sectors = num_sectors;
	msp432_bank->sector_length = sector_length;

	for (unsigned int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * sector_length;
		bank->sectors[i].size = sector_length;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	/* We've successfully determined the stats on this flash bank */
	if (is_main)
		msp432_bank->probed_main = true;
	if (is_info)
		msp432_bank->probed_info = true;

	if (is_main && MSP432P4 == msp432_bank->family_type) {
		/* Create the info flash bank needed by MSP432P4 variants */
		struct flash_bank *info = calloc(sizeof(struct flash_bank), 1);
		if (!info)
			return ERROR_FAIL;

		/* Create a name for the info bank, append "_1" to main name */
		char *name = malloc(strlen(bank->name) + 3);
		strcpy(name, bank->name);
		strcat(name, "_1");

		/* Initialize info bank */
		info->name = name;
		info->target = bank->target;
		info->driver = bank->driver;
		info->driver_priv = msp432_bank;
		info->base = P4_FLASH_INFO_BASE;

		flash_bank_add(info);
	}

	/* If we fall through to here, then all went well */

	return ERROR_OK;
}

static int msp432_auto_probe(struct flash_bank *bank)
{
	struct msp432_bank *msp432_bank = bank->driver_priv;

	bool is_main = bank->base == FLASH_BASE;
	bool is_info = bank->base == P4_FLASH_INFO_BASE;

	int retval = ERROR_OK;

	if (is_main)
		if (!msp432_bank->probed_main)
			retval = msp432_probe(bank);
	if (is_info)
		if (!msp432_bank->probed_info)
			retval = msp432_probe(bank);

	return retval;
}

static int msp432_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct msp432_bank *msp432_bank = bank->driver_priv;

	switch (msp432_bank->device_type) {
		case MSP432P401X_DEPR:
			if (msp432_bank->device_id == 0xFFFF) {
				/* Very early pre-production silicon currently deprecated */
				command_print_sameline(cmd, "MSP432P401x pre-production device (deprecated silicon)\n"
					SUPPORT_MESSAGE);
			} else {
				/* Revision A or B silicon, also deprecated */
				command_print_sameline(cmd, "MSP432P401x Device Rev %c (deprecated silicon)\n"
					SUPPORT_MESSAGE, (char)msp432_bank->hardware_rev);
			}
			break;
		case MSP432P401X:
			command_print_sameline(cmd, "MSP432P401x Device Rev %c\n",
				(char)msp432_bank->hardware_rev);
			break;
		case MSP432P411X:
			command_print_sameline(cmd, "MSP432P411x Device Rev %c\n",
				(char)msp432_bank->hardware_rev);
			break;
		case MSP432E401Y:
			command_print_sameline(cmd, "MSP432E401Y Device\n");
			break;
		case MSP432E411Y:
			command_print_sameline(cmd, "MSP432E411Y Device\n");
			break;
		case MSP432E4X_GUESS:
			command_print_sameline(cmd,
				"Unrecognized MSP432E4 DID0 and DID1 IDs (%08" PRIX32 ", %08" PRIX32 ")",
				msp432_bank->device_id, msp432_bank->hardware_rev);
			break;
		case MSP432P401X_GUESS:
		case MSP432P411X_GUESS:
		default:
			command_print_sameline(cmd,
				"Unrecognized MSP432P4 Device ID and Hardware Rev (%04" PRIX32 ", %02" PRIX32 ")",
				msp432_bank->device_id, msp432_bank->hardware_rev);
			break;
	}

	return ERROR_OK;
}

static int msp432_protect_check(struct flash_bank *bank)
{
	/* Added to suppress warning, not needed for MSP432 flash */
	return ERROR_OK;
}

static void msp432_flash_free_driver_priv(struct flash_bank *bank)
{
	bool is_main = bank->base == FLASH_BASE;

	/* A single private struct is shared between main and info banks */
	/* Only free it on the call for main bank */
	if (is_main)
		free(bank->driver_priv);

	/* Forget about the private struct on both main and info banks */
	bank->driver_priv = NULL;
}

static const struct command_registration msp432_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = msp432_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase entire flash memory on device.",
		.usage = "bank_id ['main' | 'all']",
	},
	{
		.name = "bsl",
		.handler = msp432_bsl_command,
		.mode = COMMAND_EXEC,
		.help = "Allow BSL to be erased or written by flash commands.",
		.usage = "bank_id ['unlock' | 'lock']",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration msp432_command_handlers[] = {
	{
		.name = "msp432",
		.mode = COMMAND_EXEC,
		.help = "MSP432 flash command group",
		.usage = "",
		.chain = msp432_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver msp432_flash = {
	.name = "msp432",
	.commands = msp432_command_handlers,
	.flash_bank_command = msp432_flash_bank_command,
	.erase = msp432_erase,
	.write = msp432_write,
	.read = default_flash_read,
	.probe = msp432_probe,
	.auto_probe = msp432_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = msp432_protect_check,
	.info = msp432_info,
	.free_driver_priv = msp432_flash_free_driver_priv,
};
