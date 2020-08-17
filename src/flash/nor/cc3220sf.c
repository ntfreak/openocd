/***************************************************************************
 *   Copyright (C) 2017 by Texas Instruments, Inc.                         *
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
#include "cc3220sf.h"
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLASH_TIMEOUT 5000

struct cc3220sf_bank {
	bool probed;
	struct armv7m_algorithm armv7m_info;
};

static int cc3220sf_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	bool done;
	long long start_ms;
	long long elapsed_ms;
	uint32_t value;

	int retval = ERROR_OK;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set starting address to erase to zero */
	retval = target_write_u32(target, FMA_REGISTER_ADDR, 0);
	if (ERROR_OK != retval)
		return retval;

	/* Write the MERASE bit of the FMC register */
	retval = target_write_u32(target, FMC_REGISTER_ADDR, FMC_MERASE_VALUE);
	if (ERROR_OK != retval)
		return retval;

	/* Poll the MERASE bit until the mass erase is complete */
	done = false;
	start_ms = timeval_ms();
	while (!done) {
		retval = target_read_u32(target, FMC_REGISTER_ADDR, &value);
		if (ERROR_OK != retval)
			return retval;

		if ((value & FMC_MERASE_BIT) == 0) {
			/* Bit clears when mass erase is finished */
			done = true;
		} else {
			elapsed_ms = timeval_ms() - start_ms;
			if (elapsed_ms > 500)
				keep_alive();
			if (elapsed_ms > FLASH_TIMEOUT)
				break;
		}
	}

	if (!done) {
		/* Mass erase timed out waiting for confirmation */
		return ERROR_FAIL;
	}

	return retval;
}

FLASH_BANK_COMMAND_HANDLER(cc3220sf_flash_bank_command)
{
	struct cc3220sf_bank *cc3220sf_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	cc3220sf_bank = malloc(sizeof(struct cc3220sf_bank));
	if (NULL == cc3220sf_bank)
		return ERROR_FAIL;

	/* Initialize private flash information */
	cc3220sf_bank->probed = false;

	/* Finish initialization of flash bank */
	bank->driver_priv = cc3220sf_bank;
	bank->next = NULL;

	return ERROR_OK;
}

static int cc3220sf_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	bool done;
	long long start_ms;
	long long elapsed_ms;
	uint32_t address;
	uint32_t value;

	int retval = ERROR_OK;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Do a mass erase if user requested all sectors of flash */
	if ((first == 0) && (last == (bank->num_sectors - 1))) {
		/* Request mass erase of flash */
		return cc3220sf_mass_erase(bank);
	}

	/* Erase requested sectors one by one */
	for (unsigned int i = first; i <= last; i++) {

		/* Determine address of sector to erase */
		address = FLASH_BASE_ADDR + i * FLASH_SECTOR_SIZE;

		/* Set starting address to erase */
		retval = target_write_u32(target, FMA_REGISTER_ADDR, address);
		if (ERROR_OK != retval)
			return retval;

		/* Write the ERASE bit of the FMC register */
		retval = target_write_u32(target, FMC_REGISTER_ADDR, FMC_ERASE_VALUE);
		if (ERROR_OK != retval)
			return retval;

		/* Poll the ERASE bit until the erase is complete */
		done = false;
		start_ms = timeval_ms();
		while (!done) {
			retval = target_read_u32(target, FMC_REGISTER_ADDR, &value);
			if (ERROR_OK != retval)
				return retval;

			if ((value & FMC_ERASE_BIT) == 0) {
				/* Bit clears when mass erase is finished */
				done = true;
			} else {
				elapsed_ms = timeval_ms() - start_ms;
				if (elapsed_ms > 500)
					keep_alive();
				if (elapsed_ms > FLASH_TIMEOUT)
					break;
			}
		}

		if (!done) {
			/* Sector erase timed out waiting for confirmation */
			return ERROR_FAIL;
		}
	}

	return retval;
}

static int cc3220sf_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct cc3220sf_bank *cc3220sf_bank = bank->driver_priv;
	struct working_area *algo_working_area;
	struct working_area *buffer_working_area;
	struct reg_param reg_params[3];
	uint32_t algo_base_address;
	uint32_t algo_buffer_address;
	uint32_t algo_buffer_size;
	uint32_t address;
	uint32_t remaining;
	uint32_t words;
	uint32_t result;

	int retval = ERROR_OK;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Obtain working area to use for flash helper algorithm */
	retval = target_alloc_working_area(target, sizeof(cc3220sf_algo),
				&algo_working_area);
	if (ERROR_OK != retval)
		return retval;

	/* Obtain working area to use for flash buffer */
	retval = target_alloc_working_area(target,
				target_get_working_area_avail(target), &buffer_working_area);
	if (ERROR_OK != retval) {
		target_free_working_area(target, algo_working_area);
		return retval;
	}

	algo_base_address = algo_working_area->address;
	algo_buffer_address = buffer_working_area->address;
	algo_buffer_size = buffer_working_area->size;

	/* Make sure buffer size is a multiple of 32 word (0x80 byte) chunks */
	/* (algo runs more efficiently if it operates on 32 words at a time) */
	if (algo_buffer_size > 0x80)
		algo_buffer_size &= ~0x7f;

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, algo_base_address,
				sizeof(cc3220sf_algo), cc3220sf_algo);
	if (ERROR_OK != retval) {
		target_free_working_area(target, algo_working_area);
		target_free_working_area(target, buffer_working_area);
		return retval;
	}

	/* Initialize the ARMv7m specific info to run the algorithm */
	cc3220sf_bank->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	cc3220sf_bank->armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Initialize register params for flash helper algorithm */
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);

	/* Prepare to write to flash */
	address = FLASH_BASE_ADDR + offset;
	remaining = count;

	/* The flash hardware can only write complete words to flash. If
	 * an unaligned address is passed in, we must do a read-modify-write
	 * on a word with enough bytes to align the rest of the buffer. And
	 * if less than a whole word remains at the end, we must also do a
	 * read-modify-write on a final word to finish up.
	 */

	/* Do one word write to align address on 32-bit boundary if needed */
	if (0 != (address & 0x3)) {
		uint8_t head[4];

		/* Get starting offset for data to write (will be 1 to 3) */
		uint32_t head_offset = address & 0x03;

		/* Get the aligned address to write this first word to */
		uint32_t head_address = address & 0xfffffffc;

		/* Retrieve what is already in flash at the head address */
		retval = target_read_buffer(target, head_address, sizeof(head), head);

		if (ERROR_OK == retval) {
			/* Substitute in the new data to write */
			while ((remaining > 0) && (head_offset < 4)) {
				head[head_offset] = *buffer;
				head_offset++;
				address++;
				buffer++;
				remaining--;
			}
		}

		if (ERROR_OK == retval) {
			/* Helper parameters are passed in registers R0-R2 */
			/* Set start of data buffer, address to write to, and word count */
			buf_set_u32(reg_params[0].value, 0, 32, algo_buffer_address);
			buf_set_u32(reg_params[1].value, 0, 32, head_address);
			buf_set_u32(reg_params[2].value, 0, 32, 1);

			/* Write head value into buffer to flash */
			retval = target_write_buffer(target, algo_buffer_address,
						sizeof(head), head);
		}

		if (ERROR_OK == retval) {
			/* Execute the flash helper algorithm */
			retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
						algo_base_address, 0, FLASH_TIMEOUT,
						&cc3220sf_bank->armv7m_info);
			if (ERROR_OK != retval)
				LOG_ERROR("cc3220sf: Flash algorithm failed to run");

			/* Check that the head value was written to flash */
			result = buf_get_u32(reg_params[2].value, 0, 32);
			if (0 != result) {
				retval = ERROR_FAIL;
				LOG_ERROR("cc3220sf: Flash operation failed");
			}
		}
	}

	/* Check if there's data at end of buffer that isn't a full word */
	uint32_t tail_count = remaining & 0x03;
	/* Adjust remaining so it is a multiple of whole words */
	remaining -= tail_count;

	while ((ERROR_OK == retval) && (remaining > 0)) {
		/* Set start of data buffer and address to write to */
		buf_set_u32(reg_params[0].value, 0, 32, algo_buffer_address);
		buf_set_u32(reg_params[1].value, 0, 32, address);

		/* Download data to write into memory buffer */
		if (remaining >= algo_buffer_size) {
			/* Fill up buffer with data to flash */
			retval = target_write_buffer(target, algo_buffer_address,
						algo_buffer_size, buffer);
			if (ERROR_OK != retval)
				break;

			/* Count to write is in 32-bit words */
			words = algo_buffer_size / 4;

			/* Bump variables to next data */
			address += algo_buffer_size;
			buffer += algo_buffer_size;
			remaining -= algo_buffer_size;
		} else {
			/* Fill buffer with what's left of the data */
			retval = target_write_buffer(target, algo_buffer_address,
						remaining, buffer);
			if (ERROR_OK != retval)
				break;

			/* Calculate the final word count to write */
			words = remaining / 4;
			if (0 != (remaining % 4))
				words++;

			/* Bump variables to any final data */
			address += remaining;
			buffer += remaining;
			remaining = 0;
		}

		/* Set number of words to write */
		buf_set_u32(reg_params[2].value, 0, 32, words);

		/* Execute the flash helper algorithm */
		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
					algo_base_address, 0, FLASH_TIMEOUT,
					&cc3220sf_bank->armv7m_info);
		if (ERROR_OK != retval) {
			LOG_ERROR("cc3220sf: Flash algorithm failed to run");
			break;
		}

		/* Check that all words were written to flash */
		result = buf_get_u32(reg_params[2].value, 0, 32);
		if (0 != result) {
			retval = ERROR_FAIL;
			LOG_ERROR("cc3220sf: Flash operation failed");
			break;
		}

		keep_alive();
	}

	/* Do one word write for any final bytes less than a full word */
	if ((ERROR_OK == retval) && (0 != tail_count)) {
		uint8_t tail[4];

		/* Set starting byte offset for data to write */
		uint32_t tail_offset = 0;

		/* Retrieve what is already in flash at the tail address */
		retval = target_read_buffer(target, address, sizeof(tail), tail);

		if (ERROR_OK == retval) {
			/* Substitute in the new data to write */
			while (tail_count > 0) {
				tail[tail_offset] = *buffer;
				tail_offset++;
				buffer++;
				tail_count--;
			}
		}

		if (ERROR_OK == retval) {
			/* Set start of data buffer, address to write to, and word count */
			buf_set_u32(reg_params[0].value, 0, 32, algo_buffer_address);
			buf_set_u32(reg_params[1].value, 0, 32, address);
			buf_set_u32(reg_params[2].value, 0, 32, 1);

			/* Write tail value into buffer to flash */
			retval = target_write_buffer(target, algo_buffer_address,
						sizeof(tail), tail);
		}

		if (ERROR_OK == retval) {
			/* Execute the flash helper algorithm */
			retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
						algo_base_address, 0, FLASH_TIMEOUT,
						&cc3220sf_bank->armv7m_info);
			if (ERROR_OK != retval)
				LOG_ERROR("cc3220sf: Flash algorithm failed to run");

			/* Check that the tail was written to flash */
			result = buf_get_u32(reg_params[2].value, 0, 32);
			if (0 != result) {
				retval = ERROR_FAIL;
				LOG_ERROR("cc3220sf: Flash operation failed");
			}
		}
	}

	/* Free resources  */
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	target_free_working_area(target, algo_working_area);
	target_free_working_area(target, buffer_working_area);

	return retval;
}

static int cc3220sf_probe(struct flash_bank *bank)
{
	struct cc3220sf_bank *cc3220sf_bank = bank->driver_priv;

	uint32_t base;
	uint32_t size;
	unsigned int num_sectors;

	base = FLASH_BASE_ADDR;
	size = FLASH_NUM_SECTORS * FLASH_SECTOR_SIZE;
	num_sectors = FLASH_NUM_SECTORS;

	free(bank->sectors);

	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (NULL == bank->sectors)
		return ERROR_FAIL;

	bank->base = base;
	bank->size = size;
	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;
	bank->num_sectors = num_sectors;

	for (unsigned int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
		bank->sectors[i].size = FLASH_SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	/* We've successfully recorded the stats on this flash bank */
	cc3220sf_bank->probed = true;

	/* If we fall through to here, then all went well */

	return ERROR_OK;
}

static int cc3220sf_auto_probe(struct flash_bank *bank)
{
	struct cc3220sf_bank *cc3220sf_bank = bank->driver_priv;

	int retval = ERROR_OK;

	if (!cc3220sf_bank->probed)
		retval = cc3220sf_probe(bank);

	return retval;
}

static int cc3220sf_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed;

	printed = snprintf(buf, buf_size, "CC3220SF with 1MB internal flash\n");

	if (printed >= buf_size)
		return ERROR_BUF_TOO_SMALL;

	return ERROR_OK;
}

const struct flash_driver cc3220sf_flash = {
	.name = "cc3220sf",
	.flash_bank_command = cc3220sf_flash_bank_command,
	.erase = cc3220sf_erase,
	.write = cc3220sf_write,
	.read = default_flash_read,
	.probe = cc3220sf_probe,
	.auto_probe = cc3220sf_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = cc3220sf_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
