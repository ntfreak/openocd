/***************************************************************************
 *   Copyright (C) 2008 by Kevin McGuire                                   *
 *   Copyright (C) 2008 by Marcel Wijlaars                                 *
 *   Copyright (C) 2009 by Michael Ashton                                  *
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
#include <target/algorithm.h>
#include <target/arm.h>

static int aduc702x_build_sector_list(struct flash_bank *bank);
static int aduc702x_check_flash_completion(struct target *target, unsigned int timeout_ms);
static int aduc702x_set_write_enable(struct target *target, int enable);

#define ADUC702X_FLASH                          0xfffff800
#define ADUC702X_FLASH_FEESTA           (0*4)
#define ADUC702X_FLASH_FEEMOD           (1*4)
#define ADUC702X_FLASH_FEECON           (2*4)
#define ADUC702X_FLASH_FEEDAT           (3*4)
#define ADUC702X_FLASH_FEEADR           (4*4)
#define ADUC702X_FLASH_FEESIGN          (5*4)
#define ADUC702X_FLASH_FEEPRO           (6*4)
#define ADUC702X_FLASH_FEEHIDE          (7*4)

/* flash bank aduc702x 0 0 0 0 <target#>
 * The ADC7019-28 devices all have the same flash layout */
FLASH_BANK_COMMAND_HANDLER(aduc702x_flash_bank_command)
{
	bank->base = 0x80000;
	bank->size = 0xF800;	/* top 4k not accessible */

	aduc702x_build_sector_list(bank);

	return ERROR_OK;
}

static int aduc702x_build_sector_list(struct flash_bank *bank)
{
	/* aduc7026_struct flash_bank *aduc7026_info = bank->driver_priv; */

	uint32_t offset = 0;

	/* sector size is 512 */
	bank->num_sectors = bank->size / 512;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	for (unsigned int i = 0; i < bank->num_sectors; ++i) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = 512;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	return ERROR_OK;
}

static int aduc702x_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	/* int res; */
	int x;
	int count;
	/* uint32_t v; */
	struct target *target = bank->target;

	aduc702x_set_write_enable(target, 1);

	/* mass erase */
	if (((first | last) == 0) || ((first == 0) && (last >= bank->num_sectors))) {
		LOG_DEBUG("performing mass erase.");
		target_write_u16(target, ADUC702X_FLASH + ADUC702X_FLASH_FEEDAT, 0x3cff);
		target_write_u16(target, ADUC702X_FLASH + ADUC702X_FLASH_FEEADR, 0xffc3);
		target_write_u8(target, ADUC702X_FLASH + ADUC702X_FLASH_FEECON, 0x06);

		if (aduc702x_check_flash_completion(target, 3500) != ERROR_OK) {
			LOG_ERROR("mass erase failed");
			aduc702x_set_write_enable(target, 0);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		LOG_DEBUG("mass erase successful.");
		return ERROR_OK;
	} else {
		unsigned long adr;

		count = last - first + 1;
		for (x = 0; x < count; ++x) {
			adr = bank->base + ((first + x) * 512);

			target_write_u16(target, ADUC702X_FLASH + ADUC702X_FLASH_FEEADR, adr);
			target_write_u8(target, ADUC702X_FLASH + ADUC702X_FLASH_FEECON, 0x05);

			if (aduc702x_check_flash_completion(target, 50) != ERROR_OK) {
				LOG_ERROR("failed to erase sector at address 0x%08lX", adr);
				aduc702x_set_write_enable(target, 0);
				return ERROR_FLASH_SECTOR_NOT_ERASED;
			}

			LOG_DEBUG("erased sector at address 0x%08lX", adr);
		}
	}

	aduc702x_set_write_enable(target, 0);

	return ERROR_OK;
}

/* If this fn returns ERROR_TARGET_RESOURCE_NOT_AVAILABLE, then the caller can fall
 * back to another mechanism that does not require onboard RAM
 *
 * Caller should not check for other return values specifically
 */
static int aduc702x_write_block(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 7000;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[6];
	struct arm_algorithm arm_algo;
	int retval = ERROR_OK;

	if (((count%2) != 0) || ((offset%2) != 0)) {
		LOG_ERROR("write block must be multiple of two bytes in offset & length");
		return ERROR_FAIL;
	}

	/* parameters:

	r0 - address of source data (absolute)
	r1 - number of halfwords to be copied
	r2 - start address in flash (offset from beginning of flash memory)
	r3 - exit code
	r4 - base address of flash controller (0xFFFFF800)

	registers:

	r5 - scratch
	r6 - set to 2, used to write flash command

	*/
	static const uint32_t aduc702x_flash_write_code[] = {
		/* <_start>: */
		0xe3a05008,	/* mov	r5, #8	; 0x8 */
		0xe5845004,	/* str	r5, [r4, #4] */
		0xe3a06002,	/* mov	r6, #2	; 0x2 */
		/* <next>: */
		0xe1c421b0,	/* strh	r2, [r4, #16] */
		0xe0d050b2,	/* ldrh	r5, [r0], #2 */
		0xe1c450bc,	/* strh	r5, [r4, #12] */
		0xe5c46008,	/* strb	r6, [r4, #8] */
		/* <wait_complete>: */
		0xe1d430b0,	/* ldrh	r3, [r4] */
		0xe3130004,	/* tst	r3, #4	; 0x4 */
		0x1afffffc,	/* bne	1001c <wait_complete> */
		0xe2822002,	/* add	r2, r2, #2	; 0x2 */
		0xe2511001,	/* subs	r1, r1, #1	; 0x1 */
		0x0a000001,	/* beq	1003c <done> */
		0xe3130001,	/* tst	r3, #1	; 0x1 */
		0x1afffff3,	/* bne	1000c <next> */
		/* <done>: */
		0xeafffffe	/* b	1003c <done> */
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(aduc702x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	uint8_t code[sizeof(aduc702x_flash_write_code)];
	target_buffer_set_u32_array(target, code, ARRAY_SIZE(aduc702x_flash_write_code),
			aduc702x_flash_write_code);
	retval = target_write_buffer(target, write_algorithm->address, sizeof(code), code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a buffer,
			 *free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	arm_algo.common_magic = ARM_COMMON_MAGIC;
	arm_algo.core_mode = ARM_MODE_SVC;
	arm_algo.core_state = ARM_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);

	while (count > 0) {
		uint32_t thisrun_count = (count > buffer_size) ? buffer_size : count;

		retval = target_write_buffer(target, source->address, thisrun_count, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, thisrun_count/2);
		buf_set_u32(reg_params[2].value, 0, 32, address);
		buf_set_u32(reg_params[4].value, 0, 32, 0xFFFFF800);

		retval = target_run_algorithm(target, 0, NULL, 5,
				reg_params, write_algorithm->address,
				write_algorithm->address +
				sizeof(aduc702x_flash_write_code) - 4,
				10000, &arm_algo);
		if (retval != ERROR_OK) {
			LOG_ERROR("error executing aduc702x flash write algorithm");
			break;
		}

		if ((buf_get_u32(reg_params[3].value, 0, 32) & 1) != 1) {
			/* FIX!!!! what does this mean??? replace w/sensible error message */
			LOG_ERROR("aduc702x detected error writing flash");
			retval = ERROR_FAIL;
			break;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

/* All-JTAG, single-access method.  Very slow.  Used only if there is no
 * working area available. */
static int aduc702x_write_single(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	uint32_t x;
	uint8_t b;
	struct target *target = bank->target;

	aduc702x_set_write_enable(target, 1);

	for (x = 0; x < count; x += 2) {
		/* FEEADR = address */
		target_write_u16(target, ADUC702X_FLASH + ADUC702X_FLASH_FEEADR, offset + x);

		/* set up data */
		if ((x + 1) == count) {
			/* last byte */
			target_read_u8(target, offset + x + 1, &b);
		} else
			b = buffer[x + 1];

		target_write_u16(target, ADUC702X_FLASH + ADUC702X_FLASH_FEEDAT, buffer[x] | (b << 8));

		/* do single-write command */
		target_write_u8(target, ADUC702X_FLASH + ADUC702X_FLASH_FEECON, 0x02);

		if (aduc702x_check_flash_completion(target, 1) != ERROR_OK) {
			LOG_ERROR("single write failed for address 0x%08lX",
				(unsigned long)(offset + x));
			aduc702x_set_write_enable(target, 0);
			return ERROR_FLASH_OPERATION_FAILED;
		}

	}
	LOG_DEBUG("wrote %d bytes at address 0x%08lX", (int)count, (unsigned long)(offset + x));

	aduc702x_set_write_enable(target, 0);

	return ERROR_OK;
}

static int aduc702x_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	/* try using a block write */
	retval = aduc702x_write_block(bank, buffer, offset, count);
	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* if block write failed (no sufficient working area),
			 * use normal (slow) JTAG method */
			LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

			retval = aduc702x_write_single(bank, buffer, offset, count);
			if (retval != ERROR_OK) {
				LOG_ERROR("slow write failed");
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
	}

	return retval;
}

static int aduc702x_probe(struct flash_bank *bank)
{
	return ERROR_OK;
}

/* sets FEEMOD bit 3
 * enable = 1 enables writes & erases, 0 disables them */
static int aduc702x_set_write_enable(struct target *target, int enable)
{
	/* don't bother to preserve int enable bit here */
	target_write_u16(target, ADUC702X_FLASH + ADUC702X_FLASH_FEEMOD, enable ? 8 : 0);

	return ERROR_OK;
}

/* wait up to timeout_ms for controller to not be busy,
 * then check whether the command passed or failed.
 *
 * this function sleeps 1ms between checks (after the first one),
 * so in some cases may slow things down without a usleep after the first read */
static int aduc702x_check_flash_completion(struct target *target, unsigned int timeout_ms)
{
	uint8_t v = 4;

	int64_t endtime = timeval_ms() + timeout_ms;
	while (1) {
		target_read_u8(target, ADUC702X_FLASH + ADUC702X_FLASH_FEESTA, &v);
		if ((v & 4) == 0)
			break;
		alive_sleep(1);
		if (timeval_ms() >= endtime)
			break;
	}

	if (v & 2)
		return ERROR_FAIL;
	/* if a command is ignored, both the success and fail bits may be 0 */
	else if ((v & 3) == 0)
		return ERROR_FAIL;
	else
		return ERROR_OK;
}

const struct flash_driver aduc702x_flash = {
	.name = "aduc702x",
	.flash_bank_command = aduc702x_flash_bank_command,
	.erase = aduc702x_erase,
	.write = aduc702x_write,
	.read = default_flash_read,
	.probe = aduc702x_probe,
	.auto_probe = aduc702x_probe,
	.erase_check = default_flash_blank_check,
};
