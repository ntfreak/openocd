/***************************************************************************
 *   Copyright (C) 2015 by Ivan Buliev                                     *
 *   i.buliev@mikrosistemi.com                                             *
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

/***************************************************************************
 *  This version for ADuCM360 is largely based on the following flash      *
 *  drivers:                                                               *
 *    - aduc702x.c                                                         *
 *          Copyright (C) 2008 by Kevin McGuire                            *
 *          Copyright (C) 2008 by Marcel Wijlaars                          *
 *          Copyright (C) 2009 by Michael Ashton                           *
 *   and                                                                   *
 *    - stm32f1x.c                                                         *
 *          Copyright (C) 2005 by Dominic Rath                             *
 *          Dominic.Rath@gmx.de                                            *
 *                                                                         *
 *          Copyright (C) 2008 by Spencer Oliver                           *
 *          spen@spen-soft.co.uk                                           *
 *                                                                         *
 *          Copyright (C) 2011 by Andreas Fritiofson                       *
 *          andreas.fritiofson@gmail.com                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

static int aducm360_build_sector_list(struct flash_bank *bank);
static int aducm360_check_flash_completion(struct target *target, unsigned int timeout_ms);
static int aducm360_set_write_enable(struct target *target, int enable);

#define ADUCM360_FLASH_BASE             0x40002800
#define ADUCM360_FLASH_FEESTA           0x0000
#define ADUCM360_FLASH_FEECON0          0x0004
#define ADUCM360_FLASH_FEECMD           0x0008
#define ADUCM360_FLASH_FEEADR0L         0x0010
#define ADUCM360_FLASH_FEEADR0H         0x0014
#define ADUCM360_FLASH_FEEADR1L         0x0018
#define ADUCM360_FLASH_FEEADR1H         0x001C
#define ADUCM360_FLASH_FEEKEY           0x0020
#define ADUCM360_FLASH_FEEPROL          0x0028
#define ADUCM360_FLASH_FEEPROH          0x002C
#define ADUCM360_FLASH_FEESIGL          0x0030
#define ADUCM360_FLASH_FEESIGH          0x0034
#define ADUCM360_FLASH_FEECON1          0x0038
#define ADUCM360_FLASH_FEEADRAL         0x0048
#define ADUCM360_FLASH_FEEADRAH         0x004C
#define ADUCM360_FLASH_FEEAEN0          0x0078
#define ADUCM360_FLASH_FEEAEN1          0x007C
#define ADUCM360_FLASH_FEEAEN2          0x0080

/* flash bank aducm360 0 0 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(aducm360_flash_bank_command)
{
	bank->base = 0x00000000;
	bank->size = 0x00020000;

	aducm360_build_sector_list(bank);

	return ERROR_OK;
}

#define FLASH_SECTOR_SIZE	512

/* ----------------------------------------------------------------------- */
static int aducm360_build_sector_list(struct flash_bank *bank)
{
	uint32_t offset = 0;

	/* sector size is 512 */
	bank->num_sectors = bank->size / FLASH_SECTOR_SIZE;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	for (unsigned i = 0; i < bank->num_sectors; ++i) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = FLASH_SECTOR_SIZE;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	return ERROR_OK;
}

/* ----------------------------------------------------------------------- */
static int aducm360_mass_erase(struct target *target)
{
	uint32_t		value;
	int				res = ERROR_OK;

	/* Clear any old status */
	target_read_u32(target, ADUCM360_FLASH_BASE + ADUCM360_FLASH_FEESTA, &value);

	/* Enable the writing to the flash*/
	aducm360_set_write_enable(target, 1);

	/* Unlock for writing */
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEEKEY, 0x0000F456);
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEEKEY, 0x0000F123);
	/* Issue the 'MASSERASE' command */
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEECMD, 0x00000003);

	/* Check the result */
	res = aducm360_check_flash_completion(target, 3500);
	if (res != ERROR_OK) {
		LOG_ERROR("mass erase failed.");
		aducm360_set_write_enable(target, 0);
		res = ERROR_FLASH_OPERATION_FAILED;
	}

	return res;
}

/* ----------------------------------------------------------------------- */
static int aducm360_page_erase(struct target *target, uint32_t padd)
{
	uint32_t		value;
	int				res = ERROR_OK;

	/* Clear any old status */
	target_read_u32(target, ADUCM360_FLASH_BASE + ADUCM360_FLASH_FEESTA, &value);

	/* Enable the writing to the flash*/
	aducm360_set_write_enable(target, 1);

	/* Unlock for writing */
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEEKEY, 0x0000F456);
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEEKEY, 0x0000F123);
	/* Write the sector address */
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEEADR0L, padd & 0xFFFF);
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEEADR0H, (padd>>16) & 0xFFFF);
	/* Issue the 'ERASEPAGE' command */
	target_write_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEECMD, 0x00000001);

	/* Check the result */
	res = aducm360_check_flash_completion(target, 50);
	if (res != ERROR_OK) {
		LOG_ERROR("page erase failed at 0x%08" PRIx32, padd);
		aducm360_set_write_enable(target, 0);
		res = ERROR_FLASH_OPERATION_FAILED;
	}

	return res;
}

/* ----------------------------------------------------------------------- */
static int aducm360_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int             res = ERROR_OK;
	int             i;
	int             count;
	struct target   *target = bank->target;
	uint32_t        padd;

	if (((first | last) == 0) || ((first == 0) && (last >= bank->num_sectors))) {
		res = aducm360_mass_erase(target);
	} else {
		count = last - first + 1;
		for (i = 0; i < count; ++i) {
			padd = bank->base + ((first+i)*FLASH_SECTOR_SIZE);
			res = aducm360_page_erase(target, padd);
			if (res != ERROR_OK)
				break;
		}
	}

	return res;
}

/* ----------------------------------------------------------------------- */
static int aducm360_write_block_sync(
		struct flash_bank *bank,
		const uint8_t *buffer,
		uint32_t offset,
		uint32_t count)
{
	struct target           *target = bank->target;
	uint32_t                target_buffer_size = 8192;
	struct working_area     *helper;
	struct working_area     *target_buffer;
	uint32_t                address = bank->base + offset;
	struct reg_param        reg_params[8];
	int                     retval = ERROR_OK;
	uint32_t                entry_point = 0, exit_point = 0;
	uint32_t                res;
	struct armv7m_algorithm armv7m_algo;

	static const uint32_t aducm360_flash_write_code[] = {
			/* helper.code */
			0x88AF4D10, 0x0704F047, 0x682F80AF, 0x600E6806,
			0xF017882F, 0xF43F0F08, 0xF851AFFB, 0x42B77B04,
			0x800DF040, 0x0004F100, 0xF47F3A04, 0x686FAFEF,
			0x0704F027, 0xF04F80AF, 0xF0000400, 0xF04FB802,
			0xBE000480, 0x40002800, 0x00015000, 0x20000000,
			0x00013000
	};

	LOG_DEBUG("'aducm360_write_block_sync' requested, dst:0x%08" PRIx32 ", count:0x%08" PRIx32 "bytes.",
			address, count);

	/*  ----- Check the destination area for a Long Word alignment -----  */
	if (((count%4) != 0) || ((offset%4) != 0)) {
		LOG_ERROR("write block must be multiple of four bytes in offset & length");
		return ERROR_FAIL;
	}

	/*  ----- Allocate space in the target's RAM for the helper code -----  */
	if (target_alloc_working_area(target, sizeof(aducm360_flash_write_code),
			&helper) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/*  ----- Upload the helper code to the space in the target's RAM -----  */
	uint8_t code[sizeof(aducm360_flash_write_code)];
	target_buffer_set_u32_array(target, code, ARRAY_SIZE(aducm360_flash_write_code),
			aducm360_flash_write_code);
	retval = target_write_buffer(target, helper->address, sizeof(code), code);
	if (retval != ERROR_OK)
		return retval;
	entry_point = helper->address;

	/*  ----- Allocate space in the target's RAM for the user application's object code -----  */
	while (target_alloc_working_area_try(target, target_buffer_size, &target_buffer) != ERROR_OK) {
		LOG_WARNING("couldn't allocate a buffer space of 0x%08" PRIx32 "bytes in the target's SRAM.",
				target_buffer_size);
		target_buffer_size /= 2;
		if (target_buffer_size <= 256) {		/* No room available */
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			target_free_working_area(target, helper);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	/* ----- Prepare the target for the helper ----- */
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /*SRC      */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /*DST      */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /*COUNT    */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT); /*not used */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN);	 /*RESULT   */

	/*  ===== Execute the Main Programming Loop! ===== */
	while (count > 0) {
		uint32_t thisrun_count = (count > target_buffer_size) ? target_buffer_size : count;

		/* ----- Upload the chunk ----- */
		retval = target_write_buffer(target, target_buffer->address, thisrun_count, buffer);
		if (retval != ERROR_OK)
			break;
		/* Set the arguments for the helper */
		buf_set_u32(reg_params[0].value, 0, 32, target_buffer->address);	/*SRC     */
		buf_set_u32(reg_params[1].value, 0, 32, address);					/*DST     */
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);				/*COUNT   */
		buf_set_u32(reg_params[3].value, 0, 32, 0);							/*NOT USED*/

		retval = target_run_algorithm(target, 0, NULL, 5,
				reg_params,	entry_point, exit_point, 10000, &armv7m_algo);
		if (retval != ERROR_OK) {
			LOG_ERROR("error executing aducm360 flash write algorithm");
			break;
		}

		res = buf_get_u32(reg_params[4].value, 0, 32);
		if (res) {
			LOG_ERROR("aducm360 fast sync algorithm reports an error (%02" PRIX32 ")", res);
			retval = ERROR_FAIL;
			break;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}

	target_free_working_area(target, target_buffer);
	target_free_working_area(target, helper);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

/* ----------------------------------------------------------------------- */
static int aducm360_write_block_async(
		struct flash_bank *bank,
		const uint8_t *buffer,
		uint32_t offset,
		uint32_t count)
{
	struct target           *target = bank->target;
	uint32_t                target_buffer_size = 1024;
	struct working_area     *helper;
	struct working_area     *target_buffer;
	uint32_t                address = bank->base + offset;
	struct reg_param        reg_params[9];
	int                     retval = ERROR_OK;
	uint32_t                entry_point = 0, exit_point = 0;
	uint32_t                res;
	uint32_t                wcount;
	struct armv7m_algorithm armv7m_algo;

	static const uint32_t aducm360_flash_write_code[] = {
			/* helper.code */
			0x4050F8DF,	0xF04588A5,	0x80A50504,	0x8000F8D0,
			0x0F00F1B8, 0x8016F000,	0x45476847,	0xAFF6F43F,
			0x6B04F857, 0x6B04F842,	0xF0158825,	0xF43F0F08,
			0x428FAFFB, 0xF100BF28,	0x60470708,	0xB10B3B01,
			0xBFE4F7FF, 0xF02588A5,	0x80A50504,	0x0900F04F,
			0xBE00BF00, 0x40002800,	0x20000000,	0x20000100,
			0x00013000
	};

	LOG_DEBUG("'aducm360_write_block_async' requested, dst:0x%08" PRIx32 ", count:0x%08" PRIx32 "bytes.",
			address, count);

	/*  ----- Check the destination area for a Long Word alignment -----  */
	if (((count%4) != 0) || ((offset%4) != 0)) {
		LOG_ERROR("write block must be multiple of four bytes in offset & length");
		return ERROR_FAIL;
	}
	wcount = count/4;

	/*  ----- Allocate space in the target's RAM for the helper code -----  */
	if (target_alloc_working_area(target, sizeof(aducm360_flash_write_code),
			&helper) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/*  ----- Upload the helper code to the space in the target's RAM -----  */
	uint8_t code[sizeof(aducm360_flash_write_code)];
	target_buffer_set_u32_array(target, code, ARRAY_SIZE(aducm360_flash_write_code),
			aducm360_flash_write_code);
	retval = target_write_buffer(target, helper->address, sizeof(code), code);
	if (retval != ERROR_OK)
		return retval;
	entry_point = helper->address;

	/*  ----- Allocate space in the target's RAM for the user application's object code ----- */
	while (target_alloc_working_area_try(target, target_buffer_size, &target_buffer) != ERROR_OK) {
		LOG_WARNING("couldn't allocate a buffer space of 0x%08" PRIx32 "bytes in the target's SRAM.",
				target_buffer_size);
		target_buffer_size /= 2;
		if (target_buffer_size <= 256) {		/* No room available */
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			target_free_working_area(target, helper);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	/* ----- Prepare the target for the helper ----- */
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /*SRCBEG     */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /*SRCEND     */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /*DST        */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT); /*COUNT (LWs)*/
	init_reg_param(&reg_params[4], "r9", 32, PARAM_IN);  /*RESULT     */

	buf_set_u32(reg_params[0].value, 0, 32, target_buffer->address);
	buf_set_u32(reg_params[1].value, 0, 32, target_buffer->address + target_buffer->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, wcount);

	retval = target_run_flash_async_algorithm(target, buffer, wcount, 4,
			0, NULL,
			5, reg_params,
			target_buffer->address, target_buffer->size,
			entry_point, exit_point,
			&armv7m_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("error executing aducm360 flash write algorithm");
	} else {
		res = buf_get_u32(reg_params[4].value, 0, 32);	/*RESULT*/
		if (res) {
			LOG_ERROR("aducm360 fast async algorithm reports an error (%02" PRIX32 ")", res);
			retval = ERROR_FAIL;
		}
	}

	target_free_working_area(target, target_buffer);
	target_free_working_area(target, helper);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

/* ----------------------------------------------------------------------- */
/* If this fn returns ERROR_TARGET_RESOURCE_NOT_AVAILABLE, then the caller can fall
 * back to another mechanism that does not require onboard RAM
 *
 * Caller should not check for other return values specifically
 */
static int aducm360_write_block(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	int	choice = 0;

	switch (choice) {
	case 0:
		return aducm360_write_block_sync(bank, buffer, offset, count);
	case 1:
		return aducm360_write_block_async(bank, buffer, offset, count);
	default:
		LOG_ERROR("aducm360_write_block was cancelled (no writing method was chosen)!");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
}

/* ----------------------------------------------------------------------- */
#define FEESTA_WRDONE	0x00000008

static int aducm360_write_modified(struct flash_bank *bank,
		const uint8_t *buffer,
		uint32_t offset,
		uint32_t count)
{
	uint32_t		value;
	int				res = ERROR_OK;
	uint32_t        i, j, a, d;
	struct target   *target = bank->target;

	LOG_DEBUG("performing slow write (offset=0x%08" PRIx32 ", count=0x%08" PRIx32 ")...",
			offset, count);

	/* Enable the writing to the flash */
	aducm360_set_write_enable(target, 1);

	/* Clear any old status */
	target_read_u32(target, ADUCM360_FLASH_BASE + ADUCM360_FLASH_FEESTA, &value);

	for (i = 0; i < count; i += 4) {
		a = offset+i;
		for (j = 0; i < 4; i += 1)
			*((uint8_t *)(&d) + j) = buffer[i+j];
		target_write_u32(target, a, d);
		do {
			target_read_u32(target, ADUCM360_FLASH_BASE + ADUCM360_FLASH_FEESTA, &value);
		} while (!(value & FEESTA_WRDONE));
	}
	aducm360_set_write_enable(target, 0);

	return res;
}

/* ----------------------------------------------------------------------- */
static int aducm360_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	/* try using a block write */
	retval = aducm360_write_block(bank, buffer, offset, count);
	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* if block write failed (no sufficient working area),
			 * use normal (slow) JTAG method */
			LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

			retval = aducm360_write_modified(bank, buffer, offset, count);
			if (retval != ERROR_OK) {
				LOG_ERROR("slow write failed");
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
	}
	return retval;
}

/* ----------------------------------------------------------------------- */
static int aducm360_probe(struct flash_bank *bank)
{
	return ERROR_OK;
}

/* ----------------------------------------------------------------------- */
/* sets FEECON0 bit 2
 * enable = 1 enables writes & erases, 0 disables them */
static int aducm360_set_write_enable(struct target *target, int enable)
{
	/* don't bother to preserve int enable bit here */
	uint32_t	value;

	target_read_u32(target, ADUCM360_FLASH_BASE + ADUCM360_FLASH_FEECON0, &value);
	if (enable)
		value |= 0x00000004;
	else
		value &= ~0x00000004;
	target_write_u32(target, ADUCM360_FLASH_BASE + ADUCM360_FLASH_FEECON0, value);

	return ERROR_OK;
}

/* ----------------------------------------------------------------------- */
/* wait up to timeout_ms for controller to not be busy,
 * then check whether the command passed or failed.
 *
 * this function sleeps 1ms between checks (after the first one),
 * so in some cases may slow things down without a usleep after the first read */
static int aducm360_check_flash_completion(struct target *target, unsigned int timeout_ms)
{
	uint32_t v = 1;

	int64_t endtime = timeval_ms() + timeout_ms;
	while (1) {
		target_read_u32(target, ADUCM360_FLASH_BASE+ADUCM360_FLASH_FEESTA, &v);
		if ((v & 0x00000001) == 0)
			break;
		alive_sleep(1);
		if (timeval_ms() >= endtime)
			break;
	}

	if (!(v & 0x00000004))	/* b2 */
		return ERROR_FAIL;

	return ERROR_OK;
}

/* ----------------------------------------------------------------------- */
const struct flash_driver aducm360_flash = {
	.name = "aducm360",
	.flash_bank_command = aducm360_flash_bank_command,
	.erase = aducm360_erase,
	.write = aducm360_write,
	.read = default_flash_read,
	.probe = aducm360_probe,
	.auto_probe = aducm360_probe,
	.erase_check = default_flash_blank_check,
};
