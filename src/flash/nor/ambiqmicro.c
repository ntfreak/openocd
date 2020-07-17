/******************************************************************************
 *
 * @file ambiqmicro.c
 *
 * @brief Ambiq Micro flash driver.
 *
 *****************************************************************************/

/******************************************************************************
 * Copyright (c) 2015, David Racine <dracine at ambiqmicro.com>
 *
 * Copyright (c) 2016, Rick Foos <rfoos at solengtech.com>
 *
 * Copyright (c) 2015-2016, Ambiq Micro, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "imp.h"
#include "target/algorithm.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"

/** Check error, log error. */
#define CHECK_STATUS(rc, msg) {	\
		if (rc != ERROR_OK) { \
			LOG_ERROR("status(%d):%s\n", rc, msg); } }

/*
 * Address and Key defines.
 */
#define PROGRAM_KEY      (0x12344321)
#define OTP_PROGRAM_KEY  (0x87655678)

#define FLASH_PROGRAM_MAIN_FROM_SRAM                0x0800005d
#define FLASH_PROGRAM_OTP_FROM_SRAM                 0x08000061
#define FLASH_ERASE_LIST_MAIN_PAGES_FROM_SRAM       0x08000065
#define FLASH_MASS_ERASE_MAIN_PAGES_FROM_SRAM       0x08000069


static const uint32_t apollo_flash_size[] = {
	1 << 15,
	1 << 16,
	1 << 17,
	1 << 18,
	1 << 19,
	1 << 20,
	1 << 21
};

static const uint32_t apollo_sram_size[] = {
	1 << 15,
	1 << 16,
	1 << 17,
	1 << 18,
	1 << 19,
	1 << 20,
	1 << 21
};

struct ambiqmicro_flash_bank {
	/* chip id register */

	bool probed;

	const char *target_name;
	uint8_t target_class;

	uint32_t sramsiz;
	uint32_t flshsiz;

	/* flash geometry */
	uint32_t num_pages;
	uint32_t pagesize;
	uint32_t pages_in_lockregion;

	/* nv memory bits */
	uint16_t num_lockbits;

	/* main clock status */
	uint32_t rcc;
	uint32_t rcc2;
	uint8_t mck_valid;
	uint8_t xtal_mask;
	uint32_t iosc_freq;
	uint32_t mck_freq;
	const char *iosc_desc;
	const char *mck_desc;
};

static struct {
	uint8_t class;
	uint8_t partno;
	const char *partname;
} ambiqmicroParts[6] = {
	{0xFF, 0x00, "Unknown"},
	{0x01, 0x00, "Apollo"},
	{0x02, 0x00, "Apollo2"},
	{0x03, 0x00, "Unknown"},
	{0x04, 0x00, "Unknown"},
	{0x05, 0x00, "Apollo"},
};

static char *ambiqmicroClassname[6] = {
	"Unknown", "Apollo", "Apollo2", "Unknown", "Unknown", "Apollo"
};

/***************************************************************************
*	openocd command interface                                              *
***************************************************************************/

/* flash_bank ambiqmicro <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(ambiqmicro_flash_bank_command)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ambiqmicro_info = calloc(sizeof(struct ambiqmicro_flash_bank), 1);

	bank->driver_priv = ambiqmicro_info;

	ambiqmicro_info->target_name = "Unknown target";

	/* part wasn't probed yet */
	ambiqmicro_info->probed = false;

	return ERROR_OK;
}

static int get_ambiqmicro_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int printed;
	char *classname;

	if (!ambiqmicro_info->probed) {
		LOG_ERROR("Target not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Check class name in range. */
	if (ambiqmicro_info->target_class < sizeof(ambiqmicroClassname))
		classname = ambiqmicroClassname[ambiqmicro_info->target_class];
	else
		classname = ambiqmicroClassname[0];

	printed = snprintf(buf,
		buf_size,
		"\nAmbiq Micro information: Chip is "
		"class %d (%s) %s\n",
		ambiqmicro_info->target_class,
		classname,
		ambiqmicro_info->target_name);

	if ((printed < 0))
		return ERROR_BUF_TOO_SMALL;
	return ERROR_OK;
}

/***************************************************************************
*	chip identification and status                                         *
***************************************************************************/

/* Fill in driver info structure */
static int ambiqmicro_read_part_info(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t PartNum = 0;
	int retval;

	/*
	 * Read Part Number.
	 */
	retval = target_read_u32(target, 0x40020000, &PartNum);
	if (retval != ERROR_OK) {
		LOG_ERROR("status(0x%x):Could not read PartNum.\n", retval);
		/* Set PartNum to default device */
		PartNum = 0;
	}
	LOG_DEBUG("Part number: 0x%x", PartNum);

	/*
	 * Determine device class.
	 */
	ambiqmicro_info->target_class = (PartNum & 0xFF000000) >> 24;

	switch (ambiqmicro_info->target_class) {
		case 1:		/* 1 - Apollo */
		case 5:		/* 5 - Apollo Bootloader */
			bank->base = bank->bank_number * 0x40000;
			ambiqmicro_info->pagesize = 2048;
			ambiqmicro_info->flshsiz =
			apollo_flash_size[(PartNum & 0x00F00000) >> 20];
			ambiqmicro_info->sramsiz =
			apollo_sram_size[(PartNum & 0x000F0000) >> 16];
			ambiqmicro_info->num_pages = ambiqmicro_info->flshsiz /
			ambiqmicro_info->pagesize;
			if (ambiqmicro_info->num_pages > 128) {
				ambiqmicro_info->num_pages = 128;
				ambiqmicro_info->flshsiz = 1024 * 256;
			}
			break;

		default:
			LOG_INFO("Unknown Class. Using Apollo-64 as default.");

			bank->base = bank->bank_number * 0x40000;
			ambiqmicro_info->pagesize = 2048;
			ambiqmicro_info->flshsiz = apollo_flash_size[1];
			ambiqmicro_info->sramsiz = apollo_sram_size[0];
			ambiqmicro_info->num_pages = ambiqmicro_info->flshsiz /
			ambiqmicro_info->pagesize;
			if (ambiqmicro_info->num_pages > 128) {
				ambiqmicro_info->num_pages = 128;
				ambiqmicro_info->flshsiz = 1024 * 256;
			}
			break;

	}

	if (ambiqmicro_info->target_class < ARRAY_SIZE(ambiqmicroParts))
		ambiqmicro_info->target_name =
			ambiqmicroParts[ambiqmicro_info->target_class].partname;
	else
		ambiqmicro_info->target_name =
			ambiqmicroParts[0].partname;

	LOG_DEBUG("num_pages: %d, pagesize: %d, flash: %d, sram: %d",
		ambiqmicro_info->num_pages,
		ambiqmicro_info->pagesize,
		ambiqmicro_info->flshsiz,
		ambiqmicro_info->sramsiz);

	return ERROR_OK;
}

/***************************************************************************
*	flash operations                                                       *
***************************************************************************/

static int ambiqmicro_protect_check(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro = bank->driver_priv;
	int status = ERROR_OK;
	uint32_t i;


	if (!ambiqmicro->probed) {
		LOG_ERROR("Target not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (i = 0; i < (unsigned) bank->num_sectors; i++)
		bank->sectors[i].is_protected = -1;

	return status;
}
/** Read flash status from bootloader. */
static int check_flash_status(struct target *target, uint32_t address)
{
	uint32_t retflash;
	int rc;
	rc = target_read_u32(target, address, &retflash);
	/* target connection failed. */
	if (rc != ERROR_OK) {
		LOG_DEBUG("%s:%d:%s(): status(0x%x)\n",
			__FILE__, __LINE__, __func__, rc);
		return rc;
	}
	/* target flash failed, unknown cause. */
	if (retflash != 0) {
		LOG_ERROR("Flash not happy: status(0x%x)", retflash);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int ambiqmicro_exec_command(struct target *target,
	uint32_t command,
	uint32_t flash_return_address)
{
	int retval, retflash;

	retval = target_resume(
		target,
		false,
		command,
		true,
		true);

	CHECK_STATUS(retval, "error executing ambiqmicro command");

	/*
	 * Wait for halt.
	 */
	for (;; ) {
		target_poll(target);
		if (target->state == TARGET_HALTED)
			break;
		else if (target->state == TARGET_RUNNING ||
			target->state == TARGET_DEBUG_RUNNING) {
			/*
			 * Keep polling until target halts.
			 */
			target_poll(target);
			alive_sleep(100);
			LOG_DEBUG("state = %d", target->state);
		} else {
			LOG_ERROR("Target not halted or running %d", target->state);
			break;
		}
	}

	/*
	 * Read return value, flash error takes precedence.
	 */
	retflash = check_flash_status(target, flash_return_address);
	if (retflash != ERROR_OK)
		retval = retflash;

	/* Return code from target_resume OR flash. */
	return retval;
}

static int ambiqmicro_mass_erase(struct flash_bank *bank)
{
	struct target *target = NULL;
	struct ambiqmicro_flash_bank *ambiqmicro_info = NULL;
	int retval = ERROR_OK;

	ambiqmicro_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!ambiqmicro_info->probed) {
		LOG_ERROR("Target not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/*
	 * Clear Bootloader bit.
	 */
	retval = target_write_u32(target, 0x400201a0, 0x0);
	CHECK_STATUS(retval, "error clearing bootloader bit.");

	/*
	 * Set up the SRAM.
	 */

	/*
	 * Bank.
	 */
	retval = target_write_u32(target, 0x10000000, bank->bank_number);
	CHECK_STATUS(retval, "error writing target SRAM parameters.");

	/*
	 * Write Key.
	 */
	retval = target_write_u32(target, 0x10000004, PROGRAM_KEY);
	CHECK_STATUS(retval, "error writing target SRAM parameters.");

	/*
	 * Breakpoint.
	 */
	retval = target_write_u32(target, 0x10000008, 0xfffffffe);
	CHECK_STATUS(retval, "error writing target SRAM parameters.");

	/*
	 * Erase the main array.
	 */
	LOG_INFO("Mass erase on bank %d.", bank->bank_number);

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_command(target, FLASH_MASS_ERASE_MAIN_PAGES_FROM_SRAM, 0x10000008);
	CHECK_STATUS(retval, "error executing ambiqmicro flash mass erase.");
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Set Bootloader bit, regardless of command execution.
	 */
	retval = target_write_u32(target, 0x400201a0, 0x1);
	CHECK_STATUS(retval, "error setting bootloader bit.");

	return retval;
}


static int ambiqmicro_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!ambiqmicro_info->probed) {
		LOG_ERROR("Target not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/*
	 * Check pages.
	 * Fix num_pages for the device.
	 */
	if ((last < first) || (last >= ambiqmicro_info->num_pages))
		return ERROR_FLASH_SECTOR_INVALID;

	/*
	 * Just Mass Erase if all pages are given.
	 * TODO: Fix num_pages for the device
	 */
	if ((first == 0) && (last == (ambiqmicro_info->num_pages - 1)))
		return ambiqmicro_mass_erase(bank);

	/*
	 * Clear Bootloader bit.
	 */
	retval = target_write_u32(target, 0x400201a0, 0x0);
	CHECK_STATUS(retval, "error clearing bootloader bit.");

	/*
	 * Set up the SRAM.
	 */

	/*
	 * Bank.
	 */
	retval = target_write_u32(target, 0x10000000, bank->bank_number);
	CHECK_STATUS(retval, "error writing target SRAM parameters.");

	/*
	 * Number of pages to erase.
	 */
	retval = target_write_u32(target, 0x10000004, 1 + (last-first));
	CHECK_STATUS(retval, "error writing target SRAM parameters.");

	/*
	 * Write Key.
	 */
	retval = target_write_u32(target, 0x10000008, PROGRAM_KEY);
	CHECK_STATUS(retval, "error writing target SRAM parameters.");

	/*
	 * Breakpoint.
	 */
	retval = target_write_u32(target, 0x1000000c, 0xfffffffe);
	CHECK_STATUS(retval, "error writing target SRAM parameters.");

	/*
	 * Pointer to flash address.
	 */
	retval = target_write_u32(target, 0x10000010, first);
	CHECK_STATUS(retval, "error writing target SRAM parameters.");
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Erase the pages.
	 */
	LOG_INFO("Erasing pages %u to %u on bank %u", first, last, bank->bank_number);

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_command(target, FLASH_ERASE_LIST_MAIN_PAGES_FROM_SRAM, 0x1000000C);
	CHECK_STATUS(retval, "error executing flash page erase");
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("%u pages erased!", 1+(last-first));

	if (first == 0) {
		/*
		 * Set Bootloader bit.
		 */
		retval = target_write_u32(target, 0x400201a0, 0x1);
		CHECK_STATUS(retval, "error setting bootloader bit.");
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int ambiqmicro_protect(struct flash_bank *bank, bool set,
		unsigned int first, unsigned int last)
{
	/* struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	 * struct target *target = bank->target; */

	/*
	 * TODO
	 */
	LOG_INFO("Not yet implemented");

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

static int ambiqmicro_write_block(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	/* struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv; */
	struct target *target = bank->target;
	uint32_t address = bank->base + offset;
	uint32_t buffer_pointer = 0x10000010;
	uint32_t maxbuffer;
	uint32_t thisrun_count;
	int retval = ERROR_OK;

	if (((count%4) != 0) || ((offset%4) != 0)) {
		LOG_ERROR("write block must be multiple of 4 bytes in offset & length");
		return ERROR_FAIL;
	}

	/*
	 * Max buffer size for this device.
	 * Hard code 6kB for the buffer.
	 */
	maxbuffer = 0x1800;

	LOG_INFO("Flashing main array");

	while (count > 0) {
		if (count > maxbuffer)
			thisrun_count = maxbuffer;
		else
			thisrun_count = count;

		/*
		 * Set up the SRAM.
		 */

		/*
		 * Pointer to flash.
		 */
		retval = target_write_u32(target, 0x10000000, address);
		CHECK_STATUS(retval, "error writing target SRAM parameters.");

		/*
		 * Number of 32-bit words to program.
		 */
		retval = target_write_u32(target, 0x10000004, thisrun_count/4);
		CHECK_STATUS(retval, "error writing target SRAM parameters.");

		/*
		 * Write Key.
		 */
		retval = target_write_u32(target, 0x10000008, PROGRAM_KEY);
		CHECK_STATUS(retval, "error writing target SRAM parameters.");

		/*
		 * Breakpoint.
		 */
		retval = target_write_u32(target, 0x1000000c, 0xfffffffe);
		CHECK_STATUS(retval, "error writing target SRAM parameters.");

		/*
		 * Write Buffer.
		 */
		retval = target_write_buffer(target, buffer_pointer, thisrun_count, buffer);

		if (retval != ERROR_OK) {
			CHECK_STATUS(retval, "error writing target SRAM parameters.");
			break;
		}

		LOG_DEBUG("address = 0x%08x", address);

		retval = ambiqmicro_exec_command(target, FLASH_PROGRAM_MAIN_FROM_SRAM, 0x1000000c);
		CHECK_STATUS(retval, "error executing ambiqmicro flash write algorithm");
		if (retval != ERROR_OK)
			break;
		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}


	LOG_INFO("Main array flashed");

	/*
	 * Clear Bootloader bit.
	 */
	retval = target_write_u32(target, 0x400201a0, 0x0);
	CHECK_STATUS(retval, "error clearing bootloader bit");

	return retval;
}

static int ambiqmicro_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	uint32_t retval;

	/* try using a block write */
	retval = ambiqmicro_write_block(bank, buffer, offset, count);
	if (retval != ERROR_OK)
		LOG_ERROR("write failed");

	return retval;
}

static int ambiqmicro_probe(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	uint32_t retval;

	/* If this is a ambiqmicro chip, it has flash; probe() is just
	 * to figure out how much is present.  Only do it once.
	 */
	if (ambiqmicro_info->probed) {
		LOG_INFO("Target already probed");
		return ERROR_OK;
	}

	/* ambiqmicro_read_part_info() already handled error checking and
	 * reporting.  Note that it doesn't write, so we don't care about
	 * whether the target is halted or not.
	 */
	retval = ambiqmicro_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* provide this for the benefit of the NOR flash framework */
	bank->size = ambiqmicro_info->pagesize * ambiqmicro_info->num_pages;
	bank->num_sectors = ambiqmicro_info->num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * ambiqmicro_info->pagesize;
		bank->sectors[i].size = ambiqmicro_info->pagesize;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	/*
	 * Part has been probed.
	 */
	ambiqmicro_info->probed = true;

	return retval;
}

static int ambiqmicro_otp_program(struct flash_bank *bank,
	uint32_t offset, uint32_t count)
{
	struct target *target = NULL;
	struct ambiqmicro_flash_bank *ambiqmicro_info = NULL;
	uint32_t retval = ERROR_OK;

	ambiqmicro_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!ambiqmicro_info->probed) {
		LOG_ERROR("Target not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (count > 256) {
		LOG_ERROR("Count must be < 256");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/*
	 * Clear Bootloader bit.
	 */
	retval = target_write_u32(target, 0x400201a0, 0x0);
	CHECK_STATUS(retval, "error clearing bootloader bit.");

	/*
	 * Set up the SRAM.
	 */

	/*
	 * Bank.
	 */
	retval = target_write_u32(target, 0x10000000, offset);
	CHECK_STATUS(retval, "error setting target SRAM parameters.");

	/*
	 * Num of words to program.
	 */
	retval = target_write_u32(target, 0x10000004, count);
	CHECK_STATUS(retval, "error setting target SRAM parameters.");

	/*
	 * Write Key.
	 */
	retval = target_write_u32(target, 0x10000008, OTP_PROGRAM_KEY);
	CHECK_STATUS(retval, "error setting target SRAM parameters.");

	/*
	 * Breakpoint.
	 */
	retval = target_write_u32(target, 0x1000000c, 0xfffffffe);
	CHECK_STATUS(retval, "error setting target SRAM parameters.");
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Program OTP.
	 */
	LOG_INFO("Programming OTP offset 0x%08x", offset);

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_command(target, FLASH_PROGRAM_OTP_FROM_SRAM, 0x1000000C);
	CHECK_STATUS(retval, "error executing ambiqmicro otp program algorithm");

	LOG_INFO("Programming OTP finished.");

	return retval;
}



COMMAND_HANDLER(ambiqmicro_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	uint32_t retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (ambiqmicro_mass_erase(bank) == ERROR_OK) {
		/* set all sectors as erased */
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "ambiqmicro mass erase complete");
	} else
		command_print(CMD, "ambiqmicro mass erase failed");

	return ERROR_OK;
}

COMMAND_HANDLER(ambiqmicro_handle_page_erase_command)
{
	struct flash_bank *bank;
	uint32_t first, last;
	uint32_t retval;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], first);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], last);

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (ambiqmicro_erase(bank, first, last) == ERROR_OK)
		command_print(CMD, "ambiqmicro page erase complete");
	else
		command_print(CMD, "ambiqmicro page erase failed");

	return ERROR_OK;
}


/**
 * Program the otp block.
 */
COMMAND_HANDLER(ambiqmicro_handle_program_otp_command)
{
	struct flash_bank *bank;
	uint32_t offset, count;
	uint32_t retval;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], offset);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);

	command_print(CMD, "offset=0x%08x count=%d", offset, count);

	CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	retval = ambiqmicro_otp_program(bank, offset, count);

	if (retval != ERROR_OK)
		LOG_ERROR("error check log");

	return ERROR_OK;
}



static const struct command_registration ambiqmicro_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.usage = "<bank>",
		.handler = ambiqmicro_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase entire device",
	},
	{
		.name = "page_erase",
		.usage = "<bank> <first> <last>",
		.handler = ambiqmicro_handle_page_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase device pages",
	},
	{
		.name = "program_otp",
		.handler = ambiqmicro_handle_program_otp_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank> <offset> <count>",
		.help =
			"Program OTP (assumes you have already written array starting at 0x10000010)",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration ambiqmicro_command_handlers[] = {
	{
		.name = "ambiqmicro",
		.mode = COMMAND_EXEC,
		.help = "ambiqmicro flash command group",
		.usage = "Support for Ambiq Micro parts.",
		.chain = ambiqmicro_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ambiqmicro_flash = {
	.name = "ambiqmicro",
	.commands = ambiqmicro_command_handlers,
	.flash_bank_command = ambiqmicro_flash_bank_command,
	.erase = ambiqmicro_erase,
	.protect = ambiqmicro_protect,
	.write = ambiqmicro_write,
	.read = default_flash_read,
	.probe = ambiqmicro_probe,
	.auto_probe = ambiqmicro_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = ambiqmicro_protect_check,
	.info = get_ambiqmicro_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
