/***************************************************************************
 *   Copyright (C) 2009 by                                                 *
 *   Rolf Meeser <rolfm_9dq@yahoo.de>                                      *
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
#include <target/algorithm.h>
#include <target/arm.h>
#include <target/image.h>

/* 1024 bytes */
#define KiB                 1024

/* Some flash constants */
#define FLASH_PAGE_SIZE     512		/* bytes */
#define FLASH_ERASE_TIME    100000	/* microseconds */
#define FLASH_PROGRAM_TIME  1000	/* microseconds */

/* Chip ID / Feature Registers */
#define CHIPID          0xE0000000	/* Chip ID */
#define FEAT0           0xE0000100	/* Chip feature 0 */
#define FEAT1           0xE0000104	/* Chip feature 1 */
#define FEAT2           0xE0000108	/* Chip feature 2 (contains flash size indicator) */
#define FEAT3           0xE000010C	/* Chip feature 3 */

#define EXPECTED_CHIPID 0x209CE02B	/* Chip ID of all LPC2900 devices */

/* Flash/EEPROM Control Registers */
#define FCTR            0x20200000	/* Flash control */
#define FPTR            0x20200008	/* Flash program-time */
#define FTCTR           0x2020000C	/* Flash test control */
#define FBWST           0x20200010	/* Flash bridge wait-state */
#define FCRA            0x2020001C	/* Flash clock divider */
#define FMSSTART        0x20200020	/* Flash Built-In Self Test start address */
#define FMSSTOP         0x20200024	/* Flash Built-In Self Test stop address */
#define FMS16           0x20200028	/* Flash 16-bit signature */
#define FMSW0           0x2020002C	/* Flash 128-bit signature Word 0 */
#define FMSW1           0x20200030	/* Flash 128-bit signature Word 1 */
#define FMSW2           0x20200034	/* Flash 128-bit signature Word 2 */
#define FMSW3           0x20200038	/* Flash 128-bit signature Word 3 */

#define EECMD           0x20200080	/* EEPROM command */
#define EEADDR          0x20200084	/* EEPROM address */
#define EEWDATA         0x20200088	/* EEPROM write data */
#define EERDATA         0x2020008C	/* EEPROM read data */
#define EEWSTATE        0x20200090	/* EEPROM wait state */
#define EECLKDIV        0x20200094	/* EEPROM clock divider */
#define EEPWRDWN        0x20200098	/* EEPROM power-down/start */
#define EEMSSTART       0x2020009C	/* EEPROM BIST start address */
#define EEMSSTOP        0x202000A0	/* EEPROM BIST stop address */
#define EEMSSIG         0x202000A4	/* EEPROM 24-bit BIST signature */

#define INT_CLR_ENABLE  0x20200FD8	/* Flash/EEPROM interrupt clear enable */
#define INT_SET_ENABLE  0x20200FDC	/* Flash/EEPROM interrupt set enable */
#define INT_STATUS      0x20200FE0	/* Flash/EEPROM interrupt status */
#define INT_ENABLE      0x20200FE4	/* Flash/EEPROM interrupt enable */
#define INT_CLR_STATUS  0x20200FE8	/* Flash/EEPROM interrupt clear status */
#define INT_SET_STATUS  0x20200FEC	/* Flash/EEPROM interrupt set status */

/* Interrupt sources */
#define INTSRC_END_OF_PROG    (1 << 28)
#define INTSRC_END_OF_BIST    (1 << 27)
#define INTSRC_END_OF_RDWR    (1 << 26)
#define INTSRC_END_OF_MISR    (1 << 2)
#define INTSRC_END_OF_BURN    (1 << 1)
#define INTSRC_END_OF_ERASE   (1 << 0)

/* FCTR bits */
#define FCTR_FS_LOADREQ       (1 << 15)
#define FCTR_FS_CACHECLR      (1 << 14)
#define FCTR_FS_CACHEBYP      (1 << 13)
#define FCTR_FS_PROGREQ       (1 << 12)
#define FCTR_FS_RLS           (1 << 11)
#define FCTR_FS_PDL           (1 << 10)
#define FCTR_FS_PD            (1 << 9)
#define FCTR_FS_WPB           (1 << 7)
#define FCTR_FS_ISS           (1 << 6)
#define FCTR_FS_RLD           (1 << 5)
#define FCTR_FS_DCR           (1 << 4)
#define FCTR_FS_WEB           (1 << 2)
#define FCTR_FS_WRE           (1 << 1)
#define FCTR_FS_CS            (1 << 0)
/* FPTR bits */
#define FPTR_EN_T             (1 << 15)
/* FTCTR bits */
#define FTCTR_FS_BYPASS_R     (1 << 29)
#define FTCTR_FS_BYPASS_W     (1 << 28)
/* FMSSTOP bits */
#define FMSSTOP_MISR_START    (1 << 17)
/* EEMSSTOP bits */
#define EEMSSTOP_STRTBIST     (1 << 31)

/* Index sector */
#define ISS_CUSTOMER_START1   (0x830)
#define ISS_CUSTOMER_END1     (0xA00)
#define ISS_CUSTOMER_SIZE1    (ISS_CUSTOMER_END1 - ISS_CUSTOMER_START1)
#define ISS_CUSTOMER_NWORDS1  (ISS_CUSTOMER_SIZE1 / 4)
#define ISS_CUSTOMER_START2   (0xA40)
#define ISS_CUSTOMER_END2     (0xC00)
#define ISS_CUSTOMER_SIZE2    (ISS_CUSTOMER_END2 - ISS_CUSTOMER_START2)
#define ISS_CUSTOMER_NWORDS2  (ISS_CUSTOMER_SIZE2 / 4)
#define ISS_CUSTOMER_SIZE     (ISS_CUSTOMER_SIZE1 + ISS_CUSTOMER_SIZE2)

/**
 * Private data for \c lpc2900 flash driver.
 */
struct lpc2900_flash_bank {
	/**
	 * This flag is set when the device has been successfully probed.
	 */
	bool is_probed;

	/**
	 * Holds the value read from CHIPID register.
	 * The driver will not load if the chipid doesn't match the expected
	 * value of 0x209CE02B of the LPC2900 family. A probe will only be done
	 * if the chipid does not yet contain the expected value.
	 */
	uint32_t chipid;

	/**
	 * String holding device name.
	 * This string is set by the probe function to the type number of the
	 * device. It takes the form "LPC29xx".
	 */
	char *target_name;

	/**
	 * System clock frequency.
	 * Holds the clock frequency in Hz, as passed by the configuration file
	 * to the <tt>flash bank</tt> command.
	 */
	uint32_t clk_sys_fmc;

	/**
	 * Flag to indicate that dangerous operations are possible.
	 * This flag can be set by passing the correct password to the
	 * <tt>lpc2900 password</tt> command. If set, other dangerous commands,
	 * which operate on the index sector, can be executed.
	 */
	uint32_t risky;

	/**
	 * Maximum contiguous block of internal SRAM (bytes).
	 * Autodetected by the driver. Not the total amount of SRAM, only
	 * the largest \em contiguous block!
	 */
	uint32_t max_ram_block;

};

static uint32_t lpc2900_wait_status(struct flash_bank *bank, uint32_t mask, int timeout);
static void lpc2900_setup(struct flash_bank *bank);
static uint32_t lpc2900_is_ready(struct flash_bank *bank);
static uint32_t lpc2900_read_security_status(struct flash_bank *bank);
static uint32_t lpc2900_run_bist128(struct flash_bank *bank,
		uint32_t addr_from, uint32_t addr_to,
		uint32_t signature[4]);
static unsigned int lpc2900_address2sector(struct flash_bank *bank, uint32_t offset);
static uint32_t lpc2900_calc_tr(uint32_t clock_var, uint32_t time_var);

/***********************  Helper functions  **************************/

/**
 * Wait for an event in mask to occur in INT_STATUS.
 *
 * Return when an event occurs, or after a timeout.
 *
 * @param[in] bank Pointer to the flash bank descriptor
 * @param[in] mask Mask to be used for INT_STATUS
 * @param[in] timeout Timeout in ms
 */
static uint32_t lpc2900_wait_status(struct flash_bank *bank,
	uint32_t mask,
	int timeout)
{
	uint32_t int_status;
	struct target *target = bank->target;

	do {
		alive_sleep(1);
		timeout--;
		target_read_u32(target, INT_STATUS, &int_status);
	} while (((int_status & mask) == 0) && (timeout != 0));

	if (timeout == 0) {
		LOG_DEBUG("Timeout!");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

/**
 * Set up the flash for erase/program operations.
 *
 * Enable the flash, and set the correct CRA clock of 66 kHz.
 *
 * @param bank Pointer to the flash bank descriptor
 */
static void lpc2900_setup(struct flash_bank *bank)
{
	uint32_t fcra;
	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;

	/* Power up the flash block */
	target_write_u32(bank->target, FCTR, FCTR_FS_WEB | FCTR_FS_CS);

	fcra = (lpc2900_info->clk_sys_fmc / (3 * 66000)) - 1;
	target_write_u32(bank->target, FCRA, fcra);
}

/**
 * Check if device is ready.
 *
 * Check if device is ready for flash operation:
 * Must have been successfully probed.
 * Must be halted.
 */
static uint32_t lpc2900_is_ready(struct flash_bank *bank)
{
	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;

	if (!lpc2900_info->is_probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

/**
 * Read the status of sector security from the index sector.
 *
 * @param bank Pointer to the flash bank descriptor
 */
static uint32_t lpc2900_read_security_status(struct flash_bank *bank)
{
	uint32_t status = lpc2900_is_ready(bank);
	if (status != ERROR_OK)
		return status;

	struct target *target = bank->target;

	/* Enable ISS access */
	target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB | FCTR_FS_ISS);

	/* Read the relevant block of memory from the ISS sector */
	uint32_t iss_secured_field[0x230/16][4];
	target_read_memory(target, bank->base + 0xC00, 4, 0x230/4,
		(uint8_t *)iss_secured_field);

	/* Disable ISS access */
	target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

	/* Check status of each sector. Note that the sector numbering in the LPC2900
	 * is different from the logical sector numbers used in OpenOCD!
	 * Refer to the user manual for details.
	 *
	 * All zeros (16x 0x00) are treated as a secured sector (is_protected = 1)
	 * All ones (16x 0xFF) are treated as a non-secured sector (is_protected = 0)
	 * Anything else is undefined (is_protected = -1). This is treated as
	 * a protected sector!
	 */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		unsigned int index_t;

		/* Convert logical sector number to physical sector number */
		if (sector <= 4)
			index_t = sector + 11;
		else if (sector <= 7)
			index_t = sector + 27;
		else
			index_t = sector - 8;

		bank->sectors[sector].is_protected = -1;

		if ((iss_secured_field[index_t][0] == 0x00000000) &&
			(iss_secured_field[index_t][1] == 0x00000000) &&
			(iss_secured_field[index_t][2] == 0x00000000) &&
			(iss_secured_field[index_t][3] == 0x00000000))
			bank->sectors[sector].is_protected = 1;

		if ((iss_secured_field[index_t][0] == 0xFFFFFFFF) &&
			(iss_secured_field[index_t][1] == 0xFFFFFFFF) &&
			(iss_secured_field[index_t][2] == 0xFFFFFFFF) &&
			(iss_secured_field[index_t][3] == 0xFFFFFFFF))
			bank->sectors[sector].is_protected = 0;
	}

	return ERROR_OK;
}

/**
 * Use BIST to calculate a 128-bit hash value over a range of flash.
 *
 * @param bank Pointer to the flash bank descriptor
 * @param addr_from
 * @param addr_to
 * @param signature
 */
static uint32_t lpc2900_run_bist128(struct flash_bank *bank,
	uint32_t addr_from,
	uint32_t addr_to,
	uint32_t signature[4])
{
	struct target *target = bank->target;

	/* Clear END_OF_MISR interrupt status */
	target_write_u32(target, INT_CLR_STATUS, INTSRC_END_OF_MISR);

	/* Start address */
	target_write_u32(target, FMSSTART, addr_from >> 4);
	/* End address, and issue start command */
	target_write_u32(target, FMSSTOP, (addr_to >> 4) | FMSSTOP_MISR_START);

	/* Poll for end of operation. Calculate a reasonable timeout. */
	if (lpc2900_wait_status(bank, INTSRC_END_OF_MISR, 1000) != ERROR_OK)
		return ERROR_FLASH_OPERATION_FAILED;

	/* Return the signature */
	uint8_t sig_buf[4 * 4];
	target_read_memory(target, FMSW0, 4, 4, sig_buf);
	target_buffer_get_u32_array(target, sig_buf, 4, signature);

	return ERROR_OK;
}

/**
 * Return sector number for given address.
 *
 * Return the (logical) sector number for a given relative address.
 * No sanity check is done. It assumed that the address is valid.
 *
 * @param bank Pointer to the flash bank descriptor
 * @param offset Offset address relative to bank start
 */
static unsigned int lpc2900_address2sector(struct flash_bank *bank,
	uint32_t offset)
{
	uint32_t address = bank->base + offset;

	/* Run through all sectors of this bank */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Return immediately if address is within the current sector */
		if (address < (bank->sectors[sector].offset + bank->sectors[sector].size))
			return sector;
	}

	/* We should never come here. If we do, return an arbitrary sector number. */
	return 0;
}

/**
 * Write one page to the index sector.
 *
 * @param bank Pointer to the flash bank descriptor
 * @param pagenum Page number (0...7)
 * @param page Page array (FLASH_PAGE_SIZE bytes)
 */
static int lpc2900_write_index_page(struct flash_bank *bank,
	int pagenum,
	uint8_t page[FLASH_PAGE_SIZE])
{
	/* Only pages 4...7 are user writable */
	if ((pagenum < 4) || (pagenum > 7)) {
		LOG_ERROR("Refuse to burn index sector page %d", pagenum);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	/* Get target, and check if it's halted */
	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Private info */
	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;

	/* Enable flash block and set the correct CRA clock of 66 kHz */
	lpc2900_setup(bank);

	/* Un-protect the index sector */
	target_write_u32(target, bank->base, 0);
	target_write_u32(target, FCTR,
		FCTR_FS_LOADREQ | FCTR_FS_WPB | FCTR_FS_ISS |
		FCTR_FS_WEB | FCTR_FS_WRE | FCTR_FS_CS);

	/* Set latch load mode */
	target_write_u32(target, FCTR,
		FCTR_FS_ISS | FCTR_FS_WEB | FCTR_FS_WRE | FCTR_FS_CS);

	/* Write whole page to flash data latches */
	if (target_write_memory(target,
			bank->base + pagenum * FLASH_PAGE_SIZE,
			4, FLASH_PAGE_SIZE / 4, page) != ERROR_OK) {
		LOG_ERROR("Index sector write failed @ page %d", pagenum);
		target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Clear END_OF_BURN interrupt status */
	target_write_u32(target, INT_CLR_STATUS, INTSRC_END_OF_BURN);

	/* Set the program/erase time to FLASH_PROGRAM_TIME */
	target_write_u32(target, FPTR,
		FPTR_EN_T | lpc2900_calc_tr(lpc2900_info->clk_sys_fmc,
			FLASH_PROGRAM_TIME));

	/* Trigger flash write */
	target_write_u32(target, FCTR,
		FCTR_FS_PROGREQ | FCTR_FS_ISS |
		FCTR_FS_WPB | FCTR_FS_WRE | FCTR_FS_CS);

	/* Wait for the end of the write operation. If it's not over after one
	 * second, something went dreadfully wrong... :-(
	 */
	if (lpc2900_wait_status(bank, INTSRC_END_OF_BURN, 1000) != ERROR_OK) {
		LOG_ERROR("Index sector write failed @ page %d", pagenum);
		target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

		return ERROR_FLASH_OPERATION_FAILED;
	}

	target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

	return ERROR_OK;
}

/**
 * Calculate FPTR.TR register value for desired program/erase time.
 *
 * @param clock_var System clock in Hz
 * @param time_var Program/erase time in µs
 */
static uint32_t lpc2900_calc_tr(uint32_t clock_var, uint32_t time_var)
{
	/*           ((time[µs]/1e6) * f[Hz]) + 511
	 * FPTR.TR = -------------------------------
	 *                         512
	 */

	uint32_t tr_val = (uint32_t)((((time_var / 1e6) * clock_var) + 511.0) / 512.0);

	return tr_val;
}

/***********************  Private flash commands  **************************/


/**
 * Command to determine the signature of the whole flash.
 *
 * Uses the Built-In-Self-Test (BIST) to generate a 128-bit hash value
 * of the flash content.
 */
COMMAND_HANDLER(lpc2900_handle_signature_command)
{
	uint32_t status;
	uint32_t signature[4];

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Run BIST over whole flash range */
	status = lpc2900_run_bist128(bank, bank->base, bank->base + (bank->size - 1), signature);
	if (status != ERROR_OK)
		return status;

	command_print(CMD, "signature: 0x%8.8" PRIx32
		":0x%8.8" PRIx32
		":0x%8.8" PRIx32
		":0x%8.8" PRIx32,
		signature[3], signature[2], signature[1], signature[0]);

	return ERROR_OK;
}

/**
 * Store customer info in file.
 *
 * Read customer info from index sector, and store that block of data into
 * a disk file. The format is binary.
 */
COMMAND_HANDLER(lpc2900_handle_read_custom_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;
	lpc2900_info->risky = 0;

	/* Get target, and check if it's halted */
	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Storage for customer info. Read in two parts */
	uint8_t customer[4 * (ISS_CUSTOMER_NWORDS1 + ISS_CUSTOMER_NWORDS2)];

	/* Enable access to index sector */
	target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB | FCTR_FS_ISS);

	/* Read two parts */
	target_read_memory(target, bank->base+ISS_CUSTOMER_START1, 4,
		ISS_CUSTOMER_NWORDS1,
		&customer[0]);
	target_read_memory(target, bank->base+ISS_CUSTOMER_START2, 4,
		ISS_CUSTOMER_NWORDS2,
		&customer[4 * ISS_CUSTOMER_NWORDS1]);

	/* Deactivate access to index sector */
	target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

	/* Try and open the file */
	struct fileio *fileio;
	const char *filename = CMD_ARGV[1];
	int ret = fileio_open(&fileio, filename, FILEIO_WRITE, FILEIO_BINARY);
	if (ret != ERROR_OK) {
		LOG_WARNING("Could not open file %s", filename);
		return ret;
	}

	size_t nwritten;
	ret = fileio_write(fileio, sizeof(customer), customer, &nwritten);
	if (ret != ERROR_OK) {
		LOG_ERROR("Write operation to file %s failed", filename);
		fileio_close(fileio);
		return ret;
	}

	fileio_close(fileio);

	return ERROR_OK;
}

/**
 * Enter password to enable potentially dangerous options.
 */
COMMAND_HANDLER(lpc2900_handle_password_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;

#define ISS_PASSWORD "I_know_what_I_am_doing"

	lpc2900_info->risky = !strcmp(CMD_ARGV[1], ISS_PASSWORD);

	if (!lpc2900_info->risky) {
		command_print(CMD, "Wrong password (use '%s')", ISS_PASSWORD);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	command_print(CMD,
		"Potentially dangerous operation allowed in next command!");

	return ERROR_OK;
}

/**
 * Write customer info from file to the index sector.
 */
COMMAND_HANDLER(lpc2900_handle_write_custom_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;

	/* Check if command execution is allowed. */
	if (!lpc2900_info->risky) {
		command_print(CMD, "Command execution not allowed!");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	lpc2900_info->risky = 0;

	/* Get target, and check if it's halted */
	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The image will always start at offset 0 */
	struct image image;
	image.base_address_set = true;
	image.base_address = 0;
	image.start_address_set = false;

	const char *filename = CMD_ARGV[1];
	const char *type = (CMD_ARGC >= 3) ? CMD_ARGV[2] : NULL;
	retval = image_open(&image, filename, type);
	if (retval != ERROR_OK)
		return retval;

	/* Do a sanity check: The image must be exactly the size of the customer
	   programmable area. Any other size is rejected. */
	if (image.num_sections != 1) {
		LOG_ERROR("Only one section allowed in image file.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if ((image.sections[0].base_address != 0) ||
			(image.sections[0].size != ISS_CUSTOMER_SIZE)) {
		LOG_ERROR("Incorrect image file size. Expected %d, "
			"got %" PRIu32,
			ISS_CUSTOMER_SIZE, image.sections[0].size);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* Well boys, I reckon this is it... */

	/* Customer info is split into two blocks in pages 4 and 5. */
	uint8_t page[FLASH_PAGE_SIZE];

	/* Page 4 */
	uint32_t offset = ISS_CUSTOMER_START1 % FLASH_PAGE_SIZE;
	memset(page, 0xff, FLASH_PAGE_SIZE);
	size_t size_read;
	retval = image_read_section(&image, 0, 0,
			ISS_CUSTOMER_SIZE1, &page[offset], &size_read);
	if (retval != ERROR_OK) {
		LOG_ERROR("couldn't read from file '%s'", filename);
		image_close(&image);
		return retval;
	}
	retval = lpc2900_write_index_page(bank, 4, page);
	if (retval != ERROR_OK) {
		image_close(&image);
		return retval;
	}

	/* Page 5 */
	offset = ISS_CUSTOMER_START2 % FLASH_PAGE_SIZE;
	memset(page, 0xff, FLASH_PAGE_SIZE);
	retval = image_read_section(&image, 0, ISS_CUSTOMER_SIZE1,
			ISS_CUSTOMER_SIZE2, &page[offset], &size_read);
	if (retval != ERROR_OK) {
		LOG_ERROR("couldn't read from file '%s'", filename);
		image_close(&image);
		return retval;
	}
	retval = lpc2900_write_index_page(bank, 5, page);
	if (retval != ERROR_OK) {
		image_close(&image);
		return retval;
	}

	image_close(&image);

	return ERROR_OK;
}

/**
 * Activate 'sector security' for a range of sectors.
 */
COMMAND_HANDLER(lpc2900_handle_secure_sector_command)
{
	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Get the bank descriptor */
	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;

	/* Check if command execution is allowed. */
	if (!lpc2900_info->risky) {
		command_print(CMD, "Command execution not allowed! "
			"(use 'password' command first)");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	lpc2900_info->risky = 0;

	/* Read sector range, and do a sanity check. */
	unsigned int first, last;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], first);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[2], last);
	if ((first >= bank->num_sectors) ||
			(last >= bank->num_sectors) ||
			(first > last)) {
		command_print(CMD, "Illegal sector range");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	uint8_t page[FLASH_PAGE_SIZE];

	/* Sectors in page 6 */
	if ((first <= 4) || (last >= 8)) {
		memset(&page, 0xff, FLASH_PAGE_SIZE);
		for (unsigned int sector = first; sector <= last; sector++) {
			if (sector <= 4)
				memset(&page[0xB0 + 16*sector], 0, 16);
			else if (sector >= 8)
				memset(&page[0x00 + 16*(sector - 8)], 0, 16);
		}

		retval = lpc2900_write_index_page(bank, 6, page);
		if (retval != ERROR_OK) {
			LOG_ERROR("failed to update index sector page 6");
			return retval;
		}
	}

	/* Sectors in page 7 */
	if ((first <= 7) && (last >= 5)) {
		memset(&page, 0xff, FLASH_PAGE_SIZE);
		for (unsigned int sector = first; sector <= last; sector++) {
			if ((sector >= 5) && (sector <= 7))
				memset(&page[0x00 + 16*(sector - 5)], 0, 16);
		}

		retval = lpc2900_write_index_page(bank, 7, page);
		if (retval != ERROR_OK) {
			LOG_ERROR("failed to update index sector page 7");
			return retval;
		}
	}

	command_print(CMD,
		"Sectors security will become effective after next power cycle");

	/* Update the sector security status */
	if (lpc2900_read_security_status(bank) != ERROR_OK) {
		LOG_ERROR("Cannot determine sector security status");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

/**
 * Activate JTAG protection.
 */
COMMAND_HANDLER(lpc2900_handle_secure_jtag_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Get the bank descriptor */
	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;

	/* Check if command execution is allowed. */
	if (!lpc2900_info->risky) {
		command_print(CMD, "Command execution not allowed! "
			"(use 'password' command first)");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	lpc2900_info->risky = 0;

	/* Prepare page */
	uint8_t page[FLASH_PAGE_SIZE];
	memset(&page, 0xff, FLASH_PAGE_SIZE);


	/* Insert "soft" protection word */
	page[0x30 + 15] = 0x7F;
	page[0x30 + 11] = 0x7F;
	page[0x30 +  7] = 0x7F;
	page[0x30 +  3] = 0x7F;

	/* Write to page 5 */
	retval = lpc2900_write_index_page(bank, 5, page);
	if (retval != ERROR_OK) {
		LOG_ERROR("failed to update index sector page 5");
		return retval;
	}

	LOG_INFO("JTAG security set. Good bye!");

	return ERROR_OK;
}

/***********************  Flash interface functions  **************************/

static const struct command_registration lpc2900_exec_command_handlers[] = {
	{
		.name = "signature",
		.usage = "<bank>",
		.handler = lpc2900_handle_signature_command,
		.mode = COMMAND_EXEC,
		.help = "Calculate and display signature of flash bank.",
	},
	{
		.name = "read_custom",
		.handler = lpc2900_handle_read_custom_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename",
		.help = "Copies 912 bytes of customer information "
			"from index sector into file.",
	},
	{
		.name = "password",
		.handler = lpc2900_handle_password_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id password",
		.help = "Enter fixed password to enable 'dangerous' options.",
	},
	{
		.name = "write_custom",
		.handler = lpc2900_handle_write_custom_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename ('bin'|'ihex'|'elf'|'s19')",
		.help = "Copies 912 bytes of customer info from file "
			"to index sector.",
	},
	{
		.name = "secure_sector",
		.handler = lpc2900_handle_secure_sector_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id first_sector last_sector",
		.help = "Activate sector security for a range of sectors.  "
			"It will be effective after a power cycle.",
	},
	{
		.name = "secure_jtag",
		.handler = lpc2900_handle_secure_jtag_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Disable the JTAG port.  "
			"It will be effective after a power cycle.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration lpc2900_command_handlers[] = {
	{
		.name = "lpc2900",
		.mode = COMMAND_ANY,
		.help = "LPC2900 flash command group",
		.usage = "",
		.chain = lpc2900_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Evaluate flash bank command. */
FLASH_BANK_COMMAND_HANDLER(lpc2900_flash_bank_command)
{
	struct lpc2900_flash_bank *lpc2900_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	lpc2900_info = malloc(sizeof(struct lpc2900_flash_bank));
	bank->driver_priv = lpc2900_info;

	/* Get flash clock.
	 * Reject it if we can't meet the requirements for program time
	 * (if clock too slow), or for erase time (clock too fast).
	 */
	uint32_t clk_sys_fmc;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], clk_sys_fmc);
	lpc2900_info->clk_sys_fmc = clk_sys_fmc * 1000;

	uint32_t clock_limit;
	/* Check program time limit */
	clock_limit = 512000000l / FLASH_PROGRAM_TIME;
	if (lpc2900_info->clk_sys_fmc < clock_limit) {
		LOG_WARNING("flash clock must be at least %" PRIu32 " kHz",
			(clock_limit / 1000));
		return ERROR_FLASH_BANK_INVALID;
	}

	/* Check erase time limit */
	clock_limit = (uint32_t)((32767.0 * 512.0 * 1e6) / FLASH_ERASE_TIME);
	if (lpc2900_info->clk_sys_fmc > clock_limit) {
		LOG_WARNING("flash clock must be a maximum of %" PRIu32 " kHz",
			(clock_limit / 1000));
		return ERROR_FLASH_BANK_INVALID;
	}

	/* Chip ID will be obtained by probing the device later */
	lpc2900_info->chipid = 0;
	lpc2900_info->is_probed = false;

	return ERROR_OK;
}

/**
 * Erase sector(s).
 *
 * @param bank Pointer to the flash bank descriptor
 * @param first First sector to be erased
 * @param last Last sector (including) to be erased
 */
static int lpc2900_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	uint32_t status;
	unsigned int last_unsecured_sector;
	bool has_unsecured_sector;
	struct target *target = bank->target;
	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;


	status = lpc2900_is_ready(bank);
	if (status != ERROR_OK)
		return status;

	/* Sanity check on sector range */
	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_INFO("Bad sector range");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* Update the info about secured sectors */
	lpc2900_read_security_status(bank);

	/* The selected sector range might include secured sectors. An attempt
	 * to erase such a sector will cause the erase to fail also for unsecured
	 * sectors. It is necessary to determine the last unsecured sector now,
	 * because we have to treat the last relevant sector in the list in
	 * a special way.
	 */
	last_unsecured_sector = -1;
	has_unsecured_sector = false;
	for (unsigned int sector = first; sector <= last; sector++) {
		if (!bank->sectors[sector].is_protected) {
			last_unsecured_sector = sector;
			has_unsecured_sector = true;
		}
	}

	/* Exit now, in case of the rare constellation where all sectors in range
	 * are secured. This is regarded a success, since erasing/programming of
	 * secured sectors shall be handled transparently.
	 */
	if (!has_unsecured_sector)
		return ERROR_OK;

	/* Enable flash block and set the correct CRA clock of 66 kHz */
	lpc2900_setup(bank);

	/* Clear END_OF_ERASE interrupt status */
	target_write_u32(target, INT_CLR_STATUS, INTSRC_END_OF_ERASE);

	/* Set the program/erase timer to FLASH_ERASE_TIME */
	target_write_u32(target, FPTR,
		FPTR_EN_T | lpc2900_calc_tr(lpc2900_info->clk_sys_fmc,
			FLASH_ERASE_TIME));

	/* Sectors are marked for erasure, then erased all together */
	for (unsigned int sector = first; sector <= last_unsecured_sector; sector++) {
		/* Only mark sectors that aren't secured. Any attempt to erase a group
		 * of sectors will fail if any single one of them is secured!
		 */
		if (!bank->sectors[sector].is_protected) {
			/* Unprotect the sector */
			target_write_u32(target, bank->sectors[sector].offset, 0);
			target_write_u32(target, FCTR,
				FCTR_FS_LOADREQ | FCTR_FS_WPB |
				FCTR_FS_WEB | FCTR_FS_WRE | FCTR_FS_CS);

			/* Mark the sector for erasure. The last sector in the list
			   triggers the erasure. */
			target_write_u32(target, bank->sectors[sector].offset, 0);
			if (sector == last_unsecured_sector) {
				target_write_u32(target, FCTR,
					FCTR_FS_PROGREQ | FCTR_FS_WPB | FCTR_FS_CS);
			} else {
				target_write_u32(target, FCTR,
					FCTR_FS_LOADREQ | FCTR_FS_WPB |
					FCTR_FS_WEB | FCTR_FS_CS);
			}
		}
	}

	/* Wait for the end of the erase operation. If it's not over after two seconds,
	 * something went dreadfully wrong... :-(
	 */
	if (lpc2900_wait_status(bank, INTSRC_END_OF_ERASE, 2000) != ERROR_OK)
		return ERROR_FLASH_OPERATION_FAILED;

	/* Normal flash operating mode */
	target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

	return ERROR_OK;
}

/* lpc2900_protect command is not supported.
* "Protection" in LPC2900 terms is handled transparently. Sectors will
* automatically be unprotected as needed.
* Instead we use the concept of sector security. A secured sector is shown
* as "protected" in OpenOCD. Sector security is a permanent feature, and
* cannot be disabled once activated.
*/

/**
 * Write data to flash.
 *
 * @param bank Pointer to the flash bank descriptor
 * @param buffer Buffer with data
 * @param offset Start address (relative to bank start)
 * @param count Number of bytes to be programmed
 */
static int lpc2900_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	uint8_t page[FLASH_PAGE_SIZE];
	uint32_t status;
	uint32_t num_bytes;
	struct target *target = bank->target;
	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;
	int retval;

	static const uint32_t write_target_code[] = {
		/* Set auto latch mode: FCTR=CS|WRE|WEB */
		0xe3a0a007,	/* loop       mov r10, #0x007 */
		0xe583a000,	/*            str r10,[r3,#0] */

		/* Load complete page into latches */
		0xe3a06020,	/*            mov r6,#(512/16) */
		0xe8b00f00,	/* next       ldmia r0!,{r8-r11} */
		0xe8a10f00,	/*            stmia r1!,{r8-r11} */
		0xe2566001,	/*            subs r6,#1 */
		0x1afffffb,	/*            bne next */

		/* Clear END_OF_BURN interrupt status */
		0xe3a0a002,	/*            mov r10,#(1 << 1) */
		0xe583afe8,	/*            str r10,[r3,#0xfe8] */

		/* Set the erase time to FLASH_PROGRAM_TIME */
		0xe5834008,	/*            str r4,[r3,#8] */

		/* Trigger flash write
		 * FCTR = CS | WRE | WPB | PROGREQ */
		0xe3a0a083,	/*            mov r10,#0x83 */
		0xe38aaa01,	/*            orr r10,#0x1000 */
		0xe583a000,	/*            str r10,[r3,#0] */

		/* Wait for end of burn */
		0xe593afe0,	/* wait       ldr r10,[r3,#0xfe0] */
		0xe21aa002,	/*            ands r10,#(1 << 1) */
		0x0afffffc,	/*            beq wait */

		/* End? */
		0xe2522001,	/*            subs r2,#1 */
		0x1affffed,	/*            bne loop */

		0xeafffffe	/* done       b done */
	};


	status = lpc2900_is_ready(bank);
	if (status != ERROR_OK)
		return status;

	/* Enable flash block and set the correct CRA clock of 66 kHz */
	lpc2900_setup(bank);

	/* Update the info about secured sectors */
	lpc2900_read_security_status(bank);

	/* Unprotect all involved sectors */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start address in or before this sector?
		 * End address in or behind this sector? */
		if (((bank->base + offset) <
				(bank->sectors[sector].offset + bank->sectors[sector].size)) &&
				((bank->base + (offset + count - 1)) >= bank->sectors[sector].offset)) {
			/* This sector is involved and needs to be unprotected.
			 * Don't do it for secured sectors.
			 */
			if (!bank->sectors[sector].is_protected) {
				target_write_u32(target, bank->sectors[sector].offset, 0);
				target_write_u32(target, FCTR,
					FCTR_FS_LOADREQ | FCTR_FS_WPB |
					FCTR_FS_WEB | FCTR_FS_WRE | FCTR_FS_CS);
			}
		}
	}

	/* Set the program/erase time to FLASH_PROGRAM_TIME */
	uint32_t prog_time = FPTR_EN_T | lpc2900_calc_tr(lpc2900_info->clk_sys_fmc, FLASH_PROGRAM_TIME);

	/* If there is a working area of reasonable size, use it to program via
	 * a target algorithm. If not, fall back to host programming. */

	/* We need some room for target code. */
	const uint32_t target_code_size = sizeof(write_target_code);

	/* Try working area allocation. Start with a large buffer, and try with
	 * reduced size if that fails. */
	struct working_area *warea;
	uint32_t buffer_size = lpc2900_info->max_ram_block - 1 * KiB;
	while ((retval = target_alloc_working_area_try(target,
				 buffer_size + target_code_size,
				 &warea)) != ERROR_OK) {
		/* Try a smaller buffer now, and stop if it's too small. */
		buffer_size -= 1 * KiB;
		if (buffer_size < 2 * KiB) {
			LOG_INFO("no (large enough) working area, falling back to host mode");
			warea = NULL;
			break;
		}
	}

	if (warea) {
		struct reg_param reg_params[5];
		struct arm_algorithm arm_algo;

		/* We can use target mode. Download the algorithm. */
		uint8_t code[sizeof(write_target_code)];
		target_buffer_set_u32_array(target, code, ARRAY_SIZE(write_target_code),
				write_target_code);
		retval = target_write_buffer(target, (warea->address) + buffer_size, sizeof(code), code);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write block write code to target");
			target_free_all_working_areas(target);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
		init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
		init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
		init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
		init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);

		/* Write to flash in large blocks */
		while (count != 0) {
			uint32_t this_npages;
			const uint8_t *this_buffer;
			unsigned int start_sector = lpc2900_address2sector(bank, offset);

			/* First page / last page / rest */
			if (offset % FLASH_PAGE_SIZE) {
				/* Block doesn't start on page boundary.
				 * Burn first partial page separately. */
				memset(&page, 0xff, sizeof(page));
				memcpy(&page[offset % FLASH_PAGE_SIZE],
					buffer,
					FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE));
				this_npages = 1;
				this_buffer = &page[0];
				count = count + (offset % FLASH_PAGE_SIZE);
				offset = offset - (offset % FLASH_PAGE_SIZE);
			} else if (count < FLASH_PAGE_SIZE) {
				/* Download last incomplete page separately. */
				memset(&page, 0xff, sizeof(page));
				memcpy(&page, buffer, count);
				this_npages = 1;
				this_buffer = &page[0];
				count = FLASH_PAGE_SIZE;
			} else {
				/* Download as many full pages as possible */
				this_npages = (count < buffer_size) ?
					count / FLASH_PAGE_SIZE :
					buffer_size / FLASH_PAGE_SIZE;
				this_buffer = buffer;

				/* Make sure we stop at the next secured sector */
				unsigned int sector = start_sector + 1;
				while (sector < bank->num_sectors) {
					/* Secured? */
					if (bank->sectors[sector].is_protected) {
						/* Is that next sector within the current block? */
						if ((bank->sectors[sector].offset - bank->base) <
								(offset + (this_npages * FLASH_PAGE_SIZE))) {
							/* Yes! Split the block */
							this_npages =
								(bank->sectors[sector].offset -
								 bank->base - offset)
								/ FLASH_PAGE_SIZE;
							break;
						}
					}

					sector++;
				}
			}

			/* Skip the current sector if it is secured */
			if (bank->sectors[start_sector].is_protected) {
				LOG_DEBUG("Skip secured sector %u",
					start_sector);

				/* Stop if this is the last sector */
				if (start_sector == bank->num_sectors - 1)
					break;

				/* Skip */
				uint32_t nskip = bank->sectors[start_sector].size -
					(offset % bank->sectors[start_sector].size);
				offset += nskip;
				buffer += nskip;
				count = (count >= nskip) ? (count - nskip) : 0;
				continue;
			}

			/* Execute buffer download */
			retval = target_write_buffer(target, warea->address,
					this_npages * FLASH_PAGE_SIZE, this_buffer);
			if (retval != ERROR_OK) {
				LOG_ERROR("Unable to write data to target");
				target_free_all_working_areas(target);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			/* Prepare registers */
			buf_set_u32(reg_params[0].value, 0, 32, warea->address);
			buf_set_u32(reg_params[1].value, 0, 32, offset);
			buf_set_u32(reg_params[2].value, 0, 32, this_npages);
			buf_set_u32(reg_params[3].value, 0, 32, FCTR);
			buf_set_u32(reg_params[4].value, 0, 32, FPTR_EN_T | prog_time);

			/* Execute algorithm, assume breakpoint for last instruction */
			arm_algo.common_magic = ARM_COMMON_MAGIC;
			arm_algo.core_mode = ARM_MODE_SVC;
			arm_algo.core_state = ARM_STATE_ARM;

			retval = target_run_algorithm(target, 0, NULL, 5, reg_params,
					(warea->address) + buffer_size,
					(warea->address) + buffer_size + target_code_size - 4,
					10000,	/* 10s should be enough for max. 16 KiB of data */
					&arm_algo);

			if (retval != ERROR_OK) {
				LOG_ERROR("Execution of flash algorithm failed.");
				target_free_all_working_areas(target);
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			}

			count -= this_npages * FLASH_PAGE_SIZE;
			buffer += this_npages * FLASH_PAGE_SIZE;
			offset += this_npages * FLASH_PAGE_SIZE;
		}

		/* Free all resources */
		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		destroy_reg_param(&reg_params[3]);
		destroy_reg_param(&reg_params[4]);
		target_free_all_working_areas(target);
	} else {
		/* Write to flash memory page-wise */
		while (count != 0) {
			/* How many bytes do we copy this time? */
			num_bytes = (count >= FLASH_PAGE_SIZE) ?
				FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE) :
				count;

			/* Don't do anything with it if the page is in a secured sector. */
			if (!bank->sectors[lpc2900_address2sector(bank, offset)].is_protected) {
				/* Set latch load mode */
				target_write_u32(target, FCTR,
					FCTR_FS_CS | FCTR_FS_WRE | FCTR_FS_WEB);

				/* Always clear the buffer (a little overhead, but who cares) */
				memset(page, 0xFF, FLASH_PAGE_SIZE);

				/* Copy them to the buffer */
				memcpy(&page[offset % FLASH_PAGE_SIZE],
					&buffer[offset % FLASH_PAGE_SIZE],
					num_bytes);

				/* Write whole page to flash data latches */
				if (target_write_memory(target,
						bank->base + (offset - (offset % FLASH_PAGE_SIZE)),
						4, FLASH_PAGE_SIZE / 4, page) != ERROR_OK) {
					LOG_ERROR("Write failed @ 0x%8.8" PRIx32, offset);
					target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

					return ERROR_FLASH_OPERATION_FAILED;
				}

				/* Clear END_OF_BURN interrupt status */
				target_write_u32(target, INT_CLR_STATUS, INTSRC_END_OF_BURN);

				/* Set the programming time */
				target_write_u32(target, FPTR, FPTR_EN_T | prog_time);

				/* Trigger flash write */
				target_write_u32(target, FCTR,
					FCTR_FS_CS | FCTR_FS_WRE | FCTR_FS_WPB | FCTR_FS_PROGREQ);

				/* Wait for the end of the write operation. If it's not over
				 * after one second, something went dreadfully wrong... :-(
				 */
				if (lpc2900_wait_status(bank, INTSRC_END_OF_BURN, 1000) != ERROR_OK) {
					LOG_ERROR("Write failed @ 0x%8.8" PRIx32, offset);
					target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

					return ERROR_FLASH_OPERATION_FAILED;
				}
			}

			/* Update pointers and counters */
			offset += num_bytes;
			buffer += num_bytes;
			count -= num_bytes;
		}

		retval = ERROR_OK;
	}

	/* Normal flash operating mode */
	target_write_u32(target, FCTR, FCTR_FS_CS | FCTR_FS_WEB);

	return retval;
}

/**
 * Try and identify the device.
 *
 * Determine type number and its memory layout.
 *
 * @param bank Pointer to the flash bank descriptor
 */
static int lpc2900_probe(struct flash_bank *bank)
{
	struct lpc2900_flash_bank *lpc2900_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t offset;


	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* We want to do this only once. */
	if (lpc2900_info->is_probed)
		return ERROR_OK;

	/* Probing starts with reading the CHIPID register. We will continue only
	 * if this identifies as an LPC2900 device.
	 */
	target_read_u32(target, CHIPID, &lpc2900_info->chipid);

	if (lpc2900_info->chipid != EXPECTED_CHIPID) {
		LOG_WARNING("Device is not an LPC29xx");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* It's an LPC29xx device. Now read the feature register FEAT0...FEAT3. */
	uint32_t feat0, feat1, feat2, feat3;
	target_read_u32(target, FEAT0, &feat0);
	target_read_u32(target, FEAT1, &feat1);
	target_read_u32(target, FEAT2, &feat2);
	target_read_u32(target, FEAT3, &feat3);

	/* Base address */
	bank->base = 0x20000000;

	/* Determine flash layout from FEAT2 register */
	uint32_t num_64k_sectors = (feat2 >> 16) & 0xFF;
	uint32_t num_8k_sectors = (feat2 >> 0) & 0xFF;
	bank->num_sectors = num_64k_sectors + num_8k_sectors;
	bank->size = KiB * (64 * num_64k_sectors + 8 * num_8k_sectors);

	/* Determine maximum contiguous RAM block */
	lpc2900_info->max_ram_block = 16 * KiB;
	if ((feat1 & 0x30) == 0x30) {
		lpc2900_info->max_ram_block = 32 * KiB;
		if ((feat1 & 0x0C) == 0x0C)
			lpc2900_info->max_ram_block = 48 * KiB;
	}

	/* Determine package code and ITCM size */
	uint32_t package_code = feat0 & 0x0F;
	uint32_t itcm_code = (feat1 >> 16) & 0x1F;

	/* Determine the exact type number. */
	uint32_t found = 1;
	if ((package_code == 4) && (itcm_code == 5)) {
		/* Old LPC2917 or LPC2919 (non-/01 devices) */
		lpc2900_info->target_name = (bank->size == 768*KiB) ? "LPC2919" : "LPC2917";
	} else {
		if (package_code == 2) {
			/* 100-pin package */
			if (bank->size == 128*KiB)
				lpc2900_info->target_name = "LPC2921";
			else if (bank->size == 256*KiB)
				lpc2900_info->target_name = "LPC2923";
			else if (bank->size == 512*KiB)
				lpc2900_info->target_name = "LPC2925";
			else
				found = 0;
		} else if (package_code == 4) {
			/* 144-pin package */
			if ((bank->size == 256*KiB) && (feat3 == 0xFFFFFFE9))
				lpc2900_info->target_name = "LPC2926";
			else if ((bank->size == 512*KiB) && (feat3 == 0xFFFFFCF0))
				lpc2900_info->target_name = "LPC2917/01";
			else if ((bank->size == 512*KiB) && (feat3 == 0xFFFFFFF1))
				lpc2900_info->target_name = "LPC2927";
			else if ((bank->size == 768*KiB) && (feat3 == 0xFFFFFCF8))
				lpc2900_info->target_name = "LPC2919/01";
			else if ((bank->size == 768*KiB) && (feat3 == 0xFFFFFFF9))
				lpc2900_info->target_name = "LPC2929";
			else
				found = 0;
		} else if (package_code == 5) {
			/* 208-pin package */
			lpc2900_info->target_name = (bank->size == 0) ? "LPC2930" : "LPC2939";
		} else
			found = 0;
	}

	if (!found) {
		LOG_WARNING("Unknown LPC29xx derivative (FEATx="
			"%08" PRIx32 ":%08" PRIx32 ":%08" PRIx32 ":%08" PRIx32 ")",
			feat0, feat1, feat2, feat3);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Show detected device */
	LOG_INFO("Flash bank %u: Device %s, %" PRIu32
		" KiB in %u sectors",
		bank->bank_number,
		lpc2900_info->target_name, bank->size / KiB,
		bank->num_sectors);

	/* Flashless devices cannot be handled */
	if (bank->num_sectors == 0) {
		LOG_WARNING("Flashless device cannot be handled");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Sector layout.
	 * These are logical sector numbers. When doing real flash operations,
	 * the logical flash number are translated into the physical flash numbers
	 * of the device.
	 */
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	offset = 0;
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;

		if (i <= 7)
			bank->sectors[i].size = 8 * KiB;
		else if (i <= 18)
			bank->sectors[i].size = 64 * KiB;
		else {
			/* We shouldn't come here. But there might be a new part out there
			 * that has more than 19 sectors. Politely ask for a fix then.
			 */
			bank->sectors[i].size = 0;
			LOG_ERROR("Never heard about sector %u", i);
		}

		offset += bank->sectors[i].size;
	}

	lpc2900_info->is_probed = true;

	/* Read sector security status */
	if (lpc2900_read_security_status(bank) != ERROR_OK) {
		LOG_ERROR("Cannot determine sector security status");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

/**
 * Run a blank check for each sector.
 *
 * For speed reasons, the device isn't read word by word.
 * A hash value is calculated by the hardware ("BIST") for each sector.
 * This value is then compared against the known hash of an empty sector.
 *
 * @param bank Pointer to the flash bank descriptor
 */
static int lpc2900_erase_check(struct flash_bank *bank)
{
	uint32_t status = lpc2900_is_ready(bank);
	if (status != ERROR_OK) {
		LOG_INFO("Processor not halted/not probed");
		return status;
	}

	/* Use the BIST (Built-In Self Test) to generate a signature of each flash
	 * sector. Compare against the expected signature of an empty sector.
	 */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		uint32_t signature[4];
		status = lpc2900_run_bist128(bank, bank->sectors[sector].offset,
				bank->sectors[sector].offset + (bank->sectors[sector].size - 1), signature);
		if (status != ERROR_OK)
			return status;

		/* The expected signatures for an empty sector are different
		 * for 8 KiB and 64 KiB sectors.
		 */
		if (bank->sectors[sector].size == 8*KiB) {
			bank->sectors[sector].is_erased =
				(signature[3] == 0x01ABAAAA) &&
				(signature[2] == 0xAAAAAAAA) &&
				(signature[1] == 0xAAAAAAAA) &&
				(signature[0] == 0xAAA00AAA);
		}
		if (bank->sectors[sector].size == 64*KiB) {
			bank->sectors[sector].is_erased =
				(signature[3] == 0x11801222) &&
				(signature[2] == 0xB88844FF) &&
				(signature[1] == 0x11A22008) &&
				(signature[0] == 0x2B1BFE44);
		}
	}

	return ERROR_OK;
}

/**
 * Get protection (sector security) status.
 *
 * Determine the status of "sector security" for each sector.
 * A secured sector is one that can never be erased/programmed again.
 *
 * @param bank Pointer to the flash bank descriptor
 */
static int lpc2900_protect_check(struct flash_bank *bank)
{
	return lpc2900_read_security_status(bank);
}

const struct flash_driver lpc2900_flash = {
	.name = "lpc2900",
	.commands = lpc2900_command_handlers,
	.flash_bank_command = lpc2900_flash_bank_command,
	.erase = lpc2900_erase,
	.write = lpc2900_write,
	.read = default_flash_read,
	.probe = lpc2900_probe,
	.auto_probe = lpc2900_probe,
	.erase_check = lpc2900_erase_check,
	.protect_check = lpc2900_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
