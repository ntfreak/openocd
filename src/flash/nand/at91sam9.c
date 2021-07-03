/*
 * Copyright (C) 2009 by Dean Glazeski
 * dnglaze@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/arm.h>
#include <helper/log.h>
#include "imp.h"
#include "arm_io.h"

#define AT91C_PIOX_SODR (0x30)	/**< Offset to PIO SODR. */
#define AT91C_PIOX_CODR (0x34)	/**< Offset to PIO CODR. */
#define AT91C_PIOX_PDSR (0x3C)	/**< Offset to PIO PDSR. */
#define AT91C_ECCX_CR (0x00)	/**< Offset to ECC CR. */
#define AT91C_ECCX_SR (0x08)	/**< Offset to ECC SR. */
#define AT91C_ECCX_PR (0x0C)	/**< Offset to ECC PR. */
#define AT91C_ECCX_NPR (0x10)	/**< Offset to ECC NPR. */

/**
 * Representation of a pin on an AT91SAM9 chip.
 */
struct at91sam9_pin {
	/** Address of the PIO controller. */
	uint32_t pioc;

	/** Pin number. */
	uint32_t num;
};

/**
 * Private data for the controller that is stored in the NAND device structure.
 */
struct at91sam9_nand {
	/** Address of the ECC controller for NAND. */
	uint32_t ecc;

	/** Address data is written to. */
	uint32_t data;

	/** Address commands are written to. */
	uint32_t cmd;

	/** Address addresses are written to. */
	uint32_t addr;

	/** I/O structure for hosted reads/writes. */
	struct arm_nand_data io;

	/** Pin representing the ready/~busy line. */
	struct at91sam9_pin busy;

	/** Pin representing the chip enable. */
	struct at91sam9_pin ce;
};

/**
 * Checks if the target is halted and prints an error message if it isn't.
 *
 * @param target Target to be checked.
 * @param label String label for where function is called from.
 * @return True if the target is halted.
 */
static int at91sam9_halted(struct target *target, const char *label)
{
	if (target->state == TARGET_HALTED)
		return true;

	LOG_ERROR("Target must be halted to use NAND controller (%s)", label);
	return false;
}

/**
 * Initialize the AT91SAM9 NAND controller.
 *
 * @param nand NAND device the controller is attached to.
 * @return Success or failure of initialization.
 */
static int at91sam9_init(struct nand_device *nand)
{
	struct target *target = nand->target;

	if (!at91sam9_halted(target, "init"))
		return ERROR_NAND_OPERATION_FAILED;

	return ERROR_OK;
}

/**
 * Enable NAND device attached to a controller.
 *
 * @param nand NAND controller information for controlling NAND device.
 * @return Success or failure of the enabling.
 */
static int at91sam9_enable(struct nand_device *nand)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	return target_write_u32(target, info->ce.pioc + AT91C_PIOX_CODR, 1 << info->ce.num);
}

/**
 * Disable NAND device attached to a controller.
 *
 * @param nand NAND controller information for controlling NAND device.
 * @return Success or failure of the disabling.
 */
static int at91sam9_disable(struct nand_device *nand)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	return target_write_u32(target, info->ce.pioc + AT91C_PIOX_SODR, 1 << info->ce.num);
}

/**
 * Send a command to the NAND device.
 *
 * @param nand NAND device to write the command to.
 * @param command Command to be written.
 * @return Success or failure of writing the command.
 */
static int at91sam9_command(struct nand_device *nand, uint8_t command)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!at91sam9_halted(target, "command"))
		return ERROR_NAND_OPERATION_FAILED;

	at91sam9_enable(nand);

	return target_write_u8(target, info->cmd, command);
}

/**
 * Reset the AT91SAM9 NAND controller.
 *
 * @param nand NAND device to be reset.
 * @return Success or failure of reset.
 */
static int at91sam9_reset(struct nand_device *nand)
{
	if (!at91sam9_halted(nand->target, "reset"))
		return ERROR_NAND_OPERATION_FAILED;

	return at91sam9_disable(nand);
}

/**
 * Send an address to the NAND device attached to an AT91SAM9 NAND controller.
 *
 * @param nand NAND device to send the address to.
 * @param address Address to be sent.
 * @return Success or failure of sending the address.
 */
static int at91sam9_address(struct nand_device *nand, uint8_t address)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!at91sam9_halted(nand->target, "address"))
		return ERROR_NAND_OPERATION_FAILED;

	return target_write_u8(target, info->addr, address);
}

/**
 * Read data directly from the NAND device attached to an AT91SAM9 NAND
 * controller.
 *
 * @param nand NAND device to read from.
 * @param data Pointer to where the data should be put.
 * @return Success or failure of reading the data.
 */
static int at91sam9_read_data(struct nand_device *nand, void *data)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!at91sam9_halted(nand->target, "read data"))
		return ERROR_NAND_OPERATION_FAILED;

	return target_read_u8(target, info->data, data);
}

/**
 * Write data directly to the NAND device attached to an AT91SAM9 NAND
 * controller.
 *
 * @param nand NAND device to be written to.
 * @param data Data to be written.
 * @return Success or failure of the data write.
 */
static int at91sam9_write_data(struct nand_device *nand, uint16_t data)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;

	if (!at91sam9_halted(target, "write data"))
		return ERROR_NAND_OPERATION_FAILED;

	return target_write_u8(target, info->data, data);
}

/**
 * Determine if the NAND device is ready by looking at the ready/~busy pin.
 *
 * @param nand NAND device to check.
 * @param timeout Time in milliseconds to wait for NAND to be ready.
 * @return True if the NAND is ready in the timeout period.
 */
static int at91sam9_nand_ready(struct nand_device *nand, int timeout)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t status;

	if (!at91sam9_halted(target, "nand ready"))
		return 0;

	do {
		target_read_u32(target, info->busy.pioc + AT91C_PIOX_PDSR, &status);

		if (status & (1 << info->busy.num))
			return 1;

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

/**
 * Read a block of data from the NAND device attached to an AT91SAM9.  This
 * utilizes the ARM hosted NAND read function.
 *
 * @param nand NAND device to read from.
 * @param data Pointer to where the read data should be placed.
 * @param size Size of the data being read.
 * @return Success or failure of the hosted read.
 */
static int at91sam9_read_block_data(struct nand_device *nand, uint8_t *data, int size)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct arm_nand_data *io = &info->io;
	int status;

	if (!at91sam9_halted(nand->target, "read block"))
		return ERROR_NAND_OPERATION_FAILED;

	io->chunk_size = nand->page_size;
	status = arm_nandread(io, data, size);

	return status;
}

/**
 * Write a block of data to a NAND device attached to an AT91SAM9.  This uses
 * the ARM hosted write function to write the data.
 *
 * @param nand NAND device to write to.
 * @param data Data to be written to device.
 * @param size Size of the data being written.
 * @return Success or failure of the hosted write.
 */
static int at91sam9_write_block_data(struct nand_device *nand, uint8_t *data, int size)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct arm_nand_data *io = &info->io;
	int status;

	if (!at91sam9_halted(nand->target, "write block"))
		return ERROR_NAND_OPERATION_FAILED;

	io->chunk_size = nand->page_size;
	status = arm_nandwrite(io, data, size);

	return status;
}

/**
 * Initialize the ECC controller on the AT91SAM9.
 *
 * @param target Target to configure ECC on.
 * @param info NAND controller information for where the ECC is.
 * @return Success or failure of initialization.
 */
static int at91sam9_ecc_init(struct target *target, struct at91sam9_nand *info)
{
	if (!info->ecc) {
		LOG_ERROR("ECC controller address must be set when not reading raw NAND data");
		return ERROR_NAND_OPERATION_FAILED;
	}

	/* reset ECC parity registers */
	return target_write_u32(target, info->ecc + AT91C_ECCX_CR, 1);
}

/**
 * Initialize an area for the OOB based on whether a user is requesting the OOB
 * data.  This determines the size of the OOB and allocates the space in case
 * the user has not requested the OOB data.
 *
 * @param nand NAND device we are creating an OOB for.
 * @param oob Pointer to the user supplied OOB area.
 * @param size Size of the OOB.
 * @return Pointer to an area to store OOB data.
 */
static uint8_t *at91sam9_oob_init(struct nand_device *nand, uint8_t *oob, uint32_t *size)
{
	if (!oob) {
		/* user doesn't want OOB, allocate it */
		if (nand->page_size == 512)
			*size = 16;
		else if (nand->page_size == 2048)
			*size = 64;

		oob = malloc(*size);
		if (!oob) {
			LOG_ERROR("Unable to allocate space for OOB");
			return NULL;
		}

		memset(oob, 0xFF, *size);
	}

	return oob;
}

/**
 * Reads a page from an AT91SAM9 NAND controller and verifies using 1-bit ECC
 * controller on chip.  This makes an attempt to correct any errors that are
 * encountered while reading the page of data.
 *
 * @param nand NAND device to read from
 * @param page Page to be read.
 * @param data Pointer to where data should be read to.
 * @param data_size Size of the data to be read.
 * @param oob Pointer to where OOB data should be read to.
 * @param oob_size Size of the OOB data to be read.
 * @return Success or failure of reading the NAND page.
 */
static int at91sam9_read_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	int retval;
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	uint8_t *oob_data;
	uint32_t status;

	retval = at91sam9_ecc_init(target, info);
	if (retval != ERROR_OK)
		return retval;

	retval = nand_page_command(nand, page, NAND_CMD_READ0, !data);
	if (retval != ERROR_OK)
		return retval;

	if (data) {
		retval = nand_read_data_page(nand, data, data_size);
		if (retval != ERROR_OK)
			return retval;
	}

	oob_data = at91sam9_oob_init(nand, oob, &oob_size);
	retval = nand_read_data_page(nand, oob_data, oob_size);
	if (retval == ERROR_OK && data) {
		target_read_u32(target, info->ecc + AT91C_ECCX_SR, &status);
		if (status & 1) {
			LOG_ERROR("Error detected!");
			if (status & 4)
				LOG_ERROR("Multiple errors encountered; unrecoverable!");
			else {
				/* attempt recovery */
				uint32_t parity;

				target_read_u32(target,
					info->ecc + AT91C_ECCX_PR,
					&parity);
				uint32_t word = (parity & 0x0000FFF0) >> 4;
				uint32_t bit = parity & 0x0F;

				data[word] ^= (0x1) << bit;
				LOG_INFO("Data word %d, bit %d corrected.",
					(unsigned) word,
					(unsigned) bit);
			}
		}

		if (status & 2) {
			/* we could write back correct ECC data */
			LOG_ERROR("Error in ECC bytes detected");
		}
	}

	if (!oob) {
		/* if it wasn't asked for, free it */
		free(oob_data);
	}

	return retval;
}

/**
 * Write a page of data including 1-bit ECC information to a NAND device
 * attached to an AT91SAM9 controller.  If there is OOB data to be written,
 * this will ignore the computed ECC from the ECC controller.
 *
 * @param nand NAND device to write to.
 * @param page Page to write.
 * @param data Pointer to data being written.
 * @param data_size Size of the data being written.
 * @param oob Pointer to OOB data being written.
 * @param oob_size Size of the OOB data.
 * @return Success or failure of the page write.
 */
static int at91sam9_write_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size, uint8_t *oob, uint32_t oob_size)
{
	struct at91sam9_nand *info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;
	uint8_t *oob_data = oob;
	uint32_t parity, nparity;

	retval = at91sam9_ecc_init(target, info);
	if (retval != ERROR_OK)
		return retval;

	retval = nand_page_command(nand, page, NAND_CMD_SEQIN, !data);
	if (retval != ERROR_OK)
		return retval;

	if (data) {
		retval = nand_write_data_page(nand, data, data_size);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write data to NAND device");
			return retval;
		}
	}

	oob_data = at91sam9_oob_init(nand, oob, &oob_size);

	if (!oob) {
		/* no OOB given, so read in the ECC parity from the ECC controller */
		target_read_u32(target, info->ecc + AT91C_ECCX_PR, &parity);
		target_read_u32(target, info->ecc + AT91C_ECCX_NPR, &nparity);

		oob_data[0] = (uint8_t) parity;
		oob_data[1] = (uint8_t) (parity >> 8);
		oob_data[2] = (uint8_t) nparity;
		oob_data[3] = (uint8_t) (nparity >> 8);
	}

	retval = nand_write_data_page(nand, oob_data, oob_size);

	if (!oob)
		free(oob_data);

	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to write OOB data to NAND");
		return retval;
	}

	retval = nand_write_finish(nand);

	return retval;
}

/**
 * Handle the initial NAND device command for AT91SAM9 controllers.  This
 * initializes much of the controller information struct to be ready for future
 * reads and writes.
 */
NAND_DEVICE_COMMAND_HANDLER(at91sam9_nand_device_command)
{
	unsigned long chip = 0, ecc = 0;
	struct at91sam9_nand *info = NULL;

	LOG_DEBUG("AT91SAM9 NAND Device Command");

	if (CMD_ARGC < 3 || CMD_ARGC > 4) {
		LOG_ERROR("parameters: %s target chip_addr", CMD_ARGV[0]);
		return ERROR_NAND_OPERATION_FAILED;
	}

	COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[2], chip);
	if (chip == 0) {
		LOG_ERROR("invalid NAND chip address: %s", CMD_ARGV[2]);
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[3], ecc);
		if (ecc == 0) {
			LOG_ERROR("invalid ECC controller address: %s", CMD_ARGV[3]);
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	info = calloc(1, sizeof(*info));
	if (!info) {
		LOG_ERROR("unable to allocate space for controller private data");
		return ERROR_NAND_OPERATION_FAILED;
	}

	info->data = chip;
	info->cmd = chip | (1 << 22);
	info->addr = chip | (1 << 21);
	info->ecc = ecc;

	nand->controller_priv = info;
	info->io.target = nand->target;
	info->io.data = info->data;
	info->io.op = ARM_NAND_NONE;

	return ERROR_OK;
}

/**
 * Handle the AT91SAM9 CLE command for specifying the address line to use for
 * writing commands to a NAND device.
 */
COMMAND_HANDLER(handle_at91sam9_cle_command)
{
	struct nand_device *nand = NULL;
	struct at91sam9_nand *info = NULL;
	unsigned num, address_line;

	if (CMD_ARGC != 2) {
		command_print(CMD, "incorrect number of arguments for 'at91sam9 cle' command");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num);
	nand = get_nand_device_by_num(num);
	if (!nand) {
		command_print(CMD, "invalid nand device number: %s", CMD_ARGV[0]);
		return ERROR_OK;
	}

	info = nand->controller_priv;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], address_line);
	info->cmd = info->data | (1 << address_line);

	return ERROR_OK;
}

/**
 * Handle the AT91SAM9 ALE command for specifying the address line to use for
 * writing addresses to the NAND device.
 */
COMMAND_HANDLER(handle_at91sam9_ale_command)
{
	struct nand_device *nand = NULL;
	struct at91sam9_nand *info = NULL;
	unsigned num, address_line;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num);
	nand = get_nand_device_by_num(num);
	if (!nand) {
		command_print(CMD, "invalid nand device number: %s", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	info = nand->controller_priv;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], address_line);
	info->addr = info->data | (1 << address_line);

	return ERROR_OK;
}

/**
 * Handle the AT91SAM9 RDY/~BUSY command for specifying the pin that watches the
 * RDY/~BUSY line from the NAND device.
 */
COMMAND_HANDLER(handle_at91sam9_rdy_busy_command)
{
	struct nand_device *nand = NULL;
	struct at91sam9_nand *info = NULL;
	unsigned num, base_pioc, pin_num;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num);
	nand = get_nand_device_by_num(num);
	if (!nand) {
		command_print(CMD, "invalid nand device number: %s", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	info = nand->controller_priv;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], base_pioc);
	info->busy.pioc = base_pioc;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[2], pin_num);
	info->busy.num = pin_num;

	return ERROR_OK;
}

/**
 * Handle the AT91SAM9 CE command for specifying the pin that is used to enable
 * or disable the NAND device.
 */
COMMAND_HANDLER(handle_at91sam9_ce_command)
{
	struct nand_device *nand = NULL;
	struct at91sam9_nand *info = NULL;
	unsigned num, base_pioc, pin_num;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num);
	nand = get_nand_device_by_num(num);
	if (!nand) {
		command_print(CMD, "invalid nand device number: %s", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	info = nand->controller_priv;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], base_pioc);
	info->ce.pioc = base_pioc;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[2], pin_num);
	info->ce.num = pin_num;

	return ERROR_OK;
}

static const struct command_registration at91sam9_sub_command_handlers[] = {
	{
		.name = "cle",
		.handler = handle_at91sam9_cle_command,
		.mode = COMMAND_CONFIG,
		.help = "set command latch enable address line (default is 22)",
		.usage = "bank_id address_line",
	},
	{
		.name = "ale",
		.handler = handle_at91sam9_ale_command,
		.mode = COMMAND_CONFIG,
		.help = "set address latch enable address line (default is 21)",
		.usage = "bank_id address_line",
	},
	{
		.name = "rdy_busy",
		.handler = handle_at91sam9_rdy_busy_command,
		.mode = COMMAND_CONFIG,
		.help = "set the GPIO input pin connected to "
			"the RDY/~BUSY signal (no default)",
		.usage = "bank_id pio_base_addr pin_num",
	},
	{
		.name = "ce",
		.handler = handle_at91sam9_ce_command,
		.mode = COMMAND_CONFIG,
		.help = "set the GPIO output pin connected to "
			"the chip enable signal (no default)",
		.usage = "bank_id pio_base_addr pin_num",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration at91sam9_command_handler[] = {
	{
		.name = "at91sam9",
		.mode = COMMAND_ANY,
		.help = "AT91SAM9 NAND flash controller commands",
		.usage = "",
		.chain = at91sam9_sub_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/**
 * Structure representing the AT91SAM9 NAND controller.
 */
struct nand_flash_controller at91sam9_nand_controller = {
	.name = "at91sam9",
	.nand_device_command = at91sam9_nand_device_command,
	.commands = at91sam9_command_handler,
	.init = at91sam9_init,
	.command = at91sam9_command,
	.reset = at91sam9_reset,
	.address = at91sam9_address,
	.read_data = at91sam9_read_data,
	.write_data = at91sam9_write_data,
	.nand_ready = at91sam9_nand_ready,
	.read_block_data = at91sam9_read_block_data,
	.write_block_data = at91sam9_write_block_data,
	.read_page = at91sam9_read_page,
	.write_page = at91sam9_write_page,
};
