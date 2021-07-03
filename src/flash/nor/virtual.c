/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

static struct flash_bank *virtual_get_master_bank(struct flash_bank *bank)
{
	struct flash_bank *master_bank;

	master_bank = get_flash_bank_by_name_noprobe(bank->driver_priv);
	if (!master_bank)
		LOG_ERROR("master flash bank '%s' does not exist", (char *)bank->driver_priv);

	return master_bank;
}

static void virtual_update_bank_info(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return;

	/* update the info we do not have */
	bank->size = master_bank->size;
	bank->chip_width = master_bank->chip_width;
	bank->bus_width = master_bank->bus_width;
	bank->erased_value = master_bank->erased_value;
	bank->default_padded_value = master_bank->default_padded_value;
	bank->write_start_alignment = master_bank->write_start_alignment;
	bank->write_end_alignment = master_bank->write_end_alignment;
	bank->minimal_write_gap = master_bank->minimal_write_gap;
	bank->num_sectors = master_bank->num_sectors;
	bank->sectors = master_bank->sectors;
	bank->num_prot_blocks = master_bank->num_prot_blocks;
	bank->prot_blocks = master_bank->prot_blocks;
}

FLASH_BANK_COMMAND_HANDLER(virtual_flash_bank_command)
{
	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* get the master flash bank */
	const char *bank_name = CMD_ARGV[6];
	struct flash_bank *master_bank = get_flash_bank_by_name_noprobe(bank_name);

	if (!master_bank) {
		LOG_ERROR("master flash bank '%s' does not exist", bank_name);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* save master bank name - use this to get settings later */
	bank->driver_priv = strdup(bank_name);

	return ERROR_OK;
}

static int virtual_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	return flash_driver_protect(master_bank, set, first, last);
}

static int virtual_protect_check(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	if (!master_bank->driver->protect_check)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	/* call master handler */
	return master_bank->driver->protect_check(master_bank);
}

static int virtual_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->erase(master_bank, first, last);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int virtual_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->write(master_bank, buffer, offset, count);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int virtual_probe(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->probe(master_bank);
	if (retval != ERROR_OK)
		return retval;

	/* update the info we do not have */
	virtual_update_bank_info(bank);

	return ERROR_OK;
}

static int virtual_auto_probe(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->auto_probe(master_bank);
	if (retval != ERROR_OK)
		return retval;

	/* update the info we do not have */
	virtual_update_bank_info(bank);

	return ERROR_OK;
}

static int virtual_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	command_print_sameline(cmd, "%s driver for flash bank %s at " TARGET_ADDR_FMT,
			bank->driver->name, master_bank->name, master_bank->base);

	return ERROR_OK;
}

static int virtual_blank_check(struct flash_bank *bank)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->erase_check(master_bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int virtual_flash_read(struct flash_bank *bank,
		uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct flash_bank *master_bank = virtual_get_master_bank(bank);
	int retval;

	if (!master_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* call master handler */
	retval = master_bank->driver->read(master_bank, buffer, offset, count);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

const struct flash_driver virtual_flash = {
	.name = "virtual",
	.flash_bank_command = virtual_flash_bank_command,
	.erase = virtual_erase,
	.protect = virtual_protect,
	.write = virtual_write,
	.read = virtual_flash_read,
	.probe = virtual_probe,
	.auto_probe = virtual_auto_probe,
	.erase_check = virtual_blank_check,
	.protect_check = virtual_protect_check,
	.info = virtual_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
