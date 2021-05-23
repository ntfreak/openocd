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

/*
 * NAND controller interface for Nuvoton NUC910
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "nuc910.h"
#include "arm_io.h"
#include <target/arm.h>

struct nuc910_nand_controller {
	struct arm_nand_data io;
};

static int validate_target_state(struct nand_device *nand)
{
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_NAND_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int nuc910_nand_command(struct nand_device *nand, uint8_t command)
{
	struct target *target = nand->target;
	int result;

	result = validate_target_state(nand);
	if (result != ERROR_OK)
		return result;

	target_write_u8(target, NUC910_SMCMD, command);
	return ERROR_OK;
}

static int nuc910_nand_address(struct nand_device *nand, uint8_t address)
{
	struct target *target = nand->target;
	int result;

	result = validate_target_state(nand);
	if (result != ERROR_OK)
		return result;

	target_write_u32(target, NUC910_SMADDR, ((address & 0xff) | NUC910_SMADDR_EOA));
	return ERROR_OK;
}

static int nuc910_nand_read(struct nand_device *nand, void *data)
{
	struct target *target = nand->target;
	int result;

	result = validate_target_state(nand);
	if (result != ERROR_OK)
		return result;

	target_read_u8(target, NUC910_SMDATA, data);
	return ERROR_OK;
}

static int nuc910_nand_write(struct nand_device *nand, uint16_t data)
{
	struct target *target = nand->target;
	int result;

	result = validate_target_state(nand);
	if (result != ERROR_OK)
		return result;

	target_write_u8(target, NUC910_SMDATA, data);
	return ERROR_OK;
}

static int nuc910_nand_read_block_data(struct nand_device *nand,
		uint8_t *data, int data_size)
{
	struct nuc910_nand_controller *nuc910_nand = nand->controller_priv;
	int result;

	result = validate_target_state(nand);
	if (result != ERROR_OK)
		return result;

	nuc910_nand->io.chunk_size = nand->page_size;

	/* try the fast way first */
	result = arm_nandread(&nuc910_nand->io, data, data_size);
	if (result != ERROR_NAND_NO_BUFFER)
		return result;

	/* else do it slowly */
	while (data_size--)
		nuc910_nand_read(nand, data++);

	return ERROR_OK;
}

static int nuc910_nand_write_block_data(struct nand_device *nand,
		uint8_t *data, int data_size)
{
	struct nuc910_nand_controller *nuc910_nand = nand->controller_priv;
	int result;

	result = validate_target_state(nand);
	if (result != ERROR_OK)
		return result;

	nuc910_nand->io.chunk_size = nand->page_size;

	/* try the fast way first */
	result = arm_nandwrite(&nuc910_nand->io, data, data_size);
	if (result != ERROR_NAND_NO_BUFFER)
		return result;

	/* else do it slowly */
	while (data_size--)
		nuc910_nand_write(nand, *data++);

	return ERROR_OK;
}

static int nuc910_nand_reset(struct nand_device *nand)
{
	return nuc910_nand_command(nand, NAND_CMD_RESET);
}

static int nuc910_nand_ready(struct nand_device *nand, int timeout)
{
	struct target *target = nand->target;
	uint32_t status;

	do {
		target_read_u32(target, NUC910_SMISR, &status);
		if (status & NUC910_SMISR_RB_)
			return 1;
		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

NAND_DEVICE_COMMAND_HANDLER(nuc910_nand_device_command)
{
	struct nuc910_nand_controller *nuc910_nand;

	nuc910_nand = calloc(1, sizeof(struct nuc910_nand_controller));
	if (!nuc910_nand) {
		LOG_ERROR("no memory for nand controller");
		return ERROR_NAND_DEVICE_INVALID;
	}

	nand->controller_priv = nuc910_nand;
	return ERROR_OK;
}

static int nuc910_nand_init(struct nand_device *nand)
{
	struct nuc910_nand_controller *nuc910_nand = nand->controller_priv;
	struct target *target = nand->target;
	int bus_width = nand->bus_width ? nand->bus_width : 8;
	int result;

	result = validate_target_state(nand);
	if (result != ERROR_OK)
		return result;

	/* nuc910 only supports 8bit */
	if (bus_width != 8) {
		LOG_ERROR("nuc910 only supports 8 bit bus width, not %i", bus_width);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}

	/* inform calling code about selected bus width */
	nand->bus_width = bus_width;

	nuc910_nand->io.target = target;
	nuc910_nand->io.data = NUC910_SMDATA;
	nuc910_nand->io.op = ARM_NAND_NONE;

	/* configure nand controller */
	target_write_u32(target, NUC910_FMICSR, NUC910_FMICSR_SM_EN);
	target_write_u32(target, NUC910_SMCSR, 0x010000a8);	/* 2048 page size */
	target_write_u32(target, NUC910_SMTCR, 0x00010204);
	target_write_u32(target, NUC910_SMIER, 0x00000000);

	return ERROR_OK;
}

struct nand_flash_controller nuc910_nand_controller = {
	.name = "nuc910",
	.command = nuc910_nand_command,
	.address = nuc910_nand_address,
	.read_data = nuc910_nand_read,
	.write_data	= nuc910_nand_write,
	.write_block_data = nuc910_nand_write_block_data,
	.read_block_data = nuc910_nand_read_block_data,
	.nand_ready = nuc910_nand_ready,
	.reset = nuc910_nand_reset,
	.nand_device_command = nuc910_nand_device_command,
	.init = nuc910_nand_init,
};
