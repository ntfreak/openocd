/***************************************************************************
 *   Copyright (C) 2020 Lubomir Rintel                                     *
 *   lkundrak@v3.sk                                                        *
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
#include <target/target.h>
#include <target/mips32.h>

#define EMC_BASE		KSEG1 + 0x13010000

#define NAND_DATAPORT		KSEG1 + 0x14000000
#define NAND_COMMPORT		KSEG1 + 0x14040000
#define NAND_ADDRPORT		KSEG1 + 0x14080000

#define EMC_NFCSR		0x50
#define EMC_NFECC		0x54

#define EMC_NFCSR_RB		(1 << 7)
#define EMC_NFCSR_FCE		(1 << 1)
#define EMC_NFCSR_NFE		(1 << 0)

static int jz4730_nand_command(struct nand_device *nand, uint8_t command)
{
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("NAND flash access requires halted target");
		return ERROR_NAND_OPERATION_FAILED;
	}
	target_write_u8(target, NAND_COMMPORT, command);

	return ERROR_OK;
}

static int jz4730_nand_address(struct nand_device *nand, uint8_t address)
{
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("NAND flash access requires halted target");
		return ERROR_NAND_OPERATION_FAILED;
	}
	target_write_u8(target, NAND_ADDRPORT, address);

	return ERROR_OK;
}

static int jz4730_nand_read(struct nand_device *nand, void *data)
{
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("NAND flash access requires halted target");
		return ERROR_NAND_OPERATION_FAILED;
	}
	target_read_u8(target, NAND_DATAPORT, data);

	return ERROR_OK;
}

static int jz4730_nand_write(struct nand_device *nand, uint16_t data)
{
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("NAND flash access requires halted target");
		return ERROR_NAND_OPERATION_FAILED;
	}
	target_write_u8(target, NAND_DATAPORT, data);

	return ERROR_OK;
}

static int jz4730_nand_reset(struct nand_device *nand)
{
	return jz4730_nand_command(nand, NAND_CMD_RESET);
}

NAND_DEVICE_COMMAND_HANDLER(jz4730_nand_device_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static int jz4730_nand_init(struct nand_device *nand)
{
	struct target *target = nand->target;
	uint32_t value;

	target_read_u32(target, EMC_BASE + EMC_NFCSR, &value);
	value |= EMC_NFCSR_NFE | EMC_NFCSR_FCE;
	target_write_u32(target, EMC_BASE + EMC_NFCSR, value);

	return ERROR_OK;
}

int jz4730_nand_ready(struct nand_device *nand, int timeout)
{
	struct target *target = nand->target;
	uint32_t val;

	while (timeout--) {
		target_read_u32(target, EMC_BASE + EMC_NFCSR, &val);
		if (val & EMC_NFCSR_RB)
			return 1;
		alive_sleep(1);
	}

	return 0;
}

struct nand_flash_controller jz4730_nand_controller = {
	.name = "jz4730",
	.usage = "",
	.nand_device_command = jz4730_nand_device_command,
	.init = jz4730_nand_init,
	.reset = jz4730_nand_reset,
	.command = jz4730_nand_command,
	.address = jz4730_nand_address,
	.write_data = jz4730_nand_write,
	.read_data = jz4730_nand_read,
	.nand_ready = jz4730_nand_ready,
};
