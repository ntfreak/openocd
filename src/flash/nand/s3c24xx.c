/***************************************************************************
 *   Copyright (C) 2007, 2008 by Ben Dooks                                 *
 *   ben@fluff.org                                                         *
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
 * S3C24XX Series OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "s3c24xx.h"

S3C24XX_DEVICE_COMMAND()
{
	*info = NULL;

	struct s3c24xx_nand_controller *s3c24xx_info;
	s3c24xx_info = malloc(sizeof(struct s3c24xx_nand_controller));
	if (!s3c24xx_info) {
		LOG_ERROR("no memory for nand controller");
		return -ENOMEM;
	}

	nand->controller_priv = s3c24xx_info;
	*info = s3c24xx_info;

	return ERROR_OK;
}

int s3c24xx_reset(struct nand_device *nand)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u32(target, s3c24xx_info->cmd, 0xff);

	return ERROR_OK;
}

int s3c24xx_command(struct nand_device *nand, uint8_t command)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u16(target, s3c24xx_info->cmd, command);
	return ERROR_OK;
}

int s3c24xx_address(struct nand_device *nand, uint8_t address)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u16(target, s3c24xx_info->addr, address);
	return ERROR_OK;
}

int s3c24xx_write_data(struct nand_device *nand, uint16_t data)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u8(target, s3c24xx_info->data, data);
	return ERROR_OK;
}

int s3c24xx_read_data(struct nand_device *nand, void *data)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_read_u8(target, s3c24xx_info->data, data);
	return ERROR_OK;
}
