/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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

#ifndef OPENOCD_FLASH_NAND_DRIVER_H
#define OPENOCD_FLASH_NAND_DRIVER_H

struct nand_device;

#define __NAND_DEVICE_COMMAND(name) \
	COMMAND_HELPER(name, struct nand_device *nand)

/**
 * Interface for NAND flash controllers.  Not all of these functions are
 * required for full functionality of the NAND driver, but better performance
 * can be achieved by implementing each function.
 */
struct nand_flash_controller {
	/** Driver name that is used to select it from configuration files. */
	const char *name;

	/** Usage of flash command registration. */
	const char *usage;

	const struct command_registration *commands;

	/** NAND device command called when driver is instantiated during configuration. */
	__NAND_DEVICE_COMMAND((*nand_device_command));

	/** Initialize the NAND device. */
	int (*init)(struct nand_device *nand);

	/** Reset the NAND device. */
	int (*reset)(struct nand_device *nand);

	/** Issue a command to the NAND device. */
	int (*command)(struct nand_device *nand, uint8_t command);

	/** Write an address to the NAND device. */
	int (*address)(struct nand_device *nand, uint8_t address);

	/** Write word of data to the NAND device. */
	int (*write_data)(struct nand_device *nand, uint16_t data);

	/** Read word of data from the NAND device. */
	int (*read_data)(struct nand_device *nand, void *data);

	/** Write a block of data to the NAND device. */
	int (*write_block_data)(struct nand_device *nand, uint8_t *data, int size);

	/** Read a block of data from the NAND device. */
	int (*read_block_data)(struct nand_device *nand, uint8_t *data, int size);

	/** Write a page to the NAND device. */
	int (*write_page)(struct nand_device *nand, uint32_t page, uint8_t *data,
			  uint32_t data_size, uint8_t *oob, uint32_t oob_size);

	/** Read a page from the NAND device. */
	int (*read_page)(struct nand_device *nand, uint32_t page, uint8_t *data, uint32_t data_size,
			 uint8_t *oob, uint32_t oob_size);

	/** Check if the NAND device is ready for more instructions with timeout. */
	int (*nand_ready)(struct nand_device *nand, int timeout);
};

#define NAND_DEVICE_COMMAND_HANDLER(name) static __NAND_DEVICE_COMMAND(name)

/**
 * Find a NAND flash controller by name.
 * @param name Identifies the NAND controller to find.
 * @returns The nand_flash_controller named @c name, or NULL if not found.
 */
struct nand_flash_controller *nand_driver_find_by_name(const char *name);

/** Signature for callback functions passed to nand_driver_walk */
typedef int (*nand_driver_walker_t)(struct nand_flash_controller *c, void *);
/**
 * Walk the list of drivers, encapsulating the data structure type.
 * Application state/context can be passed through the @c x pointer.
 * @param f The callback function to invoke for each function.
 * @param x For use as private data storage, passed directly to @c f.
 * @returns ERROR_OK if successful, or the non-zero return value of @c f.
 * This allows a walker to terminate the loop early.
 */
int nand_driver_walk(nand_driver_walker_t f, void *x);

#endif /* OPENOCD_FLASH_NAND_DRIVER_H */
