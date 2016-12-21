/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
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

#ifndef OPENOCD_FLASH_NOR_DRIVER_H
#define OPENOCD_FLASH_NOR_DRIVER_H

struct flash_bank;

#define __FLASH_BANK_COMMAND(name) \
		COMMAND_HELPER(name, struct flash_bank *bank)

/**
 * @brief Provides the implementation-independent structure that defines
 * all of the callbacks required by OpenOCD flash drivers.
 *
 * Driver authors must implement the routines defined here, providing an
 * instance with the fields filled out.  After that, the instance must
 * be registered in flash.c, so it can be used by the driver lookup system.
 *
 * Specifically, the user can issue the command: @par
 * @code
 * flash bank DRIVERNAME ...parameters...
 * @endcode
 *
 * OpenOCD will search for the driver with a @c flash_driver_s::name
 * that matches @c DRIVERNAME.
 *
 * The flash subsystem calls some of the other drivers routines a using
 * corresponding static <code>flash_driver_<i>callback</i>()</code>
 * routine in flash.c.
 */
struct flash_driver {
	/**
	 * Gives a human-readable name of this flash driver,
	 * This field is used to select and initialize the driver.
	 */
	const char *name;

	/**
	 * Gives a human-readable description of arguments.
	 */
	const char *usage;

	/**
	 * An array of driver-specific commands to register.  When called
	 * during the "flash bank" command, the driver can register addition
	 * commands to support new flash chip functions.
	 */
	const struct command_registration *commands;

	/**
	 * Finish the "flash bank" command for @a bank.  The
	 * @a bank parameter will have been filled in by the core flash
	 * layer when this routine is called, and the driver can store
	 * additional information in its struct flash_bank::driver_priv field.
	 *
	 * The CMD_ARGV are: @par
	 * @code
	 * CMD_ARGV[0] = bank
	 * CMD_ARGV[1] = drivername {name above}
	 * CMD_ARGV[2] = baseaddress
	 * CMD_ARGV[3] = lengthbytes
	 * CMD_ARGV[4] = chip_width_in bytes
	 * CMD_ARGV[5] = bus_width_in_bytes
	 * CMD_ARGV[6] = driver-specific parameters
	 * @endcode
	 *
	 * For example, CMD_ARGV[4] = 2 (for 16 bit flash),
	 *	CMD_ARGV[5] = 4 (for 32 bit bus).
	 *
	 * If extra arguments are provided (@a CMD_ARGC > 6), they will
	 * start in @a CMD_ARGV[6].  These can be used to implement
	 * driver-specific extensions.
	 *
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	__FLASH_BANK_COMMAND((*flash_bank_command));

	/**
	 * Bank/sector erase routine (target-specific).  When
	 * called, the flash driver should erase the specified sectors
	 * using whatever means are at its disposal.
	 *
	 * @param bank The bank of flash to be erased.
	 * @param first The number of the first sector to erase, typically 0.
	 * @param last The number of the last sector to erase, typically N-1.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*erase)(struct flash_bank *bank, unsigned int first,
		unsigned int last);

	/**
	 * Bank/sector protection routine (target-specific).
	 *
	 * If protection is not implemented, set method to NULL
	 *
	 * When called, the driver should enable/disable protection
	 * for MINIMUM the range covered by first..last sectors
	 * inclusive. Some chips have alignment requirements will
	 * cause the actual range to be protected / unprotected to
	 * be larger than the first..last range.
	 *
	 * @param bank The bank to protect or unprotect.
	 * @param set If non-zero, enable protection; if 0, disable it.
	 * @param first The first sector to (un)protect, typically 0.
	 * @param last The last sector to (un)project, typically N-1.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*protect)(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last);

	/**
	 * Program data into the flash.  Note CPU address will be
	 * "bank->base + offset", while the physical address is
	 * dependent upon current target MMU mappings.
	 *
	 * @param bank The bank to program
	 * @param buffer The data bytes to write.
	 * @param offset The offset into the chip to program.
	 * @param count The number of bytes to write.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*write)(struct flash_bank *bank,
			const uint8_t *buffer, uint32_t offset, uint32_t count);

	/**
	 * Read data from the flash. Note CPU address will be
	 * "bank->base + offset", while the physical address is
	 * dependent upon current target MMU mappings.
	 *
	 * @param bank The bank to read.
	 * @param buffer The data bytes read.
	 * @param offset The offset into the chip to read.
	 * @param count The number of bytes to read.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	 int (*read)(struct flash_bank *bank,
			uint8_t *buffer, uint32_t offset, uint32_t count);

	/**
	 * Verify data in flash.  Note CPU address will be
	 * "bank->base + offset", while the physical address is
	 * dependent upon current target MMU mappings.
	 *
	 * @param bank The bank to verify
	 * @param buffer The data bytes to verify against.
	 * @param offset The offset into the chip to verify.
	 * @param count The number of bytes to verify.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*verify)(struct flash_bank *bank,
			const uint8_t *buffer, uint32_t offset, uint32_t count);

	/**
	 * Probe to determine what kind of flash is present.
	 * This is invoked by the "probe" script command.
	 *
	 * @param bank The bank to probe
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*probe)(struct flash_bank *bank);

	/**
	 * Check the erasure status of a flash bank.
	 * When called, the driver routine must perform the required
	 * checks and then set the @c flash_sector_s::is_erased field
	 * for each of the flash banks's sectors.
	 *
	 * @param bank The bank to check
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*erase_check)(struct flash_bank *bank);

	/**
	 * Determine if the specific bank is "protected" or not.
	 * When called, the driver routine must must perform the
	 * required protection check(s) and then set the @c
	 * flash_sector_s::is_protected field for each of the flash
	 * bank's sectors.
	 *
	 * If protection is not implemented, set method to NULL
	 *
	 * @param bank - the bank to check
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*protect_check)(struct flash_bank *bank);

	/**
	 * Display human-readable information about the flash
	 * bank into the given buffer.  Drivers must be careful to avoid
	 * overflowing the buffer.
	 *
	 * @param bank - the bank to get info about
	 * @param char - where to put the text for the human to read
	 * @param buf_size - the size of the human buffer.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*info)(struct flash_bank *bank, char *buf, int buf_size);

	/**
	 * A more gentle flavor of flash_driver_s::probe, performing
	 * setup with less noise.  Generally, driver routines should test
	 * to see if the bank has already been probed; if it has, the
	 * driver probably should not perform its probe a second time.
	 *
	 * This callback is often called from the inside of other
	 * routines (e.g. GDB flash downloads) to autoprobe the flash as
	 * it is programming the flash.
	 *
	 * @param bank - the bank to probe
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*auto_probe)(struct flash_bank *bank);

	/**
	 * Deallocates private driver structures.
	 * Use default_flash_free_driver_priv() to simply free(bank->driver_priv)
	 *
	 * @param bank - the bank being destroyed
	 */
	void (*free_driver_priv)(struct flash_bank *bank);
};

#define FLASH_BANK_COMMAND_HANDLER(name) \
	static __FLASH_BANK_COMMAND(name)

/**
 * Find a NOR flash driver by its name.
 * @param name The name of the requested driver.
 * @returns The flash_driver called @c name, or NULL if not found.
 */
const struct flash_driver *flash_driver_find_by_name(const char *name);

#endif /* OPENOCD_FLASH_NOR_DRIVER_H */
