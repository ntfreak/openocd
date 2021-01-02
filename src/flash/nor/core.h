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

#ifndef OPENOCD_FLASH_NOR_CORE_H
#define OPENOCD_FLASH_NOR_CORE_H

#include <flash/common.h>

/**
 * @file
 * Upper level NOR flash interfaces.
 */

struct image;

#define FLASH_MAX_ERROR_STR	(128)

/**
 * Describes the geometry and status of a single flash sector
 * within a flash bank.  A single bank typically consists of multiple
 * sectors, each of which can be erased and protected independently.
 */
struct flash_sector {
	/** Bus offset from start of the flash chip (in bytes). */
	uint32_t offset;
	/** Number of bytes in this flash sector. */
	uint32_t size;
	/**
	 * Indication of erasure status: 0 = not erased, 1 = erased,
	 * other = unknown.  Set by @c flash_driver_s::erase_check only.
	 *
	 * This information must be considered stale immediately.
	 * Don't set it in flash_driver_s::erase or a device mass_erase
	 * Don't clear it in flash_driver_s::write
	 * The flag is not used in a protection block
	 */
	int is_erased;
	/**
	 * Indication of protection status: 0 = unprotected/unlocked,
	 * 1 = protected/locked, other = unknown.  Set by
	 * @c flash_driver_s::protect_check.
	 *
	 * This information must be considered stale immediately.
	 * A million things could make it stale: power cycle,
	 * reset of target, code running on target, etc.
	 *
	 * If a flash_bank uses an extra array of protection blocks,
	 * protection flag is not valid in sector array
	 */
	int is_protected;
};

/** Special value for write_start_alignment and write_end_alignment field */
#define FLASH_WRITE_ALIGN_SECTOR	UINT32_MAX

/** Special values for minimal_write_gap field */
#define FLASH_WRITE_CONTINUOUS		0
#define FLASH_WRITE_GAP_SECTOR		UINT32_MAX

/**
 * Provides details of a flash bank, available either on-chip or through
 * a major interface.
 *
 * This structure will be passed as a parameter to the callbacks in the
 * flash_driver_s structure, some of which may modify the contents of
 * this structure of the area of flash that it defines.  Driver writers
 * may use the @c driver_priv member to store additional data on a
 * per-bank basis, if required.
 */
struct flash_bank {
	char *name;

	struct target *target; /**< Target to which this bank belongs. */

	const struct flash_driver *driver; /**< Driver for this bank. */
	void *driver_priv; /**< Private driver storage pointer */

	unsigned int bank_number; /**< The 'bank' (or chip number) of this instance. */
	target_addr_t base; /**< The base address of this bank */
	uint32_t size; /**< The size of this chip bank, in bytes */

	unsigned int chip_width; /**< Width of the chip in bytes (1,2,4 bytes) */
	unsigned int bus_width; /**< Maximum bus width, in bytes (1,2,4 bytes) */

	/** Erased value. Defaults to 0xFF. */
	uint8_t erased_value;

	/** Default padded value used, normally this matches the  flash
	 * erased value. Defaults to 0xFF. */
	uint8_t default_padded_value;

	/** Required alignment of flash write start address.
	 * Default 0, no alignment. Can be any power of two or FLASH_WRITE_ALIGN_SECTOR */
	uint32_t write_start_alignment;
	/** Required alignment of flash write end address.
	 * Default 0, no alignment. Can be any power of two or FLASH_WRITE_ALIGN_SECTOR */
	uint32_t write_end_alignment;
	/** Minimal gap between sections to discontinue flash write
	 * Default FLASH_WRITE_GAP_SECTOR splits the write if one or more untouched
	 * sectors in between.
     * Can be size in bytes or FLASH_WRITE_CONTINUOUS */
	uint32_t minimal_write_gap;

	/**
	 * The number of sectors on this chip.  This value will
	 * be set initially to 0, and the flash driver must set this to
	 * some non-zero value during "probe()" or "auto_probe()".
	 */
	unsigned int num_sectors;
	/** Array of sectors, allocated and initialized by the flash driver */
	struct flash_sector *sectors;

	/**
	 * The number of protection blocks in this bank. This value
	 * is set initially to 0 and sectors are used as protection blocks.
	 * Driver probe can set protection blocks array to work with
	 * protection granularity different than sector size.
	 */
	unsigned int num_prot_blocks;
	/** Array of protection blocks, allocated and initialized by the flash driver */
	struct flash_sector *prot_blocks;

	struct flash_bank *next; /**< The next flash bank on this chip */
};

/** Registers the 'flash' subsystem commands */
int flash_register_commands(struct command_context *cmd_ctx);

/**
 * Erases @a length bytes in the @a target flash, starting at @a addr.
 * The range @a addr to @a addr + @a length - 1 must be strictly
 * sector aligned, unless @a pad is true.  Setting @a pad true extends
 * the range, at beginning and/or end, if needed for sector alignment.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int flash_erase_address_range(struct target *target,
		bool pad, target_addr_t addr, uint32_t length);

int flash_unlock_address_range(struct target *target, target_addr_t addr,
		uint32_t length);

/**
 * Align start address of a flash write region according to bank requirements.
 * @param bank Pointer to bank descriptor structure
 * @param addr Address to align
 * @returns Aligned address
*/
target_addr_t flash_write_align_start(struct flash_bank *bank, target_addr_t addr);
/**
 * Align end address of a flash write region according to bank requirements.
 * Note: Use address of the last byte to write, not the next after the region.
 * @param bank Pointer to bank descriptor structure
 * @param addr Address to align (address of the last byte to write)
 * @returns Aligned address (address of the last byte of padded region)
*/
target_addr_t flash_write_align_end(struct flash_bank *bank, target_addr_t addr);

/**
 * Writes @a image into the @a target flash.  The @a written parameter
 * will contain the
 * @param target The target with the flash to be programmed.
 * @param image The image that will be programmed to flash.
 * @param written On return, contains the number of bytes written.
 * @param erase Indicates whether the flash driver should first
 * erase the corresponding banks or sectors before programming.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int flash_write(struct target *target,
		struct image *image, uint32_t *written, bool erase);

/**
 * Forces targets to re-examine their erase/protection state.
 * This routine must be called when the system may modify the status.
 */
void flash_set_dirty(void);

/** @returns The number of flash banks currently defined. */
unsigned int flash_get_bank_count(void);

/** Deallocates bank->driver_priv */
void default_flash_free_driver_priv(struct flash_bank *bank);

/** Deallocates all flash banks */
void flash_free_all_banks(void);

/**
 * Provides default read implementation for flash memory.
 * @param bank The bank to read.
 * @param buffer The data bytes read.
 * @param offset The offset into the chip to read.
 * @param count The number of bytes to read.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int default_flash_read(struct flash_bank *bank,
		uint8_t *buffer, uint32_t offset, uint32_t count);

/**
 * Provides default verify implementation for flash memory.
 * @param bank The bank to verify.
 * @param buffer The data bytes to verify.
 * @param offset The offset into the chip to verify.
 * @param count The number of bytes to verify.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int default_flash_verify(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count);

/**
 * Provides default erased-bank check handling. Checks to see if
 * the flash driver knows they are erased; if things look uncertain,
 * this routine will call default_flash_mem_blank_check() to confirm.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int default_flash_blank_check(struct flash_bank *bank);
/**
 * Returns the flash bank specified by @a name, which matches the
 * driver name and a suffix (option) specify the driver-specific
 * bank number. The suffix consists of the '.' and the driver-specific
 * bank number: when two str9x banks are defined, then 'str9x.1' refers
 * to the second.
 */
int get_flash_bank_by_name(const char *name, struct flash_bank **bank_result);
/**
 * Returns the flash bank specified by @a name, which matches the
 * driver name and a suffix (option) specify the driver-specific
 * bank number. The suffix consists of the '.' and the driver-specific
 * bank number: when two str9x banks are defined, then 'str9x.1' refers
 * to the second.
 */
struct flash_bank *get_flash_bank_by_name_noprobe(const char *name);
/**
 * Returns the flash bank like get_flash_bank_by_name(), without probing.
 * @param num The flash bank number.
 * @param bank returned bank if fn returns ERROR_OK
 * @returns ERROR_OK if successful
 */
int get_flash_bank_by_num(unsigned int num, struct flash_bank **bank);
/**
 * Retrieves @a bank from a command argument, reporting errors parsing
 * the bank identifier or retrieving the specified bank.  The bank
 * may be identified by its bank number or by @c name.instance, where
 * @a instance is driver-specific.
 * @param name_index The index to the string in args containing the
 * bank identifier.
 * @param bank On output, contains a pointer to the bank or NULL.
 * @returns ERROR_OK on success, or an error indicating the problem.
 */
COMMAND_HELPER(flash_command_get_bank, unsigned name_index,
		struct flash_bank **bank);
/**
 * Returns the flash bank like get_flash_bank_by_num(), without probing.
 * @param num The flash bank number.
 * @returns A struct flash_bank for flash bank @a num, or NULL.
 */
struct flash_bank *get_flash_bank_by_num_noprobe(unsigned int num);
/**
 * Returns the flash bank located at a specified address.
 * @param target The target, presumed to contain one or more banks.
 * @param addr An address that is within the range of the bank.
 * @param check return ERROR_OK and result_bank NULL if the bank does not exist
 * @param result_bank The struct flash_bank located at @a addr, or NULL.
 * @returns ERROR_OK on success, or an error indicating the problem.
 */
int get_flash_bank_by_addr(struct target *target, target_addr_t addr, bool check,
		struct flash_bank **result_bank);
/**
 * Allocate and fill an array of sectors or protection blocks.
 * @param offset Offset of first block.
 * @param size Size of each block.
 * @param num_blocks Number of blocks in array.
 * @returns A struct flash_sector pointer or NULL when allocation failed.
 */
struct flash_sector *alloc_block_array(uint32_t offset, uint32_t size,
		unsigned int num_blocks);

#endif /* OPENOCD_FLASH_NOR_CORE_H */
