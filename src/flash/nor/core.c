/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007-2010 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 *   Copyright (C) 2017-2018 Tomas Vanek <vanekt@fbl.cz>                   *
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
#include <config.h>
#endif
#include <flash/common.h>
#include <flash/nor/core.h>
#include <flash/nor/imp.h>
#include <target/image.h>

/**
 * @file
 * Upper level of NOR flash framework.
 * The lower level interfaces are to drivers.  These upper level ones
 * primarily support access from Tcl scripts or from GDB.
 */

static struct flash_bank *flash_banks;

int flash_driver_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval;

	retval = bank->driver->erase(bank, first, last);
	if (retval != ERROR_OK)
		LOG_ERROR("failed erasing sectors %u to %u", first, last);

	return retval;
}

int flash_driver_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	int retval;
	unsigned int num_blocks;

	if (bank->num_prot_blocks)
		num_blocks = bank->num_prot_blocks;
	else
		num_blocks = bank->num_sectors;


	/* callers may not supply illegal parameters ... */
	if (first > last || last >= num_blocks) {
		LOG_ERROR("illegal protection block range");
		return ERROR_FAIL;
	}

	/* force "set" to 0/1 */
	set = !!set;

	if (!bank->driver->protect) {
		LOG_ERROR("Flash protection is not supported.");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	/* DANGER!
	 *
	 * We must not use any cached information about protection state!!!!
	 *
	 * There are a million things that could change the protect state:
	 *
	 * the target could have reset, power cycled, been hot plugged,
	 * the application could have run, etc.
	 *
	 * Drivers only receive valid protection block range.
	 */
	retval = bank->driver->protect(bank, set, first, last);
	if (retval != ERROR_OK)
		LOG_ERROR("failed setting protection for blocks %u to %u", first, last);

	return retval;
}

int flash_driver_write(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	retval = bank->driver->write(bank, buffer, offset, count);
	if (retval != ERROR_OK) {
		LOG_ERROR(
			"error writing to flash at address " TARGET_ADDR_FMT
			" at offset 0x%8.8" PRIx32,
			bank->base,
			offset);
	}

	return retval;
}

int flash_driver_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	LOG_DEBUG("call flash_driver_read()");

	retval = bank->driver->read(bank, buffer, offset, count);
	if (retval != ERROR_OK) {
		LOG_ERROR(
			"error reading to flash at address " TARGET_ADDR_FMT
			" at offset 0x%8.8" PRIx32,
			bank->base,
			offset);
	}

	return retval;
}

int default_flash_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return target_read_buffer(bank->target, offset + bank->base, count, buffer);
}

int flash_driver_verify(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	retval = bank->driver->verify ? bank->driver->verify(bank, buffer, offset, count) :
		default_flash_verify(bank, buffer, offset, count);
	if (retval != ERROR_OK) {
		LOG_ERROR("verify failed in bank at " TARGET_ADDR_FMT " starting at 0x%8.8" PRIx32,
			bank->base, offset);
	}

	return retval;
}

int default_flash_verify(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	uint32_t target_crc, image_crc;
	int retval;

	retval = image_calculate_checksum(buffer, count, &image_crc);
	if (retval != ERROR_OK)
		return retval;

	retval = target_checksum_memory(bank->target, offset + bank->base, count, &target_crc);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("addr " TARGET_ADDR_FMT ", len 0x%08" PRIx32 ", crc 0x%08" PRIx32 " 0x%08" PRIx32,
		offset + bank->base, count, ~image_crc, ~target_crc);
	if (target_crc == image_crc)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

void flash_bank_add(struct flash_bank *bank)
{
	/* put flash bank in linked list */
	unsigned bank_num = 0;
	if (flash_banks) {
		/* find last flash bank */
		struct flash_bank *p = flash_banks;
		while (p->next) {
			bank_num += 1;
			p = p->next;
		}
		p->next = bank;
		bank_num += 1;
	} else
		flash_banks = bank;

	bank->bank_number = bank_num;
}

struct flash_bank *flash_bank_list(void)
{
	return flash_banks;
}

struct flash_bank *get_flash_bank_by_num_noprobe(unsigned int num)
{
	struct flash_bank *p;
	unsigned int i = 0;

	for (p = flash_banks; p; p = p->next) {
		if (i++ == num)
			return p;
	}
	LOG_ERROR("flash bank %d does not exist", num);
	return NULL;
}

unsigned int flash_get_bank_count(void)
{
	struct flash_bank *p;
	unsigned int i = 0;
	for (p = flash_banks; p; p = p->next)
		i++;
	return i;
}

void default_flash_free_driver_priv(struct flash_bank *bank)
{
	free(bank->driver_priv);
	bank->driver_priv = NULL;
}

void flash_free_all_banks(void)
{
	struct flash_bank *bank = flash_banks;
	while (bank) {
		struct flash_bank *next = bank->next;
		if (bank->driver->free_driver_priv)
			bank->driver->free_driver_priv(bank);
		else
			LOG_WARNING("Flash driver of %s does not support free_driver_priv()", bank->name);

		/* For 'virtual' flash driver bank->sectors and bank->prot_blocks pointers are copied from
		 * master flash_bank structure. They point to memory locations allocated by master flash driver
		 * so master driver is responsible for releasing them.
		 * Avoid UB caused by double-free memory corruption if flash bank is 'virtual'. */

		if (strcmp(bank->driver->name, "virtual") != 0) {
			free(bank->sectors);
			free(bank->prot_blocks);
		}

		free(bank->name);
		free(bank);
		bank = next;
	}
	flash_banks = NULL;
}

struct flash_bank *get_flash_bank_by_name_noprobe(const char *name)
{
	unsigned requested = get_flash_name_index(name);
	unsigned found = 0;

	struct flash_bank *bank;
	for (bank = flash_banks; bank; bank = bank->next) {
		if (strcmp(bank->name, name) == 0)
			return bank;
		if (!flash_driver_name_matches(bank->driver->name, name))
			continue;
		if (++found < requested)
			continue;
		return bank;
	}
	return NULL;
}

int get_flash_bank_by_name(const char *name, struct flash_bank **bank_result)
{
	struct flash_bank *bank;
	int retval;

	bank = get_flash_bank_by_name_noprobe(name);
	if (bank) {
		retval = bank->driver->auto_probe(bank);

		if (retval != ERROR_OK) {
			LOG_ERROR("auto_probe failed");
			return retval;
		}
	}

	*bank_result = bank;
	return ERROR_OK;
}

int get_flash_bank_by_num(unsigned int num, struct flash_bank **bank)
{
	struct flash_bank *p = get_flash_bank_by_num_noprobe(num);
	int retval;

	if (!p)
		return ERROR_FAIL;

	retval = p->driver->auto_probe(p);

	if (retval != ERROR_OK) {
		LOG_ERROR("auto_probe failed");
		return retval;
	}
	*bank = p;
	return ERROR_OK;
}

/* lookup flash bank by address, bank not found is success, but
 * result_bank is set to NULL. */
int get_flash_bank_by_addr(struct target *target,
	target_addr_t addr,
	bool check,
	struct flash_bank **result_bank)
{
	struct flash_bank *c;

	/* cycle through bank list */
	for (c = flash_banks; c; c = c->next) {
		if (c->target != target)
			continue;

		int retval;
		retval = c->driver->auto_probe(c);

		if (retval != ERROR_OK) {
			LOG_ERROR("auto_probe failed");
			return retval;
		}
		/* check whether address belongs to this flash bank */
		if ((addr >= c->base) && (addr <= c->base + (c->size - 1))) {
			*result_bank = c;
			return ERROR_OK;
		}
	}
	*result_bank = NULL;
	if (check) {
		LOG_ERROR("No flash at address " TARGET_ADDR_FMT, addr);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int default_flash_mem_blank_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	const int buffer_size = 1024;
	uint32_t n_bytes;
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint8_t *buffer = malloc(buffer_size);

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		uint32_t j;
		bank->sectors[i].is_erased = 1;

		for (j = 0; j < bank->sectors[i].size; j += buffer_size) {
			uint32_t chunk;
			chunk = buffer_size;
			if (chunk > (bank->sectors[i].size - j))
				chunk = (bank->sectors[i].size - j);

			retval = target_read_memory(target,
					bank->base + bank->sectors[i].offset + j,
					4,
					chunk/4,
					buffer);
			if (retval != ERROR_OK)
				goto done;

			for (n_bytes = 0; n_bytes < chunk; n_bytes++) {
				if (buffer[n_bytes] != bank->erased_value) {
					bank->sectors[i].is_erased = 0;
					break;
				}
			}
		}
	}

done:
	free(buffer);

	return retval;
}

int default_flash_blank_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct target_memory_check_block *block_array;
	block_array = malloc(bank->num_sectors * sizeof(struct target_memory_check_block));
	if (!block_array)
		return default_flash_mem_blank_check(bank);

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		block_array[i].address = bank->base + bank->sectors[i].offset;
		block_array[i].size = bank->sectors[i].size;
		block_array[i].result = UINT32_MAX; /* erase state unknown */
	}

	bool fast_check = true;
	for (unsigned int i = 0; i < bank->num_sectors; ) {
		retval = target_blank_check_memory(target,
				block_array + i, bank->num_sectors - i,
				bank->erased_value);
		if (retval < 1) {
			/* Run slow fallback if the first run gives no result
			 * otherwise use possibly incomplete results */
			if (i == 0)
				fast_check = false;
			break;
		}
		i += retval; /* add number of blocks done this round */
	}

	if (fast_check) {
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = block_array[i].result;
		retval = ERROR_OK;
	} else {
		LOG_USER("Running slow fallback erase check - add working memory");
		retval = default_flash_mem_blank_check(bank);
	}
	free(block_array);

	return retval;
}

/* Manipulate given flash region, selecting the bank according to target
 * and address.  Maps an address range to a set of sectors, and issues
 * the callback() on that set ... e.g. to erase or unprotect its members.
 *
 * Parameter iterate_protect_blocks switches iteration of protect block
 * instead of erase sectors. If there is no protect blocks array, sectors
 * are used in iteration, so compatibility for old flash drivers is retained.
 *
 * The "pad_reason" parameter is a kind of boolean:  when it's NULL, the
 * range must fit those sectors exactly.  This is clearly safe; it can't
 * erase data which the caller said to leave alone, for example.  If it's
 * non-NULL, rather than failing, extra data in the first and/or last
 * sectors will be added to the range, and that reason string is used when
 * warning about those additions.
 */
static int flash_iterate_address_range_inner(struct target *target,
	char *pad_reason, target_addr_t addr, uint32_t length,
	bool iterate_protect_blocks,
	int (*callback)(struct flash_bank *bank, unsigned int first,
		unsigned int last))
{
	struct flash_bank *c;
	struct flash_sector *block_array;
	target_addr_t last_addr = addr + length - 1;	/* the last address of range */
	int first = -1;
	int last = -1;
	int i;
	int num_blocks;

	int retval = get_flash_bank_by_addr(target, addr, true, &c);
	if (retval != ERROR_OK)
		return retval;

	if (c->size == 0 || c->num_sectors == 0) {
		LOG_ERROR("Bank is invalid");
		return ERROR_FLASH_BANK_INVALID;
	}

	if (length == 0) {
		/* special case, erase whole bank when length is zero */
		if (addr != c->base) {
			LOG_ERROR("Whole bank access must start at beginning of bank.");
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}

		return callback(c, 0, c->num_sectors - 1);
	}

	/* check whether it all fits in this bank */
	if (last_addr > c->base + c->size - 1) {
		LOG_ERROR("Flash access does not fit into bank.");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (!c->prot_blocks || c->num_prot_blocks == 0) {
		/* flash driver does not define protect blocks, use sectors instead */
		iterate_protect_blocks = false;
	}

	if (iterate_protect_blocks) {
		block_array = c->prot_blocks;
		num_blocks = c->num_prot_blocks;
	} else {
		block_array = c->sectors;
		num_blocks = c->num_sectors;
	}

	for (i = 0; i < num_blocks; i++) {
		struct flash_sector *f = &block_array[i];
		target_addr_t sector_addr = c->base + f->offset;
		target_addr_t sector_last_addr = sector_addr + f->size - 1;

		/* start only on a sector boundary */
		if (first < 0) {
			/* scanned past the first sector? */
			if (addr < sector_addr)
				break;

			/* is this the first sector? */
			if (addr == sector_addr)
				first = i;

			/* Does this need head-padding?  If so, pad and warn;
			 * or else force an error.
			 *
			 * Such padding can make trouble, since *WE* can't
			 * ever know if that data was in use.  The warning
			 * should help users sort out messes later.
			 */
			else if (addr <= sector_last_addr && pad_reason) {
				/* FIXME say how many bytes (e.g. 80 KB) */
				LOG_WARNING("Adding extra %s range, "
					TARGET_ADDR_FMT " .. " TARGET_ADDR_FMT,
					pad_reason,
					sector_addr,
					addr - 1);
				first = i;
			} else
				continue;
		}

		/* is this (also?) the last sector? */
		if (last_addr == sector_last_addr) {
			last = i;
			break;
		}

		/* Does this need tail-padding?  If so, pad and warn;
		 * or else force an error.
		 */
		if (last_addr < sector_last_addr && pad_reason) {
			/* FIXME say how many bytes (e.g. 80 KB) */
			LOG_WARNING("Adding extra %s range, "
				TARGET_ADDR_FMT " .. " TARGET_ADDR_FMT,
				pad_reason,
				last_addr + 1,
				sector_last_addr);
			last = i;
			break;
		}

		/* MUST finish on a sector boundary */
		if (last_addr < sector_addr)
			break;
	}

	/* invalid start or end address? */
	if (first == -1 || last == -1) {
		LOG_ERROR("address range " TARGET_ADDR_FMT " .. " TARGET_ADDR_FMT
			" is not sector-aligned",
			addr,
			last_addr);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* The NOR driver may trim this range down, based on what
	 * sectors are already erased/unprotected.  GDB currently
	 * blocks such optimizations.
	 */
	return callback(c, first, last);
}

/* The inner fn only handles a single bank, we could be spanning
 * multiple chips.
 */
static int flash_iterate_address_range(struct target *target,
	char *pad_reason, target_addr_t addr, uint32_t length,
	bool iterate_protect_blocks,
	int (*callback)(struct flash_bank *bank, unsigned int first,
		unsigned int last))
{
	struct flash_bank *c;
	int retval = ERROR_OK;

	/* Danger! zero-length iterations means entire bank! */
	do {
		retval = get_flash_bank_by_addr(target, addr, true, &c);
		if (retval != ERROR_OK)
			return retval;

		uint32_t cur_length = length;
		/* check whether it all fits in this bank */
		if (addr + length - 1 > c->base + c->size - 1) {
			LOG_DEBUG("iterating over more than one flash bank.");
			cur_length = c->base + c->size - addr;
		}
		retval = flash_iterate_address_range_inner(target,
				pad_reason, addr, cur_length,
				iterate_protect_blocks,
				callback);
		if (retval != ERROR_OK)
			break;

		length -= cur_length;
		addr += cur_length;
	} while (length > 0);

	return retval;
}

int flash_erase_address_range(struct target *target,
	bool pad, target_addr_t addr, uint32_t length)
{
	return flash_iterate_address_range(target, pad ? "erase" : NULL,
		addr, length, false, &flash_driver_erase);
}

static int flash_driver_unprotect(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	return flash_driver_protect(bank, 0, first, last);
}

int flash_unlock_address_range(struct target *target, target_addr_t addr,
		uint32_t length)
{
	/* By default, pad to sector boundaries ... the real issue here
	 * is that our (only) caller *permanently* removes protection,
	 * and doesn't restore it.
	 */
	return flash_iterate_address_range(target, "unprotect",
		addr, length, true, &flash_driver_unprotect);
}

static int compare_section(const void *a, const void *b)
{
	struct imagesection *b1, *b2;
	b1 = *((struct imagesection **)a);
	b2 = *((struct imagesection **)b);

	if (b1->base_address == b2->base_address)
		return 0;
	else if (b1->base_address > b2->base_address)
		return 1;
	else
		return -1;
}

/**
 * Get aligned start address of a flash write region
 */
target_addr_t flash_write_align_start(struct flash_bank *bank, target_addr_t addr)
{
	if (addr < bank->base || addr >= bank->base + bank->size
			|| bank->write_start_alignment <= 1)
		return addr;

	if (bank->write_start_alignment == FLASH_WRITE_ALIGN_SECTOR) {
		uint32_t offset = addr - bank->base;
		uint32_t aligned = 0;
		for (unsigned int sect = 0; sect < bank->num_sectors; sect++) {
			if (bank->sectors[sect].offset > offset)
				break;

			aligned = bank->sectors[sect].offset;
		}
		return bank->base + aligned;
	}

	return addr & ~(bank->write_start_alignment - 1);
}

/**
 * Get aligned end address of a flash write region
 */
target_addr_t flash_write_align_end(struct flash_bank *bank, target_addr_t addr)
{
	if (addr < bank->base || addr >= bank->base + bank->size
			|| bank->write_end_alignment <= 1)
		return addr;

	if (bank->write_end_alignment == FLASH_WRITE_ALIGN_SECTOR) {
		uint32_t offset = addr - bank->base;
		uint32_t aligned = 0;
		for (unsigned int sect = 0; sect < bank->num_sectors; sect++) {
			aligned = bank->sectors[sect].offset + bank->sectors[sect].size - 1;
			if (aligned >= offset)
				break;
		}
		return bank->base + aligned;
	}

	return addr | (bank->write_end_alignment - 1);
}

/**
 * Check if gap between sections is bigger than minimum required to discontinue flash write
 */
static bool flash_write_check_gap(struct flash_bank *bank,
				target_addr_t addr1, target_addr_t addr2)
{
	if (bank->minimal_write_gap == FLASH_WRITE_CONTINUOUS
			|| addr1 < bank->base || addr1 >= bank->base + bank->size
			|| addr2 < bank->base || addr2 >= bank->base + bank->size)
		return false;

	if (bank->minimal_write_gap == FLASH_WRITE_GAP_SECTOR) {
		unsigned int sect;
		uint32_t offset1 = addr1 - bank->base;
		/* find the sector following the one containing addr1 */
		for (sect = 0; sect < bank->num_sectors; sect++) {
			if (bank->sectors[sect].offset > offset1)
				break;
		}
		if (sect >= bank->num_sectors)
			return false;

		uint32_t offset2 = addr2 - bank->base;
		return bank->sectors[sect].offset + bank->sectors[sect].size <= offset2;
	}

	target_addr_t aligned1 = flash_write_align_end(bank, addr1);
	target_addr_t aligned2 = flash_write_align_start(bank, addr2);
	return aligned1 + bank->minimal_write_gap < aligned2;
}


int flash_write_unlock_verify(struct target *target, struct image *image,
	uint32_t *written, bool erase, bool unlock, bool write, bool verify)
{
	int retval = ERROR_OK;

	unsigned int section;
	uint32_t section_offset;
	struct flash_bank *c;
	int *padding;

	section = 0;
	section_offset = 0;

	if (written)
		*written = 0;

	if (erase) {
		/* assume all sectors need erasing - stops any problems
		 * when flash_write is called multiple times */

		flash_set_dirty();
	}

	/* allocate padding array */
	padding = calloc(image->num_sections, sizeof(*padding));

	/* This fn requires all sections to be in ascending order of addresses,
	 * whereas an image can have sections out of order. */
	struct imagesection **sections = malloc(sizeof(struct imagesection *) *
			image->num_sections);

	for (unsigned int i = 0; i < image->num_sections; i++)
		sections[i] = &image->sections[i];

	qsort(sections, image->num_sections, sizeof(struct imagesection *),
		compare_section);

	/* loop until we reach end of the image */
	while (section < image->num_sections) {
		uint32_t buffer_idx;
		uint8_t *buffer;
		unsigned int section_last;
		target_addr_t run_address = sections[section]->base_address + section_offset;
		uint32_t run_size = sections[section]->size - section_offset;
		int pad_bytes = 0;

		if (sections[section]->size ==  0) {
			LOG_WARNING("empty section %d", section);
			section++;
			section_offset = 0;
			continue;
		}

		/* find the corresponding flash bank */
		retval = get_flash_bank_by_addr(target, run_address, false, &c);
		if (retval != ERROR_OK)
			goto done;
		if (!c) {
			LOG_WARNING("no flash bank found for address " TARGET_ADDR_FMT, run_address);
			section++;	/* and skip it */
			section_offset = 0;
			continue;
		}

		/* collect consecutive sections which fall into the same bank */
		section_last = section;
		padding[section] = 0;
		while ((run_address + run_size - 1 < c->base + c->size - 1) &&
				(section_last + 1 < image->num_sections)) {
			/* sections are sorted */
			assert(sections[section_last + 1]->base_address >= c->base);
			if (sections[section_last + 1]->base_address >= (c->base + c->size)) {
				/* Done with this bank */
				break;
			}

			/* if we have multiple sections within our image,
			 * flash programming could fail due to alignment issues
			 * attempt to rebuild a consecutive buffer for the flash loader */
			target_addr_t run_next_addr = run_address + run_size;
			target_addr_t next_section_base = sections[section_last + 1]->base_address;
			if (next_section_base < run_next_addr) {
				LOG_ERROR("Section at " TARGET_ADDR_FMT
					" overlaps section ending at " TARGET_ADDR_FMT,
					next_section_base, run_next_addr);
				LOG_ERROR("Flash write aborted.");
				retval = ERROR_FAIL;
				goto done;
			}

			pad_bytes = next_section_base - run_next_addr;
			if (pad_bytes) {
				if (flash_write_check_gap(c, run_next_addr - 1, next_section_base)) {
					LOG_INFO("Flash write discontinued at " TARGET_ADDR_FMT
						", next section at " TARGET_ADDR_FMT,
						run_next_addr, next_section_base);
					break;
				}
			}
			if (pad_bytes > 0)
				LOG_INFO("Padding image section %d at " TARGET_ADDR_FMT
					" with %d bytes",
					section_last, run_next_addr, pad_bytes);

			padding[section_last] = pad_bytes;
			run_size += pad_bytes;
			run_size += sections[++section_last]->size;
		}

		if (run_address + run_size - 1 > c->base + c->size - 1) {
			/* If we have more than one flash chip back to back, then we limit
			 * the current write operation to the current chip.
			 */
			LOG_DEBUG("Truncate flash run size to the current flash chip.");

			run_size = c->base + c->size - run_address;
			assert(run_size > 0);
		}

		uint32_t padding_at_start = 0;
		if (c->write_start_alignment || c->write_end_alignment) {
			/* align write region according to bank requirements */
			target_addr_t aligned_start = flash_write_align_start(c, run_address);
			padding_at_start = run_address - aligned_start;
			if (padding_at_start > 0) {
				LOG_WARNING("Section start address " TARGET_ADDR_FMT
					" breaks the required alignment of flash bank %s",
					run_address, c->name);
				LOG_WARNING("Padding %" PRIu32 " bytes from " TARGET_ADDR_FMT,
					padding_at_start, aligned_start);

				run_address -= padding_at_start;
				run_size += padding_at_start;
			}

			target_addr_t run_end = run_address + run_size - 1;
			target_addr_t aligned_end = flash_write_align_end(c, run_end);
			pad_bytes = aligned_end - run_end;
			if (pad_bytes > 0) {
				LOG_INFO("Padding image section %d at " TARGET_ADDR_FMT
					" with %d bytes (bank write end alignment)",
					section_last, run_end + 1, pad_bytes);

				padding[section_last] += pad_bytes;
				run_size += pad_bytes;
			}

		} else if (unlock || erase) {
			/* If we're applying any sector automagic, then pad this
			 * (maybe-combined) segment to the end of its last sector.
			 */
			uint32_t offset_start = run_address - c->base;
			uint32_t offset_end = offset_start + run_size;
			uint32_t end = offset_end, delta;

			for (unsigned int sector = 0; sector < c->num_sectors; sector++) {
				end = c->sectors[sector].offset
					+ c->sectors[sector].size;
				if (offset_end <= end)
					break;
			}

			delta = end - offset_end;
			padding[section_last] += delta;
			run_size += delta;
		}

		/* allocate buffer */
		buffer = malloc(run_size);
		if (!buffer) {
			LOG_ERROR("Out of memory for flash bank buffer");
			retval = ERROR_FAIL;
			goto done;
		}

		if (padding_at_start)
			memset(buffer, c->default_padded_value, padding_at_start);

		buffer_idx = padding_at_start;

		/* read sections to the buffer */
		while (buffer_idx < run_size) {
			size_t size_read;

			size_read = run_size - buffer_idx;
			if (size_read > sections[section]->size - section_offset)
				size_read = sections[section]->size - section_offset;

			/* KLUDGE!
			 *
			 * #¤%#"%¤% we have to figure out the section # from the sorted
			 * list of pointers to sections to invoke image_read_section()...
			 */
			intptr_t diff = (intptr_t)sections[section] - (intptr_t)image->sections;
			int t_section_num = diff / sizeof(struct imagesection);

			LOG_DEBUG("image_read_section: section = %d, t_section_num = %d, "
					"section_offset = %"PRIu32", buffer_idx = %"PRIu32", size_read = %zu",
				section, t_section_num, section_offset,
				buffer_idx, size_read);
			retval = image_read_section(image, t_section_num, section_offset,
					size_read, buffer + buffer_idx, &size_read);
			if (retval != ERROR_OK || size_read == 0) {
				free(buffer);
				goto done;
			}

			buffer_idx += size_read;
			section_offset += size_read;

			/* see if we need to pad the section */
			if (padding[section]) {
				memset(buffer + buffer_idx, c->default_padded_value, padding[section]);
				buffer_idx += padding[section];
			}

			if (section_offset >= sections[section]->size) {
				section++;
				section_offset = 0;
			}
		}

		retval = ERROR_OK;

		if (unlock)
			retval = flash_unlock_address_range(target, run_address, run_size);
		if (retval == ERROR_OK) {
			if (erase) {
				/* calculate and erase sectors */
				retval = flash_erase_address_range(target,
						true, run_address, run_size);
			}
		}

		if (retval == ERROR_OK) {
			if (write) {
				/* write flash sectors */
				retval = flash_driver_write(c, buffer, run_address - c->base, run_size);
			}
		}

		if (retval == ERROR_OK) {
			if (verify) {
				/* verify flash sectors */
				retval = flash_driver_verify(c, buffer, run_address - c->base, run_size);
			}
		}

		free(buffer);

		if (retval != ERROR_OK) {
			/* abort operation */
			goto done;
		}

		if (written)
			*written += run_size;	/* add run size to total written counter */
	}

done:
	free(sections);
	free(padding);

	return retval;
}

int flash_write(struct target *target, struct image *image,
	uint32_t *written, bool erase)
{
	return flash_write_unlock_verify(target, image, written, erase, false, true, false);
}

struct flash_sector *alloc_block_array(uint32_t offset, uint32_t size,
		unsigned int num_blocks)
{
	struct flash_sector *array = calloc(num_blocks, sizeof(struct flash_sector));
	if (!array)
		return NULL;

	for (unsigned int i = 0; i < num_blocks; i++) {
		array[i].offset = offset;
		array[i].size = size;
		array[i].is_erased = -1;
		array[i].is_protected = -1;
		offset += size;
	}

	return array;
}
