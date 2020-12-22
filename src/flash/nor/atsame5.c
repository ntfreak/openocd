/***************************************************************************
 *   Copyright (C) 2017 by Tomas Vanek					   *
 *   vanekt@fbl.cz							   *
 *                                                                         *
 *   Based on at91samd.c                                                   *
 *   Copyright (C) 2013 by Andrey Yurovsky                                 *
 *   Andrey Yurovsky <yurovsky@gmail.com>                                  *
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
#include "helper/binarybuffer.h"

#include <helper/time_support.h>
#include <target/cortex_m.h>

/* A note to prefixing.
 * Definitions and functions inherited from at91samd.c without
 * any change retained the original prefix samd_ so they eventually
 * may go to samd_common.h and .c
 * As currently there are only 3 short functions identical with
 * the original source, no common file was created. */

#define SAME5_PAGES_PER_BLOCK	16
#define SAME5_NUM_PROT_BLOCKS	32
#define SAMD_PAGE_SIZE_MAX	1024

#define SAMD_FLASH		0x00000000	/* physical Flash memory */
#define SAMD_USER_ROW		0x00804000	/* User Row of Flash */

#define SAME5_PAC		0x40000000	/* Peripheral Access Control */

#define SAMD_DSU		0x41002000	/* Device Service Unit */
#define SAMD_NVMCTRL		0x41004000	/* Non-volatile memory controller */

#define SAMD_DSU_STATUSA        1               /* DSU status register */
#define SAMD_DSU_DID		0x18		/* Device ID register */
#define SAMD_DSU_CTRL_EXT	0x100		/* CTRL register, external access */

#define SAME5_NVMCTRL_CTRLA	0x00		/* NVM control A register */
#define SAME5_NVMCTRL_CTRLB	0x04		/* NVM control B register */
#define SAMD_NVMCTRL_PARAM	0x08		/* NVM parameters register */
#define SAME5_NVMCTRL_INTFLAG	0x10		/* NVM interrupt flag register */
#define SAME5_NVMCTRL_STATUS	0x12		/* NVM status register */
#define SAME5_NVMCTRL_ADDR	0x14		/* NVM address register */
#define SAME5_NVMCTRL_LOCK	0x18		/* NVM Lock section register */

#define SAMD_CMDEX_KEY		0xA5UL
#define SAMD_NVM_CMD(n)		((SAMD_CMDEX_KEY << 8) | (n & 0x7F))

/* NVMCTRL commands. */
#define SAME5_NVM_CMD_EP	0x00		/* Erase Page (User Page only) */
#define SAME5_NVM_CMD_EB	0x01		/* Erase Block */
#define SAME5_NVM_CMD_WP	0x03		/* Write Page */
#define SAME5_NVM_CMD_WQW	0x04		/* Write Quad Word */
#define SAME5_NVM_CMD_LR	0x11		/* Lock Region */
#define SAME5_NVM_CMD_UR	0x12		/* Unlock Region */
#define SAME5_NVM_CMD_PBC	0x15		/* Page Buffer Clear */
#define SAME5_NVM_CMD_SSB	0x16		/* Set Security Bit */

/* NVMCTRL bits */
#define SAME5_NVMCTRL_CTRLA_WMODE_MASK	0x30

#define SAME5_NVMCTRL_INTFLAG_DONE	(1 << 0)
#define SAME5_NVMCTRL_INTFLAG_ADDRE	(1 << 1)
#define SAME5_NVMCTRL_INTFLAG_PROGE	(1 << 2)
#define SAME5_NVMCTRL_INTFLAG_LOCKE	(1 << 3)
#define SAME5_NVMCTRL_INTFLAG_ECCSE	(1 << 4)
#define SAME5_NVMCTRL_INTFLAG_ECCDE	(1 << 5)
#define SAME5_NVMCTRL_INTFLAG_NVME	(1 << 6)


/* Known identifiers */
#define SAMD_PROCESSOR_M0	0x01
#define SAMD_PROCESSOR_M4	0x06
#define SAMD_FAMILY_D		0x00
#define SAMD_FAMILY_E		0x03
#define SAMD_SERIES_51		0x06
#define SAME_SERIES_51		0x01
#define SAME_SERIES_53		0x03
#define SAME_SERIES_54		0x04

/* Device ID macros */
#define SAMD_GET_PROCESSOR(id) (id >> 28)
#define SAMD_GET_FAMILY(id) (((id >> 23) & 0x1F))
#define SAMD_GET_SERIES(id) (((id >> 16) & 0x3F))
#define SAMD_GET_DEVSEL(id) (id & 0xFF)

/* Bits to mask user row */
#define NVMUSERROW_SAM_E5_D5_MASK	((uint64_t)0x7FFF00FF3C007FFF)

struct samd_part {
	uint8_t id;
	const char *name;
	uint32_t flash_kb;
	uint32_t ram_kb;
};

/* See SAM D5x/E5x Family Silicon Errata and Data Sheet Clarification
 * DS80000748K */
/* Known SAMD51 parts. */
static const struct samd_part samd51_parts[] = {
	{ 0x00, "SAMD51P20A", 1024, 256 },
	{ 0x01, "SAMD51P19A", 512, 192 },
	{ 0x02, "SAMD51N20A", 1024, 256 },
	{ 0x03, "SAMD51N19A", 512, 192 },
	{ 0x04, "SAMD51J20A", 1024, 256 },
	{ 0x05, "SAMD51J19A", 512, 192 },
	{ 0x06, "SAMD51J18A", 256, 128 },
	{ 0x07, "SAMD51G19A", 512, 192 },
	{ 0x08, "SAMD51G18A", 256, 128 },
};

/* Known SAME51 parts. */
static const struct samd_part same51_parts[] = {
	{ 0x00, "SAME51N20A", 1024, 256 },
	{ 0x01, "SAME51N19A", 512, 192 },
	{ 0x02, "SAME51J19A", 512, 192 },
	{ 0x03, "SAME51J18A", 256, 128 },
	{ 0x04, "SAME51J20A", 1024, 256 },
	{ 0x05, "SAME51G19A", 512, 192 },	/* New in rev D */
	{ 0x06, "SAME51G18A", 256, 128 },	/* New in rev D */
};

/* Known SAME53 parts. */
static const struct samd_part same53_parts[] = {
	{ 0x02, "SAME53N20A", 1024, 256 },
	{ 0x03, "SAME53N19A", 512, 192 },
	{ 0x04, "SAME53J20A", 1024, 256 },
	{ 0x05, "SAME53J19A", 512, 192 },
	{ 0x06, "SAME53J18A", 256, 128 },
};

/* Known SAME54 parts. */
static const struct samd_part same54_parts[] = {
	{ 0x00, "SAME54P20A", 1024, 256 },
	{ 0x01, "SAME54P19A", 512, 192 },
	{ 0x02, "SAME54N20A", 1024, 256 },
	{ 0x03, "SAME54N19A", 512, 192 },
};

/* Each family of parts contains a parts table in the DEVSEL field of DID.  The
 * processor ID, family ID, and series ID are used to determine which exact
 * family this is and then we can use the corresponding table. */
struct samd_family {
	uint8_t processor;
	uint8_t family;
	uint8_t series;
	const struct samd_part *parts;
	size_t num_parts;
};

/* Known SAMD families */
static const struct samd_family samd_families[] = {
	{ SAMD_PROCESSOR_M4, SAMD_FAMILY_D, SAMD_SERIES_51,
		samd51_parts, ARRAY_SIZE(samd51_parts) },
	{ SAMD_PROCESSOR_M4, SAMD_FAMILY_E, SAME_SERIES_51,
		same51_parts, ARRAY_SIZE(same51_parts) },
	{ SAMD_PROCESSOR_M4, SAMD_FAMILY_E, SAME_SERIES_53,
		same53_parts, ARRAY_SIZE(same53_parts) },
	{ SAMD_PROCESSOR_M4, SAMD_FAMILY_E, SAME_SERIES_54,
		same54_parts, ARRAY_SIZE(same54_parts) },
};

struct samd_info {
	const struct samd_params *par;
	uint32_t page_size;
	int num_pages;
	int sector_size;
	int prot_block_size;

	bool probed;
	struct target *target;
};


/**
 * Gives the family structure to specific device id.
 * @param id The id of the device.
 * @return On failure NULL, otherwise a pointer to the structure.
 */
static const struct samd_family *samd_find_family(uint32_t id)
{
	uint8_t processor = SAMD_GET_PROCESSOR(id);
	uint8_t family = SAMD_GET_FAMILY(id);
	uint8_t series = SAMD_GET_SERIES(id);

	for (unsigned i = 0; i < ARRAY_SIZE(samd_families); i++) {
		if (samd_families[i].processor == processor &&
			samd_families[i].series == series &&
			samd_families[i].family == family)
			return &samd_families[i];
	}

	return NULL;
}

/**
 * Gives the part structure to specific device id.
 * @param id The id of the device.
 * @return On failure NULL, otherwise a pointer to the structure.
 */
static const struct samd_part *samd_find_part(uint32_t id)
{
	uint8_t devsel = SAMD_GET_DEVSEL(id);
	const struct samd_family *family = samd_find_family(id);
	if (family == NULL)
		return NULL;

	for (unsigned i = 0; i < family->num_parts; i++) {
		if (family->parts[i].id == devsel)
			return &family->parts[i];
	}

	return NULL;
}

static int same5_protect_check(struct flash_bank *bank)
{
	int res;
	uint32_t lock;

	res = target_read_u32(bank->target,
			SAMD_NVMCTRL + SAME5_NVMCTRL_LOCK, &lock);
	if (res != ERROR_OK)
		return res;

	/* Lock bits are active-low */
	for (unsigned int prot_block = 0; prot_block < bank->num_prot_blocks; prot_block++)
		bank->prot_blocks[prot_block].is_protected = !(lock & (1u<<prot_block));

	return ERROR_OK;
}

static int samd_get_flash_page_info(struct target *target,
		uint32_t *sizep, int *nump)
{
	int res;
	uint32_t param;

	res = target_read_u32(target, SAMD_NVMCTRL + SAMD_NVMCTRL_PARAM, &param);
	if (res == ERROR_OK) {
		/* The PSZ field (bits 18:16) indicate the page size bytes as 2^(3+n)
		 * so 0 is 8KB and 7 is 1024KB. */
		if (sizep)
			*sizep = (8 << ((param >> 16) & 0x7));
		/* The NVMP field (bits 15:0) indicates the total number of pages */
		if (nump)
			*nump = param & 0xFFFF;
	} else {
		LOG_ERROR("Couldn't read NVM Parameters register");
	}

	return res;
}

static int same5_probe(struct flash_bank *bank)
{
	uint32_t id;
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	const struct samd_part *part;

	if (chip->probed)
		return ERROR_OK;

	res = target_read_u32(bank->target, SAMD_DSU + SAMD_DSU_DID, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID register");
		return res;
	}

	part = samd_find_part(id);
	if (part == NULL) {
		LOG_ERROR("Couldn't find part corresponding to DID %08" PRIx32, id);
		return ERROR_FAIL;
	}

	bank->size = part->flash_kb * 1024;

	res = samd_get_flash_page_info(bank->target, &chip->page_size,
			&chip->num_pages);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Sanity check: the total flash size in the DSU should match the page size
	 * multiplied by the number of pages. */
	if (bank->size != chip->num_pages * chip->page_size) {
		LOG_WARNING("SAM: bank size doesn't match NVM parameters. "
				"Identified %" PRIu32 "KB Flash but NVMCTRL reports %u %" PRIu32 "B pages",
				part->flash_kb, chip->num_pages, chip->page_size);
	}

	/* Erase granularity = 1 block = 16 pages */
	chip->sector_size = chip->page_size * SAME5_PAGES_PER_BLOCK;

	/* Allocate the sector table */
	bank->num_sectors = chip->num_pages / SAME5_PAGES_PER_BLOCK;
	bank->sectors = alloc_block_array(0, chip->sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* 16 protection blocks per device */
	chip->prot_block_size = bank->size / SAME5_NUM_PROT_BLOCKS;

	/* Allocate the table of protection blocks */
	bank->num_prot_blocks = SAME5_NUM_PROT_BLOCKS;
	bank->prot_blocks = alloc_block_array(0, chip->prot_block_size, bank->num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	same5_protect_check(bank);

	/* Done */
	chip->probed = true;

	LOG_INFO("SAM MCU: %s (%" PRIu32 "KB Flash, %" PRIu32 "KB RAM)", part->name,
			part->flash_kb, part->ram_kb);

	return ERROR_OK;
}

static int same5_wait_and_check_error(struct target *target)
{
	int ret, ret2;
	/* Table 54-40 lists the maximum erase block time as 200 ms.
	 * Include some margin.
	 */
	int timeout_ms = 200 * 5;
	int64_t ts_start = timeval_ms();
	uint16_t intflag;

	do {
		ret = target_read_u16(target,
			SAMD_NVMCTRL + SAME5_NVMCTRL_INTFLAG, &intflag);
		if (ret != ERROR_OK) {
			LOG_ERROR("SAM: error reading the NVMCTRL_INTFLAG register");
			return ret;
		}
		if (intflag & SAME5_NVMCTRL_INTFLAG_DONE)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < timeout_ms);

	if (!(intflag & SAME5_NVMCTRL_INTFLAG_DONE)) {
		LOG_ERROR("SAM: NVM programming timed out");
		ret = ERROR_FLASH_OPERATION_FAILED;
	}
#if 0
	if (intflag & SAME5_NVMCTRL_INTFLAG_ECCSE)
		LOG_ERROR("SAM: ECC Single Error");

	if (intflag & SAME5_NVMCTRL_INTFLAG_ECCDE) {
		LOG_ERROR("SAM: ECC Double Error");
		ret = ERROR_FLASH_OPERATION_FAILED;
	}
#endif
	if (intflag & SAME5_NVMCTRL_INTFLAG_ADDRE) {
		LOG_ERROR("SAM: Addr Error");
		ret = ERROR_FLASH_OPERATION_FAILED;
	}

	if (intflag & SAME5_NVMCTRL_INTFLAG_NVME) {
		LOG_ERROR("SAM: NVM Error");
		ret = ERROR_FLASH_OPERATION_FAILED;
	}

	if (intflag & SAME5_NVMCTRL_INTFLAG_LOCKE) {
		LOG_ERROR("SAM: NVM lock error");
		ret = ERROR_FLASH_PROTECTED;
	}

	if (intflag & SAME5_NVMCTRL_INTFLAG_PROGE) {
		LOG_ERROR("SAM: NVM programming error");
		ret = ERROR_FLASH_OPER_UNSUPPORTED;
	}

	/* Clear the error conditions by writing a one to them */
	ret2 = target_write_u16(target,
			SAMD_NVMCTRL + SAME5_NVMCTRL_INTFLAG, intflag);
	if (ret2 != ERROR_OK)
		LOG_ERROR("Can't clear NVM error conditions");

	return ret;
}

static int same5_issue_nvmctrl_command(struct target *target, uint16_t cmd)
{
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Issue the NVM command */
	/* 32-bit write is used to ensure atomic operation on ST-Link */
	res = target_write_u32(target,
			SAMD_NVMCTRL + SAME5_NVMCTRL_CTRLB, SAMD_NVM_CMD(cmd));
	if (res != ERROR_OK)
		return res;

	/* Check to see if the NVM command resulted in an error condition. */
	return same5_wait_and_check_error(target);
}

/**
 * Erases a flash block or page at the given address.
 * @param target Pointer to the target structure.
 * @param address The address of the row.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int same5_erase_block(struct target *target, uint32_t address)
{
	int res;

	/* Set an address contained in the block to be erased */
	res = target_write_u32(target,
			SAMD_NVMCTRL + SAME5_NVMCTRL_ADDR, address);

	/* Issue the Erase Block command. */
	if (res == ERROR_OK)
		res = same5_issue_nvmctrl_command(target,
				address == SAMD_USER_ROW ? SAME5_NVM_CMD_EP : SAME5_NVM_CMD_EB);

	if (res != ERROR_OK)  {
		LOG_ERROR("Failed to erase block containing %08" PRIx32, address);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}


static int same5_pre_write_check(struct target *target)
{
	int res;
	uint32_t nvm_ctrla;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Check if manual write mode is set */
	res = target_read_u32(target, SAMD_NVMCTRL + SAME5_NVMCTRL_CTRLA, &nvm_ctrla);
	if (res != ERROR_OK)
		return res;

	if (nvm_ctrla & SAME5_NVMCTRL_CTRLA_WMODE_MASK) {
		LOG_ERROR("The flash controller must be in manual write mode. Issue 'reset init' and retry.");
		return ERROR_FAIL;
	}

	return res;
}


/**
 * Modify the contents of the User Row in Flash. The User Row itself
 * has a size of one page and contains a combination of "fuses" and
 * calibration data. Bits which have a value of zero in the mask will
 * not be changed.
 * @param target Pointer to the target structure.
 * @param data Pointer to the value to write.
 * @param mask Pointer to bitmask, 0 -> value stays untouched.
 * @param offset Offset in user row where new data will be applied.
 * @param count Size of buffer and mask in bytes.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int same5_modify_user_row_masked(struct target *target,
		const uint8_t *data, const uint8_t *mask,
		uint32_t offset, uint32_t count)
{
	int res;

	/* Retrieve the MCU's flash page size, in bytes. */
	uint32_t page_size;
	res = samd_get_flash_page_info(target, &page_size, NULL);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Make sure the size is sane. */
	assert(page_size <= SAMD_PAGE_SIZE_MAX &&
		page_size >= offset + count);

	uint8_t buf[SAMD_PAGE_SIZE_MAX];
	/* Read the user row (comprising one page) by words. */
	res = target_read_memory(target, SAMD_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		return res;

	/* Modify buffer and check if really changed */
	bool changed = false;
	uint32_t i;
	for (i = 0; i < count; i++) {
		uint8_t old_b = buf[offset+i];
		uint8_t new_b = (old_b & ~mask[i]) | (data[i] & mask[i]);
		buf[offset+i] = new_b;
		if (old_b != new_b)
			changed = true;
	}

	if (!changed)
		return ERROR_OK;

	res = same5_pre_write_check(target);
	if (res != ERROR_OK)
		return res;

	res = same5_erase_block(target, SAMD_USER_ROW);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't erase user row");
		return res;
	}

	/* Write the page buffer back out to the target using Write Quad Word */
	for (i = 0; i < page_size; i += 4 * 4) {
		res = target_write_memory(target, SAMD_USER_ROW + i, 4, 4, buf + i);
		if (res != ERROR_OK)
			return res;

		/* Trigger flash write */
		res = same5_issue_nvmctrl_command(target, SAME5_NVM_CMD_WQW);
		if (res != ERROR_OK)
			return res;
	}

	return res;
}

/**
 * Modifies the user row register to the given value.
 * @param target Pointer to the target structure.
 * @param value The value to write.
 * @param startb The bit-offset by which the given value is shifted.
 * @param endb The bit-offset of the last bit in value to write.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int same5_modify_user_row(struct target *target, uint32_t value,
		uint8_t startb, uint8_t endb)
{
	uint8_t buf_val[8] = { 0 };
	uint8_t buf_mask[8] = { 0 };

	assert(startb <= endb && endb < 64);
	buf_set_u32(buf_val, startb, endb + 1 - startb, value);
	buf_set_u32(buf_mask, startb, endb + 1 - startb, 0xffffffff);

	return same5_modify_user_row_masked(target,
			buf_val, buf_mask, 0, 8);
}

static int same5_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	int res = ERROR_OK;

	/* We can issue lock/unlock region commands with the target running but
	 * the settings won't persist unless we're able to modify the LOCK regions
	 * and that requires the target to be halted. */
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (unsigned int prot_block = first; prot_block <= last; prot_block++) {
		if (set != bank->prot_blocks[prot_block].is_protected) {
			/* Load an address that is within this protection block (we use offset 0) */
			res = target_write_u32(bank->target,
					SAMD_NVMCTRL + SAME5_NVMCTRL_ADDR,
					bank->prot_blocks[prot_block].offset);
			if (res != ERROR_OK)
				goto exit;

			/* Tell the controller to lock that block */
			res = same5_issue_nvmctrl_command(bank->target,
					set ? SAME5_NVM_CMD_LR : SAME5_NVM_CMD_UR);
			if (res != ERROR_OK)
				goto exit;
		}
	}

	/* We've now applied our changes, however they will be undone by the next
	 * reset unless we also apply them to the LOCK bits in the User Page.
	 * A '1' means unlocked and a '0' means locked. */
	const uint8_t lock[4] = { 0, 0, 0, 0 };
	const uint8_t unlock[4] = { 0xff, 0xff, 0xff, 0xff };
	uint8_t mask[4] = { 0, 0, 0, 0 };

	buf_set_u32(mask, first, last + 1 - first, 0xffffffff);

	res = same5_modify_user_row_masked(bank->target,
			set ? lock : unlock, mask, 8, 4);
	if (res != ERROR_OK)
		LOG_WARNING("SAM: protect settings were not made persistent!");

	res = ERROR_OK;

exit:
	same5_protect_check(bank);

	return res;
}

static int same5_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* For each sector to be erased */
	for (unsigned int s = first; s <= last; s++) {
		res = same5_erase_block(bank->target, bank->sectors[s].offset);
		if (res != ERROR_OK) {
			LOG_ERROR("SAM: failed to erase sector %d at 0x%08" PRIx32, s, bank->sectors[s].offset);
			return res;
		}
	}

	return ERROR_OK;
}


static int same5_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int res;
	uint32_t address;
	uint32_t pg_offset;
	uint32_t nb;
	uint32_t nw;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	uint8_t *pb = NULL;

	res = same5_pre_write_check(bank->target);
	if (res != ERROR_OK)
		return res;

	if (!chip->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	res = same5_issue_nvmctrl_command(bank->target, SAME5_NVM_CMD_PBC);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: %d", __func__, __LINE__);
		return res;
	}

	while (count) {
		nb = chip->page_size - offset % chip->page_size;
		if (count < nb)
			nb = count;

		address = bank->base + offset;
		pg_offset = offset % chip->page_size;

		if (offset % 4 || (offset + nb) % 4) {
			/* Either start or end of write is not word aligned */
			if (!pb) {
				pb = malloc(chip->page_size);
				if (!pb)
					return ERROR_FAIL;
			}

			/* Set temporary page buffer to 0xff and overwrite the relevant part */
			memset(pb, 0xff, chip->page_size);
			memcpy(pb + pg_offset, buffer, nb);

			/* Align start address to a word boundary */
			address -= offset % 4;
			pg_offset -= offset % 4;
			assert(pg_offset % 4 == 0);

			/* Extend length to whole words */
			nw = (nb + offset % 4 + 3) / 4;
			assert(pg_offset + 4 * nw <= chip->page_size);

			/* Now we have original data extended by 0xff bytes
			 * to the nearest word boundary on both start and end */
			res = target_write_memory(bank->target, address, 4, nw, pb + pg_offset);
		} else {
			assert(nb % 4 == 0);
			nw = nb / 4;
			assert(pg_offset + 4 * nw <= chip->page_size);

			/* Word aligned data, use direct write from buffer */
			res = target_write_memory(bank->target, address, 4, nw, buffer);
		}
		if (res != ERROR_OK) {
			LOG_ERROR("%s: %d", __func__, __LINE__);
			goto free_pb;
		}

		res = same5_issue_nvmctrl_command(bank->target, SAME5_NVM_CMD_WP);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: write failed at address 0x%08" PRIx32, __func__, address);
			goto free_pb;
		}

		/* We're done with the page contents */
		count -= nb;
		offset += nb;
		buffer += nb;
	}

free_pb:
	free(pb);
	return res;
}


FLASH_BANK_COMMAND_HANDLER(same5_flash_bank_command)
{
	if (bank->base != SAMD_FLASH) {
		LOG_ERROR("Address " TARGET_ADDR_FMT " invalid bank address (try "
			"0x%08x[same5] )", bank->base, SAMD_FLASH);
		return ERROR_FAIL;
	}

	struct samd_info *chip;
	chip = calloc(1, sizeof(*chip));
	if (!chip) {
		LOG_ERROR("No memory for flash bank chip info");
		return ERROR_FAIL;
	}

	chip->target = bank->target;
	chip->probed = false;

	bank->driver_priv = chip;

	return ERROR_OK;
}


COMMAND_HANDLER(same5_handle_chip_erase_command)
{
	struct target *target = get_current_target(CMD_CTX);
	if (!target)
		return ERROR_FAIL;

	/* Enable access to the DSU by disabling the write protect bit */
	target_write_u32(target, SAME5_PAC, (1<<16) | (1<<5) | (1<<1));
	/* intentionally without error checking - not accessible on secured chip */

	/* Tell the DSU to perform a full chip erase.  It takes about 240ms to
	 * perform the erase. */
	int res = target_write_u8(target, SAMD_DSU + SAMD_DSU_CTRL_EXT, (1<<4));
	if (res == ERROR_OK)
		command_print(CMD, "chip erase started");
	else
		command_print(CMD, "write to DSU CTRL failed");

	return res;
}


COMMAND_HANDLER(same5_handle_userpage_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);
	if (!target)
		return ERROR_FAIL;

	if (CMD_ARGC > 2) {
		command_print(CMD, "Too much Arguments given.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC >= 1) {
		uint64_t mask = NVMUSERROW_SAM_E5_D5_MASK;
		uint64_t value = strtoull(CMD_ARGV[0], NULL, 0);

		if (CMD_ARGC == 2) {
			uint64_t mask_temp = strtoull(CMD_ARGV[1], NULL, 0);
			mask &= mask_temp;
		}

		uint8_t val_buf[8], mask_buf[8];
		target_buffer_set_u64(target, val_buf, value);
		target_buffer_set_u64(target, mask_buf, mask);

		res = same5_modify_user_row_masked(target,
				val_buf, mask_buf, 0, sizeof(val_buf));
	}

	uint8_t buffer[8];
	int res2 = target_read_memory(target, SAMD_USER_ROW, 4, 2, buffer);
	if (res2 == ERROR_OK) {
		uint64_t value = target_buffer_get_u64(target, buffer);
		command_print(CMD, "USER PAGE: 0x%016"PRIX64, value);
	} else {
		LOG_ERROR("USER PAGE could not be read.");
	}

	if (CMD_ARGC >= 1)
		return res;
	else
		return res2;
}


COMMAND_HANDLER(same5_handle_bootloader_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);
	if (!target)
		return ERROR_FAIL;

	if (CMD_ARGC >= 1) {
		unsigned long size = strtoul(CMD_ARGV[0], NULL, 0);
		uint32_t code = (size + 8191) / 8192;
		if (code > 15) {
			command_print(CMD, "Invalid bootloader size.  Please "
						"see datasheet for a list valid sizes.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		res = same5_modify_user_row(target, 15 - code, 26, 29);
	}

	uint32_t val;
	int res2 = target_read_u32(target, SAMD_USER_ROW, &val);
	if (res2 == ERROR_OK) {
		uint32_t code = (val >> 26) & 0xf; /* grab size code */
		uint32_t size = (15 - code) * 8192;
		command_print(CMD, "Bootloader protected in the first %"
				      PRIu32 " bytes", size);
	}

	if (CMD_ARGC >= 1)
		return res;
	else
		return res2;
}


COMMAND_HANDLER(samd_handle_reset_deassert)
{
	struct target *target = get_current_target(CMD_CTX);
	int res = ERROR_OK;
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (!target)
		return ERROR_FAIL;

	/* If the target has been unresponsive before, try to re-establish
	 * communication now - CPU is held in reset by DSU, DAP is working */
	if (!target_was_examined(target))
		target_examine_one(target);
	target_poll(target);

	/* In case of sysresetreq, debug retains state set in cortex_m_assert_reset()
	 * so we just release reset held by DSU
	 *
	 * n_RESET (srst) clears the DP, so reenable debug and set vector catch here
	 *
	 * After vectreset DSU release is not needed however makes no harm
	 */
	if (target->reset_halt && (jtag_reset_config & RESET_HAS_SRST)) {
		res = target_write_u32(target, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
		if (res == ERROR_OK)
			res = target_write_u32(target, DCB_DEMCR,
				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
		/* do not return on error here, releasing DSU reset is more important */
	}

	/* clear CPU Reset Phase Extension bit */
	int res2 = target_write_u8(target, SAMD_DSU + SAMD_DSU_STATUSA, (1<<1));
	if (res2 != ERROR_OK)
		return res2;

	return res;
}

static const struct command_registration same5_exec_command_handlers[] = {
	{
		.name = "dsu_reset_deassert",
		.usage = "",
		.handler = samd_handle_reset_deassert,
		.mode = COMMAND_EXEC,
		.help = "Deassert internal reset held by DSU."
	},
	{
		.name = "chip-erase",
		.usage = "",
		.handler = same5_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase the entire Flash by using the Chip-"
			"Erase feature in the Device Service Unit (DSU).",
	},
	{
		.name = "bootloader",
		.usage = "[size_in_bytes]",
		.handler = same5_handle_bootloader_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the bootloader protection size, stored in the User Row. "
			"Changes are stored immediately but take affect after the MCU is "
			"reset.",
	},
	{
		.name = "userpage",
		.usage = "[value] [mask]",
		.handler = same5_handle_userpage_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the first 64-bit part of user page "
			"located at address 0x804000. Use the optional mask argument "
			"to prevent changes at positions where the bitvalue is zero. "
			"For security reasons the reserved-bits are masked out "
			"in background and therefore cannot be changed.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration same5_command_handlers[] = {
	{
		.name = "atsame5",
		.mode = COMMAND_ANY,
		.help = "atsame5 flash command group",
		.usage = "",
		.chain = same5_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver atsame5_flash = {
	.name = "atsame5",
	.commands = same5_command_handlers,
	.flash_bank_command = same5_flash_bank_command,
	.erase = same5_erase,
	.protect = same5_protect,
	.write = same5_write,
	.read = default_flash_read,
	.probe = same5_probe,
	.auto_probe = same5_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = same5_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
