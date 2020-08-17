/***************************************************************************
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

#include <target/cortex_m.h>

#define SAMD_NUM_PROT_BLOCKS	16
#define SAMD_PAGE_SIZE_MAX	1024

#define SAMD_FLASH			((uint32_t)0x00000000)	/* physical Flash memory */
#define SAMD_USER_ROW		((uint32_t)0x00804000)	/* User Row of Flash */
#define SAMD_PAC1			0x41000000	/* Peripheral Access Control 1 */
#define SAMD_DSU			0x41002000	/* Device Service Unit */
#define SAMD_NVMCTRL		0x41004000	/* Non-volatile memory controller */

#define SAMD_DSU_STATUSA        1               /* DSU status register */
#define SAMD_DSU_DID		0x18		/* Device ID register */
#define SAMD_DSU_CTRL_EXT	0x100		/* CTRL register, external access */

#define SAMD_NVMCTRL_CTRLA		0x00	/* NVM control A register */
#define SAMD_NVMCTRL_CTRLB		0x04	/* NVM control B register */
#define SAMD_NVMCTRL_PARAM		0x08	/* NVM parameters register */
#define SAMD_NVMCTRL_INTFLAG	0x18	/* NVM Interrupt Flag Status & Clear */
#define SAMD_NVMCTRL_STATUS		0x18	/* NVM status register */
#define SAMD_NVMCTRL_ADDR		0x1C	/* NVM address register */
#define SAMD_NVMCTRL_LOCK		0x20	/* NVM Lock section register */

#define SAMD_CMDEX_KEY		0xA5UL
#define SAMD_NVM_CMD(n)		((SAMD_CMDEX_KEY << 8) | (n & 0x7F))

/* NVMCTRL commands.  See Table 20-4 in 42129F–SAM–10/2013 */
#define SAMD_NVM_CMD_ER		0x02		/* Erase Row */
#define SAMD_NVM_CMD_WP		0x04		/* Write Page */
#define SAMD_NVM_CMD_EAR	0x05		/* Erase Auxiliary Row */
#define SAMD_NVM_CMD_WAP	0x06		/* Write Auxiliary Page */
#define SAMD_NVM_CMD_LR		0x40		/* Lock Region */
#define SAMD_NVM_CMD_UR		0x41		/* Unlock Region */
#define SAMD_NVM_CMD_SPRM	0x42		/* Set Power Reduction Mode */
#define SAMD_NVM_CMD_CPRM	0x43		/* Clear Power Reduction Mode */
#define SAMD_NVM_CMD_PBC	0x44		/* Page Buffer Clear */
#define SAMD_NVM_CMD_SSB	0x45		/* Set Security Bit */
#define SAMD_NVM_CMD_INVALL	0x46		/* Invalidate all caches */

/* NVMCTRL bits */
#define SAMD_NVM_CTRLB_MANW 0x80

/* Known identifiers */
#define SAMD_PROCESSOR_M0	0x01
#define SAMD_FAMILY_D		0x00
#define SAMD_FAMILY_L		0x01
#define SAMD_FAMILY_C		0x02
#define SAMD_SERIES_20		0x00
#define SAMD_SERIES_21		0x01
#define SAMD_SERIES_22		0x02
#define SAMD_SERIES_10		0x02
#define SAMD_SERIES_11		0x03
#define SAMD_SERIES_09		0x04

/* Device ID macros */
#define SAMD_GET_PROCESSOR(id) (id >> 28)
#define SAMD_GET_FAMILY(id) (((id >> 23) & 0x1F))
#define SAMD_GET_SERIES(id) (((id >> 16) & 0x3F))
#define SAMD_GET_DEVSEL(id) (id & 0xFF)

/* Bits to mask out lockbits in user row */
#define NVMUSERROW_LOCKBIT_MASK ((uint64_t)0x0000FFFFFFFFFFFF)

struct samd_part {
	uint8_t id;
	const char *name;
	uint32_t flash_kb;
	uint32_t ram_kb;
};

/* Known SAMD09 parts. DID reset values missing in RM, see
 * https://github.com/avrxml/asf/blob/master/sam0/utils/cmsis/samd09/include/ */
static const struct samd_part samd09_parts[] = {
	{ 0x0, "SAMD09D14A", 16, 4 },
	{ 0x7, "SAMD09C13A", 8, 4 },
};

/* Known SAMD10 parts */
static const struct samd_part samd10_parts[] = {
	{ 0x0, "SAMD10D14AMU", 16, 4 },
	{ 0x1, "SAMD10D13AMU", 8, 4 },
	{ 0x2, "SAMD10D12AMU", 4, 4 },
	{ 0x3, "SAMD10D14ASU", 16, 4 },
	{ 0x4, "SAMD10D13ASU", 8, 4 },
	{ 0x5, "SAMD10D12ASU", 4, 4 },
	{ 0x6, "SAMD10C14A", 16, 4 },
	{ 0x7, "SAMD10C13A", 8, 4 },
	{ 0x8, "SAMD10C12A", 4, 4 },
};

/* Known SAMD11 parts */
static const struct samd_part samd11_parts[] = {
	{ 0x0, "SAMD11D14AM", 16, 4 },
	{ 0x1, "SAMD11D13AMU", 8, 4 },
	{ 0x2, "SAMD11D12AMU", 4, 4 },
	{ 0x3, "SAMD11D14ASS", 16, 4 },
	{ 0x4, "SAMD11D13ASU", 8, 4 },
	{ 0x5, "SAMD11D12ASU", 4, 4 },
	{ 0x6, "SAMD11C14A", 16, 4 },
	{ 0x7, "SAMD11C13A", 8, 4 },
	{ 0x8, "SAMD11C12A", 4, 4 },
	{ 0x9, "SAMD11D14AU", 16, 4 },
};

/* Known SAMD20 parts. See Table 12-8 in 42129F–SAM–10/2013 */
static const struct samd_part samd20_parts[] = {
	{ 0x0, "SAMD20J18A", 256, 32 },
	{ 0x1, "SAMD20J17A", 128, 16 },
	{ 0x2, "SAMD20J16A", 64, 8 },
	{ 0x3, "SAMD20J15A", 32, 4 },
	{ 0x4, "SAMD20J14A", 16, 2 },
	{ 0x5, "SAMD20G18A", 256, 32 },
	{ 0x6, "SAMD20G17A", 128, 16 },
	{ 0x7, "SAMD20G16A", 64, 8 },
	{ 0x8, "SAMD20G15A", 32, 4 },
	{ 0x9, "SAMD20G14A", 16, 2 },
	{ 0xA, "SAMD20E18A", 256, 32 },
	{ 0xB, "SAMD20E17A", 128, 16 },
	{ 0xC, "SAMD20E16A", 64, 8 },
	{ 0xD, "SAMD20E15A", 32, 4 },
	{ 0xE, "SAMD20E14A", 16, 2 },
};

/* Known SAMD21 parts. */
static const struct samd_part samd21_parts[] = {
	{ 0x0, "SAMD21J18A", 256, 32 },
	{ 0x1, "SAMD21J17A", 128, 16 },
	{ 0x2, "SAMD21J16A", 64, 8 },
	{ 0x3, "SAMD21J15A", 32, 4 },
	{ 0x4, "SAMD21J14A", 16, 2 },
	{ 0x5, "SAMD21G18A", 256, 32 },
	{ 0x6, "SAMD21G17A", 128, 16 },
	{ 0x7, "SAMD21G16A", 64, 8 },
	{ 0x8, "SAMD21G15A", 32, 4 },
	{ 0x9, "SAMD21G14A", 16, 2 },
	{ 0xA, "SAMD21E18A", 256, 32 },
	{ 0xB, "SAMD21E17A", 128, 16 },
	{ 0xC, "SAMD21E16A", 64, 8 },
	{ 0xD, "SAMD21E15A", 32, 4 },
	{ 0xE, "SAMD21E14A", 16, 2 },

    /* SAMR21 parts have integrated SAMD21 with a radio */
	{ 0x18, "SAMR21G19A", 256, 32 }, /* with 512k of serial flash */
	{ 0x19, "SAMR21G18A", 256, 32 },
	{ 0x1A, "SAMR21G17A", 128, 32 },
	{ 0x1B, "SAMR21G16A",  64, 16 },
	{ 0x1C, "SAMR21E18A", 256, 32 },
	{ 0x1D, "SAMR21E17A", 128, 32 },
	{ 0x1E, "SAMR21E16A",  64, 16 },

    /* SAMD21 B Variants (Table 3-7 from rev I of datasheet) */
	{ 0x20, "SAMD21J16B", 64, 8 },
	{ 0x21, "SAMD21J15B", 32, 4 },
	{ 0x23, "SAMD21G16B", 64, 8 },
	{ 0x24, "SAMD21G15B", 32, 4 },
	{ 0x26, "SAMD21E16B", 64, 8 },
	{ 0x27, "SAMD21E15B", 32, 4 },

	/* SAMD21 D and L Variants (from Errata)
	   http://ww1.microchip.com/downloads/en/DeviceDoc/
	   SAM-D21-Family-Silicon-Errata-and-DataSheet-Clarification-DS80000760D.pdf */
	{ 0x55, "SAMD21E16BU", 64, 8 },
	{ 0x56, "SAMD21E15BU", 32, 4 },
	{ 0x57, "SAMD21G16L", 64, 8 },
	{ 0x3E, "SAMD21E16L", 64, 8 },
	{ 0x3F, "SAMD21E15L", 32, 4 },
	{ 0x62, "SAMD21E16CU", 64, 8 },
	{ 0x63, "SAMD21E15CU", 32, 4 },
	{ 0x92, "SAMD21J17D", 128, 16 },
	{ 0x93, "SAMD21G17D", 128, 16 },
	{ 0x94, "SAMD21E17D", 128, 16 },
	{ 0x95, "SAMD21E17DU", 128, 16 },
	{ 0x96, "SAMD21G17L", 128, 16 },
	{ 0x97, "SAMD21E17L", 128, 16 },

	/* Known SAMDA1 parts.
	   SAMD-A1 series uses the same series identifier like the SAMD21
	   taken from http://ww1.microchip.com/downloads/en/DeviceDoc/40001895A.pdf (pages 14-17) */
	{ 0x29, "SAMDA1J16A", 64, 8 },
	{ 0x2A, "SAMDA1J15A", 32, 4 },
	{ 0x2B, "SAMDA1J14A", 16, 4 },
	{ 0x2C, "SAMDA1G16A", 64, 8 },
	{ 0x2D, "SAMDA1G15A", 32, 4 },
	{ 0x2E, "SAMDA1G14A", 16, 4 },
	{ 0x2F, "SAMDA1E16A", 64, 8 },
	{ 0x30, "SAMDA1E15A", 32, 4 },
	{ 0x31, "SAMDA1E14A", 16, 4 },
	{ 0x64, "SAMDA1J16B", 64, 8 },
	{ 0x65, "SAMDA1J15B", 32, 4 },
	{ 0x66, "SAMDA1J14B", 16, 4 },
	{ 0x67, "SAMDA1G16B", 64, 8 },
	{ 0x68, "SAMDA1G15B", 32, 4 },
	{ 0x69, "SAMDA1G14B", 16, 4 },
	{ 0x6A, "SAMDA1E16B", 64, 8 },
	{ 0x6B, "SAMDA1E15B", 32, 4 },
	{ 0x6C, "SAMDA1E14B", 16, 4 },
};

/* Known SAML21 parts. */
static const struct samd_part saml21_parts[] = {
	{ 0x00, "SAML21J18A", 256, 32 },
	{ 0x01, "SAML21J17A", 128, 16 },
	{ 0x02, "SAML21J16A", 64, 8 },
	{ 0x05, "SAML21G18A", 256, 32 },
	{ 0x06, "SAML21G17A", 128, 16 },
	{ 0x07, "SAML21G16A", 64, 8 },
	{ 0x0A, "SAML21E18A", 256, 32 },
	{ 0x0B, "SAML21E17A", 128, 16 },
	{ 0x0C, "SAML21E16A", 64, 8 },
	{ 0x0D, "SAML21E15A", 32, 4 },
	{ 0x0F, "SAML21J18B", 256, 32 },
	{ 0x10, "SAML21J17B", 128, 16 },
	{ 0x11, "SAML21J16B", 64, 8 },
	{ 0x14, "SAML21G18B", 256, 32 },
	{ 0x15, "SAML21G17B", 128, 16 },
	{ 0x16, "SAML21G16B", 64, 8 },
	{ 0x19, "SAML21E18B", 256, 32 },
	{ 0x1A, "SAML21E17B", 128, 16 },
	{ 0x1B, "SAML21E16B", 64, 8 },
	{ 0x1C, "SAML21E15B", 32, 4 },

    /* SAMR30 parts have integrated SAML21 with a radio */
	{ 0x1E, "SAMR30G18A", 256, 32 },
	{ 0x1F, "SAMR30E18A", 256, 32 },

    /* SAMR34/R35 parts have integrated SAML21 with a lora radio */
	{ 0x28, "SAMR34J18", 256, 32 },
};

/* Known SAML22 parts. */
static const struct samd_part saml22_parts[] = {
	{ 0x00, "SAML22N18A", 256, 32 },
	{ 0x01, "SAML22N17A", 128, 16 },
	{ 0x02, "SAML22N16A", 64, 8 },
	{ 0x05, "SAML22J18A", 256, 32 },
	{ 0x06, "SAML22J17A", 128, 16 },
	{ 0x07, "SAML22J16A", 64, 8 },
	{ 0x0A, "SAML22G18A", 256, 32 },
	{ 0x0B, "SAML22G17A", 128, 16 },
	{ 0x0C, "SAML22G16A", 64, 8 },
};

/* Known SAMC20 parts. */
static const struct samd_part samc20_parts[] = {
	{ 0x00, "SAMC20J18A", 256, 32 },
	{ 0x01, "SAMC20J17A", 128, 16 },
	{ 0x02, "SAMC20J16A", 64, 8 },
	{ 0x03, "SAMC20J15A", 32, 4 },
	{ 0x05, "SAMC20G18A", 256, 32 },
	{ 0x06, "SAMC20G17A", 128, 16 },
	{ 0x07, "SAMC20G16A", 64, 8 },
	{ 0x08, "SAMC20G15A", 32, 4 },
	{ 0x0A, "SAMC20E18A", 256, 32 },
	{ 0x0B, "SAMC20E17A", 128, 16 },
	{ 0x0C, "SAMC20E16A", 64, 8 },
	{ 0x0D, "SAMC20E15A", 32, 4 },
	{ 0x20, "SAMC20N18A", 256, 32 },
	{ 0x21, "SAMC20N17A", 128, 16 },
};

/* Known SAMC21 parts. */
static const struct samd_part samc21_parts[] = {
	{ 0x00, "SAMC21J18A", 256, 32 },
	{ 0x01, "SAMC21J17A", 128, 16 },
	{ 0x02, "SAMC21J16A", 64, 8 },
	{ 0x03, "SAMC21J15A", 32, 4 },
	{ 0x05, "SAMC21G18A", 256, 32 },
	{ 0x06, "SAMC21G17A", 128, 16 },
	{ 0x07, "SAMC21G16A", 64, 8 },
	{ 0x08, "SAMC21G15A", 32, 4 },
	{ 0x0A, "SAMC21E18A", 256, 32 },
	{ 0x0B, "SAMC21E17A", 128, 16 },
	{ 0x0C, "SAMC21E16A", 64, 8 },
	{ 0x0D, "SAMC21E15A", 32, 4 },
	{ 0x20, "SAMC21N18A", 256, 32 },
	{ 0x21, "SAMC21N17A", 128, 16 },
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
	uint64_t nvm_userrow_res_mask; /* protect bits which are reserved, 0 -> protect */
};

/* Known SAMD families */
static const struct samd_family samd_families[] = {
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_20,
		samd20_parts, ARRAY_SIZE(samd20_parts),
		(uint64_t)0xFFFF01FFFE01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_21,
		samd21_parts, ARRAY_SIZE(samd21_parts),
		(uint64_t)0xFFFF01FFFE01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_09,
		samd09_parts, ARRAY_SIZE(samd09_parts),
		(uint64_t)0xFFFF01FFFE01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_10,
		samd10_parts, ARRAY_SIZE(samd10_parts),
		(uint64_t)0xFFFF01FFFE01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_11,
		samd11_parts, ARRAY_SIZE(samd11_parts),
		(uint64_t)0xFFFF01FFFE01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_L, SAMD_SERIES_21,
		saml21_parts, ARRAY_SIZE(saml21_parts),
		(uint64_t)0xFFFF03FFFC01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_L, SAMD_SERIES_22,
		saml22_parts, ARRAY_SIZE(saml22_parts),
		(uint64_t)0xFFFF03FFFC01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_C, SAMD_SERIES_20,
		samc20_parts, ARRAY_SIZE(samc20_parts),
		(uint64_t)0xFFFF03FFFC01FF77 },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_C, SAMD_SERIES_21,
		samc21_parts, ARRAY_SIZE(samc21_parts),
		(uint64_t)0xFFFF03FFFC01FF77 },
};

struct samd_info {
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

static int samd_protect_check(struct flash_bank *bank)
{
	int res;
	uint16_t lock;

	res = target_read_u16(bank->target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_LOCK, &lock);
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

static int samd_probe(struct flash_bank *bank)
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
		LOG_WARNING("SAMD: bank size doesn't match NVM parameters. "
				"Identified %" PRIu32 "KB Flash but NVMCTRL reports %u %" PRIu32 "B pages",
				part->flash_kb, chip->num_pages, chip->page_size);
	}

	/* Erase granularity = 1 row = 4 pages */
	chip->sector_size = chip->page_size * 4;

	/* Allocate the sector table */
	bank->num_sectors = chip->num_pages / 4;
	bank->sectors = alloc_block_array(0, chip->sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* 16 protection blocks per device */
	chip->prot_block_size = bank->size / SAMD_NUM_PROT_BLOCKS;

	/* Allocate the table of protection blocks */
	bank->num_prot_blocks = SAMD_NUM_PROT_BLOCKS;
	bank->prot_blocks = alloc_block_array(0, chip->prot_block_size, bank->num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	samd_protect_check(bank);

	/* Done */
	chip->probed = true;

	LOG_INFO("SAMD MCU: %s (%" PRIu32 "KB Flash, %" PRIu32 "KB RAM)", part->name,
			part->flash_kb, part->ram_kb);

	return ERROR_OK;
}

static int samd_check_error(struct target *target)
{
	int ret, ret2;
	uint16_t status;

	ret = target_read_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_STATUS, &status);
	if (ret != ERROR_OK) {
		LOG_ERROR("Can't read NVM status");
		return ret;
	}

	if ((status & 0x001C) == 0)
		return ERROR_OK;

	if (status & (1 << 4)) { /* NVME */
		LOG_ERROR("SAMD: NVM Error");
		ret = ERROR_FLASH_OPERATION_FAILED;
	}

	if (status & (1 << 3)) { /* LOCKE */
		LOG_ERROR("SAMD: NVM lock error");
		ret = ERROR_FLASH_PROTECTED;
	}

	if (status & (1 << 2)) { /* PROGE */
		LOG_ERROR("SAMD: NVM programming error");
		ret = ERROR_FLASH_OPER_UNSUPPORTED;
	}

	/* Clear the error conditions by writing a one to them */
	ret2 = target_write_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_STATUS, status);
	if (ret2 != ERROR_OK)
		LOG_ERROR("Can't clear NVM error conditions");

	return ret;
}

static int samd_issue_nvmctrl_command(struct target *target, uint16_t cmd)
{
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Issue the NVM command */
	res = target_write_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLA, SAMD_NVM_CMD(cmd));
	if (res != ERROR_OK)
		return res;

	/* Check to see if the NVM command resulted in an error condition. */
	return samd_check_error(target);
}

/**
 * Erases a flash-row at the given address.
 * @param target Pointer to the target structure.
 * @param address The address of the row.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int samd_erase_row(struct target *target, uint32_t address)
{
	int res;

	/* Set an address contained in the row to be erased */
	res = target_write_u32(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR, address >> 1);

	/* Issue the Erase Row command to erase that row. */
	if (res == ERROR_OK)
		res = samd_issue_nvmctrl_command(target,
				address == SAMD_USER_ROW ? SAMD_NVM_CMD_EAR : SAMD_NVM_CMD_ER);

	if (res != ERROR_OK)  {
		LOG_ERROR("Failed to erase row containing %08" PRIx32, address);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/**
 * Returns the bitmask of reserved bits in register.
 * @param target Pointer to the target structure.
 * @param mask Bitmask, 0 -> value stays untouched.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int samd_get_reservedmask(struct target *target, uint64_t *mask)
{
	int res;
	/* Get the devicetype */
	uint32_t id;
	res = target_read_u32(target, SAMD_DSU + SAMD_DSU_DID, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID register");
		return res;
	}
	const struct samd_family *family;
	family = samd_find_family(id);
	if (family == NULL) {
		LOG_ERROR("Couldn't determine device family");
		return ERROR_FAIL;
	}
	*mask = family->nvm_userrow_res_mask;
	return ERROR_OK;
}

static int read_userrow(struct target *target, uint64_t *userrow)
{
	int res;
	uint8_t buffer[8];

	res = target_read_memory(target, SAMD_USER_ROW, 4, 2, buffer);
	if (res != ERROR_OK)
		return res;

	*userrow = target_buffer_get_u64(target, buffer);
	return ERROR_OK;
}

/**
 * Modify the contents of the User Row in Flash. The User Row itself
 * has a size of one page and contains a combination of "fuses" and
 * calibration data. Bits which have a value of zero in the mask will
 * not be changed. Up to now devices only use the first 64 bits.
 * @param target Pointer to the target structure.
 * @param value_input The value to write.
 * @param value_mask Bitmask, 0 -> value stays untouched.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int samd_modify_user_row_masked(struct target *target,
		uint64_t value_input, uint64_t value_mask)
{
	int res;
	uint32_t nvm_ctrlb;
	bool manual_wp = true;

	/* Retrieve the MCU's page size, in bytes. This is also the size of the
	 * entire User Row. */
	uint32_t page_size;
	res = samd_get_flash_page_info(target, &page_size, NULL);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Make sure the size is sane. */
	assert(page_size <= SAMD_PAGE_SIZE_MAX &&
		page_size >= sizeof(value_input));

	uint8_t buf[SAMD_PAGE_SIZE_MAX];
	/* Read the user row (comprising one page) by words. */
	res = target_read_memory(target, SAMD_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		return res;

	uint64_t value_device;
	res = read_userrow(target, &value_device);
	if (res != ERROR_OK)
		return res;
	uint64_t value_new = (value_input & value_mask) | (value_device & ~value_mask);

	/* We will need to erase before writing if the new value needs a '1' in any
	 * position for which the current value had a '0'.  Otherwise we can avoid
	 * erasing. */
	if ((~value_device) & value_new) {
		res = samd_erase_row(target, SAMD_USER_ROW);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't erase user row");
			return res;
		}
	}

	/* Modify */
	target_buffer_set_u64(target, buf, value_new);

	/* Write the page buffer back out to the target. */
	res = target_write_memory(target, SAMD_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		return res;

	/* Check if we need to do manual page write commands */
	res = target_read_u32(target, SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLB, &nvm_ctrlb);
	if (res == ERROR_OK)
		manual_wp = (nvm_ctrlb & SAMD_NVM_CTRLB_MANW) != 0;
	else {
		LOG_ERROR("Read of NVM register CTRKB failed.");
		return ERROR_FAIL;
	}
	if (manual_wp) {
		/* Trigger flash write */
		res = samd_issue_nvmctrl_command(target, SAMD_NVM_CMD_WAP);
	} else {
		res = samd_check_error(target);
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
static int samd_modify_user_row(struct target *target, uint64_t value,
		uint8_t startb, uint8_t endb)
{
	uint64_t mask = 0;
	int i;
	for (i = startb ; i <= endb ; i++)
		mask |= ((uint64_t)1) << i;

	return samd_modify_user_row_masked(target, value << startb, mask);
}

static int samd_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
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
							SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR,
							bank->prot_blocks[prot_block].offset >> 1);
			if (res != ERROR_OK)
				goto exit;

			/* Tell the controller to lock that block */
			res = samd_issue_nvmctrl_command(bank->target,
					set ? SAMD_NVM_CMD_LR : SAMD_NVM_CMD_UR);
			if (res != ERROR_OK)
				goto exit;
		}
	}

	/* We've now applied our changes, however they will be undone by the next
	 * reset unless we also apply them to the LOCK bits in the User Page.  The
	 * LOCK bits start at bit 48, corresponding to Sector 0 and end with bit 63,
	 * corresponding to Sector 15.  A '1' means unlocked and a '0' means
	 * locked.  See Table 9-3 in the SAMD20 datasheet for more details. */

	res = samd_modify_user_row(bank->target,
			set ? (uint64_t)0 : (uint64_t)UINT64_MAX,
			48 + first, 48 + last);
	if (res != ERROR_OK)
		LOG_WARNING("SAMD: protect settings were not made persistent!");

	res = ERROR_OK;

exit:
	samd_protect_check(bank);

	return res;
}

static int samd_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (samd_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* For each sector to be erased */
	for (unsigned int s = first; s <= last; s++) {
		res = samd_erase_row(bank->target, bank->sectors[s].offset);
		if (res != ERROR_OK) {
			LOG_ERROR("SAMD: failed to erase sector %d at 0x%08" PRIx32, s, bank->sectors[s].offset);
			return res;
		}
	}

	return ERROR_OK;
}


static int samd_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int res;
	uint32_t nvm_ctrlb;
	uint32_t address;
	uint32_t pg_offset;
	uint32_t nb;
	uint32_t nw;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	uint8_t *pb = NULL;
	bool manual_wp;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (samd_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Check if we need to do manual page write commands */
	res = target_read_u32(bank->target, SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLB, &nvm_ctrlb);

	if (res != ERROR_OK)
		return res;

	if (nvm_ctrlb & SAMD_NVM_CTRLB_MANW)
		manual_wp = true;
	else
		manual_wp = false;

	res = samd_issue_nvmctrl_command(bank->target, SAMD_NVM_CMD_PBC);
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

		/* Devices with errata 13134 have automatic page write enabled by default
		 * For other devices issue a write page CMD to the NVM
		 * If the page has not been written up to the last word
		 * then issue CMD_WP always */
		if (manual_wp || pg_offset + 4 * nw < chip->page_size) {
			res = samd_issue_nvmctrl_command(bank->target, SAMD_NVM_CMD_WP);
		} else {
			/* Access through AHB is stalled while flash is being programmed */
			usleep(200);

			res = samd_check_error(bank->target);
		}

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

FLASH_BANK_COMMAND_HANDLER(samd_flash_bank_command)
{
	if (bank->base != SAMD_FLASH) {
		LOG_ERROR("Address " TARGET_ADDR_FMT
				" invalid bank address (try 0x%08" PRIx32
				"[at91samd series] )",
				bank->base, SAMD_FLASH);
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

COMMAND_HANDLER(samd_handle_info_command)
{
	return ERROR_OK;
}

COMMAND_HANDLER(samd_handle_chip_erase_command)
{
	struct target *target = get_current_target(CMD_CTX);
	int res = ERROR_FAIL;

	if (target) {
		/* Enable access to the DSU by disabling the write protect bit */
		target_write_u32(target, SAMD_PAC1, (1<<1));
		/* intentionally without error checking - not accessible on secured chip */

		/* Tell the DSU to perform a full chip erase.  It takes about 240ms to
		 * perform the erase. */
		res = target_write_u8(target, SAMD_DSU + SAMD_DSU_CTRL_EXT, (1<<4));
		if (res == ERROR_OK)
			command_print(CMD, "chip erase started");
		else
			command_print(CMD, "write to DSU CTRL failed");
	}

	return res;
}

COMMAND_HANDLER(samd_handle_set_security_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC < 1 || (CMD_ARGC >= 1 && (strcmp(CMD_ARGV[0], "enable")))) {
		command_print(CMD, "supply the \"enable\" argument to proceed.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		res = samd_issue_nvmctrl_command(target, SAMD_NVM_CMD_SSB);

		/* Check (and clear) error conditions */
		if (res == ERROR_OK)
			command_print(CMD, "chip secured on next power-cycle");
		else
			command_print(CMD, "failed to secure chip");
	}

	return res;
}

COMMAND_HANDLER(samd_handle_eeprom_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if (val == (2 << (13 - code)))
						break;
				}

				if (code > 6) {
					command_print(CMD, "Invalid EEPROM size.  Please see "
							"datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}

			res = samd_modify_user_row(target, code, 4, 6);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAMD_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = ((val >> 4) & 0x7); /* grab size code */

				if (size == 0x7)
					command_print(CMD, "EEPROM is disabled");
				else {
					/* Otherwise, 6 is 256B, 0 is 16KB */
					command_print(CMD, "EEPROM size is %u bytes",
							(2 << (13 - size)));
				}
			}
		}
	}

	return res;
}

static COMMAND_HELPER(get_u64_from_hexarg, unsigned int num, uint64_t *value)
{
	if (num >= CMD_ARGC) {
		command_print(CMD, "Too few Arguments.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (strlen(CMD_ARGV[num]) >= 3 &&
		CMD_ARGV[num][0] == '0' &&
		CMD_ARGV[num][1] == 'x') {
		char *check = NULL;
		*value = strtoull(&(CMD_ARGV[num][2]), &check, 16);
		if ((value == 0 && errno == ERANGE) ||
			check == NULL || *check != 0) {
			command_print(CMD, "Invalid 64-bit hex value in argument %d.",
				num + 1);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else {
		command_print(CMD, "Argument %d needs to be a hex value.", num + 1);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(samd_handle_nvmuserrow_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (CMD_ARGC > 2) {
			command_print(CMD, "Too much Arguments given.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (CMD_ARGC > 0) {
			if (target->state != TARGET_HALTED) {
				LOG_ERROR("Target not halted.");
				return ERROR_TARGET_NOT_HALTED;
			}

			uint64_t mask;
			res = samd_get_reservedmask(target, &mask);
			if (res != ERROR_OK) {
				LOG_ERROR("Couldn't determine the mask for reserved bits.");
				return ERROR_FAIL;
			}
			mask &= NVMUSERROW_LOCKBIT_MASK;

			uint64_t value;
			res = CALL_COMMAND_HANDLER(get_u64_from_hexarg, 0, &value);
			if (res != ERROR_OK)
				return res;
			if (CMD_ARGC == 2) {
				uint64_t mask_temp;
				res = CALL_COMMAND_HANDLER(get_u64_from_hexarg, 1, &mask_temp);
				if (res != ERROR_OK)
					return res;
				mask &= mask_temp;
			}
			res = samd_modify_user_row_masked(target, value, mask);
			if (res != ERROR_OK)
				return res;
		}

		/* read register */
		uint64_t value;
		res = read_userrow(target, &value);
		if (res == ERROR_OK)
			command_print(CMD, "NVMUSERROW: 0x%016"PRIX64, value);
		else
			LOG_ERROR("NVMUSERROW could not be read.");
	}
	return res;
}

COMMAND_HANDLER(samd_handle_bootloader_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		/* Retrieve the MCU's page size, in bytes. */
		uint32_t page_size;
		res = samd_get_flash_page_info(target, &page_size, NULL);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't determine Flash page size");
			return res;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if ((unsigned int)val == (2UL << (8UL - code)) * page_size)
						break;
				}

				if (code > 6) {
					command_print(CMD, "Invalid bootloader size.  Please "
							"see datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

			}

			res = samd_modify_user_row(target, code, 0, 2);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAMD_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = (val & 0x7); /* grab size code */
				uint32_t nb;

				if (size == 0x7)
					nb = 0;
				else
					nb = (2 << (8 - size)) * page_size;

				/* There are 4 pages per row */
				command_print(CMD, "Bootloader size is %" PRIu32 " bytes (%" PRIu32 " rows)",
					   nb, (uint32_t)(nb / (page_size * 4)));
			}
		}
	}

	return res;
}



COMMAND_HANDLER(samd_handle_reset_deassert)
{
	struct target *target = get_current_target(CMD_CTX);
	int retval = ERROR_OK;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

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
		retval = target_write_u32(target, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, DCB_DEMCR,
				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
		/* do not return on error here, releasing DSU reset is more important */
	}

	/* clear CPU Reset Phase Extension bit */
	int retval2 = target_write_u8(target, SAMD_DSU + SAMD_DSU_STATUSA, (1<<1));
	if (retval2 != ERROR_OK)
		return retval2;

	return retval;
}

static const struct command_registration at91samd_exec_command_handlers[] = {
	{
		.name = "dsu_reset_deassert",
		.handler = samd_handle_reset_deassert,
		.mode = COMMAND_EXEC,
		.help = "Deassert internal reset held by DSU.",
		.usage = "",
	},
	{
		.name = "info",
		.handler = samd_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "Print information about the current at91samd chip "
			"and its flash configuration.",
		.usage = "",
	},
	{
		.name = "chip-erase",
		.handler = samd_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase the entire Flash by using the Chip-"
			"Erase feature in the Device Service Unit (DSU).",
		.usage = "",
	},
	{
		.name = "set-security",
		.handler = samd_handle_set_security_command,
		.mode = COMMAND_EXEC,
		.help = "Secure the chip's Flash by setting the Security Bit. "
			"This makes it impossible to read the Flash contents. "
			"The only way to undo this is to issue the chip-erase "
			"command.",
		.usage = "'enable'",
	},
	{
		.name = "eeprom",
		.usage = "[size_in_bytes]",
		.handler = samd_handle_eeprom_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the EEPROM size setting, stored in the User Row. "
			"Please see Table 20-3 of the SAMD20 datasheet for allowed values. "
			"Changes are stored immediately but take affect after the MCU is "
			"reset.",
	},
	{
		.name = "bootloader",
		.usage = "[size_in_bytes]",
		.handler = samd_handle_bootloader_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the bootloader size, stored in the User Row. "
			"Please see Table 20-2 of the SAMD20 datasheet for allowed values. "
			"Changes are stored immediately but take affect after the MCU is "
			"reset.",
	},
	{
		.name = "nvmuserrow",
		.usage = "[value] [mask]",
		.handler = samd_handle_nvmuserrow_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the nvmuserrow register. It is 64 bit wide "
			"and located at address 0x804000. Use the optional mask argument "
			"to prevent changes at positions where the bitvalue is zero. "
			"For security reasons the lock- and reserved-bits are masked out "
			"in background and therefore cannot be changed.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration at91samd_command_handlers[] = {
	{
		.name = "at91samd",
		.mode = COMMAND_ANY,
		.help = "at91samd flash command group",
		.usage = "",
		.chain = at91samd_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver at91samd_flash = {
	.name = "at91samd",
	.commands = at91samd_command_handlers,
	.flash_bank_command = samd_flash_bank_command,
	.erase = samd_erase,
	.protect = samd_protect,
	.write = samd_write,
	.read = default_flash_read,
	.probe = samd_probe,
	.auto_probe = samd_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = samd_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
