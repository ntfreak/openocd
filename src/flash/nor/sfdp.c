/***************************************************************************
 *   Copyright (C) 2019 by Andreas Bolsch <andreas.bolsch@mni.thm.de	   *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
 *   (at your option) any later version.								   *
 *																		   *
 *   This program is distributed in the hope that it will be useful,	   *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *   GNU General Public License for more details.						   *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License	   *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include "sfdp.h"

#define SFDP_MAGIC			0x50444653
#define SFDP_ACCESS_PROT	0xFF
#define SFDP_BASIC_FLASH	0xFF00
#define SFDP_4BYTE_ADDR		0xFF84

static const char *sfdp_name = "sfdp";

struct sfdp_hdr {
	uint32_t			signature;
	uint32_t			revision;
};

struct sfdp_phdr {
	uint32_t			revision;
	uint32_t			ptr;
};

struct sfdp_basic_flash_param {
	uint32_t			fast_addr;		/* 01: fast read and 3/4 address bytes */
	uint32_t			density;		/* 02: memory density */
	uint32_t			fast_1x4;		/* 03: 1-1-4 and 1-4-4 fast read */
	uint32_t			fast_1x2;		/* 04: 1-2-2 and 1-1-2 fast read */
	uint32_t			fast_444;		/* 05: 4-4-4 and 2-2-2 fast read */
	uint32_t			read_222;		/* 06: 2-2-2 fast read instr and dummy */
	uint32_t			read_444;		/* 07: 4-4-4 fast read instr and dummy */
	uint32_t			erase_t12;		/* 08: erase types 1, 2 */
	uint32_t			erase_t34;		/* 09: erase types 3, 4 */
	uint32_t			erase_time;		/* 10: erase times for types 1 - 4 */
	uint32_t			chip_byte;		/* 11: chip erase time, byte prog time, page prog */
	uint32_t			susp_time;		/* 12: suspend and resume times */
	uint32_t			susp_instr;		/* 13: suspend and resume instr */
	uint32_t			pwrd_instr;		/* 14: powerdown instr */
	uint32_t			quad_req;		/* 15: quad enable requirements */
	uint32_t			addr_reset;		/* 16: 3-/4-byte addressing and reset */
	uint32_t			read_1x8;		/* 17: 1-1-8 and 1-8-8 fast read instr and dummy */
	uint32_t			dtr_drive;		/* 18: dtr modes and drive strength */
	uint32_t			octal_req;		/* 19: octal enable requirements */
	uint32_t			speed_888;		/* 20: speed in 8-8-8 modes */
};

struct sfdp_4byte_addr_param {
	uint32_t			flags;			/* 01: various flags */
	uint32_t			erase_t1234;	/* 02: erase commands */
};

/* Try to get parameters from flash via SFDP */
int spi_sfdp(struct flash_bank *bank, struct flash_device *dev,
	read_sfdp_block_t read_sfdp_block)
{
	struct sfdp_hdr header;
	struct sfdp_phdr *pheaders = NULL;
	uint32_t *ptable = NULL;
	unsigned int j, k, nph;
	int retval, erase_type = 0;

	memset(dev, 0, sizeof(struct flash_device));

	/* retrieve SFDP header */
	memset(&header, 0, sizeof(header));
	retval = read_sfdp_block(bank, 0x0, sizeof(header) >> 2, (uint32_t *)&header);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("header 0x%08" PRIx32 " 0x%08" PRIx32, header.signature, header.revision);
	if (header.signature != SFDP_MAGIC) {
		LOG_INFO("no SDFP found");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}
	if (((header.revision >> 24) & 0xFF) != SFDP_ACCESS_PROT) {
		LOG_ERROR("access protocol 0x%02x not implemented",
			(header.revision >> 24) & 0xFFU);
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* retrieve table of parameter headers */
	nph = ((header.revision >> 16) & 0xFF) + 1;
	LOG_DEBUG("parameter headers: %d", nph);
	pheaders = malloc(sizeof(struct sfdp_phdr) * nph);
	if (pheaders == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}
	memset(pheaders, 0, sizeof(struct sfdp_phdr) * nph);
	retval = read_sfdp_block(bank, sizeof(header),
		(sizeof(struct sfdp_phdr) >> 2) * nph, (uint32_t *)pheaders);
	if (retval != ERROR_OK)
		goto err;

	for (k = 0; k < nph; k++) {
		uint8_t words = (pheaders[k].revision >> 24) & 0xFF;
		uint16_t id = (((pheaders[k].ptr) >> 16) & 0xFF00) | (pheaders[k].revision & 0xFF);
		uint32_t ptr = pheaders[k].ptr & 0xFFFFFF;

		LOG_DEBUG("pheader %d len=0x%02" PRIx8 " id=0x%04" PRIx16
			" ptr=0x%06" PRIx32, k, words, id, ptr);

		/* retrieve parameter table */
		ptable = malloc(words << 2);
		if (ptable == NULL) {
			LOG_ERROR("not enough memory");
			retval = ERROR_FAIL;
			goto err;
		}
		retval = read_sfdp_block(bank, ptr, words, ptable);
		if (retval != ERROR_OK)
			goto err;

		for (j = 0; j < words; j++)
			LOG_DEBUG("word %02d 0x%08X", j + 1, ptable[j]);

		if (id == SFDP_BASIC_FLASH) {
			struct sfdp_basic_flash_param *table = (struct sfdp_basic_flash_param *)ptable;
			uint16_t erase;

			if (words < 9) {
				LOG_ERROR("id=0x%04" PRIx16 " invalid length %d", id, words);
				retval = ERROR_FLASH_BANK_NOT_PROBED;
				goto err;
			}

			LOG_DEBUG("basic flash parameter table");
			/* dummy device name */
			dev->name = sfdp_name;

			/* default instructions */
			dev->read_cmd = SPIFLASH_READ;
			dev->pprog_cmd = SPIFLASH_PAGE_PROGRAM;
			dev->chip_erase_cmd = SPIFLASH_MASS_ERASE;

			/* get device size */
			if (table->density & (1UL << 31))
				dev->size_in_bytes = 1UL << ((table->density & ~(1UL << 31)) - 3);
			else
				dev->size_in_bytes = (table->density + 1) >> 3;

			/* 2-2-2 read instruction, not used */
			if (table->fast_444 & (1UL << 0))
				dev->qread_cmd = (table->read_222 >> 24) & 0xFF;

			/* 4-4-4 read instruction */
			if (table->fast_444 & (1UL << 4))
				dev->qread_cmd = (table->read_444 >> 24) & 0xFF;

			/* find the largest erase block size and instruction */
			erase = (table->erase_t12 >> 0) & 0xFFFF;
			erase_type = 1;
			if (((table->erase_t12 >> 16) & 0xFF) > (erase & 0xFF)) {
				erase = (table->erase_t12 >> 16) & 0xFFFF;
				erase_type = 2;
			}
			if (((table->erase_t34 >> 0) & 0xFF) > (erase & 0xFF)) {
				erase = (table->erase_t34 >> 0) & 0xFFFF;
				erase_type = 3;
			}
			if (((table->erase_t34 >> 16) & 0xFF) > (erase & 0xFF)) {
				erase = (table->erase_t34 >> 16) & 0xFFFF;
				erase_type = 4;
			}
			dev->erase_cmd = (erase >> 8) & 0xFF;
			dev->sectorsize = 1UL << (erase & 0xFF);

			if ((offsetof(struct sfdp_basic_flash_param, chip_byte) >> 2) < words) {
				/* get Program Page Size, if chip_byte present, that's optional */
				dev->pagesize = 1UL << ((table->chip_byte >> 4) & 0x0F);
			} else {
				/* no explicit page size specified ... */
				if (table->fast_addr & (1UL << 2)) {
					/* Write Granularity = 1, use 64 bytes */
					dev->pagesize = 1UL << 6;
				} else {
					/* Write Granularity = 0, use 16 bytes */
					dev->pagesize = 1UL << 4;
				}
			}

			if (dev->size_in_bytes > (1UL << 24)) {
				if (((table->fast_addr >> 17) & 0x3) == 0x0)
					LOG_ERROR("device needs paging - not implemented");

				/* 4-byte addresses needed if more than 16 MBytes */
				if (((offsetof(struct sfdp_basic_flash_param, addr_reset) >> 2) < words) &&
					(table->addr_reset & (1UL << 29))) {
					/* dedicated 4-byte-address instructions, hopefully these ...
					 * this entry is unfortunately optional as well
					 * a subsequent 4-byte address table may overwrite this */
					dev->read_cmd = 0x13;
					dev->pprog_cmd = 0x12;
					dev->erase_cmd = 0xDC;
					if (dev->qread_cmd != 0)
						dev->qread_cmd = 0xEC;
				} else if (((table->fast_addr >> 17) & 0x3) == 0x1)
					LOG_INFO("device has to be switched to 4-byte addresses");
			}
		} else if (id == SFDP_4BYTE_ADDR) {
			struct sfdp_4byte_addr_param *table = (struct sfdp_4byte_addr_param *)ptable;

			if (words >= (offsetof(struct sfdp_4byte_addr_param, erase_t1234)
				+ sizeof(table->erase_t1234)) >> 2) {
				LOG_INFO("4-byte address parameter table");

				/* read and page program instructions */
				if (table->flags & (1UL << 0))
					dev->read_cmd = 0x13;
				if (table->flags & (1UL << 5))
					dev->qread_cmd = 0xEC;
				if (table->flags & (1UL << 6))
					dev->pprog_cmd = 0x12;

				/* erase instructions */
				if ((erase_type == 1) && (table->flags & (1UL << 9)))
					dev->erase_cmd = (table->erase_t1234 >> 0) & 0xFF;
				else if ((erase_type == 2) && (table->flags & (1UL << 10)))
					dev->erase_cmd = (table->erase_t1234 >> 8) & 0xFF;
				else if ((erase_type == 3) && (table->flags & (1UL << 11)))
					dev->erase_cmd = (table->erase_t1234 >> 16) & 0xFF;
				else if ((erase_type == 4) && (table->flags & (1UL << 12)))
					dev->erase_cmd = (table->erase_t1234 >> 24) & 0xFF;
			} else
				LOG_ERROR("parameter table id=0x%04" PRIx16 " invalid length %d", id, words);
		} else
			LOG_DEBUG("unimplemented parameter table id=0x%04" PRIx16, id);

		free(ptable);
		ptable = NULL;
	}

	if (erase_type != 0) {
		LOG_INFO("valid SFDP detected");
		retval = ERROR_OK;
	} else {
		LOG_ERROR("incomplete/invalid SFDP");
		retval = ERROR_FLASH_BANK_NOT_PROBED;
	}

err:
	free(pheaders);
	free(ptable);

	return retval;
}
