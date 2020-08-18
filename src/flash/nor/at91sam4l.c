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

#include <target/cortex_m.h>

/* At this time, the SAM4L Flash is available in these capacities:
 * ATSAM4Lx4xx: 256KB (512 pages)
 * ATSAM4Lx2xx: 128KB (256 pages)
 * ATSAM4Lx8xx: 512KB (1024 pages)
 */

/* There are 16 lockable regions regardless of overall capacity. The number
 * of pages per sector is therefore dependant on capacity. */
#define SAM4L_NUM_SECTORS 16

/* Locations in memory map */
#define SAM4L_FLASH			((uint32_t)0x00000000) /* Flash region */
#define SAM4L_FLASH_USER	0x00800000 /* Flash user page region */
#define SAM4L_FLASHCALW		0x400A0000 /* Flash controller */
#define SAM4L_CHIPID		0x400E0740 /* Chip Identification */

/* Offsets from SAM4L_FLASHCALW */
#define SAM4L_FCR			0x00 /* Flash Control Register (RW) */
#define SAM4L_FCMD			0x04 /* Flash Command Register (RW) */
#define SAM4L_FSR			0x08 /* Flash Status Register (RO) */
#define SAM4L_FPR			0x0C /* Flash Parameter Register (RO) */
#define SAM4L_FVR			0x10 /* Flash Version Register (RO) */
#define SAM4L_FGPFRHI		0x14 /* Flash General Purpose Register High (RO) */
#define SAM4L_FGPFRLO		0x18 /* Flash General Purpose Register Low (RO) */

/* Offsets from SAM4L_CHIPID */
#define SAM4L_CIDR			0x00 /* Chip ID Register (RO) */
#define SAM4L_EXID			0x04 /* Chip ID Extension Register (RO) */

/* Flash commands (for SAM4L_FCMD), see Table 14-5 */
#define SAM4L_FCMD_NOP		0	 /* No Operation */
#define SAM4L_FCMD_WP		1	 /* Write Page */
#define SAM4L_FCMD_EP		2	 /* Erase Page */
#define SAM4L_FCMD_CPB		3	 /* Clear Page Buffer */
#define SAM4L_FCMD_LP		4	 /* Lock region containing given page */
#define SAM4L_FCMD_UP		5	 /* Unlock region containing given page */
#define SAM4L_FCMD_EA		6	 /* Erase All */
#define SAM4L_FCMD_WGPB		7	 /* Write general-purpose fuse bit */
#define SAM4L_FCMD_EGPB		8	 /* Erase general-purpose fuse bit */
#define SAM4L_FCMD_SSB		9	 /* Set security fuses */
#define SAM4L_FCMD_PGPFB	10	/* Program general-purpose fuse byte */
#define SAM4L_FCMD_EAGPF	11	/* Erase all general-purpose fuse bits */
#define SAM4L_FCMD_QPR		12	/* Quick page read */
#define SAM4L_FCMD_WUP		13	/* Write user page */
#define SAM4L_FCMD_EUP		14	/* Erase user page */
#define SAM4L_FCMD_QPRUP	15	/* Quick page read (user page) */
#define SAM4L_FCMD_HSEN		16	/* High speed mode enable */
#define SAM4L_FCMD_HSDIS	17	/* High speed mode disable */

#define SAM4L_FMCD_CMDKEY	0xA5UL	/* 'key' to issue commands, see 14.10.2 */


/* SMAP registers and bits */
#define SMAP_BASE 0x400A3000

#define SMAP_SCR (SMAP_BASE + 8)
#define SMAP_SCR_HCR (1 << 1)


struct sam4l_chip_info {
	uint32_t id;
	uint32_t exid;
	const char *name;
};

/* These are taken from Table 9-1 in 42023E-SAM-07/2013 */
static const struct sam4l_chip_info sam4l_known_chips[] = {
	{ 0xAB0B0AE0, 0x1400000F, "ATSAM4LC8C" },
	{ 0xAB0A09E0, 0x0400000F, "ATSAM4LC4C" },
	{ 0xAB0A07E0, 0x0400000F, "ATSAM4LC2C" },
	{ 0xAB0B0AE0, 0x1300000F, "ATSAM4LC8B" },
	{ 0xAB0A09E0, 0x0300000F, "ATSAM4LC4B" },
	{ 0xAB0A07E0, 0x0300000F, "ATSAM4LC2B" },
	{ 0xAB0B0AE0, 0x1200000F, "ATSAM4LC8A" },
	{ 0xAB0A09E0, 0x0200000F, "ATSAM4LC4A" },
	{ 0xAB0A07E0, 0x0200000F, "ATSAM4LC2A" },
	{ 0xAB0B0AE0, 0x14000002, "ATSAM4LS8C" },
	{ 0xAB0A09E0, 0x04000002, "ATSAM4LS4C" },
	{ 0xAB0A07E0, 0x04000002, "ATSAM4LS2C" },
	{ 0xAB0B0AE0, 0x13000002, "ATSAM4LS8B" },
	{ 0xAB0A09E0, 0x03000002, "ATSAM4LS4B" },
	{ 0xAB0A07E0, 0x03000002, "ATSAM4LS2B" },
	{ 0xAB0B0AE0, 0x12000002, "ATSAM4LS8A" },
	{ 0xAB0A09E0, 0x02000002, "ATSAM4LS4A" },
	{ 0xAB0A07E0, 0x02000002, "ATSAM4LS2A" },
};

/* Meaning of SRAMSIZ field in CHIPID, see 9.3.1 in 42023E-SAM-07/2013 */
static const uint16_t sam4l_ram_sizes[16] = { 48, 1, 2, 6, 24, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512 };

/* Meaning of PSZ field in FPR, see 14.10.4 in 42023E-SAM-07/2013 */
static const uint16_t sam4l_page_sizes[8] = { 32, 64, 128, 256, 512, 1024, 2048, 4096 };

struct sam4l_info {
	const struct sam4l_chip_info *details;

	uint32_t flash_kb;
	uint32_t ram_kb;
	uint32_t page_size;
	int num_pages;
	int sector_size;
	unsigned int pages_per_sector;

	bool probed;
	struct target *target;
};


static int sam4l_flash_wait_until_ready(struct target *target)
{
	volatile unsigned int t = 0;
	uint32_t st;
	int res;

	/* Poll the status register until the FRDY bit is set */
	do {
		res = target_read_u32(target, SAM4L_FLASHCALW + SAM4L_FSR, &st);
	} while (res == ERROR_OK && !(st & (1<<0)) && ++t < 10);

	return res;
}

static int sam4l_flash_check_error(struct target *target, uint32_t *err)
{
	uint32_t st;
	int res;

	res = target_read_u32(target, SAM4L_FLASHCALW + SAM4L_FSR, &st);

	if (res == ERROR_OK)
		*err = st & ((1<<3) | (1<<2)); /* grab PROGE and LOCKE bits */

	return res;
}

static int sam4l_flash_command(struct target *target, uint8_t cmd, int page)
{
	int res;
	uint32_t fcmd;
	uint32_t err;

	res = sam4l_flash_wait_until_ready(target);
	if (res != ERROR_OK)
		return res;

	if (page >= 0) {
		/* Set the page number. For some commands, the page number is just an
		 * argument (ex: fuse bit number). */
		fcmd = (SAM4L_FMCD_CMDKEY << 24) | ((page & 0xFFFF) << 8) | (cmd & 0x3F);
	} else {
		/* Reuse the page number that was read from the flash command register. */
		res = target_read_u32(target, SAM4L_FLASHCALW + SAM4L_FCMD, &fcmd);
		if (res != ERROR_OK)
			return res;

		fcmd &= ~0x3F; /* clear out the command code */
		fcmd |= (SAM4L_FMCD_CMDKEY << 24) | (cmd & 0x3F);
	}

	/* Send the command */
	res = target_write_u32(target, SAM4L_FLASHCALW + SAM4L_FCMD, fcmd);
	if (res != ERROR_OK)
		return res;

	res = sam4l_flash_check_error(target, &err);
	if (res != ERROR_OK)
		return res;

	if (err != 0)
		LOG_ERROR("%s got error status 0x%08" PRIx32, __func__, err);

	res = sam4l_flash_wait_until_ready(target);

	return res;
}

FLASH_BANK_COMMAND_HANDLER(sam4l_flash_bank_command)
{
	if (bank->base != SAM4L_FLASH) {
		LOG_ERROR("Address " TARGET_ADDR_FMT
				" invalid bank address (try 0x%08" PRIx32
				"[at91sam4l series] )",
				bank->base, SAM4L_FLASH);
		return ERROR_FAIL;
	}

	struct sam4l_info *chip;
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

static const struct sam4l_chip_info *sam4l_find_chip_name(uint32_t id, uint32_t exid)
{
	unsigned int i;

	id &= ~0xF;

	for (i = 0; i < ARRAY_SIZE(sam4l_known_chips); i++) {
		if (sam4l_known_chips[i].id == id && sam4l_known_chips[i].exid == exid)
			return &sam4l_known_chips[i];
	}

	return NULL;
}

static int sam4l_check_page_erased(struct flash_bank *bank, uint32_t pn,
		bool *is_erased_p)
{
	int res;
	uint32_t st;

	/* Issue a quick page read to verify that we've erased this page */
	res = sam4l_flash_command(bank->target, SAM4L_FCMD_QPR, pn);
	if (res != ERROR_OK) {
		LOG_ERROR("Quick page read %" PRIu32 " failed", pn);
		return res;
	}

	/* Retrieve the flash status */
	res = target_read_u32(bank->target, SAM4L_FLASHCALW + SAM4L_FSR, &st);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read erase status");
		return res;
	}

	/* Is the page in question really erased? */
	*is_erased_p = !!(st & (1<<5));

	return ERROR_OK;
}

static int sam4l_probe(struct flash_bank *bank)
{
	uint32_t id, exid, param;
	int res;
	struct sam4l_info *chip = (struct sam4l_info *)bank->driver_priv;

	if (chip->probed)
		return ERROR_OK;

	res = target_read_u32(bank->target, SAM4L_CHIPID + SAM4L_CIDR, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read chip ID");
		return res;
	}

	res = target_read_u32(bank->target, SAM4L_CHIPID + SAM4L_EXID, &exid);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read extended chip ID");
		return res;
	}

	chip->details = sam4l_find_chip_name(id, exid);

	/* The RAM capacity is in a lookup table. */
	chip->ram_kb = sam4l_ram_sizes[0xF & (id >> 16)];

	switch (0xF & (id >> 8)) {
		case 0x07:
			chip->flash_kb = 128;
			break;
		case 0x09:
			chip->flash_kb = 256;
			break;
		case 0x0A:
			chip->flash_kb = 512;
			break;
		default:
			LOG_ERROR("Unknown flash size (chip ID is %08" PRIx32 "), assuming 128K", id);
			chip->flash_kb = 128;
			break;
	}

	/* Retrieve the Flash parameters */
	res = target_read_u32(bank->target, SAM4L_FLASHCALW + SAM4L_FPR, &param);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Flash parameters");
		return res;
	}

	/* Fetch the page size from the parameter register.	Technically the flash
	 * capacity is there too though the manual mentions that not all parts will
	 * have it set so we use the Chip ID capacity information instead. */
	chip->page_size = sam4l_page_sizes[0x7 & (param >> 8)];
	assert(chip->page_size);
	chip->num_pages = chip->flash_kb * 1024 / chip->page_size;

	chip->sector_size = (chip->flash_kb * 1024) / SAM4L_NUM_SECTORS;
	chip->pages_per_sector = chip->sector_size / chip->page_size;

	/* Make sure the bank size is correct */
	bank->size = chip->flash_kb * 1024;

	/* Allocate the sector table. */
	bank->num_sectors = SAM4L_NUM_SECTORS;
	bank->sectors = calloc(bank->num_sectors, (sizeof((bank->sectors)[0])));
	if (!bank->sectors)
		return ERROR_FAIL;

	/* Fill out the sector information: all SAM4L sectors are the same size and
	 * there is always a fixed number of them. */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = chip->sector_size;
		bank->sectors[i].offset = i * chip->sector_size;
		/* mark as unknown */
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	/* Done */
	chip->probed = true;

	LOG_INFO("SAM4L MCU: %s (Rev %c) (%" PRIu32 "KB Flash with %d %" PRIu32 "B pages, %" PRIu32 "KB RAM)",
			chip->details ? chip->details->name : "unknown", (char)('A' + (id & 0xF)),
			chip->flash_kb, chip->num_pages, chip->page_size, chip->ram_kb);

	return ERROR_OK;
}

static int sam4l_protect_check(struct flash_bank *bank)
{
	int res;
	uint32_t st;
	struct sam4l_info *chip = (struct sam4l_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (sam4l_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	res = target_read_u32(bank->target, SAM4L_FLASHCALW + SAM4L_FSR, &st);
	if (res != ERROR_OK)
		return res;

	st >>= 16; /* There are 16 lock region bits in the upper half word */
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = !!(st & (1<<i));

	return ERROR_OK;
}

static int sam4l_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct sam4l_info *chip = (struct sam4l_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (sam4l_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Make sure the pages make sense. */
	if (first >= bank->num_sectors || last >= bank->num_sectors) {
		LOG_ERROR("Protect range %u - %u not valid (%u sectors total)", first, last,
				bank->num_sectors);
		return ERROR_FAIL;
	}

	/* Try to lock or unlock each sector in the range.	This is done by locking
	 * a region containing one page in that sector, we arbitrarily choose the 0th
	 * page in the sector. */
	for (unsigned int i = first; i <= last; i++) {
		int res;

		res = sam4l_flash_command(bank->target,
				set ? SAM4L_FCMD_LP : SAM4L_FCMD_UP, i * chip->pages_per_sector);
		if (res != ERROR_OK) {
			LOG_ERROR("Can't %slock region containing page %d", set ? "" : "un", i);
			return res;
		}
	}

	return ERROR_OK;
}

static int sam4l_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int ret;
	struct sam4l_info *chip = (struct sam4l_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (sam4l_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Make sure the pages make sense. */
	if (first >= bank->num_sectors || last >= bank->num_sectors) {
		LOG_ERROR("Erase range %u - %u not valid (%u sectors total)", first, last,
				bank->num_sectors);
		return ERROR_FAIL;
	}

	/* Erase */
	if ((first == 0) && ((last + 1) == bank->num_sectors)) {
		LOG_DEBUG("Erasing the whole chip");

		ret = sam4l_flash_command(bank->target, SAM4L_FCMD_EA, -1);
		if (ret != ERROR_OK) {
			LOG_ERROR("Erase All failed");
			return ret;
		}
	} else {
		LOG_DEBUG("Erasing sectors %u through %u...\n", first, last);

		/* For each sector... */
		for (unsigned int i = first; i <= last; i++) {
			/* For each page in that sector... */
			for (unsigned int j = 0; j < chip->pages_per_sector; j++) {
				unsigned int pn = i * chip->pages_per_sector + j;
				bool is_erased = false;

				/* Issue the page erase */
				ret = sam4l_flash_command(bank->target, SAM4L_FCMD_EP, pn);
				if (ret != ERROR_OK) {
					LOG_ERROR("Erasing page %u failed", pn);
					return ret;
				}

				ret = sam4l_check_page_erased(bank, pn, &is_erased);
				if (ret != ERROR_OK)
					return ret;

				if (!is_erased) {
					LOG_DEBUG("Page %u was not erased.", pn);
					return ERROR_FAIL;
				}
			}

			/* This sector is definitely erased. */
			bank->sectors[i].is_erased = 1;
		}
	}

	return ERROR_OK;
}

/* Write an entire page from host buffer 'buf' to page-aligned 'address' in the
 * Flash. */
static int sam4l_write_page(struct sam4l_info *chip, struct target *target,
		uint32_t address, const uint8_t *buf)
{
	int res;

	LOG_DEBUG("sam4l_write_page address=%08" PRIx32, address);

	/* Clear the page buffer before we write to it */
	res = sam4l_flash_command(target, SAM4L_FCMD_CPB, -1);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: can't clear page buffer", __func__);
		return res;
	}

	/* Write the modified page back to the target's page buffer */
	res = target_write_memory(target, address, 4, chip->page_size / 4, buf);

	if (res != ERROR_OK) {
		LOG_ERROR("%s: %d", __func__, __LINE__);
		return res;
	}

	/* Commit the page contents to Flash: erase the current page and then
	 * write it out. */
	res = sam4l_flash_command(target, SAM4L_FCMD_EP, -1);
	if (res != ERROR_OK)
		return res;
	res = sam4l_flash_command(target, SAM4L_FCMD_WP, -1);

	return res;
}

/* Write partial contents into page-aligned 'address' on the Flash from host
 * buffer 'buf' by writing 'nb' of 'buf' at 'offset' into the Flash page. */
static int sam4l_write_page_partial(struct sam4l_info *chip,
		struct flash_bank *bank, uint32_t address, const uint8_t *buf,
		uint32_t page_offset, uint32_t nb)
{
	int res;
	uint8_t *pg = malloc(chip->page_size);
	if (!pg)
		return ERROR_FAIL;

	LOG_DEBUG("sam4l_write_page_partial address=%08" PRIx32 " nb=%08" PRIx32, address, nb);

	assert(page_offset + nb < chip->page_size);
	assert((address % chip->page_size) == 0);

	/* Retrieve the full page contents from Flash */
	res = target_read_memory(bank->target, address, 4,
			chip->page_size / 4, pg);
	if (res != ERROR_OK) {
		free(pg);
		return res;
	}

	/* Insert our partial page over the data from Flash */
	memcpy(pg + (page_offset % chip->page_size), buf, nb);

	/* Write the page back out */
	res = sam4l_write_page(chip, bank->target, address, pg);
	free(pg);

	return res;
}

static int sam4l_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int res;
	uint32_t nb = 0;
	struct sam4l_info *chip = (struct sam4l_info *)bank->driver_priv;

	LOG_DEBUG("sam4l_write offset=%08" PRIx32 " count=%08" PRIx32, offset, count);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (sam4l_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset % chip->page_size) {
		/* We're starting at an unaligned offset so we'll write a partial page
		 * comprising that offset and up to the end of that page. */
		nb = chip->page_size - (offset % chip->page_size);
		if (nb > count)
			nb = count;
	} else if (count < chip->page_size) {
		/* We're writing an aligned but partial page. */
		nb = count;
	}

	if (nb > 0) {
		res = sam4l_write_page_partial(chip, bank,
				(offset / chip->page_size) * chip->page_size + bank->base,
				buffer,
				offset % chip->page_size, nb);
		if (res != ERROR_OK)
			return res;

		/* We're done with the page contents */
		count -= nb;
		offset += nb;
	}

	/* There's at least one aligned page to write out. */
	if (count >= chip->page_size) {
		assert(chip->page_size > 0);
		int np = count / chip->page_size + ((count % chip->page_size) ? 1 : 0);

		for (int i = 0; i < np; i++) {
			if (count >= chip->page_size) {
				res = sam4l_write_page(chip, bank->target,
						bank->base + offset,
						buffer + (i * chip->page_size));
				/* Advance one page */
				offset += chip->page_size;
				count -= chip->page_size;
			} else {
				res = sam4l_write_page_partial(chip, bank,
						bank->base + offset,
						buffer + (i * chip->page_size), 0, count);
				/* We're done after this. */
				offset += count;
				count = 0;
			}

			if (res != ERROR_OK)
				return res;
		}
	}

	return ERROR_OK;
}


COMMAND_HANDLER(sam4l_handle_reset_deassert)
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
	 * so we just release reset held by SMAP
	 *
	 * n_RESET (srst) clears the DP, so reenable debug and set vector catch here
	 *
	 * After vectreset SMAP release is not needed however makes no harm
	 */
	if (target->reset_halt && (jtag_reset_config & RESET_HAS_SRST)) {
		retval = target_write_u32(target, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, DCB_DEMCR,
				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
		/* do not return on error here, releasing SMAP reset is more important */
	}

	int retval2 = target_write_u32(target, SMAP_SCR, SMAP_SCR_HCR);
	if (retval2 != ERROR_OK)
		return retval2;

	return retval;
}

static const struct command_registration at91sam4l_exec_command_handlers[] = {
	{
		.name = "smap_reset_deassert",
		.handler = sam4l_handle_reset_deassert,
		.mode = COMMAND_EXEC,
		.help = "deassert internal reset held by SMAP",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration at91sam4l_command_handlers[] = {
	{
		.name = "at91sam4l",
		.mode = COMMAND_ANY,
		.help = "at91sam4l flash command group",
		.usage = "",
		.chain = at91sam4l_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver at91sam4l_flash = {
	.name = "at91sam4l",
	.commands = at91sam4l_command_handlers,
	.flash_bank_command = sam4l_flash_bank_command,
	.erase = sam4l_erase,
	.protect = sam4l_protect,
	.write = sam4l_write,
	.read = default_flash_read,
	.probe = sam4l_probe,
	.auto_probe = sam4l_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = sam4l_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
