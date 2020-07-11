/**************************************************************************
*   Copyright (C) 2015 Jeff Ciesielski <jeffciesielski@gmail.com>         *
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
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* Maximum number of sectors */
#define MAX_XMC_SECTORS 12

/* System control unit registers */
#define SCU_REG_BASE 0x50004000

#define SCU_ID_CHIP 0x04

/* Base of the non-cached flash memory */
#define PFLASH_BASE	0x0C000000

/* User configuration block offsets */
#define UCB0_BASE       0x00000000
#define UCB1_BASE       0x00000400
#define UCB2_BASE       0x00000800

/* Flash register base */
#define FLASH_REG_BASE 0x58000000

/* PMU ID Registers */
#define FLASH_REG_PMU_ID	(FLASH_REG_BASE | 0x0508)

/* PMU Fields */
#define PMU_MOD_REV_MASK	0xFF
#define PMU_MOD_TYPE_MASK	0xFF00
#define PMU_MOD_NO_MASK		0xFFFF0000

/* Prefetch Config */
#define FLASH_REG_PREF_PCON	(FLASH_REG_BASE | 0x4000)

/* Prefetch Fields */
#define PCON_IBYP	(1 << 0)
#define PCON_IINV	(1 << 1)

/* Flash ID Register */
#define FLASH_REG_FLASH0_ID	(FLASH_REG_BASE | 0x2008)

/* Flash Status Register */
#define FLASH_REG_FLASH0_FSR	(FLASH_REG_BASE | 0x2010)

#define FSR_PBUSY	(0)
#define FSR_FABUSY	(1)
#define FSR_PROG	(4)
#define FSR_ERASE	(5)
#define FSR_PFPAGE	(6)
#define FSR_PFOPER	(8)
#define FSR_SQER	(10)
#define FSR_PROER	(11)
#define FSR_PFSBER	(12)
#define FSR_PFDBER	(14)
#define FSR_PROIN	(16)
#define FSR_RPROIN	(18)
#define FSR_RPRODIS	(19)
#define FSR_WPROIN0	(21)
#define FSR_WPROIN1	(22)
#define FSR_WPROIN2	(23)
#define FSR_WPRODIS0	(25)
#define FSR_WPRODIS1	(26)
#define FSR_SLM		(28)
#define FSR_VER		(31)

#define FSR_PBUSY_MASK		(0x01 << FSR_PBUSY)
#define FSR_FABUSY_MASK		(0x01 << FSR_FABUSY)
#define FSR_PROG_MASK		(0x01 << FSR_PROG)
#define FSR_ERASE_MASK		(0x01 << FSR_ERASE)
#define FSR_PFPAGE_MASK		(0x01 << FSR_PFPAGE)
#define FSR_PFOPER_MASK		(0x01 << FSR_PFOPER)
#define FSR_SQER_MASK		(0x01 << FSR_SQER)
#define FSR_PROER_MASK		(0x01 << FSR_PROER)
#define FSR_PFSBER_MASK		(0x01 << FSR_PFSBER)
#define FSR_PFDBER_MASK		(0x01 << FSR_PFDBER)
#define FSR_PROIN_MASK		(0x01 << FSR_PROIN)
#define FSR_RPROIN_MASK		(0x01 << FSR_RPROIN)
#define FSR_RPRODIS_MASK	(0x01 << FSR_RPRODIS)
#define FSR_WPROIN0_MASK	(0x01 << FSR_WPROIN0)
#define FSR_WPROIN1_MASK	(0x01 << FSR_WPROIN1)
#define FSR_WPROIN2_MASK	(0x01 << FSR_WPROIN2)
#define FSR_WPRODIS0_MASK	(0x01 << FSR_WPRODIS0)
#define FSR_WPRODIS1_MASK	(0x01 << FSR_WPRODIS1)
#define FSR_SLM_MASK		(0x01 << FSR_SLM)
#define FSR_VER_MASK		(0x01 << FSR_VER)

/* Flash Config Register */
#define FLASH_REG_FLASH0_FCON	(FLASH_REG_BASE | 0x2014)

#define FCON_WSPFLASH           (0)
#define FCON_WSECPF             (4)
#define FCON_IDLE               (13)
#define FCON_ESLDIS             (14)
#define FCON_SLEEP              (15)
#define FCON_RPA                (16)
#define FCON_DCF                (17)
#define FCON_DDF                (18)
#define FCON_VOPERM             (24)
#define FCON_SQERM              (25)
#define FCON_PROERM             (26)
#define FCON_PFSBERM            (27)
#define FCON_PFDBERM            (29)
#define FCON_EOBM               (31)

#define FCON_WSPFLASH_MASK      (0x0f << FCON_WSPFLASH)
#define FCON_WSECPF_MASK        (0x01 << FCON_WSECPF)
#define FCON_IDLE_MASK          (0x01 << FCON_IDLE)
#define FCON_ESLDIS_MASK        (0x01 << FCON_ESLDIS)
#define FCON_SLEEP_MASK         (0x01 << FCON_SLEEP)
#define FCON_RPA_MASK           (0x01 << FCON_RPA)
#define FCON_DCF_MASK           (0x01 << FCON_DCF)
#define FCON_DDF_MASK           (0x01 << FCON_DDF)
#define FCON_VOPERM_MASK        (0x01 << FCON_VOPERM)
#define FCON_SQERM_MASK         (0x01 << FCON_SQERM)
#define FCON_PROERM_MASK        (0x01 << FCON_PROERM)
#define FCON_PFSBERM_MASK       (0x01 << FCON_PFSBERM)
#define FCON_PFDBERM_MASK       (0x01 << FCON_PFDBERM)
#define FCON_EOBM_MASK          (0x01 << FCON_EOBM)

/* Flash Margin Control Register */
#define FLASH_REG_FLASH0_MARP	(FLASH_REG_BASE | 0x2018)

#define MARP_MARGIN		(0)
#define MARP_TRAPDIS		(15)

#define MARP_MARGIN_MASK        (0x0f << MARP_MARGIN)
#define MARP_TRAPDIS_MASK       (0x01 << MARP_TRAPDIS)

/* Flash Protection Registers */
#define FLASH_REG_FLASH0_PROCON0	(FLASH_REG_BASE | 0x2020)
#define FLASH_REG_FLASH0_PROCON1	(FLASH_REG_BASE | 0x2024)
#define FLASH_REG_FLASH0_PROCON2	(FLASH_REG_BASE | 0x2028)

#define PROCON_S0L             (0)
#define PROCON_S1L             (1)
#define PROCON_S2L             (2)
#define PROCON_S3L             (3)
#define PROCON_S4L             (4)
#define PROCON_S5L             (5)
#define PROCON_S6L             (6)
#define PROCON_S7L             (7)
#define PROCON_S8L             (8)
#define PROCON_S9L             (9)
#define PROCON_S10_S11L        (10)
#define PROCON_RPRO            (15)

#define PROCON_S0L_MASK        (0x01 << PROCON_S0L)
#define PROCON_S1L_MASK        (0x01 << PROCON_S1L)
#define PROCON_S2L_MASK        (0x01 << PROCON_S2L)
#define PROCON_S3L_MASK        (0x01 << PROCON_S3L)
#define PROCON_S4L_MASK        (0x01 << PROCON_S4L)
#define PROCON_S5L_MASK        (0x01 << PROCON_S5L)
#define PROCON_S6L_MASK        (0x01 << PROCON_S6L)
#define PROCON_S7L_MASK        (0x01 << PROCON_S7L)
#define PROCON_S8L_MASK        (0x01 << PROCON_S8L)
#define PROCON_S9L_MASK        (0x01 << PROCON_S9L)
#define PROCON_S10_S11L_MASK   (0x01 << PROCON_S10_S11L)
#define PROCON_RPRO_MASK       (0x01 << PROCON_RPRO)

#define FLASH_PROTECT_CONFIRMATION_CODE 0x8AFE15C3

/* Flash controller configuration values */
#define FLASH_ID_XMC4500        0xA2
#define FLASH_ID_XMC4300_XMC4700_4800   0x92
#define FLASH_ID_XMC4100_4200   0x9C
#define FLASH_ID_XMC4400        0x9F

/* Timeouts */
#define FLASH_OP_TIMEOUT 5000

/* Flash commands (write/erase/protect) are performed using special
 * command sequences that are written to magic addresses in the flash controller */
/* Command sequence addresses.  See reference manual, section 8: Flash Command Sequences */
#define FLASH_CMD_ERASE_1 0x0C005554
#define FLASH_CMD_ERASE_2 0x0C00AAA8
#define FLASH_CMD_ERASE_3 FLASH_CMD_ERASE_1
#define FLASH_CMD_ERASE_4 FLASH_CMD_ERASE_1
#define FLASH_CMD_ERASE_5 FLASH_CMD_ERASE_2
/* ERASE_6 is the sector base address */

#define FLASH_CMD_CLEAR_STATUS FLASH_CMD_ERASE_1

#define FLASH_CMD_ENTER_PAGEMODE FLASH_CMD_ERASE_1

#define FLASH_CMD_LOAD_PAGE_1 0x0C0055F0
#define FLASH_CMD_LOAD_PAGE_2 0x0C0055F4

#define FLASH_CMD_WRITE_PAGE_1 FLASH_CMD_ERASE_1
#define FLASH_CMD_WRITE_PAGE_2 FLASH_CMD_ERASE_2
#define FLASH_CMD_WRITE_PAGE_3 FLASH_CMD_ERASE_1
/* WRITE_PAGE_4 is the page base address */

#define FLASH_CMD_TEMP_UNPROT_1 FLASH_CMD_ERASE_1
#define FLASH_CMD_TEMP_UNPROT_2 FLASH_CMD_ERASE_2
#define FLASH_CMD_TEMP_UNPROT_3 0x0C00553C
#define FLASH_CMD_TEMP_UNPROT_4 FLASH_CMD_ERASE_2
#define FLASH_CMD_TEMP_UNPROT_5 FLASH_CMD_ERASE_2
#define FLASH_CMD_TEMP_UNPROT_6 0x0C005558

struct xmc4xxx_flash_bank {
	bool probed;

	/* We need the flash controller ID to choose the sector layout */
	uint32_t fcon_id;

	/* Passwords used for protection operations */
	uint32_t pw1;
	uint32_t pw2;
	bool pw_set;

	/* Protection flags */
	bool read_protected;

	bool write_prot_otp[MAX_XMC_SECTORS];
};

struct xmc4xxx_command_seq {
	uint32_t address;
	uint32_t magic;
};

/* Sector capacities.  See section 8 of xmc4x00_rm */
static const unsigned int sector_capacity_8[8] = {
	16, 16, 16, 16, 16, 16, 16, 128
};

static const unsigned int sector_capacity_9[9] = {
	16, 16, 16, 16, 16, 16, 16, 128, 256
};

static const unsigned int sector_capacity_12[12] = {
	16, 16, 16, 16, 16, 16, 16, 16, 128, 256, 256, 256
};

static const unsigned int sector_capacity_16[16] = {
	16, 16, 16, 16, 16, 16, 16, 16, 128, 256, 256, 256, 256, 256, 256, 256
};

static int xmc4xxx_write_command_sequence(struct flash_bank *bank,
					 struct xmc4xxx_command_seq *seq,
					 int seq_len)
{
	int res = ERROR_OK;

	for (int i = 0; i < seq_len; i++) {
		res = target_write_u32(bank->target, seq[i].address,
				       seq[i].magic);
		if (res != ERROR_OK)
			return res;
	}

	return ERROR_OK;
}

static int xmc4xxx_load_bank_layout(struct flash_bank *bank)
{
	const unsigned int *capacity = NULL;

	/* At this point, we know which flash controller ID we're
	 * talking to and simply need to fill out the bank structure accordingly */
	LOG_DEBUG("%u sectors", bank->num_sectors);

	switch (bank->num_sectors) {
	case 8:
		capacity = sector_capacity_8;
		break;
	case 9:
		capacity = sector_capacity_9;
		break;
	case 12:
		capacity = sector_capacity_12;
		break;
	case 16:
		capacity = sector_capacity_16;
		break;
	default:
		LOG_ERROR("Unexpected number of sectors, %u\n",
			  bank->num_sectors);
		return ERROR_FAIL;
	}

	/* This looks like a bank that we understand, now we know the
	 * corresponding sector capacities and we can add those up into the
	 * bank size. */
	uint32_t total_offset = 0;
	bank->sectors = calloc(bank->num_sectors,
			       sizeof(struct flash_sector));
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = capacity[i] * 1024;
		bank->sectors[i].offset = total_offset;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;

		bank->size += bank->sectors[i].size;
		LOG_DEBUG("\t%d: %uk", i, capacity[i]);
		total_offset += bank->sectors[i].size;
	}

	/* This part doesn't follow the typical standard of 0xff
	 * being the erased value.*/
	bank->default_padded_value = bank->erased_value = 0x00;

	return ERROR_OK;
}

static int xmc4xxx_probe(struct flash_bank *bank)
{
	int res;
	uint32_t devid, config;
	struct xmc4xxx_flash_bank *fb = bank->driver_priv;
	uint8_t flash_id;

	if (fb->probed)
		return ERROR_OK;

	/* It's not possible for the DAP to access the OTP locations needed for
	 * probing the part info and Flash geometry so we require that the target
	 * be halted before proceeding. */
	if (bank->target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The SCU registers contain the ID of the chip */
	res = target_read_u32(bank->target, SCU_REG_BASE + SCU_ID_CHIP, &devid);
	if (res != ERROR_OK) {
		LOG_ERROR("Cannot read device identification register.");
		return res;
	}

	/* Make sure this is a XMC4000 family device */
	if ((devid & 0xF0000) != 0x40000 && devid != 0) {
		LOG_ERROR("Platform ID doesn't match XMC4xxx: 0x%08" PRIx32, devid);
		return ERROR_FAIL;
	}

	LOG_DEBUG("Found XMC4xxx with devid: 0x%08" PRIx32, devid);

	/* Now sanity-check the Flash controller itself. */
	res = target_read_u32(bank->target, FLASH_REG_FLASH0_ID,
			&config);
	if (res != ERROR_OK) {
		LOG_ERROR("Cannot read Flash bank configuration.");
		return res;
	}
	flash_id = (config & 0xff0000) >> 16;

	/* The Flash configuration register is our only means of
	 * determining the sector layout. We need to make sure that
	 * we understand the type of controller we're dealing with */
	switch (flash_id) {
	case FLASH_ID_XMC4100_4200:
		bank->num_sectors = 8;
		LOG_DEBUG("XMC4xxx: XMC4100/4200 detected.");
		break;
	case FLASH_ID_XMC4400:
		bank->num_sectors = 9;
		LOG_DEBUG("XMC4xxx: XMC4400 detected.");
		break;
	case FLASH_ID_XMC4500:
		bank->num_sectors = 12;
		LOG_DEBUG("XMC4xxx: XMC4500 detected.");
		break;
	case FLASH_ID_XMC4300_XMC4700_4800:
		bank->num_sectors = 16;
		LOG_DEBUG("XMC4xxx: XMC4700/4800 detected.");
		break;
	default:
		LOG_ERROR("XMC4xxx: Unexpected flash ID. got %02" PRIx8,
			  flash_id);
		return ERROR_FAIL;
	}

	/* Retrieve information about the particular bank we're probing and fill in
	 * the bank structure accordingly. */
	res = xmc4xxx_load_bank_layout(bank);
	if (res == ERROR_OK) {
		/* We're done */
		fb->probed = true;
	} else {
		LOG_ERROR("Unable to load bank information.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int xmc4xxx_get_sector_start_addr(struct flash_bank *bank,
		unsigned int sector, uint32_t *ret_addr)
{
	/* Make sure we understand this sector */
	if (sector > bank->num_sectors)
		return ERROR_FAIL;

	*ret_addr = bank->base + bank->sectors[sector].offset;

	return ERROR_OK;

}

static int xmc4xxx_clear_flash_status(struct flash_bank *bank)
{
	int res;
	/* TODO: Do we need to check for sequence error? */
	LOG_INFO("Clearing flash status");
	res = target_write_u32(bank->target, FLASH_CMD_CLEAR_STATUS,
			       0xF5);
	if (res != ERROR_OK) {
		LOG_ERROR("Unable to write erase command sequence");
		return res;
	}

	return ERROR_OK;
}

static int xmc4xxx_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	int res;

	res = target_read_u32(bank->target, FLASH_REG_FLASH0_FSR, status);

	if (res != ERROR_OK)
		LOG_ERROR("Cannot read flash status register.");

	return res;
}

static int xmc4xxx_wait_status_busy(struct flash_bank *bank, int timeout)
{
	int res;
	uint32_t status;

	res = xmc4xxx_get_flash_status(bank, &status);
	if (res != ERROR_OK)
		return res;

	/* While the flash controller is busy, wait */
	while (status & FSR_PBUSY_MASK) {
		res = xmc4xxx_get_flash_status(bank, &status);
		if (res != ERROR_OK)
			return res;

		if (timeout-- <= 0) {
			LOG_ERROR("Timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
		keep_alive();
	}

	if (status & FSR_PROER_MASK) {
		LOG_ERROR("XMC4xxx flash protected");
		res = ERROR_FAIL;
	}

	return res;
}

static int xmc4xxx_erase_sector(struct flash_bank *bank, uint32_t address,
				bool user_config)
{
	int res;
	uint32_t status;

	/* See reference manual table 8.4: Command Sequences for Flash Control */
	struct xmc4xxx_command_seq erase_cmd_seq[6] = {
		{FLASH_CMD_ERASE_1, 0xAA},
		{FLASH_CMD_ERASE_2, 0x55},
		{FLASH_CMD_ERASE_3, 0x80},
		{FLASH_CMD_ERASE_4, 0xAA},
		{FLASH_CMD_ERASE_5, 0x55},
		{0xFF,              0xFF} /* Needs filled in */
	};

	/* We need to fill in the base address of the sector we'll be
	 * erasing, as well as the magic code that determines whether
	 * this is a standard flash sector or a user configuration block */

	erase_cmd_seq[5].address = address;
	if (user_config) {
		/* Removing flash protection requires the addition of
		 * the base address */
		erase_cmd_seq[5].address += bank->base;
		erase_cmd_seq[5].magic = 0xC0;
	} else {
		erase_cmd_seq[5].magic = 0x30;
	}

	res = xmc4xxx_write_command_sequence(bank, erase_cmd_seq,
					     ARRAY_SIZE(erase_cmd_seq));
	if (res != ERROR_OK)
		return res;

	/* Read the flash status register */
	res = target_read_u32(bank->target, FLASH_REG_FLASH0_FSR, &status);
	if (res != ERROR_OK) {
		LOG_ERROR("Cannot read flash status register.");
		return res;
	}

	/* Check for a sequence error */
	if (status & FSR_SQER_MASK) {
		LOG_ERROR("Error with flash erase sequence");
		return ERROR_FAIL;
	}

	/* Make sure a flash erase was triggered */
	if (!(status & FSR_ERASE_MASK)) {
		LOG_ERROR("Flash failed to erase");
		return ERROR_FAIL;
	}

	/* Now we must wait for the erase operation to end */
	res = xmc4xxx_wait_status_busy(bank, FLASH_OP_TIMEOUT);

	return res;
}

static int xmc4xxx_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct xmc4xxx_flash_bank *fb = bank->driver_priv;
	int res;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Unable to erase, target is not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!fb->probed) {
		res = xmc4xxx_probe(bank);
		if (res != ERROR_OK)
			return res;
	}

	uint32_t tmp_addr;
	/* Loop through the sectors and erase each one */
	for (unsigned int i = first; i <= last; i++) {
		res = xmc4xxx_get_sector_start_addr(bank, i, &tmp_addr);
		if (res != ERROR_OK) {
			LOG_ERROR("Invalid sector %u", i);
			return res;
		}

		LOG_DEBUG("Erasing sector %u @ 0x%08"PRIx32, i, tmp_addr);

		res = xmc4xxx_erase_sector(bank, tmp_addr, false);
		if (res != ERROR_OK) {
			LOG_ERROR("Unable to write erase command sequence");
			goto clear_status_and_exit;
		}

		/* Now we must wait for the erase operation to end */
		res = xmc4xxx_wait_status_busy(bank, FLASH_OP_TIMEOUT);

		if (res != ERROR_OK)
			goto clear_status_and_exit;

		bank->sectors[i].is_erased = 1;
	}

clear_status_and_exit:
	res = xmc4xxx_clear_flash_status(bank);
	return res;

}

static int xmc4xxx_enter_page_mode(struct flash_bank *bank)
{
	int res;
	uint32_t status;

	res = target_write_u32(bank->target, FLASH_CMD_ENTER_PAGEMODE, 0x50);
	if (res != ERROR_OK) {
		LOG_ERROR("Unable to write enter page mode command");
		return ERROR_FAIL;
	}

	res = xmc4xxx_get_flash_status(bank, &status);

	if (res != ERROR_OK)
		return res;

	/* Make sure we're in page mode */
	if (!(status & FSR_PFPAGE_MASK)) {
		LOG_ERROR("Unable to enter page mode");
		return ERROR_FAIL;
	}

	/* Make sure we didn't encounter a sequence error */
	if (status & FSR_SQER_MASK) {
		LOG_ERROR("Sequence error while entering page mode");
		return ERROR_FAIL;
	}

	return res;
}

static int xmc4xxx_write_page(struct flash_bank *bank, const uint8_t *pg_buf,
			      uint32_t offset, bool user_config)
{
	int res;
	uint32_t status;

	/* Base of the flash write command */
	struct xmc4xxx_command_seq write_cmd_seq[4] = {
		{FLASH_CMD_WRITE_PAGE_1, 0xAA},
		{FLASH_CMD_WRITE_PAGE_2, 0x55},
		{FLASH_CMD_WRITE_PAGE_3, 0xFF}, /* Needs filled in */
		{0xFF,                   0xFF}	/* Needs filled in */
	};

	/* The command sequence differs depending on whether this is
	 * being written to standard flash or the user configuration
	 * area */
	if (user_config)
		write_cmd_seq[2].magic = 0xC0;
	else
		write_cmd_seq[2].magic = 0xA0;

	/* Finally, we need to add the address that this page will be
	 * written to */
	write_cmd_seq[3].address = bank->base + offset;
	write_cmd_seq[3].magic = 0xAA;


	/* Flash pages are written 256 bytes at a time.  For each 256
	 * byte chunk, we need to:
	 * 1. Enter page mode. This activates the flash write buffer
	 * 2. Load the page buffer with data (2x 32 bit words at a time)
	 * 3. Burn the page buffer into its intended location
	 * If the starting offset is not on a 256 byte boundary, we
	 * will need to pad the beginning of the write buffer
	 * accordingly. Likewise, if the last page does not fill the
	 * buffer, we should pad it to avoid leftover data from being
	 * written to flash
	 */
	res = xmc4xxx_enter_page_mode(bank);
	if (res != ERROR_OK)
		return res;

	/* Copy the data into the page buffer*/
	for (int i = 0; i < 256; i += 8) {
		uint32_t w_lo = target_buffer_get_u32(bank->target, &pg_buf[i]);
		uint32_t w_hi = target_buffer_get_u32(bank->target, &pg_buf[i + 4]);
		LOG_DEBUG("WLO: %08"PRIx32, w_lo);
		LOG_DEBUG("WHI: %08"PRIx32, w_hi);

		/* Data is loaded 2x 32 bit words at a time */
		res = target_write_u32(bank->target, FLASH_CMD_LOAD_PAGE_1, w_lo);
		if (res != ERROR_OK)
			return res;

		res = target_write_u32(bank->target, FLASH_CMD_LOAD_PAGE_2, w_hi);
		if (res != ERROR_OK)
			return res;

		/* Check for an error */
		res = xmc4xxx_get_flash_status(bank, &status);
		if (res != ERROR_OK)
			return res;

		if (status & FSR_SQER_MASK) {
			LOG_ERROR("Error loading page buffer");
			return ERROR_FAIL;
		}
	}

	/* The page buffer is now full, time to commit it to flash */

	res = xmc4xxx_write_command_sequence(bank, write_cmd_seq, ARRAY_SIZE(write_cmd_seq));
	if (res != ERROR_OK) {
		LOG_ERROR("Unable to enter write command sequence");
		return res;
	}

	/* Read the flash status register */
	res = xmc4xxx_get_flash_status(bank, &status);
	if (res != ERROR_OK)
		return res;

	/* Check for a sequence error */
	if (status & FSR_SQER_MASK) {
		LOG_ERROR("Error with flash write sequence");
		return ERROR_FAIL;
	}

	/* Make sure a flash write was triggered */
	if (!(status & FSR_PROG_MASK)) {
		LOG_ERROR("Failed to write flash page");
		return ERROR_FAIL;
	}

	/* Wait for the write operation to end */
	res = xmc4xxx_wait_status_busy(bank, FLASH_OP_TIMEOUT);
	if (res != ERROR_OK)
		return res;

	/* TODO: Verify that page was written without error */
	return res;
}

static int xmc4xxx_write(struct flash_bank *bank, const uint8_t *buffer,
			 uint32_t offset, uint32_t count)
{
	struct xmc4xxx_flash_bank *fb = bank->driver_priv;
	int res = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Unable to erase, target is not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!fb->probed) {
		res = xmc4xxx_probe(bank);
		if (res != ERROR_OK)
			return res;
	}

	/* Make sure we won't run off the end of the flash bank */
	if ((offset + count) > (bank->size)) {
		LOG_ERROR("Attempting to write past the end of flash");
		return ERROR_FAIL;
	}


	/* Attempt to write the passed in buffer to flash */
	/* Pages are written 256 bytes at a time, we need to handle
	 * scenarios where padding is required at the beginning and
	 * end of a page */
	while (count) {
		/* page working area */
		uint8_t tmp_buf[256] = {0};

		/* Amount of data we'll be writing to this page */
		int remaining;
		int end_pad;

		remaining = MIN(count, sizeof(tmp_buf));
		end_pad   = sizeof(tmp_buf) - remaining;

		/* Make sure we're starting on a page boundary */
		int start_pad = offset % 256;
		if (start_pad) {
			LOG_INFO("Write does not start on a 256 byte boundary. "
				 "Padding by %d bytes", start_pad);
			memset(tmp_buf, 0xff, start_pad);
			/* Subtract the amount of start offset from
			 * the amount of data we'll need to write */
			remaining -= start_pad;
		}

		/* Remove the amount we'll be writing from the total count */
		count -= remaining;

		/* Now copy in the remaining data */
		memcpy(&tmp_buf[start_pad], buffer, remaining);

		if (end_pad) {
			LOG_INFO("Padding end of page @" TARGET_ADDR_FMT " by %d bytes",
				 bank->base + offset, end_pad);
			memset(&tmp_buf[256 - end_pad], 0xff, end_pad);
		}

		/* Now commit this page to flash, if there was start
		 * padding, we should subtract that from the target offset */
		res = xmc4xxx_write_page(bank, tmp_buf, (offset - start_pad), false);
		if (res != ERROR_OK) {
			LOG_ERROR("Unable to write flash page");
			goto abort_write_and_exit;
		}

		/* Advance the buffer pointer */
		buffer += remaining;

		/* Advance the offset */
		offset += remaining;
	}

abort_write_and_exit:
	xmc4xxx_clear_flash_status(bank);
	return res;

}

static int xmc4xxx_get_info_command(struct flash_bank *bank, char *buf, int buf_size)
{
	struct xmc4xxx_flash_bank *fb = bank->driver_priv;
	uint32_t scu_idcode;

	if (bank->target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The SCU registers contain the ID of the chip */
	int res = target_read_u32(bank->target, SCU_REG_BASE + SCU_ID_CHIP, &scu_idcode);
	if (res != ERROR_OK) {
		LOG_ERROR("Cannot read device identification register.");
		return res;
	}

	uint16_t dev_id = (scu_idcode & 0xfff0) >> 4;
	uint16_t rev_id = scu_idcode & 0xf;
	const char *dev_str;
	const char *rev_str = NULL;

	switch (dev_id) {
	case 0x100:
		dev_str = "XMC4100";

		switch (rev_id) {
		case 0x1:
			rev_str = "AA";
			break;
		case 0x2:
			rev_str = "AB";
			break;
		}
		break;
	case 0x200:
		dev_str = "XMC4200";

		switch (rev_id) {
		case 0x1:
			rev_str = "AA";
			break;
		case 0x2:
			rev_str = "AB";
			break;
		}
		break;
	case 0x300:
		dev_str = "XMC4300";

		switch (rev_id) {
		case 0x1:
			rev_str = "AA";
		}
		break;
	case 0x400:
		dev_str = "XMC4400";

		switch (rev_id) {
		case 0x1:
			rev_str = "AA";
			break;
		case 0x2:
			rev_str = "AB";
			break;
		}
		break;
	case 0:
		/* XMC4500 EES AA13 with date codes before GE212
		 * had zero SCU_IDCHIP
		 */
		dev_str = "XMC4500 EES";
		rev_str = "AA13";
		break;
	case 0x500:
		dev_str = "XMC4500";

		switch (rev_id) {
		case 0x2:
			rev_str = "AA";
			break;
		case 0x3:
			rev_str = "AB";
			break;
		case 0x4:
			rev_str = "AC";
			break;
		}
		break;
	case 0x700:
		dev_str = "XMC4700";

		switch (rev_id) {
		case 0x1:
			rev_str = "EES-AA";
			break;
		}
		break;
	case 0x800:
		dev_str = "XMC4800";

		switch (rev_id) {
		case 0x1:
			rev_str = "EES-AA";
			break;
		}
		break;

	default:
		snprintf(buf, buf_size,
			 "Cannot identify target as an XMC4xxx. SCU_ID: %"PRIx32"\n",
			 scu_idcode);
		return ERROR_OK;
	}

	/* String to declare protection data held in the private driver */
	char prot_str[512] = {0};
	if (fb->read_protected)
		snprintf(prot_str, sizeof(prot_str), "\nFlash is read protected");

	bool otp_enabled = false;
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		if (fb->write_prot_otp[i])
			otp_enabled = true;

	/* If OTP Write protection is enabled (User 2), list each
	 * sector that has it enabled */
	char otp_str[14];
	if (otp_enabled) {
		strcat(prot_str, "\nOTP Protection is enabled for sectors:\n");
		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			if (fb->write_prot_otp[i]) {
				snprintf(otp_str, sizeof(otp_str), "- %d\n", i);
				strncat(prot_str, otp_str, sizeof(prot_str) - strlen(prot_str) - 1);
			}
		}
	}

	if (rev_str != NULL)
		snprintf(buf, buf_size, "%s - Rev: %s%s",
			 dev_str, rev_str, prot_str);
	else
		snprintf(buf, buf_size, "%s - Rev: unknown (0x%01x)%s",
			 dev_str, rev_id, prot_str);

	return ERROR_OK;
}

static int xmc4xxx_temp_unprotect(struct flash_bank *bank, int user_level)
{
	struct xmc4xxx_flash_bank *fb;
	int res = ERROR_OK;
	uint32_t status = 0;

	struct xmc4xxx_command_seq temp_unprot_seq[6] = {
		{FLASH_CMD_TEMP_UNPROT_1, 0xAA},
		{FLASH_CMD_TEMP_UNPROT_2, 0x55},
		{FLASH_CMD_TEMP_UNPROT_3, 0xFF}, /* Needs filled in */
		{FLASH_CMD_TEMP_UNPROT_4, 0xFF}, /* Needs filled in */
		{FLASH_CMD_TEMP_UNPROT_5, 0xFF}, /* Needs filled in */
		{FLASH_CMD_TEMP_UNPROT_6, 0x05}
	};

	if (user_level < 0 || user_level > 2) {
		LOG_ERROR("Invalid user level, must be 0-2");
		return ERROR_FAIL;
	}

	fb = bank->driver_priv;

	/* Fill in the user level and passwords */
	temp_unprot_seq[2].magic = user_level;
	temp_unprot_seq[3].magic = fb->pw1;
	temp_unprot_seq[4].magic = fb->pw2;

	res = xmc4xxx_write_command_sequence(bank, temp_unprot_seq,
					     ARRAY_SIZE(temp_unprot_seq));
	if (res != ERROR_OK) {
		LOG_ERROR("Unable to write temp unprotect sequence");
		return res;
	}

	res = xmc4xxx_get_flash_status(bank, &status);
	if (res != ERROR_OK)
		return res;

	if (status & FSR_WPRODIS0) {
		LOG_INFO("Flash is temporarily unprotected");
	} else {
		LOG_INFO("Unable to disable flash protection");
		res = ERROR_FAIL;
	}


	return res;
}

static int xmc4xxx_flash_unprotect(struct flash_bank *bank, int32_t level)
{
	uint32_t addr;
	int res;

	switch (level) {
	case 0:
		addr = UCB0_BASE;
		break;
	case 1:
		addr = UCB1_BASE;
		break;
	default:
		LOG_ERROR("Invalid user level. Must be 0-1");
		return ERROR_FAIL;
	}

	res = xmc4xxx_erase_sector(bank, addr, true);

	if (res != ERROR_OK)
		LOG_ERROR("Error erasing user configuration block");

	return res;
}

/* Reference: "XMC4500 Flash Protection.pptx" app note */
static int xmc4xxx_flash_protect(struct flash_bank *bank, int level, bool read_protect,
		unsigned int first, unsigned int last)
{
	/* User configuration block buffers */
	uint8_t ucp0_buf[8 * sizeof(uint32_t)] = {0};
	uint32_t ucb_base = 0;
	uint32_t procon = 0;
	int res = ERROR_OK;
	uint32_t status = 0;
	bool proin = false;

	struct xmc4xxx_flash_bank *fb = bank->driver_priv;

	/* Read protect only works for user 0, make sure we don't try
	 * to do something silly */
	if (level != 0 && read_protect) {
		LOG_ERROR("Read protection is for user level 0 only!");
		return ERROR_FAIL;
	}

	/* Check to see if protection is already installed for the
	 * specified user level.  If it is, the user configuration
	 * block will need to be erased before we can continue */

	/* Grab the flash status register*/
	res = xmc4xxx_get_flash_status(bank, &status);
	if (res != ERROR_OK)
		return res;

	switch (level) {
	case 0:
		if ((status & FSR_RPROIN_MASK) || (status & FSR_WPROIN0_MASK))
			proin = true;
		break;
	case 1:
		if (status & FSR_WPROIN1_MASK)
			proin = true;
		break;
	case 2:
		if (status & FSR_WPROIN2_MASK)
			proin = true;
		break;
	}

	if (proin) {
		LOG_ERROR("Flash protection is installed for user %d"
			  " and must be removed before continuing", level);
		return ERROR_FAIL;
	}

	/* If this device has 12 flash sectors, protection for
	 * sectors 10 & 11 are handled jointly. If we are trying to
	 * write all sectors, we should decrement
	 * last to ensure we don't write to a register bit that
	 * doesn't exist*/
	if ((bank->num_sectors == 12) && (last == 12))
		last--;

	/*  We need to fill out the procon register representation
	 *   that we will be writing to the device */
	for (unsigned int i = first; i <= last; i++)
		procon |= 1 << i;

	/* If read protection is requested, set the appropriate bit
	 * (we checked that this is allowed above) */
	if (read_protect)
		procon |= PROCON_RPRO_MASK;

	LOG_DEBUG("Setting flash protection with procon:");
	LOG_DEBUG("PROCON: %"PRIx32, procon);

	/* First we need to copy in the procon register to the buffer
	 * we're going to attempt to write.  This is written twice */
	target_buffer_set_u32(bank->target, &ucp0_buf[0 * 4], procon);
	target_buffer_set_u32(bank->target, &ucp0_buf[2 * 4], procon);

	/* Now we must copy in both flash passwords.  As with the
	 * procon data, this must be written twice (4 total words
	 * worth of data) */
	target_buffer_set_u32(bank->target, &ucp0_buf[4 * 4], fb->pw1);
	target_buffer_set_u32(bank->target, &ucp0_buf[5 * 4], fb->pw2);
	target_buffer_set_u32(bank->target, &ucp0_buf[6 * 4], fb->pw1);
	target_buffer_set_u32(bank->target, &ucp0_buf[7 * 4], fb->pw2);

	/* Finally, (if requested) we copy in the confirmation
	 * code so that the protection is permanent and will
	 * require a password to undo. */
	target_buffer_set_u32(bank->target, &ucp0_buf[0 * 4], FLASH_PROTECT_CONFIRMATION_CODE);
	target_buffer_set_u32(bank->target, &ucp0_buf[2 * 4], FLASH_PROTECT_CONFIRMATION_CODE);

	/* Now that the data is copied into place, we must write
	 * these pages into flash */

	/* The user configuration block base depends on what level of
	 * protection we're trying to install, select the proper one */
	switch (level) {
	case 0:
		ucb_base = UCB0_BASE;
		break;
	case 1:
		ucb_base = UCB1_BASE;
		break;
	case 2:
		ucb_base = UCB2_BASE;
		break;
	}

	/* Write the user config pages */
	res = xmc4xxx_write_page(bank, ucp0_buf, ucb_base, true);
	if (res != ERROR_OK) {
		LOG_ERROR("Error writing user configuration block 0");
		return res;
	}

	return ERROR_OK;
}

static int xmc4xxx_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	int ret;
	struct xmc4xxx_flash_bank *fb = bank->driver_priv;

	/* Check for flash passwords */
	if (!fb->pw_set) {
		LOG_ERROR("Flash passwords not set, use xmc4xxx flash_password to set them");
		return ERROR_FAIL;
	}

	/* We want to clear flash protection temporarily*/
	if (set == 0) {
		LOG_WARNING("Flash protection will be temporarily disabled"
			    " for all pages (User 0 only)!");
		ret = xmc4xxx_temp_unprotect(bank, 0);
		return ret;
	}

	/* Install write protection for user 0 on the specified pages */
	ret = xmc4xxx_flash_protect(bank, 0, false, first, last);

	return ret;
}

static int xmc4xxx_protect_check(struct flash_bank *bank)
{
	int ret;
	uint32_t protection[3] = {0};
	struct xmc4xxx_flash_bank *fb = bank->driver_priv;

	ret = target_read_u32(bank->target, FLASH_REG_FLASH0_PROCON0, &protection[0]);
	if (ret != ERROR_OK) {
		LOG_ERROR("Unable to read flash User0 protection register");
		return ret;
	}

	ret = target_read_u32(bank->target, FLASH_REG_FLASH0_PROCON1, &protection[1]);
	if (ret != ERROR_OK) {
		LOG_ERROR("Unable to read flash User1 protection register");
		return ret;
	}

	ret = target_read_u32(bank->target, FLASH_REG_FLASH0_PROCON2, &protection[2]);
	if (ret != ERROR_OK) {
		LOG_ERROR("Unable to read flash User2 protection register");
		return ret;
	}

	unsigned int sectors = bank->num_sectors;

	/* On devices with 12 sectors, sectors 10 & 11 are protected
	 * together instead of individually */
	if (sectors == 12)
		sectors--;

	/* Clear the protection status */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].is_protected = 0;
		fb->write_prot_otp[i] = false;
	}
	fb->read_protected = false;

	/* The xmc4xxx series supports 3 levels of user protection
	 * (User0, User1 (low priority), and User 2(OTP), we need to
	 * check all 3 */
	for (unsigned int i = 0; i < ARRAY_SIZE(protection); i++) {

		/* Check for write protection on every available
		*  sector */
		for (unsigned int j = 0; j < sectors; j++) {
			int set = (protection[i] & (1 << j)) ? 1 : 0;
			bank->sectors[j].is_protected |= set;

			/* Handle sector 11 */
			if (j == 10)
				bank->sectors[j + 1].is_protected |= set;

			/* User 2 indicates this protection is
			 * permanent, make note in the private driver structure */
			if (i == 2 && set) {
				fb->write_prot_otp[j] = true;

				/* Handle sector 11 */
				if (j == 10)
					fb->write_prot_otp[j + 1] = true;
			}

		}
	}

	/* XMC4xxx also supports read protection, make a note
	 * in the private driver structure */
	if (protection[0] & PROCON_RPRO_MASK)
		fb->read_protected = true;

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(xmc4xxx_flash_bank_command)
{
	bank->driver_priv = malloc(sizeof(struct xmc4xxx_flash_bank));

	if (!bank->driver_priv)
		return ERROR_FLASH_OPERATION_FAILED;

	(void)memset(bank->driver_priv, 0, sizeof(struct xmc4xxx_flash_bank));

	return ERROR_OK;
}

COMMAND_HANDLER(xmc4xxx_handle_flash_password_command)
{
	int res;
	struct flash_bank *bank;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	res = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (res != ERROR_OK)
		return res;

	struct xmc4xxx_flash_bank *fb = bank->driver_priv;

	errno = 0;

	/* We skip over the flash bank */
	fb->pw1 = strtol(CMD_ARGV[1], NULL, 16);

	if (errno)
		return ERROR_COMMAND_SYNTAX_ERROR;

	fb->pw2 = strtol(CMD_ARGV[2], NULL, 16);

	if (errno)
		return ERROR_COMMAND_SYNTAX_ERROR;

	fb->pw_set = true;

	command_print(CMD, "XMC4xxx flash passwords set to:\n");
	command_print(CMD, "-0x%08"PRIx32"\n", fb->pw1);
	command_print(CMD, "-0x%08"PRIx32"\n", fb->pw2);
	return ERROR_OK;
}

COMMAND_HANDLER(xmc4xxx_handle_flash_unprotect_command)
{
	struct flash_bank *bank;
	int res;
	int32_t level;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	res = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (res != ERROR_OK)
		return res;

	COMMAND_PARSE_NUMBER(s32, CMD_ARGV[1], level);

	res = xmc4xxx_flash_unprotect(bank, level);

	return res;
}

static const struct command_registration xmc4xxx_exec_command_handlers[] = {
	{
		.name = "flash_password",
		.handler = xmc4xxx_handle_flash_password_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id password1 password2",
		.help = "Set the flash passwords used for protect operations. "
		"Passwords should be in standard hex form (0x00000000). "
		"(You must call this before any other protect commands) "
		"NOTE: The xmc4xxx's UCB area only allows for FOUR cycles. "
		"Please use protection carefully!",
	},
	{
		.name = "flash_unprotect",
		.handler = xmc4xxx_handle_flash_unprotect_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id user_level[0-1]",
		.help = "Permanently Removes flash protection (read and write) "
		"for the specified user level",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration xmc4xxx_command_handlers[] = {
	{
		.name = "xmc4xxx",
		.mode = COMMAND_ANY,
		.help = "xmc4xxx flash command group",
		.usage = "",
		.chain = xmc4xxx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver xmc4xxx_flash = {
	.name = "xmc4xxx",
	.commands = xmc4xxx_command_handlers,
	.flash_bank_command = xmc4xxx_flash_bank_command,
	.erase = xmc4xxx_erase,
	.write = xmc4xxx_write,
	.read = default_flash_read,
	.probe = xmc4xxx_probe,
	.auto_probe = xmc4xxx_probe,
	.erase_check = default_flash_blank_check,
	.info = xmc4xxx_get_info_command,
	.protect_check = xmc4xxx_protect_check,
	.protect = xmc4xxx_protect,
	.free_driver_priv = default_flash_free_driver_priv,
};
