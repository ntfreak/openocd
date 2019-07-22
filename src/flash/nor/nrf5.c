/***************************************************************************
 *   Copyright (C) 2013 Synapse Product Development                        *
 *   Andrey Smirnov <andrew.smironv@gmail.com>                             *
 *   Angus Gratton <gus@projectgus.com>                                    *
 *   Erdem U. Altunyurt <spamjunkeater@gmail.com>                          *
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
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <helper/types.h>
#include <helper/time_support.h>

/* Both those values are constant across the current spectrum ofr nRF5 devices */
#define WATCHDOG_REFRESH_REGISTER       0x40010600
#define WATCHDOG_REFRESH_VALUE          0x6e524635

enum {
	NRF5_FLASH_BASE = 0x00000000,
};

enum nrf5_ficr_registers {
	NRF5_FICR_BASE = 0x10000000, /* Factory Information Configuration Registers */

#define NRF5_FICR_REG(offset) (NRF5_FICR_BASE + offset)

	NRF5_FICR_CODEPAGESIZE		= NRF5_FICR_REG(0x010),
	NRF5_FICR_CODESIZE		= NRF5_FICR_REG(0x014),
	NRF5_FICR_CLENR0		= NRF5_FICR_REG(0x028),
	NRF5_FICR_PPFC			= NRF5_FICR_REG(0x02C),
	NRF5_FICR_NUMRAMBLOCK		= NRF5_FICR_REG(0x034),
	NRF5_FICR_SIZERAMBLOCK0	= NRF5_FICR_REG(0x038),
	NRF5_FICR_SIZERAMBLOCK1	= NRF5_FICR_REG(0x03C),
	NRF5_FICR_SIZERAMBLOCK2	= NRF5_FICR_REG(0x040),
	NRF5_FICR_SIZERAMBLOCK3	= NRF5_FICR_REG(0x044),
	NRF5_FICR_CONFIGID		= NRF5_FICR_REG(0x05C),
	NRF5_FICR_DEVICEID0		= NRF5_FICR_REG(0x060),
	NRF5_FICR_DEVICEID1		= NRF5_FICR_REG(0x064),
	NRF5_FICR_ER0			= NRF5_FICR_REG(0x080),
	NRF5_FICR_ER1			= NRF5_FICR_REG(0x084),
	NRF5_FICR_ER2			= NRF5_FICR_REG(0x088),
	NRF5_FICR_ER3			= NRF5_FICR_REG(0x08C),
	NRF5_FICR_IR0			= NRF5_FICR_REG(0x090),
	NRF5_FICR_IR1			= NRF5_FICR_REG(0x094),
	NRF5_FICR_IR2			= NRF5_FICR_REG(0x098),
	NRF5_FICR_IR3			= NRF5_FICR_REG(0x09C),
	NRF5_FICR_DEVICEADDRTYPE	= NRF5_FICR_REG(0x0A0),
	NRF5_FICR_DEVICEADDR0		= NRF5_FICR_REG(0x0A4),
	NRF5_FICR_DEVICEADDR1		= NRF5_FICR_REG(0x0A8),
	NRF5_FICR_OVERRIDEN		= NRF5_FICR_REG(0x0AC),
	NRF5_FICR_NRF_1MBIT0		= NRF5_FICR_REG(0x0B0),
	NRF5_FICR_NRF_1MBIT1		= NRF5_FICR_REG(0x0B4),
	NRF5_FICR_NRF_1MBIT2		= NRF5_FICR_REG(0x0B8),
	NRF5_FICR_NRF_1MBIT3		= NRF5_FICR_REG(0x0BC),
	NRF5_FICR_NRF_1MBIT4		= NRF5_FICR_REG(0x0C0),
	NRF5_FICR_BLE_1MBIT0		= NRF5_FICR_REG(0x0EC),
	NRF5_FICR_BLE_1MBIT1		= NRF5_FICR_REG(0x0F0),
	NRF5_FICR_BLE_1MBIT2		= NRF5_FICR_REG(0x0F4),
	NRF5_FICR_BLE_1MBIT3		= NRF5_FICR_REG(0x0F8),
	NRF5_FICR_BLE_1MBIT4		= NRF5_FICR_REG(0x0FC),
};

enum nrf5_uicr_registers {
	NRF5_UICR_BASE = 0x10001000, /* User Information
				       * Configuration Regsters */

	NRF5_UICR_SIZE = 0x100,

#define NRF5_UICR_REG(offset) (NRF5_UICR_BASE + offset)

	NRF5_UICR_CLENR0	= NRF5_UICR_REG(0x000),
	NRF5_UICR_RBPCONF	= NRF5_UICR_REG(0x004),
	NRF5_UICR_XTALFREQ	= NRF5_UICR_REG(0x008),
	NRF5_UICR_FWID		= NRF5_UICR_REG(0x010),
};

enum nrf5_nvmc_registers {
	NRF5_NVMC_BASE = 0x4001E000, /* Non-Volatile Memory
				       * Controller Regsters */

#define NRF5_NVMC_REG(offset) (NRF5_NVMC_BASE + offset)

	NRF5_NVMC_READY	= NRF5_NVMC_REG(0x400),
	NRF5_NVMC_CONFIG	= NRF5_NVMC_REG(0x504),
	NRF5_NVMC_ERASEPAGE	= NRF5_NVMC_REG(0x508),
	NRF5_NVMC_ERASEALL	= NRF5_NVMC_REG(0x50C),
	NRF5_NVMC_ERASEUICR	= NRF5_NVMC_REG(0x514),
};

enum nrf5_nvmc_config_bits {
	NRF5_NVMC_CONFIG_REN = 0x00,
	NRF5_NVMC_CONFIG_WEN = 0x01,
	NRF5_NVMC_CONFIG_EEN = 0x02,

};

struct nrf5_info {
	uint32_t code_page_size;
	uint32_t refcount;

	struct {
		bool probed;
		int (*write) (struct flash_bank *bank,
			      struct nrf5_info *chip,
			      const uint8_t *buffer, uint32_t offset, uint32_t count);
	} bank[2];
	struct target *target;
};

struct nrf5_device_spec {
	uint16_t hwid;
	const char *part;
	const char *variant;
	const char *build_code;
	unsigned int flash_size_kb;
};

#define NRF5_DEVICE_DEF(id, pt, var, bcode, fsize) \
{                                                   \
.hwid          = (id),                              \
.part          = pt,                                \
.variant       = var,                               \
.build_code    = bcode,                             \
.flash_size_kb = (fsize),                           \
}

/* The known devices table below is derived from the "nRF51 Series
 * Compatibility Matrix" document, which can be found by searching for
 * ATTN-51 on the Nordic Semi website:
 *
 * http://www.nordicsemi.com/eng/content/search?SearchText=ATTN-51
 *
 * Up to date with Matrix v2.0, plus some additional HWIDs.
 *
 * The additional HWIDs apply where the build code in the matrix is
 * shown as Gx0, Bx0, etc. In these cases the HWID in the matrix is
 * for x==0, x!=0 means different (unspecified) HWIDs.
 */
static const struct nrf5_device_spec nrf5_known_devices_table[] = {
	/* nRF51822 Devices (IC rev 1). */
	NRF5_DEVICE_DEF(0x001D, "51822", "QFAA", "CA/C0", 256),
	NRF5_DEVICE_DEF(0x0026, "51822", "QFAB", "AA",    128),
	NRF5_DEVICE_DEF(0x0027, "51822", "QFAB", "A0",    128),
	NRF5_DEVICE_DEF(0x0020, "51822", "CEAA", "BA",    256),
	NRF5_DEVICE_DEF(0x002F, "51822", "CEAA", "B0",    256),

	/* Some early nRF51-DK (PCA10028) & nRF51-Dongle (PCA10031) boards
	   with built-in jlink seem to use engineering samples not listed
	   in the nRF51 Series Compatibility Matrix V1.0. */
	NRF5_DEVICE_DEF(0x0071, "51822", "QFAC", "AB",    256),

	/* nRF51822 Devices (IC rev 2). */
	NRF5_DEVICE_DEF(0x002A, "51822", "QFAA", "FA0",   256),
	NRF5_DEVICE_DEF(0x0044, "51822", "QFAA", "GC0",   256),
	NRF5_DEVICE_DEF(0x003C, "51822", "QFAA", "G0",    256),
	NRF5_DEVICE_DEF(0x0057, "51822", "QFAA", "G2",    256),
	NRF5_DEVICE_DEF(0x0058, "51822", "QFAA", "G3",    256),
	NRF5_DEVICE_DEF(0x004C, "51822", "QFAB", "B0",    128),
	NRF5_DEVICE_DEF(0x0040, "51822", "CEAA", "CA0",   256),
	NRF5_DEVICE_DEF(0x0047, "51822", "CEAA", "DA0",   256),
	NRF5_DEVICE_DEF(0x004D, "51822", "CEAA", "D00",   256),

	/* nRF51822 Devices (IC rev 3). */
	NRF5_DEVICE_DEF(0x0072, "51822", "QFAA", "H0",    256),
	NRF5_DEVICE_DEF(0x00D1, "51822", "QFAA", "H2",    256),
	NRF5_DEVICE_DEF(0x007B, "51822", "QFAB", "C0",    128),
	NRF5_DEVICE_DEF(0x0083, "51822", "QFAC", "A0",    256),
	NRF5_DEVICE_DEF(0x0084, "51822", "QFAC", "A1",    256),
	NRF5_DEVICE_DEF(0x007D, "51822", "CDAB", "A0",    128),
	NRF5_DEVICE_DEF(0x0079, "51822", "CEAA", "E0",    256),
	NRF5_DEVICE_DEF(0x0087, "51822", "CFAC", "A0",    256),
	NRF5_DEVICE_DEF(0x008F, "51822", "QFAA", "H1",    256),

	/* nRF51422 Devices (IC rev 1). */
	NRF5_DEVICE_DEF(0x001E, "51422", "QFAA", "CA",    256),
	NRF5_DEVICE_DEF(0x0024, "51422", "QFAA", "C0",    256),
	NRF5_DEVICE_DEF(0x0031, "51422", "CEAA", "A0A",   256),

	/* nRF51422 Devices (IC rev 2). */
	NRF5_DEVICE_DEF(0x002D, "51422", "QFAA", "DAA",   256),
	NRF5_DEVICE_DEF(0x002E, "51422", "QFAA", "E0",    256),
	NRF5_DEVICE_DEF(0x0061, "51422", "QFAB", "A00",   128),
	NRF5_DEVICE_DEF(0x0050, "51422", "CEAA", "B0",    256),

	/* nRF51422 Devices (IC rev 3). */
	NRF5_DEVICE_DEF(0x0073, "51422", "QFAA", "F0",    256),
	NRF5_DEVICE_DEF(0x007C, "51422", "QFAB", "B0",    128),
	NRF5_DEVICE_DEF(0x0085, "51422", "QFAC", "A0",    256),
	NRF5_DEVICE_DEF(0x0086, "51422", "QFAC", "A1",    256),
	NRF5_DEVICE_DEF(0x007E, "51422", "CDAB", "A0",    128),
	NRF5_DEVICE_DEF(0x007A, "51422", "CEAA", "C0",    256),
	NRF5_DEVICE_DEF(0x0088, "51422", "CFAC", "A0",    256),

	/* nRF52810 Devices */
	NRF5_DEVICE_DEF(0x0142, "52810", "QFAA", "B0",    192),
	NRF5_DEVICE_DEF(0x0143, "52810", "QCAA", "C0",    192),

	/* nRF52832 Devices */
	NRF5_DEVICE_DEF(0x00C7, "52832", "QFAA", "B0",    512),
	NRF5_DEVICE_DEF(0x0139, "52832", "QFAA", "E0",    512),
	NRF5_DEVICE_DEF(0x00E3, "52832", "CIAA", "B0",    512),

	/* nRF52840 Devices */
	NRF5_DEVICE_DEF(0x0150, "52840", "QIAA", "C0",    1024),
};

static int nrf5_bank_is_probed(struct flash_bank *bank)
{
	struct nrf5_info *chip = bank->driver_priv;

	assert(chip != NULL);

	return chip->bank[bank->bank_number].probed;
}
static int nrf5_probe(struct flash_bank *bank);

static int nrf5_get_probed_chip_if_halted(struct flash_bank *bank, struct nrf5_info **chip)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	*chip = bank->driver_priv;

	int probed = nrf5_bank_is_probed(bank);
	if (probed < 0)
		return probed;
	else if (!probed)
		return nrf5_probe(bank);
	else
		return ERROR_OK;
}

static int nrf5_wait_for_nvmc(struct nrf5_info *chip)
{
	uint32_t ready;
	int res;
	int timeout_ms = 340;
	int64_t ts_start = timeval_ms();

	do {
		res = target_read_u32(chip->target, NRF5_NVMC_READY, &ready);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read NVMC_READY register");
			return res;
		}

		if (ready == 0x00000001)
			return ERROR_OK;

		keep_alive();

	} while ((timeval_ms()-ts_start) < timeout_ms);

	LOG_DEBUG("Timed out waiting for NVMC_READY");
	return ERROR_FLASH_BUSY;
}

static int nrf5_nvmc_erase_enable(struct nrf5_info *chip)
{
	int res;
	res = target_write_u32(chip->target,
			       NRF5_NVMC_CONFIG,
			       NRF5_NVMC_CONFIG_EEN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable erase operation");
		return res;
	}

	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Erase enable did not complete");

	return res;
}

static int nrf5_nvmc_write_enable(struct nrf5_info *chip)
{
	int res;
	res = target_write_u32(chip->target,
			       NRF5_NVMC_CONFIG,
			       NRF5_NVMC_CONFIG_WEN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable write operation");
		return res;
	}

	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Write enable did not complete");

	return res;
}

static int nrf5_nvmc_read_only(struct nrf5_info *chip)
{
	int res;
	res = target_write_u32(chip->target,
			       NRF5_NVMC_CONFIG,
			       NRF5_NVMC_CONFIG_REN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable read-only operation");
		return res;
	}
	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Read only enable did not complete");

	return res;
}

static int nrf5_nvmc_generic_erase(struct nrf5_info *chip,
			       uint32_t erase_register, uint32_t erase_value)
{
	int res;

	res = nrf5_nvmc_erase_enable(chip);
	if (res != ERROR_OK)
		goto error;

	res = target_write_u32(chip->target,
			       erase_register,
			       erase_value);
	if (res != ERROR_OK)
		goto set_read_only;

	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		goto set_read_only;

	return nrf5_nvmc_read_only(chip);

set_read_only:
	nrf5_nvmc_read_only(chip);
error:
	LOG_ERROR("Failed to erase reg: 0x%08"PRIx32" val: 0x%08"PRIx32,
		  erase_register, erase_value);
	return ERROR_FAIL;
}

static int nrf5_protect_check(struct flash_bank *bank)
{
	int res;
	uint32_t clenr0;

	/* UICR cannot be write protected so just return early */
	if (bank->base == NRF5_UICR_BASE)
		return ERROR_OK;

	struct nrf5_info *chip = bank->driver_priv;

	assert(chip != NULL);

	res = target_read_u32(chip->target, NRF5_FICR_CLENR0,
			      &clenr0);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code region 0 size[FICR]");
		return res;
	}

	if (clenr0 == 0xFFFFFFFF) {
		res = target_read_u32(chip->target, NRF5_UICR_CLENR0,
				      &clenr0);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read code region 0 size[UICR]");
			return res;
		}
	}

	for (int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected =
			clenr0 != 0xFFFFFFFF && bank->sectors[i].offset < clenr0;

	return ERROR_OK;
}

static int nrf5_protect(struct flash_bank *bank, int set, int first, int last)
{
	int res;
	uint32_t clenr0, ppfc;
	struct nrf5_info *chip;

	/* UICR cannot be write protected so just bail out early */
	if (bank->base == NRF5_UICR_BASE)
		return ERROR_FAIL;

	res = nrf5_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	if (first != 0) {
		LOG_ERROR("Code region 0 must start at the begining of the bank");
		return ERROR_FAIL;
	}

	res = target_read_u32(chip->target, NRF5_FICR_PPFC,
			      &ppfc);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read PPFC register");
		return res;
	}

	if ((ppfc & 0xFF) == 0x00) {
		LOG_ERROR("Code region 0 size was pre-programmed at the factory, can't change flash protection settings");
		return ERROR_FAIL;
	}

	res = target_read_u32(chip->target, NRF5_UICR_CLENR0,
			      &clenr0);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code region 0 size[UICR]");
		return res;
	}

	if (clenr0 == 0xFFFFFFFF) {
		res = target_write_u32(chip->target, NRF5_UICR_CLENR0,
				       clenr0);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't write code region 0 size[UICR]");
			return res;
		}

	} else {
		LOG_ERROR("You need to perform chip erase before changing the protection settings");
	}

	nrf5_protect_check(bank);

	return ERROR_OK;
}

static int nrf5_probe(struct flash_bank *bank)
{
	uint32_t hwid;
	int res;
	struct nrf5_info *chip = bank->driver_priv;

	res = target_read_u32(chip->target, NRF5_FICR_CONFIGID, &hwid);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read CONFIGID register");
		return res;
	}

	hwid &= 0xFFFF;	/* HWID is stored in the lower two
			 * bytes of the CONFIGID register */

	const struct nrf5_device_spec *spec = NULL;
	for (size_t i = 0; i < ARRAY_SIZE(nrf5_known_devices_table); i++) {
		if (hwid == nrf5_known_devices_table[i].hwid) {
			spec = &nrf5_known_devices_table[i];
			break;
		}
	}

	if (!chip->bank[0].probed && !chip->bank[1].probed) {
		if (spec)
			LOG_INFO("nRF%s-%s(build code: %s) %ukB Flash",
				 spec->part, spec->variant, spec->build_code,
				 spec->flash_size_kb);
		else
			LOG_WARNING("Unknown device (HWID 0x%08" PRIx32 ")", hwid);
	}

	if (bank->base == NRF5_FLASH_BASE) {
		/* The value stored in NRF5_FICR_CODEPAGESIZE is the number of bytes in one page of FLASH. */
		res = target_read_u32(chip->target, NRF5_FICR_CODEPAGESIZE,
				&chip->code_page_size);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read code page size");
			return res;
		}

		/* Note the register name is misleading,
		 * NRF5_FICR_CODESIZE is the number of pages in flash memory, not the number of bytes! */
		uint32_t num_sectors;
		res = target_read_u32(chip->target, NRF5_FICR_CODESIZE, &num_sectors);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read code memory size");
			return res;
		}

		bank->num_sectors = num_sectors;
		bank->size = num_sectors * chip->code_page_size;

		if (spec && bank->size / 1024 != spec->flash_size_kb)
			LOG_WARNING("Chip's reported Flash capacity does not match expected one");

		bank->sectors = calloc(bank->num_sectors,
				       sizeof((bank->sectors)[0]));
		if (!bank->sectors)
			return ERROR_FLASH_BANK_NOT_PROBED;

		/* Fill out the sector information: all NRF5 sectors are the same size and
		 * there is always a fixed number of them. */
		for (int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].size = chip->code_page_size;
			bank->sectors[i].offset	= i * chip->code_page_size;

			/* mark as unknown */
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = -1;
		}

		nrf5_protect_check(bank);

		chip->bank[0].probed = true;
	} else {
		bank->size = NRF5_UICR_SIZE;
		bank->num_sectors = 1;
		bank->sectors = calloc(bank->num_sectors,
				       sizeof((bank->sectors)[0]));
		if (!bank->sectors)
			return ERROR_FLASH_BANK_NOT_PROBED;

		bank->sectors[0].size = bank->size;
		bank->sectors[0].offset	= 0;

		bank->sectors[0].is_erased = 0;
		bank->sectors[0].is_protected = 0;

		chip->bank[1].probed = true;
	}

	return ERROR_OK;
}

static int nrf5_auto_probe(struct flash_bank *bank)
{
	int probed = nrf5_bank_is_probed(bank);

	if (probed < 0)
		return probed;
	else if (probed)
		return ERROR_OK;
	else
		return nrf5_probe(bank);
}

static int nrf5_erase_all(struct nrf5_info *chip)
{
	LOG_DEBUG("Erasing all non-volatile memory");
	return nrf5_nvmc_generic_erase(chip,
					NRF5_NVMC_ERASEALL,
					0x00000001);
}

static int nrf5_erase_page(struct flash_bank *bank,
							struct nrf5_info *chip,
							struct flash_sector *sector)
{
	int res;

	LOG_DEBUG("Erasing page at 0x%"PRIx32, sector->offset);
	if (sector->is_protected) {
		LOG_ERROR("Cannot erase protected sector at 0x%" PRIx32, sector->offset);
		return ERROR_FAIL;
	}

	if (bank->base == NRF5_UICR_BASE) {
		uint32_t ppfc;
		res = target_read_u32(chip->target, NRF5_FICR_PPFC,
				      &ppfc);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read PPFC register");
			return res;
		}

		if ((ppfc & 0xFF) == 0xFF) {
			/* We can't erase the UICR.  Double-check to
			   see if it's already erased before complaining. */
			default_flash_blank_check(bank);
			if (sector->is_erased == 1)
				return ERROR_OK;

			LOG_ERROR("The chip was not pre-programmed with SoftDevice stack and UICR cannot be erased separately. Please issue mass erase before trying to write to this region");
			return ERROR_FAIL;
		}

		res = nrf5_nvmc_generic_erase(chip,
					       NRF5_NVMC_ERASEUICR,
					       0x00000001);


	} else {
		res = nrf5_nvmc_generic_erase(chip,
					       NRF5_NVMC_ERASEPAGE,
					       sector->offset);
	}

	return res;
}

/* Start a low level flash write for the specified region */
static int nrf5_ll_flash_write(struct nrf5_info *chip, uint32_t offset, const uint8_t *buffer, uint32_t bytes)
{
	struct target *target = chip->target;
	uint32_t buffer_size = 8192;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = NRF5_FLASH_BASE + offset;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t nrf5_flash_write_code[] = {
#include "../../../contrib/loaders/flash/nrf5/nrf5.inc"
	};

	LOG_DEBUG("Writing buffer to flash offset=0x%"PRIx32" bytes=0x%"PRIx32, offset, bytes);
	assert(bytes % 4 == 0);

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(nrf5_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, falling back to slow memory writes");

		for (; bytes > 0; bytes -= 4) {
			retval = target_write_memory(chip->target, offset, 4, 1, buffer);
			if (retval != ERROR_OK)
				return retval;

			retval = nrf5_wait_for_nvmc(chip);
			if (retval != ERROR_OK)
				return retval;

			offset += 4;
			buffer += 4;
		}

		return ERROR_OK;
	}

	LOG_WARNING("using fast async flash loader. This is currently supported");
	LOG_WARNING("only with ST-Link and CMSIS-DAP. If you have issues, add");
	LOG_WARNING("\"set WORKAREASIZE 0\" before sourcing nrf51.cfg/nrf52.cfg to disable it");

	retval = target_write_buffer(target, write_algorithm->address,
				sizeof(nrf5_flash_write_code),
				nrf5_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* free working area, write algorithm already allocated */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("No large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* byte count */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN_OUT);	/* target address */
	init_reg_param(&reg_params[4], "r6", 32, PARAM_IN);	/* watchdog refresh value */
	init_reg_param(&reg_params[5], "r7", 32, PARAM_IN);	/* watchdog refresh register address */

	buf_set_u32(reg_params[0].value, 0, 32, bytes);
	buf_set_u32(reg_params[1].value, 0, 32, source->address);
	buf_set_u32(reg_params[2].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[3].value, 0, 32, address);
	buf_set_u32(reg_params[4].value, 0, 32, WATCHDOG_REFRESH_VALUE);
	buf_set_u32(reg_params[5].value, 0, 32, WATCHDOG_REFRESH_REGISTER);

	retval = target_run_flash_async_algorithm(target, buffer, bytes/4, 4,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

	return retval;
}

/* Check and erase flash sectors in specified range then start a low level page write.
   start/end must be sector aligned.
*/
static int nrf5_write_pages(struct flash_bank *bank, uint32_t start, uint32_t end, const uint8_t *buffer)
{
	int res = ERROR_FAIL;
	struct nrf5_info *chip = bank->driver_priv;

	assert(start % chip->code_page_size == 0);
	assert(end % chip->code_page_size == 0);

	res = nrf5_nvmc_write_enable(chip);
	if (res != ERROR_OK)
		goto error;

	res = nrf5_ll_flash_write(chip, start, buffer, (end - start));
	if (res != ERROR_OK)
		goto error;

	return nrf5_nvmc_read_only(chip);

error:
	nrf5_nvmc_read_only(chip);
	LOG_ERROR("Failed to write to nrf5 flash");
	return res;
}

static int nrf5_erase(struct flash_bank *bank, int first, int last)
{
	int res;
	struct nrf5_info *chip;

	res = nrf5_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	/* For each sector to be erased */
	for (int s = first; s <= last && res == ERROR_OK; s++)
		res = nrf5_erase_page(bank, chip, &bank->sectors[s]);

	return res;
}

static int nrf5_code_flash_write(struct flash_bank *bank,
				  struct nrf5_info *chip,
				  const uint8_t *buffer, uint32_t offset, uint32_t count)
{

	int res;
	/* Need to perform reads to fill any gaps we need to preserve in the first page,
	   before the start of buffer, or in the last page, after the end of buffer */
	uint32_t first_page = offset/chip->code_page_size;
	uint32_t last_page = DIV_ROUND_UP(offset+count, chip->code_page_size);

	uint32_t first_page_offset = first_page * chip->code_page_size;
	uint32_t last_page_offset = last_page * chip->code_page_size;

	LOG_DEBUG("Padding write from 0x%08"PRIx32"-0x%08"PRIx32" as 0x%08"PRIx32"-0x%08"PRIx32,
		offset, offset+count, first_page_offset, last_page_offset);

	uint32_t page_cnt = last_page - first_page;
	uint8_t buffer_to_flash[page_cnt*chip->code_page_size];

	/* Fill in any space between start of first page and start of buffer */
	uint32_t pre = offset - first_page_offset;
	if (pre > 0) {
		res = target_read_memory(bank->target,
					first_page_offset,
					1,
					pre,
					buffer_to_flash);
		if (res != ERROR_OK)
			return res;
	}

	/* Fill in main contents of buffer */
	memcpy(buffer_to_flash+pre, buffer, count);

	/* Fill in any space between end of buffer and end of last page */
	uint32_t post = last_page_offset - (offset+count);
	if (post > 0) {
		/* Retrieve the full row contents from Flash */
		res = target_read_memory(bank->target,
					offset + count,
					1,
					post,
					buffer_to_flash+pre+count);
		if (res != ERROR_OK)
			return res;
	}

	return nrf5_write_pages(bank, first_page_offset, last_page_offset, buffer_to_flash);
}

static int nrf5_uicr_flash_write(struct flash_bank *bank,
				  struct nrf5_info *chip,
				  const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int res;
	uint8_t uicr[NRF5_UICR_SIZE];
	struct flash_sector *sector = &bank->sectors[0];

	if ((offset + count) > NRF5_UICR_SIZE)
		return ERROR_FAIL;

	res = target_read_memory(bank->target,
				 NRF5_UICR_BASE,
				 1,
				 NRF5_UICR_SIZE,
				 uicr);

	if (res != ERROR_OK)
		return res;

	res = nrf5_erase_page(bank, chip, sector);
	if (res != ERROR_OK)
		return res;

	res = nrf5_nvmc_write_enable(chip);
	if (res != ERROR_OK)
		return res;

	memcpy(&uicr[offset], buffer, count);

	res = nrf5_ll_flash_write(chip, NRF5_UICR_BASE, uicr, NRF5_UICR_SIZE);
	if (res != ERROR_OK) {
		nrf5_nvmc_read_only(chip);
		return res;
	}

	return nrf5_nvmc_read_only(chip);
}


static int nrf5_write(struct flash_bank *bank, const uint8_t *buffer,
		       uint32_t offset, uint32_t count)
{
	int res;
	struct nrf5_info *chip;

	res = nrf5_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	return chip->bank[bank->bank_number].write(bank, chip, buffer, offset, count);
}

static void nrf5_free_driver_priv(struct flash_bank *bank)
{
	struct nrf5_info *chip = bank->driver_priv;
	if (chip == NULL)
		return;

	chip->refcount--;
	if (chip->refcount == 0) {
		free(chip);
		bank->driver_priv = NULL;
	}
}

FLASH_BANK_COMMAND_HANDLER(nrf5_flash_bank_command)
{
	static struct nrf5_info *chip;

	switch (bank->base) {
	case NRF5_FLASH_BASE:
		bank->bank_number = 0;
		break;
	case NRF5_UICR_BASE:
		bank->bank_number = 1;
		break;
	default:
		LOG_ERROR("Invalid bank address " TARGET_ADDR_FMT, bank->base);
		return ERROR_FAIL;
	}

	if (!chip) {
		/* Create a new chip */
		chip = calloc(1, sizeof(*chip));
		if (!chip)
			return ERROR_FAIL;

		chip->target = bank->target;
	}

	switch (bank->base) {
	case NRF5_FLASH_BASE:
		chip->bank[bank->bank_number].write = nrf5_code_flash_write;
		break;
	case NRF5_UICR_BASE:
		chip->bank[bank->bank_number].write = nrf5_uicr_flash_write;
		break;
	}

	chip->refcount++;
	chip->bank[bank->bank_number].probed = false;
	bank->driver_priv = chip;

	return ERROR_OK;
}

COMMAND_HANDLER(nrf5_handle_mass_erase_command)
{
	int res;
	struct flash_bank *bank = NULL;
	struct target *target = get_current_target(CMD_CTX);

	res = get_flash_bank_by_addr(target, NRF5_FLASH_BASE, true, &bank);
	if (res != ERROR_OK)
		return res;

	assert(bank != NULL);

	struct nrf5_info *chip;

	res = nrf5_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	uint32_t ppfc;

	res = target_read_u32(target, NRF5_FICR_PPFC,
			      &ppfc);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read PPFC register");
		return res;
	}

	if ((ppfc & 0xFF) == 0x00) {
		LOG_ERROR("Code region 0 size was pre-programmed at the factory, "
			  "mass erase command won't work.");
		return ERROR_FAIL;
	}

	res = nrf5_erase_all(chip);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to erase the chip");
		nrf5_protect_check(bank);
		return res;
	}

	res = nrf5_protect_check(bank);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to check chip's write protection");
		return res;
	}

	res = get_flash_bank_by_addr(target, NRF5_UICR_BASE, true, &bank);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

static int nrf5_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int res;

	struct nrf5_info *chip;

	res = nrf5_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	static struct {
		const uint32_t address;
		uint32_t value;
	} ficr[] = {
		{ .address = NRF5_FICR_CODEPAGESIZE	},
		{ .address = NRF5_FICR_CODESIZE	},
		{ .address = NRF5_FICR_CLENR0		},
		{ .address = NRF5_FICR_PPFC		},
		{ .address = NRF5_FICR_NUMRAMBLOCK	},
		{ .address = NRF5_FICR_SIZERAMBLOCK0	},
		{ .address = NRF5_FICR_SIZERAMBLOCK1	},
		{ .address = NRF5_FICR_SIZERAMBLOCK2	},
		{ .address = NRF5_FICR_SIZERAMBLOCK3	},
		{ .address = NRF5_FICR_CONFIGID	},
		{ .address = NRF5_FICR_DEVICEID0	},
		{ .address = NRF5_FICR_DEVICEID1	},
		{ .address = NRF5_FICR_ER0		},
		{ .address = NRF5_FICR_ER1		},
		{ .address = NRF5_FICR_ER2		},
		{ .address = NRF5_FICR_ER3		},
		{ .address = NRF5_FICR_IR0		},
		{ .address = NRF5_FICR_IR1		},
		{ .address = NRF5_FICR_IR2		},
		{ .address = NRF5_FICR_IR3		},
		{ .address = NRF5_FICR_DEVICEADDRTYPE	},
		{ .address = NRF5_FICR_DEVICEADDR0	},
		{ .address = NRF5_FICR_DEVICEADDR1	},
		{ .address = NRF5_FICR_OVERRIDEN	},
		{ .address = NRF5_FICR_NRF_1MBIT0	},
		{ .address = NRF5_FICR_NRF_1MBIT1	},
		{ .address = NRF5_FICR_NRF_1MBIT2	},
		{ .address = NRF5_FICR_NRF_1MBIT3	},
		{ .address = NRF5_FICR_NRF_1MBIT4	},
		{ .address = NRF5_FICR_BLE_1MBIT0	},
		{ .address = NRF5_FICR_BLE_1MBIT1	},
		{ .address = NRF5_FICR_BLE_1MBIT2	},
		{ .address = NRF5_FICR_BLE_1MBIT3	},
		{ .address = NRF5_FICR_BLE_1MBIT4	},
	}, uicr[] = {
		{ .address = NRF5_UICR_CLENR0,		},
		{ .address = NRF5_UICR_RBPCONF		},
		{ .address = NRF5_UICR_XTALFREQ	},
		{ .address = NRF5_UICR_FWID		},
	};

	for (size_t i = 0; i < ARRAY_SIZE(ficr); i++) {
		res = target_read_u32(chip->target, ficr[i].address,
				      &ficr[i].value);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read %" PRIx32, ficr[i].address);
			return res;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(uicr); i++) {
		res = target_read_u32(chip->target, uicr[i].address,
				      &uicr[i].value);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read %" PRIx32, uicr[i].address);
			return res;
		}
	}

	snprintf(buf, buf_size,
		 "\n[factory information control block]\n\n"
		 "code page size: %"PRIu32"B\n"
		 "code memory size: %"PRIu32"kB\n"
		 "code region 0 size: %"PRIu32"kB\n"
		 "pre-programmed code: %s\n"
		 "number of ram blocks: %"PRIu32"\n"
		 "ram block 0 size: %"PRIu32"B\n"
		 "ram block 1 size: %"PRIu32"B\n"
		 "ram block 2 size: %"PRIu32"B\n"
		 "ram block 3 size: %"PRIu32 "B\n"
		 "config id: %" PRIx32 "\n"
		 "device id: 0x%"PRIx32"%08"PRIx32"\n"
		 "encryption root: 0x%08"PRIx32"%08"PRIx32"%08"PRIx32"%08"PRIx32"\n"
		 "identity root: 0x%08"PRIx32"%08"PRIx32"%08"PRIx32"%08"PRIx32"\n"
		 "device address type: 0x%"PRIx32"\n"
		 "device address: 0x%"PRIx32"%08"PRIx32"\n"
		 "override enable: %"PRIx32"\n"
		 "NRF_1MBIT values: %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32"\n"
		 "BLE_1MBIT values: %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32"\n"
		 "\n[user information control block]\n\n"
		 "code region 0 size: %"PRIu32"kB\n"
		 "read back protection configuration: %"PRIx32"\n"
		 "reset value for XTALFREQ: %"PRIx32"\n"
		 "firmware id: 0x%04"PRIx32,
		 ficr[0].value,
		 (ficr[1].value * ficr[0].value) / 1024,
		 (ficr[2].value == 0xFFFFFFFF) ? 0 : ficr[2].value / 1024,
		 ((ficr[3].value & 0xFF) == 0x00) ? "present" : "not present",
		 ficr[4].value,
		 ficr[5].value,
		 (ficr[6].value == 0xFFFFFFFF) ? 0 : ficr[6].value,
		 (ficr[7].value == 0xFFFFFFFF) ? 0 : ficr[7].value,
		 (ficr[8].value == 0xFFFFFFFF) ? 0 : ficr[8].value,
		 ficr[9].value,
		 ficr[10].value, ficr[11].value,
		 ficr[12].value, ficr[13].value, ficr[14].value, ficr[15].value,
		 ficr[16].value, ficr[17].value, ficr[18].value, ficr[19].value,
		 ficr[20].value,
		 ficr[21].value, ficr[22].value,
		 ficr[23].value,
		 ficr[24].value, ficr[25].value, ficr[26].value, ficr[27].value, ficr[28].value,
		 ficr[29].value, ficr[30].value, ficr[31].value, ficr[32].value, ficr[33].value,
		 (uicr[0].value == 0xFFFFFFFF) ? 0 : uicr[0].value / 1024,
		 uicr[1].value & 0xFFFF,
		 uicr[2].value & 0xFF,
		 uicr[3].value & 0xFFFF);

	return ERROR_OK;
}

static const struct command_registration nrf5_exec_command_handlers[] = {
	{
		.name		= "mass_erase",
		.handler	= nrf5_handle_mass_erase_command,
		.mode		= COMMAND_EXEC,
		.help		= "Erase all flash contents of the chip.",
		.usage		= "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration nrf5_command_handlers[] = {
	{
		.name	= "nrf5",
		.mode	= COMMAND_ANY,
		.help	= "nrf5 flash command group",
		.usage	= "",
		.chain	= nrf5_exec_command_handlers,
	},
	{
		.name	= "nrf51",
		.mode	= COMMAND_ANY,
		.help	= "nrf51 flash command group",
		.usage	= "",
		.chain	= nrf5_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver nrf5_flash = {
	.name			= "nrf5",
	.commands		= nrf5_command_handlers,
	.flash_bank_command	= nrf5_flash_bank_command,
	.info			= nrf5_info,
	.erase			= nrf5_erase,
	.protect		= nrf5_protect,
	.write			= nrf5_write,
	.read			= default_flash_read,
	.probe			= nrf5_probe,
	.auto_probe		= nrf5_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= nrf5_protect_check,
	.free_driver_priv	= nrf5_free_driver_priv,
};

/* We need to retain the flash-driver name as well as the commands
 * for backwards compatability */
const struct flash_driver nrf51_flash = {
	.name			= "nrf51",
	.commands		= nrf5_command_handlers,
	.flash_bank_command	= nrf5_flash_bank_command,
	.info			= nrf5_info,
	.erase			= nrf5_erase,
	.protect		= nrf5_protect,
	.write			= nrf5_write,
	.read			= default_flash_read,
	.probe			= nrf5_probe,
	.auto_probe		= nrf5_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= nrf5_protect_check,
	.free_driver_priv	= nrf5_free_driver_priv,
};
