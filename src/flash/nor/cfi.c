/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2009 Michael Schwingen                                  *
 *   michael@schwingen.org                                                 *
 *   Copyright (C) 2010 Øyvind Harboe <oyvind.harboe@zylin.com>            *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "cfi.h"
#include "non_cfi.h"
#include <target/arm.h>
#include <target/arm7_9_common.h>
#include <target/armv7m.h>
#include <target/mips32.h>
#include <helper/binarybuffer.h>
#include <target/algorithm.h>

/* defines internal maximum size for code fragment in cfi_intel_write_block() */
#define CFI_MAX_INTEL_CODESIZE 256

/* some id-types with specific handling */
#define AT49BV6416      0x00d6
#define AT49BV6416T     0x00d2

static const struct cfi_unlock_addresses cfi_unlock_addresses[] = {
	[CFI_UNLOCK_555_2AA] = { .unlock1 = 0x555, .unlock2 = 0x2aa },
	[CFI_UNLOCK_5555_2AAA] = { .unlock1 = 0x5555, .unlock2 = 0x2aaa },
};

static const int cfi_status_poll_mask_dq6_dq7 = CFI_STATUS_POLL_MASK_DQ6_DQ7;

/* CFI fixups forward declarations */
static void cfi_fixup_0002_erase_regions(struct flash_bank *bank, const void *param);
static void cfi_fixup_0002_unlock_addresses(struct flash_bank *bank, const void *param);
static void cfi_fixup_reversed_erase_regions(struct flash_bank *bank, const void *param);
static void cfi_fixup_0002_write_buffer(struct flash_bank *bank, const void *param);
static void cfi_fixup_0002_polling_bits(struct flash_bank *bank, const void *param);

/* fixup after reading cmdset 0002 primary query table */
static const struct cfi_fixup cfi_0002_fixups[] = {
	{CFI_MFR_SST, 0x00D4, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x00D5, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x00D6, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x00D7, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x2780, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x274b, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_SST, 0x235f, cfi_fixup_0002_polling_bits,	/* 39VF3201C */
	 &cfi_status_poll_mask_dq6_dq7},
	{CFI_MFR_SST, 0x236d, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_ATMEL, 0x00C8, cfi_fixup_reversed_erase_regions, NULL},
	{CFI_MFR_ST, 0x22C4, cfi_fixup_reversed_erase_regions, NULL},	/* M29W160ET */
	{CFI_MFR_FUJITSU, 0x22ea, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_FUJITSU, 0x226b, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_5555_2AAA]},
	{CFI_MFR_AMIC, 0xb31a, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_MX, 0x225b, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_EON, 0x225b, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_AMD, 0x225b, cfi_fixup_0002_unlock_addresses,
	 &cfi_unlock_addresses[CFI_UNLOCK_555_2AA]},
	{CFI_MFR_ANY, CFI_ID_ANY, cfi_fixup_0002_erase_regions, NULL},
	{CFI_MFR_ST, 0x227E, cfi_fixup_0002_write_buffer, NULL},/* M29W128G */
	{0, 0, NULL, NULL}
};

/* fixup after reading cmdset 0001 primary query table */
static const struct cfi_fixup cfi_0001_fixups[] = {
	{0, 0, NULL, NULL}
};

static void cfi_fixup(struct flash_bank *bank, const struct cfi_fixup *fixups)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	for (const struct cfi_fixup *f = fixups; f->fixup; f++) {
		if (((f->mfr == CFI_MFR_ANY) || (f->mfr == cfi_info->manufacturer)) &&
				((f->id  == CFI_ID_ANY)  || (f->id  == cfi_info->device_id)))
			f->fixup(bank, f->param);
	}
}

uint32_t cfi_flash_address(struct flash_bank *bank, int sector, uint32_t offset)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	if (cfi_info->x16_as_x8)
		offset *= 2;

	/* while the sector list isn't built, only accesses to sector 0 work */
	if (sector == 0)
		return bank->base + offset * bank->bus_width;
	else {
		if (!bank->sectors) {
			LOG_ERROR("BUG: sector list not yet built");
			exit(-1);
		}
		return bank->base + bank->sectors[sector].offset + offset * bank->bus_width;
	}
}

static int cfi_target_write_memory(struct flash_bank *bank, target_addr_t addr,
				   uint32_t count, const uint8_t *buffer)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	if (cfi_info->write_mem) {
		return cfi_info->write_mem(bank, addr, count, buffer);
	} else {
		return target_write_memory(bank->target, addr, bank->bus_width,
					   count, buffer);
	}
}

int cfi_target_read_memory(struct flash_bank *bank, target_addr_t addr,
			   uint32_t count, uint8_t *buffer)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	if (cfi_info->read_mem) {
		return cfi_info->read_mem(bank, addr, count, buffer);
	} else {
		return target_read_memory(bank->target, addr, bank->bus_width,
					  count, buffer);
	}
}

static void cfi_command(struct flash_bank *bank, uint8_t cmd, uint8_t *cmd_buf)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	/* clear whole buffer, to ensure bits that exceed the bus_width
	 * are set to zero
	 */
	for (size_t i = 0; i < CFI_MAX_BUS_WIDTH; i++)
		cmd_buf[i] = 0;

	if (cfi_info->endianness == TARGET_LITTLE_ENDIAN) {
		for (unsigned int i = bank->bus_width; i > 0; i--)
			*cmd_buf++ = (i & (bank->chip_width - 1)) ? 0x0 : cmd;
	} else {
		for (unsigned int i = 1; i <= bank->bus_width; i++)
			*cmd_buf++ = (i & (bank->chip_width - 1)) ? 0x0 : cmd;
	}
}

int cfi_send_command(struct flash_bank *bank, uint8_t cmd, uint32_t address)
{
	uint8_t command[CFI_MAX_BUS_WIDTH];

	cfi_command(bank, cmd, command);
	return cfi_target_write_memory(bank, address, 1, command);
}

/* read unsigned 8-bit value from the bank
 * flash banks are expected to be made of similar chips
 * the query result should be the same for all
 */
static int cfi_query_u8(struct flash_bank *bank, int sector, uint32_t offset, uint8_t *val)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	uint8_t data[CFI_MAX_BUS_WIDTH];

	int retval;
	retval = cfi_target_read_memory(bank, cfi_flash_address(bank, sector, offset),
					1, data);
	if (retval != ERROR_OK)
		return retval;

	if (cfi_info->endianness == TARGET_LITTLE_ENDIAN)
		*val = data[0];
	else
		*val = data[bank->bus_width - 1];

	return ERROR_OK;
}

/* read unsigned 8-bit value from the bank
 * in case of a bank made of multiple chips,
 * the individual values are ORed
 */
static int cfi_get_u8(struct flash_bank *bank, int sector, uint32_t offset, uint8_t *val)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	uint8_t data[CFI_MAX_BUS_WIDTH];

	int retval;
	retval = cfi_target_read_memory(bank, cfi_flash_address(bank, sector, offset),
					1, data);
	if (retval != ERROR_OK)
		return retval;

	if (cfi_info->endianness == TARGET_LITTLE_ENDIAN) {
		for (unsigned int i = 0; i < bank->bus_width / bank->chip_width; i++)
			data[0] |= data[i];

		*val = data[0];
	} else {
		uint8_t value = 0;
		for (unsigned int i = 0; i < bank->bus_width / bank->chip_width; i++)
			value |= data[bank->bus_width - 1 - i];

		*val = value;
	}
	return ERROR_OK;
}

static int cfi_query_u16(struct flash_bank *bank, int sector, uint32_t offset, uint16_t *val)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	uint8_t data[CFI_MAX_BUS_WIDTH * 2];
	int retval;

	if (cfi_info->x16_as_x8) {
		for (uint8_t i = 0; i < 2; i++) {
			retval = cfi_target_read_memory(bank, cfi_flash_address(bank, sector, offset + i),
							1, &data[i * bank->bus_width]);
			if (retval != ERROR_OK)
				return retval;
		}
	} else {
		retval = cfi_target_read_memory(bank, cfi_flash_address(bank, sector, offset),
						2, data);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cfi_info->endianness == TARGET_LITTLE_ENDIAN)
		*val = data[0] | data[bank->bus_width] << 8;
	else
		*val = data[bank->bus_width - 1] | data[(2 * bank->bus_width) - 1] << 8;

	return ERROR_OK;
}

static int cfi_query_u32(struct flash_bank *bank, int sector, uint32_t offset, uint32_t *val)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	uint8_t data[CFI_MAX_BUS_WIDTH * 4];
	int retval;

	if (cfi_info->x16_as_x8) {
		for (uint8_t i = 0; i < 4; i++) {
			retval = cfi_target_read_memory(bank, cfi_flash_address(bank, sector, offset + i),
							1, &data[i * bank->bus_width]);
			if (retval != ERROR_OK)
				return retval;
		}
	} else {
		retval = cfi_target_read_memory(bank, cfi_flash_address(bank, sector, offset),
						4, data);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cfi_info->endianness == TARGET_LITTLE_ENDIAN)
		*val = data[0] | data[bank->bus_width] << 8 |
			data[bank->bus_width * 2] << 16 | data[bank->bus_width * 3] << 24;
	else
		*val = data[bank->bus_width - 1] | data[(2 * bank->bus_width) - 1] << 8 |
			data[(3 * bank->bus_width) - 1] << 16 |
			data[(4 * bank->bus_width) - 1] << 24;

	return ERROR_OK;
}

int cfi_reset(struct flash_bank *bank)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	int retval = ERROR_OK;

	retval = cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
	if (retval != ERROR_OK)
		return retval;

	if (cfi_info->manufacturer == 0x20 &&
			(cfi_info->device_id == 0x227E || cfi_info->device_id == 0x7E)) {
		/* Numonix M29W128G is cmd 0xFF intolerant - causes internal undefined state
		 * so we send an extra 0xF0 reset to fix the bug */
		retval = cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x00));
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static void cfi_intel_clear_status_register(struct flash_bank *bank)
{
	cfi_send_command(bank, 0x50, cfi_flash_address(bank, 0, 0x0));
}

static int cfi_intel_wait_status_busy(struct flash_bank *bank, int timeout, uint8_t *val)
{
	uint8_t status;

	int retval = ERROR_OK;

	for (;; ) {
		if (timeout-- < 0) {
			LOG_ERROR("timeout while waiting for WSM to become ready");
			return ERROR_FAIL;
		}

		retval = cfi_get_u8(bank, 0, 0x0, &status);
		if (retval != ERROR_OK)
			return retval;

		if (status & 0x80)
			break;

		alive_sleep(1);
	}

	/* mask out bit 0 (reserved) */
	status = status & 0xfe;

	LOG_DEBUG("status: 0x%x", status);

	if (status != 0x80) {
		LOG_ERROR("status register: 0x%x", status);
		if (status & 0x2)
			LOG_ERROR("Block Lock-Bit Detected, Operation Abort");
		if (status & 0x4)
			LOG_ERROR("Program suspended");
		if (status & 0x8)
			LOG_ERROR("Low Programming Voltage Detected, Operation Aborted");
		if (status & 0x10)
			LOG_ERROR("Program Error / Error in Setting Lock-Bit");
		if (status & 0x20)
			LOG_ERROR("Error in Block Erasure or Clear Lock-Bits");
		if (status & 0x40)
			LOG_ERROR("Block Erase Suspended");

		cfi_intel_clear_status_register(bank);

		retval = ERROR_FAIL;
	}

	*val = status;
	return retval;
}

int cfi_spansion_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint8_t status, oldstatus;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	int retval;

	retval = cfi_get_u8(bank, 0, 0x0, &oldstatus);
	if (retval != ERROR_OK)
		return retval;

	do {
		retval = cfi_get_u8(bank, 0, 0x0, &status);

		if (retval != ERROR_OK)
			return retval;

		if ((status ^ oldstatus) & 0x40) {
			if (status & cfi_info->status_poll_mask & 0x20) {
				retval = cfi_get_u8(bank, 0, 0x0, &oldstatus);
				if (retval != ERROR_OK)
					return retval;
				retval = cfi_get_u8(bank, 0, 0x0, &status);
				if (retval != ERROR_OK)
					return retval;
				if ((status ^ oldstatus) & 0x40) {
					LOG_ERROR("dq5 timeout, status: 0x%x", status);
					return ERROR_FLASH_OPERATION_FAILED;
				} else {
					LOG_DEBUG("status: 0x%x", status);
					return ERROR_OK;
				}
			}
		} else {/* no toggle: finished, OK */
			LOG_DEBUG("status: 0x%x", status);
			return ERROR_OK;
		}

		oldstatus = status;
		alive_sleep(1);
	} while (timeout-- > 0);

	LOG_ERROR("timeout, status: 0x%x", status);

	return ERROR_FLASH_BUSY;
}

static int cfi_read_intel_pri_ext(struct flash_bank *bank)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_intel_pri_ext *pri_ext;

	free(cfi_info->pri_ext);

	pri_ext = malloc(sizeof(struct cfi_intel_pri_ext));
	if (pri_ext == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	cfi_info->pri_ext = pri_ext;

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0, &pri_ext->pri[0]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 1, &pri_ext->pri[1]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 2, &pri_ext->pri[2]);
	if (retval != ERROR_OK)
		return retval;

	if ((pri_ext->pri[0] != 'P') || (pri_ext->pri[1] != 'R') || (pri_ext->pri[2] != 'I')) {
		retval = cfi_reset(bank);
		if (retval != ERROR_OK)
			return retval;
		LOG_ERROR("Could not read bank flash bank information");
		return ERROR_FLASH_BANK_INVALID;
	}

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 3, &pri_ext->major_version);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 4, &pri_ext->minor_version);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("pri: '%c%c%c', version: %c.%c", pri_ext->pri[0], pri_ext->pri[1],
		pri_ext->pri[2], pri_ext->major_version, pri_ext->minor_version);

	retval = cfi_query_u32(bank, 0, cfi_info->pri_addr + 5, &pri_ext->feature_support);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 9, &pri_ext->suspend_cmd_support);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u16(bank, 0, cfi_info->pri_addr + 0xa, &pri_ext->blk_status_reg_mask);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("feature_support: 0x%" PRIx32 ", suspend_cmd_support: "
		"0x%x, blk_status_reg_mask: 0x%x",
		pri_ext->feature_support,
		pri_ext->suspend_cmd_support,
		pri_ext->blk_status_reg_mask);

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0xc, &pri_ext->vcc_optimal);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0xd, &pri_ext->vpp_optimal);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Vcc opt: %x.%x, Vpp opt: %u.%x",
		(pri_ext->vcc_optimal & 0xf0) >> 4, pri_ext->vcc_optimal & 0x0f,
		(pri_ext->vpp_optimal & 0xf0) >> 4, pri_ext->vpp_optimal & 0x0f);

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0xe, &pri_ext->num_protection_fields);
	if (retval != ERROR_OK)
		return retval;
	if (pri_ext->num_protection_fields != 1) {
		LOG_WARNING("expected one protection register field, but found %i",
			pri_ext->num_protection_fields);
	}

	retval = cfi_query_u16(bank, 0, cfi_info->pri_addr + 0xf, &pri_ext->prot_reg_addr);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0x11, &pri_ext->fact_prot_reg_size);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0x12, &pri_ext->user_prot_reg_size);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("protection_fields: %i, prot_reg_addr: 0x%x, "
		"factory pre-programmed: %i, user programmable: %i",
		pri_ext->num_protection_fields, pri_ext->prot_reg_addr,
		1 << pri_ext->fact_prot_reg_size, 1 << pri_ext->user_prot_reg_size);

	return ERROR_OK;
}

static int cfi_read_spansion_pri_ext(struct flash_bank *bank)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext;

	free(cfi_info->pri_ext);

	pri_ext = malloc(sizeof(struct cfi_spansion_pri_ext));
	if (pri_ext == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	cfi_info->pri_ext = pri_ext;

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0, &pri_ext->pri[0]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 1, &pri_ext->pri[1]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 2, &pri_ext->pri[2]);
	if (retval != ERROR_OK)
		return retval;

	/* default values for implementation specific workarounds */
	pri_ext->_unlock1 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock1;
	pri_ext->_unlock2 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock2;
	pri_ext->_reversed_geometry = 0;

	if ((pri_ext->pri[0] != 'P') || (pri_ext->pri[1] != 'R') || (pri_ext->pri[2] != 'I')) {
		retval = cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
		if (retval != ERROR_OK)
			return retval;
		LOG_ERROR("Could not read spansion bank information");
		return ERROR_FLASH_BANK_INVALID;
	}

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 3, &pri_ext->major_version);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 4, &pri_ext->minor_version);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("pri: '%c%c%c', version: %c.%c", pri_ext->pri[0], pri_ext->pri[1],
		pri_ext->pri[2], pri_ext->major_version, pri_ext->minor_version);

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 5, &pri_ext->SiliconRevision);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 6, &pri_ext->EraseSuspend);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 7, &pri_ext->BlkProt);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 8, &pri_ext->TmpBlkUnprotect);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 9, &pri_ext->BlkProtUnprot);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 10, &pri_ext->SimultaneousOps);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 11, &pri_ext->BurstMode);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 12, &pri_ext->PageMode);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 13, &pri_ext->VppMin);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 14, &pri_ext->VppMax);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 15, &pri_ext->TopBottom);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Silicon Revision: 0x%x, Erase Suspend: 0x%x, Block protect: 0x%x",
		pri_ext->SiliconRevision, pri_ext->EraseSuspend, pri_ext->BlkProt);

	LOG_DEBUG("Temporary Unprotect: 0x%x, Block Protect Scheme: 0x%x, "
		"Simultaneous Ops: 0x%x", pri_ext->TmpBlkUnprotect,
		pri_ext->BlkProtUnprot, pri_ext->SimultaneousOps);

	LOG_DEBUG("Burst Mode: 0x%x, Page Mode: 0x%x, ", pri_ext->BurstMode, pri_ext->PageMode);


	LOG_DEBUG("Vpp min: %u.%x, Vpp max: %u.%x",
		(pri_ext->VppMin & 0xf0) >> 4, pri_ext->VppMin & 0x0f,
		(pri_ext->VppMax & 0xf0) >> 4, pri_ext->VppMax & 0x0f);

	LOG_DEBUG("WP# protection 0x%x", pri_ext->TopBottom);

	return ERROR_OK;
}

static int cfi_read_atmel_pri_ext(struct flash_bank *bank)
{
	int retval;
	struct cfi_atmel_pri_ext atmel_pri_ext;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext;

	free(cfi_info->pri_ext);

	pri_ext = malloc(sizeof(struct cfi_spansion_pri_ext));
	if (pri_ext == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	/* ATMEL devices use the same CFI primary command set (0x2) as AMD/Spansion,
	 * but a different primary extended query table.
	 * We read the atmel table, and prepare a valid AMD/Spansion query table.
	 */

	memset(pri_ext, 0, sizeof(struct cfi_spansion_pri_ext));

	cfi_info->pri_ext = pri_ext;

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 0, &atmel_pri_ext.pri[0]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 1, &atmel_pri_ext.pri[1]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 2, &atmel_pri_ext.pri[2]);
	if (retval != ERROR_OK)
		return retval;

	if ((atmel_pri_ext.pri[0] != 'P') || (atmel_pri_ext.pri[1] != 'R')
			|| (atmel_pri_ext.pri[2] != 'I')) {
		retval = cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
		if (retval != ERROR_OK)
			return retval;
		LOG_ERROR("Could not read atmel bank information");
		return ERROR_FLASH_BANK_INVALID;
	}

	pri_ext->pri[0] = atmel_pri_ext.pri[0];
	pri_ext->pri[1] = atmel_pri_ext.pri[1];
	pri_ext->pri[2] = atmel_pri_ext.pri[2];

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 3, &atmel_pri_ext.major_version);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 4, &atmel_pri_ext.minor_version);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("pri: '%c%c%c', version: %c.%c", atmel_pri_ext.pri[0],
		atmel_pri_ext.pri[1], atmel_pri_ext.pri[2],
		atmel_pri_ext.major_version, atmel_pri_ext.minor_version);

	pri_ext->major_version = atmel_pri_ext.major_version;
	pri_ext->minor_version = atmel_pri_ext.minor_version;

	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 5, &atmel_pri_ext.features);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 6, &atmel_pri_ext.bottom_boot);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 7, &atmel_pri_ext.burst_mode);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, cfi_info->pri_addr + 8, &atmel_pri_ext.page_mode);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG(
		"features: 0x%2.2x, bottom_boot: 0x%2.2x, burst_mode: 0x%2.2x, page_mode: 0x%2.2x",
		atmel_pri_ext.features,
		atmel_pri_ext.bottom_boot,
		atmel_pri_ext.burst_mode,
		atmel_pri_ext.page_mode);

	if (atmel_pri_ext.features & 0x02)
		pri_ext->EraseSuspend = 2;

	/* some chips got it backwards... */
	if (cfi_info->device_id == AT49BV6416 ||
			cfi_info->device_id == AT49BV6416T) {
		if (atmel_pri_ext.bottom_boot)
			pri_ext->TopBottom = 3;
		else
			pri_ext->TopBottom = 2;
	} else {
		if (atmel_pri_ext.bottom_boot)
			pri_ext->TopBottom = 2;
		else
			pri_ext->TopBottom = 3;
	}

	pri_ext->_unlock1 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock1;
	pri_ext->_unlock2 = cfi_unlock_addresses[CFI_UNLOCK_555_2AA].unlock2;

	return ERROR_OK;
}

static int cfi_read_0002_pri_ext(struct flash_bank *bank)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	if (cfi_info->manufacturer == CFI_MFR_ATMEL)
		return cfi_read_atmel_pri_ext(bank);
	else
		return cfi_read_spansion_pri_ext(bank);
}

static int cfi_spansion_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;

	printed = snprintf(buf, buf_size, "\nSpansion primary algorithm extend information:\n");
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "pri: '%c%c%c', version: %c.%c\n", pri_ext->pri[0],
			pri_ext->pri[1], pri_ext->pri[2],
			pri_ext->major_version, pri_ext->minor_version);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "Silicon Rev.: 0x%x, Address Sensitive unlock: 0x%x\n",
			(pri_ext->SiliconRevision) >> 2,
			(pri_ext->SiliconRevision) & 0x03);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "Erase Suspend: 0x%x, Sector Protect: 0x%x\n",
			pri_ext->EraseSuspend,
			pri_ext->BlkProt);
	buf += printed;
	buf_size -= printed;

	snprintf(buf, buf_size, "VppMin: %u.%x, VppMax: %u.%x\n",
		(pri_ext->VppMin & 0xf0) >> 4, pri_ext->VppMin & 0x0f,
		(pri_ext->VppMax & 0xf0) >> 4, pri_ext->VppMax & 0x0f);

	return ERROR_OK;
}

static int cfi_intel_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_intel_pri_ext *pri_ext = cfi_info->pri_ext;

	printed = snprintf(buf, buf_size, "\nintel primary algorithm extend information:\n");
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf,
			buf_size,
			"pri: '%c%c%c', version: %c.%c\n",
			pri_ext->pri[0],
			pri_ext->pri[1],
			pri_ext->pri[2],
			pri_ext->major_version,
			pri_ext->minor_version);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf,
			buf_size,
			"feature_support: 0x%" PRIx32 ", "
			"suspend_cmd_support: 0x%x, blk_status_reg_mask: 0x%x\n",
			pri_ext->feature_support,
			pri_ext->suspend_cmd_support,
			pri_ext->blk_status_reg_mask);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "Vcc opt: %x.%x, Vpp opt: %u.%x\n",
			(pri_ext->vcc_optimal & 0xf0) >> 4, pri_ext->vcc_optimal & 0x0f,
			(pri_ext->vpp_optimal & 0xf0) >> 4, pri_ext->vpp_optimal & 0x0f);
	buf += printed;
	buf_size -= printed;

	snprintf(buf, buf_size, "protection_fields: %i, prot_reg_addr: 0x%x, "
		"factory pre-programmed: %i, user programmable: %i\n",
		pri_ext->num_protection_fields, pri_ext->prot_reg_addr,
		1 << pri_ext->fact_prot_reg_size, 1 << pri_ext->user_prot_reg_size);

	return ERROR_OK;
}

int cfi_flash_bank_cmd(struct flash_bank *bank, unsigned int argc, const char **argv)
{
	struct cfi_flash_bank *cfi_info;
	bool bus_swap = false;

	if (argc < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* both widths must:
	 * - not exceed max value;
	 * - not be null;
	 * - be equal to a power of 2.
	 * bus must be wide enough to hold one chip */
	if ((bank->chip_width > CFI_MAX_CHIP_WIDTH)
			|| (bank->bus_width > CFI_MAX_BUS_WIDTH)
			|| (bank->chip_width == 0)
			|| (bank->bus_width == 0)
			|| (bank->chip_width & (bank->chip_width - 1))
			|| (bank->bus_width & (bank->bus_width - 1))
			|| (bank->chip_width > bank->bus_width)) {
		LOG_ERROR("chip and bus width have to specified in bytes");
		return ERROR_FLASH_BANK_INVALID;
	}

	cfi_info = calloc(1, sizeof(struct cfi_flash_bank));
	if (cfi_info == NULL) {
		LOG_ERROR("No memory for flash bank info");
		return ERROR_FAIL;
	}
	bank->driver_priv = cfi_info;

	for (unsigned i = 6; i < argc; i++) {
		if (strcmp(argv[i], "x16_as_x8") == 0)
			cfi_info->x16_as_x8 = true;
		else if (strcmp(argv[i], "data_swap") == 0)
			cfi_info->data_swap = true;
		else if (strcmp(argv[i], "bus_swap") == 0)
			bus_swap = true;
		else if (strcmp(argv[i], "jedec_probe") == 0)
			cfi_info->jedec_probe = true;
	}

	if (bus_swap)
		cfi_info->endianness =
			bank->target->endianness == TARGET_LITTLE_ENDIAN ?
			TARGET_BIG_ENDIAN : TARGET_LITTLE_ENDIAN;
	else
		cfi_info->endianness = bank->target->endianness;

	/* bank wasn't probed yet */
	cfi_info->qry[0] = 0xff;

	return ERROR_OK;
}

/* flash_bank cfi <base> <size> <chip_width> <bus_width> <target#> [options]
 */
FLASH_BANK_COMMAND_HANDLER(cfi_flash_bank_command)
{
	return cfi_flash_bank_cmd(bank, CMD_ARGC, CMD_ARGV);
}

static int cfi_intel_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	cfi_intel_clear_status_register(bank);

	for (unsigned int i = first; i <= last; i++) {
		retval = cfi_send_command(bank, 0x20, cfi_flash_address(bank, i, 0x0));
		if (retval != ERROR_OK)
			return retval;

		retval = cfi_send_command(bank, 0xd0, cfi_flash_address(bank, i, 0x0));
		if (retval != ERROR_OK)
			return retval;

		uint8_t status;
		retval = cfi_intel_wait_status_busy(bank, cfi_info->block_erase_timeout, &status);
		if (retval != ERROR_OK)
			return retval;

		if (status == 0x80)
			bank->sectors[i].is_erased = 1;
		else {
			retval = cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
			if (retval != ERROR_OK)
				return retval;

			LOG_ERROR("couldn't erase block %u of flash bank at base "
					TARGET_ADDR_FMT, i, bank->base);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
}

int cfi_spansion_unlock_seq(struct flash_bank *bank)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;

	retval = cfi_send_command(bank, 0xaa, cfi_flash_address(bank, 0, pri_ext->_unlock1));
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_send_command(bank, 0x55, cfi_flash_address(bank, 0, pri_ext->_unlock2));
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int cfi_spansion_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;

	for (unsigned int i = first; i <= last; i++) {
		retval = cfi_spansion_unlock_seq(bank);
		if (retval != ERROR_OK)
			return retval;

		retval = cfi_send_command(bank, 0x80, cfi_flash_address(bank, 0, pri_ext->_unlock1));
		if (retval != ERROR_OK)
			return retval;

		retval = cfi_spansion_unlock_seq(bank);
		if (retval != ERROR_OK)
			return retval;

		retval = cfi_send_command(bank, 0x30, cfi_flash_address(bank, i, 0x0));
		if (retval != ERROR_OK)
			return retval;

		if (cfi_spansion_wait_status_busy(bank, cfi_info->block_erase_timeout) == ERROR_OK)
			bank->sectors[i].is_erased = 1;
		else {
			retval = cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
			if (retval != ERROR_OK)
				return retval;

			LOG_ERROR("couldn't erase block %i of flash bank at base "
				TARGET_ADDR_FMT, i, bank->base);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
}

int cfi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	switch (cfi_info->pri_id) {
		case 1:
		case 3:
			return cfi_intel_erase(bank, first, last);
		case 2:
			return cfi_spansion_erase(bank, first, last);
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_OK;
}

static int cfi_intel_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_intel_pri_ext *pri_ext = cfi_info->pri_ext;
	int retry = 0;

	/* if the device supports neither legacy lock/unlock (bit 3) nor
	 * instant individual block locking (bit 5).
	 */
	if (!(pri_ext->feature_support & 0x28)) {
		LOG_ERROR("lock/unlock not supported on flash");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	cfi_intel_clear_status_register(bank);

	for (unsigned int i = first; i <= last; i++) {
		retval = cfi_send_command(bank, 0x60, cfi_flash_address(bank, i, 0x0));
		if (retval != ERROR_OK)
			return retval;
		if (set) {
			retval = cfi_send_command(bank, 0x01, cfi_flash_address(bank, i, 0x0));
			if (retval != ERROR_OK)
				return retval;
			bank->sectors[i].is_protected = 1;
		} else {
			retval = cfi_send_command(bank, 0xd0, cfi_flash_address(bank, i, 0x0));
			if (retval != ERROR_OK)
				return retval;
			bank->sectors[i].is_protected = 0;
		}

		/* instant individual block locking doesn't require reading of the status register
		 **/
		if (!(pri_ext->feature_support & 0x20)) {
			/* Clear lock bits operation may take up to 1.4s */
			uint8_t status;
			retval = cfi_intel_wait_status_busy(bank, 1400, &status);
			if (retval != ERROR_OK)
				return retval;
		} else {
			uint8_t block_status;
			/* read block lock bit, to verify status */
			retval = cfi_send_command(bank, 0x90, cfi_flash_address(bank, 0, 0x55));
			if (retval != ERROR_OK)
				return retval;
			retval = cfi_get_u8(bank, i, 0x2, &block_status);
			if (retval != ERROR_OK)
				return retval;

			if ((block_status & 0x1) != set) {
				LOG_ERROR(
					"couldn't change block lock status (set = %i, block_status = 0x%2.2x)",
					set, block_status);
				retval = cfi_send_command(bank, 0x70, cfi_flash_address(bank, 0, 0x55));
				if (retval != ERROR_OK)
					return retval;
				uint8_t status;
				retval = cfi_intel_wait_status_busy(bank, 10, &status);
				if (retval != ERROR_OK)
					return retval;

				if (retry > 10)
					return ERROR_FLASH_OPERATION_FAILED;
				else {
					i--;
					retry++;
				}
			}
		}
	}

	/* if the device doesn't support individual block lock bits set/clear,
	 * all blocks have been unlocked in parallel, so we set those that should be protected
	 */
	if ((!set) && (!(pri_ext->feature_support & 0x20))) {
		/* FIX!!! this code path is broken!!!
		 *
		 * The correct approach is:
		 *
		 * 1. read out current protection status
		 *
		 * 2. override read out protection status w/unprotected.
		 *
		 * 3. re-protect what should be protected.
		 *
		 */
		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			if (bank->sectors[i].is_protected == 1) {
				cfi_intel_clear_status_register(bank);

				retval = cfi_send_command(bank, 0x60, cfi_flash_address(bank, i, 0x0));
				if (retval != ERROR_OK)
					return retval;

				retval = cfi_send_command(bank, 0x01, cfi_flash_address(bank, i, 0x0));
				if (retval != ERROR_OK)
					return retval;

				uint8_t status;
				retval = cfi_intel_wait_status_busy(bank, 100, &status);
				if (retval != ERROR_OK)
					return retval;
			}
		}
	}

	return cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
}

int cfi_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	switch (cfi_info->pri_id) {
		case 1:
		case 3:
			return cfi_intel_protect(bank, set, first, last);
		default:
			LOG_WARNING("protect: cfi primary command set %i unsupported", cfi_info->pri_id);
			return ERROR_OK;
	}
}

static uint32_t cfi_command_val(struct flash_bank *bank, uint8_t cmd)
{
	struct target *target = bank->target;

	uint8_t buf[CFI_MAX_BUS_WIDTH];
	cfi_command(bank, cmd, buf);
	switch (bank->bus_width) {
		case 1:
			return buf[0];
		case 2:
			return target_buffer_get_u16(target, buf);
		case 4:
			return target_buffer_get_u32(target, buf);
		default:
			LOG_ERROR("Unsupported bank buswidth %u, can't do block memory writes",
					bank->bus_width);
			return 0;
	}
}

static int cfi_intel_write_block(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t address, uint32_t count)
{
	struct target *target = bank->target;
	struct reg_param reg_params[7];
	struct arm_algorithm arm_algo;
	struct working_area *write_algorithm;
	struct working_area *source = NULL;
	uint32_t buffer_size = 32768;
	uint32_t write_command_val, busy_pattern_val, error_pattern_val;

	/* algorithm register usage:
	 * r0: source address (in RAM)
	 * r1: target address (in Flash)
	 * r2: count
	 * r3: flash write command
	 * r4: status byte (returned to host)
	 * r5: busy test pattern
	 * r6: error test pattern
	 */

	/* see contrib/loaders/flash/armv4_5_cfi_intel_32.s for src */
	static const uint32_t word_32_code[] = {
		0xe4904004,	/* loop: ldr r4, [r0], #4 */
		0xe5813000,	/*       str r3, [r1] */
		0xe5814000,	/*       str r4, [r1] */
		0xe5914000,	/* busy: ldr r4, [r1] */
		0xe0047005,	/*        and r7, r4, r5 */
		0xe1570005,	/*       cmp r7, r5 */
		0x1afffffb,	/*       bne busy */
		0xe1140006,	/*       tst r4, r6 */
		0x1a000003,	/*       bne done */
		0xe2522001,	/*       subs r2, r2, #1 */
		0x0a000001,	/*       beq done */
		0xe2811004,	/*       add r1, r1 #4 */
		0xeafffff2,	/*       b loop */
		0xeafffffe	/* done: b -2 */
	};

	/* see contrib/loaders/flash/armv4_5_cfi_intel_16.s for src */
	static const uint32_t word_16_code[] = {
		0xe0d040b2,	/* loop: ldrh r4, [r0], #2 */
		0xe1c130b0,	/*       strh r3, [r1] */
		0xe1c140b0,	/*       strh r4, [r1] */
		0xe1d140b0,	/* busy  ldrh r4, [r1] */
		0xe0047005,	/*       and r7, r4, r5 */
		0xe1570005,	/*       cmp r7, r5 */
		0x1afffffb,	/*       bne busy */
		0xe1140006,	/*       tst r4, r6 */
		0x1a000003,	/*       bne done */
		0xe2522001,	/*       subs r2, r2, #1 */
		0x0a000001,	/*       beq done */
		0xe2811002,	/*       add r1, r1 #2 */
		0xeafffff2,	/*       b loop */
		0xeafffffe	/* done:	b -2 */
	};

	/* see contrib/loaders/flash/armv4_5_cfi_intel_8.s for src */
	static const uint32_t word_8_code[] = {
		0xe4d04001,	/* loop: ldrb r4, [r0], #1 */
		0xe5c13000,	/*       strb r3, [r1] */
		0xe5c14000,	/*       strb r4, [r1] */
		0xe5d14000,	/* busy  ldrb r4, [r1] */
		0xe0047005,	/*       and r7, r4, r5 */
		0xe1570005,	/*       cmp r7, r5 */
		0x1afffffb,	/*       bne busy */
		0xe1140006,	/*       tst r4, r6 */
		0x1a000003,	/*       bne done */
		0xe2522001,	/*       subs r2, r2, #1 */
		0x0a000001,	/*       beq done */
		0xe2811001,	/*       add r1, r1 #1 */
		0xeafffff2,	/*       b loop */
		0xeafffffe	/* done: b -2 */
	};
	uint8_t target_code[4*CFI_MAX_INTEL_CODESIZE];
	const uint32_t *target_code_src;
	uint32_t target_code_size;
	int retval = ERROR_OK;

	/* check we have a supported arch */
	if (is_arm(target_to_arm(target))) {
		/* All other ARM CPUs have 32 bit instructions */
		arm_algo.common_magic = ARM_COMMON_MAGIC;
		arm_algo.core_mode = ARM_MODE_SVC;
		arm_algo.core_state = ARM_STATE_ARM;
	} else {
		LOG_ERROR("Unknown architecture");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	cfi_intel_clear_status_register(bank);

	/* If we are setting up the write_algorithm, we need target_code_src
	 * if not we only need target_code_size. */

	/* However, we don't want to create multiple code paths, so we
	 * do the unnecessary evaluation of target_code_src, which the
	 * compiler will probably nicely optimize away if not needed */

	/* prepare algorithm code for target endian */
	switch (bank->bus_width) {
		case 1:
			target_code_src = word_8_code;
			target_code_size = sizeof(word_8_code);
			break;
		case 2:
			target_code_src = word_16_code;
			target_code_size = sizeof(word_16_code);
			break;
		case 4:
			target_code_src = word_32_code;
			target_code_size = sizeof(word_32_code);
			break;
		default:
			LOG_ERROR("Unsupported bank buswidth %u, can't do block memory writes",
					bank->bus_width);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* flash write code */
	if (target_code_size > sizeof(target_code)) {
		LOG_WARNING("Internal error - target code buffer to small. "
				"Increase CFI_MAX_INTEL_CODESIZE and recompile.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	target_buffer_set_u32_array(target, target_code, target_code_size / 4, target_code_src);

	/* Get memory for block write handler */
	retval = target_alloc_working_area(target,
			target_code_size,
			&write_algorithm);
	if (retval != ERROR_OK) {
		LOG_WARNING("No working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* write algorithm code to working area */
	retval = target_write_buffer(target, write_algorithm->address,
			target_code_size, target_code);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to write block write code to target");
		goto cleanup;
	}

	/* Get a workspace buffer for the data to flash starting with 32k size.
	 * Half size until buffer would be smaller 256 Bytes then fail back */
	/* FIXME Why 256 bytes, why not 32 bytes (smallest flash write page */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			LOG_WARNING(
				"no large enough working area available, can't do block memory writes");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto cleanup;
		}
	}

	/* setup algo registers */
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN);
	init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);
	init_reg_param(&reg_params[6], "r6", 32, PARAM_OUT);

	/* prepare command and status register patterns */
	write_command_val = cfi_command_val(bank, 0x40);
	busy_pattern_val  = cfi_command_val(bank, 0x80);
	error_pattern_val = cfi_command_val(bank, 0x7e);

	LOG_DEBUG("Using target buffer at " TARGET_ADDR_FMT " and of size 0x%04" PRIx32,
		source->address, buffer_size);

	/* Programming main loop */
	while (count > 0) {
		uint32_t thisrun_count = (count > buffer_size) ? buffer_size : count;
		uint32_t wsm_error;

		retval = target_write_buffer(target, source->address, thisrun_count, buffer);
		if (retval != ERROR_OK)
			goto cleanup;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count / bank->bus_width);

		buf_set_u32(reg_params[3].value, 0, 32, write_command_val);
		buf_set_u32(reg_params[5].value, 0, 32, busy_pattern_val);
		buf_set_u32(reg_params[6].value, 0, 32, error_pattern_val);

		LOG_DEBUG("Write 0x%04" PRIx32 " bytes to flash at 0x%08" PRIx32,
			thisrun_count, address);

		/* Execute algorithm, assume breakpoint for last instruction */
		retval = target_run_algorithm(target, 0, NULL, 7, reg_params,
				write_algorithm->address,
				write_algorithm->address + target_code_size -
				sizeof(uint32_t),
				10000,	/* 10s should be enough for max. 32k of data */
				&arm_algo);

		/* On failure try a fall back to direct word writes */
		if (retval != ERROR_OK) {
			cfi_intel_clear_status_register(bank);
			LOG_ERROR(
				"Execution of flash algorithm failed. Can't fall back. Please report.");
			retval = ERROR_FLASH_OPERATION_FAILED;
			/* retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE; */
			/* FIXME To allow fall back or recovery, we must save the actual status
			 * somewhere, so that a higher level code can start recovery. */
			goto cleanup;
		}

		/* Check return value from algo code */
		wsm_error = buf_get_u32(reg_params[4].value, 0, 32) & error_pattern_val;
		if (wsm_error) {
			/* read status register (outputs debug information) */
			uint8_t status;
			cfi_intel_wait_status_busy(bank, 100, &status);
			cfi_intel_clear_status_register(bank);
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto cleanup;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;

		keep_alive();
	}

	/* free up resources */
cleanup:
	if (source)
		target_free_working_area(target, source);

	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);

	return retval;
}

static int cfi_spansion_write_block_mips(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t address, uint32_t count)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;
	struct target *target = bank->target;
	struct reg_param reg_params[10];
	struct mips32_algorithm mips32_info;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t buffer_size = 32768;
	uint32_t status;
	int retval = ERROR_OK;

	/* input parameters -
	 *	4  A0 = source address
	 *	5  A1 = destination address
	 *	6  A2 = number of writes
	 *	7  A3 = flash write command
	 *	8  T0 = constant to mask DQ7 bits (also used for Dq5 with shift)
	 * output parameters -
	 *	9  T1 = 0x80 ok 0x00 bad
	 * temp registers -
	 *	10 T2 = value read from flash to test status
	 *	11 T3 = holding register
	 * unlock registers -
	 *  12 T4 = unlock1_addr
	 *  13 T5 = unlock1_cmd
	 *  14 T6 = unlock2_addr
	 *  15 T7 = unlock2_cmd */

	static const uint32_t mips_word_16_code[] = {
		/* start:	*/
		MIPS32_LHU(0, 9, 0, 4),		/* lhu $t1, ($a0)		; out = &saddr */
		MIPS32_ADDI(0, 4, 4, 2),	/* addi $a0, $a0, 2		; saddr += 2 */
		MIPS32_SH(0, 13, 0, 12),	/* sh $t5, ($t4)		; *fl_unl_addr1 = fl_unl_cmd1 */
		MIPS32_SH(0, 15, 0, 14),	/* sh $t7, ($t6)		; *fl_unl_addr2 = fl_unl_cmd2 */
		MIPS32_SH(0, 7, 0, 12),		/* sh $a3, ($t4)		; *fl_unl_addr1 = fl_write_cmd */
		MIPS32_SH(0, 9, 0, 5),		/* sh $t1, ($a1)		; *daddr = out */
		MIPS32_NOP,						/* nop */
		/* busy:	*/
		MIPS32_LHU(0, 10, 0, 5),		/* lhu $t2, ($a1)		; temp1 = *daddr */
		MIPS32_XOR(0, 11, 9, 10),		/* xor $t3, $a0, $t2	; temp2 = out ^ temp1; */
		MIPS32_AND(0, 11, 8, 11),		/* and $t3, $t0, $t3	; temp2 = temp2 & DQ7mask */
		MIPS32_BNE(0, 11, 8, 13),		/* bne $t3, $t0, cont	; if (temp2 != DQ7mask) goto cont */
		MIPS32_NOP,						/* nop					*/

		MIPS32_SRL(0, 10, 8, 2),		/* srl $t2,$t0,2		; temp1 = DQ7mask >> 2 */
		MIPS32_AND(0, 11, 10, 11),			/* and $t3, $t2, $t3	; temp2 = temp2 & temp1	*/
		MIPS32_BNE(0, 11, 10, NEG16(8)),	/* bne $t3, $t2, busy	; if (temp2 != temp1) goto busy	*/
		MIPS32_NOP,						/* nop					*/

		MIPS32_LHU(0, 10, 0, 5),		/* lhu $t2, ($a1)		; temp1 = *daddr */
		MIPS32_XOR(0, 11, 9, 10),		/* xor $t3, $a0, $t2	; temp2 = out ^ temp1; */
		MIPS32_AND(0, 11, 8, 11),		/* and $t3, $t0, $t3	; temp2 = temp2 & DQ7mask */
		MIPS32_BNE(0, 11, 8, 4),		/* bne $t3, $t0, cont	; if (temp2 != DQ7mask) goto cont */
		MIPS32_NOP,						/* nop */

		MIPS32_XOR(0, 9, 9, 9),			/* xor $t1, $t1, $t1	; out = 0 */
		MIPS32_BEQ(0, 9, 0, 11),			/* beq $t1, $zero, done	; if (out == 0) goto done */
		MIPS32_NOP,						/* nop */
		/* cont:	*/
		MIPS32_ADDI(0, 6, 6, NEG16(1)),	/* addi, $a2, $a2, -1	; numwrites-- */
		MIPS32_BNE(0, 6, 0, 5),		/* bne $a2, $zero, cont2	; if (numwrite != 0) goto cont2 */
		MIPS32_NOP,						/* nop */

		MIPS32_LUI(0, 9, 0),				/* lui $t1, 0 */
		MIPS32_ORI(0, 9, 9, 0x80),			/* ori $t1, $t1, 0x80	; out = 0x80 */

		MIPS32_B(0, 4),					/* b done			; goto done */
		MIPS32_NOP,						/* nop */
		/* cont2:	*/
		MIPS32_ADDI(0, 5, 5, 2),			/* addi $a0, $a0, 2	; daddr += 2 */
		MIPS32_B(0, NEG16(33)),			/* b start			; goto start */
		MIPS32_NOP,						/* nop */
		/* done: */
		MIPS32_SDBBP(0),					/* sdbbp			; break(); */
	};

	mips32_info.common_magic = MIPS32_COMMON_MAGIC;
	mips32_info.isa_mode = MIPS32_ISA_MIPS32;

	int target_code_size = 0;
	const uint32_t *target_code_src = NULL;

	switch (bank->bus_width) {
		case 2:
			/* Check for DQ5 support */
			if (cfi_info->status_poll_mask & (1 << 5)) {
				target_code_src = mips_word_16_code;
				target_code_size = sizeof(mips_word_16_code);
			} else {
				LOG_ERROR("Need DQ5 support");
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				/* target_code_src = mips_word_16_code_dq7only; */
				/* target_code_size = sizeof(mips_word_16_code_dq7only); */
			}
			break;
		default:
			LOG_ERROR("Unsupported bank buswidth %u, can't do block memory writes",
					bank->bus_width);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* flash write code */
	uint8_t *target_code;

	/* convert bus-width dependent algorithm code to correct endianness */
	target_code = malloc(target_code_size);
	if (target_code == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	target_buffer_set_u32_array(target, target_code, target_code_size / 4, target_code_src);

	/* allocate working area */
	retval = target_alloc_working_area(target, target_code_size,
			&write_algorithm);
	if (retval != ERROR_OK) {
		free(target_code);
		return retval;
	}

	/* write algorithm code to working area */
	retval = target_write_buffer(target, write_algorithm->address,
			target_code_size, target_code);
	if (retval != ERROR_OK) {
		free(target_code);
		return retval;
	}

	free(target_code);

	/* the following code still assumes target code is fixed 24*4 bytes */

	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING(
				"not enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "r4", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r5", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r6", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r7", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r8", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "r9", 32, PARAM_IN);
	init_reg_param(&reg_params[6], "r12", 32, PARAM_OUT);
	init_reg_param(&reg_params[7], "r13", 32, PARAM_OUT);
	init_reg_param(&reg_params[8], "r14", 32, PARAM_OUT);
	init_reg_param(&reg_params[9], "r15", 32, PARAM_OUT);

	while (count > 0) {
		uint32_t thisrun_count = (count > buffer_size) ? buffer_size : count;

		retval = target_write_buffer(target, source->address, thisrun_count, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count / bank->bus_width);
		buf_set_u32(reg_params[3].value, 0, 32, cfi_command_val(bank, 0xA0));
		buf_set_u32(reg_params[4].value, 0, 32, cfi_command_val(bank, 0x80));
		buf_set_u32(reg_params[6].value, 0, 32, cfi_flash_address(bank, 0, pri_ext->_unlock1));
		buf_set_u32(reg_params[7].value, 0, 32, 0xaaaaaaaa);
		buf_set_u32(reg_params[8].value, 0, 32, cfi_flash_address(bank, 0, pri_ext->_unlock2));
		buf_set_u32(reg_params[9].value, 0, 32, 0x55555555);

		retval = target_run_algorithm(target, 0, NULL, 10, reg_params,
				write_algorithm->address,
				write_algorithm->address + ((target_code_size) - 4),
				10000, &mips32_info);
		if (retval != ERROR_OK)
			break;

		status = buf_get_u32(reg_params[5].value, 0, 32);
		if (status != 0x80) {
			LOG_ERROR("flash write block failed status: 0x%" PRIx32, status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}

	target_free_all_working_areas(target);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);
	destroy_reg_param(&reg_params[7]);
	destroy_reg_param(&reg_params[8]);
	destroy_reg_param(&reg_params[9]);

	return retval;
}

static int cfi_spansion_write_block(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t address, uint32_t count)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;
	struct target *target = bank->target;
	struct reg_param reg_params[10];
	void *arm_algo;
	struct arm_algorithm armv4_5_algo;
	struct armv7m_algorithm armv7m_algo;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t buffer_size = 32768;
	uint32_t status;
	int retval = ERROR_OK;

	/* input parameters -
	 *	R0 = source address
	 *	R1 = destination address
	 *	R2 = number of writes
	 *	R3 = flash write command
	 *	R4 = constant to mask DQ7 bits (also used for Dq5 with shift)
	 * output parameters -
	 *	R5 = 0x80 ok 0x00 bad
	 * temp registers -
	 *	R6 = value read from flash to test status
	 *	R7 = holding register
	 * unlock registers -
	 *  R8 = unlock1_addr
	 *  R9 = unlock1_cmd
	 *  R10 = unlock2_addr
	 *  R11 = unlock2_cmd */

	/* see contrib/loaders/flash/armv4_5_cfi_span_32.s for src */
	static const uint32_t armv4_5_word_32_code[] = {
		/* 00008100 <sp_32_code>:		*/
		0xe4905004,		/* ldr	r5, [r0], #4			*/
		0xe5889000,		/* str	r9, [r8]				*/
		0xe58ab000,		/* str	r11, [r10]				*/
		0xe5883000,		/* str	r3, [r8]				*/
		0xe5815000,		/* str	r5, [r1]				*/
		0xe1a00000,		/* nop							*/
		/* 00008110 <sp_32_busy>:		*/
		0xe5916000,		/* ldr	r6, [r1]				*/
		0xe0257006,		/* eor	r7, r5, r6				*/
		0xe0147007,		/* ands	r7, r4, r7				*/
		0x0a000007,		/* beq	8140 <sp_32_cont> ; b if DQ7 == Data7 */
		0xe0166124,		/* ands	r6, r6, r4, lsr #2		*/
		0x0afffff9,		/* beq	8110 <sp_32_busy> ;	b if DQ5 low */
		0xe5916000,		/* ldr	r6, [r1]				*/
		0xe0257006,		/* eor	r7, r5, r6				*/
		0xe0147007,		/* ands	r7, r4, r7				*/
		0x0a000001,		/* beq	8140 <sp_32_cont> ; b if DQ7 == Data7 */
		0xe3a05000,		/* mov	r5, #0	; 0x0 - return 0x00, error */
		0x1a000004,		/* bne	8154 <sp_32_done>		*/
		/* 00008140 <sp_32_cont>:		*/
		0xe2522001,		/* subs	r2, r2, #1	; 0x1		*/
		0x03a05080,		/* moveq	r5, #128	; 0x80	*/
		0x0a000001,		/* beq	8154 <sp_32_done>		*/
		0xe2811004,		/* add	r1, r1, #4	; 0x4		*/
		0xeaffffe8,		/* b	8100 <sp_32_code>		*/
		/* 00008154 <sp_32_done>:		*/
		0xeafffffe		/* b	8154 <sp_32_done>		*/
	};

	/* see contrib/loaders/flash/armv4_5_cfi_span_16.s for src */
	static const uint32_t armv4_5_word_16_code[] = {
		/* 00008158 <sp_16_code>:		*/
		0xe0d050b2,		/* ldrh	r5, [r0], #2			*/
		0xe1c890b0,		/* strh	r9, [r8]				*/
		0xe1cab0b0,		/* strh	r11, [r10]				*/
		0xe1c830b0,		/* strh	r3, [r8]				*/
		0xe1c150b0,		/* strh	r5, [r1]				*/
		0xe1a00000,		/* nop			(mov r0,r0)		*/
		/* 00008168 <sp_16_busy>:		*/
		0xe1d160b0,		/* ldrh	r6, [r1]				*/
		0xe0257006,		/* eor	r7, r5, r6				*/
		0xe0147007,		/* ands	r7, r4, r7				*/
		0x0a000007,		/* beq	8198 <sp_16_cont>		*/
		0xe0166124,		/* ands	r6, r6, r4, lsr #2		*/
		0x0afffff9,		/* beq	8168 <sp_16_busy>		*/
		0xe1d160b0,		/* ldrh	r6, [r1]				*/
		0xe0257006,		/* eor	r7, r5, r6				*/
		0xe0147007,		/* ands	r7, r4, r7				*/
		0x0a000001,		/* beq	8198 <sp_16_cont>		*/
		0xe3a05000,		/* mov	r5, #0	; 0x0			*/
		0x1a000004,		/* bne	81ac <sp_16_done>		*/
		/* 00008198 <sp_16_cont>:		*/
		0xe2522001,	/* subs	r2, r2, #1	; 0x1		*/
		0x03a05080,	/* moveq	r5, #128	; 0x80	*/
		0x0a000001,	/* beq	81ac <sp_16_done>		*/
		0xe2811002,	/* add	r1, r1, #2	; 0x2		*/
		0xeaffffe8,	/* b	8158 <sp_16_code>		*/
		/* 000081ac <sp_16_done>:		*/
		0xeafffffe		/* b	81ac <sp_16_done>		*/
	};

	/* see contrib/loaders/flash/armv7m_cfi_span_16.s for src */
	static const uint32_t armv7m_word_16_code[] = {
		0x5B02F830,
		0x9000F8A8,
		0xB000F8AA,
		0x3000F8A8,
		0xBF00800D,
		0xEA85880E,
		0x40270706,
		0xEA16D00A,
		0xD0F70694,
		0xEA85880E,
		0x40270706,
		0xF04FD002,
		0xD1070500,
		0xD0023A01,
		0x0102F101,
		0xF04FE7E0,
		0xE7FF0580,
		0x0000BE00
	};

	/* see contrib/loaders/flash/armv7m_cfi_span_16_dq7.s for src */
	static const uint32_t armv7m_word_16_code_dq7only[] = {
		/* 00000000 <code>: */
		0x5B02F830,		/* ldrh.w	r5, [r0], #2	*/
		0x9000F8A8,		/* strh.w	r9, [r8]		*/
		0xB000F8AA,		/* strh.w	fp, [sl]		*/
		0x3000F8A8,		/* strh.w	r3, [r8]		*/
		0xBF00800D,		/* strh	r5, [r1, #0]		*/
						/* nop						*/

		/* 00000014 <busy>: */
		0xEA85880E,		/* ldrh	r6, [r1, #0]		*/
						/* eor.w	r7, r5, r6		*/
		0x40270706,		/* ands		r7, r4			*/
		0x3A01D1FA,		/* bne.n	14 <busy>		*/
						/* subs	r2, #1				*/
		0xF101D002,		/* beq.n	28 <success>	*/
		0xE7EB0102,		/* add.w	r1, r1, #2		*/
						/* b.n	0 <code>			*/

		/* 00000028 <success>: */
		0x0580F04F,		/* mov.w	r5, #128		*/
		0xBF00E7FF,		/* b.n	30 <done>			*/
						/* nop (for alignment purposes)	*/

		/* 00000030 <done>: */
		0x0000BE00		/* bkpt	0x0000				*/
	};

	/* see contrib/loaders/flash/armv4_5_cfi_span_16_dq7.s for src */
	static const uint32_t armv4_5_word_16_code_dq7only[] = {
		/* <sp_16_code>:				*/
		0xe0d050b2,		/* ldrh r5, [r0], #2			*/
		0xe1c890b0,		/* strh r9, [r8]				*/
		0xe1cab0b0,		/* strh	r11, [r10]				*/
		0xe1c830b0,		/* strh	r3, [r8]				*/
		0xe1c150b0,		/* strh	r5, [r1]				*/
		0xe1a00000,		/* nop			(mov r0,r0)		*/
		/* <sp_16_busy>:				*/
		0xe1d160b0,		/* ldrh	r6, [r1]				*/
		0xe0257006,		/* eor	r7, r5, r6				*/
		0xe2177080,		/* ands	r7, #0x80				*/
		0x1afffffb,		/* bne	8168 <sp_16_busy>		*/
		/*								*/
		0xe2522001,		/* subs	r2, r2, #1	; 0x1		*/
		0x03a05080,		/* moveq	r5, #128	; 0x80	*/
		0x0a000001,		/* beq	81ac <sp_16_done>		*/
		0xe2811002,		/* add	r1, r1, #2	; 0x2		*/
		0xeafffff0,		/* b	8158 <sp_16_code>		*/
		/* 000081ac <sp_16_done>:		*/
		0xeafffffe		/* b	81ac <sp_16_done>		*/
	};

	/* see contrib/loaders/flash/armv4_5_cfi_span_8.s for src */
	static const uint32_t armv4_5_word_8_code[] = {
		/* 000081b0 <sp_16_code_end>:	*/
		0xe4d05001,		/* ldrb	r5, [r0], #1			*/
		0xe5c89000,		/* strb	r9, [r8]				*/
		0xe5cab000,		/* strb	r11, [r10]				*/
		0xe5c83000,		/* strb	r3, [r8]				*/
		0xe5c15000,		/* strb	r5, [r1]				*/
		0xe1a00000,		/* nop			(mov r0,r0)		*/
		/* 000081c0 <sp_8_busy>:		*/
		0xe5d16000,		/* ldrb	r6, [r1]				*/
		0xe0257006,		/* eor	r7, r5, r6				*/
		0xe0147007,		/* ands	r7, r4, r7				*/
		0x0a000007,		/* beq	81f0 <sp_8_cont>		*/
		0xe0166124,		/* ands	r6, r6, r4, lsr #2		*/
		0x0afffff9,		/* beq	81c0 <sp_8_busy>		*/
		0xe5d16000,		/* ldrb	r6, [r1]				*/
		0xe0257006,		/* eor	r7, r5, r6				*/
		0xe0147007,		/* ands	r7, r4, r7				*/
		0x0a000001,		/* beq	81f0 <sp_8_cont>		*/
		0xe3a05000,		/* mov	r5, #0	; 0x0			*/
		0x1a000004,		/* bne	8204 <sp_8_done>		*/
		/* 000081f0 <sp_8_cont>:		*/
		0xe2522001,		/* subs	r2, r2, #1	; 0x1		*/
		0x03a05080,		/* moveq	r5, #128	; 0x80	*/
		0x0a000001,		/* beq	8204 <sp_8_done>		*/
		0xe2811001,		/* add	r1, r1, #1	; 0x1		*/
		0xeaffffe8,		/* b	81b0 <sp_16_code_end>	*/
		/* 00008204 <sp_8_done>:		*/
		0xeafffffe		/* b	8204 <sp_8_done>		*/
	};

	if (strncmp(target_type_name(target), "mips_m4k", 8) == 0)
		return cfi_spansion_write_block_mips(bank, buffer, address, count);

	if (is_armv7m(target_to_armv7m(target))) {	/* armv7m target */
		armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
		armv7m_algo.core_mode = ARM_MODE_THREAD;
		arm_algo = &armv7m_algo;
	} else if (is_arm(target_to_arm(target))) {
		/* All other ARM CPUs have 32 bit instructions */
		armv4_5_algo.common_magic = ARM_COMMON_MAGIC;
		armv4_5_algo.core_mode = ARM_MODE_SVC;
		armv4_5_algo.core_state = ARM_STATE_ARM;
		arm_algo = &armv4_5_algo;
	} else {
		LOG_ERROR("Unknown architecture");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	int target_code_size = 0;
	const uint32_t *target_code_src = NULL;

	switch (bank->bus_width) {
		case 1:
			if (is_armv7m(target_to_armv7m(target))) {
				LOG_ERROR("Unknown ARM architecture");
				return ERROR_FAIL;
			}
			target_code_src = armv4_5_word_8_code;
			target_code_size = sizeof(armv4_5_word_8_code);
			break;
		case 2:
			/* Check for DQ5 support */
			if (cfi_info->status_poll_mask & (1 << 5)) {
				if (is_armv7m(target_to_armv7m(target))) {
					/* armv7m target */
					target_code_src = armv7m_word_16_code;
					target_code_size = sizeof(armv7m_word_16_code);
				} else { /* armv4_5 target */
					target_code_src = armv4_5_word_16_code;
					target_code_size = sizeof(armv4_5_word_16_code);
				}
			} else {
				/* No DQ5 support. Use DQ7 DATA# polling only. */
				if (is_armv7m(target_to_armv7m(target))) {
					/* armv7m target */
					target_code_src = armv7m_word_16_code_dq7only;
					target_code_size = sizeof(armv7m_word_16_code_dq7only);
				} else { /* armv4_5 target */
					target_code_src = armv4_5_word_16_code_dq7only;
					target_code_size = sizeof(armv4_5_word_16_code_dq7only);
				}
			}
			break;
		case 4:
			if (is_armv7m(target_to_armv7m(target))) {
				LOG_ERROR("Unknown ARM architecture");
				return ERROR_FAIL;
			}
			target_code_src = armv4_5_word_32_code;
			target_code_size = sizeof(armv4_5_word_32_code);
			break;
		default:
			LOG_ERROR("Unsupported bank buswidth %u, can't do block memory writes",
					bank->bus_width);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* flash write code */
	uint8_t *target_code;

	/* convert bus-width dependent algorithm code to correct endianness */
	target_code = malloc(target_code_size);
	if (target_code == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	target_buffer_set_u32_array(target, target_code, target_code_size / 4, target_code_src);

	/* allocate working area */
	retval = target_alloc_working_area(target, target_code_size,
			&write_algorithm);
	if (retval != ERROR_OK) {
		free(target_code);
		return retval;
	}

	/* write algorithm code to working area */
	retval = target_write_buffer(target, write_algorithm->address,
			target_code_size, target_code);
	if (retval != ERROR_OK) {
		free(target_code);
		return retval;
	}

	free(target_code);

	/* the following code still assumes target code is fixed 24*4 bytes */

	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING(
				"not enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "r5", 32, PARAM_IN);
	init_reg_param(&reg_params[6], "r8", 32, PARAM_OUT);
	init_reg_param(&reg_params[7], "r9", 32, PARAM_OUT);
	init_reg_param(&reg_params[8], "r10", 32, PARAM_OUT);
	init_reg_param(&reg_params[9], "r11", 32, PARAM_OUT);

	while (count > 0) {
		uint32_t thisrun_count = (count > buffer_size) ? buffer_size : count;

		retval = target_write_buffer(target, source->address, thisrun_count, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count / bank->bus_width);
		buf_set_u32(reg_params[3].value, 0, 32, cfi_command_val(bank, 0xA0));
		buf_set_u32(reg_params[4].value, 0, 32, cfi_command_val(bank, 0x80));
		buf_set_u32(reg_params[6].value, 0, 32, cfi_flash_address(bank, 0, pri_ext->_unlock1));
		buf_set_u32(reg_params[7].value, 0, 32, 0xaaaaaaaa);
		buf_set_u32(reg_params[8].value, 0, 32, cfi_flash_address(bank, 0, pri_ext->_unlock2));
		buf_set_u32(reg_params[9].value, 0, 32, 0x55555555);

		retval = target_run_algorithm(target, 0, NULL, 10, reg_params,
				write_algorithm->address,
				write_algorithm->address + ((target_code_size) - 4),
				10000, arm_algo);
		if (retval != ERROR_OK)
			break;

		status = buf_get_u32(reg_params[5].value, 0, 32);
		if (status != 0x80) {
			LOG_ERROR("flash write block failed status: 0x%" PRIx32, status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}

	target_free_all_working_areas(target);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[6]);
	destroy_reg_param(&reg_params[7]);
	destroy_reg_param(&reg_params[8]);
	destroy_reg_param(&reg_params[9]);

	return retval;
}

static int cfi_intel_write_word(struct flash_bank *bank, uint8_t *word, uint32_t address)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	cfi_intel_clear_status_register(bank);
	retval = cfi_send_command(bank, 0x40, address);
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_target_write_memory(bank, address, 1, word);
	if (retval != ERROR_OK)
		return retval;

	uint8_t status;
	retval = cfi_intel_wait_status_busy(bank, cfi_info->word_write_timeout, &status);
	if (retval != ERROR_OK)
		return retval;
	if (status != 0x80) {
		retval = cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
		if (retval != ERROR_OK)
			return retval;

		LOG_ERROR("couldn't write word at base " TARGET_ADDR_FMT
				", address 0x%" PRIx32,
				bank->base, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int cfi_intel_write_words(struct flash_bank *bank, const uint8_t *word,
	uint32_t wordcount, uint32_t address)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	/* Calculate buffer size and boundary mask
	 * buffersize is (buffer size per chip) * (number of chips)
	 * bufferwsize is buffersize in words */
	uint32_t buffersize =
		(1UL << cfi_info->max_buf_write_size) * (bank->bus_width / bank->chip_width);
	uint32_t buffermask = buffersize-1;
	uint32_t bufferwsize = buffersize / bank->bus_width;

	/* Check for valid range */
	if (address & buffermask) {
		LOG_ERROR("Write address at base " TARGET_ADDR_FMT ", address 0x%"
				PRIx32 " not aligned to 2^%d boundary",
				bank->base, address, cfi_info->max_buf_write_size);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check for valid size */
	if (wordcount > bufferwsize) {
		LOG_ERROR("Number of data words %" PRIu32 " exceeds available buffersize %" PRIu32,
			wordcount, buffersize);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Write to flash buffer */
	cfi_intel_clear_status_register(bank);

	/* Initiate buffer operation _*/
	retval = cfi_send_command(bank, 0xe8, address);
	if (retval != ERROR_OK)
		return retval;
	uint8_t status;
	retval = cfi_intel_wait_status_busy(bank, cfi_info->buf_write_timeout, &status);
	if (retval != ERROR_OK)
		return retval;
	if (status != 0x80) {
		retval = cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
		if (retval != ERROR_OK)
			return retval;

		LOG_ERROR(
			"couldn't start buffer write operation at base " TARGET_ADDR_FMT
			", address 0x%" PRIx32,
			bank->base,
			address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Write buffer wordcount-1 and data words */
	retval = cfi_send_command(bank, bufferwsize-1, address);
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_target_write_memory(bank, address, bufferwsize, word);
	if (retval != ERROR_OK)
		return retval;

	/* Commit write operation */
	retval = cfi_send_command(bank, 0xd0, address);
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_intel_wait_status_busy(bank, cfi_info->buf_write_timeout, &status);
	if (retval != ERROR_OK)
		return retval;

	if (status != 0x80) {
		retval = cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
		if (retval != ERROR_OK)
			return retval;

		LOG_ERROR("Buffer write at base " TARGET_ADDR_FMT
			", address 0x%" PRIx32 " failed.", bank->base, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int cfi_spansion_write_word(struct flash_bank *bank, uint8_t *word, uint32_t address)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;

	retval = cfi_spansion_unlock_seq(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_send_command(bank, 0xa0, cfi_flash_address(bank, 0, pri_ext->_unlock1));
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_target_write_memory(bank, address, 1, word);
	if (retval != ERROR_OK)
		return retval;

	if (cfi_spansion_wait_status_busy(bank, cfi_info->word_write_timeout) != ERROR_OK) {
		retval = cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
		if (retval != ERROR_OK)
			return retval;

		LOG_ERROR("couldn't write word at base " TARGET_ADDR_FMT
			", address 0x%" PRIx32, bank->base, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int cfi_spansion_write_words(struct flash_bank *bank, const uint8_t *word,
	uint32_t wordcount, uint32_t address)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	/* Calculate buffer size and boundary mask
	 * buffersize is (buffer size per chip) * (number of chips)
	 * bufferwsize is buffersize in words */
	uint32_t buffersize =
		(1UL << cfi_info->max_buf_write_size) * (bank->bus_width / bank->chip_width);
	uint32_t buffermask = buffersize-1;
	uint32_t bufferwsize = buffersize / bank->bus_width;

	/* Check for valid range */
	if (address & buffermask) {
		LOG_ERROR("Write address at base " TARGET_ADDR_FMT
			", address 0x%" PRIx32 " not aligned to 2^%d boundary",
			bank->base, address, cfi_info->max_buf_write_size);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check for valid size */
	if (wordcount > bufferwsize) {
		LOG_ERROR("Number of data words %" PRIu32 " exceeds available buffersize %"
			PRIu32, wordcount, buffersize);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Unlock */
	retval = cfi_spansion_unlock_seq(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Buffer load command */
	retval = cfi_send_command(bank, 0x25, address);
	if (retval != ERROR_OK)
		return retval;

	/* Write buffer wordcount-1 and data words */
	retval = cfi_send_command(bank, bufferwsize-1, address);
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_target_write_memory(bank, address, bufferwsize, word);
	if (retval != ERROR_OK)
		return retval;

	/* Commit write operation */
	retval = cfi_send_command(bank, 0x29, address);
	if (retval != ERROR_OK)
		return retval;

	if (cfi_spansion_wait_status_busy(bank, cfi_info->buf_write_timeout) != ERROR_OK) {
		retval = cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
		if (retval != ERROR_OK)
			return retval;

		LOG_ERROR("couldn't write block at base " TARGET_ADDR_FMT
			", address 0x%" PRIx32 ", size 0x%" PRIx32, bank->base, address,
			bufferwsize);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int cfi_write_word(struct flash_bank *bank, uint8_t *word, uint32_t address)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	switch (cfi_info->pri_id) {
		case 1:
		case 3:
			return cfi_intel_write_word(bank, word, address);
		case 2:
			return cfi_spansion_write_word(bank, word, address);
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_FLASH_OPERATION_FAILED;
}

static int cfi_write_words(struct flash_bank *bank, const uint8_t *word,
	uint32_t wordcount, uint32_t address)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	if (cfi_info->buf_write_timeout_typ == 0) {
		/* buffer writes are not supported */
		LOG_DEBUG("Buffer Writes Not Supported");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	switch (cfi_info->pri_id) {
		case 1:
		case 3:
			return cfi_intel_write_words(bank, word, wordcount, address);
		case 2:
			return cfi_spansion_write_words(bank, word, wordcount, address);
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_FLASH_OPERATION_FAILED;
}

static int cfi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	uint32_t address = bank->base + offset;
	uint32_t read_p;
	int align;	/* number of unaligned bytes */
	uint8_t current_word[CFI_MAX_BUS_WIDTH];
	int retval;

	LOG_DEBUG("reading buffer of %i byte at 0x%8.8x",
		(int)count, (unsigned)offset);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* start at the first byte of the first word (bus_width size) */
	read_p = address & ~(bank->bus_width - 1);
	align = address - read_p;
	if (align != 0) {
		LOG_INFO("Fixup %d unaligned read head bytes", align);

		/* read a complete word from flash */
		retval = cfi_target_read_memory(bank, read_p, 1, current_word);
		if (retval != ERROR_OK)
			return retval;

		/* take only bytes we need */
		for (unsigned int i = align; (i < bank->bus_width) && (count > 0); i++, count--)
			*buffer++ = current_word[i];

		read_p += bank->bus_width;
	}

	align = count / bank->bus_width;
	if (align) {
		retval = cfi_target_read_memory(bank, read_p, align, buffer);
		if (retval != ERROR_OK)
			return retval;

		read_p += align * bank->bus_width;
		buffer += align * bank->bus_width;
		count -= align * bank->bus_width;
	}

	if (count) {
		LOG_INFO("Fixup %" PRIu32 " unaligned read tail bytes", count);

		/* read a complete word from flash */
		retval = cfi_target_read_memory(bank, read_p, 1, current_word);
		if (retval != ERROR_OK)
			return retval;

		/* take only bytes we need */
		for (unsigned int i = 0; (i < bank->bus_width) && (count > 0); i++, count--)
			*buffer++ = current_word[i];
	}

	return ERROR_OK;
}

static int cfi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	uint32_t address = bank->base + offset;	/* address of first byte to be programmed */
	uint32_t write_p;
	int align;	/* number of unaligned bytes */
	int blk_count;	/* number of bus_width bytes for block copy */
	uint8_t current_word[CFI_MAX_BUS_WIDTH * 4];	/* word (bus_width size) currently being
							 *programmed */
	uint8_t *swapped_buffer = NULL;
	const uint8_t *real_buffer = NULL;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* start at the first byte of the first word (bus_width size) */
	write_p = address & ~(bank->bus_width - 1);
	align = address - write_p;
	if (align != 0) {
		LOG_INFO("Fixup %d unaligned head bytes", align);

		/* read a complete word from flash */
		retval = cfi_target_read_memory(bank, write_p, 1, current_word);
		if (retval != ERROR_OK)
			return retval;

		/* replace only bytes that must be written */
		for (unsigned int i = align; (i < bank->bus_width) && (count > 0); i++, count--)
			if (cfi_info->data_swap)
				/* data bytes are swapped (reverse endianness) */
				current_word[bank->bus_width - i] = *buffer++;
			else
				current_word[i] = *buffer++;

		retval = cfi_write_word(bank, current_word, write_p);
		if (retval != ERROR_OK)
			return retval;
		write_p += bank->bus_width;
	}

	if (cfi_info->data_swap && count) {
		swapped_buffer = malloc(count & ~(bank->bus_width - 1));
		switch (bank->bus_width) {
		case 2:
			buf_bswap16(swapped_buffer, buffer,
				    count & ~(bank->bus_width - 1));
			break;
		case 4:
			buf_bswap32(swapped_buffer, buffer,
				    count & ~(bank->bus_width - 1));
			break;
		}
		real_buffer = buffer;
		buffer = swapped_buffer;
	}

	/* handle blocks of bus_size aligned bytes */
	blk_count = count & ~(bank->bus_width - 1);	/* round down, leave tail bytes */
	switch (cfi_info->pri_id) {
		/* try block writes (fails without working area) */
		case 1:
		case 3:
			retval = cfi_intel_write_block(bank, buffer, write_p, blk_count);
			break;
		case 2:
			retval = cfi_spansion_write_block(bank, buffer, write_p, blk_count);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
	}
	if (retval == ERROR_OK) {
		/* Increment pointers and decrease count on successful block write */
		buffer += blk_count;
		write_p += blk_count;
		count -= blk_count;
	} else {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* Calculate buffer size and boundary mask
			 * buffersize is (buffer size per chip) * (number of chips)
			 * bufferwsize is buffersize in words */
			uint32_t buffersize =
				(1UL <<
				 cfi_info->max_buf_write_size) *
				(bank->bus_width / bank->chip_width);
			uint32_t buffermask = buffersize-1;
			uint32_t bufferwsize = buffersize / bank->bus_width;

			/* fall back to memory writes */
			while (count >= (uint32_t)bank->bus_width) {
				bool fallback;
				if ((write_p & 0xff) == 0) {
					LOG_INFO("Programming at 0x%08" PRIx32 ", count 0x%08"
						PRIx32 " bytes remaining", write_p, count);
				}
				fallback = true;
				if ((bufferwsize > 0) && (count >= buffersize) &&
						!(write_p & buffermask)) {
					retval = cfi_write_words(bank, buffer, bufferwsize, write_p);
					if (retval == ERROR_OK) {
						buffer += buffersize;
						write_p += buffersize;
						count -= buffersize;
						fallback = false;
					} else if (retval != ERROR_FLASH_OPER_UNSUPPORTED)
						return retval;
				}
				/* try the slow way? */
				if (fallback) {
					for (unsigned int i = 0; i < bank->bus_width; i++)
						current_word[i] = *buffer++;

					retval = cfi_write_word(bank, current_word, write_p);
					if (retval != ERROR_OK)
						return retval;

					write_p += bank->bus_width;
					count -= bank->bus_width;
				}
			}
		} else
			return retval;
	}

	if (swapped_buffer) {
		buffer = real_buffer + (buffer - swapped_buffer);
		free(swapped_buffer);
	}

	/* return to read array mode, so we can read from flash again for padding */
	retval = cfi_reset(bank);
	if (retval != ERROR_OK)
		return retval;

	/* handle unaligned tail bytes */
	if (count > 0) {
		LOG_INFO("Fixup %" PRIu32 " unaligned tail bytes", count);

		/* read a complete word from flash */
		retval = cfi_target_read_memory(bank, write_p, 1, current_word);
		if (retval != ERROR_OK)
			return retval;

		/* replace only bytes that must be written */
		for (unsigned int i = 0; (i < bank->bus_width) && (count > 0); i++, count--)
			if (cfi_info->data_swap)
				/* data bytes are swapped (reverse endianness) */
				current_word[bank->bus_width - i] = *buffer++;
			else
				current_word[i] = *buffer++;

		retval = cfi_write_word(bank, current_word, write_p);
		if (retval != ERROR_OK)
			return retval;
	}

	/* return to read array mode */
	return cfi_reset(bank);
}

static void cfi_fixup_reversed_erase_regions(struct flash_bank *bank, const void *param)
{
	(void) param;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;

	pri_ext->_reversed_geometry = 1;
}

static void cfi_fixup_0002_erase_regions(struct flash_bank *bank, const void *param)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;
	(void) param;

	if ((pri_ext->_reversed_geometry) || (pri_ext->TopBottom == 3)) {
		LOG_DEBUG("swapping reversed erase region information on cmdset 0002 device");

		for (unsigned int i = 0; i < cfi_info->num_erase_regions / 2; i++) {
			int j = (cfi_info->num_erase_regions - 1) - i;
			uint32_t swap;

			swap = cfi_info->erase_region_info[i];
			cfi_info->erase_region_info[i] = cfi_info->erase_region_info[j];
			cfi_info->erase_region_info[j] = swap;
		}
	}
}

static void cfi_fixup_0002_unlock_addresses(struct flash_bank *bank, const void *param)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;
	const struct cfi_unlock_addresses *unlock_addresses = param;

	pri_ext->_unlock1 = unlock_addresses->unlock1;
	pri_ext->_unlock2 = unlock_addresses->unlock2;
}

static void cfi_fixup_0002_polling_bits(struct flash_bank *bank, const void *param)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	const int *status_poll_mask = param;

	cfi_info->status_poll_mask = *status_poll_mask;
}


static int cfi_query_string(struct flash_bank *bank, int address)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	int retval;

	retval = cfi_send_command(bank, 0x98, cfi_flash_address(bank, 0, address));
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_query_u8(bank, 0, 0x10, &cfi_info->qry[0]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, 0x11, &cfi_info->qry[1]);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_query_u8(bank, 0, 0x12, &cfi_info->qry[2]);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("CFI qry returned: 0x%2.2x 0x%2.2x 0x%2.2x",
		cfi_info->qry[0], cfi_info->qry[1], cfi_info->qry[2]);

	if ((cfi_info->qry[0] != 'Q') || (cfi_info->qry[1] != 'R') || (cfi_info->qry[2] != 'Y')) {
		retval = cfi_reset(bank);
		if (retval != ERROR_OK)
			return retval;
		LOG_ERROR("Could not probe bank: no QRY");
		return ERROR_FLASH_BANK_INVALID;
	}

	return ERROR_OK;
}

int cfi_probe(struct flash_bank *bank)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct target *target = bank->target;
	unsigned int num_sectors = 0;
	int sector = 0;
	uint32_t unlock1 = 0x555;
	uint32_t unlock2 = 0x2aa;
	int retval;
	uint8_t value_buf0[CFI_MAX_BUS_WIDTH], value_buf1[CFI_MAX_BUS_WIDTH];

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	cfi_info->probed = false;
	cfi_info->num_erase_regions = 0;

	free(bank->sectors);
	bank->sectors = NULL;

	free(cfi_info->erase_region_info);
	cfi_info->erase_region_info = NULL;

	/* JEDEC standard JESD21C uses 0x5555 and 0x2aaa as unlock addresses,
	 * while CFI compatible AMD/Spansion flashes use 0x555 and 0x2aa
	 */
	if (cfi_info->jedec_probe) {
		unlock1 = 0x5555;
		unlock2 = 0x2aaa;
	}

	/* switch to read identifier codes mode ("AUTOSELECT") */
	retval = cfi_send_command(bank, 0xaa, cfi_flash_address(bank, 0, unlock1));
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_send_command(bank, 0x55, cfi_flash_address(bank, 0, unlock2));
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_send_command(bank, 0x90, cfi_flash_address(bank, 0, unlock1));
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_target_read_memory(bank, cfi_flash_address(bank, 0, 0x00),
					1, value_buf0);
	if (retval != ERROR_OK)
		return retval;
	retval = cfi_target_read_memory(bank, cfi_flash_address(bank, 0, 0x01),
					1, value_buf1);
	if (retval != ERROR_OK)
		return retval;
	switch (bank->chip_width) {
		case 1:
			cfi_info->manufacturer = *value_buf0;
			cfi_info->device_id = *value_buf1;
			break;
		case 2:
			cfi_info->manufacturer = target_buffer_get_u16(target, value_buf0);
			cfi_info->device_id = target_buffer_get_u16(target, value_buf1);
			break;
		case 4:
			cfi_info->manufacturer = target_buffer_get_u32(target, value_buf0);
			cfi_info->device_id = target_buffer_get_u32(target, value_buf1);
			break;
		default:
			LOG_ERROR("Unsupported bank chipwidth %u, can't probe memory",
					bank->chip_width);
			return ERROR_FLASH_OPERATION_FAILED;
	}

	LOG_INFO("Flash Manufacturer/Device: 0x%04x 0x%04x",
		cfi_info->manufacturer, cfi_info->device_id);
	/* switch back to read array mode */
	retval = cfi_reset(bank);
	if (retval != ERROR_OK)
		return retval;

	/* check device/manufacturer ID for known non-CFI flashes. */
	cfi_fixup_non_cfi(bank);

	/* query only if this is a CFI compatible flash,
	 * otherwise the relevant info has already been filled in
	 */
	if (!cfi_info->not_cfi) {
		/* enter CFI query mode
		 * according to JEDEC Standard No. 68.01,
		 * a single bus sequence with address = 0x55, data = 0x98 should put
		 * the device into CFI query mode.
		 *
		 * SST flashes clearly violate this, and we will consider them incompatible for now
		 */

		retval = cfi_query_string(bank, 0x55);
		if (retval != ERROR_OK) {
			/*
			 * Spansion S29WS-N CFI query fix is to try 0x555 if 0x55 fails. Should
			 * be harmless enough:
			 *
			 * http://www.infradead.org/pipermail/linux-mtd/2005-September/013618.html
			 */
			LOG_USER("Try workaround w/0x555 instead of 0x55 to get QRY.");
			retval = cfi_query_string(bank, 0x555);
		}
		if (retval != ERROR_OK)
			return retval;

		retval = cfi_query_u16(bank, 0, 0x13, &cfi_info->pri_id);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u16(bank, 0, 0x15, &cfi_info->pri_addr);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u16(bank, 0, 0x17, &cfi_info->alt_id);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u16(bank, 0, 0x19, &cfi_info->alt_addr);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("qry: '%c%c%c', pri_id: 0x%4.4x, pri_addr: 0x%4.4x, alt_id: "
			"0x%4.4x, alt_addr: 0x%4.4x", cfi_info->qry[0], cfi_info->qry[1],
			cfi_info->qry[2], cfi_info->pri_id, cfi_info->pri_addr,
			cfi_info->alt_id, cfi_info->alt_addr);

		retval = cfi_query_u8(bank, 0, 0x1b, &cfi_info->vcc_min);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x1c, &cfi_info->vcc_max);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x1d, &cfi_info->vpp_min);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x1e, &cfi_info->vpp_max);
		if (retval != ERROR_OK)
			return retval;

		retval = cfi_query_u8(bank, 0, 0x1f, &cfi_info->word_write_timeout_typ);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x20, &cfi_info->buf_write_timeout_typ);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x21, &cfi_info->block_erase_timeout_typ);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x22, &cfi_info->chip_erase_timeout_typ);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x23, &cfi_info->word_write_timeout_max);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x24, &cfi_info->buf_write_timeout_max);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x25, &cfi_info->block_erase_timeout_max);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x26, &cfi_info->chip_erase_timeout_max);
		if (retval != ERROR_OK)
			return retval;

		uint8_t data;
		retval = cfi_query_u8(bank, 0, 0x27, &data);
		if (retval != ERROR_OK)
			return retval;
		cfi_info->dev_size = 1 << data;

		retval = cfi_query_u16(bank, 0, 0x28, &cfi_info->interface_desc);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u16(bank, 0, 0x2a, &cfi_info->max_buf_write_size);
		if (retval != ERROR_OK)
			return retval;
		retval = cfi_query_u8(bank, 0, 0x2c, &cfi_info->num_erase_regions);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("size: 0x%" PRIx32 ", interface desc: %i, max buffer write size: 0x%x",
			cfi_info->dev_size, cfi_info->interface_desc,
			(1 << cfi_info->max_buf_write_size));

		if (cfi_info->num_erase_regions) {
			cfi_info->erase_region_info = malloc(sizeof(*cfi_info->erase_region_info)
					* cfi_info->num_erase_regions);
			for (unsigned int i = 0; i < cfi_info->num_erase_regions; i++) {
				retval = cfi_query_u32(bank,
						0,
						0x2d + (4 * i),
						&cfi_info->erase_region_info[i]);
				if (retval != ERROR_OK)
					return retval;
				LOG_DEBUG(
					"erase region[%i]: %" PRIu32 " blocks of size 0x%" PRIx32 "",
					i,
					(cfi_info->erase_region_info[i] & 0xffff) + 1,
					(cfi_info->erase_region_info[i] >> 16) * 256);
			}
		} else
			cfi_info->erase_region_info = NULL;

		/* We need to read the primary algorithm extended query table before calculating
		 * the sector layout to be able to apply fixups
		 */
		switch (cfi_info->pri_id) {
			/* Intel command set (standard and extended) */
			case 0x0001:
			case 0x0003:
				cfi_read_intel_pri_ext(bank);
				break;
			/* AMD/Spansion, Atmel, ... command set */
			case 0x0002:
				cfi_info->status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7;	/*
												 *default
												 *for
												 *all
												 *CFI
												 *flashes
												 **/
				cfi_read_0002_pri_ext(bank);
				break;
			default:
				LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
				break;
		}

		/* return to read array mode
		 * we use both reset commands, as some Intel flashes fail to recognize the 0xF0 command
		 */
		retval = cfi_reset(bank);
		if (retval != ERROR_OK)
			return retval;
	}	/* end CFI case */

	LOG_DEBUG("Vcc min: %x.%x, Vcc max: %x.%x, Vpp min: %u.%x, Vpp max: %u.%x",
		(cfi_info->vcc_min & 0xf0) >> 4, cfi_info->vcc_min & 0x0f,
		(cfi_info->vcc_max & 0xf0) >> 4, cfi_info->vcc_max & 0x0f,
		(cfi_info->vpp_min & 0xf0) >> 4, cfi_info->vpp_min & 0x0f,
		(cfi_info->vpp_max & 0xf0) >> 4, cfi_info->vpp_max & 0x0f);

	LOG_DEBUG("typ. word write timeout: %u us, typ. buf write timeout: %u us, "
		"typ. block erase timeout: %u ms, typ. chip erase timeout: %u ms",
		1 << cfi_info->word_write_timeout_typ, 1 << cfi_info->buf_write_timeout_typ,
		1 << cfi_info->block_erase_timeout_typ, 1 << cfi_info->chip_erase_timeout_typ);

	LOG_DEBUG("max. word write timeout: %u us, max. buf write timeout: %u us, "
		"max. block erase timeout: %u ms, max. chip erase timeout: %u ms",
		(1 << cfi_info->word_write_timeout_max) * (1 << cfi_info->word_write_timeout_typ),
		(1 << cfi_info->buf_write_timeout_max) * (1 << cfi_info->buf_write_timeout_typ),
		(1 << cfi_info->block_erase_timeout_max) * (1 << cfi_info->block_erase_timeout_typ),
		(1 << cfi_info->chip_erase_timeout_max) * (1 << cfi_info->chip_erase_timeout_typ));

	/* convert timeouts to real values in ms */
	cfi_info->word_write_timeout = DIV_ROUND_UP((1L << cfi_info->word_write_timeout_typ) *
			(1L << cfi_info->word_write_timeout_max), 1000);
	cfi_info->buf_write_timeout = DIV_ROUND_UP((1L << cfi_info->buf_write_timeout_typ) *
			(1L << cfi_info->buf_write_timeout_max), 1000);
	cfi_info->block_erase_timeout = (1L << cfi_info->block_erase_timeout_typ) *
		(1L << cfi_info->block_erase_timeout_max);
	cfi_info->chip_erase_timeout = (1L << cfi_info->chip_erase_timeout_typ) *
		(1L << cfi_info->chip_erase_timeout_max);

	LOG_DEBUG("calculated word write timeout: %u ms, buf write timeout: %u ms, "
		"block erase timeout: %u ms, chip erase timeout: %u ms",
		cfi_info->word_write_timeout, cfi_info->buf_write_timeout,
		cfi_info->block_erase_timeout, cfi_info->chip_erase_timeout);

	/* apply fixups depending on the primary command set */
	switch (cfi_info->pri_id) {
		/* Intel command set (standard and extended) */
		case 0x0001:
		case 0x0003:
			cfi_fixup(bank, cfi_0001_fixups);
			break;
		/* AMD/Spansion, Atmel, ... command set */
		case 0x0002:
			cfi_fixup(bank, cfi_0002_fixups);
			break;
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	if ((cfi_info->dev_size * bank->bus_width / bank->chip_width) != bank->size) {
		LOG_WARNING("configuration specifies 0x%" PRIx32 " size, but a 0x%" PRIx32
			" size flash was found", bank->size, cfi_info->dev_size);
	}

	if (cfi_info->num_erase_regions == 0) {
		/* a device might have only one erase block, spanning the whole device */
		bank->num_sectors = 1;
		bank->sectors = malloc(sizeof(struct flash_sector));

		bank->sectors[sector].offset = 0x0;
		bank->sectors[sector].size = bank->size;
		bank->sectors[sector].is_erased = -1;
		bank->sectors[sector].is_protected = -1;
	} else {
		uint32_t offset = 0;

		for (unsigned int i = 0; i < cfi_info->num_erase_regions; i++)
			num_sectors += (cfi_info->erase_region_info[i] & 0xffff) + 1;

		bank->num_sectors = num_sectors;
		bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);

		for (unsigned int i = 0; i < cfi_info->num_erase_regions; i++) {
			for (uint32_t j = 0; j < (cfi_info->erase_region_info[i] & 0xffff) + 1; j++) {
				bank->sectors[sector].offset = offset;
				bank->sectors[sector].size =
					((cfi_info->erase_region_info[i] >> 16) * 256)
					* bank->bus_width / bank->chip_width;
				offset += bank->sectors[sector].size;
				bank->sectors[sector].is_erased = -1;
				bank->sectors[sector].is_protected = -1;
				sector++;
			}
		}
		if (offset != (cfi_info->dev_size * bank->bus_width / bank->chip_width)) {
			LOG_WARNING(
				"CFI size is 0x%" PRIx32 ", but total sector size is 0x%" PRIx32 "",
				(cfi_info->dev_size * bank->bus_width / bank->chip_width),
				offset);
		}
	}

	cfi_info->probed = true;

	return ERROR_OK;
}

int cfi_auto_probe(struct flash_bank *bank)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	if (cfi_info->probed)
		return ERROR_OK;
	return cfi_probe(bank);
}

static int cfi_intel_protect_check(struct flash_bank *bank)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_intel_pri_ext *pri_ext = cfi_info->pri_ext;

	/* check if block lock bits are supported on this device */
	if (!(pri_ext->blk_status_reg_mask & 0x1))
		return ERROR_FLASH_OPERATION_FAILED;

	retval = cfi_send_command(bank, 0x90, cfi_flash_address(bank, 0, 0x55));
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		uint8_t block_status;
		retval = cfi_get_u8(bank, i, 0x2, &block_status);
		if (retval != ERROR_OK)
			return retval;

		if (block_status & 1)
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}

	return cfi_send_command(bank, 0xff, cfi_flash_address(bank, 0, 0x0));
}

static int cfi_spansion_protect_check(struct flash_bank *bank)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;

	retval = cfi_spansion_unlock_seq(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = cfi_send_command(bank, 0x90, cfi_flash_address(bank, 0, pri_ext->_unlock1));
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		uint8_t block_status;
		retval = cfi_get_u8(bank, i, 0x2, &block_status);
		if (retval != ERROR_OK)
			return retval;

		if (block_status & 1)
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}

	return cfi_send_command(bank, 0xf0, cfi_flash_address(bank, 0, 0x0));
}

int cfi_protect_check(struct flash_bank *bank)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	switch (cfi_info->pri_id) {
		case 1:
		case 3:
			return cfi_intel_protect_check(bank);
		case 2:
			return cfi_spansion_protect_check(bank);
		default:
			LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
			break;
	}

	return ERROR_OK;
}

int cfi_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	if (cfi_info->qry[0] == 0xff) {
		snprintf(buf, buf_size, "\ncfi flash bank not probed yet\n");
		return ERROR_OK;
	}

	if (!cfi_info->not_cfi)
		printed = snprintf(buf, buf_size, "\nCFI flash: ");
	else
		printed = snprintf(buf, buf_size, "\nnon-CFI flash: ");
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "mfr: 0x%4.4x, id:0x%4.4x\n\n",
			cfi_info->manufacturer, cfi_info->device_id);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "qry: '%c%c%c', pri_id: 0x%4.4x, pri_addr: "
			"0x%4.4x, alt_id: 0x%4.4x, alt_addr: 0x%4.4x\n",
			cfi_info->qry[0], cfi_info->qry[1], cfi_info->qry[2],
			cfi_info->pri_id, cfi_info->pri_addr, cfi_info->alt_id, cfi_info->alt_addr);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "Vcc min: %x.%x, Vcc max: %x.%x, "
			"Vpp min: %u.%x, Vpp max: %u.%x\n",
			(cfi_info->vcc_min & 0xf0) >> 4, cfi_info->vcc_min & 0x0f,
			(cfi_info->vcc_max & 0xf0) >> 4, cfi_info->vcc_max & 0x0f,
			(cfi_info->vpp_min & 0xf0) >> 4, cfi_info->vpp_min & 0x0f,
			(cfi_info->vpp_max & 0xf0) >> 4, cfi_info->vpp_max & 0x0f);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "typ. word write timeout: %u us, "
			"typ. buf write timeout: %u us, "
			"typ. block erase timeout: %u ms, "
			"typ. chip erase timeout: %u ms\n",
			1 << cfi_info->word_write_timeout_typ,
			1 << cfi_info->buf_write_timeout_typ,
			1 << cfi_info->block_erase_timeout_typ,
			1 << cfi_info->chip_erase_timeout_typ);
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf,
			buf_size,
			"max. word write timeout: %u us, "
			"max. buf write timeout: %u us, max. "
			"block erase timeout: %u ms, max. chip erase timeout: %u ms\n",
			(1 <<
			 cfi_info->word_write_timeout_max) * (1 << cfi_info->word_write_timeout_typ),
			(1 <<
			 cfi_info->buf_write_timeout_max) * (1 << cfi_info->buf_write_timeout_typ),
			(1 <<
			 cfi_info->block_erase_timeout_max) *
			(1 << cfi_info->block_erase_timeout_typ),
			(1 <<
			 cfi_info->chip_erase_timeout_max) *
			(1 << cfi_info->chip_erase_timeout_typ));
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, "size: 0x%" PRIx32 ", interface desc: %i, "
			"max buffer write size: 0x%x\n",
			cfi_info->dev_size,
			cfi_info->interface_desc,
			1 << cfi_info->max_buf_write_size);
	buf += printed;
	buf_size -= printed;

	switch (cfi_info->pri_id) {
	    case 1:
	    case 3:
		    cfi_intel_info(bank, buf, buf_size);
		    break;
	    case 2:
		    cfi_spansion_info(bank, buf, buf_size);
		    break;
	    default:
		    LOG_ERROR("cfi primary command set %i unsupported", cfi_info->pri_id);
		    break;
	}

	return ERROR_OK;
}

static void cfi_fixup_0002_write_buffer(struct flash_bank *bank, const void *param)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;

	/* disable write buffer for M29W128G */
	cfi_info->buf_write_timeout_typ = 0;
}

const struct flash_driver cfi_flash = {
	.name = "cfi",
	.flash_bank_command = cfi_flash_bank_command,
	.erase = cfi_erase,
	.protect = cfi_protect,
	.write = cfi_write,
	.read = cfi_read,
	.probe = cfi_probe,
	.auto_probe = cfi_auto_probe,
	/* FIXME: access flash at bus_width size */
	.erase_check = default_flash_blank_check,
	.protect_check = cfi_protect_check,
	.info = cfi_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
