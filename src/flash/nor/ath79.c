/***************************************************************************
 *   Copyright (C) 2015 by Tobias Diedrich                                 *
 *   <ranma+openwrt@tdiedrich.de>                                          *
 *                                                                         *
 *   based on the stmsmi code written by Antonio Borneo                    *
 *   <borneo.antonio@gmail.com>                                            *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.                                        *
 *                                                                         *
 ***************************************************************************/
/*
 * Driver for the Atheros AR7xxx/AR9xxx SPI flash interface.
 *
 * Since no SPI mode register is present, presumably only
 * SPI "mode 3" (CPOL=1 and CPHA=1) is supported.
 *
 * The SPI interface supports up to 3 chip selects, however the SPI flash
 * used for booting the system must be connected to CS0.
 *
 * On boot, the first 4MiB of flash space are memory-mapped into the
 * area bf000000 - bfffffff (4 copies), so the MIPS bootstrap
 * vector bfc00000 is mapped to the beginning of the flash.
 *
 * By writing a 1 to the REMAP_DISABLE bit in the SPI_CONTROL register,
 * the full area of 16MiB is mapped.
 *
 * By writing a 0 to the SPI_FUNCTION_SELECT register (write-only dword
 * register @bf000000), memory mapping is disabled and the SPI registers
 * are exposed to the CPU instead:
 * bf000000 SPI_FUNCTION_SELECT
 * bf000004 SPI_CONTROL
 * bf000008 SPI_IO_CONTROL
 * bf00000c SPI_READ_DATA
 *
 * When not memory-mapped, the SPI interface is essentially bitbanged
 * using SPI_CONTROL and SPI_IO_CONTROL with the only hardware-assistance
 * being the 32bit read-only shift-register SPI_READ_DATA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <helper/types.h>
#include <target/mips32.h>
#include <target/mips32_pracc.h>
#include <target/target.h>

#define BITS_PER_BYTE 8

#define ATH79_REG_FS     0
#define ATH79_REG_CLOCK  4
#define ATH79_REG_WRITE  8
#define ATH79_REG_DATA  12

#define ATH79_SPI_CS_ALLHI 0x70000
#define ATH79_SPI_CS0_HI   0x10000
#define ATH79_SPI_CS1_HI   0x20000
#define ATH79_SPI_CS2_HI   0x40000
#define ATH79_SPI_CE_HI    0x00100
#define ATH79_SPI_DO_HI    0x00001

#define ATH79_XFER_FINAL   0x00000001
#define ATH79_XFER_PARTIAL 0x00000000

/* Timeout in ms */
#define ATH79_MAX_TIMEOUT  (3000)

struct ath79_spi_ctx {
	uint8_t *page_buf;
	int pre_deselect;
	int post_deselect;
};

struct ath79_flash_bank {
	bool probed;
	int chipselect;
	uint32_t io_base;
	const struct flash_device *dev;
	struct ath79_spi_ctx spi;
};

struct ath79_target {
	char *name;
	uint32_t tap_idcode;
	uint32_t io_base;
};

static const struct ath79_target target_devices[] = {
	/* name,   tap_idcode, io_base */
	{ "ATH79", 0x00000001, 0xbf000000 },
	{ NULL,    0,          0 }
};

static const uint32_t ath79_chipselects[] = {
	(~ATH79_SPI_CS0_HI & ATH79_SPI_CS_ALLHI),
	(~ATH79_SPI_CS1_HI & ATH79_SPI_CS_ALLHI),
	(~ATH79_SPI_CS2_HI & ATH79_SPI_CS_ALLHI),
};

static void ath79_pracc_addn(struct pracc_queue_info *ctx,
			     const uint32_t *instr,
			     int n)
{
	for (int i = 0; i < n; i++)
		pracc_add(ctx, 0, instr[i]);
}

static int ath79_spi_bitbang_codegen(struct ath79_flash_bank *ath79_info,
				     struct pracc_queue_info *ctx,
				     uint8_t *data, int len,
				     int partial_xfer)
{
	uint32_t cs_high = ATH79_SPI_CS_ALLHI;
	uint32_t cs_low = ath79_chipselects[ath79_info->chipselect];
	uint32_t clock_high = cs_low | ATH79_SPI_CE_HI;
	uint32_t clock_low = cs_low;
	uint32_t pracc_out = 0;
	uint32_t io_base = ath79_info->io_base;

	const uint32_t preamble1[] = {
		/* $15 = MIPS32_PRACC_BASE_ADDR */
		MIPS32_LUI(0, 15, PRACC_UPPER_BASE_ADDR),
		/* $1 = io_base */
		MIPS32_LUI(0, 1, UPPER16(io_base)),
	};
	ath79_pracc_addn(ctx, preamble1, ARRAY_SIZE(preamble1));
	if (ath79_info->spi.pre_deselect) {
		/* Clear deselect flag so we don't deselect again if
		 * this is a partial xfer.
		 */
		ath79_info->spi.pre_deselect = 0;
		const uint32_t pre_deselect[] = {
			/* [$1 + FS] = 1  (enable flash io register access) */
			MIPS32_LUI(0, 2, UPPER16(1)),
			MIPS32_ORI(0, 2, 2, LOWER16(1)),
			MIPS32_SW(0, 2, ATH79_REG_FS, 1),
			/* deselect flash just in case */
			/* $2 = SPI_CS_DIS */
			MIPS32_LUI(0, 2, UPPER16(cs_high)),
			MIPS32_ORI(0, 2, 2, LOWER16(cs_high)),
			/* [$1 + WRITE] = $2 */
			MIPS32_SW(0, 2, ATH79_REG_WRITE, 1),
		};
		ath79_pracc_addn(ctx, pre_deselect, ARRAY_SIZE(pre_deselect));
	}
	const uint32_t preamble2[] = {
		/* t0 = CLOCK_LOW + 0-bit */
		MIPS32_LUI(0, 8, UPPER16((clock_low + 0))),
		MIPS32_ORI(0, 8, 8, LOWER16((clock_low + 0))),
		/* t1 = CLOCK_LOW + 1-bit */
		MIPS32_LUI(0, 9, UPPER16((clock_low + 1))),
		MIPS32_ORI(0, 9, 9, LOWER16((clock_low + 1))),
		/* t2 = CLOCK_HIGH + 0-bit */
		MIPS32_LUI(0, 10, UPPER16((clock_high + 0))),
		MIPS32_ORI(0, 10, 10, LOWER16((clock_high + 0))),
		/* t3 = CLOCK_HIGH + 1-bit */
		MIPS32_LUI(0, 11, UPPER16((clock_high + 1))),
		MIPS32_ORI(0, 11, 11, LOWER16((clock_high + 1))),
	};
	ath79_pracc_addn(ctx, preamble2, ARRAY_SIZE(preamble2));

	for (int i = 0; i < len; i++) {
		uint8_t x = data[i];

		/* Generate bitbang code for one byte, highest bit first .*/
		for (int j = BITS_PER_BYTE - 1; j >= 0; j--) {
			int bit = ((x >> j) & 1);

			if (bit) {
				/* [$1 + WRITE] = t1 */
				pracc_add(ctx, 0,
					  MIPS32_SW(0, 9, ATH79_REG_WRITE, 1));
				/* [$1 + WRITE] = t3 */
				pracc_add(ctx, 0,
					  MIPS32_SW(0, 11, ATH79_REG_WRITE, 1));
			} else {
				/* [$1 + WRITE] = t0 */
				pracc_add(ctx, 0,
					  MIPS32_SW(0, 8, ATH79_REG_WRITE, 1));
				/* [$1 + WRITE] = t2 */
				pracc_add(ctx, 0,
					  MIPS32_SW(0, 10, ATH79_REG_WRITE, 1));
			}
		}
		if (i % 4 == 3) {
			/* $3 = [$1 + DATA] */
			pracc_add(ctx, 0, MIPS32_LW(0, 3, ATH79_REG_DATA, 1));
			/* [OUTi] = $3 */
			pracc_add(ctx, MIPS32_PRACC_PARAM_OUT + pracc_out,
				  MIPS32_SW(0, 3, PRACC_OUT_OFFSET +
				 pracc_out, 15));
			pracc_out += 4;
		}
	}
	if (len & 3) { /* not a multiple of 4 bytes */
		/* $3 = [$1 + DATA] */
		pracc_add(ctx, 0, MIPS32_LW(0, 3, ATH79_REG_DATA, 1));
		/* [OUTi] = $3 */
		pracc_add(ctx, MIPS32_PRACC_PARAM_OUT + pracc_out,
			  MIPS32_SW(0, 3, PRACC_OUT_OFFSET + pracc_out, 15));
		pracc_out += 4;
	}

	if (ath79_info->spi.post_deselect && !partial_xfer) {
		const uint32_t post_deselect[] = {
			/* $2 = SPI_CS_DIS */
			MIPS32_LUI(0, 2, UPPER16(cs_high)),
			MIPS32_ORI(0, 2, 2, LOWER16(cs_high)),
			/* [$1 + WRITE] = $2 */
			MIPS32_SW(0, 2, ATH79_REG_WRITE, 1),

			/* [$1 + FS] = 0  (disable flash io register access) */
			MIPS32_XORI(0, 2, 2, 0),
			MIPS32_SW(0, 2, ATH79_REG_FS, 1),
		};
		ath79_pracc_addn(ctx, post_deselect, ARRAY_SIZE(post_deselect));
	}

	/* common pracc epilogue */
	/* jump to start */
	pracc_add(ctx, 0, MIPS32_B(0, NEG16(ctx->code_count + 1)));
	/* restore $15 from DeSave */
	pracc_add(ctx, 0, MIPS32_MFC0(0, 15, 31, 0));

	return pracc_out / 4;
}

static int ath79_spi_bitbang_chunk(struct flash_bank *bank,
				   uint8_t *data, int len, int *transferred)
{
	struct target *target = bank->target;
	struct ath79_flash_bank *ath79_info = bank->driver_priv;
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int pracc_words;

	/*
	 * These constants must match the worst case in the above code
	 * generator function ath79_spi_bitbang_codegen.
	 */
	const int pracc_pre_post = 26;
	const int pracc_loop_byte = 8 * 2 + 2;

	struct pracc_queue_info ctx = {
		.ejtag_info = ejtag_info
	};
	int max_len = (PRACC_MAX_INSTRUCTIONS - pracc_pre_post) / pracc_loop_byte;
	int to_xfer = len > max_len ? max_len : len;
	int partial_xfer = len != to_xfer;
	int padded_len = (to_xfer + 3) & ~3;
	uint32_t *out = malloc(padded_len);

	if (!out) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	*transferred = 0;
	pracc_queue_init(&ctx);

	LOG_DEBUG("ath79_spi_bitbang_bytes(%p, %08" PRIx32 ", %p, %d)",
		  target, ath79_info->io_base, data, len);

	LOG_DEBUG("max code %d => max len %d. to_xfer %d",
		  PRACC_MAX_INSTRUCTIONS, max_len, to_xfer);

	pracc_words = ath79_spi_bitbang_codegen(
		ath79_info, &ctx, data, to_xfer, partial_xfer);

	LOG_DEBUG("Assembled %d instructions, %d stores",
		  ctx.code_count, ctx.store_count);

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, out, 1);
	if (ctx.retval != ERROR_OK)
		goto exit;

	if (to_xfer & 3) { /* Not a multiple of 4 bytes. */
		/*
		 * Need to realign last word since we didn't shift the
		 * full 32 bits.
		 */
		int missed_bytes = 4 - (to_xfer & 3);

		out[pracc_words - 1] <<= BITS_PER_BYTE * missed_bytes;
	}

	/*
	 * pracc reads return uint32_t in host endianness, convert to
	 * target endianness.
	 * Since we know the ATH79 target is big endian and the SPI
	 * shift register has the bytes in highest to lowest bit order,
	 * this will ensure correct memory byte order regardless of host
	 * endianness.
	 */
	target_buffer_set_u32_array(target, (uint8_t *)out, pracc_words, out);

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG)) {
		for (int i = 0; i < to_xfer; i++) {
			LOG_DEBUG("bitbang %02x => %02x",
				  data[i], ((uint8_t *)out)[i]);
		}
	}
	memcpy(data, out, to_xfer);
	*transferred = to_xfer;

exit:
	pracc_queue_free(&ctx);
	free(out);
	return ctx.retval;
}

static void ath79_spi_bitbang_prepare(struct flash_bank *bank)
{
	struct ath79_flash_bank *ath79_info = bank->driver_priv;

	ath79_info->spi.pre_deselect = 1;
}

static int ath79_spi_bitbang_bytes(struct flash_bank *bank,
				   uint8_t *data, int len, uint32_t flags)
{
	struct ath79_flash_bank *ath79_info = bank->driver_priv;
	int retval;
	int transferred;

	ath79_info->spi.post_deselect = !!(flags & ATH79_XFER_FINAL);

	do {
		transferred = 0;
		retval = ath79_spi_bitbang_chunk(
			bank, data, len, &transferred);
		if (retval != ERROR_OK)
			return retval;

		data += transferred;
		len -= transferred;
	} while (len > 0);

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(ath79_flash_bank_command)
{
	struct ath79_flash_bank *ath79_info;
	int chipselect = 0;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6 || CMD_ARGC > 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 7) {
		if (strcmp(CMD_ARGV[6], "cs0") == 0)
			chipselect = 0;  /* default */
		else if (strcmp(CMD_ARGV[6], "cs1") == 0)
			chipselect = 1;
		else if (strcmp(CMD_ARGV[6], "cs2") == 0)
			chipselect = 2;
		else {
			LOG_ERROR("Unknown arg: %s", CMD_ARGV[6]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	ath79_info = calloc(1, sizeof(struct ath79_flash_bank));
	if (!ath79_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	ath79_info->chipselect = chipselect;
	bank->driver_priv = ath79_info;

	return ERROR_OK;
}

/* Read the status register of the external SPI flash chip. */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	uint8_t spi_bytes[] = {SPIFLASH_READ_STATUS, 0};
	int retval;

	/* Send SPI command "read STATUS" */
	ath79_spi_bitbang_prepare(bank);
	retval = ath79_spi_bitbang_bytes(
		bank, spi_bytes, sizeof(spi_bytes),
		ATH79_XFER_FINAL);

	*status = spi_bytes[1];

	return retval;
}

/* check for WIP (write in progress) bit in status register */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval;
	long long endtime;

	endtime = timeval_ms() + timeout;
	do {
		/* read flash status register */
		retval = read_status_reg(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & SPIFLASH_BSY_BIT) == 0)
			return ERROR_OK;
		alive_sleep(1);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

/* Send "write enable" command to SPI flash chip. */
static int ath79_write_enable(struct flash_bank *bank)
{
	uint32_t status;
	int retval;

	uint8_t spi_bytes[] = {SPIFLASH_WRITE_ENABLE};

	/* Send SPI command "write enable" */
	ath79_spi_bitbang_prepare(bank);
	retval = ath79_spi_bitbang_bytes(
		bank, spi_bytes, sizeof(spi_bytes),
		ATH79_XFER_FINAL);
	if (retval != ERROR_OK)
		return retval;

	/* read flash status register */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		return retval;

	/* Check write enabled */
	if ((status & SPIFLASH_WE_BIT) == 0) {
		LOG_ERROR("Cannot enable write to flash. Status=0x%08" PRIx32,
			  status);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int erase_command(struct flash_bank *bank, int sector)
{
	struct ath79_flash_bank *ath79_info = bank->driver_priv;
	uint32_t offset = bank->sectors[sector].offset;

	uint8_t spi_bytes[] = {
		ath79_info->dev->erase_cmd,
		offset >> 16,
		offset >> 8,
		offset
	};

	/* bitbang command */
	ath79_spi_bitbang_prepare(bank);
	return ath79_spi_bitbang_bytes(
		bank, spi_bytes, sizeof(spi_bytes),
		ATH79_XFER_FINAL);
}

static int ath79_erase_sector(struct flash_bank *bank, int sector)
{
	int retval = ath79_write_enable(bank);

	if (retval != ERROR_OK)
		return retval;

	/* send SPI command "block erase" */
	retval = erase_command(bank, sector);
	if (retval != ERROR_OK)
		return retval;

	/* poll WIP for end of self timed Sector Erase cycle */
	return wait_till_ready(bank, ATH79_MAX_TIMEOUT);
}

static int ath79_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct ath79_flash_bank *ath79_info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: from sector %u to sector %u", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!ath79_info->probed) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (ath79_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (unsigned sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = ath79_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	return retval;
}

static int ath79_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int ath79_write_page(struct flash_bank *bank, const uint8_t *buffer,
			    uint32_t address, uint32_t len)
{
	struct ath79_flash_bank *ath79_info = bank->driver_priv;
	uint8_t spi_bytes[] = {
		SPIFLASH_PAGE_PROGRAM,
		address >> 16,
		address >> 8,
		address,
	};
	int retval;
	uint32_t i, pagesize;

	/* if no write pagesize, use reasonable default */
	pagesize = ath79_info->dev->pagesize ?
		ath79_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	if (address & 0xff) {
		LOG_ERROR("ath79_write_page: unaligned write address: %08" PRIx32,
			  address);
		return ERROR_FAIL;
	}
	if (!ath79_info->spi.page_buf) {
		LOG_ERROR("ath79_write_page: page buffer not initialized");
		return ERROR_FAIL;
	}
	if (len > ath79_info->dev->pagesize) {
		LOG_ERROR("ath79_write_page: len bigger than page size %" PRIu32 ": %" PRIu32,
			pagesize, len);
		return ERROR_FAIL;
	}

	for (i = 0; i < len; i++) {
		if (buffer[i] != 0xff)
			break;
	}
	if (i == len)  /* all 0xff, no need to program. */
		return ERROR_OK;

	LOG_INFO("writing %" PRIu32 " bytes to flash page @0x%08" PRIx32, len, address);

	memcpy(ath79_info->spi.page_buf, buffer, len);

	/* unlock writes */
	retval = ath79_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	/* bitbang command */
	ath79_spi_bitbang_prepare(bank);
	retval = ath79_spi_bitbang_bytes(
		bank, spi_bytes, sizeof(spi_bytes),
		ATH79_XFER_PARTIAL);
	if (retval != ERROR_OK)
		return retval;

	/* write data */
	return ath79_spi_bitbang_bytes(
		bank, ath79_info->spi.page_buf, len,
		ATH79_XFER_FINAL);
}

static int ath79_write_buffer(struct flash_bank *bank, const uint8_t *buffer,
			      uint32_t address, uint32_t len)
{
	struct ath79_flash_bank *ath79_info = bank->driver_priv;
	uint32_t page_size;
	int retval;

	LOG_DEBUG("%s: address=0x%08" PRIx32 " len=0x%08" PRIx32,
		  __func__, address, len);

	/* if no valid page_size, use reasonable default */
	page_size = ath79_info->dev->pagesize ?
		ath79_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	while (len > 0) {
		int page_len = len > page_size ? page_size : len;

		retval = ath79_write_page(
			bank, buffer, address, page_len);
		if (retval != ERROR_OK)
			return retval;

		buffer += page_size;
		address += page_size;
		len -= page_len;
	}

	return ERROR_OK;
}

static int ath79_write(struct flash_bank *bank, const uint8_t *buffer,
		       uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		  __func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Write pasts end of flash. Extra data discarded.");
		count = bank->size - offset;
	}

	/* Check sector protection */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		struct flash_sector *bs = &bank->sectors[sector];

		if ((offset < (bs->offset + bs->size)) &&
		    ((offset + count - 1) >= bs->offset) &&
		    bs->is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	return ath79_write_buffer(bank, buffer, offset, count);
}

static int ath79_read_buffer(struct flash_bank *bank, uint8_t *buffer,
			     uint32_t address, uint32_t len)
{
	uint8_t spi_bytes[] = {
		SPIFLASH_READ,
		address >> 16,
		address >> 8,
		address,
	};
	int retval;

	LOG_DEBUG("%s: address=0x%08" PRIx32 " len=0x%08" PRIx32,
		  __func__, address, len);

	if (address & 0xff) {
		LOG_ERROR("ath79_read_buffer: unaligned read address: %08" PRIx32,
			  address);
		return ERROR_FAIL;
	}

	LOG_INFO("reading %" PRIu32 " bytes from flash @0x%08" PRIx32, len, address);

	/* bitbang command */
	ath79_spi_bitbang_prepare(bank);
	retval = ath79_spi_bitbang_bytes(
		bank, spi_bytes, sizeof(spi_bytes), ATH79_XFER_PARTIAL);
	if (retval != ERROR_OK)
		return retval;

	/* read data */
	return ath79_spi_bitbang_bytes(
		bank, buffer, len, ATH79_XFER_FINAL);
}

static int ath79_read(struct flash_bank *bank, uint8_t *buffer,
		      uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		  __func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Reads past end of flash. Extra data discarded.");
		count = bank->size - offset;
	}

	return ath79_read_buffer(bank, buffer, offset, count);
}

/* Return ID of flash device */
static int read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	int retval;
	uint8_t spi_bytes[] = {SPIFLASH_READ_ID, 0, 0, 0};

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Send SPI command "read ID" */
	ath79_spi_bitbang_prepare(bank);
	retval = ath79_spi_bitbang_bytes(
		bank, spi_bytes, sizeof(spi_bytes), ATH79_XFER_FINAL);
	if (retval != ERROR_OK)
		return retval;

	*id = (spi_bytes[1] << 0)
		| (spi_bytes[2] << 8)
		| (spi_bytes[3] << 16);

	if (*id == 0xffffff) {
		LOG_ERROR("No SPI flash found");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int ath79_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct ath79_flash_bank *ath79_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	uint32_t pagesize, sectorsize;
	const struct ath79_target *target_device;
	int retval;

	if (ath79_info->probed) {
		free(bank->sectors);
		free(ath79_info->spi.page_buf);
	}
	ath79_info->probed = false;

	for (target_device = target_devices; target_device->name;
		++target_device)
		if (target_device->tap_idcode == target->tap->idcode)
			break;
	if (!target_device->name) {
		LOG_ERROR("Device ID 0x%" PRIx32 " is not known",
			  target->tap->idcode);
		return ERROR_FAIL;
	}

	ath79_info->io_base = target_device->io_base;

	LOG_DEBUG("Found device %s at address " TARGET_ADDR_FMT,
		  target_device->name, bank->base);

	retval = read_flash_id(bank, &id);
	if (retval != ERROR_OK)
		return retval;

	ath79_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name; p++)
		if (p->device_id == id) {
			ath79_info->dev = p;
			break;
		}

	if (!ath79_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		 ath79_info->dev->name, ath79_info->dev->device_id);

	/* Set correct size value */
	bank->size = ath79_info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = ath79_info->dev->sectorsize ?
		ath79_info->dev->sectorsize : ath79_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = ath79_info->dev->size_in_bytes / sectorsize;
	sectors = calloc(1, sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	/* if no write pagesize, use reasonable default */
	pagesize = ath79_info->dev->pagesize ? ath79_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	ath79_info->spi.page_buf = malloc(pagesize);
	if (!ath79_info->spi.page_buf) {
		LOG_ERROR("not enough memory");
		free(sectors);
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = 0;
		sectors[sector].is_protected = 1;
	}

	bank->sectors = sectors;
	ath79_info->probed = true;
	return ERROR_OK;
}

static int ath79_auto_probe(struct flash_bank *bank)
{
	struct ath79_flash_bank *ath79_info = bank->driver_priv;

	if (ath79_info->probed)
		return ERROR_OK;
	return ath79_probe(bank);
}

static int ath79_flash_blank_check(struct flash_bank *bank)
{
	/* Not implemented */
	return ERROR_OK;
}

static int ath79_protect_check(struct flash_bank *bank)
{
	/* Not implemented */
	return ERROR_OK;
}

static int get_ath79_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct ath79_flash_bank *ath79_info = bank->driver_priv;

	if (!ath79_info->probed) {
		snprintf(buf, buf_size,
			 "\nATH79 flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nATH79 flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		ath79_info->dev->name, ath79_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver ath79_flash = {
	.name = "ath79",
	.flash_bank_command = ath79_flash_bank_command,
	.erase = ath79_erase,
	.protect = ath79_protect,
	.write = ath79_write,
	.read = ath79_read,
	.probe = ath79_probe,
	.auto_probe = ath79_auto_probe,
	.erase_check = ath79_flash_blank_check,
	.protect_check = ath79_protect_check,
	.info = get_ath79_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
