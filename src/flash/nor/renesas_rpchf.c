/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Renesas RCar Gen3 RPC Hyperflash driver
 * Based on U-Boot RPC Hyperflash driver
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 * Copyright (C) 2016 Cogent Embedded, Inc.
 * Copyright (C) 2017-2019 Marek Vasut <marek.vasut@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "cfi.h"
#include "non_cfi.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>

#define RPC_CMNCR		0x0000	/* R/W */
#define RPC_CMNCR_MD		BIT(31)
#define RPC_CMNCR_MOIIO0(val)	(((val) & 0x3) << 16)
#define RPC_CMNCR_MOIIO1(val)	(((val) & 0x3) << 18)
#define RPC_CMNCR_MOIIO2(val)	(((val) & 0x3) << 20)
#define RPC_CMNCR_MOIIO3(val)	(((val) & 0x3) << 22)
#define RPC_CMNCR_MOIIO_HIZ	(RPC_CMNCR_MOIIO0(3) | RPC_CMNCR_MOIIO1(3) | \
				 RPC_CMNCR_MOIIO2(3) | RPC_CMNCR_MOIIO3(3))
#define RPC_CMNCR_IO0FV(val)	(((val) & 0x3) << 8)
#define RPC_CMNCR_IO2FV(val)	(((val) & 0x3) << 12)
#define RPC_CMNCR_IO3FV(val)	(((val) & 0x3) << 14)
#define RPC_CMNCR_IOFV_HIZ	(RPC_CMNCR_IO0FV(3) | RPC_CMNCR_IO2FV(3) | \
				 RPC_CMNCR_IO3FV(3))
#define RPC_CMNCR_BSZ(val)	(((val) & 0x3) << 0)

#define RPC_SSLDR		0x0004	/* R/W */
#define RPC_SSLDR_SPNDL(d)	(((d) & 0x7) << 16)
#define RPC_SSLDR_SLNDL(d)	(((d) & 0x7) << 8)
#define RPC_SSLDR_SCKDL(d)	(((d) & 0x7) << 0)

#define RPC_DRCR		0x000C	/* R/W */
#define RPC_DRCR_SSLN		BIT(24)
#define RPC_DRCR_RBURST(v)	(((v) & 0x1F) << 16)
#define RPC_DRCR_RCF		BIT(9)
#define RPC_DRCR_RBE		BIT(8)
#define RPC_DRCR_SSLE		BIT(0)

#define RPC_DRCMR		0x0010	/* R/W */
#define RPC_DRCMR_CMD(c)	(((c) & 0xFF) << 16)
#define RPC_DRCMR_OCMD(c)	(((c) & 0xFF) << 0)

#define RPC_DREAR		0x0014	/* R/W */
#define RPC_DREAR_EAV(v)	(((v) & 0xFF) << 16)
#define RPC_DREAR_EAC(v)	(((v) & 0x7) << 0)

#define RPC_DROPR		0x0018	/* R/W */
#define RPC_DROPR_OPD3(o)	(((o) & 0xFF) << 24)
#define RPC_DROPR_OPD2(o)	(((o) & 0xFF) << 16)
#define RPC_DROPR_OPD1(o)	(((o) & 0xFF) << 8)
#define RPC_DROPR_OPD0(o)	(((o) & 0xFF) << 0)

#define RPC_DRENR		0x001C	/* R/W */
#define RPC_DRENR_CDB(o)	(uint32_t)((((o) & 0x3) << 30))
#define RPC_DRENR_OCDB(o)	(((o) & 0x3) << 28)
#define RPC_DRENR_ADB(o)	(((o) & 0x3) << 24)
#define RPC_DRENR_OPDB(o)	(((o) & 0x3) << 20)
#define RPC_DRENR_SPIDB(o)	(((o) & 0x3) << 16)
#define RPC_DRENR_DME		BIT(15)
#define RPC_DRENR_CDE		BIT(14)
#define RPC_DRENR_OCDE		BIT(12)
#define RPC_DRENR_ADE(v)	(((v) & 0xF) << 8)
#define RPC_DRENR_OPDE(v)	(((v) & 0xF) << 4)

#define RPC_SMCR		0x0020	/* R/W */
#define RPC_SMCR_SSLKP		BIT(8)
#define RPC_SMCR_SPIRE		BIT(2)
#define RPC_SMCR_SPIWE		BIT(1)
#define RPC_SMCR_SPIE		BIT(0)

#define RPC_SMCMR		0x0024	/* R/W */
#define RPC_SMCMR_CMD(c)	(((c) & 0xFF) << 16)
#define RPC_SMCMR_OCMD(c)	(((c) & 0xFF) << 0)

#define RPC_SMADR		0x0028	/* R/W */
#define RPC_SMOPR		0x002C	/* R/W */
#define RPC_SMOPR_OPD0(o)	(((o) & 0xFF) << 0)
#define RPC_SMOPR_OPD1(o)	(((o) & 0xFF) << 8)
#define RPC_SMOPR_OPD2(o)	(((o) & 0xFF) << 16)
#define RPC_SMOPR_OPD3(o)	(((o) & 0xFF) << 24)

#define RPC_SMENR		0x0030	/* R/W */
#define RPC_SMENR_CDB(o)	(((o) & 0x3) << 30)
#define RPC_SMENR_OCDB(o)	(((o) & 0x3) << 28)
#define RPC_SMENR_ADB(o)	(((o) & 0x3) << 24)
#define RPC_SMENR_OPDB(o)	(((o) & 0x3) << 20)
#define RPC_SMENR_SPIDB(o)	(((o) & 0x3) << 16)
#define RPC_SMENR_DME		BIT(15)
#define RPC_SMENR_CDE		BIT(14)
#define RPC_SMENR_OCDE		BIT(12)
#define RPC_SMENR_ADE(v)	(((v) & 0xF) << 8)
#define RPC_SMENR_OPDE(v)	(((v) & 0xF) << 4)
#define RPC_SMENR_SPIDE(v)	(((v) & 0xF) << 0)

#define RPC_SMRDR0		0x0038	/* R */
#define RPC_SMRDR1		0x003C	/* R */
#define RPC_SMWDR0		0x0040	/* R/W */
#define RPC_SMWDR1		0x0044	/* R/W */
#define RPC_CMNSR		0x0048	/* R */
#define RPC_CMNSR_SSLF		BIT(1)
#define	RPC_CMNSR_TEND		BIT(0)

#define RPC_DRDMCR		0x0058	/* R/W */
#define RPC_DRDMCR_DMCYC(v)	(((v) & 0xF) << 0)

#define RPC_DRDRENR		0x005C	/* R/W */
#define RPC_DRDRENR_HYPE	(0x5 << 12)
#define RPC_DRDRENR_ADDRE	BIT(8)
#define RPC_DRDRENR_OPDRE	BIT(4)
#define RPC_DRDRENR_DRDRE	BIT(0)

#define RPC_SMDMCR		0x0060	/* R/W */
#define RPC_SMDMCR_DMCYC(v)	(((v) & 0xF) << 0)

#define RPC_SMDRENR		0x0064	/* R/W */
#define RPC_SMDRENR_HYPE	(0x5 << 12)
#define RPC_SMDRENR_ADDRE	BIT(8)
#define RPC_SMDRENR_OPDRE	BIT(4)
#define RPC_SMDRENR_SPIDRE	BIT(0)

#define RPC_PHYCNT		0x007C	/* R/W */
#define RPC_PHYCNT_CAL		BIT(31)
#define PRC_PHYCNT_OCTA_AA	BIT(22)
#define PRC_PHYCNT_OCTA_SA	BIT(23)
#define PRC_PHYCNT_EXDS		BIT(21)
#define RPC_PHYCNT_OCT		BIT(20)
#define RPC_PHYCNT_WBUF2	BIT(4)
#define RPC_PHYCNT_WBUF		BIT(2)
#define RPC_PHYCNT_MEM(v)	(((v) & 0x3) << 0)

#define RPC_PHYINT		0x0088	/* R/W */
#define RPC_PHYINT_RSTEN	BIT(18)
#define RPC_PHYINT_WPEN		BIT(17)
#define RPC_PHYINT_INTEN	BIT(16)
#define RPC_PHYINT_RST		BIT(2)
#define RPC_PHYINT_WP		BIT(1)
#define RPC_PHYINT_INT		BIT(0)

#define RPC_WBUF		0x8000	/* R/W size=4/8/16/32/64Bytes */
#define RPC_WBUF_SIZE		0x100

static uint32_t rpc_base = 0xee200000;
static uint32_t mem_base = 0x08000000;

enum rpc_hf_size {
	RPC_HF_SIZE_16BIT = RPC_SMENR_SPIDE(0x8),
	RPC_HF_SIZE_32BIT = RPC_SMENR_SPIDE(0xC),
	RPC_HF_SIZE_64BIT = RPC_SMENR_SPIDE(0xF),
};

static int rpc_hf_wait_tend(struct target *target)
{
	uint32_t reg = rpc_base + RPC_CMNSR;
	uint32_t val;
	unsigned long timeout = 1000;
	long long endtime;
	int ret;

	endtime = timeval_ms() + timeout;
	do {
		ret = target_read_u32(target, reg, &val);
		if (ret != ERROR_OK)
			return ERROR_FAIL;

		if (val & RPC_CMNSR_TEND)
			return ERROR_OK;

		alive_sleep(1);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_TIMEOUT_REACHED;
}

static int clrsetbits_u32(struct target *target, uint32_t reg,
			   uint32_t clr, uint32_t set)
{
	uint32_t val;
	int ret;

	ret = target_read_u32(target, reg, &val);
	if (ret != ERROR_OK)
		return ret;

	val &= ~clr;
	val |= set;

	return target_write_u32(target, reg, val);
}

static int rpc_hf_mode(struct target *target, bool manual)
{
	uint32_t val;
	int ret;

	ret = rpc_hf_wait_tend(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Mode TEND timeout");
		return ret;
	}

	ret = clrsetbits_u32(target, rpc_base + RPC_PHYCNT,
				RPC_PHYCNT_WBUF | RPC_PHYCNT_WBUF2 |
				RPC_PHYCNT_CAL | RPC_PHYCNT_MEM(3),
				RPC_PHYCNT_CAL | RPC_PHYCNT_MEM(3));
	if (ret != ERROR_OK)
		return ret;

	ret = clrsetbits_u32(target, rpc_base + RPC_CMNCR,
				RPC_CMNCR_MD | RPC_CMNCR_BSZ(3),
				RPC_CMNCR_MOIIO_HIZ | RPC_CMNCR_IOFV_HIZ |
				(manual ? RPC_CMNCR_MD : 0) | RPC_CMNCR_BSZ(1));
	if (ret != ERROR_OK)
		return ret;

	if (manual)
		return ERROR_OK;

	ret = target_write_u32(target, rpc_base + RPC_DRCR,
			       RPC_DRCR_RBURST(0x1F) | RPC_DRCR_RCF |
			       RPC_DRCR_RBE);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rpc_base + RPC_DRCMR,
			       RPC_DRCMR_CMD(0xA0));
	if (ret != ERROR_OK)
		return ret;
	ret = target_write_u32(target, rpc_base + RPC_DRENR,
			       RPC_DRENR_CDB(2) | RPC_DRENR_OCDB(2) |
			       RPC_DRENR_ADB(2) | RPC_DRENR_SPIDB(2) |
			       RPC_DRENR_CDE | RPC_DRENR_OCDE |
			       RPC_DRENR_ADE(4));
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rpc_base + RPC_DRDMCR,
			       RPC_DRDMCR_DMCYC(0xE));
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rpc_base + RPC_DRDRENR,
			       RPC_DRDRENR_HYPE | RPC_DRDRENR_ADDRE |
			       RPC_DRDRENR_DRDRE);
	if (ret != ERROR_OK)
		return ret;

	/* Dummy read */
	return target_read_u32(target, rpc_base + RPC_DRCR, &val);
}

static int rpc_hf_xfer(struct target *target, target_addr_t addr,
		       uint32_t wdata, uint32_t *rdata, enum rpc_hf_size size,
		       bool write, const uint8_t *wbuf, unsigned int wbuf_size)
{
	int ret;
	uint32_t val;

	if (wbuf_size != 0) {
		ret = rpc_hf_wait_tend(target);
		if (ret != ERROR_OK) {
			LOG_ERROR("Xfer TEND timeout");
			return ret;
		}

		/* Write calibration magic */
		ret = target_write_u32(target, rpc_base + RPC_DRCR, 0x01FF0301);
		if (ret != ERROR_OK)
			return ret;

		ret = target_write_u32(target, rpc_base + RPC_PHYCNT, 0x80030277);
		if (ret != ERROR_OK)
			return ret;

		ret = target_write_memory(target, rpc_base | RPC_WBUF, 4,
					  wbuf_size / 4, wbuf);
		if (ret != ERROR_OK)
			return ret;

		ret = clrsetbits_u32(target, rpc_base + RPC_CMNCR,
					RPC_CMNCR_MD | RPC_CMNCR_BSZ(3),
					RPC_CMNCR_MOIIO_HIZ | RPC_CMNCR_IOFV_HIZ |
					RPC_CMNCR_MD | RPC_CMNCR_BSZ(1));
		if (ret != ERROR_OK)
			return ret;
	} else {
		ret = rpc_hf_mode(target, 1);
		if (ret != ERROR_OK)
			return ret;
	}

	/* Submit HF address, SMCMR CMD[7] ~= CA Bit# 47 (R/nW) */
	ret = target_write_u32(target, rpc_base + RPC_SMCMR,
			       write ? 0 : RPC_SMCMR_CMD(0x80));
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rpc_base + RPC_SMADR,
			       addr >> 1);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rpc_base + RPC_SMOPR, 0x0);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, rpc_base + RPC_SMDRENR,
			       RPC_SMDRENR_HYPE | RPC_SMDRENR_ADDRE |
			       RPC_SMDRENR_SPIDRE);
	if (ret != ERROR_OK)
		return ret;

	val = RPC_SMENR_CDB(2) | RPC_SMENR_OCDB(2) |
	      RPC_SMENR_ADB(2) | RPC_SMENR_SPIDB(2) |
	      (wbuf_size ? RPC_SMENR_OPDB(2) : 0) |
	      RPC_SMENR_CDE | RPC_SMENR_OCDE | RPC_SMENR_ADE(4) | size;

	if (write) {
		ret = target_write_u32(target, rpc_base + RPC_SMENR, val);
		if (ret != ERROR_OK)
			return ret;

		if (wbuf_size == 0) {
			buf_bswap32((uint8_t *)&wdata, (uint8_t *)&wdata, 4);
			ret = target_write_u32(target, rpc_base + RPC_SMWDR0,
					       wdata);
			if (ret != ERROR_OK)
				return ret;
		}

		ret = target_write_u32(target, rpc_base + RPC_SMCR,
				       RPC_SMCR_SPIWE | RPC_SMCR_SPIE);
		if (ret != ERROR_OK)
			return ret;
	} else {
		val |= RPC_SMENR_DME;

		ret = target_write_u32(target, rpc_base + RPC_SMDMCR,
				       RPC_SMDMCR_DMCYC(0xE));
		if (ret != ERROR_OK)
			return ret;

		ret = target_write_u32(target, rpc_base + RPC_SMENR, val);
		if (ret != ERROR_OK)
			return ret;

		ret = target_write_u32(target, rpc_base + RPC_SMCR,
				       RPC_SMCR_SPIRE | RPC_SMCR_SPIE);
		if (ret != ERROR_OK)
			return ret;

		ret = rpc_hf_wait_tend(target);
		if (ret != ERROR_OK)
			return ret;

		uint32_t val32;
		ret = target_read_u32(target, rpc_base + RPC_SMRDR0, &val32);
		if (ret != ERROR_OK)
			return ret;
		buf_bswap32((uint8_t *)&val32, (uint8_t *)&val32, 4);
		*rdata = val32;
	}

	ret = rpc_hf_mode(target, 0);
	if (ret != ERROR_OK)
		LOG_ERROR("Xfer done TEND timeout");
	return ret;
}

static int rpchf_target_write_memory(struct flash_bank *bank, target_addr_t addr,
				     uint32_t count, const uint8_t *buffer)
{
	struct target *target = bank->target;
	uint32_t wdata;

	if (count != 2)
		return ERROR_FAIL;

	wdata = buffer[0] | (buffer[1] << 8);

	return rpc_hf_xfer(target, addr, wdata, NULL, RPC_HF_SIZE_16BIT,
			   true, NULL, 0);
}

static int rpchf_target_read_memory(struct flash_bank *bank, target_addr_t addr,
				    uint32_t count, uint8_t *buffer)
{
	struct target *target = bank->target;
	uint32_t i, rdata;
	int ret;

	for (i = 0; i < count; i++) {
		ret = rpc_hf_xfer(target, addr + (2 * i), 0, &rdata,
					RPC_HF_SIZE_16BIT, false, NULL, 0);
		if (ret != ERROR_OK)
			return ret;
		buffer[(2 * i) + 0] = rdata & 0xff;
		buffer[(2 * i) + 1] = (rdata >> 8) & 0xff;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(rpchf_flash_bank_command)
{
	struct cfi_flash_bank *cfi_info;
	int ret;

	ret = cfi_flash_bank_cmd(bank, CMD_ARGC, CMD_ARGV);
	if (ret != ERROR_OK)
		return ret;

	cfi_info = bank->driver_priv;
	cfi_info->read_mem = rpchf_target_read_memory;
	cfi_info->write_mem = rpchf_target_write_memory;

	return ERROR_OK;
}

static int rpchf_spansion_write_words(struct flash_bank *bank, const uint8_t *word,
	uint32_t wordcount, uint32_t address)
{
	int retval;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct cfi_spansion_pri_ext *pri_ext = cfi_info->pri_ext;

	/* Calculate buffer size and boundary mask
	 * buffersize is (buffer size per chip) * (number of chips)
	 * bufferwsize is buffersize in words */
	uint32_t buffersize = RPC_WBUF_SIZE;
	uint32_t buffermask = buffersize - 1;
	uint32_t bufferwsize = buffersize / 2;

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

	retval = cfi_send_command(bank, 0xa0, cfi_flash_address(bank, 0, pri_ext->_unlock1));
	if (retval != ERROR_OK)
		return retval;

	retval = rpc_hf_xfer(bank->target, address, 0, NULL, RPC_HF_SIZE_64BIT, true, word, wordcount * 2);
	if (retval != ERROR_OK)
		return retval;

	if (cfi_spansion_wait_status_busy(bank, cfi_info->word_write_timeout) != ERROR_OK) {
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

static int rpchf_write_words(struct flash_bank *bank, const uint8_t *word,
	uint32_t wordcount, uint32_t address)
{
	return rpchf_spansion_write_words(bank, word, wordcount, address);
}

static int rpchf_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	uint32_t address = bank->base + offset;	/* address of first byte to be programmed */
	uint32_t write_p;
	int align;	/* number of unaligned bytes */
	uint8_t current_word[CFI_MAX_BUS_WIDTH * 4];	/* word (bus_width size) currently being
							 *programmed */
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
		for (unsigned int i = align; (i < bank->bus_width) && (count > 0); i++, count--) {
			if (cfi_info->data_swap)
				/* data bytes are swapped (reverse endianness) */
				current_word[bank->bus_width - i] = *buffer++;
			else
				current_word[i] = *buffer++;
		}

		retval = cfi_write_word(bank, current_word, write_p);
		if (retval != ERROR_OK)
			return retval;
		write_p += bank->bus_width;
	}

	/* Calculate buffer size and boundary mask
	 * buffersize is (buffer size per chip) * (number of chips)
	 * bufferwsize is buffersize in words */
	uint32_t buffersize = RPC_WBUF_SIZE;
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
			retval = rpchf_write_words(bank, buffer, bufferwsize, write_p);
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

static int rpchf_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	struct target *target = bank->target;

	LOG_DEBUG("reading buffer of %" PRIu32 " byte at 0x%8.8" PRIx32,
		  count, offset);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	if (cfi_info->qry[0] != 'Q')
		return ERROR_FLASH_BANK_NOT_PROBED;

	return target_read_memory(target, offset | mem_base,
				  4, count / 4, buffer);
}

const struct flash_driver renesas_rpchf_flash = {
	.name = "rpchf",
	.flash_bank_command = rpchf_flash_bank_command,
	.erase = cfi_erase,
	.protect = cfi_protect,
	.write = rpchf_write,
	.read = rpchf_read,
	.probe = cfi_probe,
	.auto_probe = cfi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = cfi_protect_check,
	.info = cfi_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
