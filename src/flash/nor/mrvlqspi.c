/***************************************************************************
 *   Copyright (C) 2014 by Mahavir Jain <mjain@marvell.com>                *
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

 /*
  * This is QSPI flash controller driver for Marvell's Wireless
  * Microcontroller platform.
  *
  * For more information please refer,
  * https://origin-www.marvell.com/microcontrollers/wi-fi-microcontroller-platform/
  */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#define QSPI_R_EN (0x0)
#define QSPI_W_EN (0x1)
#define QSPI_SS_DISABLE (0x0)
#define QSPI_SS_ENABLE (0x1)
#define WRITE_DISBALE (0x0)
#define WRITE_ENABLE (0x1)

#define QSPI_TIMEOUT (1000)
#define FIFO_FLUSH_TIMEOUT (1000)
#define BLOCK_ERASE_TIMEOUT (1000)
#define CHIP_ERASE_TIMEOUT (10000)

#define SS_EN (1 << 0)
#define XFER_RDY (1 << 1)
#define RFIFO_EMPTY (1 << 4)
#define WFIFO_EMPTY (1 << 6)
#define WFIFO_FULL (1 << 7)
#define FIFO_FLUSH (1 << 9)
#define RW_EN (1 << 13)
#define XFER_STOP (1 << 14)
#define XFER_START (1 << 15)
#define CONF_MASK (0x7)
#define CONF_OFFSET (10)

#define INS_WRITE_ENABLE 0x06
#define INS_WRITE_DISABLE 0x04
#define INS_READ_STATUS 0x05
#define INS_PAGE_PROGRAM 0x02

#define CNTL 0x0 /* QSPI_BASE + 0x0 */
#define CONF 0x4
#define DOUT 0x8
#define DIN 0xc
#define INSTR 0x10
#define ADDR 0x14
#define RDMODE 0x18
#define HDRCNT 0x1c
#define DINCNT 0x20

struct mrvlqspi_flash_bank {
	bool probed;
	uint32_t reg_base;
	uint32_t bank_num;
	const struct flash_device *dev;
};

static inline uint32_t mrvlqspi_get_reg(struct flash_bank *bank, uint32_t reg)
{
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;
	return reg + mrvlqspi_info->reg_base;
}

static inline int mrvlqspi_set_din_cnt(struct flash_bank *bank, uint32_t count)
{
	struct target *target = bank->target;

	return target_write_u32(target, mrvlqspi_get_reg(bank, DINCNT), count);
}

static inline int mrvlqspi_set_addr(struct flash_bank *bank, uint32_t addr)
{
	struct target *target = bank->target;

	return target_write_u32(target, mrvlqspi_get_reg(bank, ADDR), addr);
}

static inline int mrvlqspi_set_instr(struct flash_bank *bank, uint32_t instr)
{
	struct target *target = bank->target;

	return target_write_u32(target, mrvlqspi_get_reg(bank, INSTR), instr);
}

static inline int mrvlqspi_set_hdr_cnt(struct flash_bank *bank, uint32_t hdr_cnt)
{
	struct target *target = bank->target;

	return target_write_u32(target, mrvlqspi_get_reg(bank, HDRCNT), hdr_cnt);
}

static int mrvlqspi_set_conf(struct flash_bank *bank, uint32_t conf_val)
{
	int retval;
	uint32_t regval;
	struct target *target = bank->target;

	retval = target_read_u32(target,
			mrvlqspi_get_reg(bank, CONF), &regval);
	if (retval != ERROR_OK)
		return retval;

	regval &= ~(CONF_MASK << CONF_OFFSET);
	regval |= (conf_val << CONF_OFFSET);

	return target_write_u32(target,
			mrvlqspi_get_reg(bank, CONF), regval);
}

static int mrvlqspi_set_ss_state(struct flash_bank *bank, bool state, int timeout)
{
	int retval;
	uint32_t regval;
	struct target *target = bank->target;

	retval = target_read_u32(target,
			mrvlqspi_get_reg(bank, CNTL), &regval);
	if (retval != ERROR_OK)
		return retval;

	if (state)
		regval |= SS_EN;
	else
		regval &= ~(SS_EN);

	retval = target_write_u32(target,
			mrvlqspi_get_reg(bank, CNTL), regval);
	if (retval != ERROR_OK)
		return retval;

	/* wait for xfer_ready to set */
	for (;;) {
		retval = target_read_u32(target,
				mrvlqspi_get_reg(bank, CNTL), &regval);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%08" PRIx32, regval);
		if ((regval & XFER_RDY) == XFER_RDY)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}
	return ERROR_OK;
}

static int mrvlqspi_start_transfer(struct flash_bank *bank, bool rw_mode)
{
	int retval;
	uint32_t regval;
	struct target *target = bank->target;

	retval = mrvlqspi_set_ss_state(bank, QSPI_SS_ENABLE, QSPI_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target,
			mrvlqspi_get_reg(bank, CONF), &regval);
	if (retval != ERROR_OK)
		return retval;

	if (rw_mode)
		regval |= RW_EN;
	else
		regval &= ~(RW_EN);

	regval |= XFER_START;

	retval = target_write_u32(target,
			mrvlqspi_get_reg(bank, CONF), regval);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int mrvlqspi_stop_transfer(struct flash_bank *bank)
{
	int retval;
	uint32_t regval;
	struct target *target = bank->target;
	int timeout = QSPI_TIMEOUT;

	/* wait for xfer_ready and wfifo_empty to set */
	for (;;) {
		retval = target_read_u32(target,
				mrvlqspi_get_reg(bank, CNTL), &regval);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%08" PRIx32, regval);
		if ((regval & (XFER_RDY | WFIFO_EMPTY)) ==
					(XFER_RDY | WFIFO_EMPTY))
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	retval = target_read_u32(target,
			mrvlqspi_get_reg(bank, CONF), &regval);
	if (retval != ERROR_OK)
		return retval;

	regval |= XFER_STOP;

	retval = target_write_u32(target,
			mrvlqspi_get_reg(bank, CONF), regval);
	if (retval != ERROR_OK)
		return retval;

	/* wait for xfer_start to reset */
	for (;;) {
		retval = target_read_u32(target,
				mrvlqspi_get_reg(bank, CONF), &regval);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%08" PRIx32, regval);
		if ((regval & XFER_START) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	retval = mrvlqspi_set_ss_state(bank, QSPI_SS_DISABLE, QSPI_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int mrvlqspi_fifo_flush(struct flash_bank *bank, int timeout)
{
	int retval;
	uint32_t val;
	struct target *target = bank->target;

	retval = target_read_u32(target,
			mrvlqspi_get_reg(bank, CONF), &val);
	if (retval != ERROR_OK)
		return retval;

	val |= FIFO_FLUSH;

	retval = target_write_u32(target,
			mrvlqspi_get_reg(bank, CONF), val);
	if (retval != ERROR_OK)
		return retval;

	/* wait for fifo_flush to clear */
	for (;;) {
		retval = target_read_u32(target,
				mrvlqspi_get_reg(bank, CONF), &val);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%08" PRIX32, val);
		if ((val & FIFO_FLUSH) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}
	return ERROR_OK;
}

static int mrvlqspi_read_byte(struct flash_bank *bank, uint8_t *data)
{
	int retval;
	uint32_t val;
	struct target *target = bank->target;

	/* wait for rfifo_empty to reset */
	for (;;) {
		retval = target_read_u32(target,
				mrvlqspi_get_reg(bank, CNTL), &val);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%08" PRIx32, val);
		if ((val & RFIFO_EMPTY) == 0)
			break;
		usleep(10);
	}

	retval = target_read_u32(target,
			mrvlqspi_get_reg(bank, DIN), &val);
	if (retval != ERROR_OK)
		return retval;

	*data = val & 0xFF;

	return ERROR_OK;
}

static int mrvlqspi_flash_busy_status(struct flash_bank *bank, int timeout)
{
	uint8_t val;
	int retval;

	/* Flush read/write fifos */
	retval = mrvlqspi_fifo_flush(bank, FIFO_FLUSH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction/addr count value */
	retval = mrvlqspi_set_hdr_cnt(bank, 0x1);
	if (retval != ERROR_OK)
		return retval;

	/* Read flash status register in continuous manner */
	retval = mrvlqspi_set_din_cnt(bank, 0x0);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction */
	retval = mrvlqspi_set_instr(bank, INS_READ_STATUS);
	if (retval != ERROR_OK)
		return retval;

	/* Set data and addr pin length */
	retval = mrvlqspi_set_conf(bank, 0x0);
	if (retval != ERROR_OK)
		return retval;

	/* Enable read mode transfer */
	retval = mrvlqspi_start_transfer(bank, QSPI_R_EN);
	if (retval != ERROR_OK)
		return retval;

	for (;;) {
		retval = mrvlqspi_read_byte(bank, &val);
		if (retval != ERROR_OK)
			return retval;
		if (!(val & 0x1))
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	return mrvlqspi_stop_transfer(bank);
}

static int mrvlqspi_set_write_status(struct flash_bank *bank, bool mode)
{
	int retval;
	uint32_t instr;

	/* Flush read/write fifos */
	retval = mrvlqspi_fifo_flush(bank, FIFO_FLUSH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction/addr count value */
	retval = mrvlqspi_set_hdr_cnt(bank, 0x1);
	if (retval != ERROR_OK)
		return retval;

	if (mode)
		instr = INS_WRITE_ENABLE;
	else
		instr = INS_WRITE_DISABLE;

	/* Set instruction */
	retval = mrvlqspi_set_instr(bank, instr);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_start_transfer(bank, QSPI_W_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_stop_transfer(bank);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int mrvlqspi_read_id(struct flash_bank *bank, uint32_t *id)
{
	uint8_t id_buf[3] = {0, 0, 0};
	int retval, i;

	LOG_DEBUG("Getting ID");

	/* Flush read/write fifos */
	retval = mrvlqspi_fifo_flush(bank, FIFO_FLUSH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction/addr count value */
	retval = mrvlqspi_set_hdr_cnt(bank, 0x1);
	if (retval != ERROR_OK)
		return retval;

	/* Set count for number of bytes to read */
	retval = mrvlqspi_set_din_cnt(bank, 0x3);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction */
	retval = mrvlqspi_set_instr(bank, SPIFLASH_READ_ID);
	if (retval != ERROR_OK)
		return retval;

	/* Set data and addr pin length */
	retval = mrvlqspi_set_conf(bank, 0x0);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_start_transfer(bank, QSPI_R_EN);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < 3; i++) {
		retval = mrvlqspi_read_byte(bank, &id_buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	LOG_DEBUG("ID is 0x%02" PRIx8 " 0x%02" PRIx8 " 0x%02" PRIx8,
					id_buf[0], id_buf[1], id_buf[2]);
	retval = mrvlqspi_set_ss_state(bank, QSPI_SS_DISABLE, QSPI_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	*id = id_buf[2] << 16 | id_buf[1] << 8 | id_buf[0];
	return ERROR_OK;
}

static int mrvlqspi_block_erase(struct flash_bank *bank, uint32_t offset)
{
	int retval;
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;

	/* Set flash write enable */
	retval = mrvlqspi_set_write_status(bank, WRITE_ENABLE);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction/addr count value */
	retval = mrvlqspi_set_hdr_cnt(bank, (0x1 | (0x3 << 4)));
	if (retval != ERROR_OK)
		return retval;

	/* Set read offset address */
	retval = mrvlqspi_set_addr(bank, offset);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction */
	retval = mrvlqspi_set_instr(bank, mrvlqspi_info->dev->erase_cmd);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_start_transfer(bank, QSPI_W_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_stop_transfer(bank);
	if (retval != ERROR_OK)
		return retval;

	return mrvlqspi_flash_busy_status(bank, BLOCK_ERASE_TIMEOUT);
}

static int mrvlqspi_bulk_erase(struct flash_bank *bank)
{
	int retval;
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;

	if (mrvlqspi_info->dev->chip_erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	/* Set flash write enable */
	retval = mrvlqspi_set_write_status(bank, WRITE_ENABLE);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction */
	retval = mrvlqspi_set_instr(bank, mrvlqspi_info->dev->chip_erase_cmd);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_start_transfer(bank, QSPI_W_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_stop_transfer(bank);
	if (retval != ERROR_OK)
		return retval;

	return mrvlqspi_flash_busy_status(bank, CHIP_ERASE_TIMEOUT);
}

static int mrvlqspi_flash_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("erase from sector %u to sector %u", first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(mrvlqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	/* If we're erasing the entire chip and the flash supports
	 * it, use a bulk erase instead of going sector-by-sector. */
	if (first == 0 && last == (bank->num_sectors - 1)
		&& mrvlqspi_info->dev->chip_erase_cmd !=
					mrvlqspi_info->dev->erase_cmd) {
		LOG_DEBUG("Chip supports the bulk erase command."
		" Will use bulk erase instead of sector-by-sector erase.");
		retval = mrvlqspi_bulk_erase(bank);
		if (retval == ERROR_OK) {
			return retval;
		} else
			LOG_WARNING("Bulk flash erase failed."
				" Falling back to sector-by-sector erase.");
	}

	if (mrvlqspi_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = mrvlqspi_block_erase(bank,
				sector * mrvlqspi_info->dev->sectorsize);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int mrvlqspi_flash_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;
	int retval = ERROR_OK;
	uint32_t page_size, fifo_size;
	struct working_area *fifo;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	struct working_area *write_algorithm;

	LOG_DEBUG("offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > mrvlqspi_info->dev->size_in_bytes) {
		LOG_WARNING("Writes past end of flash. Extra data discarded.");
		count = mrvlqspi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
			(bank->sectors[sector].offset + bank->sectors[sector].size))
			&& ((offset + count - 1) >= bank->sectors[sector].offset)
			&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	/* if no valid page_size, use reasonable default */
	page_size = mrvlqspi_info->dev->pagesize ?
		mrvlqspi_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	/* See contrib/loaders/flash/mrvlqspi.S for src */
	static const uint8_t mrvlqspi_flash_write_code[] = {
		0x4f, 0xf0, 0x00, 0x0a, 0xa2, 0x44, 0x92, 0x45,
		0x7f, 0xf6, 0xfc, 0xaf, 0x00, 0xf0, 0x6b, 0xf8,
		0x5f, 0xf0, 0x01, 0x08, 0xc5, 0xf8, 0x1c, 0x80,
		0x5f, 0xf0, 0x06, 0x08, 0xc5, 0xf8, 0x10, 0x80,
		0x5f, 0xf0, 0x01, 0x09, 0x00, 0xf0, 0x6b, 0xf8,
		0x00, 0xf0, 0x7d, 0xf8, 0x5f, 0xf0, 0x31, 0x08,
		0xc5, 0xf8, 0x1c, 0x80, 0x90, 0x46, 0xc5, 0xf8,
		0x14, 0x80, 0x5f, 0xf0, 0x02, 0x08, 0xc5, 0xf8,
		0x10, 0x80, 0x5f, 0xf0, 0x01, 0x09, 0x00, 0xf0,
		0x5a, 0xf8, 0xd0, 0xf8, 0x00, 0x80, 0xb8, 0xf1,
		0x00, 0x0f, 0x00, 0xf0, 0x8b, 0x80, 0x47, 0x68,
		0x47, 0x45, 0x3f, 0xf4, 0xf6, 0xaf, 0x17, 0xf8,
		0x01, 0x9b, 0x00, 0xf0, 0x30, 0xf8, 0x8f, 0x42,
		0x28, 0xbf, 0x00, 0xf1, 0x08, 0x07, 0x47, 0x60,
		0x01, 0x3b, 0x00, 0x2b, 0x00, 0xf0, 0x05, 0x80,
		0x02, 0xf1, 0x01, 0x02, 0x92, 0x45, 0x7f, 0xf4,
		0xe4, 0xaf, 0x00, 0xf0, 0x50, 0xf8, 0xa2, 0x44,
		0x00, 0xf0, 0x2d, 0xf8, 0x5f, 0xf0, 0x01, 0x08,
		0xc5, 0xf8, 0x1c, 0x80, 0x5f, 0xf0, 0x00, 0x08,
		0xc5, 0xf8, 0x20, 0x80, 0x5f, 0xf0, 0x05, 0x08,
		0xc5, 0xf8, 0x10, 0x80, 0x5f, 0xf0, 0x00, 0x09,
		0x00, 0xf0, 0x29, 0xf8, 0x00, 0xf0, 0x13, 0xf8,
		0x09, 0xf0, 0x01, 0x09, 0xb9, 0xf1, 0x00, 0x0f,
		0xf8, 0xd1, 0x00, 0xf0, 0x34, 0xf8, 0x00, 0x2b,
		0xa4, 0xd1, 0x00, 0xf0, 0x53, 0xb8, 0xd5, 0xf8,
		0x00, 0x80, 0x5f, 0xea, 0x08, 0x68, 0xfa, 0xd4,
		0xc5, 0xf8, 0x08, 0x90, 0x70, 0x47, 0xd5, 0xf8,
		0x00, 0x80, 0x5f, 0xea, 0xc8, 0x68, 0xfa, 0xd4,
		0xd5, 0xf8, 0x0c, 0x90, 0x70, 0x47, 0xd5, 0xf8,
		0x04, 0x80, 0x48, 0xf4, 0x00, 0x78, 0xc5, 0xf8,
		0x04, 0x80, 0xd5, 0xf8, 0x04, 0x80, 0x5f, 0xea,
		0x88, 0x58, 0xfa, 0xd4, 0x70, 0x47, 0xd5, 0xf8,
		0x00, 0x80, 0x48, 0xf0, 0x01, 0x08, 0xc5, 0xf8,
		0x00, 0x80, 0xd5, 0xf8, 0x00, 0x80, 0x5f, 0xea,
		0x88, 0x78, 0xfa, 0xd5, 0xd5, 0xf8, 0x04, 0x80,
		0x69, 0xf3, 0x4d, 0x38, 0x48, 0xf4, 0x00, 0x48,
		0xc5, 0xf8, 0x04, 0x80, 0x70, 0x47, 0xd5, 0xf8,
		0x00, 0x80, 0x5f, 0xea, 0x88, 0x78, 0xfa, 0xd5,
		0xd5, 0xf8, 0x00, 0x80, 0x5f, 0xea, 0x48, 0x68,
		0xfa, 0xd5, 0xd5, 0xf8, 0x04, 0x80, 0x48, 0xf4,
		0x80, 0x48, 0xc5, 0xf8, 0x04, 0x80, 0xd5, 0xf8,
		0x04, 0x80, 0x5f, 0xea, 0x08, 0x48, 0xfa, 0xd4,
		0xd5, 0xf8, 0x00, 0x80, 0x28, 0xf0, 0x01, 0x08,
		0xc5, 0xf8, 0x00, 0x80, 0xd5, 0xf8, 0x00, 0x80,
		0x5f, 0xea, 0x88, 0x78, 0xfa, 0xd5, 0x70, 0x47,
		0x00, 0x20, 0x50, 0x60, 0x30, 0x46, 0x00, 0xbe
	};

	if (target_alloc_working_area(target, sizeof(mrvlqspi_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_ERROR("Insufficient working area. You must configure"
			" a working area > %zdB in order to write to SPIFI flash.",
			sizeof(mrvlqspi_flash_write_code));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(mrvlqspi_flash_write_code),
			mrvlqspi_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* FIFO allocation */
	fifo_size = target_get_working_area_avail(target);

	if (fifo_size == 0) {
		/* if we already allocated the writing code but failed to get fifo
		 * space, free the algorithm */
		target_free_working_area(target, write_algorithm);

		LOG_ERROR("Insufficient working area. Please allocate at least"
			" %zdB of working area to enable flash writes.",
			sizeof(mrvlqspi_flash_write_code) + 1
		);

		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	} else if (fifo_size < page_size)
		LOG_WARNING("Working area size is limited; flash writes may be"
			" slow. Increase working area size to at least %zdB"
			" to reduce write times.",
			(size_t)(sizeof(mrvlqspi_flash_write_code) + page_size)
		);

	if (target_alloc_working_area(target, fifo_size, &fifo) != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);	/* page size */
	init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);	/* qspi base address */

	buf_set_u32(reg_params[0].value, 0, 32, fifo->address);
	buf_set_u32(reg_params[1].value, 0, 32, fifo->address + fifo->size);
	buf_set_u32(reg_params[2].value, 0, 32, offset);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, page_size);
	buf_set_u32(reg_params[5].value, 0, 32, (uint32_t) mrvlqspi_info->reg_base);

	retval = target_run_flash_async_algorithm(target, buffer, count, 1,
			0, NULL,
			6, reg_params,
			fifo->address, fifo->size,
			write_algorithm->address, 0,
			&armv7m_info
	);

	if (retval != ERROR_OK)
		LOG_ERROR("Error executing flash write algorithm");

	target_free_working_area(target, fifo);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

	return retval;
}

static int mrvlqspi_flash_read(struct flash_bank *bank, uint8_t *buffer,
				uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;
	int retval;
	uint32_t i;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(mrvlqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Flush read/write fifos */
	retval = mrvlqspi_fifo_flush(bank, FIFO_FLUSH_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction/addr count value */
	retval = mrvlqspi_set_hdr_cnt(bank, (0x1 | (0x3 << 4)));
	if (retval != ERROR_OK)
		return retval;

	/* Set count for number of bytes to read */
	retval = mrvlqspi_set_din_cnt(bank, count);
	if (retval != ERROR_OK)
		return retval;

	/* Set read address */
	retval = mrvlqspi_set_addr(bank, offset);
	if (retval != ERROR_OK)
		return retval;

	/* Set instruction */
	retval = mrvlqspi_set_instr(bank, SPIFLASH_READ);
	if (retval != ERROR_OK)
		return retval;

	/* Set data and addr pin length */
	retval = mrvlqspi_set_conf(bank, 0x0);
	if (retval != ERROR_OK)
		return retval;

	retval = mrvlqspi_start_transfer(bank, QSPI_R_EN);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < count; i++) {
		retval = mrvlqspi_read_byte(bank, &buffer[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = mrvlqspi_set_ss_state(bank, QSPI_SS_DISABLE, QSPI_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int mrvlqspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;
	uint32_t id = 0;
	int retval;
	struct flash_sector *sectors;
	uint32_t sectorsize;

	/* If we've already probed, we should be fine to skip this time. */
	if (mrvlqspi_info->probed)
		return ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	mrvlqspi_info->probed = false;
	mrvlqspi_info->bank_num = bank->bank_number;

	/* Read flash JEDEC ID */
	retval = mrvlqspi_read_id(bank, &id);
	if (retval != ERROR_OK)
		return retval;

	mrvlqspi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			mrvlqspi_info->dev = p;
			break;
		}

	if (!mrvlqspi_info->dev) {
		LOG_ERROR("Unknown flash device ID 0x%08" PRIx32, id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' ID 0x%08" PRIx32,
		mrvlqspi_info->dev->name, mrvlqspi_info->dev->device_id);


	/* Set correct size value */
	bank->size = mrvlqspi_info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = mrvlqspi_info->dev->sectorsize ?
		mrvlqspi_info->dev->sectorsize : mrvlqspi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = mrvlqspi_info->dev->size_in_bytes / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	mrvlqspi_info->probed = true;

	return ERROR_OK;
}

static int mrvlqspi_auto_probe(struct flash_bank *bank)
{
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;
	if (mrvlqspi_info->probed)
		return ERROR_OK;
	return mrvlqspi_probe(bank);
}

static int mrvlqspi_flash_erase_check(struct flash_bank *bank)
{
	/* Not implemented yet */
	return ERROR_OK;
}

static int mrvlqspi_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct mrvlqspi_flash_bank *mrvlqspi_info = bank->driver_priv;

	if (!(mrvlqspi_info->probed)) {
		snprintf(buf, buf_size,
			"\nQSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nQSPI flash information:\n"
		"  Device \'%s\' ID 0x%08" PRIx32 "\n",
		mrvlqspi_info->dev->name, mrvlqspi_info->dev->device_id);

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(mrvlqspi_flash_bank_command)
{
	struct mrvlqspi_flash_bank *mrvlqspi_info;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	mrvlqspi_info = malloc(sizeof(struct mrvlqspi_flash_bank));
	if (mrvlqspi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	/* Get QSPI controller register map base address */
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], mrvlqspi_info->reg_base);
	bank->driver_priv = mrvlqspi_info;
	mrvlqspi_info->probed = false;

	return ERROR_OK;
}

const struct flash_driver mrvlqspi_flash = {
	.name = "mrvlqspi",
	.flash_bank_command = mrvlqspi_flash_bank_command,
	.erase = mrvlqspi_flash_erase,
	.write = mrvlqspi_flash_write,
	.read = mrvlqspi_flash_read,
	.probe = mrvlqspi_probe,
	.auto_probe = mrvlqspi_auto_probe,
	.erase_check = mrvlqspi_flash_erase_check,
	.info = mrvlqspi_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
