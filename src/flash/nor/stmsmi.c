/***************************************************************************
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

/* STM Serial Memory Interface (SMI) controller is a SPI bus controller
 * specifically designed for SPI memories.
 * Only SPI "mode 3" (CPOL=1 and CPHA=1) is supported.
 * Two working modes are available:
 * - SW mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus.
 * - HW mode: the SPI but is under SMI control. Memory content is directly
 *   accessible in CPU memory space. CPU can read, write and execute memory
 *   content. */

/* ATTENTION:
 * To have flash memory mapped in CPU memory space, the SMI controller
 * have to be in "HW mode". This requires following constraints:
 * 1) The command "reset init" have to initialize SMI controller and put
 *    it in HW mode;
 * 2) every command in this file have to return to prompt in HW mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>

#define SMI_READ_REG(a)				\
({									\
	int _ret;						\
	uint32_t _value;				\
									\
	_ret = target_read_u32(target, io_base + (a), &_value); \
	if (_ret != ERROR_OK)			\
		return _ret;				\
	_value;							\
})

#define SMI_WRITE_REG(a, v)			\
{									\
	int _retval;					\
									\
	_retval = target_write_u32(target, io_base + (a), (v)); \
	if (_retval != ERROR_OK)		\
		return _retval;				\
}

#define SMI_POLL_TFF(timeout)		\
{									\
	int _retval;					\
									\
	_retval = poll_tff(target, io_base, timeout); \
	if (_retval != ERROR_OK)		\
		return _retval;				\
}

#define SMI_SET_SW_MODE()	SMI_WRITE_REG(SMI_CR1, \
	SMI_READ_REG(SMI_CR1) | SMI_SW_MODE)
#define SMI_SET_HWWB_MODE() SMI_WRITE_REG(SMI_CR1, \
	(SMI_READ_REG(SMI_CR1) | SMI_WB_MODE) & ~SMI_SW_MODE)
#define SMI_SET_HW_MODE()	SMI_WRITE_REG(SMI_CR1, \
	SMI_READ_REG(SMI_CR1) & ~(SMI_SW_MODE | SMI_WB_MODE))
#define SMI_CLEAR_TFF()		SMI_WRITE_REG(SMI_SR, ~SMI_TFF)

#define SMI_BANK_SIZE      (0x01000000)

#define SMI_CR1 (0x00) /* Control register 1 */
#define SMI_CR2 (0x04) /* Control register 2 */
#define SMI_SR  (0x08) /* Status register */
#define SMI_TR  (0x0c) /* TX */
#define SMI_RR  (0x10) /* RX */

/* fields in SMI_CR1 */
#define SMI_SW_MODE       0x10000000 /* set to enable SW Mode */
#define SMI_WB_MODE       0x20000000 /* Write Burst Mode */

/* fields in SMI_CR2 */
#define SMI_TX_LEN_1      0x00000001 /* data length = 1 byte */
#define SMI_TX_LEN_4      0x00000004 /* data length = 4 byte */
#define SMI_RX_LEN_3      0x00000030 /* data length = 3 byte */
#define SMI_SEND          0x00000080 /* Send data */
#define SMI_RSR           0x00000400 /* reads status reg */
#define SMI_WE            0x00000800 /* Write Enable */
#define SMI_SEL_BANK0     0x00000000 /* Select Bank0 */
#define SMI_SEL_BANK1     0x00001000 /* Select Bank1 */
#define SMI_SEL_BANK2     0x00002000 /* Select Bank2 */
#define SMI_SEL_BANK3     0x00003000 /* Select Bank3 */

/* fields in SMI_SR */
#define SMI_TFF           0x00000100 /* Transfer Finished Flag */

/* Commands */
#define SMI_READ_ID       0x0000009F /* Read Flash Identification */

/* Timeout in ms */
#define SMI_CMD_TIMEOUT   (100)
#define SMI_PROBE_TIMEOUT (100)
#define SMI_MAX_TIMEOUT  (3000)

struct stmsmi_flash_bank {
	bool probed;
	uint32_t io_base;
	uint32_t bank_num;
	const struct flash_device *dev;
};

struct stmsmi_target {
	char *name;
	uint32_t tap_idcode;
	uint32_t smi_base;
	uint32_t io_base;
};

static const struct stmsmi_target target_devices[] = {
	/* name,          tap_idcode, smi_base,   io_base */
	{ "SPEAr3xx/6xx", 0x07926041, 0xf8000000, 0xfc000000 },
	{ "STR75x",       0x4f1f0041, 0x80000000, 0x90000000 },
	{ NULL,           0,          0,          0 }
};

FLASH_BANK_COMMAND_HANDLER(stmsmi_flash_bank_command)
{
	struct stmsmi_flash_bank *stmsmi_info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	stmsmi_info = malloc(sizeof(struct stmsmi_flash_bank));
	if (stmsmi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = stmsmi_info;
	stmsmi_info->probed = false;

	return ERROR_OK;
}

/* Poll transmit finished flag */
/* timeout in ms */
static int poll_tff(struct target *target, uint32_t io_base, int timeout)
{
	int64_t endtime;

	if (SMI_READ_REG(SMI_SR) & SMI_TFF)
		return ERROR_OK;

	endtime = timeval_ms() + timeout;
	do {
		alive_sleep(1);
		if (SMI_READ_REG(SMI_SR) & SMI_TFF)
			return ERROR_OK;
	} while (timeval_ms() < endtime);

	LOG_ERROR("Timeout while polling TFF");
	return ERROR_FLASH_OPERATION_FAILED;
}

/* Read the status register of the external SPI flash chip.
 * The operation is triggered by setting SMI_RSR bit.
 * SMI sends the proper SPI command (0x05) and returns value in SMI_SR */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base = stmsmi_info->io_base;

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* Read status */
	SMI_WRITE_REG(SMI_CR2, stmsmi_info->bank_num | SMI_RSR);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	*status = SMI_READ_REG(SMI_SR) & 0x0000ffff;

	/* clean-up SMI_CR2 */
	SMI_WRITE_REG(SMI_CR2, 0); /* AB: Required ? */

	return ERROR_OK;
}

/* check for WIP (write in progress) bit in status register */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval;
	int64_t endtime;

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

/* Send "write enable" command to SPI flash chip.
 * The operation is triggered by setting SMI_WE bit, and SMI sends
 * the proper SPI command (0x06) */
static int smi_write_enable(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base = stmsmi_info->io_base;
	uint32_t status;
	int retval;

	/* Enter in HW mode */
	SMI_SET_HW_MODE(); /* AB: is this correct ?*/

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* Send write enable command */
	SMI_WRITE_REG(SMI_CR2, stmsmi_info->bank_num | SMI_WE);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* read flash status register */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		return retval;

	/* Check write enabled */
	if ((status & SPIFLASH_WE_BIT) == 0) {
		LOG_ERROR("Cannot enable write to flash. Status=0x%08" PRIx32, status);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static uint32_t erase_command(struct stmsmi_flash_bank *stmsmi_info,
	uint32_t offset)
{
	uint8_t cmd_bytes[] = {
		stmsmi_info->dev->erase_cmd,
		offset >> 16,
		offset >> 8,
		offset
	};

	return le_to_h_u32(cmd_bytes);
}

static int smi_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base = stmsmi_info->io_base;
	uint32_t cmd;
	int retval;

	retval = smi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Switch to SW mode to send sector erase command */
	SMI_SET_SW_MODE();

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* send SPI command "block erase" */
	cmd = erase_command(stmsmi_info, bank->sectors[sector].offset);
	SMI_WRITE_REG(SMI_TR, cmd);
	SMI_WRITE_REG(SMI_CR2, stmsmi_info->bank_num | SMI_SEND | SMI_TX_LEN_4);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* poll WIP for end of self timed Sector Erase cycle */
	retval = wait_till_ready(bank, SMI_MAX_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stmsmi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base = stmsmi_info->io_base;
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

	if (!(stmsmi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	if (stmsmi_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = smi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	/* Switch to HW mode before return to prompt */
	SMI_SET_HW_MODE();
	return retval;
}

static int stmsmi_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int smi_write_buffer(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t address, uint32_t len)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base = stmsmi_info->io_base;
	int retval;

	LOG_DEBUG("%s: address=0x%08" PRIx32 " len=0x%08" PRIx32,
			__func__, address, len);

	retval = smi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	/* HW mode, write burst mode */
	SMI_SET_HWWB_MODE();

	retval = target_write_buffer(target, address, len, buffer);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stmsmi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base = stmsmi_info->io_base;
	uint32_t cur_count, page_size, page_offset;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > stmsmi_info->dev->size_in_bytes) {
		LOG_WARNING("Write pasts end of flash. Extra data discarded.");
		count = stmsmi_info->dev->size_in_bytes - offset;
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
	page_size = stmsmi_info->dev->pagesize ?
		stmsmi_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	/* unaligned buffer head */
	if (count > 0 && (offset & 3) != 0) {
		cur_count = 4 - (offset & 3);
		if (cur_count > count)
			cur_count = count;
		retval = smi_write_buffer(bank, buffer, bank->base + offset,
			cur_count);
		if (retval != ERROR_OK)
			goto err;
		offset += cur_count;
		buffer += cur_count;
		count -= cur_count;
	}

	page_offset = offset % page_size;
	/* central part, aligned words */
	while (count >= 4) {
		/* clip block at page boundary */
		if (page_offset + count > page_size)
			cur_count = page_size - page_offset;
		else
			cur_count = count & ~3;

		retval = smi_write_buffer(bank, buffer, bank->base + offset,
			cur_count);
		if (retval != ERROR_OK)
			goto err;

		page_offset = 0;
		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;

		keep_alive();
	}

	/* buffer tail */
	if (count > 0)
		retval = smi_write_buffer(bank, buffer, bank->base + offset, count);

err:
	/* Switch to HW mode before return to prompt */
	SMI_SET_HW_MODE();
	return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base = stmsmi_info->io_base;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* poll WIP */
	retval = wait_till_ready(bank, SMI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* enter in SW mode */
	SMI_SET_SW_MODE();

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* Send SPI command "read ID" */
	SMI_WRITE_REG(SMI_TR, SMI_READ_ID);
	SMI_WRITE_REG(SMI_CR2,
		stmsmi_info->bank_num | SMI_SEND | SMI_RX_LEN_3 | SMI_TX_LEN_1);

	/* Poll transmit finished flag */
	SMI_POLL_TFF(SMI_CMD_TIMEOUT);

	/* clear transmit finished flag */
	SMI_CLEAR_TFF();

	/* read ID from Receive Register */
	*id = SMI_READ_REG(SMI_RR) & 0x00ffffff;
	return ERROR_OK;
}

static int stmsmi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	uint32_t io_base, sectorsize;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	const struct stmsmi_target *target_device;
	int retval;

	if (stmsmi_info->probed)
		free(bank->sectors);
	stmsmi_info->probed = false;

	for (target_device = target_devices ; target_device->name ; ++target_device)
		if (target_device->tap_idcode == target->tap->idcode)
			break;
	if (!target_device->name) {
		LOG_ERROR("Device ID 0x%" PRIx32 " is not known as SMI capable",
				target->tap->idcode);
		return ERROR_FAIL;
	}

	switch (bank->base - target_device->smi_base) {
		case 0:
			stmsmi_info->bank_num = SMI_SEL_BANK0;
			break;
		case SMI_BANK_SIZE:
			stmsmi_info->bank_num = SMI_SEL_BANK1;
			break;
		case 2*SMI_BANK_SIZE:
			stmsmi_info->bank_num = SMI_SEL_BANK2;
			break;
		case 3*SMI_BANK_SIZE:
			stmsmi_info->bank_num = SMI_SEL_BANK3;
			break;
		default:
			LOG_ERROR("Invalid SMI base address " TARGET_ADDR_FMT, bank->base);
			return ERROR_FAIL;
	}
	io_base = target_device->io_base;
	stmsmi_info->io_base = io_base;

	LOG_DEBUG("Valid SMI on device %s at address " TARGET_ADDR_FMT,
		target_device->name, bank->base);

	/* read and decode flash ID; returns in SW mode */
	retval = read_flash_id(bank, &id);
	SMI_SET_HW_MODE();
	if (retval != ERROR_OK)
		return retval;

	stmsmi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			stmsmi_info->dev = p;
			break;
		}

	if (!stmsmi_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
		stmsmi_info->dev->name, stmsmi_info->dev->device_id);

	/* Set correct size value */
	bank->size = stmsmi_info->dev->size_in_bytes;
	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = stmsmi_info->dev->sectorsize ?
		stmsmi_info->dev->sectorsize : stmsmi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors =
		stmsmi_info->dev->size_in_bytes / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 1;
	}

	bank->sectors = sectors;
	stmsmi_info->probed = true;
	return ERROR_OK;
}

static int stmsmi_auto_probe(struct flash_bank *bank)
{
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;
	if (stmsmi_info->probed)
		return ERROR_OK;
	return stmsmi_probe(bank);
}

static int stmsmi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_stmsmi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct stmsmi_flash_bank *stmsmi_info = bank->driver_priv;

	if (!(stmsmi_info->probed)) {
		snprintf(buf, buf_size,
			"\nSMI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nSMI flash information:\n"
		"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
		stmsmi_info->dev->name, stmsmi_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver stmsmi_flash = {
	.name = "stmsmi",
	.flash_bank_command = stmsmi_flash_bank_command,
	.erase = stmsmi_erase,
	.protect = stmsmi_protect,
	.write = stmsmi_write,
	.read = default_flash_read,
	.probe = stmsmi_probe,
	.auto_probe = stmsmi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = stmsmi_protect_check,
	.info = get_stmsmi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
