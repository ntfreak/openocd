/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
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
#include <target/avrt.h>

/* AVR_JTAG_Instructions */
#define AVR_JTAG_INS_LEN                                        4
/* Public Instructions: */
#define AVR_JTAG_INS_EXTEST                                     0x00
#define AVR_JTAG_INS_IDCODE                                     0x01
#define AVR_JTAG_INS_SAMPLE_PRELOAD                             0x02
#define AVR_JTAG_INS_BYPASS                                     0x0F
/* AVR Specified Public Instructions: */
#define AVR_JTAG_INS_AVR_RESET                                  0x0C
#define AVR_JTAG_INS_PROG_ENABLE                                0x04
#define AVR_JTAG_INS_PROG_COMMANDS                              0x05
#define AVR_JTAG_INS_PROG_PAGELOAD                              0x06
#define AVR_JTAG_INS_PROG_PAGEREAD                              0x07

/* Data Registers: */
#define AVR_JTAG_REG_Bypass_Len                                 1
#define AVR_JTAG_REG_DeviceID_Len                               32

#define AVR_JTAG_REG_Reset_Len                                  1
#define AVR_JTAG_REG_JTAGID_Len                                 32
#define AVR_JTAG_REG_ProgrammingEnable_Len                      16
#define AVR_JTAG_REG_ProgrammingCommand_Len                     15
#define AVR_JTAG_REG_FlashDataByte_Len                          16

struct avrf_type {
	char name[15];
	uint16_t chip_id;
	int flash_page_size;
	int flash_page_num;
	int eeprom_page_size;
	int eeprom_page_num;
	int user_signature_page_num;
};

struct avrf_flash_bank {
	int ppage_size;
	bool probed;
};

static const struct avrf_type avft_chips_info[] = {
/*	name, chip_id,	flash_page_size, flash_page_num,
 *			eeprom_page_size, eeprom_page_num,
 *			user_signature_num
 */
	{"atmega128", 0x9702, 256, 512, 8, 512, 0},
	{"atmega128rfa1", 0xa701, 128, 512, 8, 512, 3},
	{"atmega256rfr2", 0xa802, 256, 1024, 8, 1024, 3},
	{"at90can128", 0x9781, 256, 512, 8, 512, 0},
	{"at90usb128", 0x9782, 256, 512, 8, 512, 0},
	{"atmega164p", 0x940a, 128, 128, 4, 128, 0},
	{"atmega324p", 0x9508, 128, 256, 4, 256, 0},
	{"atmega324pa", 0x9511, 128, 256, 4, 256, 0},
	{"atmega644p", 0x960a, 256, 256, 8, 256, 0},
	{"atmega1284p", 0x9705, 256, 512, 8, 512, 0},
};

/* avr program functions */
static int avr_jtag_reset(struct avr_common *avr, uint32_t reset)
{
	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_AVR_RESET);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, reset, AVR_JTAG_REG_Reset_Len);

	return ERROR_OK;
}

static int avr_jtag_read_jtagid(struct avr_common *avr, uint32_t *id)
{
	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_IDCODE);
	avr_jtag_senddat(avr->jtag_info.tap, id, 0, AVR_JTAG_REG_JTAGID_Len);

	return ERROR_OK;
}

static int avr_jtagprg_enterprogmode(struct avr_common *avr)
{
	avr_jtag_reset(avr, 1);

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_ENABLE);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0xA370, AVR_JTAG_REG_ProgrammingEnable_Len);

	return ERROR_OK;
}

static int avr_jtagprg_leaveprogmode(struct avr_common *avr)
{
	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x2300, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_ENABLE);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0, AVR_JTAG_REG_ProgrammingEnable_Len);

	avr_jtag_reset(avr, 0);

	return ERROR_OK;
}

static int avr_jtagprg_chiperase(struct avr_common *avr, uint32_t enter_cmd)
{
	uint32_t poll_value;

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, enter_cmd /* 0x2380 */, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3180, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3380, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3380, AVR_JTAG_REG_ProgrammingCommand_Len);

	do {
		poll_value = 0;
		avr_jtag_senddat(avr->jtag_info.tap,
			&poll_value,
			0x3380,
			AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;
		LOG_DEBUG("poll_value = 0x%04" PRIx32 "", poll_value);
	} while (!(poll_value & 0x0200));

	return ERROR_OK;
}

static int avr_jtagprg_writeflashpage(struct avr_common *avr,
	const bool ext_addressing,
	const uint8_t *page_buf,
	uint32_t buf_size,
	uint32_t addr,
	uint32_t page_size,
	uint32_t enter_cmd)
{
	uint32_t poll_value;

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, enter_cmd/* 0x2310 */, AVR_JTAG_REG_ProgrammingCommand_Len);

	/* load extended high byte */
	if (ext_addressing)
		avr_jtag_senddat(avr->jtag_info.tap,
			NULL,
			0x0b00 | ((addr >> 17) & 0xFF),
			AVR_JTAG_REG_ProgrammingCommand_Len);

	/* load addr high byte */
	avr_jtag_senddat(avr->jtag_info.tap,
		NULL,
		0x0700 | ((addr >> 9) & 0xFF),
		AVR_JTAG_REG_ProgrammingCommand_Len);

	/* load addr low byte */
	avr_jtag_senddat(avr->jtag_info.tap,
		NULL,
		0x0300 | ((addr >> 1) & 0xFF),
		AVR_JTAG_REG_ProgrammingCommand_Len);

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_PAGELOAD);

	for (uint32_t i = 0; i < page_size; i++) {
		if (i < buf_size)
			avr_jtag_senddat(avr->jtag_info.tap, NULL, page_buf[i], 8);
		else
			avr_jtag_senddat(avr->jtag_info.tap, NULL, 0xFF, 8);
	}

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);

	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3500, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);

	do {
		poll_value = 0;
		avr_jtag_senddat(avr->jtag_info.tap,
			&poll_value,
			0x3700,
			AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;
		LOG_DEBUG("poll_value = 0x%04" PRIx32 "", poll_value);
	} while (!(poll_value & 0x0200));

	return ERROR_OK;
}

static int avr_jtagprg_writeeeprompage(struct avr_common *avr,
	const uint8_t *page_buf,
	uint32_t buf_size,
	uint32_t addr,
	uint32_t page_size)
{
	uint32_t poll_value;

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x2311, AVR_JTAG_REG_ProgrammingCommand_Len);

	/* load addr high byte */
	avr_jtag_senddat(avr->jtag_info.tap,
		NULL,
		0x0700 | ((addr >> 8) & 0xFF),
		AVR_JTAG_REG_ProgrammingCommand_Len);

	for (uint32_t i = 0; i < page_size; i++) {
		/* load addr low byte */
		avr_jtag_senddat(avr->jtag_info.tap,
				NULL,
				0x0300 | (addr & 0xFF),
				AVR_JTAG_REG_ProgrammingCommand_Len);

		if (i < buf_size)
			avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x1300 | page_buf[i], AVR_JTAG_REG_ProgrammingCommand_Len);
		else
			avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x1300 | 0xFF, AVR_JTAG_REG_ProgrammingCommand_Len);
		avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);
		avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x7700, AVR_JTAG_REG_ProgrammingCommand_Len);
		avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);
	}

	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3100, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);

	do {
		poll_value = 0;
		avr_jtag_senddat(avr->jtag_info.tap,
			&poll_value,
			0x3300,
			AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;
		LOG_DEBUG("poll_value = 0x%04" PRIx32 "", poll_value);
	} while (!(poll_value & 0x0200));

	return ERROR_OK;
}

static int avr_jtagprg_writefuses(struct avr_common *avr,
	const uint8_t *page_buf,
	uint32_t buf_size,
	uint32_t addr,
	uint32_t page_size)
{
	uint32_t poll_value;

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x2340, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x1300 | page_buf[0], AVR_JTAG_REG_ProgrammingCommand_Len);

	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3b00, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3900, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3b00, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3b00, AVR_JTAG_REG_ProgrammingCommand_Len);


	do {
		poll_value = 0;
		avr_jtag_senddat(avr->jtag_info.tap,
			&poll_value,
			0x3700,
			AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;
		LOG_DEBUG("poll_value = 0x%04" PRIx32 "", poll_value);
	} while (!(poll_value & 0x0200));

	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x1300 | page_buf[1], AVR_JTAG_REG_ProgrammingCommand_Len);

	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3500, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);

	do {
		poll_value = 0;
		avr_jtag_senddat(avr->jtag_info.tap,
			&poll_value,
			0x3700,
			AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;
		LOG_DEBUG("poll_value = 0x%04" PRIx32 "", poll_value);
	} while (!(poll_value & 0x0200));

	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x1300 | page_buf[2], AVR_JTAG_REG_ProgrammingCommand_Len);

	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3100, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);

	do {
		poll_value = 0;
		avr_jtag_senddat(avr->jtag_info.tap,
			&poll_value,
			0x3300,
			AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;
		LOG_DEBUG("poll_value = 0x%04" PRIx32 "", poll_value);
	} while (!(poll_value & 0x0200));

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(avrf_flash_bank_command)
{
	struct avrf_flash_bank *avrf_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	avrf_info = malloc(sizeof(struct avrf_flash_bank));
	bank->driver_priv = avrf_info;

	avrf_info->probed = false;

	return ERROR_OK;
}

static int avrf_common_erase(struct flash_bank *bank, int first, int last, uint32_t enter_cmd)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	int status;

	LOG_DEBUG("%s", __func__);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	status = avr_jtagprg_enterprogmode(avr);
	if (status != ERROR_OK)
		return status;

	status = avr_jtagprg_chiperase(avr, enter_cmd);
	if (status != ERROR_OK)
		return status;

	return avr_jtagprg_leaveprogmode(avr);
}

static int avrf_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	return avrf_common_erase(bank, first, last, 0x2380);
}

static int avrf_common_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count, uint32_t enter_cmd)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	uint32_t cur_size, cur_buffer_size, page_size;
	bool ext_addressing;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	page_size = bank->sectors[0].size;
	if ((offset % page_size) != 0) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required %" PRIu32 "-byte alignment",
			offset,
			page_size);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	LOG_DEBUG("offset is 0x%08" PRIx32 "", offset);
	LOG_DEBUG("count is %" PRIu32 "", count);

	if (ERROR_OK != avr_jtagprg_enterprogmode(avr))
		return ERROR_FAIL;

	if (bank->size > 0x20000)
		ext_addressing = true;
	else
		ext_addressing = false;

	cur_size = 0;
	while (count > 0) {
		if (count > page_size)
			cur_buffer_size = page_size;
		else
			cur_buffer_size = count;
		avr_jtagprg_writeflashpage(avr,
			ext_addressing,
			buffer + cur_size,
			cur_buffer_size,
			offset + cur_size,
			page_size,
			enter_cmd);
		count -= cur_buffer_size;
		cur_size += cur_buffer_size;

		keep_alive();
	}

	return avr_jtagprg_leaveprogmode(avr);
}

static int avrf_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return avrf_common_write(bank, buffer, offset, count, 0x2310);
}

static int avr_common_flash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count, uint32_t enter_cmd)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	uint32_t tmp_buf[2];
	bool ext_addressing;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((offset % 2) != 0) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment",
			offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	LOG_DEBUG("offset is 0x%08" PRIx32 "", offset);
	LOG_DEBUG("count is %" PRIu32 "", count);

	if (ERROR_OK != avr_jtagprg_enterprogmode(avr))
		return ERROR_FAIL;

	if (bank->size > 0x20000)
		ext_addressing = true;
	else
		ext_addressing = false;

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, enter_cmd, AVR_JTAG_REG_ProgrammingCommand_Len);

	for (uint32_t i = 0; i < count; i += 2) {
		/* load extended high byte */
		if (ext_addressing)
			avr_jtag_senddat(avr->jtag_info.tap,
					NULL,
					0x0b00 | (((offset + i) >> 17) & 0xFF),
					AVR_JTAG_REG_ProgrammingCommand_Len);

		/* load addr high byte */
		avr_jtag_senddat(avr->jtag_info.tap,
				NULL,
				0x0700 | (((offset + i) >> 9) & 0xFF),
				AVR_JTAG_REG_ProgrammingCommand_Len);

		/* load addr low byte */
		avr_jtag_senddat(avr->jtag_info.tap,
				NULL,
				0x0300 | (((offset + i) >> 1) & 0xFF),
				AVR_JTAG_REG_ProgrammingCommand_Len);

		avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3200, AVR_JTAG_REG_ProgrammingCommand_Len);
		avr_jtag_senddat(avr->jtag_info.tap, &tmp_buf[0], 0x3600, AVR_JTAG_REG_ProgrammingCommand_Len);
		avr_jtag_senddat(avr->jtag_info.tap, &tmp_buf[1], 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;

		buffer[i] = tmp_buf[0];
		buffer[i + 1] = tmp_buf[1];
	}

	return avr_jtagprg_leaveprogmode(avr);
}

static int avrf_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return avr_common_flash_read(bank, buffer, offset, count, 0x2302);
}

#define EXTRACT_MFG(X)  (((X) & 0xffe) >> 1)
#define EXTRACT_PART(X) (((X) & 0xffff000) >> 12)
#define EXTRACT_VER(X)  (((X) & 0xf0000000) >> 28)

static int avrf_identify_chip(struct flash_bank *bank, const struct avrf_type **info, uint8_t *ver)
{
	struct target *target = bank->target;
	struct avrf_flash_bank *avrf_info = bank->driver_priv;
	struct avr_common *avr = target->arch_info;
	uint32_t device_id;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	avrf_info->probed = false;

	avr_jtag_read_jtagid(avr, &device_id);
	if (ERROR_OK != mcu_execute_queue())
		return ERROR_FAIL;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	if (EXTRACT_MFG(device_id) != 0x1F)
		LOG_ERROR("0x%" PRIx32 " is invalid Manufacturer for avr, 0x%X is expected",
			EXTRACT_MFG(device_id),
			0x1F);

	for (size_t i = 0; i < ARRAY_SIZE(avft_chips_info); i++) {
		if (avft_chips_info[i].chip_id == EXTRACT_PART(device_id)) {
			*info = &avft_chips_info[i];
			if (ver != NULL)
				*ver = EXTRACT_VER(device_id);

			LOG_INFO("target device is %s", (*info)->name);
			return ERROR_OK;
		}
	}

	return ERROR_FLASH_OPERATION_FAILED;
}

static int avrf_probe(struct flash_bank *bank)
{
	struct avrf_flash_bank *avrf_info = bank->driver_priv;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, NULL);

	if (error != ERROR_OK) {
		if (error == ERROR_FLASH_OPERATION_FAILED) {
			/* chip not supported */
			LOG_ERROR("device id is not supported for avr");
			avrf_info->probed = true;
			return ERROR_FAIL;
		}

		return error;
	}

	free(bank->sectors);

	/* chip found */
	bank->base = 0x00000000;
	bank->size = (avr_info->flash_page_size * avr_info->flash_page_num);
	bank->num_sectors = avr_info->flash_page_num;
	bank->sectors = malloc(sizeof(struct flash_sector) * avr_info->flash_page_num);

	for (int i = 0; i < avr_info->flash_page_num; i++) {
		bank->sectors[i].offset = i * avr_info->flash_page_size;
		bank->sectors[i].size = avr_info->flash_page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	avrf_info->probed = true;
	return ERROR_OK;
}

static int avrf_auto_probe(struct flash_bank *bank)
{
	struct avrf_flash_bank *avrf_info = bank->driver_priv;
	if (avrf_info->probed)
		return ERROR_OK;
	return avrf_probe(bank);
}

static int avrf_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint8_t ver;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, &ver);

	if (error == ERROR_OK) {
		/* chip found */
		snprintf(buf, buf_size, "%s - Rev: 0x%" PRIx32 "", avr_info->name,
			       ver);
	} else
		if (error == ERROR_FLASH_OPERATION_FAILED)
			snprintf(buf, buf_size, "Cannot identify target as a avr\n");

	return error;
}

static int avrf_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((ERROR_OK != avr_jtagprg_enterprogmode(avr))
	    || (ERROR_OK != avr_jtagprg_chiperase(avr, 0x2380))
	    || (ERROR_OK != avr_jtagprg_leaveprogmode(avr)))
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(avrf_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (avrf_mass_erase(bank) == ERROR_OK) {
		/* set all sectors as erased */
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "avr mass erase complete");
	} else
		command_print(CMD, "avr mass erase failed");

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static const struct command_registration avrf_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.usage = "<bank>",
		.handler = avrf_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "erase entire device",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration avrf_command_handlers[] = {
	{
		.name = "avrf",
		.mode = COMMAND_ANY,
		.help = "AVR flash command group",
		.usage = "",
		.chain = avrf_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver avr_flash = {
	.name = "avr",
	.commands = avrf_command_handlers,
	.flash_bank_command = avrf_flash_bank_command,
	.erase = avrf_erase,
	.write = avrf_write,
	.read = avrf_read,
	.probe = avrf_probe,
	.auto_probe = avrf_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = avrf_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

/*
 * EEPROM
 */
struct avr_eeprom_flash_bank {
	int ppage_size;
	bool probed;
	const struct avrf_type *avr_info;
};

static int avr_eeprom_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	uint32_t cur_size, cur_buffer_size, page_size;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	page_size = bank->sectors[0].size;
	if ((offset % page_size) != 0) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required %" PRIu32 "-byte alignment",
			offset,
			page_size);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	LOG_DEBUG("offset is 0x%08" PRIx32 "", offset);
	LOG_DEBUG("count is %" PRIu32 "", count);

	if (ERROR_OK != avr_jtagprg_enterprogmode(avr))
		return ERROR_FAIL;

	cur_size = 0;
	while (count > 0) {
		if (count > page_size)
			cur_buffer_size = page_size;
		else
			cur_buffer_size = count;
		avr_jtagprg_writeeeprompage(avr,
			buffer + cur_size,
			cur_buffer_size,
			offset + cur_size,
			page_size);
		count -= cur_buffer_size;
		cur_size += cur_buffer_size;

		keep_alive();
	}

	return avr_jtagprg_leaveprogmode(avr);
}

static int avr_eeprom_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	uint32_t tmp_buf;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("offset is 0x%08" PRIx32 "", offset);
	LOG_DEBUG("count is %" PRIu32 "", count);

	if (ERROR_OK != avr_jtagprg_enterprogmode(avr))
		return ERROR_FAIL;

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x2303, AVR_JTAG_REG_ProgrammingCommand_Len);
	for (uint32_t i = 0; i < count; i++) {
		/* load addr high byte */
		avr_jtag_senddat(avr->jtag_info.tap,
				NULL,
				0x0700 | (((offset + i) >> 8) & 0xFF),
				AVR_JTAG_REG_ProgrammingCommand_Len);

		/* load addr low byte */
		avr_jtag_senddat(avr->jtag_info.tap,
				NULL,
				0x0300 | ((offset + i) & 0xFF),
				AVR_JTAG_REG_ProgrammingCommand_Len);

		/* load addr low byte again for some reason */
		avr_jtag_senddat(avr->jtag_info.tap,
				NULL,
				0x3300 | ((offset + i) & 0xFF),
				AVR_JTAG_REG_ProgrammingCommand_Len);

		avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3200, AVR_JTAG_REG_ProgrammingCommand_Len);
		avr_jtag_senddat(avr->jtag_info.tap, &tmp_buf, 0x3300, AVR_JTAG_REG_ProgrammingCommand_Len);
		if (ERROR_OK != mcu_execute_queue())
			return ERROR_FAIL;

		buffer[i] = tmp_buf;
	}

	return avr_jtagprg_leaveprogmode(avr);
}

static int avr_eeprom_probe(struct flash_bank *bank)
{
	struct avr_eeprom_flash_bank *avr_eeprom_bank = bank->driver_priv;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, NULL);

	if (error != ERROR_OK) {
		if (error == ERROR_FLASH_OPERATION_FAILED) {
			/* chip not supported */
			LOG_ERROR("device id is not supported for avr");
			avr_eeprom_bank->probed = true;
			return ERROR_FAIL;
		}

		return error;
	}

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* chip found */
	bank->base = 0x00000000;
	bank->size = (avr_info->eeprom_page_size * avr_info->eeprom_page_num);
	bank->num_sectors = avr_info->eeprom_page_num;
	bank->sectors = malloc(sizeof(struct flash_sector) * avr_info->eeprom_page_num);

	for (int i = 0; i < avr_info->eeprom_page_num; i++) {
		bank->sectors[i].offset = i * avr_info->eeprom_page_size;
		bank->sectors[i].size = avr_info->eeprom_page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	avr_eeprom_bank->probed = true;
	avr_eeprom_bank->avr_info = avr_info;
	return ERROR_OK;
}

static int avr_eeprom_auto_probe(struct flash_bank *bank)
{
	struct avr_eeprom_flash_bank *avr_eeprom_bank = bank->driver_priv;

	if (avr_eeprom_bank->probed)
		return ERROR_OK;

	return avr_eeprom_probe(bank);
}

static int avr_eeprom_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint8_t ver;
	struct avr_eeprom_flash_bank *avr_eeprom_bank = bank->driver_priv;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, &ver);

	if (error == ERROR_OK) {
		/* chip found */
		snprintf(buf, buf_size, "%s - Rev: 0x%" PRIx32 "", avr_info->name,
				ver);
		avr_eeprom_bank->avr_info = avr_info;
	} else
		if (error == ERROR_FLASH_OPERATION_FAILED)
			snprintf(buf, buf_size, "Cannot identify target as a avr\n");

	return error;
}

FLASH_BANK_COMMAND_HANDLER(avr_eeprom_flash_bank_command)
{
	struct avr_eeprom_flash_bank *avr_eeprom_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	avr_eeprom_bank = malloc(sizeof(struct avr_eeprom_flash_bank));
	if (!avr_eeprom_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	avr_eeprom_bank->probed = false;
	bank->driver_priv = avr_eeprom_bank;

	return ERROR_OK;
}

static const struct command_registration avr_eeprom_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration avr_eeprom_command_handlers[] = {
	{
		.name = "avr_eeprom",
		.mode = COMMAND_ANY,
		.help = "AVR EEPROM command group",
		.usage = "",
		.chain = avr_eeprom_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver avr_eeprom = {
	.name = "avr_eeprom",
	.commands = avr_eeprom_command_handlers,
	.flash_bank_command = avr_eeprom_flash_bank_command,
	.info = avr_eeprom_info,
	.probe = avr_eeprom_probe,
	.auto_probe = avr_eeprom_auto_probe,
	.read = avr_eeprom_read,
	.erase = avrf_erase,
	.erase_check = default_flash_blank_check,
	.write = avr_eeprom_write,
	.free_driver_priv = default_flash_free_driver_priv,
};

/*
 * Fuses
 */
struct avr_fuses_flash_bank {
	int ppage_size;
	int probed;
	const struct avrf_type *avr_info;
};

static int avr_fuses_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	uint32_t cur_size, cur_buffer_size, page_size;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	page_size = bank->sectors[0].size;
	if ((offset % page_size) != 0) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required %" PRIu32 "-byte alignment",
			offset,
			page_size);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	LOG_DEBUG("offset is 0x%08" PRIx32 "", offset);
	LOG_DEBUG("count is %" PRIu32 "", count);

	if (ERROR_OK != avr_jtagprg_enterprogmode(avr))
		return ERROR_FAIL;

	cur_size = 0;
	while (count > 0) {
		if (count > page_size)
			cur_buffer_size = page_size;
		else
			cur_buffer_size = count;
		avr_jtagprg_writefuses(avr,
			buffer + cur_size,
			cur_buffer_size,
			offset + cur_size,
			page_size);
		count -= cur_buffer_size;
		cur_size += cur_buffer_size;

		keep_alive();
	}

	return avr_jtagprg_leaveprogmode(avr);
}

static int avr_fuses_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct avr_common *avr = target->arch_info;
	uint32_t tmp_buf[4];

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("offset is 0x%08" PRIx32 "", offset);
	LOG_DEBUG("count is %" PRIu32 "", count);

	if (ERROR_OK != avr_jtagprg_enterprogmode(avr))
		return ERROR_FAIL;

	avr_jtag_sendinstr(avr->jtag_info.tap, NULL, AVR_JTAG_INS_PROG_COMMANDS);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x2304, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, NULL, 0x3a00, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, &tmp_buf[0], 0x3e00, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, &tmp_buf[1], 0x3200, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, &tmp_buf[2], 0x3600, AVR_JTAG_REG_ProgrammingCommand_Len);
	avr_jtag_senddat(avr->jtag_info.tap, &tmp_buf[3], 0x3700, AVR_JTAG_REG_ProgrammingCommand_Len);

	if (ERROR_OK != mcu_execute_queue())
		return ERROR_FAIL;

	for (unsigned int i = 0; i < count; i++) {
		buffer[i] = tmp_buf[i + offset];
	}

	return avr_jtagprg_leaveprogmode(avr);
}

static int avr_fuses_probe(struct flash_bank *bank)
{
	struct avr_fuses_flash_bank *avr_fuses_bank = bank->driver_priv;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, NULL);

	if (error != ERROR_OK) {
		if (error == ERROR_FLASH_OPERATION_FAILED) {
			/* chip not supported */
			LOG_ERROR("device id is not supported for avr");
			avr_fuses_bank->probed = true;
			return ERROR_FAIL;
		}
		return error;
	}

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* chip found */
	bank->base = 0x00000000;
	bank->size = 3;
	bank->num_sectors = 1;
	bank->sectors = malloc(sizeof(struct flash_sector));

	bank->sectors[0].offset = 0;
	bank->sectors[0].size = 3;
	bank->sectors[0].is_erased = -1;
	bank->sectors[0].is_protected = -1;

	avr_fuses_bank->probed = true;
	avr_fuses_bank->avr_info = avr_info;
	return ERROR_OK;
}

static int avr_fuses_auto_probe(struct flash_bank *bank)
{
	struct avr_fuses_flash_bank *avr_fuses_bank = bank->driver_priv;

	if (avr_fuses_bank->probed)
		return ERROR_OK;

	return avr_fuses_probe(bank);
}

static int avr_fuses_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint8_t ver;
	struct avr_fuses_flash_bank *avr_fuses_bank = bank->driver_priv;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, &ver);

	if (error == ERROR_OK) {
		/* chip found */
		snprintf(buf, buf_size, "%s - Rev: 0x%" PRIx32 "", avr_info->name,
				ver);
		avr_fuses_bank->avr_info = avr_info;
	} else
		if (error == ERROR_FLASH_OPERATION_FAILED)
			snprintf(buf, buf_size, "Cannot identify target as a avr\n");

	return error;
}

FLASH_BANK_COMMAND_HANDLER(avr_fuses_flash_bank_command)
{
	struct avr_fuses_flash_bank *avr_fuses_bank;

	avr_fuses_bank = malloc(sizeof(struct avr_fuses_flash_bank));
	if (!avr_fuses_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	avr_fuses_bank->probed = false;
	bank->driver_priv = avr_fuses_bank;

	return ERROR_OK;
}

static const struct command_registration avr_fuses_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration avr_fuses_command_handlers[] = {
	{
		.name = "avr_fuses",
		.mode = COMMAND_ANY,
		.help = "AVR fuses command group",
		.usage = "",
		.chain = avr_fuses_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver avr_fuses = {
	.name = "avr_fuses",
	.commands = avr_fuses_command_handlers,
	.flash_bank_command = avr_fuses_flash_bank_command,
	.info = avr_fuses_info,
	.probe = avr_fuses_probe,
	.auto_probe = avr_fuses_auto_probe,
	.read = avr_fuses_read,
	.erase = avrf_erase,
	.erase_check = default_flash_blank_check,
	.write = avr_fuses_write,
	.free_driver_priv = default_flash_free_driver_priv,
};

/*
 * User Signature data
 */
struct avr_user_signature_flash_bank {
	int ppage_size;
	int probed;
	const struct avrf_type *avr_info;
};

static int avr_user_signature_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	return avrf_common_erase(bank, first, last, 0x2384);
}

static int avr_user_signature_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return avrf_common_write(bank, buffer, offset, count, 0x2312);
}

static int avr_user_signature_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return avr_common_flash_read(bank, buffer, offset, count, 0x2308);
}

static int avr_user_signature_probe(struct flash_bank *bank)
{
	uint8_t ver;
	struct avrf_flash_bank *avrf_info = bank->driver_priv;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, &ver);

	if ((error == ERROR_OK) && (avr_info->user_signature_page_num > 0)) {
		if (bank->sectors) {
			free(bank->sectors);
			bank->sectors = NULL;
		}

		/* chip found */
		bank->base = 0x00000000;
		bank->size = (avr_info->flash_page_size * avr_info->user_signature_page_num);
		bank->num_sectors = avr_info->user_signature_page_num;
		bank->sectors = malloc(sizeof(struct flash_sector) * avr_info->user_signature_page_num);

		for (int i = 0; i < avr_info->user_signature_page_num; i++) {
			bank->sectors[i].offset = i * avr_info->flash_page_size;
			bank->sectors[i].size = avr_info->flash_page_size;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = -1;
		}

		avrf_info->probed = true;
		return ERROR_OK;
	}

	avrf_info->probed = true;
	if (error == ERROR_FLASH_OPERATION_FAILED) {
		/* chip not supported */
		LOG_ERROR("device id is not supported for avr");
		return ERROR_FAIL;
	}

	if (avr_info->user_signature_page_num == 0) {
		LOG_ERROR("This chip does not have user signature pages.");
	}

	return error;
}

static int avr_user_signature_auto_probe(struct flash_bank *bank)
{
	struct avrf_flash_bank *avrf_info = bank->driver_priv;
	if (avrf_info->probed)
		return ERROR_OK;
	return avr_user_signature_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(avr_user_signature_flash_bank_command)
{
	struct avr_user_signature_flash_bank *avr_user_signature_bank;

	avr_user_signature_bank = malloc(sizeof(struct avr_user_signature_flash_bank));
	if (!avr_user_signature_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	avr_user_signature_bank->probed = false;
	bank->driver_priv = avr_user_signature_bank;

	return ERROR_OK;
}

static int avr_user_signature_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint8_t ver;
	struct avr_user_signature_flash_bank *avr_user_signature_bank = bank->driver_priv;
	const struct avrf_type *avr_info;
	int error = avrf_identify_chip(bank, &avr_info, &ver);

	if (error == ERROR_OK) {
		/* chip found */
		snprintf(buf, buf_size, "%s - Rev: 0x%" PRIx32 "", avr_info->name,
				ver);
		avr_user_signature_bank->avr_info = avr_info;
	} else
		if (error == ERROR_FLASH_OPERATION_FAILED)
			snprintf(buf, buf_size, "Cannot identify target as a avr\n");

	return error;
}

static const struct command_registration avr_user_signature_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration avr_user_signature_command_handlers[] = {
	{
		.name = "avr_user_signature",
		.mode = COMMAND_ANY,
		.help = "AVR flash command group",
		.usage = "",
		.chain = avr_user_signature_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver avr_user_signature = {
	.name = "avr_user_signature",
	.commands = avr_user_signature_command_handlers,
	.flash_bank_command = avr_user_signature_flash_bank_command,
	.erase = avr_user_signature_erase,
	.write = avr_user_signature_write,
	.read = avr_user_signature_read,
	.probe = avr_user_signature_probe,
	.auto_probe = avr_user_signature_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = avr_user_signature_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
