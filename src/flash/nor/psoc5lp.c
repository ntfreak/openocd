/*
 * PSoC 5LP flash driver
 *
 * Copyright (c) 2016 Andreas Färber
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/time_support.h>
#include <target/armv7m.h>

#define PM_ACT_CFG0             0x400043A0
#define PM_ACT_CFG12            0x400043AC
#define SPC_CPU_DATA            0x40004720
#define SPC_SR                  0x40004722
#define PRT1_PC2                0x4000500A
#define PHUB_CH0_BASIC_CFG      0x40007010
#define PHUB_CH0_ACTION         0x40007014
#define PHUB_CH0_BASIC_STATUS   0x40007018
#define PHUB_CH1_BASIC_CFG      0x40007020
#define PHUB_CH1_ACTION         0x40007024
#define PHUB_CH1_BASIC_STATUS   0x40007028
#define PHUB_CFGMEM0_CFG0       0x40007600
#define PHUB_CFGMEM0_CFG1       0x40007604
#define PHUB_CFGMEM1_CFG0       0x40007608
#define PHUB_CFGMEM1_CFG1       0x4000760C
#define PHUB_TDMEM0_ORIG_TD0    0x40007800
#define PHUB_TDMEM0_ORIG_TD1    0x40007804
#define PHUB_TDMEM1_ORIG_TD0    0x40007808
#define PHUB_TDMEM1_ORIG_TD1    0x4000780C
#define PANTHER_DEVICE_ID       0x4008001C

/* NVL is not actually mapped to the Cortex-M address space
 * As we need a base address different from other banks in the device
 * we use the address of NVL programming data in Cypress images */
#define NVL_META_BASE			0x90000000

#define PM_ACT_CFG12_EN_EE (1 << 4)

#define SPC_KEY1 0xB6
#define SPC_KEY2 0xD3

#define SPC_LOAD_BYTE           0x00
#define SPC_LOAD_MULTI_BYTE     0x01
#define SPC_LOAD_ROW            0x02
#define SPC_READ_BYTE           0x03
#define SPC_READ_MULTI_BYTE     0x04
#define SPC_WRITE_ROW           0x05
#define SPC_WRITE_USER_NVL      0x06
#define SPC_PRG_ROW             0x07
#define SPC_ERASE_SECTOR        0x08
#define SPC_ERASE_ALL           0x09
#define SPC_READ_HIDDEN_ROW     0x0A
#define SPC_PROGRAM_PROTECT_ROW 0x0B
#define SPC_GET_CHECKSUM        0x0C
#define SPC_GET_TEMP            0x0E
#define SPC_READ_VOLATILE_BYTE  0x10

#define SPC_ARRAY_ALL      0x3F
#define SPC_ARRAY_EEPROM   0x40
#define SPC_ARRAY_NVL_USER 0x80
#define SPC_ARRAY_NVL_WO   0xF8

#define SPC_ROW_PROTECTION 0

#define SPC_OPCODE_LEN 3

#define SPC_SR_DATA_READY (1 << 0)
#define SPC_SR_IDLE       (1 << 1)

#define PM_ACT_CFG0_EN_CLK_SPC      (1 << 3)

#define PHUB_CHX_BASIC_CFG_EN       (1 << 0)
#define PHUB_CHX_BASIC_CFG_WORK_SEP (1 << 5)

#define PHUB_CHX_ACTION_CPU_REQ (1 << 0)

#define PHUB_CFGMEMX_CFG0 (1 << 7)

#define PHUB_TDMEMX_ORIG_TD0_NEXT_TD_PTR_LAST (0xff << 16)
#define PHUB_TDMEMX_ORIG_TD0_INC_SRC_ADDR     (1 << 24)

#define NVL_3_ECCEN  (1 << 3)

#define ROW_SIZE           256
#define ROW_ECC_SIZE       32
#define ROWS_PER_SECTOR    64
#define SECTOR_SIZE        (ROWS_PER_SECTOR * ROW_SIZE)
#define ROWS_PER_BLOCK     256
#define BLOCK_SIZE         (ROWS_PER_BLOCK * ROW_SIZE)
#define SECTORS_PER_BLOCK  (BLOCK_SIZE / SECTOR_SIZE)
#define EEPROM_ROW_SIZE    16
#define EEPROM_SECTOR_SIZE (ROWS_PER_SECTOR * EEPROM_ROW_SIZE)
#define EEPROM_BLOCK_SIZE  (ROWS_PER_BLOCK * EEPROM_ROW_SIZE)

#define PART_NUMBER_LEN (17 + 1)

struct psoc5lp_device {
	uint32_t id;
	unsigned fam;
	unsigned speed_mhz;
	unsigned flash_kb;
	unsigned eeprom_kb;
};

/*
 * Device information collected from datasheets.
 * Different temperature ranges (C/I/Q/A) may share IDs, not differing otherwise.
 */
static const struct psoc5lp_device psoc5lp_devices[] = {
	/* CY8C58LP Family Datasheet */
	{ .id = 0x2E11F069, .fam = 8, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E120069, .fam = 8, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E123069, .fam = 8, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E124069, .fam = 8, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E126069, .fam = 8, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E127069, .fam = 8, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E117069, .fam = 8, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E118069, .fam = 8, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E119069, .fam = 8, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E11C069, .fam = 8, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E114069, .fam = 8, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E115069, .fam = 8, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E116069, .fam = 8, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E160069, .fam = 8, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	/*           ''                                                               */
	{ .id = 0x2E161069, .fam = 8, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	/*           ''                                                               */
	{ .id = 0x2E1D2069, .fam = 8, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E1D6069, .fam = 8, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },

	/* CY8C56LP Family Datasheet */
	{ .id = 0x2E10A069, .fam = 6, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E10D069, .fam = 6, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E10E069, .fam = 6, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E106069, .fam = 6, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E108069, .fam = 6, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E109069, .fam = 6, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E101069, .fam = 6, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E104069, .fam = 6, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	/*           ''                                                               */
	{ .id = 0x2E105069, .fam = 6, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E128069, .fam = 6, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	/*           ''                                                               */
	{ .id = 0x2E122069, .fam = 6, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E129069, .fam = 6, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E163069, .fam = 6, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E156069, .fam = 6, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E1D3069, .fam = 6, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },

	/* CY8C54LP Family Datasheet */
	{ .id = 0x2E11A069, .fam = 4, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E16A069, .fam = 4, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E12A069, .fam = 4, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E103069, .fam = 4, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E16C069, .fam = 4, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E102069, .fam = 4, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E148069, .fam = 4, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E155069, .fam = 4, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E16B069, .fam = 4, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E12B069, .fam = 4, .speed_mhz = 67, .flash_kb =  32, .eeprom_kb = 2 },
	{ .id = 0x2E168069, .fam = 4, .speed_mhz = 67, .flash_kb =  32, .eeprom_kb = 2 },
	{ .id = 0x2E178069, .fam = 4, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E15D069, .fam = 4, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E1D4069, .fam = 4, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },

	/* CY8C52LP Family Datasheet */
	{ .id = 0x2E11E069, .fam = 2, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E12F069, .fam = 2, .speed_mhz = 67, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E133069, .fam = 2, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E159069, .fam = 2, .speed_mhz = 67, .flash_kb = 128, .eeprom_kb = 2 },
	{ .id = 0x2E11D069, .fam = 2, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E121069, .fam = 2, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E184069, .fam = 2, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E196069, .fam = 2, .speed_mhz = 67, .flash_kb =  64, .eeprom_kb = 2 },
	{ .id = 0x2E132069, .fam = 2, .speed_mhz = 67, .flash_kb =  32, .eeprom_kb = 2 },
	{ .id = 0x2E138069, .fam = 2, .speed_mhz = 67, .flash_kb =  32, .eeprom_kb = 2 },
	{ .id = 0x2E13A069, .fam = 2, .speed_mhz = 67, .flash_kb =  32, .eeprom_kb = 2 },
	{ .id = 0x2E152069, .fam = 2, .speed_mhz = 67, .flash_kb =  32, .eeprom_kb = 2 },
	{ .id = 0x2E15F069, .fam = 2, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E15A069, .fam = 2, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
	{ .id = 0x2E1D5069, .fam = 2, .speed_mhz = 80, .flash_kb = 256, .eeprom_kb = 2 },
};

static void psoc5lp_get_part_number(const struct psoc5lp_device *dev, char *str)
{
	strcpy(str, "CY8Cabcdefg-LPxxx");

	str[4] = '5';
	str[5] = '0' + dev->fam;

	switch (dev->speed_mhz) {
	case 67:
		str[6] = '6';
		break;
	case 80:
		str[6] = '8';
		break;
	default:
		str[6] = '?';
	}

	switch (dev->flash_kb) {
	case 32:
		str[7] = '5';
		break;
	case 64:
		str[7] = '6';
		break;
	case 128:
		str[7] = '7';
		break;
	case 256:
		str[7] = '8';
		break;
	default:
		str[7] = '?';
	}

	/* Package does not matter. */
	str[8] = 'x';
	str[9] = 'x';

	/* Temperate range cannot uniquely be identified. */
	str[10] = 'x';
}

static int psoc5lp_get_device_id(struct target *target, uint32_t *id)
{
	int retval;

	retval = target_read_u32(target, PANTHER_DEVICE_ID, id); /* dummy read */
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u32(target, PANTHER_DEVICE_ID, id);
	return retval;
}

static int psoc5lp_find_device(struct target *target,
	const struct psoc5lp_device **device)
{
	uint32_t device_id;
	unsigned i;
	int retval;

	*device = NULL;

	retval = psoc5lp_get_device_id(target, &device_id);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("PANTHER_DEVICE_ID = 0x%08" PRIX32, device_id);

	for (i = 0; i < ARRAY_SIZE(psoc5lp_devices); i++) {
		if (psoc5lp_devices[i].id == device_id) {
			*device = &psoc5lp_devices[i];
			return ERROR_OK;
		}
	}

	LOG_ERROR("Device 0x%08" PRIX32 " not supported", device_id);
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int psoc5lp_spc_enable_clock(struct target *target)
{
	int retval;
	uint8_t pm_act_cfg0;

	retval = target_read_u8(target, PM_ACT_CFG0, &pm_act_cfg0);
	if (retval != ERROR_OK) {
		LOG_ERROR("Cannot read PM_ACT_CFG0");
		return retval;
	}

	if (pm_act_cfg0 & PM_ACT_CFG0_EN_CLK_SPC)
		return ERROR_OK;	/* clock already enabled */

	retval = target_write_u8(target, PM_ACT_CFG0, pm_act_cfg0 | PM_ACT_CFG0_EN_CLK_SPC);
	if (retval != ERROR_OK)
		LOG_ERROR("Cannot enable SPC clock");

	return retval;
}

static int psoc5lp_spc_write_opcode(struct target *target, uint8_t opcode)
{
	int retval;

	retval = target_write_u8(target, SPC_CPU_DATA, SPC_KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, SPC_KEY2 + opcode);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, opcode);
	return retval;
}

static void psoc5lp_spc_write_opcode_buffer(struct target *target,
	uint8_t *buf, uint8_t opcode)
{
	buf[0] = SPC_KEY1;
	buf[1] = SPC_KEY2 + opcode;
	buf[2] = opcode;
}

static int psoc5lp_spc_busy_wait_data(struct target *target)
{
	int64_t endtime;
	uint8_t sr;
	int retval;

	retval = target_read_u8(target, SPC_SR, &sr); /* dummy read */
	if (retval != ERROR_OK)
		return retval;

	endtime = timeval_ms() + 1000; /* 1 second timeout */
	do {
		alive_sleep(1);
		retval = target_read_u8(target, SPC_SR, &sr);
		if (retval != ERROR_OK)
			return retval;
		if (sr == SPC_SR_DATA_READY)
			return ERROR_OK;
	} while (timeval_ms() < endtime);

	return ERROR_FLASH_OPERATION_FAILED;
}

static int psoc5lp_spc_busy_wait_idle(struct target *target)
{
	int64_t endtime;
	uint8_t sr;
	int retval;

	retval = target_read_u8(target, SPC_SR, &sr); /* dummy read */
	if (retval != ERROR_OK)
		return retval;

	endtime = timeval_ms() + 1000; /* 1 second timeout */
	do {
		alive_sleep(1);
		retval = target_read_u8(target, SPC_SR, &sr);
		if (retval != ERROR_OK)
			return retval;
		if (sr == SPC_SR_IDLE)
			return ERROR_OK;
	} while (timeval_ms() < endtime);

	return ERROR_FLASH_OPERATION_FAILED;
}

static int psoc5lp_spc_load_byte(struct target *target,
	uint8_t array_id, uint8_t offset, uint8_t value)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_LOAD_BYTE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, offset);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, value);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_load_row(struct target *target,
	uint8_t array_id, const uint8_t *data, unsigned row_size)
{
	unsigned i;
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_LOAD_ROW);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < row_size; i++) {
		retval = target_write_u8(target, SPC_CPU_DATA, data[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_read_byte(struct target *target,
	uint8_t array_id, uint8_t offset, uint8_t *data)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_READ_BYTE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, offset);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_data(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(target, SPC_CPU_DATA, data);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_write_row(struct target *target,
	uint8_t array_id, uint16_t row_id, const uint8_t *temp)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_WRITE_ROW);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, row_id >> 8);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, row_id & 0xff);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, temp[0]);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, temp[1]);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_write_user_nvl(struct target *target,
	uint8_t array_id)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_WRITE_USER_NVL);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_erase_sector(struct target *target,
	uint8_t array_id, uint8_t row_id)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_ERASE_SECTOR);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, row_id);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_erase_all(struct target *target)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_ERASE_ALL);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_read_hidden_row(struct target *target,
	uint8_t array_id, uint8_t row_id, uint8_t *data)
{
	int i, retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_READ_HIDDEN_ROW);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, row_id);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_data(target);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < ROW_SIZE; i++) {
		retval = target_read_u8(target, SPC_CPU_DATA, &data[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_get_temp(struct target *target, uint8_t samples,
	uint8_t *data)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_GET_TEMP);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, samples);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_data(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(target, SPC_CPU_DATA, &data[0]);
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u8(target, SPC_CPU_DATA, &data[1]);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int psoc5lp_spc_read_volatile_byte(struct target *target,
	uint8_t array_id, uint8_t offset, uint8_t *data)
{
	int retval;

	retval = psoc5lp_spc_write_opcode(target, SPC_READ_VOLATILE_BYTE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, array_id);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u8(target, SPC_CPU_DATA, offset);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_data(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(target, SPC_CPU_DATA, data);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_busy_wait_idle(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/*
 * NV Latch
 */

struct psoc5lp_nvl_flash_bank {
	bool probed;
	const struct psoc5lp_device *device;
};

static int psoc5lp_nvl_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	retval = psoc5lp_spc_enable_clock(bank->target);
	if (retval != ERROR_OK)
		return retval;

	while (count > 0) {
		retval = psoc5lp_spc_read_byte(bank->target,
				SPC_ARRAY_NVL_USER, offset, buffer);
		if (retval != ERROR_OK)
			return retval;
		buffer++;
		offset++;
		count--;
	}

	return ERROR_OK;
}

static int psoc5lp_nvl_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	LOG_WARNING("There is no erase operation for NV Latches");
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int psoc5lp_nvl_erase_check(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 0;

	return ERROR_OK;
}

static int psoc5lp_nvl_write(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t byte_count)
{
	struct target *target = bank->target;
	uint8_t *current_data, val;
	bool write_required = false, pullup_needed = false, ecc_changed = false;
	uint32_t i;
	int retval;

	if (offset != 0 || byte_count != bank->size) {
		LOG_ERROR("NVL can only be written in whole");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	current_data = calloc(1, bank->size);
	if (!current_data)
		return ERROR_FAIL;
	retval = psoc5lp_nvl_read(bank, current_data, offset, byte_count);
	if (retval != ERROR_OK) {
		free(current_data);
		return retval;
	}
	for (i = offset; i < byte_count; i++) {
		if (current_data[i] != buffer[i]) {
			write_required = true;
			break;
		}
	}
	if (((buffer[2] & 0x80) == 0x80) && ((current_data[0] & 0x0C) != 0x08))
		pullup_needed = true;
	if (((buffer[3] ^ current_data[3]) & 0x08) == 0x08)
		ecc_changed = true;
	free(current_data);

	if (!write_required) {
		LOG_INFO("Unchanged, skipping NVL write");
		return ERROR_OK;
	}
	if (pullup_needed) {
		retval = target_read_u8(target, PRT1_PC2, &val);
		if (retval != ERROR_OK)
			return retval;
		val &= 0xF0;
		val |= 0x05;
		retval = target_write_u8(target, PRT1_PC2, val);
		if (retval != ERROR_OK)
			return retval;
	}

	for (i = offset; i < byte_count; i++) {
		retval = psoc5lp_spc_load_byte(target,
				SPC_ARRAY_NVL_USER, i, buffer[i]);
		if (retval != ERROR_OK)
			return retval;

		retval = psoc5lp_spc_read_volatile_byte(target,
				SPC_ARRAY_NVL_USER, i, &val);
		if (retval != ERROR_OK)
			return retval;
		if (val != buffer[i]) {
			LOG_ERROR("Failed to load NVL byte %" PRIu32 ": "
				"expected 0x%02" PRIx8 ", read 0x%02" PRIx8,
				i, buffer[i], val);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	retval = psoc5lp_spc_write_user_nvl(target, SPC_ARRAY_NVL_USER);
	if (retval != ERROR_OK)
		return retval;

	if (ecc_changed) {
		retval = target_call_reset_callbacks(target, RESET_INIT);
		if (retval != ERROR_OK)
			LOG_WARNING("Reset failed after enabling or disabling ECC");
	}

	return ERROR_OK;
}

static int psoc5lp_nvl_get_info_command(struct flash_bank *bank,
	struct command_invocation *cmd)
{
	struct psoc5lp_nvl_flash_bank *psoc_nvl_bank = bank->driver_priv;
	char part_number[PART_NUMBER_LEN];

	psoc5lp_get_part_number(psoc_nvl_bank->device, part_number);

	command_print_sameline(cmd, "%s", part_number);

	return ERROR_OK;
}

static int psoc5lp_nvl_probe(struct flash_bank *bank)
{
	struct psoc5lp_nvl_flash_bank *psoc_nvl_bank = bank->driver_priv;
	int retval;

	if (psoc_nvl_bank->probed)
		return ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = psoc5lp_find_device(bank->target, &psoc_nvl_bank->device);
	if (retval != ERROR_OK)
		return retval;

	bank->base = NVL_META_BASE;
	bank->size = 4;
	bank->num_sectors = 1;
	bank->sectors = calloc(bank->num_sectors,
			       sizeof(struct flash_sector));
	bank->sectors[0].offset = 0;
	bank->sectors[0].size = 4;
	bank->sectors[0].is_erased = -1;
	bank->sectors[0].is_protected = -1;

	psoc_nvl_bank->probed = true;

	return ERROR_OK;
}

static int psoc5lp_nvl_auto_probe(struct flash_bank *bank)
{
	struct psoc5lp_nvl_flash_bank *psoc_nvl_bank = bank->driver_priv;

	if (psoc_nvl_bank->probed)
		return ERROR_OK;

	return psoc5lp_nvl_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(psoc5lp_nvl_flash_bank_command)
{
	struct psoc5lp_nvl_flash_bank *psoc_nvl_bank;

	psoc_nvl_bank = malloc(sizeof(struct psoc5lp_nvl_flash_bank));
	if (!psoc_nvl_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	psoc_nvl_bank->probed = false;

	bank->driver_priv = psoc_nvl_bank;

	return ERROR_OK;
}

static const struct command_registration psoc5lp_nvl_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc5lp_nvl_command_handlers[] = {
	{
		.name = "psoc5lp_nvl",
		.mode = COMMAND_ANY,
		.help = "PSoC 5LP NV Latch command group",
		.usage = "",
		.chain = psoc5lp_nvl_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver psoc5lp_nvl_flash = {
	.name = "psoc5lp_nvl",
	.commands = psoc5lp_nvl_command_handlers,
	.flash_bank_command = psoc5lp_nvl_flash_bank_command,
	.info = psoc5lp_nvl_get_info_command,
	.probe = psoc5lp_nvl_probe,
	.auto_probe = psoc5lp_nvl_auto_probe,
	.read = psoc5lp_nvl_read,
	.erase = psoc5lp_nvl_erase,
	.erase_check = psoc5lp_nvl_erase_check,
	.write = psoc5lp_nvl_write,
	.free_driver_priv = default_flash_free_driver_priv,
};

/*
 * EEPROM
 */

struct psoc5lp_eeprom_flash_bank {
	bool probed;
	const struct psoc5lp_device *device;
};

static int psoc5lp_eeprom_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int retval;

	for (unsigned int i = first; i <= last; i++) {
		retval = psoc5lp_spc_erase_sector(bank->target,
				SPC_ARRAY_EEPROM, i);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int psoc5lp_eeprom_write(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t byte_count)
{
	struct target *target = bank->target;
	uint8_t temp[2];
	unsigned row;
	int retval;

	if (offset % EEPROM_ROW_SIZE != 0) {
		LOG_ERROR("Writes must be row-aligned, got offset 0x%08" PRIx32,
			offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = psoc5lp_spc_get_temp(target, 3, temp);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to read Die temperature");
		return retval;
	}
	LOG_DEBUG("Get_Temp: sign 0x%02" PRIx8 ", magnitude 0x%02" PRIx8,
		temp[0], temp[1]);

	for (row = offset / EEPROM_ROW_SIZE; byte_count >= EEPROM_ROW_SIZE; row++) {
		retval = psoc5lp_spc_load_row(target, SPC_ARRAY_EEPROM,
				buffer, EEPROM_ROW_SIZE);
		if (retval != ERROR_OK)
			return retval;

		retval = psoc5lp_spc_write_row(target, SPC_ARRAY_EEPROM,
				row, temp);
		if (retval != ERROR_OK)
			return retval;

		buffer += EEPROM_ROW_SIZE;
		byte_count -= EEPROM_ROW_SIZE;
		offset += EEPROM_ROW_SIZE;
	}
	if (byte_count > 0) {
		uint8_t buf[EEPROM_ROW_SIZE];

		memcpy(buf, buffer, byte_count);
		memset(buf + byte_count, bank->default_padded_value,
				EEPROM_ROW_SIZE - byte_count);

		LOG_DEBUG("Padding %" PRIu32 " bytes", EEPROM_ROW_SIZE - byte_count);
		retval = psoc5lp_spc_load_row(target, SPC_ARRAY_EEPROM,
				buf, EEPROM_ROW_SIZE);
		if (retval != ERROR_OK)
			return retval;

		retval = psoc5lp_spc_write_row(target, SPC_ARRAY_EEPROM,
				row, temp);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int psoc5lp_eeprom_get_info_command(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct psoc5lp_eeprom_flash_bank *psoc_eeprom_bank = bank->driver_priv;
	char part_number[PART_NUMBER_LEN];

	psoc5lp_get_part_number(psoc_eeprom_bank->device, part_number);

	command_print_sameline(cmd, "%s", part_number);

	return ERROR_OK;
}

static int psoc5lp_eeprom_probe(struct flash_bank *bank)
{
	struct psoc5lp_eeprom_flash_bank *psoc_eeprom_bank = bank->driver_priv;
	uint32_t flash_addr = bank->base;
	uint32_t val;
	int retval;

	if (psoc_eeprom_bank->probed)
		return ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = psoc5lp_find_device(bank->target, &psoc_eeprom_bank->device);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(bank->target, PM_ACT_CFG12, &val);
	if (retval != ERROR_OK)
		return retval;
	if (!(val & PM_ACT_CFG12_EN_EE)) {
		val |= PM_ACT_CFG12_EN_EE;
		retval = target_write_u32(bank->target, PM_ACT_CFG12, val);
		if (retval != ERROR_OK)
			return retval;
	}

	bank->size = psoc_eeprom_bank->device->eeprom_kb * 1024;
	bank->num_sectors = DIV_ROUND_UP(bank->size, EEPROM_SECTOR_SIZE);
	bank->sectors = calloc(bank->num_sectors,
			       sizeof(struct flash_sector));
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = EEPROM_SECTOR_SIZE;
		bank->sectors[i].offset = flash_addr - bank->base;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;

		flash_addr += bank->sectors[i].size;
	}

	bank->default_padded_value = bank->erased_value = 0x00;

	psoc_eeprom_bank->probed = true;

	return ERROR_OK;
}

static int psoc5lp_eeprom_auto_probe(struct flash_bank *bank)
{
	struct psoc5lp_eeprom_flash_bank *psoc_eeprom_bank = bank->driver_priv;

	if (psoc_eeprom_bank->probed)
		return ERROR_OK;

	return psoc5lp_eeprom_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(psoc5lp_eeprom_flash_bank_command)
{
	struct psoc5lp_eeprom_flash_bank *psoc_eeprom_bank;

	psoc_eeprom_bank = malloc(sizeof(struct psoc5lp_eeprom_flash_bank));
	if (!psoc_eeprom_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	psoc_eeprom_bank->probed = false;
	psoc_eeprom_bank->device = NULL;

	bank->driver_priv = psoc_eeprom_bank;

	return ERROR_OK;
}

static const struct command_registration psoc5lp_eeprom_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc5lp_eeprom_command_handlers[] = {
	{
		.name = "psoc5lp_eeprom",
		.mode = COMMAND_ANY,
		.help = "PSoC 5LP EEPROM command group",
		.usage = "",
		.chain = psoc5lp_eeprom_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver psoc5lp_eeprom_flash = {
	.name = "psoc5lp_eeprom",
	.commands = psoc5lp_eeprom_command_handlers,
	.flash_bank_command = psoc5lp_eeprom_flash_bank_command,
	.info = psoc5lp_eeprom_get_info_command,
	.probe = psoc5lp_eeprom_probe,
	.auto_probe = psoc5lp_eeprom_auto_probe,
	.read = default_flash_read,
	.erase = psoc5lp_eeprom_erase,
	.erase_check = default_flash_blank_check,
	.write = psoc5lp_eeprom_write,
	.free_driver_priv = default_flash_free_driver_priv,
};

/*
 * Program Flash
 */

struct psoc5lp_flash_bank {
	bool probed;
	const struct psoc5lp_device *device;
	bool ecc_enabled;
	/* If ecc is disabled, num_sectors counts both std and ecc sectors.
	 * If ecc is enabled, num_sectors indicates just the number of std sectors.
	 * However ecc sector descriptors bank->sector[num_sectors..2*num_sectors-1]
	 * are used for driver private flash operations */
};

static int psoc5lp_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct psoc5lp_flash_bank *psoc_bank = bank->driver_priv;
	int retval;

	if (!psoc_bank->ecc_enabled) {
		/* Silently avoid erasing sectors twice */
		if (last >= first + bank->num_sectors / 2) {
			LOG_DEBUG("Skipping duplicate erase of sectors %u to %u",
				first + bank->num_sectors / 2, last);
			last = first + (bank->num_sectors / 2) - 1;
		}
		/* Check for any remaining ECC sectors */
		if (last >= bank->num_sectors / 2) {
			LOG_WARNING("Skipping erase of ECC region sectors %u to %u",
				bank->num_sectors / 2, last);
			last = (bank->num_sectors / 2) - 1;
		}
	}

	for (unsigned int i = first; i <= last; i++) {
		retval = psoc5lp_spc_erase_sector(bank->target,
				i / SECTORS_PER_BLOCK, i % SECTORS_PER_BLOCK);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/* Derived from core.c:default_flash_blank_check() */
static int psoc5lp_erase_check(struct flash_bank *bank)
{
	struct psoc5lp_flash_bank *psoc_bank = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	unsigned int num_sectors = bank->num_sectors;
	if (psoc_bank->ecc_enabled)
		num_sectors *= 2;	/* count both std and ecc sector always */

	struct target_memory_check_block *block_array;
	block_array = malloc(num_sectors * sizeof(struct target_memory_check_block));
	if (!block_array)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < num_sectors; i++) {
		block_array[i].address = bank->base + bank->sectors[i].offset;
		block_array[i].size = bank->sectors[i].size;
		block_array[i].result = UINT32_MAX; /* erase state unknown */
	}

	bool fast_check = true;
	for (unsigned int i = 0; i < num_sectors; ) {
		retval = armv7m_blank_check_memory(target,
					block_array + i, num_sectors - i,
					bank->erased_value);
		if (retval < 1) {
			/* Run slow fallback if the first run gives no result
			 * otherwise use possibly incomplete results */
			if (i == 0)
				fast_check = false;
			break;
		}
		i += retval; /* add number of blocks done this round */
	}

	if (fast_check) {
		if (psoc_bank->ecc_enabled) {
			for (unsigned int i = 0; i < bank->num_sectors; i++)
				bank->sectors[i].is_erased =
					(block_array[i].result != 1)
					? block_array[i].result
					: block_array[i + bank->num_sectors].result;
				/* if std sector is erased, use status of ecc sector */
		} else {
			for (unsigned int i = 0; i < num_sectors; i++)
				bank->sectors[i].is_erased = block_array[i].result;
		}
		retval = ERROR_OK;
	} else {
		LOG_ERROR("Can't run erase check - add working memory");
		retval = ERROR_FAIL;
	}
	free(block_array);

	return retval;
}

static int psoc5lp_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t byte_count)
{
	struct psoc5lp_flash_bank *psoc_bank = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *code_area, *even_row_area, *odd_row_area;
	uint32_t row_size;
	uint8_t temp[2], buf[12], ecc_bytes[ROW_ECC_SIZE];
	unsigned array_id, row;
	int i, retval;

	if (offset + byte_count > bank->size) {
		LOG_ERROR("Writing to ECC not supported");
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}

	if (offset % ROW_SIZE != 0) {
		LOG_ERROR("Writes must be row-aligned, got offset 0x%08" PRIx32,
			offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	row_size = ROW_SIZE;
	if (!psoc_bank->ecc_enabled) {
		row_size += ROW_ECC_SIZE;
		memset(ecc_bytes, bank->default_padded_value, ROW_ECC_SIZE);
	}

	retval = psoc5lp_spc_get_temp(target, 3, temp);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to read Die temperature");
		return retval;
	}
	LOG_DEBUG("Get_Temp: sign 0x%02" PRIx8 ", magnitude 0x%02" PRIx8,
		temp[0], temp[1]);

	assert(target_get_working_area_avail(target) == target->working_area_size);
	retval = target_alloc_working_area(target,
			target_get_working_area_avail(target) / 2, &code_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not allocate working area for program SRAM");
		return retval;
	}
	assert(code_area->address < 0x20000000);

	retval = target_alloc_working_area(target,
			SPC_OPCODE_LEN + 1 + row_size + 3 + SPC_OPCODE_LEN + 6,
			&even_row_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not allocate working area for even row");
		goto err_alloc_even;
	}
	assert(even_row_area->address >= 0x20000000);

	retval = target_alloc_working_area(target, even_row_area->size,
			&odd_row_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not allocate working area for odd row");
		goto err_alloc_odd;
	}
	assert(odd_row_area->address >= 0x20000000);

	for (array_id = offset / BLOCK_SIZE; byte_count > 0; array_id++) {
		for (row = (offset / ROW_SIZE) % ROWS_PER_BLOCK;
		     row < ROWS_PER_BLOCK && byte_count > 0; row++) {
			bool even_row = (row % 2 == 0);
			struct working_area *data_area = even_row ? even_row_area : odd_row_area;
			unsigned len = MIN(ROW_SIZE, byte_count);

			LOG_DEBUG("Writing load command for array %u row %u at " TARGET_ADDR_FMT,
				array_id, row, data_area->address);

			psoc5lp_spc_write_opcode_buffer(target, buf, SPC_LOAD_ROW);
			buf[SPC_OPCODE_LEN] = array_id;
			retval = target_write_buffer(target, data_area->address, 4, buf);
			if (retval != ERROR_OK)
				goto err_write;

			retval = target_write_buffer(target,
				data_area->address + SPC_OPCODE_LEN + 1,
				len, buffer);
			if (retval != ERROR_OK)
				goto err_write;
			buffer += len;
			byte_count -= len;
			offset += len;

			if (len < ROW_SIZE) {
				uint8_t padding[ROW_SIZE];

				memset(padding, bank->default_padded_value, ROW_SIZE);

				LOG_DEBUG("Padding %d bytes", ROW_SIZE - len);
				retval = target_write_buffer(target,
					data_area->address + SPC_OPCODE_LEN + 1 + len,
					ROW_SIZE - len, padding);
				if (retval != ERROR_OK)
					goto err_write;
			}

			if (!psoc_bank->ecc_enabled) {
				retval = target_write_buffer(target,
					data_area->address + SPC_OPCODE_LEN + 1 + ROW_SIZE,
					sizeof(ecc_bytes), ecc_bytes);
				if (retval != ERROR_OK)
					goto err_write;
			}

			for (i = 0; i < 3; i++)
				buf[i] = 0x00; /* 3 NOPs for short delay */
			psoc5lp_spc_write_opcode_buffer(target, buf + 3, SPC_PRG_ROW);
			buf[3 + SPC_OPCODE_LEN] = array_id;
			buf[3 + SPC_OPCODE_LEN + 1] = row >> 8;
			buf[3 + SPC_OPCODE_LEN + 2] = row & 0xff;
			memcpy(buf + 3 + SPC_OPCODE_LEN + 3, temp, 2);
			buf[3 + SPC_OPCODE_LEN + 5] = 0x00; /* padding */
			retval = target_write_buffer(target,
				data_area->address + SPC_OPCODE_LEN + 1 + row_size,
				12, buf);
			if (retval != ERROR_OK)
				goto err_write;

			retval = target_write_u32(target,
				even_row ? PHUB_CH0_BASIC_STATUS : PHUB_CH1_BASIC_STATUS,
				(even_row ? 0 : 1) << 8);
			if (retval != ERROR_OK)
				goto err_dma;

			retval = target_write_u32(target,
				even_row ? PHUB_CH0_BASIC_CFG : PHUB_CH1_BASIC_CFG,
				PHUB_CHX_BASIC_CFG_WORK_SEP | PHUB_CHX_BASIC_CFG_EN);
			if (retval != ERROR_OK)
				goto err_dma;

			retval = target_write_u32(target,
				even_row ? PHUB_CFGMEM0_CFG0 : PHUB_CFGMEM1_CFG0,
				PHUB_CFGMEMX_CFG0);
			if (retval != ERROR_OK)
				goto err_dma;

			retval = target_write_u32(target,
				even_row ? PHUB_CFGMEM0_CFG1 : PHUB_CFGMEM1_CFG1,
				((SPC_CPU_DATA >> 16) << 16) | (data_area->address >> 16));
			if (retval != ERROR_OK)
				goto err_dma;

			retval = target_write_u32(target,
				even_row ? PHUB_TDMEM0_ORIG_TD0 : PHUB_TDMEM1_ORIG_TD0,
				PHUB_TDMEMX_ORIG_TD0_INC_SRC_ADDR |
				PHUB_TDMEMX_ORIG_TD0_NEXT_TD_PTR_LAST |
				((SPC_OPCODE_LEN + 1 + row_size + 3 + SPC_OPCODE_LEN + 5) & 0xfff));
			if (retval != ERROR_OK)
				goto err_dma;

			retval = target_write_u32(target,
				even_row ? PHUB_TDMEM0_ORIG_TD1 : PHUB_TDMEM1_ORIG_TD1,
				((SPC_CPU_DATA & 0xffff) << 16) | (data_area->address & 0xffff));
			if (retval != ERROR_OK)
				goto err_dma;

			retval = psoc5lp_spc_busy_wait_idle(target);
			if (retval != ERROR_OK)
				goto err_idle;

			retval = target_write_u32(target,
				even_row ? PHUB_CH0_ACTION : PHUB_CH1_ACTION,
				PHUB_CHX_ACTION_CPU_REQ);
			if (retval != ERROR_OK)
				goto err_dma_action;
		}
	}

	retval = psoc5lp_spc_busy_wait_idle(target);

err_dma_action:
err_idle:
err_dma:
err_write:
	target_free_working_area(target, odd_row_area);
err_alloc_odd:
	target_free_working_area(target, even_row_area);
err_alloc_even:
	target_free_working_area(target, code_area);

	return retval;
}

static int psoc5lp_protect_check(struct flash_bank *bank)
{
	struct psoc5lp_flash_bank *psoc_bank = bank->driver_priv;
	uint8_t row_data[ROW_SIZE];
	const unsigned protection_bytes_per_sector = ROWS_PER_SECTOR * 2 / 8;
	unsigned i, k, num_sectors;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < DIV_ROUND_UP(bank->size, BLOCK_SIZE); i++) {
		retval = psoc5lp_spc_read_hidden_row(bank->target, i,
				SPC_ROW_PROTECTION, row_data);
		if (retval != ERROR_OK)
			return retval;

		/* Last flash array may have less rows, but in practice full sectors. */
		if (i == bank->size / BLOCK_SIZE)
			num_sectors = (bank->size % BLOCK_SIZE) / SECTOR_SIZE;
		else
			num_sectors = SECTORS_PER_BLOCK;

		for (unsigned int j = 0; j < num_sectors; j++) {
			int sector_nr = i * SECTORS_PER_BLOCK + j;
			struct flash_sector *sector = &bank->sectors[sector_nr];
			struct flash_sector *ecc_sector;

			if (psoc_bank->ecc_enabled)
				ecc_sector = &bank->sectors[bank->num_sectors + sector_nr];
			else
				ecc_sector = &bank->sectors[bank->num_sectors / 2 + sector_nr];

			sector->is_protected = ecc_sector->is_protected = 0;
			for (k = protection_bytes_per_sector * j;
			     k < protection_bytes_per_sector * (j + 1); k++) {
				assert(k < protection_bytes_per_sector * SECTORS_PER_BLOCK);
				LOG_DEBUG("row[%u][%02u] = 0x%02" PRIx8, i, k, row_data[k]);
				if (row_data[k] != 0x00) {
					sector->is_protected = ecc_sector->is_protected = 1;
					break;
				}
			}
		}
	}

	return ERROR_OK;
}

static int psoc5lp_get_info_command(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct psoc5lp_flash_bank *psoc_bank = bank->driver_priv;
	char part_number[PART_NUMBER_LEN];
	const char *ecc;

	psoc5lp_get_part_number(psoc_bank->device, part_number);
	ecc = psoc_bank->ecc_enabled ? "ECC enabled" : "ECC disabled";

	command_print_sameline(cmd, "%s %s", part_number, ecc);

	return ERROR_OK;
}

static int psoc5lp_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct psoc5lp_flash_bank *psoc_bank = bank->driver_priv;
	uint32_t flash_addr = bank->base;
	uint8_t nvl[4], temp[2];
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!psoc_bank->device) {
		retval = psoc5lp_find_device(target, &psoc_bank->device);
		if (retval != ERROR_OK)
			return retval;

		bank->size = psoc_bank->device->flash_kb * 1024;
	}

	bank->num_sectors = DIV_ROUND_UP(bank->size, SECTOR_SIZE);

	if (!psoc_bank->probed) {
		retval = psoc5lp_spc_enable_clock(target);
		if (retval != ERROR_OK)
			return retval;

		/* First values read are inaccurate, so do it once now. */
		retval = psoc5lp_spc_get_temp(target, 3, temp);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to read Die temperature");
			return retval;
		}

		bank->sectors = calloc(bank->num_sectors * 2,
				       sizeof(struct flash_sector));
		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].size = SECTOR_SIZE;
			bank->sectors[i].offset = flash_addr - bank->base;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = -1;

			flash_addr += bank->sectors[i].size;
		}
		flash_addr = 0x48000000;
		for (unsigned int i = bank->num_sectors; i < bank->num_sectors * 2; i++) {
			bank->sectors[i].size = ROWS_PER_SECTOR * ROW_ECC_SIZE;
			bank->sectors[i].offset = flash_addr - bank->base;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = -1;

			flash_addr += bank->sectors[i].size;
		}

		bank->default_padded_value = bank->erased_value = 0x00;

		psoc_bank->probed = true;
	}

	retval = psoc5lp_spc_read_byte(target, SPC_ARRAY_NVL_USER, 3, &nvl[3]);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("NVL[%d] = 0x%02" PRIx8, 3, nvl[3]);
	psoc_bank->ecc_enabled = nvl[3] & NVL_3_ECCEN;

	if (!psoc_bank->ecc_enabled)
		bank->num_sectors *= 2;

	return ERROR_OK;
}

static int psoc5lp_auto_probe(struct flash_bank *bank)
{
	return psoc5lp_probe(bank);
}

COMMAND_HANDLER(psoc5lp_handle_mass_erase_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = psoc5lp_spc_erase_all(bank->target);
	if (retval == ERROR_OK)
		command_print(CMD, "PSoC 5LP erase succeeded");
	else
		command_print(CMD, "PSoC 5LP erase failed");

	return retval;
}

FLASH_BANK_COMMAND_HANDLER(psoc5lp_flash_bank_command)
{
	struct psoc5lp_flash_bank *psoc_bank;

	psoc_bank = malloc(sizeof(struct psoc5lp_flash_bank));
	if (!psoc_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	psoc_bank->probed = false;
	psoc_bank->device = NULL;

	bank->driver_priv = psoc_bank;

	return ERROR_OK;
}

static const struct command_registration psoc5lp_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc5lp_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase all flash data and ECC/configuration bytes, "
			"all flash protection rows, "
			"and all row latches in all flash arrays on the device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc5lp_command_handlers[] = {
	{
		.name = "psoc5lp",
		.mode = COMMAND_ANY,
		.help = "PSoC 5LP flash command group",
		.usage = "",
		.chain = psoc5lp_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver psoc5lp_flash = {
	.name = "psoc5lp",
	.commands = psoc5lp_command_handlers,
	.flash_bank_command = psoc5lp_flash_bank_command,
	.info = psoc5lp_get_info_command,
	.probe = psoc5lp_probe,
	.auto_probe = psoc5lp_auto_probe,
	.protect_check = psoc5lp_protect_check,
	.read = default_flash_read,
	.erase = psoc5lp_erase,
	.erase_check = psoc5lp_erase_check,
	.write = psoc5lp_write,
	.free_driver_priv = default_flash_free_driver_priv,
};
