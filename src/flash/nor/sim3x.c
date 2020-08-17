/***************************************************************************
 *   Copyright (C) 2014 by Ladislav Bábel                                  *
 *   ladababel@seznam.cz                                                   *
 *                                                                         *
 *   Copyright (C) 2015 by Andreas Bomholtz                                *
 *   andreas@seluxit.com                                                   *
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
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

/* SI32_DEVICEID0 */
#define DEVICEID0_DEVICEID0              (0x400490C0)
#define DEVICEID0_DEVICEID1              (0x400490D0)
#define DEVICEID0_DEVICEID2              (0x400490E0)
#define DEVICEID0_DEVICEID3              (0x400490F0)

/* cortex_m CPUID */
#define CPUID_CHECK_VALUE                (0x410FC230)
#define CPUID_CHECK_VALUE_MASK           (0xFF0FFFF0)

/* Flash */
#define FLASH_BASE_ADDRESS               (0x00000000)
#define LOCK_WORD_ADDRESS                (0x0003FFFC)

#define LOCK_WORD_MCU_UNLOCKED           (0xFFFFFFFF)
/* Can't by locked again without erase, because LOCK_WORD is in FLASH */
#define LOCK_WORD_MCU_UNLOCKED_BY_FIRMWARE (0x00000000)

/* SI32_FLASHCTRL_0 */
#define FLASHCTRL0_CONFIG_ALL            (0x4002E000)
#define FLASHCTRL0_CONFIG_SET            (0x4002E004)
#define FLASHCTRL0_CONFIG_CLR            (0x4002E008)
#define FLASHCTRL0_CONFIG_ERASEEN_MASK   (0x00040000)
#define FLASHCTRL0_CONFIG_BUSYF_MASK     (0x00100000)

#define FLASHCTRL0_WRADDR                (0x4002E0A0)
#define FLASHCTRL0_WRDATA                (0x4002E0B0)

#define FLASHCTRL0_KEY                   (0x4002E0C0)
#define FLASHCTRL0_KEY_INITIAL_UNLOCK    (0x000000A5)
#define FLASHCTRL0_KEY_SINGLE_UNLOCK     (0x000000F1)
#define FLASHCTRL0_KEY_MULTIPLE_UNLOCK   (0x000000F2)
#define FLASHCTRL0_KEY_MULTIPLE_LOCK     (0x0000005A)

#define FLASH_BUSY_TIMEOUT               (100)

/* SI32_RSTSRC_0 */
#define RSTSRC0_RESETEN_ALL              (0x4002D060)
#define RSTSRC0_RESETEN_SET              (0x4002D064)
#define RSTSRC0_RESETEN_CLR              (0x4002D068)
#define RSTSRC0_RESETEN_VMONREN_MASK     (0x00000004)
#define RSTSRC0_RESETEN_SWREN_MASK       (0x00000040)

/* SI32_VMON_0 */
#define VMON0_CONTROL_ALL                (0x4002F000)
#define VMON0_CONTROL_SET                (0x4002F004)
#define VMON0_CONTROL_CLR                (0x4002F008)
#define VMON0_CONTROL_VMONEN_MASK        (0x80000000)

/* SI32_CLKCTRL_0 */
#define CLKCTRL0_APBCLKG0_ALL            (0x4002D020)
#define CLKCTRL0_APBCLKG0_SET            (0x4002D024)
#define CLKCTRL0_APBCLKG0_CLR            (0x4002D028)
#define CLKCTRL0_APBCLKG0_FLCTRLCEN_MASK (0x40000000)

/* SI32_WDTIMER_0 */
#define WDTIMER0_CONTROL_ALL             (0x40030000)
#define WDTIMER0_CONTROL_SET             (0x40030004)
#define WDTIMER0_CONTROL_CLR             (0x40030008)
#define WDTIMER0_CONTROL_DBGMD_MASK      (0x00000002)

#define WDTIMER0_STATUS_ALL              (0x40030010)
#define WDTIMER0_STATUS_SET              (0x40030014)
#define WDTIMER0_STATUS_CLR              (0x40030018)
#define WDTIMER0_STATUS_KEYSTS_MASK      (0x00000001)
#define WDTIMER0_STATUS_PRIVSTS_MASK     (0x00000002)

#define WDTIMER0_THRESHOLD               (0x40030020)

#define WDTIMER0_WDTKEY                  (0x40030030)
#define WDTIMER0_KEY_ATTN                (0x000000A5)
#define WDTIMER0_KEY_WRITE               (0x000000F1)
#define WDTIMER0_KEY_RESET               (0x000000CC)
#define WDTIMER0_KEY_DISABLE             (0x000000DD)
#define WDTIMER0_KEY_START               (0x000000EE)
#define WDTIMER0_KEY_LOCK                (0x000000FF)

/* DAP */
#define SIM3X_AP                         (0x0A)

#define SIM3X_AP_CTRL1                   (0x00)
#define SIM3X_AP_CTRL2                   (0x04)
#define SIM3X_AP_LOCK                    (0x08)
#define SIM3X_AP_CRC                     (0x0C)

#define SIM3X_AP_INIT_STAT               (0x10)
#define SIM3X_AP_DAP_IN                  (0x14)
#define SIM3X_AP_DAP_OUT                 (0x18)

#define SIM3X_AP_ID                      (0xFC)

/* DAP register values */
#define SIM3X_AP_CTRL1_MASS_ERASE_REQ    (0x00000001)
#define SIM3X_AP_CTRL1_RESET_REQ         (0x00000008)
/* this bit is set if MCU is locked */
#define SIM3X_AP_INIT_STAT_LOCK          (0x00000004)
/* expected value inside SIM3X_AP_ID */
#define SIM3X_AP_ID_VALUE                (0x2430002)

#define SIM3X_FLASH_PAGE_SIZE            1024

struct sim3x_info {
	uint16_t flash_size_kb;
	uint16_t part_number;
	char part_family;
	uint8_t device_revision;
	char device_package[4];
	bool probed;
	bool need_init;
	bool flash_locked;
};

/* flash bank sim3x 0 0 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(sim3x_flash_bank_command)
{
	struct sim3x_info *sim3x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Init sim3x_info struct */
	sim3x_info = malloc(sizeof(struct sim3x_info));
	sim3x_info->probed = false;
	sim3x_info->need_init = true;
	sim3x_info->device_revision = 0;
	memset(sim3x_info->device_package, 0, 4);
	bank->driver_priv = sim3x_info;

	return ERROR_OK;
}

static int sim3x_init(struct flash_bank *bank)
{
	int ret;
	struct target *target;
	struct sim3x_info *sim3x_info;

	target = bank->target;

	/* Disable watchdog timer */
	ret = target_write_u32(target, WDTIMER0_WDTKEY, WDTIMER0_KEY_ATTN);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, WDTIMER0_WDTKEY, WDTIMER0_KEY_DISABLE);
	if (ret != ERROR_OK)
		return ret;

	/* Enable one write command */
	ret = target_write_u32(target, WDTIMER0_WDTKEY, WDTIMER0_KEY_ATTN);
	if (ret != ERROR_OK)
		return ret;

	ret = target_write_u32(target, WDTIMER0_WDTKEY, WDTIMER0_KEY_WRITE);
	if (ret != ERROR_OK)
		return ret;

	/* Watchdog Timer Debug Mode */
	ret = target_write_u32(target, WDTIMER0_CONTROL_SET,
			WDTIMER0_CONTROL_DBGMD_MASK);
	if (ret != ERROR_OK)
		return ret;

	/* Enable VDD Supply Monitor */
	ret = target_write_u32(target, VMON0_CONTROL_SET,
			VMON0_CONTROL_VMONEN_MASK);
	if (ret != ERROR_OK)
		return ret;

	/* Set VDD Supply Monitor as a reset source */
	ret = target_write_u32(target, RSTSRC0_RESETEN_SET,
			RSTSRC0_RESETEN_VMONREN_MASK);
	if (ret != ERROR_OK)
		return ret;

	/* Flash Controller Clock Enable */
	ret = target_write_u32(target, CLKCTRL0_APBCLKG0_SET,
			CLKCTRL0_APBCLKG0_FLCTRLCEN_MASK);
	if (ret != ERROR_OK)
		return ret;

	/* Disable Flash Erase Mode */
	ret = target_write_u32(target, FLASHCTRL0_CONFIG_CLR,
			FLASHCTRL0_CONFIG_ERASEEN_MASK);
	if (ret != ERROR_OK)
		return ret;

	sim3x_info = bank->driver_priv;
	sim3x_info->need_init = 0;
	return ERROR_OK;
}

static int sim3x_erase_page(struct flash_bank *bank, uint32_t addr)
{
	int ret, i;
	uint32_t temp;
	struct target *target;

	target = bank->target;

	for (i = 0; i < FLASH_BUSY_TIMEOUT; i++) {
		ret = target_read_u32(target, FLASHCTRL0_CONFIG_ALL, &temp);
		if (ret != ERROR_OK)
			return ret;

		/* If is not busy */
		if ((temp & FLASHCTRL0_CONFIG_BUSYF_MASK) == 0) {
			/* If erase is not enabled */
			if ((temp & FLASHCTRL0_CONFIG_ERASEEN_MASK) == 0) {
				/* Enter Flash Erase Mode */
				ret = target_write_u32(target, FLASHCTRL0_CONFIG_SET,
						FLASHCTRL0_CONFIG_ERASEEN_MASK);
				if (ret != ERROR_OK)
					return ret;
			}

			/* Write the address of the Flash page to WRADDR */
			ret = target_write_u32(target, FLASHCTRL0_WRADDR, addr);
			if (ret != ERROR_OK)
				return ret;

			/* Write the initial unlock value to KEY */
			ret = target_write_u32(target, FLASHCTRL0_KEY,
			FLASHCTRL0_KEY_INITIAL_UNLOCK);
			if (ret != ERROR_OK)
				return ret;

			/* Write the single unlock value to KEY */
			ret = target_write_u32(target, FLASHCTRL0_KEY,
			FLASHCTRL0_KEY_SINGLE_UNLOCK);
			if (ret != ERROR_OK)
				return ret;

			/* Write any value to WRDATA to initiate the page erase */
			ret = target_write_u32(target, FLASHCTRL0_WRDATA, 0);
			if (ret != ERROR_OK)
				return ret;

			return ERROR_OK;
		}

		alive_sleep(1);
	}

	LOG_ERROR("timed out waiting for FLASHCTRL0_CONFIG_BUSYF");
	return ERROR_FAIL;
}

static int sim3x_flash_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int ret;
	uint32_t temp;
	struct sim3x_info *sim3x_info;
	struct target *target;

	/* Check if target is halted */
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	sim3x_info = bank->driver_priv;

	/* Init MCU after reset */
	if (sim3x_info->need_init) {
		ret = sim3x_init(bank);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to init MCU");
			return ret;
		}
	}

	/* erase pages */
	for (unsigned int i = first; i <= last; i++) {
		ret = sim3x_erase_page(bank, bank->sectors[i].offset);
		if (ret != ERROR_OK)
			return ret;
	}

	target = bank->target;

	/* Wait until busy */
	for (unsigned int i = 0; i < FLASH_BUSY_TIMEOUT; i++) {
		ret = target_read_u32(target, FLASHCTRL0_CONFIG_ALL, &temp);
		if (ret != ERROR_OK)
			return ret;

		if ((temp & FLASHCTRL0_CONFIG_BUSYF_MASK) == 0) { /* If is not busy */
			if ((temp & FLASHCTRL0_CONFIG_ERASEEN_MASK) != 0) { /* If erase is enabled */
				/* Disable Flash Erase Mode */
				ret = target_write_u32(target, FLASHCTRL0_CONFIG_CLR,
						FLASHCTRL0_CONFIG_ERASEEN_MASK);
				if (ret != ERROR_OK)
					return ret;
			}

			return ERROR_OK;
		}

		alive_sleep(1);
	}

	LOG_ERROR("timed out waiting for FLASHCTRL0_CONFIG_BUSYF");
	return ERROR_FAIL;
}

static int sim3x_write_block(struct flash_bank *bank, const uint8_t *buf,
		uint32_t offset, uint32_t count) /* count is count of half words (2 bytes)! */
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int ret = ERROR_OK;

	/* see contrib/loaders/flash/sim3x.s for src */

	static const uint8_t sim3x_flash_write_code[] = {
		/* Write the initial unlock value to KEY (0xA5) */
		0xA5, 0x26, /* movs    r6, #INITIAL_UNLOCK */
		0xC0, 0xF8, 0xC0, 0x60, /* str     r6, [r0, #FLASHCTRL_KEY] */

		/* Write the multiple unlock value to KEY (0xF2) */
		0xF2, 0x26, /* movs    r6, #MULTIPLE_UNLOCK */
		0xC0, 0xF8, 0xC0, 0x60, /* str     r6, [r0, #FLASHCTRL_KEY] */

		/* wait_fifo: */
		0x16, 0x68, /* ldr     r6, [r2, #0] */
		0x00, 0x2E, /* cmp     r6, #0 */
		0x16, 0xD0, /* beq     exit */
		0x55, 0x68, /* ldr     r5, [r2, #4] */
		0xB5, 0x42, /* cmp     r5, r6 */
		0xF9, 0xD0, /* beq     wait_fifo */

		/* wait for BUSYF flag */
		/* wait_busy1: */
		0x06, 0x68, /* ldr     r6, [r0, #FLASHCTRL_CONFIG] */
		0x16, 0xF4, 0x80, 0x1F, /* tst     r6, #BUSYF */
		0xFB, 0xD1, /* bne     wait_busy1 */

		/* Write the destination address to WRADDR */
		0xC0, 0xF8, 0xA0, 0x40, /* str     r4, [r0, #FLASHCTRL_WRADDR] */

		/* Write the data half-word to WRDATA in right-justified format */
		0x2E, 0x88, /* ldrh    r6, [r5] */
		0xC0, 0xF8, 0xB0, 0x60, /* str     r6, [r0, #FLASHCTRL_WRDATA] */

		0x02, 0x35, /* adds    r5, #2 */
		0x02, 0x34, /* adds    r4, #2 */

		/* wrap rp at end of buffer */
		0x9D, 0x42, /* cmp     r5, r3 */
		0x01, 0xD3, /* bcc     no_wrap */
		0x15, 0x46, /* mov     r5, r2 */
		0x08, 0x35, /* adds    r5, #8 */

		/* no_wrap: */
		0x55, 0x60, /* str     r5, [r2, #4] */
		0x49, 0x1E, /* subs    r1, r1, #1 */
		0x00, 0x29, /* cmp     r1, #0 */
		0x00, 0xD0, /* beq     exit */
		0xE5, 0xE7, /* b       wait_fifo */

		/* exit: */
		0x5A, 0x26, /* movs    r6, #MULTIPLE_LOCK */
		0xC0, 0xF8, 0xC0, 0x60, /* str     r6, [r0, #FLASHCTRL_KEY] */

		/* wait for BUSYF flag */
		/* wait_busy2: */
		0x06, 0x68, /* ldr     r6, [r0, #FLASHCTRL_CONFIG] */
		0x16, 0xF4, 0x80, 0x1F, /* tst     r6, #BUSYF */
		0xFB, 0xD1, /* bne     wait_busy2 */

		0x00, 0xBE /* bkpt    #0 */
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(sim3x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	ret = target_write_buffer(target, write_algorithm->address,
			sizeof(sim3x_flash_write_code), sim3x_flash_write_code);
	if (ret != ERROR_OK)
		return ret;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~1UL; /* Make sure it's 2 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm
			 */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* flash base */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* count */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT); /* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT); /* target address */

	buf_set_u32(reg_params[0].value, 0, 32, FLASHCTRL0_CONFIG_ALL);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	ret = target_run_flash_async_algorithm(target, buf, count, 2, 0, NULL, 5,
			reg_params, source->address, source->size, write_algorithm->address,
			0, &armv7m_info);

	if (ret == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
			buf_get_u32(reg_params[4].value, 0, 32));
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return ret;
}

static int sim3x_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int ret;
	struct target *target;
	struct sim3x_info *sim3x_info;
	uint8_t *new_buffer = NULL;

	target = bank->target;

	/* Check if target is halted */
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	sim3x_info = bank->driver_priv;

	if (sim3x_info->flash_locked) {
		LOG_ERROR("Flash is locked");
		return ERROR_FAIL;
	}

	/* Init MCU after reset */
	if (sim3x_info->need_init) {
		ret = sim3x_init(bank);
		if (ret != ERROR_OK)
			return ret;
	}

	if (offset & 0x1) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x1) {
		uint32_t old_count = count;
		count++;
		new_buffer = malloc(count);

		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory "
					"for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write (%" PRIu32 "), extending to %" PRIu32
				" and padding with 0xff", old_count, count);

		new_buffer[count - 1] = 0xff;
		buffer = memcpy(new_buffer, buffer, old_count);
	}

	ret = sim3x_write_block(bank, buffer, offset, count / 2);
	free(new_buffer);
	return ret;
}

static int sim3x_flash_lock_check(struct flash_bank *bank)
{
	int ret;
	uint32_t lock_word;
	struct sim3x_info *sim3x_info;

	ret = target_read_u32(bank->target, LOCK_WORD_ADDRESS, &lock_word);
	if (ret != ERROR_OK) {
		LOG_ERROR("Can not read Lock Word");
		return ret;
	}

	sim3x_info = bank->driver_priv;
	sim3x_info->flash_locked = (lock_word != 0xFFFFFFFF);

	return ERROR_OK;
}

static int sim3x_flash_protect_check(struct flash_bank *bank)
{
	int ret;
	struct sim3x_info *sim3x_info;

	/* Check if target is halted */
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = sim3x_flash_lock_check(bank);
	if (ret != ERROR_OK)
		return ret;

	sim3x_info = bank->driver_priv;

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = sim3x_info->flash_locked;

	return ERROR_OK;
}

static int sim3x_flash_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	int ret;
	uint8_t lock_word[4];
	struct sim3x_info *sim3x_info;
	struct target *target;

	target = bank->target;

	/* Check if target is halted */
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (first != 0 || last != bank->num_sectors - 1) {
		LOG_ERROR("Flash does not support finer granularity");
		return ERROR_FAIL;
	}

	sim3x_info = bank->driver_priv;

	if (set) {
		if (sim3x_info->flash_locked) {
			LOG_INFO("Flash is already locked");
			return ERROR_OK;
		}

		/* Lock Flash */
		target_buffer_set_u32(target, lock_word, 0xFFFFFFFE);
		ret = sim3x_flash_write(bank, lock_word, LOCK_WORD_ADDRESS, 4);
		if (ret != ERROR_OK)
			return ret;

	} else {
		/* Flash is unlocked by an erase operation */
		ret = sim3x_flash_erase(bank, 0, 0);
		if (ret != ERROR_OK)
			return ret;
	}

	ret = sim3x_flash_protect_check(bank);
	if (ret != ERROR_OK)
		return ret;

	if (set) {
		if (sim3x_info->flash_locked) {
			LOG_INFO("Flash locked");
			return ERROR_OK;
		} else {
			LOG_ERROR("Flash lock error");
			return ERROR_FAIL;
		}
	} else {
		if (sim3x_info->flash_locked) {
			LOG_ERROR("Flash unlock error");
			return ERROR_FAIL;
		} else {
			LOG_INFO("Flash unlocked");
			return ERROR_OK;
		}
	}
}

static int sim3x_read_deviceid(struct flash_bank *bank)
{
	int ret;
	struct sim3x_info *sim3x_info;

	uint32_t device_id;
	int part_number;
	char part_num_string[4];

	sim3x_info = bank->driver_priv;

	/* MCU check */
	ret = target_read_u32(bank->target, DEVICEID0_DEVICEID2, &device_id);
	if (ret != ERROR_OK)
		return ret;

	/* Device ID should be 'M3' */
	if (device_id != 0x00004D33)
		return ERROR_FAIL;

	/* Family and Part number */
	ret = target_read_u32(bank->target, DEVICEID0_DEVICEID1, &device_id);
	if (ret != ERROR_OK)
		return ret;

	part_num_string[0] = device_id >> 16;
	part_num_string[1] = device_id >> 8;
	part_num_string[2] = device_id;
	part_num_string[3] = 0;

	part_number = atoi(part_num_string);

	/* Part Number should be between 100 and 999 */
	if (!isalpha(device_id >> 24) || part_number < 100 || part_number > 999)
		return ERROR_FAIL;

	sim3x_info->part_family = device_id >> 24;
	sim3x_info->part_number = part_number;

	/* Package and Revision */
	ret = target_read_u32(bank->target, DEVICEID0_DEVICEID0, &device_id);
	if (ret != ERROR_OK)
		return ret;

	sim3x_info->device_package[0] = device_id >> 24;
	sim3x_info->device_package[1] = device_id >> 16;
	sim3x_info->device_package[2] = device_id >> 8;
	sim3x_info->device_package[3] = 0;

	sim3x_info->device_revision = device_id;

	return ERROR_OK;
}

static int sim3x_parse_part_info(struct sim3x_info *sim3x_info)
{
	switch (sim3x_info->part_number) {
	case 134:
	case 136:
		sim3x_info->flash_size_kb = 32;
		break;
	case 144:
	case 146:
		sim3x_info->flash_size_kb = 64;
		break;
	case 154:
	case 156:
	case 157:
		sim3x_info->flash_size_kb = 128;
		break;
	case 164:
	case 166:
	case 167:
		sim3x_info->flash_size_kb = 256;
		break;
	default:
		LOG_ERROR("Unknown Part number %d", sim3x_info->part_number);
		sim3x_info->part_number = 0;
		return ERROR_FAIL;
	}

	switch (sim3x_info->part_family) {
	case 'c':
	case 'C':
		LOG_INFO("SiM3C%d detected", sim3x_info->part_number);
		break;
	case 'u':
	case 'U':
		LOG_INFO("SiM3U%d detected", sim3x_info->part_number);
		break;
	case 'l':
	case 'L':
		LOG_INFO("SiM3L%d detected", sim3x_info->part_number);
		break;
	default:
		LOG_ERROR("Unsupported MCU family %c", sim3x_info->part_family);
		sim3x_info->part_family = 0;
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int sim3x_read_info(struct flash_bank *bank)
{
	int ret;
	struct sim3x_info *sim3x_info;
	uint32_t cpuid;

	sim3x_info = bank->driver_priv;

	/* Core check */
	ret = target_read_u32(bank->target, CPUID, &cpuid);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read CPU ID");
		return ret;
	}

	if (((cpuid >> 4) & 0xfff) != 0xc23) {
		LOG_ERROR("Target is not Cortex-M3");
		return ERROR_FAIL;
	}

	/* Read info from chip */
	ret = sim3x_read_deviceid(bank);
	if (ret == ERROR_OK) {
		ret = sim3x_parse_part_info(sim3x_info);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to parse info from MCU");
			return ERROR_FAIL;
		}
	} else {
		LOG_WARNING("Failed to read info from MCU, using info from flash bank parameters");

		/* Check if flash size is given in flash bank command */
		if (!bank->size) {
			LOG_ERROR("Flash size not set in the flash bank command");
			return ERROR_FAIL;
		}

		/* Convert bank size to kb */
		sim3x_info->flash_size_kb = bank->size / 1024;
	}

	LOG_INFO("Flash size = %dKB", sim3x_info->flash_size_kb);

	return ERROR_OK;
}

static int sim3x_probe(struct flash_bank *bank)
{
	int ret, i;
	struct sim3x_info *sim3x_info;

	sim3x_info = bank->driver_priv;
	sim3x_info->probed = false;
	sim3x_info->need_init = true;

	/* Read info from chip */
	ret = sim3x_read_info(bank);
	if (ret != ERROR_OK)
		return ret;

	ret = sim3x_flash_lock_check(bank);
	if (ret != ERROR_OK)
		return ret;

	free(bank->sectors);

	bank->base = FLASH_BASE_ADDRESS;
	bank->size = sim3x_info->flash_size_kb * SIM3X_FLASH_PAGE_SIZE;
	bank->num_sectors = SIM3X_FLASH_PAGE_SIZE;
	bank->sectors = malloc(sizeof(struct flash_sector) * sim3x_info->flash_size_kb);

	for (i = 0; i < sim3x_info->flash_size_kb; i++) {
		bank->sectors[i].offset = i * SIM3X_FLASH_PAGE_SIZE;
		bank->sectors[i].size = SIM3X_FLASH_PAGE_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = sim3x_info->flash_locked;
	}

	sim3x_info->probed = true;

	return ERROR_OK;
}

static int sim3x_auto_probe(struct flash_bank *bank)
{
	struct sim3x_info *sim3x_info;

	sim3x_info = bank->driver_priv;

	if (sim3x_info->probed) {
		sim3x_info->need_init = true;
		return ERROR_OK;
	} else {
		return sim3x_probe(bank);
	}
}

static int sim3x_flash_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int ret;
	int printed = 0;
	struct sim3x_info *sim3x_info;

	sim3x_info = bank->driver_priv;

	/* Read info about chip */
	ret = sim3x_read_info(bank);
	if (ret != ERROR_OK)
		return ret;

	/* Part */
	if (sim3x_info->part_family && sim3x_info->part_number) {
		printed = snprintf(buf, buf_size, "SiM3%c%d", sim3x_info->part_family, sim3x_info->part_number);
		buf += printed;
		buf_size -= printed;

		if (buf_size <= 0)
			return ERROR_BUF_TOO_SMALL;

		/* Revision */
		if (sim3x_info->device_revision && sim3x_info->device_revision <= 'Z' - 'A') {
			printed = snprintf(buf, buf_size, "-%c", sim3x_info->device_revision + 'A');
			buf += printed;
			buf_size -= printed;

			if (buf_size <= 0)
				return ERROR_BUF_TOO_SMALL;

			/* Package */
			printed = snprintf(buf, buf_size, "-G%s", sim3x_info->device_package);
			buf += printed;
			buf_size -= printed;

			if (buf_size <= 0)
				return ERROR_BUF_TOO_SMALL;
		}
	}

	/* Print flash size */
	printed = snprintf(buf, buf_size, " flash_size = %dKB", sim3x_info->flash_size_kb);
	buf_size -= printed;

	if (buf_size <= 0)
		return ERROR_BUF_TOO_SMALL;

	return ERROR_OK;
}
/**
 *  reg 31:8 - no effect
 *  reg 7:4  - bank
 *  reg 3:2  - register
 *  reg 1:0  - no effect
 */
static int ap_write_register(struct adiv5_dap *dap, unsigned reg, uint32_t value)
{
	int retval;
	LOG_DEBUG("DAP_REG[0x%02x] <- %08" PRIX32, reg, value);

	retval = dap_queue_ap_write(dap_ap(dap, SIM3X_AP), reg, value);
	if (retval != ERROR_OK) {
		LOG_DEBUG("DAP: failed to queue a write request");
		return retval;
	}

	retval = dap_run(dap);
	if (retval != ERROR_OK) {
		LOG_DEBUG("DAP: dap_run failed");
		return retval;
	}

	return ERROR_OK;
}

static int ap_read_register(struct adiv5_dap *dap, unsigned reg, uint32_t *result)
{
	int retval;

	retval = dap_queue_ap_read(dap_ap(dap, SIM3X_AP), reg, result);
	if (retval != ERROR_OK) {
		LOG_DEBUG("DAP: failed to queue a read request");
		return retval;
	}

	retval = dap_run(dap);
	if (retval != ERROR_OK) {
		LOG_DEBUG("DAP: dap_run failed");
		return retval;
	}

	LOG_DEBUG("DAP_REG[0x%02x]: %08" PRIX32, reg, *result);
	return ERROR_OK;
}

static int ap_poll_register(struct adiv5_dap *dap, unsigned reg, uint32_t mask, uint32_t value, int timeout)
{
	uint32_t val;
	int retval;

	do {
		retval = ap_read_register(dap, reg, &val);
		if (retval != ERROR_OK || (val & mask) == value)
			return retval;

		alive_sleep(1);
	} while (timeout--);

	LOG_DEBUG("DAP: polling timed out");
	return ERROR_FAIL;
}

COMMAND_HANDLER(sim3x_mass_erase)
{
	uint32_t val;
	int ret;

	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct adiv5_dap *dap = cortex_m->armv7m.arm.dap;

	if (dap == NULL) {
		/* Used debug interface doesn't support direct DAP access */
		LOG_ERROR("mass_erase can't be used by this debug interface");
		return ERROR_FAIL;
	}

	ret = ap_read_register(dap, SIM3X_AP_ID, &val);
	if (ret != ERROR_OK)
		return ret;

	if (val != SIM3X_AP_ID_VALUE) {
		LOG_ERROR("Wrong SIM3X_AP_ID");
		return ERROR_FAIL;
	}

	/* Mass erase sequence */
	ret = ap_write_register(dap, SIM3X_AP_CTRL1, SIM3X_AP_CTRL1_RESET_REQ);
	if (ret != ERROR_OK)
		return ret;

	ret = ap_write_register(dap, SIM3X_AP_CTRL1, SIM3X_AP_CTRL1_RESET_REQ | SIM3X_AP_CTRL1_MASS_ERASE_REQ);
	if (ret != ERROR_OK)
		return ret;

	ret = ap_poll_register(dap, SIM3X_AP_CTRL1, SIM3X_AP_CTRL1_MASS_ERASE_REQ, 0x00000000, FLASH_BUSY_TIMEOUT);
	if (ret != ERROR_OK)
		return ret;

	ret = ap_write_register(dap, SIM3X_AP_CTRL1, 0x00000000); /* clear SIM3X_AP_CTRL1_RESET_REQ */
	if (ret != ERROR_OK)
		return ret;

	LOG_INFO("Mass erase success");
	return ERROR_OK;
}

COMMAND_HANDLER(sim3x_lock)
{
	uint32_t val;
	int ret;

	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct adiv5_dap *dap = cortex_m->armv7m.arm.dap;

	if (dap == NULL) {
		/* Used debug interface doesn't support direct DAP access */
		LOG_INFO("Target can't by unlocked by this debug interface");

		/* Core check */
		ret = target_read_u32(target, CPUID, &val);
		if (ret != ERROR_OK)
			return ret;

		if ((val & CPUID_CHECK_VALUE_MASK) != CPUID_CHECK_VALUE) {
			LOG_ERROR("Target is not ARM Cortex-M3 or is already locked");
			return ERROR_FAIL;
		}
	} else {
		/* check SIM3X_AP_ID */
		ret = ap_read_register(dap, SIM3X_AP_ID, &val);
		if (ret != ERROR_OK)
			return ret;

		if (val != SIM3X_AP_ID_VALUE) {
			LOG_ERROR("Wrong SIM3X_AP_ID");
			return ERROR_FAIL;
		}

		/* check if locked */
		ret = target_read_u32(target, CPUID, &val);
		/* if correct value is read, then it will continue */
		if (ret != ERROR_OK || (val & CPUID_CHECK_VALUE_MASK) != CPUID_CHECK_VALUE) {
			/* if correct value isn't read, then it will check SIM3X_AP_INIT_STAT register */
			ret = ap_read_register(dap, SIM3X_AP_INIT_STAT, &val);
			if (ret != ERROR_OK)
				return ret;

			if (val & SIM3X_AP_INIT_STAT_LOCK) {
				LOG_INFO("Target is already locked");
				return ERROR_OK;
			} else {
				LOG_ERROR("Target doesn't seem to be locked but memory was not read correct");
				return ERROR_FAIL;
			}
		}
	}

	ret = target_read_u32(target, LOCK_WORD_ADDRESS, &val);
	if (ret != ERROR_OK)
		return ret;

	if (val == LOCK_WORD_MCU_UNLOCKED) {
		/* Lock Flash */
		uint8_t lock_word[4];
		target_buffer_set_u32(target, lock_word, 0xFFFFFFFE);

		/* Get Flash Bank */
		struct flash_bank *bank;
		int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
		if (retval != ERROR_OK)
			return retval;

		ret = sim3x_flash_write(bank, lock_word, LOCK_WORD_ADDRESS, 4);
		if (ERROR_OK != ret)
			return ret;

		LOG_INFO("Target is successfully locked");
		return ERROR_OK;
	} else if (val == LOCK_WORD_MCU_UNLOCKED_BY_FIRMWARE) {
		/* Can't by locked again without erase, because LOCK_WORD is in FLASH */
		LOG_ERROR("Target is unlocked by firmware and can't by locked again without the lock page erase or mass erase");
		return ERROR_FAIL;
	} else {
		LOG_ERROR("Unexpected lock word value");

		/* SIM3X_AP_ID_VALUE is not checked */
		if (dap == NULL)
			LOG_INFO("Maybe this isn't a SiM3x MCU");

		return ERROR_FAIL;
	}
}

static const struct command_registration sim3x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.mode = COMMAND_EXEC,
		.help = "Erase the complete flash",
		.usage = "",
		.handler = sim3x_mass_erase,
	},
	{
		.name = "lock",
		.mode = COMMAND_EXEC,
		.help = "Locks the flash. Unlock by mass erase",
		.usage = "",
		.handler = sim3x_lock,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration sim3x_command_handlers[] = {
	{
		.name = "sim3x",
		.mode = COMMAND_ANY,
		.help = "sim3x flash command group",
		.usage = "",
		.chain = sim3x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver sim3x_flash = {
	.name = "sim3x",
	.commands = sim3x_command_handlers,
	.flash_bank_command = sim3x_flash_bank_command,
	.erase = sim3x_flash_erase,
	.protect = sim3x_flash_protect,
	.write = sim3x_flash_write,
	.read = default_flash_read,
	.probe = sim3x_probe,
	.auto_probe = sim3x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = sim3x_flash_protect_check,
	.info = sim3x_flash_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
