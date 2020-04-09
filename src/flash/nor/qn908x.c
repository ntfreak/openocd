/***************************************************************************
 *   Copyright (C) 2020 iosabi                                             *
 *   iosabi <iosabi@protonmail.com>                                        *
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
#include <helper/types.h>

/* The QN908x has two flash regions, one is the main flash region holding the
 * user code and the second one is a small (0x800 bytes) "Flash information
 * page" that can't be written to by the used. This page contains information
 * programmed at the factory.
 *
 * The main flash region is normally 512kB, there's a field in the "Flash
 * information page" that allows to specify for 256kB size chip however at the
 * time of writing none of the variants in the market have 256kB.
 *
 * The flash is divided into blocks of 256kB each, therefore containing two
 * blocks. A block is subdivided into pages, of 2048 bytes. A page is the
 * smallest region that can be erased or protected independently, although it is
 * also possible to erase a whole block or both blocks.
 * A page is subdivided into 8 rows of 64 words (32-bit words). The word
 * subdivision is only relevant because DMA can write multiple words in the same
 * row in the same flash program operation.
 *
 * For the Flash information page we are only interested in the last 0x100 bytes
 * which contain a CRC-32 checksum of that 0x100 bytes long region and a field
 * stating the size of the flash. This is also a good check that we are dealing
 * with the right chip/flash configuration and is used in the probe() function.
 */
#define QN908X_FLASH_BASE				0x01000000

#define QN908X_FLASH_PAGE_SIZE			2048
#define QN908X_FLASH_PAGES_PER_BLOCK	128
#define QN908X_FLASH_BLOCK_SIZE			(QN908X_FLASH_PAGES_PER_BLOCK * \
										 QN908X_FLASH_PAGE_SIZE)
#define QN908X_FLASH_MAX_BLOCKS			2

/* Flash information page memory fields. */
#define QN908X_INFO_PAGE_BASE 0x210b0000u
enum qn908x_info_page_fields {
	QN908X_INFO_PAGE_CRC32			= 0x700,
	QN908X_INFO_PAGE_CRC_START		= 0x704,
	QN908X_INFO_PAGE_BOOTLOADER_VER	= 0x704,
	QN908X_INFO_PAGE_FLASH_SIZE		= 0x708,
	QN908X_INFO_PAGE_BLUETOOTH_ADDR	= 0x7fa,
	QN908X_INFO_PAGE_CRC_END		= 0x800,
};

/* Possible values of the QN908X_INFO_PAGE_FLASH_SIZE field. */
enum qn908x_info_page_flash_size {
	QN908X_FLASH_SIZE_512K = 0xfffff0ff,
	QN908X_FLASH_SIZE_256K = 0xffffe0ff,
};

/* QN908x "Flash memory controller", described in section 28 of the user
 * manual. In the NXP SDK this peripheral is called "FLASH", however we use the
 * name "FMC" (Flash Memory Controller) here when referring to the controller
 * to avoid confusion with other "flash" terms in OpenOCD. */
#define QN908X_FMC_BASE 0x40081000u
enum qn908x_fmc_registers {
	QN908X_FMC_INI_RD_EN		= 0x0,
	QN908X_FMC_ERASE_CTRL		= 0x4,
	QN908X_FMC_ERASE_TIME		= 0x8,
	QN908X_FMC_TIME_CTRL		= 0xc,
	QN908X_FMC_SMART_CTRL		= 0x10,
	QN908X_FMC_INT_STAT			= 0x18,
	QN908X_FMC_LOCK_STAT_0		= 0x20,
	QN908X_FMC_LOCK_STAT_1		= 0x24,
	QN908X_FMC_LOCK_STAT_2		= 0x28,
	QN908X_FMC_LOCK_STAT_3		= 0x2c,
	QN908X_FMC_LOCK_STAT_4		= 0x30,
	QN908X_FMC_LOCK_STAT_5		= 0x34,
	QN908X_FMC_LOCK_STAT_6		= 0x38,
	QN908X_FMC_LOCK_STAT_7		= 0x3c,
	QN908X_FMC_LOCK_STAT_8		= 0x40,
	QN908X_FMC_STATUS1			= 0x48,
	QN908X_FMC_DEBUG_PASSWORD	= 0xa8,
	QN908X_FMC_ERASE_PASSWORD	= 0xac,
};

#define QN908X_FMC_INI_RD_EN_INI_RD_EN_MASK		0x1u

#define QN908X_FMC_STATUS1_FSH_ERA_BUSY_L_MASK		0x200u
#define QN908X_FMC_STATUS1_FSH_WR_BUSY_L_MASK		0x400u
#define QN908X_FMC_STATUS1_FSH_ERA_BUSY_H_MASK		0x1000u
#define QN908X_FMC_STATUS1_FSH_WR_BUSY_H_MASK		0x2000u
#define QN908X_FMC_STATUS1_INI_RD_DONE_MASK			0x8000u
#define QN908X_FMC_STATUS1_FSH_STA_MASK				0x4000000u

#define QN908X_FMC_ERASE_CTRL_PAGE_IDXL_SHIFT		0
#define QN908X_FMC_ERASE_CTRL_PAGE_IDXH_SHIFT		8
#define QN908X_FMC_ERASE_CTRL_HALF_ERASEL_EN_SHIFT	28
#define QN908X_FMC_ERASE_CTRL_HALF_ERASEH_EN_SHIFT	29
#define QN908X_FMC_ERASE_CTRL_PAGE_ERASEL_EN_SHIFT	30
#define QN908X_FMC_ERASE_CTRL_PAGE_ERASEH_EN_SHIFT	31

#define QN908X_FMC_INT_STAT_ERASEL_INT_MASK		0x4u
#define QN908X_FMC_INT_STAT_WRITEL_INT_MASK		0x8u
#define QN908X_FMC_INT_STAT_WRBUFL_INT_MASK		0x10u
#define QN908X_FMC_INT_STAT_ERASEH_INT_MASK		0x400u
#define QN908X_FMC_INT_STAT_WRITEH_INT_MASK		0x800u
#define QN908X_FMC_INT_STAT_WRBUFH_INT_MASK		0x1000u

#define QN908X_FMC_SMART_CTRL_PRGML_EN_MASK			0x1u
#define QN908X_FMC_SMART_CTRL_PRGMH_EN_MASK			0x2u
#define QN908X_FMC_SMART_CTRL_SMART_WRITEL_EN_MASK	0x4u
#define QN908X_FMC_SMART_CTRL_SMART_WRITEH_EN_MASK	0x8u
#define QN908X_FMC_SMART_CTRL_SMART_ERASEL_EN_MASK	0x10u
#define QN908X_FMC_SMART_CTRL_SMART_ERASEH_EN_MASK	0x20u
#define QN908X_FMC_SMART_CTRL_MAX_WRITE_MASK		0xf00u
#define QN908X_FMC_SMART_CTRL_MAX_WRITE_SHIFT		8u
#define QN908X_FMC_SMART_CTRL_MAX_WRITE(x) \
	(((uint32_t)(((uint32_t)(x)) << QN908X_FMC_SMART_CTRL_MAX_WRITE_SHIFT)) \
	 & QN908X_FMC_SMART_CTRL_MAX_WRITE_MASK)
#define QN908X_FMC_SMART_CTRL_MAX_ERASE_MASK		0x3f000u
#define QN908X_FMC_SMART_CTRL_MAX_ERASE_SHIFT		12u
#define QN908X_FMC_SMART_CTRL_MAX_ERASE(x) \
	(((uint32_t)(((uint32_t)(x)) << QN908X_FMC_SMART_CTRL_MAX_ERASE_SHIFT)) \
	 & QN908X_FMC_SMART_CTRL_MAX_ERASE_MASK)

#define QN908X_FMC_SMART_CTRL_MAX_ERASE_TIME 49
#define QN908X_FMC_SMART_CTRL_MAX_WRITE_TIME 49

#define QN908X_FMC_TIME_CTRL_PRGM_CYCLE_MASK		0xfffu
#define QN908X_FMC_TIME_CTRL_PRGM_CYCLE_SHIFT		0u
#define QN908X_FMC_TIME_CTRL_PRGM_CYCLE(x) \
	(((uint32_t)(((uint32_t)(x)) << QN908X_FMC_TIME_CTRL_PRGM_CYCLE_SHIFT)) \
	 & QN908X_FMC_TIME_CTRL_PRGM_CYCLE_MASK)
#define QN908X_FMC_TIME_CTRL_TIME_BASE_MASK			0xff000u
#define QN908X_FMC_TIME_CTRL_TIME_BASE_SHIFT		12u
#define QN908X_FMC_TIME_CTRL_TIME_BASE(x) \
	(((uint32_t)(((uint32_t)(x)) << QN908X_FMC_TIME_CTRL_TIME_BASE_SHIFT)) \
	 & QN908X_FMC_TIME_CTRL_TIME_BASE_MASK)

#define QN908X_FMC_LOCK_STAT_8_MASS_ERASE_LOCK_EN	1u
#define QN908X_FMC_LOCK_STAT_8_FSH_PROTECT_EN		2u
#define QN908X_FMC_LOCK_STAT_8_MEM_PROTECT_EN		4u

/* Page lock information located at the beginning of the last page. */
struct qn908x_flash_page_lock {
	uint8_t bits[QN908X_FLASH_MAX_BLOCKS * QN908X_FLASH_PAGES_PER_BLOCK / 8];
	uint8_t protection;
	uint8_t _reserved[3];
	/* nvds_size is unused here, but we need to preserve it across erases
	 * when locking and unlocking pages. */
	uint8_t nvds_size[4];
} __attribute__ ((packed));

/* Clock configuration is stored in the SYSCON. */
#define QN908X_SYSCON_BASE		0x40000000u
enum qn908x_syscon_registers {
	QN908X_SYSCON_CLK_EN	= 0x00cu,
	QN908X_SYSCON_CLK_CTRL	= 0x010u,
	QN908X_SYSCON_XTAL_CTRL	= 0x180u,
};

/* Internal 16MHz / 8MHz clock used by the erase operation. */
#define QN908X_SYSCON_CLK_EN_CLK_DP_EN_MASK	(1u << 21u)

#define SYSCON_XTAL_CTRL_XTAL_DIV_MASK			0x80000000u

#define SYSCON_CLK_CTRL_AHB_DIV_MASK			0x1FFF0u
#define SYSCON_CLK_CTRL_AHB_DIV_SHIFT			4u
#define SYSCON_CLK_CTRL_CLK_XTAL_SEL_MASK		0x80000u
#define SYSCON_CLK_CTRL_CLK_OSC32M_DIV_MASK		0x100000u
#define SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK		0xC0000000u
#define SYSCON_CLK_CTRL_SYS_CLK_SEL_SHIFT		30u

#define CLOCK_16MHZ	16000000u
#define CLOCK_32MHZ	32000000u
#define CLOCK_32KHZ	32000u

/* Watchdog block registers */
#define QN908X_WDT_BASE	0x40001000u
enum qn908x_wdt_registers {
	QN908X_WDT_CTRL	= 0x08u,
	QN908X_WDT_LOCK	= 0x20u,
};

struct qn908x_flash_bank {
	/* The number of flash blocks. Initially set to zero until the flash
	 * is probed. This determines the size of the flash. */
	uint32_t num_blocks;

	uint32_t user_bank_size;
	bool calc_checksum;

	/* Whether we allow to flash an image that disables SWD access, potentially
	 * bricking the device since the image can't be reflashed from SWD. */
	bool allow_swd_disabled;

	bool page_lock_loaded;
	struct qn908x_flash_page_lock page_lock;
};

/* 500 ms timeout. */
#define QN908X_DEFAULT_TIMEOUT_MS 500

/* Forward declaration of commands. */
static int qn908x_probe(struct flash_bank *bank);
static int qn908x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count);

/* Compute the CRC32 of a small region. This is only used to compute the CRC32
 * of about 256 bytes, required to verify the "Flash information page". */
static uint32_t qn908x_crc32(uint8_t *buffer, uint32_t nbytes, uint32_t seed)
{
	uint32_t crc = seed ^ 0xffffffff;

	static uint32_t crc32_table[256];
	static bool initialized; /* false by default. */
	if (!initialized) {
		/* Initialize the CRC table and the decoding table. */
		uint32_t i, j, k;
		for (i = 0; i < 256; i++) {
			k = i;
			for (j = 0; j < 8; j++)
				k = (k & 1) ? (k >> 1) ^ 0xedb88320 : (k >> 1);
			crc32_table[i] = k;
		}
		initialized = true;
	}

	while (nbytes--)
		crc = (crc >> 8) ^ crc32_table[(crc & 0xff) ^ *buffer++];
	keep_alive();
	return crc ^ 0xffffffff;
}

/* Obtain the address of the register. This makes it easier in the future to
 * change the base address of the different blocks for other similar chips. */
#define QN908X_REG_ADDR(block, reg) \
	((QN908X_##block##_BASE) + (QN908X_##block##_##reg))

/* Update the value of a register with a mask. This helper allows to read a
 * register, modify a subset of the bits and write back the value, which is a
 * common operation when modifying only a bit filed in a register. */
static int qn908x_update_reg(struct target *target, target_addr_t reg,
		uint32_t mask, uint32_t value)
{
	uint32_t orig_value = 0;
	uint32_t new_value;
	int retval;
	if (mask != 0xffffffff) {
		/* No need to read the old value if we request a mask of 32 bits. */
		retval = target_read_u32(target, reg, &orig_value);
		if (retval != ERROR_OK) {
			LOG_DEBUG("Error reading reg at " TARGET_ADDR_FMT
				": %d", reg, retval);
			return retval;
		}
	}
	new_value = (orig_value & ~mask) | (value & mask);
	retval = target_write_u32(target, reg, new_value);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Error writing reg at " TARGET_ADDR_FMT " with 0x%08"
				PRIx32 ": %d", reg, new_value, retval);
		return retval;
	}
	if (mask == 0xffffffff) {
		LOG_DEBUG("Updated reg at " TARGET_ADDR_FMT ": ?? -> 0x%.08"
			PRIx32 "", reg, new_value);
	} else {
		LOG_DEBUG("Updated reg at " TARGET_ADDR_FMT ": 0x%.08" PRIx32
			" -> 0x%.08" PRIx32, reg, orig_value, new_value);
	}
	return ERROR_OK;
}

/* Load lock bit and protection bit and load redundancy page info.
 * This populates the LOCK_STAT_n registers with the values from the lock page,
 * making protection bit changes to the last page effective. */
static int qn908x_load_lock_stat(struct target *target)
{
	int retval = target_write_u32(target, QN908X_REG_ADDR(FMC, INI_RD_EN),
		QN908X_FMC_INI_RD_EN_INI_RD_EN_MASK);
	if (retval != ERROR_OK)
		return retval;

	uint32_t status1;
	do {
		retval = target_read_u32(target,
			QN908X_REG_ADDR(FMC, STATUS1), &status1);
		if (retval != ERROR_OK)
			return retval;
	} while ((status1 & (QN908X_FMC_STATUS1_FSH_STA_MASK |
		QN908X_FMC_STATUS1_INI_RD_DONE_MASK)) !=
		QN908X_FMC_STATUS1_INI_RD_DONE_MASK);

	for (int i = 0; i <= 8; i++) {
		uint32_t addr = QN908X_REG_ADDR(FMC, LOCK_STAT_0) + i * 4;
		uint32_t lock_stat;
		if (ERROR_OK == target_read_u32(target, addr, &lock_stat))
			LOG_DEBUG("LOCK_STAT_%d = 0x%08" PRIx32, i, lock_stat);
	}
	return ERROR_OK;
}

/* Initializes the FMC controller registers for allowing writing. */
static int qn908x_init_flash(struct target *target)
{
	/* Determine the current clock configuration. */
	uint32_t clk_ctrl;
	int retval = target_read_u32(target,
		QN908X_REG_ADDR(SYSCON, CLK_CTRL), &clk_ctrl);
	if (retval != ERROR_OK)
		return retval;

	uint32_t clk_sel = (clk_ctrl & SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK) >>
		SYSCON_CLK_CTRL_SYS_CLK_SEL_SHIFT;
	LOG_DEBUG("Clock clk_sel=0x%08" PRIu32, clk_sel);

	/* Core clock frequency. */
	uint32_t core_freq = 0;
	switch (clk_sel) {
	case 0: /* RCO 32 MHz */
		core_freq = (clk_ctrl & SYSCON_CLK_CTRL_CLK_OSC32M_DIV_MASK) ?
			CLOCK_16MHZ : CLOCK_32MHZ;
		break;
	case 1: /* Xin frequency */
		{
			uint32_t clk_xtal;
			retval = target_read_u32(target,
				QN908X_REG_ADDR(SYSCON, XTAL_CTRL), &clk_xtal);
			if (retval != ERROR_OK)
				return retval;
			core_freq = (
				clk_ctrl & SYSCON_CLK_CTRL_CLK_XTAL_SEL_MASK) &&
				(clk_xtal & SYSCON_XTAL_CTRL_XTAL_DIV_MASK) ?
				CLOCK_32MHZ : CLOCK_16MHZ;
		}
		break;
	case 2: /* 32 Kz */
		core_freq = CLOCK_32KHZ;
		break;
	default:
		return ERROR_FAIL;
	}
	uint32_t ahb_freq = core_freq / (
		((clk_ctrl & SYSCON_CLK_CTRL_AHB_DIV_MASK) >>
		 SYSCON_CLK_CTRL_AHB_DIV_SHIFT) + 1);

	LOG_DEBUG("Core freq: %" PRIu32 " Hz | AHB freq: %" PRIu32 " Hz",
		core_freq, ahb_freq);

	/* TIME_BASE is 2uS at the current AHB clock speed. */
	retval = target_write_u32(target, QN908X_REG_ADDR(FMC, TIME_CTRL),
		QN908X_FMC_TIME_CTRL_TIME_BASE(2 * ahb_freq / 1000000) |
		QN908X_FMC_TIME_CTRL_PRGM_CYCLE(30));
	if (retval != ERROR_OK)
		return retval;

	return qn908x_load_lock_stat(target);
}

/* flash bank qn908x <base> <size> 0 0 <target#> [calc_checksum] */
FLASH_BANK_COMMAND_HANDLER(qn908x_flash_bank_command)
{
	struct qn908x_flash_bank *qn908x_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (bank->base != QN908X_FLASH_BASE) {
		LOG_ERROR("Address " TARGET_ADDR_FMT
			" is an invalid bank address (try 0x%08" PRIx32")",
			bank->base, QN908X_FLASH_BASE);
		return ERROR_FAIL;
	}

	qn908x_info = malloc(sizeof(struct qn908x_flash_bank));

	bank->driver_priv = qn908x_info;
	qn908x_info->num_blocks = 0;
	qn908x_info->user_bank_size = bank->size;
	qn908x_info->page_lock_loaded = false;
	qn908x_info->allow_swd_disabled = false;

	qn908x_info->calc_checksum = false;
	if (CMD_ARGC >= 7) {
		if (strcmp(CMD_ARGV[6], "calc_checksum") == 0)
			qn908x_info->calc_checksum = true;
	}

	return ERROR_OK;
}

static int qn908x_read_page_lock(struct flash_bank *bank)
{
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The last page of the flash contains the "Flash lock and protect"
	 * information. It is not clear where this is located on chips with only
	 * one block. */
	uint32_t prot_offset =
		qn908x_info->num_blocks * QN908X_FLASH_BLOCK_SIZE -
		QN908X_FLASH_PAGE_SIZE;

	int retval = target_read_memory(bank->target,
		bank->base + prot_offset, 4,
		sizeof(qn908x_info->page_lock) / 4,
		(void *)(&qn908x_info->page_lock));
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("Flash protection = 0x%02x", qn908x_info->page_lock.protection);

	qn908x_info->page_lock_loaded = true;
	return ERROR_OK;
}

static int qn908x_busy_check(struct target *target)
{
	uint32_t status1;
	int retval = target_read_u32(target,
		QN908X_REG_ADDR(FMC, STATUS1), &status1);
	if (retval != ERROR_OK)
		return retval;

	if ((status1 & (QN908X_FMC_STATUS1_FSH_ERA_BUSY_L_MASK |
			QN908X_FMC_STATUS1_FSH_WR_BUSY_L_MASK |
			QN908X_FMC_STATUS1_FSH_ERA_BUSY_H_MASK |
			QN908X_FMC_STATUS1_FSH_WR_BUSY_H_MASK))) {
		return ERROR_FLASH_BUSY;
	}
	return ERROR_OK;
}

static int qn908x_status_check(struct target *target)
{
	uint32_t int_stat;
	int retval = target_read_u32(
		target, QN908X_REG_ADDR(FMC, INT_STAT), &int_stat);
	if (retval != ERROR_OK)
		return retval;

	if ((int_stat & ~(
		QN908X_FMC_INT_STAT_ERASEL_INT_MASK |
		QN908X_FMC_INT_STAT_WRITEL_INT_MASK |
		QN908X_FMC_INT_STAT_ERASEH_INT_MASK |
		QN908X_FMC_INT_STAT_WRITEH_INT_MASK |
		QN908X_FMC_INT_STAT_WRBUFL_INT_MASK |
		QN908X_FMC_INT_STAT_WRBUFH_INT_MASK)) == 0) {
		return ERROR_OK;
	}
	LOG_ERROR("INT_STAT status error: 0x%08" PRIx32, int_stat);
	/* TODO: return a more appropriate error based on the flag. */
	return ERROR_FAIL;
}

static int qn908x_wait_for_idle(struct target *target, int64_t timeout_ms)
{
	int64_t ms_start = timeval_ms();

	int busy = ERROR_FLASH_BUSY;
	while (busy != ERROR_OK) {
		busy = qn908x_busy_check(target);
		if (busy != ERROR_OK && busy != ERROR_FLASH_BUSY)
			return busy;
		if (timeval_ms() - ms_start > timeout_ms) {
			LOG_ERROR("Timeout waiting to be idle.");
			return ERROR_TIMEOUT_REACHED;
		}
	}
	return ERROR_OK;
}

/* Set up the chip to perform an erase (page or block) operation. */
static int qn908x_setup_erase(struct target *target)
{
	int retval;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Enable 8MHz clock. */
	retval = qn908x_update_reg(target, QN908X_REG_ADDR(SYSCON, CLK_EN),
		QN908X_SYSCON_CLK_EN_CLK_DP_EN_MASK,
		QN908X_SYSCON_CLK_EN_CLK_DP_EN_MASK);
	if (retval != ERROR_OK)
		return retval;

	/* Set ERASE_TIME to 2ms for smart erase. */
	retval = qn908x_update_reg(target, QN908X_REG_ADDR(FMC, ERASE_TIME),
		(1u << 20) - 1,
		2000 * 8); /* 2000 uS * 8 MHz = x cycles */
	if (retval != ERROR_OK)
		return retval;

	/* Set up smart erase. SWD can only perform smart erase. */
	retval = target_write_u32(target, QN908X_REG_ADDR(FMC, SMART_CTRL),
		QN908X_FMC_SMART_CTRL_SMART_ERASEH_EN_MASK |
		QN908X_FMC_SMART_CTRL_SMART_ERASEL_EN_MASK |
		QN908X_FMC_SMART_CTRL_MAX_ERASE(QN908X_FMC_SMART_CTRL_MAX_ERASE_TIME) |
		QN908X_FMC_SMART_CTRL_MAX_WRITE(QN908X_FMC_SMART_CTRL_MAX_WRITE_TIME));
	if (retval != ERROR_OK)
		return retval;

	retval = qn908x_wait_for_idle(target, QN908X_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int qn908x_erase(struct flash_bank *bank, int first, int last)
{
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;
	int retval = ERROR_OK;

	if (!qn908x_info->num_blocks) {
		if (qn908x_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	retval = qn908x_setup_erase(bank->target);
	if (retval != ERROR_OK)
		return retval;

	for (int i = first; i <= last; i++) {
		if (i < 0 || i >= bank->num_sectors)
			return ERROR_FLASH_SECTOR_INVALID;
		uint32_t block_idx = i / QN908X_FLASH_PAGES_PER_BLOCK;
		uint32_t page_idx = i % QN908X_FLASH_PAGES_PER_BLOCK;
		if (block_idx >= qn908x_info->num_blocks)
			return ERROR_FLASH_SECTOR_INVALID;

		LOG_DEBUG("Erasing page %" PRIu32 " of block %" PRIu32,
			page_idx, block_idx);

		/* Depending on the block the page we are erasing is located we
		 * need to use a different set of bits in the registers. */
		uint32_t ctrl_page_idx_shift = block_idx ?
			QN908X_FMC_ERASE_CTRL_PAGE_IDXH_SHIFT :
			QN908X_FMC_ERASE_CTRL_PAGE_IDXL_SHIFT;
		uint32_t ctrl_erase_en_shift = block_idx ?
			QN908X_FMC_ERASE_CTRL_PAGE_ERASEH_EN_SHIFT :
			QN908X_FMC_ERASE_CTRL_PAGE_ERASEL_EN_SHIFT;

		retval = target_write_u32(
			bank->target, QN908X_REG_ADDR(FMC, ERASE_CTRL),
			(1u << ctrl_erase_en_shift) | (page_idx << ctrl_page_idx_shift));
		if (retval != ERROR_OK)
			return retval;

		retval = qn908x_wait_for_idle(bank->target, QN908X_DEFAULT_TIMEOUT_MS);
		if (retval != ERROR_OK)
			return retval;

		retval = qn908x_status_check(bank->target);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int qn908x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!qn908x_info->page_lock_loaded) {
		int retval = qn908x_read_page_lock(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Use [first, last) interval open on the right side from now on. */
	last++;
	/* We use sectors as prot_blocks. */
	bool needs_update = false;
	for (int i = first; i < last; i++) {
		if (set != (((qn908x_info->page_lock.bits[i / 8] >> (i % 8)) & 1)
			^ 1)) {
			needs_update = true;
		}
	}

	/* Check that flash protection still allows SWD access to flash and RAM,
	 * otherwise we won't be able to re-flash this chip from SWD unless we do a
	 * mass erase. */
	if ((qn908x_info->page_lock.protection & (
			QN908X_FMC_LOCK_STAT_8_FSH_PROTECT_EN |
			QN908X_FMC_LOCK_STAT_8_MEM_PROTECT_EN)) != 0) {
		/* Bits 1 and 2 set mean that flash and memory protection from
		 * SWD is disabled, respectively. */
		LOG_WARNING(
			"SWD flash/RAM access disabled in the Flash lock and protect "
			"descriptor. You might need to issue a mass_erase to regain "
			"SWD access to this chip after reboot.");
	}

	if (!needs_update)
		return ERROR_OK;

	int last_page = qn908x_info->num_blocks * QN908X_FLASH_PAGES_PER_BLOCK - 1;
	int retval;

	if (qn908x_info->page_lock.bits[sizeof(qn908x_info->page_lock.bits) - 1] &
		0x80) {
		/* A bit 1 in the MSB in the page_lock.bits array means that the last
		 * page is unlocked, so we can just erase it. */
		retval = qn908x_erase(bank, last_page, last_page);
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* TODO: The last page is locked and we can't erase unless we use the
		 * ERASE_PASSWORD from code running on the device. For this we need to
		 * copy a little program to RAM and execute the erase command from there
		 * since there's no way to override the page protection from SWD. */
		LOG_ERROR(
			"Unprotecting the last page is not supported. Issue a \"qn908x "
			"mass_erase\" command to erase the whole flash, including the last "
			"page and its protection.");
		return ERROR_FAIL;
	}

	for (int i = first / 8; i < (last + 7) / 8; i++) {
		/* first_mask contains a bit set if the bit corresponds to a block id
		 * that is larger or equal than first. This is basically 0xff in all
		 * cases except potentially the first iteration. */
		uint8_t first_mask =
			first <= i * 8 ? 0xff : 0xff ^ ((1u << (first - i * 8)) - 1);
		/* Similar to first_mask, this contains a bit set if the corresponding
		 * is is smaller than last. */
		uint8_t last_mask =
			i * 8 + 8 <= last ? 0xff : ((1u << (last - i * 8)) - 1);

		uint8_t mask = first_mask & last_mask;
		LOG_DEBUG("protect set=%d bits[%d] with mask=0x%02x", set, i, mask);
		/* To "set" the protection bit means to clear the bit in the page_lock
		 * bit array. */
		if (set)
			qn908x_info->page_lock.bits[i] &= ~mask;
		else
			qn908x_info->page_lock.bits[i] |= mask;
	}

	retval = qn908x_write(bank, (void *)(&qn908x_info->page_lock),
		last_page * QN908X_FLASH_PAGE_SIZE, sizeof(qn908x_info->page_lock));
	if (retval != ERROR_OK)
		return retval;

	/* Reload the lock_stat to make the changes effective. */
	retval = qn908x_load_lock_stat(bank->target);
	if (retval != ERROR_OK)
		return retval;

	for (int i = first; i < last; i++)
		bank->sectors[i].is_protected = set;

	return ERROR_OK;
}

static int qn908x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* TODO: We can actually write not aligned to word boundary but it
	 * requires a read-modify-write operation. */
	if ((offset % 4) != 0) {
		LOG_WARNING("offset 0x%" PRIx32
			" breaks required word-alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	if ((count % 4) != 0) {
		LOG_WARNING("offset 0x%" PRIx32
			" breaks required word-alignment", count);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* Compute the calc_checksum even if it wasn't requested. */
	uint32_t checksum = 0;
	if (offset == 0 && count >= 0x20) {
		for (int i = 0; i < 7; i++)
			checksum += buf_get_u32(buffer + (i * 4), 0, 32);
		checksum = 0 - checksum;
		LOG_DEBUG("computed image checksum: 0x%8.8" PRIx32, checksum);
		uint32_t stored_checksum = buf_get_u32(buffer + 7 * 4, 0, 32);
		if (checksum != stored_checksum) {
			LOG_WARNING(
				"Image vector table checksum mismatch: expected 0x%08" PRIx32
				" but found 0x%08" PRIx32, checksum, stored_checksum);
			if (!qn908x_info->calc_checksum) {
				LOG_WARNING(
					"This device will not boot, use calc_checksum in the flash "
					"bank.");
			} else {
				LOG_WARNING("Updating checksum, verification will fail.");
			}
		}
	}

	/* Check the Code Read Protection (CRP) word for invalid values or not
	 * allowed ones. */
	if (offset <= 0x20 && offset + count >= 0x24) {
		uint32_t crp = buf_get_u32(buffer + 0x20 - offset, 0, 32);
		/* 2-bit fields at bits 10, 12, 14, 16 and 18 must not be 00 or 11. */
		for (int i = 10; i <= 18; i += 2) {
			uint32_t field = (crp >> i) & 3;
			if (field == 0 || field == 3) {
				LOG_DEBUG("Code Read Protection = 0x%08" PRIx32, crp);
				LOG_ERROR(
					"The Code Read Protection (CRP) field at bit %d is invalid "
					"(%" PRIu32 "). An invalid value could make the flash "
					"inaccessible.", i, field);
				return ERROR_FAIL;
			}
		}

		uint32_t swd_allowed = (crp >> 18) & 3;
		if (swd_allowed != 2) {
			LOG_WARNING(
				"The Code Read Protection (CRP) in this image (0x%08" PRIx32
				") is disabling the SWD access, which is currently used by "
				"OpenOCD to flash this device. After reboot, this device will "
				"not be accessible to OpenOCD anymore.", crp);
			if (!qn908x_info->allow_swd_disabled) {
				LOG_ERROR(
					"Disabling SWD is not allowed, run \"qn908x allow_brick\" "
					"before if you really want to disable SWD. You won't be "
					"able to access this chip anymore from OpenOCD.");
				return ERROR_FAIL;
			}
		}
	}

	retval = qn908x_wait_for_idle(bank->target, QN908X_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	uint32_t smart_ctrl = QN908X_FMC_SMART_CTRL_SMART_WRITEH_EN_MASK |
		QN908X_FMC_SMART_CTRL_PRGMH_EN_MASK;
	if (qn908x_info->num_blocks > 1) {
		smart_ctrl |= QN908X_FMC_SMART_CTRL_SMART_WRITEL_EN_MASK |
			QN908X_FMC_SMART_CTRL_PRGML_EN_MASK;
	}
	retval = target_write_u32(
		bank->target, QN908X_REG_ADDR(FMC, SMART_CTRL), smart_ctrl);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t BUF_SIZE = 256;
	while (count > 0) {
		uint32_t next_offset = (offset & ~(BUF_SIZE - 1)) + BUF_SIZE;
		uint32_t chunk_len = next_offset - offset;
		if (chunk_len > count)
			chunk_len = count;
		if (offset <= 0x1c && offset + chunk_len >= 0x20 &&
			qn908x_info->calc_checksum) {
			uint8_t copybuf[BUF_SIZE];
			memcpy(copybuf, buffer, chunk_len);
			buf_set_u32(copybuf + 0x1c - offset, 0, 32, checksum);
			retval = target_write_buffer(bank->target,
				bank->base + offset, chunk_len, copybuf);
		} else {
			retval = target_write_buffer(bank->target,
				bank->base + offset, chunk_len, buffer);
		}
		if (retval != ERROR_OK)
			return retval;
		keep_alive();
		buffer += chunk_len;
		count -= chunk_len;
		offset += chunk_len;
	}

	retval = qn908x_wait_for_idle(bank->target, QN908X_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = qn908x_status_check(bank->target);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int qn908x_probe(struct flash_bank *bank)
{
	int retval;
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;
	uint8_t info_page[
		QN908X_INFO_PAGE_CRC_END - QN908X_INFO_PAGE_CRC_START];
	qn908x_info->num_blocks = 0;

	/* When the SWD access to the RAM is locked by the LOCK_STAT_8 register we
	 * can't access the info page to verify the chip/bank version and it will
	 * read all zeros. This situation prevents the bank from being initialized
	 * at all so no other operation can be performed. The only option to
	 * re-flash the chip is to perform a mass_erase from SWD, which can be
	 * performed even if the mass_erase operation is locked as well.
	 * We attempt to read the info page and redirect the user to perform a
	 * mass_erase if we detect this situation. */
	retval = target_read_memory(bank->target,
		QN908X_REG_ADDR(INFO_PAGE, CRC_START),
		sizeof(uint32_t), sizeof(info_page) / sizeof(uint32_t),
		info_page);
	if (retval != ERROR_OK)
		return retval;

	uint32_t computed_crc = qn908x_crc32(info_page, sizeof(info_page), 0);
	uint32_t read_crc;
	retval = target_read_u32(bank->target, QN908X_REG_ADDR(INFO_PAGE, CRC32),
		&read_crc);
	if (retval != ERROR_OK)
		return retval;

	if (computed_crc != read_crc) {
		uint32_t info_page_or = 0;
		for (uint32_t i = 0; i < sizeof(info_page); i++)
			info_page_or |= info_page[0];
		uint32_t lock_stat_8;
		retval = target_read_u32(
			bank->target, QN908X_REG_ADDR(FMC, LOCK_STAT_8), &lock_stat_8);
		if (retval != ERROR_OK)
			return retval;
		if (info_page_or == 0 &&
			(lock_stat_8 & (QN908X_FMC_LOCK_STAT_8_FSH_PROTECT_EN |
							QN908X_FMC_LOCK_STAT_8_MEM_PROTECT_EN)) != 0) {
			LOG_ERROR(
				"The flash or memory in this chip is protected and can't be "
				"accessed from the SWD interface. However, a 'qn908x "
				"mass_erase' can erase the device and lift this protection.");
			return ERROR_FAIL;
		}

		LOG_ERROR("Flash information page CRC32 mismatch, found 0x%08"
			PRIx32 " but computed 0x%08" PRIx32
			". Flash size unknown",
			read_crc, computed_crc);
		return ERROR_FAIL;
	}

	uint32_t flash_size_fld = target_buffer_get_u32(bank->target,
		info_page +
		(QN908X_INFO_PAGE_FLASH_SIZE - QN908X_INFO_PAGE_CRC_START));

	switch (flash_size_fld) {
	case QN908X_FLASH_SIZE_512K:
		qn908x_info->num_blocks = 2;
		break;
	case QN908X_FLASH_SIZE_256K:
		qn908x_info->num_blocks = 1;
		break;
	default:
		LOG_ERROR("Unknown Flash size field: 0x%08" PRIx32,
			flash_size_fld);
		return ERROR_FAIL;
	}

	bank->size = qn908x_info->num_blocks * QN908X_FLASH_BLOCK_SIZE;

	/* The flash supports erasing and protecting individual pages. */
	bank->num_sectors = qn908x_info->num_blocks *
		QN908X_FLASH_PAGES_PER_BLOCK;
	bank->sectors = alloc_block_array(0, QN908X_FLASH_PAGE_SIZE,
		bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	retval = qn908x_init_flash(bank->target);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("Detected flash size: %d Kb", bank->size / 1024);

	return ERROR_OK;
}

static int qn908x_auto_probe(struct flash_bank *bank)
{
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;
	if (qn908x_info->num_blocks != 0)
		return ERROR_OK;
	LOG_DEBUG("auto_probe");
	return qn908x_probe(bank);
}

static int qn908x_protect_check(struct flash_bank *bank)
{
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;

	int retval = qn908x_read_page_lock(bank);
	if (retval != ERROR_OK)
		return retval;

	for (uint32_t i = 0;
		i < qn908x_info->num_blocks * QN908X_FLASH_PAGES_PER_BLOCK;
		i++) {
		/* A bit 0 in page_lock means page is locked. */
		bank->sectors[i].is_protected =
			((qn908x_info->page_lock.bits[i / 8] >> (i % 8)) & 1) ^ 1;
	}
	return ERROR_OK;
}

static int qn908x_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	uint32_t bootloader_version;
	uint8_t bluetooth[6];
	int retval;
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;

	retval = target_read_u32(bank->target,
		QN908X_REG_ADDR(INFO_PAGE, BOOTLOADER_VER),
		&bootloader_version);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_memory(bank->target,
			QN908X_REG_ADDR(INFO_PAGE, BOOTLOADER_VER), 1,
			sizeof(bluetooth), bluetooth);
	if (retval != ERROR_OK)
		return retval;

	snprintf(buf, buf_size,
		"qn908x: bdaddr:%02x:%02x:%02x:%02x:%02x:%02x bldr:0x%08" PRIx32
		" blocks:%" PRIu32, bluetooth[0], bluetooth[1], bluetooth[2],
		bluetooth[3], bluetooth[4], bluetooth[5], bootloader_version,
		qn908x_info->num_blocks);
	return ERROR_OK;
}

COMMAND_HANDLER(qn908x_handle_allow_brick_command)
{
	int retval;

	struct target *target = get_current_target(CMD_CTX);
	struct flash_bank *bank = NULL;

	retval = get_flash_bank_by_addr(target, QN908X_FLASH_BASE, true, &bank);
	if (retval != ERROR_OK)
		return retval;

	assert(bank != NULL);
	struct qn908x_flash_bank *qn908x_info = bank->driver_priv;

	LOG_WARNING("Flashing images that disable SWD in qn908x is now allowed.");
	qn908x_info->allow_swd_disabled = true;

	return ERROR_OK;
}

COMMAND_HANDLER(qn908x_handle_disable_wdog_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* To change any value in the watchdog block (WDT) we need to first write
	 * 0x1ACCE551 to the LOCK register, and we can then set it back to any other
	 * value to prevent accidental changes to the watchdog. */
	retval = target_write_u32(target, QN908X_REG_ADDR(WDT, LOCK), 0x1ACCE551);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, QN908X_REG_ADDR(WDT, CTRL), 0);
	if (retval != ERROR_OK)
		return retval;

	return target_write_u32(target, QN908X_REG_ADDR(WDT, LOCK), 0);
}

COMMAND_HANDLER(qn908x_handle_mass_erase_command)
{
	int retval;
	bool keep_lock = false;
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		if (strcmp("keep_lock", CMD_ARGV[0]))
			return ERROR_COMMAND_SYNTAX_ERROR;
		keep_lock = true;
	}

	/* This operation can be performed without probing the bank since it is the
	 * only way to unlock a chip when the flash and ram have been locked. */
	struct target *target = get_current_target(CMD_CTX);

	retval = qn908x_setup_erase(target);
	if (retval != ERROR_OK)
		return retval;

	/* Check the mass-erase locking status for information purposes only. This
	 * lock applies to both the SWD and the code running in the core but can be
	 * bypassed in either case. */
	uint32_t lock_stat_8;
	retval = target_read_u32(target, QN908X_REG_ADDR(FMC, LOCK_STAT_8),
		&lock_stat_8);
	LOG_DEBUG("LOCK_STAT_8 before erasing: 0x%" PRIx32, lock_stat_8);
	if (retval != ERROR_OK)
		return retval;
	if ((lock_stat_8 & QN908X_FMC_LOCK_STAT_8_MASS_ERASE_LOCK_EN) == 0) {
		LOG_INFO(
			"mass_erase disabled by Flash lock and protection, forcing "
			"mass_erase.");
	}
	/* Set the DEBUG_PASSWORD so we can force the mass erase from the SWD. We do
	 * this regardless of the lock status. */
	retval = target_write_u32(
		target, QN908X_REG_ADDR(FMC, DEBUG_PASSWORD), 0xCA1E093F);
	if (retval != ERROR_OK)
		return retval;

	/* Erase both halves of the flash at the same time. These are actually done
	 * sequentially but we need to send the command to erase both blocks since
	 * doing so in a locked flash will change the LOCK_STAT_8 register to 0x01,
	 * allowing us to access the (now erase) flash an memory. Erasing only one
	 * block at a time does not reset the LOCK_STAT_8 register and therefore
	 * will not grant access to program the chip. */
	uint32_t erase_cmd = (1u << QN908X_FMC_ERASE_CTRL_HALF_ERASEH_EN_SHIFT) |
		(1u << QN908X_FMC_ERASE_CTRL_HALF_ERASEL_EN_SHIFT);
	LOG_DEBUG("Erasing both blocks with command 0x%" PRIx32, erase_cmd);

	retval = target_write_u32(
		target, QN908X_REG_ADDR(FMC, ERASE_CTRL), erase_cmd);
	if (retval != ERROR_OK)
		return retval;

	retval = qn908x_wait_for_idle(target, QN908X_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	retval = qn908x_status_check(target);
	if (retval != ERROR_OK)
		return retval;

	/* Set the debug password back to 0 to avoid accidental mass_erase. */
	retval = target_write_u32(
		target, QN908X_REG_ADDR(FMC, DEBUG_PASSWORD), 0);
	if (retval != ERROR_OK)
		return retval;

	/* At this point the flash is erased and we are able to write to the flash
	 * since the LOCK_STAT_8 gets updated to 0x01 after the mass_erase. However,
	 * after a hard reboot this value will be realoaded from flash which after
	 * an erase is 0xff. This means that after flashing an image that doesn't
	 * set the protection bits we end up with a chip that we can't debug. We
	 * update this value to 0x01 unless "keep_lock" is passed to allow the SWD
	 * interface to debug the flash and RAM after a hard reset. */
	if (keep_lock)
		return retval;

	retval = qn908x_init_flash(target);
	if (retval != ERROR_OK)
		return retval;

	/* Unlock access to RAM and FLASH in the last page of the flash and
	 * reloading */
	retval = qn908x_wait_for_idle(target, QN908X_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	uint32_t smart_ctrl = QN908X_FMC_SMART_CTRL_SMART_WRITEH_EN_MASK |
		QN908X_FMC_SMART_CTRL_PRGMH_EN_MASK;
	retval = target_write_u32(
		target, QN908X_REG_ADDR(FMC, SMART_CTRL), smart_ctrl);
	if (retval != ERROR_OK)
		return retval;

	uint32_t lock_stat_8_addr = QN908X_FLASH_BASE + 0x7f820;
	retval = target_write_u32(
		target, lock_stat_8_addr, QN908X_FMC_LOCK_STAT_8_MASS_ERASE_LOCK_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = qn908x_wait_for_idle(target, QN908X_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	/* Force a page_lock reload after the mass_erase . */
	retval = qn908x_load_lock_stat(target);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static const struct command_registration qn908x_exec_command_handlers[] = {
	{
		.name		= "allow_brick",
		.handler	= qn908x_handle_allow_brick_command,
		.mode		= COMMAND_EXEC,
		.help		= "Allow writing images that disable SWD access in their "
			"Code Read Protection (CRP) word. Warning: This can make your "
			"chip inaccessible from OpenOCD or any other SWD debugger.",
		.usage		= "",
	},
	{
		.name		= "disable_wdog",
		.handler	= qn908x_handle_disable_wdog_command,
		.mode		= COMMAND_EXEC,
		.help		= "Disabled the watchdog (WDT).",
		.usage		= "",
	},
	{
		.name		= "mass_erase",
		.handler	= qn908x_handle_mass_erase_command,
		.mode		= COMMAND_EXEC,
		.help		= "Erase the whole flash chip.",
		.usage		= "[keep_lock]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration qn908x_command_handlers[] = {
	{
		.name	= "qn908x",
		.mode	= COMMAND_ANY,
		.help	= "qn908x flash controller commands",
		.usage	= "",
		.chain	= qn908x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver qn908x_flash = {
	.name				= "qn908x",
	.commands			= qn908x_command_handlers,
	.flash_bank_command	= qn908x_flash_bank_command,
	.info				= qn908x_get_info,
	.erase				= qn908x_erase,
	.protect			= qn908x_protect,
	.write				= qn908x_write,
	.read				= default_flash_read,
	.probe				= qn908x_probe,
	.auto_probe			= qn908x_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= qn908x_protect_check,
	.free_driver_priv	= default_flash_free_driver_priv,
};
