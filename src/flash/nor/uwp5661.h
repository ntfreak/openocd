/***************************************************************************
 *   Copyright (C) 2019 by UNISOC                                          *
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

#ifndef UWP5661_H
#define UWP5661_H

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <stdlib.h>

#define min(a, b) (((a) < (b)) ? (a) : (b))

#define BIT(x) (1 << (x))

#define FUNC_3					(3<<4)
#define FUNC_MSK				(7<<4)

/* CMD_READ_STATUS1 */
#define STATUS_WIP              BIT(0)

/* CMD_READ_STATUS3 */
#define STATUS_ADS		BIT(0)

#define CONFIG_SYS_HZ                       1000
#define CACHE_CMD_TIMEOUT                   (1   * CONFIG_SYS_HZ)
#define SFC_DRVREQ_TIMEOUT                  (1   * CONFIG_SYS_HZ)
#define SPI_FLASH_ADS_TIMEOUT               (2   * CONFIG_SYS_HZ)
#define SPI_FLASH_PAGE_PROG_TIMEOUT         (20  * CONFIG_SYS_HZ)
#define SPI_FLASH_SECTOR_ERASE_TIMEOUT      (125 * CONFIG_SYS_HZ)

#define UWP5661_FLASH_BASE_ADDRESS    0x2000000

#define BASE_ICACHE_REG         0x401C0000
#define BASE_DCACHE_REG         0x401E0000
#define BASE_AON_GPIO1_REG      0x40808000
#define BASE_AON_GLB_REG        0x4083C000
#define BASE_AON_PIN_REG        0x40840000
#define BASE_AON_CLK_REG        0x40844200
#define BASE_AON_SFC_CFG        0x40890000

#define REG_ICACHE_CFG0                     (BASE_ICACHE_REG + 0x0000)
#define REG_ICACHE_CMD_CFG2                 (BASE_ICACHE_REG + 0x0058)
#define REG_ICACHE_INT_EN                   (BASE_ICACHE_REG + 0x0060)
#define REG_ICACHE_INT_RAW_STS              (BASE_ICACHE_REG + 0x0064)
#define REG_ICACHE_INT_CLR                  (BASE_ICACHE_REG + 0x006C)
#define REG_DCACHE_CFG0                     (BASE_DCACHE_REG + 0x0000)
#define REG_DCACHE_CMD_CFG2                 (BASE_DCACHE_REG + 0x0058)
#define REG_DCACHE_INT_EN                   (BASE_DCACHE_REG + 0x0060)
#define REG_DCACHE_INT_RAW_STS              (BASE_DCACHE_REG + 0x0064)
#define REG_DCACHE_INT_CLR                  (BASE_DCACHE_REG + 0x006C)

/* REG_ICACHE_INT_EN and REG_DCACHE_INT_EN */
#define RF_CMD_IRQ_ENABLE	1
#define RF_CMD_IRQ_DISABLE	0

/* REG_ICACHE_INT_CLR and REG_DCACHE_INT_CLR */
#define RF_CMD_IRQ_CLR	1

/* REG_ICACHE_CMD_CFG2 and REG_DCACHE_CMD_CFG2 */
#define RF_CMD_TYPE_INVALID_ALL			4
#define RF_CMD_TYPE_CLEAN_INVALID_ALL	8
#define RF_CMD_STR_CMD_START			(1<<31)

/* REG_DCACHE_CFG0 */
#define ALL_DISABLED	0

#define REG_AON_GPIO1_RF_GPIO_VAL           (BASE_AON_GPIO1_REG + 0x0000)
#define REG_AON_GPIO1_RF_GPIO_MSK           (BASE_AON_GPIO1_REG + 0x0004)
#define REG_AON_GPIO1_RF_GPIO_DIR           (BASE_AON_GPIO1_REG + 0x0008)
#define REG_AON_GLB_RF_GPIO_MODE1           (BASE_AON_GLB_REG + 0x020c)
#define REG_AON_GLB_RF_APB_EB_SET           (BASE_AON_GLB_REG + 0x1024)
#define REG_AON_PIN_RF_ESMCSN_CFG           (BASE_AON_PIN_REG + 0x0030)
#define REG_AON_CLK_RF_CGM_ARM_CFG          (BASE_AON_CLK_REG + 0x0020)
#define REG_AON_CLK_RF_CGM_MTX_CFG          (BASE_AON_CLK_REG + 0x0024)
#define REG_AON_CLK_RF_CGM_SFC_2X_CFG       (BASE_AON_CLK_REG + 0x0030)
#define REG_AON_CLK_RF_CGM_SFC_1X_CFG       (BASE_AON_CLK_REG + 0x0034)

/* REG_AON_CLK_RF_CGM_ARM_CFG */
#define CGM_ARM_XTAL_MHZ			0

/* REG_AON_CLK_RF_CGM_MTX_CFG */
#define CGM_MTX_DIV_1		(0<<8)
#define CGM_MTX_DIV_2		(1<<8)	/* clk_src/(bit 9:8 + 1) */

/* REG_AON_CLK_RF_CGM_SFC_2X_CFG */
#define CLK_SFC_2X_XTAL_MHZ			0
#define CLK_SFC_2X_133_MHZ			1
#define CLK_SFC_2X_139_MHZ			2
#define CLK_SFC_2X_160_MHZ			3
#define CLK_SFC_2X_208_MHZ			4

/* REG_AON_CLK_RF_CGM_SFC_1X_CFG */
#define CLK_SFC_1X_DIV_2	(1<<8)

#define SFC_CMD_CFG         (BASE_AON_SFC_CFG + 0x0000)
#define SFC_SOFT_REQ        (BASE_AON_SFC_CFG + 0x0004)
#define SFC_TBUF_CLR        (BASE_AON_SFC_CFG + 0x0008)
#define SFC_INT_CLR         (BASE_AON_SFC_CFG + 0x000C)
#define SFC_STATUS          (BASE_AON_SFC_CFG + 0x0010)
#define SFC_CS_TIMING_CFG   (BASE_AON_SFC_CFG + 0x0014)
#define SFC_RD_SAMPLE_CFG   (BASE_AON_SFC_CFG + 0x0018)
#define SFC_CLK_CFG         (BASE_AON_SFC_CFG + 0x001C)
#define SFC_CS_CFG          (BASE_AON_SFC_CFG + 0x0020)
#define SFC_ENDIAN_CFG      (BASE_AON_SFC_CFG + 0x0024)
#define SFC_IO_DLY_CFG      (BASE_AON_SFC_CFG + 0x0028)
#define SFC_WP_HLD_INIT     (BASE_AON_SFC_CFG + 0x002C)
#define SFC_CMD_BUF0        (BASE_AON_SFC_CFG + 0x0040)
#define SFC_CMD_BUF1        (BASE_AON_SFC_CFG + 0x0044)
#define SFC_CMD_BUF2        (BASE_AON_SFC_CFG + 0x0048)
#define SFC_CMD_BUF3        (BASE_AON_SFC_CFG + 0x004C)
#define SFC_CMD_BUF4        (BASE_AON_SFC_CFG + 0x0050)
#define SFC_CMD_BUF5        (BASE_AON_SFC_CFG + 0x0054)
#define SFC_CMD_BUF6        (BASE_AON_SFC_CFG + 0x0058)
#define SFC_CMD_BUF7        (BASE_AON_SFC_CFG + 0x005C)
#define SFC_CMD_BUF8        (BASE_AON_SFC_CFG + 0x0060)
#define SFC_CMD_BUF9        (BASE_AON_SFC_CFG + 0x0064)
#define SFC_CMD_BUF10       (BASE_AON_SFC_CFG + 0x0068)
#define SFC_CMD_BUF11       (BASE_AON_SFC_CFG + 0x006C)
#define SFC_TYPE_BUF0       (BASE_AON_SFC_CFG + 0x0070)
#define SFC_TYPE_BUF1       (BASE_AON_SFC_CFG + 0x0074)
#define SFC_TYPE_BUF2       (BASE_AON_SFC_CFG + 0x0078)

#define SFC_IEN             (BASE_AON_SFC_CFG + 0x0204)
#define SFC_INT_RAW         (BASE_AON_SFC_CFG + 0x0208)
#define SFC_INT_STS         (BASE_AON_SFC_CFG + 0x020C)

/* SFC_CMD_CFG */
#define SHIFT_CMD_SET                   0
#define SHIFT_RDATA_BIT_MODE            1
#define SHIFT_STS_INI_ADDR_SEL          3
#define MSK_STS_INI_ADDR_SEL            (3<<3)

/* SFC_SOFT_REQ */
#define SHIFT_SOFT_REQ                  0

/* SFC_INT_CLR */
#define SHIFT_INT_CLR                   0

/* SFC_CLK_CFG */
#define SFC_CLK_OUT_DIV_2               BIT(0)
#define SFC_CLK_SAMPLE_2X_EN            BIT(7)
#define SFC_CLK_SAMPLE_2X_PHASE_1       BIT(8)
#define SFC_CLK_OUT_2X_EN               BIT(9)
#define SFC_CLK_2X_EN                   BIT(10)

/* SFC_TYPE_BUF0 */
#define VALID0                          BIT(0)
#define SHIFT_BIT_MODE0                 1
#define SHIFT_BYTE_NUM0                 3
#define SHIFT_OPERATION_STATUS0         5
#define SHIFT_BYTE_SEND_MODE0           7

#define TYPE_BUF_DEFAULT_VALUE			0

/* SFC_IEN */
#define CS1_INT 0x000000FF
#define CS0_INT 0x00000000


#define CMD_READ_ID             0x9f
#define CMD_RSTEN               0x66
#define CMD_RST                 0x99
#define CMD_ENTER_QPI           0x38
#define CMD_EXIT_QPI            0xFF
#define CMD_WRITE_DISABLE       0x04
#define CMD_WRITE_ENABLE        0x06
#define CMD_NORMAL_READ         0x03
#define CMD_FAST_READ           0x0B

#define CMD_ENTER_4ADDR         0xB7
#define CMD_EXIT_4ADDR          0xE9

#define CMD_WRITE_STATUS        0x01
#define CMD_READ_STATUS1        0x05
#define CMD_READ_STATUS2        0x35
#define CMD_READ_STATUS3        0x15

#define CMD_PAGE_PROGRAM        0x02

#define CMD_SECTOR_ERASE        0x20
#define CMD_CHIP_ERASE          0xC7

enum cmd_mode {
	CMD_MODE_WRITE = 0,
	CMD_MODE_READ,
	CMD_MODE_HIGHZ,
	CMD_MODE_MAX
};

enum send_mode {
	SEND_MODE_0 = 0,
	SEND_MODE_1,
	SEND_MODE_MAX
};

enum bit_mode {
	BIT_MODE_1 = 0,
	BIT_MODE_2,
	BIT_MODE_4,
	BIT_MODE_MAX
};

enum byte_num {
	BYTE_NUM_1 = 0,
	BYTE_NUM_2,
	BYTE_NUM_3,
	BYTE_NUM_4,
	BYTE_NUM_MAX
};

enum ini_add_sel {
	INI_CMD_BUF_7 = 0,
	INI_CMD_BUF_6,
	INI_CMD_BUF_5,
	INI_CMD_BUF_4,
	INI_CMD_BUF_MAX
};

enum cmd_buf_index {
	CMD_BUF_0 = 0,
	CMD_BUF_1,
	CMD_BUF_2,
	CMD_BUF_3,
	CMD_BUF_4,
	CMD_BUF_5,
	CMD_BUF_6,
	CMD_BUF_7,
	CMD_BUF_8,
	CMD_BUF_9,
	CMD_BUF_10,
	CMD_BUF_11,
	CMD_BUF_MAX
};

enum info_buf_index {
	INFO_BUF_0 = CMD_BUF_MAX,
	INFO_BUF_1,
	INFO_BUF_2,
	INFO_BUF_MAX
};

enum read_cmd_type {
	READ_SPI = 0,
	READ_SPI_FAST,
	READ_SPI_2IO,
	READ_SPI_4IO,
	READ_QPI_FAST,
	READ_QPI_4IO,
};

struct sfc_cmd_des {
	uint32_t cmd;
	uint32_t cmd_byte_len;
	enum cmd_mode cmd_mode;
	enum bit_mode bit_mode;
	enum send_mode send_mode;
};

struct uwp_flash {
	uint32_t page_size;
	uint32_t sector_size;
	uint8_t support_4addr;
};

struct uwp5661_flash_bank {
	struct uwp_flash flash;
	uint32_t cmd_info_buf_cache[INFO_BUF_MAX];
	uint32_t prev_cmd_info_buf_cache[INFO_BUF_MAX];
	uint32_t cmd_buf_cache_bitmap;
	uint32_t sfc_cmd_cfg_cache;
};

#endif /* UWP5661_H */
