/***************************************************************************
 *   Copyright (C) 2020 by Nuvoton Technology Corporation                  *
 *   Mulin Chao <mlchao@nuvoton.com>                                       *
 *   Wealian Liao <WHLIAO@nuvoton.com>                                     *
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

#ifndef OPENOCD_LOADERS_FLASH_NPCX_NPCX_FLASH_H
#define OPENOCD_LOADERS_FLASH_NPCX_NPCX_FLASH_H

/******************************************************************************/
/* Bit functions */
#define SET_BIT(reg, bit)           ((reg) |= (0x1 << (bit)))
#define CLEAR_BIT(reg, bit)         ((reg) &= (~(0x1 << (bit))))
#define IS_BIT_SET(reg, bit)        (((reg) >> (bit)) & (0x1))

/* Field functions */
#define GET_POS_FIELD(pos, size)    (pos)
#define GET_SIZE_FIELD(pos, size)   (size)
#define FIELD_POS(field)            GET_POS_##field
#define FIELD_SIZE(field)           GET_SIZE_##field
/* Read field functions */
#define GET_FIELD(reg, field) \
	_GET_FIELD_((reg), FIELD_POS(field), FIELD_SIZE(field))
#define _GET_FIELD_(reg, f_pos, f_size) \
	(((reg) >> (f_pos)) & ((1 << (f_size)) - 1))
/* Write field functions */
#define SET_FIELD(reg, field, value) \
	_SET_FIELD_((reg), FIELD_POS(field), FIELD_SIZE(field), (value))
#define _SET_FIELD_(reg, f_pos, f_size, value) \
	((reg) = ((reg) & (~(((1 << (f_size)) - 1) << (f_pos)))) | ((value) << (f_pos)))

/* Register definitions */
#define REG32_ADDR(addr)            ((volatile uint32_t *)(addr))
#define REG16_ADDR(addr)            ((volatile uint16_t *)(addr))
#define REG8_ADDR(addr)             ((volatile uint8_t  *)(addr))

#define HW_BYTE(addr)               (*REG8_ADDR(addr))
#define HW_WORD(addr)               (*REG16_ADDR(addr))
#define HW_DWORD(addr)              (*REG32_ADDR(addr))

/* devalt */
#define NPCX_SCFG_BASE_ADDR         0x400C3000
#define NPCX_DEVCNT                 HW_BYTE(NPCX_SCFG_BASE_ADDR + 0x000)
#define NPCX_DEVALT(n)	            HW_BYTE(NPCX_SCFG_BASE_ADDR + 0x010 + (n))

#define NPCX_DEVCNT_HIF_TYP_SEL_FIELD    FIELD(2, 2)
#define NPCX_DEVCNT_JEN0_HEN             4
#define NPCX_DEVCNT_JEN1_HEN             5
#define NPCX_DEVCNT_F_SPI_TRIS           6

/* pin-mux for SPI/FIU */
#define NPCX_DEVALT0_SPIP_SL             0
#define NPCX_DEVALT0_GPIO_NO_SPIP        3
#define NPCX_DEVALT0_F_SPI_CS1_2         4
#define NPCX_DEVALT0_F_SPI_CS1_1         5
#define NPCX_DEVALT0_F_SPI_QUAD          6
#define NPCX_DEVALT0_NO_F_SPI            7


/* Flash Interface Unit (FIU) registers */
#define FIU_BASE_ADDR               0x40020000
#define FIU_CFG                     HW_BYTE(FIU_BASE_ADDR + 0x000)
#define BURST_CFG                   HW_BYTE(FIU_BASE_ADDR + 0x001)
#define RESP_CFG                    HW_BYTE(FIU_BASE_ADDR + 0x002)
#define SPI_FL_CFG                  HW_BYTE(FIU_BASE_ADDR + 0x014)
#define UMA_CODE                    HW_BYTE(FIU_BASE_ADDR + 0x016)
#define UMA_AB0                     HW_BYTE(FIU_BASE_ADDR + 0x017)
#define UMA_AB1                     HW_BYTE(FIU_BASE_ADDR + 0x018)
#define UMA_AB2                     HW_BYTE(FIU_BASE_ADDR + 0x019)
#define UMA_DB0                     HW_BYTE(FIU_BASE_ADDR + 0x01A)
#define UMA_DB1                     HW_BYTE(FIU_BASE_ADDR + 0x01B)
#define UMA_DB2                     HW_BYTE(FIU_BASE_ADDR + 0x01C)
#define UMA_DB3                     HW_BYTE(FIU_BASE_ADDR + 0x01D)
#define UMA_CTS                     HW_BYTE(FIU_BASE_ADDR + 0x01E)
#define UMA_ECTS                    HW_BYTE(FIU_BASE_ADDR + 0x01F)
#define UMA_DB0_3                  HW_DWORD(FIU_BASE_ADDR + 0x020)
#define FIU_RD_CMD                  HW_BYTE(FIU_BASE_ADDR + 0x030)
#define FIU_DMM_CYC                 HW_BYTE(FIU_BASE_ADDR + 0x032)
#define FIU_EXT_CFG                 HW_BYTE(FIU_BASE_ADDR + 0x033)
#define FIU_UMA_AB0_3              HW_DWORD(FIU_BASE_ADDR + 0x034)

/* FIU register fields */
#define RESP_CFG_IAD_EN             0
#define RESP_CFG_DEV_SIZE_EX        2
#define UMA_CTS_A_SIZE              3
#define UMA_CTS_C_SIZE              4
#define UMA_CTS_RD_WR               5
#define UMA_CTS_DEV_NUM             6
#define UMA_CTS_EXEC_DONE           7
#define UMA_ECTS_SW_CS0             0
#define UMA_ECTS_SW_CS1             1
#define UMA_ECTS_SEC_CS             2
#define UMA_ECTS_UMA_LOCK           3

/* Flash UMA commands for npcx internal SPI flash */
#define CMD_READ_ID                 0x9F
#define CMD_READ_MAN_DEV_ID         0x90
#define CMD_WRITE_EN                0x06
#define CMD_WRITE_STATUS            0x50
#define CMD_READ_STATUS_REG         0x05
#define CMD_READ_STATUS_REG2        0x35
#define CMD_WRITE_STATUS_REG        0x01
#define CMD_FLASH_PROGRAM           0x02
#define CMD_SECTOR_ERASE            0x20
#define CMD_PROGRAM_UINT_SIZE       0x08
#define CMD_PAGE_SIZE               0x00
#define CMD_READ_ID_TYPE            0x47
#define CMD_FAST_READ               0x0B
#define CMD_CHIP_ERASE              0xC7

/*
 * Status registers for SPI flash
 */
#define SPI_FLASH_SR2_SUS               (1 << 7)
#define SPI_FLASH_SR2_CMP               (1 << 6)
#define SPI_FLASH_SR2_LB3               (1 << 5)
#define SPI_FLASH_SR2_LB2               (1 << 4)
#define SPI_FLASH_SR2_LB1               (1 << 3)
#define SPI_FLASH_SR2_QE                (1 << 1)
#define SPI_FLASH_SR2_SRP1              (1 << 0)
#define SPI_FLASH_SR1_SRP0              (1 << 7)
#define SPI_FLASH_SR1_SEC               (1 << 6)
#define SPI_FLASH_SR1_TB                (1 << 5)
#define SPI_FLASH_SR1_BP2               (1 << 4)
#define SPI_FLASH_SR1_BP1               (1 << 3)
#define SPI_FLASH_SR1_BP0               (1 << 2)
#define SPI_FLASH_SR1_WEL               (1 << 1)
#define SPI_FLASH_SR1_BUSY              (1 << 0)

#define MASK_CMD_ONLY      (0xC0)
#define MASK_CMD_ADR       (0xC0 | 0x08)
#define MASK_CMD_ADR_WR    (0xC0 | 0x20 | 0x08 | 0x01)
#define MASK_RD_1BYTE      (0xC0 | 0x10 | 0x01)
#define MASK_RD_2BYTE      (0xC0 | 0x10 | 0x02)
#define MASK_RD_3BYTE      (0xC0 | 0x10 | 0x03)
#define MASK_RD_4BYTE      (0xC0 | 0x10 | 0x04)
#define MASK_CMD_RD_1BYTE  (0xC0 | 0x01)
#define MASK_CMD_RD_2BYTE  (0xC0 | 0x02)
#define MASK_CMD_RD_3BYTE  (0xC0 | 0x03)
#define MASK_CMD_RD_4BYTE  (0xC0 | 0x04)
#define MASK_CMD_WR_ONLY   (0xC0 | 0x20)
#define MASK_CMD_WR_1BYTE  (0xC0 | 0x20 | 0x10 | 0x01)
#define MASK_CMD_WR_2BYTE  (0xC0 | 0x20 | 0x10 | 0x02)
#define MASK_CMD_WR_ADR    (0xC0 | 0x20 | 0x08)


#define FLASH_ABORT_TIMEOUT           0xFFFFFF
#define CONFIG_FLASH_WRITE_IDEAL_SIZE 256L   /* one page size for write */


/* flash feature */
#define NPCX_MONITOR_FLASH_ERASE_SIZE	0x1000


/* flash loader paremeters */
struct __attribute__((__packed__)) flash_params {
	uint32_t dest;     /* destination address in flash */
	uint32_t len;      /* number of bytes */
	uint32_t cmd;      /* command */
	uint32_t full;     /* handshake signal */
};

/* flash trigger signal */
typedef enum {
	LOADER_WAIT    = 0x0,       /* Idle */
	LOADER_EXECUTE = 0xFFFFFFFF /* Execute Command */
} npcx_flash_handshake_t;

/* flash loader command */
typedef enum {
	CMD_NO_ACTION = 0, /* No action, default value */
	CMD_GET_FLASH_ID,  /* Get the internal flash ID */
	CMD_ERASE_SECTORS, /* Erase unprotected sectors */
	CMD_ERASE_ALL,     /* Erase all */
	CMD_PROGRAM,       /* Program data */
} flash_commands_t;

/* status */
typedef enum {
	STATUS_OK = 0,
	STATUS_FAILED_UNKNOWN_COMMAND,
	STATUS_FAILED,
	STATUS_FAILED_TIMEOUT,
} flash_status_t;

#endif /* OPENOCD_LOADERS_FLASH_NPCX_NPCX_FLASH_H */
