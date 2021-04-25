/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef OPENOCD_FLASH_NOR_CFI_H
#define OPENOCD_FLASH_NOR_CFI_H

#define CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7 0xE0 /* DQ5..DQ7 */
#define CFI_STATUS_POLL_MASK_DQ6_DQ7     0xC0 /* DQ6..DQ7 */

struct cfi_flash_bank {
	bool x16_as_x8;
	bool jedec_probe;
	bool not_cfi;
	bool probed;

	enum target_endianness endianness;
	bool data_swap;

	uint16_t manufacturer;
	uint16_t device_id;

	uint8_t qry[3];

	/* identification string */
	uint16_t pri_id;
	uint16_t pri_addr;
	uint16_t alt_id;
	uint16_t alt_addr;

	/* device-system interface */
	uint8_t vcc_min;
	uint8_t vcc_max;
	uint8_t vpp_min;
	uint8_t vpp_max;
	uint8_t word_write_timeout_typ;
	uint8_t buf_write_timeout_typ;
	uint8_t block_erase_timeout_typ;
	uint8_t chip_erase_timeout_typ;
	uint8_t word_write_timeout_max;
	uint8_t buf_write_timeout_max;
	uint8_t block_erase_timeout_max;
	uint8_t chip_erase_timeout_max;

	uint8_t status_poll_mask;

	/* flash geometry */
	uint32_t dev_size;
	uint16_t interface_desc;
	uint16_t max_buf_write_size;
	uint8_t num_erase_regions;
	uint32_t *erase_region_info;

	void *pri_ext;
	void *alt_ext;

	/* calculated timeouts */
	unsigned word_write_timeout;
	unsigned buf_write_timeout;
	unsigned block_erase_timeout;
	unsigned chip_erase_timeout;

	/* memory accessors */
	int (*write_mem)(struct flash_bank *bank, target_addr_t addr,
			 uint32_t count, const uint8_t *buffer);
	int (*read_mem)(struct flash_bank *bank, target_addr_t addr,
			uint32_t count, uint8_t *buffer);
};

/* Intel primary extended query table
 * as defined for the Advanced+ Boot Block Flash Memory (C3)
 * and used by the linux kernel cfi driver (as of 2.6.14)
 */
struct cfi_intel_pri_ext {
	uint8_t pri[3];
	uint8_t major_version;
	uint8_t minor_version;
	uint32_t feature_support;
	uint8_t suspend_cmd_support;
	uint16_t blk_status_reg_mask;
	uint8_t vcc_optimal;
	uint8_t vpp_optimal;
	uint8_t num_protection_fields;
	uint16_t prot_reg_addr;
	uint8_t fact_prot_reg_size;
	uint8_t user_prot_reg_size;
	uint8_t extra[0];
};

/* Spansion primary extended query table as defined for and used by
 * the linux kernel cfi driver (as of 2.6.15)
 */
struct cfi_spansion_pri_ext {
	uint8_t  pri[3];
	uint8_t  major_version;
	uint8_t  minor_version;
	uint8_t  silicon_revision; /* bits 1-0: Address Sensitive Unlock */
	uint8_t  erase_suspend;
	uint8_t  blk_prot;
	uint8_t  tmp_blk_unprotected;
	uint8_t  blk_prot_unprot;
	uint8_t  simultaneous_ops;
	uint8_t  burst_mode;
	uint8_t  page_mode;
	uint8_t  vpp_min;
	uint8_t  vpp_max;
	uint8_t  top_bottom;
	int _reversed_geometry;
	uint32_t _unlock1;
	uint32_t _unlock2;
};

/* Atmel primary extended query table as defined for and used by
 * the linux kernel cfi driver (as of 2.6.20+)
 */
struct cfi_atmel_pri_ext {
	uint8_t pri[3];
	uint8_t major_version;
	uint8_t minor_version;
	uint8_t features;
	uint8_t bottom_boot;
	uint8_t burst_mode;
	uint8_t page_mode;
};

enum {
	CFI_UNLOCK_555_2AA,
	CFI_UNLOCK_5555_2AAA,
};

struct cfi_unlock_addresses {
	uint32_t unlock1;
	uint32_t unlock2;
};

struct cfi_fixup {
	uint16_t mfr;
	uint16_t id;
	void (*fixup)(struct flash_bank *bank, const void *param);
	const void *param;
};

int cfi_erase(struct flash_bank *bank, unsigned int first, unsigned int last);
int cfi_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last);
int cfi_probe(struct flash_bank *bank);
int cfi_auto_probe(struct flash_bank *bank);
int cfi_protect_check(struct flash_bank *bank);
int cfi_get_info(struct flash_bank *bank, struct command_invocation *cmd);
int cfi_flash_bank_cmd(struct flash_bank *bank, unsigned int argc, const char **argv);

uint32_t cfi_flash_address(struct flash_bank *bank, int sector, uint32_t offset);
int cfi_spansion_unlock_seq(struct flash_bank *bank);
int cfi_send_command(struct flash_bank *bank, uint8_t cmd, uint32_t address);
int cfi_write_word(struct flash_bank *bank, uint8_t *word, uint32_t address);
int cfi_spansion_wait_status_busy(struct flash_bank *bank, int timeout);
int cfi_reset(struct flash_bank *bank);

int cfi_target_read_memory(struct flash_bank *bank, target_addr_t addr,
			   uint32_t count, uint8_t *buffer);

#define CFI_MFR_AMD		0x0001
#define CFI_MFR_FUJITSU	0x0004
#define CFI_MFR_ATMEL	0x001F
#define CFI_MFR_ST		0x0020	/* STMicroelectronics */
#define CFI_MFR_AMIC	0x0037
#define CFI_MFR_SST		0x00BF
#define CFI_MFR_MX		0x00C2
#define CFI_MFR_EON		0x007F

#define CFI_MFR_ANY		0xffff
#define CFI_ID_ANY		0xffff

#define CFI_MAX_BUS_WIDTH       4
#define CFI_MAX_CHIP_WIDTH      4

#endif /* OPENOCD_FLASH_NOR_CFI_H */
