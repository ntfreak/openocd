/***************************************************************************
 *   Copyright (C) 2013-2014,2019 Synopsys, Inc.                           *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Mischa Jonker <mischa.jonker@synopsys.com>                            *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#define ARC_TRANSACTION_CMD_REG 	0x9 /* Command to perform */
#define ARC_TRANSACTION_CMD_REG_LENGTH 4

/* Jtag status register, value is placed in IR to read jtag status register */
#define ARC_JTAG_STATUS_REG   0x8
/* Jtag status register field */
#define ARC_JTAG_STAT_ST		0x1
#define ARC_JTAG_STAT_FL		0x2
#define ARC_JTAG_STAT_RD		0x4
#define ARC_JTAG_STAT_NIU		0x8 /* Not in use */
#define ARC_JTAG_STAT_RU		0x10
#define ARC_JTAG_STAT_RA		0x20

#define ARC_IDCODE_REG        0xC /* ARC core type information */
#define ARC_ADDRESS_REG				0xA /* SoC address to access */
#define ARC_DATA_REG				0xB /* Data read/written from SoC */



typedef enum arc_jtag_transaction {
	ARC_JTAG_WRITE_TO_MEMORY = 0x0,
	ARC_JTAG_WRITE_TO_CORE_REG = 0x1,
	ARC_JTAG_WRITE_TO_AUX_REG = 0x2,
	ARC_JTAG_CMD_NOP = 0x3,
	ARC_JTAG_READ_FROM_MEMORY = 0x4,
	ARC_JTAG_READ_FROM_CORE_REG = 0x5,
	ARC_JTAG_READ_FROM_AUX_REG = 0x6,
} arc_jtag_transaction_t;

#ifndef ARC_JTAG_H
#define ARC_JTAG_H

struct arc_jtag {
	struct jtag_tap *tap;
	uint32_t tap_end_state;
	uint32_t intest_instr;
	uint32_t cur_trans;

	uint32_t scann_size;
	uint32_t scann_instr;
	uint32_t cur_scan_chain;

	uint32_t dpc; /* Debug PC value */

	int fast_access_save;
	bool always_check_status_rd;
	bool check_status_fl;
	bool wait_until_write_finished;
};

/* ----- Exported JTAG functions ------------------------------------------- */

int arc_jtag_startup(struct arc_jtag *jtag_info);
int arc_jtag_shutdown(struct arc_jtag *jtag_info);
int arc_jtag_status(struct arc_jtag *const jtag_info, uint32_t *const value);
int arc_jtag_idcode(struct arc_jtag *const jtag_info, uint32_t *const value);

int arc_jtag_write_core_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, const uint32_t *buffer);
int arc_jtag_read_core_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, uint32_t *buffer);
int arc_jtag_write_core_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	const uint32_t buffer);
int arc_jtag_read_core_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t *buffer);

int arc_jtag_write_aux_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, const uint32_t* buffer);
int arc_jtag_write_aux_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t value);
int arc_jtag_read_aux_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, uint32_t *buffer);
int arc_jtag_read_aux_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t *value);

int arc_jtag_write_memory(struct arc_jtag *jtag_info, uint32_t addr,
		uint32_t count, const uint32_t* buffer);
int arc_jtag_read_memory(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t count, uint32_t *buffer, bool slow_memory);
#endif /* ARC_JTAG_H */
