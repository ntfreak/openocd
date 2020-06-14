/***************************************************************************
 *   Copyright (C) 2005, 2006 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#ifndef OPENOCD_TARGET_EMBEDDEDICE_H
#define OPENOCD_TARGET_EMBEDDEDICE_H

#include "arm7_9_common.h"

enum {
	EICE_DBG_CTRL = 0,
	EICE_DBG_STAT = 1,
	EICE_COMMS_CTRL = 2,
	EICE_COMMS_DATA = 3,
	EICE_W0_ADDR_VALUE = 4,
	EICE_W0_ADDR_MASK = 5,
	EICE_W0_DATA_VALUE  = 6,
	EICE_W0_DATA_MASK = 7,
	EICE_W0_CONTROL_VALUE = 8,
	EICE_W0_CONTROL_MASK = 9,
	EICE_W1_ADDR_VALUE = 10,
	EICE_W1_ADDR_MASK = 11,
	EICE_W1_DATA_VALUE = 12,
	EICE_W1_DATA_MASK = 13,
	EICE_W1_CONTROL_VALUE = 14,
	EICE_W1_CONTROL_MASK = 15,
	EICE_VEC_CATCH = 16
};

enum {
	EICE_DBG_CONTROL_ICEDIS = 5,
	EICE_DBG_CONTROL_MONEN = 4,
	EICE_DBG_CONTROL_INTDIS = 2,
	EICE_DBG_CONTROL_DBGRQ = 1,
	EICE_DBG_CONTROL_DBGACK = 0,
};

enum {
	EICE_DBG_STATUS_IJBIT = 5,
	EICE_DBG_STATUS_ITBIT = 4,
	EICE_DBG_STATUS_SYSCOMP = 3,
	EICE_DBG_STATUS_IFEN = 2,
	EICE_DBG_STATUS_DBGRQ = 1,
	EICE_DBG_STATUS_DBGACK = 0
};

enum {
	EICE_W_CTRL_ENABLE = 0x100,
	EICE_W_CTRL_RANGE = 0x80,
	EICE_W_CTRL_CHAIN = 0x40,
	EICE_W_CTRL_EXTERN = 0x20,
	EICE_W_CTRL_nTRANS = 0x10,
	EICE_W_CTRL_nOPC = 0x8,
	EICE_W_CTRL_MAS = 0x6,
	EICE_W_CTRL_ITBIT = 0x2,
	EICE_W_CTRL_nRW = 0x1
};

enum {
	EICE_COMM_CTRL_WBIT = 1,
	EICE_COMM_CTRL_RBIT = 0
};

struct embeddedice_reg {
	int addr;
	struct arm_jtag *jtag_info;
};

struct reg_cache *embeddedice_build_reg_cache(struct target *target,
		struct arm7_9_common *arm7_9);
void embeddedice_free_reg_cache(struct reg_cache *reg_cache);

int embeddedice_setup(struct target *target);

int embeddedice_read_reg(struct reg *reg);
int embeddedice_read_reg_w_check(struct reg *reg,
		uint8_t *check_value, uint8_t *check_mask);

void embeddedice_write_reg(struct reg *reg, uint32_t value);
void embeddedice_store_reg(struct reg *reg);

void embeddedice_set_reg(struct reg *reg, uint32_t value);

int embeddedice_receive(struct arm_jtag *jtag_info, uint32_t *data, uint32_t size);
int embeddedice_send(struct arm_jtag *jtag_info, uint32_t *data, uint32_t size);

int embeddedice_handshake(struct arm_jtag *jtag_info, int hsbit, uint32_t timeout);

/* If many embeddedice_write_reg() follow eachother, then the >1 invocations can be
 * this faster version of embeddedice_write_reg
 */
static inline void embeddedice_write_reg_inner(struct jtag_tap *tap, int reg_addr, uint32_t value)
{
	uint8_t out_reg_addr = (1 << 5) | reg_addr;
	uint8_t out_value[4];
	buf_set_u32(out_value, 0, 32, value);

	struct scan_field fields[2] = {
			{ .num_bits = 32, .out_value = out_value },
			{ .num_bits = 6, .out_value = &out_reg_addr },
	};

	jtag_add_dr_scan(tap, 2, fields, TAP_IDLE);
}

void embeddedice_write_dcc(struct jtag_tap *tap, int reg_addr, const uint8_t *buffer,
		int little, int count);

#endif /* OPENOCD_TARGET_EMBEDDEDICE_H */
