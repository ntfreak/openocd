/***************************************************************************
 *   Copyright (C) 2013-2014,2019-2020 Synopsys, Inc.                      *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Mischa Jonker <mischa.jonker@synopsys.com>                            *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>                                    *
 *                                                                         *
 *   SPDX-License-Identifier: GPL-2.0-or-later                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARC_JTAG_H
#define OPENOCD_TARGET_ARC_JTAG_H

#define ARC_TRANSACTION_CMD_REG		0x9 /* Command to perform */
#define ARC_TRANSACTION_CMD_REG_LENGTH	4

/* Jtag status register, value is placed in IR to read jtag status register */
#define ARC_JTAG_STATUS_REG		0x8
#define ARC_JTAG_ADDRESS_REG		0xA /* SoC address to access */
#define ARC_JTAG_DATA_REG		0xB /* Data read/written from SoC */

/* Jtag status register field */
#define ARC_JTAG_STAT_RU		0x10

/* ARC Jtag transactions */
#define ARC_JTAG_WRITE_TO_MEMORY	0x0
#define ARC_JTAG_WRITE_TO_CORE_REG	0x1
#define ARC_JTAG_WRITE_TO_AUX_REG	0x2
#define ARC_JTAG_CMD_NOP		0x3
#define ARC_JTAG_READ_FROM_MEMORY	0x4
#define ARC_JTAG_READ_FROM_CORE_REG	0x5
#define ARC_JTAG_READ_FROM_AUX_REG	0x6

#define ARC_JTAG_CORE_REG		0x0
#define ARC_JTAG_AUX_REG		0x1


struct arc_jtag {
	struct jtag_tap *tap;
	uint32_t cur_trans;
};

/* ----- Exported JTAG functions ------------------------------------------- */

int arc_jtag_startup(struct arc_jtag *jtag_info);
int arc_jtag_status(struct arc_jtag *const jtag_info, uint32_t *const value);

int arc_jtag_write_core_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, const uint32_t *buffer);
int arc_jtag_read_core_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, uint32_t *buffer);
int arc_jtag_write_core_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	const uint32_t buffer);
int arc_jtag_read_core_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t *buffer);

int arc_jtag_write_aux_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, const uint32_t *buffer);
int arc_jtag_write_aux_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t value);
int arc_jtag_read_aux_reg(struct arc_jtag *jtag_info, uint32_t *addr,
	uint32_t count, uint32_t *buffer);
int arc_jtag_read_aux_reg_one(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t *value);

int arc_jtag_write_memory(struct arc_jtag *jtag_info, uint32_t addr,
		uint32_t count, const uint32_t *buffer);
int arc_jtag_read_memory(struct arc_jtag *jtag_info, uint32_t addr,
	uint32_t count, uint32_t *buffer, bool slow_memory);
#endif /* OPENOCD_TARGET_ARC_JTAG_H */
