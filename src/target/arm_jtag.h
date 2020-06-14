/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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

#ifndef OPENOCD_TARGET_ARM_JTAG_H
#define OPENOCD_TARGET_ARM_JTAG_H

#include <jtag/jtag.h>

struct arm_jtag {
	struct jtag_tap *tap;

	uint32_t scann_size;
	uint32_t scann_instr;
	uint32_t cur_scan_chain;

	uint32_t intest_instr;
};

int arm_jtag_set_instr_inner(struct jtag_tap *tap, uint32_t new_instr,
		void *no_verify_capture,
		tap_state_t end_state);

static inline int arm_jtag_set_instr(struct jtag_tap *tap,
		uint32_t new_instr, void *no_verify_capture, tap_state_t end_state)
{
	/* inline most common code path */
	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr)
		return arm_jtag_set_instr_inner(tap, new_instr, no_verify_capture, end_state);

	return ERROR_OK;

}

int arm_jtag_scann_inner(struct arm_jtag *jtag_info, uint32_t new_scan_chain, tap_state_t end_state);
static inline int arm_jtag_scann(struct arm_jtag *jtag_info, uint32_t new_scan_chain, tap_state_t end_state)
{
	/* inline most common code path */
	int retval = ERROR_OK;
	if (jtag_info->cur_scan_chain != new_scan_chain)
		return arm_jtag_scann_inner(jtag_info, new_scan_chain, end_state);

	return retval;
}

int arm_jtag_setup_connection(struct arm_jtag *jtag_info);
int arm_jtag_close_connection(struct arm_jtag *jtag_info);

/* use this as a static so we can inline it in -O3 and refer to it via a pointer  */
static inline void arm7flip32(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)arg) = flip_u32(le_to_h_u32(in), 32);
}

static inline void arm_le_to_h_u32(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)arg) = le_to_h_u32(in);
}

#endif /* OPENOCD_TARGET_ARM_JTAG_H */
