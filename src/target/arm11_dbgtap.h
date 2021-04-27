/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
 *   Michael Bruck                                                         *
 *                                                                         *
 *   Copyright (C) 2008,2009 Oyvind Harboe oyvind.harboe@zylin.com         *
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

#ifndef OPENOCD_TARGET_ARM11_DBGTAP_H
#define OPENOCD_TARGET_ARM11_DBGTAP_H

#include "arm11.h"

/* ARM11 internals */

void arm11_setup_field(struct arm11_common *arm11, int num_bits,
		void *in_data, void *out_data, struct scan_field *field);
void arm11_add_ir(struct arm11_common *arm11,
		uint8_t instr, tap_state_t state);
int arm11_add_debug_scan_n(struct arm11_common *arm11,
		uint8_t chain, tap_state_t state);
int arm11_read_dscr(struct arm11_common *arm11);
int arm11_write_dscr(struct arm11_common *arm11, uint32_t dscr);

int arm11_run_instr_data_prepare(struct arm11_common *arm11);
int arm11_run_instr_data_finish(struct arm11_common *arm11);
int arm11_run_instr_no_data1(struct arm11_common *arm11, uint32_t opcode);
int arm11_run_instr_data_to_core(struct arm11_common *arm11,
		uint32_t opcode, uint32_t *data, size_t count);
int arm11_run_instr_data_to_core_noack(struct arm11_common *arm11,
		uint32_t opcode, uint32_t *data, size_t count);
int arm11_run_instr_data_to_core1(struct arm11_common *arm11,
		uint32_t opcode, uint32_t data);
int arm11_run_instr_data_from_core(struct arm11_common *arm11,
		uint32_t opcode, uint32_t *data, size_t count);
int arm11_run_instr_data_from_core_via_r0(struct arm11_common *arm11,
		uint32_t opcode, uint32_t *data);
int arm11_run_instr_data_to_core_via_r0(struct arm11_common *arm11,
		uint32_t opcode, uint32_t data);

void arm11_add_dr_scan_vc(struct jtag_tap *tap, int num_fields, struct scan_field *fields,
		tap_state_t state);

/**
 * Used with arm11_sc7_run to make a list of read/write commands for
 * scan chain 7
 */
struct arm11_sc7_action {
	bool write;	/**< Access mode: true for write, false for read. */
	uint8_t address;/**< Register address mode. Use enum #arm11_sc7 */
	/**
	 * If write then set this to value to be written.  In read mode
	 * this receives the read value when the function returns.
	 */
	uint32_t value;
};

int arm11_sc7_run(struct arm11_common *arm11,
		struct arm11_sc7_action *actions, size_t count);

/* Mid-level helper functions */
int arm11_sc7_clear_vbw(struct arm11_common *arm11);
int arm11_sc7_set_vcr(struct arm11_common *arm11, uint32_t value);

int arm11_read_memory_word(struct arm11_common *arm11,
		uint32_t address, uint32_t *result);

int arm11_dpm_init(struct arm11_common *arm11, uint32_t didr);
void arm11_dpm_deinit(struct arm11_common *arm11);
int arm11_bpwp_flush(struct arm11_common *arm11);

#endif /* OPENOCD_TARGET_ARM11_DBGTAP_H */
