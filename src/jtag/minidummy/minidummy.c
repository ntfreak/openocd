/***************************************************************************
 *   Copyright (C) 2007-2008 by Øyvind Harboe                              *
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

#include <jtag/jtag.h>
#include <target/embeddedice.h>
#include <jtag/minidriver.h>
#include <jtag/interface.h>

static struct jtag_interface minidummy_interface = {
	.execute_queue = NULL,
};

struct adapter_driver minidummy_adapter_driver = {
	.name = "minidummy",
	.transports = jtag_only,
	.commands = NULL,

	.init = NULL,
	.quit = NULL,
	.speed = NULL,
	.khz = NULL,
	.speed_div = NULL,
	.power_dropout = NULL,
	.srst_asserted = NULL,

	.jtag_ops = &minidummy_interface,
};

int interface_jtag_execute_queue(void)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_ir_scan(struct jtag_tap *active, const struct scan_field *fields,
		tap_state_t state)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_plain_ir_scan(int num_bits, const uint8_t *out_bits,
		uint8_t *in_bits, tap_state_t state)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_dr_scan(struct jtag_tap *active, int num_fields,
		const struct scan_field *fields, tap_state_t state)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_plain_dr_scan(int num_bits, const uint8_t *out_bits,
		uint8_t *in_bits, tap_state_t state)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_tlr(void)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_reset(int req_trst, int req_srst)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_runtest(int num_cycles, tap_state_t state)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_clocks(int num_cycles)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_jtag_add_sleep(uint32_t us)
{
	jtag_sleep(us);
	return ERROR_OK;
}

int interface_jtag_add_pathmove(int num_states, const tap_state_t *path)
{
	int state_count;
	int tms = 0;

	state_count = 0;

	tap_state_t cur_state = cmd_queue_cur_state;

	while (num_states) {
		if (tap_state_transition(cur_state, false) == path[state_count])
			tms = 0;
		else if (tap_state_transition(cur_state, true) == path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
					tap_state_name(cur_state), tap_state_name(path[state_count]));
			exit(-1);
		}

		/* synchronously do the operation here */

		cur_state = path[state_count];
		state_count++;
		num_states--;
	}


	/* synchronously do the operation here */

	return ERROR_OK;
}

int interface_add_tms_seq(unsigned num_bits, const uint8_t *seq, enum tap_state state)
{
	/* synchronously do the operation here */

	return ERROR_OK;
}

void embeddedice_write_dcc(struct jtag_tap *tap, int reg_addr, const uint8_t *buffer,
		int little, int count)
{
	int i;
	for (i = 0; i < count; i++) {
		embeddedice_write_reg_inner(tap, reg_addr, fast_target_buffer_get_u32(buffer, little));
		buffer += 4;
	}
}

int arm11_run_instr_data_to_core_noack_inner(struct jtag_tap *tap, uint32_t opcode,
		uint32_t *data, size_t count)
{
	int arm11_run_instr_data_to_core_noack_inner_default(struct jtag_tap *tap,
			uint32_t opcode, uint32_t *data, size_t count);
	return arm11_run_instr_data_to_core_noack_inner_default(tap, opcode, data, count);
}
