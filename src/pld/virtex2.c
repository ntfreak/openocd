/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "virtex2.h"
#include "xilinx_bit.h"
#include "pld.h"

static int virtex2_set_instr(struct jtag_tap *tap, uint32_t new_instr)
{
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr) {
		struct scan_field field;

		field.num_bits = tap->ir_length;
		void *t = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);
		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);

		free(t);
	}

	return ERROR_OK;
}

static int virtex2_send_32(struct pld_device *pld_device,
	int num_words, uint32_t *words)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	struct scan_field scan_field;
	uint8_t *values;
	int i;

	values = malloc(num_words * 4);

	scan_field.num_bits = num_words * 32;
	scan_field.out_value = values;
	scan_field.in_value = NULL;

	for (i = 0; i < num_words; i++)
		buf_set_u32(values + 4 * i, 0, 32, flip_u32(*words++, 32));

	virtex2_set_instr(virtex2_info->tap, 0x5);	/* CFG_IN */

	jtag_add_dr_scan(virtex2_info->tap, 1, &scan_field, TAP_DRPAUSE);

	free(values);

	return ERROR_OK;
}

static inline void virtexflip32(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)arg) = flip_u32(le_to_h_u32(in), 32);
}

static int virtex2_receive_32(struct pld_device *pld_device,
	int num_words, uint32_t *words)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	struct scan_field scan_field;

	scan_field.num_bits = 32;
	scan_field.out_value = NULL;
	scan_field.in_value = NULL;

	virtex2_set_instr(virtex2_info->tap, 0x4);	/* CFG_OUT */

	while (num_words--) {
		scan_field.in_value = (uint8_t *)words;

		jtag_add_dr_scan(virtex2_info->tap, 1, &scan_field, TAP_DRPAUSE);

		jtag_add_callback(virtexflip32, (jtag_callback_data_t)words);

		words++;
	}

	return ERROR_OK;
}

static int virtex2_read_stat(struct pld_device *pld_device, uint32_t *status)
{
	uint32_t data[5];

	jtag_add_tlr();

	data[0] = 0xaa995566;	/* synch word */
	data[1] = 0x2800E001;	/* Type 1, read, address 7, 1 word */
	data[2] = 0x20000000;	/* NOOP (Type 1, read, address 0, 0 words */
	data[3] = 0x20000000;	/* NOOP */
	data[4] = 0x20000000;	/* NOOP */
	virtex2_send_32(pld_device, 5, data);

	virtex2_receive_32(pld_device, 1, status);

	jtag_execute_queue();

	LOG_DEBUG("status: 0x%8.8" PRIx32 "", *status);

	return ERROR_OK;
}

static int virtex2_load(struct pld_device *pld_device, const char *filename)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	struct xilinx_bit_file bit_file;
	int retval;
	unsigned int i;
	struct scan_field field;

	field.in_value = NULL;

	retval = xilinx_read_bit_file(&bit_file, filename);
	if (retval != ERROR_OK)
		return retval;

	virtex2_set_instr(virtex2_info->tap, 0xb);	/* JPROG_B */
	jtag_execute_queue();
	jtag_add_sleep(1000);

	virtex2_set_instr(virtex2_info->tap, 0x5);	/* CFG_IN */
	jtag_execute_queue();

	for (i = 0; i < bit_file.length; i++)
		bit_file.data[i] = flip_u32(bit_file.data[i], 8);

	field.num_bits = bit_file.length * 8;
	field.out_value = bit_file.data;

	jtag_add_dr_scan(virtex2_info->tap, 1, &field, TAP_DRPAUSE);
	jtag_execute_queue();

	jtag_add_tlr();

	if (!(virtex2_info->no_jstart))
		virtex2_set_instr(virtex2_info->tap, 0xc);	/* JSTART */
	jtag_add_runtest(13, TAP_IDLE);
	virtex2_set_instr(virtex2_info->tap, 0x3f);		/* BYPASS */
	virtex2_set_instr(virtex2_info->tap, 0x3f);		/* BYPASS */
	if (!(virtex2_info->no_jstart))
		virtex2_set_instr(virtex2_info->tap, 0xc);	/* JSTART */
	jtag_add_runtest(13, TAP_IDLE);
	virtex2_set_instr(virtex2_info->tap, 0x3f);		/* BYPASS */
	jtag_execute_queue();

	return ERROR_OK;
}

COMMAND_HANDLER(virtex2_handle_read_stat_command)
{
	struct pld_device *device;
	uint32_t status;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned dev_id;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	virtex2_read_stat(device, &status);

	command_print(CMD, "virtex2 status register: 0x%8.8" PRIx32 "", status);

	return ERROR_OK;
}

PLD_DEVICE_COMMAND_HANDLER(virtex2_pld_device_command)
{
	struct jtag_tap *tap;

	struct virtex2_pld_device *virtex2_info;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	tap = jtag_tap_by_string(CMD_ARGV[1]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[1]);
		return ERROR_OK;
	}

	virtex2_info = malloc(sizeof(struct virtex2_pld_device));
	virtex2_info->tap = tap;

	virtex2_info->no_jstart = 0;
	if (CMD_ARGC >= 3)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], virtex2_info->no_jstart);

	pld->driver_priv = virtex2_info;

	return ERROR_OK;
}

static const struct command_registration virtex2_exec_command_handlers[] = {
	{
		.name = "read_stat",
		.mode = COMMAND_EXEC,
		.handler = virtex2_handle_read_stat_command,
		.help = "read status register",
		.usage = "pld_num",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration virtex2_command_handler[] = {
	{
		.name = "virtex2",
		.mode = COMMAND_ANY,
		.help = "Virtex-II specific commands",
		.usage = "",
		.chain = virtex2_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct pld_driver virtex2_pld = {
	.name = "virtex2",
	.commands = virtex2_command_handler,
	.pld_device_command = &virtex2_pld_device_command,
	.load = &virtex2_load,
};
