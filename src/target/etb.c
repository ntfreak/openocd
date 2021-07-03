/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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

#include "arm.h"
#include "etm.h"
#include "etb.h"
#include "register.h"

static const char * const etb_reg_list[] = {
	"ETB_identification",
	"ETB_ram_depth",
	"ETB_ram_width",
	"ETB_status",
	"ETB_ram_data",
	"ETB_ram_read_pointer",
	"ETB_ram_write_pointer",
	"ETB_trigger_counter",
	"ETB_control",
};

static int etb_get_reg(struct reg *reg);

static int etb_set_instr(struct etb *etb, uint32_t new_instr)
{
	struct jtag_tap *tap;

	tap = etb->tap;
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

static int etb_scann(struct etb *etb, uint32_t new_scan_chain)
{
	if (etb->cur_scan_chain != new_scan_chain) {
		struct scan_field field;

		field.num_bits = 5;
		void *t = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_scan_chain);

		field.in_value = NULL;

		/* select INTEST instruction */
		etb_set_instr(etb, 0x2);
		jtag_add_dr_scan(etb->tap, 1, &field, TAP_IDLE);

		etb->cur_scan_chain = new_scan_chain;

		free(t);
	}

	return ERROR_OK;
}

static int etb_read_reg_w_check(struct reg *, uint8_t *, uint8_t *);
static int etb_set_reg_w_exec(struct reg *, uint8_t *);

static int etb_read_reg(struct reg *reg)
{
	return etb_read_reg_w_check(reg, NULL, NULL);
}

static int etb_get_reg(struct reg *reg)
{
	int retval;

	retval = etb_read_reg(reg);
	if (retval != ERROR_OK) {
		LOG_ERROR("BUG: error scheduling ETB register read");
		return retval;
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("ETB register read failed");
		return retval;
	}

	return ERROR_OK;
}

static const struct reg_arch_type etb_reg_type = {
	.get = etb_get_reg,
	.set = etb_set_reg_w_exec,
};

struct reg_cache *etb_build_reg_cache(struct etb *etb)
{
	struct reg_cache *reg_cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = NULL;
	struct etb_reg *arch_info = NULL;
	int num_regs = 9;
	int i;

	/* the actual registers are kept in two arrays */
	reg_list = calloc(num_regs, sizeof(struct reg));
	arch_info = calloc(num_regs, sizeof(struct etb_reg));

	/* fill in values for the reg cache */
	reg_cache->name = "etb registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = num_regs;

	/* set up registers */
	for (i = 0; i < num_regs; i++) {
		reg_list[i].name = etb_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].type = &etb_reg_type;
		reg_list[i].size = 32;
		arch_info[i].addr = i;
		arch_info[i].etb = etb;
	}

	return reg_cache;
}

static void etb_getbuf(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;

	*((uint32_t *)arg) = buf_get_u32(in, 0, 32);
}

static int etb_read_ram(struct etb *etb, uint32_t *data, int num_frames)
{
	struct scan_field fields[3];
	int i;

	etb_scann(etb, 0x0);
	etb_set_instr(etb, 0xc);

	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].num_bits = 7;
	uint8_t temp1 = 0;
	fields[1].out_value = &temp1;
	buf_set_u32(&temp1, 0, 7, 4);
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	uint8_t temp2 = 0;
	fields[2].out_value = &temp2;
	buf_set_u32(&temp2, 0, 1, 0);
	fields[2].in_value = NULL;

	jtag_add_dr_scan(etb->tap, 3, fields, TAP_IDLE);

	for (i = 0; i < num_frames; i++) {
		/* ensure nR/W remains set to read */
		buf_set_u32(&temp2, 0, 1, 0);

		/* address remains set to 0x4 (RAM data) until we read the last frame */
		if (i < num_frames - 1)
			buf_set_u32(&temp1, 0, 7, 4);
		else
			buf_set_u32(&temp1, 0, 7, 0);

		fields[0].in_value = (uint8_t *)(data + i);
		jtag_add_dr_scan(etb->tap, 3, fields, TAP_IDLE);

		jtag_add_callback(etb_getbuf, (jtag_callback_data_t)(data + i));
	}

	jtag_execute_queue();

	return ERROR_OK;
}

static int etb_read_reg_w_check(struct reg *reg,
	uint8_t *check_value, uint8_t *check_mask)
{
	struct etb_reg *etb_reg = reg->arch_info;
	uint8_t reg_addr = etb_reg->addr & 0x7f;
	struct scan_field fields[3];

	LOG_DEBUG("%i", (int)(etb_reg->addr));

	etb_scann(etb_reg->etb, 0x0);
	etb_set_instr(etb_reg->etb, 0xc);

	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].in_value = NULL;
	fields[0].check_value = NULL;
	fields[0].check_mask = NULL;

	fields[1].num_bits = 7;
	uint8_t temp1 = 0;
	fields[1].out_value = &temp1;
	buf_set_u32(&temp1, 0, 7, reg_addr);
	fields[1].in_value = NULL;
	fields[1].check_value = NULL;
	fields[1].check_mask = NULL;

	fields[2].num_bits = 1;
	uint8_t temp2 = 0;
	fields[2].out_value = &temp2;
	buf_set_u32(&temp2, 0, 1, 0);
	fields[2].in_value = NULL;
	fields[2].check_value = NULL;
	fields[2].check_mask = NULL;

	jtag_add_dr_scan(etb_reg->etb->tap, 3, fields, TAP_IDLE);

	/* read the identification register in the second run, to make sure we
	 * don't read the ETB data register twice, skipping every second entry
	 */
	buf_set_u32(&temp1, 0, 7, 0x0);
	fields[0].in_value = reg->value;
	fields[0].check_value = check_value;
	fields[0].check_mask = check_mask;

	jtag_add_dr_scan_check(etb_reg->etb->tap, 3, fields, TAP_IDLE);

	return ERROR_OK;
}

static int etb_write_reg(struct reg *, uint32_t);

static int etb_set_reg(struct reg *reg, uint32_t value)
{
	int retval;

	retval = etb_write_reg(reg, value);
	if (retval != ERROR_OK) {
		LOG_ERROR("BUG: error scheduling ETB register write");
		return retval;
	}

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = true;
	reg->dirty = false;

	return ERROR_OK;
}

static int etb_set_reg_w_exec(struct reg *reg, uint8_t *buf)
{
	int retval;

	etb_set_reg(reg, buf_get_u32(buf, 0, reg->size));

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("ETB: register write failed");
		return retval;
	}
	return ERROR_OK;
}

static int etb_write_reg(struct reg *reg, uint32_t value)
{
	struct etb_reg *etb_reg = reg->arch_info;
	uint8_t reg_addr = etb_reg->addr & 0x7f;
	struct scan_field fields[3];

	LOG_DEBUG("%i: 0x%8.8" PRIx32 "", (int)(etb_reg->addr), value);

	etb_scann(etb_reg->etb, 0x0);
	etb_set_instr(etb_reg->etb, 0xc);

	fields[0].num_bits = 32;
	uint8_t temp0[4];
	fields[0].out_value = temp0;
	buf_set_u32(temp0, 0, 32, value);
	fields[0].in_value = NULL;

	fields[1].num_bits = 7;
	uint8_t temp1 = 0;
	fields[1].out_value = &temp1;
	buf_set_u32(&temp1, 0, 7, reg_addr);
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	uint8_t temp2 = 0;
	fields[2].out_value = &temp2;
	buf_set_u32(&temp2, 0, 1, 1);
	fields[2].in_value = NULL;

	jtag_add_dr_scan(etb_reg->etb->tap, 3, fields, TAP_IDLE);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etb_config_command)
{
	struct target *target;
	struct jtag_tap *tap;
	struct arm *arm;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target = get_target(CMD_ARGV[0]);

	if (!target) {
		LOG_ERROR("ETB: target '%s' not defined", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETB: '%s' isn't an ARM", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	tap = jtag_tap_by_string(CMD_ARGV[1]);
	if (!tap) {
		command_print(CMD, "ETB: TAP %s does not exist", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	if (arm->etm) {
		struct etb *etb = malloc(sizeof(struct etb));

		arm->etm->capture_driver_priv = etb;

		etb->tap  = tap;
		etb->cur_scan_chain = 0xffffffff;
		etb->reg_cache = NULL;
		etb->ram_width = 0;
		etb->ram_depth = 0;
	} else {
		LOG_ERROR("ETM: target has no ETM defined, ETB left unconfigured");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etb_trigger_percent_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm;
	struct etb *etb;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETB: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm = arm->etm;
	if (!etm) {
		command_print(CMD, "ETB: target has no ETM configured");
		return ERROR_FAIL;
	}
	if (etm->capture_driver != &etb_capture_driver) {
		command_print(CMD, "ETB: target not using ETB");
		return ERROR_FAIL;
	}
	etb = arm->etm->capture_driver_priv;

	if (CMD_ARGC > 0) {
		uint32_t new_value;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], new_value);
		if ((new_value < 2) || (new_value > 100))
			command_print(CMD,
				"valid percentages are 2%% to 100%%");
		else
			etb->trigger_percent = (unsigned) new_value;
	}

	command_print(CMD, "%d percent of tracebuffer fills after trigger",
		etb->trigger_percent);

	return ERROR_OK;
}

static const struct command_registration etb_config_command_handlers[] = {
	{
		/* NOTE:  with ADIv5, ETBs are accessed using DAP operations,
		 * possibly over SWD, not through separate TAPs...
		 */
		.name = "config",
		.handler = handle_etb_config_command,
		.mode = COMMAND_CONFIG,
		.help = "Associate ETB with target and JTAG TAP.",
		.usage = "target tap",
	},
	{
		.name = "trigger_percent",
		.handler = handle_etb_trigger_percent_command,
		.mode = COMMAND_EXEC,
		.help = "Set percent of trace buffer to be filled "
			"after the trigger occurs (2..100).",
		.usage = "[percent]",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration etb_command_handlers[] = {
	{
		.name = "etb",
		.mode = COMMAND_ANY,
		.help = "Embedded Trace Buffer command group",
		.chain = etb_config_command_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int etb_init(struct etm_context *etm_ctx)
{
	struct etb *etb = etm_ctx->capture_driver_priv;

	etb->etm_ctx = etm_ctx;

	/* identify ETB RAM depth and width */
	etb_read_reg(&etb->reg_cache->reg_list[ETB_RAM_DEPTH]);
	etb_read_reg(&etb->reg_cache->reg_list[ETB_RAM_WIDTH]);
	jtag_execute_queue();

	etb->ram_depth = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_DEPTH].value, 0, 32);
	etb->ram_width = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_WIDTH].value, 0, 32);

	etb->trigger_percent = 50;

	return ERROR_OK;
}

static trace_status_t etb_status(struct etm_context *etm_ctx)
{
	struct etb *etb = etm_ctx->capture_driver_priv;
	struct reg *control = &etb->reg_cache->reg_list[ETB_CTRL];
	struct reg *status = &etb->reg_cache->reg_list[ETB_STATUS];
	trace_status_t retval = 0;
	int etb_timeout = 100;

	etb->etm_ctx = etm_ctx;

	/* read control and status registers */
	etb_read_reg(control);
	etb_read_reg(status);
	jtag_execute_queue();

	/* See if it's (still) active */
	retval = buf_get_u32(control->value, 0, 1) ? TRACE_RUNNING : TRACE_IDLE;

	/* check Full bit to identify wraparound/overflow */
	if (buf_get_u32(status->value, 0, 1) == 1)
		retval |= TRACE_OVERFLOWED;

	/* check Triggered bit to identify trigger condition */
	if (buf_get_u32(status->value, 1, 1) == 1)
		retval |= TRACE_TRIGGERED;

	/* check AcqComp to see if trigger counter dropped to zero */
	if (buf_get_u32(status->value, 2, 1) == 1) {
		/* wait for DFEmpty */
		while (etb_timeout-- && buf_get_u32(status->value, 3, 1) == 0)
			etb_get_reg(status);

		if (etb_timeout == 0)
			LOG_ERROR("ETB:  DFEmpty won't go high, status 0x%02x",
				(unsigned) buf_get_u32(status->value, 0, 4));

		if (!(etm_ctx->capture_status & TRACE_TRIGGERED))
			LOG_WARNING("ETB: trace complete without triggering?");

		retval |= TRACE_COMPLETED;
	}

	/* NOTE: using a trigger is optional; and at least ETB11 has a mode
	 * where it can ignore the trigger counter.
	 */

	/* update recorded state */
	etm_ctx->capture_status = retval;

	return retval;
}

static int etb_read_trace(struct etm_context *etm_ctx)
{
	struct etb *etb = etm_ctx->capture_driver_priv;
	int first_frame = 0;
	int num_frames = etb->ram_depth;
	uint32_t *trace_data = NULL;
	int i, j;

	etb_read_reg(&etb->reg_cache->reg_list[ETB_STATUS]);
	etb_read_reg(&etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER]);
	jtag_execute_queue();

	/* check if we overflowed, and adjust first frame of the trace accordingly
	 * if we didn't overflow, read only up to the frame that would be written next,
	 * i.e. don't read invalid entries
	 */
	if (buf_get_u32(etb->reg_cache->reg_list[ETB_STATUS].value, 0, 1))
		first_frame = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER].value,
				0,
				32);
	else
		num_frames = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER].value,
				0,
				32);

	etb_write_reg(&etb->reg_cache->reg_list[ETB_RAM_READ_POINTER], first_frame);

	/* read data into temporary array for unpacking */
	trace_data = malloc(sizeof(uint32_t) * num_frames);
	etb_read_ram(etb, trace_data, num_frames);

	if (etm_ctx->trace_depth > 0)
		free(etm_ctx->trace_data);

	if ((etm_ctx->control & ETM_PORT_WIDTH_MASK) == ETM_PORT_4BIT)
		etm_ctx->trace_depth = num_frames * 3;
	else if ((etm_ctx->control & ETM_PORT_WIDTH_MASK) == ETM_PORT_8BIT)
		etm_ctx->trace_depth = num_frames * 2;
	else
		etm_ctx->trace_depth = num_frames;

	etm_ctx->trace_data = malloc(sizeof(struct etmv1_trace_data) * etm_ctx->trace_depth);

	for (i = 0, j = 0; i < num_frames; i++) {
		if ((etm_ctx->control & ETM_PORT_WIDTH_MASK) == ETM_PORT_4BIT) {
			/* trace word j */
			etm_ctx->trace_data[j].pipestat = trace_data[i] & 0x7;
			etm_ctx->trace_data[j].packet = (trace_data[i] & 0x78) >> 3;
			etm_ctx->trace_data[j].flags = 0;
			if ((trace_data[i] & 0x80) >> 7)
				etm_ctx->trace_data[j].flags |= ETMV1_TRACESYNC_CYCLE;
			if (etm_ctx->trace_data[j].pipestat == STAT_TR) {
				etm_ctx->trace_data[j].pipestat = etm_ctx->trace_data[j].packet &
					0x7;
				etm_ctx->trace_data[j].flags |= ETMV1_TRIGGER_CYCLE;
			}

			/* trace word j + 1 */
			etm_ctx->trace_data[j + 1].pipestat = (trace_data[i] & 0x100) >> 8;
			etm_ctx->trace_data[j + 1].packet = (trace_data[i] & 0x7800) >> 11;
			etm_ctx->trace_data[j + 1].flags = 0;
			if ((trace_data[i] & 0x8000) >> 15)
				etm_ctx->trace_data[j + 1].flags |= ETMV1_TRACESYNC_CYCLE;
			if (etm_ctx->trace_data[j + 1].pipestat == STAT_TR) {
				etm_ctx->trace_data[j +
				1].pipestat = etm_ctx->trace_data[j + 1].packet & 0x7;
				etm_ctx->trace_data[j + 1].flags |= ETMV1_TRIGGER_CYCLE;
			}

			/* trace word j + 2 */
			etm_ctx->trace_data[j + 2].pipestat = (trace_data[i] & 0x10000) >> 16;
			etm_ctx->trace_data[j + 2].packet = (trace_data[i] & 0x780000) >> 19;
			etm_ctx->trace_data[j + 2].flags = 0;
			if ((trace_data[i] & 0x800000) >> 23)
				etm_ctx->trace_data[j + 2].flags |= ETMV1_TRACESYNC_CYCLE;
			if (etm_ctx->trace_data[j + 2].pipestat == STAT_TR) {
				etm_ctx->trace_data[j +
				2].pipestat = etm_ctx->trace_data[j + 2].packet & 0x7;
				etm_ctx->trace_data[j + 2].flags |= ETMV1_TRIGGER_CYCLE;
			}

			j += 3;
		} else if ((etm_ctx->control & ETM_PORT_WIDTH_MASK) == ETM_PORT_8BIT) {
			/* trace word j */
			etm_ctx->trace_data[j].pipestat = trace_data[i] & 0x7;
			etm_ctx->trace_data[j].packet = (trace_data[i] & 0x7f8) >> 3;
			etm_ctx->trace_data[j].flags = 0;
			if ((trace_data[i] & 0x800) >> 11)
				etm_ctx->trace_data[j].flags |= ETMV1_TRACESYNC_CYCLE;
			if (etm_ctx->trace_data[j].pipestat == STAT_TR) {
				etm_ctx->trace_data[j].pipestat = etm_ctx->trace_data[j].packet &
					0x7;
				etm_ctx->trace_data[j].flags |= ETMV1_TRIGGER_CYCLE;
			}

			/* trace word j + 1 */
			etm_ctx->trace_data[j + 1].pipestat = (trace_data[i] & 0x7000) >> 12;
			etm_ctx->trace_data[j + 1].packet = (trace_data[i] & 0x7f8000) >> 15;
			etm_ctx->trace_data[j + 1].flags = 0;
			if ((trace_data[i] & 0x800000) >> 23)
				etm_ctx->trace_data[j + 1].flags |= ETMV1_TRACESYNC_CYCLE;
			if (etm_ctx->trace_data[j + 1].pipestat == STAT_TR) {
				etm_ctx->trace_data[j +
				1].pipestat = etm_ctx->trace_data[j + 1].packet & 0x7;
				etm_ctx->trace_data[j + 1].flags |= ETMV1_TRIGGER_CYCLE;
			}

			j += 2;
		} else {
			/* trace word j */
			etm_ctx->trace_data[j].pipestat = trace_data[i] & 0x7;
			etm_ctx->trace_data[j].packet = (trace_data[i] & 0x7fff8) >> 3;
			etm_ctx->trace_data[j].flags = 0;
			if ((trace_data[i] & 0x80000) >> 19)
				etm_ctx->trace_data[j].flags |= ETMV1_TRACESYNC_CYCLE;
			if (etm_ctx->trace_data[j].pipestat == STAT_TR) {
				etm_ctx->trace_data[j].pipestat = etm_ctx->trace_data[j].packet &
					0x7;
				etm_ctx->trace_data[j].flags |= ETMV1_TRIGGER_CYCLE;
			}

			j += 1;
		}
	}

	free(trace_data);

	return ERROR_OK;
}

static int etb_start_capture(struct etm_context *etm_ctx)
{
	struct etb *etb = etm_ctx->capture_driver_priv;
	uint32_t etb_ctrl_value = 0x1;
	uint32_t trigger_count;

	if ((etm_ctx->control & ETM_PORT_MODE_MASK) == ETM_PORT_DEMUXED) {
		if ((etm_ctx->control & ETM_PORT_WIDTH_MASK) != ETM_PORT_8BIT) {
			LOG_ERROR("ETB can't run in demultiplexed mode with a 4 or 16 bit port");
			return ERROR_ETM_PORTMODE_NOT_SUPPORTED;
		}
		etb_ctrl_value |= 0x2;
	}

	if ((etm_ctx->control & ETM_PORT_MODE_MASK) == ETM_PORT_MUXED) {
		LOG_ERROR("ETB: can't run in multiplexed mode");
		return ERROR_ETM_PORTMODE_NOT_SUPPORTED;
	}

	trigger_count = (etb->ram_depth * etb->trigger_percent) / 100;

	etb_write_reg(&etb->reg_cache->reg_list[ETB_TRIGGER_COUNTER], trigger_count);
	etb_write_reg(&etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER], 0x0);
	etb_write_reg(&etb->reg_cache->reg_list[ETB_CTRL], etb_ctrl_value);
	jtag_execute_queue();

	/* we're starting a new trace, initialize capture status */
	etm_ctx->capture_status = TRACE_RUNNING;

	return ERROR_OK;
}

static int etb_stop_capture(struct etm_context *etm_ctx)
{
	struct etb *etb = etm_ctx->capture_driver_priv;
	struct reg *etb_ctrl_reg = &etb->reg_cache->reg_list[ETB_CTRL];

	etb_write_reg(etb_ctrl_reg, 0x0);
	jtag_execute_queue();

	/* trace stopped, just clear running flag, but preserve others */
	etm_ctx->capture_status &= ~TRACE_RUNNING;

	return ERROR_OK;
}

struct etm_capture_driver etb_capture_driver = {
	.name = "etb",
	.commands = etb_command_handlers,
	.init = etb_init,
	.status = etb_status,
	.start_capture = etb_start_capture,
	.stop_capture = etb_stop_capture,
	.read_trace = etb_read_trace,
};
