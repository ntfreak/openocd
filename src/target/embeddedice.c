/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "embeddedice.h"
#include "register.h"
#include <helper/time_support.h>

/**
 * @file
 *
 * This provides lowlevel glue to the EmbeddedICE (or EmbeddedICE-RT)
 * module found on scan chain 2 in ARM7, ARM9, and some other families
 * of ARM cores.  The module is called "EmbeddedICE-RT" if it has
 * monitor mode support.
 *
 * EmbeddedICE provides basic watchpoint/breakpoint hardware and a Debug
 * Communications Channel (DCC) used to read or write 32-bit words to
 * OpenOCD-aware code running on the target CPU.
 * Newer modules also include vector catch hardware.  Some versions
 * support hardware single-stepping, "monitor mode" debug (which is not
 * currently supported by OpenOCD), or extended reporting on why the
 * core entered debug mode.
 */

static int embeddedice_set_reg_w_exec(struct reg *reg, uint8_t *buf);

/*
 * From:  ARM9E-S TRM, DDI 0165, table C-4 (and similar, for other cores)
 */
static const struct {
	const char     *name;
	unsigned short addr;
	unsigned short width;
} eice_regs[] = {
	[EICE_DBG_CTRL] = {
		.name =		"debug_ctrl",
		.addr =		0,
		/* width is assigned based on EICE version */
	},
	[EICE_DBG_STAT] = {
		.name =		"debug_status",
		.addr =		1,
		/* width is assigned based on EICE version */
	},
	[EICE_COMMS_CTRL] = {
		.name =		"comms_ctrl",
		.addr =		4,
		.width =	6,
	},
	[EICE_COMMS_DATA] = {
		.name =		"comms_data",
		.addr =		5,
		.width =	32,
	},
	[EICE_W0_ADDR_VALUE] = {
		.name =		"watch_0_addr_value",
		.addr =		8,
		.width =	32,
	},
	[EICE_W0_ADDR_MASK] = {
		.name =		"watch_0_addr_mask",
		.addr =		9,
		.width =	32,
	},
	[EICE_W0_DATA_VALUE] = {
		.name =		"watch_0_data_value",
		.addr =		10,
		.width =	32,
	},
	[EICE_W0_DATA_MASK] = {
		.name =		"watch_0_data_mask",
		.addr =		11,
		.width =	32,
	},
	[EICE_W0_CONTROL_VALUE] = {
		.name =		"watch_0_control_value",
		.addr =		12,
		.width =	9,
	},
	[EICE_W0_CONTROL_MASK] = {
		.name =		"watch_0_control_mask",
		.addr =		13,
		.width =	8,
	},
	[EICE_W1_ADDR_VALUE] = {
		.name =		"watch_1_addr_value",
		.addr =		16,
		.width =	32,
	},
	[EICE_W1_ADDR_MASK] = {
		.name =		"watch_1_addr_mask",
		.addr =		17,
		.width =	32,
	},
	[EICE_W1_DATA_VALUE] = {
		.name =		"watch_1_data_value",
		.addr =		18,
		.width =	32,
	},
	[EICE_W1_DATA_MASK] = {
		.name =		"watch_1_data_mask",
		.addr =		19,
		.width =	32,
	},
	[EICE_W1_CONTROL_VALUE] = {
		.name =		"watch_1_control_value",
		.addr =		20,
		.width =	9,
	},
	[EICE_W1_CONTROL_MASK] = {
		.name =		"watch_1_control_mask",
		.addr =		21,
		.width =	8,
	},
	/* vector_catch isn't always present */
	[EICE_VEC_CATCH] = {
		.name =		"vector_catch",
		.addr =		2,
		.width =	8,
	},
};

static int embeddedice_get_reg(struct reg *reg)
{
	int retval = embeddedice_read_reg(reg);
	if (retval != ERROR_OK) {
		LOG_ERROR("error queueing EmbeddedICE register read");
		return retval;
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		LOG_ERROR("EmbeddedICE register read failed");

	return retval;
}

static const struct reg_arch_type eice_reg_type = {
	.get = embeddedice_get_reg,
	.set = embeddedice_set_reg_w_exec,
};

/**
 * Probe EmbeddedICE module and set up local records of its registers.
 * Different versions of the modules have different capabilities, such as
 * hardware support for vector_catch, single stepping, and monitor mode.
 */
struct reg_cache *embeddedice_build_reg_cache(struct target *target,
		struct arm7_9_common *arm7_9)
{
	int retval;
	struct reg_cache *reg_cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = NULL;
	struct embeddedice_reg *arch_info = NULL;
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	int num_regs = ARRAY_SIZE(eice_regs);
	int i;
	int eice_version = 0;

	/* vector_catch isn't always present */
	if (!arm7_9->has_vector_catch)
		num_regs--;

	/* the actual registers are kept in two arrays */
	reg_list = calloc(num_regs, sizeof(struct reg));
	arch_info = calloc(num_regs, sizeof(struct embeddedice_reg));

	/* fill in values for the reg cache */
	reg_cache->name = "EmbeddedICE registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = num_regs;

	/* FIXME the second watchpoint unit on Feroceon and Dragonite
	 * seems not to work ... we should have a way to not set up
	 * its four registers here!
	 */

	/* set up registers */
	for (i = 0; i < num_regs; i++) {
		reg_list[i].name = eice_regs[i].name;
		reg_list[i].size = eice_regs[i].width;
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].type = &eice_reg_type;
		arch_info[i].addr = eice_regs[i].addr;
		arch_info[i].jtag_info = jtag_info;
	}

	/* identify EmbeddedICE version by reading DCC control register */
	embeddedice_read_reg(&reg_list[EICE_COMMS_CTRL]);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		for (i = 0; i < num_regs; i++)
			free(reg_list[i].value);
		free(reg_list);
		free(reg_cache);
		free(arch_info);
		return NULL;
	}

	eice_version = buf_get_u32(reg_list[EICE_COMMS_CTRL].value, 28, 4);
	LOG_INFO("Embedded ICE version %d", eice_version);

	switch (eice_version) {
		case 1:
			/* ARM7TDMI r3, ARM7TDMI-S r3
			 *
			 * REVISIT docs say ARM7TDMI-S r4 uses version 1 but
			 * that it has 6-bit CTRL and 5-bit STAT... doc bug?
			 * ARM7TDMI r4 docs say EICE v4.
			 */
			reg_list[EICE_DBG_CTRL].size = 3;
			reg_list[EICE_DBG_STAT].size = 5;
			break;
		case 2:
			/* ARM9TDMI */
			reg_list[EICE_DBG_CTRL].size = 4;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_single_step = 1;
			break;
		case 3:
			LOG_ERROR("EmbeddedICE v%d handling might be broken",
					eice_version);
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_single_step = 1;
			arm7_9->has_monitor_mode = 1;
			break;
		case 4:
			/* ARM7TDMI r4 */
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_monitor_mode = 1;
			break;
		case 5:
			/* ARM9E-S rev 1 */
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_single_step = 1;
			arm7_9->has_monitor_mode = 1;
			break;
		case 6:
			/* ARM7EJ-S, ARM9E-S rev 2, ARM9EJ-S */
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 10;
			/* DBG_STAT has MOE bits */
			arm7_9->has_monitor_mode = 1;
			break;
		case 7:
			LOG_ERROR("EmbeddedICE v%d handling might be broken",
					eice_version);
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_monitor_mode = 1;
			break;
		default:
			/*
			 * The Feroceon implementation has the version number
			 * in some unusual bits.  Let feroceon.c validate it
			 * and do the appropriate setup itself.
			 */
			if (strcmp(target_type_name(target), "feroceon") == 0 ||
					strcmp(target_type_name(target), "dragonite") == 0)
				break;
			LOG_ERROR("unknown EmbeddedICE version "
				"(comms ctrl: 0x%8.8" PRIx32 ")",
				buf_get_u32(reg_list[EICE_COMMS_CTRL].value, 0, 32));
	}

	/* On Feroceon and Dragonite the second unit is seemingly missing. */
	LOG_INFO("%s: hardware has %d breakpoint/watchpoint unit%s",
			target_name(target), arm7_9->wp_available_max,
			(arm7_9->wp_available_max != 1) ? "s" : "");

	return reg_cache;
}

/**
 * Free all memory allocated for EmbeddedICE register cache
 */
void embeddedice_free_reg_cache(struct reg_cache *reg_cache)
{
	if (!reg_cache)
		return;

	for (unsigned int i = 0; i < reg_cache->num_regs; i++)
		free(reg_cache->reg_list[i].value);

	free(reg_cache->reg_list[0].arch_info);
	free(reg_cache->reg_list);
	free(reg_cache);
}

/**
 * Initialize EmbeddedICE module, if needed.
 */
int embeddedice_setup(struct target *target)
{
	int retval;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);

	/* Explicitly disable monitor mode.  For now we only support halting
	 * debug ... we don't know how to talk with a resident debug monitor
	 * that manages break requests.  ARM's "Angel Debug Monitor" is one
	 * common example of such code.
	 */
	if (arm7_9->has_monitor_mode) {
		struct reg *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];

		embeddedice_read_reg(dbg_ctrl);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;
		buf_set_u32(dbg_ctrl->value, 4, 1, 0);
		embeddedice_set_reg_w_exec(dbg_ctrl, dbg_ctrl->value);
	}
	return jtag_execute_queue();
}

/**
 * Queue a read for an EmbeddedICE register into the register cache,
 * optionally checking the value read.
 * Note that at this level, all registers are 32 bits wide.
 */
int embeddedice_read_reg_w_check(struct reg *reg,
		uint8_t *check_value, uint8_t *check_mask)
{
	struct embeddedice_reg *ice_reg = reg->arch_info;
	uint8_t reg_addr = ice_reg->addr & 0x1f;
	struct scan_field fields[3];
	uint8_t field1_out[1];
	uint8_t field2_out[1];
	int retval;

	retval = arm_jtag_scann(ice_reg->jtag_info, 0x2, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	retval = arm_jtag_set_instr(ice_reg->jtag_info->tap,
			ice_reg->jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	/* bits 31:0 -- data (ignored here) */
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].in_value = NULL;
	fields[0].check_value = NULL;
	fields[0].check_mask = NULL;

	/* bits 36:32 -- register */
	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	field1_out[0] = reg_addr;
	fields[1].in_value = NULL;
	fields[1].check_value = NULL;
	fields[1].check_mask = NULL;

	/* bit 37 -- 0/read */
	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	field2_out[0] = 0;
	fields[2].in_value = NULL;
	fields[2].check_value = NULL;
	fields[2].check_mask = NULL;

	/* traverse Update-DR, setting address for the next read */
	jtag_add_dr_scan(ice_reg->jtag_info->tap, 3, fields, TAP_IDLE);

	/* bits 31:0 -- the data we're reading (and maybe checking) */
	fields[0].in_value = reg->value;
	fields[0].check_value = check_value;
	fields[0].check_mask = check_mask;

	/* when reading the DCC data register, leaving the address field set to
	 * EICE_COMMS_DATA would read the register twice
	 * reading the control register is safe
	 */
	field1_out[0] = eice_regs[EICE_COMMS_CTRL].addr;

	/* traverse Update-DR, reading but with no other side effects */
	jtag_add_dr_scan_check(ice_reg->jtag_info->tap, 3, fields, TAP_IDLE);

	return ERROR_OK;
}

/**
 * Receive a block of size 32-bit words from the DCC.
 * We assume the target is always going to be fast enough (relative to
 * the JTAG clock) that the debugger won't need to poll the handshake
 * bit.  The JTAG clock is usually at least six times slower than the
 * functional clock, so the 50+ JTAG clocks needed to receive the word
 * allow hundreds of instruction cycles (per word) in the target.
 */
int embeddedice_receive(struct arm_jtag *jtag_info, uint32_t *data, uint32_t size)
{
	struct scan_field fields[3];
	uint8_t field1_out[1];
	uint8_t field2_out[1];
	int retval;

	retval = arm_jtag_scann(jtag_info, 0x2, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	field1_out[0] = eice_regs[EICE_COMMS_DATA].addr;
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	field2_out[0] = 0;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

	while (size > 0) {
		/* when reading the last item, set the register address to the DCC control reg,
		 * to avoid reading additional data from the DCC data reg
		 */
		if (size == 1)
			field1_out[0] = eice_regs[EICE_COMMS_CTRL].addr;

		fields[0].in_value = (uint8_t *)data;
		jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);
		jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)data);

		data++;
		size--;
	}

	return jtag_execute_queue();
}

/**
 * Queue a read for an EmbeddedICE register into the register cache,
 * not checking the value read.
 */
int embeddedice_read_reg(struct reg *reg)
{
	return embeddedice_read_reg_w_check(reg, NULL, NULL);
}

/**
 * Queue a write for an EmbeddedICE register, updating the register cache.
 * Uses embeddedice_write_reg().
 */
void embeddedice_set_reg(struct reg *reg, uint32_t value)
{
	embeddedice_write_reg(reg, value);

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = true;
	reg->dirty = false;

}

/**
 * Write an EmbeddedICE register, updating the register cache.
 * Uses embeddedice_set_reg(); not queued.
 */
static int embeddedice_set_reg_w_exec(struct reg *reg, uint8_t *buf)
{
	int retval;

	embeddedice_set_reg(reg, buf_get_u32(buf, 0, reg->size));
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		LOG_ERROR("register write failed");
	return retval;
}

/**
 * Queue a write for an EmbeddedICE register, bypassing the register cache.
 */
void embeddedice_write_reg(struct reg *reg, uint32_t value)
{
	struct embeddedice_reg *ice_reg = reg->arch_info;

	LOG_DEBUG("%i: 0x%8.8" PRIx32 "", ice_reg->addr, value);

	arm_jtag_scann(ice_reg->jtag_info, 0x2, TAP_IDLE);

	arm_jtag_set_instr(ice_reg->jtag_info->tap, ice_reg->jtag_info->intest_instr, NULL, TAP_IDLE);

	uint8_t reg_addr = ice_reg->addr & 0x1f;
	embeddedice_write_reg_inner(ice_reg->jtag_info->tap, reg_addr, value);
}

/**
 * Queue a write for an EmbeddedICE register, using cached value.
 * Uses embeddedice_write_reg().
 */
void embeddedice_store_reg(struct reg *reg)
{
	embeddedice_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

/**
 * Send a block of size 32-bit words to the DCC.
 * We assume the target is always going to be fast enough (relative to
 * the JTAG clock) that the debugger won't need to poll the handshake
 * bit.  The JTAG clock is usually at least six times slower than the
 * functional clock, so the 50+ JTAG clocks needed to receive the word
 * allow hundreds of instruction cycles (per word) in the target.
 */
int embeddedice_send(struct arm_jtag *jtag_info, uint32_t *data, uint32_t size)
{
	struct scan_field fields[3];
	uint8_t field0_out[4];
	uint8_t field1_out[1];
	uint8_t field2_out[1];
	int retval;

	retval = arm_jtag_scann(jtag_info, 0x2, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = field0_out;
	fields[0].in_value = NULL;

	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	field1_out[0] = eice_regs[EICE_COMMS_DATA].addr;
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	field2_out[0] = 1;

	fields[2].in_value = NULL;

	while (size > 0) {
		buf_set_u32(field0_out, 0, 32, *data);
		jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

		data++;
		size--;
	}

	/* call to jtag_execute_queue() intentionally omitted */
	return ERROR_OK;
}

/**
 * Poll DCC control register until read or write handshake completes.
 */
int embeddedice_handshake(struct arm_jtag *jtag_info, int hsbit, uint32_t timeout)
{
	struct scan_field fields[3];
	uint8_t field0_in[4];
	uint8_t field1_out[1];
	uint8_t field2_out[1];
	int retval;
	uint32_t hsact;
	struct timeval now;
	struct timeval timeout_end;

	if (hsbit == EICE_COMM_CTRL_WBIT)
		hsact = 1;
	else if (hsbit == EICE_COMM_CTRL_RBIT)
		hsact = 0;
	else {
		LOG_ERROR("Invalid arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = arm_jtag_scann(jtag_info, 0x2, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = field0_in;

	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	field1_out[0] = eice_regs[EICE_COMMS_DATA].addr;
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	field2_out[0] = 0;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);
	gettimeofday(&timeout_end, NULL);
	timeval_add_time(&timeout_end, 0, timeout * 1000);
	do {
		jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		if (buf_get_u32(field0_in, hsbit, 1) == hsact)
			return ERROR_OK;

		gettimeofday(&now, NULL);
	} while (timeval_compare(&now, &timeout_end) <= 0);

	LOG_ERROR("embeddedice handshake timeout");
	return ERROR_TARGET_TIMEOUT;
}

/**
 * This is an inner loop of the open loop DCC write of data to target
 */
void embeddedice_write_dcc(struct jtag_tap *tap,
		int reg_addr, const uint8_t *buffer, int little, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		embeddedice_write_reg_inner(tap, reg_addr,
				fast_target_buffer_get_u32(buffer, little));
		buffer += 4;
	}
}
