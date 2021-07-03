/*****************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky <matthias.welwarsky@sysgo.com> *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 2 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
 *   This program is distributed in the hope that it will be useful,         *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *   GNU General Public License for more details.                            *
 ****************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "arm_adi_v5.h"
#include "register.h"

#include <jtag/jtag.h>

#define MEM_AP_COMMON_MAGIC 0x4DE4DA50

struct mem_ap {
	int common_magic;
	struct adiv5_dap *dap;
	struct adiv5_ap *ap;
	int ap_num;
};

static int mem_ap_target_create(struct target *target, Jim_Interp *interp)
{
	struct mem_ap *mem_ap;
	struct adiv5_private_config *pc;

	pc = (struct adiv5_private_config *)target->private_config;
	if (!pc)
		return ERROR_FAIL;

	if (pc->ap_num == DP_APSEL_INVALID) {
		LOG_ERROR("AP number not specified");
		return ERROR_FAIL;
	}

	mem_ap = calloc(1, sizeof(struct mem_ap));
	if (!mem_ap) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	mem_ap->ap_num = pc->ap_num;
	mem_ap->common_magic = MEM_AP_COMMON_MAGIC;
	mem_ap->dap = pc->dap;

	target->arch_info = mem_ap;

	if (!target->gdb_port_override)
		target->gdb_port_override = strdup("disabled");

	return ERROR_OK;
}

static int mem_ap_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_UNKNOWN;
	target->debug_reason = DBG_REASON_UNDEFINED;
	return ERROR_OK;
}

static void mem_ap_deinit_target(struct target *target)
{
	LOG_DEBUG("%s", __func__);

	free(target->private_config);
	free(target->arch_info);
	return;
}

static int mem_ap_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int mem_ap_poll(struct target *target)
{
	if (target->state == TARGET_UNKNOWN) {
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
	}

	return ERROR_OK;
}

static int mem_ap_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

static int mem_ap_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

static int mem_ap_step(struct target *target, int current, target_addr_t address,
				int handle_breakpoints)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

static int mem_ap_assert_reset(struct target *target)
{
	target->state = TARGET_RESET;
	target->debug_reason = DBG_REASON_UNDEFINED;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int mem_ap_examine(struct target *target)
{
	struct mem_ap *mem_ap = target->arch_info;

	if (!target_was_examined(target)) {
		mem_ap->ap = dap_ap(mem_ap->dap, mem_ap->ap_num);
		target_set_examined(target);
		target->state = TARGET_UNKNOWN;
		target->debug_reason = DBG_REASON_UNDEFINED;
		return mem_ap_init(mem_ap->ap);
	}

	return ERROR_OK;
}

static int mem_ap_deassert_reset(struct target *target)
{
	if (target->reset_halt) {
		target->state = TARGET_HALTED;
		target->debug_reason = DBG_REASON_DBGRQ;
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	} else {
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
	}

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int mem_ap_reg_get(struct reg *reg)
{
	return ERROR_OK;
}

static int mem_ap_reg_set(struct reg *reg, uint8_t *buf)
{
	return ERROR_OK;
}

static struct reg_arch_type mem_ap_reg_arch_type = {
	.get = mem_ap_reg_get,
	.set = mem_ap_reg_set,
};

const char *mem_ap_get_gdb_arch(struct target *target)
{
	return "arm";
}

/*
 * Dummy ARM register emulation:
 * reg[0..15]:  32 bits, r0~r12, sp, lr, pc
 * reg[16..23]: 96 bits, f0~f7
 * reg[24]:     32 bits, fps
 * reg[25]:     32 bits, cpsr
 *
 * Set 'exist' only to reg[0..15], so initial response to GDB is correct
 */
#define NUM_REGS     26
#define MAX_REG_SIZE 96
#define REG_EXIST(n) ((n) < 16)
#define REG_SIZE(n)  ((((n) >= 16) && ((n) < 24)) ? 96 : 32)

struct mem_ap_alloc_reg_list {
	/* reg_list must be the first field */
	struct reg *reg_list[NUM_REGS];
	struct reg regs[NUM_REGS];
	uint8_t regs_value[MAX_REG_SIZE / 8];
};

static int mem_ap_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
				int *reg_list_size, enum target_register_class reg_class)
{
	struct mem_ap_alloc_reg_list *mem_ap_alloc = calloc(1, sizeof(struct mem_ap_alloc_reg_list));
	if (!mem_ap_alloc) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	*reg_list = mem_ap_alloc->reg_list;
	*reg_list_size = NUM_REGS;
	struct reg *regs = mem_ap_alloc->regs;

	for (int i = 0; i < NUM_REGS; i++) {
		regs[i].number = i;
		regs[i].value = mem_ap_alloc->regs_value;
		regs[i].size = REG_SIZE(i);
		regs[i].exist = REG_EXIST(i);
		regs[i].type = &mem_ap_reg_arch_type;
		(*reg_list)[i] = &regs[i];
	}

	return ERROR_OK;
}

static int mem_ap_read_memory(struct target *target, target_addr_t address,
			       uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct mem_ap *mem_ap = target->arch_info;

	LOG_DEBUG("Reading memory at physical address " TARGET_ADDR_FMT
		  "; size %" PRIu32 "; count %" PRIu32, address, size, count);

	if (count == 0 || !buffer)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return mem_ap_read_buf(mem_ap->ap, buffer, size, count, address);
}

static int mem_ap_write_memory(struct target *target, target_addr_t address,
				uint32_t size, uint32_t count,
				const uint8_t *buffer)
{
	struct mem_ap *mem_ap = target->arch_info;

	LOG_DEBUG("Writing memory at physical address " TARGET_ADDR_FMT
		  "; size %" PRIu32 "; count %" PRIu32, address, size, count);

	if (count == 0 || !buffer)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return mem_ap_write_buf(mem_ap->ap, buffer, size, count, address);
}

struct target_type mem_ap_target = {
	.name = "mem_ap",

	.target_create = mem_ap_target_create,
	.init_target = mem_ap_init_target,
	.deinit_target = mem_ap_deinit_target,
	.examine = mem_ap_examine,
	.target_jim_configure = adiv5_jim_configure,

	.poll = mem_ap_poll,
	.arch_state = mem_ap_arch_state,

	.halt = mem_ap_halt,
	.resume = mem_ap_resume,
	.step = mem_ap_step,

	.assert_reset = mem_ap_assert_reset,
	.deassert_reset = mem_ap_deassert_reset,

	.get_gdb_arch = mem_ap_get_gdb_arch,
	.get_gdb_reg_list = mem_ap_get_gdb_reg_list,

	.read_memory = mem_ap_read_memory,
	.write_memory = mem_ap_write_memory,
};
