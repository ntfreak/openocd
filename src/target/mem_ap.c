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
#include "arm.h"
#include "arm_adi_v5.h"

#include <jtag/jtag.h>

struct mem_ap {
	struct arm arm;
	struct adiv5_ap *ap;
	int ap_num;
};

static int mem_ap_target_create(struct target *target, Jim_Interp *interp)
{
	struct mem_ap *mem_ap;
	struct adiv5_private_config *pc;

	pc = (struct adiv5_private_config *)target->private_config;
	if (pc == NULL)
		return ERROR_FAIL;

	if (pc->ap_num == DP_APSEL_INVALID) {
		LOG_ERROR("AP number not specified");
		return ERROR_FAIL;
	}

	mem_ap = calloc(1, sizeof(struct mem_ap));
	if (mem_ap == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	mem_ap->ap_num = pc->ap_num;
	mem_ap->arm.common_magic = ARM_COMMON_MAGIC;
	mem_ap->arm.dap = pc->dap;

	target->arch_info = mem_ap;

	return ERROR_OK;
}

static int mem_ap_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_UNKNOWN;
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
	if (target->state == TARGET_UNKNOWN)
		target->state = TARGET_RUNNING;

	return ERROR_OK;
}

static int mem_ap_halt(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_HALTED;
	return ERROR_OK;
}

static int mem_ap_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

static int mem_ap_step(struct target *target, int current, target_addr_t address,
				int handle_breakpoints)
{
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_HALTED;
	return ERROR_OK;
}

static int mem_ap_assert_reset(struct target *target)
{
	target->state = TARGET_RESET;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int mem_ap_examine(struct target *target)
{
	struct mem_ap *mem_ap = target->arch_info;

	if (!target_was_examined(target)) {
		mem_ap->ap = dap_ap(mem_ap->arm.dap, mem_ap->ap_num);
		target_set_examined(target);
		target->state = TARGET_UNKNOWN;
		return mem_ap_init(mem_ap->ap);
	}

	return ERROR_OK;
}

static int mem_ap_deassert_reset(struct target *target)
{
	if (target->reset_halt)
		target->state = TARGET_HALTED;
	else
		target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int mem_ap_read_memory(struct target *target, target_addr_t address,
			       uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct mem_ap *mem_ap = target->arch_info;

	LOG_DEBUG("Reading memory at physical address " TARGET_ADDR_FMT
		  "; size %" PRIu32 "; count %" PRIu32, address, size, count);

	if (count == 0 || buffer == NULL)
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

	if (count == 0 || buffer == NULL)
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

	.read_memory = mem_ap_read_memory,
	.write_memory = mem_ap_write_memory,
};
