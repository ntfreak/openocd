/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
 *   Based on mips_m4k code:                                               *
 *       Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>       *
 *       Copyright (C) 2008 by David T.L. Wong                             *
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

#include "jtag/jtag.h"
#include "register.h"
#include "algorithm.h"
#include "target.h"
#include "breakpoints.h"
#include "target_type.h"
#include "avr32_jtag.h"
#include "avr32_mem.h"
#include "avr32_regs.h"
#include "avr32_ap7k.h"

static const char * const avr32_core_reg_list[] = {
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8",
	"r9", "r10", "r11", "r12", "sp", "lr", "pc", "sr"
};

static const struct avr32_core_reg
	avr32_core_reg_list_arch_info[AVR32NUMCOREREGS] = {
	{0, NULL, NULL},
	{1, NULL, NULL},
	{2, NULL, NULL},
	{3, NULL, NULL},
	{4, NULL, NULL},
	{5, NULL, NULL},
	{6, NULL, NULL},
	{7, NULL, NULL},
	{8, NULL, NULL},
	{9, NULL, NULL},
	{10, NULL, NULL},
	{11, NULL, NULL},
	{12, NULL, NULL},
	{13, NULL, NULL},
	{14, NULL, NULL},
	{15, NULL, NULL},
	{16, NULL, NULL},
};


static int avr32_read_core_reg(struct target *target, int num);
static int avr32_write_core_reg(struct target *target, int num);

static int avr32_ap7k_save_context(struct target *target)
{
	int retval, i;
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	retval = avr32_jtag_read_regs(&ap7k->jtag, ap7k->core_regs);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < AVR32NUMCOREREGS; i++) {
		if (!ap7k->core_cache->reg_list[i].valid)
			avr32_read_core_reg(target, i);
	}

	return ERROR_OK;
}

static int avr32_ap7k_restore_context(struct target *target)
{
	int i;

	/* get pointers to arch-specific information */
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	for (i = 0; i < AVR32NUMCOREREGS; i++) {
		if (ap7k->core_cache->reg_list[i].dirty)
			avr32_write_core_reg(target, i);
	}

	/* write core regs */
	avr32_jtag_write_regs(&ap7k->jtag, ap7k->core_regs);

	return ERROR_OK;
}

static int avr32_read_core_reg(struct target *target, int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	if ((num < 0) || (num >= AVR32NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = ap7k->core_regs[num];
	buf_set_u32(ap7k->core_cache->reg_list[num].value, 0, 32, reg_value);
	ap7k->core_cache->reg_list[num].valid = true;
	ap7k->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int avr32_write_core_reg(struct target *target, int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	if ((num < 0) || (num >= AVR32NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = buf_get_u32(ap7k->core_cache->reg_list[num].value, 0, 32);
	ap7k->core_regs[num] = reg_value;
	LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", num, reg_value);
	ap7k->core_cache->reg_list[num].valid = true;
	ap7k->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int avr32_get_core_reg(struct reg *reg)
{
	int retval;
	struct avr32_core_reg *avr32_reg = reg->arch_info;
	struct target *target = avr32_reg->target;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = avr32_read_core_reg(target, avr32_reg->num);

	return retval;
}

static int avr32_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct avr32_core_reg *avr32_reg = reg->arch_info;
	struct target *target = avr32_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static const struct reg_arch_type avr32_reg_type = {
	.get = avr32_get_core_reg,
	.set = avr32_set_core_reg,
};

static struct reg_cache *avr32_build_reg_cache(struct target *target)
{
	int num_regs = AVR32NUMCOREREGS;
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct avr32_core_reg *arch_info =
		malloc(sizeof(struct avr32_core_reg) * num_regs);
	int i;

	/* Build the process context cache */
	cache->name = "avr32 registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	ap7k->core_cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i] = avr32_core_reg_list_arch_info[i];
		arch_info[i].target = target;
		arch_info[i].avr32_common = ap7k;
		reg_list[i].name = avr32_core_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].type = &avr32_reg_type;
		reg_list[i].arch_info = &arch_info[i];
	}

	return cache;
}

static int avr32_ap7k_debug_entry(struct target *target)
{

	uint32_t dpc, dinst;
	int retval;
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	retval = avr32_jtag_nexus_read(&ap7k->jtag, AVR32_OCDREG_DPC, &dpc);
	if (retval != ERROR_OK)
		return retval;

	retval = avr32_jtag_nexus_read(&ap7k->jtag, AVR32_OCDREG_DINST, &dinst);
	if (retval != ERROR_OK)
		return retval;

	ap7k->jtag.dpc = dpc;

	avr32_ap7k_save_context(target);

	return ERROR_OK;
}


static int avr32_ap7k_poll(struct target *target)
{
	uint32_t ds;
	int retval;
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	retval = avr32_jtag_nexus_read(&ap7k->jtag, AVR32_OCDREG_DS, &ds);
	if (retval != ERROR_OK)
		return retval;

	/* check for processor halted */
	if (ds & OCDREG_DS_DBA) {
		if ((target->state == TARGET_RUNNING) || (target->state == TARGET_RESET)) {
			target->state = TARGET_HALTED;

			retval = avr32_ap7k_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		} else if (target->state == TARGET_DEBUG_RUNNING) {
			target->state = TARGET_HALTED;

			retval = avr32_ap7k_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	} else
		target->state = TARGET_RUNNING;


	return ERROR_OK;
}

static int avr32_ap7k_halt(struct target *target)
{
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	if (target->state == TARGET_RESET) {
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) && jtag_get_srst()) {
			LOG_ERROR("can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		} else {
			target->debug_reason = DBG_REASON_DBGRQ;

			return ERROR_OK;
		}
	}


	avr32_ocd_setbits(&ap7k->jtag, AVR32_OCDREG_DC, OCDREG_DC_DBR);
	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int avr32_ap7k_assert_reset(struct target *target)
{
	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32_ap7k_deassert_reset(struct target *target)
{
	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32_ap7k_resume(struct target *target, int current,
	target_addr_t address, int handle_breakpoints, int debug_execution)
{
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);
		/*
		avr32_ap7k_enable_breakpoints(target);
		avr32_ap7k_enable_watchpoints(target);
		*/
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
#if 0
		if (retval != ERROR_OK)
			return retval;
#endif
	}

	resume_pc = buf_get_u32(ap7k->core_cache->reg_list[AVR32_REG_PC].value, 0, 32);
	avr32_ap7k_restore_context(target);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at 0x%8.8" TARGET_PRIxADDR "", breakpoint->address);
#if 0
			avr32_ap7k_unset_breakpoint(target, breakpoint);
			avr32_ap7k_single_step_core(target);
			avr32_ap7k_set_breakpoint(target, breakpoint);
#endif
		}
	}

#if 0
	/* enable interrupts if we are running */
	avr32_ap7k_enable_interrupts(target, !debug_execution);

	/* exit debug mode */
	mips_ejtag_exit_debug(ejtag_info);
#endif


	retval = avr32_ocd_clearbits(&ap7k->jtag, AVR32_OCDREG_DC,
			OCDREG_DC_DBR);
	if (retval != ERROR_OK)
		return retval;

	retval = avr32_jtag_exec(&ap7k->jtag, RETD);
	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	register_cache_invalidate(ap7k->core_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32 "", resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32 "", resume_pc);
	}

	return ERROR_OK;
}

static int avr32_ap7k_step(struct target *target, int current,
	target_addr_t address, int handle_breakpoints)
{
	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32_ap7k_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32_ap7k_remove_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32_ap7k_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32_ap7k_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	LOG_ERROR("%s: implement me", __func__);

	return ERROR_OK;
}

static int avr32_ap7k_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	LOG_DEBUG("address: 0x%8.8" TARGET_PRIxADDR ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		address,
		size,
		count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	switch (size) {
		case 4:
			return avr32_jtag_read_memory32(&ap7k->jtag, address, count,
				(uint32_t *)(void *)buffer);
			break;
		case 2:
			return avr32_jtag_read_memory16(&ap7k->jtag, address, count,
				(uint16_t *)(void *)buffer);
			break;
		case 1:
			return avr32_jtag_read_memory8(&ap7k->jtag, address, count, buffer);
		default:
			break;
	}

	return ERROR_OK;
}

static int avr32_ap7k_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	LOG_DEBUG("address: 0x%8.8" TARGET_PRIxADDR ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		address,
		size,
		count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	switch (size) {
		case 4:
			return avr32_jtag_write_memory32(&ap7k->jtag, address, count,
				(uint32_t *)(void *)buffer);
			break;
		case 2:
			return avr32_jtag_write_memory16(&ap7k->jtag, address, count,
				(uint16_t *)(void *)buffer);
			break;
		case 1:
			return avr32_jtag_write_memory8(&ap7k->jtag, address, count, buffer);
		default:
			break;
	}

	return ERROR_OK;
}

static int avr32_ap7k_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	ap7k->jtag.tap = target->tap;
	avr32_build_reg_cache(target);
	return ERROR_OK;
}

static int avr32_ap7k_target_create(struct target *target, Jim_Interp *interp)
{
	struct avr32_ap7k_common *ap7k = calloc(1, sizeof(struct
			avr32_ap7k_common));

	ap7k->common_magic = AP7K_COMMON_MAGIC;
	target->arch_info = ap7k;

	return ERROR_OK;
}

static int avr32_ap7k_examine(struct target *target)
{
	uint32_t devid, ds;
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	if (!target_was_examined(target)) {
		target_set_examined(target);
		avr32_jtag_nexus_read(&ap7k->jtag, AVR32_OCDREG_DID, &devid);
		LOG_INFO("device id: %08" PRIx32, devid);
		avr32_ocd_setbits(&ap7k->jtag, AVR32_OCDREG_DC, OCDREG_DC_DBE);
		avr32_jtag_nexus_read(&ap7k->jtag, AVR32_OCDREG_DS, &ds);

		/* check for processor halted */
		if (ds & OCDREG_DS_DBA) {
			LOG_INFO("target is halted");
			target->state = TARGET_HALTED;
		} else
			target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

static int avr32_ap7k_arch_state(struct target *target)
{
	struct avr32_ap7k_common *ap7k = target_to_ap7k(target);

	LOG_USER("target halted due to %s, pc: 0x%8.8" PRIx32 "",
		debug_reason_name(target), ap7k->jtag.dpc);

	return ERROR_OK;
}

static int avr32_ap7k_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
#if 0
	/* get pointers to arch-specific information */
	int i;

	/* include floating point registers */
	*reg_list_size = AVR32NUMCOREREGS + AVR32NUMFPREGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < AVR32NUMCOREREGS; i++)
		(*reg_list)[i] = &mips32->core_cache->reg_list[i];

	/* add dummy floating points regs */
	for (i = AVR32NUMCOREREGS; i < (AVR32NUMCOREREGS + AVR32NUMFPREGS); i++)
		(*reg_list)[i] = &avr32_ap7k_gdb_dummy_fp_reg;

#endif

	LOG_ERROR("%s: implement me", __func__);
	return ERROR_FAIL;
}

struct target_type avr32_ap7k_target = {
	.name = "avr32_ap7k",

	.poll = avr32_ap7k_poll,
	.arch_state = avr32_ap7k_arch_state,

	.halt = avr32_ap7k_halt,
	.resume = avr32_ap7k_resume,
	.step = avr32_ap7k_step,

	.assert_reset = avr32_ap7k_assert_reset,
	.deassert_reset = avr32_ap7k_deassert_reset,

	.get_gdb_reg_list = avr32_ap7k_get_gdb_reg_list,

	.read_memory = avr32_ap7k_read_memory,
	.write_memory = avr32_ap7k_write_memory,
	/* .checksum_memory = avr32_ap7k_checksum_memory, */
	/* .blank_check_memory = avr32_ap7k_blank_check_memory, */

	/* .run_algorithm = avr32_ap7k_run_algorithm, */

	.add_breakpoint = avr32_ap7k_add_breakpoint,
	.remove_breakpoint = avr32_ap7k_remove_breakpoint,
	.add_watchpoint = avr32_ap7k_add_watchpoint,
	.remove_watchpoint = avr32_ap7k_remove_watchpoint,

	.target_create = avr32_ap7k_target_create,
	.init_target = avr32_ap7k_init_target,
	.examine = avr32_ap7k_examine,
};
