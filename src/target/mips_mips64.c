/*
 * MIPS64 generic target support
 *
 * Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>
 * Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>
 * Copyright (C) 2014-2019 by Peter Mamonov <pmamonov@gmail.com>
 *
 * Based on the work of:
 *     Copyright (C) 2008 by Spencer Oliver
 *     Copyright (C) 2008 by David T.L. Wong
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "mips32.h"
#include "mips64.h"
#include "mips_mips64.h"
#include "target_type.h"
#include "register.h"

static int mips_mips64_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint);

static uint64_t mips64_extend_sign(uint64_t addr)
{
	if (addr >> 32)
		return addr;
	if (addr >> 31)
		return addr | (ULLONG_MAX << 32);
	return addr;
}

static int mips_mips64_examine_debug_reason(struct target *target)
{
	if ((target->debug_reason != DBG_REASON_DBGRQ)
		&& (target->debug_reason != DBG_REASON_SINGLESTEP))
		target->debug_reason = DBG_REASON_BREAKPOINT;

	return ERROR_OK;
}

static int mips_mips64_debug_entry(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	struct reg *pc = &mips64->core_cache->reg_list[MIPS64_PC];

	mips64_save_context(target);

	/* make sure stepping disabled, SSt bit in CP0 debug register cleared */
	mips64_ejtag_config_step(ejtag_info, 0);

	/* make sure break unit configured */
	mips64_configure_break_unit(target);

	/* attempt to find halt reason */
	mips_mips64_examine_debug_reason(target);

	LOG_DEBUG("entered debug state at PC 0x%" PRIx64 ", target->state: %s",
		  buf_get_u64(pc->value, 0, 64), target_state_name(target));

	return ERROR_OK;
}

static int mips_mips64_poll(struct target *target)
{
	int retval;
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl;

	/* read ejtag control reg */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* clear this bit before handling polling
	 * as after reset registers will read zero */
	if (ejtag_ctrl & EJTAG_CTRL_ROCC) {
		/* we have detected a reset, clear flag
		 * otherwise ejtag will not work */
		ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_ROCC;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		LOG_DEBUG("Reset Detected");
	}

	/* check for processor halted */
	if (ejtag_ctrl & EJTAG_CTRL_BRKST) {
		if ((target->state == TARGET_RUNNING) || (target->state == TARGET_RESET)) {
			target->state = TARGET_HALTED;
			retval = mips_mips64_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		} else if (target->state == TARGET_DEBUG_RUNNING) {
			target->state = TARGET_HALTED;
			retval = mips_mips64_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	} else {
		target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

static int mips_mips64_halt(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;

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
			/* we came here in a reset_halt or reset_init sequence
			 * debug entry was already prepared in mips64_prepare_reset_halt()
			 */
			target->debug_reason = DBG_REASON_DBGRQ;

			return ERROR_OK;
		}
	}

	/* break processor */
	mips_ejtag_enter_debug(ejtag_info);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int mips_mips64_assert_reset(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	int retval;

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (!(jtag_reset_config & RESET_HAS_SRST)) {
		LOG_ERROR("Can't assert SRST");
		return ERROR_FAIL;
	}

	if (target->reset_halt)
		/* use hardware to catch reset */
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_EJTAGBOOT);
	else
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_NORMALBOOT);

	/* here we should issue a srst only, but we may have to assert trst as well */
	if (jtag_reset_config & RESET_SRST_PULLS_TRST)
		jtag_add_reset(1, 1);
	else
		jtag_add_reset(0, 1);

	target->state = TARGET_RESET;
	jtag_add_sleep(5000);

	retval = mips64_invalidate_core_regs(target);
	if (retval != ERROR_OK)
		return retval;

	if (target->reset_halt) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mips_mips64_deassert_reset(struct target *target)
{
	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	return ERROR_OK;
}

static int mips_mips64_soft_reset_halt(struct target *target)
{
	/* TODO */
	return ERROR_OK;
}

static int mips_mips64_single_step_core(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	int retval;

	/* configure single step mode */
	mips64_ejtag_config_step(ejtag_info, 1);

	/* disable interrupts while stepping */
	retval = mips64_enable_interrupts(target, false);
	if (retval != ERROR_OK)
		return retval;

	/* exit debug mode */
	retval = mips64_ejtag_exit_debug(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	mips_mips64_debug_entry(target);

	return ERROR_OK;
}

/* TODO: HW breakpoints are in EJTAG spec. Should we share it for MIPS32? */
static int mips_mips64_set_hwbp(struct target *target, struct breakpoint *bp)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *c, *cl = mips64->inst_break_list;
	uint64_t bp_value;
	int retval, bp_num = 0;

	while (cl[bp_num].used && (bp_num < mips64->num_inst_bpoints))
		bp_num++;

	if (bp_num >= mips64->num_inst_bpoints) {
		LOG_DEBUG("ERROR Can not find free FP Comparator(bpid: %" PRIu32 ")",
			  bp->unique_id);
		LOG_WARNING("ERROR Can not find free FP Comparator");
		exit(-1);
	}

	c = &cl[bp_num];
	c->used = true;
	c->bp_value = bp->address;
	bp_value = bp->address;

	/* Instruction Breakpoint Address n (IBAn) Register */
	retval = target_write_u64(target, c->reg_address, bp_value);
	if (retval != ERROR_OK)
		return retval;

	/* TODO: use defines */
	/* Instruction Breakpoint Address Mask n (IBMn) Register */
	retval = target_write_u64(target, c->reg_address + 0x08, 0);
	if (retval != ERROR_OK)
		return retval;

	/* Instruction Breakpoint Control n (IBCn) Register */
	retval = target_write_u64(target, c->reg_address + 0x18, 1);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("bpid: %" PRIu32 ", bp_num %i bp_value 0x%" PRIx64, bp->unique_id,
		  bp_num, c->bp_value);

	return ERROR_OK;
}

/* TODO: is it MIPS64 or MIPS32 instruction. If MIPS32, can it be shared with
 * MIPS32 code? */
static int mips_mips64_set_sdbbp(struct target *target, struct breakpoint *bp)
{
	uint32_t verify;
	int retval;

	retval = target_read_memory(target,
				    bp->address, bp->length, 1,
				    bp->orig_instr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, bp->address, MIPS64_SDBBP);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, bp->address, &verify);
	if (retval != ERROR_OK)
		return retval;

	if (verify != MIPS64_SDBBP) {
		LOG_ERROR("Unable to set 32bit breakpoint at address %16" PRIx64,
			  bp->address);
		retval = ERROR_FAIL;
	}

	return retval;
}

/* TODO do MIPS64 support MIPS16 instructions? Can it be shared with MIPS32
 * code? */
static int mips_mips16_set_sdbbp(struct target *target, struct breakpoint *bp)
{
	uint32_t isa_req = bp->length & 1;
	uint16_t verify;
	int retval;

	retval = target_read_memory(target,
				    bp->address, bp->length, 1,
				    bp->orig_instr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u16(target, bp->address, MIPS16_SDBBP(isa_req));
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u16(target, bp->address, &verify);
	if (retval != ERROR_OK)
		return retval;

	if (verify != MIPS16_SDBBP(isa_req)) {
		LOG_ERROR("Unable to set 16bit breakpoint at address %16" PRIx64,
			  bp->address);
		retval = ERROR_FAIL;
	}

	return retval;
}

static int mips_mips64_set_breakpoint(struct target *target,
				      struct breakpoint *bp)
{
	int retval;

	if (bp->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (bp->type == BKPT_HARD) {
		retval = mips_mips64_set_hwbp(target, bp);
	} else {
		LOG_DEBUG("bpid: %" PRIu32, bp->unique_id);

		switch (bp->length) {
		case MIPS64_SDBBP_SIZE:
			retval = mips_mips64_set_sdbbp(target, bp);
			break;
		case MIPS16_SDBBP_SIZE:
			retval = mips_mips16_set_sdbbp(target, bp);
			break;
		default:
			retval = ERROR_FAIL;
		}
	}

	if (retval != ERROR_OK) {
		LOG_ERROR("can't unset breakpoint. Some thing wrong happened");
		return retval;
	}

	bp->set = true;

	return ERROR_OK;
}

static int mips_mips64_enable_breakpoints(struct target *target)
{
	struct breakpoint *bp = target->breakpoints;
	int retval = ERROR_OK;

	/* set any pending breakpoints */
	while (bp) {
		if (!bp->set) {
			retval = mips_mips64_set_breakpoint(target, bp);
			if (retval != ERROR_OK)
				return retval;
		}
		bp = bp->next;
	}

	return ERROR_OK;
}

/* TODO: HW data breakpoints are in EJTAG spec. Should we share it for MIPS32? */
static int mips_mips64_set_watchpoint(struct target *target,
				      struct watchpoint *watchpoint)
{
	uint64_t wp_value;
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *c, *cl = mips64->data_break_list;
	int retval, wp_num = 0;

	/*
	 * watchpoint enabled, ignore all byte lanes in value register
	 * and exclude both load and store accesses from  watchpoint
	 * condition evaluation
	*/
	int enable = EJTAG_DBCn_NOSB | EJTAG_DBCn_NOLB | EJTAG_DBCn_BE
		| (0xff << EJTAG_DBCn_BLM_SHIFT);

	if (watchpoint->set) {
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}

	while (cl[wp_num].used && (wp_num < mips64->num_data_bpoints))
		wp_num++;

	if (wp_num >= mips64->num_data_bpoints) {
		LOG_ERROR("ERROR Can not find free comparator");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (watchpoint->length != 4) {
		LOG_ERROR("Only watchpoints of length 4 are supported");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	if (watchpoint->address % 4) {
		LOG_ERROR("Watchpoints address should be word aligned");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	switch (watchpoint->rw)	{
	case WPT_READ:
		enable &= ~EJTAG_DBCn_NOLB;
		break;
	case WPT_WRITE:
		enable &= ~EJTAG_DBCn_NOSB;
		break;
	case WPT_ACCESS:
		enable &= ~(EJTAG_DBCn_NOLB | EJTAG_DBCn_NOSB);
		break;
	default:
		LOG_ERROR("BUG: watchpoint->rw neither read, write nor access");
	}

	c = &cl[wp_num];
	watchpoint->set = wp_num + 1;
	c->used = true;
	c->bp_value = watchpoint->address;

	wp_value = watchpoint->address;
	if (wp_value & 0x80000000)
		wp_value |= ULLONG_MAX << 32;

	retval = target_write_u64(target, c->reg_address, wp_value);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u64(target, c->reg_address + 0x08, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u64(target, c->reg_address + 0x10, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u64(target, c->reg_address + 0x18, enable);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u64(target, c->reg_address + 0x20, 0);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("wp_num %i bp_value 0x%" PRIx64 "", wp_num, c->bp_value);

	return ERROR_OK;
}

static int mips_mips64_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;
	int retval;

	/* set any pending watchpoints */
	while (watchpoint) {
		if (watchpoint->set == 0) {
			retval = mips_mips64_set_watchpoint(target, watchpoint);
			if (retval != ERROR_OK)
				return retval;
		}
		watchpoint = watchpoint->next;
	}

	return ERROR_OK;
}

static int mips_mips64_unset_hwbp(struct target *target, struct breakpoint *bp)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *comparator_list = mips64->inst_break_list;
	int bp_num;

	bp_num = bp->set - 1;

	if ((bp_num < 0) || (bp_num >= mips64->num_inst_bpoints)) {
		LOG_DEBUG("Invalid FP Comparator number in breakpoint (bpid: %" PRIu32 ")",
			  bp->unique_id);
		return ERROR_OK;
	}

	LOG_DEBUG("bpid: %" PRIu32 " - releasing hw: %d", bp->unique_id, bp_num);
	comparator_list[bp_num].used = false;
	comparator_list[bp_num].bp_value = 0;

	return target_write_u64(target,
				comparator_list[bp_num].reg_address + 0x18, 0);
}

static int mips_mips64_unset_sdbbp(struct target *target, struct breakpoint *bp)
{
	uint8_t buf[MIPS64_SDBBP_SIZE];
	uint32_t instr;
	int retval;

	retval = target_read_memory(target, bp->address, MIPS64_SDBBP_SIZE, 1,
				    &buf[0]);
	if (retval != ERROR_OK)
		return retval;

	instr = target_buffer_get_u32(target, &buf[0]);
	if (instr != MIPS64_SDBBP)
		return ERROR_OK;

	return target_write_memory(target, bp->address, MIPS64_SDBBP_SIZE, 1,
				   bp->orig_instr);
}

static int mips_mips16_unset_sdbbp(struct target *target, struct breakpoint *bp)
{
	uint8_t buf[MIPS16_SDBBP_SIZE];
	uint16_t instr;
	int retval;

	retval = target_read_memory(target, bp->address, MIPS16_SDBBP_SIZE, 1,
				    &buf[0]);
	if (retval != ERROR_OK)
		return retval;

	instr = target_buffer_get_u16(target, &buf[0]);
	if (instr != MIPS16_SDBBP(bp->length & 1))
		return ERROR_OK;

	return target_write_memory(target, bp->address, MIPS16_SDBBP_SIZE, 1,
				   bp->orig_instr);
}

static int mips_mips64_unset_breakpoint(struct target *target,
					struct breakpoint *bp)
{
	/* get pointers to arch-specific information */
	int retval;

	if (!bp->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (bp->type == BKPT_HARD) {
		retval = mips_mips64_unset_hwbp(target, bp);
	} else {
		LOG_DEBUG("bpid: %" PRIu32, bp->unique_id);

		switch (bp->length) {
		case MIPS64_SDBBP_SIZE:
			retval = mips_mips64_unset_sdbbp(target, bp);
			break;
		case MIPS16_SDBBP_SIZE:
			retval = mips_mips16_unset_sdbbp(target, bp);
			break;
		default:
			retval = ERROR_FAIL;
		}
	}
	if (retval != ERROR_OK) {
		LOG_ERROR("can't unset breakpoint. Some thing wrong happened");
		return retval;
	}

	bp->set = false;

	return ERROR_OK;
}

static int mips_mips64_resume(struct target *target, int current,
			      uint64_t address, int handle_breakpoints,
			      int debug_execution)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	int retval = ERROR_OK;
	uint64_t resume_pc;
	struct reg *pc;

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted %d", target->state);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);
		retval = mips_mips64_enable_breakpoints(target);
		if (retval != ERROR_OK)
			return retval;

		retval = mips_mips64_enable_watchpoints(target);
		if (retval != ERROR_OK)
			return retval;
	}

	pc = &mips64->core_cache->reg_list[MIPS64_PC];
	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u64(pc->value, 0, 64, address);
		pc->dirty = 1;
		pc->valid = 1;
	}

	resume_pc = buf_get_u64(pc->value, 0, 64);

	retval = mips64_restore_context(target);
	if (retval != ERROR_OK)
		return retval;

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		struct breakpoint *bp;

		/* Single step past breakpoint at current address */
		bp = breakpoint_find(target, (uint64_t) resume_pc);
		if (bp) {
			LOG_DEBUG("unset breakpoint at 0x%16.16" PRIx64 "",
				  bp->address);
			retval = mips_mips64_unset_breakpoint(target, bp);
			if (retval != ERROR_OK)
				return retval;

			retval = mips_mips64_single_step_core(target);
			if (retval != ERROR_OK)
				return retval;

			retval = mips_mips64_set_breakpoint(target, bp);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* enable interrupts if we are running */
	retval = mips64_enable_interrupts(target, !debug_execution);
	if (retval != ERROR_OK)
		return retval;

	/* exit debug mode */
	retval = mips64_ejtag_exit_debug(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	retval = mips64_invalidate_core_regs(target);
	if (retval != ERROR_OK)
		return retval;

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		retval = target_call_event_callbacks(target,
						     TARGET_EVENT_RESUMED);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("target resumed at 0x%" PRIx64 "", resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		retval = target_call_event_callbacks(target,
						     TARGET_EVENT_DEBUG_RESUMED);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("target debug resumed at 0x%" PRIx64 "", resume_pc);
	}

	return ERROR_OK;
}

static int mips_mips64_step(struct target *target, int current,
			    uint64_t address, int handle_breakpoints)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	struct reg *pc = &mips64->core_cache->reg_list[MIPS64_PC];
	struct breakpoint *bp = NULL;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	/* current = 1: continue on current pc, otherwise continue at
	 * <address> */
	if (!current) {
		buf_set_u64(pc->value, 0, 64, address);
		pc->dirty = 1;
		pc->valid = 1;
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		bp = breakpoint_find(target, buf_get_u64(pc->value, 0, 64));
		if (bp) {
			retval = mips_mips64_unset_breakpoint(target, bp);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	retval = mips64_restore_context(target);
	if (retval != ERROR_OK)
		return retval;

	/* configure single step mode */
	retval = mips64_ejtag_config_step(ejtag_info, 1);
	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_SINGLESTEP;

	retval = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	if (retval != ERROR_OK)
		return retval;

	/* disable interrupts while stepping */
	retval = mips64_enable_interrupts(target, false);
	if (retval != ERROR_OK)
		return retval;

	/* exit debug mode */
	retval = mips64_ejtag_exit_debug(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	/* registers are now invalid */
	retval = mips64_invalidate_core_regs(target);
	if (retval != ERROR_OK)
		return retval;

	if (bp) {
		retval = mips_mips64_set_breakpoint(target, bp);
		if (retval != ERROR_OK)
			return retval;
	}

	LOG_DEBUG("target stepped ");

	retval = mips_mips64_debug_entry(target);
	if (retval != ERROR_OK)
		return retval;

	return target_call_event_callbacks(target, TARGET_EVENT_HALTED);
}

static int mips_mips64_add_breakpoint(struct target *target,
				      struct breakpoint *bp)
{
	struct mips64_common *mips64 = target->arch_info;

	if (mips64->mips64mode32)
		bp->address = mips64_extend_sign(bp->address);

	if (bp->type == BKPT_HARD) {
		if (mips64->num_inst_bpoints_avail < 1) {
			LOG_INFO("no hardware breakpoint available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		mips64->num_inst_bpoints_avail--;
	}

	return mips_mips64_set_breakpoint(target, bp);
}

static int mips_mips64_remove_breakpoint(struct target *target,
					 struct breakpoint *bp)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (bp->set)
		retval = mips_mips64_unset_breakpoint(target, bp);

	if (bp->type == BKPT_HARD)
		mips64->num_inst_bpoints_avail++;

	return retval;
}

static int mips_mips64_unset_watchpoint(struct target *target,
					struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *comparator_list = mips64->data_break_list;

	if (!watchpoint->set) {
		LOG_WARNING("watchpoint not set");
		return ERROR_OK;
	}

	int wp_num = watchpoint->set - 1;
	if ((wp_num < 0) || (wp_num >= mips64->num_data_bpoints)) {
		LOG_DEBUG("Invalid FP Comparator number in watchpoint");
		return ERROR_OK;
	}
	comparator_list[wp_num].used = false;
	comparator_list[wp_num].bp_value = 0;
	target_write_u64(target, comparator_list[wp_num].reg_address + 0x18, 0);
	watchpoint->set = 0;

	return ERROR_OK;
}

static int mips_mips64_add_watchpoint(struct target *target,
				      struct watchpoint *watchpoint)
{
	struct mips64_common *mips64 = target->arch_info;

	if (mips64->num_data_bpoints_avail < 1) {
		LOG_INFO("no hardware watchpoints available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	mips64->num_data_bpoints_avail--;

	return mips_mips64_set_watchpoint(target, watchpoint);
}

static int mips_mips64_remove_watchpoint(struct target *target,
					 struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
		retval = mips_mips64_unset_watchpoint(target, watchpoint);

	mips64->num_data_bpoints_avail++;

	return retval;
}

static int mips_mips64_read_memory(struct target *target, uint64_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	int retval;
	void *t;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted %d", target->state);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	/* sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1))
	    || !count || !buffer)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (((size == 8) && (address & 0x7)) || ((size == 4) && (address & 0x3))
	    || ((size == 2) && (address & 0x1)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (size > 1) {
		t = calloc(count, size);
		if (!t) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
	} else
		t = buffer;

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		  address, size, count);
	retval = mips64_pracc_read_mem(ejtag_info, address, size, count,
				       (void *)t);

	if (ERROR_OK != retval) {
		LOG_ERROR("mips64_pracc_read_mem filed");
		goto read_done;
	}

	switch (size) {
	case 8:
		target_buffer_set_u64_array(target, buffer, count, t);
		break;
	case 4:
		target_buffer_set_u32_array(target, buffer, count, t);
		break;
	case 2:
		target_buffer_set_u16_array(target, buffer, count, t);
		break;
	}

read_done:
	if (size > 1)
		free(t);

	return retval;
}

static int mips_mips64_bulk_write_memory(struct target *target,
					 target_addr_t address, uint32_t count,
					 const uint8_t *buffer)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	struct working_area *fast_data_area;
	int retval;

	LOG_DEBUG("address: " TARGET_ADDR_FMT ", count: 0x%8.8" PRIx32 "",
		  address, count);

	if (address & 0x7)
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (!mips64->fast_data_area) {
		/* Get memory for block write handler
		 * we preserve this area between calls and gain a speed increase
		 * of about 3kb/sec when writing flash
		 * this will be released/nulled by the system when the target is resumed or reset */
		retval = target_alloc_working_area(target,
						   MIPS64_FASTDATA_HANDLER_SIZE,
						   &mips64->fast_data_area);
		if (retval != ERROR_OK) {
			LOG_ERROR("No working area available");
			return retval;
		}

		/* reset fastadata state so the algo get reloaded */
		ejtag_info->fast_access_save = -1;
	}

	fast_data_area = mips64->fast_data_area;

	if (address <= fast_data_area->address + fast_data_area->size &&
	    fast_data_area->address <= address + count) {
		LOG_ERROR("fast_data (" TARGET_ADDR_FMT ") is within write area "
			  "(" TARGET_ADDR_FMT "-" TARGET_ADDR_FMT ").",
			  fast_data_area->address, address, address + count);
		LOG_ERROR("Change work-area-phys or load_image address!");
		return ERROR_FAIL;
	}

	/* mips32_pracc_fastdata_xfer requires uint32_t in host endianness, */
	/* but byte array represents target endianness                      */
	uint64_t *t;

	t = calloc(count, sizeof(uint64_t));
	if (!t) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	target_buffer_get_u64_array(target, buffer, count, t);

	retval = mips64_pracc_fastdata_xfer(ejtag_info, mips64->fast_data_area,
					    true, address, count, t);

	if (retval != ERROR_OK)
		LOG_ERROR("Fastdata access Failed");

	free(t);

	return retval;
}

static int mips_mips64_write_memory(struct target *target, uint64_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	/* sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1))
	    || !count || !buffer)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (((size == 8) && (address & 0x7)) || ((size == 4) && (address & 0x3))
	    || ((size == 2) && (address & 0x1)))
		return ERROR_TARGET_UNALIGNED_ACCESS;



	if (size == 8 && count > 8) {
		retval = mips_mips64_bulk_write_memory(target, address, count,
						       buffer);
		if (retval == ERROR_OK)
			return ERROR_OK;

		LOG_WARNING("Falling back to non-bulk write");
	}

	void *t = NULL;
	if (size > 1) {
		t = calloc(count, size);
		if (!t) {
			LOG_ERROR("unable to allocate t for write buffer");
			return ERROR_FAIL;
		}

		switch (size) {
		case 8:
			target_buffer_get_u64_array(target, buffer, count,
						    (uint64_t *)t);
			break;
		case 4:
			target_buffer_get_u32_array(target, buffer, count,
						    (uint32_t *)t);
			break;
		case 2:
			target_buffer_get_u16_array(target, buffer, count,
						    (uint16_t *)t);
			break;
		}
		buffer = t;
	}

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		  address, size, count);

	retval = mips64_pracc_write_mem(ejtag_info, address, size, count,
					(void *)buffer);
	free(t);

	return retval;
}

static int mips_mips64_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	return mips64_build_reg_cache(target);
}

static int mips_mips64_target_create(struct target *target, Jim_Interp *interp)
{
	struct mips_mips64_common *mips_mips64;
	struct mips64_common *mips64;

	mips_mips64 = calloc(1, sizeof(*mips_mips64));
	if (!mips_mips64) {
		LOG_ERROR("unable to allocate mips_mips64");
		return ERROR_FAIL;
	}

	mips_mips64->common_magic = MIPS64_COMMON_MAGIC;

	mips64 = &mips_mips64->mips64_common;
	mips64->arch_info = mips_mips64;
	target->arch_info = mips64;

	return mips64_init_arch_info(target, mips64, target->tap);
}

static int mips_mips64_examine(struct target *target)
{
	int retval;
	struct mips64_common *mips64 = target->arch_info;

	retval = mips_ejtag_init(&mips64->ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	return mips64_examine(target);
}

static int mips_mips64_checksum_memory(struct target *target, uint64_t address,
				       uint32_t size, uint32_t *checksum)
{
	return ERROR_FAIL; /* use bulk read method */
}

COMMAND_HANDLER(handle_mips64mode32)
{
	struct target *target = get_current_target(CMD_CTX);
	struct mips64_common *mips64 = target->arch_info;

	if (CMD_ARGC > 0)
		COMMAND_PARSE_BOOL(CMD_ARGV[0], mips64->mips64mode32, "on", "off");

	if (mips64->mips64mode32)
		command_print(CMD, "enabled");
	else
		command_print(CMD, "disabled");

	return ERROR_OK;
}


static const struct command_registration mips64_commands_handlers[] = {
	{
		.name = "mips64mode32",
		.mode = COMMAND_EXEC,
		.help = "Enable/disable 32 bit mode",
		.usage = "[1|0]",
		.handler = handle_mips64mode32
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type mips_mips64_target = {
	.name = "mips_mips64",

	.poll = mips_mips64_poll,
	.arch_state = mips64_arch_state,

	.target_request_data = NULL,

	.halt = mips_mips64_halt,
	.resume = mips_mips64_resume,
	.step = mips_mips64_step,

	.assert_reset = mips_mips64_assert_reset,
	.deassert_reset = mips_mips64_deassert_reset,
	.soft_reset_halt = mips_mips64_soft_reset_halt,

	.get_gdb_reg_list = mips64_get_gdb_reg_list,

	.read_memory = mips_mips64_read_memory,
	.write_memory = mips_mips64_write_memory,
	.checksum_memory = mips_mips64_checksum_memory,
	.blank_check_memory = NULL,

	.run_algorithm = mips64_run_algorithm,

	.add_breakpoint = mips_mips64_add_breakpoint,
	.remove_breakpoint = mips_mips64_remove_breakpoint,
	.add_watchpoint = mips_mips64_add_watchpoint,
	.remove_watchpoint = mips_mips64_remove_watchpoint,

	.target_create = mips_mips64_target_create,
	.init_target = mips_mips64_init_target,
	.examine = mips_mips64_examine,

	.commands = mips64_commands_handlers,
};
