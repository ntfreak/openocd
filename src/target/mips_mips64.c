/***************************************************************************
 *   MIPS64 generic target support                                         *
 *                                                                         *
 *   Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>           *
 *   Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>             *
 *   Copyright (C) 2014 by Peter Mamonov <pmamonov@gmail.com>              *
 *                                                                         *
 *   Based on the work of:                                                 *
 *       Copyright (C) 2008 by Spencer Oliver                              *
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
 *   along with this program. If not, see <http://www.gnu.org/licenses/>.  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if BUILD_TARGET64 == 1

#include "breakpoints.h"
#include "mips64.h"
#include "mips_mips64.h"
#include "target_type.h"
#include "register.h"

static void mips_mips64_enable_breakpoints(struct target *target);
static void mips_mips64_enable_watchpoints(struct target *target);
static int mips_mips64_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
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

	mips64_save_context(target);

	/* make sure stepping disabled, SSt bit in CP0 debug register cleared */
	mips64_ejtag_config_step(ejtag_info, 0);

	/* make sure break unit configured */
	mips64_configure_break_unit(target);

	/* attempt to find halt reason */
	mips_mips64_examine_debug_reason(target);

	LOG_DEBUG("entered debug state at PC 0x%" PRIx64 ", target->state: %s",
		*(uint64_t *)(mips64->core_cache->reg_list[MIPS64_PC].value),
		  target_state_name(target));

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

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (!(jtag_reset_config & RESET_HAS_SRST)) {
		LOG_ERROR("Can't assert SRST");
		return ERROR_FAIL;
	}

	if (target->reset_halt)
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_EJTAGBOOT); /* use hardware to catch reset */
	else
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_NORMALBOOT);

	/* here we should issue a srst only, but we may have to assert trst as well */
	if (jtag_reset_config & RESET_SRST_PULLS_TRST)
		jtag_add_reset(1, 1);
	else
		jtag_add_reset(0, 1);

	target->state = TARGET_RESET;
	jtag_add_sleep(5000);

	mips64_invalidate_core_regs(target);

	if (target->reset_halt) {
		int retval = target_halt(target);
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

	/* configure single step mode */
	mips64_ejtag_config_step(ejtag_info, 1);

	/* disable interrupts while stepping */
	mips64_enable_interrupts(target, 0);

	/* exit debug mode */
	mips64_ejtag_exit_debug(ejtag_info);

	mips_mips64_debug_entry(target);

	return ERROR_OK;
}

static int mips_mips64_resume(struct target *target, int current, uint64_t address,
	int handle_breakpoints, int debug_execution)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	struct breakpoint *breakpoint = NULL;
	uint64_t resume_pc;

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted %d", target->state);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);
		mips_mips64_enable_breakpoints(target);
		mips_mips64_enable_watchpoints(target);
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u64(mips64->core_cache->reg_list[MIPS64_PC].value, 0, 64, (uint64_t) address);
		mips64->core_cache->reg_list[MIPS64_PC].dirty = 1;
		mips64->core_cache->reg_list[MIPS64_PC].valid = 1;
	}

	resume_pc = buf_get_u64(mips64->core_cache->reg_list[MIPS64_PC].value, 0, 64);

	mips64_restore_context(target);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, (uint64_t) resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at 0x%16.16" PRIx64 "", breakpoint->address);
			mips_mips64_unset_breakpoint(target, breakpoint);
			mips_mips64_single_step_core(target);
			mips_mips64_set_breakpoint(target, breakpoint);
		}
	}

	/* enable interrupts if we are running */
	mips64_enable_interrupts(target, !debug_execution);

	/* exit debug mode */
	mips64_ejtag_exit_debug(ejtag_info);
	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	mips64_invalidate_core_regs(target);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx64 "", resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx64 "", resume_pc);
	}

	return ERROR_OK;
}

static int mips_mips64_step(struct target *target, int current, uint64_t address, int handle_breakpoints)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	struct breakpoint *breakpoint = NULL;

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u64(mips64->core_cache->reg_list[MIPS64_PC].value, 0, 64, (uint64_t) address);
		mips64->core_cache->reg_list[MIPS64_PC].dirty = 1;
		mips64->core_cache->reg_list[MIPS64_PC].valid = 1;
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target,
			(uint64_t) buf_get_u64(mips64->core_cache->reg_list[MIPS64_PC].value, 0, 64));
		if (breakpoint)
			mips_mips64_unset_breakpoint(target, breakpoint);
	}

	/* restore context */
	mips64_restore_context(target);

	/* configure single step mode */
	mips64_ejtag_config_step(ejtag_info, 1);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* disable interrupts while stepping */
	mips64_enable_interrupts(target, 0);

	/* exit debug mode */
	mips64_ejtag_exit_debug(ejtag_info);

	/* registers are now invalid */
	mips64_invalidate_core_regs(target);

	if (breakpoint)
		mips_mips64_set_breakpoint(target, breakpoint);

	LOG_DEBUG("target stepped ");

	mips_mips64_debug_entry(target);
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

static void mips_mips64_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint) {
		if (breakpoint->set == 0)
			mips_mips64_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

static int mips_mips64_set_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *comparator_list = mips64->inst_break_list;
	int retval;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = 0;
		uint64_t bp_value;

		while (comparator_list[bp_num].used && (bp_num < mips64->num_inst_bpoints))
			bp_num++;
		if (bp_num >= mips64->num_inst_bpoints) {
			LOG_DEBUG("ERROR Can not find free FP Comparator(bpid: %d)",
					  breakpoint->unique_id);
			LOG_WARNING("ERROR Can not find free FP Comparator");
			exit(-1);
		}
		breakpoint->set = bp_num + 1;
		comparator_list[bp_num].used = 1;
		comparator_list[bp_num].bp_value = breakpoint->address;
		bp_value = breakpoint->address;

		if (bp_value & 0x80000000)
			bp_value |= ULLONG_MAX << 32;

		target_write_u64(target, comparator_list[bp_num].reg_address, bp_value);
		target_write_u64(target, comparator_list[bp_num].reg_address + 0x08, 0x00000000);
		target_write_u64(target, comparator_list[bp_num].reg_address + 0x18, 1);
		LOG_DEBUG("bpid: %d, bp_num %i bp_value 0x%" PRIx64 "",
				  breakpoint->unique_id,
				  bp_num, comparator_list[bp_num].bp_value);
	} else if (breakpoint->type == BKPT_SOFT) {
		LOG_DEBUG("bpid: %d", breakpoint->unique_id);
		if (breakpoint->length == 4) {
			uint32_t verify = 0xffffffff;
			retval = target_read_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u32(target, breakpoint->address, MIPS64_SDBBP);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_u32(target, breakpoint->address, &verify);
			if (retval != ERROR_OK)
				return retval;
			if (verify != MIPS64_SDBBP) {
				LOG_ERROR("Unable to set 32bit breakpoint at address %16" PRIx64, breakpoint->address);
				return ERROR_OK;
			}
		} else {
			uint16_t verify = 0xffff;
			retval = target_read_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u16(target, breakpoint->address, MIPS16_SDBBP);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_u16(target, breakpoint->address, &verify);
			if (retval != ERROR_OK)
				return retval;
			if (verify != MIPS16_SDBBP) {
				LOG_ERROR("Unable to set 16bit breakpoint at address %16" PRIx64, breakpoint->address);
				return ERROR_OK;
			}
		}

		breakpoint->set = 20; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int mips_mips64_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *comparator_list = mips64->inst_break_list;
	int retval;
	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = breakpoint->set - 1;
		if ((bp_num < 0) || (bp_num >= mips64->num_inst_bpoints)) {
			LOG_DEBUG("Invalid FP Comparator number in breakpoint (bpid: %d)",
					  breakpoint->unique_id);
			return ERROR_OK;
		}
		LOG_DEBUG("bpid: %d - releasing hw: %d",
				breakpoint->unique_id,
				bp_num);
		comparator_list[bp_num].used = 0;
		comparator_list[bp_num].bp_value = 0;
		target_write_u64(target, comparator_list[bp_num].reg_address + 0x18, 0);

	} else {
		/* restore original instruction (kept in target endianness) */
		LOG_DEBUG("bpid: %d", breakpoint->unique_id);
		if (breakpoint->length == 4) {
			uint32_t current_instr;

			/* check that user program has not modified breakpoint instruction */
			retval = target_read_memory(target, breakpoint->address, 4, 1, (uint8_t *)&current_instr);
			if (retval != ERROR_OK)
				return retval;
			if (target_buffer_get_u32(target, (uint8_t *)&current_instr) == MIPS64_SDBBP) {
				retval = target_write_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;
			}
		} else {
			uint16_t current_instr;

			/* check that user program has not modified breakpoint instruction */
			retval = target_read_memory(target, breakpoint->address, 2, 1, (uint8_t *)&current_instr);
			if (retval != ERROR_OK)
				return retval;

			if (current_instr == MIPS16_SDBBP) {
				retval = target_write_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;
			}
		}
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

static int mips_mips64_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct mips64_common *mips64 = target->arch_info;

	if (mips64->mips64mode32)
		breakpoint->address = mips64_extend_sign(breakpoint->address);

	if (breakpoint->type == BKPT_HARD) {
		if (mips64->num_inst_bpoints_avail < 1) {
			LOG_INFO("no hardware breakpoint available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		mips64->num_inst_bpoints_avail--;
	}

	mips_mips64_set_breakpoint(target, breakpoint);

	return ERROR_OK;
}

static int mips_mips64_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->set)
		mips_mips64_unset_breakpoint(target, breakpoint);

	if (breakpoint->type == BKPT_HARD)
		mips64->num_inst_bpoints_avail++;

	return ERROR_OK;
}

static int mips_mips64_set_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	uint64_t wp_value;
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *comparator_list = mips64->data_break_list;
	int wp_num = 0;
	/*
	 * watchpoint enabled, ignore all byte lanes in value register
	 * and exclude both load and store accesses from  watchpoint
	 * condition evaluation
	*/
	int enable = EJTAG_DBCn_NOSB | EJTAG_DBCn_NOLB | EJTAG_DBCn_BE | (0xff << EJTAG_DBCn_BLM_SHIFT);

	if (watchpoint->set) {
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}

	while (comparator_list[wp_num].used && (wp_num < mips64->num_data_bpoints))
		wp_num++;
	if (wp_num >= mips64->num_data_bpoints) {
		LOG_DEBUG("ERROR Can not find free FP Comparator");
		LOG_WARNING("ERROR Can not find free FP Comparator");
		exit(-1);
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

	watchpoint->set = wp_num + 1;
	comparator_list[wp_num].used = 1;
	comparator_list[wp_num].bp_value = watchpoint->address;

	wp_value = watchpoint->address;
	if (wp_value & 0x80000000)
		wp_value |= ULLONG_MAX << 32;

	target_write_u64(target, comparator_list[wp_num].reg_address, wp_value);
	target_write_u64(target, comparator_list[wp_num].reg_address + 0x08, 0x00000000);
	target_write_u64(target, comparator_list[wp_num].reg_address + 0x10, 0x00000000);
	target_write_u64(target, comparator_list[wp_num].reg_address + 0x18, enable);
	target_write_u64(target, comparator_list[wp_num].reg_address + 0x20, 0);
	LOG_DEBUG("wp_num %i bp_value 0x%" PRIx64 "", wp_num, comparator_list[wp_num].bp_value);

	return ERROR_OK;
}

static int mips_mips64_unset_watchpoint(struct target *target, struct watchpoint *watchpoint)
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
	comparator_list[wp_num].used = 0;
	comparator_list[wp_num].bp_value = 0;
	target_write_u64(target, comparator_list[wp_num].reg_address + 0x18, 0);
	watchpoint->set = 0;

	return ERROR_OK;
}

static int mips_mips64_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct mips64_common *mips64 = target->arch_info;

	if (mips64->num_data_bpoints_avail < 1) {
		LOG_INFO("no hardware watchpoints available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	mips64->num_data_bpoints_avail--;

	mips_mips64_set_watchpoint(target, watchpoint);
	return ERROR_OK;
}

static int mips_mips64_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
		mips_mips64_unset_watchpoint(target, watchpoint);

	mips64->num_data_bpoints_avail++;

	return ERROR_OK;
}

static void mips_mips64_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	/* set any pending watchpoints */
	while (watchpoint) {
		if (watchpoint->set == 0)
			mips_mips64_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

static int mips_mips64_read_memory(struct target *target, uint64_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "", address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted %d", target->state);
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (((size == 8) && (address & 0x7u)) || ((size == 4) && (address & 0x3u)) ||
	    ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* since we don't know if buffer is aligned, we allocate new mem that is always aligned */
	void *t = NULL;

	if (size > 1) {
		t = malloc(count * size * sizeof(uint8_t));
		if (t == NULL) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
	} else
		t = buffer;

	/* if noDMA off, use DMAACC mode for memory read */
	int retval;
	if (ejtag_info->impcode & EJTAG_IMP_NODMA)
		retval = mips64_pracc_read_mem(ejtag_info, (uint64_t) address, size, count, (void *)t);
	else
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		/* retval = mips64_dmaacc_read_mem(ejtag_info, (uint64_t) address, size, count, (void *)buffer); */

	/* mips32_..._read_mem with size 4/2 returns uint32_t/uint16_t in host */
	/* endianness, but byte array should represent target endianness       */
	if (ERROR_OK == retval) {
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
	}

	if ((size > 1) && (t != NULL))
		free(t);

	return retval;
}

static int mips_mips64_bulk_write_memory(struct target *target, target_addr_t address,
		uint32_t count, const uint8_t *buffer)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	struct working_area *fast_data_area;
	int retval;
	int write_t = 1;

	LOG_DEBUG("address: " TARGET_ADDR_FMT ", count: 0x%8.8" PRIx32 "", address, count);

	/* check alignment */
	if (address & 0x7u)
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (mips64->fast_data_area == NULL) {
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
	uint64_t *t = NULL;
	t = malloc(count * sizeof(uint64_t));
	if (t == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	target_buffer_get_u64_array(target, buffer, count, t);

	retval = mips64_pracc_fastdata_xfer(ejtag_info, mips64->fast_data_area,
			write_t, address,
			count, t);

	if (t != NULL)
		free(t);

	if (retval != ERROR_OK)
		LOG_ERROR("Fastdata access Failed");

	return retval;
}

static int mips_mips64_write_memory(struct target *target, uint64_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	int retval;

	if (mips64->mips64mode32)
		address = mips64_extend_sign(address);

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "", address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}


	/* sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (((size == 8) && (address & 0x7u)) || ((size == 4) && (address & 0x3u)) ||
	    ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (size == 8 && count > 8) {
		retval = mips_mips64_bulk_write_memory(target,
			address,
			count,
			buffer);
		if (retval == ERROR_OK)
			return ERROR_OK;
		LOG_WARNING("Falling back to non-bulk write");
	}

	void *t = NULL;
	if (size > 1) {
		/* mips32_..._write_mem with size 4/2 requires uint32_t/uint16_t in host */
		/* endianness, but byte array represents target endianness               */
		t = malloc(count * size * sizeof(uint8_t));
		if (t == NULL) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}

		switch (size) {
		case 8:
			target_buffer_get_u64_array(target, buffer, count, (uint64_t *)t);
			break;
		case 4:
			target_buffer_get_u32_array(target, buffer, count, (uint32_t *)t);
			break;
		case 2:
			target_buffer_get_u16_array(target, buffer, count, (uint16_t *)t);
			break;
		}
		buffer = t;
	}

	/* if noDMA off, use DMAACC mode for memory write */
	if (ejtag_info->impcode & EJTAG_IMP_NODMA)
		retval = mips64_pracc_write_mem(ejtag_info, (uint64_t) address, size, count, (void *)buffer);
	else
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		/* return mips64_dmaacc_write_mem(ejtag_info, (uint64_t) address, size, count, (void *)buffer); */

	if (t != NULL)
		free(t);

	if (ERROR_OK != retval)
		return retval;

	return ERROR_OK;
}

static int mips_mips64_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	mips64_build_reg_cache(target);

	return ERROR_OK;
}

static int mips_mips64_init_arch_info(struct target *target, struct mips_mips64_common *mips_mips64, struct jtag_tap *tap)
{
	struct mips64_common *mips64 = &mips_mips64->mips64_common;

	mips_mips64->common_magic = MIPS64_COMMON_MAGIC;

	/* initialize mips4k specific info */
	mips64_init_arch_info(target, mips64, tap);
	mips64->arch_info = mips_mips64;

	return ERROR_OK;
}

static int mips_mips64_target_create(struct target *target, Jim_Interp *interp)
{
	struct mips_mips64_common *mips_mips64 = calloc(1, sizeof(struct mips_mips64_common));

	mips_mips64_init_arch_info(target, mips_mips64, target->tap);

	return ERROR_OK;
}

static int mips_mips64_examine(struct target *target)
{
	int retval;
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;
	uint32_t idcode = 0;

	if (!target_was_examined(target)) {
		mips_ejtag_get_idcode(ejtag_info, &idcode);
		ejtag_info->idcode = idcode;

		if (((idcode >> 1) & 0x7FF) == 0x29) {
			/* we are using a pic32mx so select ejtag port
			 * as it is not selected by default */
			mips_ejtag_set_instr(ejtag_info, 0x05);
			LOG_DEBUG("PIC32MX Detected - using EJTAG Interface");
		}
	}

	/* init rest of ejtag interface */
	retval = mips_ejtag_init(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	return mips64_examine(target);
}

static int mips_mips64_checksum_memory(struct target *target, uint64_t address, uint32_t size, uint32_t *checksum)
{
	return ERROR_FAIL; /* use bulk read method */
}

COMMAND_HANDLER(handle_mips64mode32)
{
	struct target *target = get_current_target(CMD_CTX);
	struct mips64_common *mips64 = target->arch_info;

	if (CMD_ARGC > 0)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], mips64->mips64mode32);

	if (mips64->mips64mode32)
		command_print(CMD_CTX, "enabled");
	else
		command_print(CMD_CTX, "disabled");

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

#endif /* BUILD_TARGET64 */
