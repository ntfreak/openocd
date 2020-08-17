/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2009 by David N. Claffey <dnclaffey@gmail.com>          *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
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

#include "breakpoints.h"
#include "mips32.h"
#include "mips_m4k.h"
#include "mips32_dmaacc.h"
#include "target_type.h"
#include "register.h"
#include "smp.h"

static void mips_m4k_enable_breakpoints(struct target *target);
static void mips_m4k_enable_watchpoints(struct target *target);
static int mips_m4k_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
static int mips_m4k_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
static int mips_m4k_internal_restore(struct target *target, int current,
		target_addr_t address, int handle_breakpoints,
		int debug_execution);
static int mips_m4k_halt(struct target *target);
static int mips_m4k_bulk_write_memory(struct target *target, target_addr_t address,
		uint32_t count, const uint8_t *buffer);

static int mips_m4k_examine_debug_reason(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	uint32_t break_status;
	int retval;

	if ((target->debug_reason != DBG_REASON_DBGRQ)
			&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {
		if (ejtag_info->debug_caps & EJTAG_DCR_IB) {
			/* get info about inst breakpoint support */
			retval = target_read_u32(target,
				ejtag_info->ejtag_ibs_addr, &break_status);
			if (retval != ERROR_OK)
				return retval;
			if (break_status & 0x1f) {
				/* we have halted on a  breakpoint */
				retval = target_write_u32(target,
					ejtag_info->ejtag_ibs_addr, 0);
				if (retval != ERROR_OK)
					return retval;
				target->debug_reason = DBG_REASON_BREAKPOINT;
			}
		}

		if (ejtag_info->debug_caps & EJTAG_DCR_DB) {
			/* get info about data breakpoint support */
			retval = target_read_u32(target,
				ejtag_info->ejtag_dbs_addr, &break_status);
			if (retval != ERROR_OK)
				return retval;
			if (break_status & 0x1f) {
				/* we have halted on a  breakpoint */
				retval = target_write_u32(target,
					ejtag_info->ejtag_dbs_addr, 0);
				if (retval != ERROR_OK)
					return retval;
				target->debug_reason = DBG_REASON_WATCHPOINT;
			}
		}
	}

	return ERROR_OK;
}

static int mips_m4k_debug_entry(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	mips32_save_context(target);

	/* make sure stepping disabled, SSt bit in CP0 debug register cleared */
	mips_ejtag_config_step(ejtag_info, 0);

	/* make sure break unit configured */
	mips32_configure_break_unit(target);

	/* attempt to find halt reason */
	mips_m4k_examine_debug_reason(target);

	mips32_read_config_regs(target);

	/* default to mips32 isa, it will be changed below if required */
	mips32->isa_mode = MIPS32_ISA_MIPS32;

	/* other than mips32 only and isa bit set ? */
	if (mips32->isa_imp && buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 1))
		mips32->isa_mode = mips32->isa_imp == 2 ? MIPS32_ISA_MIPS16E : MIPS32_ISA_MMIPS32;

	LOG_DEBUG("entered debug state at PC 0x%" PRIx32 ", target->state: %s",
			buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32),
			target_state_name(target));

	return ERROR_OK;
}

static struct target *get_mips_m4k(struct target *target, int32_t coreid)
{
	struct target_list *head;
	struct target *curr;

	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr->coreid == coreid) && (curr->state == TARGET_HALTED))
			return curr;
		head = head->next;
	}
	return target;
}

static int mips_m4k_halt_smp(struct target *target)
{
	int retval = ERROR_OK;
	struct target_list *head;
	struct target *curr;
	head = target->head;
	while (head != (struct target_list *)NULL) {
		int ret = ERROR_OK;
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_HALTED))
			ret = mips_m4k_halt(curr);

		if (ret != ERROR_OK) {
			LOG_ERROR("halt failed target->coreid: %" PRId32, curr->coreid);
			retval = ret;
		}
		head = head->next;
	}
	return retval;
}

static int update_halt_gdb(struct target *target)
{
	int retval = ERROR_OK;
	if (target->gdb_service->core[0] == -1) {
		target->gdb_service->target = target;
		target->gdb_service->core[0] = target->coreid;
		retval = mips_m4k_halt_smp(target);
	}
	return retval;
}

static int mips_m4k_poll(struct target *target)
{
	int retval = ERROR_OK;
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl;
	enum target_state prev_target_state = target->state;

	/*  toggle to another core is done by gdb as follow */
	/*  maint packet J core_id */
	/*  continue */
	/*  the next polling trigger an halt event sent to gdb */
	if ((target->state == TARGET_HALTED) && (target->smp) &&
		(target->gdb_service) &&
		(target->gdb_service->target == NULL)) {
		target->gdb_service->target =
			get_mips_m4k(target, target->gdb_service->core[1]);
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return retval;
	}

	/* read ejtag control reg */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		return retval;

	ejtag_info->isa = (ejtag_ctrl & EJTAG_CTRL_DBGISA) ? 1 : 0;

	/* clear this bit before handling polling
	 * as after reset registers will read zero */
	if (ejtag_ctrl & EJTAG_CTRL_ROCC) {
		/* we have detected a reset, clear flag
		 * otherwise ejtag will not work */
		ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_ROCC;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("Reset Detected");
	}

	/* check for processor halted */
	if (ejtag_ctrl & EJTAG_CTRL_BRKST) {
		if ((target->state != TARGET_HALTED)
		    && (target->state != TARGET_DEBUG_RUNNING)) {
			if (target->state == TARGET_UNKNOWN)
				LOG_DEBUG("EJTAG_CTRL_BRKST already set during server startup.");

			/* OpenOCD was was probably started on the board with EJTAG_CTRL_BRKST already set
			 * (maybe put on by HALT-ing the board in the previous session).
			 *
			 * Force enable debug entry for this session.
			 */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_NORMALBOOT);
			target->state = TARGET_HALTED;
			retval = mips_m4k_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			if (target->smp &&
				((prev_target_state == TARGET_RUNNING)
			     || (prev_target_state == TARGET_RESET))) {
				retval = update_halt_gdb(target);
				if (retval != ERROR_OK)
					return retval;
			}
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		} else if (target->state == TARGET_DEBUG_RUNNING) {
			target->state = TARGET_HALTED;

			retval = mips_m4k_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			if (target->smp) {
				retval = update_halt_gdb(target);
				if (retval != ERROR_OK)
					return retval;
			}

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	} else
		target->state = TARGET_RUNNING;

/*	LOG_DEBUG("ctrl = 0x%08X", ejtag_ctrl); */

	return ERROR_OK;
}

static int mips_m4k_halt(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	LOG_DEBUG("target->state: %s", target_state_name(target));

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
			 * debug entry was already prepared in mips_m4k_assert_reset()
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

static int mips_m4k_assert_reset(struct target *target)
{
	struct mips_m4k_common *mips_m4k = target_to_m4k(target);
	struct mips_ejtag *ejtag_info = &mips_m4k->mips32.ejtag_info;

	/* TODO: apply hw reset signal in not examined state */
	if (!(target_was_examined(target))) {
		LOG_WARNING("Reset is not asserted because the target is not examined.");
		LOG_WARNING("Use a reset button or power cycle the target.");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	/* some cores support connecting while srst is asserted
	 * use that mode is it has been configured */

	bool srst_asserted = false;

	if (!(jtag_reset_config & RESET_SRST_PULLS_TRST) &&
			(jtag_reset_config & RESET_SRST_NO_GATING)) {
		jtag_add_reset(0, 1);
		srst_asserted = true;
	}


	/* EJTAG before v2.5/2.6 does not support EJTAGBOOT or NORMALBOOT */
	if (ejtag_info->ejtag_version != EJTAG_VERSION_20) {
		if (target->reset_halt) {
			/* use hardware to catch reset */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_EJTAGBOOT);
		} else
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_NORMALBOOT);
	}

	if (jtag_reset_config & RESET_HAS_SRST) {
		/* here we should issue a srst only, but we may have to assert trst as well */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
			jtag_add_reset(1, 1);
		else if (!srst_asserted)
			jtag_add_reset(0, 1);
	} else if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	} else {
		if (mips_m4k->is_pic32mx) {
			LOG_DEBUG("Using MTAP reset to reset processor...");

			/* use microchip specific MTAP reset */
			mips_ejtag_set_instr(ejtag_info, MTAP_SW_MTAP);
			mips_ejtag_set_instr(ejtag_info, MTAP_COMMAND);

			mips_ejtag_drscan_8_out(ejtag_info, MCHP_ASERT_RST);
			mips_ejtag_drscan_8_out(ejtag_info, MCHP_DE_ASSERT_RST);
			mips_ejtag_set_instr(ejtag_info, MTAP_SW_ETAP);
		} else {
			/* use ejtag reset - not supported by all cores */
			uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl | EJTAG_CTRL_PRRST | EJTAG_CTRL_PERRST;
			LOG_DEBUG("Using EJTAG reset (PRRST) to reset processor...");
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
			mips_ejtag_drscan_32_out(ejtag_info, ejtag_ctrl);
		}
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(50000);

	register_cache_invalidate(mips_m4k->mips32.core_cache);

	if (target->reset_halt) {
		int retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int mips_m4k_deassert_reset(struct target *target)
{
	LOG_DEBUG("target->state: %s", target_state_name(target));

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	return ERROR_OK;
}

static int mips_m4k_single_step_core(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	/* configure single step mode */
	mips_ejtag_config_step(ejtag_info, 1);

	/* disable interrupts while stepping */
	mips32_enable_interrupts(target, 0);

	/* exit debug mode */
	mips_ejtag_exit_debug(ejtag_info);

	mips_m4k_debug_entry(target);

	return ERROR_OK;
}

static int mips_m4k_restore_smp(struct target *target, uint32_t address, int handle_breakpoints)
{
	int retval = ERROR_OK;
	struct target_list *head;
	struct target *curr;

	head = target->head;
	while (head != (struct target_list *)NULL) {
		int ret = ERROR_OK;
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_RUNNING)) {
			/*  resume current address , not in step mode */
			ret = mips_m4k_internal_restore(curr, 1, address,
						   handle_breakpoints, 0);

			if (ret != ERROR_OK) {
				LOG_ERROR("target->coreid :%" PRId32 " failed to resume at address :0x%" PRIx32,
						  curr->coreid, address);
				retval = ret;
			}
		}
		head = head->next;
	}
	return retval;
}

static int mips_m4k_internal_restore(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);
		mips_m4k_enable_breakpoints(target);
		mips_m4k_enable_watchpoints(target);
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		mips_m4k_isa_filter(mips32->isa_imp, &address);
		buf_set_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32, address);
		mips32->core_cache->reg_list[MIPS32_PC].dirty = true;
		mips32->core_cache->reg_list[MIPS32_PC].valid = true;
	}

	if ((mips32->isa_imp > 1) &&  debug_execution)	/* if more than one isa supported */
		buf_set_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 1, mips32->isa_mode);

	if (!current)
		resume_pc = address;
	else
		resume_pc = buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32);

	mips32_restore_context(target);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at " TARGET_ADDR_FMT "",
					  breakpoint->address);
			mips_m4k_unset_breakpoint(target, breakpoint);
			mips_m4k_single_step_core(target);
			mips_m4k_set_breakpoint(target, breakpoint);
		}
	}

	/* enable interrupts if we are running */
	mips32_enable_interrupts(target, !debug_execution);

	/* exit debug mode */
	mips_ejtag_exit_debug(ejtag_info);
	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	register_cache_invalidate(mips32->core_cache);

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

static int mips_m4k_resume(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	int retval = ERROR_OK;

	/* dummy resume for smp toggle in order to reduce gdb impact  */
	if ((target->smp) && (target->gdb_service->core[1] != -1)) {
		/*   simulate a start and halt of target */
		target->gdb_service->target = NULL;
		target->gdb_service->core[0] = target->gdb_service->core[1];
		/*  fake resume at next poll we play the  target core[1], see poll*/
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		return retval;
	}

	retval = mips_m4k_internal_restore(target, current, address,
				handle_breakpoints,
				debug_execution);

	if (retval == ERROR_OK && target->smp) {
		target->gdb_service->core[0] = -1;
		retval = mips_m4k_restore_smp(target, address, handle_breakpoints);
	}

	return retval;
}

static int mips_m4k_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct breakpoint *breakpoint = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		mips_m4k_isa_filter(mips32->isa_imp, &address);
		buf_set_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32, address);
		mips32->core_cache->reg_list[MIPS32_PC].dirty = true;
		mips32->core_cache->reg_list[MIPS32_PC].valid = true;
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target,
				buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32));
		if (breakpoint)
			mips_m4k_unset_breakpoint(target, breakpoint);
	}

	/* restore context */
	mips32_restore_context(target);

	/* configure single step mode */
	mips_ejtag_config_step(ejtag_info, 1);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* disable interrupts while stepping */
	mips32_enable_interrupts(target, 0);

	/* exit debug mode */
	mips_ejtag_exit_debug(ejtag_info);

	/* registers are now invalid */
	register_cache_invalidate(mips32->core_cache);

	LOG_DEBUG("target stepped ");
	mips_m4k_debug_entry(target);

	if (breakpoint)
		mips_m4k_set_breakpoint(target, breakpoint);

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

static void mips_m4k_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint) {
		if (breakpoint->set == 0)
			mips_m4k_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

static int mips_m4k_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct mips32_comparator *comparator_list = mips32->inst_break_list;
	int retval;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = 0;

		while (comparator_list[bp_num].used && (bp_num < mips32->num_inst_bpoints))
			bp_num++;
		if (bp_num >= mips32->num_inst_bpoints) {
			LOG_ERROR("Can not find free FP Comparator(bpid: %" PRIu32 ")",
					breakpoint->unique_id);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint->set = bp_num + 1;
		comparator_list[bp_num].used = 1;
		comparator_list[bp_num].bp_value = breakpoint->address;

		if (breakpoint->length != 4)			/* make sure isa bit set */
			comparator_list[bp_num].bp_value |= 1;
		else						/* make sure isa bit cleared */
			comparator_list[bp_num].bp_value &= ~1;

		/* EJTAG 2.0 uses 30bit IBA. First 2 bits are reserved.
		 * Warning: there is no IB ASID registers in 2.0.
		 * Do not set it! :) */
		if (ejtag_info->ejtag_version == EJTAG_VERSION_20)
			comparator_list[bp_num].bp_value &= 0xFFFFFFFC;

		target_write_u32(target, comparator_list[bp_num].reg_address,
				comparator_list[bp_num].bp_value);
		target_write_u32(target, comparator_list[bp_num].reg_address +
				 ejtag_info->ejtag_ibm_offs, 0x00000000);
		target_write_u32(target, comparator_list[bp_num].reg_address +
				 ejtag_info->ejtag_ibc_offs, 1);
		LOG_DEBUG("bpid: %" PRIu32 ", bp_num %i bp_value 0x%" PRIx32 "",
				  breakpoint->unique_id,
				  bp_num, comparator_list[bp_num].bp_value);
	} else if (breakpoint->type == BKPT_SOFT) {
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);

		uint32_t isa_req = breakpoint->length & 1;	/* micro mips request bit */
		uint32_t bplength = breakpoint->length & ~1;	/* drop micro mips request bit for length */
		uint32_t bpaddr = breakpoint->address & ~1;	/* drop isa bit from address, if set */

		if (bplength == 4) {
			uint32_t verify = 0xffffffff;
			uint32_t sdbbp32_instr = MIPS32_SDBBP(isa_req);
			if (ejtag_info->endianness && isa_req)
				sdbbp32_instr = SWAP16(sdbbp32_instr);

			if ((breakpoint->address & 3) == 0) {	/* word aligned */

				retval = target_read_memory(target, bpaddr, bplength, 1, breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;

				retval = target_write_u32(target, bpaddr, sdbbp32_instr);
				if (retval != ERROR_OK)
					return retval;

				retval = target_read_u32(target, bpaddr, &verify);
				if (retval != ERROR_OK)
					return retval;

				if (verify != sdbbp32_instr)
					verify = 0;

			} else {	/* 16 bit aligned */
				retval = target_read_memory(target, bpaddr, 2, 2, breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;

				uint8_t sdbbp_buf[4];
				target_buffer_set_u32(target, sdbbp_buf, sdbbp32_instr);

				retval = target_write_memory(target, bpaddr, 2, 2, sdbbp_buf);
				if (retval != ERROR_OK)
					return retval;

				retval = target_read_memory(target, bpaddr, 2, 2, sdbbp_buf);
				if (retval != ERROR_OK)
					return retval;

				if (target_buffer_get_u32(target, sdbbp_buf) != sdbbp32_instr)
					verify = 0;
			}

			if (verify == 0) {
				LOG_ERROR("Unable to set 32bit breakpoint at address %08" TARGET_PRIxADDR
					" - check that memory is read/writable", breakpoint->address);
				return ERROR_OK;
			}

		} else {
			uint16_t verify = 0xffff;

			retval = target_read_memory(target, bpaddr, bplength, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;

			retval = target_write_u16(target, bpaddr, MIPS16_SDBBP(isa_req));
			if (retval != ERROR_OK)
				return retval;

			retval = target_read_u16(target, bpaddr, &verify);
			if (retval != ERROR_OK)
				return retval;

			if (verify != MIPS16_SDBBP(isa_req)) {
				LOG_ERROR("Unable to set 16bit breakpoint at address %08" TARGET_PRIxADDR
						" - check that memory is read/writable", breakpoint->address);
				return ERROR_OK;
			}
		}

		breakpoint->set = 20; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int mips_m4k_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct mips32_comparator *comparator_list = mips32->inst_break_list;
	int retval;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = breakpoint->set - 1;
		if ((bp_num < 0) || (bp_num >= mips32->num_inst_bpoints)) {
			LOG_DEBUG("Invalid FP Comparator number in breakpoint (bpid: %" PRIu32 ")",
					  breakpoint->unique_id);
			return ERROR_OK;
		}
		LOG_DEBUG("bpid: %" PRIu32 " - releasing hw: %d",
				breakpoint->unique_id,
				bp_num);
		comparator_list[bp_num].used = 0;
		comparator_list[bp_num].bp_value = 0;
		target_write_u32(target, comparator_list[bp_num].reg_address +
				 ejtag_info->ejtag_ibc_offs, 0);

	} else {
		/* restore original instruction (kept in target endianness) */
		uint32_t isa_req = breakpoint->length & 1;
		uint32_t bplength = breakpoint->length & ~1;
		uint8_t current_instr[4];
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);
		if (bplength == 4) {
			uint32_t sdbbp32_instr =  MIPS32_SDBBP(isa_req);
			if (ejtag_info->endianness && isa_req)
				sdbbp32_instr = SWAP16(sdbbp32_instr);

			if ((breakpoint->address & 3) == 0) {		/* 32bit aligned */
				/* check that user program has not modified breakpoint instruction */
				retval = target_read_memory(target, breakpoint->address, 4, 1, current_instr);
				if (retval != ERROR_OK)
					return retval;
				/**
				* target_read_memory() gets us data in _target_ endianness.
				* If we want to use this data on the host for comparisons with some macros
				* we must first transform it to _host_ endianness using target_buffer_get_u16().
				*/
				if (sdbbp32_instr == target_buffer_get_u32(target, current_instr)) {
					retval = target_write_memory(target, breakpoint->address, 4, 1,
										breakpoint->orig_instr);
					if (retval != ERROR_OK)
						return retval;
				}
			} else {	/* 16bit aligned */
				retval = target_read_memory(target, breakpoint->address, 2, 2, current_instr);
				if (retval != ERROR_OK)
					return retval;

				if (sdbbp32_instr == target_buffer_get_u32(target, current_instr)) {
					retval = target_write_memory(target, breakpoint->address, 2, 2,
										breakpoint->orig_instr);
					if (retval != ERROR_OK)
						return retval;
				}
			}
		} else {
			/* check that user program has not modified breakpoint instruction */
			retval = target_read_memory(target, breakpoint->address, 2, 1, current_instr);
			if (retval != ERROR_OK)
				return retval;

			if (target_buffer_get_u16(target, current_instr) == MIPS16_SDBBP(isa_req)) {
				retval = target_write_memory(target, breakpoint->address, 2, 1,
									breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;
			}
		}
	}

	breakpoint->set = 0;

	return ERROR_OK;
}

static int mips_m4k_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct mips32_common *mips32 = target_to_mips32(target);

	if ((breakpoint->length > 5 || breakpoint->length < 2) ||		/* out of range */
		(breakpoint->length == 4 && (breakpoint->address & 2)) ||	/* mips32 unaligned */
		(mips32->isa_imp == MIPS32_ONLY && breakpoint->length != 4) ||	/* misp32 specific */
		((mips32->isa_imp & 1) != (breakpoint->length & 1)))		/* isa not implemented */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (breakpoint->type == BKPT_HARD) {
		if (mips32->num_inst_bpoints_avail < 1) {
			LOG_INFO("no hardware breakpoint available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		mips32->num_inst_bpoints_avail--;
	}

	return mips_m4k_set_breakpoint(target, breakpoint);
}

static int mips_m4k_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->set)
		mips_m4k_unset_breakpoint(target, breakpoint);

	if (breakpoint->type == BKPT_HARD)
		mips32->num_inst_bpoints_avail++;

	return ERROR_OK;
}

static int mips_m4k_set_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct mips32_comparator *comparator_list = mips32->data_break_list;
	int wp_num = 0;
	/*
	 * watchpoint enabled, ignore all byte lanes in value register
	 * and exclude both load and store accesses from  watchpoint
	 * condition evaluation
	*/
	int enable = EJTAG_DBCn_NOSB | EJTAG_DBCn_NOLB | EJTAG_DBCn_BE |
			(0xff << EJTAG_DBCn_BLM_SHIFT);

	if (watchpoint->set) {
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}

	while (comparator_list[wp_num].used && (wp_num < mips32->num_data_bpoints))
		wp_num++;
	if (wp_num >= mips32->num_data_bpoints) {
		LOG_ERROR("Can not find free FP Comparator");
		return ERROR_FAIL;
	}

	if (watchpoint->length != 4) {
		LOG_ERROR("Only watchpoints of length 4 are supported");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	if (watchpoint->address % 4) {
		LOG_ERROR("Watchpoints address should be word aligned");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	switch (watchpoint->rw) {
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

	/* EJTAG 2.0 uses 29bit DBA. First 3 bits are reserved.
	 * There is as well no ASID register support. */
	if (ejtag_info->ejtag_version == EJTAG_VERSION_20)
		comparator_list[wp_num].bp_value &= 0xFFFFFFF8;
	else
		target_write_u32(target, comparator_list[wp_num].reg_address +
			 ejtag_info->ejtag_dbasid_offs, 0x00000000);

	target_write_u32(target, comparator_list[wp_num].reg_address,
			 comparator_list[wp_num].bp_value);
	target_write_u32(target, comparator_list[wp_num].reg_address +
			 ejtag_info->ejtag_dbm_offs, 0x00000000);

	target_write_u32(target, comparator_list[wp_num].reg_address +
			 ejtag_info->ejtag_dbc_offs, enable);
	/* TODO: probably this value is ignored on 2.0 */
	target_write_u32(target, comparator_list[wp_num].reg_address +
			 ejtag_info->ejtag_dbv_offs, 0);
	LOG_DEBUG("wp_num %i bp_value 0x%" PRIx32 "", wp_num, comparator_list[wp_num].bp_value);

	return ERROR_OK;
}

static int mips_m4k_unset_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct mips32_comparator *comparator_list = mips32->data_break_list;

	if (!watchpoint->set) {
		LOG_WARNING("watchpoint not set");
		return ERROR_OK;
	}

	int wp_num = watchpoint->set - 1;
	if ((wp_num < 0) || (wp_num >= mips32->num_data_bpoints)) {
		LOG_DEBUG("Invalid FP Comparator number in watchpoint");
		return ERROR_OK;
	}
	comparator_list[wp_num].used = 0;
	comparator_list[wp_num].bp_value = 0;
	target_write_u32(target, comparator_list[wp_num].reg_address +
			 ejtag_info->ejtag_dbc_offs, 0);
	watchpoint->set = 0;

	return ERROR_OK;
}

static int mips_m4k_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct mips32_common *mips32 = target_to_mips32(target);

	if (mips32->num_data_bpoints_avail < 1) {
		LOG_INFO("no hardware watchpoints available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	mips32->num_data_bpoints_avail--;

	mips_m4k_set_watchpoint(target, watchpoint);
	return ERROR_OK;
}

static int mips_m4k_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
		mips_m4k_unset_watchpoint(target, watchpoint);

	mips32->num_data_bpoints_avail++;

	return ERROR_OK;
}

static void mips_m4k_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	/* set any pending watchpoints */
	while (watchpoint) {
		if (watchpoint->set == 0)
			mips_m4k_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

static int mips_m4k_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	LOG_DEBUG("address: " TARGET_ADDR_FMT ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
			address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
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
		retval = mips32_pracc_read_mem(ejtag_info, address, size, count, t);
	else
		retval = mips32_dmaacc_read_mem(ejtag_info, address, size, count, t);

	/* mips32_..._read_mem with size 4/2 returns uint32_t/uint16_t in host */
	/* endianness, but byte array should represent target endianness       */
	if (ERROR_OK == retval) {
		switch (size) {
		case 4:
			target_buffer_set_u32_array(target, buffer, count, t);
			break;
		case 2:
			target_buffer_set_u16_array(target, buffer, count, t);
			break;
		}
	}

	if (size > 1)
		free(t);

	return retval;
}

static int mips_m4k_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	LOG_DEBUG("address: " TARGET_ADDR_FMT ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
			address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (size == 4 && count > 32) {
		int retval = mips_m4k_bulk_write_memory(target, address, count, buffer);
		if (retval == ERROR_OK)
			return ERROR_OK;
		LOG_WARNING("Falling back to non-bulk write");
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/** correct endianness if we have word or hword access */
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
	int retval;
	if (ejtag_info->impcode & EJTAG_IMP_NODMA)
		retval = mips32_pracc_write_mem(ejtag_info, address, size, count, buffer);
	else
		retval = mips32_dmaacc_write_mem(ejtag_info, address, size, count, buffer);

	free(t);

	if (ERROR_OK != retval)
		return retval;

	return ERROR_OK;
}

static int mips_m4k_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	mips32_build_reg_cache(target);

	return ERROR_OK;
}

static int mips_m4k_init_arch_info(struct target *target,
		struct mips_m4k_common *mips_m4k, struct jtag_tap *tap)
{
	struct mips32_common *mips32 = &mips_m4k->mips32;

	mips_m4k->common_magic = MIPSM4K_COMMON_MAGIC;

	/* initialize mips4k specific info */
	mips32_init_arch_info(target, mips32, tap);
	mips32->arch_info = mips_m4k;

	return ERROR_OK;
}

static int mips_m4k_target_create(struct target *target, Jim_Interp *interp)
{
	struct mips_m4k_common *mips_m4k = calloc(1, sizeof(struct mips_m4k_common));

	mips_m4k_init_arch_info(target, mips_m4k, target->tap);

	return ERROR_OK;
}

static int mips_m4k_examine(struct target *target)
{
	struct mips_m4k_common *mips_m4k = target_to_m4k(target);
	struct mips_ejtag *ejtag_info = &mips_m4k->mips32.ejtag_info;

	if (!target_was_examined(target)) {
		int retval = mips_ejtag_get_idcode(ejtag_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("idcode read failed");
			return retval;
		}
		if (((ejtag_info->idcode >> 1) & 0x7FF) == 0x29) {
			/* we are using a pic32mx so select ejtag port
			 * as it is not selected by default */
			mips_ejtag_set_instr(ejtag_info, MTAP_SW_ETAP);
			LOG_DEBUG("PIC32 Detected - using EJTAG Interface");
			mips_m4k->is_pic32mx = true;
		}
	}

	/* init rest of ejtag interface */
	int retval = mips_ejtag_init(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	return mips32_examine(target);
}

static int mips_m4k_bulk_write_memory(struct target *target, target_addr_t address,
		uint32_t count, const uint8_t *buffer)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct working_area *fast_data_area;
	int retval;
	int write_t = 1;

	LOG_DEBUG("address: " TARGET_ADDR_FMT ", count: 0x%8.8" PRIx32 "",
			  address, count);

	/* check alignment */
	if (address & 0x3u)
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (mips32->fast_data_area == NULL) {
		/* Get memory for block write handler
		 * we preserve this area between calls and gain a speed increase
		 * of about 3kb/sec when writing flash
		 * this will be released/nulled by the system when the target is resumed or reset */
		retval = target_alloc_working_area(target,
				MIPS32_FASTDATA_HANDLER_SIZE,
				&mips32->fast_data_area);
		if (retval != ERROR_OK) {
			LOG_ERROR("No working area available");
			return retval;
		}

		/* reset fastadata state so the algo get reloaded */
		ejtag_info->fast_access_save = -1;
	}

	fast_data_area = mips32->fast_data_area;

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
	uint32_t *t = NULL;
	t = malloc(count * sizeof(uint32_t));
	if (t == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	target_buffer_get_u32_array(target, buffer, count, t);

	retval = mips32_pracc_fastdata_xfer(ejtag_info, mips32->fast_data_area, write_t, address,
			count, t);

	free(t);

	if (retval != ERROR_OK)
		LOG_ERROR("Fastdata access Failed");

	return retval;
}

static int mips_m4k_verify_pointer(struct command_invocation *cmd,
		struct mips_m4k_common *mips_m4k)
{
	if (mips_m4k->common_magic != MIPSM4K_COMMON_MAGIC) {
		command_print(cmd, "target is not an MIPS_M4K");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(mips_m4k_handle_cp0_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct mips_m4k_common *mips_m4k = target_to_m4k(target);
	struct mips_ejtag *ejtag_info = &mips_m4k->mips32.ejtag_info;

	retval = mips_m4k_verify_pointer(CMD, mips_m4k);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* two or more argument, access a single register/select (write if third argument is given) */
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	else {
		uint32_t cp0_reg, cp0_sel;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], cp0_reg);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], cp0_sel);

		if (CMD_ARGC == 2) {
			uint32_t value;
			retval = mips32_cp0_read(ejtag_info, &value, cp0_reg, cp0_sel);
			if (retval != ERROR_OK) {
				command_print(CMD,
						"couldn't access reg %" PRIu32,
						cp0_reg);
				return ERROR_OK;
			}
			command_print(CMD, "cp0 reg %" PRIu32 ", select %" PRIu32 ": %8.8" PRIx32,
					cp0_reg, cp0_sel, value);

		} else if (CMD_ARGC == 3) {
			uint32_t value;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], value);
			retval = mips32_cp0_write(ejtag_info, value, cp0_reg, cp0_sel);
			if (retval != ERROR_OK) {
				command_print(CMD,
						"couldn't access cp0 reg %" PRIu32 ", select %" PRIu32,
						cp0_reg,  cp0_sel);
				return ERROR_OK;
			}
			command_print(CMD, "cp0 reg %" PRIu32 ", select %" PRIu32 ": %8.8" PRIx32,
					cp0_reg, cp0_sel, value);
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(mips_m4k_handle_scan_delay_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct mips_m4k_common *mips_m4k = target_to_m4k(target);
	struct mips_ejtag *ejtag_info = &mips_m4k->mips32.ejtag_info;

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], ejtag_info->scan_delay);
	else if (CMD_ARGC > 1)
			return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "scan delay: %d nsec", ejtag_info->scan_delay);
	if (ejtag_info->scan_delay >= MIPS32_SCAN_DELAY_LEGACY_MODE) {
		ejtag_info->mode = 0;
		command_print(CMD, "running in legacy mode");
	} else {
		ejtag_info->mode = 1;
		command_print(CMD, "running in fast queued mode");
	}

	return ERROR_OK;
}

static const struct command_registration mips_m4k_exec_command_handlers[] = {
	{
		.name = "cp0",
		.handler = mips_m4k_handle_cp0_command,
		.mode = COMMAND_EXEC,
		.usage = "regnum [value]",
		.help = "display/modify cp0 register",
	},
	{
		.name = "scan_delay",
		.handler = mips_m4k_handle_scan_delay_command,
		.mode = COMMAND_ANY,
		.help = "display/set scan delay in nano seconds",
		.usage = "[value]",
	},
	{
		.chain = smp_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration mips_m4k_command_handlers[] = {
	{
		.chain = mips32_command_handlers,
	},
	{
		.name = "mips_m4k",
		.mode = COMMAND_ANY,
		.help = "mips_m4k command group",
		.usage = "",
		.chain = mips_m4k_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type mips_m4k_target = {
	.name = "mips_m4k",

	.poll = mips_m4k_poll,
	.arch_state = mips32_arch_state,

	.halt = mips_m4k_halt,
	.resume = mips_m4k_resume,
	.step = mips_m4k_step,

	.assert_reset = mips_m4k_assert_reset,
	.deassert_reset = mips_m4k_deassert_reset,

	.get_gdb_reg_list = mips32_get_gdb_reg_list,

	.read_memory = mips_m4k_read_memory,
	.write_memory = mips_m4k_write_memory,
	.checksum_memory = mips32_checksum_memory,
	.blank_check_memory = mips32_blank_check_memory,

	.run_algorithm = mips32_run_algorithm,

	.add_breakpoint = mips_m4k_add_breakpoint,
	.remove_breakpoint = mips_m4k_remove_breakpoint,
	.add_watchpoint = mips_m4k_add_watchpoint,
	.remove_watchpoint = mips_m4k_remove_watchpoint,

	.commands = mips_m4k_command_handlers,
	.target_create = mips_m4k_target_create,
	.init_target = mips_m4k_init_target,
	.examine = mips_m4k_examine,
};
