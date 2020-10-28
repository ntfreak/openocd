/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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
#include "nds32_cmd.h"
#include "nds32_aice.h"
#include "nds32_v3m.h"
#include "nds32_v3_common.h"

static int nds32_v3m_activate_hardware_breakpoint(struct target *target)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct breakpoint *bp;
	unsigned brp_num = nds32_v3m->n_hbr - 1;

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT) {
			/* already set at nds32_v3m_add_breakpoint() */
			continue;
		} else if (bp->type == BKPT_HARD) {
			/* set address */
			aice_write_debug_reg(aice, NDS_EDM_SR_BPA0 + brp_num, bp->address);
			/* set mask */
			aice_write_debug_reg(aice, NDS_EDM_SR_BPAM0 + brp_num, 0);

			if (nds32_v3m->nds32.memory.address_translation)
				/* enable breakpoint (virtual address) */
				aice_write_debug_reg(aice, NDS_EDM_SR_BPC0 + brp_num, 0x2);
			else
				/* enable breakpoint (physical address) */
				aice_write_debug_reg(aice, NDS_EDM_SR_BPC0 + brp_num, 0xA);

			LOG_DEBUG("Add hardware BP %u at %08" TARGET_PRIxADDR, brp_num,
					bp->address);

			brp_num--;
		} else {
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int nds32_v3m_deactivate_hardware_breakpoint(struct target *target)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct breakpoint *bp;
	unsigned brp_num = nds32_v3m->n_hbr - 1;

	for (bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->type == BKPT_SOFT)
			continue;
		else if (bp->type == BKPT_HARD)
			/* disable breakpoint */
			aice_write_debug_reg(aice, NDS_EDM_SR_BPC0 + brp_num, 0x0);
		else
			return ERROR_FAIL;

		LOG_DEBUG("Remove hardware BP %u at %08" TARGET_PRIxADDR, brp_num,
				bp->address);

		brp_num--;
	}

	return ERROR_OK;
}

static int nds32_v3m_activate_hardware_watchpoint(struct target *target)
{
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);
	struct watchpoint *wp;
	int32_t  wp_num = 0;
	uint32_t wp_config = 0;
	bool ld_stop, st_stop;

	if (nds32_v3m->nds32.global_stop)
		ld_stop = st_stop = false;

	for (wp = target->watchpoints; wp; wp = wp->next) {

		if (wp_num < nds32_v3m->used_n_wp) {
			wp->mask = wp->length - 1;
			if ((wp->address % wp->length) != 0)
				wp->mask = (wp->mask << 1) + 1;

			if (wp->rw == WPT_READ)
				wp_config = 0x3;
			else if (wp->rw == WPT_WRITE)
				wp_config = 0x5;
			else if (wp->rw == WPT_ACCESS)
				wp_config = 0x7;

			/* set/unset physical address bit of BPCn according to PSW.DT */
			if (nds32_v3m->nds32.memory.address_translation == false)
				wp_config |= 0x8;

			/* set address */
			aice_write_debug_reg(aice, NDS_EDM_SR_BPA0 + wp_num,
					wp->address - (wp->address % wp->length));
			/* set mask */
			aice_write_debug_reg(aice, NDS_EDM_SR_BPAM0 + wp_num, wp->mask);
			/* enable watchpoint */
			aice_write_debug_reg(aice, NDS_EDM_SR_BPC0 + wp_num, wp_config);

			LOG_DEBUG("Add hardware watchpoint %" PRId32 " at %08" TARGET_PRIxADDR
					" mask %08" PRIx32, wp_num, wp->address, wp->mask);

			wp_num++;
		} else if (nds32_v3m->nds32.global_stop) {
			if (wp->rw == WPT_READ)
				ld_stop = true;
			else if (wp->rw == WPT_WRITE)
				st_stop = true;
			else if (wp->rw == WPT_ACCESS)
				ld_stop = st_stop = true;
		}
	}

	if (nds32_v3m->nds32.global_stop) {
		uint32_t edm_ctl;
		aice_read_debug_reg(aice, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		if (ld_stop)
			edm_ctl |= 0x10;
		if (st_stop)
			edm_ctl |= 0x20;
		aice_write_debug_reg(aice, NDS_EDM_SR_EDM_CTL, edm_ctl);
	}

	return ERROR_OK;
}

static int nds32_v3m_deactivate_hardware_watchpoint(struct target *target)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct watchpoint *wp;
	int32_t wp_num = 0;
	bool clean_global_stop = false;

	for (wp = target->watchpoints; wp; wp = wp->next) {

		if (wp_num < nds32_v3m->used_n_wp) {
			/* disable watchpoint */
			aice_write_debug_reg(aice, NDS_EDM_SR_BPC0 + wp_num, 0x0);

			LOG_DEBUG("Remove hardware watchpoint %" PRId32 " at %08" TARGET_PRIxADDR
					" mask %08" PRIx32, wp_num, wp->address, wp->mask);
			wp_num++;
		} else if (nds32_v3m->nds32.global_stop) {
			clean_global_stop = true;
		}
	}

	if (clean_global_stop) {
		uint32_t edm_ctl;
		aice_read_debug_reg(aice, NDS_EDM_SR_EDM_CTL, &edm_ctl);
		edm_ctl = edm_ctl & (~0x30);
		aice_write_debug_reg(aice, NDS_EDM_SR_EDM_CTL, edm_ctl);
	}

	return ERROR_OK;
}

static int nds32_v3m_check_interrupt_stack(struct nds32 *nds32)
{
	uint32_t val_ir0;
	uint32_t value;

	/* Save interrupt level */
	nds32_get_mapped_reg(nds32, IR0, &val_ir0);
	nds32->current_interrupt_level = (val_ir0 >> 1) & 0x3;

	if (nds32_reach_max_interrupt_level(nds32))
		LOG_ERROR("<-- TARGET ERROR! Reaching the max interrupt stack level %" PRIu32 ". -->",
				nds32->current_interrupt_level);

	/* backup $ir6 to avoid suppressed exception overwrite */
	nds32_get_mapped_reg(nds32, IR6, &value);

	return ERROR_OK;
}

static int nds32_v3m_restore_interrupt_stack(struct nds32 *nds32)
{
	uint32_t value;

	/* get backup value from cache */
	/* then set back to make the register dirty */
	nds32_get_mapped_reg(nds32, IR0, &value);
	nds32_set_mapped_reg(nds32, IR0, value);

	nds32_get_mapped_reg(nds32, IR6, &value);
	nds32_set_mapped_reg(nds32, IR6, value);

	return ERROR_OK;
}

static int nds32_v3m_deassert_reset(struct target *target)
{
	int retval;

	CHECK_RETVAL(nds32_poll(target));

	if (target->state != TARGET_HALTED) {
		/* reset only */
		LOG_WARNING("%s: ran after reset and before halt ...",
				target_name(target));
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;

	}

	return ERROR_OK;
}

static int nds32_v3m_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);
	struct nds32 *nds32 = &(nds32_v3m->nds32);
	int result;

	if (breakpoint->type == BKPT_HARD) {
		/* check hardware resource */
		if (nds32_v3m->next_hbr_index < nds32_v3m->next_hwp_index) {
			LOG_WARNING("<-- TARGET WARNING! Insert too many "
					"hardware breakpoints/watchpoints! "
					"The limit of combined hardware "
					"breakpoints/watchpoints is %" PRId32 ". -->",
					nds32_v3m->n_hbr);
			LOG_WARNING("<-- TARGET STATUS: Inserted number of "
					"hardware breakpoint: %" PRId32 ", hardware "
					"watchpoints: %" PRId32 ". -->",
					nds32_v3m->n_hbr - nds32_v3m->next_hbr_index - 1,
					nds32_v3m->used_n_wp);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		/* update next place to put hardware breakpoint */
		nds32_v3m->next_hbr_index--;

		/* hardware breakpoint insertion occurs before 'continue' actually */
		return ERROR_OK;
	} else if (breakpoint->type == BKPT_SOFT) {
		result = nds32_add_software_breakpoint(target, breakpoint);
		if (ERROR_OK != result) {
			/* auto convert to hardware breakpoint if failed */
			if (nds32->auto_convert_hw_bp) {
				/* convert to hardware breakpoint */
				breakpoint->type = BKPT_HARD;

				return nds32_v3m_add_breakpoint(target, breakpoint);
			}
		}

		return result;
	} else /* unrecognized breakpoint type */
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nds32_v3m_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);

	if (breakpoint->type == BKPT_HARD) {
		if (nds32_v3m->next_hbr_index >= nds32_v3m->n_hbr - 1)
			return ERROR_FAIL;

		/* update next place to put hardware breakpoint */
		nds32_v3m->next_hbr_index++;

		/* hardware breakpoint removal occurs after 'halted' actually */
		return ERROR_OK;
	} else if (breakpoint->type == BKPT_SOFT) {
		return nds32_remove_software_breakpoint(target, breakpoint);
	} else /* unrecognized breakpoint type */
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nds32_v3m_add_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);

	/* check hardware resource */
	if (nds32_v3m->next_hwp_index >= nds32_v3m->n_hwp) {
		/* No hardware resource */
		if (nds32_v3m->nds32.global_stop) {
			LOG_WARNING("<-- TARGET WARNING! The number of "
					"watchpoints exceeds the hardware "
					"resources. Stop at every load/store "
					"instruction to check for watchpoint matches. -->");
			return ERROR_OK;
		}

		LOG_WARNING("<-- TARGET WARNING! Insert too many hardware "
				"watchpoints! The limit of hardware watchpoints "
				"is %" PRId32 ". -->", nds32_v3m->n_hwp);
		LOG_WARNING("<-- TARGET STATUS: Inserted number of "
				"hardware watchpoint: %" PRId32 ". -->",
				nds32_v3m->used_n_wp);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (nds32_v3m->next_hwp_index > nds32_v3m->next_hbr_index) {
		/* No hardware resource */
		if (nds32_v3m->nds32.global_stop) {
			LOG_WARNING("<-- TARGET WARNING! The number of "
					"watchpoints exceeds the hardware "
					"resources. Stop at every load/store "
					"instruction to check for watchpoint matches. -->");
			return ERROR_OK;
		}

		LOG_WARNING("<-- TARGET WARNING! Insert too many hardware "
				"breakpoints/watchpoints! The limit of combined "
				"hardware breakpoints/watchpoints is %" PRId32 ". -->",
				nds32_v3m->n_hbr);
		LOG_WARNING("<-- TARGET STATUS: Inserted number of "
				"hardware breakpoint: %" PRId32 ", hardware "
				"watchpoints: %" PRId32 ". -->",
				nds32_v3m->n_hbr - nds32_v3m->next_hbr_index - 1,
				nds32_v3m->used_n_wp);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* update next place to put hardware watchpoint */
	nds32_v3m->next_hwp_index++;
	nds32_v3m->used_n_wp++;

	return ERROR_OK;
}

static int nds32_v3m_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);

	if (nds32_v3m->next_hwp_index <= 0) {
		if (nds32_v3m->nds32.global_stop)
			return ERROR_OK;

		return ERROR_FAIL;
	}

	/* update next place to put hardware watchpoint */
	nds32_v3m->next_hwp_index--;
	nds32_v3m->used_n_wp--;

	return ERROR_OK;
}

static struct nds32_v3_common_callback nds32_v3m_common_callback = {
	.check_interrupt_stack = nds32_v3m_check_interrupt_stack,
	.restore_interrupt_stack = nds32_v3m_restore_interrupt_stack,
	.activate_hardware_breakpoint = nds32_v3m_activate_hardware_breakpoint,
	.activate_hardware_watchpoint = nds32_v3m_activate_hardware_watchpoint,
	.deactivate_hardware_breakpoint = nds32_v3m_deactivate_hardware_breakpoint,
	.deactivate_hardware_watchpoint = nds32_v3m_deactivate_hardware_watchpoint,
};

static int nds32_v3m_target_create(struct target *target, Jim_Interp *interp)
{
	struct nds32_v3m_common *nds32_v3m;

	nds32_v3m = calloc(1, sizeof(*nds32_v3m));
	if (!nds32_v3m)
		return ERROR_FAIL;

	nds32_v3_common_register_callback(&nds32_v3m_common_callback);
	nds32_v3_target_create_common(target, &(nds32_v3m->nds32));

	return ERROR_OK;
}

/* talk to the target and set things up */
static int nds32_v3m_examine(struct target *target)
{
	struct nds32_v3m_common *nds32_v3m = target_to_nds32_v3m(target);
	struct nds32 *nds32 = &(nds32_v3m->nds32);
	struct aice_port_s *aice = target_to_aice(target);

	if (!target_was_examined(target)) {
		CHECK_RETVAL(nds32_edm_config(nds32));

		if (nds32->reset_halt_as_examine)
			CHECK_RETVAL(nds32_reset_halt(nds32));
	}

	uint32_t edm_cfg;
	aice_read_debug_reg(aice, NDS_EDM_SR_EDM_CFG, &edm_cfg);

	/* get the number of hardware breakpoints */
	nds32_v3m->n_hbr = (edm_cfg & 0x7) + 1;
	nds32_v3m->used_n_wp = 0;

	/* get the number of hardware watchpoints */
	/* If the WP field is hardwired to zero, it means this is a
	 * simple breakpoint.  Otherwise, if the WP field is writable
	 * then it means this is a regular watchpoints. */
	nds32_v3m->n_hwp = 0;
	for (int32_t i = 0 ; i < nds32_v3m->n_hbr ; i++) {
		/** check the hardware breakpoint is simple or not */
		uint32_t tmp_value;
		aice_write_debug_reg(aice, NDS_EDM_SR_BPC0 + i, 0x1);
		aice_read_debug_reg(aice, NDS_EDM_SR_BPC0 + i, &tmp_value);

		if (tmp_value)
			nds32_v3m->n_hwp++;
	}
	/* hardware breakpoint is inserted from high index to low index */
	nds32_v3m->next_hbr_index = nds32_v3m->n_hbr - 1;
	/* hardware watchpoint is inserted from low index to high index */
	nds32_v3m->next_hwp_index = 0;

	LOG_INFO("%s: total hardware breakpoint %" PRId32 " (simple breakpoint %" PRId32 ")",
			target_name(target), nds32_v3m->n_hbr, nds32_v3m->n_hbr - nds32_v3m->n_hwp);
	LOG_INFO("%s: total hardware watchpoint %" PRId32, target_name(target), nds32_v3m->n_hwp);

	nds32->target->state = TARGET_RUNNING;
	nds32->target->debug_reason = DBG_REASON_NOTHALTED;

	target_set_examined(target);

	return ERROR_OK;
}

/** Holds methods for NDS32 V3m targets. */
struct target_type nds32_v3m_target = {
	.name = "nds32_v3m",

	.poll = nds32_poll,
	.arch_state = nds32_arch_state,

	.target_request_data = nds32_v3_target_request_data,

	.halt = nds32_halt,
	.resume = nds32_resume,
	.step = nds32_step,

	.assert_reset = nds32_assert_reset,
	.deassert_reset = nds32_v3m_deassert_reset,

	/* register access */
	.get_gdb_reg_list = nds32_get_gdb_reg_list,

	/* memory access */
	.read_buffer = nds32_v3_read_buffer,
	.write_buffer = nds32_v3_write_buffer,
	.read_memory = nds32_v3_read_memory,
	.write_memory = nds32_v3_write_memory,

	.checksum_memory = nds32_v3_checksum_memory,

	/* breakpoint/watchpoint */
	.add_breakpoint = nds32_v3m_add_breakpoint,
	.remove_breakpoint = nds32_v3m_remove_breakpoint,
	.add_watchpoint = nds32_v3m_add_watchpoint,
	.remove_watchpoint = nds32_v3m_remove_watchpoint,
	.hit_watchpoint = nds32_v3_hit_watchpoint,

	/* MMU */
	.mmu = nds32_mmu,
	.virt2phys = nds32_virtual_to_physical,
	.read_phys_memory = nds32_read_phys_memory,
	.write_phys_memory = nds32_write_phys_memory,

	.run_algorithm = nds32_v3_run_algorithm,

	.commands = nds32_command_handlers,
	.target_create = nds32_v3m_target_create,
	.init_target = nds32_v3_init_target,
	.examine = nds32_v3m_examine,

	.get_gdb_fileio_info = nds32_get_gdb_fileio_info,
	.gdb_fileio_end = nds32_gdb_fileio_end,
};
