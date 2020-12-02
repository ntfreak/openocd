/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009 by Dirk Behme                                      *
 *   dirk.behme@gmail.com - copy from cortex_m3                            *
 *                                                                         *
 *   Copyright (C) 2010 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011                                     *
 *   michel.jaouen@stericsson.com : smp minimum support                    *
 *                                                                         *
 *   Copyright (C) Broadcom 2012                                           *
 *   ehunter@broadcom.com : Cortex-R4 support                              *
 *                                                                         *
 *   Copyright (C) 2013 Kamal Dasu                                         *
 *   kdasu.kdev@gmail.com                                                  *
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
 *                                                                         *
 *   Cortex-A8(tm) TRM, ARM DDI 0344H                                      *
 *   Cortex-A9(tm) TRM, ARM DDI 0407F                                      *
 *   Cortex-A4(tm) TRM, ARM DDI 0363E                                      *
 *   Cortex-A15(tm)TRM, ARM DDI 0438C                                      *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "cortex_a.h"
#include "register.h"
#include "armv7a_mmu.h"
#include "target_request.h"
#include "target_type.h"
#include "arm_opcodes.h"
#include "arm_semihosting.h"
#include "jtag/interface.h"
#include "transport/transport.h"
#include "smp.h"
#include <helper/time_support.h>

static int cortex_a_poll(struct target *target);
static int cortex_a_debug_entry(struct target *target);
static int cortex_a_restore_context(struct target *target, bool bpwp);
static int cortex_a_set_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode);
static int cortex_a_set_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode);
static int cortex_a_set_hybrid_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int cortex_a_unset_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int cortex_a_wait_dscr_bits(struct target *target, uint32_t mask,
	uint32_t value, uint32_t *dscr);
static int cortex_a_mmu(struct target *target, int *enabled);
static int cortex_a_mmu_modify(struct target *target, int enable);
static int cortex_a_virt2phys(struct target *target,
	target_addr_t virt, target_addr_t *phys);
static int cortex_a_read_cpu_memory(struct target *target,
	uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);


/*  restore cp15_control_reg at resume */
static int cortex_a_restore_cp15_control_reg(struct target *target)
{
	int retval = ERROR_OK;
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (cortex_a->cp15_control_reg != cortex_a->cp15_control_reg_curr) {
		cortex_a->cp15_control_reg_curr = cortex_a->cp15_control_reg;
		/* LOG_INFO("cp15_control_reg: %8.8" PRIx32, cortex_a->cp15_control_reg); */
		retval = armv7a->arm.mcr(target, 15,
				0, 0,	/* op1, op2 */
				1, 0,	/* CRn, CRm */
				cortex_a->cp15_control_reg);
	}
	return retval;
}

/*
 * Set up ARM core for memory access.
 * If !phys_access, switch to SVC mode and make sure MMU is on
 * If phys_access, switch off mmu
 */
static int cortex_a_prep_memaccess(struct target *target, int phys_access)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	int mmu_enabled = 0;

	if (phys_access == 0) {
		arm_dpm_modeswitch(&armv7a->dpm, ARM_MODE_SVC);
		cortex_a_mmu(target, &mmu_enabled);
		if (mmu_enabled)
			cortex_a_mmu_modify(target, 1);
		if (cortex_a->dacrfixup_mode == CORTEX_A_DACRFIXUP_ON) {
			/* overwrite DACR to all-manager */
			armv7a->arm.mcr(target, 15,
					0, 0, 3, 0,
					0xFFFFFFFF);
		}
	} else {
		cortex_a_mmu(target, &mmu_enabled);
		if (mmu_enabled)
			cortex_a_mmu_modify(target, 0);
	}
	return ERROR_OK;
}

/*
 * Restore ARM core after memory access.
 * If !phys_access, switch to previous mode
 * If phys_access, restore MMU setting
 */
static int cortex_a_post_memaccess(struct target *target, int phys_access)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);

	if (phys_access == 0) {
		if (cortex_a->dacrfixup_mode == CORTEX_A_DACRFIXUP_ON) {
			/* restore */
			armv7a->arm.mcr(target, 15,
					0, 0, 3, 0,
					cortex_a->cp15_dacr_reg);
		}
		arm_dpm_modeswitch(&armv7a->dpm, ARM_MODE_ANY);
	} else {
		int mmu_enabled = 0;
		cortex_a_mmu(target, &mmu_enabled);
		if (mmu_enabled)
			cortex_a_mmu_modify(target, 1);
	}
	return ERROR_OK;
}


/*  modify cp15_control_reg in order to enable or disable mmu for :
 *  - virt2phys address conversion
 *  - read or write memory in phys or virt address */
static int cortex_a_mmu_modify(struct target *target, int enable)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int retval = ERROR_OK;
	int need_write = 0;

	if (enable) {
		/*  if mmu enabled at target stop and mmu not enable */
		if (!(cortex_a->cp15_control_reg & 0x1U)) {
			LOG_ERROR("trying to enable mmu on target stopped with mmu disable");
			return ERROR_FAIL;
		}
		if ((cortex_a->cp15_control_reg_curr & 0x1U) == 0) {
			cortex_a->cp15_control_reg_curr |= 0x1U;
			need_write = 1;
		}
	} else {
		if ((cortex_a->cp15_control_reg_curr & 0x1U) == 0x1U) {
			cortex_a->cp15_control_reg_curr &= ~0x1U;
			need_write = 1;
		}
	}

	if (need_write) {
		LOG_DEBUG("%s, writing cp15 ctrl: %" PRIx32,
			enable ? "enable mmu" : "disable mmu",
			cortex_a->cp15_control_reg_curr);

		retval = armv7a->arm.mcr(target, 15,
				0, 0,	/* op1, op2 */
				1, 0,	/* CRn, CRm */
				cortex_a->cp15_control_reg_curr);
	}
	return retval;
}

/*
 * Cortex-A Basic debug access, very low level assumes state is saved
 */
static int cortex_a_init_debug_access(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t dscr;
	int retval;

	/* lock memory-mapped access to debug registers to prevent
	 * software interference */
	retval = mem_ap_write_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_LOCKACCESS, 0);
	if (retval != ERROR_OK)
		return retval;

	/* Disable cacheline fills and force cache write-through in debug state */
	retval = mem_ap_write_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCCR, 0);
	if (retval != ERROR_OK)
		return retval;

	/* Disable TLB lookup and refill/eviction in debug state */
	retval = mem_ap_write_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSMCR, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(armv7a->debug_ap->dap);
	if (retval != ERROR_OK)
		return retval;

	/* Enabling of instruction execution in debug mode is done in debug_entry code */

	/* Resync breakpoint registers */

	/* Enable halt for breakpoint, watchpoint and vector catch */
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr | DSCR_HALT_DBG_MODE);
	if (retval != ERROR_OK)
		return retval;

	/* Since this is likely called from init or reset, update target state information*/
	return cortex_a_poll(target);
}

static int cortex_a_wait_instrcmpl(struct target *target, uint32_t *dscr, bool force)
{
	/* Waits until InstrCmpl_l becomes 1, indicating instruction is done.
	 * Writes final value of DSCR into *dscr. Pass force to force always
	 * reading DSCR at least once. */
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int retval;

	if (force) {
		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register");
			return retval;
		}
	}

	retval = cortex_a_wait_dscr_bits(target, DSCR_INSTR_COMP, DSCR_INSTR_COMP, dscr);
	if (retval != ERROR_OK)
		LOG_ERROR("Error waiting for InstrCompl=1");
	return retval;
}

/* To reduce needless round-trips, pass in a pointer to the current
 * DSCR value.  Initialize it to zero if you just need to know the
 * value on return from this function; or DSCR_INSTR_COMP if you
 * happen to know that no instruction is pending.
 */
static int cortex_a_exec_opcode(struct target *target,
	uint32_t opcode, uint32_t *dscr_p)
{
	uint32_t dscr;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);

	dscr = dscr_p ? *dscr_p : 0;

	LOG_DEBUG("exec opcode 0x%08" PRIx32, opcode);

	/* Wait for InstrCompl bit to be set */
	retval = cortex_a_wait_instrcmpl(target, dscr_p, false);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_write_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_ITR, opcode);
	if (retval != ERROR_OK)
		return retval;

	/* Wait for InstrCompl bit to be set */
	retval = cortex_a_wait_instrcmpl(target, &dscr, true);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error waiting for cortex_a_exec_opcode");
		return retval;
	}

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

/* Write to memory mapped registers directly with no cache or mmu handling */
static int cortex_a_dap_write_memap_register_u32(struct target *target,
	uint32_t address,
	uint32_t value)
{
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);

	retval = mem_ap_write_atomic_u32(armv7a->debug_ap, address, value);

	return retval;
}

/*
 * Cortex-A implementation of Debug Programmer's Model
 *
 * NOTE the invariant:  these routines return with DSCR_INSTR_COMP set,
 * so there's no need to poll for it before executing an instruction.
 *
 * NOTE that in several of these cases the "stall" mode might be useful.
 * It'd let us queue a few operations together... prepare/finish might
 * be the places to enable/disable that mode.
 */

static inline struct cortex_a_common *dpm_to_a(struct arm_dpm *dpm)
{
	return container_of(dpm, struct cortex_a_common, armv7a_common.dpm);
}

static int cortex_a_write_dcc(struct cortex_a_common *a, uint32_t data)
{
	LOG_DEBUG("write DCC 0x%08" PRIx32, data);
	return mem_ap_write_u32(a->armv7a_common.debug_ap,
			a->armv7a_common.debug_base + CPUDBG_DTRRX, data);
}

static int cortex_a_read_dcc(struct cortex_a_common *a, uint32_t *data,
	uint32_t *dscr_p)
{
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	if (dscr_p)
		dscr = *dscr_p;

	/* Wait for DTRRXfull */
	retval = cortex_a_wait_dscr_bits(a->armv7a_common.arm.target,
			DSCR_DTR_TX_FULL, DSCR_DTR_TX_FULL, &dscr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error waiting for read dcc");
		return retval;
	}

	retval = mem_ap_read_atomic_u32(a->armv7a_common.debug_ap,
			a->armv7a_common.debug_base + CPUDBG_DTRTX, data);
	if (retval != ERROR_OK)
		return retval;
	/* LOG_DEBUG("read DCC 0x%08" PRIx32, *data); */

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

static int cortex_a_dpm_prepare(struct arm_dpm *dpm)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	uint32_t dscr;
	int retval;

	/* set up invariant:  INSTR_COMP is set after ever DPM operation */
	retval = cortex_a_wait_instrcmpl(dpm->arm->target, &dscr, true);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error waiting for dpm prepare");
		return retval;
	}

	/* this "should never happen" ... */
	if (dscr & DSCR_DTR_RX_FULL) {
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX */
		retval = cortex_a_exec_opcode(
				a->armv7a_common.arm.target,
				ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int cortex_a_dpm_finish(struct arm_dpm *dpm)
{
	/* REVISIT what could be done here? */
	return ERROR_OK;
}

static int cortex_a_instr_write_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	retval = cortex_a_write_dcc(a, data);
	if (retval != ERROR_OK)
		return retval;

	return cortex_a_exec_opcode(
			a->armv7a_common.arm.target,
			opcode,
			&dscr);
}

static int cortex_a_instr_write_data_rt_dcc(struct arm_dpm *dpm,
	uint8_t rt, uint32_t data)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	if (rt > 15)
		return ERROR_TARGET_INVALID;

	retval = cortex_a_write_dcc(a, data);
	if (retval != ERROR_OK)
		return retval;

	/* DCCRX to Rt, "MCR p14, 0, R0, c0, c5, 0", 0xEE000E15 */
	return cortex_a_exec_opcode(
			a->armv7a_common.arm.target,
			ARMV4_5_MRC(14, 0, rt, 0, 5, 0),
			&dscr);
}

static int cortex_a_instr_write_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	retval = cortex_a_instr_write_data_rt_dcc(dpm, 0, data);
	if (retval != ERROR_OK)
		return retval;

	/* then the opcode, taking data from R0 */
	retval = cortex_a_exec_opcode(
			a->armv7a_common.arm.target,
			opcode,
			&dscr);

	return retval;
}

static int cortex_a_instr_cpsr_sync(struct arm_dpm *dpm)
{
	struct target *target = dpm->arm->target;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* "Prefetch flush" after modifying execution status in CPSR */
	return cortex_a_exec_opcode(target,
			ARMV4_5_MCR(15, 0, 0, 7, 5, 4),
			&dscr);
}

static int cortex_a_instr_read_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* the opcode, writing data to DCC */
	retval = cortex_a_exec_opcode(
			a->armv7a_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return cortex_a_read_dcc(a, data, &dscr);
}

static int cortex_a_instr_read_data_rt_dcc(struct arm_dpm *dpm,
	uint8_t rt, uint32_t *data)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	if (rt > 15)
		return ERROR_TARGET_INVALID;

	retval = cortex_a_exec_opcode(
			a->armv7a_common.arm.target,
			ARMV4_5_MCR(14, 0, rt, 0, 5, 0),
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return cortex_a_read_dcc(a, data, &dscr);
}

static int cortex_a_instr_read_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	/* the opcode, writing data to R0 */
	retval = cortex_a_exec_opcode(
			a->armv7a_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	/* write R0 to DCC */
	return cortex_a_instr_read_data_rt_dcc(dpm, 0, data);
}

static int cortex_a_bpwp_enable(struct arm_dpm *dpm, unsigned index_t,
	uint32_t addr, uint32_t control)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	uint32_t vr = a->armv7a_common.debug_base;
	uint32_t cr = a->armv7a_common.debug_base;
	int retval;

	switch (index_t) {
		case 0 ... 15:	/* breakpoints */
			vr += CPUDBG_BVR_BASE;
			cr += CPUDBG_BCR_BASE;
			break;
		case 16 ... 31:	/* watchpoints */
			vr += CPUDBG_WVR_BASE;
			cr += CPUDBG_WCR_BASE;
			index_t -= 16;
			break;
		default:
			return ERROR_FAIL;
	}
	vr += 4 * index_t;
	cr += 4 * index_t;

	LOG_DEBUG("A: bpwp enable, vr %08x cr %08x",
		(unsigned) vr, (unsigned) cr);

	retval = cortex_a_dap_write_memap_register_u32(dpm->arm->target,
			vr, addr);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a_dap_write_memap_register_u32(dpm->arm->target,
			cr, control);
	return retval;
}

static int cortex_a_bpwp_disable(struct arm_dpm *dpm, unsigned index_t)
{
	struct cortex_a_common *a = dpm_to_a(dpm);
	uint32_t cr;

	switch (index_t) {
		case 0 ... 15:
			cr = a->armv7a_common.debug_base + CPUDBG_BCR_BASE;
			break;
		case 16 ... 31:
			cr = a->armv7a_common.debug_base + CPUDBG_WCR_BASE;
			index_t -= 16;
			break;
		default:
			return ERROR_FAIL;
	}
	cr += 4 * index_t;

	LOG_DEBUG("A: bpwp disable, cr %08x", (unsigned) cr);

	/* clear control register */
	return cortex_a_dap_write_memap_register_u32(dpm->arm->target, cr, 0);
}

static int cortex_a_dpm_setup(struct cortex_a_common *a, uint32_t didr)
{
	struct arm_dpm *dpm = &a->armv7a_common.dpm;
	int retval;

	dpm->arm = &a->armv7a_common.arm;
	dpm->didr = didr;

	dpm->prepare = cortex_a_dpm_prepare;
	dpm->finish = cortex_a_dpm_finish;

	dpm->instr_write_data_dcc = cortex_a_instr_write_data_dcc;
	dpm->instr_write_data_r0 = cortex_a_instr_write_data_r0;
	dpm->instr_cpsr_sync = cortex_a_instr_cpsr_sync;

	dpm->instr_read_data_dcc = cortex_a_instr_read_data_dcc;
	dpm->instr_read_data_r0 = cortex_a_instr_read_data_r0;

	dpm->bpwp_enable = cortex_a_bpwp_enable;
	dpm->bpwp_disable = cortex_a_bpwp_disable;

	retval = arm_dpm_setup(dpm);
	if (retval == ERROR_OK)
		retval = arm_dpm_initialize(dpm);

	return retval;
}
static struct target *get_cortex_a(struct target *target, int32_t coreid)
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
static int cortex_a_halt(struct target *target);

static int cortex_a_halt_smp(struct target *target)
{
	int retval = 0;
	struct target_list *head;
	struct target *curr;
	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_HALTED)
			&& target_was_examined(curr))
			retval += cortex_a_halt(curr);
		head = head->next;
	}
	return retval;
}

static int update_halt_gdb(struct target *target)
{
	struct target *gdb_target = NULL;
	struct target_list *head;
	struct target *curr;
	int retval = 0;

	if (target->gdb_service && target->gdb_service->core[0] == -1) {
		target->gdb_service->target = target;
		target->gdb_service->core[0] = target->coreid;
		retval += cortex_a_halt_smp(target);
	}

	if (target->gdb_service)
		gdb_target = target->gdb_service->target;

	foreach_smp_target(head, target->head) {
		curr = head->target;
		/* skip calling context */
		if (curr == target)
			continue;
		if (!target_was_examined(curr))
			continue;
		/* skip targets that were already halted */
		if (curr->state == TARGET_HALTED)
			continue;
		/* Skip gdb_target; it alerts GDB so has to be polled as last one */
		if (curr == gdb_target)
			continue;

		/* avoid recursion in cortex_a_poll() */
		curr->smp = 0;
		cortex_a_poll(curr);
		curr->smp = 1;
	}

	/* after all targets were updated, poll the gdb serving target */
	if (gdb_target != NULL && gdb_target != target)
		cortex_a_poll(gdb_target);
	return retval;
}

/*
 * Cortex-A Run control
 */

static int cortex_a_poll(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	enum target_state prev_target_state = target->state;
	/*  toggle to another core is done by gdb as follow */
	/*  maint packet J core_id */
	/*  continue */
	/*  the next polling trigger an halt event sent to gdb */
	if ((target->state == TARGET_HALTED) && (target->smp) &&
		(target->gdb_service) &&
		(target->gdb_service->target == NULL)) {
		target->gdb_service->target =
			get_cortex_a(target, target->gdb_service->core[1]);
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return retval;
	}
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;
	cortex_a->cpudbg_dscr = dscr;

	if (DSCR_RUN_MODE(dscr) == (DSCR_CORE_HALTED | DSCR_CORE_RESTARTED)) {
		if (prev_target_state != TARGET_HALTED) {
			/* We have a halting debug event */
			LOG_DEBUG("Target halted");
			target->state = TARGET_HALTED;

			retval = cortex_a_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			if (target->smp) {
				retval = update_halt_gdb(target);
				if (retval != ERROR_OK)
					return retval;
			}

			if (prev_target_state == TARGET_DEBUG_RUNNING) {
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			} else { /* prev_target_state is RUNNING, UNKNOWN or RESET */
				if (arm_semihosting(target, &retval) != 0)
					return retval;

				target_call_event_callbacks(target,
					TARGET_EVENT_HALTED);
			}
		}
	} else
		target->state = TARGET_RUNNING;

	return retval;
}

static int cortex_a_halt(struct target *target)
{
	int retval;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);

	/*
	 * Tell the core to be halted by writing DRCR with 0x1
	 * and then wait for the core to be halted.
	 */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DRCR, DRCR_HALT);
	if (retval != ERROR_OK)
		return retval;

	dscr = 0; /* force read of dscr */
	retval = cortex_a_wait_dscr_bits(target, DSCR_CORE_HALTED,
			DSCR_CORE_HALTED, &dscr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error waiting for halt");
		return retval;
	}

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int cortex_a_internal_restore(struct target *target, int current,
	target_addr_t *address, int handle_breakpoints, int debug_execution)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	int retval;
	uint32_t resume_pc;

	if (!debug_execution)
		target_free_all_working_areas(target);

#if 0
	if (debug_execution) {
		/* Disable interrupts */
		/* We disable interrupts in the PRIMASK register instead of
		 * masking with C_MASKINTS,
		 * This is probably the same issue as Cortex-M3 Errata 377493:
		 * C_MASKINTS in parallel with disabled interrupts can cause
		 * local faults to not be taken. */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_PRIMASK].value, 0, 32, 1);
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].dirty = true;
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].valid = true;

		/* Make sure we are in Thumb mode */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32,
			buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0,
			32) | (1 << 24));
		armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty = true;
		armv7m->core_cache->reg_list[ARMV7M_xPSR].valid = true;
	}
#endif

	/* current = 1: continue on current pc, otherwise continue at <address> */
	resume_pc = buf_get_u32(arm->pc->value, 0, 32);
	if (!current)
		resume_pc = *address;
	else
		*address = resume_pc;

	/* Make sure that the Armv7 gdb thumb fixups does not
	 * kill the return address
	 */
	switch (arm->core_state) {
		case ARM_STATE_ARM:
			resume_pc &= 0xFFFFFFFC;
			break;
		case ARM_STATE_THUMB:
		case ARM_STATE_THUMB_EE:
			/* When the return address is loaded into PC
			 * bit 0 must be 1 to stay in Thumb state
			 */
			resume_pc |= 0x1;
			break;
		case ARM_STATE_JAZELLE:
			LOG_ERROR("How do I resume into Jazelle state??");
			return ERROR_FAIL;
		case ARM_STATE_AARCH64:
			LOG_ERROR("Shouldn't be in AARCH64 state");
			return ERROR_FAIL;
	}
	LOG_DEBUG("resume pc = 0x%08" PRIx32, resume_pc);
	buf_set_u32(arm->pc->value, 0, 32, resume_pc);
	arm->pc->dirty = true;
	arm->pc->valid = true;

	/* restore dpm_mode at system halt */
	arm_dpm_modeswitch(&armv7a->dpm, ARM_MODE_ANY);
	/* called it now before restoring context because it uses cpu
	 * register r0 for restoring cp15 control register */
	retval = cortex_a_restore_cp15_control_reg(target);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a_restore_context(target, handle_breakpoints);
	if (retval != ERROR_OK)
		return retval;
	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	/* registers are now invalid */
	register_cache_invalidate(arm->core_cache);

#if 0
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			cortex_m3_unset_breakpoint(target, breakpoint);
			cortex_m3_single_step_core(target);
			cortex_m3_set_breakpoint(target, breakpoint);
		}
	}

#endif
	return retval;
}

static int cortex_a_internal_restart(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	int retval;
	uint32_t dscr;
	/*
	 * * Restart core and wait for it to be started.  Clear ITRen and sticky
	 * * exception flags: see ARMv7 ARM, C5.9.
	 *
	 * REVISIT: for single stepping, we probably want to
	 * disable IRQs by default, with optional override...
	 */

	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	if ((dscr & DSCR_INSTR_COMP) == 0)
		LOG_ERROR("DSCR InstrCompl must be set before leaving debug!");

	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr & ~DSCR_ITR_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DRCR, DRCR_RESTART |
			DRCR_CLEAR_EXCEPTIONS);
	if (retval != ERROR_OK)
		return retval;

	dscr = 0; /* force read of dscr */
	retval = cortex_a_wait_dscr_bits(target, DSCR_CORE_RESTARTED,
			DSCR_CORE_RESTARTED, &dscr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error waiting for resume");
		return retval;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	/* registers are now invalid */
	register_cache_invalidate(arm->core_cache);

	return ERROR_OK;
}

static int cortex_a_restore_smp(struct target *target, int handle_breakpoints)
{
	int retval = 0;
	struct target_list *head;
	struct target *curr;
	target_addr_t address;
	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_RUNNING)
			&& target_was_examined(curr)) {
			/*  resume current address , not in step mode */
			retval += cortex_a_internal_restore(curr, 1, &address,
					handle_breakpoints, 0);
			retval += cortex_a_internal_restart(curr);
		}
		head = head->next;

	}
	return retval;
}

static int cortex_a_resume(struct target *target, int current,
	target_addr_t address, int handle_breakpoints, int debug_execution)
{
	int retval = 0;
	/* dummy resume for smp toggle in order to reduce gdb impact  */
	if ((target->smp) && (target->gdb_service->core[1] != -1)) {
		/*   simulate a start and halt of target */
		target->gdb_service->target = NULL;
		target->gdb_service->core[0] = target->gdb_service->core[1];
		/*  fake resume at next poll we play the  target core[1], see poll*/
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		return 0;
	}
	cortex_a_internal_restore(target, current, &address, handle_breakpoints, debug_execution);
	if (target->smp) {
		target->gdb_service->core[0] = -1;
		retval = cortex_a_restore_smp(target, handle_breakpoints);
		if (retval != ERROR_OK)
			return retval;
	}
	cortex_a_internal_restart(target);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at " TARGET_ADDR_FMT, address);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at " TARGET_ADDR_FMT, address);
	}

	return ERROR_OK;
}

static int cortex_a_debug_entry(struct target *target)
{
	uint32_t dscr;
	int retval = ERROR_OK;
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;

	LOG_DEBUG("dscr = 0x%08" PRIx32, cortex_a->cpudbg_dscr);

	/* REVISIT surely we should not re-read DSCR !! */
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	/* REVISIT see A TRM 12.11.4 steps 2..3 -- make sure that any
	 * imprecise data aborts get discarded by issuing a Data
	 * Synchronization Barrier:  ARMV4_5_MCR(15, 0, 0, 7, 10, 4).
	 */

	/* Enable the ITR execution once we are in debug mode */
	dscr |= DSCR_ITR_EN;
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Examine debug reason */
	arm_dpm_report_dscr(&armv7a->dpm, cortex_a->cpudbg_dscr);

	/* save address of instruction that triggered the watchpoint? */
	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		uint32_t wfar;

		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_WFAR,
				&wfar);
		if (retval != ERROR_OK)
			return retval;
		arm_dpm_report_wfar(&armv7a->dpm, wfar);
	}

	/* First load register accessible through core debug port */
	retval = arm_dpm_read_current_registers(&armv7a->dpm);
	if (retval != ERROR_OK)
		return retval;

	if (arm->spsr) {
		/* read SPSR */
		retval = arm_dpm_read_reg(&armv7a->dpm, arm->spsr, 17);
		if (retval != ERROR_OK)
			return retval;
	}

#if 0
/* TODO, Move this */
	uint32_t cp15_control_register, cp15_cacr, cp15_nacr;
	cortex_a_read_cp(target, &cp15_control_register, 15, 0, 1, 0, 0);
	LOG_DEBUG("cp15_control_register = 0x%08x", cp15_control_register);

	cortex_a_read_cp(target, &cp15_cacr, 15, 0, 1, 0, 2);
	LOG_DEBUG("cp15 Coprocessor Access Control Register = 0x%08x", cp15_cacr);

	cortex_a_read_cp(target, &cp15_nacr, 15, 0, 1, 1, 2);
	LOG_DEBUG("cp15 Nonsecure Access Control Register = 0x%08x", cp15_nacr);
#endif

	/* Are we in an exception handler */
/*	armv4_5->exception_number = 0; */
	if (armv7a->post_debug_entry) {
		retval = armv7a->post_debug_entry(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int cortex_a_post_debug_entry(struct target *target)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	int retval;

	/* MRC p15,0,<Rt>,c1,c0,0 ; Read CP15 System Control Register */
	retval = armv7a->arm.mrc(target, 15,
			0, 0,	/* op1, op2 */
			1, 0,	/* CRn, CRm */
			&cortex_a->cp15_control_reg);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("cp15_control_reg: %8.8" PRIx32, cortex_a->cp15_control_reg);
	cortex_a->cp15_control_reg_curr = cortex_a->cp15_control_reg;

	if (!armv7a->is_armv7r)
		armv7a_read_ttbcr(target);

	if (armv7a->armv7a_mmu.armv7a_cache.info == -1)
		armv7a_identify_cache(target);

	if (armv7a->is_armv7r) {
		armv7a->armv7a_mmu.mmu_enabled = 0;
	} else {
		armv7a->armv7a_mmu.mmu_enabled =
			(cortex_a->cp15_control_reg & 0x1U) ? 1 : 0;
	}
	armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled =
		(cortex_a->cp15_control_reg & 0x4U) ? 1 : 0;
	armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled =
		(cortex_a->cp15_control_reg & 0x1000U) ? 1 : 0;
	cortex_a->curr_mode = armv7a->arm.core_mode;

	/* switch to SVC mode to read DACR */
	arm_dpm_modeswitch(&armv7a->dpm, ARM_MODE_SVC);
	armv7a->arm.mrc(target, 15,
			0, 0, 3, 0,
			&cortex_a->cp15_dacr_reg);

	LOG_DEBUG("cp15_dacr_reg: %8.8" PRIx32,
			cortex_a->cp15_dacr_reg);

	arm_dpm_modeswitch(&armv7a->dpm, ARM_MODE_ANY);
	return ERROR_OK;
}

static int cortex_a_set_dscr_bits(struct target *target,
		unsigned long bit_mask, unsigned long value)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t dscr;

	/* Read DSCR */
	int retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (ERROR_OK != retval)
		return retval;

	/* clear bitfield */
	dscr &= ~bit_mask;
	/* put new value */
	dscr |= value & bit_mask;

	/* write new DSCR */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr);
	return retval;
}

static int cortex_a_step(struct target *target, int current, target_addr_t address,
	int handle_breakpoints)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	struct breakpoint *breakpoint = NULL;
	struct breakpoint stepbreakpoint;
	struct reg *r;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	r = arm->pc;
	if (!current)
		buf_set_u32(r->value, 0, 32, address);
	else
		address = buf_get_u32(r->value, 0, 32);

	/* The front-end may request us not to handle breakpoints.
	 * But since Cortex-A uses breakpoint for single step,
	 * we MUST handle breakpoints.
	 */
	handle_breakpoints = 1;
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, address);
		if (breakpoint)
			cortex_a_unset_breakpoint(target, breakpoint);
	}

	/* Setup single step breakpoint */
	stepbreakpoint.address = address;
	stepbreakpoint.asid = 0;
	stepbreakpoint.length = (arm->core_state == ARM_STATE_THUMB)
		? 2 : 4;
	stepbreakpoint.type = BKPT_HARD;
	stepbreakpoint.set = 0;

	/* Disable interrupts during single step if requested */
	if (cortex_a->isrmasking_mode == CORTEX_A_ISRMASK_ON) {
		retval = cortex_a_set_dscr_bits(target, DSCR_INT_DIS, DSCR_INT_DIS);
		if (ERROR_OK != retval)
			return retval;
	}

	/* Break on IVA mismatch */
	cortex_a_set_breakpoint(target, &stepbreakpoint, 0x04);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	retval = cortex_a_resume(target, 1, address, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	int64_t then = timeval_ms();
	while (target->state != TARGET_HALTED) {
		retval = cortex_a_poll(target);
		if (retval != ERROR_OK)
			return retval;
		if (target->state == TARGET_HALTED)
			break;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("timeout waiting for target halt");
			return ERROR_FAIL;
		}
	}

	cortex_a_unset_breakpoint(target, &stepbreakpoint);

	/* Re-enable interrupts if they were disabled */
	if (cortex_a->isrmasking_mode == CORTEX_A_ISRMASK_ON) {
		retval = cortex_a_set_dscr_bits(target, DSCR_INT_DIS, 0);
		if (ERROR_OK != retval)
			return retval;
	}


	target->debug_reason = DBG_REASON_BREAKPOINT;

	if (breakpoint)
		cortex_a_set_breakpoint(target, breakpoint, 0);

	if (target->state != TARGET_HALTED)
		LOG_DEBUG("target stepped");

	return ERROR_OK;
}

static int cortex_a_restore_context(struct target *target, bool bpwp)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	LOG_DEBUG(" ");

	if (armv7a->pre_restore_context)
		armv7a->pre_restore_context(target);

	return arm_dpm_write_dirty_registers(&armv7a->dpm, bpwp);
}

/*
 * Cortex-A Breakpoint and watchpoint functions
 */

/* Setup hardware Breakpoint Register Pair */
static int cortex_a_set_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode)
{
	int retval;
	int brp_i = 0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	struct cortex_a_brp *brp_list = cortex_a->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		while (brp_list[brp_i].used && (brp_i < cortex_a->brp_num))
			brp_i++;
		if (brp_i >= cortex_a->brp_num) {
			LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint->set = brp_i + 1;
		if (breakpoint->length == 2)
			byte_addr_select = (3 << (breakpoint->address & 0x02));
		control = ((matchmode & 0x7) << 20)
			| (byte_addr_select << 5)
			| (3 << 1) | 1;
		brp_list[brp_i].used = 1;
		brp_list[brp_i].value = (breakpoint->address & 0xFFFFFFFC);
		brp_list[brp_i].control = control;
		retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].value);
		if (retval != ERROR_OK)
			return retval;
		retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
			brp_list[brp_i].control,
			brp_list[brp_i].value);
	} else if (breakpoint->type == BKPT_SOFT) {
		uint8_t code[4];
		/* length == 2: Thumb breakpoint */
		if (breakpoint->length == 2)
			buf_set_u32(code, 0, 32, ARMV5_T_BKPT(0x11));
		else
		/* length == 3: Thumb-2 breakpoint, actual encoding is
		 * a regular Thumb BKPT instruction but we replace a
		 * 32bit Thumb-2 instruction, so fix-up the breakpoint
		 * length
		 */
		if (breakpoint->length == 3) {
			buf_set_u32(code, 0, 32, ARMV5_T_BKPT(0x11));
			breakpoint->length = 4;
		} else
			/* length == 4, normal ARM breakpoint */
			buf_set_u32(code, 0, 32, ARMV5_BKPT(0x11));

		retval = target_read_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;

		/* make sure data cache is cleaned & invalidated down to PoC */
		if (!armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled) {
			armv7a_cache_flush_virt(target, breakpoint->address,
						breakpoint->length);
		}

		retval = target_write_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1, code);
		if (retval != ERROR_OK)
			return retval;

		/* update i-cache at breakpoint location */
		armv7a_l1_d_cache_inval_virt(target, breakpoint->address,
					breakpoint->length);
		armv7a_l1_i_cache_inval_virt(target, breakpoint->address,
						 breakpoint->length);

		breakpoint->set = 0x11;	/* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int cortex_a_set_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode)
{
	int retval = ERROR_FAIL;
	int brp_i = 0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	struct cortex_a_brp *brp_list = cortex_a->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return retval;
	}
	/*check available context BRPs*/
	while ((brp_list[brp_i].used ||
		(brp_list[brp_i].type != BRP_CONTEXT)) && (brp_i < cortex_a->brp_num))
		brp_i++;

	if (brp_i >= cortex_a->brp_num) {
		LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
		return ERROR_FAIL;
	}

	breakpoint->set = brp_i + 1;
	control = ((matchmode & 0x7) << 20)
		| (byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_i].used = 1;
	brp_list[brp_i].value = (breakpoint->asid);
	brp_list[brp_i].control = control;
	retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
			brp_list[brp_i].value);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
			brp_list[brp_i].control);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
		brp_list[brp_i].control,
		brp_list[brp_i].value);
	return ERROR_OK;

}

static int cortex_a_set_hybrid_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval = ERROR_FAIL;
	int brp_1 = 0;	/* holds the contextID pair */
	int brp_2 = 0;	/* holds the IVA pair */
	uint32_t control_CTX, control_IVA;
	uint8_t CTX_byte_addr_select = 0x0F;
	uint8_t IVA_byte_addr_select = 0x0F;
	uint8_t CTX_machmode = 0x03;
	uint8_t IVA_machmode = 0x01;
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	struct cortex_a_brp *brp_list = cortex_a->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return retval;
	}
	/*check available context BRPs*/
	while ((brp_list[brp_1].used ||
		(brp_list[brp_1].type != BRP_CONTEXT)) && (brp_1 < cortex_a->brp_num))
		brp_1++;

	printf("brp(CTX) found num: %d\n", brp_1);
	if (brp_1 >= cortex_a->brp_num) {
		LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
		return ERROR_FAIL;
	}

	while ((brp_list[brp_2].used ||
		(brp_list[brp_2].type != BRP_NORMAL)) && (brp_2 < cortex_a->brp_num))
		brp_2++;

	printf("brp(IVA) found num: %d\n", brp_2);
	if (brp_2 >= cortex_a->brp_num) {
		LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
		return ERROR_FAIL;
	}

	breakpoint->set = brp_1 + 1;
	breakpoint->linked_BRP = brp_2;
	control_CTX = ((CTX_machmode & 0x7) << 20)
		| (brp_2 << 16)
		| (0 << 14)
		| (CTX_byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_1].used = 1;
	brp_list[brp_1].value = (breakpoint->asid);
	brp_list[brp_1].control = control_CTX;
	retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_1].BRPn,
			brp_list[brp_1].value);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_1].BRPn,
			brp_list[brp_1].control);
	if (retval != ERROR_OK)
		return retval;

	control_IVA = ((IVA_machmode & 0x7) << 20)
		| (brp_1 << 16)
		| (IVA_byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_2].used = 1;
	brp_list[brp_2].value = (breakpoint->address & 0xFFFFFFFC);
	brp_list[brp_2].control = control_IVA;
	retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_2].BRPn,
			brp_list[brp_2].value);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_2].BRPn,
			brp_list[brp_2].control);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int cortex_a_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	struct cortex_a_brp *brp_list = cortex_a->brp_list;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		if ((breakpoint->address != 0) && (breakpoint->asid != 0)) {
			int brp_i = breakpoint->set - 1;
			int brp_j = breakpoint->linked_BRP;
			if ((brp_i < 0) || (brp_i >= cortex_a->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;
			if ((brp_j < 0) || (brp_j >= cortex_a->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_j,
				brp_list[brp_j].control, brp_list[brp_j].value);
			brp_list[brp_j].used = 0;
			brp_list[brp_j].value = 0;
			brp_list[brp_j].control = 0;
			retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BCR_BASE + 4 * brp_list[brp_j].BRPn,
					brp_list[brp_j].control);
			if (retval != ERROR_OK)
				return retval;
			retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BVR_BASE + 4 * brp_list[brp_j].BRPn,
					brp_list[brp_j].value);
			if (retval != ERROR_OK)
				return retval;
			breakpoint->linked_BRP = 0;
			breakpoint->set = 0;
			return ERROR_OK;

		} else {
			int brp_i = breakpoint->set - 1;
			if ((brp_i < 0) || (brp_i >= cortex_a->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			retval = cortex_a_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;
			breakpoint->set = 0;
			return ERROR_OK;
		}
	} else {

		/* make sure data cache is cleaned & invalidated down to PoC */
		if (!armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled) {
			armv7a_cache_flush_virt(target, breakpoint->address,
						breakpoint->length);
		}

		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4) {
			retval = target_write_memory(target,
					breakpoint->address & 0xFFFFFFFE,
					4, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = target_write_memory(target,
					breakpoint->address & 0xFFFFFFFE,
					2, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}

		/* update i-cache at breakpoint location */
		armv7a_l1_d_cache_inval_virt(target, breakpoint->address,
						 breakpoint->length);
		armv7a_l1_i_cache_inval_virt(target, breakpoint->address,
						 breakpoint->length);
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

static int cortex_a_add_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a->brp_num_available--;

	return cortex_a_set_breakpoint(target, breakpoint, 0x00);	/* Exact match */
}

static int cortex_a_add_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a->brp_num_available--;

	return cortex_a_set_context_breakpoint(target, breakpoint, 0x02);	/* asid match */
}

static int cortex_a_add_hybrid_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a->brp_num_available--;

	return cortex_a_set_hybrid_breakpoint(target, breakpoint);	/* ??? */
}


static int cortex_a_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);

#if 0
/* It is perfectly possible to remove breakpoints while the target is running */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
#endif

	if (breakpoint->set) {
		cortex_a_unset_breakpoint(target, breakpoint);
		if (breakpoint->type == BKPT_HARD)
			cortex_a->brp_num_available++;
	}


	return ERROR_OK;
}

/*
 * Cortex-A Reset functions
 */

static int cortex_a_assert_reset(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	LOG_DEBUG(" ");

	/* FIXME when halt is requested, make it work somehow... */

	/* This function can be called in "target not examined" state */

	/* Issue some kind of warm reset. */
	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT))
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	else if (jtag_get_reset_config() & RESET_HAS_SRST) {
		/* REVISIT handle "pulls" cases, if there's
		 * hardware that needs them to work.
		 */

		/*
		 * FIXME: fix reset when transport is not JTAG. This is a temporary
		 * work-around for release v0.10 that is not intended to stay!
		 */
		if (!transport_is_jtag() ||
				(target->reset_halt && (jtag_get_reset_config() & RESET_SRST_NO_GATING)))
			adapter_assert_reset();

	} else {
		LOG_ERROR("%s: how to reset?", target_name(target));
		return ERROR_FAIL;
	}

	/* registers are now invalid */
	if (target_was_examined(target))
		register_cache_invalidate(armv7a->arm.core_cache);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int cortex_a_deassert_reset(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int retval;

	LOG_DEBUG(" ");

	/* be certain SRST is off */
	adapter_deassert_reset();

	if (target_was_examined(target)) {
		retval = cortex_a_poll(target);
		if (retval != ERROR_OK)
			return retval;
	}

	if (target->reset_halt) {
		if (target->state != TARGET_HALTED) {
			LOG_WARNING("%s: ran after reset and before halt ...",
				target_name(target));
			if (target_was_examined(target)) {
				retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
						armv7a->debug_base + CPUDBG_DRCR, DRCR_HALT);
				if (retval != ERROR_OK)
					return retval;
			} else
				target->state = TARGET_UNKNOWN;
		}
	}

	return ERROR_OK;
}

static int cortex_a_set_dcc_mode(struct target *target, uint32_t mode, uint32_t *dscr)
{
	/* Changes the mode of the DCC between non-blocking, stall, and fast mode.
	 * New desired mode must be in mode. Current value of DSCR must be in
	 * *dscr, which is updated with new value.
	 *
	 * This function elides actually sending the mode-change over the debug
	 * interface if the mode is already set as desired.
	 */
	uint32_t new_dscr = (*dscr & ~DSCR_EXT_DCC_MASK) | mode;
	if (new_dscr != *dscr) {
		struct armv7a_common *armv7a = target_to_armv7a(target);
		int retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, new_dscr);
		if (retval == ERROR_OK)
			*dscr = new_dscr;
		return retval;
	} else {
		return ERROR_OK;
	}
}

static int cortex_a_wait_dscr_bits(struct target *target, uint32_t mask,
	uint32_t value, uint32_t *dscr)
{
	/* Waits until the specified bit(s) of DSCR take on a specified value. */
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int64_t then;
	int retval;

	if ((*dscr & mask) == value)
		return ERROR_OK;

	then = timeval_ms();
	while (1) {
		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register");
			return retval;
		}
		if ((*dscr & mask) == value)
			break;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("timeout waiting for DSCR bit change");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int cortex_a_read_copro(struct target *target, uint32_t opcode,
	uint32_t *data, uint32_t *dscr)
{
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);

	/* Move from coprocessor to R0. */
	retval = cortex_a_exec_opcode(target, opcode, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Move from R0 to DTRTX. */
	retval = cortex_a_exec_opcode(target, ARMV4_5_MCR(14, 0, 0, 0, 5, 0), dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Wait until DTRTX is full (according to ARMv7-A/-R architecture
	 * manual section C8.4.3, checking InstrCmpl_l is not sufficient; one
	 * must also check TXfull_l). Most of the time this will be free
	 * because TXfull_l will be set immediately and cached in dscr. */
	retval = cortex_a_wait_dscr_bits(target, DSCR_DTRTX_FULL_LATCHED,
			DSCR_DTRTX_FULL_LATCHED, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Read the value transferred to DTRTX. */
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRTX, data);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int cortex_a_read_dfar_dfsr(struct target *target, uint32_t *dfar,
	uint32_t *dfsr, uint32_t *dscr)
{
	int retval;

	if (dfar) {
		retval = cortex_a_read_copro(target, ARMV4_5_MRC(15, 0, 0, 6, 0, 0), dfar, dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	if (dfsr) {
		retval = cortex_a_read_copro(target, ARMV4_5_MRC(15, 0, 0, 5, 0, 0), dfsr, dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int cortex_a_write_copro(struct target *target, uint32_t opcode,
	uint32_t data, uint32_t *dscr)
{
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);

	/* Write the value into DTRRX. */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRRX, data);
	if (retval != ERROR_OK)
		return retval;

	/* Move from DTRRX to R0. */
	retval = cortex_a_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0), dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Move from R0 to coprocessor. */
	retval = cortex_a_exec_opcode(target, opcode, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Wait until DTRRX is empty (according to ARMv7-A/-R architecture manual
	 * section C8.4.3, checking InstrCmpl_l is not sufficient; one must also
	 * check RXfull_l). Most of the time this will be free because RXfull_l
	 * will be cleared immediately and cached in dscr. */
	retval = cortex_a_wait_dscr_bits(target, DSCR_DTRRX_FULL_LATCHED, 0, dscr);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int cortex_a_write_dfar_dfsr(struct target *target, uint32_t dfar,
	uint32_t dfsr, uint32_t *dscr)
{
	int retval;

	retval = cortex_a_write_copro(target, ARMV4_5_MCR(15, 0, 0, 6, 0, 0), dfar, dscr);
	if (retval != ERROR_OK)
		return retval;

	retval = cortex_a_write_copro(target, ARMV4_5_MCR(15, 0, 0, 5, 0, 0), dfsr, dscr);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int cortex_a_dfsr_to_error_code(uint32_t dfsr)
{
	uint32_t status, upper4;

	if (dfsr & (1 << 9)) {
		/* LPAE format. */
		status = dfsr & 0x3f;
		upper4 = status >> 2;
		if (upper4 == 1 || upper4 == 2 || upper4 == 3 || upper4 == 15)
			return ERROR_TARGET_TRANSLATION_FAULT;
		else if (status == 33)
			return ERROR_TARGET_UNALIGNED_ACCESS;
		else
			return ERROR_TARGET_DATA_ABORT;
	} else {
		/* Normal format. */
		status = ((dfsr >> 6) & 0x10) | (dfsr & 0xf);
		if (status == 1)
			return ERROR_TARGET_UNALIGNED_ACCESS;
		else if (status == 5 || status == 7 || status == 3 || status == 6 ||
				status == 9 || status == 11 || status == 13 || status == 15)
			return ERROR_TARGET_TRANSLATION_FAULT;
		else
			return ERROR_TARGET_DATA_ABORT;
	}
}

static int cortex_a_write_cpu_memory_slow(struct target *target,
	uint32_t size, uint32_t count, const uint8_t *buffer, uint32_t *dscr)
{
	/* Writes count objects of size size from *buffer. Old value of DSCR must
	 * be in *dscr; updated to new value. This is slow because it works for
	 * non-word-sized objects. Avoid unaligned accesses as they do not work
	 * on memory address space without "Normal" attribute. If size == 4 and
	 * the address is aligned, cortex_a_write_cpu_memory_fast should be
	 * preferred.
	 * Preconditions:
	 * - Address is in R0.
	 * - R0 is marked dirty.
	 */
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	int retval;

	/* Mark register R1 as dirty, to use for transferring data. */
	arm_reg_current(arm, 1)->dirty = true;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Go through the objects. */
	while (count) {
		/* Write the value to store into DTRRX. */
		uint32_t data, opcode;
		if (size == 1)
			data = *buffer;
		else if (size == 2)
			data = target_buffer_get_u16(target, buffer);
		else
			data = target_buffer_get_u32(target, buffer);
		retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DTRRX, data);
		if (retval != ERROR_OK)
			return retval;

		/* Transfer the value from DTRRX to R1. */
		retval = cortex_a_exec_opcode(target, ARMV4_5_MRC(14, 0, 1, 0, 5, 0), dscr);
		if (retval != ERROR_OK)
			return retval;

		/* Write the value transferred to R1 into memory. */
		if (size == 1)
			opcode = ARMV4_5_STRB_IP(1, 0);
		else if (size == 2)
			opcode = ARMV4_5_STRH_IP(1, 0);
		else
			opcode = ARMV4_5_STRW_IP(1, 0);
		retval = cortex_a_exec_opcode(target, opcode, dscr);
		if (retval != ERROR_OK)
			return retval;

		/* Check for faults and return early. */
		if (*dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE))
			return ERROR_OK; /* A data fault is not considered a system failure. */

		/* Wait until DTRRX is empty (according to ARMv7-A/-R architecture
		 * manual section C8.4.3, checking InstrCmpl_l is not sufficient; one
		 * must also check RXfull_l). Most of the time this will be free
		 * because RXfull_l will be cleared immediately and cached in dscr. */
		retval = cortex_a_wait_dscr_bits(target, DSCR_DTRRX_FULL_LATCHED, 0, dscr);
		if (retval != ERROR_OK)
			return retval;

		/* Advance. */
		buffer += size;
		--count;
	}

	return ERROR_OK;
}

static int cortex_a_write_cpu_memory_fast(struct target *target,
	uint32_t count, const uint8_t *buffer, uint32_t *dscr)
{
	/* Writes count objects of size 4 from *buffer. Old value of DSCR must be
	 * in *dscr; updated to new value. This is fast but only works for
	 * word-sized objects at aligned addresses.
	 * Preconditions:
	 * - Address is in R0 and must be a multiple of 4.
	 * - R0 is marked dirty.
	 */
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int retval;

	/* Switch to fast mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_FAST_MODE, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Latch STC instruction. */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_ITR, ARMV4_5_STC(0, 1, 0, 1, 14, 5, 0, 4));
	if (retval != ERROR_OK)
		return retval;

	/* Transfer all the data and issue all the instructions. */
	return mem_ap_write_buf_noincr(armv7a->debug_ap, buffer,
			4, count, armv7a->debug_base + CPUDBG_DTRRX);
}

static int cortex_a_write_cpu_memory(struct target *target,
	uint32_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	/* Write memory through the CPU. */
	int retval, final_retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	uint32_t dscr, orig_dfar, orig_dfsr, fault_dscr, fault_dfar, fault_dfsr;

	LOG_DEBUG("Writing CPU memory address 0x%" PRIx32 " size %"  PRIu32 " count %"  PRIu32,
			  address, size, count);
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!count)
		return ERROR_OK;

	/* Clear any abort. */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DRCR, DRCR_CLEAR_EXCEPTIONS);
	if (retval != ERROR_OK)
		return retval;

	/* Read DSCR. */
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, &dscr);
	if (retval != ERROR_OK)
		goto out;

	/* Mark R0 as dirty. */
	arm_reg_current(arm, 0)->dirty = true;

	/* Read DFAR and DFSR, as they will be modified in the event of a fault. */
	retval = cortex_a_read_dfar_dfsr(target, &orig_dfar, &orig_dfsr, &dscr);
	if (retval != ERROR_OK)
		goto out;

	/* Get the memory address into R0. */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRRX, address);
	if (retval != ERROR_OK)
		goto out;
	retval = cortex_a_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0), &dscr);
	if (retval != ERROR_OK)
		goto out;

	if (size == 4 && (address % 4) == 0) {
		/* We are doing a word-aligned transfer, so use fast mode. */
		retval = cortex_a_write_cpu_memory_fast(target, count, buffer, &dscr);
	} else {
		/* Use slow path. Adjust size for aligned accesses */
		switch (address % 4) {
			case 1:
			case 3:
				count *= size;
				size = 1;
				break;
			case 2:
				if (size == 4) {
					count *= 2;
					size = 2;
				}
			case 0:
			default:
				break;
		}
		retval = cortex_a_write_cpu_memory_slow(target, size, count, buffer, &dscr);
	}

out:
	final_retval = retval;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, &dscr);
	if (final_retval == ERROR_OK)
		final_retval = retval;

	/* Wait for last issued instruction to complete. */
	retval = cortex_a_wait_instrcmpl(target, &dscr, true);
	if (final_retval == ERROR_OK)
		final_retval = retval;

	/* Wait until DTRRX is empty (according to ARMv7-A/-R architecture manual
	 * section C8.4.3, checking InstrCmpl_l is not sufficient; one must also
	 * check RXfull_l). Most of the time this will be free because RXfull_l
	 * will be cleared immediately and cached in dscr. However, don't do this
	 * if there is fault, because then the instruction might not have completed
	 * successfully. */
	if (!(dscr & DSCR_STICKY_ABORT_PRECISE)) {
		retval = cortex_a_wait_dscr_bits(target, DSCR_DTRRX_FULL_LATCHED, 0, &dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	/* If there were any sticky abort flags, clear them. */
	if (dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE)) {
		fault_dscr = dscr;
		mem_ap_write_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DRCR, DRCR_CLEAR_EXCEPTIONS);
		dscr &= ~(DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE);
	} else {
		fault_dscr = 0;
	}

	/* Handle synchronous data faults. */
	if (fault_dscr & DSCR_STICKY_ABORT_PRECISE) {
		if (final_retval == ERROR_OK) {
			/* Final return value will reflect cause of fault. */
			retval = cortex_a_read_dfar_dfsr(target, &fault_dfar, &fault_dfsr, &dscr);
			if (retval == ERROR_OK) {
				LOG_ERROR("data abort at 0x%08" PRIx32 ", dfsr = 0x%08" PRIx32, fault_dfar, fault_dfsr);
				final_retval = cortex_a_dfsr_to_error_code(fault_dfsr);
			} else
				final_retval = retval;
		}
		/* Fault destroyed DFAR/DFSR; restore them. */
		retval = cortex_a_write_dfar_dfsr(target, orig_dfar, orig_dfsr, &dscr);
		if (retval != ERROR_OK)
			LOG_ERROR("error restoring dfar/dfsr - dscr = 0x%08" PRIx32, dscr);
	}

	/* Handle asynchronous data faults. */
	if (fault_dscr & DSCR_STICKY_ABORT_IMPRECISE) {
		if (final_retval == ERROR_OK)
			/* No other error has been recorded so far, so keep this one. */
			final_retval = ERROR_TARGET_DATA_ABORT;
	}

	/* If the DCC is nonempty, clear it. */
	if (dscr & DSCR_DTRTX_FULL_LATCHED) {
		uint32_t dummy;
		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DTRTX, &dummy);
		if (final_retval == ERROR_OK)
			final_retval = retval;
	}
	if (dscr & DSCR_DTRRX_FULL_LATCHED) {
		retval = cortex_a_exec_opcode(target, ARMV4_5_MRC(14, 0, 1, 0, 5, 0), &dscr);
		if (final_retval == ERROR_OK)
			final_retval = retval;
	}

	/* Done. */
	return final_retval;
}

static int cortex_a_read_cpu_memory_slow(struct target *target,
	uint32_t size, uint32_t count, uint8_t *buffer, uint32_t *dscr)
{
	/* Reads count objects of size size into *buffer. Old value of DSCR must be
	 * in *dscr; updated to new value. This is slow because it works for
	 * non-word-sized objects. Avoid unaligned accesses as they do not work
	 * on memory address space without "Normal" attribute. If size == 4 and
	 * the address is aligned, cortex_a_read_cpu_memory_fast should be
	 * preferred.
	 * Preconditions:
	 * - Address is in R0.
	 * - R0 is marked dirty.
	 */
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	int retval;

	/* Mark register R1 as dirty, to use for transferring data. */
	arm_reg_current(arm, 1)->dirty = true;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Go through the objects. */
	while (count) {
		/* Issue a load of the appropriate size to R1. */
		uint32_t opcode, data;
		if (size == 1)
			opcode = ARMV4_5_LDRB_IP(1, 0);
		else if (size == 2)
			opcode = ARMV4_5_LDRH_IP(1, 0);
		else
			opcode = ARMV4_5_LDRW_IP(1, 0);
		retval = cortex_a_exec_opcode(target, opcode, dscr);
		if (retval != ERROR_OK)
			return retval;

		/* Issue a write of R1 to DTRTX. */
		retval = cortex_a_exec_opcode(target, ARMV4_5_MCR(14, 0, 1, 0, 5, 0), dscr);
		if (retval != ERROR_OK)
			return retval;

		/* Check for faults and return early. */
		if (*dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE))
			return ERROR_OK; /* A data fault is not considered a system failure. */

		/* Wait until DTRTX is full (according to ARMv7-A/-R architecture
		 * manual section C8.4.3, checking InstrCmpl_l is not sufficient; one
		 * must also check TXfull_l). Most of the time this will be free
		 * because TXfull_l will be set immediately and cached in dscr. */
		retval = cortex_a_wait_dscr_bits(target, DSCR_DTRTX_FULL_LATCHED,
				DSCR_DTRTX_FULL_LATCHED, dscr);
		if (retval != ERROR_OK)
			return retval;

		/* Read the value transferred to DTRTX into the buffer. */
		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DTRTX, &data);
		if (retval != ERROR_OK)
			return retval;
		if (size == 1)
			*buffer = (uint8_t) data;
		else if (size == 2)
			target_buffer_set_u16(target, buffer, (uint16_t) data);
		else
			target_buffer_set_u32(target, buffer, data);

		/* Advance. */
		buffer += size;
		--count;
	}

	return ERROR_OK;
}

static int cortex_a_read_cpu_memory_fast(struct target *target,
	uint32_t count, uint8_t *buffer, uint32_t *dscr)
{
	/* Reads count objects of size 4 into *buffer. Old value of DSCR must be in
	 * *dscr; updated to new value. This is fast but only works for word-sized
	 * objects at aligned addresses.
	 * Preconditions:
	 * - Address is in R0 and must be a multiple of 4.
	 * - R0 is marked dirty.
	 */
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t u32;
	int retval;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Issue the LDC instruction via a write to ITR. */
	retval = cortex_a_exec_opcode(target, ARMV4_5_LDC(0, 1, 0, 1, 14, 5, 0, 4), dscr);
	if (retval != ERROR_OK)
		return retval;

	count--;

	if (count > 0) {
		/* Switch to fast mode if not already in that mode. */
		retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_FAST_MODE, dscr);
		if (retval != ERROR_OK)
			return retval;

		/* Latch LDC instruction. */
		retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_ITR, ARMV4_5_LDC(0, 1, 0, 1, 14, 5, 0, 4));
		if (retval != ERROR_OK)
			return retval;

		/* Read the value transferred to DTRTX into the buffer. Due to fast
		 * mode rules, this blocks until the instruction finishes executing and
		 * then reissues the read instruction to read the next word from
		 * memory. The last read of DTRTX in this call reads the second-to-last
		 * word from memory and issues the read instruction for the last word.
		 */
		retval = mem_ap_read_buf_noincr(armv7a->debug_ap, buffer,
				4, count, armv7a->debug_base + CPUDBG_DTRTX);
		if (retval != ERROR_OK)
			return retval;

		/* Advance. */
		buffer += count * 4;
	}

	/* Wait for last issued instruction to complete. */
	retval = cortex_a_wait_instrcmpl(target, dscr, false);
	if (retval != ERROR_OK)
		return retval;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Check for faults and return early. */
	if (*dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE))
		return ERROR_OK; /* A data fault is not considered a system failure. */

	/* Wait until DTRTX is full (according to ARMv7-A/-R architecture manual
	 * section C8.4.3, checking InstrCmpl_l is not sufficient; one must also
	 * check TXfull_l). Most of the time this will be free because TXfull_l
	 * will be set immediately and cached in dscr. */
	retval = cortex_a_wait_dscr_bits(target, DSCR_DTRTX_FULL_LATCHED,
			DSCR_DTRTX_FULL_LATCHED, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Read the value transferred to DTRTX into the buffer. This is the last
	 * word. */
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRTX, &u32);
	if (retval != ERROR_OK)
		return retval;
	target_buffer_set_u32(target, buffer, u32);

	return ERROR_OK;
}

static int cortex_a_read_cpu_memory(struct target *target,
	uint32_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	/* Read memory through the CPU. */
	int retval, final_retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	uint32_t dscr, orig_dfar, orig_dfsr, fault_dscr, fault_dfar, fault_dfsr;

	LOG_DEBUG("Reading CPU memory address 0x%" PRIx32 " size %"  PRIu32 " count %"  PRIu32,
			  address, size, count);
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!count)
		return ERROR_OK;

	/* Clear any abort. */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DRCR, DRCR_CLEAR_EXCEPTIONS);
	if (retval != ERROR_OK)
		return retval;

	/* Read DSCR */
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, &dscr);
	if (retval != ERROR_OK)
		goto out;

	/* Mark R0 as dirty. */
	arm_reg_current(arm, 0)->dirty = true;

	/* Read DFAR and DFSR, as they will be modified in the event of a fault. */
	retval = cortex_a_read_dfar_dfsr(target, &orig_dfar, &orig_dfsr, &dscr);
	if (retval != ERROR_OK)
		goto out;

	/* Get the memory address into R0. */
	retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRRX, address);
	if (retval != ERROR_OK)
		goto out;
	retval = cortex_a_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0), &dscr);
	if (retval != ERROR_OK)
		goto out;

	if (size == 4 && (address % 4) == 0) {
		/* We are doing a word-aligned transfer, so use fast mode. */
		retval = cortex_a_read_cpu_memory_fast(target, count, buffer, &dscr);
	} else {
		/* Use slow path. Adjust size for aligned accesses */
		switch (address % 4) {
			case 1:
			case 3:
				count *= size;
				size = 1;
				break;
			case 2:
				if (size == 4) {
					count *= 2;
					size = 2;
				}
				break;
			case 0:
			default:
				break;
		}
		retval = cortex_a_read_cpu_memory_slow(target, size, count, buffer, &dscr);
	}

out:
	final_retval = retval;

	/* Switch to non-blocking mode if not already in that mode. */
	retval = cortex_a_set_dcc_mode(target, DSCR_EXT_DCC_NON_BLOCKING, &dscr);
	if (final_retval == ERROR_OK)
		final_retval = retval;

	/* Wait for last issued instruction to complete. */
	retval = cortex_a_wait_instrcmpl(target, &dscr, true);
	if (final_retval == ERROR_OK)
		final_retval = retval;

	/* If there were any sticky abort flags, clear them. */
	if (dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE)) {
		fault_dscr = dscr;
		mem_ap_write_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DRCR, DRCR_CLEAR_EXCEPTIONS);
		dscr &= ~(DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE);
	} else {
		fault_dscr = 0;
	}

	/* Handle synchronous data faults. */
	if (fault_dscr & DSCR_STICKY_ABORT_PRECISE) {
		if (final_retval == ERROR_OK) {
			/* Final return value will reflect cause of fault. */
			retval = cortex_a_read_dfar_dfsr(target, &fault_dfar, &fault_dfsr, &dscr);
			if (retval == ERROR_OK) {
				LOG_ERROR("data abort at 0x%08" PRIx32 ", dfsr = 0x%08" PRIx32, fault_dfar, fault_dfsr);
				final_retval = cortex_a_dfsr_to_error_code(fault_dfsr);
			} else
				final_retval = retval;
		}
		/* Fault destroyed DFAR/DFSR; restore them. */
		retval = cortex_a_write_dfar_dfsr(target, orig_dfar, orig_dfsr, &dscr);
		if (retval != ERROR_OK)
			LOG_ERROR("error restoring dfar/dfsr - dscr = 0x%08" PRIx32, dscr);
	}

	/* Handle asynchronous data faults. */
	if (fault_dscr & DSCR_STICKY_ABORT_IMPRECISE) {
		if (final_retval == ERROR_OK)
			/* No other error has been recorded so far, so keep this one. */
			final_retval = ERROR_TARGET_DATA_ABORT;
	}

	/* If the DCC is nonempty, clear it. */
	if (dscr & DSCR_DTRTX_FULL_LATCHED) {
		uint32_t dummy;
		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DTRTX, &dummy);
		if (final_retval == ERROR_OK)
			final_retval = retval;
	}
	if (dscr & DSCR_DTRRX_FULL_LATCHED) {
		retval = cortex_a_exec_opcode(target, ARMV4_5_MRC(14, 0, 1, 0, 5, 0), &dscr);
		if (final_retval == ERROR_OK)
			final_retval = retval;
	}

	/* Done. */
	return final_retval;
}


/*
 * Cortex-A Memory access
 *
 * This is same Cortex-M3 but we must also use the correct
 * ap number for every access.
 */

static int cortex_a_read_phys_memory(struct target *target,
	target_addr_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	int retval;

	if (!count || !buffer)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_DEBUG("Reading memory at real address " TARGET_ADDR_FMT "; size %" PRIu32 "; count %" PRIu32,
		address, size, count);

	/* read memory through the CPU */
	cortex_a_prep_memaccess(target, 1);
	retval = cortex_a_read_cpu_memory(target, address, size, count, buffer);
	cortex_a_post_memaccess(target, 1);

	return retval;
}

static int cortex_a_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;

	/* cortex_a handles unaligned memory access */
	LOG_DEBUG("Reading memory at address " TARGET_ADDR_FMT "; size %" PRIu32 "; count %" PRIu32,
		address, size, count);

	cortex_a_prep_memaccess(target, 0);
	retval = cortex_a_read_cpu_memory(target, address, size, count, buffer);
	cortex_a_post_memaccess(target, 0);

	return retval;
}

static int cortex_a_write_phys_memory(struct target *target,
	target_addr_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	int retval;

	if (!count || !buffer)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_DEBUG("Writing memory to real address " TARGET_ADDR_FMT "; size %" PRIu32 "; count %" PRIu32,
		address, size, count);

	/* write memory through the CPU */
	cortex_a_prep_memaccess(target, 1);
	retval = cortex_a_write_cpu_memory(target, address, size, count, buffer);
	cortex_a_post_memaccess(target, 1);

	return retval;
}

static int cortex_a_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;

	/* cortex_a handles unaligned memory access */
	LOG_DEBUG("Writing memory at address " TARGET_ADDR_FMT "; size %" PRIu32 "; count %" PRIu32,
		address, size, count);

	/* memory writes bypass the caches, must flush before writing */
	armv7a_cache_auto_flush_on_write(target, address, size * count);

	cortex_a_prep_memaccess(target, 0);
	retval = cortex_a_write_cpu_memory(target, address, size, count, buffer);
	cortex_a_post_memaccess(target, 0);
	return retval;
}

static int cortex_a_read_buffer(struct target *target, target_addr_t address,
				uint32_t count, uint8_t *buffer)
{
	uint32_t size;

	/* Align up to maximum 4 bytes. The loop condition makes sure the next pass
	 * will have something to do with the size we leave to it. */
	for (size = 1; size < 4 && count >= size * 2 + (address & size); size *= 2) {
		if (address & size) {
			int retval = target_read_memory(target, address, size, 1, buffer);
			if (retval != ERROR_OK)
				return retval;
			address += size;
			count -= size;
			buffer += size;
		}
	}

	/* Read the data with as large access size as possible. */
	for (; size > 0; size /= 2) {
		uint32_t aligned = count - count % size;
		if (aligned > 0) {
			int retval = target_read_memory(target, address, size, aligned / size, buffer);
			if (retval != ERROR_OK)
				return retval;
			address += aligned;
			count -= aligned;
			buffer += aligned;
		}
	}

	return ERROR_OK;
}

static int cortex_a_write_buffer(struct target *target, target_addr_t address,
				 uint32_t count, const uint8_t *buffer)
{
	uint32_t size;

	/* Align up to maximum 4 bytes. The loop condition makes sure the next pass
	 * will have something to do with the size we leave to it. */
	for (size = 1; size < 4 && count >= size * 2 + (address & size); size *= 2) {
		if (address & size) {
			int retval = target_write_memory(target, address, size, 1, buffer);
			if (retval != ERROR_OK)
				return retval;
			address += size;
			count -= size;
			buffer += size;
		}
	}

	/* Write the data with as large access size as possible. */
	for (; size > 0; size /= 2) {
		uint32_t aligned = count - count % size;
		if (aligned > 0) {
			int retval = target_write_memory(target, address, size, aligned / size, buffer);
			if (retval != ERROR_OK)
				return retval;
			address += aligned;
			count -= aligned;
			buffer += aligned;
		}
	}

	return ERROR_OK;
}

static int cortex_a_handle_target_request(void *priv)
{
	struct target *target = priv;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int retval;

	if (!target_was_examined(target))
		return ERROR_OK;
	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING) {
		uint32_t request;
		uint32_t dscr;
		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);

		/* check if we have data */
		int64_t then = timeval_ms();
		while ((dscr & DSCR_DTR_TX_FULL) && (retval == ERROR_OK)) {
			retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
					armv7a->debug_base + CPUDBG_DTRTX, &request);
			if (retval == ERROR_OK) {
				target_request(target, request);
				retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
						armv7a->debug_base + CPUDBG_DSCR, &dscr);
			}
			if (timeval_ms() > then + 1000) {
				LOG_ERROR("Timeout waiting for dtr tx full");
				return ERROR_FAIL;
			}
		}
	}

	return ERROR_OK;
}

/*
 * Cortex-A target information and configuration
 */

static int cortex_a_examine_first(struct target *target)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	struct adiv5_dap *swjdp = armv7a->arm.dap;

	int i;
	int retval = ERROR_OK;
	uint32_t didr, cpuid, dbg_osreg, dbg_idpfr1;

	/* Search for the APB-AP - it is needed for access to debug registers */
	retval = dap_find_ap(swjdp, AP_TYPE_APB_AP, &armv7a->debug_ap);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not find APB-AP for debug access");
		return retval;
	}

	retval = mem_ap_init(armv7a->debug_ap);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not initialize the APB-AP");
		return retval;
	}

	armv7a->debug_ap->memaccess_tck = 80;

	if (!target->dbgbase_set) {
		uint32_t dbgbase;
		/* Get ROM Table base */
		uint32_t apid;
		int32_t coreidx = target->coreid;
		LOG_DEBUG("%s's dbgbase is not set, trying to detect using the ROM table",
			  target->cmd_name);
		retval = dap_get_debugbase(armv7a->debug_ap, &dbgbase, &apid);
		if (retval != ERROR_OK)
			return retval;
		/* Lookup 0x15 -- Processor DAP */
		retval = dap_lookup_cs_component(armv7a->debug_ap, dbgbase, 0x15,
				&armv7a->debug_base, &coreidx);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't detect %s's dbgbase from the ROM table; you need to specify it explicitly.",
				  target->cmd_name);
			return retval;
		}
		LOG_DEBUG("Detected core %" PRId32 " dbgbase: %08" PRIx32,
			  target->coreid, armv7a->debug_base);
	} else
		armv7a->debug_base = target->dbgbase;

	if ((armv7a->debug_base & (1UL<<31)) == 0)
		LOG_WARNING("Debug base address for target %s has bit 31 set to 0. Access to debug registers will likely fail!\n"
			    "Please fix the target configuration.", target_name(target));

	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DIDR, &didr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "DIDR");
		return retval;
	}

	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_CPUID, &cpuid);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "CPUID");
		return retval;
	}

	LOG_DEBUG("didr = 0x%08" PRIx32, didr);
	LOG_DEBUG("cpuid = 0x%08" PRIx32, cpuid);

	cortex_a->didr = didr;
	cortex_a->cpuid = cpuid;

	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				    armv7a->debug_base + CPUDBG_PRSR, &dbg_osreg);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("target->coreid %" PRId32 " DBGPRSR  0x%" PRIx32, target->coreid, dbg_osreg);

	if ((dbg_osreg & PRSR_POWERUP_STATUS) == 0) {
		LOG_ERROR("target->coreid %" PRId32 " powered down!", target->coreid);
		target->state = TARGET_UNKNOWN; /* TARGET_NO_POWER? */
		return ERROR_TARGET_INIT_FAILED;
	}

	if (dbg_osreg & PRSR_STICKY_RESET_STATUS)
		LOG_DEBUG("target->coreid %" PRId32 " was reset!", target->coreid);

	/* Read DBGOSLSR and check if OSLK is implemented */
	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_OSLSR, &dbg_osreg);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("target->coreid %" PRId32 " DBGOSLSR 0x%" PRIx32, target->coreid, dbg_osreg);

	/* check if OS Lock is implemented */
	if ((dbg_osreg & OSLSR_OSLM) == OSLSR_OSLM0 || (dbg_osreg & OSLSR_OSLM) == OSLSR_OSLM1) {
		/* check if OS Lock is set */
		if (dbg_osreg & OSLSR_OSLK) {
			LOG_DEBUG("target->coreid %" PRId32 " OSLock set! Trying to unlock", target->coreid);

			retval = mem_ap_write_atomic_u32(armv7a->debug_ap,
							armv7a->debug_base + CPUDBG_OSLAR,
							0);
			if (retval == ERROR_OK)
				retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
							armv7a->debug_base + CPUDBG_OSLSR, &dbg_osreg);

			/* if we fail to access the register or cannot reset the OSLK bit, bail out */
			if (retval != ERROR_OK || (dbg_osreg & OSLSR_OSLK) != 0) {
				LOG_ERROR("target->coreid %" PRId32 " OSLock sticky, core not powered?",
						target->coreid);
				target->state = TARGET_UNKNOWN; /* TARGET_NO_POWER? */
				return ERROR_TARGET_INIT_FAILED;
			}
		}
	}

	retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				 armv7a->debug_base + CPUDBG_ID_PFR1, &dbg_idpfr1);
	if (retval != ERROR_OK)
		return retval;

	if (dbg_idpfr1 & 0x000000f0) {
		LOG_DEBUG("target->coreid %" PRId32 " has security extensions",
				target->coreid);
		armv7a->arm.core_type = ARM_CORE_TYPE_SEC_EXT;
	}
	if (dbg_idpfr1 & 0x0000f000) {
		LOG_DEBUG("target->coreid %" PRId32 " has virtualization extensions",
				target->coreid);
		/*
		 * overwrite and simplify the checks.
		 * virtualization extensions require implementation of security extension
		 */
		armv7a->arm.core_type = ARM_CORE_TYPE_VIRT_EXT;
	}

	/* Avoid recreating the registers cache */
	if (!target_was_examined(target)) {
		retval = cortex_a_dpm_setup(cortex_a, didr);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Setup Breakpoint Register Pairs */
	cortex_a->brp_num = ((didr >> 24) & 0x0F) + 1;
	cortex_a->brp_num_context = ((didr >> 20) & 0x0F) + 1;
	cortex_a->brp_num_available = cortex_a->brp_num;
	free(cortex_a->brp_list);
	cortex_a->brp_list = calloc(cortex_a->brp_num, sizeof(struct cortex_a_brp));
/*	cortex_a->brb_enabled = ????; */
	for (i = 0; i < cortex_a->brp_num; i++) {
		cortex_a->brp_list[i].used = 0;
		if (i < (cortex_a->brp_num-cortex_a->brp_num_context))
			cortex_a->brp_list[i].type = BRP_NORMAL;
		else
			cortex_a->brp_list[i].type = BRP_CONTEXT;
		cortex_a->brp_list[i].value = 0;
		cortex_a->brp_list[i].control = 0;
		cortex_a->brp_list[i].BRPn = i;
	}

	LOG_DEBUG("Configured %i hw breakpoints", cortex_a->brp_num);

	/* select debug_ap as default */
	swjdp->apsel = armv7a->debug_ap->ap_num;

	target_set_examined(target);
	return ERROR_OK;
}

static int cortex_a_examine(struct target *target)
{
	int retval = ERROR_OK;

	/* Reestablish communication after target reset */
	retval = cortex_a_examine_first(target);

	/* Configure core debug access */
	if (retval == ERROR_OK)
		retval = cortex_a_init_debug_access(target);

	return retval;
}

/*
 *	Cortex-A target creation and initialization
 */

static int cortex_a_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	/* examine_first() does a bunch of this */
	arm_semihosting_init(target);
	return ERROR_OK;
}

static int cortex_a_init_arch_info(struct target *target,
	struct cortex_a_common *cortex_a, struct adiv5_dap *dap)
{
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;

	/* Setup struct cortex_a_common */
	cortex_a->common_magic = CORTEX_A_COMMON_MAGIC;
	armv7a->arm.dap = dap;

	/* register arch-specific functions */
	armv7a->examine_debug_reason = NULL;

	armv7a->post_debug_entry = cortex_a_post_debug_entry;

	armv7a->pre_restore_context = NULL;

	armv7a->armv7a_mmu.read_physical_memory = cortex_a_read_phys_memory;


/*	arm7_9->handle_target_request = cortex_a_handle_target_request; */

	/* REVISIT v7a setup should be in a v7a-specific routine */
	armv7a_init_arch_info(target, armv7a);
	target_register_timer_callback(cortex_a_handle_target_request, 1,
		TARGET_TIMER_TYPE_PERIODIC, target);

	return ERROR_OK;
}

static int cortex_a_target_create(struct target *target, Jim_Interp *interp)
{
	struct cortex_a_common *cortex_a;
	struct adiv5_private_config *pc;

	if (target->private_config == NULL)
		return ERROR_FAIL;

	pc = (struct adiv5_private_config *)target->private_config;

	cortex_a = calloc(1, sizeof(struct cortex_a_common));
	if (cortex_a == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	cortex_a->common_magic = CORTEX_A_COMMON_MAGIC;
	cortex_a->armv7a_common.is_armv7r = false;
	cortex_a->armv7a_common.arm.arm_vfp_version = ARM_VFP_V3;

	return cortex_a_init_arch_info(target, cortex_a, pc->dap);
}

static int cortex_r4_target_create(struct target *target, Jim_Interp *interp)
{
	struct cortex_a_common *cortex_a;
	struct adiv5_private_config *pc;

	pc = (struct adiv5_private_config *)target->private_config;
	if (adiv5_verify_config(pc) != ERROR_OK)
		return ERROR_FAIL;

	cortex_a = calloc(1, sizeof(struct cortex_a_common));
	if (cortex_a == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	cortex_a->common_magic = CORTEX_A_COMMON_MAGIC;
	cortex_a->armv7a_common.is_armv7r = true;

	return cortex_a_init_arch_info(target, cortex_a, pc->dap);
}

static void cortex_a_deinit_target(struct target *target)
{
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = &cortex_a->armv7a_common;
	struct arm_dpm *dpm = &armv7a->dpm;
	uint32_t dscr;
	int retval;

	if (target_was_examined(target)) {
		/* Disable halt for breakpoint, watchpoint and vector catch */
		retval = mem_ap_read_atomic_u32(armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval == ERROR_OK)
			mem_ap_write_atomic_u32(armv7a->debug_ap,
					armv7a->debug_base + CPUDBG_DSCR,
					dscr & ~DSCR_HALT_DBG_MODE);
	}

	free(cortex_a->brp_list);
	arm_free_reg_cache(dpm->arm);
	free(dpm->dbp);
	free(dpm->dwp);
	free(target->private_config);
	free(cortex_a);
}

static int cortex_a_mmu(struct target *target, int *enabled)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_INVALID;
	}

	if (armv7a->is_armv7r)
		*enabled = 0;
	else
		*enabled = target_to_cortex_a(target)->armv7a_common.armv7a_mmu.mmu_enabled;

	return ERROR_OK;
}

static int cortex_a_virt2phys(struct target *target,
	target_addr_t virt, target_addr_t *phys)
{
	int retval;
	int mmu_enabled = 0;

	/*
	 * If the MMU was not enabled at debug entry, there is no
	 * way of knowing if there was ever a valid configuration
	 * for it and thus it's not safe to enable it. In this case,
	 * just return the virtual address as physical.
	 */
	cortex_a_mmu(target, &mmu_enabled);
	if (!mmu_enabled) {
		*phys = virt;
		return ERROR_OK;
	}

	/* mmu must be enable in order to get a correct translation */
	retval = cortex_a_mmu_modify(target, 1);
	if (retval != ERROR_OK)
		return retval;
	return armv7a_mmu_translate_va_pa(target, (uint32_t)virt,
						    phys, 1);
}

COMMAND_HANDLER(cortex_a_handle_cache_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	return armv7a_handle_cache_info_command(CMD,
			&armv7a->armv7a_mmu.armv7a_cache);
}


COMMAND_HANDLER(cortex_a_handle_dbginit_command)
{
	struct target *target = get_current_target(CMD_CTX);
	if (!target_was_examined(target)) {
		LOG_ERROR("target not examined yet");
		return ERROR_FAIL;
	}

	return cortex_a_init_debug_access(target);
}

COMMAND_HANDLER(handle_cortex_a_mask_interrupts_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);

	static const Jim_Nvp nvp_maskisr_modes[] = {
		{ .name = "off", .value = CORTEX_A_ISRMASK_OFF },
		{ .name = "on", .value = CORTEX_A_ISRMASK_ON },
		{ .name = NULL, .value = -1 },
	};
	const Jim_Nvp *n;

	if (CMD_ARGC > 0) {
		n = Jim_Nvp_name2value_simple(nvp_maskisr_modes, CMD_ARGV[0]);
		if (n->name == NULL) {
			LOG_ERROR("Unknown parameter: %s - should be off or on", CMD_ARGV[0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		cortex_a->isrmasking_mode = n->value;
	}

	n = Jim_Nvp_value2name_simple(nvp_maskisr_modes, cortex_a->isrmasking_mode);
	command_print(CMD, "cortex_a interrupt mask %s", n->name);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cortex_a_dacrfixup_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);

	static const Jim_Nvp nvp_dacrfixup_modes[] = {
		{ .name = "off", .value = CORTEX_A_DACRFIXUP_OFF },
		{ .name = "on", .value = CORTEX_A_DACRFIXUP_ON },
		{ .name = NULL, .value = -1 },
	};
	const Jim_Nvp *n;

	if (CMD_ARGC > 0) {
		n = Jim_Nvp_name2value_simple(nvp_dacrfixup_modes, CMD_ARGV[0]);
		if (n->name == NULL)
			return ERROR_COMMAND_SYNTAX_ERROR;
		cortex_a->dacrfixup_mode = n->value;

	}

	n = Jim_Nvp_value2name_simple(nvp_dacrfixup_modes, cortex_a->dacrfixup_mode);
	command_print(CMD, "cortex_a domain access control fixup %s", n->name);

	return ERROR_OK;
}

static const struct command_registration cortex_a_exec_command_handlers[] = {
	{
		.name = "cache_info",
		.handler = cortex_a_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.help = "display information about target caches",
		.usage = "",
	},
	{
		.name = "dbginit",
		.handler = cortex_a_handle_dbginit_command,
		.mode = COMMAND_EXEC,
		.help = "Initialize core debug",
		.usage = "",
	},
	{
		.name = "maskisr",
		.handler = handle_cortex_a_mask_interrupts_command,
		.mode = COMMAND_ANY,
		.help = "mask cortex_a interrupts",
		.usage = "['on'|'off']",
	},
	{
		.name = "dacrfixup",
		.handler = handle_cortex_a_dacrfixup_command,
		.mode = COMMAND_ANY,
		.help = "set domain access control (DACR) to all-manager "
			"on memory access",
		.usage = "['on'|'off']",
	},
	{
		.chain = armv7a_mmu_command_handlers,
	},
	{
		.chain = smp_command_handlers,
	},

	COMMAND_REGISTRATION_DONE
};
static const struct command_registration cortex_a_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.chain = armv7a_command_handlers,
	},
	{
		.name = "cortex_a",
		.mode = COMMAND_ANY,
		.help = "Cortex-A command group",
		.usage = "",
		.chain = cortex_a_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type cortexa_target = {
	.name = "cortex_a",
	.deprecated_name = "cortex_a8",

	.poll = cortex_a_poll,
	.arch_state = armv7a_arch_state,

	.halt = cortex_a_halt,
	.resume = cortex_a_resume,
	.step = cortex_a_step,

	.assert_reset = cortex_a_assert_reset,
	.deassert_reset = cortex_a_deassert_reset,

	/* REVISIT allow exporting VFP3 registers ... */
	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = cortex_a_read_memory,
	.write_memory = cortex_a_write_memory,

	.read_buffer = cortex_a_read_buffer,
	.write_buffer = cortex_a_write_buffer,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = cortex_a_add_breakpoint,
	.add_context_breakpoint = cortex_a_add_context_breakpoint,
	.add_hybrid_breakpoint = cortex_a_add_hybrid_breakpoint,
	.remove_breakpoint = cortex_a_remove_breakpoint,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.commands = cortex_a_command_handlers,
	.target_create = cortex_a_target_create,
	.target_jim_configure = adiv5_jim_configure,
	.init_target = cortex_a_init_target,
	.examine = cortex_a_examine,
	.deinit_target = cortex_a_deinit_target,

	.read_phys_memory = cortex_a_read_phys_memory,
	.write_phys_memory = cortex_a_write_phys_memory,
	.mmu = cortex_a_mmu,
	.virt2phys = cortex_a_virt2phys,
};

static const struct command_registration cortex_r4_exec_command_handlers[] = {
	{
		.name = "dbginit",
		.handler = cortex_a_handle_dbginit_command,
		.mode = COMMAND_EXEC,
		.help = "Initialize core debug",
		.usage = "",
	},
	{
		.name = "maskisr",
		.handler = handle_cortex_a_mask_interrupts_command,
		.mode = COMMAND_EXEC,
		.help = "mask cortex_r4 interrupts",
		.usage = "['on'|'off']",
	},

	COMMAND_REGISTRATION_DONE
};
static const struct command_registration cortex_r4_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.name = "cortex_r4",
		.mode = COMMAND_ANY,
		.help = "Cortex-R4 command group",
		.usage = "",
		.chain = cortex_r4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type cortexr4_target = {
	.name = "cortex_r4",

	.poll = cortex_a_poll,
	.arch_state = armv7a_arch_state,

	.halt = cortex_a_halt,
	.resume = cortex_a_resume,
	.step = cortex_a_step,

	.assert_reset = cortex_a_assert_reset,
	.deassert_reset = cortex_a_deassert_reset,

	/* REVISIT allow exporting VFP3 registers ... */
	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = cortex_a_read_phys_memory,
	.write_memory = cortex_a_write_phys_memory,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = cortex_a_add_breakpoint,
	.add_context_breakpoint = cortex_a_add_context_breakpoint,
	.add_hybrid_breakpoint = cortex_a_add_hybrid_breakpoint,
	.remove_breakpoint = cortex_a_remove_breakpoint,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.commands = cortex_r4_command_handlers,
	.target_create = cortex_r4_target_create,
	.target_jim_configure = adiv5_jim_configure,
	.init_target = cortex_a_init_target,
	.examine = cortex_a_examine,
	.deinit_target = cortex_a_deinit_target,
};
