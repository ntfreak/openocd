/*
 * Copyright (C) 2009 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "armv8.h"
#include "armv8_dpm.h"
#include <jtag/jtag.h>
#include "register.h"
#include "breakpoints.h"
#include "target_type.h"
#include "armv8_opcodes.h"

#include "helper/time_support.h"

/* T32 ITR format */
#define T32_FMTITR(instr) (((instr & 0x0000FFFF) << 16) | ((instr & 0xFFFF0000) >> 16))

/**
 * @file
 * Implements various ARM DPM operations using architectural debug registers.
 * These routines layer over core-specific communication methods to cope with
 * implementation differences between cores like ARM1136 and Cortex-A8.
 *
 * The "Debug Programmers' Model" (DPM) for ARMv6 and ARMv7 is defined by
 * Part C (Debug Architecture) of the ARM Architecture Reference Manual,
 * ARMv7-A and ARMv7-R edition (ARM DDI 0406B).  In OpenOCD, DPM operations
 * are abstracted through internal programming interfaces to share code and
 * to minimize needless differences in debug behavior between cores.
 */

/**
 * Get core state from EDSCR, without necessity to retrieve CPSR
 */
enum arm_state armv8_dpm_get_core_state(struct arm_dpm *dpm)
{
	int el = (dpm->dscr >> 8) & 0x3;
	int rw = (dpm->dscr >> 10) & 0xF;

	dpm->last_el = el;

	/* In Debug state, each bit gives the current Execution state of each EL */
	if ((rw >> el) & 0b1)
		return ARM_STATE_AARCH64;

	return ARM_STATE_ARM;
}

/*----------------------------------------------------------------------*/

static int dpmv8_write_dcc(struct armv8_common *armv8, uint32_t data)
{
	return mem_ap_write_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DTRRX, data);
}

static int dpmv8_write_dcc_64(struct armv8_common *armv8, uint64_t data)
{
	int ret;
	ret = mem_ap_write_u32(armv8->debug_ap,
			       armv8->debug_base + CPUV8_DBG_DTRRX, data);
	if (ret == ERROR_OK)
		ret = mem_ap_write_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DTRTX, data >> 32);
	return ret;
}

static int dpmv8_read_dcc(struct armv8_common *armv8, uint32_t *data,
	uint32_t *dscr_p)
{
	uint32_t dscr = DSCR_ITE;
	int retval;

	if (dscr_p)
		dscr = *dscr_p;

	/* Wait for DTRRXfull */
	long long then = timeval_ms();
	while ((dscr & DSCR_DTR_TX_FULL) == 0) {
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for read dcc");
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
					    armv8->debug_base + CPUV8_DBG_DTRTX,
					    data);
	if (retval != ERROR_OK)
		return retval;

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

static int dpmv8_read_dcc_64(struct armv8_common *armv8, uint64_t *data,
	uint32_t *dscr_p)
{
	uint32_t dscr = DSCR_ITE;
	uint32_t higher;
	int retval;

	if (dscr_p)
		dscr = *dscr_p;

	/* Wait for DTRRXfull */
	long long then = timeval_ms();
	while ((dscr & DSCR_DTR_TX_FULL) == 0) {
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for DTR_TX_FULL, dscr = 0x%08" PRIx32, dscr);
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
					    armv8->debug_base + CPUV8_DBG_DTRTX,
					    (uint32_t *)data);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
					    armv8->debug_base + CPUV8_DBG_DTRRX,
					    &higher);
	if (retval != ERROR_OK)
		return retval;

	*data = *(uint32_t *)data | (uint64_t)higher << 32;

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

static int dpmv8_dpm_prepare(struct arm_dpm *dpm)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	uint32_t dscr;
	int retval;

	/* set up invariant:  ITE is set after ever DPM operation */
	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		if ((dscr & DSCR_ITE) != 0)
			break;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for dpm prepare");
			return ERROR_FAIL;
		}
	}

	/* update the stored copy of dscr */
	dpm->dscr = dscr;

	/* this "should never happen" ... */
	if (dscr & DSCR_DTR_RX_FULL) {
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX */
		retval = mem_ap_read_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DTRRX, &dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int dpmv8_dpm_finish(struct arm_dpm *dpm)
{
	/* REVISIT what could be done here? */
	return ERROR_OK;
}

static int dpmv8_exec_opcode(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *p_dscr)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	uint32_t dscr = dpm->dscr;
	int retval;

	if (p_dscr)
		dscr = *p_dscr;

	/* Wait for InstrCompl bit to be set */
	long long then = timeval_ms();
	while ((dscr & DSCR_ITE) == 0) {
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register, opcode = 0x%08" PRIx32, opcode);
			return retval;
		}
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for aarch64_exec_opcode");
			return ERROR_FAIL;
		}
	}

	if (armv8_dpm_get_core_state(dpm) != ARM_STATE_AARCH64)
		opcode = T32_FMTITR(opcode);

	retval = mem_ap_write_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_ITR, opcode);
	if (retval != ERROR_OK)
		return retval;

	then = timeval_ms();
	do {
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register");
			return retval;
		}
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for aarch64_exec_opcode");
			return ERROR_FAIL;
		}
	} while ((dscr & DSCR_ITE) == 0);	/* Wait for InstrCompl bit to be set */

	/* update dscr and el after each command execution */
	dpm->dscr = dscr;
	if (dpm->last_el != ((dscr >> 8) & 3))
		LOG_DEBUG("EL %i -> %" PRIu32, dpm->last_el, (dscr >> 8) & 3);
	dpm->last_el = (dscr >> 8) & 3;

	if (dscr & DSCR_ERR) {
		LOG_ERROR("Opcode 0x%08" PRIx32 ", DSCR.ERR=1, DSCR.EL=%i", opcode, dpm->last_el);
		armv8_dpm_handle_exception(dpm, true);
		retval = ERROR_FAIL;
	}

	if (p_dscr)
		*p_dscr = dscr;

	return retval;
}

static int dpmv8_instr_execute(struct arm_dpm *dpm, uint32_t opcode)
{
	return dpmv8_exec_opcode(dpm, opcode, NULL);
}

static int dpmv8_instr_write_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval;

	retval = dpmv8_write_dcc(armv8, data);
	if (retval != ERROR_OK)
		return retval;

	return dpmv8_exec_opcode(dpm, opcode, 0);
}

static int dpmv8_instr_write_data_dcc_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval;

	retval = dpmv8_write_dcc_64(armv8, data);
	if (retval != ERROR_OK)
		return retval;

	return dpmv8_exec_opcode(dpm, opcode, 0);
}

static int dpmv8_instr_write_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	uint32_t dscr = DSCR_ITE;
	int retval;

	retval = dpmv8_write_dcc(armv8, data);
	if (retval != ERROR_OK)
		return retval;

	retval = dpmv8_exec_opcode(dpm, armv8_opcode(armv8, READ_REG_DTRRX), &dscr);
	if (retval != ERROR_OK)
		return retval;

	/* then the opcode, taking data from R0 */
	return dpmv8_exec_opcode(dpm, opcode, &dscr);
}

static int dpmv8_instr_write_data_r0_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval;

	if (dpm->arm->core_state != ARM_STATE_AARCH64)
		return dpmv8_instr_write_data_r0(dpm, opcode, data);

	/* transfer data from DCC to R0 */
	retval = dpmv8_write_dcc_64(armv8, data);
	if (retval == ERROR_OK)
		retval = dpmv8_exec_opcode(dpm, ARMV8_MRS(SYSTEM_DBG_DBGDTR_EL0, 0), &dpm->dscr);

	/* then the opcode, taking data from R0 */
	if (retval == ERROR_OK)
		retval = dpmv8_exec_opcode(dpm, opcode, &dpm->dscr);

	return retval;
}

static int dpmv8_instr_cpsr_sync(struct arm_dpm *dpm)
{
	int retval;
	struct armv8_common *armv8 = dpm->arm->arch_info;

	/* "Prefetch flush" after modifying execution status in CPSR */
	retval = dpmv8_exec_opcode(dpm, armv8_opcode(armv8, ARMV8_OPC_DSB_SY), &dpm->dscr);
	if (retval == ERROR_OK)
		dpmv8_exec_opcode(dpm, armv8_opcode(armv8, ARMV8_OPC_ISB_SY), &dpm->dscr);
	return retval;
}

static int dpmv8_instr_read_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval;

	/* the opcode, writing data to DCC */
	retval = dpmv8_exec_opcode(dpm, opcode, &dpm->dscr);
	if (retval != ERROR_OK)
		return retval;

	return dpmv8_read_dcc(armv8, data, &dpm->dscr);
}

static int dpmv8_instr_read_data_dcc_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t *data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval;

	/* the opcode, writing data to DCC */
	retval = dpmv8_exec_opcode(dpm, opcode, &dpm->dscr);
	if (retval != ERROR_OK)
		return retval;

	return dpmv8_read_dcc_64(armv8, data, &dpm->dscr);
}

static int dpmv8_instr_read_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval;

	/* the opcode, writing data to R0 */
	retval = dpmv8_exec_opcode(dpm, opcode, &dpm->dscr);
	if (retval != ERROR_OK)
		return retval;

	/* write R0 to DCC */
	retval = dpmv8_exec_opcode(dpm, armv8_opcode(armv8, WRITE_REG_DTRTX), &dpm->dscr);
	if (retval != ERROR_OK)
		return retval;

	return dpmv8_read_dcc(armv8, data, &dpm->dscr);
}

static int dpmv8_instr_read_data_r0_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t *data)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval;

	if (dpm->arm->core_state != ARM_STATE_AARCH64) {
		uint32_t tmp;
		retval = dpmv8_instr_read_data_r0(dpm, opcode, &tmp);
		if (retval == ERROR_OK)
			*data = tmp;
		return retval;
	}

	/* the opcode, writing data to R0 */
	retval = dpmv8_exec_opcode(dpm, opcode, &dpm->dscr);
	if (retval != ERROR_OK)
		return retval;

	/* write R0 to DCC */
	retval = dpmv8_exec_opcode(dpm, ARMV8_MSR_GP(SYSTEM_DBG_DBGDTR_EL0, 0), &dpm->dscr);
	if (retval != ERROR_OK)
		return retval;

	return dpmv8_read_dcc_64(armv8, data, &dpm->dscr);
}

#if 0
static int dpmv8_bpwp_enable(struct arm_dpm *dpm, unsigned index_t,
	target_addr_t addr, uint32_t control)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	uint32_t vr = armv8->debug_base;
	uint32_t cr = armv8->debug_base;
	int retval;

	switch (index_t) {
		case 0 ... 15:	/* breakpoints */
			vr += CPUV8_DBG_BVR_BASE;
			cr += CPUV8_DBG_BCR_BASE;
			break;
		case 16 ... 31:	/* watchpoints */
			vr += CPUV8_DBG_WVR_BASE;
			cr += CPUV8_DBG_WCR_BASE;
			index_t -= 16;
			break;
		default:
			return ERROR_FAIL;
	}
	vr += 16 * index_t;
	cr += 16 * index_t;

	LOG_DEBUG("A8: bpwp enable, vr %08x cr %08x",
		(unsigned) vr, (unsigned) cr);

	retval = mem_ap_write_atomic_u32(armv8->debug_ap, vr, addr);
	if (retval != ERROR_OK)
		return retval;
	return mem_ap_write_atomic_u32(armv8->debug_ap, cr, control);
}
#endif

static int dpmv8_bpwp_disable(struct arm_dpm *dpm, unsigned index_t)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	uint32_t cr;

	switch (index_t) {
		case 0 ... 15:
			cr = armv8->debug_base + CPUV8_DBG_BCR_BASE;
			break;
		case 16 ... 31:
			cr = armv8->debug_base + CPUV8_DBG_WCR_BASE;
			index_t -= 16;
			break;
		default:
			return ERROR_FAIL;
	}
	cr += 16 * index_t;

	LOG_DEBUG("A: bpwp disable, cr %08x", (unsigned) cr);

	/* clear control register */
	return mem_ap_write_atomic_u32(armv8->debug_ap, cr, 0);
}

/*
 * Coprocessor support
 */

/* Read coprocessor */
static int dpmv8_mrc(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm,
	uint32_t *value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("MRC p%d, %d, r0, c%d, c%d, %d", cpnum,
		(int) op1, (int) CRn,
		(int) CRm, (int) op2);

	/* read coprocessor register into R0; return via DCC */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(cpnum, op1, 0, CRn, CRm, op2),
			value);

	/* (void) */ dpm->finish(dpm);
	return retval;
}

static int dpmv8_mcr(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm,
	uint32_t value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("MCR p%d, %d, r0, c%d, c%d, %d", cpnum,
		(int) op1, (int) CRn,
		(int) CRm, (int) op2);

	/* read DCC into r0; then write coprocessor register from R0 */
	retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_MCR(cpnum, op1, 0, CRn, CRm, op2),
			value);

	/* (void) */ dpm->finish(dpm);
	return retval;
}

/*----------------------------------------------------------------------*/

/*
 * Register access utilities
 */

int armv8_dpm_modeswitch(struct arm_dpm *dpm, enum arm_mode mode)
{
	struct armv8_common *armv8 = (struct armv8_common *)dpm->arm->arch_info;
	int retval = ERROR_OK;
	unsigned int target_el;
	enum arm_state core_state;
	uint32_t cpsr;

	/* restore previous mode */
	if (mode == ARM_MODE_ANY) {
		cpsr = buf_get_u32(dpm->arm->cpsr->value, 0, 32);

		LOG_DEBUG("restoring mode, cpsr = 0x%08"PRIx32, cpsr);

	} else {
		LOG_DEBUG("setting mode 0x%x", mode);
		cpsr = mode;
	}

	switch (cpsr & 0x1f) {
	/* aarch32 modes */
	case ARM_MODE_USR:
		target_el = 0;
		break;
	case ARM_MODE_SVC:
	case ARM_MODE_ABT:
	case ARM_MODE_IRQ:
	case ARM_MODE_FIQ:
	case ARM_MODE_SYS:
		target_el = 1;
		break;
	/*
	 * TODO: handle ARM_MODE_HYP
	 * case ARM_MODE_HYP:
	 *      target_el = 2;
	 *      break;
	 */
	case ARM_MODE_MON:
		target_el = 3;
		break;
	/* aarch64 modes */
	default:
		target_el = (cpsr >> 2) & 3;
	}

	if (target_el > SYSTEM_CUREL_EL3) {
		LOG_ERROR("%s: Invalid target exception level %i", __func__, target_el);
		return ERROR_FAIL;
	}

	LOG_DEBUG("target_el = %i, last_el = %i", target_el, dpm->last_el);
	if (target_el > dpm->last_el) {
		retval = dpm->instr_execute(dpm,
				armv8_opcode(armv8, ARMV8_OPC_DCPS) | target_el);

		/* DCPS clobbers registers just like an exception taken */
		armv8_dpm_handle_exception(dpm, false);
	} else {
		core_state = armv8_dpm_get_core_state(dpm);
		if (core_state != ARM_STATE_AARCH64) {
			/* cannot do DRPS/ERET when already in EL0 */
			if (dpm->last_el != 0) {
				/* load SPSR with the desired mode and execute DRPS */
				LOG_DEBUG("SPSR = 0x%08"PRIx32, cpsr);
				retval = dpm->instr_write_data_r0(dpm,
						ARMV8_MSR_GP_xPSR_T1(1, 0, 15), cpsr);
				if (retval == ERROR_OK)
					retval = dpm->instr_execute(dpm, armv8_opcode(armv8, ARMV8_OPC_DRPS));
			}
		} else {
			/*
			 * need to execute multiple DRPS instructions until target_el
			 * is reached
			 */
			while (retval == ERROR_OK && dpm->last_el != target_el) {
				unsigned int cur_el = dpm->last_el;
				retval = dpm->instr_execute(dpm, armv8_opcode(armv8, ARMV8_OPC_DRPS));
				if (cur_el == dpm->last_el) {
					LOG_INFO("Cannot reach EL %i, SPSR corrupted?", target_el);
					break;
				}
			}
		}

		/* On executing DRPS, DSPSR and DLR become UNKNOWN, mark them as dirty */
		dpm->arm->cpsr->dirty = true;
		dpm->arm->pc->dirty = true;

		/*
		 * re-evaluate the core state, we might be in Aarch32 state now
		 * we rely on dpm->dscr being up-to-date
		 */
		core_state = armv8_dpm_get_core_state(dpm);
		armv8_select_opcodes(armv8, core_state == ARM_STATE_AARCH64);
		armv8_select_reg_access(armv8, core_state == ARM_STATE_AARCH64);
	}

	return retval;
}

/*
 * Common register read, relies on armv8_select_reg_access() having been called.
 */
static int dpmv8_read_reg(struct arm_dpm *dpm, struct reg *r, unsigned regnum)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval = ERROR_FAIL;

	if (r->size <= 64) {
		uint64_t value_64;
		retval = armv8->read_reg_u64(armv8, regnum, &value_64);

		if (retval == ERROR_OK) {
			r->valid = true;
			r->dirty = false;
			buf_set_u64(r->value, 0, r->size, value_64);
			if (r->size == 64)
				LOG_DEBUG("READ: %s, %16.8llx", r->name, (unsigned long long) value_64);
			else
				LOG_DEBUG("READ: %s, %8.8x", r->name, (unsigned int) value_64);
		}
	} else if (r->size <= 128) {
		uint64_t lvalue = 0, hvalue = 0;
		retval = armv8->read_reg_u128(armv8, regnum, &lvalue, &hvalue);

		if (retval == ERROR_OK) {
			r->valid = true;
			r->dirty = false;

			buf_set_u64(r->value, 0, 64, lvalue);
			buf_set_u64(r->value + 8, 0, r->size - 64, hvalue);

			LOG_DEBUG("READ: %s, lvalue=%16.8llx", r->name, (unsigned long long) lvalue);
			LOG_DEBUG("READ: %s, hvalue=%16.8llx", r->name, (unsigned long long) hvalue);
		}
	}

	if (retval != ERROR_OK)
		LOG_ERROR("Failed to read %s register", r->name);

	return retval;
}

/*
 * Common register write, relies on armv8_select_reg_access() having been called.
 */
static int dpmv8_write_reg(struct arm_dpm *dpm, struct reg *r, unsigned regnum)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	int retval = ERROR_FAIL;

	if (r->size <= 64) {
		uint64_t value_64;

		value_64 = buf_get_u64(r->value, 0, r->size);
		retval = armv8->write_reg_u64(armv8, regnum, value_64);

		if (retval == ERROR_OK) {
			r->dirty = false;
			if (r->size == 64)
				LOG_DEBUG("WRITE: %s, %16.8llx", r->name, (unsigned long long)value_64);
			else
				LOG_DEBUG("WRITE: %s, %8.8x", r->name, (unsigned int)value_64);
		}
	} else if (r->size <= 128) {
		uint64_t lvalue, hvalue;

		lvalue = buf_get_u64(r->value, 0, 64);
		hvalue = buf_get_u64(r->value + 8, 0, r->size - 64);
		retval = armv8->write_reg_u128(armv8, regnum, lvalue, hvalue);

		if (retval == ERROR_OK) {
			r->dirty = false;

			LOG_DEBUG("WRITE: %s, lvalue=%16.8llx", r->name, (unsigned long long) lvalue);
			LOG_DEBUG("WRITE: %s, hvalue=%16.8llx", r->name, (unsigned long long) hvalue);
		}
	}

	if (retval != ERROR_OK)
		LOG_ERROR("Failed to write %s register", r->name);

	return retval;
}

/**
 * Read basic registers of the current context:  R0 to R15, and CPSR;
 * sets the core mode (such as USR or IRQ) and state (such as ARM or Thumb).
 * In normal operation this is called on entry to halting debug state,
 * possibly after some other operations supporting restore of debug state
 * or making sure the CPU is fully idle (drain write buffer, etc).
 */
int armv8_dpm_read_current_registers(struct arm_dpm *dpm)
{
	struct arm *arm = dpm->arm;
	struct armv8_common *armv8 = (struct armv8_common *)arm->arch_info;
	struct reg_cache *cache;
	struct reg *r;
	uint32_t cpsr;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	cache = arm->core_cache;

	/* read R0 first (it's used for scratch), then CPSR */
	r = cache->reg_list + ARMV8_R0;
	if (!r->valid) {
		retval = dpmv8_read_reg(dpm, r, ARMV8_R0);
		if (retval != ERROR_OK)
			goto fail;
	}
	r->dirty = true;

	/* read R1, too, it will be clobbered during memory access */
	r = cache->reg_list + ARMV8_R1;
	if (!r->valid) {
		retval = dpmv8_read_reg(dpm, r, ARMV8_R1);
		if (retval != ERROR_OK)
			goto fail;
	}

	/* read cpsr to r0 and get it back */
	retval = dpm->instr_read_data_r0(dpm,
			armv8_opcode(armv8, READ_REG_DSPSR), &cpsr);
	if (retval != ERROR_OK)
		goto fail;

	/* update core mode and state */
	armv8_set_cpsr(arm, cpsr);

	for (unsigned int i = ARMV8_PC; i < cache->num_regs ; i++) {
		struct arm_reg *arm_reg;

		r = armv8_reg_current(arm, i);
		if (r->valid)
			continue;

		/* Skip reading FP-SIMD registers */
		if (r->number >= ARMV8_V0 && r->number <= ARMV8_FPCR)
			continue;

		/*
		 * Only read registers that are available from the
		 * current EL (or core mode).
		 */
		arm_reg = r->arch_info;
		if (arm_reg->mode != ARM_MODE_ANY &&
				dpm->last_el != armv8_curel_from_core_mode(arm_reg->mode))
			continue;

		/* Special case: ARM_MODE_SYS has no SPSR at EL1 */
		if (r->number == ARMV8_SPSR_EL1 && arm->core_mode == ARM_MODE_SYS)
			continue;

		retval = dpmv8_read_reg(dpm, r, i);
		if (retval != ERROR_OK)
			goto fail;

	}

fail:
	dpm->finish(dpm);
	return retval;
}

/* Avoid needless I/O ... leave breakpoints and watchpoints alone
 * unless they're removed, or need updating because of single-stepping
 * or running debugger code.
 */
static int dpmv8_maybe_update_bpwp(struct arm_dpm *dpm, bool bpwp,
	struct dpm_bpwp *xp, int *set_p)
{
	int retval = ERROR_OK;
	bool disable;

	if (!set_p) {
		if (!xp->dirty)
			goto done;
		xp->dirty = false;
		/* removed or startup; we must disable it */
		disable = true;
	} else if (bpwp) {
		if (!xp->dirty)
			goto done;
		/* disabled, but we must set it */
		xp->dirty = disable = false;
		*set_p = true;
	} else {
		if (!*set_p)
			goto done;
		/* set, but we must temporarily disable it */
		xp->dirty = disable = true;
		*set_p = false;
	}

	if (disable)
		retval = dpm->bpwp_disable(dpm, xp->number);
	else
		retval = dpm->bpwp_enable(dpm, xp->number,
				xp->address, xp->control);

	if (retval != ERROR_OK)
		LOG_ERROR("%s: can't %s HW %spoint %d",
			disable ? "disable" : "enable",
			target_name(dpm->arm->target),
			(xp->number < 16) ? "break" : "watch",
			xp->number & 0xf);
done:
	return retval;
}

static int dpmv8_add_breakpoint(struct target *target, struct breakpoint *bp);

/**
 * Writes all modified core registers for all processor modes.  In normal
 * operation this is called on exit from halting debug state.
 *
 * @param dpm: represents the processor
 * @param bpwp: true ensures breakpoints and watchpoints are set,
 *	false ensures they are cleared
 */
int armv8_dpm_write_dirty_registers(struct arm_dpm *dpm, bool bpwp)
{
	struct arm *arm = dpm->arm;
	struct reg_cache *cache = arm->core_cache;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* If we're managing hardware breakpoints for this core, enable
	 * or disable them as requested.
	 *
	 * REVISIT We don't yet manage them for ANY cores.  Eventually
	 * we should be able to assume we handle them; but until then,
	 * cope with the hand-crafted breakpoint code.
	 */
	if (arm->target->type->add_breakpoint == dpmv8_add_breakpoint) {
		for (unsigned i = 0; i < dpm->nbp; i++) {
			struct dpm_bp *dbp = dpm->dbp + i;
			struct breakpoint *bp = dbp->bp;

			retval = dpmv8_maybe_update_bpwp(dpm, bpwp, &dbp->bpwp,
					bp ? &bp->set : NULL);
			if (retval != ERROR_OK)
				goto done;
		}
	}

	/* enable/disable watchpoints */
	for (unsigned i = 0; i < dpm->nwp; i++) {
		struct dpm_wp *dwp = dpm->dwp + i;
		struct watchpoint *wp = dwp->wp;

		retval = dpmv8_maybe_update_bpwp(dpm, bpwp, &dwp->bpwp,
				wp ? &wp->set : NULL);
		if (retval != ERROR_OK)
			goto done;
	}

	/* NOTE:  writes to breakpoint and watchpoint registers might
	 * be queued, and need (efficient/batched) flushing later.
	 */

	/* Restore original core mode and state */
	retval = armv8_dpm_modeswitch(dpm, ARM_MODE_ANY);
	if (retval != ERROR_OK)
		goto done;

	/* check everything except our scratch register R0 */
	for (unsigned i = 1; i < cache->num_regs; i++) {
		struct arm_reg *r;

		/* skip PC and CPSR */
		if (i == ARMV8_PC || i == ARMV8_xPSR)
			continue;
		/* skip invalid */
		if (!cache->reg_list[i].valid)
			continue;
		/* skip non-dirty */
		if (!cache->reg_list[i].dirty)
			continue;

		/* skip all registers not on the current EL */
		r = cache->reg_list[i].arch_info;
		if (r->mode != ARM_MODE_ANY &&
				dpm->last_el != armv8_curel_from_core_mode(r->mode))
			continue;

		retval = dpmv8_write_reg(dpm, &cache->reg_list[i], i);
		if (retval != ERROR_OK)
			break;
	}

	/* flush CPSR and PC */
	if (retval == ERROR_OK)
		retval = dpmv8_write_reg(dpm, &cache->reg_list[ARMV8_xPSR], ARMV8_xPSR);
	if (retval == ERROR_OK)
		retval = dpmv8_write_reg(dpm, &cache->reg_list[ARMV8_PC], ARMV8_PC);
	/* flush R0 -- it's *very* dirty by now */
	if (retval == ERROR_OK)
		retval = dpmv8_write_reg(dpm, &cache->reg_list[0], 0);
	if (retval == ERROR_OK)
		dpm->instr_cpsr_sync(dpm);
done:
	dpm->finish(dpm);
	return retval;
}

/*
 * Standard ARM register accessors ... there are three methods
 * in "struct arm", to support individual read/write and bulk read
 * of registers.
 */

static int armv8_dpm_read_core_reg(struct target *target, struct reg *r,
	int regnum, enum arm_mode mode)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = target_to_arm(target)->dpm;
	int retval;
	int max = arm->core_cache->num_regs;

	if (regnum < 0 || regnum >= max)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/*
	 * REVISIT what happens if we try to read SPSR in a core mode
	 * which has no such register?
	 */
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	retval = dpmv8_read_reg(dpm, r, regnum);
	if (retval != ERROR_OK)
		goto fail;

fail:
	/* (void) */ dpm->finish(dpm);
	return retval;
}

static int armv8_dpm_write_core_reg(struct target *target, struct reg *r,
	int regnum, enum arm_mode mode, uint8_t *value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = target_to_arm(target)->dpm;
	int retval;
	int max = arm->core_cache->num_regs;

	if (regnum < 0 || regnum > max)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* REVISIT what happens if we try to write SPSR in a core mode
	 * which has no such register?
	 */

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	retval = dpmv8_write_reg(dpm, r, regnum);

	/* always clean up, regardless of error */
	dpm->finish(dpm);

	return retval;
}

static int armv8_dpm_full_context(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	struct reg_cache *cache = arm->core_cache;
	int retval;
	bool did_read;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	do {
		enum arm_mode mode = ARM_MODE_ANY;

		did_read = false;

		/* We "know" arm_dpm_read_current_registers() was called so
		 * the unmapped registers (R0..R7, PC, AND CPSR) and some
		 * view of R8..R14 are current.  We also "know" oddities of
		 * register mapping: special cases for R8..R12 and SPSR.
		 *
		 * Pick some mode with unread registers and read them all.
		 * Repeat until done.
		 */
		for (unsigned i = 0; i < cache->num_regs; i++) {
			struct arm_reg *r;

			if (cache->reg_list[i].valid)
				continue;
			r = cache->reg_list[i].arch_info;

			/* may need to pick a mode and set CPSR */
			if (!did_read) {
				did_read = true;
				mode = r->mode;

				/* For regular (ARM_MODE_ANY) R8..R12
				 * in case we've entered debug state
				 * in FIQ mode we need to patch mode.
				 */
				if (mode != ARM_MODE_ANY)
					retval = armv8_dpm_modeswitch(dpm, mode);
				else
					retval = armv8_dpm_modeswitch(dpm, ARM_MODE_USR);

				if (retval != ERROR_OK)
					goto done;
			}
			if (r->mode != mode)
				continue;

			/* CPSR was read, so "R16" must mean SPSR */
			retval = dpmv8_read_reg(dpm,
					&cache->reg_list[i],
					(r->num == 16) ? 17 : r->num);
			if (retval != ERROR_OK)
				goto done;
		}

	} while (did_read);

	retval = armv8_dpm_modeswitch(dpm, ARM_MODE_ANY);
	/* (void) */ dpm->finish(dpm);
done:
	return retval;
}


/*----------------------------------------------------------------------*/

/*
 * Breakpoint and Watchpoint support.
 *
 * Hardware {break,watch}points are usually left active, to minimize
 * debug entry/exit costs.  When they are set or cleared, it's done in
 * batches.  Also, DPM-conformant hardware can update debug registers
 * regardless of whether the CPU is running or halted ... though that
 * fact isn't currently leveraged.
 */

static int dpmv8_bpwp_setup(struct arm_dpm *dpm, struct dpm_bpwp *xp,
	uint32_t addr, uint32_t length)
{
	uint32_t control;

	control = (1 << 0)	/* enable */
		| (3 << 1);	/* both user and privileged access */

	/* Match 1, 2, or all 4 byte addresses in this word.
	 *
	 * FIXME:  v7 hardware allows lengths up to 2 GB for BP and WP.
	 * Support larger length, when addr is suitably aligned.  In
	 * particular, allow watchpoints on 8 byte "double" values.
	 *
	 * REVISIT allow watchpoints on unaligned 2-bit values; and on
	 * v7 hardware, unaligned 4-byte ones too.
	 */
	switch (length) {
		case 1:
			control |= (1 << (addr & 3)) << 5;
			break;
		case 2:
			/* require 2-byte alignment */
			if (!(addr & 1)) {
				control |= (3 << (addr & 2)) << 5;
				break;
			}
		/* FALL THROUGH */
		case 4:
			/* require 4-byte alignment */
			if (!(addr & 3)) {
				control |= 0xf << 5;
				break;
			}
		/* FALL THROUGH */
		default:
			LOG_ERROR("unsupported {break,watch}point length/alignment");
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* other shared control bits:
	 * bits 15:14 == 0 ... both secure and nonsecure states (v6.1+ only)
	 * bit 20 == 0 ... not linked to a context ID
	 * bit 28:24 == 0 ... not ignoring N LSBs (v7 only)
	 */

	xp->address = addr & ~3;
	xp->control = control;
	xp->dirty = true;

	LOG_DEBUG("BPWP: addr %8.8" PRIx32 ", control %" PRIx32 ", number %d",
		xp->address, control, xp->number);

	/* hardware is updated in write_dirty_registers() */
	return ERROR_OK;
}

static int dpmv8_add_breakpoint(struct target *target, struct breakpoint *bp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (bp->length < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (!dpm->bpwp_enable)
		return retval;

	/* FIXME we need a generic solution for software breakpoints. */
	if (bp->type == BKPT_SOFT)
		LOG_DEBUG("using HW bkpt, not SW...");

	for (unsigned i = 0; i < dpm->nbp; i++) {
		if (!dpm->dbp[i].bp) {
			retval = dpmv8_bpwp_setup(dpm, &dpm->dbp[i].bpwp,
					bp->address, bp->length);
			if (retval == ERROR_OK)
				dpm->dbp[i].bp = bp;
			break;
		}
	}

	return retval;
}

static int dpmv8_remove_breakpoint(struct target *target, struct breakpoint *bp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned i = 0; i < dpm->nbp; i++) {
		if (dpm->dbp[i].bp == bp) {
			dpm->dbp[i].bp = NULL;
			dpm->dbp[i].bpwp.dirty = true;

			/* hardware is updated in write_dirty_registers() */
			retval = ERROR_OK;
			break;
		}
	}

	return retval;
}

static int dpmv8_watchpoint_setup(struct arm_dpm *dpm, unsigned index_t,
	struct watchpoint *wp)
{
	int retval;
	struct dpm_wp *dwp = dpm->dwp + index_t;
	uint32_t control;

	/* this hardware doesn't support data value matching or masking */
	if (wp->value || wp->mask != ~(uint32_t)0) {
		LOG_DEBUG("watchpoint values and masking not supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = dpmv8_bpwp_setup(dpm, &dwp->bpwp, wp->address, wp->length);
	if (retval != ERROR_OK)
		return retval;

	control = dwp->bpwp.control;
	switch (wp->rw) {
		case WPT_READ:
			control |= 1 << 3;
			break;
		case WPT_WRITE:
			control |= 2 << 3;
			break;
		case WPT_ACCESS:
			control |= 3 << 3;
			break;
	}
	dwp->bpwp.control = control;

	dpm->dwp[index_t].wp = wp;

	return retval;
}

static int dpmv8_add_watchpoint(struct target *target, struct watchpoint *wp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (dpm->bpwp_enable) {
		for (unsigned i = 0; i < dpm->nwp; i++) {
			if (!dpm->dwp[i].wp) {
				retval = dpmv8_watchpoint_setup(dpm, i, wp);
				break;
			}
		}
	}

	return retval;
}

static int dpmv8_remove_watchpoint(struct target *target, struct watchpoint *wp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned i = 0; i < dpm->nwp; i++) {
		if (dpm->dwp[i].wp == wp) {
			dpm->dwp[i].wp = NULL;
			dpm->dwp[i].bpwp.dirty = true;

			/* hardware is updated in write_dirty_registers() */
			retval = ERROR_OK;
			break;
		}
	}

	return retval;
}

void armv8_dpm_report_wfar(struct arm_dpm *dpm, uint64_t addr)
{
	switch (dpm->arm->core_state) {
		case ARM_STATE_ARM:
		case ARM_STATE_AARCH64:
			addr -= 8;
			break;
		case ARM_STATE_THUMB:
		case ARM_STATE_THUMB_EE:
			addr -= 4;
			break;
		case ARM_STATE_JAZELLE:
			/* ?? */
			break;
		default:
			LOG_DEBUG("Unknown core_state");
			break;
	}
	dpm->wp_pc = addr;
}

/*
 * Handle exceptions taken in debug state. This happens mostly for memory
 * accesses that violated a MMU policy. Taking an exception while in debug
 * state clobbers certain state registers on the target exception level.
 * Just mark those registers dirty so that they get restored on resume.
 * This works both for Aarch32 and Aarch64 states.
 *
 * This function must not perform any actions that trigger another exception
 * or a recursion will happen.
 */
void armv8_dpm_handle_exception(struct arm_dpm *dpm, bool do_restore)
{
	struct armv8_common *armv8 = dpm->arm->arch_info;
	struct reg_cache *cache = dpm->arm->core_cache;
	enum arm_state core_state;
	uint64_t dlr;
	uint32_t dspsr;
	unsigned int el;

	static const int clobbered_regs_by_el[3][5] = {
		{ ARMV8_PC, ARMV8_xPSR, ARMV8_ELR_EL1, ARMV8_ESR_EL1, ARMV8_SPSR_EL1 },
		{ ARMV8_PC, ARMV8_xPSR, ARMV8_ELR_EL2, ARMV8_ESR_EL2, ARMV8_SPSR_EL2 },
		{ ARMV8_PC, ARMV8_xPSR, ARMV8_ELR_EL3, ARMV8_ESR_EL3, ARMV8_SPSR_EL3 },
	};

	el = (dpm->dscr >> 8) & 3;

	/* safety check, must not happen since EL0 cannot be a target for an exception */
	if (el < SYSTEM_CUREL_EL1 || el > SYSTEM_CUREL_EL3) {
		LOG_ERROR("%s: EL %i is invalid, DSCR corrupted?", __func__, el);
		return;
	}

	/* Clear sticky error */
	mem_ap_write_u32(armv8->debug_ap,
		armv8->debug_base + CPUV8_DBG_DRCR, DRCR_CSE);

	armv8->read_reg_u64(armv8, ARMV8_xPSR, &dlr);
	dspsr = dlr;
	armv8->read_reg_u64(armv8, ARMV8_PC, &dlr);

	LOG_DEBUG("Exception taken to EL %i, DLR=0x%016"PRIx64" DSPSR=0x%08"PRIx32,
			el, dlr, dspsr);

	/* mark all clobbered registers as dirty */
	for (int i = 0; i < 5; i++)
		cache->reg_list[clobbered_regs_by_el[el-1][i]].dirty = true;

	/*
	 * re-evaluate the core state, we might be in Aarch64 state now
	 * we rely on dpm->dscr being up-to-date
	 */
	core_state = armv8_dpm_get_core_state(dpm);
	armv8_select_opcodes(armv8, core_state == ARM_STATE_AARCH64);
	armv8_select_reg_access(armv8, core_state == ARM_STATE_AARCH64);

	if (do_restore)
		armv8_dpm_modeswitch(dpm, ARM_MODE_ANY);
}

/*----------------------------------------------------------------------*/

/*
 * Other debug and support utilities
 */

void armv8_dpm_report_dscr(struct arm_dpm *dpm, uint32_t dscr)
{
	struct target *target = dpm->arm->target;

	dpm->dscr = dscr;
	dpm->last_el = (dscr >> 8) & 3;

	/* Examine debug reason */
	switch (DSCR_ENTRY(dscr)) {
		/* FALL THROUGH -- assume a v6 core in abort mode */
		case DSCRV8_ENTRY_EXT_DEBUG:	/* EDBGRQ */
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case DSCRV8_ENTRY_HALT_STEP_EXECLU:	/* HALT step */
		case DSCRV8_ENTRY_HALT_STEP_NORMAL: /* Halt step*/
		case DSCRV8_ENTRY_HALT_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case DSCRV8_ENTRY_HLT:	/* HLT instruction (software breakpoint) */
		case DSCRV8_ENTRY_BKPT:	/* SW BKPT (?) */
		case DSCRV8_ENTRY_RESET_CATCH:	/* Reset catch */
		case DSCRV8_ENTRY_OS_UNLOCK:  /*OS unlock catch*/
		case DSCRV8_ENTRY_SW_ACCESS_DBG: /*SW access dbg register*/
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case DSCRV8_ENTRY_WATCHPOINT:	/* asynch watchpoint */
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case DSCRV8_ENTRY_EXCEPTION_CATCH:  /*exception catch*/
			target->debug_reason = DBG_REASON_EXC_CATCH;
			break;
		default:
			target->debug_reason = DBG_REASON_UNDEFINED;
			break;
	}

}

/*----------------------------------------------------------------------*/

/*
 * Setup and management support.
 */

/**
 * Hooks up this DPM to its associated target; call only once.
 * Initially this only covers the register cache.
 *
 * Oh, and watchpoints.  Yeah.
 */
int armv8_dpm_setup(struct arm_dpm *dpm)
{
	struct arm *arm = dpm->arm;
	struct target *target = arm->target;
	struct reg_cache *cache;
	arm->dpm = dpm;

	/* register access setup */
	arm->full_context = armv8_dpm_full_context;
	arm->read_core_reg = armv8_dpm_read_core_reg;
	arm->write_core_reg = armv8_dpm_write_core_reg;

	if (arm->core_cache == NULL) {
		cache = armv8_build_reg_cache(target);
		if (!cache)
			return ERROR_FAIL;
	}

	/* coprocessor access setup */
	arm->mrc = dpmv8_mrc;
	arm->mcr = dpmv8_mcr;

	dpm->prepare = dpmv8_dpm_prepare;
	dpm->finish = dpmv8_dpm_finish;

	dpm->instr_execute = dpmv8_instr_execute;
	dpm->instr_write_data_dcc = dpmv8_instr_write_data_dcc;
	dpm->instr_write_data_dcc_64 = dpmv8_instr_write_data_dcc_64;
	dpm->instr_write_data_r0 = dpmv8_instr_write_data_r0;
	dpm->instr_write_data_r0_64 = dpmv8_instr_write_data_r0_64;
	dpm->instr_cpsr_sync = dpmv8_instr_cpsr_sync;

	dpm->instr_read_data_dcc = dpmv8_instr_read_data_dcc;
	dpm->instr_read_data_dcc_64 = dpmv8_instr_read_data_dcc_64;
	dpm->instr_read_data_r0 = dpmv8_instr_read_data_r0;
	dpm->instr_read_data_r0_64 = dpmv8_instr_read_data_r0_64;

	dpm->arm_reg_current = armv8_reg_current;

/*	dpm->bpwp_enable = dpmv8_bpwp_enable; */
	dpm->bpwp_disable = dpmv8_bpwp_disable;

	/* breakpoint setup -- optional until it works everywhere */
	if (!target->type->add_breakpoint) {
		target->type->add_breakpoint = dpmv8_add_breakpoint;
		target->type->remove_breakpoint = dpmv8_remove_breakpoint;
	}

	/* watchpoint setup */
	target->type->add_watchpoint = dpmv8_add_watchpoint;
	target->type->remove_watchpoint = dpmv8_remove_watchpoint;

	/* FIXME add vector catch support */

	dpm->nbp = 1 + ((dpm->didr >> 12) & 0xf);
	dpm->dbp = calloc(dpm->nbp, sizeof(*dpm->dbp));

	dpm->nwp = 1 + ((dpm->didr >> 20) & 0xf);
	dpm->dwp = calloc(dpm->nwp, sizeof(*dpm->dwp));

	if (!dpm->dbp || !dpm->dwp) {
		free(dpm->dbp);
		free(dpm->dwp);
		return ERROR_FAIL;
	}

	LOG_INFO("%s: hardware has %d breakpoints, %d watchpoints",
		target_name(target), dpm->nbp, dpm->nwp);

	/* REVISIT ... and some of those breakpoints could match
	 * execution context IDs...
	 */

	return ERROR_OK;
}

/**
 * Reinitializes DPM state at the beginning of a new debug session
 * or after a reset which may have affected the debug module.
 */
int armv8_dpm_initialize(struct arm_dpm *dpm)
{
	/* Disable all breakpoints and watchpoints at startup. */
	if (dpm->bpwp_disable) {
		unsigned i;

		for (i = 0; i < dpm->nbp; i++) {
			dpm->dbp[i].bpwp.number = i;
			(void) dpm->bpwp_disable(dpm, i);
		}
		for (i = 0; i < dpm->nwp; i++) {
			dpm->dwp[i].bpwp.number = 16 + i;
			(void) dpm->bpwp_disable(dpm, 16 + i);
		}
	} else
		LOG_WARNING("%s: can't disable breakpoints and watchpoints",
			target_name(dpm->arm->target));

	return ERROR_OK;
}
