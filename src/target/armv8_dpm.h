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
 */

#ifndef OPENOCD_TARGET_ARMV8_DPM_H
#define OPENOCD_TARGET_ARMV8_DPM_H

#include "arm_dpm.h"
#include "helper/bits.h"

/* forward-declare struct armv8_common */
struct armv8_common;

/**
 * This wraps an implementation of DPM primitives.  Each interface
 * provider supplies a structure like this, which is the glue between
 * upper level code and the lower level hardware access.
 *
 * It is a PRELIMINARY AND INCOMPLETE set of primitives, starting with
 * support for CPU register access.
 */
int armv8_dpm_setup(struct arm_dpm *dpm);
int armv8_dpm_initialize(struct arm_dpm *dpm);

int armv8_dpm_read_current_registers(struct arm_dpm *dpm);
int armv8_dpm_modeswitch(struct arm_dpm *dpm, enum arm_mode mode);


int armv8_dpm_write_dirty_registers(struct arm_dpm *dpm, bool bpwp);

/* DSCR bits; see ARMv7a arch spec section C10.3.1.
 * Not all v7 bits are valid in v6.
 */
#define DSCR_DEBUG_STATUS_MASK		(0x1F <<  0)
#define DSCR_ERR					(0x1 <<  6)
#define DSCR_SYS_ERROR_PEND			(0x1 <<  7)
#define DSCR_CUR_EL					(0x3 <<  8)
#define DSCR_EL_STATUS_MASK			(0xF << 10)
#define DSCR_HDE					(0x1 << 14)
#define DSCR_SDD					(0x1 << 16)
#define DSCR_NON_SECURE             (0x1 << 18)
#define DSCR_MA						(0x1 << 20)
#define DSCR_TDA					(0x1 << 21)
#define DSCR_INTDIS_MASK			(0x3 << 22)
#define DSCR_ITE					(0x1 << 24)
#define DSCR_PIPE_ADVANCE           (0x1 << 25)
#define DSCR_TXU					(0x1 << 26)
#define DSCR_RTO					(0x1 << 27) /* bit 28 is reserved */
#define DSCR_ITO					(0x1 << 28)
#define DSCR_DTR_TX_FULL            (0x1 << 29)
#define DSCR_DTR_RX_FULL            (0x1 << 30) /* bit 31 is reserved */



/* Methods of entry into debug mode */
#define DSCRV8_ENTRY_NON_DEBUG			(0x2)
#define DSCRV8_ENTRY_RESTARTING			(0x1)
#define DSCRV8_ENTRY_BKPT				(0x7)
#define DSCRV8_ENTRY_EXT_DEBUG			(0x13)
#define DSCRV8_ENTRY_HALT_STEP_NORMAL	(0x1B)
#define DSCRV8_ENTRY_HALT_STEP_EXECLU	(0x1F)
#define DSCRV8_ENTRY_OS_UNLOCK			(0x23)
#define DSCRV8_ENTRY_RESET_CATCH		(0x27)
#define DSCRV8_ENTRY_WATCHPOINT			(0x2B)
#define DSCRV8_ENTRY_HLT				(0x2F)
#define DSCRV8_ENTRY_SW_ACCESS_DBG		(0x33)
#define DSCRV8_ENTRY_EXCEPTION_CATCH	(0x37)
#define DSCRV8_ENTRY_HALT_STEP			(0x3B)
#define DSCRV8_HALT_MASK			(0x3C)

/*DRCR registers*/
#define DRCR_CSE				(1 << 2)
#define DRCR_CSPA				(1 << 3)
#define DRCR_CBRRQ				(1 << 4)


/* DTR modes */
#define DSCR_EXT_DCC_NON_BLOCKING     (0x0 << 20)
#define DSCR_EXT_DCC_STALL_MODE       (0x1 << 20)
#define DSCR_EXT_DCC_FAST_MODE        (0x2 << 20)  /* bits 22, 23 are reserved */


/* DRCR (debug run control register) bits */
#define DRCR_HALT				(1 << 0)
#define DRCR_RESTART			(1 << 1)
#define DRCR_CLEAR_EXCEPTIONS	(1 << 2)

/* ECR (Execution Control Register) bits */
#define ECR_RCE         BIT(1)

/* ESR (Event Status Register) bits */
#define ESR_RC          BIT(1)

/* PRSR (processor debug status register) bits */
#define PRSR_PU					(1 << 0)
#define PRSR_SPD				(1 << 1)
#define PRSR_RESET				(1 << 2)
#define PRSR_SR					(1 << 3)
#define PRSR_HALT				(1 << 4)
#define PRSR_OSLK				(1 << 5)
#define PRSR_DLK				(1 << 6)
#define PRSR_EDAD				(1 << 7)
#define PRSR_SDAD				(1 << 8)
#define PRSR_EPMAD				(1 << 9)
#define PRSR_SPMAD				(1 << 10)
#define PRSR_SDR				(1 << 11)

/* PRCR (processor debug control register) bits */
#define PRCR_CORENPDRQ			(1 << 0)
#define PRCR_CWRR				(1 << 2)
#define PRCR_COREPURQ			(1 << 3)

void armv8_dpm_report_dscr(struct arm_dpm *dpm, uint32_t dcsr);
void armv8_dpm_handle_exception(struct arm_dpm *dpm, bool do_restore);
enum arm_state armv8_dpm_get_core_state(struct arm_dpm *dpm);

#endif /* OPENOCD_TARGET_ARM_DPM_H */
