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

#ifndef OPENOCD_TARGET_CORTEX_M_H
#define OPENOCD_TARGET_CORTEX_M_H

#include "armv7m.h"
#include "helper/bits.h"

#define CORTEX_M_COMMON_MAGIC 0x1A451A45

#define SYSTEM_CONTROL_BASE 0x400FE000

#define ITM_TER0	0xE0000E00
#define ITM_TPR		0xE0000E40
#define ITM_TCR		0xE0000E80
#define ITM_LAR		0xE0000FB0
#define ITM_LAR_KEY	0xC5ACCE55

#define CPUID		0xE000ED00

#define ARM_CPUID_PARTNO_MASK	0xFFF0

#define CORTEX_M23_PARTNO	0xD200
#define CORTEX_M33_PARTNO	0xD210
#define CORTEX_M35P_PARTNO	0xD310
#define CORTEX_M55_PARTNO	0xD220

/* Debug Control Block */
#define DCB_DHCSR	0xE000EDF0
#define DCB_DCRSR	0xE000EDF4
#define DCB_DCRDR	0xE000EDF8
#define DCB_DEMCR	0xE000EDFC
#define DCB_DSCSR	0xE000EE08

#define DCRSR_WnR	BIT(16)

#define DWT_CTRL	0xE0001000
#define DWT_CYCCNT	0xE0001004
#define DWT_PCSR	0xE000101C
#define DWT_COMP0	0xE0001020
#define DWT_MASK0	0xE0001024
#define DWT_FUNCTION0	0xE0001028
#define DWT_DEVARCH		0xE0001FBC

#define DWT_DEVARCH_ARMV8M	0x101A02

#define FP_CTRL		0xE0002000
#define FP_REMAP	0xE0002004
#define FP_COMP0	0xE0002008
#define FP_COMP1	0xE000200C
#define FP_COMP2	0xE0002010
#define FP_COMP3	0xE0002014
#define FP_COMP4	0xE0002018
#define FP_COMP5	0xE000201C
#define FP_COMP6	0xE0002020
#define FP_COMP7	0xE0002024

#define FPU_CPACR	0xE000ED88
#define FPU_FPCCR	0xE000EF34
#define FPU_FPCAR	0xE000EF38
#define FPU_FPDSCR	0xE000EF3C

#define TPIU_SSPSR	0xE0040000
#define TPIU_CSPSR	0xE0040004
#define TPIU_ACPR	0xE0040010
#define TPIU_SPPR	0xE00400F0
#define TPIU_FFSR	0xE0040300
#define TPIU_FFCR	0xE0040304
#define TPIU_FSCR	0xE0040308

/* Maximum SWO prescaler value. */
#define TPIU_ACPR_MAX_SWOSCALER	0x1fff

/* DCB_DHCSR bit and field definitions */
#define DBGKEY		(0xA05Ful << 16)
#define C_DEBUGEN	BIT(0)
#define C_HALT		BIT(1)
#define C_STEP		BIT(2)
#define C_MASKINTS	BIT(3)
#define S_REGRDY	BIT(16)
#define S_HALT		BIT(17)
#define S_SLEEP		BIT(18)
#define S_LOCKUP	BIT(19)
#define S_RETIRE_ST	BIT(24)
#define S_RESET_ST	BIT(25)

/* DCB_DEMCR bit and field definitions */
#define TRCENA			BIT(24)
#define VC_HARDERR		BIT(10)
#define VC_INTERR		BIT(9)
#define VC_BUSERR		BIT(8)
#define VC_STATERR		BIT(7)
#define VC_CHKERR		BIT(6)
#define VC_NOCPERR		BIT(5)
#define VC_MMERR		BIT(4)
#define VC_CORERESET	BIT(0)

/* DCB_DSCSR bit and field definitions */
#define DSCSR_CDS		BIT(16)

/* NVIC registers */
#define NVIC_ICTR		0xE000E004
#define NVIC_ISE0		0xE000E100
#define NVIC_ICSR		0xE000ED04
#define NVIC_AIRCR		0xE000ED0C
#define NVIC_SHCSR		0xE000ED24
#define NVIC_CFSR		0xE000ED28
#define NVIC_MMFSRb		0xE000ED28
#define NVIC_BFSRb		0xE000ED29
#define NVIC_USFSRh		0xE000ED2A
#define NVIC_HFSR		0xE000ED2C
#define NVIC_DFSR		0xE000ED30
#define NVIC_MMFAR		0xE000ED34
#define NVIC_BFAR		0xE000ED38
#define NVIC_SFSR		0xE000EDE4
#define NVIC_SFAR		0xE000EDE8

/* NVIC_AIRCR bits */
#define AIRCR_VECTKEY		(0x5FAul << 16)
#define AIRCR_SYSRESETREQ	BIT(2)
#define AIRCR_VECTCLRACTIVE	BIT(1)
#define AIRCR_VECTRESET		BIT(0)
/* NVIC_SHCSR bits */
#define SHCSR_BUSFAULTENA	BIT(17)
/* NVIC_DFSR bits */
#define DFSR_HALTED			1
#define DFSR_BKPT			2
#define DFSR_DWTTRAP		4
#define DFSR_VCATCH			8
#define DFSR_EXTERNAL		16

#define FPCR_CODE 0
#define FPCR_LITERAL 1
#define FPCR_REPLACE_REMAP  (0ul << 30)
#define FPCR_REPLACE_BKPT_LOW  (1ul << 30)
#define FPCR_REPLACE_BKPT_HIGH  (2ul << 30)
#define FPCR_REPLACE_BKPT_BOTH  (3ul << 30)

struct cortex_m_fp_comparator {
	bool used;
	int type;
	uint32_t fpcr_value;
	uint32_t fpcr_address;
};

struct cortex_m_dwt_comparator {
	bool used;
	uint32_t comp;
	uint32_t mask;
	uint32_t function;
	uint32_t dwt_comparator_address;
};

enum cortex_m_soft_reset_config {
	CORTEX_M_RESET_SYSRESETREQ,
	CORTEX_M_RESET_VECTRESET,
};

enum cortex_m_isrmasking_mode {
	CORTEX_M_ISRMASK_AUTO,
	CORTEX_M_ISRMASK_OFF,
	CORTEX_M_ISRMASK_ON,
	CORTEX_M_ISRMASK_STEPONLY,
};

struct cortex_m_common {
	int common_magic;

	/* Context information */
	uint32_t dcb_dhcsr;
	uint32_t nvic_dfsr;  /* Debug Fault Status Register - shows reason for debug halt */
	uint32_t nvic_icsr;  /* Interrupt Control State Register - shows active and pending IRQ */

	/* Flash Patch and Breakpoint (FPB) */
	int fp_num_lit;
	int fp_num_code;
	int fp_rev;
	bool fpb_enabled;
	struct cortex_m_fp_comparator *fp_comparator_list;

	/* Data Watchpoint and Trace (DWT) */
	int dwt_num_comp;
	int dwt_comp_available;
	uint32_t dwt_devarch;
	struct cortex_m_dwt_comparator *dwt_comparator_list;
	struct reg_cache *dwt_cache;

	enum cortex_m_soft_reset_config soft_reset_config;
	bool vectreset_supported;

	enum cortex_m_isrmasking_mode isrmasking_mode;

	struct armv7m_common armv7m;

	int apsel;

	/* Whether this target has the erratum that makes C_MASKINTS not apply to
	 * already pending interrupts */
	bool maskints_erratum;
};

static inline struct cortex_m_common *
target_to_cm(struct target *target)
{
	return container_of(target->arch_info,
			struct cortex_m_common, armv7m);
}

int cortex_m_examine(struct target *target);
int cortex_m_set_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_unset_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_remove_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int cortex_m_remove_watchpoint(struct target *target, struct watchpoint *watchpoint);
void cortex_m_enable_breakpoints(struct target *target);
void cortex_m_enable_watchpoints(struct target *target);
void cortex_m_deinit_target(struct target *target);
int cortex_m_profiling(struct target *target, uint32_t *samples,
	uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds);

#endif /* OPENOCD_TARGET_CORTEX_M_H */
