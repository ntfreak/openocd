/***************************************************************************
 *    Copyright (C) 2009 by David Brownell                                 *
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

#ifndef OPENOCD_TARGET_ARMV7A_H
#define OPENOCD_TARGET_ARMV7A_H

#include "arm_adi_v5.h"
#include "armv7a_cache.h"
#include "arm.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"
#include "arm_dpm.h"

enum {
	ARM_PC  = 15,
	ARM_CPSR = 16
};

#define ARMV7_COMMON_MAGIC 0x0A450999

/* VA to PA translation operations opc2 values*/
#define V2PCWPR  0
#define V2PCWPW  1
#define V2PCWUR  2
#define V2PCWUW  3
#define V2POWPR  4
#define V2POWPW  5
#define V2POWUR  6
#define V2POWUW  7
/*   L210/L220 cache controller support */
struct armv7a_l2x_cache {
	uint32_t base;
	uint32_t way;
};

struct armv7a_cachesize {
	/*  cache dimensioning */
	uint32_t linelen;
	uint32_t associativity;
	uint32_t nsets;
	uint32_t cachesize;
	/* info for set way operation on cache */
	uint32_t index;
	uint32_t index_shift;
	uint32_t way;
	uint32_t way_shift;
};

/* information about one architecture cache at any level */
struct armv7a_arch_cache {
	int ctype;				/* cache type, CLIDR encoding */
	struct armv7a_cachesize d_u_size;	/* data cache */
	struct armv7a_cachesize i_size;		/* instruction cache */
};

/* common cache information */
struct armv7a_cache_common {
	int info;				/* -1 invalid, else valid */
	int loc;				/* level of coherency */
	uint32_t dminline;			/* minimum d-cache linelen */
	uint32_t iminline;			/* minimum i-cache linelen */
	struct armv7a_arch_cache arch[6];	/* cache info, L1 - L7 */
	int i_cache_enabled;
	int d_u_cache_enabled;
	int auto_cache_enabled;			/* openocd automatic
						 * cache handling */
	/* outer unified cache if some */
	void *outer_cache;
	int (*flush_all_data_cache)(struct target *target);
};

struct armv7a_mmu_common {
	/* following field mmu working way */
	int32_t cached;     /* 0: not initialized, 1: initialized */
	uint32_t ttbcr;     /* cache for ttbcr register */
	uint32_t ttbr[2];
	uint32_t ttbr_mask[2];
	uint32_t ttbr_range[2];

	int (*read_physical_memory)(struct target *target, target_addr_t address, uint32_t size,
			uint32_t count, uint8_t *buffer);
	struct armv7a_cache_common armv7a_cache;
	uint32_t mmu_enabled;
};

struct armv7a_common {
	struct arm arm;
	int common_magic;
	struct reg_cache *core_cache;

	/* Core Debug Unit */
	struct arm_dpm dpm;
	uint32_t debug_base;
	struct adiv5_ap *debug_ap;
	/* mdir */
	uint8_t multi_processor_system;
	uint8_t multi_threading_processor;
	uint8_t level2_id;
	uint8_t cluster_id;
	uint8_t cpu_id;
	bool is_armv7r;
	uint32_t rev;
	uint32_t partnum;
	uint32_t arch;
	uint32_t variant;
	uint32_t implementor;

	/* cache specific to V7 Memory Management Unit compatible with v4_5*/
	struct armv7a_mmu_common armv7a_mmu;

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv7a_common *
target_to_armv7a(struct target *target)
{
	return container_of(target->arch_info, struct armv7a_common, arm);
}

static inline bool is_armv7a(struct armv7a_common *armv7a)
{
	return armv7a->common_magic == ARMV7_COMMON_MAGIC;
}


/* register offsets from armv7a.debug_base */

/* See ARMv7a arch spec section C10.2 */
#define CPUDBG_DIDR		0x000

/* See ARMv7a arch spec section C10.3 */
#define CPUDBG_WFAR		0x018
/* PCSR at 0x084 -or- 0x0a0 -or- both ... based on flags in DIDR */
#define CPUDBG_DSCR		0x088
#define CPUDBG_DRCR		0x090
#define CPUDBG_PRCR		0x310
#define CPUDBG_PRSR		0x314

/* See ARMv7a arch spec section C10.4 */
#define CPUDBG_DTRRX		0x080
#define CPUDBG_ITR		0x084
#define CPUDBG_DTRTX		0x08c

/* See ARMv7a arch spec section C10.5 */
#define CPUDBG_BVR_BASE		0x100
#define CPUDBG_BCR_BASE		0x140
#define CPUDBG_WVR_BASE		0x180
#define CPUDBG_WCR_BASE		0x1C0
#define CPUDBG_VCR		0x01C

/* See ARMv7a arch spec section C10.6 */
#define CPUDBG_OSLAR		0x300
#define CPUDBG_OSLSR		0x304
#define CPUDBG_OSSRR		0x308
#define CPUDBG_ECR		0x024

/* See ARMv7a arch spec section C10.7 */
#define CPUDBG_DSCCR		0x028
#define CPUDBG_DSMCR		0x02C

/* See ARMv7a arch spec section C10.8 */
#define CPUDBG_AUTHSTATUS	0xFB8

/* See ARMv7a arch spec DDI 0406C C11.10 */
#define CPUDBG_ID_PFR1		0xD24

/* Masks for Vector Catch register */
#define DBG_VCR_FIQ_MASK	((1 << 31) | (1 << 7))
#define DBG_VCR_IRQ_MASK	((1 << 30) | (1 << 6))
#define DBG_VCR_DATA_ABORT_MASK	((1 << 28) | (1 << 4))
#define DBG_VCR_PREF_ABORT_MASK	((1 << 27) | (1 << 3))
#define DBG_VCR_SVC_MASK	((1 << 26) | (1 << 2))

/* Masks for Multiprocessor Affinity Register */
#define MPIDR_MP_EXT		(1UL << 31)

int armv7a_arch_state(struct target *target);
int armv7a_identify_cache(struct target *target);
int armv7a_init_arch_info(struct target *target, struct armv7a_common *armv7a);

int armv7a_handle_cache_info_command(struct command_invocation *cmd,
		struct armv7a_cache_common *armv7a_cache);
int armv7a_read_ttbcr(struct target *target);

extern const struct command_registration armv7a_command_handlers[];

#endif /* OPENOCD_TARGET_ARMV7A_H */
