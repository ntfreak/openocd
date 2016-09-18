/***************************************************************************
 *   Copyright (C) 2015 by David Ung                                       *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARMV8_H
#define OPENOCD_TARGET_ARMV8_H

#include "arm_adi_v5.h"
#include "arm.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"
#include "armv8_dpm.h"

enum {
	ARMV8_R0,
	ARMV8_R1,
	ARMV8_R2,
	ARMV8_R3,
	ARMV8_R4,
	ARMV8_R5,
	ARMV8_R6,
	ARMV8_R7,
	ARMV8_R8,
	ARMV8_R9,
	ARMV8_R10,
	ARMV8_R11,
	ARMV8_R12,
	ARMV8_R13,
	ARMV8_R14,
	ARMV8_R15,
	ARMV8_R16,
	ARMV8_R17,
	ARMV8_R18,
	ARMV8_R19,
	ARMV8_R20,
	ARMV8_R21,
	ARMV8_R22,
	ARMV8_R23,
	ARMV8_R24,
	ARMV8_R25,
	ARMV8_R26,
	ARMV8_R27,
	ARMV8_R28,
	ARMV8_R29,
	ARMV8_R30,
	ARMV8_R31,

	ARMV8_PC = 32,
	ARMV8_xPSR = 33,

	ARMV8_LAST_REG,
};


#define ARMV8_COMMON_MAGIC 0x0A450AAA

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
struct armv8_l2x_cache {
	uint32_t base;
	uint32_t way;
};

struct armv8_cachesize {
	uint32_t level_num;
	/*  cache dimensionning */
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

struct armv8_cache_common {
	int ctype;
	struct armv8_cachesize d_u_size;	/* data cache */
	struct armv8_cachesize i_size;		/* instruction cache */
	int i_cache_enabled;
	int d_u_cache_enabled;
	/* l2 external unified cache if some */
	void *l2_cache;
	int (*flush_all_data_cache)(struct target *target);
	int (*display_cache_info)(struct command_context *cmd_ctx,
			struct armv8_cache_common *armv8_cache);
};

struct armv8_mmu_common {
	/* following field mmu working way */
	int32_t ttbr1_used; /*  -1 not initialized, 0 no ttbr1 1 ttbr1 used and  */
	uint64_t ttbr0_mask;/*  masked to be used  */
	uint32_t os_border;

	int (*read_physical_memory)(struct target *target, target_addr_t address,
			uint32_t size, uint32_t count, uint8_t *buffer);
	struct armv8_cache_common armv8_cache;
	uint32_t mmu_enabled;
};

struct armv8_common {
	struct arm arm;
	int common_magic;
	struct reg_cache *core_cache;

	/* Core Debug Unit */
	struct arm_dpm dpm;
	uint32_t debug_base;
	uint32_t cti_base;
	struct adiv5_ap *debug_ap;

	/* mdir */
	uint8_t multi_processor_system;
	uint8_t cluster_id;
	uint8_t cpu_id;

	/* armv8 aarch64 need below information for page translation */
	uint8_t va_size;
	uint8_t pa_size;
	uint32_t page_size;
	uint64_t ttbr_base;

	/* cache specific to V7 Memory Management Unit compatible with v4_5*/
	struct armv8_mmu_common armv8_mmu;

	/* Direct processor core register read and writes */
	int (*load_core_reg_u64)(struct target *target, uint32_t num, uint64_t *value);
	int (*store_core_reg_u64)(struct target *target, uint32_t num, uint64_t value);

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv8_common *
target_to_armv8(struct target *target)
{
	return container_of(target->arch_info, struct armv8_common, arm);
}

/* register offsets from armv8.debug_base */
#define CPUV8_DBG_MAINID0		0xD00
#define CPUV8_DBG_CPUFEATURE0	0xD20
#define CPUV8_DBG_DBGFEATURE0	0xD28
#define CPUV8_DBG_MEMFEATURE0	0xD38

#define CPUV8_DBG_LOCKACCESS 0xFB0
#define CPUV8_DBG_LOCKSTATUS 0xFB4

#define CPUV8_DBG_EDESR		0x20
#define CPUV8_DBG_EDECR		0x24
#define CPUV8_DBG_WFAR0		0x30
#define CPUV8_DBG_WFAR1		0x34
#define CPUV8_DBG_DSCR		0x088
#define CPUV8_DBG_DRCR		0x090
#define CPUV8_DBG_PRCR		0x310
#define CPUV8_DBG_PRSR		0x314

#define CPUV8_DBG_DTRRX		0x080
#define CPUV8_DBG_ITR		0x084
#define CPUV8_DBG_SCR		0x088
#define CPUV8_DBG_DTRTX		0x08c

#define CPUV8_DBG_BVR_BASE	0x400
#define CPUV8_DBG_BCR_BASE	0x408
#define CPUV8_DBG_WVR_BASE	0x800
#define CPUV8_DBG_WCR_BASE	0x808
#define CPUV8_DBG_VCR		0x01C

#define CPUV8_DBG_OSLAR		0x300

#define CPUV8_DBG_AUTHSTATUS	0xFB8

/*define CTI(cross trigger interface)*/
#define CTI_CTR				0x0
#define CTI_INACK			0x10
#define CTI_APPSET			0x14
#define CTI_APPCLEAR		0x18
#define CTI_APPPULSE		0x1C
#define CTI_INEN0			0x20
#define CTI_INEN1			0x24
#define CTI_INEN2			0x28
#define CTI_INEN3			0x2C
#define CTI_INEN4			0x30
#define CTI_INEN5			0x34
#define CTI_INEN6			0x38
#define CTI_INEN7			0x3C
#define CTI_OUTEN0			0xA0
#define CTI_OUTEN1			0xA4
#define CTI_OUTEN2			0xA8
#define CTI_OUTEN3			0xAC
#define CTI_OUTEN4			0xB0
#define CTI_OUTEN5			0xB4
#define CTI_OUTEN6			0xB8
#define CTI_OUTEN7			0xBC
#define CTI_TRIN_STATUS		0x130
#define CTI_TROUT_STATUS	0x134
#define CTI_CHIN_STATUS		0x138
#define CTI_CHOU_STATUS		0x13C
#define CTI_GATE			0x140
#define CTI_UNLOCK			0xFB0

#define PAGE_SIZE_4KB				0x1000
#define PAGE_SIZE_4KB_LEVEL0_BITS	39
#define PAGE_SIZE_4KB_LEVEL1_BITS	30
#define PAGE_SIZE_4KB_LEVEL2_BITS	21
#define PAGE_SIZE_4KB_LEVEL3_BITS	12

#define PAGE_SIZE_4KB_LEVEL0_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL0_BITS)
#define PAGE_SIZE_4KB_LEVEL1_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL1_BITS)
#define PAGE_SIZE_4KB_LEVEL2_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL2_BITS)
#define PAGE_SIZE_4KB_LEVEL3_MASK	((0x1FFULL) << PAGE_SIZE_4KB_LEVEL3_BITS)

#define PAGE_SIZE_4KB_TRBBASE_MASK	0xFFFFFFFFF000

int armv8_arch_state(struct target *target);
int armv8_identify_cache(struct target *target);
int armv8_init_arch_info(struct target *target, struct armv8_common *armv8);
int armv8_mmu_translate_va_pa(struct target *target, target_addr_t va,
		target_addr_t *val, int meminfo);
int armv8_mmu_translate_va(struct target *target,  target_addr_t va, target_addr_t *val);

int armv8_handle_cache_info_command(struct command_context *cmd_ctx,
		struct armv8_cache_common *armv8_cache);

void armv8_set_cpsr(struct arm *arm, uint32_t cpsr);

extern const struct command_registration armv8_command_handlers[];

#endif
