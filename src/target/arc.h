/***************************************************************************
 *   Copyright (C) 2013-2015,2019-2020 Synopsys, Inc.                      *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Mischa Jonker <mischa.jonker@synopsys.com>                            *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>                                    *
 *                                                                         *
 *   SPDX-License-Identifier: GPL-2.0-or-later                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARC_H
#define OPENOCD_TARGET_ARC_H

#include <helper/time_support.h>
#include <jtag/jtag.h>

#include "algorithm.h"
#include "breakpoints.h"
#include "jtag/interface.h"
#include "register.h"
#include "target.h"
#include "target_request.h"
#include "target_type.h"
#include "helper/bits.h"

#include "arc_jtag.h"
#include "arc_cmd.h"
#include "arc_mem.h"

#define ARC_COMMON_MAGIC	0xB32EB324  /* just a unique number */

#define AUX_DEBUG_REG                   0x5
#define AUX_PC_REG                      0x6
#define AUX_STATUS32_REG                0xA


#define SET_CORE_FORCE_HALT             BIT(1)
#define SET_CORE_HALT_BIT               BIT(0)      /* STATUS32[0] = H field */
#define SET_CORE_ENABLE_INTERRUPTS      BIT(31)
/* STATUS32[5] or AE bit indicates if the processor is in exception state */
#define SET_CORE_AE_BIT                 BIT(5)
/* Single instruction step bit in Debug register */
#define SET_CORE_SINGLE_INSTR_STEP      BIT(11)

#define AUX_STATUS32_REG_HALT_BIT       BIT(0)
#define AUX_STATUS32_REG_IE_BIT         BIT(31)    /* STATUS32[31] = IE field */

/* ARC register numbers */
enum {
	ARC_R0,
	ARC_R1,
	ARC_R2,
	ARC_R3,
	ARC_R4,
	ARC_R5,
	ARC_R6,
	ARC_R7,
	ARC_R8,
	ARC_R9,
	ARC_R10,
	ARC_R11,
	ARC_R12,
	ARC_R13,
	ARC_R14,
	ARC_R15,
	ARC_R16,
	ARC_R17,
	ARC_R18,
	ARC_R19,
	ARC_R20,
	ARC_R21,
	ARC_R22,
	ARC_R23,
	ARC_R24,
	ARC_R25,
	ARC_GP		= 26,
	ARC_FP		= 27,
	ARC_SP		= 28,
	ARC_ILINK	= 29,
	ARC_R30,
	ARC_BLINK	= 31,
	ARC_LP_COUNT	= 60,

	/* Reserved registers */
	ARC_R61		= 61,
	ARC_R62		= 62,

	ARC_PCL		= 63,
	ARC_PC		= 64,
	ARC_LP_START	= 65,
	ARC_LP_END	= 66,
	ARC_STATUS32	= 67,
};

#define CORE_REG_MAX_NUMBER		(63)

/* Limit reg_type/reg_type_field  name to 20 symbols */
#define REG_TYPE_MAX_NAME_LENGTH	20

/* ARC 32bits opcodes */
#define ARC_SDBBP_32 0x256F003FU  /* BRK */

/* ARC 16bits opcodes */
#define ARC_SDBBP_16 0x7FFF      /* BRK_S */

/* Cache registers */
#define AUX_IC_IVIC_REG			0X10
#define IC_IVIC_INVALIDATE		0XFFFFFFFF

#define AUX_DC_IVDC_REG			0X47
#define DC_IVDC_INVALIDATE		BIT(0)
#define AUX_DC_CTRL_REG			0X48
#define DC_CTRL_IM			BIT(6)

/* L2 cache registers */
#define SLC_AUX_CACHE_CTRL		0x903
#define L2_CTRL_IM			BIT(6)
#define L2_CTRL_BS			BIT(8)		/* Busy flag */
#define SLC_AUX_CACHE_FLUSH		0x904
#define L2_FLUSH_FL			BIT(0)
#define SLC_AUX_CACHE_INV		0x905
#define L2_INV_IV			BIT(0)

 /* Action Point */
#define AP_AC_AT_INST_ADDR		0x0
#define AP_AC_AT_MEMORY_ADDR	0x2
#define AP_AC_AT_AUXREG_ADDR	0x4

#define AP_AC_TT_DISABLE		0x00
#define AP_AC_TT_WRITE			0x10
#define AP_AC_TT_READ			0x20
#define AP_AC_TT_READWRITE		0x30

struct arc_reg_bitfield {
	struct reg_data_type_bitfield bitfield;
	char name[REG_TYPE_MAX_NAME_LENGTH];
};
/* Register data type */
struct arc_reg_data_type {
	struct list_head list;
	struct reg_data_type data_type;
	struct reg_data_type_flags data_type_flags;
	struct reg_data_type_struct data_type_struct;
	char data_type_id[REG_TYPE_MAX_NAME_LENGTH];
	struct arc_reg_bitfield *bitfields;
	union {
		struct reg_data_type_struct_field *reg_type_struct_field;
		struct reg_data_type_flags_field *reg_type_flags_field;
	};
};

/* Standard GDB register types */
static const struct reg_data_type standard_gdb_types[] = {
	{ .type = REG_TYPE_INT,         .id = "int" },
	{ .type = REG_TYPE_INT8,        .id = "int8" },
	{ .type = REG_TYPE_INT16,       .id = "int16" },
	{ .type = REG_TYPE_INT32,       .id = "int32" },
	{ .type = REG_TYPE_INT64,       .id = "int64" },
	{ .type = REG_TYPE_INT128,      .id = "int128" },
	{ .type = REG_TYPE_UINT8,       .id = "uint8" },
	{ .type = REG_TYPE_UINT16,      .id = "uint16" },
	{ .type = REG_TYPE_UINT32,      .id = "uint32" },
	{ .type = REG_TYPE_UINT64,      .id = "uint64" },
	{ .type = REG_TYPE_UINT128,     .id = "uint128" },
	{ .type = REG_TYPE_CODE_PTR,    .id = "code_ptr" },
	{ .type = REG_TYPE_DATA_PTR,    .id = "data_ptr" },
	{ .type = REG_TYPE_FLOAT,       .id = "float" },
	{ .type = REG_TYPE_IEEE_SINGLE, .id = "ieee_single" },
	{ .type = REG_TYPE_IEEE_DOUBLE, .id = "ieee_double" },
};

enum arc_actionpointype {
	ARC_AP_BREAKPOINT,
	ARC_AP_WATCHPOINT,
};

/* Actionpoint related fields  */
struct arc_actionpoint {
	int used;
	uint32_t bp_value;
	uint32_t reg_address;
	enum arc_actionpointype type;
};

struct arc_common {
	uint32_t common_magic;

	struct arc_jtag jtag_info;

	struct reg_cache *core_and_aux_cache;
	struct reg_cache *bcr_cache;

	/* Cache control */
	bool has_dcache;
	bool has_icache;
	bool has_l2cache;
	/* If true, then D$ has been already flushed since core has been
	 * halted. */
	bool dcache_flushed;
	/* If true, then L2 has been already flushed since core has been
	 * halted. */
	bool l2cache_flushed;
	/* If true, then caches have been already flushed since core has been
	 * halted. */
	bool icache_invalidated;
	bool dcache_invalidated;
	bool l2cache_invalidated;

	/* Indicate if cache was built (for deinit function) */
	bool core_aux_cache_built;
	bool bcr_cache_built;
	/* Closely Coupled memory(CCM) regions for performance-critical
	 * code (optional). */
	uint32_t iccm0_start;
	uint32_t iccm0_end;
	uint32_t iccm1_start;
	uint32_t iccm1_end;
	uint32_t dccm_start;
	uint32_t dccm_end;

	int irq_state;

	/* Register descriptions */
	struct list_head reg_data_types;
	struct list_head core_reg_descriptions;
	struct list_head aux_reg_descriptions;
	struct list_head bcr_reg_descriptions;
	unsigned long num_regs;
	unsigned long num_core_regs;
	unsigned long num_aux_regs;
	unsigned long num_bcr_regs;
	unsigned long last_general_reg;

	/* PC register location in register cache. */
	unsigned long pc_index_in_cache;
	/* DEBUG register location in register cache. */
	unsigned long debug_index_in_cache;

	/* Actionpoints */
	unsigned int actionpoints_num;
	unsigned int actionpoints_num_avail;
	struct arc_actionpoint *actionpoints_list;
};

/* Borrowed from nds32.h */
#define CHECK_RETVAL(action)			\
	do {					\
		int __retval = (action);	\
		if (__retval != ERROR_OK) {	\
			LOG_DEBUG("error while calling \"%s\"",	\
				# action);     \
			return __retval;	\
		}				\
	} while (0)

#define JIM_CHECK_RETVAL(action)		\
	do {					\
		int __retval = (action);	\
		if (__retval != JIM_OK) {	\
			LOG_DEBUG("error while calling \"%s\"",	\
				# action);     \
			return __retval;	\
		}				\
	} while (0)

static inline struct arc_common *target_to_arc(struct target *target)
{
	return target->arch_info;
}

/* ----- Inlined functions ------------------------------------------------- */

/**
 * Convert data in host endianness to the middle endian. This is required to
 * write 4-byte instructions.
 */
static inline void arc_h_u32_to_me(uint8_t *buf, int val)
{
	buf[1] = (uint8_t) (val >> 24);
	buf[0] = (uint8_t) (val >> 16);
	buf[3] = (uint8_t) (val >> 8);
	buf[2] = (uint8_t) (val >> 0);
}

/**
 * Convert data in middle endian to host endian. This is required to read 32-bit
 * instruction from little endian ARCs.
 */
static inline uint32_t arc_me_to_h_u32(const uint8_t *buf)
{
	return (uint32_t)(buf[2] | buf[3] << 8 | buf[0] << 16 | buf[1] << 24);
}


/* ARC Register description */
struct arc_reg_desc {

	struct target *target;

	/* Register name */
	char *name;

	/* Actual place of storing reg_value */
	uint8_t reg_value[4];

	/* Actual place of storing register feature */
	struct reg_feature feature;

	/* GDB XML feature */
	char *gdb_xml_feature;

	/* Is this a register in g/G-packet? */
	bool is_general;

	/* Architectural number: core reg num or AUX reg num */
	uint32_t arch_num;

	/* Core or AUX register? */
	bool is_core;

	/* Build configuration register? */
	bool is_bcr;

	/* Data type */
	struct reg_data_type *data_type;

	struct list_head list;
};

/* Error codes */
#define ERROR_ARC_REGISTER_NOT_FOUND       (-700)
#define ERROR_ARC_REGISTER_FIELD_NOT_FOUND (-701)
#define ERROR_ARC_REGISTER_IS_NOT_STRUCT   (-702)
#define ERROR_ARC_FIELD_IS_NOT_BITFIELD    (-703)
#define ERROR_ARC_REGTYPE_NOT_FOUND        (-704)

void free_reg_desc(struct arc_reg_desc *r);


void arc_reg_data_type_add(struct target *target,
		struct arc_reg_data_type *data_type);

int arc_reg_add(struct target *target, struct arc_reg_desc *arc_reg,
		const char * const type_name, const size_t type_name_len);

struct reg *arc_reg_get_by_name(struct reg_cache *first,
					const char *name, bool search_all);

int arc_reg_get_field(struct target *target, const char *reg_name,
		const char *field_name, uint32_t *value_ptr);

int arc_cache_flush(struct target *target);
int arc_cache_invalidate(struct target *target);

int arc_add_auxreg_actionpoint(struct target *target,
	uint32_t auxreg_addr, uint32_t transaction);
int arc_remove_auxreg_actionpoint(struct target *target, uint32_t auxreg_addr);
int arc_set_actionpoints_num(struct target *target, uint32_t ap_num);

#endif /* OPENOCD_TARGET_ARC_H */
