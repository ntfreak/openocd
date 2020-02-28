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
#define SET_CORE_ENABLE_INTERRUPTS			BIT(31)

#define AUX_STATUS32_REG_HALT_BIT       BIT(0)
#define AUX_STATUS32_REG_IE_BIT         BIT(31)    /* STATUS32[31] = IE field */

/* Reserved core registers */
#define CORE_R61_NUM			(61)
#define CORE_R62_NUM			(62)

#define CORE_REG_MAX_NUMBER		(63)

/* Limit reg_type/reg_type_field  name to 20 symbols */
#define REG_TYPE_MAX_NAME_LENGTH	20

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


struct arc_common {
	uint32_t common_magic;

	struct arc_jtag jtag_info;

	struct reg_cache *core_and_aux_cache;
	struct reg_cache *bcr_cache;

	/* Indicate if cach was built (for deinit function) */
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

#endif /* OPENOCD_TARGET_ARC_H */
