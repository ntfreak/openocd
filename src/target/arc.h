/***************************************************************************
 *   Copyright (C) 2013-2015,2019 Synopsys, Inc.                           *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Mischa Jonker <mischa.jonker@synopsys.com>                            *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef ARC_H
#define ARC_H

#include <helper/time_support.h>
#include <jtag/jtag.h>

#include "algorithm.h"
#include "breakpoints.h"
#include "jtag/interface.h"
#include "register.h"
#include "target.h"
#include "target_request.h"
#include "target_type.h"

#include "arc_jtag.h"
#include "arc_cmd.h"
#include "arc_mem.h"

#define ARC_COMMON_MAGIC	0xB32EB324  /* just a unique number */

#define AUX_DEBUG_REG                   0x5
#define AUX_PC_REG                      0x6
#define AUX_STATUS32_REG                0xA

#define SET_CORE_FORCE_HALT             (1 << 1)
#define SET_CORE_HALT_BIT               (1)      /* STATUS32[0] = H field */


#define AUX_STATUS32_REG_HALT_BIT       (1)

#define AUX_IRQ_ENABLE_REG			0x40C
#define SET_CORE_DISABLE_INTERRUPTS		0x0
#define SET_CORE_ENABLE_INTERRUPTS		0x1

/* Register data type */
struct arc_reg_data_type {
	struct list_head list;
	struct reg_data_type data_type;
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
	void *arch_info;

	struct arc_jtag jtag_info;

	struct reg_cache *core_cache;

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	int bp_scanned;

	/* Actionpoints */
	unsigned int actionpoints_num;
	unsigned int actionpoints_num_avail;
	struct arc_comparator *actionpoints_list;

	/* Cache control */
	bool has_dcache;
	/* If true, then D$ has been already flushed since core has been
	 * halted. */
	bool dcache_flushed;
	/* If true, then caches have been already flushed since core has been
	 * halted. */
	bool cache_invalidated;

	/* Whether DEBUG.SS bit is present. This is a unique feature of ARC 600. */
	bool has_debug_ss;

	/* Workaround for a problem with ARC 600 - writing RA | IS | SS will not
	 * step an instruction - it must be only a (IS | SS). However RA is set by
	 * the processor itself and since OpenOCD does read-modify-write of DEBUG
	 * register when stepping, it is required to explicitly disable RA before
	 * stepping. */
	bool on_step_reset_debug_ra;

	/* CCM memory regions (optional). */
	uint32_t iccm0_start;
	uint32_t iccm0_end;
	uint32_t iccm1_start;
	uint32_t iccm1_end;
	uint32_t dccm_start;
	uint32_t dccm_end;

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

static inline struct arc_common * target_to_arc(struct target *target)
{
	return target->arch_info;
}


/* ARC Register description */
struct arc_reg_desc {
	/* Register name */
	char *name;

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

struct arc_reg_t {
	struct arc_reg_desc *desc;
	struct target *target;
	struct arc_common *arc_common;
	uint32_t value;
};

const struct reg_arch_type arc_reg_type;

/* GDB register groups. For now we suport only general and "empty" */
static const char * const reg_group_general = "general";
static const char * const reg_group_other = "";

/* Error codes */
#define ERROR_ARC_REGISTER_NOT_FOUND       (-700)
#define ERROR_ARC_REGISTER_FIELD_NOT_FOUND (-701)
#define ERROR_ARC_REGISTER_IS_NOT_STRUCT   (-702)
#define ERROR_ARC_FIELD_IS_NOT_BITFIELD    (-703)
#define ERROR_ARC_REGTYPE_NOT_FOUND        (-704)

/* ----- Exported functions ------------------------------------------------ */
int arc_init_arch_info(struct target *target, struct arc_common *arc,
	struct jtag_tap *tap);

/* Configurable registers functions */
void arc_add_reg_data_type(struct target *target,
               struct arc_reg_data_type *data_type);

int arc_add_reg(struct target *target, struct arc_reg_desc *arc_reg,
               const char * const type_name, const size_t type_name_len);

int arc_build_reg_cache(struct target *target);
int arc_build_bcr_reg_cache(struct target *target);

struct reg *arc_register_get_by_name(struct reg_cache *first,
					const char *name, bool search_all);

/* Get value of field in struct register */
int arc_get_register_field(struct target *target, const char *reg_name,
		const char *field_name, uint32_t *value_ptr);
/* Get value of 32-bit register. */
int arc_get_register_value(struct target *target, const char *reg_name,
		uint32_t * value_ptr);
/* Set value of 32-bit register. */
int arc_set_register_value(struct target *target, const char *reg_name,
		uint32_t value);

int arc_examine(struct target *target);
int arc_halt(struct target *target);
int arc_poll(struct target *target);
int arc_assert_reset(struct target *target);
int arc_deassert_reset(struct target *target);
int arc_arch_state(struct target *target);
int arc_enable_interrupts(struct target *target, int enable);
int arc_resume(struct target *target, int current, target_addr_t address,
	int handle_breakpoints, int debug_execution);

int arc_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
	int *reg_list_size, enum target_register_class reg_class);

#endif /* ARC_H */
