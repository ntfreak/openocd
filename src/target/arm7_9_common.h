/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by Hongtao Zheng                                   *
 *   hontor@126.com                                                        *
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

#ifndef OPENOCD_TARGET_ARM7_9_COMMON_H
#define OPENOCD_TARGET_ARM7_9_COMMON_H

#include "arm.h"
#include "arm_jtag.h"

#define	ARM7_9_COMMON_MAGIC 0x0a790a79 /**< */

/**
 * Structure for items that are common between both ARM7 and ARM9 targets.
 */
struct arm7_9_common {
	struct arm arm;
	uint32_t common_magic;

	struct arm_jtag jtag_info; /**< JTAG information for target */
	struct reg_cache *eice_cache; /**< Embedded ICE register cache */

	uint32_t arm_bkpt; /**< ARM breakpoint instruction */
	uint16_t thumb_bkpt; /**< Thumb breakpoint instruction */

	int sw_breakpoints_added; /**< Specifies which watchpoint software breakpoints are setup on */
	int sw_breakpoint_count; /**< keep track of number of software breakpoints we have set */
	int breakpoint_count; /**< Current number of set breakpoints */
	int wp_available; /**< Current number of available watchpoint units */
	int wp_available_max; /**< Maximum number of available watchpoint units */
	int wp0_used; /**< Specifies if and how watchpoint unit 0 is used */
	int wp1_used; /**< Specifies if and how watchpoint unit 1 is used */
	int wp1_used_default; /**< Specifies if and how watchpoint unit 1 is used by default */
	int dbgreq_adjust_pc; /**< Amount of PC adjustment caused by a DBGREQ */
	bool use_dbgrq; /**< Specifies if DBGRQ should be used to halt the target */
	bool need_bypass_before_restart; /**< Specifies if there should be a bypass before a JTAG restart */

	bool has_single_step;
	bool has_monitor_mode;
	bool has_vector_catch; /**< Specifies if the target has a reset vector catch */

	bool debug_entry_from_reset; /**< Specifies if debug entry was from a reset */

	bool fast_memory_access;
	bool dcc_downloads;

	struct working_area *dcc_working_area;

	int (*examine_debug_reason)(struct target *target);
	/**< Function for determining why debug state was entered */

	void (*change_to_arm)(struct target *target, uint32_t *r0, uint32_t *pc);
	/**< Function for changing from Thumb to ARM mode */

	void (*read_core_regs)(struct target *target, uint32_t mask, uint32_t *core_regs[16]);
	/**< Function for reading the core registers */

	void (*read_core_regs_target_buffer)(struct target *target, uint32_t mask,
			void *buffer, int size);
	void (*read_xpsr)(struct target *target, uint32_t *xpsr, int spsr);
	/**< Function for reading CPSR or SPSR */

	void (*write_xpsr)(struct target *target, uint32_t xpsr, int spsr);
	/**< Function for writing to CPSR or SPSR */

	void (*write_xpsr_im8)(struct target *target, uint8_t xpsr_im, int rot, int spsr);
	/**< Function for writing an immediate value to CPSR or SPSR */

	void (*write_core_regs)(struct target *target, uint32_t mask, uint32_t core_regs[16]);

	void (*load_word_regs)(struct target *target, uint32_t mask);
	void (*load_hword_reg)(struct target *target, int num);
	void (*load_byte_reg)(struct target *target, int num);

	void (*store_word_regs)(struct target *target, uint32_t mask);
	void (*store_hword_reg)(struct target *target, int num);
	void (*store_byte_reg)(struct target *target, int num);

	void (*write_pc)(struct target *target, uint32_t pc);
	/**< Function for writing to the program counter */

	void (*branch_resume)(struct target *target);
	void (*branch_resume_thumb)(struct target *target);

	void (*enable_single_step)(struct target *target, uint32_t next_pc);
	void (*disable_single_step)(struct target *target);

	void (*set_special_dbgrq)(struct target *target);
	/**< Function for setting DBGRQ if the normal way won't work */

	int (*post_debug_entry)(struct target *target);
	/**< Callback function called after entering debug mode */

	void (*pre_restore_context)(struct target *target);
	/**< Callback function called before restoring the processor context */

	/**
	 * Variant specific memory write function that does not dispatch to bulk_write_memory.
	 * Used as a fallback when bulk writes are unavailable, or for writing data needed to
	 * do the bulk writes.
	 */
	int (*write_memory)(struct target *target, target_addr_t address,
			uint32_t size, uint32_t count, const uint8_t *buffer);
	/**
	 * Write target memory in multiples of 4 bytes, optimized for
	 * writing large quantities of data.
	 */
	int (*bulk_write_memory)(struct target *target, target_addr_t address,
			uint32_t count, const uint8_t *buffer);
};

static inline struct arm7_9_common *target_to_arm7_9(struct target *target)
{
	return container_of(target->arch_info, struct arm7_9_common, arm);
}

static inline bool is_arm7_9(struct arm7_9_common *arm7_9)
{
	return arm7_9->common_magic == ARM7_9_COMMON_MAGIC;
}

extern const struct command_registration arm7_9_command_handlers[];

int arm7_9_poll(struct target *target);

int arm7_9_target_request_data(struct target *target, uint32_t size, uint8_t *buffer);

int arm7_9_assert_reset(struct target *target);
int arm7_9_deassert_reset(struct target *target);
int arm7_9_reset_request_halt(struct target *target);
int arm7_9_early_halt(struct target *target);
int arm7_9_soft_reset_halt(struct target *target);

int arm7_9_halt(struct target *target);
int arm7_9_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution);
int arm7_9_step(struct target *target, int current, target_addr_t address,
		int handle_breakpoints);
int arm7_9_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
int arm7_9_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
int arm7_9_write_memory_opt(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
int arm7_9_write_memory_no_opt(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
int arm7_9_bulk_write_memory(struct target *target, target_addr_t address,
		uint32_t count, const uint8_t *buffer);

int arm7_9_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_prams,
		struct reg_param *reg_param, uint32_t entry_point, void *arch_info);

int arm7_9_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int arm7_9_remove_breakpoint(struct target *target, struct breakpoint *breakpoint);
int arm7_9_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int arm7_9_remove_watchpoint(struct target *target, struct watchpoint *watchpoint);

void arm7_9_enable_eice_step(struct target *target, uint32_t next_pc);
void arm7_9_disable_eice_step(struct target *target);

int arm7_9_execute_sys_speed(struct target *target);

int arm7_9_init_arch_info(struct target *target, struct arm7_9_common *arm7_9);
int arm7_9_examine(struct target *target);
void arm7_9_deinit(struct target *target);
int arm7_9_check_reset(struct target *target);

int arm7_9_endianness_callback(jtag_callback_data_t pu8_in,
		jtag_callback_data_t i_size, jtag_callback_data_t i_be,
		jtag_callback_data_t i_flip);

#endif /* OPENOCD_TARGET_ARM7_9_COMMON_H */
