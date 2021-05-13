/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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

#ifndef OPENOCD_TARGET_TARGET_TYPE_H
#define OPENOCD_TARGET_TARGET_TYPE_H

#include <jim-nvp.h>

struct target;

/**
 * This holds methods shared between all instances of a given target
 * type.  For example, all Cortex-M3 targets on a scan chain share
 * the same method table.
 */
struct target_type {
	/**
	 * Name of this type of target.  Do @b not access this
	 * field directly, use target_type_name() instead.
	 */
	const char *name;

	/* poll current target status */
	int (*poll)(struct target *target);
	/* Invoked only from target_arch_state().
	 * Issue USER() w/architecture specific status.  */
	int (*arch_state)(struct target *target);

	/* target request support */
	int (*target_request_data)(struct target *target, uint32_t size, uint8_t *buffer);

	/* halt will log a warning, but return ERROR_OK if the target is already halted. */
	int (*halt)(struct target *target);
	/* See target.c target_resume() for documentation. */
	int (*resume)(struct target *target, int current, target_addr_t address,
			int handle_breakpoints, int debug_execution);
	int (*step)(struct target *target, int current, target_addr_t address,
			int handle_breakpoints);
	/* target reset control. assert reset can be invoked when OpenOCD and
	 * the target is out of sync.
	 *
	 * A typical example is that the target was power cycled while OpenOCD
	 * thought the target was halted or running.
	 *
	 * assert_reset() can therefore make no assumptions whatsoever about the
	 * state of the target
	 *
	 * Before assert_reset() for the target is invoked, a TRST/tms and
	 * chain validation is executed. TRST should not be asserted
	 * during target assert unless there is no way around it due to
	 * the way reset's are configured.
	 *
	 */
	int (*assert_reset)(struct target *target);
	/**
	 * The implementation is responsible for polling the
	 * target such that target->state reflects the
	 * state correctly.
	 *
	 * Otherwise the following would fail, as there will not
	 * be any "poll" invoked inbetween the "reset run" and
	 * "halt".
	 *
	 * reset run; halt
	 */
	int (*deassert_reset)(struct target *target);
	int (*soft_reset_halt)(struct target *target);

	/**
	 * Target architecture for GDB.
	 *
	 * The string returned by this function will not be automatically freed;
	 * if dynamic allocation is used for this value, it must be managed by
	 * the target, ideally by caching the result for subsequent calls.
	 */
	const char *(*get_gdb_arch)(struct target *target);

	/**
	 * Target register access for GDB.  Do @b not call this function
	 * directly, use target_get_gdb_reg_list() instead.
	 *
	 * Danger! this function will succeed even if the target is running
	 * and return a register list with dummy values.
	 *
	 * The reason is that GDB connection will fail without a valid register
	 * list, however it is after GDB is connected that monitor commands can
	 * be run to properly initialize the target
	 */
	int (*get_gdb_reg_list)(struct target *target, struct reg **reg_list[],
			int *reg_list_size, enum target_register_class reg_class);

	/**
	 * Same as get_gdb_reg_list, but doesn't read the register values.
	 * */
	int (*get_gdb_reg_list_noread)(struct target *target,
			struct reg **reg_list[], int *reg_list_size,
			enum target_register_class reg_class);

	/* target memory access
	* size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
	* count: number of items of <size>
	*/

	/**
	 * Target memory read callback.  Do @b not call this function
	 * directly, use target_read_memory() instead.
	 */
	int (*read_memory)(struct target *target, target_addr_t address,
			uint32_t size, uint32_t count, uint8_t *buffer);
	/**
	 * Target memory write callback.  Do @b not call this function
	 * directly, use target_write_memory() instead.
	 */
	int (*write_memory)(struct target *target, target_addr_t address,
			uint32_t size, uint32_t count, const uint8_t *buffer);

	/* Default implementation will do some fancy alignment to improve performance, target can override */
	int (*read_buffer)(struct target *target, target_addr_t address,
			uint32_t size, uint8_t *buffer);

	/* Default implementation will do some fancy alignment to improve performance, target can override */
	int (*write_buffer)(struct target *target, target_addr_t address,
			uint32_t size, const uint8_t *buffer);

	int (*checksum_memory)(struct target *target, target_addr_t address,
			uint32_t count, uint32_t *checksum);
	int (*blank_check_memory)(struct target *target,
			struct target_memory_check_block *blocks, int num_blocks,
			uint8_t erased_value);

	/*
	 * target break-/watchpoint control
	 * rw: 0 = write, 1 = read, 2 = access
	 *
	 * Target must be halted while this is invoked as this
	 * will actually set up breakpoints on target.
	 *
	 * The breakpoint hardware will be set up upon adding the
	 * first breakpoint.
	 *
	 * Upon GDB connection all breakpoints/watchpoints are cleared.
	 */
	int (*add_breakpoint)(struct target *target, struct breakpoint *breakpoint);
	int (*add_context_breakpoint)(struct target *target, struct breakpoint *breakpoint);
	int (*add_hybrid_breakpoint)(struct target *target, struct breakpoint *breakpoint);

	/* remove breakpoint. hw will only be updated if the target
	 * is currently halted.
	 * However, this method can be invoked on unresponsive targets.
	 */
	int (*remove_breakpoint)(struct target *target, struct breakpoint *breakpoint);

	/* add watchpoint ... see add_breakpoint() comment above. */
	int (*add_watchpoint)(struct target *target, struct watchpoint *watchpoint);

	/* remove watchpoint. hw will only be updated if the target
	 * is currently halted.
	 * However, this method can be invoked on unresponsive targets.
	 */
	int (*remove_watchpoint)(struct target *target, struct watchpoint *watchpoint);

	/* Find out just hit watchpoint. After the target hits a watchpoint, the
	 * information could assist gdb to locate where the modified/accessed memory is.
	 */
	int (*hit_watchpoint)(struct target *target, struct watchpoint **hit_watchpoint);

	/**
	 * Target algorithm support.  Do @b not call this method directly,
	 * use target_run_algorithm() instead.
	 */
	int (*run_algorithm)(struct target *target, int num_mem_params,
			struct mem_param *mem_params, int num_reg_params,
			struct reg_param *reg_param, target_addr_t entry_point,
			target_addr_t exit_point, int timeout_ms, void *arch_info);
	int (*start_algorithm)(struct target *target, int num_mem_params,
			struct mem_param *mem_params, int num_reg_params,
			struct reg_param *reg_param, target_addr_t entry_point,
			target_addr_t exit_point, void *arch_info);
	int (*wait_algorithm)(struct target *target, int num_mem_params,
			struct mem_param *mem_params, int num_reg_params,
			struct reg_param *reg_param, target_addr_t exit_point,
			int timeout_ms, void *arch_info);

	const struct command_registration *commands;

	/* called when target is created */
	int (*target_create)(struct target *target, Jim_Interp *interp);

	/* called for various config parameters */
	/* returns JIM_CONTINUE - if option not understood */
	/* otherwise: JIM_OK, or JIM_ERR, */
	int (*target_jim_configure)(struct target *target, Jim_GetOptInfo *goi);

	/* target commands specifically handled by the target */
	/* returns JIM_OK, or JIM_ERR, or JIM_CONTINUE - if option not understood */
	int (*target_jim_commands)(struct target *target, Jim_GetOptInfo *goi);

	/**
	 * This method is used to perform target setup that requires
	 * JTAG access.
	 *
	 * This may be called multiple times.  It is called after the
	 * scan chain is initially validated, or later after the target
	 * is enabled by a JRC.  It may also be called during some
	 * parts of the reset sequence.
	 *
	 * For one-time initialization tasks, use target_was_examined()
	 * and target_set_examined().  For example, probe the hardware
	 * before setting up chip-specific state, and then set that
	 * flag so you don't do that again.
	 */
	int (*examine)(struct target *target);

	/* Set up structures for target.
	 *
	 * It is illegal to talk to the target at this stage as this fn is invoked
	 * before the JTAG chain has been examined/verified
	 * */
	int (*init_target)(struct command_context *cmd_ctx, struct target *target);

	/**
	 * Free all the resources allocated by the target.
	 *
	 * @param target The target to deinit
	 */
	void (*deinit_target)(struct target *target);

	/* translate from virtual to physical address. Default implementation is successful
	 * no-op(i.e. virtual==physical).
	 */
	int (*virt2phys)(struct target *target, target_addr_t address, target_addr_t *physical);

	/* read directly from physical memory. caches are bypassed and untouched.
	 *
	 * If the target does not support disabling caches, leaving them untouched,
	 * then minimally the actual physical memory location will be read even
	 * if cache states are unchanged, flushed, etc.
	 *
	 * Default implementation is to call read_memory.
	 */
	int (*read_phys_memory)(struct target *target, target_addr_t phys_address,
			uint32_t size, uint32_t count, uint8_t *buffer);

	/*
	 * same as read_phys_memory, except that it writes...
	 */
	int (*write_phys_memory)(struct target *target, target_addr_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer);

	int (*mmu)(struct target *target, int *enabled);

	/* after reset is complete, the target can check if things are properly set up.
	 *
	 * This can be used to check if e.g. DCC memory writes have been enabled for
	 * arm7/9 targets, which they really should except in the most contrived
	 * circumstances.
	 */
	int (*check_reset)(struct target *target);

	/* get GDB file-I/O parameters from target
	 */
	int (*get_gdb_fileio_info)(struct target *target, struct gdb_fileio_info *fileio_info);

	/* pass GDB file-I/O response to target
	 */
	int (*gdb_fileio_end)(struct target *target, int retcode, int fileio_errno, bool ctrl_c);

	/* do target profiling
	 */
	int (*profiling)(struct target *target, uint32_t *samples,
			uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds);

	/* Return the number of address bits this target supports. This will
	 * typically be 32 for 32-bit targets, and 64 for 64-bit targets. If not
	 * implemented, it's assumed to be 32. */
	unsigned (*address_bits)(struct target *target);

	/* Return the number of system bus data bits this target supports. This
	 * will typically be 32 for 32-bit targets, and 64 for 64-bit targets. If
	 * not implemented, it's assumed to be 32. */
	unsigned (*data_bits)(struct target *target);
};

#endif /* OPENOCD_TARGET_TARGET_TYPE_H */
