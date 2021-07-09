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
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011                                     *
 *   michel.jaouen@stericsson.com : smp minimum support                    *
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

#ifndef OPENOCD_TARGET_TARGET_H
#define OPENOCD_TARGET_TARGET_H

#include <helper/list.h>
#include "helper/replacements.h"
#include "helper/system.h"
#include <jim.h>

struct reg;
struct trace;
struct command_context;
struct command_invocation;
struct breakpoint;
struct watchpoint;
struct mem_param;
struct reg_param;
struct target_list;
struct gdb_fileio_info;

/*
 * TARGET_UNKNOWN = 0: we don't know anything about the target yet
 * TARGET_RUNNING = 1: the target is executing or ready to execute user code
 * TARGET_HALTED  = 2: the target is not executing code, and ready to talk to the
 * debugger. on an xscale it means that the debug handler is executing
 * TARGET_RESET   = 3: the target is being held in reset (only a temporary state,
 * not sure how this is used with all the recent changes)
 * TARGET_DEBUG_RUNNING = 4: the target is running, but it is executing code on
 * behalf of the debugger (e.g. algorithm for flashing)
 *
 * also see: target_state_name();
 */

enum target_state {
	TARGET_UNKNOWN = 0,
	TARGET_RUNNING = 1,
	TARGET_HALTED = 2,
	TARGET_RESET = 3,
	TARGET_DEBUG_RUNNING = 4,
};

enum nvp_assert {
	NVP_DEASSERT,
	NVP_ASSERT,
};

enum target_reset_mode {
	RESET_UNKNOWN = 0,
	RESET_RUN = 1,		/* reset and let target run */
	RESET_HALT = 2,		/* reset and halt target out of reset */
	RESET_INIT = 3,		/* reset and halt target out of reset, then run init script */
};

enum target_debug_reason {
	DBG_REASON_DBGRQ = 0,
	DBG_REASON_BREAKPOINT = 1,
	DBG_REASON_WATCHPOINT = 2,
	DBG_REASON_WPTANDBKPT = 3,
	DBG_REASON_SINGLESTEP = 4,
	DBG_REASON_NOTHALTED = 5,
	DBG_REASON_EXIT = 6,
	DBG_REASON_EXC_CATCH = 7,
	DBG_REASON_UNDEFINED = 8,
};

enum target_endianness {
	TARGET_ENDIAN_UNKNOWN = 0,
	TARGET_BIG_ENDIAN = 1, TARGET_LITTLE_ENDIAN = 2
};

struct working_area {
	target_addr_t address;
	uint32_t size;
	bool free;
	uint8_t *backup;
	struct working_area **user;
	struct working_area *next;
};

struct gdb_service {
	struct target *target;
	/*  field for smp display  */
	/*  element 0 coreid currently displayed ( 1 till n) */
	/*  element 1 coreid to be displayed at next resume 1 till n 0 means resume
	 *  all cores core displayed  */
	int32_t core[2];
};

/* target back off timer */
struct backoff_timer {
	int times;
	int count;
};

/* split target registers into multiple class */
enum target_register_class {
	REG_CLASS_ALL,
	REG_CLASS_GENERAL,
};

/* target_type.h contains the full definition of struct target_type */
struct target {
	struct target_type *type;			/* target type definition (name, access functions) */
	char *cmd_name;				/* tcl Name of target */
	int target_number;					/* DO NOT USE!  field to be removed in 2010 */
	struct jtag_tap *tap;				/* where on the jtag chain is this */
	int32_t coreid;						/* which device on the TAP? */

	/** Should we defer examine to later */
	bool defer_examine;

	/**
	 * Indicates whether this target has been examined.
	 *
	 * Do @b not access this field directly, use target_was_examined()
	 * or target_set_examined().
	 */
	bool examined;

	/**
	 * true if the  target is currently running a downloaded
	 * "algorithm" instead of arbitrary user code. OpenOCD code
	 * invoking algorithms is trusted to maintain correctness of
	 * any cached state (e.g. for flash status), which arbitrary
	 * code will have no reason to know about.
	 */
	bool running_alg;

	struct target_event_action *event_action;

	bool reset_halt;						/* attempt resetting the CPU into the halted mode? */
	target_addr_t working_area;				/* working area (initialised RAM). Evaluated
										 * upon first allocation from virtual/physical address. */
	bool working_area_virt_spec;		/* virtual address specified? */
	target_addr_t working_area_virt;			/* virtual address */
	bool working_area_phys_spec;		/* physical address specified? */
	target_addr_t working_area_phys;			/* physical address */
	uint32_t working_area_size;			/* size in bytes */
	uint32_t backup_working_area;		/* whether the content of the working area has to be preserved */
	struct working_area *working_areas;/* list of allocated working areas */
	enum target_debug_reason debug_reason;/* reason why the target entered debug state */
	enum target_endianness endianness;	/* target endianness */
	/* also see: target_state_name() */
	enum target_state state;			/* the current backend-state (running, halted, ...) */
	struct reg_cache *reg_cache;		/* the first register cache of the target (core regs) */
	struct breakpoint *breakpoints;		/* list of breakpoints */
	struct watchpoint *watchpoints;		/* list of watchpoints */
	struct trace *trace_info;			/* generic trace information */
	struct debug_msg_receiver *dbgmsg;	/* list of debug message receivers */
	uint32_t dbg_msg_enabled;			/* debug message status */
	void *arch_info;					/* architecture specific information */
	void *private_config;				/* pointer to target specific config data (for jim_configure hook) */
	struct target *next;				/* next target in list */

	bool verbose_halt_msg;				/* display async info in telnet session. Do not display
										 * lots of halted/resumed info when stepping in debugger. */
	bool halt_issued;					/* did we transition to halted state? */
	int64_t halt_issued_time;			/* Note time when halt was issued */

										/* ARM v7/v8 targets with ADIv5 interface */
	bool dbgbase_set;					/* By default the debug base is not set */
	uint32_t dbgbase;					/* Really a Cortex-A specific option, but there is no
										 * system in place to support target specific options
										 * currently. */
	bool has_dap;						/* set to true if target has ADIv5 support */
	bool dap_configured;				/* set to true if ADIv5 DAP is configured */
	bool tap_configured;				/* set to true if JTAG tap has been configured
										 * through -chain-position */

	struct rtos *rtos;					/* Instance of Real Time Operating System support */
	bool rtos_auto_detect;				/* A flag that indicates that the RTOS has been specified as "auto"
										 * and must be detected when symbols are offered */
	struct backoff_timer backoff;
	int smp;							/* add some target attributes for smp support */
	struct target_list *head;
	/* the gdb service is there in case of smp, we have only one gdb server
	 * for all smp target
	 * the target attached to the gdb is changing dynamically by changing
	 * gdb_service->target pointer */
	struct gdb_service *gdb_service;

	/* file-I/O information for host to do syscall */
	struct gdb_fileio_info *fileio_info;

	char *gdb_port_override;			/* target-specific override for gdb_port */

	int gdb_max_connections;			/* max number of simultaneous gdb connections */

	/* The semihosting information, extracted from the target. */
	struct semihosting *semihosting;
};

struct target_list {
	struct target *target;
	struct target_list *next;
};

struct gdb_fileio_info {
	char *identifier;
	uint64_t param_1;
	uint64_t param_2;
	uint64_t param_3;
	uint64_t param_4;
};

/** Returns a description of the endianness for the specified target. */
static inline const char *target_endianness(struct target *target)
{
	return (target->endianness == TARGET_ENDIAN_UNKNOWN) ? "unknown" :
			(target->endianness == TARGET_BIG_ENDIAN) ? "big endian" : "little endian";
}

/** Returns the instance-specific name of the specified target. */
static inline const char *target_name(struct target *target)
{
	return target->cmd_name;
}

const char *debug_reason_name(struct target *t);

enum target_event {

	/* allow GDB to do stuff before others handle the halted event,
	 * this is in lieu of defining ordering of invocation of events,
	 * which would be more complicated
	 *
	 * Telling GDB to halt does not mean that the target stopped running,
	 * simply that we're dropping out of GDB's waiting for step or continue.
	 *
	 * This can be useful when e.g. detecting power dropout.
	 */
	TARGET_EVENT_GDB_HALT,
	TARGET_EVENT_HALTED,		/* target entered debug state from normal execution or reset */
	TARGET_EVENT_RESUMED,		/* target resumed to normal execution */
	TARGET_EVENT_RESUME_START,
	TARGET_EVENT_RESUME_END,
	TARGET_EVENT_STEP_START,
	TARGET_EVENT_STEP_END,

	TARGET_EVENT_GDB_START, /* debugger started execution (step/run) */
	TARGET_EVENT_GDB_END, /* debugger stopped execution (step/run) */

	TARGET_EVENT_RESET_START,
	TARGET_EVENT_RESET_ASSERT_PRE,
	TARGET_EVENT_RESET_ASSERT,	/* C code uses this instead of SRST */
	TARGET_EVENT_RESET_ASSERT_POST,
	TARGET_EVENT_RESET_DEASSERT_PRE,
	TARGET_EVENT_RESET_DEASSERT_POST,
	TARGET_EVENT_RESET_INIT,
	TARGET_EVENT_RESET_END,

	TARGET_EVENT_DEBUG_HALTED,	/* target entered debug state, but was executing on behalf of the debugger */
	TARGET_EVENT_DEBUG_RESUMED, /* target resumed to execute on behalf of the debugger */

	TARGET_EVENT_EXAMINE_START,
	TARGET_EVENT_EXAMINE_FAIL,
	TARGET_EVENT_EXAMINE_END,

	TARGET_EVENT_GDB_ATTACH,
	TARGET_EVENT_GDB_DETACH,

	TARGET_EVENT_GDB_FLASH_ERASE_START,
	TARGET_EVENT_GDB_FLASH_ERASE_END,
	TARGET_EVENT_GDB_FLASH_WRITE_START,
	TARGET_EVENT_GDB_FLASH_WRITE_END,

	TARGET_EVENT_TRACE_CONFIG,
};

struct target_event_action {
	enum target_event event;
	Jim_Interp *interp;
	Jim_Obj *body;
	struct target_event_action *next;
};

bool target_has_event_action(struct target *target, enum target_event event);

struct target_event_callback {
	int (*callback)(struct target *target, enum target_event event, void *priv);
	void *priv;
	struct target_event_callback *next;
};

struct target_reset_callback {
	struct list_head list;
	void *priv;
	int (*callback)(struct target *target, enum target_reset_mode reset_mode, void *priv);
};

struct target_trace_callback {
	struct list_head list;
	void *priv;
	int (*callback)(struct target *target, size_t len, uint8_t *data, void *priv);
};

enum target_timer_type {
	TARGET_TIMER_TYPE_ONESHOT,
	TARGET_TIMER_TYPE_PERIODIC
};

struct target_timer_callback {
	int (*callback)(void *priv);
	unsigned int time_ms;
	enum target_timer_type type;
	bool removed;
	int64_t when;	/* output of timeval_ms() */
	void *priv;
	struct target_timer_callback *next;
};

struct target_memory_check_block {
	target_addr_t address;
	uint32_t size;
	uint32_t result;
};

int target_register_commands(struct command_context *cmd_ctx);
int target_examine(void);

int target_register_event_callback(
		int (*callback)(struct target *target,
		enum target_event event, void *priv),
		void *priv);
int target_unregister_event_callback(
		int (*callback)(struct target *target,
		enum target_event event, void *priv),
		void *priv);

int target_register_reset_callback(
		int (*callback)(struct target *target,
		enum target_reset_mode reset_mode, void *priv),
		void *priv);
int target_unregister_reset_callback(
		int (*callback)(struct target *target,
		enum target_reset_mode reset_mode, void *priv),
		void *priv);

int target_register_trace_callback(
		int (*callback)(struct target *target,
		size_t len, uint8_t *data, void *priv),
		void *priv);
int target_unregister_trace_callback(
		int (*callback)(struct target *target,
		size_t len, uint8_t *data, void *priv),
		void *priv);

/* Poll the status of the target, detect any error conditions and report them.
 *
 * Also note that this fn will clear such error conditions, so a subsequent
 * invocation will then succeed.
 *
 * These error conditions can be "sticky" error conditions. E.g. writing
 * to memory could be implemented as an open loop and if memory writes
 * fails, then a note is made of it, the error is sticky, but the memory
 * write loop still runs to completion. This improves performance in the
 * normal case as there is no need to verify that every single write succeed,
 * yet it is possible to detect error conditions.
 */
int target_poll(struct target *target);
int target_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution);
int target_halt(struct target *target);
int target_call_event_callbacks(struct target *target, enum target_event event);
int target_call_reset_callbacks(struct target *target, enum target_reset_mode reset_mode);
int target_call_trace_callbacks(struct target *target, size_t len, uint8_t *data);

/**
 * The period is very approximate, the callback can happen much more often
 * or much more rarely than specified
 */
int target_register_timer_callback(int (*callback)(void *priv),
		unsigned int time_ms, enum target_timer_type type, void *priv);
int target_unregister_timer_callback(int (*callback)(void *priv), void *priv);
int target_call_timer_callbacks(void);
/**
 * Invoke this to ensure that e.g. polling timer callbacks happen before
 * a synchronous command completes.
 */
int target_call_timer_callbacks_now(void);
/**
 * Returns when the next registered event will take place. Callers can use this
 * to go to sleep until that time occurs.
 */
int64_t target_timer_next_event(void);

struct target *get_target_by_num(int num);
struct target *get_current_target(struct command_context *cmd_ctx);
struct target *get_current_target_or_null(struct command_context *cmd_ctx);
struct target *get_target(const char *id);

/**
 * Get the target type name.
 *
 * This routine is a wrapper for the target->type->name field.
 * Note that this is not an instance-specific name for his target.
 */
const char *target_type_name(struct target *target);

/**
 * Examine the specified @a target, letting it perform any
 * Initialisation that requires JTAG access.
 *
 * This routine is a wrapper for target->type->examine.
 */
int target_examine_one(struct target *target);

/** @returns @c true if target_set_examined() has been called. */
static inline bool target_was_examined(struct target *target)
{
	return target->examined;
}

/** Sets the @c examined flag for the given target. */
/** Use in target->type->examine() after one-time setup is done. */
static inline void target_set_examined(struct target *target)
{
	target->examined = true;
}

/**
 * Add the @a breakpoint for @a target.
 *
 * This routine is a wrapper for target->type->add_breakpoint.
 */
int target_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
/**
 * Add the @a ContextID breakpoint  for @a target.
 *
 * This routine is a wrapper for target->type->add_context_breakpoint.
 */
int target_add_context_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
/**
 * Add the @a ContextID & IVA breakpoint  for @a target.
 *
 * This routine is a wrapper for target->type->add_hybrid_breakpoint.
 */
int target_add_hybrid_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
/**
 * Remove the @a breakpoint for @a target.
 *
 * This routine is a wrapper for target->type->remove_breakpoint.
 */

int target_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
/**
 * Add the @a watchpoint for @a target.
 *
 * This routine is a wrapper for target->type->add_watchpoint.
 */
int target_add_watchpoint(struct target *target,
		struct watchpoint *watchpoint);
/**
 * Remove the @a watchpoint for @a target.
 *
 * This routine is a wrapper for target->type->remove_watchpoint.
 */
int target_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint);

/**
 * Find out the just hit @a watchpoint for @a target.
 *
 * This routine is a wrapper for target->type->hit_watchpoint.
 */
int target_hit_watchpoint(struct target *target,
		struct watchpoint **watchpoint);

/**
 * Obtain the architecture for GDB.
 *
 * This routine is a wrapper for target->type->get_gdb_arch.
 */
const char *target_get_gdb_arch(struct target *target);

/**
 * Obtain the registers for GDB.
 *
 * This routine is a wrapper for target->type->get_gdb_reg_list.
 */
int target_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);

/**
 * Obtain the registers for GDB, but don't read register values from the
 * target.
 *
 * This routine is a wrapper for target->type->get_gdb_reg_list_noread.
 */
int target_get_gdb_reg_list_noread(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);

/**
 * Check if @a target allows GDB connections.
 *
 * Some target do not implement the necessary code required by GDB.
 */
bool target_supports_gdb_connection(struct target *target);

/**
 * Step the target.
 *
 * This routine is a wrapper for target->type->step.
 */
int target_step(struct target *target,
		int current, target_addr_t address, int handle_breakpoints);
/**
 * Run an algorithm on the @a target given.
 *
 * This routine is a wrapper for target->type->run_algorithm.
 */
int target_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_param,
		uint32_t entry_point, uint32_t exit_point,
		int timeout_ms, void *arch_info);

/**
 * Starts an algorithm in the background on the @a target given.
 *
 * This routine is a wrapper for target->type->start_algorithm.
 */
int target_start_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		uint32_t entry_point, uint32_t exit_point,
		void *arch_info);

/**
 * Wait for an algorithm on the @a target given.
 *
 * This routine is a wrapper for target->type->wait_algorithm.
 */
int target_wait_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		uint32_t exit_point, int timeout_ms,
		void *arch_info);

/**
 * This routine is a wrapper for asynchronous algorithms.
 *
 */
int target_run_flash_async_algorithm(struct target *target,
		const uint8_t *buffer, uint32_t count, int block_size,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		uint32_t buffer_start, uint32_t buffer_size,
		uint32_t entry_point, uint32_t exit_point,
		void *arch_info);

/**
 * This routine is a wrapper for asynchronous algorithms.
 *
 */
int target_run_read_async_algorithm(struct target *target,
		uint8_t *buffer, uint32_t count, int block_size,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		uint32_t buffer_start, uint32_t buffer_size,
		uint32_t entry_point, uint32_t exit_point,
		void *arch_info);

/**
 * Read @a count items of @a size bytes from the memory of @a target at
 * the @a address given.
 *
 * This routine is a wrapper for target->type->read_memory.
 */
int target_read_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer);
int target_read_phys_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer);
/**
 * Write @a count items of @a size bytes to the memory of @a target at
 * the @a address given. @a address must be aligned to @a size
 * in target memory.
 *
 * The endianness is the same in the host and target memory for this
 * function.
 *
 * \todo TODO:
 * Really @a buffer should have been defined as "const void *" and
 * @a buffer should have been aligned to @a size in the host memory.
 *
 * This is not enforced via e.g. assert's today and e.g. the
 * target_write_buffer fn breaks this assumption.
 *
 * This routine is wrapper for target->type->write_memory.
 */
int target_write_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
int target_write_phys_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer);

/*
 * Write to target memory using the virtual address.
 *
 * Note that this fn is used to implement software breakpoints. Targets
 * can implement support for software breakpoints to memory marked as read
 * only by making this fn write to ram even if it is read only(MMU or
 * MPUs).
 *
 * It is sufficient to implement for writing a single word(16 or 32 in
 * ARM32/16 bit case) to write the breakpoint to ram.
 *
 * The target should also take care of "other things" to make sure that
 * software breakpoints can be written using this function. E.g.
 * when there is a separate instruction and data cache, this fn must
 * make sure that the instruction cache is synced up to the potential
 * code change that can happen as a result of the memory write(typically
 * by invalidating the cache).
 *
 * The high level wrapper fn in target.c will break down this memory write
 * request to multiple write requests to the target driver to e.g. guarantee
 * that writing 4 bytes to an aligned address happens with a single 32 bit
 * write operation, thus making this fn suitable to e.g. write to special
 * peripheral registers which do not support byte operations.
 */
int target_write_buffer(struct target *target,
		target_addr_t address, uint32_t size, const uint8_t *buffer);
int target_read_buffer(struct target *target,
		target_addr_t address, uint32_t size, uint8_t *buffer);
int target_checksum_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t *crc);
int target_blank_check_memory(struct target *target,
		struct target_memory_check_block *blocks, int num_blocks,
		uint8_t erased_value);
int target_wait_state(struct target *target, enum target_state state, int ms);

/**
 * Obtain file-I/O information from target for GDB to do syscall.
 *
 * This routine is a wrapper for target->type->get_gdb_fileio_info.
 */
int target_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info);

/**
 * Pass GDB file-I/O response to target after finishing host syscall.
 *
 * This routine is a wrapper for target->type->gdb_fileio_end.
 */
int target_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c);

/**
 * Return the highest accessible address for this target.
 */
target_addr_t target_address_max(struct target *target);

/**
 * Return the number of address bits this target supports.
 *
 * This routine is a wrapper for target->type->address_bits.
 */
unsigned target_address_bits(struct target *target);

/**
 * Return the number of data bits this target supports.
 *
 * This routine is a wrapper for target->type->data_bits.
 */
unsigned int target_data_bits(struct target *target);

/** Return the *name* of this targets current state */
const char *target_state_name(struct target *target);

/** Return the *name* of a target event enumeration value */
const char *target_event_name(enum target_event event);

/** Return the *name* of a target reset reason enumeration value */
const char *target_reset_mode_name(enum target_reset_mode reset_mode);

/* DANGER!!!!!
 *
 * if "area" passed in to target_alloc_working_area() points to a memory
 * location that goes out of scope (e.g. a pointer on the stack), then
 * the caller of target_alloc_working_area() is responsible for invoking
 * target_free_working_area() before "area" goes out of scope.
 *
 * target_free_all_working_areas() will NULL out the "area" pointer
 * upon resuming or resetting the CPU.
 *
 */
int target_alloc_working_area(struct target *target,
		uint32_t size, struct working_area **area);
/* Same as target_alloc_working_area, except that no error is logged
 * when ERROR_TARGET_RESOURCE_NOT_AVAILABLE is returned.
 *
 * This allows the calling code to *try* to allocate target memory
 * and have a fallback to another behaviour(slower?).
 */
int target_alloc_working_area_try(struct target *target,
		uint32_t size, struct working_area **area);
int target_free_working_area(struct target *target, struct working_area *area);
void target_free_all_working_areas(struct target *target);
uint32_t target_get_working_area_avail(struct target *target);

/**
 * Free all the resources allocated by targets and the target layer
 */
void target_quit(void);

extern struct target *all_targets;

uint64_t target_buffer_get_u64(struct target *target, const uint8_t *buffer);
uint32_t target_buffer_get_u32(struct target *target, const uint8_t *buffer);
uint32_t target_buffer_get_u24(struct target *target, const uint8_t *buffer);
uint16_t target_buffer_get_u16(struct target *target, const uint8_t *buffer);
void target_buffer_set_u64(struct target *target, uint8_t *buffer, uint64_t value);
void target_buffer_set_u32(struct target *target, uint8_t *buffer, uint32_t value);
void target_buffer_set_u24(struct target *target, uint8_t *buffer, uint32_t value);
void target_buffer_set_u16(struct target *target, uint8_t *buffer, uint16_t value);

void target_buffer_get_u64_array(struct target *target, const uint8_t *buffer, uint32_t count, uint64_t *dstbuf);
void target_buffer_get_u32_array(struct target *target, const uint8_t *buffer, uint32_t count, uint32_t *dstbuf);
void target_buffer_get_u16_array(struct target *target, const uint8_t *buffer, uint32_t count, uint16_t *dstbuf);
void target_buffer_set_u64_array(struct target *target, uint8_t *buffer, uint32_t count, const uint64_t *srcbuf);
void target_buffer_set_u32_array(struct target *target, uint8_t *buffer, uint32_t count, const uint32_t *srcbuf);
void target_buffer_set_u16_array(struct target *target, uint8_t *buffer, uint32_t count, const uint16_t *srcbuf);

int target_read_u64(struct target *target, target_addr_t address, uint64_t *value);
int target_read_u32(struct target *target, target_addr_t address, uint32_t *value);
int target_read_u16(struct target *target, target_addr_t address, uint16_t *value);
int target_read_u8(struct target *target, target_addr_t address, uint8_t *value);
int target_write_u64(struct target *target, target_addr_t address, uint64_t value);
int target_write_u32(struct target *target, target_addr_t address, uint32_t value);
int target_write_u16(struct target *target, target_addr_t address, uint16_t value);
int target_write_u8(struct target *target, target_addr_t address, uint8_t value);

int target_write_phys_u64(struct target *target, target_addr_t address, uint64_t value);
int target_write_phys_u32(struct target *target, target_addr_t address, uint32_t value);
int target_write_phys_u16(struct target *target, target_addr_t address, uint16_t value);
int target_write_phys_u8(struct target *target, target_addr_t address, uint8_t value);

/* Issues USER() statements with target state information */
int target_arch_state(struct target *target);

void target_handle_event(struct target *t, enum target_event e);

void target_handle_md_output(struct command_invocation *cmd,
	struct target *target, target_addr_t address, unsigned size,
	unsigned count, const uint8_t *buffer);

int target_profiling_default(struct target *target, uint32_t *samples, uint32_t
		max_num_samples, uint32_t *num_samples, uint32_t seconds);

#define ERROR_TARGET_INVALID	(-300)
#define ERROR_TARGET_INIT_FAILED (-301)
#define ERROR_TARGET_TIMEOUT	(-302)
#define ERROR_TARGET_NOT_HALTED (-304)
#define ERROR_TARGET_FAILURE	(-305)
#define ERROR_TARGET_UNALIGNED_ACCESS	(-306)
#define ERROR_TARGET_DATA_ABORT	(-307)
#define ERROR_TARGET_RESOURCE_NOT_AVAILABLE	(-308)
#define ERROR_TARGET_TRANSLATION_FAULT	(-309)
#define ERROR_TARGET_NOT_RUNNING (-310)
#define ERROR_TARGET_NOT_EXAMINED (-311)
#define ERROR_TARGET_DUPLICATE_BREAKPOINT (-312)
#define ERROR_TARGET_ALGO_EXIT  (-313)

extern bool get_target_reset_nag(void);

#define TARGET_DEFAULT_POLLING_INTERVAL		100

#endif /* OPENOCD_TARGET_TARGET_H */
