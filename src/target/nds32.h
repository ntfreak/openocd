/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#ifndef OPENOCD_TARGET_NDS32_H
#define OPENOCD_TARGET_NDS32_H

#include <jtag/jtag.h>
#include "target.h"
#include "target_type.h"
#include "register.h"
#include "breakpoints.h"
#include "nds32_reg.h"
#include "nds32_insn.h"
#include "nds32_edm.h"

#define NDS32_EDM_OPERATION_MAX_NUM 64

#define CHECK_RETVAL(action)			\
	do {					\
		int __retval = (action);	\
		if (__retval != ERROR_OK) {	\
			LOG_DEBUG("error while calling \"%s\"",	\
				# action);     \
			return __retval;	\
		}				\
	} while (0)

/**
 * @file
 * Holds the interface to Andes cores.
 */

extern const char *nds32_debug_type_name[11];

enum nds32_debug_reason {
	NDS32_DEBUG_BREAK = 0,
	NDS32_DEBUG_BREAK_16,
	NDS32_DEBUG_INST_BREAK,
	NDS32_DEBUG_DATA_ADDR_WATCHPOINT_PRECISE,
	NDS32_DEBUG_DATA_VALUE_WATCHPOINT_PRECISE,
	NDS32_DEBUG_DATA_VALUE_WATCHPOINT_IMPRECISE,
	NDS32_DEBUG_DEBUG_INTERRUPT,
	NDS32_DEBUG_HARDWARE_SINGLE_STEP,
	NDS32_DEBUG_DATA_ADDR_WATCHPOINT_NEXT_PRECISE,
	NDS32_DEBUG_DATA_VALUE_WATCHPOINT_NEXT_PRECISE,
	NDS32_DEBUG_LOAD_STORE_GLOBAL_STOP,
};

#define NDS32_STRUCT_STAT_SIZE 60
#define NDS32_STRUCT_TIMEVAL_SIZE 8

enum nds32_syscall_id {
	NDS32_SYSCALL_UNDEFINED = 0,
	NDS32_SYSCALL_EXIT = 1,
	NDS32_SYSCALL_OPEN = 2,
	NDS32_SYSCALL_CLOSE = 3,
	NDS32_SYSCALL_READ = 4,
	NDS32_SYSCALL_WRITE = 5,
	NDS32_SYSCALL_LSEEK = 6,
	NDS32_SYSCALL_UNLINK = 7,
	NDS32_SYSCALL_RENAME = 3001,
	NDS32_SYSCALL_FSTAT = 10,
	NDS32_SYSCALL_STAT = 15,
	NDS32_SYSCALL_GETTIMEOFDAY = 19,
	NDS32_SYSCALL_ISATTY = 3002,
	NDS32_SYSCALL_SYSTEM = 3003,
	NDS32_SYSCALL_ERRNO = 6001,
};

#define NDS32_COMMON_MAGIC 0xADE5ADE5U

struct nds32_edm {

	/** EDM_CFG.VER, indicate the EDM version */
	int version;

	/** The number of hardware breakpoints */
	int breakpoint_num;

	/** EDM_CFG.DALM, indicate if direct local memory access
	 * feature is supported or not */
	bool direct_access_local_memory;

	/** Support ACC_CTL register */
	bool access_control;

	/** */
	bool support_max_stop;
};

struct nds32_cache {

	/** enable cache or not */
	bool enable;

	/** cache sets per way */
	int set;

	/** cache ways */
	int way;

	/** cache line size */
	int line_size;

	/** cache locking support */
	bool lock_support;
};

struct nds32_memory {

	/** ICache */
	struct nds32_cache icache;

	/** DCache */
	struct nds32_cache dcache;

	/** On-chip instruction local memory base */
	int ilm_base;

	/** On-chip instruction local memory size */
	int ilm_size;

	/** ILM base register alignment version */
	int ilm_align_ver;

	/** DLM is enabled or not */
	bool ilm_enable;

	/** DLM start address */
	int ilm_start;

	/** DLM end address */
	int ilm_end;

	/** On-chip data local memory base */
	int dlm_base;

	/** On-chip data local memory size */
	int dlm_size;

	/** DLM base register alignment version */
	int dlm_align_ver;

	/** DLM is enabled or not */
	bool dlm_enable;

	/** DLM start address */
	int dlm_start;

	/** DLM end address */
	int dlm_end;

	/** Memory access method */
	enum nds_memory_access access_channel;

	/** Memory access mode */
	enum nds_memory_select mode;

	/** Address translation */
	bool address_translation;
};

struct nds32_cpu_version {
	bool performance_extension;
	bool _16bit_extension;
	bool performance_extension_2;
	bool cop_fpu_extension;
	bool string_extension;

	int revision;
	int cpu_id_family;
	int cpu_id_version;
};

struct nds32_mmu_config {
	int memory_protection;
	int memory_protection_version;
	bool fully_associative_tlb;
	int tlb_size;
	int tlb_ways;
	int tlb_sets;
	bool _8k_page_support;
	int extra_page_size_support;
	bool tlb_lock;
	bool hardware_page_table_walker;
	bool default_endian;
	int partition_num;
	bool invisible_tlb;
	bool vlpt;
	bool ntme;
	bool drde;
	int default_min_page_size;
	bool multiple_page_size_in_use;
};

struct nds32_misc_config {
	bool edm;
	bool local_memory_dma;
	bool performance_monitor;
	bool high_speed_memory_port;
	bool debug_tracer;
	bool div_instruction;
	bool mac_instruction;
	int audio_isa;
	bool l2_cache;
	bool reduce_register;
	bool addr_24;
	bool interruption_level;
	int baseline_instruction;
	bool no_dx_register;
	bool implement_dependant_register;
	bool implement_dependant_sr_encoding;
	bool ifc;
	bool mcu;
	bool ex9;
	int shadow;
};

/**
 * Represents a generic Andes core.
 */
struct nds32 {
	uint32_t common_magic;
	struct reg_cache *core_cache;

	/** Handle for the debug module. */
	struct nds32_edm edm;

	/** Memory information */
	struct nds32_memory memory;

	/** cpu version */
	struct nds32_cpu_version cpu_version;

	/** MMU configuration */
	struct nds32_mmu_config mmu_config;

	/** Misc configuration */
	struct nds32_misc_config misc_config;

	/** Retrieve all core registers, for display. */
	int (*full_context)(struct nds32 *nds32);

	/** Register mappings */
	int (*register_map)(struct nds32 *nds32, int reg_no);

	/** Get debug exception virtual address */
	int (*get_debug_reason)(struct nds32 *nds32, uint32_t *reason);

	/** Restore target registers may be modified in debug state */
	int (*leave_debug_state)(struct nds32 *nds32, bool enable_watchpoint);

	/** Backup target registers may be modified in debug state */
	int (*enter_debug_state)(struct nds32 *nds32, bool enable_watchpoint);

	/** Get address hit watchpoint */
	int (*get_watched_address)(struct nds32 *nds32, uint32_t *address, uint32_t reason);

	/** maximum interrupt level */
	uint32_t max_interrupt_level;

	/** current interrupt level */
	uint32_t current_interrupt_level;

	uint32_t watched_address;

	/** Flag reporting whether virtual hosting is active. */
	bool virtual_hosting;

	/** Flag reporting whether continue/step hits syscall or not */
	bool hit_syscall;

	/** Value to be returned by virtual hosting SYS_ERRNO request. */
	int virtual_hosting_errno;

	/** Flag reporting whether syscall is aborted */
	bool virtual_hosting_ctrl_c;

	/** Record syscall ID for other operations to do special processing for target */
	int active_syscall_id;

	struct breakpoint syscall_break;

	/** Flag reporting whether global stop is active. */
	bool global_stop;

	/** Flag reporting whether to use soft-reset-halt or not as issuing reset-halt. */
	bool soft_reset_halt;

	/** reset-halt as target examine */
	bool reset_halt_as_examine;

	/** backup/restore target EDM_CTL value. As debugging target debug
	 * handler, it should be true. */
	bool keep_target_edm_ctl;

	/* Value of $EDM_CTL before target enters debug mode */
	uint32_t backup_edm_ctl;

	/** always use word-aligned address to access memory */
	bool word_access_mem;

	/** EDM passcode for debugging secure MCU */
	char *edm_passcode;

	/** current privilege_level if using secure MCU. value 0 is the highest level.  */
	int privilege_level;

	/** Period to wait after SRST. */
	uint32_t boot_time;

	/** Flag to indicate HSS steps into ISR or not */
	bool step_isr_enable;

	/** Flag to indicate register table is ready or not */
	bool init_arch_info_after_halted;

	/** Flag to indicate audio-extension is enabled or not */
	bool audio_enable;

	/** Flag to indicate fpu-extension is enabled or not */
	bool fpu_enable;

	/* Andes Core has mixed endian model. Instruction is always big-endian.
	 * Data may be big or little endian. Device registers may have different
	 * endian from data and instruction. */
	/** Endian of data memory */
	enum target_endianness data_endian;

	/** Endian of device registers */
	enum target_endianness device_reg_endian;

	/** Flag to indicate if auto convert software breakpoints to
	 *  hardware breakpoints or not in ROM */
	bool auto_convert_hw_bp;

	/* Flag to indicate the target is attached by debugger or not */
	bool attached;

	/** Backpointer to the target. */
	struct target *target;

	void *arch_info;
};

struct nds32_reg {
	int32_t num;
	uint8_t value[8];
	struct target *target;
	struct nds32 *nds32;
	bool enable;
};

struct nds32_edm_operation {
	uint32_t reg_no;
	uint32_t value;
};

extern int nds32_config(struct nds32 *nds32);
extern int nds32_init_arch_info(struct target *target, struct nds32 *nds32);
extern int nds32_full_context(struct nds32 *nds32);
extern int nds32_arch_state(struct target *target);
extern int nds32_add_software_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
extern int nds32_remove_software_breakpoint(struct target *target,
		struct breakpoint *breakpoint);

extern int nds32_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);

extern int nds32_write_buffer(struct target *target, uint32_t address,
		uint32_t size, const uint8_t *buffer);
extern int nds32_read_buffer(struct target *target, uint32_t address,
		uint32_t size, uint8_t *buffer);
extern int nds32_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
extern int nds32_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);

extern int nds32_init_register_table(struct nds32 *nds32);
extern int nds32_init_memory_info(struct nds32 *nds32);
extern int nds32_restore_context(struct target *target);
extern int nds32_get_mapped_reg(struct nds32 *nds32, unsigned regnum, uint32_t *value);
extern int nds32_set_mapped_reg(struct nds32 *nds32, unsigned regnum, uint32_t value);

extern int nds32_edm_config(struct nds32 *nds32);
extern int nds32_cache_sync(struct target *target, target_addr_t address, uint32_t length);
extern int nds32_mmu(struct target *target, int *enabled);
extern int nds32_virtual_to_physical(struct target *target, target_addr_t address,
		target_addr_t *physical);
extern int nds32_read_phys_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
extern int nds32_write_phys_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
extern uint32_t nds32_nextpc(struct nds32 *nds32, int current, uint32_t address);
extern int nds32_examine_debug_reason(struct nds32 *nds32);
extern int nds32_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints);
extern int nds32_target_state(struct nds32 *nds32, enum target_state *state);
extern int nds32_halt(struct target *target);
extern int nds32_poll(struct target *target);
extern int nds32_resume(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution);
extern int nds32_assert_reset(struct target *target);
extern int nds32_init(struct nds32 *nds32);
extern int nds32_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info);
extern int nds32_gdb_fileio_write_memory(struct nds32 *nds32, uint32_t address,
		uint32_t size, const uint8_t *buffer);
extern int nds32_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c);
extern int nds32_reset_halt(struct nds32 *nds32);
extern int nds32_login(struct nds32 *nds32);
extern int nds32_profiling(struct target *target, uint32_t *samples,
			uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds);

/** Convert target handle to generic Andes target state handle. */
static inline struct nds32 *target_to_nds32(struct target *target)
{
	assert(target);
	return target->arch_info;
}

/** */
static inline struct aice_port_s *target_to_aice(struct target *target)
{
	assert(target);
	return target->tap->priv;
}

static inline bool is_nds32(struct nds32 *nds32)
{
	assert(nds32);
	return nds32->common_magic == NDS32_COMMON_MAGIC;
}

static inline bool nds32_reach_max_interrupt_level(struct nds32 *nds32)
{
	assert(nds32);
	return nds32->max_interrupt_level == nds32->current_interrupt_level;
}

#endif /* OPENOCD_TARGET_NDS32_H */
