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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "nds32_reg.h"
#include "nds32_disassembler.h"
#include "nds32.h"
#include "nds32_aice.h"
#include "nds32_v3_common.h"

static struct nds32_v3_common_callback *v3_common_callback;

static int nds32_v3_register_mapping(struct nds32 *nds32, int reg_no)
{
	if (reg_no == PC)
		return IR11;

	return reg_no;
}

static int nds32_v3_get_debug_reason(struct nds32 *nds32, uint32_t *reason)
{
	uint32_t edmsw;
	struct aice_port_s *aice = target_to_aice(nds32->target);
	aice_read_debug_reg(aice, NDS_EDM_SR_EDMSW, &edmsw);

	*reason = (edmsw >> 12) & 0x0F;

	return ERROR_OK;
}

/**
 * Save processor state.  This is called after a HALT instruction
 * succeeds, and on other occasions the processor enters debug mode
 * (breakpoint, watchpoint, etc).
 */
static int nds32_v3_debug_entry(struct nds32 *nds32, bool enable_watchpoint)
{
	LOG_DEBUG("nds32_v3_debug_entry");

	enum target_state backup_state = nds32->target->state;
	nds32->target->state = TARGET_HALTED;

	if (nds32->init_arch_info_after_halted == false) {
		/* init architecture info according to config registers */
		CHECK_RETVAL(nds32_config(nds32));

		nds32->init_arch_info_after_halted = true;
	}

	/* REVISIT entire cache should already be invalid !!! */
	register_cache_invalidate(nds32->core_cache);

	/* deactivate all hardware breakpoints */
	CHECK_RETVAL(v3_common_callback->deactivate_hardware_breakpoint(nds32->target));

	if (enable_watchpoint)
		CHECK_RETVAL(v3_common_callback->deactivate_hardware_watchpoint(nds32->target));

	struct breakpoint *syscall_break = &(nds32->syscall_break);
	if (nds32->virtual_hosting) {
		if (syscall_break->set) {
			/** disable virtual hosting */

			/* remove breakpoint at syscall entry */
			target_remove_breakpoint(nds32->target, syscall_break);
			syscall_break->set = 0;

			uint32_t value_pc;
			nds32_get_mapped_reg(nds32, PC, &value_pc);
			if (value_pc == syscall_break->address)
				/** process syscall for virtual hosting */
				nds32->hit_syscall = true;
		}
	}

	if (nds32_examine_debug_reason(nds32) != ERROR_OK) {
		nds32->target->state = backup_state;

		/* re-activate all hardware breakpoints & watchpoints */
		CHECK_RETVAL(v3_common_callback->activate_hardware_breakpoint(nds32->target));

		if (enable_watchpoint)
			CHECK_RETVAL(v3_common_callback->activate_hardware_watchpoint(nds32->target));

		return ERROR_FAIL;
	}

	/* Save registers. */
	nds32_full_context(nds32);

	/* check interrupt level */
	v3_common_callback->check_interrupt_stack(nds32);

	return ERROR_OK;
}

/**
 * Restore processor state.
 */
static int nds32_v3_leave_debug_state(struct nds32 *nds32, bool enable_watchpoint)
{
	LOG_DEBUG("nds32_v3_leave_debug_state");

	struct target *target = nds32->target;

	/* activate all hardware breakpoints */
	CHECK_RETVAL(v3_common_callback->activate_hardware_breakpoint(target));

	if (enable_watchpoint) {
		/* activate all watchpoints */
		CHECK_RETVAL(v3_common_callback->activate_hardware_watchpoint(target));
	}

	/* restore interrupt stack */
	v3_common_callback->restore_interrupt_stack(nds32);

	/* REVISIT once we start caring about MMU and cache state,
	 * address it here ...
	 */

	/* restore PSW, PC, and R0 ... after flushing any modified
	 * registers.
	 */
	CHECK_RETVAL(nds32_restore_context(target));

	if (nds32->virtual_hosting) {
		/** enable virtual hosting */
		uint32_t value_ir3;
		uint32_t entry_size;
		uint32_t syscall_address;

		/* get syscall entry address */
		nds32_get_mapped_reg(nds32, IR3, &value_ir3);
		entry_size = 0x4 << (((value_ir3 >> 14) & 0x3) << 1);
		syscall_address = (value_ir3 & 0xFFFF0000) + entry_size * 8; /* The index of SYSCALL is 8 */

		if (nds32->hit_syscall) {
			/* single step to skip syscall entry */
			/* use IRET to skip syscall */
			struct aice_port_s *aice = target_to_aice(target);
			uint32_t value_ir9;
			uint32_t value_ir6;
			uint32_t syscall_id;

			nds32_get_mapped_reg(nds32, IR6, &value_ir6);
			syscall_id = (value_ir6 >> 16) & 0x7FFF;

			if (syscall_id == NDS32_SYSCALL_EXIT) {
				/* If target hits exit syscall, do not use IRET to skip handler. */
				aice_step(aice);
			} else {
				/* use api->read/write_reg to skip nds32 register cache */
				uint32_t value_dimbr;
				aice_read_debug_reg(aice, NDS_EDM_SR_DIMBR, &value_dimbr);
				aice_write_register(aice, IR11, value_dimbr + 0xC);

				aice_read_register(aice, IR9, &value_ir9);
				value_ir9 += 4; /* syscall is always 4 bytes */
				aice_write_register(aice, IR9, value_ir9);

				/* backup hardware breakpoint 0 */
				uint32_t backup_bpa, backup_bpam, backup_bpc;
				aice_read_debug_reg(aice, NDS_EDM_SR_BPA0, &backup_bpa);
				aice_read_debug_reg(aice, NDS_EDM_SR_BPAM0, &backup_bpam);
				aice_read_debug_reg(aice, NDS_EDM_SR_BPC0, &backup_bpc);

				/* use hardware breakpoint 0 to stop cpu after skipping syscall */
				aice_write_debug_reg(aice, NDS_EDM_SR_BPA0, value_ir9);
				aice_write_debug_reg(aice, NDS_EDM_SR_BPAM0, 0);
				aice_write_debug_reg(aice, NDS_EDM_SR_BPC0, 0xA);

				/* Execute two IRET.
				 * First IRET is used to quit debug mode.
				 * Second IRET is used to quit current syscall. */
				uint32_t dim_inst[4] = {NOP, NOP, IRET, IRET};
				aice_execute(aice, dim_inst, 4);

				/* restore origin hardware breakpoint 0 */
				aice_write_debug_reg(aice, NDS_EDM_SR_BPA0, backup_bpa);
				aice_write_debug_reg(aice, NDS_EDM_SR_BPAM0, backup_bpam);
				aice_write_debug_reg(aice, NDS_EDM_SR_BPC0, backup_bpc);
			}

			nds32->hit_syscall = false;
		}

		/* insert breakpoint at syscall entry */
		struct breakpoint *syscall_break = &(nds32->syscall_break);

		syscall_break->address = syscall_address;
		syscall_break->type = BKPT_SOFT;
		syscall_break->set = 1;
		target_add_breakpoint(target, syscall_break);
	}

	return ERROR_OK;
}

static int nds32_v3_get_exception_address(struct nds32 *nds32,
		uint32_t *address, uint32_t reason)
{
	LOG_DEBUG("nds32_v3_get_exception_address");

	struct aice_port_s *aice = target_to_aice(nds32->target);
	struct target *target = nds32->target;
	uint32_t edmsw;
	uint32_t edm_cfg;
	uint32_t match_bits;
	uint32_t match_count;
	int32_t i;
	static int32_t number_of_hard_break;
	uint32_t bp_control;

	if (number_of_hard_break == 0) {
		aice_read_debug_reg(aice, NDS_EDM_SR_EDM_CFG, &edm_cfg);
		number_of_hard_break = (edm_cfg & 0x7) + 1;
	}

	aice_read_debug_reg(aice, NDS_EDM_SR_EDMSW, &edmsw);
	/* clear matching bits (write-one-clear) */
	aice_write_debug_reg(aice, NDS_EDM_SR_EDMSW, edmsw);
	match_bits = (edmsw >> 4) & 0xFF;
	match_count = 0;
	for (i = 0 ; i < number_of_hard_break ; i++) {
		if (match_bits & (1 << i)) {
			aice_read_debug_reg(aice, NDS_EDM_SR_BPA0 + i, address);
			match_count++;

			/* If target hits multiple read/access watchpoint,
			 * select the first one. */
			aice_read_debug_reg(aice, NDS_EDM_SR_BPC0 + i, &bp_control);
			if (0x3 == (bp_control & 0x3)) {
				match_count = 1;
				break;
			}
		}
	}

	if (match_count > 1) { /* multiple hits */
		*address = 0;
		return ERROR_OK;
	} else if (match_count == 1) {
		uint32_t val_pc;
		uint32_t opcode;
		struct nds32_instruction instruction;
		struct watchpoint *wp;
		bool hit;

		nds32_get_mapped_reg(nds32, PC, &val_pc);

		if ((reason == NDS32_DEBUG_DATA_ADDR_WATCHPOINT_NEXT_PRECISE) ||
				(reason == NDS32_DEBUG_DATA_VALUE_WATCHPOINT_NEXT_PRECISE)) {
			if (edmsw & 0x4) /* check EDMSW.IS_16BIT */
				val_pc -= 2;
			else
				val_pc -= 4;
		}

		nds32_read_opcode(nds32, val_pc, &opcode);
		nds32_evaluate_opcode(nds32, opcode, val_pc, &instruction);

		LOG_DEBUG("PC: 0x%08" PRIx32 ", access start: 0x%08" PRIx32 ", end: 0x%08" PRIx32,
				val_pc, instruction.access_start, instruction.access_end);

		/* check if multiple hits in the access range */
		uint32_t in_range_watch_count = 0;
		for (wp = target->watchpoints; wp; wp = wp->next) {
			if ((instruction.access_start <= wp->address) &&
					(wp->address < instruction.access_end))
				in_range_watch_count++;
		}
		if (in_range_watch_count > 1) {
			/* Hit LSMW instruction. */
			*address = 0;
			return ERROR_OK;
		}

		/* dispel false match */
		hit = false;
		for (wp = target->watchpoints; wp; wp = wp->next) {
			if (((*address ^ wp->address) & (~wp->mask)) == 0) {
				uint32_t watch_start;
				uint32_t watch_end;

				watch_start = wp->address;
				watch_end = wp->address + wp->length;

				if ((watch_end <= instruction.access_start) ||
						(instruction.access_end <= watch_start))
					continue;

				hit = true;
				break;
			}
		}

		if (hit)
			return ERROR_OK;
		else
			return ERROR_FAIL;
	} else if (match_count == 0) {
		/* global stop is precise exception */
		if ((reason == NDS32_DEBUG_LOAD_STORE_GLOBAL_STOP) && nds32->global_stop) {
			/* parse instruction to get correct access address */
			uint32_t val_pc;
			uint32_t opcode;
			struct nds32_instruction instruction;

			nds32_get_mapped_reg(nds32, PC, &val_pc);
			nds32_read_opcode(nds32, val_pc, &opcode);
			nds32_evaluate_opcode(nds32, opcode, val_pc, &instruction);

			*address = instruction.access_start;

			return ERROR_OK;
		}
	}

	*address = 0xFFFFFFFF;
	return ERROR_FAIL;
}

void nds32_v3_common_register_callback(struct nds32_v3_common_callback *callback)
{
	v3_common_callback = callback;
}

/** target_type functions: */
/* target request support */
int nds32_v3_target_request_data(struct target *target,
		uint32_t size, uint8_t *buffer)
{
	/* AndesCore could use DTR register to communicate with OpenOCD
	 * to output messages
	 * Target data will be put in buffer
	 * The format of DTR is as follow
	 * DTR[31:16] => length, DTR[15:8] => size, DTR[7:0] => target_req_cmd
	 * target_req_cmd has three possible values:
	 *   TARGET_REQ_TRACEMSG
	 *   TARGET_REQ_DEBUGMSG
	 *   TARGET_REQ_DEBUGCHAR
	 * if size == 0, target will call target_asciimsg(),
	 * else call target_hexmsg()
	 */
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_OK;
}

int nds32_v3_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count, uint32_t *checksum)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

/**
 * find out which watchpoint hits
 * get exception address and compare the address to watchpoints
 */
int nds32_v3_hit_watchpoint(struct target *target,
		struct watchpoint **hit_watchpoint)
{
	static struct watchpoint scan_all_watchpoint;

	uint32_t exception_address;
	struct watchpoint *wp;
	struct nds32 *nds32 = target_to_nds32(target);

	exception_address = nds32->watched_address;

	if (exception_address == 0xFFFFFFFF)
		return ERROR_FAIL;

	if (exception_address == 0) {
		scan_all_watchpoint.address = 0;
		scan_all_watchpoint.rw = WPT_WRITE;
		scan_all_watchpoint.next = 0;
		scan_all_watchpoint.unique_id = 0x5CA8;

		*hit_watchpoint = &scan_all_watchpoint;
		return ERROR_OK;
	}

	for (wp = target->watchpoints; wp; wp = wp->next) {
		if (((exception_address ^ wp->address) & (~wp->mask)) == 0) {
			*hit_watchpoint = wp;

			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

int nds32_v3_target_create_common(struct target *target, struct nds32 *nds32)
{
	nds32->register_map = nds32_v3_register_mapping;
	nds32->get_debug_reason = nds32_v3_get_debug_reason;
	nds32->enter_debug_state = nds32_v3_debug_entry;
	nds32->leave_debug_state = nds32_v3_leave_debug_state;
	nds32->get_watched_address = nds32_v3_get_exception_address;

	/* Init target->arch_info in nds32_init_arch_info().
	 * After this, user could use target_to_nds32() to get nds32 object */
	nds32_init_arch_info(target, nds32);

	return ERROR_OK;
}

int nds32_v3_run_algorithm(struct target *target,
		int num_mem_params,
		struct mem_param *mem_params,
		int num_reg_params,
		struct reg_param *reg_params,
		target_addr_t entry_point,
		target_addr_t exit_point,
		int timeout_ms,
		void *arch_info)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

int nds32_v3_read_buffer(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	if ((memory->access_channel == NDS_MEMORY_ACC_CPU) &&
			(target->state != TARGET_HALTED)) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_addr_t physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	if (target->type->virt2phys(target, address, &physical_address) == ERROR_OK)
		address = physical_address;
	else
		return ERROR_FAIL;

	int result;
	struct aice_port_s *aice = target_to_aice(target);
	/* give arbitrary initial value to avoid warning messages */
	enum nds_memory_access origin_access_channel = NDS_MEMORY_ACC_CPU;

	if (nds32->hit_syscall) {
		/* Use bus mode to access memory during virtual hosting */
		origin_access_channel = memory->access_channel;
		memory->access_channel = NDS_MEMORY_ACC_BUS;
		aice_memory_access(aice, NDS_MEMORY_ACC_BUS);
	}

	result = nds32_read_buffer(target, address, size, buffer);

	if (nds32->hit_syscall) {
		/* Restore access_channel after virtual hosting */
		memory->access_channel = origin_access_channel;
		aice_memory_access(aice, origin_access_channel);
	}

	return result;
}

int nds32_v3_write_buffer(struct target *target, target_addr_t address,
		uint32_t size, const uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	if ((memory->access_channel == NDS_MEMORY_ACC_CPU) &&
			(target->state != TARGET_HALTED)) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_addr_t physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	if (target->type->virt2phys(target, address, &physical_address) == ERROR_OK)
		address = physical_address;
	else
		return ERROR_FAIL;

	if (nds32->hit_syscall) {
		struct aice_port_s *aice = target_to_aice(target);
		enum nds_memory_access origin_access_channel;
		origin_access_channel = memory->access_channel;

		/* If target has no cache, use BUS mode to access memory. */
		if ((memory->dcache.line_size == 0)
			|| (memory->dcache.enable == false)) {
			/* There is no Dcache or Dcache is disabled. */
			memory->access_channel = NDS_MEMORY_ACC_BUS;
			aice_memory_access(aice, NDS_MEMORY_ACC_BUS);
		}

		int result;
		result = nds32_gdb_fileio_write_memory(nds32, address, size, buffer);

		if (origin_access_channel == NDS_MEMORY_ACC_CPU) {
			memory->access_channel = NDS_MEMORY_ACC_CPU;
			aice_memory_access(aice, NDS_MEMORY_ACC_CPU);
		}

		return result;
	}

	return nds32_write_buffer(target, address, size, buffer);
}

int nds32_v3_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	if ((memory->access_channel == NDS_MEMORY_ACC_CPU) &&
			(target->state != TARGET_HALTED)) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_addr_t physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	if (target->type->virt2phys(target, address, &physical_address) == ERROR_OK)
		address = physical_address;
	else
		return ERROR_FAIL;

	struct aice_port_s *aice = target_to_aice(target);
	/* give arbitrary initial value to avoid warning messages */
	enum nds_memory_access origin_access_channel = NDS_MEMORY_ACC_CPU;
	int result;

	if (nds32->hit_syscall) {
		/* Use bus mode to access memory during virtual hosting */
		origin_access_channel = memory->access_channel;
		memory->access_channel = NDS_MEMORY_ACC_BUS;
		aice_memory_access(aice, NDS_MEMORY_ACC_BUS);
	}

	result = nds32_read_memory(target, address, size, count, buffer);

	if (nds32->hit_syscall) {
		/* Restore access_channel after virtual hosting */
		memory->access_channel = origin_access_channel;
		aice_memory_access(aice, origin_access_channel);
	}

	return result;
}

int nds32_v3_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	if ((memory->access_channel == NDS_MEMORY_ACC_CPU) &&
			(target->state != TARGET_HALTED)) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_addr_t physical_address;
	/* BUG: If access range crosses multiple pages, the translation will not correct
	 * for second page or so. */

	/* When DEX is set to one, hardware will enforce the following behavior without
	 * modifying the corresponding control bits in PSW.
	 *
	 * Disable all interrupts
	 * Become superuser mode
	 * Turn off IT/DT
	 * Use MMU_CFG.DE as the data access endian
	 * Use MMU_CFG.DRDE as the device register access endian if MMU_CTL.DREE is asserted
	 * Disable audio special features
	 * Disable inline function call
	 *
	 * Because hardware will turn off IT/DT by default, it MUST translate virtual address
	 * to physical address.
	 */
	if (target->type->virt2phys(target, address, &physical_address) == ERROR_OK)
		address = physical_address;
	else
		return ERROR_FAIL;

	return nds32_write_memory(target, address, size, count, buffer);
}

int nds32_v3_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	/* Initialize anything we can set up without talking to the target */
	struct nds32 *nds32 = target_to_nds32(target);

	nds32_init(nds32);

	target->fileio_info = malloc(sizeof(struct gdb_fileio_info));
	target->fileio_info->identifier = NULL;

	return ERROR_OK;
}
