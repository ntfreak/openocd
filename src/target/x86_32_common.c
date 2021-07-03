/*
 * Copyright(c) 2013 Intel Corporation.
 *
 * Adrian Burns (adrian.burns@intel.com)
 * Thomas Faust (thomas.faust@intel.com)
 * Ivan De Cesaris (ivan.de.cesaris@intel.com)
 * Julien Carreno (julien.carreno@intel.com)
 * Jeffrey Maxwell (jeffrey.r.maxwell@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information:
 * Intel Corporation
 */

/*
 * @file
 * This implements generic x86 32 bit memory and breakpoint operations.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>

#include "target.h"
#include "target_type.h"
#include "register.h"
#include "breakpoints.h"
#include "x86_32_common.h"

static int set_debug_regs(struct target *t, uint32_t address,
			uint8_t bp_num, uint8_t bp_type, uint8_t bp_length);
static int unset_debug_regs(struct target *t, uint8_t bp_num);
static int read_mem(struct target *t, uint32_t size,
			uint32_t addr, uint8_t *buf);
static int write_mem(struct target *t, uint32_t size,
			uint32_t addr, const uint8_t *buf);
static int calcaddr_physfromlin(struct target *t, target_addr_t addr,
			target_addr_t *physaddr);
static int read_phys_mem(struct target *t, uint32_t phys_address,
			uint32_t size, uint32_t count, uint8_t *buffer);
static int write_phys_mem(struct target *t, uint32_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer);
static int set_breakpoint(struct target *target,
			struct breakpoint *breakpoint);
static int unset_breakpoint(struct target *target,
			struct breakpoint *breakpoint);
static int set_watchpoint(struct target *target,
			struct watchpoint *watchpoint);
static int unset_watchpoint(struct target *target,
			struct watchpoint *watchpoint);
static int read_hw_reg_to_cache(struct target *t, int num);
static int write_hw_reg_from_cache(struct target *t, int num);

int x86_32_get_gdb_reg_list(struct target *t,
			struct reg **reg_list[], int *reg_list_size,
			enum target_register_class reg_class)
{

	struct x86_32_common *x86_32 = target_to_x86_32(t);
	int i;
	*reg_list_size = x86_32->cache->num_regs;
	LOG_DEBUG("num_regs=%d, reg_class=%d", (*reg_list_size), reg_class);
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));
	if (!*reg_list) {
		LOG_ERROR("%s out of memory", __func__);
		return ERROR_FAIL;
	}
	/* this will copy the values from our reg list to gdbs */
	for (i = 0; i < (*reg_list_size); i++) {
		(*reg_list)[i] = &x86_32->cache->reg_list[i];
		LOG_DEBUG("value %s = %08" PRIx32, x86_32->cache->reg_list[i].name,
				buf_get_u32(x86_32->cache->reg_list[i].value, 0, 32));
	}
	return ERROR_OK;
}

int x86_32_common_init_arch_info(struct target *t, struct x86_32_common *x86_32)
{
	t->arch_info = x86_32;
	x86_32->common_magic = X86_32_COMMON_MAGIC;
	x86_32->num_hw_bpoints = MAX_DEBUG_REGS;
	x86_32->hw_break_list = calloc(x86_32->num_hw_bpoints,
				sizeof(struct x86_32_dbg_reg));
	if (!x86_32->hw_break_list) {
		LOG_ERROR("%s out of memory", __func__);
		return ERROR_FAIL;
	}
	x86_32->curr_tap = t->tap;
	x86_32->fast_data_area = NULL;
	x86_32->flush = 1;
	x86_32->read_hw_reg_to_cache = read_hw_reg_to_cache;
	x86_32->write_hw_reg_from_cache = write_hw_reg_from_cache;
	return ERROR_OK;
}

int x86_32_common_mmu(struct target *t, int *enabled)
{
	*enabled = true;
	return ERROR_OK;
}

int x86_32_common_virt2phys(struct target *t, target_addr_t address, target_addr_t *physical)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	/*
	 * We need to ignore 'segmentation' for now, as OpenOCD can't handle
	 * segmented addresses.
	 * In protected mode that is almost OK, as (almost) any known OS is using
	 * flat segmentation. In real mode we use use the base of the DS segment,
	 * as we don't know better ...
	 */

	uint32_t cr0 = buf_get_u32(x86_32->cache->reg_list[CR0].value, 0, 32);
	if (!(cr0 & CR0_PG)) {
		/* target halted in real mode */
		/* TODO: needs validation !!! */
		uint32_t dsb = buf_get_u32(x86_32->cache->reg_list[DSB].value, 0, 32);
		*physical = dsb + address;

	} else {
		/* target halted in protected mode */
		if (calcaddr_physfromlin(t, address, physical) != ERROR_OK) {
			LOG_ERROR("%s failed to calculate physical address from " TARGET_ADDR_FMT,
					__func__, address);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

int x86_32_common_read_phys_mem(struct target *t, target_addr_t phys_address,
			uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	int error;

	error = read_phys_mem(t, phys_address, size, count, buffer);
	if (error != ERROR_OK)
		return error;

	/* After reading memory from target, we must replace software breakpoints
	 * with the original instructions again.
	 */
	struct swbp_mem_patch *iter = x86_32->swbbp_mem_patch_list;
	while (iter) {
		if (iter->physaddr >= phys_address && iter->physaddr < phys_address+(size*count)) {
			uint32_t offset = iter->physaddr - phys_address;
			buffer[offset] = iter->orig_byte;
		}
		iter = iter->next;
	}
	return ERROR_OK;
}

static int read_phys_mem(struct target *t, uint32_t phys_address,
			uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	bool pg_disabled = false;
	LOG_DEBUG("addr=0x%08" PRIx32 ", size=%" PRIu32 ", count=0x%" PRIx32 ", buf=%p",
			phys_address, size, count, buffer);
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	if (!count || !buffer || !phys_address) {
		LOG_ERROR("%s invalid params count=0x%" PRIx32 ", buf=%p, addr=0x%08" PRIx32,
				__func__, count, buffer, phys_address);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	/* to access physical memory, switch off the CR0.PG bit */
	if (x86_32->is_paging_enabled(t)) {
		retval = x86_32->disable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not disable paging", __func__);
			return retval;
		}
		pg_disabled = true;
	}

	for (uint32_t i = 0; i < count; i++) {
		switch (size) {
		case BYTE:
			retval = read_mem(t, size, phys_address + i, buffer + i);
			break;
		case WORD:
			retval = read_mem(t, size, phys_address + i * 2, buffer + i * 2);
			break;
		case DWORD:
			retval = read_mem(t, size, phys_address + i * 4, buffer + i * 4);
			break;
		default:
			LOG_ERROR("%s invalid read size", __func__);
			break;
		}
		if (retval != ERROR_OK)
			break;
	}
	/* restore CR0.PG bit if needed (regardless of retval) */
	if (pg_disabled) {
		int retval2 = x86_32->enable_paging(t);
		if (retval2 != ERROR_OK) {
			LOG_ERROR("%s could not enable paging", __func__);
			return retval2;
		}
	}
	/* TODO: After reading memory from target, we must replace
	 * software breakpoints with the original instructions again.
	 * Solve this with the breakpoint fix
	 */
	return retval;
}

int x86_32_common_write_phys_mem(struct target *t, target_addr_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	int error = ERROR_OK;
	uint8_t *newbuffer = NULL;

	check_not_halted(t);
	if (!count || !buffer || !phys_address) {
		LOG_ERROR("%s invalid params count=0x%" PRIx32 ", buf=%p, addr=" TARGET_ADDR_FMT,
				__func__, count, buffer, phys_address);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	/* Before writing memory to target, we must update software breakpoints
	 * with the new instructions and patch the memory buffer with the
	 * breakpoint instruction.
	 */
	newbuffer = malloc(size*count);
	if (!newbuffer) {
		LOG_ERROR("%s out of memory", __func__);
		return ERROR_FAIL;
	}
	memcpy(newbuffer, buffer, size*count);
	struct swbp_mem_patch *iter = x86_32->swbbp_mem_patch_list;
	while (iter) {
		if (iter->physaddr >= phys_address && iter->physaddr < phys_address+(size*count)) {
			uint32_t offset = iter->physaddr - phys_address;
			newbuffer[offset] = SW_BP_OPCODE;

			/* update the breakpoint */
			struct breakpoint *pbiter = t->breakpoints;
			while (pbiter && pbiter->unique_id != iter->swbp_unique_id)
				pbiter = pbiter->next;
			if (pbiter)
				pbiter->orig_instr[0] = buffer[offset];
		}
		iter = iter->next;
	}

	error = write_phys_mem(t, phys_address, size, count, newbuffer);
	free(newbuffer);
	return error;
}

static int write_phys_mem(struct target *t, uint32_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	bool pg_disabled = false;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("addr=0x%08" PRIx32 ", size=%" PRIu32 ", count=0x%" PRIx32 ", buf=%p",
			phys_address, size, count, buffer);

	check_not_halted(t);
	if (!count || !buffer || !phys_address) {
		LOG_ERROR("%s invalid params count=0x%" PRIx32 ", buf=%p, addr=0x%08" PRIx32,
				__func__, count, buffer, phys_address);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	/* TODO: Before writing memory to target, we must update
	 * software breakpoints with the new instructions and
	 * patch the memory buffer with the breakpoint instruction.
	 * Solve this with the breakpoint fix
	 */

	/* to access physical memory, switch off the CR0.PG bit */
	if (x86_32->is_paging_enabled(t)) {
		retval = x86_32->disable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not disable paging", __func__);
			return retval;
		}
		pg_disabled = true;
	}
	for (uint32_t i = 0; i < count; i++) {
		switch (size) {
		case BYTE:
			retval = write_mem(t, size, phys_address + i, buffer + i);
			break;
		case WORD:
			retval = write_mem(t, size, phys_address + i * 2, buffer + i * 2);
			break;
		case DWORD:
			retval = write_mem(t, size, phys_address + i * 4, buffer + i * 4);
			break;
		default:
			LOG_DEBUG("invalid read size");
			break;
		}
	}
	/* restore CR0.PG bit if needed (regardless of retval) */
	if (pg_disabled) {
		retval = x86_32->enable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not enable paging", __func__);
			return retval;
		}
	}
	return retval;
}

static int read_mem(struct target *t, uint32_t size,
			uint32_t addr, uint8_t *buf)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	/* if CS.D bit=1 then its a 32 bit code segment, else 16 */
	bool use32 = (buf_get_u32(x86_32->cache->reg_list[CSAR].value, 0, 32)) & CSAR_D;
	int retval = x86_32->write_hw_reg(t, EAX, addr, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error write EAX", __func__);
		return retval;
	}

	switch (size) {
		case BYTE:
			if (use32)
				retval = x86_32->submit_instruction(t, MEMRDB32);
			else
				retval = x86_32->submit_instruction(t, MEMRDB16);
			break;
		case WORD:
			if (use32)
				retval = x86_32->submit_instruction(t, MEMRDH32);
			else
				retval = x86_32->submit_instruction(t, MEMRDH16);
			break;
		case DWORD:
			if (use32)
				retval = x86_32->submit_instruction(t, MEMRDW32);
			else
				retval = x86_32->submit_instruction(t, MEMRDW16);
			break;
		default:
			LOG_ERROR("%s invalid read mem size", __func__);
			break;
	}

	if (retval != ERROR_OK)
		return retval;

	/* read_hw_reg() will write to 4 bytes (uint32_t)
	 * Watch out, the buffer passed into read_mem() might be 1 or 2 bytes.
	 */
	uint32_t regval;
	retval = x86_32->read_hw_reg(t, EDX, &regval, 0);

	if (retval != ERROR_OK) {
		LOG_ERROR("%s error read EDX", __func__);
		return retval;
	}
	for (uint8_t i = 0; i < size; i++)
		buf[i] = (regval >> (i*8)) & 0x000000FF;

	retval = x86_32->transaction_status(t);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error on mem read", __func__);
		return retval;
	}
	return retval;
}

static int write_mem(struct target *t, uint32_t size,
			uint32_t addr, const uint8_t *buf)
{
	uint32_t i = 0;
	uint32_t buf4bytes = 0;
	int retval = ERROR_OK;
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	for (i = 0; i < size; ++i) {
		buf4bytes = buf4bytes << 8; /* first time we only shift 0s */
		buf4bytes += buf[(size-1)-i]; /* it was hard to write, should be hard to read! */
	}
	/* if CS.D bit=1 then its a 32 bit code segment, else 16 */
	bool use32 = (buf_get_u32(x86_32->cache->reg_list[CSAR].value, 0, 32)) & CSAR_D;
	retval = x86_32->write_hw_reg(t, EAX, addr, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error write EAX", __func__);
		return retval;
	}

	/* write_hw_reg() will write to 4 bytes (uint32_t)
	 * Watch out, the buffer passed into write_mem() might be 1 or 2 bytes.
	 */
	retval = x86_32->write_hw_reg(t, EDX, buf4bytes, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error write EDX", __func__);
		return retval;
	}
	switch (size) {
		case BYTE:
			if (use32)
				retval = x86_32->submit_instruction(t, MEMWRB32);
			else
				retval = x86_32->submit_instruction(t, MEMWRB16);
			break;
		case WORD:
			if (use32)
				retval = x86_32->submit_instruction(t, MEMWRH32);
			else
				retval = x86_32->submit_instruction(t, MEMWRH16);
			break;
		case DWORD:
			if (use32)
				retval = x86_32->submit_instruction(t, MEMWRW32);
			else
				retval = x86_32->submit_instruction(t, MEMWRW16);
			break;
		default:
			LOG_ERROR("%s invalid write mem size", __func__);
			return ERROR_FAIL;
	}

	if (retval != ERROR_OK)
		return retval;

	retval = x86_32->transaction_status(t);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error on mem write", __func__);
		return retval;
	}
	return retval;
}

int calcaddr_physfromlin(struct target *t, target_addr_t addr, target_addr_t *physaddr)
{
	uint8_t entry_buffer[8];

	if (!physaddr || !t)
		return ERROR_FAIL;

	struct x86_32_common *x86_32 = target_to_x86_32(t);

	/* The 'user-visible' CR0.PG should be set - otherwise the function shouldn't be called
	 * (Don't check the CR0.PG on the target, this might be temporally disabled at this point)
	 */
	uint32_t cr0 = buf_get_u32(x86_32->cache->reg_list[CR0].value, 0, 32);
	if (!(cr0 & CR0_PG)) {
		/* you are wrong in this function, never mind */
		*physaddr = addr;
		return ERROR_OK;
	}

	uint32_t cr4 = buf_get_u32(x86_32->cache->reg_list[CR4].value, 0, 32);
	bool is_pae = cr4 & 0x00000020; /* PAE - Physical Address Extension */

	uint32_t cr3 = buf_get_u32(x86_32->cache->reg_list[CR3].value, 0, 32);
	if (is_pae) {
		uint32_t pdpt_base = cr3 & 0xFFFFF000; /* lower 12 bits of CR3 must always be 0 */
		uint32_t pdpt_index = (addr & 0xC0000000) >> 30; /* A[31:30] index to PDPT */
		uint32_t pdpt_addr = pdpt_base + (8 * pdpt_index);
		if (x86_32_common_read_phys_mem(t, pdpt_addr, 4, 2, entry_buffer) != ERROR_OK) {
			LOG_ERROR("%s couldn't read page directory pointer table entry at 0x%08" PRIx32,
					__func__, pdpt_addr);
			return ERROR_FAIL;
		}
		uint64_t pdpt_entry = target_buffer_get_u64(t, entry_buffer);
		if (!(pdpt_entry & 0x0000000000000001)) {
			LOG_ERROR("%s page directory pointer table entry at 0x%08" PRIx32 " is not present",
					__func__, pdpt_addr);
			return ERROR_FAIL;
		}

		uint32_t pd_base = pdpt_entry & 0xFFFFF000; /* A[31:12] is PageTable/Page Base Address */
		uint32_t pd_index = (addr & 0x3FE00000) >> 21; /* A[29:21] index to PD entry with PAE */
		uint32_t pd_addr = pd_base + (8 * pd_index);
		if (x86_32_common_read_phys_mem(t, pd_addr, 4, 2, entry_buffer) != ERROR_OK) {
			LOG_ERROR("%s couldn't read page directory entry at 0x%08" PRIx32,
					__func__, pd_addr);
			return ERROR_FAIL;
		}
		uint64_t pd_entry = target_buffer_get_u64(t, entry_buffer);
		if (!(pd_entry & 0x0000000000000001)) {
			LOG_ERROR("%s page directory entry at 0x%08" PRIx32 " is not present",
					__func__, pd_addr);
			return ERROR_FAIL;
		}

		/* PS bit in PD entry is indicating 4KB or 2MB page size */
		if (pd_entry & 0x0000000000000080) {

			uint32_t page_base = (uint32_t)(pd_entry & 0x00000000FFE00000); /* [31:21] */
			uint32_t offset = addr & 0x001FFFFF; /* [20:0] */
			*physaddr = page_base + offset;
			return ERROR_OK;

		} else {

			uint32_t pt_base = (uint32_t)(pd_entry & 0x00000000FFFFF000); /*[31:12]*/
			uint32_t pt_index = (addr & 0x001FF000) >> 12; /*[20:12]*/
			uint32_t pt_addr = pt_base + (8 * pt_index);
			if (x86_32_common_read_phys_mem(t, pt_addr, 4, 2, entry_buffer) != ERROR_OK) {
				LOG_ERROR("%s couldn't read page table entry at 0x%08" PRIx32, __func__, pt_addr);
				return ERROR_FAIL;
			}
			uint64_t pt_entry = target_buffer_get_u64(t, entry_buffer);
			if (!(pt_entry & 0x0000000000000001)) {
				LOG_ERROR("%s page table entry at 0x%08" PRIx32 " is not present", __func__, pt_addr);
				return ERROR_FAIL;
			}

			uint32_t page_base = (uint32_t)(pt_entry & 0x00000000FFFFF000); /*[31:12]*/
			uint32_t offset =  addr & 0x00000FFF; /*[11:0]*/
			*physaddr = page_base + offset;
			return ERROR_OK;
		}
	} else {
		uint32_t pd_base = cr3 & 0xFFFFF000; /* lower 12 bits of CR3 must always be 0 */
		uint32_t pd_index = (addr & 0xFFC00000) >> 22; /* A[31:22] index to PD entry */
		uint32_t pd_addr = pd_base + (4 * pd_index);
		if (x86_32_common_read_phys_mem(t, pd_addr, 4, 1, entry_buffer) != ERROR_OK) {
			LOG_ERROR("%s couldn't read page directory entry at 0x%08" PRIx32, __func__, pd_addr);
			return ERROR_FAIL;
		}
		uint32_t pd_entry = target_buffer_get_u32(t, entry_buffer);
		if (!(pd_entry & 0x00000001)) {
			LOG_ERROR("%s page directory entry at 0x%08" PRIx32 " is not present", __func__, pd_addr);
			return ERROR_FAIL;
		}

		/* Bit 7 in page directory entry is page size.
		 */
		if (pd_entry & 0x00000080) {
			/* 4MB pages */
			uint32_t page_base = pd_entry & 0xFFC00000;
			*physaddr = page_base + (addr & 0x003FFFFF);

		} else {
			/* 4KB pages */
			uint32_t pt_base = pd_entry & 0xFFFFF000; /* A[31:12] is PageTable/Page Base Address */
			uint32_t pt_index = (addr & 0x003FF000) >> 12; /* A[21:12] index to page table entry */
			uint32_t pt_addr = pt_base + (4 * pt_index);
			if (x86_32_common_read_phys_mem(t, pt_addr, 4, 1, entry_buffer) != ERROR_OK) {
				LOG_ERROR("%s couldn't read page table entry at 0x%08" PRIx32, __func__, pt_addr);
				return ERROR_FAIL;
			}
			uint32_t pt_entry = target_buffer_get_u32(t, entry_buffer);
			if (!(pt_entry & 0x00000001)) {
				LOG_ERROR("%s page table entry at 0x%08" PRIx32 " is not present", __func__, pt_addr);
				return ERROR_FAIL;
			}
			uint32_t page_base = pt_entry & 0xFFFFF000; /* A[31:12] is PageTable/Page Base Address */
			*physaddr = page_base + (addr & 0x00000FFF); /* A[11:0] offset to 4KB page in linear address */
		}
	}
	return ERROR_OK;
}

int x86_32_common_read_memory(struct target *t, target_addr_t addr,
			uint32_t size, uint32_t count, uint8_t *buf)
{
	int retval = ERROR_OK;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("addr=" TARGET_ADDR_FMT ", size=%" PRIu32 ", count=0x%" PRIx32 ", buf=%p",
			addr, size, count, buf);
	check_not_halted(t);
	if (!count || !buf || !addr) {
		LOG_ERROR("%s invalid params count=0x%" PRIx32 ", buf=%p, addr=" TARGET_ADDR_FMT,
				__func__, count, buf, addr);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (x86_32->is_paging_enabled(t)) {
		/* all memory accesses from debugger must be physical (CR0.PG == 0)
		 * conversion to physical address space needed
		 */
		retval = x86_32->disable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not disable paging", __func__);
			return retval;
		}
		target_addr_t physaddr = 0;
		if (calcaddr_physfromlin(t, addr, &physaddr) != ERROR_OK) {
			LOG_ERROR("%s failed to calculate physical address from " TARGET_ADDR_FMT,
					  __func__, addr);
			retval = ERROR_FAIL;
		}
		/* TODO: !!! Watch out for page boundaries
		 * for every 4kB, the physical address has to be re-calculated
		 * This should be fixed together with bulk memory reads
		 */

		if (retval == ERROR_OK
			&& x86_32_common_read_phys_mem(t, physaddr, size, count, buf) != ERROR_OK) {
			LOG_ERROR("%s failed to read memory from physical address " TARGET_ADDR_FMT,
					  __func__, physaddr);
		}
		/* restore PG bit if it was cleared prior (regardless of retval) */
		retval = x86_32->enable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not enable paging", __func__);
			return retval;
		}
	} else {
		/* paging is off - linear address is physical address */
		if (x86_32_common_read_phys_mem(t, addr, size, count, buf) != ERROR_OK) {
			LOG_ERROR("%s failed to read memory from address " TARGET_ADDR_FMT,
					  __func__, addr);
			retval = ERROR_FAIL;
		}
	}

	return retval;
}

int x86_32_common_write_memory(struct target *t, target_addr_t addr,
			uint32_t size, uint32_t count, const uint8_t *buf)
{
	int retval = ERROR_OK;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("addr=" TARGET_ADDR_FMT ", size=%" PRIu32 ", count=0x%" PRIx32 ", buf=%p",
			addr, size, count, buf);
	check_not_halted(t);
	if (!count || !buf || !addr) {
		LOG_ERROR("%s invalid params count=0x%" PRIx32 ", buf=%p, addr=" TARGET_ADDR_FMT,
					__func__, count, buf, addr);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	if (x86_32->is_paging_enabled(t)) {
		/* all memory accesses from debugger must be physical (CR0.PG == 0)
		 * conversion to physical address space needed
		 */
		retval = x86_32->disable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not disable paging", __func__);
			return retval;
		}
		target_addr_t physaddr = 0;
		if (calcaddr_physfromlin(t, addr, &physaddr) != ERROR_OK) {
			LOG_ERROR("%s failed to calculate physical address from " TARGET_ADDR_FMT,
					__func__, addr);
			retval = ERROR_FAIL;
		}
		/* TODO: !!! Watch out for page boundaries
		 * for every 4kB, the physical address has to be re-calculated
		 * This should be fixed together with bulk memory reads
		 */
		if (retval == ERROR_OK
			&& x86_32_common_write_phys_mem(t, physaddr, size, count, buf) != ERROR_OK) {
			LOG_ERROR("%s failed to write memory to physical address " TARGET_ADDR_FMT,
					__func__, physaddr);
		}
		/* restore PG bit if it was cleared prior (regardless of retval) */
		retval = x86_32->enable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not enable paging", __func__);
			return retval;
		}
	} else {

		/* paging is off - linear address is physical address */
		if (x86_32_common_write_phys_mem(t, addr, size, count, buf) != ERROR_OK) {
			LOG_ERROR("%s failed to write memory to address " TARGET_ADDR_FMT,
					__func__, addr);
			retval = ERROR_FAIL;
		}
	}
	return retval;
}

int x86_32_common_read_io(struct target *t, uint32_t addr,
			uint32_t size, uint8_t *buf)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	/* if CS.D bit=1 then its a 32 bit code segment, else 16 */
	bool use32 = (buf_get_u32(x86_32->cache->reg_list[CSAR].value, 0, 32)) & CSAR_D;
	int retval = ERROR_FAIL;
	bool pg_disabled = false;
	LOG_DEBUG("addr=0x%08" PRIx32 ", size=%" PRIu32 ", buf=%p", addr, size, buf);
	check_not_halted(t);
	if (!buf || !addr) {
		LOG_ERROR("%s invalid params buf=%p, addr=%08" PRIx32, __func__, buf, addr);
		return retval;
	}
	retval = x86_32->write_hw_reg(t, EDX, addr, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error EDX write", __func__);
		return retval;
	}
	/* to access physical memory, switch off the CR0.PG bit */
	if (x86_32->is_paging_enabled(t)) {
		retval = x86_32->disable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not disable paging", __func__);
			return retval;
		}
		pg_disabled = true;
	}
	switch (size) {
		case BYTE:
			if (use32)
				retval = x86_32->submit_instruction(t, IORDB32);
			else
				retval = x86_32->submit_instruction(t, IORDB16);
			break;
		case WORD:
			if (use32)
				retval = x86_32->submit_instruction(t, IORDH32);
			else
				retval = x86_32->submit_instruction(t, IORDH16);
			break;
		case DWORD:
			if (use32)
				retval = x86_32->submit_instruction(t, IORDW32);
			else
				retval = x86_32->submit_instruction(t, IORDW16);
			break;
		default:
			LOG_ERROR("%s invalid read io size", __func__);
			return ERROR_FAIL;
	}

	/* restore CR0.PG bit if needed */
	if (pg_disabled) {
		int retval2 = x86_32->enable_paging(t);
		if (retval2 != ERROR_OK) {
			LOG_ERROR("%s could not enable paging", __func__);
			return retval2;
		}
	}

	if (retval != ERROR_OK)
		return retval;

	uint32_t regval = 0;
	retval = x86_32->read_hw_reg(t, EAX, &regval, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error on read EAX", __func__);
		return retval;
	}
	for (uint8_t i = 0; i < size; i++)
		buf[i] = (regval >> (i*8)) & 0x000000FF;
	retval = x86_32->transaction_status(t);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error on io read", __func__);
		return retval;
	}
	return retval;
}

int x86_32_common_write_io(struct target *t, uint32_t addr,
			uint32_t size, const uint8_t *buf)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	/* if CS.D bit=1 then its a 32 bit code segment, else 16 */
	bool use32 = (buf_get_u32(x86_32->cache->reg_list[CSAR].value, 0, 32)) & CSAR_D;
	LOG_DEBUG("addr=0x%08" PRIx32 ", size=%" PRIu32 ", buf=%p", addr, size, buf);
	check_not_halted(t);
	int retval = ERROR_FAIL;
	bool pg_disabled = false;
	if (!buf || !addr) {
		LOG_ERROR("%s invalid params buf=%p, addr=0x%08" PRIx32, __func__, buf, addr);
		return retval;
	}
	/* no do the write */
	retval = x86_32->write_hw_reg(t, EDX, addr, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error on EDX write", __func__);
		return retval;
	}
	uint32_t regval = 0;
	for (uint8_t i = 0; i < size; i++)
		regval += (buf[i] << (i*8));
	retval = x86_32->write_hw_reg(t, EAX, regval, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error on EAX write", __func__);
		return retval;
	}
	/* to access physical memory, switch off the CR0.PG bit */
	if (x86_32->is_paging_enabled(t)) {
		retval = x86_32->disable_paging(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s could not disable paging", __func__);
			return retval;
		}
		pg_disabled = true;
	}
	switch (size) {
		case BYTE:
			if (use32)
				retval = x86_32->submit_instruction(t, IOWRB32);
			else
				retval = x86_32->submit_instruction(t, IOWRB16);
			break;
		case WORD:
			if (use32)
				retval = x86_32->submit_instruction(t, IOWRH32);
			else
				retval = x86_32->submit_instruction(t, IOWRH16);
			break;
		case DWORD:
			if (use32)
				retval = x86_32->submit_instruction(t, IOWRW32);
			else
				retval = x86_32->submit_instruction(t, IOWRW16);
			break;
		default:
			LOG_ERROR("%s invalid write io size", __func__);
			return ERROR_FAIL;
	}

	/* restore CR0.PG bit if needed */
	if (pg_disabled) {
		int retval2 = x86_32->enable_paging(t);
		if (retval2 != ERROR_OK) {
			LOG_ERROR("%s could not enable paging", __func__);
			return retval2;
		}
	}

	if (retval != ERROR_OK)
		return retval;

	retval = x86_32->transaction_status(t);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s error on io write", __func__);
		return retval;
	}
	return retval;
}

int x86_32_common_add_watchpoint(struct target *t, struct watchpoint *wp)
{
	check_not_halted(t);
	/* set_watchpoint() will return ERROR_TARGET_RESOURCE_NOT_AVAILABLE if all
	 * hardware registers are gone
	 */
	return set_watchpoint(t, wp);
}

int x86_32_common_remove_watchpoint(struct target *t, struct watchpoint *wp)
{
	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	if (wp->set)
		unset_watchpoint(t, wp);
	return ERROR_OK;
}

int x86_32_common_add_breakpoint(struct target *t, struct breakpoint *bp)
{
	LOG_DEBUG("type=%d, addr=" TARGET_ADDR_FMT, bp->type, bp->address);
	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	/* set_breakpoint() will return ERROR_TARGET_RESOURCE_NOT_AVAILABLE if all
	 * hardware registers are gone (for hardware breakpoints)
	 */
	return set_breakpoint(t, bp);
}

int x86_32_common_remove_breakpoint(struct target *t, struct breakpoint *bp)
{
	LOG_DEBUG("type=%d, addr=" TARGET_ADDR_FMT, bp->type, bp->address);
	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	if (bp->set)
		unset_breakpoint(t, bp);

	return ERROR_OK;
}

static int set_debug_regs(struct target *t, uint32_t address,
			uint8_t bp_num, uint8_t bp_type, uint8_t bp_length)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("addr=0x%08" PRIx32 ", bp_num=%" PRIu8 ", bp_type=%" PRIu8 ", pb_length=%" PRIu8,
			address, bp_num, bp_type, bp_length);

	/* DR7 - set global enable */
	uint32_t dr7 = buf_get_u32(x86_32->cache->reg_list[DR7].value, 0, 32);

	if (bp_length != 1 && bp_length != 2 && bp_length != 4)
		return ERROR_FAIL;

	if (DR7_BP_FREE(dr7, bp_num))
		DR7_GLOBAL_ENABLE(dr7, bp_num);
	else {
		LOG_ERROR("%s dr7 error, already enabled, val=%08" PRIx32, __func__, dr7);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	switch (bp_type) {
		case 0:
			/* 00 - only on instruction execution */
			DR7_SET_EXE(dr7, bp_num);
			DR7_SET_LENGTH(dr7, bp_num, bp_length);
		break;
		case 1:
			/* 01 - only on data writes */
			DR7_SET_WRITE(dr7, bp_num);
			DR7_SET_LENGTH(dr7, bp_num, bp_length);
		break;
		case 2:
			/* 10 UNSUPPORTED - an I/O read and I/O write */
			LOG_ERROR("%s unsupported feature bp_type=%d", __func__, bp_type);
			return ERROR_FAIL;
		break;
		case 3:
			/* on data read or data write */
			DR7_SET_ACCESS(dr7, bp_num);
			DR7_SET_LENGTH(dr7, bp_num, bp_length);
		break;
		default:
			LOG_ERROR("%s invalid request [only 0-3] bp_type=%d", __func__, bp_type);
			return ERROR_FAIL;
	}

	/* update regs in the reg cache ready to be written to hardware
	 * when we exit PM
	*/
	buf_set_u32(x86_32->cache->reg_list[bp_num+DR0].value, 0, 32, address);
	x86_32->cache->reg_list[bp_num+DR0].dirty = true;
	x86_32->cache->reg_list[bp_num+DR0].valid = true;
	buf_set_u32(x86_32->cache->reg_list[DR6].value, 0, 32, PM_DR6);
	x86_32->cache->reg_list[DR6].dirty = true;
	x86_32->cache->reg_list[DR6].valid = true;
	buf_set_u32(x86_32->cache->reg_list[DR7].value, 0, 32, dr7);
	x86_32->cache->reg_list[DR7].dirty = true;
	x86_32->cache->reg_list[DR7].valid = true;
	return ERROR_OK;
}

static int unset_debug_regs(struct target *t, uint8_t bp_num)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("bp_num=%" PRIu8, bp_num);

	uint32_t dr7 = buf_get_u32(x86_32->cache->reg_list[DR7].value, 0, 32);

	if (!(DR7_BP_FREE(dr7, bp_num))) {
		DR7_GLOBAL_DISABLE(dr7, bp_num);
	} else {
		LOG_ERROR("%s dr7 error, not enabled, val=0x%08" PRIx32, __func__, dr7);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	/* this will clear rw and len bits */
	DR7_RESET_RWLEN_BITS(dr7, bp_num);

	/* update regs in the reg cache ready to be written to hardware
	 * when we exit PM
	*/
	buf_set_u32(x86_32->cache->reg_list[bp_num+DR0].value, 0, 32, 0);
	x86_32->cache->reg_list[bp_num+DR0].dirty = true;
	x86_32->cache->reg_list[bp_num+DR0].valid = true;
	buf_set_u32(x86_32->cache->reg_list[DR6].value, 0, 32, PM_DR6);
	x86_32->cache->reg_list[DR6].dirty = true;
	x86_32->cache->reg_list[DR6].valid = true;
	buf_set_u32(x86_32->cache->reg_list[DR7].value, 0, 32, dr7);
	x86_32->cache->reg_list[DR7].dirty = true;
	x86_32->cache->reg_list[DR7].valid = true;
	return ERROR_OK;
}

static int set_hwbp(struct target *t, struct breakpoint *bp)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct x86_32_dbg_reg *debug_reg_list = x86_32->hw_break_list;
	uint8_t hwbp_num = 0;

	while (debug_reg_list[hwbp_num].used && (hwbp_num < x86_32->num_hw_bpoints))
		hwbp_num++;
	if (hwbp_num >= x86_32->num_hw_bpoints) {
		LOG_ERROR("%s no free hw breakpoint bpid=0x%" PRIx32, __func__, bp->unique_id);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	if (set_debug_regs(t, bp->address, hwbp_num, DR7_BP_EXECUTE, 1) != ERROR_OK)
		return ERROR_FAIL;
	bp->set = hwbp_num + 1;
	debug_reg_list[hwbp_num].used = 1;
	debug_reg_list[hwbp_num].bp_value = bp->address;
	LOG_USER("%s hardware breakpoint %" PRIu32 " set at 0x%08" PRIx32 " (hwreg=%" PRIu8 ")", __func__,
			bp->unique_id, debug_reg_list[hwbp_num].bp_value, hwbp_num);
	return ERROR_OK;
}

static int unset_hwbp(struct target *t, struct breakpoint *bp)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct x86_32_dbg_reg *debug_reg_list = x86_32->hw_break_list;
	int hwbp_num = bp->set - 1;

	if ((hwbp_num < 0) || (hwbp_num >= x86_32->num_hw_bpoints)) {
		LOG_ERROR("%s invalid breakpoint number=%d, bpid=%" PRIu32,
				__func__, hwbp_num, bp->unique_id);
		return ERROR_OK;
	}

	if (unset_debug_regs(t, hwbp_num) != ERROR_OK)
		return ERROR_FAIL;
	debug_reg_list[hwbp_num].used = 0;
	debug_reg_list[hwbp_num].bp_value = 0;

	LOG_USER("%s hardware breakpoint %" PRIu32 " removed from " TARGET_ADDR_FMT " (hwreg=%d)",
			__func__, bp->unique_id, bp->address, hwbp_num);
	return ERROR_OK;
}

static int set_swbp(struct target *t, struct breakpoint *bp)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("id %" PRIx32, bp->unique_id);
	target_addr_t physaddr;
	uint8_t opcode = SW_BP_OPCODE;
	uint8_t readback;

	if (calcaddr_physfromlin(t, bp->address, &physaddr) != ERROR_OK)
		return ERROR_FAIL;
	if (read_phys_mem(t, physaddr, 1, 1, bp->orig_instr))
		return ERROR_FAIL;

	LOG_DEBUG("set software breakpoint - orig byte=0x%02" PRIx8 "", *bp->orig_instr);

	/* just write the instruction trap byte */
	if (write_phys_mem(t, physaddr, 1, 1, &opcode))
		return ERROR_FAIL;

	/* verify that this is not invalid/read-only memory */
	if (read_phys_mem(t, physaddr, 1, 1, &readback))
		return ERROR_FAIL;

	if (readback != SW_BP_OPCODE) {
		LOG_ERROR("%s software breakpoint error at " TARGET_ADDR_FMT ", check memory",
				__func__, bp->address);
		LOG_ERROR("%s readback=0x%02" PRIx8 " orig=0x%02" PRIx8 "",
				__func__, readback, *bp->orig_instr);
		return ERROR_FAIL;
	}
	bp->set = SW_BP_OPCODE; /* just non 0 */

	/* add the memory patch */
	struct swbp_mem_patch *new_patch = malloc(sizeof(struct swbp_mem_patch));
	if (!new_patch) {
		LOG_ERROR("%s out of memory", __func__);
		return ERROR_FAIL;
	}
	new_patch->next = NULL;
	new_patch->orig_byte = *bp->orig_instr;
	new_patch->physaddr = physaddr;
	new_patch->swbp_unique_id = bp->unique_id;

	struct swbp_mem_patch *addto = x86_32->swbbp_mem_patch_list;
	if (!addto)
		x86_32->swbbp_mem_patch_list = new_patch;
	else {
		while (addto->next)
			addto = addto->next;
		addto->next = new_patch;
	}
	LOG_USER("%s software breakpoint %" PRIu32 " set at " TARGET_ADDR_FMT,
			__func__, bp->unique_id, bp->address);
	return ERROR_OK;
}

static int unset_swbp(struct target *t, struct breakpoint *bp)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("id %" PRIx32, bp->unique_id);
	target_addr_t physaddr;
	uint8_t current_instr;

	/* check that user program has not modified breakpoint instruction */
	if (calcaddr_physfromlin(t, bp->address, &physaddr) != ERROR_OK)
		return ERROR_FAIL;
	if (read_phys_mem(t, physaddr, 1, 1, &current_instr))
		return ERROR_FAIL;

	if (current_instr == SW_BP_OPCODE) {
		if (write_phys_mem(t, physaddr, 1, 1, bp->orig_instr))
			return ERROR_FAIL;
	} else {
		LOG_ERROR("%s software breakpoint remove error at " TARGET_ADDR_FMT ", check memory",
				__func__, bp->address);
		LOG_ERROR("%s current=0x%02" PRIx8 " orig=0x%02" PRIx8 "",
				__func__, current_instr, *bp->orig_instr);
		return ERROR_FAIL;
	}

	/* remove from patch */
	struct swbp_mem_patch *iter = x86_32->swbbp_mem_patch_list;
	if (iter) {
		if (iter->swbp_unique_id == bp->unique_id) {
			/* it's the first item */
			x86_32->swbbp_mem_patch_list = iter->next;
			free(iter);
		} else {
			while (iter->next && iter->next->swbp_unique_id != bp->unique_id)
				iter = iter->next;
			if (iter->next) {
				/* it's the next one */
				struct swbp_mem_patch *freeme = iter->next;
				iter->next = iter->next->next;
				free(freeme);
			}
		}
	}

	LOG_USER("%s software breakpoint %" PRIu32 " removed from " TARGET_ADDR_FMT,
			__func__, bp->unique_id, bp->address);
	return ERROR_OK;
}

static int set_breakpoint(struct target *t, struct breakpoint *bp)
{
	int error = ERROR_OK;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	LOG_DEBUG("type=%d, addr=" TARGET_ADDR_FMT, bp->type, bp->address);
	if (bp->set) {
		LOG_ERROR("breakpoint already set");
		return error;
	}
	if (bp->type == BKPT_HARD) {
		error = set_hwbp(t, bp);
		if (error != ERROR_OK) {
			LOG_ERROR("%s error setting hardware breakpoint at " TARGET_ADDR_FMT,
					__func__, bp->address);
			return error;
		}
	} else {
		if (x86_32->sw_bpts_supported(t)) {
			error = set_swbp(t, bp);
			if (error != ERROR_OK) {
				LOG_ERROR("%s error setting software breakpoint at " TARGET_ADDR_FMT,
						__func__, bp->address);
				return error;
			}
		} else {
			LOG_ERROR("%s core doesn't support SW breakpoints", __func__);
			return ERROR_FAIL;
		}
	}
	return error;
}

static int unset_breakpoint(struct target *t, struct breakpoint *bp)
{
	LOG_DEBUG("type=%d, addr=" TARGET_ADDR_FMT, bp->type, bp->address);
	if (!bp->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (bp->type == BKPT_HARD) {
		if (unset_hwbp(t, bp) != ERROR_OK) {
			LOG_ERROR("%s error removing hardware breakpoint at " TARGET_ADDR_FMT,
					__func__, bp->address);
			return ERROR_FAIL;
		}
	} else {
		if (unset_swbp(t, bp) != ERROR_OK) {
			LOG_ERROR("%s error removing software breakpoint at " TARGET_ADDR_FMT,
					__func__, bp->address);
			return ERROR_FAIL;
		}
	}
	bp->set = 0;
	return ERROR_OK;
}

static int set_watchpoint(struct target *t, struct watchpoint *wp)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct x86_32_dbg_reg *debug_reg_list = x86_32->hw_break_list;
	int wp_num = 0;
	LOG_DEBUG("type=%d, addr=" TARGET_ADDR_FMT, wp->rw, wp->address);

	if (wp->set) {
		LOG_ERROR("%s watchpoint already set", __func__);
		return ERROR_OK;
	}

	if (wp->rw == WPT_READ) {
		LOG_ERROR("%s no support for 'read' watchpoints, use 'access' or 'write'"
				, __func__);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	while (debug_reg_list[wp_num].used && (wp_num < x86_32->num_hw_bpoints))
		wp_num++;
	if (wp_num >= x86_32->num_hw_bpoints) {
		LOG_ERROR("%s no debug registers left", __func__);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (wp->length != 4 && wp->length != 2 && wp->length != 1) {
		LOG_ERROR("%s only watchpoints of length 1, 2 or 4 are supported", __func__);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	switch (wp->rw) {
		case WPT_WRITE:
			if (set_debug_regs(t, wp->address, wp_num,
						DR7_BP_WRITE, wp->length) != ERROR_OK) {
				return ERROR_FAIL;
			}
			break;
		case WPT_ACCESS:
			if (set_debug_regs(t, wp->address, wp_num, DR7_BP_READWRITE,
						wp->length) != ERROR_OK) {
				return ERROR_FAIL;
			}
			break;
		default:
			LOG_ERROR("%s only 'access' or 'write' watchpoints are supported", __func__);
			break;
	}
	wp->set = wp_num + 1;
	debug_reg_list[wp_num].used = 1;
	debug_reg_list[wp_num].bp_value = wp->address;
	LOG_USER("'%s' watchpoint %d set at " TARGET_ADDR_FMT " with length %" PRIu32 " (hwreg=%d)",
			wp->rw == WPT_READ ? "read" : wp->rw == WPT_WRITE ?
			"write" : wp->rw == WPT_ACCESS ? "access" : "?",
			wp->unique_id, wp->address, wp->length, wp_num);
	return ERROR_OK;
}

static int unset_watchpoint(struct target *t, struct watchpoint *wp)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct x86_32_dbg_reg *debug_reg_list = x86_32->hw_break_list;
	LOG_DEBUG("type=%d, addr=" TARGET_ADDR_FMT, wp->rw, wp->address);
	if (!wp->set) {
		LOG_WARNING("watchpoint not set");
		return ERROR_OK;
	}

	int wp_num = wp->set - 1;
	if ((wp_num < 0) || (wp_num >= x86_32->num_hw_bpoints)) {
		LOG_DEBUG("Invalid FP Comparator number in watchpoint");
		return ERROR_OK;
	}
	if (unset_debug_regs(t, wp_num) != ERROR_OK)
		return ERROR_FAIL;

	debug_reg_list[wp_num].used = 0;
	debug_reg_list[wp_num].bp_value = 0;
	wp->set = 0;

	LOG_USER("'%s' watchpoint %d removed from " TARGET_ADDR_FMT " with length %" PRIu32 " (hwreg=%d)",
			wp->rw == WPT_READ ? "read" : wp->rw == WPT_WRITE ?
			"write" : wp->rw == WPT_ACCESS ? "access" : "?",
			wp->unique_id, wp->address, wp->length, wp_num);

	return ERROR_OK;
}

/* after reset breakpoints and watchpoints in memory are not valid anymore and
 * debug registers are cleared.
 * we can't afford to remove sw breakpoints using the default methods as the
 * memory doesn't have the same layout yet and an access might crash the target,
 * so we just clear the openocd breakpoints structures.
 */
void x86_32_common_reset_breakpoints_watchpoints(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct x86_32_dbg_reg *debug_reg_list = x86_32->hw_break_list;
	struct breakpoint *next_b;
	struct watchpoint *next_w;

	while (t->breakpoints) {
		next_b = t->breakpoints->next;
		free(t->breakpoints->orig_instr);
		free(t->breakpoints);
		t->breakpoints = next_b;
	}

	while (t->watchpoints) {
		next_w = t->watchpoints->next;
		free(t->watchpoints);
		t->watchpoints = next_w;
	}

	for (int i = 0; i < x86_32->num_hw_bpoints; i++) {
		debug_reg_list[i].used = 0;
		debug_reg_list[i].bp_value = 0;
	}
}

static int read_hw_reg_to_cache(struct target *t, int num)
{
	uint32_t reg_value;
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	if ((num < 0) || (num >= x86_32->get_num_user_regs(t)))
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (x86_32->read_hw_reg(t, num, &reg_value, 1) != ERROR_OK) {
		LOG_ERROR("%s fail for %s", x86_32->cache->reg_list[num].name, __func__);
		return ERROR_FAIL;
	}
	LOG_DEBUG("reg %s value 0x%08" PRIx32,
			x86_32->cache->reg_list[num].name, reg_value);
	return ERROR_OK;
}

static int write_hw_reg_from_cache(struct target *t, int num)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	if ((num < 0) || (num >= x86_32->get_num_user_regs(t)))
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (x86_32->write_hw_reg(t, num, 0, 1) != ERROR_OK) {
		LOG_ERROR("%s fail for %s", x86_32->cache->reg_list[num].name, __func__);
		return ERROR_FAIL;
	}
	LOG_DEBUG("reg %s value 0x%08" PRIx32, x86_32->cache->reg_list[num].name,
			buf_get_u32(x86_32->cache->reg_list[num].value, 0, 32));
	return ERROR_OK;
}

/* x86 32 commands */
static void handle_iod_output(struct command_invocation *cmd,
		struct target *target, uint32_t address, unsigned size,
		unsigned count, const uint8_t *buffer)
{
	const unsigned line_bytecnt = 32;
	unsigned line_modulo = line_bytecnt / size;

	char output[line_bytecnt * 4 + 1];
	unsigned output_len = 0;

	const char *value_fmt;
	switch (size) {
	case 4:
		value_fmt = "%8.8x ";
		break;
	case 2:
		value_fmt = "%4.4x ";
		break;
	case 1:
		value_fmt = "%2.2x ";
		break;
	default:
		/* "can't happen", caller checked */
		LOG_ERROR("%s invalid memory read size: %u", __func__, size);
		return;
	}

	for (unsigned i = 0; i < count; i++) {
		if (i % line_modulo == 0) {
			output_len += snprintf(output + output_len,
					sizeof(output) - output_len,
					"0x%8.8x: ",
					(unsigned)(address + (i*size)));
		}

		uint32_t value = 0;
		const uint8_t *value_ptr = buffer + i * size;
		switch (size) {
		case 4:
			value = target_buffer_get_u32(target, value_ptr);
			break;
		case 2:
			value = target_buffer_get_u16(target, value_ptr);
			break;
		case 1:
			value = *value_ptr;
		}
		output_len += snprintf(output + output_len,
				sizeof(output) - output_len,
				value_fmt, value);

		if ((i % line_modulo == line_modulo - 1) || (i == count - 1)) {
			command_print(cmd, "%s", output);
			output_len = 0;
		}
	}
}

COMMAND_HANDLER(handle_iod_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	if (address > 0xffff) {
		LOG_ERROR("%s IA-32 I/O space is 2^16, 0x%08" PRIx32 " exceeds max", __func__, address);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	unsigned size = 0;
	switch (CMD_NAME[2]) {
	case 'w':
		size = 4;
		break;
	case 'h':
		size = 2;
		break;
	case 'b':
		size = 1;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	unsigned count = 1;
	uint8_t *buffer = calloc(count, size);
	struct target *target = get_current_target(CMD_CTX);
	int retval = x86_32_common_read_io(target, address, size, buffer);
	if (retval == ERROR_OK)
		handle_iod_output(CMD, target, address, size, count, buffer);
	free(buffer);
	return retval;
}

static int target_fill_io(struct target *target,
		uint32_t address,
		unsigned data_size,
		/* value */
		uint32_t b)
{
	LOG_DEBUG("address=0x%08" PRIx32 ", data_size=%u, b=0x%08" PRIx32,
			address, data_size, b);
	uint8_t target_buf[data_size];
	switch (data_size) {
	case 4:
		target_buffer_set_u32(target, target_buf, b);
		break;
	case 2:
		target_buffer_set_u16(target, target_buf, b);
		break;
	case 1:
		target_buf[0] = (b & 0x0ff);
		break;
	default:
		exit(-1);
	}
	return x86_32_common_write_io(target, address, data_size, target_buf);
}

COMMAND_HANDLER(handle_iow_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	uint32_t address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
	struct target *target = get_current_target(CMD_CTX);

	unsigned wordsize;
	switch (CMD_NAME[2]) {
		case 'w':
			wordsize = 4;
			break;
		case 'h':
			wordsize = 2;
			break;
		case 'b':
			wordsize = 1;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return target_fill_io(target, address, wordsize, value);
}

static const struct command_registration x86_32_exec_command_handlers[] = {
	{
		.name = "iww",
		.mode = COMMAND_EXEC,
		.handler = handle_iow_command,
		.help = "write I/O port word",
		.usage = "port data[word]",
	},
	{
		.name = "iwh",
		.mode = COMMAND_EXEC,
		.handler = handle_iow_command,
		.help = "write I/O port halfword",
		.usage = "port data[halfword]",
	},
	{
		.name = "iwb",
		.mode = COMMAND_EXEC,
		.handler = handle_iow_command,
		.help = "write I/O port byte",
		.usage = "port data[byte]",
	},
	{
		.name = "idw",
		.mode = COMMAND_EXEC,
		.handler = handle_iod_command,
		.help = "display I/O port word",
		.usage = "port",
	},
	{
		.name = "idh",
		.mode = COMMAND_EXEC,
		.handler = handle_iod_command,
		.help = "display I/O port halfword",
		.usage = "port",
	},
	{
		.name = "idb",
		.mode = COMMAND_EXEC,
		.handler = handle_iod_command,
		.help = "display I/O port byte",
		.usage = "port",
	},

	COMMAND_REGISTRATION_DONE
};

const struct command_registration x86_32_command_handlers[] = {
	{
		.name = "x86_32",
		.mode = COMMAND_ANY,
		.help = "x86_32 target commands",
		.usage = "",
		.chain = x86_32_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
