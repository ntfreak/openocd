
/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "arm920t.h"
#include <helper/time_support.h>
#include "target_type.h"
#include "register.h"
#include "arm_opcodes.h"

/*
 * For information about the ARM920T, see ARM DDI 0151C especially
 * Chapter 9 about debug support, which shows how to manipulate each
 * of the different scan chains:
 *
 *   0 ... ARM920 signals, e.g. to rest of SOC (unused here)
 *   1 ... debugging; watchpoint and breakpoint status, etc; also
 *	MMU and cache access in conjunction with scan chain 15
 *   2 ... EmbeddedICE
 *   3 ... external boundary scan (SoC-specific, unused here)
 *   4 ... access to cache tag RAM
 *   6 ... ETM9
 *   15 ... access coprocessor 15, "physical" or "interpreted" modes
 *	"interpreted" works with a few actual MRC/MCR instructions
 *	"physical" provides register-like behaviors.  Section 9.6.7
 *	covers these details.
 *
 * The ARM922T is similar, but with smaller caches (8K each, vs 16K).
 */

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

/* Table 9-8 shows scan chain 15 format during physical access mode, using a
 * dedicated 6-bit address space (encoded in bits 33:38).  Writes use one
 * JTAG scan, while reads use two.
 *
 * Table 9-9 lists the thirteen registers which support physical access.
 * ARM920T_CP15_PHYS_ADDR() constructs the 6-bit reg_addr parameter passed
 * to arm920t_read_cp15_physical() and arm920t_write_cp15_physical().
 *
 *  x == bit[38]
 *  y == bits[37:34]
 *  z == bit[33]
 */
#define ARM920T_CP15_PHYS_ADDR(x, y, z) ((x << 5) | (y << 1) << (z))

/* Registers supporting physical Read access (from table 9-9) */
#define CP15PHYS_CACHETYPE      ARM920T_CP15_PHYS_ADDR(0, 0x0, 1)
#define CP15PHYS_ICACHE_IDX     ARM920T_CP15_PHYS_ADDR(1, 0xd, 1)
#define CP15PHYS_DCACHE_IDX     ARM920T_CP15_PHYS_ADDR(1, 0xe, 1)
/* NOTE: several more registers support only physical read access */

/* Registers supporting physical Read/Write access (from table 9-9) */
#define CP15PHYS_CTRL           ARM920T_CP15_PHYS_ADDR(0, 0x1, 0)
#define CP15PHYS_PID            ARM920T_CP15_PHYS_ADDR(0, 0xd, 0)
#define CP15PHYS_TESTSTATE      ARM920T_CP15_PHYS_ADDR(0, 0xf, 0)
#define CP15PHYS_ICACHE         ARM920T_CP15_PHYS_ADDR(1, 0x1, 1)
#define CP15PHYS_DCACHE         ARM920T_CP15_PHYS_ADDR(1, 0x2, 1)

static int arm920t_read_cp15_physical(struct target *target,
	int reg_addr, uint32_t *value)
{
	struct arm920t_common *arm920t = target_to_arm920(target);
	struct arm_jtag *jtag_info;
	struct scan_field fields[4];
	uint8_t access_type_buf = 1;
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 0;
	int retval;

	jtag_info = &arm920t->arm7_9_common.jtag_info;

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].in_value = NULL;

	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].in_value = NULL;

	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].in_value = NULL;

	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);

	fields[1].in_value = (uint8_t *)value;

	jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);

	jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)value);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	jtag_execute_queue();
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, *value);
#endif

	return ERROR_OK;
}

static int arm920t_write_cp15_physical(struct target *target,
	int reg_addr, uint32_t value)
{
	struct arm920t_common *arm920t = target_to_arm920(target);
	struct arm_jtag *jtag_info;
	struct scan_field fields[4];
	uint8_t access_type_buf = 1;
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 1;
	uint8_t value_buf[4];
	int retval;

	jtag_info = &arm920t->arm7_9_common.jtag_info;

	buf_set_u32(value_buf, 0, 32, value);

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].in_value = NULL;

	fields[1].num_bits = 32;
	fields[1].out_value = value_buf;
	fields[1].in_value = NULL;

	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].in_value = NULL;

	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, value);
#endif

	return ERROR_OK;
}

/* See table 9-10 for scan chain 15 format during interpreted access mode.
 * If the TESTSTATE register is set for interpreted access, certain CP15
 * MRC and MCR instructions may be executed through scan chain 15.
 *
 * Tables 9-11, 9-12, and 9-13 show which MRC and MCR instructions can be
 * executed using scan chain 15 interpreted mode.
 */
static int arm920t_execute_cp15(struct target *target, uint32_t cp15_opcode,
	uint32_t arm_opcode)
{
	int retval;
	struct arm920t_common *arm920t = target_to_arm920(target);
	struct arm_jtag *jtag_info;
	struct scan_field fields[4];
	uint8_t access_type_buf = 0;		/* interpreted access */
	uint8_t reg_addr_buf = 0x0;
	uint8_t nr_w_buf = 0;
	uint8_t cp15_opcode_buf[4];

	jtag_info = &arm920t->arm7_9_common.jtag_info;

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	buf_set_u32(cp15_opcode_buf, 0, 32, cp15_opcode);

	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].in_value = NULL;

	fields[1].num_bits = 32;
	fields[1].out_value = cp15_opcode_buf;
	fields[1].in_value = NULL;

	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].in_value = NULL;

	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 4, fields, TAP_IDLE);

	arm9tdmi_clock_out(jtag_info, arm_opcode, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	retval = arm7_9_execute_sys_speed(target);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed executing JTAG queue");
		return retval;
	}

	return ERROR_OK;
}

static int arm920t_read_cp15_interpreted(struct target *target,
	uint32_t cp15_opcode, uint32_t address, uint32_t *value)
{
	struct arm *arm = target_to_arm(target);
	uint32_t *regs_p[16];
	uint32_t regs[16];
	uint32_t cp15c15 = 0x0;
	struct reg *r = arm->core_cache->reg_list;

	/* load address into R1 */
	regs[1] = address;
	arm9tdmi_write_core_regs(target, 0x2, regs);

	/* read-modify-write CP15 test state register
	* to enable interpreted access mode */
	arm920t_read_cp15_physical(target, CP15PHYS_TESTSTATE, &cp15c15);
	jtag_execute_queue();
	cp15c15 |= 1;	/* set interpret mode */
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

	/* execute CP15 instruction and ARM load (reading from coprocessor) */
	arm920t_execute_cp15(target, cp15_opcode, ARMV4_5_LDR(0, 1));

	/* disable interpreted access mode */
	cp15c15 &= ~1U;	/* clear interpret mode */
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

	/* retrieve value from R0 */
	regs_p[0] = value;
	arm9tdmi_read_core_regs(target, 0x1, regs_p);
	jtag_execute_queue();

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("cp15_opcode: %8.8x, address: %8.8x, value: %8.8x",
		cp15_opcode, address, *value);
#endif

	if (!is_arm_mode(arm->core_mode)) {
		LOG_ERROR("not a valid arm core mode - communication failure?");
		return ERROR_FAIL;
	}

	r[0].dirty = true;
	r[1].dirty = true;

	return ERROR_OK;
}

static
int arm920t_write_cp15_interpreted(struct target *target,
	uint32_t cp15_opcode, uint32_t value, uint32_t address)
{
	uint32_t cp15c15 = 0x0;
	struct arm *arm = target_to_arm(target);
	uint32_t regs[16];
	struct reg *r = arm->core_cache->reg_list;

	/* load value, address into R0, R1 */
	regs[0] = value;
	regs[1] = address;
	arm9tdmi_write_core_regs(target, 0x3, regs);

	/* read-modify-write CP15 test state register
	* to enable interpreted access mode */
	arm920t_read_cp15_physical(target, CP15PHYS_TESTSTATE, &cp15c15);
	jtag_execute_queue();
	cp15c15 |= 1;	/* set interpret mode */
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

	/* execute CP15 instruction and ARM store (writing to coprocessor) */
	arm920t_execute_cp15(target, cp15_opcode, ARMV4_5_STR(0, 1));

	/* disable interpreted access mode */
	cp15c15 &= ~1U;	/* set interpret mode */
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("cp15_opcode: %8.8x, value: %8.8x, address: %8.8x",
		cp15_opcode, value, address);
#endif

	if (!is_arm_mode(arm->core_mode)) {
		LOG_ERROR("not a valid arm core mode - communication failure?");
		return ERROR_FAIL;
	}

	r[0].dirty = true;
	r[1].dirty = true;

	return ERROR_OK;
}

/* EXPORTED to FA256 */
int arm920t_get_ttb(struct target *target, uint32_t *result)
{
	int retval;
	uint32_t ttb = 0x0;

	retval = arm920t_read_cp15_interpreted(target,
			/* FIXME use opcode macro */
			0xeebf0f51, 0x0, &ttb);
	if (retval != ERROR_OK)
		return retval;

	*result = ttb;
	return ERROR_OK;
}

/* EXPORTED to FA256 */
int arm920t_disable_mmu_caches(struct target *target, int mmu,
	int d_u_cache, int i_cache)
{
	uint32_t cp15_control;
	int retval;

	/* read cp15 control register */
	retval = arm920t_read_cp15_physical(target, CP15PHYS_CTRL, &cp15_control);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (mmu)
		cp15_control &= ~0x1U;

	if (d_u_cache)
		cp15_control &= ~0x4U;

	if (i_cache)
		cp15_control &= ~0x1000U;

	retval = arm920t_write_cp15_physical(target, CP15PHYS_CTRL, cp15_control);
	return retval;
}

/* EXPORTED to FA256 */
int arm920t_enable_mmu_caches(struct target *target, int mmu,
	int d_u_cache, int i_cache)
{
	uint32_t cp15_control;
	int retval;

	/* read cp15 control register */
	retval = arm920t_read_cp15_physical(target, CP15PHYS_CTRL, &cp15_control);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (mmu)
		cp15_control |= 0x1U;

	if (d_u_cache)
		cp15_control |= 0x4U;

	if (i_cache)
		cp15_control |= 0x1000U;

	retval = arm920t_write_cp15_physical(target, CP15PHYS_CTRL, cp15_control);
	return retval;
}

/* EXPORTED to FA256 */
int arm920t_post_debug_entry(struct target *target)
{
	uint32_t cp15c15;
	struct arm920t_common *arm920t = target_to_arm920(target);
	int retval;

	/* examine cp15 control reg */
	retval = arm920t_read_cp15_physical(target,
			CP15PHYS_CTRL, &arm920t->cp15_control_reg);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("cp15_control_reg: %8.8" PRIx32, arm920t->cp15_control_reg);

	if (arm920t->armv4_5_mmu.armv4_5_cache.ctype == -1) {
		uint32_t cache_type_reg;
		/* identify caches */
		retval = arm920t_read_cp15_physical(target,
				CP15PHYS_CACHETYPE, &cache_type_reg);
		if (retval != ERROR_OK)
			return retval;
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;
		armv4_5_identify_cache(cache_type_reg,
			&arm920t->armv4_5_mmu.armv4_5_cache);
	}

	arm920t->armv4_5_mmu.mmu_enabled =
		(arm920t->cp15_control_reg & 0x1U) ? 1 : 0;
	arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled =
		(arm920t->cp15_control_reg & 0x4U) ? 1 : 0;
	arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled =
		(arm920t->cp15_control_reg & 0x1000U) ? 1 : 0;

	/* save i/d fault status and address register
	 * FIXME use opcode macros */
	retval = arm920t_read_cp15_interpreted(target, 0xee150f10, 0x0, &arm920t->d_fsr);
	if (retval != ERROR_OK)
		return retval;
	retval = arm920t_read_cp15_interpreted(target, 0xee150f30, 0x0, &arm920t->i_fsr);
	if (retval != ERROR_OK)
		return retval;
	retval = arm920t_read_cp15_interpreted(target, 0xee160f10, 0x0, &arm920t->d_far);
	if (retval != ERROR_OK)
		return retval;
	retval = arm920t_read_cp15_interpreted(target, 0xee160f30, 0x0, &arm920t->i_far);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("D FSR: 0x%8.8" PRIx32 ", D FAR: 0x%8.8" PRIx32
		", I FSR: 0x%8.8" PRIx32 ", I FAR: 0x%8.8" PRIx32,
		arm920t->d_fsr, arm920t->d_far, arm920t->i_fsr, arm920t->i_far);

	if (arm920t->preserve_cache) {
		/* read-modify-write CP15 test state register
		 * to disable I/D-cache linefills */
		retval = arm920t_read_cp15_physical(target,
				CP15PHYS_TESTSTATE, &cp15c15);
		if (retval != ERROR_OK)
			return retval;
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;
		cp15c15 |= 0x600;
		retval = arm920t_write_cp15_physical(target,
				CP15PHYS_TESTSTATE, cp15c15);
		if (retval != ERROR_OK)
			return retval;
	}
	return ERROR_OK;
}

/* EXPORTED to FA256 */
void arm920t_pre_restore_context(struct target *target)
{
	uint32_t cp15c15 = 0;
	struct arm920t_common *arm920t = target_to_arm920(target);

	/* restore i/d fault status and address register */
	arm920t_write_cp15_interpreted(target, 0xee050f10, arm920t->d_fsr, 0x0);
	arm920t_write_cp15_interpreted(target, 0xee050f30, arm920t->i_fsr, 0x0);
	arm920t_write_cp15_interpreted(target, 0xee060f10, arm920t->d_far, 0x0);
	arm920t_write_cp15_interpreted(target, 0xee060f30, arm920t->i_far, 0x0);

	/* read-modify-write CP15 test state register
	* to reenable I/D-cache linefills */
	if (arm920t->preserve_cache) {
		arm920t_read_cp15_physical(target,
			CP15PHYS_TESTSTATE, &cp15c15);
		jtag_execute_queue();
		cp15c15 &= ~0x600U;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);
	}
}

static const char arm920_not[] = "target is not an ARM920";

static int arm920t_verify_pointer(struct command_invocation *cmd,
	struct arm920t_common *arm920t)
{
	if (arm920t->common_magic != ARM920T_COMMON_MAGIC) {
		command_print(cmd, arm920_not);
		return ERROR_TARGET_INVALID;
	}

	return ERROR_OK;
}

/** Logs summary of ARM920 state for a halted target. */
int arm920t_arch_state(struct target *target)
{
	static const char *state[] = {
		"disabled", "enabled"
	};

	struct arm920t_common *arm920t = target_to_arm920(target);

	if (arm920t->common_magic != ARM920T_COMMON_MAGIC) {
		LOG_ERROR("BUG: %s", arm920_not);
		return ERROR_TARGET_INVALID;
	}

	arm_arch_state(target);
	LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s",
		state[arm920t->armv4_5_mmu.mmu_enabled],
		state[arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled],
		state[arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled]);

	return ERROR_OK;
}

static int arm920_mmu(struct target *target, int *enabled)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_INVALID;
	}

	*enabled = target_to_arm920(target)->armv4_5_mmu.mmu_enabled;
	return ERROR_OK;
}

static int arm920_virt2phys(struct target *target,
	target_addr_t virt, target_addr_t *phys)
{
	uint32_t cb;
	struct arm920t_common *arm920t = target_to_arm920(target);

	uint32_t ret;
	int retval = armv4_5_mmu_translate_va(target,
			&arm920t->armv4_5_mmu, virt, &cb, &ret);
	if (retval != ERROR_OK)
		return retval;
	*phys = ret;
	return ERROR_OK;
}

/** Reads a buffer, in the specified word size, with current MMU settings. */
int arm920t_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;

	retval = arm7_9_read_memory(target, address, size, count, buffer);

	return retval;
}


static int arm920t_read_phys_memory(struct target *target,
	target_addr_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	struct arm920t_common *arm920t = target_to_arm920(target);

	return armv4_5_mmu_read_physical(target, &arm920t->armv4_5_mmu,
		address, size, count, buffer);
}

static int arm920t_write_phys_memory(struct target *target,
	target_addr_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	struct arm920t_common *arm920t = target_to_arm920(target);

	return armv4_5_mmu_write_physical(target, &arm920t->armv4_5_mmu,
		address, size, count, buffer);
}

/** Writes a buffer, in the specified word size, with current MMU settings. */
int arm920t_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;
	const uint32_t cache_mask = ~0x1f;	/* cache line size : 32 byte */
	struct arm920t_common *arm920t = target_to_arm920(target);

	/* FIX!!!! this should be cleaned up and made much more general. The
	 * plan is to write up and test on arm920t specifically and
	 * then generalize and clean up afterwards.
	 *
	 * Also it should be moved to the callbacks that handle breakpoints
	 * specifically and not the generic memory write fn's. See XScale code.
	 */
	if (arm920t->armv4_5_mmu.mmu_enabled && (count == 1) &&
			((size == 2) || (size == 4))) {
		/* special case the handling of single word writes to
		 * bypass MMU, to allow implementation of breakpoints
		 * in memory marked read only
		 * by MMU
		 */
		uint32_t cb;
		uint32_t pa;

		/*
		 * We need physical address and cb
		 */
		retval = armv4_5_mmu_translate_va(target, &arm920t->armv4_5_mmu,
				address, &cb, &pa);
		if (retval != ERROR_OK)
			return retval;

		if (arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled) {
			if (cb & 0x1) {
				LOG_DEBUG("D-Cache buffered, "
					"drain write buffer");
				/*
				 * Buffered ?
				 * Drain write buffer - MCR p15,0,Rd,c7,c10,4
				 */

				retval = arm920t_write_cp15_interpreted(target,
						ARMV4_5_MCR(15, 0, 0, 7, 10, 4),
						0x0, 0);
				if (retval != ERROR_OK)
					return retval;
			}

			if (cb == 0x3) {
				/*
				 * Write back memory ? -> clean cache
				 *
				 * There is no way to clean cache lines using
				 * cp15 scan chain, so copy the full cache
				 * line from cache to physical memory.
				 */
				uint8_t data[32];

				LOG_DEBUG("D-Cache in 'write back' mode, "
					"flush cache line");

				retval = target_read_memory(target,
						address & cache_mask, 1,
						sizeof(data), &data[0]);
				if (retval != ERROR_OK)
					return retval;

				retval = armv4_5_mmu_write_physical(target,
						&arm920t->armv4_5_mmu,
						pa & cache_mask, 1,
						sizeof(data), &data[0]);
				if (retval != ERROR_OK)
					return retval;
			}

			/* Cached ? */
			if (cb & 0x2) {
				/*
				 * Cached ? -> Invalidate data cache using MVA
				 *
				 * MCR p15,0,Rd,c7,c6,1
				 */
				LOG_DEBUG("D-Cache enabled, "
					"invalidate cache line");

				retval = arm920t_write_cp15_interpreted(target,
						ARMV4_5_MCR(15, 0, 0, 7, 6, 1), 0x0,
						address & cache_mask);
				if (retval != ERROR_OK)
					return retval;
			}
		}

		/* write directly to physical memory,
		 * bypassing any read only MMU bits, etc.
		 */
		retval = armv4_5_mmu_write_physical(target,
				&arm920t->armv4_5_mmu, pa, size,
				count, buffer);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = arm7_9_write_memory(target, address, size, count, buffer);
		if (retval != ERROR_OK)
			return retval;
	}

	/* If ICache is enabled, we have to invalidate affected ICache lines
	 * the DCache is forced to write-through,
	 * so we don't have to clean it here
	 */
	if (arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled) {
		if (count <= 1) {
			/* invalidate ICache single entry with MVA
			 *   mcr	15, 0, r0, cr7, cr5, {1}
			 */
			LOG_DEBUG("I-Cache enabled, "
				"invalidating affected I-Cache line");
			retval = arm920t_write_cp15_interpreted(target,
					ARMV4_5_MCR(15, 0, 0, 7, 5, 1),
					0x0, address & cache_mask);
			if (retval != ERROR_OK)
				return retval;
		} else {
			/* invalidate ICache
			 *  mcr	15, 0, r0, cr7, cr5, {0}
			 */
			retval = arm920t_write_cp15_interpreted(target,
					ARMV4_5_MCR(15, 0, 0, 7, 5, 0),
					0x0, 0x0);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	return ERROR_OK;
}

/* EXPORTED to FA256 */
int arm920t_soft_reset_halt(struct target *target)
{
	int retval = ERROR_OK;
	struct arm920t_common *arm920t = target_to_arm920(target);
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm *arm = &arm7_9->arm;
	struct reg *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	int64_t then = timeval_ms();
	bool timeout;
	while (!(timeout = ((timeval_ms()-then) > 1000))) {
		if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1) == 0) {
			embeddedice_read_reg(dbg_stat);
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;
		} else
			break;
		if (debug_level >= 3) {
			/* do not eat all CPU, time out after 1 se*/
			alive_sleep(100);
		} else
			keep_alive();
	}
	if (timeout) {
		LOG_ERROR("Failed to halt CPU after 1 sec");
		return ERROR_TARGET_TIMEOUT;
	}

	target->state = TARGET_HALTED;

	/* SVC, ARM state, IRQ and FIQ disabled */
	uint32_t cpsr;

	cpsr = buf_get_u32(arm->cpsr->value, 0, 32);
	cpsr &= ~0xff;
	cpsr |= 0xd3;
	arm_set_cpsr(arm, cpsr);
	arm->cpsr->dirty = true;

	/* start fetching from 0x0 */
	buf_set_u32(arm->pc->value, 0, 32, 0x0);
	arm->pc->dirty = true;
	arm->pc->valid = true;

	arm920t_disable_mmu_caches(target, 1, 1, 1);
	arm920t->armv4_5_mmu.mmu_enabled = 0;
	arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 0;
	arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	return target_call_event_callbacks(target, TARGET_EVENT_HALTED);
}

/* FIXME remove forward decls */
static int arm920t_mrc(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2,
		uint32_t crn, uint32_t crm,
		uint32_t *value);
static int arm920t_mcr(struct target *target, int cpnum,
		uint32_t op1, uint32_t op2,
		uint32_t crn, uint32_t crm,
		uint32_t value);

static int arm920t_init_arch_info(struct target *target,
	struct arm920t_common *arm920t, struct jtag_tap *tap)
{
	struct arm7_9_common *arm7_9 = &arm920t->arm7_9_common;

	arm7_9->arm.mrc = arm920t_mrc;
	arm7_9->arm.mcr = arm920t_mcr;

	/* initialize arm7/arm9 specific info (including armv4_5) */
	arm9tdmi_init_arch_info(target, arm7_9, tap);

	arm920t->common_magic = ARM920T_COMMON_MAGIC;

	arm7_9->post_debug_entry = arm920t_post_debug_entry;
	arm7_9->pre_restore_context = arm920t_pre_restore_context;
	arm7_9->write_memory = arm920t_write_memory;

	arm920t->armv4_5_mmu.armv4_5_cache.ctype = -1;
	arm920t->armv4_5_mmu.get_ttb = arm920t_get_ttb;
	arm920t->armv4_5_mmu.read_memory = arm7_9_read_memory;
	arm920t->armv4_5_mmu.write_memory = arm7_9_write_memory;
	arm920t->armv4_5_mmu.disable_mmu_caches = arm920t_disable_mmu_caches;
	arm920t->armv4_5_mmu.enable_mmu_caches = arm920t_enable_mmu_caches;
	arm920t->armv4_5_mmu.has_tiny_pages = 1;
	arm920t->armv4_5_mmu.mmu_enabled = 0;

	/* disabling linefills leads to lockups, so keep them enabled for now
	 * this doesn't affect correctness, but might affect timing issues, if
	 * important data is evicted from the cache during the debug session
	 * */
	arm920t->preserve_cache = 0;

	/* override hw single-step capability from ARM9TDMI */
	arm7_9->has_single_step = 1;

	return ERROR_OK;
}

static int arm920t_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm920t_common *arm920t;

	arm920t = calloc(1, sizeof(struct arm920t_common));
	return arm920t_init_arch_info(target, arm920t, target->tap);
}

static void arm920t_deinit_target(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm920t_common *arm920t = target_to_arm920(target);

	arm7_9_deinit(target);
	arm_free_reg_cache(arm);
	free(arm920t);
}

COMMAND_HANDLER(arm920t_handle_read_cache_command)
{
	int retval = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);
	struct arm920t_common *arm920t = target_to_arm920(target);
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm *arm = &arm7_9->arm;
	uint32_t cp15c15;
	uint32_t cp15_ctrl, cp15_ctrl_saved;
	uint32_t regs[16];
	uint32_t *regs_p[16];
	uint32_t c15_c_d_ind, c15_c_i_ind;
	int i;
	FILE *output;
	int segment, index_t;
	struct reg *r;

	retval = arm920t_verify_pointer(CMD, arm920t);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	output = fopen(CMD_ARGV[0], "w");
	if (!output) {
		LOG_DEBUG("error opening cache content file");
		return ERROR_OK;
	}

	for (i = 0; i < 16; i++)
		regs_p[i] = &regs[i];

	/* disable MMU and Caches */
	arm920t_read_cp15_physical(target, CP15PHYS_CTRL, &cp15_ctrl);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	cp15_ctrl_saved = cp15_ctrl;
	cp15_ctrl &= ~(ARMV4_5_MMU_ENABLED
			| ARMV4_5_D_U_CACHE_ENABLED | ARMV4_5_I_CACHE_ENABLED);
	arm920t_write_cp15_physical(target, CP15PHYS_CTRL, cp15_ctrl);

	/* read CP15 test state register */
	arm920t_read_cp15_physical(target, CP15PHYS_TESTSTATE, &cp15c15);
	jtag_execute_queue();

	/* read DCache content */
	fprintf(output, "DCache:\n");

	/* go through segments 0 to nsets (8 on ARM920T, 4 on ARM922T) */
	for (segment = 0;
			segment < arm920t->armv4_5_mmu.armv4_5_cache.d_u_size.nsets;
			segment++) {
		fprintf(output, "\nsegment: %i\n----------", segment);

		/* Ra: r0 = SBZ(31:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* D CAM Read, loads current victim into C15.C.D.Ind */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 2, 0, 15, 6, 2), ARMV4_5_LDR(1, 0));

		/* read current victim */
		arm920t_read_cp15_physical(target,
			CP15PHYS_DCACHE_IDX, &c15_c_d_ind);

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		for (index_t = 0; index_t < 64; index_t++) {
			/* Ra:
			 * r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0)
			 */
			regs[0] = 0x0 | (segment << 5) | (index_t << 26);
			arm9tdmi_write_core_regs(target, 0x1, regs);

			/* set interpret mode */
			cp15c15 |= 0x1;
			arm920t_write_cp15_physical(target,
				CP15PHYS_TESTSTATE, cp15c15);

			/* Write DCache victim */
			arm920t_execute_cp15(target,
				ARMV4_5_MCR(15, 0, 0, 9, 1, 0), ARMV4_5_LDR(1, 0));

			/* Read D RAM */
			arm920t_execute_cp15(target,
				ARMV4_5_MCR(15, 2, 0, 15, 10, 2),
				ARMV4_5_LDMIA(0, 0x1fe, 0, 0));

			/* Read D CAM */
			arm920t_execute_cp15(target,
				ARMV4_5_MCR(15, 2, 0, 15, 6, 2),
				ARMV4_5_LDR(9, 0));

			/* clear interpret mode */
			cp15c15 &= ~0x1;
			arm920t_write_cp15_physical(target,
				CP15PHYS_TESTSTATE, cp15c15);

			/* read D RAM and CAM content */
			arm9tdmi_read_core_regs(target, 0x3fe, regs_p);
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;

			/* mask LFSR[6] */
			regs[9] &= 0xfffffffe;
			fprintf(output, "\nsegment: %i, index: %i, CAM: 0x%8.8"
				PRIx32 ", content (%s):\n",
				segment, index_t, regs[9],
				(regs[9] & 0x10) ? "valid" : "invalid");

			for (i = 1; i < 9; i++) {
				fprintf(output, "%i: 0x%8.8" PRIx32 "\n",
					i-1, regs[i]);
			}

		}

		/* Ra: r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5) | (c15_c_d_ind << 26);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* Write DCache victim */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 0, 0, 9, 1, 0), ARMV4_5_LDR(1, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);
	}

	/* read ICache content */
	fprintf(output, "ICache:\n");

	/* go through segments 0 to nsets (8 on ARM920T, 4 on ARM922T) */
	for (segment = 0;
			segment < arm920t->armv4_5_mmu.armv4_5_cache.d_u_size.nsets;
			segment++) {
		fprintf(output, "segment: %i\n----------", segment);

		/* Ra: r0 = SBZ(31:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* I CAM Read, loads current victim into C15.C.I.Ind */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 2, 0, 15, 5, 2), ARMV4_5_LDR(1, 0));

		/* read current victim */
		arm920t_read_cp15_physical(target, CP15PHYS_ICACHE_IDX,
			&c15_c_i_ind);

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		for (index_t = 0; index_t < 64; index_t++) {
			/* Ra:
			 * r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0)
			 */
			regs[0] = 0x0 | (segment << 5) | (index_t << 26);
			arm9tdmi_write_core_regs(target, 0x1, regs);

			/* set interpret mode */
			cp15c15 |= 0x1;
			arm920t_write_cp15_physical(target,
				CP15PHYS_TESTSTATE, cp15c15);

			/* Write ICache victim */
			arm920t_execute_cp15(target,
				ARMV4_5_MCR(15, 0, 0, 9, 1, 1), ARMV4_5_LDR(1, 0));

			/* Read I RAM */
			arm920t_execute_cp15(target,
				ARMV4_5_MCR(15, 2, 0, 15, 9, 2),
				ARMV4_5_LDMIA(0, 0x1fe, 0, 0));

			/* Read I CAM */
			arm920t_execute_cp15(target,
				ARMV4_5_MCR(15, 2, 0, 15, 5, 2),
				ARMV4_5_LDR(9, 0));

			/* clear interpret mode */
			cp15c15 &= ~0x1;
			arm920t_write_cp15_physical(target,
				CP15PHYS_TESTSTATE, cp15c15);

			/* read I RAM and CAM content */
			arm9tdmi_read_core_regs(target, 0x3fe, regs_p);
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;

			/* mask LFSR[6] */
			regs[9] &= 0xfffffffe;
			fprintf(output, "\nsegment: %i, index: %i, "
				"CAM: 0x%8.8" PRIx32 ", content (%s):\n",
				segment, index_t, regs[9],
				(regs[9] & 0x10) ? "valid" : "invalid");

			for (i = 1; i < 9; i++) {
				fprintf(output, "%i: 0x%8.8" PRIx32 "\n",
					i-1, regs[i]);
			}
		}

		/* Ra: r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5) | (c15_c_d_ind << 26);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* Write ICache victim */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 0, 0, 9, 1, 1), ARMV4_5_LDR(1, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);
	}

	/* restore CP15 MMU and Cache settings */
	arm920t_write_cp15_physical(target, CP15PHYS_CTRL, cp15_ctrl_saved);

	command_print(CMD, "cache content successfully output to %s",
		CMD_ARGV[0]);

	fclose(output);

	if (!is_arm_mode(arm->core_mode)) {
		LOG_ERROR("not a valid arm core mode - communication failure?");
		return ERROR_FAIL;
	}

	/* force writeback of the valid data */
	r = arm->core_cache->reg_list;
	r[0].dirty = r[0].valid;
	r[1].dirty = r[1].valid;
	r[2].dirty = r[2].valid;
	r[3].dirty = r[3].valid;
	r[4].dirty = r[4].valid;
	r[5].dirty = r[5].valid;
	r[6].dirty = r[6].valid;
	r[7].dirty = r[7].valid;

	r = arm_reg_current(arm, 8);
	r->dirty = r->valid;

	r = arm_reg_current(arm, 9);
	r->dirty = r->valid;

	return ERROR_OK;
}

COMMAND_HANDLER(arm920t_handle_read_mmu_command)
{
	int retval = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);
	struct arm920t_common *arm920t = target_to_arm920(target);
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm *arm = &arm7_9->arm;
	uint32_t cp15c15;
	uint32_t cp15_ctrl, cp15_ctrl_saved;
	uint32_t regs[16];
	uint32_t *regs_p[16];
	int i;
	FILE *output;
	uint32_t d_lockdown, i_lockdown;
	struct arm920t_tlb_entry d_tlb[64], i_tlb[64];
	int victim;
	struct reg *r;

	retval = arm920t_verify_pointer(CMD, arm920t);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	output = fopen(CMD_ARGV[0], "w");
	if (!output) {
		LOG_DEBUG("error opening mmu content file");
		return ERROR_OK;
	}

	for (i = 0; i < 16; i++)
		regs_p[i] = &regs[i];

	/* disable MMU and Caches */
	arm920t_read_cp15_physical(target, CP15PHYS_CTRL, &cp15_ctrl);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	cp15_ctrl_saved = cp15_ctrl;
	cp15_ctrl &= ~(ARMV4_5_MMU_ENABLED
			| ARMV4_5_D_U_CACHE_ENABLED | ARMV4_5_I_CACHE_ENABLED);
	arm920t_write_cp15_physical(target, CP15PHYS_CTRL, cp15_ctrl);

	/* read CP15 test state register */
	arm920t_read_cp15_physical(target, CP15PHYS_TESTSTATE, &cp15c15);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* prepare reading D TLB content
	 * */

	/* set interpret mode */
	cp15c15 |= 0x1;
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

	/* Read D TLB lockdown */
	arm920t_execute_cp15(target,
		ARMV4_5_MRC(15, 0, 0, 10, 0, 0), ARMV4_5_LDR(1, 0));

	/* clear interpret mode */
	cp15c15 &= ~0x1;
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

	/* read D TLB lockdown stored to r1 */
	arm9tdmi_read_core_regs(target, 0x2, regs_p);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	d_lockdown = regs[1];

	for (victim = 0; victim < 64; victim += 8) {
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63
		 */
		regs[1] = (d_lockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* Write D TLB lockdown */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 0, 0, 10, 0, 0),
			ARMV4_5_STR(1, 0));

		/* Read D TLB CAM */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 4, 0, 15, 6, 4),
			ARMV4_5_LDMIA(0, 0x3fc, 0, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* read D TLB CAM content stored to r2-r9 */
		arm9tdmi_read_core_regs(target, 0x3fc, regs_p);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		for (i = 0; i < 8; i++)
			d_tlb[victim + i].cam = regs[i + 2];
	}

	for (victim = 0; victim < 64; victim++) {
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63
		 */
		regs[1] = (d_lockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* Write D TLB lockdown */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 0, 0, 10, 0, 0), ARMV4_5_STR(1, 0));

		/* Read D TLB RAM1 */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 4, 0, 15, 10, 4), ARMV4_5_LDR(2, 0));

		/* Read D TLB RAM2 */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 4, 0, 15, 2, 5), ARMV4_5_LDR(3, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* read D TLB RAM content stored to r2 and r3 */
		arm9tdmi_read_core_regs(target, 0xc, regs_p);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		d_tlb[victim].ram1 = regs[2];
		d_tlb[victim].ram2 = regs[3];
	}

	/* restore D TLB lockdown */
	regs[1] = d_lockdown;
	arm9tdmi_write_core_regs(target, 0x2, regs);

	/* Write D TLB lockdown */
	arm920t_execute_cp15(target,
		ARMV4_5_MCR(15, 0, 0, 10, 0, 0), ARMV4_5_STR(1, 0));

	/* prepare reading I TLB content
	 * */

	/* set interpret mode */
	cp15c15 |= 0x1;
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

	/* Read I TLB lockdown */
	arm920t_execute_cp15(target,
		ARMV4_5_MRC(15, 0, 0, 10, 0, 1), ARMV4_5_LDR(1, 0));

	/* clear interpret mode */
	cp15c15 &= ~0x1;
	arm920t_write_cp15_physical(target, CP15PHYS_TESTSTATE, cp15c15);

	/* read I TLB lockdown stored to r1 */
	arm9tdmi_read_core_regs(target, 0x2, regs_p);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	i_lockdown = regs[1];

	for (victim = 0; victim < 64; victim += 8) {
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63
		 */
		regs[1] = (i_lockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* Write I TLB lockdown */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 0, 0, 10, 0, 1),
			ARMV4_5_STR(1, 0));

		/* Read I TLB CAM */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 4, 0, 15, 5, 4),
			ARMV4_5_LDMIA(0, 0x3fc, 0, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* read I TLB CAM content stored to r2-r9 */
		arm9tdmi_read_core_regs(target, 0x3fc, regs_p);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		for (i = 0; i < 8; i++)
			i_tlb[i + victim].cam = regs[i + 2];
	}

	for (victim = 0; victim < 64; victim++) {
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63
		 */
		regs[1] = (d_lockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* Write I TLB lockdown */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 0, 0, 10, 0, 1), ARMV4_5_STR(1, 0));

		/* Read I TLB RAM1 */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 4, 0, 15, 9, 4), ARMV4_5_LDR(2, 0));

		/* Read I TLB RAM2 */
		arm920t_execute_cp15(target,
			ARMV4_5_MCR(15, 4, 0, 15, 1, 5), ARMV4_5_LDR(3, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target,
			CP15PHYS_TESTSTATE, cp15c15);

		/* read I TLB RAM content stored to r2 and r3 */
		arm9tdmi_read_core_regs(target, 0xc, regs_p);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;

		i_tlb[victim].ram1 = regs[2];
		i_tlb[victim].ram2 = regs[3];
	}

	/* restore I TLB lockdown */
	regs[1] = i_lockdown;
	arm9tdmi_write_core_regs(target, 0x2, regs);

	/* Write I TLB lockdown */
	arm920t_execute_cp15(target,
		ARMV4_5_MCR(15, 0, 0, 10, 0, 1), ARMV4_5_STR(1, 0));

	/* restore CP15 MMU and Cache settings */
	arm920t_write_cp15_physical(target, CP15PHYS_CTRL, cp15_ctrl_saved);

	/* output data to file */
	fprintf(output, "D TLB content:\n");
	for (i = 0; i < 64; i++) {
		fprintf(output, "%i: 0x%8.8" PRIx32 " 0x%8.8" PRIx32
			" 0x%8.8" PRIx32 " %s\n",
			i, d_tlb[i].cam, d_tlb[i].ram1, d_tlb[i].ram2,
			(d_tlb[i].cam & 0x20) ? "(valid)" : "(invalid)");
	}

	fprintf(output, "\n\nI TLB content:\n");
	for (i = 0; i < 64; i++) {
		fprintf(output, "%i: 0x%8.8" PRIx32 " 0x%8.8" PRIx32
			" 0x%8.8" PRIx32 " %s\n",
			i, i_tlb[i].cam, i_tlb[i].ram1, i_tlb[i].ram2,
			(i_tlb[i].cam & 0x20) ? "(valid)" : "(invalid)");
	}

	command_print(CMD, "mmu content successfully output to %s",
		CMD_ARGV[0]);

	fclose(output);

	if (!is_arm_mode(arm->core_mode)) {
		LOG_ERROR("not a valid arm core mode - communication failure?");
		return ERROR_FAIL;
	}

	/* force writeback of the valid data */
	r = arm->core_cache->reg_list;
	r[0].dirty = r[0].valid;
	r[1].dirty = r[1].valid;
	r[2].dirty = r[2].valid;
	r[3].dirty = r[3].valid;
	r[4].dirty = r[4].valid;
	r[5].dirty = r[5].valid;
	r[6].dirty = r[6].valid;
	r[7].dirty = r[7].valid;

	r = arm_reg_current(arm, 8);
	r->dirty = r->valid;

	r = arm_reg_current(arm, 9);
	r->dirty = r->valid;

	return ERROR_OK;
}

COMMAND_HANDLER(arm920t_handle_cp15_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct arm920t_common *arm920t = target_to_arm920(target);

	retval = arm920t_verify_pointer(CMD, arm920t);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for "
			"\"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* one argument, read a register.
	 * two arguments, write it.
	 */
	if (CMD_ARGC >= 1) {
		int address;
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], address);

		if (CMD_ARGC == 1) {
			uint32_t value;
			retval = arm920t_read_cp15_physical(target, address, &value);
			if (retval != ERROR_OK) {
				command_print(CMD,
					"couldn't access reg %i", address);
				return ERROR_OK;
			}
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;

			command_print(CMD, "%i: %8.8" PRIx32,
				address, value);
		} else if (CMD_ARGC == 2)   {
			uint32_t value;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
			retval = arm920t_write_cp15_physical(target,
					address, value);
			if (retval != ERROR_OK) {
				command_print(CMD,
					"couldn't access reg %i", address);
				/* REVISIT why lie? "return retval"? */
				return ERROR_OK;
			}
			command_print(CMD, "%i: %8.8" PRIx32,
				address, value);
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(arm920t_handle_cache_info_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct arm920t_common *arm920t = target_to_arm920(target);

	retval = arm920t_verify_pointer(CMD, arm920t);
	if (retval != ERROR_OK)
		return retval;

	return armv4_5_handle_cache_info_command(CMD,
		&arm920t->armv4_5_mmu.armv4_5_cache);
}


static int arm920t_mrc(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2,
	uint32_t crn, uint32_t crm,
	uint32_t *value)
{
	if (cpnum != 15) {
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}

	/* read "to" r0 */
	return arm920t_read_cp15_interpreted(target,
		ARMV4_5_MRC(cpnum, op1, 0, crn, crm, op2),
		0, value);
}

static int arm920t_mcr(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2,
	uint32_t crn, uint32_t crm,
	uint32_t value)
{
	if (cpnum != 15) {
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}

	/* write "from" r0 */
	return arm920t_write_cp15_interpreted(target,
		ARMV4_5_MCR(cpnum, op1, 0, crn, crm, op2),
		0, value);
}

static const struct command_registration arm920t_exec_command_handlers[] = {
	{
		.name = "cp15",
		.handler = arm920t_handle_cp15_command,
		.mode = COMMAND_EXEC,
		.help = "display/modify cp15 register",
		.usage = "regnum [value]",
	},
	{
		.name = "cache_info",
		.handler = arm920t_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "display information about target caches",
	},
	{
		.name = "read_cache",
		.handler = arm920t_handle_read_cache_command,
		.mode = COMMAND_EXEC,
		.help = "dump I/D cache content to file",
		.usage = "filename",
	},
	{
		.name = "read_mmu",
		.handler = arm920t_handle_read_mmu_command,
		.mode = COMMAND_EXEC,
		.help = "dump I/D mmu content to file",
		.usage = "filename",
	},
	COMMAND_REGISTRATION_DONE
};
const struct command_registration arm920t_command_handlers[] = {
	{
		.chain = arm9tdmi_command_handlers,
	},
	{
		.name = "arm920t",
		.mode = COMMAND_ANY,
		.help = "arm920t command group",
		.usage = "",
		.chain = arm920t_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM920 targets. */
struct target_type arm920t_target = {
	.name = "arm920t",

	.poll = arm7_9_poll,
	.arch_state = arm920t_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm920t_soft_reset_halt,

	.get_gdb_arch = arm_get_gdb_arch,
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = arm920t_read_memory,
	.write_memory = arm7_9_write_memory_opt,
	.read_phys_memory = arm920t_read_phys_memory,
	.write_phys_memory = arm920t_write_phys_memory,
	.mmu = arm920_mmu,
	.virt2phys = arm920_virt2phys,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.commands = arm920t_command_handlers,
	.target_create = arm920t_target_create,
	.init_target = arm9tdmi_init_target,
	.deinit_target = arm920t_deinit_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
