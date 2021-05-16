/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   <ilg@livius.net>                                                      *
 *                                                                         *
 *   Copyright (C) 2019 by Tomas Vanek                                     *
 *   vanekt@fbl.cz                                                         *
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
 *                                                                         *
 *   ARMv7-M Architecture, Application Level Reference Manual              *
 *              ARM DDI 0405C (September 2008)                             *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "armv7m.h"
#include "algorithm.h"
#include "register.h"
#include "semihosting_common.h"

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

static const char * const armv7m_exception_strings[] = {
	"", "Reset", "NMI", "HardFault",
	"MemManage", "BusFault", "UsageFault", "SecureFault",
	"RESERVED", "RESERVED", "RESERVED", "SVCall",
	"DebugMonitor", "RESERVED", "PendSV", "SysTick"
};

/* PSP is used in some thread modes */
const int armv7m_psp_reg_map[ARMV7M_NUM_CORE_REGS] = {
	ARMV7M_R0, ARMV7M_R1, ARMV7M_R2, ARMV7M_R3,
	ARMV7M_R4, ARMV7M_R5, ARMV7M_R6, ARMV7M_R7,
	ARMV7M_R8, ARMV7M_R9, ARMV7M_R10, ARMV7M_R11,
	ARMV7M_R12, ARMV7M_PSP, ARMV7M_R14, ARMV7M_PC,
	ARMV7M_xPSR,
};

/* MSP is used in handler and some thread modes */
const int armv7m_msp_reg_map[ARMV7M_NUM_CORE_REGS] = {
	ARMV7M_R0, ARMV7M_R1, ARMV7M_R2, ARMV7M_R3,
	ARMV7M_R4, ARMV7M_R5, ARMV7M_R6, ARMV7M_R7,
	ARMV7M_R8, ARMV7M_R9, ARMV7M_R10, ARMV7M_R11,
	ARMV7M_R12, ARMV7M_MSP, ARMV7M_R14, ARMV7M_PC,
	ARMV7M_xPSR,
};

/*
 * These registers are not memory-mapped.  The ARMv7-M profile includes
 * memory mapped registers too, such as for the NVIC (interrupt controller)
 * and SysTick (timer) modules; those can mostly be treated as peripherals.
 *
 * The ARMv6-M profile is almost identical in this respect, except that it
 * doesn't include basepri or faultmask registers.
 */
static const struct {
	unsigned id;
	const char *name;
	unsigned bits;
	enum reg_type type;
	const char *group;
	const char *feature;
} armv7m_regs[] = {
	{ ARMV7M_R0, "r0", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R1, "r1", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R2, "r2", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R3, "r3", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R4, "r4", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R5, "r5", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R6, "r6", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R7, "r7", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R8, "r8", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R9, "r9", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R10, "r10", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R11, "r11", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R12, "r12", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R13, "sp", 32, REG_TYPE_DATA_PTR, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R14, "lr", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_PC, "pc", 32, REG_TYPE_CODE_PTR, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_xPSR, "xPSR", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },

	{ ARMV7M_MSP, "msp", 32, REG_TYPE_DATA_PTR, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_PSP, "psp", 32, REG_TYPE_DATA_PTR, "system", "org.gnu.gdb.arm.m-system" },

	/* A working register for packing/unpacking special regs, hidden from gdb */
	{ ARMV7M_PMSK_BPRI_FLTMSK_CTRL, "pmsk_bpri_fltmsk_ctrl", 32, REG_TYPE_INT, NULL, NULL },

	/* WARNING: If you use armv7m_write_core_reg() on one of 4 following
	 * special registers, the new data go to ARMV7M_PMSK_BPRI_FLTMSK_CTRL
	 * cache only and are not flushed to CPU HW register.
	 * To trigger write to CPU HW register, add
	 *		armv7m_write_core_reg(,,ARMV7M_PMSK_BPRI_FLTMSK_CTRL,);
	 */
	{ ARMV7M_PRIMASK, "primask", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_BASEPRI, "basepri", 8, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_FAULTMASK, "faultmask", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_CONTROL, "control", 3, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },

	/* ARMv8-M specific registers */
	{ ARMV8M_MSP_NS, "msp_ns", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },
	{ ARMV8M_PSP_NS, "psp_ns", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },
	{ ARMV8M_MSP_S, "msp_s", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },
	{ ARMV8M_PSP_S, "psp_s", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },
	{ ARMV8M_MSPLIM_S, "msplim_s", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },
	{ ARMV8M_PSPLIM_S, "psplim_s", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },
	{ ARMV8M_MSPLIM_NS, "msplim_ns", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },
	{ ARMV8M_PSPLIM_NS, "psplim_ns", 32, REG_TYPE_DATA_PTR, "stack", "v8-m.sp" },

	{ ARMV8M_PMSK_BPRI_FLTMSK_CTRL_S, "pmsk_bpri_fltmsk_ctrl_s", 32, REG_TYPE_INT, NULL, NULL },
	{ ARMV8M_PRIMASK_S, "primask_s", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV8M_BASEPRI_S, "basepri_s", 8, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV8M_FAULTMASK_S, "faultmask_s", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV8M_CONTROL_S, "control_s", 3, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },

	{ ARMV8M_PMSK_BPRI_FLTMSK_CTRL_NS, "pmsk_bpri_fltmsk_ctrl_ns", 32, REG_TYPE_INT, NULL, NULL },
	{ ARMV8M_PRIMASK_NS, "primask_ns", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV8M_BASEPRI_NS, "basepri_ns", 8, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV8M_FAULTMASK_NS, "faultmask_ns", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV8M_CONTROL_NS, "control_ns", 3, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },

	/* FPU registers */
	{ ARMV7M_D0, "d0", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D1, "d1", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D2, "d2", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D3, "d3", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D4, "d4", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D5, "d5", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D6, "d6", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D7, "d7", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D8, "d8", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D9, "d9", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D10, "d10", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D11, "d11", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D12, "d12", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D13, "d13", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D14, "d14", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },
	{ ARMV7M_D15, "d15", 64, REG_TYPE_IEEE_DOUBLE, "float", "org.gnu.gdb.arm.vfp" },

	{ ARMV7M_FPSCR, "fpscr", 32, REG_TYPE_INT, "float", "org.gnu.gdb.arm.vfp" },
};

#define ARMV7M_NUM_REGS ARRAY_SIZE(armv7m_regs)

/**
 * Restores target context using the cache of core registers set up
 * by armv7m_build_reg_cache(), calling optional core-specific hooks.
 */
int armv7m_restore_context(struct target *target)
{
	int i;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct reg_cache *cache = armv7m->arm.core_cache;

	LOG_DEBUG(" ");

	if (armv7m->pre_restore_context)
		armv7m->pre_restore_context(target);

	/* The descending order of register writes is crucial for correct
	 * packing of ARMV7M_PMSK_BPRI_FLTMSK_CTRL!
	 * See also comments in the register table above */
	for (i = cache->num_regs - 1; i >= 0; i--) {
		struct reg *r = &cache->reg_list[i];

		if (r->exist && r->dirty)
			armv7m->arm.write_core_reg(target, r, i, ARM_MODE_ANY, r->value);
	}

	return ERROR_OK;
}

/* Core state functions */

/**
 * Maps ISR number (from xPSR) to name.
 * Note that while names and meanings for the first sixteen are standardized
 * (with zero not a true exception), external interrupts are only numbered.
 * They are assigned by vendors, which generally assign different numbers to
 * peripherals (such as UART0 or a USB peripheral controller).
 */
const char *armv7m_exception_string(int number)
{
	static char enamebuf[32];

	if ((number < 0) | (number > 511))
		return "Invalid exception";
	if (number < 16)
		return armv7m_exception_strings[number];
	sprintf(enamebuf, "External Interrupt(%i)", number - 16);
	return enamebuf;
}

static int armv7m_get_core_reg(struct reg *reg)
{
	int retval;
	struct arm_reg *armv7m_reg = reg->arch_info;
	struct target *target = armv7m_reg->target;
	struct arm *arm = target_to_arm(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = arm->read_core_reg(target, reg, reg->number, arm->core_mode);

	return retval;
}

static int armv7m_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct arm_reg *armv7m_reg = reg->arch_info;
	struct target *target = armv7m_reg->target;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_cpy(buf, reg->value, reg->size);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static uint32_t armv7m_map_id_to_regsel(unsigned int arm_reg_id)
{
	switch (arm_reg_id) {
	case ARMV7M_R0 ... ARMV7M_R14:
	case ARMV7M_PC:
	case ARMV7M_xPSR:
	case ARMV7M_MSP:
	case ARMV7M_PSP:
		/* NOTE:  we "know" here that the register identifiers
		 * match the Cortex-M DCRSR.REGSEL selectors values
		 * for R0..R14, PC, xPSR, MSP, and PSP.
		 */
		return arm_reg_id;

	case ARMV7M_PMSK_BPRI_FLTMSK_CTRL:
		return ARMV7M_REGSEL_PMSK_BPRI_FLTMSK_CTRL;

	case ARMV8M_MSP_NS...ARMV8M_PSPLIM_NS:
		return arm_reg_id - ARMV8M_MSP_NS + ARMV8M_REGSEL_MSP_NS;

	case ARMV8M_PMSK_BPRI_FLTMSK_CTRL_S:
		return ARMV8M_REGSEL_PMSK_BPRI_FLTMSK_CTRL_S;

	case ARMV8M_PMSK_BPRI_FLTMSK_CTRL_NS:
		return ARMV8M_REGSEL_PMSK_BPRI_FLTMSK_CTRL_NS;

	case ARMV7M_FPSCR:
		return ARMV7M_REGSEL_FPSCR;

	case ARMV7M_D0 ... ARMV7M_D15:
		return ARMV7M_REGSEL_S0 + 2 * (arm_reg_id - ARMV7M_D0);

	default:
		LOG_ERROR("Bad register ID %u", arm_reg_id);
		return arm_reg_id;
	}
}

static bool armv7m_map_reg_packing(unsigned int arm_reg_id,
					unsigned int *reg32_id, uint32_t *offset)
{

	switch (arm_reg_id) {

	case ARMV7M_PRIMASK...ARMV7M_CONTROL:
		*reg32_id = ARMV7M_PMSK_BPRI_FLTMSK_CTRL;
		*offset = arm_reg_id - ARMV7M_PRIMASK;
		return true;
	case ARMV8M_PRIMASK_S...ARMV8M_CONTROL_S:
		*reg32_id = ARMV8M_PMSK_BPRI_FLTMSK_CTRL_S;
		*offset = arm_reg_id - ARMV8M_PRIMASK_S;
		return true;
	case ARMV8M_PRIMASK_NS...ARMV8M_CONTROL_NS:
		*reg32_id = ARMV8M_PMSK_BPRI_FLTMSK_CTRL_NS;
		*offset = arm_reg_id - ARMV8M_PRIMASK_NS;
		return true;

	default:
		return false;
	}

}

static int armv7m_read_core_reg(struct target *target, struct reg *r,
	int num, enum arm_mode mode)
{
	uint32_t reg_value;
	int retval;
	struct armv7m_common *armv7m = target_to_armv7m(target);

	assert(num < (int)armv7m->arm.core_cache->num_regs);
	assert(num == (int)r->number);

	/* If a code calls read_reg, it expects the cache is no more dirty.
	 * Clear the dirty flag regardless of the later read succeeds or not
	 * to prevent unwanted cache flush after a read error */
	r->dirty = false;

	if (r->size <= 8) {
		/* any 8-bit or shorter register is packed */
		uint32_t offset;
		unsigned int reg32_id;

		bool is_packed = armv7m_map_reg_packing(num, &reg32_id, &offset);
		if (!is_packed) {
			/* We should not get here as all 8-bit or shorter registers
			 * are packed */
			assert(false);
			/* assert() does nothing if NDEBUG is defined */
			return ERROR_FAIL;
		}
		struct reg *r32 = &armv7m->arm.core_cache->reg_list[reg32_id];

		/* Read 32-bit container register if not cached */
		if (!r32->valid) {
			retval = armv7m_read_core_reg(target, r32, reg32_id, mode);
			if (retval != ERROR_OK)
				return retval;
		}

		/* Copy required bits of 32-bit container register */
		buf_cpy(r32->value + offset, r->value, r->size);

	} else {
		assert(r->size == 32 || r->size == 64);

		struct arm_reg *armv7m_core_reg = r->arch_info;
		uint32_t regsel = armv7m_map_id_to_regsel(armv7m_core_reg->num);

		retval = armv7m->load_core_reg_u32(target, regsel, &reg_value);
		if (retval != ERROR_OK)
			return retval;
		buf_set_u32(r->value, 0, 32, reg_value);

		if (r->size == 64) {
			retval = armv7m->load_core_reg_u32(target, regsel + 1, &reg_value);
			if (retval != ERROR_OK) {
				r->valid = false;
				return retval;
			}
			buf_set_u32(r->value + 4, 0, 32, reg_value);

			uint64_t q = buf_get_u64(r->value, 0, 64);
			LOG_DEBUG("read %s value 0x%016" PRIx64, r->name, q);
		} else {
			LOG_DEBUG("read %s value 0x%08" PRIx32, r->name, reg_value);
		}
	}

	r->valid = true;

	return ERROR_OK;
}

static int armv7m_write_core_reg(struct target *target, struct reg *r,
	int num, enum arm_mode mode, uint8_t *value)
{
	int retval;
	uint32_t t;
	struct armv7m_common *armv7m = target_to_armv7m(target);

	assert(num < (int)armv7m->arm.core_cache->num_regs);
	assert(num == (int)r->number);

	if (value != r->value) {
		/* If we are not flushing the cache, store the new value to the cache */
		buf_cpy(value, r->value, r->size);
	}

	if (r->size <= 8) {
		/* any 8-bit or shorter register is packed */
		uint32_t offset;
		unsigned int reg32_id;

		bool is_packed = armv7m_map_reg_packing(num, &reg32_id, &offset);
		if (!is_packed) {
			/* We should not get here as all 8-bit or shorter registers
			 * are packed */
			assert(false);
			/* assert() does nothing if NDEBUG is defined */
			return ERROR_FAIL;
		}
		struct reg *r32 = &armv7m->arm.core_cache->reg_list[reg32_id];

		if (!r32->valid) {
			/* Before merging with other parts ensure the 32-bit register is valid */
			retval = armv7m_read_core_reg(target, r32, reg32_id, mode);
			if (retval != ERROR_OK)
				return retval;
		}

		/* Write a part to the 32-bit container register */
		buf_cpy(value, r32->value + offset, r->size);
		r32->dirty = true;

	} else {
		assert(r->size == 32 || r->size == 64);

		struct arm_reg *armv7m_core_reg = r->arch_info;
		uint32_t regsel = armv7m_map_id_to_regsel(armv7m_core_reg->num);

		t = buf_get_u32(value, 0, 32);
		retval = armv7m->store_core_reg_u32(target, regsel, t);
		if (retval != ERROR_OK)
			goto out_error;

		if (r->size == 64) {
			t = buf_get_u32(value + 4, 0, 32);
			retval = armv7m->store_core_reg_u32(target, regsel + 1, t);
			if (retval != ERROR_OK)
				goto out_error;

			uint64_t q = buf_get_u64(value, 0, 64);
			LOG_DEBUG("write %s value 0x%016" PRIx64, r->name, q);
		} else {
			LOG_DEBUG("write %s value 0x%08" PRIx32, r->name, t);
		}
	}

	r->valid = true;
	r->dirty = false;

	return ERROR_OK;

out_error:
	r->dirty = true;
	LOG_ERROR("Error setting register %s", r->name);
	return retval;
}

/**
 * Returns generic ARM userspace registers to GDB.
 */
int armv7m_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int i, size;

	if (reg_class == REG_CLASS_ALL)
		size = armv7m->arm.core_cache->num_regs;
	else
		size = ARMV7M_NUM_CORE_REGS;

	*reg_list = malloc(sizeof(struct reg *) * size);
	if (*reg_list == NULL)
		return ERROR_FAIL;

	for (i = 0; i < size; i++)
		(*reg_list)[i] = &armv7m->arm.core_cache->reg_list[i];

	*reg_list_size = size;

	return ERROR_OK;
}

/** Runs a Thumb algorithm in the target. */
int armv7m_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	int timeout_ms, void *arch_info)
{
	int retval;

	retval = armv7m_start_algorithm(target,
			num_mem_params, mem_params,
			num_reg_params, reg_params,
			entry_point, exit_point,
			arch_info);

	if (retval == ERROR_OK)
		retval = armv7m_wait_algorithm(target,
				num_mem_params, mem_params,
				num_reg_params, reg_params,
				exit_point, timeout_ms,
				arch_info);

	return retval;
}

/** Starts a Thumb algorithm in the target. */
int armv7m_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_algorithm *armv7m_algorithm_info = arch_info;
	enum arm_mode core_mode = armv7m->arm.core_mode;
	int retval = ERROR_OK;

	/* NOTE: armv7m_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (armv7m_algorithm_info->common_magic != ARMV7M_COMMON_MAGIC) {
		LOG_ERROR("current target isn't an ARMV7M target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Store all non-debug execution registers to armv7m_algorithm_info context */
	for (unsigned i = 0; i < armv7m->arm.core_cache->num_regs; i++) {

		armv7m_algorithm_info->context[i] = buf_get_u32(
				armv7m->arm.core_cache->reg_list[i].value,
				0,
				32);
	}

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;
		retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN)
			continue;

		struct reg *reg =
			register_get_by_name(armv7m->arm.core_cache, reg_params[i].reg_name, 0);
/*		uint32_t regvalue; */

		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
				reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

/*		regvalue = buf_get_u32(reg_params[i].value, 0, 32); */
		armv7m_set_core_reg(reg, reg_params[i].value);
	}

	{
		/*
		 * Ensure xPSR.T is set to avoid trying to run things in arm
		 * (non-thumb) mode, which armv7m does not support.
		 *
		 * We do this by setting the entirety of xPSR, which should
		 * remove all the unknowns about xPSR state.
		 *
		 * Because xPSR.T is populated on reset from the vector table,
		 * it might be 0 if the vector table has "bad" data in it.
		 */
		struct reg *reg = &armv7m->arm.core_cache->reg_list[ARMV7M_xPSR];
		buf_set_u32(reg->value, 0, 32, 0x01000000);
		reg->valid = true;
		reg->dirty = true;
	}

	if (armv7m_algorithm_info->core_mode != ARM_MODE_ANY &&
			armv7m_algorithm_info->core_mode != core_mode) {

		/* we cannot set ARM_MODE_HANDLER, so use ARM_MODE_THREAD instead */
		if (armv7m_algorithm_info->core_mode == ARM_MODE_HANDLER) {
			armv7m_algorithm_info->core_mode = ARM_MODE_THREAD;
			LOG_INFO("ARM_MODE_HANDLER not currently supported, using ARM_MODE_THREAD instead");
		}

		LOG_DEBUG("setting core_mode: 0x%2.2x", armv7m_algorithm_info->core_mode);
		buf_set_u32(armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].value,
			0, 1, armv7m_algorithm_info->core_mode);
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].dirty = true;
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].valid = true;
	}

	/* save previous core mode */
	armv7m_algorithm_info->core_mode = core_mode;

	retval = target_resume(target, 0, entry_point, 1, 1);

	return retval;
}

/** Waits for an algorithm in the target. */
int armv7m_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_algorithm *armv7m_algorithm_info = arch_info;
	int retval = ERROR_OK;

	/* NOTE: armv7m_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (armv7m_algorithm_info->common_magic != ARMV7M_COMMON_MAGIC) {
		LOG_ERROR("current target isn't an ARMV7M target");
		return ERROR_TARGET_INVALID;
	}

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	/* If the target fails to halt due to the breakpoint, force a halt */
	if (retval != ERROR_OK || target->state != TARGET_HALTED) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;
		return ERROR_TARGET_TIMEOUT;
	}

	if (exit_point) {
		/* PC value has been cached in cortex_m_debug_entry() */
		uint32_t pc = buf_get_u32(armv7m->arm.pc->value, 0, 32);
		if (pc != exit_point) {
			LOG_DEBUG("failed algorithm halted at 0x%" PRIx32 ", expected 0x%" TARGET_PRIxADDR,
					  pc, exit_point);
			return ERROR_TARGET_ALGO_EXIT;
		}
	}

	/* Read memory values to mem_params[] */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* Copy core register values to reg_params[] */
	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {
			struct reg *reg = register_get_by_name(armv7m->arm.core_cache,
					reg_params[i].reg_name,
					0);

			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			if (reg->size != reg_params[i].size) {
				LOG_ERROR(
					"BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	for (int i = armv7m->arm.core_cache->num_regs - 1; i >= 0; i--) {
		uint32_t regvalue;
		regvalue = buf_get_u32(armv7m->arm.core_cache->reg_list[i].value, 0, 32);
		if (regvalue != armv7m_algorithm_info->context[i]) {
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
					armv7m->arm.core_cache->reg_list[i].name,
				armv7m_algorithm_info->context[i]);
			buf_set_u32(armv7m->arm.core_cache->reg_list[i].value,
				0, 32, armv7m_algorithm_info->context[i]);
			armv7m->arm.core_cache->reg_list[i].valid = true;
			armv7m->arm.core_cache->reg_list[i].dirty = true;
		}
	}

	/* restore previous core mode */
	if (armv7m_algorithm_info->core_mode != armv7m->arm.core_mode) {
		LOG_DEBUG("restoring core_mode: 0x%2.2x", armv7m_algorithm_info->core_mode);
		buf_set_u32(armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].value,
			0, 1, armv7m_algorithm_info->core_mode);
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].dirty = true;
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].valid = true;
	}

	armv7m->arm.core_mode = armv7m_algorithm_info->core_mode;

	return retval;
}

/** Logs summary of ARMv7-M state for a halted target. */
int armv7m_arch_state(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct arm *arm = &armv7m->arm;
	uint32_t ctrl, sp;

	/* avoid filling log waiting for fileio reply */
	if (target->semihosting && target->semihosting->hit_fileio)
		return ERROR_OK;

	ctrl = buf_get_u32(arm->core_cache->reg_list[ARMV7M_CONTROL].value, 0, 32);
	sp = buf_get_u32(arm->core_cache->reg_list[ARMV7M_R13].value, 0, 32);

	LOG_USER("target halted due to %s, current mode: %s %s\n"
		"xPSR: %#8.8" PRIx32 " pc: %#8.8" PRIx32 " %csp: %#8.8" PRIx32 "%s%s",
		debug_reason_name(target),
		arm_mode_name(arm->core_mode),
		armv7m_exception_string(armv7m->exception_number),
		buf_get_u32(arm->cpsr->value, 0, 32),
		buf_get_u32(arm->pc->value, 0, 32),
		(ctrl & 0x02) ? 'p' : 'm',
		sp,
		(target->semihosting && target->semihosting->is_active) ? ", semihosting" : "",
		(target->semihosting && target->semihosting->is_fileio) ? " fileio" : "");

	return ERROR_OK;
}

static const struct reg_arch_type armv7m_reg_type = {
	.get = armv7m_get_core_reg,
	.set = armv7m_set_core_reg,
};

/** Builds cache of architecturally defined registers.  */
struct reg_cache *armv7m_build_reg_cache(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct arm *arm = &armv7m->arm;
	int num_regs = ARMV7M_NUM_REGS;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct arm_reg *arch_info = calloc(num_regs, sizeof(struct arm_reg));
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "arm v7m registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = armv7m_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].arm = arm;

		reg_list[i].name = armv7m_regs[i].name;
		reg_list[i].size = armv7m_regs[i].bits;
		reg_list[i].value = arch_info[i].value;
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].hidden = (i == ARMV7M_PMSK_BPRI_FLTMSK_CTRL ||
				i == ARMV8M_PMSK_BPRI_FLTMSK_CTRL_NS || i == ARMV8M_PMSK_BPRI_FLTMSK_CTRL_S);
		reg_list[i].type = &armv7m_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].group = armv7m_regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		if (reg_list[i].hidden)
			continue;

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = armv7m_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = armv7m_regs[i].type;
		else
			LOG_ERROR("unable to allocate reg type list");
	}

	arm->cpsr = reg_list + ARMV7M_xPSR;
	arm->pc = reg_list + ARMV7M_PC;
	arm->core_cache = cache;

	return cache;
}

void armv7m_free_reg_cache(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct arm *arm = &armv7m->arm;
	struct reg_cache *cache;
	struct reg *reg;
	unsigned int i;

	cache = arm->core_cache;

	if (!cache)
		return;

	for (i = 0; i < cache->num_regs; i++) {
		reg = &cache->reg_list[i];

		free(reg->feature);
		free(reg->reg_data_type);
	}

	free(cache->reg_list[0].arch_info);
	free(cache->reg_list);
	free(cache);

	arm->core_cache = NULL;
}

static int armv7m_setup_semihosting(struct target *target, int enable)
{
	/* nothing todo for armv7m */
	return ERROR_OK;
}

/** Sets up target as a generic ARMv7-M core */
int armv7m_init_arch_info(struct target *target, struct armv7m_common *armv7m)
{
	struct arm *arm = &armv7m->arm;

	armv7m->common_magic = ARMV7M_COMMON_MAGIC;
	armv7m->fp_feature = FP_NONE;
	armv7m->trace_config.trace_bus_id = 1;
	/* Enable stimulus port #0 by default */
	armv7m->trace_config.itm_ter[0] = 1;

	arm->core_type = ARM_CORE_TYPE_M_PROFILE;
	arm->arch_info = armv7m;
	arm->setup_semihosting = armv7m_setup_semihosting;

	arm->read_core_reg = armv7m_read_core_reg;
	arm->write_core_reg = armv7m_write_core_reg;

	return arm_init_arch_info(target, arm);
}

/** Generates a CRC32 checksum of a memory region. */
int armv7m_checksum_memory(struct target *target,
	target_addr_t address, uint32_t count, uint32_t *checksum)
{
	struct working_area *crc_algorithm;
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[2];
	int retval;

	static const uint8_t cortex_m_crc_code[] = {
#include "../../contrib/loaders/checksum/armv7m_crc.inc"
	};

	retval = target_alloc_working_area(target, sizeof(cortex_m_crc_code), &crc_algorithm);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_buffer(target, crc_algorithm->address,
			sizeof(cortex_m_crc_code), (uint8_t *)cortex_m_crc_code);
	if (retval != ERROR_OK)
		goto cleanup;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, address);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	int timeout = 20000 * (1 + (count / (1024 * 1024)));

	retval = target_run_algorithm(target, 0, NULL, 2, reg_params, crc_algorithm->address,
			crc_algorithm->address + (sizeof(cortex_m_crc_code) - 6),
			timeout, &armv7m_info);

	if (retval == ERROR_OK)
		*checksum = buf_get_u32(reg_params[0].value, 0, 32);
	else
		LOG_ERROR("error executing cortex_m crc algorithm");

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

cleanup:
	target_free_working_area(target, crc_algorithm);

	return retval;
}

/** Checks an array of memory regions whether they are erased. */
int armv7m_blank_check_memory(struct target *target,
	struct target_memory_check_block *blocks, int num_blocks, uint8_t erased_value)
{
	struct working_area *erase_check_algorithm;
	struct working_area *erase_check_params;
	struct reg_param reg_params[2];
	struct armv7m_algorithm armv7m_info;
	int retval;

	static bool timed_out;

	static const uint8_t erase_check_code[] = {
#include "../../contrib/loaders/erase_check/armv7m_erase_check.inc"
	};

	const uint32_t code_size = sizeof(erase_check_code);

	/* make sure we have a working area */
	if (target_alloc_working_area(target, code_size,
		&erase_check_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, erase_check_algorithm->address,
			code_size, erase_check_code);
	if (retval != ERROR_OK)
		goto cleanup1;

	/* prepare blocks array for algo */
	struct algo_block {
		union {
			uint32_t size;
			uint32_t result;
		};
		uint32_t address;
	};

	uint32_t avail = target_get_working_area_avail(target);
	int blocks_to_check = avail / sizeof(struct algo_block) - 1;
	if (num_blocks < blocks_to_check)
		blocks_to_check = num_blocks;

	struct algo_block *params = malloc((blocks_to_check+1)*sizeof(struct algo_block));
	if (params == NULL) {
		retval = ERROR_FAIL;
		goto cleanup1;
	}

	int i;
	uint32_t total_size = 0;
	for (i = 0; i < blocks_to_check; i++) {
		total_size += blocks[i].size;
		target_buffer_set_u32(target, (uint8_t *)&(params[i].size),
						blocks[i].size / sizeof(uint32_t));
		target_buffer_set_u32(target, (uint8_t *)&(params[i].address),
						blocks[i].address);
	}
	target_buffer_set_u32(target, (uint8_t *)&(params[blocks_to_check].size), 0);

	uint32_t param_size = (blocks_to_check + 1) * sizeof(struct algo_block);
	if (target_alloc_working_area(target, param_size,
			&erase_check_params) != ERROR_OK) {
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto cleanup2;
	}

	retval = target_write_buffer(target, erase_check_params->address,
				param_size, (uint8_t *)params);
	if (retval != ERROR_OK)
		goto cleanup3;

	uint32_t erased_word = erased_value | (erased_value << 8)
			       | (erased_value << 16) | (erased_value << 24);

	LOG_DEBUG("Starting erase check of %d blocks, parameters@"
		 TARGET_ADDR_FMT, blocks_to_check, erase_check_params->address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, erase_check_params->address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, erased_word);

	/* assume CPU clk at least 1 MHz */
	int timeout = (timed_out ? 30000 : 2000) + total_size * 3 / 1000;

	retval = target_run_algorithm(target,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				erase_check_algorithm->address,
				erase_check_algorithm->address + (code_size - 2),
				timeout,
				&armv7m_info);

	timed_out = retval == ERROR_TARGET_TIMEOUT;
	if (retval != ERROR_OK && !timed_out)
		goto cleanup4;

	retval = target_read_buffer(target, erase_check_params->address,
				param_size, (uint8_t *)params);
	if (retval != ERROR_OK)
		goto cleanup4;

	for (i = 0; i < blocks_to_check; i++) {
		uint32_t result = target_buffer_get_u32(target,
					(uint8_t *)&(params[i].result));
		if (result != 0 && result != 1)
			break;

		blocks[i].result = result;
	}
	if (i && timed_out)
		LOG_INFO("Slow CPU clock: %d blocks checked, %d remain. Continuing...", i, num_blocks-i);

	retval = i;		/* return number of blocks really checked */

cleanup4:
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

cleanup3:
	target_free_working_area(target, erase_check_params);
cleanup2:
	free(params);
cleanup1:
	target_free_working_area(target, erase_check_algorithm);

	return retval;
}

int armv7m_maybe_skip_bkpt_inst(struct target *target, bool *inst_found)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct reg *r = armv7m->arm.pc;
	bool result = false;


	/* if we halted last time due to a bkpt instruction
	 * then we have to manually step over it, otherwise
	 * the core will break again */

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		uint16_t op;
		uint32_t pc = buf_get_u32(r->value, 0, 32);

		pc &= ~1;
		if (target_read_u16(target, pc, &op) == ERROR_OK) {
			if ((op & 0xFF00) == 0xBE00) {
				pc = buf_get_u32(r->value, 0, 32) + 2;
				buf_set_u32(r->value, 0, 32, pc);
				r->dirty = true;
				r->valid = true;
				result = true;
				LOG_DEBUG("Skipping over BKPT instruction");
			}
		}
	}

	if (inst_found)
		*inst_found = result;

	return ERROR_OK;
}

const struct command_registration armv7m_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
