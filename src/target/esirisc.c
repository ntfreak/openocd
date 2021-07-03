/***************************************************************************
 *   Copyright (C) 2018 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 *   James Zhao <hjz@squareup.com>                                         *
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

#include <helper/binarybuffer.h>
#include <helper/command.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <helper/types.h>
#include <jtag/interface.h>
#include <target/breakpoints.h>
#include <target/register.h>
#include <target/target.h>
#include <target/target_type.h>

#include "esirisc.h"

#define RESET_TIMEOUT			5000	/* 5s */
#define STEP_TIMEOUT			1000	/* 1s */

/*
 * eSi-RISC targets support a configurable number of interrupts;
 * up to 32 interrupts are supported.
 */
static const char * const esirisc_exception_strings[] = {
	[EID_RESET]					= "Reset",
	[EID_HARDWARE_FAILURE]		= "HardwareFailure",
	[EID_NMI]					= "NMI",
	[EID_INST_BREAKPOINT]		= "InstBreakpoint",
	[EID_DATA_BREAKPOINT]		= "DataBreakpoint",
	[EID_UNSUPPORTED]			= "Unsupported",
	[EID_PRIVILEGE_VIOLATION]	= "PrivilegeViolation",
	[EID_INST_BUS_ERROR]		= "InstBusError",
	[EID_DATA_BUS_ERROR]		= "DataBusError",
	[EID_ALIGNMENT_ERROR]		= "AlignmentError",
	[EID_ARITHMETIC_ERROR]		= "ArithmeticError",
	[EID_SYSTEM_CALL]			= "SystemCall",
	[EID_MEMORY_MANAGEMENT]		= "MemoryManagement",
	[EID_UNRECOVERABLE]			= "Unrecoverable",
	[EID_INTERRUPT_N+0]			= "Interrupt0",
	[EID_INTERRUPT_N+1]			= "Interrupt1",
	[EID_INTERRUPT_N+2]			= "Interrupt2",
	[EID_INTERRUPT_N+3]			= "Interrupt3",
	[EID_INTERRUPT_N+4]			= "Interrupt4",
	[EID_INTERRUPT_N+5]			= "Interrupt5",
	[EID_INTERRUPT_N+6]			= "Interrupt6",
	[EID_INTERRUPT_N+7]			= "Interrupt7",
	[EID_INTERRUPT_N+8]			= "Interrupt8",
	[EID_INTERRUPT_N+9]			= "Interrupt9",
	[EID_INTERRUPT_N+10]			= "Interrupt10",
	[EID_INTERRUPT_N+11]			= "Interrupt11",
	[EID_INTERRUPT_N+12]			= "Interrupt12",
	[EID_INTERRUPT_N+13]			= "Interrupt13",
	[EID_INTERRUPT_N+14]			= "Interrupt14",
	[EID_INTERRUPT_N+15]			= "Interrupt15",
	[EID_INTERRUPT_N+16]			= "Interrupt16",
	[EID_INTERRUPT_N+17]			= "Interrupt17",
	[EID_INTERRUPT_N+18]			= "Interrupt18",
	[EID_INTERRUPT_N+19]			= "Interrupt19",
	[EID_INTERRUPT_N+20]			= "Interrupt20",
	[EID_INTERRUPT_N+21]			= "Interrupt21",
	[EID_INTERRUPT_N+22]			= "Interrupt22",
	[EID_INTERRUPT_N+23]			= "Interrupt23",
	[EID_INTERRUPT_N+24]			= "Interrupt24",
	[EID_INTERRUPT_N+25]			= "Interrupt25",
	[EID_INTERRUPT_N+26]			= "Interrupt26",
	[EID_INTERRUPT_N+27]			= "Interrupt27",
	[EID_INTERRUPT_N+28]			= "Interrupt28",
	[EID_INTERRUPT_N+29]			= "Interrupt29",
	[EID_INTERRUPT_N+30]			= "Interrupt30",
	[EID_INTERRUPT_N+31]			= "Interrupt31",
};

/*
 * eSi-RISC targets support a configurable number of general purpose
 * registers; 8, 16, and 32 registers are supported.
 */
static const struct {
	enum esirisc_reg_num number;
	const char *name;
	enum reg_type type;
	const char *group;
} esirisc_regs[] = {
	{ ESIRISC_SP, "sp", REG_TYPE_DATA_PTR, "general" },
	{ ESIRISC_RA, "ra", REG_TYPE_INT, "general" },
	{ ESIRISC_R2, "r2", REG_TYPE_INT, "general" },
	{ ESIRISC_R3, "r3", REG_TYPE_INT, "general" },
	{ ESIRISC_R4, "r4", REG_TYPE_INT, "general" },
	{ ESIRISC_R5, "r5", REG_TYPE_INT, "general" },
	{ ESIRISC_R6, "r6", REG_TYPE_INT, "general" },
	{ ESIRISC_R7, "r7", REG_TYPE_INT, "general" },
	{ ESIRISC_R8, "r8", REG_TYPE_INT, "general" },
	{ ESIRISC_R9, "r9", REG_TYPE_INT, "general" },
	{ ESIRISC_R10, "r10", REG_TYPE_INT, "general" },
	{ ESIRISC_R11, "r11", REG_TYPE_INT, "general" },
	{ ESIRISC_R12, "r12", REG_TYPE_INT, "general" },
	{ ESIRISC_R13, "r13", REG_TYPE_INT, "general" },
	{ ESIRISC_R14, "r14", REG_TYPE_INT, "general" },
	{ ESIRISC_R15, "r15", REG_TYPE_INT, "general" },
	{ ESIRISC_R16, "r16", REG_TYPE_INT, "general" },
	{ ESIRISC_R17, "r17", REG_TYPE_INT, "general" },
	{ ESIRISC_R18, "r18", REG_TYPE_INT, "general" },
	{ ESIRISC_R19, "r19", REG_TYPE_INT, "general" },
	{ ESIRISC_R20, "r20", REG_TYPE_INT, "general" },
	{ ESIRISC_R21, "r21", REG_TYPE_INT, "general" },
	{ ESIRISC_R22, "r22", REG_TYPE_INT, "general" },
	{ ESIRISC_R23, "r23", REG_TYPE_INT, "general" },
	{ ESIRISC_R24, "r24", REG_TYPE_INT, "general" },
	{ ESIRISC_R25, "r25", REG_TYPE_INT, "general" },
	{ ESIRISC_R26, "r26", REG_TYPE_INT, "general" },
	{ ESIRISC_R27, "r27", REG_TYPE_INT, "general" },
	{ ESIRISC_R28, "r28", REG_TYPE_INT, "general" },
	{ ESIRISC_R29, "r29", REG_TYPE_INT, "general" },
	{ ESIRISC_R30, "r30", REG_TYPE_INT, "general" },
	{ ESIRISC_R31, "r31", REG_TYPE_INT, "general" },
};

/*
 * Control and Status Registers (CSRs) are largely defined as belonging
 * to the system register group. The exception to this rule are the PC
 * and CAS registers, which belong to the general group. While debug is
 * active, EPC, ECAS, and ETC must be used to read and write the PC,
 * CAS, and TC CSRs, respectively.
 */
static const struct {
	enum esirisc_reg_num number;
	uint8_t bank;
	uint8_t csr;
	const char *name;
	enum reg_type type;
	const char *group;
} esirisc_csrs[] = {
	{ ESIRISC_PC, CSR_THREAD, CSR_THREAD_EPC, "PC", REG_TYPE_CODE_PTR, "general" },	/*  PC -> EPC  */
	{ ESIRISC_CAS, CSR_THREAD, CSR_THREAD_ECAS, "CAS", REG_TYPE_INT, "general" },	/* CAS -> ECAS */
	{ ESIRISC_TC, CSR_THREAD, CSR_THREAD_ETC, "TC", REG_TYPE_INT, "system" },		/*  TC -> ETC  */
	{ ESIRISC_ETA, CSR_THREAD, CSR_THREAD_ETA, "ETA", REG_TYPE_INT, "system" },
	{ ESIRISC_ETC, CSR_THREAD, CSR_THREAD_ETC, "ETC", REG_TYPE_INT, "system" },
	{ ESIRISC_EPC, CSR_THREAD, CSR_THREAD_EPC, "EPC", REG_TYPE_CODE_PTR, "system" },
	{ ESIRISC_ECAS, CSR_THREAD, CSR_THREAD_ECAS, "ECAS", REG_TYPE_INT, "system" },
	{ ESIRISC_EID, CSR_THREAD, CSR_THREAD_EID, "EID", REG_TYPE_INT, "system" },
	{ ESIRISC_ED, CSR_THREAD, CSR_THREAD_ED, "ED", REG_TYPE_INT, "system" },
	{ ESIRISC_IP, CSR_INTERRUPT, CSR_INTERRUPT_IP, "IP", REG_TYPE_INT, "system"},
	{ ESIRISC_IM, CSR_INTERRUPT, CSR_INTERRUPT_IM, "IM", REG_TYPE_INT, "system"},
	{ ESIRISC_IS, CSR_INTERRUPT, CSR_INTERRUPT_IS, "IS", REG_TYPE_INT, "system"},
	{ ESIRISC_IT, CSR_INTERRUPT, CSR_INTERRUPT_IT, "IT", REG_TYPE_INT, "system"},
};

static int esirisc_disable_interrupts(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t etc;
	int retval;

	LOG_DEBUG("-");

	retval = esirisc_jtag_read_csr(jtag_info, CSR_THREAD, CSR_THREAD_ETC, &etc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Thread CSR: ETC", target_name(target));
		return retval;
	}

	etc &= ~(1<<0);		/* TC.I */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_THREAD, CSR_THREAD_ETC, etc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Thread CSR: ETC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

#if 0
static int esirisc_enable_interrupts(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t etc;
	int retval;

	LOG_DEBUG("-");

	retval = esirisc_jtag_read_csr(jtag_info, CSR_THREAD, CSR_THREAD_ETC, &etc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Thread CSR: ETC", target_name(target));
		return retval;
	}

	etc |= (1<<0);		/* TC.I */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_THREAD, CSR_THREAD_ETC, etc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Thread CSR: ETC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}
#endif

static int esirisc_save_interrupts(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	int retval = esirisc_jtag_read_csr(jtag_info, CSR_THREAD, CSR_THREAD_ETC,
			&esirisc->etc_save);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Thread CSR: ETC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_restore_interrupts(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	int retval = esirisc_jtag_write_csr(jtag_info, CSR_THREAD, CSR_THREAD_ETC,
			esirisc->etc_save);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Thread CSR: ETC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

#if 0
static int esirisc_save_hwdc(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	int retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_HWDC,
			&esirisc->hwdc_save);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Thread CSR: HWDC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}
#endif

static int esirisc_restore_hwdc(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	int retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_HWDC,
			esirisc->hwdc_save);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: HWDC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_save_context(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);

	LOG_DEBUG("-");

	for (unsigned i = 0; i < esirisc->reg_cache->num_regs; ++i) {
		struct reg *reg = esirisc->reg_cache->reg_list + i;
		struct esirisc_reg *reg_info = reg->arch_info;

		if (reg->exist && !reg->valid)
			reg_info->read(reg);
	}

	return ERROR_OK;
}

static int esirisc_restore_context(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);

	LOG_DEBUG("-");

	for (unsigned i = 0; i < esirisc->reg_cache->num_regs; ++i) {
		struct reg *reg = esirisc->reg_cache->reg_list + i;
		struct esirisc_reg *reg_info = reg->arch_info;

		if (reg->exist && reg->dirty)
			reg_info->write(reg);
	}

	return ERROR_OK;
}

static int esirisc_flush_caches(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	int retval = esirisc_jtag_flush_caches(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to flush caches", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_wait_debug_active(struct esirisc_common *esirisc, int ms)
{
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int64_t t;

	LOG_DEBUG("-");

	t = timeval_ms();
	for (;;) {
		int retval = esirisc_jtag_enable_debug(jtag_info);
		if (retval == ERROR_OK && esirisc_jtag_is_debug_active(jtag_info))
			return retval;

		if ((timeval_ms() - t) > ms)
			return ERROR_TARGET_TIMEOUT;

		alive_sleep(100);
	}
}

static int esirisc_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	int num_bits = 8 * size;
	for (uint32_t i = 0; i < count; ++i) {
		union esirisc_memory value;
		void *value_p;

		switch (size) {
			case sizeof(value.word):
				value_p = &value.word;
				retval = esirisc_jtag_read_word(jtag_info, address, value_p);
				break;

			case sizeof(value.hword):
				value_p = &value.hword;
				retval = esirisc_jtag_read_hword(jtag_info, address, value_p);
				break;

			case sizeof(value.byte):
				value_p = &value.byte;
				retval = esirisc_jtag_read_byte(jtag_info, address, value_p);
				break;

			default:
				LOG_ERROR("%s: unsupported size: %" PRIu32, target_name(target), size);
				return ERROR_FAIL;
		}

		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to read address: 0x%" TARGET_PRIxADDR, target_name(target),
					address);
			return retval;
		}

		buf_cpy(value_p, buffer, num_bits);
		address += size;
		buffer += size;
	}

	return ERROR_OK;
}

static int esirisc_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	int num_bits = 8 * size;
	for (uint32_t i = 0; i < count; ++i) {
		union esirisc_memory value;

		switch (size) {
			case sizeof(value.word):
				value.word = buf_get_u32(buffer, 0, num_bits);
				retval = esirisc_jtag_write_word(jtag_info, address, value.word);
				break;

			case sizeof(value.hword):
				value.hword = buf_get_u32(buffer, 0, num_bits);
				retval = esirisc_jtag_write_hword(jtag_info, address, value.hword);
				break;

			case sizeof(value.byte):
				value.byte = buf_get_u32(buffer, 0, num_bits);
				retval = esirisc_jtag_write_byte(jtag_info, address, value.byte);
				break;

			default:
				LOG_ERROR("%s: unsupported size: %" PRIu32, target_name(target), size);
				return ERROR_FAIL;
		}

		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to write address: 0x%" TARGET_PRIxADDR, target_name(target),
					address);
			return retval;
		}

		address += size;
		buffer += size;
	}

	return ERROR_OK;
}

static int esirisc_checksum_memory(struct target *target, target_addr_t address,
		uint32_t count, uint32_t *checksum)
{
	return ERROR_FAIL;	/* not supported */
}

static int esirisc_next_breakpoint(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct breakpoint **breakpoints_p = esirisc->breakpoints_p;
	struct breakpoint **breakpoints_e = breakpoints_p + esirisc->num_breakpoints;

	LOG_DEBUG("-");

	for (int bp_index = 0; breakpoints_p < breakpoints_e; ++breakpoints_p, ++bp_index)
		if (!*breakpoints_p)
			return bp_index;

	return -1;
}

static int esirisc_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int bp_index;
	uint32_t ibc;
	int retval;

	LOG_DEBUG("-");

	/*
	 * The default linker scripts provided by the eSi-RISC toolchain do
	 * not specify attributes on memory regions, which results in
	 * incorrect application of software breakpoints by GDB. Targets
	 * must be configured with `gdb_breakpoint_override hard` as
	 * software breakpoints are not supported.
	 */
	if (breakpoint->type != BKPT_HARD)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	bp_index = esirisc_next_breakpoint(target);
	if (bp_index < 0) {
		LOG_ERROR("%s: out of hardware breakpoints", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = bp_index + 1;
	esirisc->breakpoints_p[bp_index] = breakpoint;

	/* specify instruction breakpoint address */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_IBA_N + bp_index,
			breakpoint->address);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: IBA", target_name(target));
		return retval;
	}

	/* enable instruction breakpoint */
	retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_IBC, &ibc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Debug CSR: IBC", target_name(target));
		return retval;
	}

	ibc |= (1 << bp_index);		/* IBC.In */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_IBC, ibc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: IBC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_add_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	LOG_DEBUG("-");

	while (breakpoint) {
		if (breakpoint->set == 0)
			esirisc_add_breakpoint(target, breakpoint);

		breakpoint = breakpoint->next;
	}

	return ERROR_OK;
}

static int esirisc_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int bp_index = breakpoint->set - 1;
	uint32_t ibc;
	int retval;

	LOG_DEBUG("-");

	/* disable instruction breakpoint */
	retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_IBC, &ibc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Debug CSR: IBC", target_name(target));
		return retval;
	}

	ibc &= ~(1 << bp_index);	/* IBC.In */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_IBC, ibc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: IBC", target_name(target));
		return retval;
	}

	esirisc->breakpoints_p[bp_index] = NULL;
	breakpoint->set = 0;

	return ERROR_OK;
}

static int esirisc_remove_breakpoints(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	/* clear instruction breakpoints */
	int retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_IBC, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: IBC", target_name(target));
		return retval;
	}

	memset(esirisc->breakpoints_p, 0, sizeof(esirisc->breakpoints_p));

	return ERROR_OK;
}

static int esirisc_next_watchpoint(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct watchpoint **watchpoints_p = esirisc->watchpoints_p;
	struct watchpoint **watchpoints_e = watchpoints_p + esirisc->num_watchpoints;

	LOG_DEBUG("-");

	for (int wp_index = 0; watchpoints_p < watchpoints_e; ++watchpoints_p, ++wp_index)
		if (!*watchpoints_p)
			return wp_index;

	return -1;
}

static int esirisc_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int wp_index;
	uint32_t dbs, dbc;
	int retval;

	LOG_DEBUG("-");

	wp_index = esirisc_next_watchpoint(target);
	if (wp_index < 0) {
		LOG_ERROR("%s: out of hardware watchpoints", target_name(target));
		return ERROR_FAIL;
	}

	watchpoint->set = wp_index + 1;
	esirisc->watchpoints_p[wp_index] = watchpoint;

	/* specify data breakpoint address */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBA_N + wp_index,
			watchpoint->address);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: DBA", target_name(target));
		return retval;
	}

	/* specify data breakpoint size */
	retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBS, &dbs);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Debug CSR: DBS", target_name(target));
		return retval;
	}

	uint32_t sn;
	switch (watchpoint->length) {
		case sizeof(uint64_t):
			sn = 0x3;
			break;
		case sizeof(uint32_t):
			sn = 0x2;
			break;

		case sizeof(uint16_t):
			sn = 0x1;
			break;

		case sizeof(uint8_t):
			sn = 0x0;
			break;

		default:
			LOG_ERROR("%s: unsupported length: %" PRIu32, target_name(target),
					watchpoint->length);
			return ERROR_FAIL;
	}

	dbs |= (sn << (2 * wp_index));		/* DBS.Sn */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBS, dbs);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: DBS", target_name(target));
		return retval;
	}

	/* enable data breakpoint */
	retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBC, &dbc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Debug CSR: DBC", target_name(target));
		return retval;
	}

	uint32_t dn;
	switch (watchpoint->rw) {
		case WPT_READ:
			dn = 0x1;
			break;

		case WPT_WRITE:
			dn = 0x2;
			break;

		case WPT_ACCESS:
			dn = 0x3;
			break;

		default:
			LOG_ERROR("%s: unsupported rw: %" PRId32, target_name(target),
					watchpoint->rw);
			return ERROR_FAIL;
	}

	dbc |= (dn << (2 * wp_index));		/* DBC.Dn */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBC, dbc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: DBC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_add_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	LOG_DEBUG("-");

	while (watchpoint) {
		if (watchpoint->set == 0)
			esirisc_add_watchpoint(target, watchpoint);

		watchpoint = watchpoint->next;
	}

	return ERROR_OK;
}

static int esirisc_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int wp_index = watchpoint->set - 1;
	uint32_t dbc;
	int retval;

	LOG_DEBUG("-");

	/* disable data breakpoint */
	retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBC, &dbc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Debug CSR: DBC", target_name(target));
		return retval;
	}

	dbc &= ~(0x3 << (2 * wp_index));	/* DBC.Dn */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBC, dbc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: DBC", target_name(target));
		return retval;
	}

	esirisc->watchpoints_p[wp_index] = NULL;
	watchpoint->set = 0;

	return ERROR_OK;
}

static int esirisc_remove_watchpoints(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	/* clear data breakpoints */
	int retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DBC, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: DBC", target_name(target));
		return retval;
	}

	memset(esirisc->watchpoints_p, 0, sizeof(esirisc->watchpoints_p));

	return ERROR_OK;
}

static int esirisc_halt(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	LOG_DEBUG("-");

	if (target->state == TARGET_HALTED)
		return ERROR_OK;

	int retval = esirisc_jtag_break(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to halt target", target_name(target));
		return retval;
	}

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int esirisc_disable_step(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t dc;
	int retval;

	LOG_DEBUG("-");

	retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DC, &dc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Debug CSR: DC", target_name(target));
		return retval;
	}

	dc &= ~(1<<0);	/* DC.S */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DC, dc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: DC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_enable_step(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t dc;
	int retval;

	LOG_DEBUG("-");

	retval = esirisc_jtag_read_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DC, &dc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Debug CSR: DC", target_name(target));
		return retval;
	}

	dc |= (1<<0);	/* DC.S */

	retval = esirisc_jtag_write_csr(jtag_info, CSR_DEBUG, CSR_DEBUG_DC, dc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Debug CSR: DC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_resume_or_step(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution, bool step)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	struct breakpoint *breakpoint = NULL;
	int retval;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (!debug_execution) {
		target_free_all_working_areas(target);
		esirisc_add_breakpoints(target);
		esirisc_add_watchpoints(target);
	}

	if (current)
		address = buf_get_u32(esirisc->epc->value, 0, esirisc->epc->size);
	else {
		buf_set_u32(esirisc->epc->value, 0, esirisc->epc->size, address);
		esirisc->epc->dirty = true;
		esirisc->epc->valid = true;
	}

	esirisc_restore_context(target);

	if (esirisc_has_cache(esirisc))
		esirisc_flush_caches(target);

	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, address);
		if (breakpoint)
			esirisc_remove_breakpoint(target, breakpoint);
	}

	if (step) {
		esirisc_disable_interrupts(target);
		esirisc_enable_step(target);
		target->debug_reason = DBG_REASON_SINGLESTEP;
	} else {
		esirisc_disable_step(target);
		esirisc_restore_interrupts(target);
		target->debug_reason = DBG_REASON_NOTHALTED;
	}

	esirisc_restore_hwdc(target);

	retval = esirisc_jtag_continue(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to resume target", target_name(target));
		return retval;
	}

	register_cache_invalidate(esirisc->reg_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
	}

	return ERROR_OK;
}

static int esirisc_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("-");

	return esirisc_resume_or_step(target, current, address,
			handle_breakpoints, debug_execution, false);
}

static int esirisc_step(struct target *target, int current, target_addr_t address,
		int handle_breakpoints)
{
	LOG_DEBUG("-");

	return esirisc_resume_or_step(target, current, address,
			handle_breakpoints, 0, true);
}

static int esirisc_debug_step(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	esirisc_disable_interrupts(target);
	esirisc_enable_step(target);

	retval = esirisc_jtag_continue(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to resume target", target_name(target));
		return retval;
	}

	retval = esirisc_wait_debug_active(esirisc, STEP_TIMEOUT);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: step timed out", target_name(target));
		return retval;
	}

	esirisc_disable_step(target);
	esirisc_restore_interrupts(target);

	return ERROR_OK;
}

static int esirisc_debug_reset(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	retval = esirisc_jtag_assert_reset(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to assert reset", target_name(target));
		return retval;
	}

	retval = esirisc_jtag_deassert_reset(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to deassert reset", target_name(target));
		return retval;
	}

	retval = esirisc_wait_debug_active(esirisc, RESET_TIMEOUT);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: reset timed out", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_debug_enable(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	retval = esirisc_jtag_enable_debug(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to enable debug mode", target_name(target));
		return retval;
	}

	/*
	 * The debug clock is inactive until the first command is sent.
	 * If the target is stopped, we must first issue a reset before
	 * attempting further communication. This also handles unpowered
	 * targets, which will respond with all ones and appear active.
	 */
	if (esirisc_jtag_is_stopped(jtag_info)) {
		LOG_INFO("%s: debug clock inactive; attempting debug reset", target_name(target));
		retval = esirisc_debug_reset(target);
		if (retval != ERROR_OK)
			return retval;

		if (esirisc_jtag_is_stopped(jtag_info)) {
			LOG_ERROR("%s: target unresponsive; giving up", target_name(target));
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int esirisc_debug_entry(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct breakpoint *breakpoint;

	LOG_DEBUG("-");

	esirisc_save_context(target);

	if (esirisc_has_cache(esirisc))
		esirisc_flush_caches(target);

	if (target->debug_reason != DBG_REASON_SINGLESTEP) {
		esirisc_save_interrupts(target);

		uint32_t eid = buf_get_u32(esirisc->eid->value, 0, esirisc->eid->size);
		switch (eid) {
			/*
			 * InstBreakpoint exceptions are also raised when a core is
			 * halted for debugging. The following is required to
			 * determine if a breakpoint was encountered.
			 */
			case EID_INST_BREAKPOINT:
				breakpoint = breakpoint_find(target,
						buf_get_u32(esirisc->epc->value, 0, esirisc->epc->size));
				target->debug_reason = (breakpoint) ?
						DBG_REASON_BREAKPOINT : DBG_REASON_DBGRQ;
				break;

			/*
			 * eSi-RISC treats watchpoints similarly to breakpoints,
			 * however GDB will not request to step over the current
			 * instruction when a watchpoint fires. The following is
			 * required to resume the target.
			 */
			case EID_DATA_BREAKPOINT:
				esirisc_remove_watchpoints(target);
				esirisc_debug_step(target);
				esirisc_add_watchpoints(target);
				target->debug_reason = DBG_REASON_WATCHPOINT;
				break;

			default:
				target->debug_reason = DBG_REASON_DBGRQ;
		}
	}

	return ERROR_OK;
}

static int esirisc_poll(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	retval = esirisc_jtag_enable_debug(jtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to poll target", target_name(target));
		return retval;
	}

	if (esirisc_jtag_is_stopped(jtag_info)) {
		LOG_ERROR("%s: target has stopped; reset required", target_name(target));
		target->state = TARGET_UNKNOWN;
		return ERROR_TARGET_FAILURE;
	}

	if (esirisc_jtag_is_debug_active(jtag_info)) {
		if (target->state == TARGET_RUNNING || target->state == TARGET_RESET) {
			target->state = TARGET_HALTED;

			retval = esirisc_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}

	} else if (target->state == TARGET_HALTED || target->state == TARGET_RESET) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	}

	return ERROR_OK;
}

static int esirisc_assert_reset(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	if (jtag_get_reset_config() & RESET_HAS_SRST) {
		jtag_add_reset(1, 1);
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) == 0)
			jtag_add_reset(0, 1);
	} else {
		esirisc_remove_breakpoints(target);
		esirisc_remove_watchpoints(target);

		retval = esirisc_jtag_assert_reset(jtag_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to assert reset", target_name(target));
			return retval;
		}
	}

	target->state = TARGET_RESET;

	register_cache_invalidate(esirisc->reg_cache);

	return ERROR_OK;
}

static int esirisc_reset_entry(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t eta, epc;
	int retval;

	LOG_DEBUG("-");

	/* read exception table address */
	retval = esirisc_jtag_read_csr(jtag_info, CSR_THREAD, CSR_THREAD_ETA, &eta);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Thread CSR: ETA", target_name(target));
		return retval;
	}

	/* read reset entry point */
	retval = esirisc_jtag_read_word(jtag_info, eta + ENTRY_RESET, &epc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read address: 0x%" TARGET_PRIxADDR, target_name(target),
				(target_addr_t)epc);
		return retval;
	}

	/* write reset entry point */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_THREAD, CSR_THREAD_EPC, epc);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Thread CSR: EPC", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_deassert_reset(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	if (jtag_get_reset_config() & RESET_HAS_SRST) {
		jtag_add_reset(0, 0);

		retval = esirisc_debug_enable(target);
		if (retval != ERROR_OK)
			return retval;

		retval = esirisc_debug_reset(target);
		if (retval != ERROR_OK)
			return retval;

	} else {
		retval = esirisc_jtag_deassert_reset(jtag_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to deassert reset", target_name(target));
			return retval;
		}
	}

	retval = esirisc_wait_debug_active(esirisc, RESET_TIMEOUT);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: reset timed out", target_name(target));
		return retval;
	}

	retval = esirisc_reset_entry(target);
	if (retval != ERROR_OK)
		return retval;

	esirisc_add_breakpoints(target);
	esirisc_add_watchpoints(target);

	esirisc_restore_hwdc(target);

	if (!target->reset_halt) {
		retval = esirisc_jtag_continue(jtag_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to resume target", target_name(target));
			return retval;
		}
	}

	return ERROR_OK;
}

static int esirisc_arch_state(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	uint32_t epc = buf_get_u32(esirisc->epc->value, 0, esirisc->epc->size);
	uint32_t ecas = buf_get_u32(esirisc->ecas->value, 0, esirisc->ecas->size);
	uint32_t eid = buf_get_u32(esirisc->eid->value, 0, esirisc->eid->size);
	uint32_t ed = buf_get_u32(esirisc->ed->value, 0, esirisc->ed->size);

	LOG_USER("target halted due to %s, exception: %s\n"
			"EPC: 0x%" PRIx32 ", ECAS: 0x%" PRIx32 ", EID: 0x%" PRIx32 ", ED: 0x%" PRIx32,
			debug_reason_name(target), esirisc_exception_strings[eid], epc, ecas, eid, ed);

	return ERROR_OK;
}

static const char *esirisc_get_gdb_arch(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);

	LOG_DEBUG("-");

	/*
	 * Targets with the UNIFIED_ADDRESS_SPACE option disabled employ a
	 * Harvard architecture. This option is not exposed in a CSR, which
	 * requires additional configuration to properly interact with these
	 * targets in GDB (also see: `esirisc cache_arch`).
	 */
	if (!esirisc->gdb_arch && target_was_examined(target))
		esirisc->gdb_arch = alloc_printf("esirisc:%d_bit_%d_reg_%s",
				esirisc->num_bits, esirisc->num_regs, esirisc_cache_arch_name(esirisc));

	return esirisc->gdb_arch;
}

static int esirisc_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);

	LOG_DEBUG("-");

	*reg_list_size = ESIRISC_NUM_REGS;

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	if (reg_class == REG_CLASS_ALL)
		for (int i = 0; i < *reg_list_size; ++i)
			(*reg_list)[i] = esirisc->reg_cache->reg_list + i;
	else {
		for (int i = 0; i < esirisc->num_regs; ++i)
			(*reg_list)[i] = esirisc->reg_cache->reg_list + i;

		(*reg_list)[ESIRISC_PC] = esirisc->reg_cache->reg_list + ESIRISC_PC;
		(*reg_list)[ESIRISC_CAS] = esirisc->reg_cache->reg_list + ESIRISC_CAS;
	}

	return ERROR_OK;
}

static int esirisc_read_reg(struct reg *reg)
{
	struct esirisc_reg *reg_info = reg->arch_info;
	struct esirisc_common *esirisc = reg_info->esirisc;
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	struct target *target = esirisc->target;
	uint32_t data;

	LOG_DEBUG("-");

	int retval = esirisc_jtag_read_reg(jtag_info, reg->number, &data);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read register: %s", target_name(target), reg->name);
		return retval;
	}

	buf_set_u32(reg->value, 0, reg->size, data);
	reg->dirty = false;
	reg->valid = true;

	return ERROR_OK;
}

static int esirisc_write_reg(struct reg *reg)
{
	struct esirisc_reg *reg_info = reg->arch_info;
	struct esirisc_common *esirisc = reg_info->esirisc;
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	struct target *target = esirisc->target;
	uint32_t data = buf_get_u32(reg->value, 0, reg->size);

	LOG_DEBUG("-");

	int retval = esirisc_jtag_write_reg(jtag_info, reg->number, data);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write register: %s", target_name(target), reg->name);
		return retval;
	}

	reg->dirty = false;
	reg->valid = true;

	return ERROR_OK;
}

static int esirisc_read_csr(struct reg *reg)
{
	struct esirisc_reg *reg_info = reg->arch_info;
	struct esirisc_common *esirisc = reg_info->esirisc;
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	struct target *target = esirisc->target;
	uint32_t data;

	LOG_DEBUG("-");

	int retval = esirisc_jtag_read_csr(jtag_info, reg_info->bank, reg_info->csr, &data);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read CSR: %s", target_name(target), reg->name);
		return retval;
	}

	buf_set_u32(reg->value, 0, reg->size, data);
	reg->dirty = false;
	reg->valid = true;

	return ERROR_OK;
}

static int esirisc_write_csr(struct reg *reg)
{
	struct esirisc_reg *reg_info = reg->arch_info;
	struct esirisc_common *esirisc = reg_info->esirisc;
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	struct target *target = esirisc->target;
	uint32_t data = buf_get_u32(reg->value, 0, reg->size);

	LOG_DEBUG("-");

	int retval = esirisc_jtag_write_csr(jtag_info, reg_info->bank, reg_info->csr, data);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write CSR: %s", target_name(target), reg->name);
		return retval;
	}

	reg->dirty = false;
	reg->valid = true;

	return ERROR_OK;
}

static int esirisc_get_reg(struct reg *reg)
{
	struct esirisc_reg *reg_info = reg->arch_info;
	struct esirisc_common *esirisc = reg_info->esirisc;
	struct target *target = esirisc->target;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	return reg_info->read(reg);
}

static int esirisc_set_reg(struct reg *reg, uint8_t *buf)
{
	struct esirisc_reg *reg_info = reg->arch_info;
	struct esirisc_common *esirisc = reg_info->esirisc;
	struct target *target = esirisc->target;
	uint32_t value = buf_get_u32(buf, 0, reg->size);

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static const struct reg_arch_type esirisc_reg_type = {
	.get = esirisc_get_reg,
	.set = esirisc_set_reg,
};

static struct reg_cache *esirisc_build_reg_cache(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(ESIRISC_NUM_REGS, sizeof(struct reg));

	LOG_DEBUG("-");

	cache->name = "eSi-RISC registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = ESIRISC_NUM_REGS;
	(*cache_p) = cache;

	esirisc->reg_cache = cache;
	esirisc->epc = reg_list + ESIRISC_EPC;
	esirisc->ecas = reg_list + ESIRISC_ECAS;
	esirisc->eid = reg_list + ESIRISC_EID;
	esirisc->ed = reg_list + ESIRISC_ED;

	for (int i = 0; i < esirisc->num_regs; ++i) {
		struct reg *reg = reg_list + esirisc_regs[i].number;
		struct esirisc_reg *reg_info = calloc(1, sizeof(struct esirisc_reg));

		reg->name = esirisc_regs[i].name;
		reg->number = esirisc_regs[i].number;
		reg->value = calloc(1, DIV_ROUND_UP(esirisc->num_bits, 8));
		reg->size = esirisc->num_bits;
		reg->reg_data_type = calloc(1, sizeof(struct reg_data_type));
		reg->reg_data_type->type = esirisc_regs[i].type;
		reg->group = esirisc_regs[i].group;
		reg_info->esirisc = esirisc;
		reg_info->read = esirisc_read_reg;
		reg_info->write = esirisc_write_reg;
		reg->arch_info = reg_info;
		reg->type = &esirisc_reg_type;
		reg->exist = true;
	}

	for (size_t i = 0; i < ARRAY_SIZE(esirisc_csrs); ++i) {
		struct reg *reg = reg_list + esirisc_csrs[i].number;
		struct esirisc_reg *reg_info = calloc(1, sizeof(struct esirisc_reg));

		reg->name = esirisc_csrs[i].name;
		reg->number = esirisc_csrs[i].number;
		reg->value = calloc(1, DIV_ROUND_UP(esirisc->num_bits, 8));
		reg->size = esirisc->num_bits;
		reg->reg_data_type = calloc(1, sizeof(struct reg_data_type));
		reg->reg_data_type->type = esirisc_csrs[i].type;
		reg->group = esirisc_csrs[i].group;
		reg_info->esirisc = esirisc;
		reg_info->bank = esirisc_csrs[i].bank;
		reg_info->csr = esirisc_csrs[i].csr;
		reg_info->read = esirisc_read_csr;
		reg_info->write = esirisc_write_csr;
		reg->arch_info = reg_info;
		reg->type = &esirisc_reg_type;
		reg->exist = true;
	}

	return cache;
}

static int esirisc_identify(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t csr;
	int retval;

	LOG_DEBUG("-");

	retval = esirisc_jtag_read_csr(jtag_info, CSR_CONFIG, CSR_CONFIG_ARCH0, &csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Configuration CSR: ARCH0", target_name(target));
		return retval;
	}

	esirisc->num_bits = (csr >> 0) & 0x3f;			/* ARCH0.B */
	esirisc->num_regs = (csr >> 10) & 0x3f;			/* ARCH0.R */

	retval = esirisc_jtag_read_csr(jtag_info, CSR_CONFIG, CSR_CONFIG_MEM, &csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Configuration CSR: MEM", target_name(target));
		return retval;
	}

	target->endianness = (csr & 1<<0) ?				/* MEM.E */
			TARGET_BIG_ENDIAN : TARGET_LITTLE_ENDIAN;

	retval = esirisc_jtag_read_csr(jtag_info, CSR_CONFIG, CSR_CONFIG_IC, &csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Configuration CSR: IC", target_name(target));
		return retval;
	}

	esirisc->has_icache = !!(csr & 1<<0);			/* IC.E */

	retval = esirisc_jtag_read_csr(jtag_info, CSR_CONFIG, CSR_CONFIG_DC, &csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Configuration CSR: DC", target_name(target));
		return retval;
	}

	esirisc->has_dcache = !!(csr & 1<<0);			/* DC.E */

	retval = esirisc_jtag_read_csr(jtag_info, CSR_CONFIG, CSR_CONFIG_DBG, &csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Configuration CSR: DBG", target_name(target));
		return retval;
	}

	esirisc->num_breakpoints = (csr >> 7) & 0xf;	/* DBG.BP */
	esirisc->num_watchpoints = (csr >> 12) & 0xf;	/* DBG.WP */

	retval = esirisc_jtag_read_csr(jtag_info, CSR_CONFIG, CSR_CONFIG_TRACE, &csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Configuration CSR: TRACE", target_name(target));
		return retval;
	}

	esirisc->has_trace = !!(csr & 1<<0);			/* TRACE.T */

	return ERROR_OK;
}

static int esirisc_target_create(struct target *target, Jim_Interp *interp)
{
	struct jtag_tap *tap = target->tap;
	struct esirisc_common *esirisc;

	if (!tap)
		return ERROR_FAIL;

	if (tap->ir_length != INSTR_LENGTH) {
		LOG_ERROR("%s: invalid IR length; expected %d", target_name(target),
				INSTR_LENGTH);
		return ERROR_FAIL;
	}

	esirisc = calloc(1, sizeof(struct esirisc_common));
	if (!esirisc)
		return ERROR_FAIL;

	esirisc->target = target;
	esirisc->jtag_info.tap = tap;
	target->arch_info = esirisc;

	return ERROR_OK;
}

static int esirisc_init_target(struct command_context *cmd_ctx, struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);

	/* trap reset, error, and debug exceptions */
	esirisc->hwdc_save = HWDC_R | HWDC_E | HWDC_D;

	return ERROR_OK;
}

static int esirisc_examine(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	LOG_DEBUG("-");

	if (!target_was_examined(target)) {
		retval = esirisc_debug_enable(target);
		if (retval != ERROR_OK)
			return retval;

		/*
		 * In order to identify the target we must first halt the core.
		 * We quietly resume once identification has completed for those
		 * targets that were running when target_examine was called.
		 */
		if (esirisc_jtag_is_debug_active(jtag_info)) {
			if (target->state == TARGET_UNKNOWN)
				target->debug_reason = DBG_REASON_DBGRQ;

			target->state = TARGET_HALTED;
		} else {
			retval = esirisc_jtag_break(jtag_info);
			if (retval != ERROR_OK) {
				LOG_ERROR("%s: failed to halt target", target_name(target));
				return retval;
			}

			target->state = TARGET_RUNNING;
		}

		retval = esirisc_identify(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to identify target", target_name(target));
			return retval;
		}

		esirisc_build_reg_cache(target);

		esirisc_remove_breakpoints(target);
		esirisc_remove_watchpoints(target);

		esirisc_disable_step(target);
		esirisc_restore_hwdc(target);

		if (target->state == TARGET_HALTED)
			esirisc_save_interrupts(target);
		else {
			retval = esirisc_jtag_continue(jtag_info);
			if (retval != ERROR_OK) {
				LOG_ERROR("%s: failed to resume target", target_name(target));
				return retval;
			}
		}

		target_set_examined(target);

		LOG_INFO("%s: %d bit, %d registers, %s%s%s", target_name(target),
				esirisc->num_bits, esirisc->num_regs,
				target_endianness(target),
				esirisc->has_icache ? ", icache" : "",
				esirisc->has_dcache ? ", dcache" : "");

		LOG_INFO("%s: hardware has %d breakpoints, %d watchpoints%s", target_name(target),
				esirisc->num_breakpoints, esirisc->num_watchpoints,
				esirisc->has_trace ? ", trace" : "");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_cache_arch_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);

	if (CMD_ARGC > 0) {
		if (strcmp(*CMD_ARGV, "harvard") == 0)
			esirisc->cache_arch = ESIRISC_CACHE_HARVARD;
		else if (strcmp(*CMD_ARGV, "von_neumann") == 0)
			esirisc->cache_arch = ESIRISC_CACHE_VON_NEUMANN;
		else {
			LOG_ERROR("invalid cache_arch: %s", *CMD_ARGV);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	command_print(CMD, "esirisc cache_arch %s", esirisc_cache_arch_name(esirisc));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_flush_caches_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	int retval;

	if (!esirisc_has_cache(esirisc)) {
		LOG_ERROR("target does not support caching");
		return ERROR_FAIL;
	}

	retval = esirisc_flush_caches(target);

	command_print(CMD, "cache flush %s",
			(retval == ERROR_OK) ? "successful" : "failed");

	return retval;
}

static const struct {
	const char *name;
	int mask;
} esirisc_hwdc_masks[] = {
	{ "reset",		HWDC_R },
	{ "interrupt",	HWDC_I },
	{ "syscall",	HWDC_S },
	{ "error",		HWDC_E },
	{ "debug",		HWDC_D },
};

static int esirisc_find_hwdc_mask(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(esirisc_hwdc_masks); ++i)
		if (strcmp(esirisc_hwdc_masks[i].name, name) == 0)
			return esirisc_hwdc_masks[i].mask;

	return -1;
}

COMMAND_HANDLER(handle_esirisc_hwdc_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "all") == 0)
			esirisc->hwdc_save = HWDC_R | HWDC_I | HWDC_S | HWDC_E | HWDC_D;
		else {
			esirisc->hwdc_save = 0;
			if (strcmp(CMD_ARGV[0], "none") != 0) {
				while (CMD_ARGC-- > 0) {
					int mask = esirisc_find_hwdc_mask(CMD_ARGV[CMD_ARGC]);
					if (mask < 0) {
						LOG_ERROR("invalid mask: %s", CMD_ARGV[CMD_ARGC]);
						return ERROR_COMMAND_SYNTAX_ERROR;
					}
					esirisc->hwdc_save |= mask;
				}
			}
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(esirisc_hwdc_masks); ++i)
		command_print(CMD, "%9s: %s", esirisc_hwdc_masks[i].name,
				(esirisc->hwdc_save & esirisc_hwdc_masks[i].mask) ? "enabled" : "disabled");

	return ERROR_OK;
}

static const struct command_registration esirisc_exec_command_handlers[] = {
	{
		.name = "flush_caches",
		.handler = handle_esirisc_flush_caches_command,
		.mode = COMMAND_EXEC,
		.help = "flush instruction and data caches",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esirisc_any_command_handlers[] = {
	{
		.name = "cache_arch",
		.handler = handle_esirisc_cache_arch_command,
		.mode = COMMAND_ANY,
		.help = "configure cache architecture",
		.usage = "['harvard'|'von_neumann']",
	},
	{
		.name = "hwdc",
		.handler = handle_esirisc_hwdc_command,
		.mode = COMMAND_ANY,
		.help = "configure hardware debug control",
		.usage = "['all'|'none'|mask ...]",
	},
	{
		.chain = esirisc_exec_command_handlers
	},
	{
		.chain = esirisc_trace_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esirisc_command_handlers[] = {
	{
		.name = "esirisc",
		.mode = COMMAND_ANY,
		.help = "eSi-RISC command group",
		.usage = "",
		.chain = esirisc_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type esirisc_target = {
	.name = "esirisc",

	.poll = esirisc_poll,
	.arch_state = esirisc_arch_state,

	.halt = esirisc_halt,
	.resume = esirisc_resume,
	.step = esirisc_step,

	.assert_reset = esirisc_assert_reset,
	.deassert_reset = esirisc_deassert_reset,

	.get_gdb_arch = esirisc_get_gdb_arch,
	.get_gdb_reg_list = esirisc_get_gdb_reg_list,

	.read_memory = esirisc_read_memory,
	.write_memory = esirisc_write_memory,
	.checksum_memory = esirisc_checksum_memory,

	.add_breakpoint = esirisc_add_breakpoint,
	.remove_breakpoint = esirisc_remove_breakpoint,
	.add_watchpoint = esirisc_add_watchpoint,
	.remove_watchpoint = esirisc_remove_watchpoint,

	.commands = esirisc_command_handlers,

	.target_create = esirisc_target_create,
	.init_target = esirisc_init_target,
	.examine = esirisc_examine,
};
