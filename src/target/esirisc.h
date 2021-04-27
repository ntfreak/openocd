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

#ifndef OPENOCD_TARGET_ESIRISC_H
#define OPENOCD_TARGET_ESIRISC_H

#include <helper/types.h>
#include <target/breakpoints.h>
#include <target/register.h>
#include <target/target.h>

#include "esirisc_jtag.h"
#include "esirisc_regs.h"
#include "esirisc_trace.h"

#define MAX_BREAKPOINTS			8
#define MAX_WATCHPOINTS			8

/* Exception IDs */
#define EID_RESET				0x00
#define EID_HARDWARE_FAILURE	0x01
#define EID_NMI					0x02
#define EID_INST_BREAKPOINT		0x03
#define EID_DATA_BREAKPOINT		0x04
#define EID_UNSUPPORTED			0x05
#define EID_PRIVILEGE_VIOLATION	0x06
#define EID_INST_BUS_ERROR		0x07
#define EID_DATA_BUS_ERROR		0x08
#define EID_ALIGNMENT_ERROR		0x09
#define EID_ARITHMETIC_ERROR	0x0a
#define EID_SYSTEM_CALL			0x0b
#define EID_MEMORY_MANAGEMENT	0x0c
#define EID_UNRECOVERABLE		0x0d
#define EID_INTERRUPT_N			0x20

/* Exception Entry Points */
#define ENTRY_RESET				0x00
#define ENTRY_UNRECOVERABLE		0x01
#define ENTRY_HARDWARE_FAILURE	0x02
#define ENTRY_RUNTIME			0x03
#define ENTRY_MEMORY			0x04
#define ENTRY_SYSCALL			0x05
#define ENTRY_DEBUG				0x06
#define ENTRY_NMI				0x07
#define ENTRY_INTERRUPT_N		0x08

/* Hardware Debug Control */
#define HWDC_R					(1<<4)	/* Reset & Hardware Failure */
#define HWDC_I					(1<<3)	/* Interrupts */
#define HWDC_S					(1<<2)	/* System Calls */
#define HWDC_E					(1<<1)	/* Program Errors */
#define HWDC_D					(1<<0)	/* Debug Exceptions */

enum esirisc_cache {
	ESIRISC_CACHE_VON_NEUMANN,
	ESIRISC_CACHE_HARVARD,
};

struct esirisc_common {
	struct target *target;
	struct esirisc_jtag jtag_info;
	enum esirisc_cache cache_arch;
	char *gdb_arch;

	struct reg_cache *reg_cache;
	struct reg *epc;
	struct reg *ecas;
	struct reg *eid;
	struct reg *ed;
	uint32_t etc_save;
	uint32_t hwdc_save;

	int num_bits;
	int num_regs;
	bool has_icache;
	bool has_dcache;
	bool has_trace;

	int num_breakpoints;
	struct breakpoint *breakpoints_p[MAX_BREAKPOINTS];

	int num_watchpoints;
	struct watchpoint *watchpoints_p[MAX_WATCHPOINTS];

	struct esirisc_trace trace_info;
};

union esirisc_memory {
	uint32_t word;
	uint16_t hword;
	uint8_t byte;
};

struct esirisc_reg {
	struct esirisc_common *esirisc;

	uint8_t bank;
	uint8_t csr;

	int (*read)(struct reg *reg);
	int (*write)(struct reg *reg);
};

static inline struct esirisc_common *target_to_esirisc(struct target *target)
{
	return (struct esirisc_common *)target->arch_info;
}

static inline char *esirisc_cache_arch_name(struct esirisc_common *esirisc)
{
	return esirisc->cache_arch == ESIRISC_CACHE_HARVARD ? "harvard" : "von_neumann";
}

static inline bool esirisc_has_cache(struct esirisc_common *esirisc)
{
	return esirisc->has_icache || esirisc->has_dcache;
}

#endif /* OPENOCD_TARGET_ESIRISC_H */
