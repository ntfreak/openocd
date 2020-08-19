/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef TARGET__RISCV__ASM_H
#define TARGET__RISCV__ASM_H

#include "riscv.h"

/*** Version-independent functions that we don't want in the main address space. ***/

static uint32_t load(const struct target *target, unsigned int rd,
		unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t load(const struct target *target, unsigned int rd,
		unsigned int base, uint16_t offset)
{
	switch (riscv_xlen(target)) {
		case 32:
			return lw(rd, base, offset);
		case 64:
			return ld(rd, base, offset);
	}
	assert(0);
	return 0; /* Silence -Werror=return-type */
}

static uint32_t store(const struct target *target, unsigned int src,
		unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t store(const struct target *target, unsigned int src,
		unsigned int base, uint16_t offset)
{
	switch (riscv_xlen(target)) {
		case 32:
			return sw(src, base, offset);
		case 64:
			return sd(src, base, offset);
	}
	assert(0);
	return 0; /* Silence -Werror=return-type */
}

#endif
