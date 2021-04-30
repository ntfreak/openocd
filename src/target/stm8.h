/*
*   OpenOCD STM8 target driver
*   Copyright (C) 2017  Ake Rehnman
*   ake.rehnman(at)gmail.com
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPENOCD_TARGET_STM8_H
#define OPENOCD_TARGET_STM8_H

struct target;

#define STM8_COMMON_MAGIC	0x53544D38
#define STM8_NUM_CORE_REGS 6

struct stm8_common {
	uint32_t common_magic;
	void *arch_info;
	struct reg_cache *core_cache;
	uint32_t core_regs[STM8_NUM_CORE_REGS];

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	bool swim_configured;
	bool bp_scanned;
	uint8_t num_hw_bpoints;
	uint8_t num_hw_bpoints_avail;
	struct stm8_comparator *hw_break_list;
	uint32_t blocksize;
	uint32_t flashstart;
	uint32_t flashend;
	uint32_t eepromstart;
	uint32_t eepromend;
	uint32_t optionstart;
	uint32_t optionend;
	bool enable_step_irq;

	bool enable_stm8l;
	uint32_t flash_cr2;
	uint32_t flash_ncr2;
	uint32_t flash_iapsr;
	uint32_t flash_dukr;
	uint32_t flash_pukr;

	/* cc value used for interrupt flags restore */
	uint32_t cc;
	bool cc_valid;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned int num);
	int (*write_core_reg)(struct target *target, unsigned int num);
};

static inline struct stm8_common *
target_to_stm8(struct target *target)
{
	return target->arch_info;
}

#endif /* OPENOCD_TARGET_STM8_H */
