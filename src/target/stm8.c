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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include "target.h"
#include "target_type.h"
#include "hello.h"
#include "jtag/interface.h"
#include "jtag/jtag.h"
#include "jtag/swim.h"
#include "register.h"
#include "breakpoints.h"
#include "algorithm.h"
#include "stm8.h"

static struct reg_cache *stm8_build_reg_cache(struct target *target);
static int stm8_read_core_reg(struct target *target, unsigned int num);
static int stm8_write_core_reg(struct target *target, unsigned int num);
static int stm8_save_context(struct target *target);
static void stm8_enable_breakpoints(struct target *target);
static int stm8_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
static int stm8_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
static void stm8_enable_watchpoints(struct target *target);
static int stm8_unset_watchpoint(struct target *target,
		struct watchpoint *watchpoint);
static int (*adapter_speed)(int speed);
extern struct adapter_driver *adapter_driver;

static const struct {
	unsigned id;
	const char *name;
	const uint8_t bits;
	enum reg_type type;
	const char *group;
	const char *feature;
	int flag;
} stm8_regs[] = {
	{  0,  "pc", 32, REG_TYPE_UINT32, "general", "org.gnu.gdb.stm8.core", 0 },
	{  1,  "a", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.stm8.core", 0 },
	{  2,  "x", 16, REG_TYPE_UINT16, "general", "org.gnu.gdb.stm8.core", 0 },
	{  3,  "y", 16, REG_TYPE_UINT16, "general", "org.gnu.gdb.stm8.core", 0 },
	{  4,  "sp", 16, REG_TYPE_UINT16, "general", "org.gnu.gdb.stm8.core", 0 },
	{  5,  "cc", 8, REG_TYPE_UINT8, "general", "org.gnu.gdb.stm8.core", 0 },
};

#define STM8_NUM_REGS ARRAY_SIZE(stm8_regs)
#define STM8_PC 0
#define STM8_A 1
#define STM8_X 2
#define STM8_Y 3
#define STM8_SP 4
#define STM8_CC 5

#define CC_I0 0x8
#define CC_I1 0x20

#define DM_REGS 0x7f00
#define DM_REG_A 0x7f00
#define DM_REG_PC 0x7f01
#define DM_REG_X 0x7f04
#define DM_REG_Y 0x7f06
#define DM_REG_SP 0x7f08
#define DM_REG_CC 0x7f0a

#define DM_BKR1E 0x7f90
#define DM_BKR2E 0x7f93
#define DM_CR1 0x7f96
#define DM_CR2 0x7f97
#define DM_CSR1 0x7f98
#define DM_CSR2 0x7f99

#define STE 0x40
#define STF 0x20
#define RST 0x10
#define BRW 0x08
#define BK2F 0x04
#define BK1F 0x02

#define SWBRK 0x20
#define SWBKF 0x10
#define STALL 0x08
#define FLUSH 0x01

#define FLASH_CR1_STM8S 0x505A
#define FLASH_CR2_STM8S 0x505B
#define FLASH_NCR2_STM8S 0x505C
#define FLASH_IAPSR_STM8S 0x505F
#define FLASH_PUKR_STM8S 0x5062
#define FLASH_DUKR_STM8S 0x5064

#define FLASH_CR1_STM8L 0x5050
#define FLASH_CR2_STM8L 0x5051
#define FLASH_NCR2_STM8L 0
#define FLASH_PUKR_STM8L 0x5052
#define FLASH_DUKR_STM8L 0x5053
#define FLASH_IAPSR_STM8L 0x5054

/* FLASH_IAPSR */
#define HVOFF 0x40
#define DUL 0x08
#define EOP 0x04
#define PUL 0x02
#define WR_PG_DIS 0x01

/* FLASH_CR2 */
#define OPT 0x80
#define WPRG 0x40
#define ERASE 0x20
#define FPRG 0x10
#define PRG 0x01

/* SWIM_CSR */
#define SAFE_MASK 0x80
#define NO_ACCESS 0x40
#define SWIM_DM 0x20
#define HS 0x10
#define OSCOFF 0x08
#define SWIM_RST 0x04
#define HSIT 0x02
#define PRI 0x01

#define SWIM_CSR 0x7f80

#define STM8_BREAK 0x8B

enum mem_type {
	RAM,
	FLASH,
	EEPROM,
	OPTION
};

struct stm8_algorithm {
	int common_magic;
};

struct stm8_core_reg {
	uint32_t num;
	struct target *target;
	struct stm8_common *stm8_common;
};

enum hw_break_type {
	/* break on execute */
	HWBRK_EXEC,
	/* break on read */
	HWBRK_RD,
	/* break on write */
	HWBRK_WR,
	/* break on read, write and execute */
	HWBRK_ACC
};

struct stm8_comparator {
	bool used;
	uint32_t bp_value;
	uint32_t reg_address;
	enum hw_break_type type;
};

static int stm8_adapter_read_memory(struct target *target,
		uint32_t addr, int size, int count, void *buf)
{
	return swim_read_mem(addr, size, count, buf);
}

static int stm8_adapter_write_memory(struct target *target,
		uint32_t addr, int size, int count, const void *buf)
{
	return swim_write_mem(addr, size, count, buf);
}

static int stm8_write_u8(struct target *target,
		uint32_t addr, uint8_t val)
{
	uint8_t buf[1];

	buf[0] = val;
	return swim_write_mem(addr, 1, 1, buf);
}

static int stm8_read_u8(struct target *target,
		uint32_t addr, uint8_t *val)
{
	return swim_read_mem(addr, 1, 1, val);
}

/*
	<enable == 0> Disables interrupts.
	If interrupts are enabled they are masked and the cc register
	is saved.

	<enable == 1> Enables interrupts.
	Enable interrupts is actually restoring I1 I0 state from previous
	call with enable == 0. Note that if stepping and breaking on a sim
	instruction will NOT work since the interrupt flags are restored on
	debug_entry. We don't have any way for the debugger to exclusively
	disable the interrupts
*/
static int stm8_enable_interrupts(struct target *target, int enable)
{
	struct stm8_common *stm8 = target_to_stm8(target);
	uint8_t cc;

	if (enable) {
		if (!stm8->cc_valid)
			return ERROR_OK; /* cc was not stashed */
		/* fetch current cc */
		stm8_read_u8(target, DM_REG_CC, &cc);
		/* clear I1 I0 */
		cc &= ~(CC_I0 + CC_I1);
		/* restore I1 & I0 from stash*/
		cc |= (stm8->cc & (CC_I0+CC_I1));
		/* update current cc */
		stm8_write_u8(target, DM_REG_CC, cc);
		stm8->cc_valid = false;
	} else {
		stm8_read_u8(target, DM_REG_CC, &cc);
		if ((cc & CC_I0) && (cc & CC_I1))
			return ERROR_OK; /* interrupts already masked */
		/* stash cc */
		stm8->cc = cc;
		stm8->cc_valid = true;
		/* mask interrupts (disable) */
		cc |= (CC_I0 + CC_I1);
		stm8_write_u8(target, DM_REG_CC, cc);
	}

	return ERROR_OK;
}

static int stm8_set_hwbreak(struct target *target,
		struct stm8_comparator comparator_list[])
{
	uint8_t buf[3];
	int i, ret;

	/* Refer to Table 4 in UM0470 */
	uint8_t bc = 0x5;
	uint8_t bir = 0;
	uint8_t biw = 0;

	uint32_t data;
	uint32_t addr;

	if (!comparator_list[0].used) {
		comparator_list[0].type = HWBRK_EXEC;
		comparator_list[0].bp_value = -1;
	}

	if (!comparator_list[1].used) {
		comparator_list[1].type = HWBRK_EXEC;
		comparator_list[1].bp_value = -1;
	}

	if ((comparator_list[0].type == HWBRK_EXEC)
			&& (comparator_list[1].type == HWBRK_EXEC)) {
		comparator_list[0].reg_address = 0;
		comparator_list[1].reg_address = 1;
	}

	if ((comparator_list[0].type == HWBRK_EXEC)
			&& (comparator_list[1].type != HWBRK_EXEC)) {
		comparator_list[0].reg_address = 0;
		comparator_list[1].reg_address = 1;
		switch (comparator_list[1].type) {
		case HWBRK_RD:
			bir = 1;
			break;
		case HWBRK_WR:
			biw = 1;
			break;
		default:
			bir = 1;
			biw = 1;
			break;
		}
	}

	if ((comparator_list[1].type == HWBRK_EXEC)
			&& (comparator_list[0].type != HWBRK_EXEC)) {
		comparator_list[0].reg_address = 1;
		comparator_list[1].reg_address = 0;
		switch (comparator_list[0].type) {
		case HWBRK_RD:
			bir = 1;
			break;
		case HWBRK_WR:
			biw = 1;
			break;
		default:
			bir = 1;
			biw = 1;
			break;
		}
	}

	if ((comparator_list[0].type != HWBRK_EXEC)
			&& (comparator_list[1].type != HWBRK_EXEC)) {
		if (comparator_list[0].type != comparator_list[1].type) {
			LOG_ERROR("data hw breakpoints must be of same type");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	for (i = 0; i < 2; i++) {
		data = comparator_list[i].bp_value;
		addr = comparator_list[i].reg_address;

		buf[0] = data >> 16;
		buf[1] = data >> 8;
		buf[2] = data;

		if (addr == 0) {
			ret = stm8_adapter_write_memory(target, DM_BKR1E, 1, 3, buf);
			LOG_DEBUG("DM_BKR1E=%" PRIx32, data);
		} else if (addr == 1) {
			ret = stm8_adapter_write_memory(target, DM_BKR2E, 1, 3, buf);
			LOG_DEBUG("DM_BKR2E=%" PRIx32, data);
		} else {
			LOG_DEBUG("addr=%" PRIu32, addr);
			return ERROR_FAIL;
		}

		if (ret != ERROR_OK)
			return ret;

		ret = stm8_write_u8(target, DM_CR1,
			(bc << 3) + (bir << 2) + (biw << 1));
		LOG_DEBUG("DM_CR1=%" PRIx8, buf[0]);
		if (ret != ERROR_OK)
			return ret;

	}
	return ERROR_OK;
}

/* read DM control and status regs */
static int stm8_read_dm_csrx(struct target *target, uint8_t *csr1,
		uint8_t *csr2)
{
	int ret;
	uint8_t buf[2];

	ret =  stm8_adapter_read_memory(target, DM_CSR1, 1, sizeof(buf), buf);
	if (ret != ERROR_OK)
		return ret;
	if (csr1)
		*csr1 = buf[0];
	if (csr2)
		*csr2 = buf[1];
	return ERROR_OK;
}

/* set or clear the single step flag in DM */
static int stm8_config_step(struct target *target, int enable)
{
	int ret;
	uint8_t csr1, csr2;

	ret = stm8_read_dm_csrx(target, &csr1, &csr2);
	if (ret != ERROR_OK)
		return ret;
	if (enable)
		csr1 |= STE;
	else
		csr1 &= ~STE;

	ret =  stm8_write_u8(target, DM_CSR1, csr1);
	if (ret != ERROR_OK)
		return ret;
	return ERROR_OK;
}

/* set the stall flag in DM */
static int stm8_debug_stall(struct target *target)
{
	int ret;
	uint8_t csr1, csr2;

	ret = stm8_read_dm_csrx(target, &csr1, &csr2);
	if (ret != ERROR_OK)
		return ret;
	csr2 |= STALL;
	ret =  stm8_write_u8(target, DM_CSR2, csr2);
	if (ret != ERROR_OK)
		return ret;
	return ERROR_OK;
}

static int stm8_configure_break_unit(struct target *target)
{
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	if (stm8->bp_scanned)
		return ERROR_OK;

	stm8->num_hw_bpoints = 2;
	stm8->num_hw_bpoints_avail = stm8->num_hw_bpoints;

	stm8->hw_break_list = calloc(stm8->num_hw_bpoints,
		sizeof(struct stm8_comparator));

	stm8->hw_break_list[0].reg_address = 0;
	stm8->hw_break_list[1].reg_address = 1;

	LOG_DEBUG("hw breakpoints: numinst %i numdata %i", stm8->num_hw_bpoints,
		stm8->num_hw_bpoints);

	stm8->bp_scanned = true;

	return ERROR_OK;
}

static int stm8_examine_debug_reason(struct target *target)
{
	int retval;
	uint8_t csr1, csr2;

	retval = stm8_read_dm_csrx(target, &csr1, &csr2);
	if (retval == ERROR_OK)
		LOG_DEBUG("csr1 = 0x%02X csr2 = 0x%02X", csr1, csr2);

	if ((target->debug_reason != DBG_REASON_DBGRQ)
		&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {

		if (retval != ERROR_OK)
			return retval;

		if (csr1 & RST)
			/* halted on reset */
			target->debug_reason = DBG_REASON_UNDEFINED;

		if (csr1 & (BK1F+BK2F))
			/* we have halted on a  breakpoint (or wp)*/
			target->debug_reason = DBG_REASON_BREAKPOINT;

		if (csr2 & SWBKF)
			/* we have halted on a  breakpoint */
			target->debug_reason = DBG_REASON_BREAKPOINT;

	}

	return ERROR_OK;
}

static int stm8_debug_entry(struct target *target)
{
	struct stm8_common *stm8 = target_to_stm8(target);

	/* restore interrupts */
	stm8_enable_interrupts(target, 1);

	stm8_save_context(target);

	/* make sure stepping disabled STE bit in CSR1 cleared */
	stm8_config_step(target, 0);

	/* attempt to find halt reason */
	stm8_examine_debug_reason(target);

	LOG_DEBUG("entered debug state at PC 0x%" PRIx32 ", target->state: %s",
		buf_get_u32(stm8->core_cache->reg_list[STM8_PC].value, 0, 32),
		target_state_name(target));

	return ERROR_OK;
}

/* clear stall flag in DM and flush instruction pipe */
static int stm8_exit_debug(struct target *target)
{
	int ret;
	uint8_t csr1, csr2;

	ret = stm8_read_dm_csrx(target, &csr1, &csr2);
	if (ret != ERROR_OK)
		return ret;
	csr2 |= FLUSH;
	ret =  stm8_write_u8(target, DM_CSR2, csr2);
	if (ret != ERROR_OK)
		return ret;

	csr2 &= ~STALL;
	csr2 |= SWBRK;
	ret =  stm8_write_u8(target, DM_CSR2, csr2);
	if (ret != ERROR_OK)
		return ret;
	return ERROR_OK;
}

static int stm8_read_regs(struct target *target, uint32_t regs[])
{
	int ret;
	uint8_t buf[11];

	ret =  stm8_adapter_read_memory(target, DM_REGS, 1, sizeof(buf), buf);
	if (ret != ERROR_OK)
		return ret;

	regs[0] = be_to_h_u24(buf+DM_REG_PC-DM_REGS);
	regs[1] = buf[DM_REG_A-DM_REGS];
	regs[2] = be_to_h_u16(buf+DM_REG_X-DM_REGS);
	regs[3] = be_to_h_u16(buf+DM_REG_Y-DM_REGS);
	regs[4] = be_to_h_u16(buf+DM_REG_SP-DM_REGS);
	regs[5] = buf[DM_REG_CC-DM_REGS];

	return ERROR_OK;
}

static int stm8_write_regs(struct target *target, uint32_t regs[])
{
	int ret;
	uint8_t buf[11];

	h_u24_to_be(buf+DM_REG_PC-DM_REGS, regs[0]);
	buf[DM_REG_A-DM_REGS] = regs[1];
	h_u16_to_be(buf+DM_REG_X-DM_REGS, regs[2]);
	h_u16_to_be(buf+DM_REG_Y-DM_REGS, regs[3]);
	h_u16_to_be(buf+DM_REG_SP-DM_REGS, regs[4]);
	buf[DM_REG_CC-DM_REGS] = regs[5];

	ret =  stm8_adapter_write_memory(target, DM_REGS, 1, sizeof(buf), buf);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int stm8_get_core_reg(struct reg *reg)
{
	int retval;
	struct stm8_core_reg *stm8_reg = reg->arch_info;
	struct target *target = stm8_reg->target;
	struct stm8_common *stm8_target = target_to_stm8(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = stm8_target->read_core_reg(target, stm8_reg->num);

	return retval;
}

static int stm8_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct stm8_core_reg *stm8_reg = reg->arch_info;
	struct target *target = stm8_reg->target;
	uint32_t value = buf_get_u32(buf, 0, reg->size);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static int stm8_save_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	/* read core registers */
	stm8_read_regs(target, stm8->core_regs);

	for (i = 0; i < STM8_NUM_REGS; i++) {
		if (!stm8->core_cache->reg_list[i].valid)
			stm8->read_core_reg(target, i);
	}

	return ERROR_OK;
}

static int stm8_restore_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	for (i = 0; i < STM8_NUM_REGS; i++) {
		if (stm8->core_cache->reg_list[i].dirty)
			stm8->write_core_reg(target, i);
	}

	/* write core regs */
	stm8_write_regs(target, stm8->core_regs);

	return ERROR_OK;
}

static int stm8_unlock_flash(struct target *target)
{
	uint8_t data[1];

	struct stm8_common *stm8 = target_to_stm8(target);

	/* check if flash is unlocked */
	stm8_read_u8(target, stm8->flash_iapsr, data);
	if (~data[0] & PUL) {
		/* unlock flash */
		stm8_write_u8(target, stm8->flash_pukr, 0x56);
		stm8_write_u8(target, stm8->flash_pukr, 0xae);
	}

	stm8_read_u8(target, stm8->flash_iapsr, data);
	if (~data[0] & PUL)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int stm8_unlock_eeprom(struct target *target)
{
	uint8_t data[1];

	struct stm8_common *stm8 = target_to_stm8(target);

	/* check if eeprom is unlocked */
	stm8_read_u8(target, stm8->flash_iapsr, data);
	if (~data[0] & DUL) {
		/* unlock eeprom */
		stm8_write_u8(target, stm8->flash_dukr, 0xae);
		stm8_write_u8(target, stm8->flash_dukr, 0x56);
	}

	stm8_read_u8(target, stm8->flash_iapsr, data);
	if (~data[0] & DUL)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int stm8_write_flash(struct target *target, enum mem_type type,
		uint32_t address,
		uint32_t size, uint32_t count, uint32_t blocksize_param,
		const uint8_t *buffer)
{
	struct stm8_common *stm8 = target_to_stm8(target);

	uint8_t iapsr;
	uint8_t opt = 0;
	unsigned int i;
	uint32_t blocksize = 0;
	uint32_t bytecnt;
	int res;

	switch (type) {
		case (FLASH):
			stm8_unlock_flash(target);
			break;
		case (EEPROM):
			stm8_unlock_eeprom(target);
			break;
		case (OPTION):
			stm8_unlock_eeprom(target);
			opt = OPT;
			break;
		default:
			LOG_ERROR("BUG: wrong mem_type %d", type);
			assert(0);
	}

	if (size == 2) {
		/* we don't support short writes */
		count = count * 2;
		size = 1;
	}

	bytecnt = count * size;

	while (bytecnt) {
		if ((bytecnt >= blocksize_param) && ((address & (blocksize_param-1)) == 0)) {
			if (stm8->flash_cr2)
				stm8_write_u8(target, stm8->flash_cr2, PRG + opt);
			if (stm8->flash_ncr2)
				stm8_write_u8(target, stm8->flash_ncr2, ~(PRG + opt));
			blocksize = blocksize_param;
		} else
		if ((bytecnt >= 4) && ((address & 0x3) == 0)) {
			if (stm8->flash_cr2)
				stm8_write_u8(target, stm8->flash_cr2, WPRG + opt);
			if (stm8->flash_ncr2)
				stm8_write_u8(target, stm8->flash_ncr2, ~(WPRG + opt));
			blocksize = 4;
		} else
		if (blocksize != 1) {
			if (stm8->flash_cr2)
				stm8_write_u8(target, stm8->flash_cr2, opt);
			if (stm8->flash_ncr2)
				stm8_write_u8(target, stm8->flash_ncr2, ~opt);
			blocksize = 1;
		}

		res = stm8_adapter_write_memory(target, address, 1, blocksize, buffer);
		if (res != ERROR_OK)
			return res;
		address += blocksize;
		buffer += blocksize;
		bytecnt -= blocksize;

		/* lets hang here until end of program (EOP) */
		for (i = 0; i < 16; i++) {
			stm8_read_u8(target, stm8->flash_iapsr, &iapsr);
			if (iapsr & EOP)
				break;
			else
				usleep(1000);
		}
		if (i == 16)
			return ERROR_FAIL;
	}

	/* disable write access */
	res = stm8_write_u8(target, stm8->flash_iapsr, 0x0);

	if (res != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int stm8_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count,
		const uint8_t *buffer)
{
	struct stm8_common *stm8 = target_to_stm8(target);

	LOG_DEBUG("address: 0x%8.8" TARGET_PRIxADDR
		", size: 0x%8.8" PRIx32
		", count: 0x%8.8" PRIx32,
		address, size, count);

	if (target->state != TARGET_HALTED)
		LOG_WARNING("target not halted");

	int retval;

	if ((address >= stm8->flashstart) && (address <= stm8->flashend))
		retval = stm8_write_flash(target, FLASH, address, size, count,
				stm8->blocksize, buffer);
	else if ((address >= stm8->eepromstart) && (address <= stm8->eepromend))
		retval = stm8_write_flash(target, EEPROM, address, size, count,
				stm8->blocksize, buffer);
	else if ((address >= stm8->optionstart) && (address <= stm8->optionend))
		retval = stm8_write_flash(target, OPTION, address, size, count, 0, buffer);
	else
		retval = stm8_adapter_write_memory(target, address, size, count,
				buffer);

	if (retval != ERROR_OK)
		return ERROR_TARGET_FAILURE;

	return retval;
}

static int stm8_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("address: 0x%8.8" TARGET_PRIxADDR
		", size: 0x%8.8" PRIx32
		", count: 0x%8.8" PRIx32,
		address, size, count);

	if (target->state != TARGET_HALTED)
		LOG_WARNING("target not halted");

	int retval;
	retval = stm8_adapter_read_memory(target, address, size, count, buffer);

	if (retval != ERROR_OK)
		return ERROR_TARGET_FAILURE;

	return retval;
}

static int stm8_speed(int speed)
{
	int retval;
	uint8_t csr;

	LOG_DEBUG("stm8_speed: %d", speed);

	csr = SAFE_MASK | SWIM_DM;
	if (speed >= SWIM_FREQ_HIGH)
		csr |= HS;

	LOG_DEBUG("writing B0 to SWIM_CSR (SAFE_MASK + SWIM_DM + HS:%d)", csr & HS ? 1 : 0);
	retval = stm8_write_u8(NULL, SWIM_CSR, csr);
	if (retval != ERROR_OK)
		return retval;
	return adapter_speed(speed);
}

static int stm8_init(struct command_context *cmd_ctx, struct target *target)
{
	/*
	 * FIXME: this is a temporarily hack that needs better implementation.
	 * Being the only overwrite of adapter_driver, it prevents declaring const
	 * the struct adapter_driver.
	 * intercept adapter_driver->speed() calls
	 */
	adapter_speed = adapter_driver->speed;
	adapter_driver->speed = stm8_speed;

	stm8_build_reg_cache(target);

	return ERROR_OK;
}

static int stm8_poll(struct target *target)
{
	int retval = ERROR_OK;
	uint8_t csr1, csr2;

#ifdef LOG_STM8
	LOG_DEBUG("target->state=%d", target->state);
#endif

	/* read dm_csrx control regs */
	retval = stm8_read_dm_csrx(target, &csr1, &csr2);
	if (retval != ERROR_OK) {
		LOG_DEBUG("stm8_read_dm_csrx failed retval=%d", retval);
		/*
		   We return ERROR_OK here even if we didn't get an answer.
		   openocd will call target_wait_state until we get target state TARGET_HALTED
		*/
		return ERROR_OK;
	}

	/* check for processor halted */
	if (csr2 & STALL) {
		if (target->state != TARGET_HALTED) {
			if (target->state == TARGET_UNKNOWN)
				LOG_DEBUG("DM_CSR2_STALL already set during server startup.");

			retval = stm8_debug_entry(target);
			if (retval != ERROR_OK) {
				LOG_DEBUG("stm8_debug_entry failed retval=%d", retval);
				return ERROR_TARGET_FAILURE;
			}

			if (target->state == TARGET_DEBUG_RUNNING) {
				target->state = TARGET_HALTED;
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			} else {
				target->state = TARGET_HALTED;
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
		}
	} else
		target->state = TARGET_RUNNING;
#ifdef LOG_STM8
	LOG_DEBUG("csr1 = 0x%02X csr2 = 0x%02X", csr1, csr2);
#endif
	return ERROR_OK;
}

static int stm8_halt(struct target *target)
{
	LOG_DEBUG("target->state: %s", target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	if (target->state == TARGET_RESET) {
		/* we came here in a reset_halt or reset_init sequence
		 * debug entry was already prepared in stm8_assert_reset()
		 */
		target->debug_reason = DBG_REASON_DBGRQ;

		return ERROR_OK;
	}


	/* break processor */
	stm8_debug_stall(target);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int stm8_reset_assert(struct target *target)
{
	int res = ERROR_OK;
	struct stm8_common *stm8 = target_to_stm8(target);
	bool use_srst_fallback = true;

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_HAS_SRST) {
		res = adapter_assert_reset();
		if (res == ERROR_OK)
			/* hardware srst supported */
			use_srst_fallback = false;
		else if (res != ERROR_COMMAND_NOTFOUND)
			/* some other failure */
			return res;
	}

	if (use_srst_fallback) {
		LOG_DEBUG("Hardware srst not supported, falling back to swim reset");
		res = swim_system_reset();
		if (res != ERROR_OK)
			return res;
	}

	/* registers are now invalid */
	register_cache_invalidate(stm8->core_cache);

	target->state = TARGET_RESET;
	target->debug_reason = DBG_REASON_NOTHALTED;

	if (target->reset_halt) {
		res = target_halt(target);
		if (res != ERROR_OK)
			return res;
	}

	return ERROR_OK;
}

static int stm8_reset_deassert(struct target *target)
{
	int res;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_HAS_SRST) {
		res = adapter_deassert_reset();
		if ((res != ERROR_OK) && (res != ERROR_COMMAND_NOTFOUND))
			return res;
	}

	/* The cpu should now be stalled. If halt was requested
	   let poll detect the stall */
	if (target->reset_halt)
		return ERROR_OK;

	/* Instead of going through saving context, polling and
	   then resuming target again just clear stall and proceed. */
	target->state = TARGET_RUNNING;
	return stm8_exit_debug(target);
}

/* stm8_single_step_core() is only used for stepping over breakpoints
   from stm8_resume() */
static int stm8_single_step_core(struct target *target)
{
	struct stm8_common *stm8 = target_to_stm8(target);

	/* configure single step mode */
	stm8_config_step(target, 1);

	/* disable interrupts while stepping */
	if (!stm8->enable_step_irq)
		stm8_enable_interrupts(target, 0);

	/* exit debug mode */
	stm8_exit_debug(target);

	stm8_debug_entry(target);

	return ERROR_OK;
}

static int stm8_resume(struct target *target, int current,
		target_addr_t address, int handle_breakpoints,
		int debug_execution)
{
	struct stm8_common *stm8 = target_to_stm8(target);
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;

	LOG_DEBUG("%d " TARGET_ADDR_FMT " %d %d", current, address,
			handle_breakpoints, debug_execution);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);
		stm8_enable_breakpoints(target);
		stm8_enable_watchpoints(target);
		struct stm8_comparator *comparator_list = stm8->hw_break_list;
		stm8_set_hwbreak(target, comparator_list);
	}

	/* current = 1: continue on current pc,
	   otherwise continue at <address> */
	if (!current) {
		buf_set_u32(stm8->core_cache->reg_list[STM8_PC].value,
			0, 32, address);
		stm8->core_cache->reg_list[STM8_PC].dirty = true;
		stm8->core_cache->reg_list[STM8_PC].valid = true;
	}

	if (!current)
		resume_pc = address;
	else
		resume_pc = buf_get_u32(
			stm8->core_cache->reg_list[STM8_PC].value,
			0, 32);

	stm8_restore_context(target);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at " TARGET_ADDR_FMT,
					breakpoint->address);
			stm8_unset_breakpoint(target, breakpoint);
			stm8_single_step_core(target);
			stm8_set_breakpoint(target, breakpoint);
		}
	}

	/* disable interrupts if we are debugging */
	if (debug_execution)
		stm8_enable_interrupts(target, 0);

	/* exit debug mode */
	stm8_exit_debug(target);
	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	register_cache_invalidate(stm8->core_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32 "", resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32 "", resume_pc);
	}

	return ERROR_OK;
}

static int stm8_init_flash_regs(bool enable_stm8l, struct stm8_common *stm8)
{
	stm8->enable_stm8l = enable_stm8l;

	if (stm8->enable_stm8l) {
		stm8->flash_cr2 = FLASH_CR2_STM8L;
		stm8->flash_ncr2 = FLASH_NCR2_STM8L;
		stm8->flash_iapsr = FLASH_IAPSR_STM8L;
		stm8->flash_dukr = FLASH_DUKR_STM8L;
		stm8->flash_pukr = FLASH_PUKR_STM8L;
	} else {
		stm8->flash_cr2 = FLASH_CR2_STM8S;
		stm8->flash_ncr2 = FLASH_NCR2_STM8S;
		stm8->flash_iapsr = FLASH_IAPSR_STM8S;
		stm8->flash_dukr = FLASH_DUKR_STM8S;
		stm8->flash_pukr = FLASH_PUKR_STM8S;
	}
	return ERROR_OK;
}

static int stm8_init_arch_info(struct target *target,
		struct stm8_common *stm8, struct jtag_tap *tap)
{
	target->endianness = TARGET_BIG_ENDIAN;
	target->arch_info = stm8;
	stm8->common_magic = STM8_COMMON_MAGIC;
	stm8->fast_data_area = NULL;
	stm8->blocksize = 0x80;
	stm8->flashstart = 0x8000;
	stm8->flashend = 0xffff;
	stm8->eepromstart = 0x4000;
	stm8->eepromend = 0x43ff;
	stm8->optionstart = 0x4800;
	stm8->optionend = 0x487F;

	/* has breakpoint/watchpoint unit been scanned */
	stm8->bp_scanned = false;
	stm8->hw_break_list = NULL;

	stm8->read_core_reg = stm8_read_core_reg;
	stm8->write_core_reg = stm8_write_core_reg;

	stm8_init_flash_regs(0, stm8);

	return ERROR_OK;
}

static int stm8_target_create(struct target *target,
		Jim_Interp *interp)
{

	struct stm8_common *stm8 = calloc(1, sizeof(struct stm8_common));

	stm8_init_arch_info(target, stm8, target->tap);
	stm8_configure_break_unit(target);

	return ERROR_OK;
}

static int stm8_read_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	if (num >= STM8_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = stm8->core_regs[num];
	LOG_DEBUG("read core reg %i value 0x%" PRIx32 "", num, reg_value);
	buf_set_u32(stm8->core_cache->reg_list[num].value, 0, 32, reg_value);
	stm8->core_cache->reg_list[num].valid = true;
	stm8->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int stm8_write_core_reg(struct target *target, unsigned int num)
{
	uint32_t reg_value;

	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	if (num >= STM8_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg_value = buf_get_u32(stm8->core_cache->reg_list[num].value, 0, 32);
	stm8->core_regs[num] = reg_value;
	LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", num, reg_value);
	stm8->core_cache->reg_list[num].valid = true;
	stm8->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static const char *stm8_get_gdb_arch(struct target *target)
{
	return "stm8";
}

static int stm8_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);
	unsigned int i;

	*reg_list_size = STM8_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < STM8_NUM_REGS; i++)
		(*reg_list)[i] = &stm8->core_cache->reg_list[i];

	return ERROR_OK;
}

static const struct reg_arch_type stm8_reg_type = {
	.get = stm8_get_core_reg,
	.set = stm8_set_core_reg,
};

static struct reg_cache *stm8_build_reg_cache(struct target *target)
{
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	int num_regs = STM8_NUM_REGS;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct stm8_core_reg *arch_info = malloc(
			sizeof(struct stm8_core_reg) * num_regs);
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "stm8 registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	stm8->core_cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = stm8_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].stm8_common = stm8;

		reg_list[i].name = stm8_regs[i].name;
		reg_list[i].size = stm8_regs[i].bits;

		reg_list[i].value = calloc(1, 4);
		reg_list[i].valid = false;
		reg_list[i].type = &stm8_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = stm8_regs[i].type;
		else {
			LOG_ERROR("unable to allocate reg type list");
			return NULL;
		}

		reg_list[i].dirty = false;
		reg_list[i].group = stm8_regs[i].group;
		reg_list[i].number = stm8_regs[i].id;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = stm8_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");
	}

	return cache;
}

static void stm8_free_reg_cache(struct target *target)
{
	struct stm8_common *stm8 = target_to_stm8(target);
	struct reg_cache *cache;
	struct reg *reg;
	unsigned int i;

	cache = stm8->core_cache;

	if (!cache)
		return;

	for (i = 0; i < cache->num_regs; i++) {
		reg = &cache->reg_list[i];

		free(reg->feature);
		free(reg->reg_data_type);
		free(reg->value);
	}

	free(cache->reg_list[0].arch_info);
	free(cache->reg_list);
	free(cache);

	stm8->core_cache = NULL;
}

static void stm8_deinit(struct target *target)
{
	struct stm8_common *stm8 = target_to_stm8(target);

	free(stm8->hw_break_list);

	stm8_free_reg_cache(target);

	free(stm8);
}

static int stm8_arch_state(struct target *target)
{
	struct stm8_common *stm8 = target_to_stm8(target);

	LOG_USER("target halted due to %s, pc: 0x%8.8" PRIx32 "",
		debug_reason_name(target),
		buf_get_u32(stm8->core_cache->reg_list[STM8_PC].value, 0, 32));

	return ERROR_OK;
}

static int stm8_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	LOG_DEBUG("%x " TARGET_ADDR_FMT " %x",
		current, address, handle_breakpoints);

	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);
	struct breakpoint *breakpoint = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u32(stm8->core_cache->reg_list[STM8_PC].value, 0, 32, address);
		stm8->core_cache->reg_list[STM8_PC].dirty = true;
		stm8->core_cache->reg_list[STM8_PC].valid = true;
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target,
				buf_get_u32(stm8->core_cache->reg_list[STM8_PC].value, 0, 32));
		if (breakpoint)
			stm8_unset_breakpoint(target, breakpoint);
	}

	/* restore context */
	stm8_restore_context(target);

	/* configure single step mode */
	stm8_config_step(target, 1);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* disable interrupts while stepping */
	if (!stm8->enable_step_irq)
		stm8_enable_interrupts(target, 0);

	/* exit debug mode */
	stm8_exit_debug(target);

	/* registers are now invalid */
	register_cache_invalidate(stm8->core_cache);

	LOG_DEBUG("target stepped ");
	stm8_debug_entry(target);

	if (breakpoint)
		stm8_set_breakpoint(target, breakpoint);

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

static void stm8_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint) {
		if (breakpoint->set == 0)
			stm8_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

static int stm8_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct stm8_common *stm8 = target_to_stm8(target);
	struct stm8_comparator *comparator_list = stm8->hw_break_list;
	int retval;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = 0;

		while (comparator_list[bp_num].used && (bp_num < stm8->num_hw_bpoints))
			bp_num++;
		if (bp_num >= stm8->num_hw_bpoints) {
			LOG_ERROR("Can not find free breakpoint register (bpid: %" PRIu32 ")",
					breakpoint->unique_id);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint->set = bp_num + 1;
		comparator_list[bp_num].used = true;
		comparator_list[bp_num].bp_value = breakpoint->address;
		comparator_list[bp_num].type = HWBRK_EXEC;

		retval = stm8_set_hwbreak(target, comparator_list);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("bpid: %" PRIu32 ", bp_num %i bp_value 0x%" PRIx32 "",
				  breakpoint->unique_id,
				  bp_num, comparator_list[bp_num].bp_value);
	} else if (breakpoint->type == BKPT_SOFT) {
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);
		if (breakpoint->length == 1) {
			uint8_t verify = 0x55;

			retval = target_read_u8(target, breakpoint->address,
					breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u8(target, breakpoint->address, STM8_BREAK);
			if (retval != ERROR_OK)
				return retval;

			retval = target_read_u8(target, breakpoint->address, &verify);
			if (retval != ERROR_OK)
				return retval;
			if (verify != STM8_BREAK) {
				LOG_ERROR("Unable to set breakpoint at address " TARGET_ADDR_FMT
						" - check that memory is read/writable",
						breakpoint->address);
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			}
		} else {
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint->set = 1; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int stm8_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct stm8_common *stm8 = target_to_stm8(target);
	int ret;

	if (breakpoint->type == BKPT_HARD) {
		if (stm8->num_hw_bpoints_avail < 1) {
			LOG_INFO("no hardware breakpoint available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		ret = stm8_set_breakpoint(target, breakpoint);
		if (ret != ERROR_OK)
			return ret;

		stm8->num_hw_bpoints_avail--;
		return ERROR_OK;
	}

	ret = stm8_set_breakpoint(target, breakpoint);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int stm8_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);
	struct stm8_comparator *comparator_list = stm8->hw_break_list;
	int retval;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = breakpoint->set - 1;
		if ((bp_num < 0) || (bp_num >= stm8->num_hw_bpoints)) {
			LOG_DEBUG("Invalid comparator number in breakpoint (bpid: %" PRIu32 ")",
					  breakpoint->unique_id);
			return ERROR_OK;
		}
		LOG_DEBUG("bpid: %" PRIu32 " - releasing hw: %d",
				breakpoint->unique_id,
				bp_num);
		comparator_list[bp_num].used = false;
		retval = stm8_set_hwbreak(target, comparator_list);
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* restore original instruction (kept in target endianness) */
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);
		if (breakpoint->length == 1) {
			uint8_t current_instr;

			/* check that user program has not
			  modified breakpoint instruction */
			retval = target_read_memory(target, breakpoint->address, 1, 1,
					(uint8_t *)&current_instr);
			if (retval != ERROR_OK)
				return retval;

			if (current_instr == STM8_BREAK) {
				retval = target_write_memory(target, breakpoint->address, 1, 1,
						breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;
			}
		} else
			return ERROR_FAIL;
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

static int stm8_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->set)
		stm8_unset_breakpoint(target, breakpoint);

	if (breakpoint->type == BKPT_HARD)
		stm8->num_hw_bpoints_avail++;

	return ERROR_OK;
}

static int stm8_set_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct stm8_common *stm8 = target_to_stm8(target);
	struct stm8_comparator *comparator_list = stm8->hw_break_list;
	int wp_num = 0;
	int ret;

	if (watchpoint->set) {
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}

	while (comparator_list[wp_num].used && (wp_num < stm8->num_hw_bpoints))
		wp_num++;
	if (wp_num >= stm8->num_hw_bpoints) {
		LOG_ERROR("Can not find free hw breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (watchpoint->length != 1) {
			LOG_ERROR("Only watchpoints of length 1 are supported");
			return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	enum hw_break_type enable = 0;

	switch (watchpoint->rw) {
		case WPT_READ:
			enable = HWBRK_RD;
			break;
		case WPT_WRITE:
			enable = HWBRK_WR;
			break;
		case WPT_ACCESS:
			enable = HWBRK_ACC;
			break;
		default:
			LOG_ERROR("BUG: watchpoint->rw neither read, write nor access");
	}

	comparator_list[wp_num].used = true;
	comparator_list[wp_num].bp_value = watchpoint->address;
	comparator_list[wp_num].type = enable;

	ret = stm8_set_hwbreak(target, comparator_list);
	if (ret != ERROR_OK) {
		comparator_list[wp_num].used = false;
		return ret;
	}

	watchpoint->set = wp_num + 1;

	LOG_DEBUG("wp_num %i bp_value 0x%" PRIx32 "",
			wp_num,
			comparator_list[wp_num].bp_value);

	return ERROR_OK;
}

static int stm8_add_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	int ret;
	struct stm8_common *stm8 = target_to_stm8(target);

	if (stm8->num_hw_bpoints_avail < 1) {
		LOG_INFO("no hardware watchpoints available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	ret = stm8_set_watchpoint(target, watchpoint);
	if (ret != ERROR_OK)
		return ret;

	stm8->num_hw_bpoints_avail--;
	return ERROR_OK;
}

static void stm8_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	/* set any pending watchpoints */
	while (watchpoint) {
		if (watchpoint->set == 0)
			stm8_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

static int stm8_unset_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);
	struct stm8_comparator *comparator_list = stm8->hw_break_list;

	if (!watchpoint->set) {
		LOG_WARNING("watchpoint not set");
		return ERROR_OK;
	}

	int wp_num = watchpoint->set - 1;
	if ((wp_num < 0) || (wp_num >= stm8->num_hw_bpoints)) {
		LOG_DEBUG("Invalid hw comparator number in watchpoint");
		return ERROR_OK;
	}
	comparator_list[wp_num].used = false;
	watchpoint->set = 0;

	stm8_set_hwbreak(target, comparator_list);

	return ERROR_OK;
}

static int stm8_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
		stm8_unset_watchpoint(target, watchpoint);

	stm8->num_hw_bpoints_avail++;

	return ERROR_OK;
}

static int stm8_examine(struct target *target)
{
	int retval;
	uint8_t csr1, csr2;
	/* get pointers to arch-specific information */
	struct stm8_common *stm8 = target_to_stm8(target);
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (!target_was_examined(target)) {
		if (!stm8->swim_configured) {
			stm8->swim_configured = true;
			/*
				Now is the time to deassert reset if connect_under_reset.
				Releasing reset line will cause the option bytes to load.
				The core will still be stalled.
			*/
			if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
				if (jtag_reset_config & RESET_SRST_NO_GATING)
					stm8_reset_deassert(target);
				else
					LOG_WARNING("\'srst_nogate\' reset_config option is required");
			}
		} else {
			LOG_INFO("trying to reconnect");

			retval = swim_reconnect();
			if (retval != ERROR_OK) {
				LOG_ERROR("reconnect failed");
				return ERROR_FAIL;
			}

			/* read dm_csrx control regs */
			retval = stm8_read_dm_csrx(target, &csr1, &csr2);
			if (retval != ERROR_OK) {
				LOG_ERROR("state query failed");
				return ERROR_FAIL;
			}
		}

		target_set_examined(target);

		return ERROR_OK;
	}

	return ERROR_OK;
}

/** Checks whether a memory region is erased. */
static int stm8_blank_check_memory(struct target *target,
		struct target_memory_check_block *blocks, int num_blocks, uint8_t erased_value)
{
	struct working_area *erase_check_algorithm;
	struct reg_param reg_params[2];
	struct mem_param mem_params[2];
	struct stm8_algorithm stm8_info;

	static const uint8_t stm8_erase_check_code[] = {
#include "../../contrib/loaders/erase_check/stm8_erase_check.inc"
	};

	if (erased_value != 0xff) {
		LOG_ERROR("Erase value 0x%02" PRIx8 " not yet supported for STM8",
			erased_value);
		return ERROR_FAIL;
	}

	/* make sure we have a working area */
	if (target_alloc_working_area(target, sizeof(stm8_erase_check_code),
			&erase_check_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	target_write_buffer(target, erase_check_algorithm->address,
			sizeof(stm8_erase_check_code), stm8_erase_check_code);

	stm8_info.common_magic = STM8_COMMON_MAGIC;

	init_mem_param(&mem_params[0], 0x0, 3, PARAM_OUT);
	buf_set_u32(mem_params[0].value, 0, 24, blocks[0].address);

	init_mem_param(&mem_params[1], 0x3, 3, PARAM_OUT);
	buf_set_u32(mem_params[1].value, 0, 24, blocks[0].size);

	init_reg_param(&reg_params[0], "a", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, erased_value);

	init_reg_param(&reg_params[1], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, erase_check_algorithm->address);

	int retval = target_run_algorithm(target, 2, mem_params, 2, reg_params,
			erase_check_algorithm->address + 6,
			erase_check_algorithm->address + (sizeof(stm8_erase_check_code) - 1),
			10000, &stm8_info);

	if (retval == ERROR_OK)
		blocks[0].result = (*(reg_params[0].value) == 0xff);

	destroy_mem_param(&mem_params[0]);
	destroy_mem_param(&mem_params[1]);
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	target_free_working_area(target, erase_check_algorithm);

	if (retval != ERROR_OK)
		return retval;

	return 1;	/* only one block has been checked */
}

static int stm8_checksum_memory(struct target *target, target_addr_t address,
		uint32_t count, uint32_t *checksum)
{
	/* let image_calculate_checksum() take care of business */
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

/* run to exit point. return error if exit point was not reached. */
static int stm8_run_and_wait(struct target *target, uint32_t entry_point,
		int timeout_ms, uint32_t exit_point, struct stm8_common *stm8)
{
	uint32_t pc;
	int retval;
	/* This code relies on the target specific resume() and
	   poll()->debug_entry() sequence to write register values to the
	   processor and the read them back */
	retval = target_resume(target, 0, entry_point, 0, 1);
	if (retval != ERROR_OK)
		return retval;

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

	pc = buf_get_u32(stm8->core_cache->reg_list[STM8_PC].value, 0, 32);
	if (exit_point && (pc != exit_point)) {
		LOG_DEBUG("failed algorithm halted at 0x%" PRIx32 " ", pc);
		return ERROR_TARGET_TIMEOUT;
	}

	return ERROR_OK;
}

static int stm8_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_params,
		struct reg_param *reg_params, target_addr_t entry_point,
		target_addr_t exit_point, int timeout_ms, void *arch_info)
{
	struct stm8_common *stm8 = target_to_stm8(target);

	uint32_t context[STM8_NUM_REGS];
	int retval = ERROR_OK;

	LOG_DEBUG("Running algorithm");

	/* NOTE: stm8_run_algorithm requires that each
	   algorithm uses a software breakpoint
	   at the exit point */

	if (stm8->common_magic != STM8_COMMON_MAGIC) {
		LOG_ERROR("current target isn't a STM8 target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache */
	for (unsigned int i = 0; i < STM8_NUM_REGS; i++) {
		if (!stm8->core_cache->reg_list[i].valid)
			stm8->read_core_reg(target, i);
		context[i] = buf_get_u32(stm8->core_cache->reg_list[i].value, 0, 32);
	}

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;
		retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size, mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN)
			continue;

		struct reg *reg = register_get_by_name(stm8->core_cache,
				reg_params[i].reg_name, 0);

		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (reg_params[i].size != 32) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		stm8_set_core_reg(reg, reg_params[i].value);
	}

	retval = stm8_run_and_wait(target, entry_point,
			timeout_ms, exit_point, stm8);

	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size, mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {
			struct reg *reg = register_get_by_name(stm8->core_cache,
					reg_params[i].reg_name, 0);
			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found",
						reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			if (reg_params[i].size != 32) {
				LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
						reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			buf_set_u32(reg_params[i].value,
					0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	/* restore everything we saved before */
	for (unsigned int i = 0; i < STM8_NUM_REGS; i++) {
		uint32_t regvalue;
		regvalue = buf_get_u32(stm8->core_cache->reg_list[i].value, 0, 32);
		if (regvalue != context[i]) {
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
				stm8->core_cache->reg_list[i].name, context[i]);
			buf_set_u32(stm8->core_cache->reg_list[i].value,
					0, 32, context[i]);
			stm8->core_cache->reg_list[i].valid = true;
			stm8->core_cache->reg_list[i].dirty = true;
		}
	}

	return ERROR_OK;
}

static int stm8_jim_configure(struct target *target, Jim_GetOptInfo *goi)
{
	struct stm8_common *stm8 = target_to_stm8(target);
	jim_wide w;
	int e;
	const char *arg;

	arg = Jim_GetString(goi->argv[0], NULL);
	if (!strcmp(arg, "-blocksize")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-blocksize ?bytes? ...");
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(goi, &w);
		if (e != JIM_OK)
			return e;

		stm8->blocksize = w;
		LOG_DEBUG("blocksize=%8.8" PRIx32, stm8->blocksize);
		return JIM_OK;
	}
	if (!strcmp(arg, "-flashstart")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-flashstart ?address? ...");
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(goi, &w);
		if (e != JIM_OK)
			return e;

		stm8->flashstart = w;
		LOG_DEBUG("flashstart=%8.8" PRIx32, stm8->flashstart);
		return JIM_OK;
	}
	if (!strcmp(arg, "-flashend")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-flashend ?address? ...");
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(goi, &w);
		if (e != JIM_OK)
			return e;

		stm8->flashend = w;
		LOG_DEBUG("flashend=%8.8" PRIx32, stm8->flashend);
		return JIM_OK;
	}
	if (!strcmp(arg, "-eepromstart")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-eepromstart ?address? ...");
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(goi, &w);
		if (e != JIM_OK)
			return e;

		stm8->eepromstart = w;
		LOG_DEBUG("eepromstart=%8.8" PRIx32, stm8->eepromstart);
		return JIM_OK;
	}
	if (!strcmp(arg, "-eepromend")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-eepromend ?address? ...");
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(goi, &w);
		if (e != JIM_OK)
			return e;

		stm8->eepromend = w;
		LOG_DEBUG("eepromend=%8.8" PRIx32, stm8->eepromend);
		return JIM_OK;
	}
	if (!strcmp(arg, "-optionstart")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-optionstart ?address? ...");
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(goi, &w);
		if (e != JIM_OK)
			return e;

		stm8->optionstart = w;
		LOG_DEBUG("optionstart=%8.8" PRIx32, stm8->optionstart);
		return JIM_OK;
	}
	if (!strcmp(arg, "-optionend")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		if (goi->argc == 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv,
					"-optionend ?address? ...");
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(goi, &w);
		if (e != JIM_OK)
			return e;

		stm8->optionend = w;
		LOG_DEBUG("optionend=%8.8" PRIx32, stm8->optionend);
		return JIM_OK;
	}
	if (!strcmp(arg, "-enable_step_irq")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		stm8->enable_step_irq = true;
		LOG_DEBUG("enable_step_irq=%8.8x", stm8->enable_step_irq);
		return JIM_OK;
	}
	if (!strcmp(arg, "-enable_stm8l")) {
		e = Jim_GetOpt_String(goi, &arg, NULL);
		if (e != JIM_OK)
			return e;

		stm8->enable_stm8l = true;
		LOG_DEBUG("enable_stm8l=%8.8x", stm8->enable_stm8l);
		stm8_init_flash_regs(stm8->enable_stm8l, stm8);
		return JIM_OK;
	}
	return JIM_CONTINUE;
}

COMMAND_HANDLER(stm8_handle_enable_step_irq_command)
{
	const char *msg;
	struct target *target = get_current_target(CMD_CTX);
	struct stm8_common *stm8 = target_to_stm8(target);
	bool enable = stm8->enable_step_irq;

	if (CMD_ARGC > 0) {
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], enable);
		stm8->enable_step_irq = enable;
	}
	msg = stm8->enable_step_irq ? "enabled" : "disabled";
	command_print(CMD, "enable_step_irq = %s", msg);
	return ERROR_OK;
}

COMMAND_HANDLER(stm8_handle_enable_stm8l_command)
{
	const char *msg;
	struct target *target = get_current_target(CMD_CTX);
	struct stm8_common *stm8 = target_to_stm8(target);
	bool enable = stm8->enable_stm8l;

	if (CMD_ARGC > 0) {
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], enable);
		stm8->enable_stm8l = enable;
	}
	msg = stm8->enable_stm8l ? "enabled" : "disabled";
	command_print(CMD, "enable_stm8l = %s", msg);
	stm8_init_flash_regs(stm8->enable_stm8l, stm8);
	return ERROR_OK;
}

static const struct command_registration stm8_exec_command_handlers[] = {
	{
		.name = "enable_step_irq",
		.handler = stm8_handle_enable_step_irq_command,
		.mode = COMMAND_ANY,
		.help = "Enable/disable irq handling during step",
		.usage = "[1/0]",
	},
	{
		.name = "enable_stm8l",
		.handler = stm8_handle_enable_stm8l_command,
		.mode = COMMAND_ANY,
		.help = "Enable/disable STM8L flash programming",
		.usage = "[1/0]",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration stm8_command_handlers[] = {
	{
		.name = "stm8",
		.mode = COMMAND_ANY,
		.help = "stm8 command group",
		.usage = "",
		.chain = stm8_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type stm8_target = {
	.name = "stm8",

	.poll = stm8_poll,
	.arch_state = stm8_arch_state,

	.halt = stm8_halt,
	.resume = stm8_resume,
	.step = stm8_step,

	.assert_reset = stm8_reset_assert,
	.deassert_reset = stm8_reset_deassert,

	.get_gdb_arch = stm8_get_gdb_arch,
	.get_gdb_reg_list = stm8_get_gdb_reg_list,

	.read_memory = stm8_read_memory,
	.write_memory = stm8_write_memory,
	.checksum_memory = stm8_checksum_memory,
	.blank_check_memory = stm8_blank_check_memory,

	.run_algorithm = stm8_run_algorithm,

	.add_breakpoint = stm8_add_breakpoint,
	.remove_breakpoint = stm8_remove_breakpoint,
	.add_watchpoint = stm8_add_watchpoint,
	.remove_watchpoint = stm8_remove_watchpoint,

	.commands = stm8_command_handlers,
	.target_create = stm8_target_create,
	.init_target = stm8_init,
	.examine = stm8_examine,

	.deinit_target = stm8_deinit,
	.target_jim_configure = stm8_jim_configure,
};
