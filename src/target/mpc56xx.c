/***************************************************************************
 *   Copyright (C) 2017 by James Murray <james@nscc.info                   *
 *   Based on code:                                                        *
 *       Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>   *
 *   Based on mips_m4k code:                                               *
 *       Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>       *
 *       Copyright (C) 2008 by David T.L. Wong                             *
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

#include "jtag/jtag.h"
#include "register.h"
#include "algorithm.h"
#include "target.h"
#include "breakpoints.h"
#include "target_type.h"
#include "mpc5xxx_jtag.h"
#include "mpc5xxx.h"
#include "mpc56xx_jtag.h"
#include "mpc56xx_regs.h"
#include "mpc56xx.h"
#include "jtag/interface.h"


static enum target_state cur_state; /* tmp */

int mpc56xx_configure_break_unit(struct target *target)
{
	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	if (mpc56xx->bp_scanned)
		return ERROR_OK;

	mpc56xx->num_inst_bpoints = MPC56XX_NUM_BPS; /* MPC5634M */
	mpc56xx->num_inst_bpoints_avail = mpc56xx->num_inst_bpoints;
	mpc56xx->inst_break_list = calloc(mpc56xx->num_inst_bpoints,
		sizeof(struct mpc5xxx_comparator));

	mpc56xx->num_data_bpoints = MPC56XX_NUM_WPS; /* MPC5634M */
	mpc56xx->num_data_bpoints_avail = mpc56xx->num_data_bpoints;
	mpc56xx->data_break_list = calloc(mpc56xx->num_data_bpoints,
		sizeof(struct mpc5xxx_comparator));

	mpc56xx->bp_scanned = 1;

	return ERROR_OK;
}


/* enable (state=1) or disable (state=0) interrupts on halt and resume  */
/* need to disable EE in MCR before single stepping, to guarantee execution */
/* so may as well just leave it like this with no extra overhead .??   */
static int mpc56xx_enable_interrupts(struct target *target, int state)
{

	uint32_t val ;
	uint32_t opcode = state ? 0x7c008146 : 0x7c000146 ;
	struct mpc5xxx_jtag *jtag_info = &target_to_mpc5xxx(target)->jtag ;

	return  mpc56xx_exec_inst(jtag_info, opcode, 0, &val, 0);

}

static int mpc56xx_debug_entry(struct target *target, int async_flag)
{

	/*uint32_t dpc, dinst;*/
	int retval;
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	mpc56xx->jtag.once = MPC56XX_TAP_ONCE; /* Only CPU */
	mpc56xx->jtag.current_tap = -1; /* Invalid to force TAP selection */
	mpc56xx->jtag.jtag_irlen = 5;

	printf("Debug entry !\n");
	retval = mpc56xx_enter_debug(&mpc56xx->jtag, async_flag);
	if (retval)
		return retval;

	register_cache_invalidate(mpc56xx->core_cache);
	/*printf("About to save_context\n");*/
	mpc56xx_save_context(target);
	/* Force to say we are halted. Debug/halt appear to be synonymous? */
	target->state = TARGET_HALTED;
	mpc56xx->ctl_on_entry = mpc56xx->saved_ctl ;
	mpc56xx->msr_on_entry = mpc56xx->saved_msr ;



	printf("MSR on entry is 0x%08x\n", mpc56xx->msr_on_entry);
	printf("CTL on entry is 0x%08x\n", mpc56xx->ctl_on_entry);

	/* get all the debug registers! */
	const struct {
		int addr;
		char *name;
	} debug_regs[] = {
			{MPC5XXX_ONCE_DBCR0, "DBCR0"},
			{MPC56XX_ONCE_DBCR1, "DBCR1"},
			{MPC56XX_ONCE_DBCR2, "DBCR2"},
			{MPC56XX_ONCE_DBCR3, "DBCR3"},
			{MPC56XX_ONCE_DBSR, "DBSR"},
			/*{574, "DSRR0"},
			{575, "DSRR1"}*/
	};
	const int num_db_regs = sizeof(debug_regs) / sizeof(debug_regs[0]) ;

	for (int i = 0 ; i < num_db_regs ; i++) {
		uint32_t val;
		uint32_t res;
		res = mpc5xxx_once_read(&mpc56xx->jtag, debug_regs[i].addr, &val, 32);
		printf("%s (spr:%d) = 0x%08x\n", debug_regs[i].name, debug_regs[i].addr, res ? 0xdeadbeef  : val);
	}

	retval = mpc5xxx_once_write(&mpc56xx->jtag, MPC56XX_ONCE_DBSR, 0xffffffff, 32);
	if (retval)
		return retval;



	/* make sure break unit configured */
	mpc56xx_configure_break_unit(target);

	target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);

	return ERROR_OK;
}

static int mpc56xx_poll(struct target *target)
{
	uint32_t osr, tmp;
	int retval;
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	retval = mpc5xxx_once_read(&mpc56xx->jtag, MPC56XX_ONCE_DBSR, &tmp, 32);
	if (retval)
		return retval;
	if (tmp) {
		/*printf("DBSR = 0x%08x\n", tmp);*/
		;
	}

	retval = mpc5xxx_once_osr_read(&mpc56xx->jtag, &osr);
	if (retval != ERROR_OK)
		return retval;

	if (cur_state != target->state) {
		printf("Current target->state = %d : %s\n", target->state, target_state_name(target));
		cur_state = target->state;
	}

	/* debug and halt appear to be synonymous */
	if (osr & (MPC5XXX_OSR_HALT | MPC5XXX_OSR_DEBUG)) {
		if ((target->state == TARGET_RUNNING) || (target->state == TARGET_RESET)) {
			target->state = TARGET_HALTED;

			printf("no.1\n");
			retval = mpc56xx_debug_entry(target, 0);
			if (retval != ERROR_OK)
				return retval;

			/*target_call_event_callbacks(target, TARGET_EVENT_HALTED);*/
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		} else if (target->state == TARGET_DEBUG_RUNNING) {
			target->state = TARGET_HALTED;

			printf("no.2\n");
			retval = mpc56xx_debug_entry(target, 1);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}

	} else if (target->state == TARGET_DEBUG_RUNNING) { /* JSM */

		printf("no.3\n");
		retval = mpc56xx_debug_entry(target, 1);
		if (retval != ERROR_OK)
			return retval;

		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);

	} else {
		if (target->state != TARGET_DEBUG_RUNNING)
			target->state = TARGET_RUNNING;
	}


	return ERROR_OK;
}

/* Read the boot address from flash
 * Simplified version that only reads from 0 address.
 * Ought to read from the multiple possible addresses.
 */
static int mpc56xx_read_boot_vector(struct target *target, uint32_t *addr)
{
	int retval;
	uint32_t ad;

	*addr = 0;

	retval = target_read_memory(target, 0x00000000, 4, 1, (uint8_t *)&ad);
	if (retval != ERROR_OK)
		return retval;
	if (ad != 0x5a00) { /* 0x005a0000 BE */
		return ERROR_FAIL;
	}
	printf("Check *0x00000000 = 0x%08x\n", ad);

	retval = target_read_memory(target, 0x00000004, 4, 1, (uint8_t *)&ad);
	if (retval != ERROR_OK)
		return retval;
	*addr = ((ad & 0x000000ff) << 24)
				| ((ad & 0x0000ff00) << 8)
				| ((ad & 0x00ff0000) >> 8)
				| ((ad & 0xff000000) >> 24);
	printf("Check *0x00000004 = 0x%08x\n", *addr);

	return ERROR_OK;
}

/* Sets up the MMU to sensible 1:1 values
 */
static int mpc56xx_init_mmu(struct target *target)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	int retval;
#if 0
	uint32_t val;

/* Freescale datasheet says:
 *	required to setup an MMU entry are:
 *	1. Set up MAS0 (SPR 624)
 *	2. Set up MAS1 (SPR 625)
 *	3. Set up MAS2 (SPR 626)
 *	4. Set up MAS3 (SPR 627)
 *	5. Execute tlbwe (0x7C0007A4)
 *
 *	Memory region	MAS0		MAS1		MAS2		MAS3
 *	PBridge B		0x10000000	0xC0000500	0xFFF0000A	0xFFF0003F
 *	Internal SRAM	0x10010000	0xC0000400	0x40000008	0x4000003F
 *	PBridge A		0x10020000	0xC0000500	0xC3F00008	0xC3F0003F
 *	Internal flash	0x10030000	0xC0000700	0x00000000	0x0000003F
 */

	/* seems to be failing sometimes ? */
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 624, 0x10000000); /* MAS0 PBridge B */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 625, 0xc0000500); /* MAS1 */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 626, 0xfff0000a); /* MAS2 */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 627, 0xfff0003f); /* MAS3 */
	if (retval)
		return retval;
	retval = mpc56xx_exec_inst(&mpc56xx->jtag, 0x7c0007a4, 0, &val, 0); /* tlbwe */
	if (retval)
		return retval;

	retval = mpc56xx_write_spr(&mpc56xx->jtag, 624, 0x10010000); /* MAS0 SRAM */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 625, 0xc0000400); /* MAS1 */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 626, 0x40000008); /* MAS2 0x40000028 for VLE */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 627, 0x4000003f); /* MAS3 */
	if (retval)
		return retval;
	retval = mpc56xx_exec_inst(&mpc56xx->jtag, 0x7c0007a4, 0, &val, 0); /* tlbwe */
	if (retval)
		return retval;

	retval = mpc56xx_write_spr(&mpc56xx->jtag, 624, 0x10020000); /* MAS0 PBridge A */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 625, 0xc0000500); /* MAS1 */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 626, 0xc3f00008); /* MAS2 */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 627, 0xc3f0003f); /* MAS3 */
	if (retval)
		return retval;
	retval = mpc56xx_exec_inst(&mpc56xx->jtag, 0x7c0007a4, 0, &val, 0); /* tlbwe */
	if (retval)
		return retval;

#if 0
	From Freescale book e script
	/* Set up Memory Controller CS0 @ $3ff8_0000 */
	retval = target_write_u32(target, 0xc3f84010, 0x3ff80001);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, 0xc3f84014, 0xfff800f0);
	if (retval != ERROR_OK)
		return retval;

	/* Set up Memory Controller CS1 @ $2000_0000 */
	retval = target_write_u32(target, 0xc3f84018, 0x20000001);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, 0xc3f8401c, 0xfff800f0);
	if (retval != ERROR_OK)
		return retval;
#endif

	retval = mpc56xx_write_spr(&mpc56xx->jtag, 624, 0x10030000); /* MAS0 Flash */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 625, 0xc0000700); /* MAS1 */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 626, 0x00000000); /* MAS2 */
	if (retval)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 627, 0x0000003f); /* MAS3 */
	if (retval)
		return retval;
	retval = mpc56xx_exec_inst(&mpc56xx->jtag, 0x7c0007a4, 0, &val, 0); /* tlbwe */
	if (retval)
		return retval;
#endif

	/* Disable watchdog timer */
	retval = target_write_u32(target, 0xfff38000, 0xff00000A);
	if (retval != ERROR_OK)
		return retval;
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 340, 0);
	if (retval)
		return retval;

	/* May want to consider errata 7244 */

	return ERROR_OK;
}

/* Performs 64bit writes to whole of SRAM to setup ECC
 */
static int mpc56xx_init_sram(struct target *target)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	int retval;
	uint32_t val, addr;

	/* Method from Freescale datasheet - very slow */
	printf("init_sram");
	for (addr = 0; addr < MPC56XX_SIZE_OF_SRAM ; addr += 0x80) {
		retval = mpc56xx_exec_inst(&mpc56xx->jtag, 0xbc000000, addr, &val, MPC56XX_EI_VAL); /* stmw r0, 0(X) */
		if (retval)
			return retval;
		if ((addr & 0x3ff) == 0) { /* keep the GDB connection alive during slow operation */
			keep_alive();
			printf(".");
			fflush(stdout);
			addr = MPC56XX_SIZE_OF_SRAM; /* FIXME cheating */
		}
	}
	printf("done.\n");
	return ERROR_OK;
}

static int mpc56xx_init_pll(struct target *target)
{
	int retval;
	uint32_t val, retry;

	retval = target_write_u32(target, MPC56XX_FMPLL_ESYNCR2, 0x00000002);
	if (retval != ERROR_OK)
		return retval;

	/* Set 40MHz with 8MHz crystal */
	retval = target_write_u32(target, MPC56XX_FMPLL_ESYNCR1, 0xF0000000 + 40);
	if (retval != ERROR_OK)
		return retval;

	retry = 100; /* arbitrary limit */
	val = 0;
	while (retry && ((val & 8) == 0)) {
		retval = target_read_memory(target, MPC56XX_FMPLL_SYNSR, 4, 1,
				(uint8_t *)&val);
		val = be_to_h_u32((uint8_t *)&val); /* swap ends again */
		if (retval != ERROR_OK)
			return retval;
		retry--;
	}
	if (retry == 0) {
		printf("1. Failed to get PLL lock in expected time.\n");
		return ERROR_FAIL;
	}

	/* Set 64MHz with 8MHz crystal */
	retval = target_write_u32(target, MPC56XX_FMPLL_ESYNCR1, 0xF0000000 + 64);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int mpc56xx_init_vectors(struct target *target)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	int retval;
	uint32_t vec;
/* From Freescale init
 *	spr 63t $40000000
 *	spr 400t $0
 *	spr 401t $0
 *	spr 402t $0
 *	spr 403t $0
 *	spr 404t $0
 *	spr 405t $0
 *	spr 406t $0
 *	spr 407t $0
 *	spr 408t $0
 *	spr 409t $0
 *	spr 410t $0
 *	spr 411t $0
 *	spr 412t $0
 *	spr 413t $0    ; MMU data error vector points into valid memory
 *	spr 414t $0    ; MMU instruction error vector points into valid memory
 *	spr 415t $0
 */
	retval = mpc56xx_write_spr(&mpc56xx->jtag, 63, 0x40000000);
	if (retval)
		return retval;

	for (vec = 400; vec <= 415; vec++) {
		retval = mpc56xx_write_spr(&mpc56xx->jtag, vec, 0);
		if (retval)
			return retval;
	}

	return ERROR_OK;
}

static int mpc56xx_halt(struct target *target)
{
	int retval;

	printf("halt called\n");
	LOG_DEBUG("target->state: %s",  target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	if (target->state == TARGET_RESET) {
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) && jtag_get_srst()) {
			LOG_ERROR("can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		} else {
			target->debug_reason = DBG_REASON_DBGRQ;

			return ERROR_OK;
		}
	}

	printf("Supposed to halt, entering debug mode\n");
	retval = mpc56xx_debug_entry(target, 1);
	if (retval != ERROR_OK)
		return retval;

	target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);

	target->debug_reason = DBG_REASON_DBGRQ;
	target->state = TARGET_HALTED;

	return ERROR_OK;
}

static int mpc56xx_deassert_reset(struct target *target)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	int retval;

	printf("mpc56xx_deassert_reset\n");
	printf("reset_halt = %d\n", target->reset_halt);
	if (target->reset_halt) {
		retval = mpc56xx_enter_debug(&mpc56xx->jtag, 1);
		if (retval)
			return retval;
		LOG_DEBUG("Enabled debug after reset");
	} else {
		/* Ensure those bits are clear */
		retval = mpc5xxx_once_write(&mpc56xx->jtag, MPC5XXX_ONCE_OCR, MPC5XXX_OCR_DEBUG_OFF, 32);
		if (retval)
			return retval;
		LOG_DEBUG("Disabled debug after reset");
	}

	printf("Disabling Interrupts\n") ;
	mpc56xx_enable_interrupts(target, 0) ;

	adapter_deassert_reset();

	if (target->reset_halt) {


		/* grab PC from boot vector */
		uint32_t pc;
		retval = mpc56xx_read_boot_vector(target, &pc);
		if (retval)
			return retval;
		printf("Boot PC = 0x%08x\n", pc);

		buf_set_u32(mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].value, 0, 32, pc);
		mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].dirty = 1;
		mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].valid = 1;
		retval = mpc56xx_write_reg(&mpc56xx->jtag, MPC5XXX_REG_PC, pc);
		if (retval)
			return retval;
		/* Initialise MMU */
		printf("Initialise MMU in C-code\n");
		retval = mpc56xx_init_mmu(target);
		if (retval)
			return retval;
		printf("Initialise SRAM in C-code\n");
		retval = mpc56xx_init_sram(target);
		if (retval)
			return retval;
		printf("Initialise PLL in C-code\n");
		retval = mpc56xx_init_pll(target);
		if (retval)
			return retval;
		printf("Intialise interrupt vectors\n");
		retval = mpc56xx_init_vectors(target);
		if (retval)
			return retval;
		retval = mpc56xx_debug_entry(target, 1);
		if (retval != ERROR_OK)
			return retval;

		target->debug_reason = DBG_REASON_DBGRQ;
		target->state = TARGET_HALTED;
	}
	return ERROR_OK;
}

static int mpc56xx_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	struct mpc5xxx_comparator *comparator_list = mpc56xx->inst_break_list;
	int retval;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = 0;

		while (comparator_list[bp_num].used && (bp_num < mpc56xx->num_inst_bpoints))
			bp_num++;
		if (bp_num >= mpc56xx->num_inst_bpoints) {
			LOG_ERROR("Can not find free FP Comparator(bpid: %" PRIu32 ")",
					breakpoint->unique_id);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint->set = bp_num + 1;
		comparator_list[bp_num].used = 1;
		comparator_list[bp_num].bp_value = breakpoint->address;

		printf("Setting breakpoint #%d at 0x%08x\n", bp_num, (unsigned int)breakpoint->address);
		retval = mpc56xx_jtag_set_bp(&mpc56xx->jtag, bp_num, breakpoint->address);
		if (retval)
			return retval;

		LOG_DEBUG("bpid: %" PRIu32 ", bp_num %i bp_value 0x%" PRIx32 "",
				  breakpoint->unique_id,
				  bp_num, comparator_list[bp_num].bp_value);
	} else if (breakpoint->type == BKPT_SOFT) {
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);
		if (breakpoint->length == 4) {
			uint32_t verify = 0xffffffff;

			retval = target_read_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u32(target, breakpoint->address, MPC56XX_SWBP);
			if (retval != ERROR_OK)
				return retval;

			retval = target_read_u32(target, breakpoint->address, &verify);
			if (retval != ERROR_OK)
				return retval;
			if (verify != MPC56XX_SWBP) {
				LOG_ERROR("Unable to set 32bit breakpoint at address %08" PRIx32
						" - check that memory is read/writable", (unsigned int)breakpoint->address);
				return ERROR_OK;
			}
		} else {
			uint16_t verify = 0xffff;

			retval = target_read_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u16(target, breakpoint->address, MPC56XX_SWBP);
			if (retval != ERROR_OK)
				return retval;

			retval = target_read_u16(target, breakpoint->address, &verify);
			if (retval != ERROR_OK)
				return retval;
			if (verify != MPC56XX_SWBP) {
				LOG_ERROR("Unable to set 16bit breakpoint at address %08" PRIx32
						" - check that memory is read/writable", (unsigned int)breakpoint->address);
				return ERROR_OK;
			}
		}

		breakpoint->set = 20; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int mpc56xx_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint) {
		if (breakpoint->set == 0)
			mpc56xx_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
	return ERROR_OK;
}

static int mpc56xx_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	struct mpc5xxx_comparator *comparator_list = mpc56xx->inst_break_list;
	int retval;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int bp_num = breakpoint->set - 1;
		if ((bp_num < 0) || (bp_num >= mpc56xx->num_inst_bpoints)) {
			LOG_DEBUG("Invalid FP Comparator number in breakpoint (bpid: %" PRIu32 ")",
					  breakpoint->unique_id);
			return ERROR_OK;
		}
		LOG_DEBUG("bpid: %" PRIu32 " - releasing hw: %d",
				breakpoint->unique_id,
				bp_num);
		comparator_list[bp_num].used = 0;
		comparator_list[bp_num].bp_value = 0;
		/*target_write_u32(target, comparator_list[bp_num].reg_address +
				 ejtag_info->ejtag_ibc_offs, 0);*/

	} else {
		/* restore original instruction (kept in target endianness) */
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);
		if (breakpoint->length == 4) {
			uint32_t current_instr;

			/* check that user program has not modified breakpoint instruction */
			retval = target_read_memory(target, breakpoint->address, 4, 1,
					(uint8_t *)&current_instr);
			if (retval != ERROR_OK)
				return retval;

			/**
			 * target_read_memory() gets us data in _target_ endianess.
			 * If we want to use this data on the host for comparisons with some macros
			 * we must first transform it to _host_ endianess using target_buffer_get_u32().
			 */
			current_instr = target_buffer_get_u32(target, (uint8_t *)&current_instr);

			/*if (current_instr == mpc5634_SDBBP) {
				retval = target_write_memory(target, breakpoint->address, 4, 1,
						breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;
			}*/
		} else {
			uint16_t current_instr;
/* Not checked to see if this makes any sense JSM */

			/* check that user program has not modified breakpoint instruction */
			retval = target_read_memory(target, breakpoint->address, 2, 1,
					(uint8_t *)&current_instr);
			if (retval != ERROR_OK)
				return retval;
			current_instr = target_buffer_get_u16(target, (uint8_t *)&current_instr);
			/*if (current_instr == MIPS16_SDBBP) {
				retval = target_write_memory(target, breakpoint->address, 2, 1,
						breakpoint->orig_instr);*
				if (retval != ERROR_OK)
					return retval;
			}*/
		}
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

static int mpc56xx_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	printf("Adding a breakpoint @ 0x%08x, type = %d\n", (unsigned int)breakpoint->address, breakpoint->type);
		if (breakpoint->type == BKPT_HARD) {
			if (mpc56xx->num_inst_bpoints_avail < 1) {
				LOG_INFO("no hardware breakpoint available");
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			}

			mpc56xx->num_inst_bpoints_avail--;
		}

	return ERROR_OK;
}

static int mpc56xx_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->set)
		mpc56xx_unset_breakpoint(target, breakpoint);

	if (breakpoint->type == BKPT_HARD)
		mpc56xx->num_inst_bpoints_avail++;

	return ERROR_OK;
}

static int mpc56xx_step(struct target *target, int current,
	target_addr_t address, int handle_breakpoints)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	struct breakpoint *breakpoint = NULL;
	int retval;
	uint32_t addr, opcode, val;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u32(mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].value, 0, 32, address);
		mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].dirty = true;
		mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].valid = true;
	}

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target,
				buf_get_u32(mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].value, 0, 32));
		if (breakpoint)
			mpc56xx_remove_breakpoint(target, breakpoint);
	}

	addr = buf_get_u32(mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].value, 0, 32);

	/* Fetch the instruction from that address */
	mpc5xxx_jtag_read_memory32(&mpc56xx->jtag,
		addr, 1, &val);

	/* Opcode comes back endian swapped, fix it. */
	opcode = ((val & 0x000000ff) << 24)
			| ((val & 0x0000ff00) << 8)
			| ((val & 0x00ff0000) >> 8)
			| ((val & 0xff000000) >> 24);

	/* restore context incl. setting PC*/
	mpc56xx_restore_context(target);


	/* disable interrupts while stepping */
	/*mpc56xx_enable_interrupts(target, 0);*/

	target->debug_reason = DBG_REASON_SINGLESTEP;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	keep_alive();

	retval = mpc56xx_exec_inst(&mpc56xx->jtag, opcode, 0, &val,
			(mpc56xx->saved_ctl & 0xFFFF0000) | MPC56XX_EI_INC);

	if (retval)
		return retval;

	/* Fetch updated PC - may not be necessary ? */
	retval = mpc56xx_read_reg(&mpc56xx->jtag, MPC5XXX_REG_PC, &val);
	if (retval)
		return retval;

	/* registers are now invalid */
	register_cache_invalidate(mpc56xx->core_cache);

	mpc56xx_save_context(target);

	/* Save PC - not sure this is strictly necessary */
	buf_set_u32(mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].value, 0, 32, mpc56xx->core_regs[MPC5XXX_REG_PC]);

	/* Seems like these aren't handled correctly? GDB isn't getting new value. */
	mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].valid = true;

	if (breakpoint)
		mpc56xx_add_breakpoint(target, breakpoint);

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

static int mpc56xx_set_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	struct mpc5xxx_comparator *comparator_list = mpc56xx->data_break_list;
	int retval;

	if (watchpoint->set) {
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}

	int wp_num = 0;

	while (comparator_list[wp_num].used && (wp_num < mpc56xx->num_data_bpoints))
		wp_num++;
	if (wp_num >= mpc56xx->num_data_bpoints) {
		LOG_ERROR("Can not find free FP Comparator(wpid: %" PRIu32 ")",
				watchpoint->unique_id);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	watchpoint->set = wp_num + 1;
	comparator_list[wp_num].used = 1;
	comparator_list[wp_num].bp_value = watchpoint->address;

	printf("Actually need to set watchpoint #%d at 0x%08x\n", wp_num, (unsigned int)watchpoint->address);
	retval = mpc56xx_jtag_set_wp(&mpc56xx->jtag, wp_num, watchpoint->address);
	if (retval)
		return retval;

	LOG_DEBUG("wpid: %" PRIu32 ", wp_num %i bp_value 0x%" PRIx32 "",
			  watchpoint->unique_id,
			  wp_num, comparator_list[wp_num].bp_value);

	watchpoint->set = 20; /* Any nice value but 0 */

	return ERROR_OK;
}

static int mpc56xx_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	/* set any pending breakpoints */
	while (watchpoint) {
		if (watchpoint->set == 0)
			mpc56xx_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
	return ERROR_OK;
}

static int mpc56xx_unset_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	struct mpc5xxx_comparator *comparator_list = mpc56xx->data_break_list;

	if (!watchpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	int wp_num = watchpoint->set - 1;
	if ((wp_num < 0) || (wp_num >= mpc56xx->num_data_bpoints)) {
		LOG_DEBUG("Invalid FP Comparator number in breakpoint (wpid: %" PRIu32 ")",
				  watchpoint->unique_id);
		return ERROR_OK;
	}
	LOG_DEBUG("wpid: %" PRIu32 " - releasing hw: %d",
			watchpoint->unique_id,
			wp_num);
	comparator_list[wp_num].used = 0;
	comparator_list[wp_num].bp_value = 0;
		/*target_write_u32(target, comparator_list[bp_num].reg_address +*/
				/* ejtag_info->ejtag_ibc_offs, 0);*/

	watchpoint->set = 0;

	return ERROR_OK;
}


static int mpc56xx_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	if (mpc56xx->num_inst_bpoints_avail < 1) {
		LOG_INFO("no hardware watchpoints available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	mpc56xx->num_inst_bpoints_avail--;

	return ERROR_OK;
}

static int mpc56xx_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
		mpc56xx_unset_watchpoint(target, watchpoint);

	mpc56xx->num_inst_bpoints_avail++;

	return ERROR_OK;
}

static int mpc56xx_resume(struct target *target, int current,
	target_addr_t address, int handle_breakpoints, int debug_execution)
{
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("1. target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);

		retval = mpc56xx_jtag_clr_bps_wps(&mpc56xx->jtag);
		if (retval)
			return retval;

		retval = mpc56xx_enable_breakpoints(target);
		if (retval)
			return retval;

		retval = mpc56xx_enable_watchpoints(target);
		if (retval)
			return retval;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u32(mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].value, 0, 32, address);
		mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].dirty = 1;
		mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].valid = 1;
		resume_pc = address;
	} else {
		resume_pc = buf_get_u32(mpc56xx->core_cache->reg_list[MPC5XXX_REG_PC].value, 0, 32);
	}

	retval = mpc56xx_restore_context(target);
	if (retval)
		return retval;

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			printf("Not yet handling resuming on breakpoint\n");
#if 0
			LOG_DEBUG("unset breakpoint at 0x%8.8" PRIx32 "", breakpoint->address);
			mpc56xx_unset_breakpoint(target, breakpoint);
			mpc56xx_single_step_core(target);
			mpc56xx_set_breakpoint(target, breakpoint);
#endif
		}
	}
#if 0

	/* enable interrupts if we are running */
	mpc56xx_enable_interrupts(target, !debug_execution);
#endif
	/* exit debug mode */
	if (mpc56xx->msr_on_entry & MPC5XXX_MSR_EE) {
		printf("Re-enabling interrupts ? debug_execution = %d\n", debug_execution) ;
		mpc56xx_enable_interrupts(target, debug_execution ? 0 : 1) ;
	} else
		printf("Not Re-enabling interrupts ? debug_execution = %d\n", debug_execution) ;
	/* 1 was handle_breakpoints */
	retval = mpc56xx_exit_debug(&mpc56xx->jtag, resume_pc, 1, mpc56xx->ctl_on_entry);

	if (retval)
		return retval;

	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	register_cache_invalidate(mpc56xx->core_cache);

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

static int mpc56xx_target_create(struct target *target, Jim_Interp *interp)
{
	struct mpc5xxx_common *mpc56xx = calloc(1, sizeof(struct
			mpc5xxx_common));
	printf("target_create\n");
	mpc56xx->common_magic = MPC56XX_COMMON_MAGIC;
	target->arch_info = mpc56xx;

	mpc56xx->jtag.once = MPC56XX_TAP_ONCE; /* There is only one CPU */
	mpc56xx->jtag.current_tap = MPC5XXX_TAP_INVALID; /* Invalid to force TAP selection */
	mpc56xx->jtag.jtag_irlen = 5;

	return ERROR_OK;
}

/* read registers from the MCU and copy them to core_regs etc. */
/* this function disables interrupts if they are enabled ... */
/* result will however be reported correctly */
int mpc56xx_save_context(struct target *target)
{
	int retval, i;
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	/*printf("save_context\n");*/
	keep_alive();
	retval = mpc56xx_jtag_read_regs(&mpc56xx->jtag, mpc56xx->core_regs, &mpc56xx->saved_ctl);
	if (retval != ERROR_OK)
		return retval;

	mpc56xx->saved_msr = mpc56xx->core_regs[MPC5XXX_REG_MSR] ;

	/* now we saved the sate of the msr before we disabled interrupts,
	 * we can stop lying about disabled interrupts - otherwise
	 * restore_context will re-enable them
	 */

	mpc56xx->core_regs[MPC5XXX_REG_MSR] &= ~MPC5XXX_MSR_EE ;

	for (i = 0; i < MPC5XXX_NUMCOREREGS; i++) {
		if (!mpc56xx->core_cache->reg_list[i].valid)
			mpc5xxx_read_core_reg(target, i);
	}

	keep_alive();

	/*printf("end save context\n");*/
	keep_alive();
	return ERROR_OK;
}

/* restore registers and state to the MCU */
int mpc56xx_restore_context(struct target *target)
{
	int i, retval;
	struct mpc5xxx_cpuscr scr;

	/*printf("restore_context\n");*/

	/* get pointers to arch-specific information */
	struct mpc5xxx_common *mpc56xx = target_to_mpc5xxx(target);

	for (i = 0; i < MPC5XXX_NUMCOREREGS; i++) {
		if (mpc56xx->core_cache->reg_list[i].dirty)
			mpc5xxx_write_core_reg(target, i);
	}

	/* write core regs */
	mpc56xx_jtag_write_regs(&mpc56xx->jtag, mpc56xx->core_regs);

	/* Restore CTL */
	retval = mpc5xxx_once_cpuscr_read(&mpc56xx->jtag, &scr);
	if (retval)
		return retval;
	scr.ctl = mpc56xx->saved_ctl;
	return mpc5xxx_once_cpuscr_write(&mpc56xx->jtag, &scr);
}

struct target_type mpc56xx_target = {
	.name = "mpc56xx",

	.poll = mpc56xx_poll,
	.arch_state = mpc5xxx_arch_state,

	.halt = mpc56xx_halt,
	.resume = mpc56xx_resume,
	.step = mpc56xx_step,

	.assert_reset = mpc5xxx_assert_reset,
	.deassert_reset = mpc56xx_deassert_reset,

	.get_gdb_reg_list = mpc5xxx_get_gdb_reg_list,

	.read_memory = mpc5xxx_read_memory,
	.write_memory = mpc5xxx_write_memory,
	/* .checksum_memory = mpc56xx_checksum_memory, */
	/* .blank_check_memory = mpc56xx_blank_check_memory, */

	/* .run_algorithm = mpc56xx_run_algorithm, */

	.add_breakpoint = mpc56xx_add_breakpoint,
	.remove_breakpoint = mpc56xx_remove_breakpoint,
	.add_watchpoint = mpc56xx_add_watchpoint,
	.remove_watchpoint = mpc56xx_remove_watchpoint,

	.target_create = mpc56xx_target_create,
	.init_target = mpc5xxx_init_target,
	.examine = mpc5xxx_examine,
};
