/**************************************************************************
*   Copyright (C) 2018 by Antoine Calando                                 *
*   acalando@free.fr                                                      *
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
**************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "target.h"
#include "target_type.h"
#include "register.h"
#include "breakpoints.h"

#include "binarybuffer.h"
#include "jtag/jtag.h"
#include "jtag/interface.h"

#include "target/xpc56.h"



/* HW DESC */


#define xPC56_IR_LEN		5

#define xPC56_IR_NONE		0
#define xPC56_IR_IDCODE		1
#define xPC56_IR_CENSOR		7
/*
WARNING: ACCESS_AUX_TAP_xxx values
may differ depending on xPC56 subfamily
*/
#define xPC56_IR_TAP_FIRST	0x10
#define xPC56_IR_TAP_NPC	0x10
#define xPC56_IR_TAP_ONCE	0x11
#define xPC56_IR_TAP_TCU	0x1B
#define xPC56_IR_BYPASS		0x1F


#define E200_IR_LEN		10

/* not a real bit, used internally */
#define E200_IR_FORCE		BIT(31)
/* means read access! */
#define E200_IR_RW		BIT(9)
#define E200_IR_GO		BIT(8)
#define E200_IR_EX		BIT(7)
#define E200_IR_JTAG_ID		0x02
#define E200_IR_CPUSCR		0x10
#define E200_IR_NOREG		0x11
#define E200_IR_OCR		0x12
#define E200_IR_IAC1		0x20
#define E200_IR_IAC2		0x21
#define E200_IR_IAC3		0x22
#define E200_IR_IAC4		0x23
#define E200_IR_DAC1		0x24
#define E200_IR_DAC2		0x25
#define E200_IR_DBSR		0x30
#define E200_IR_DBCR0		0x31
#define E200_IR_DBCR1		0x32
#define E200_IR_DBCR2		0x33
#define E200_IR_SNCRRS		0x6f
#define E200_IR_GPRS0		0x70
#define E200_IR_NEXUS		0x7c
#define E200_IR_EN_ONCE		0x7e
#define E200_IR_BYPASS		0x7f
#define E200_IR_ALLSET		0x3ff



#define NEXUS_W		BIT(0)
#define NEXUS_DID	0x00
#define NEXUS_CSC	0x02
#define NEXUS_DC1	0x04
#define NEXUS_DC2	0x06
#define NEXUS_DS	0x08
#define NEXUS_RWCS	0x0E
#define NEXUS_RWA	0x12
#define NEXUS_RWD	0x14
#define NEXUS_WT	0x16
#define NEXUS_PCR	0xFE

/* opcodes for scan chain */
#define E200_OP_ORI(reg)	(0x1800D000 | ((reg)<<21) | ((reg)<<16))
#define E200_OP_SYNC()		0x7c0004ac
#define E200_OP_LWBRX(rt, rb)	(0x7c00042c | ((rt)<<21) | ((0)<<16) | ((rb)<<11))
#define E200_OP_LHBRX(rt, rb)	(0x7c00062c | ((rt)<<21) | ((0)<<16) | ((rb)<<11))
#define E200_OP_E_LBZ(rt)	(0x30000000 | ((rt)<<21) | ((0)<<16) | (0))
#define E200_OP_E_LHZ(rt)	(0x58000000 | ((rt)<<21) | ((0)<<16) | (0))
#define E200_OP_E_LWZ(rt)	(0x50000000 | ((rt)<<21) | ((0)<<16) | (0))

#define E200_OP_E_STB(rs)	(0x34000000 | ((rs)<<21) | ((0)<<16) | (0))
#define E200_OP_E_STH(rs)	(0x5C000000 | ((rs)<<21) | ((0)<<16) | (0))
#define E200_OP_E_STW(rs)	(0x54000000 | ((rs)<<21) | ((0)<<16) | (0))


static const struct  {
	char *name;
	uint32_t spr_idx;
} xpc56_reg_const[] = {
	{ "PC",	 0, },
	{ "CR",  0, },
	{ "CTR", 9, },
	{ "LR",  8, },
	{ "XER", 1, },
	{ "MSR", 0, },
};

#define REG_GP_CNT 32
#define REG_PC_IDX 32
#define REG_CR_IDX 33
#define REG_CTR_IDX 34
#define REG_LR_IDX 35
#define REG_XER_IDX 36
#define REG_MSR_IDX 37

#define REG_CNT (REG_GP_CNT + ARRAY_SIZE(xpc56_reg_const))


/* STRUCTS */

#define REG_SAVE_CNT 2
struct xpc56 {
	uint8_t		jtagc_ir;
	uint16_t	e200_ir;
	bool		debugging;
	/* Scan chain is { WBBR_low, WBBR_high, MSR, PC, IR, CTL } */
	uint32_t	cpuscr_save[6];

	bool		nexus2;
	uint32_t	xpc56_id;
	uint32_t	e200_id;

	uint32_t	bkpt_hw_mask;
};



/* DECL */
static void xpc56_jtagc_reset(struct target *target);
static uint32_t xpc56_cpuscr_reg(struct target *target, uint32_t reg, bool write, uint32_t val);

/* DEFS */

static struct xpc56 *xpc56_info(struct target *target)
{
	return (struct xpc56 *) target->arch_info;
}



int xpc56_get_core_reg(struct reg *reg)
{
	if (reg->number < REG_GP_CNT) {
		*(uint32_t *) (reg->value) = xpc56_cpuscr_reg(reg->arch_info, reg->number, false, 0);

		/*
		printf("RR %2d : %x\n", reg->number, *(uint32_t *) (reg->value));
		*/
		reg->valid = true;
		/*reg->dirty = false;*/

		return ERROR_OK;

	} else if (reg->number == REG_MSR_IDX || reg->number == REG_PC_IDX) {
		return ERROR_OK;

	} else {
		return ERROR_FAIL;
	}
}

int xpc56_set_core_reg(struct reg *reg, uint8_t *buf)
{
	if (buf) {
		*(uint32_t *) (reg->value) = *(uint32_t *) buf;
		reg->valid = true;
	}

	if (reg->number < REG_GP_CNT && reg->valid) {
		/*
		printf("RW %2d : %x\n", reg->number, *(uint32_t *) (reg->value));
		*/
		xpc56_cpuscr_reg(reg->arch_info, reg->number, true, *(uint32_t *) (reg->value));
		reg->dirty = false;

		return ERROR_OK;
	} else {
		printf("RW %2d : inv!!! %d %x\n", reg->number, reg->valid, *(uint32_t *) (reg->value));
		return ERROR_FAIL;
	}
}

static const struct reg_arch_type xpc56_reg_type = {
	.get = xpc56_get_core_reg,
	.set = xpc56_set_core_reg,
};


int xpc56_target_create(struct target *target, Jim_Interp *interp)
{
	struct xpc56 *info  = calloc(1, sizeof(struct xpc56));
	target->arch_info = info;

	#if 1
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	target->reg_cache = cache;


	cache->name = "PPC E200 registers";
	cache->next = NULL;
	cache->num_regs = REG_CNT;
	struct reg *reg_list = calloc(REG_CNT, sizeof(struct reg));
	uint32_t *reg_values = calloc(REG_CNT, sizeof(uint32_t));
	/* for string "GPR%d" */
	char *reg_names = calloc(REG_GP_CNT, 6);
	cache->reg_list = reg_list;
	/*
	struct xpc56_reg *reg_info = calloc(REG_CNT, sizeof(struct xpc56_reg));
	*/

	for (unsigned i = 0; i < REG_CNT; i++) {
		reg_list[i].number = i;
		reg_list[i].size = 32;
		reg_list[i].value = reg_values + i;
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].exist = true;
		reg_list[i].arch_info = target;
		reg_list[i].type = &xpc56_reg_type;
		reg_list[i].caller_save = true;
		reg_list[i].feature = NULL;
		reg_list[i].reg_data_type = NULL;
		reg_list[i].group = NULL;
		if (i < REG_GP_CNT) {
			/* TODO */
			snprintf(reg_names + 6*i, 6, "GPR%d", i);
			reg_list[i].name = reg_names + 6*i;
		} else {
			reg_list[i].name = xpc56_reg_const[i - REG_GP_CNT].name;
		}
	}
	#endif

	target->reg_cache->reg_list[REG_MSR_IDX].value = info->cpuscr_save + 2;
	target->reg_cache->reg_list[REG_PC_IDX].value = info->cpuscr_save + 3;

	return ERROR_OK;
}



int xpc56_init_target(struct command_context *cmd_ctx, struct target *target)
{
	struct xpc56 *info = target->arch_info;

	info->jtagc_ir = xPC56_IR_IDCODE;
	info->e200_ir = E200_IR_JTAG_ID;

	return ERROR_OK;
}

void xpc56_deinit_target(struct target *target)
{
	struct reg_cache *cache = target->reg_cache;

	if (!cache)
		return;
	free(cache->reg_list[0].value);
	free((void *)cache->reg_list[0].name);
	free(cache->reg_list);
	free(cache);
	target->reg_cache = NULL;
}

static void xpc56_jtagc_reset(struct target *target)
{
	jtag_add_statemove(TAP_DRPAUSE);
	jtag_add_statemove(TAP_IDLE);

	xpc56_info(target)->jtagc_ir = xPC56_IR_IDCODE;
}


static uint8_t xpc56_jtagc_ir_rw(struct target *target, uint8_t val)
{
	struct xpc56 *info = target->arch_info;

	if (info->jtagc_ir == val)
		return 0;

	if (xPC56_IR_TAP_FIRST <= info->jtagc_ir && info->jtagc_ir < xPC56_IR_BYPASS)
		xpc56_jtagc_reset(target);

	uint8_t	buf_out = 0;
	uint8_t	buf_in = 0;
	struct scan_field field = { 0 };

	buf_set_u32(&buf_out, 0, xPC56_IR_LEN, val);
	field.num_bits = xPC56_IR_LEN;
	field.out_value = &buf_out;
	field.in_value = &buf_in;
	jtag_add_ir_scan(target->tap, &field, TAP_IDLE);
	CHECK_RETVAL(jtag_execute_queue());
	info->jtagc_ir = val;

	return buf_in;
}

static uint16_t xpc56_e200_ir_rw(struct target *target, uint32_t val)
{
	/* check that we are in OnCE aux TAP */
	xpc56_jtagc_ir_rw(target, xPC56_IR_TAP_ONCE);

	/* Note that we can change aux TAP but HW will remember E200 IR */
	if (xpc56_info(target)->e200_ir == val)
		return 0;

	/* FORCE fake bit allows to prevent return above */
	val &= ~E200_IR_FORCE;

	uint8_t	buf_out[2] = { 0 };
	uint8_t	buf_in[2] = { 0 };
	struct scan_field field = { 0 };

	buf_set_u32(buf_out, 0, E200_IR_LEN, val);
	field.num_bits = E200_IR_LEN;
	field.out_value = buf_out;
	field.in_value = buf_in;
	jtag_add_ir_scan(target->tap, &field, TAP_IDLE);
	CHECK_RETVAL(jtag_execute_queue());

	xpc56_info(target)->e200_ir = val;

	return buf_get_u32(buf_in, 0, E200_IR_LEN);
}


static uint32_t xpc56_dr_rw(struct target *target, int size, uint64_t val)
{
	uint8_t	buf_out[8] = { 0 };
	uint8_t	buf_in[8] = { 0 };
	struct scan_field field = { 0 };

	buf_set_u64(buf_out, 0, size, val);
	field.num_bits = size;
	field.out_value = buf_out;
	field.in_value = buf_in;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	CHECK_RETVAL(jtag_execute_queue());

	return buf_get_u64(buf_in, 0, size);
}

static int xpc56_nexus_rw(struct target *target, uint8_t reg, uint8_t *buf_io, uint32_t val)
{
	xpc56_e200_ir_rw(target, E200_IR_NEXUS);

	uint8_t	buf_out[4] = { reg };
	struct scan_field field = { 8, buf_out, NULL, NULL, NULL };

	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	field.num_bits = 32;
	if (buf_io) {
		/* same buffer for in and out, looks like it is working */
		field.out_value = buf_io;
		field.in_value = buf_io;
	} else {
		h_u32_to_le(buf_out, val);
	}

	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	CHECK_RETVAL(jtag_execute_queue());

	return ERROR_OK;
}


static int xpc56_nexus_mem(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer, bool write)
{
	assert(size == 1 || size == 2 || size == 4);
	assert(count < BIT(14));

	uint32_t rwcs = BIT(31) | ((size/2)<<27) | BIT(23) | BIT(22) | (count<<2);
	uint32_t reg = NEXUS_RWD;
	if (write) {
		rwcs |= BIT(30);
		reg |= NEXUS_W;
	}

	xpc56_nexus_rw(target, NEXUS_RWA | NEXUS_W, NULL, address);
	xpc56_nexus_rw(target, NEXUS_RWCS | NEXUS_W, NULL, rwcs);

	for (uint32_t i = 0; i < count; i++) {
		xpc56_nexus_rw(target, reg, buffer, 0);
#if 1
		uint32_t val = 0;
		uint32_t st = 0;
		xpc56_nexus_rw(target, NEXUS_RWCS, (uint8_t *) &st, 0);
		if (!write) {
			switch (size) {
			case 1:
				val = buffer[0];
				break;
			case 2:
				val = be_to_h_u16(buffer);
				break;
			case 4:
				val = be_to_h_u32(buffer);
				break;
			}
		}
		printf("mem %08lx: %0*x  (%x)\n", address + i*size, 2*size, val, st&3);
#endif
		/* MUST WAIT FOR WRITE COMPLETION!!! */
		buffer += size;
	}

	return ERROR_OK;
}

/* data must pointer on 6 words for read and write */
static int xpc56_cpuscr(struct target *target, bool write, uint32_t *data)
{
	uint32_t reg = E200_IR_CPUSCR;
	if (!write) {
		memset(data, 0, 6*4);
		reg |= E200_IR_RW;
	}
	xpc56_e200_ir_rw(target, reg);
	struct scan_field field = { 32 * 6, (uint8_t *)data, (uint8_t *)data, NULL, NULL};
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	CHECK_RETVAL(jtag_execute_queue());

	return ERROR_OK;
}

static uint32_t xpc56_cpuscr_exec(struct target *target, uint32_t inst, bool ffra, uint32_t *wbbrl)
{
	/* Scan chain is { WBBR_low, WBBR_high, MSR, PC, IR, CTL }
	Other debuggers using PC 0xFFFFc000 (BAM ROM code) */
	uint32_t scan[6] = { wbbrl ? *wbbrl : 0, 0, 0, 0, inst, BIT(1) | (ffra ? BIT(10) : 0)};

	xpc56_cpuscr(target, true, scan);
	/* E200_IR_CPUSCR not working */
	xpc56_e200_ir_rw(target, E200_IR_GO | E200_IR_NOREG);
	xpc56_dr_rw(target, 1, 0);

	xpc56_cpuscr(target, false, scan);
	/*
	printf("cpuscr %08x %08x %08x %08x %08x %08x\n", scan[0], scan[1], scan[2], scan[3], scan[4], scan[5]);
	*/
	if (wbbrl)
		*wbbrl = scan[0];

	/* test PCINV in CTL */
	if (scan[5] & BIT(13))
		return ERROR_FAIL;
	else
		return ERROR_OK;
}

/* R or W GP registers */
static uint32_t xpc56_cpuscr_reg(struct target *target, uint32_t reg, bool write, uint32_t wval)
{
	uint32_t rwval = wval;
	xpc56_cpuscr_exec(target, E200_OP_ORI(reg), write, &rwval);
	target->reg_cache->reg_list[reg].dirty = true;

	return rwval;
}

static int xpc56_cpuscr_memr(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	assert(size == 1 || size == 2 || size == 4);
	/*
	We are retrieving and storing memory data according to the access size:
	so copying from target to local using 32 bits access will reverse bytes
	in memory, but each CPU reading its own memory via 32 bits access will
	provide same values.

	e.g. in xPC56 memory (8 bits dump, then 32 bits dump):  AA BB CC DD, AABBCCDD
	x86 memory after 32 bits read: DD CC BB AA, AABBCCDD
	x86 memory after 8 bits read: AA BB CC DD, DDCCBBAA

	Not sure if this is compliant with openocd. To change this behavior,
	we can use byte reversing load/store opcodes lhbrx/lwbrx
	*/
	uint32_t inst[] = {0, E200_OP_E_LBZ(1), E200_OP_E_LHZ(1), 0, E200_OP_E_LWZ(1)};
	target->reg_cache->reg_list[1].dirty = true;

	for (uint32_t i = 0; i < count; i++) {
		uint32_t data = address;
		if (xpc56_cpuscr_exec(target, inst[size], true, &data))
			return ERROR_FAIL;
		switch (size) {
		case 1:
			*buffer = (uint8_t) data;
			break;
		case 2:
			*(uint16_t *) buffer = (uint16_t) data;
			break;
		case 4:
			*(uint32_t *) buffer = data;
			break;
		}
		address += size;
		buffer += size;
	}

	/*
	printf(" ...read (if 4 1!) 0x%x\n", *(uint32_t *) (buffer-size));
	*/
	return ERROR_OK;
}

static int xpc56_cpuscr_memw(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	assert(size == 1 || size == 2 || size == 4);

	uint32_t inst[] = {0, E200_OP_E_STB(1), E200_OP_E_STH(1), 0, E200_OP_E_STW(1)};
	target->reg_cache->reg_list[1].dirty = true;
	uint32_t address2 = address;

	for (uint32_t i = 0; i < count; i++) {
		/* reading past the buffer but dont care */
		xpc56_cpuscr_reg(target, 1, true, *(uint32_t *)buffer);
		buffer += size;
		if (xpc56_cpuscr_exec(target, inst[size], true, &address2))
			return ERROR_FAIL;
		address2 += size;
	}
	return ERROR_OK;
}

static int xpc56_save_all(struct target *target)
{
	struct xpc56 *info = target->arch_info;

	xpc56_cpuscr(target, false, info->cpuscr_save);
	target->reg_cache->reg_list[REG_MSR_IDX].valid = true;
	target->reg_cache->reg_list[REG_PC_IDX].valid = true;

	/* memory barrier */
	xpc56_cpuscr_exec(target, E200_OP_SYNC(), false, NULL);

	for (int i = 0; i < REG_GP_CNT; i++)
		xpc56_get_core_reg(target->reg_cache->reg_list + i);

	/* we will use gpr0 as 0 and gpr1 as scratch for the debug session */
	xpc56_cpuscr_reg(target, 0, true, 0);
	/*
	printf("PC: %08x\n", info->cpuscr_save[3]);
	*/

	return ERROR_OK;
}

static int xpc56_restore_all(struct target *target)
{
	struct xpc56 *info = target->arch_info;

	for (int i = 0; i < 32; i++)
		if (target->reg_cache->reg_list[i].dirty)
			xpc56_set_core_reg(target->reg_cache->reg_list + i, NULL);

	/* execute a 'nop' before restoring scan chain */
	xpc56_cpuscr_exec(target, E200_OP_ORI(0), false, NULL);

	/* Clear breakpoints events in CTL */
	info->cpuscr_save[5] &= ~0xF0;
	/* PCOFST not handled yet */
	assert((info->cpuscr_save[5] & 0xF000) == 0);
	/* PCINV not handled yet */
	assert((info->cpuscr_save[5] & 0x800) == 0);

	xpc56_cpuscr(target, true, info->cpuscr_save);

	return ERROR_OK;
}

static int xpc56_debug_enter(struct target *target)
{
	printf("=== DBG ENTER ===\n");

	struct xpc56 *info = target->arch_info;
	assert(info->debugging == false);

	xpc56_e200_ir_rw(target, E200_IR_EN_ONCE);
	xpc56_e200_ir_rw(target, E200_IR_OCR);
	xpc56_dr_rw(target, 32, BIT(2) | BIT(1) | BIT(0));

	/* bug! discarding all breakpoints */
	xpc56_e200_ir_rw(target, E200_IR_DBCR0);
	xpc56_dr_rw(target, 32, BIT(31));

	xpc56_save_all(target);
	info->debugging = true;

	return ERROR_OK;
}

static int xpc56_debug_exit(struct target *target)
{
	struct xpc56 *info = target->arch_info;
	assert(info->debugging);

	xpc56_e200_ir_rw(target, E200_IR_OCR);
	xpc56_dr_rw(target, 32, 0);

	xpc56_restore_all(target);

	/* clear EDM in DCR0 (and clear DBSR first)?
	xpc56_e200_ir_rw(target, E200_IR_DBCR0);
	xpc56_dr_rw(target, 32, BIT(31));
	*/

	xpc56_e200_ir_rw(target, E200_IR_EX | E200_IR_GO | E200_IR_NOREG);
	xpc56_dr_rw(target, 1, 0);

	for (unsigned i = 0; i < REG_CNT; i++)
		target->reg_cache->reg_list[i].valid = false;

	info->debugging = false;
	printf("--- DBG EXIT ---\n");

	return ERROR_OK;
}

int xpc56_poll(struct target *target)
{
	struct xpc56 *info = target->arch_info;
	xpc56_jtagc_ir_rw(target, xPC56_IR_IDCODE);
	uint32_t id = xpc56_dr_rw(target, 64, 0);

	if (info->xpc56_id != id)
		return ERROR_FAIL;

	uint32_t osr = xpc56_e200_ir_rw(target, E200_IR_FORCE | E200_IR_RW | E200_IR_DBCR0);
	uint32_t dbcr0 = xpc56_dr_rw(target, 32, 0);
	xpc56_e200_ir_rw(target, E200_IR_RW | E200_IR_DBSR);
	uint32_t dbsr = xpc56_dr_rw(target, 32, 0);

	if (osr & BIT(3) && target->state != TARGET_HALTED) {

		target->state = TARGET_HALTED;
		info->debugging = true;
		xpc56_save_all(target);

		xpc56_e200_ir_rw(target, E200_IR_DBSR);
		xpc56_dr_rw(target, 32, dbsr);
	}

	if (target->state == TARGET_HALTED)
		printf("osr: %03x   dbcr0: %08x  dbsr: %08x  pc:%08x  ctl:%08x\n",
				osr, dbcr0, dbsr, info->cpuscr_save[3], info->cpuscr_save[5]);
	else
		printf("osr: %03x   dbcr0: %08x  dbsr: %08x\n", osr, dbcr0, dbsr);

	return ERROR_OK;
}

int xpc56_read_memory(struct target *target, target_addr_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	/*
	printf("Read mem %dx%d bytes at 0x%lx\n", size, count, address);
	*/

	struct xpc56 *info = target->arch_info;
	if (info->nexus2)
		return xpc56_nexus_mem(target, address, size, count, buffer, false);
	else
		return xpc56_cpuscr_memr(target, address, size, count, buffer);
}

uint32_t xpc56_read_u32(struct target *target, target_addr_t address)
{
	uint32_t buf[1];
	int rc = xpc56_read_memory(target, address, 4, 1, (uint8_t *) buf);
	assert(rc == ERROR_OK);
	return buf[0];
}

int xpc56_write_memory(struct target *target, target_addr_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	printf("Write mem %dx%d bytes at 0x%lx [%02x %02x %02x %02x...]\n", size,
			count, address, buffer[3], buffer[2], buffer[1], buffer[0]);

	struct xpc56 *info = target->arch_info;
	if (info->nexus2)
		return xpc56_nexus_mem(target, address, size, count, (uint8_t *)buffer, true);
	else
		return xpc56_cpuscr_memw(target, address, size, count, (uint8_t *)buffer);
}

void xpc56_write_u32(struct target *target, target_addr_t address, uint32_t value)
{
	int rc = xpc56_write_memory(target, address, 4, 1, (const uint8_t *)&value);
	assert(rc == ERROR_OK);
}



int xpc56_halt(struct target *target)
{
	struct xpc56 *info = target->arch_info;
	if (info->debugging == true) {
		printf("Already halted\n");
		return ERROR_FAIL;
	}
	xpc56_debug_enter(target);
	target->state = TARGET_HALTED;

	return ERROR_OK;
}

int xpc56_resume(struct target *target, int current, target_addr_t address,
	int handle_breakpoints, int debug_execution)
{
	struct xpc56 *info = target->arch_info;
	if (info->debugging == false) {
		printf("Already running\n");
		return ERROR_FAIL;
	}
	xpc56_debug_exit(target);
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

int xpc56_step(struct target *target, int current, target_addr_t address, int handle_breakpoints)
{
	struct xpc56 *info = target->arch_info;
	if (info->debugging == false) {
		printf("Already running\n");
		return ERROR_FAIL;
	}

	xpc56_restore_all(target);

	xpc56_e200_ir_rw(target, E200_IR_GO | E200_IR_NOREG);
	xpc56_dr_rw(target, 1, 0);

	xpc56_save_all(target);

	return ERROR_OK;
}


int xpc56_assert_reset(struct target *target)
{
	if (jtag_get_reset_config() & RESET_HAS_SRST)
		jtag_add_reset(0, 1);

	if (target->reset_halt)
		xpc56_halt(target);

	return ERROR_OK;
}

int xpc56_deassert_reset(struct target *target)
{
	if (jtag_get_reset_config() & RESET_HAS_SRST)
		jtag_add_reset(0, 0);
	return ERROR_OK;
}


int xpc56_examine(struct target *target)
{
	struct xpc56 *info = target->arch_info; (void) info;

	/* Force IR scan since oocd cannot start with DR scan */
	info->jtagc_ir = xPC56_IR_NONE;

	if (target_was_examined(target))
		return ERROR_OK;

	printf("========================== start exam ============================ %x\n", target->tap->bypass);

	uint32_t ir, dr; (void) ir, (void) dr;
#if 0
	/* Enter debug password; actually not tested with locked chip */
	xpc56_e200_ir_rw(target, E200_IR_EN_ONCE);

	xpc56_jtagc_ir_rw(target, xPC56_IR_CENSOR);
	uint8_t	buf_out[9] = { 0 };
	uint8_t	buf_in[9] = { 0 };
	struct scan_field field = { 65, buf_out, buf_in, NULL, NULL };
	buf_set_u64(buf_out, 0, 64, 0xcafebeeffeedfaceULL);
	buf_set_u64(buf_out, 64, 1, 1);
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	CHECK_RETVAL(jtag_execute_queue());
#endif

	dr = 0;
	xpc56_jtagc_ir_rw(target, xPC56_IR_IDCODE);
	uint32_t id = xpc56_dr_rw(target, 64, 0);
	if (id == 0xffffffff)
		return ERROR_FAIL;
	info->xpc56_id = id;
	/*
	printf("JTAG ID xPC56: %08x\n", info->xpc56_id);
	*/
	xpc56_e200_ir_rw(target, E200_IR_JTAG_ID);
	info->e200_id = xpc56_dr_rw(target, 32, 0);
	/*
	printf("JTAG ID e200: %08x\n", info->e200_id);
	*/


	xpc56_debug_enter(target);

#if 0
	/*
	actually not tested with nexus2 available
	*/
	uint32_t nexus_check = 0x12345678;
	xpc56_nexus_rw(target, NEXUS_RWA | NEXUS_W , (uint8_t *) &nexus_check, 0);
	nexus_check = 0;
	xpc56_nexus_rw(target, NEXUS_RWA , (uint8_t *) &nexus_check, 0);
	if (nexus_check == 0x12345678)
		info->nexus2 = true;
#endif
	/* backup cpuscr content
	xpc56_state_save(target);
	*/

#if 0
	xpc56_e200_ir_rw(target, E200_IR_RW | E200_IR_DBCR1);
	dr = xpc56_dr_rw(target, 32, 0);
	printf("DBCR1: %08x\n", dr);
	xpc56_e200_ir_rw(target, E200_IR_RW | E200_IR_DBCR2);
	dr = xpc56_dr_rw(target, 32, 0);
	printf("DBCR2: %08x\n", dr);
/*#endif
	for (int i = 0; i < 12; i++)
		xpc56_cpuscr_exec(target, E200_OP_ORI(i), false, NULL);
	*/
	printf("-------------------------------------\n");


	uint8_t buffer[256];
	xpc56_read_memory(target, 0xC3F90004, 4, 2, buffer);
	printf("MIDR8 %02x %02x %02x %02x  %02x %02x %02x %02x\n", buffer[0], buffer[1],
		buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);


	/*
	uint16_t bufh[32];
	xpc56_read_memory(target, 0xC3FD8000, 2, 2, (uint8_t *)bufh);
	printf("mem %04x %04x\n", bufh[0], bufh[1]);
	printf("----\n");
	*/

	uint32_t buf[32]; (void) buf;
	/*
	xpc56_read_memory(target, 0x40000000, 4, 8, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	printf("mem %08x %08x %08x %08x\n", buf[0+4], buf[1+4], buf[2+4], buf[3+4]);
	printf("----\n");
	*/

	xpc56_read_memory(target, 0x00000000, 4, 8, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	printf("mem %08x %08x %08x %08x\n", buf[0+4], buf[1+4], buf[2+4], buf[3+4]);
	printf("----\n");


/*#if 0 */
	printf("----\n");
	xpc56_read_memory(target, 0xC3F90004, 2, 4, buffer);
	printf("MIDR8 %02x %02x %02x %02x  %02x %02x %02x %02x\n", buffer[0], buffer[1],
		buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
	printf("----\n");
	xpc56_read_memory(target, 0xC3F90004, 1, 8, buffer);
	printf("MIDR8 %02x %02x %02x %02x  %02x %02x %02x %02x\n", buffer[0], buffer[1],
		buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
	printf("----\n");


	xpc56_read_memory(target, 0xC3F90014, 4, 2, buffer);
	printf("----\n");
	xpc56_read_memory(target, 0xC3F90028, 4, 3, buffer);
	printf("----\n");
	xpc56_read_memory(target, 0xC3F90040, 4, 8, buffer);
	printf("----\n");



	/*
	xpc56_read_memory(target, 0x203DD8, 4, 4, buffer);
	xpc56_read_memory(target, 0x203EE0, 4, 2, buffer);
	*/

	xpc56_read_memory(target, 0x3ffffff0, 4, 32, buffer);

	printf("----\n");
	xpc56_read_memory(target, 0x400007a0, 4, 8, buffer);
	printf("----\n");

	xpc56_read_memory(target, 0x0, 4, 16, buffer);
	printf("----\n");
#endif

	#if 0
	printf("CF MTR----\n");
	xpc56_read_memory(target, 0xC3F88000, 4, 4, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);


	buf[0] = 0xA1A11111;
	xpc56_write_memory(target, 0xC3F88004, 4, 1, (uint8_t *)buf);
	buf[0] = 0x00100000;
	xpc56_write_memory(target, 0xC3F88004, 4, 1, (uint8_t *)buf);

	buf[0] = 0xC3C33333;
	xpc56_write_memory(target, 0xC3F8800C, 4, 1, (uint8_t *)buf);
	buf[0] = 0x00100000;
	xpc56_write_memory(target, 0xC3F8800C, 4, 1, (uint8_t *)buf);


	xpc56_read_memory(target, 0xC3F88000, 4, 4, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	#endif

	#if 0
	buf[0] = 0x00000004;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);
	buf[0] = 0x00000001;
	xpc56_write_memory(target, 0xC3F88010, 4, 1, (uint8_t *)buf);
	xpc56_write_memory(target, 0x0, 4, 1, (uint8_t *)buf);
	buf[0] = 0x00000005;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);
	printf("erasing...\n");
	xpc56_read_memory(target, 0xC3F88000, 4, 4, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	xpc56_read_memory(target, 0xC3F88000, 4, 4, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	xpc56_read_memory(target, 0xC3F88000, 4, 4, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);

	buf[0] = 0x00000004;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);
	buf[0] = 0x00000000;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);

	xpc56_read_memory(target, 0xC3F88000, 4, 4, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	#endif

	#if 0
	buf[0] = 0x00000010;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);

	buf[0] = 0x01234567;
	xpc56_write_memory(target, 0x8, 4, 1, (uint8_t *)buf);
	buf[0] = 0x89abcdef;
	xpc56_write_memory(target, 0xC, 4, 1, (uint8_t *)buf);

	buf[0] = 0x00000011;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);

	for (int i = 0; i < 12; i++) {
		xpc56_read_memory(target, 0xC3F88000, 4, 4, (uint8_t *)buf);
		printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	}

	buf[0] = 0x00000010;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);
	buf[0] = 0x00000000;
	xpc56_write_memory(target, 0xC3F88000, 4, 1, (uint8_t *)buf);
	#endif



	/*
	xpc56_read_memory(target, 0x00000000, 4, 8, (uint8_t *)buf);
	printf("mem %08x %08x %08x %08x\n", buf[0], buf[1], buf[2], buf[3]);
	printf("mem %08x %08x %08x %08x\n", buf[0+4], buf[1+4], buf[2+4], buf[3+4]);
	printf("----\n");
	*/

	xpc56_debug_exit(target);

	ir = xpc56_e200_ir_rw(target, E200_IR_NOREG);
	printf("osr: %03x\n", ir);

	target_set_examined(target);

	return ERROR_OK;
}


static int xpc56_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct xpc56 *info = target->arch_info;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->type == BKPT_HARD) {
		if (info->bkpt_hw_mask == 0xF || (breakpoint->length & ~1) != 0) {
			LOG_INFO("no hardware breakpoint available/length not supported");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		int idx = 0;
		while (info->bkpt_hw_mask & BIT(idx))
			idx++;
		info->bkpt_hw_mask |= BIT(idx);

		assert(breakpoint->set == 0);
		breakpoint->set = idx + 1;

		xpc56_e200_ir_rw(target, E200_IR_IAC1 + idx);
		xpc56_dr_rw(target, 32, breakpoint->address);

		xpc56_e200_ir_rw(target, E200_IR_RW | E200_IR_DBCR0);
		uint32_t dbcr0 = xpc56_dr_rw(target, 32, 0);
		dbcr0 |= BIT(23 - idx);
		xpc56_e200_ir_rw(target, E200_IR_DBCR0);
		xpc56_dr_rw(target, 32, dbcr0);

		/* keep dbcr1/2 at 0 for now */
		/*
		printf("osr: %03x   dbcr0: %08x  dbsr: %08x\n", osr, dbcr0, dbsr);
		*/

	} else {
		LOG_WARNING("not implemented");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int xpc56_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	struct xpc56 *info = target->arch_info;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	assert(breakpoint->set);

	if (breakpoint->type == BKPT_HARD) {
		assert(info->bkpt_hw_mask & BIT(breakpoint->set - 1));
		info->bkpt_hw_mask &= BIT(breakpoint->set - 1) ;

		xpc56_e200_ir_rw(target, E200_IR_RW | E200_IR_DBCR0);
		uint32_t dbcr0 = xpc56_dr_rw(target, 32, 0);
		dbcr0 &= ~BIT(23 - breakpoint->set + 1);
		xpc56_e200_ir_rw(target, E200_IR_DBCR0);
		xpc56_dr_rw(target, 32, dbcr0);

	} else {
		LOG_WARNING("not implemented");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}



COMMAND_HANDLER(handle_xpc56_dummy_command)  { return ERROR_OK; }

static const struct command_registration xpc56_exec_command_handlers[] = {
	{
		.name = "dummy",
		.handler = handle_xpc56_dummy_command,
		.mode = COMMAND_EXEC,
		.help = "Dummy",
		.usage = "Dummy",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration xpc56_command_handlers[] = {
	{
		.name = "xpc56",
		.mode = COMMAND_ANY,
		.help = "xpc56 command group",
		.usage = "",
		.chain = xpc56_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


struct target_type xpc56_target = {
	.name = "xpc56",

	.poll = xpc56_poll,
	/*.arch_state = xpc56_arch_state,*/
	/*.target_request_data = xpc56_target_request_data,*/
	.halt = xpc56_halt,
	.resume = xpc56_resume,
	.step = xpc56_step,

	.assert_reset = xpc56_assert_reset,
	.deassert_reset = xpc56_deassert_reset,
	/*.soft_reset_halt = xpc56_soft_reset_halt,*/
	/*.get_gdb_reg_list = xpc56_get_gdb_reg_list,*/

	.read_memory = xpc56_read_memory,
	.write_memory = xpc56_write_memory,

	/*.checksum_memory = xpc56_checksum_memory,*/
	/*.blank_check_memory = xpc56_blank_check_memory,*/
	/*.run_algorithm = xpc56_run_algorithm,*/

	.add_breakpoint = xpc56_breakpoint_add,
	.remove_breakpoint = xpc56_breakpoint_remove,
	/*.add_watchpoint = xpc56_add_watchpoint,*/
	/*.remove_watchpoint = xpc56_remove_watchpoint,*/

	.commands = xpc56_command_handlers,
	.target_create = xpc56_target_create,
	.init_target = xpc56_init_target,
	.deinit_target = xpc56_deinit_target,
	.examine = xpc56_examine,
	/*.check_reset = xpc56_check_reset,*/
};
