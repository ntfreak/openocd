/*
 * Support for processors implementing MIPS64 instruction set
 *
 * Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>
 * Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>
 * Copyright (C) 2014 by Antony Pavlov <antonynpavlov@gmail.com>
 * Copyright (C) 2014 by Peter Mamonov <pmamonov@gmail.com>
 *
 * Based on the work of:
 *   Copyright (C) 2008 by Spencer Oliver
 *   Copyright (C) 2008 by David T.L. Wong
 *   Copyright (C) 2010 by Konstantin Kostyukhin, Nikolay Shmyrev
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mips64.h"

static const struct {
	unsigned id;
	const char *name;
	enum reg_type type;
	const char *group;
	const char *feature;
	int flag;
} mips64_regs[] = {
	{  0,  "r0", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  1,  "r1", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  2,  "r2", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  3,  "r3", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  4,  "r4", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  5,  "r5", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  6,  "r6", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  7,  "r7", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  8,  "r8", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  9,  "r9", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 10, "r10", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 11, "r11", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 12, "r12", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 13, "r13", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 14, "r14", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 15, "r15", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 16, "r16", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 17, "r17", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 18, "r18", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 19, "r19", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 20, "r20", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 21, "r21", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 22, "r22", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 23, "r23", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 24, "r24", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 25, "r25", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 26, "r26", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 27, "r27", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 28, "r28", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 29, "r29", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 30, "r30", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 31, "r31", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 32, "lo", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 33, "hi", REG_TYPE_UINT64, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ MIPS64_NUM_CORE_REGS + 0, "pc", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cpu", 0 },
	{ MIPS64_NUM_CORE_REGS + 1, "Random", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 2, "Entrylo_0", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 3, "Entrylo_1", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 4, "Context", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 5, "Pagemask", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 6, "Wired", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 7, "badvaddr", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 8, "Count", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 9, "EntryHi", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 10, "Compare", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 11, "status", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 12, "cause", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 13, "EPC", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 14, "PrID", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 15, "Config", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 16, "LLA", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 17, "WatchLo0", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 18, "WatchLo1", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 19, "WatchHi0", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 20, "WatchHi1", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 21, "Xcontext", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 22, "ChipMemCtrl", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 23, "Debug", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 24, "Perfcount, sel=0", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 25, "Perfcount, sel=1", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 26, "Perfcount, sel=2", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 27, "Perfcount, sel=3", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 28, "ECC", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 29, "CacheErr", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 30, "TagLo", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 31, "TagHi", REG_TYPE_UINT32, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 32, "DataHi", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_REGS + 33, "EEPC", REG_TYPE_UINT64, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 0,  "f0", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 1,  "f1", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 2,  "f2", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 3,  "f3", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 4, "f4", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 5,  "f5", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 6,  "f6", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 7,  "f7", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 8,  "f8", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 9,  "f9", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 10, "f10", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 11, "f11", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 12, "f12", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 13, "f13", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 14, "f14", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 15, "f15", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 16, "f16", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 17, "f17", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 18, "f18", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 19, "f19", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 20, "f20", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 21, "f21", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 22, "f22", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 23, "f23", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 24, "f24", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 25, "f25", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 26, "f26", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 27, "f27", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 28, "f28", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 29, "f29", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 30, "f30", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 31, "f31", REG_TYPE_IEEE_DOUBLE, NULL,
		 "org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 32, "fcsr", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 33, "fir", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 34, "fconfig", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 35, "fccr", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 36, "fexr", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },
	{ MIPS64_NUM_CORE_C0_REGS + 37, "fenr", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },
};

static int reg_type2size(enum reg_type type)
{
	switch (type) {
	case REG_TYPE_UINT32:
	case REG_TYPE_INT:
		return 32;
	case REG_TYPE_UINT64:
	case REG_TYPE_IEEE_DOUBLE:
		return 64;
	default:
		return 64;
	}
}

static int mips64_get_core_reg(struct reg *reg)
{
	int retval;
	struct mips64_core_reg *mips64_reg = reg->arch_info;
	struct target *target = mips64_reg->target;
	struct mips64_common *mips64_target = target->arch_info;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = mips64_target->read_core_reg(target, mips64_reg->num);

	return retval;
}

static int mips64_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct mips64_core_reg *mips64_reg = reg->arch_info;
	struct target *target = mips64_reg->target;
	uint64_t value = buf_get_u64(buf, 0, 64);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u64(reg->value, 0, 64, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

static int mips64_read_core_reg(struct target *target, int num)
{
	uint64_t reg_value;

	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;

	if ((num < 0) || (num >= MIPS64_NUM_REGS))
		return ERROR_COMMAND_ARGUMENT_INVALID;

	reg_value = mips64->core_regs[num];
	buf_set_u64(mips64->core_cache->reg_list[num].value, 0, 64, reg_value);
	mips64->core_cache->reg_list[num].valid = 1;
	mips64->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

static int mips64_write_core_reg(struct target *target, int num)
{
	uint64_t reg_value;

	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;

	if ((num < 0) || (num >= MIPS64_NUM_REGS))
		return ERROR_COMMAND_ARGUMENT_INVALID;

	reg_value = buf_get_u64(mips64->core_cache->reg_list[num].value, 0, 64);
	mips64->core_regs[num] = reg_value;
	LOG_DEBUG("write core reg %i value 0x%" PRIx64 "", num, reg_value);
	mips64->core_cache->reg_list[num].valid = 1;
	mips64->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

int mips64_invalidate_core_regs(struct target *target)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	unsigned int i;

	for (i = 0; i < mips64->core_cache->num_regs; i++) {
		mips64->core_cache->reg_list[i].valid = 0;
		mips64->core_cache->reg_list[i].dirty = 0;
	}

	return ERROR_OK;
}


int mips64_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
	int *reg_list_size, enum target_register_class reg_class)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	register int i;

	/* include floating point registers */
	*reg_list_size = MIPS64_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < MIPS64_NUM_REGS; i++)
		(*reg_list)[i] = &mips64->core_cache->reg_list[i];

	return ERROR_OK;
}

int mips64_save_context(struct target *target)
{
	int retval;
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;

	retval = mips64_pracc_read_regs(ejtag_info, mips64->core_regs);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned i = 0; i < MIPS64_NUM_REGS; i++)
			retval = mips64->read_core_reg(target, i);

	return retval;
}

int mips64_restore_context(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips64->ejtag_info;

	for (unsigned i = 0; i < MIPS64_NUM_REGS; i++) {
		if (mips64->core_cache->reg_list[i].dirty)
			mips64->write_core_reg(target, i);
	}

	return mips64_pracc_write_regs(ejtag_info, mips64->core_regs);
}

int mips64_arch_state(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	struct reg *pc = &mips64->core_cache->reg_list[MIPS64_PC];

	if (mips64->common_magic != MIPS64_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-MIPS64 target");
		exit(-1);
	}

	LOG_USER("target halted due to %s, pc: 0x%" PRIx64 "",
		 debug_reason_name(target), buf_get_u64(pc->value, 0, 64));

	return ERROR_OK;
}

static const struct reg_arch_type mips64_reg_type = {
	.get = mips64_get_core_reg,
	.set = mips64_set_core_reg,
};

int mips64_build_reg_cache(struct target *target)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	struct reg_cache **cache_p, *cache;
	struct mips64_core_reg *arch_info = NULL;
	struct reg *reg_list = NULL;
	unsigned i;

	cache = calloc(1, sizeof(*cache));
	if (!cache) {
		LOG_ERROR("unable to allocate cache");
		return ERROR_FAIL;
	}

	reg_list = calloc(MIPS64_NUM_REGS, sizeof(*reg_list));
	if (!reg_list) {
		LOG_ERROR("unable to allocate reg_list");
		goto alloc_fail;
	}

	arch_info = calloc(MIPS64_NUM_REGS, sizeof(*arch_info));
	if (!arch_info) {
		LOG_ERROR("unable to allocate arch_info");
		goto alloc_fail;
	}

	for (i = 0; i < MIPS64_NUM_REGS; i++) {
		struct mips64_core_reg *a = &arch_info[i];
		struct reg *r = &reg_list[i];

		r->arch_info = &arch_info[i];
		r->caller_save = true;	/* gdb defaults to true */
		r->exist = true;
		r->feature = &a->feature;
		r->feature->name = mips64_regs[i].feature;
		r->group = mips64_regs[i].group;
		r->name = mips64_regs[i].name;
		r->number = i;
		r->reg_data_type = &a->reg_data_type;
		r->reg_data_type->type = mips64_regs[i].type;
		r->size = reg_type2size(mips64_regs[i].type);
		r->type = &mips64_reg_type;
		r->value = &a->value[0];

		a->mips64_common = mips64;
		a->num = mips64_regs[i].id;
		a->target = target;
	}

	cache->name = "mips64 registers";
	cache->reg_list = reg_list;
	cache->num_regs = MIPS64_NUM_REGS;

	cache_p = register_get_last_cache_p(&target->reg_cache);
	(*cache_p) = cache;

	mips64->core_cache = cache;

	return ERROR_OK;

alloc_fail:
	free(cache);
	free(reg_list);
	free(arch_info);

	return ERROR_FAIL;
}

int mips64_init_arch_info(struct target *target, struct mips64_common *mips64,
			  struct jtag_tap *tap)
{
	mips64->bp_scanned = false;
	mips64->common_magic = MIPS64_COMMON_MAGIC;
	mips64->data_break_list = NULL;
	mips64->ejtag_info.tap = tap;
	mips64->fast_data_area = NULL;
	mips64->mips64mode32 = false;
	mips64->read_core_reg = mips64_read_core_reg;
	mips64->write_core_reg = mips64_write_core_reg;

	return ERROR_OK;
}

int mips64_run_algorithm(struct target *target, int num_mem_params,
			 struct mem_param *mem_params, int num_reg_params,
			 struct reg_param *reg_params, target_addr_t entry_point,
			 target_addr_t exit_point, int timeout_ms, void *arch_info)
{
	/* TODO */
	return ERROR_OK;
}

int mips64_examine(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;

	if (target_was_examined(target))
		return ERROR_OK;

	/* TODO: why we do not do mips64_configure_break_unit() here? */
	mips64->bp_scanned = false;
	mips64->num_data_bpoints = 0;
	mips64->num_data_bpoints_avail = 0;
	mips64->num_inst_bpoints = 0;
	mips64->num_inst_bpoints_avail = 0;

	target_set_examined(target);

	return ERROR_OK;
}

static int mips64_configure_i_break_unit(struct target *target)
{
	/* get pointers to arch-specific information */
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *ibl;
	uint64_t bpinfo;
	int retval;
	int i;

	/* get number of inst breakpoints */
	retval = target_read_u64(target, EJTAG64_V25_IBS, &bpinfo);
	if (retval != ERROR_OK)
		return retval;

	mips64->num_inst_bpoints = (bpinfo >> 24) & 0x0F;
	mips64->num_inst_bpoints_avail = mips64->num_inst_bpoints;
	ibl = calloc(mips64->num_inst_bpoints, sizeof(*ibl));
	if (!ibl) {
		LOG_ERROR("unable to allocate inst_break_list");
		return ERROR_FAIL;
	}

	for (i = 0; i < mips64->num_inst_bpoints; i++)
		ibl[i].reg_address = EJTAG64_V25_IBA0 + (0x100 * i);

	mips64->inst_break_list = ibl;
	/* clear IBIS reg */
	retval = target_write_u64(target, EJTAG64_V25_IBS, 0);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int mips64_configure_d_break_unit(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	struct mips64_comparator *dbl;
	uint64_t bpinfo;
	int retval;
	int i;

	/* get number of data breakpoints */
	retval = target_read_u64(target, EJTAG64_V25_DBS, &bpinfo);
	if (retval != ERROR_OK)
		return retval;

	mips64->num_data_bpoints = (bpinfo >> 24) & 0x0F;
	mips64->num_data_bpoints_avail = mips64->num_data_bpoints;

	dbl = calloc(mips64->num_data_bpoints, sizeof(*dbl));

	if (!dbl) {
		LOG_ERROR("unable to allocate data_break_list");
		return ERROR_FAIL;
	}

	for (i = 0; i < mips64->num_data_bpoints; i++)
		dbl[i].reg_address = EJTAG64_V25_DBA0 + (0x100 * i);

	mips64->data_break_list = dbl;

	/* clear DBIS reg */
	retval = target_write_u64(target, EJTAG64_V25_DBS, 0);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int mips64_configure_break_unit(struct target *target)
{
	struct mips64_common *mips64 = target->arch_info;
	uint64_t dcr;
	int retval;

	if (mips64->bp_scanned)
		return ERROR_OK;

	/* get info about breakpoint support */
	retval = target_read_u64(target, EJTAG64_DCR, &dcr);
	if (retval != ERROR_OK)
		return retval;

	if (dcr & EJTAG64_DCR_IB) {
		retval = mips64_configure_i_break_unit(target);
		if (retval != ERROR_OK)
			return retval;
	}

	if (dcr & EJTAG64_DCR_DB) {
		retval = mips64_configure_d_break_unit(target);
		if (retval != ERROR_OK)
			return retval;
	}

	LOG_DEBUG("DCR 0x%" PRIx64 " numinst %i numdata %i", dcr,
		  mips64->num_inst_bpoints, mips64->num_data_bpoints);

	mips64->bp_scanned = true;

	return ERROR_OK;
}

int mips64_enable_interrupts(struct target *target, bool enable)
{
	int retval;
	bool update = false;
	uint64_t dcr;

	/* read debug control register */
	retval = target_read_u64(target, EJTAG64_DCR, &dcr);
	if (retval != ERROR_OK)
		return retval;

	if (enable) {
		if (!(dcr & EJTAG64_DCR_INTE)) {
			/* enable interrupts */
			dcr |= EJTAG64_DCR_INTE;
			update = true;
		}
	} else {
		if (dcr & EJTAG64_DCR_INTE) {
			/* disable interrupts */
			dcr &= ~(uint64_t)EJTAG64_DCR_INTE;
			update = true;
		}
	}

	if (update) {
		retval = target_write_u64(target, EJTAG64_DCR, dcr);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}
