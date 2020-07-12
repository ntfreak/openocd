/*
 * Copyright(c) 2013-2016 Intel Corporation.
 *
 * Adrian Burns (adrian.burns@intel.com)
 * Thomas Faust (thomas.faust@intel.com)
 * Ivan De Cesaris (ivan.de.cesaris@intel.com)
 * Julien Carreno (julien.carreno@intel.com)
 * Jeffrey Maxwell (jeffrey.r.maxwell@intel.com)
 * Jessica Gomez (jessica.gomez.hernandez@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information:
 * Intel Corporation
 */

/*
 * @file
 * This implements the probemode operations for Lakemont 1 (LMT1).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>

#include "target.h"
#include "target_type.h"
#include "lakemont.h"
#include "register.h"
#include "breakpoints.h"
#include "x86_32_common.h"

static int irscan(struct target *t, uint8_t *out,
			uint8_t *in, uint8_t ir_len);
static int drscan(struct target *t, uint8_t *out, uint8_t *in, uint8_t len);
static int save_context(struct target *target);
static int restore_context(struct target *target);
static uint32_t get_tapstatus(struct target *t);
static int enter_probemode(struct target *t);
static int exit_probemode(struct target *t);
static int halt_prep(struct target *t);
static int do_halt(struct target *t);
static int do_resume(struct target *t);
static int read_all_core_hw_regs(struct target *t);
static int write_all_core_hw_regs(struct target *t);
static int read_hw_reg(struct target *t,
			int reg, uint32_t *regval, uint8_t cache);
static int write_hw_reg(struct target *t,
			int reg, uint32_t regval, uint8_t cache);
static struct reg_cache *lakemont_build_reg_cache
			(struct target *target);
static int submit_reg_pir(struct target *t, int num);
static int submit_instruction_pir(struct target *t, int num);
static int submit_pir(struct target *t, uint64_t op);
static int lakemont_get_core_reg(struct reg *reg);
static int lakemont_set_core_reg(struct reg *reg, uint8_t *buf);

static struct scan_blk scan;

/* registers and opcodes for register access, pm_idx is used to identify the
 * registers that are modified for lakemont probemode specific operations
 */
static const struct {
	uint8_t id;
	const char *name;
	uint64_t op;
	uint8_t pm_idx;
	unsigned bits;
	enum reg_type type;
	const char *group;
	const char *feature;
} regs[] = {
	/* general purpose registers */
	{ EAX, "eax", 0x000000D01D660000, 0, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ECX, "ecx", 0x000000501D660000, 1, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ EDX, "edx", 0x000000901D660000, 2, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ EBX, "ebx", 0x000000101D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ESP, "esp", 0x000000E01D660000, NOT_PMREG, 32, REG_TYPE_DATA_PTR, "general", "org.gnu.gdb.i386.core" },
	{ EBP, "ebp", 0x000000601D660000, NOT_PMREG, 32, REG_TYPE_DATA_PTR, "general", "org.gnu.gdb.i386.core" },
	{ ESI, "esi", 0x000000A01D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ EDI, "edi", 0x000000201D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },

	/* instruction pointer & flags */
	{ EIP, "eip", 0x000000C01D660000, 3, 32, REG_TYPE_CODE_PTR, "general", "org.gnu.gdb.i386.core" },
	{ EFLAGS, "eflags", 0x000000401D660000, 4, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },

	/* segment registers */
	{ CS, "cs", 0x000000281D660000, 5, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ SS, "ss", 0x000000C81D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ DS, "ds", 0x000000481D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ES, "es", 0x000000A81D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FS, "fs", 0x000000881D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ GS, "gs", 0x000000081D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },

	/* floating point unit registers - not accessible via JTAG - here to satisfy GDB */
	{ ST0, "st0", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ST1, "st1", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ST2, "st2", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ST3, "st3", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ST4, "st4", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ST5, "st5", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ST6, "st6", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ ST7, "st7", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FCTRL, "fctrl", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FSTAT, "fstat", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FTAG, "ftag", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FISEG, "fiseg", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FIOFF, "fioff", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FOSEG, "foseg", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FOOFF, "fooff", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },
	{ FOP, "fop", 0x0, NOT_AVAIL_REG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.core" },

	/* control registers */
	{ CR0, "cr0", 0x000000001D660000, 6, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ CR2, "cr2", 0x000000BC1D660000, 7, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ CR3, "cr3", 0x000000801D660000, 8, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ CR4, "cr4", 0x0000002C1D660000, 9, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },

	/* debug registers */
	{ DR0, "dr0", 0x0000007C1D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DR1, "dr1", 0x000000FC1D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DR2, "dr2", 0x000000021D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DR3, "dr3", 0x000000821D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DR6, "dr6", 0x000000301D660000, 10, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DR7, "dr7", 0x000000B01D660000, 11, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },

	/* descriptor tables */
	{ IDTB, "idtbase", 0x000000581D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ IDTL, "idtlimit", 0x000000D81D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ IDTAR, "idtar", 0x000000981D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ GDTB, "gdtbase", 0x000000B81D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ GDTL, "gdtlimit", 0x000000781D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ GDTAR, "gdtar", 0x000000381D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ TR, "tr", 0x000000701D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ LDTR, "ldtr", 0x000000F01D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ LDTB, "ldbase", 0x000000041D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ LDTL, "ldlimit", 0x000000841D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ LDTAR, "ldtar", 0x000000F81D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },

	/* segment registers */
	{ CSB, "csbase", 0x000000F41D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ CSL, "cslimit", 0x0000000C1D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ CSAR, "csar", 0x000000741D660000, 12, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DSB, "dsbase", 0x000000941D660000, 13, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DSL, "dslimit", 0x000000541D660000, 14, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ DSAR, "dsar", 0x000000141D660000, 15, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ ESB, "esbase", 0x0000004C1D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ ESL, "eslimit", 0x000000CC1D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ ESAR, "esar", 0x0000008C1D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ FSB, "fsbase", 0x000000641D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ FSL, "fslimit", 0x000000E41D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ FSAR, "fsar", 0x000000A41D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ GSB, "gsbase", 0x000000C41D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ GSL, "gslimit", 0x000000241D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ GSAR, "gsar", 0x000000441D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ SSB, "ssbase", 0x000000341D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ SSL, "sslimit", 0x000000B41D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ SSAR, "ssar", 0x000000D41D660000, 16, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ TSSB, "tssbase", 0x000000E81D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ TSSL, "tsslimit", 0x000000181D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	{ TSSAR, "tssar", 0x000000681D660000, NOT_PMREG, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
	/* probemode control register */
	{ PMCR, "pmcr", 0x000000421D660000, 17, 32, REG_TYPE_INT32, "general", "org.gnu.gdb.i386.sys" },
};

static const struct {
	uint8_t id;
	const char *name;
	uint64_t op;
} instructions[] = {
	/* memory read/write */
	{ MEMRDB32, "MEMRDB32", 0x0909090909090851 },
	{ MEMRDB16, "MEMRDB16", 0x09090909090851E6 },
	{ MEMRDH32, "MEMRDH32", 0x090909090908D166 },
	{ MEMRDH16, "MEMRDH16", 0x090909090908D1E6 },
	{ MEMRDW32, "MEMRDW32", 0x09090909090908D1 },
	{ MEMRDW16, "MEMRDW16", 0x0909090908D1E666 },
	{ MEMWRB32, "MEMWRB32", 0x0909090909090811 },
	{ MEMWRB16, "MEMWRB16", 0x09090909090811E6 },
	{ MEMWRH32, "MEMWRH32", 0x0909090909089166 },
	{ MEMWRH16, "MEMWRH16", 0x09090909090891E6 },
	{ MEMWRW32, "MEMWRW32", 0x0909090909090891 },
	{ MEMWRW16, "MEMWRW16", 0x090909090891E666 },
	/* IO read/write */
	{ IORDB32, "IORDB32", 0x0909090909090937 },
	{ IORDB16, "IORDB16", 0x09090909090937E6 },
	{ IORDH32, "IORDH32", 0x090909090909B766 },
	{ IORDH16, "IORDH16", 0x090909090909B7E6 },
	{ IORDW32, "IORDW32", 0x09090909090909B7 },
	{ IORDW16, "IORDW16", 0x0909090909B7E666 },
	{ IOWRB32, "IOWRB32", 0x0909090909090977 },
	{ IOWRB16, "IOWRB16", 0x09090909090977E6 },
	{ IOWRH32, "IOWRH32", 0x090909090909F766 },
	{ IOWRH16, "IOWRH16", 0x090909090909F7E6 },
	{ IOWRW32, "IOWRW32", 0x09090909090909F7 },
	{ IOWRW16, "IOWRW16", 0x0909090909F7E666 },
	/* lakemont1 core shadow ram access opcodes */
	{ SRAMACCESS, "SRAMACCESS", 0x0000000E9D660000 },
	{ SRAM2PDR, "SRAM2PDR", 0x4CF0000000000000 },
	{ PDR2SRAM, "PDR2SRAM", 0x0CF0000000000000 },
	{ WBINVD, "WBINVD", 0x09090909090990F0 },
};

bool check_not_halted(const struct target *t)
{
	bool halted = t->state == TARGET_HALTED;
	if (!halted)
		LOG_ERROR("target running, halt it first");
	return !halted;
}

static int irscan(struct target *t, uint8_t *out,
			uint8_t *in, uint8_t ir_len)
{
	int retval = ERROR_OK;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	if (NULL == t->tap) {
		retval = ERROR_FAIL;
		LOG_ERROR("%s invalid target tap", __func__);
		return retval;
	}
	if (ir_len != t->tap->ir_length) {
		retval = ERROR_FAIL;
		if (t->tap->enabled)
			LOG_ERROR("%s tap enabled but tap irlen=%d",
					__func__, t->tap->ir_length);
		else
			LOG_ERROR("%s tap not enabled and irlen=%d",
					__func__, t->tap->ir_length);
		return retval;
	}
	struct scan_field *fields = &scan.field;
	fields->num_bits = ir_len;
	fields->out_value = out;
	fields->in_value = in;
	jtag_add_ir_scan(x86_32->curr_tap, fields, TAP_IDLE);
	if (x86_32->flush) {
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			LOG_ERROR("%s failed to execute queue", __func__);
	}
	return retval;
}

static int drscan(struct target *t, uint8_t *out, uint8_t *in, uint8_t len)
{
	int retval = ERROR_OK;
	uint64_t data = 0;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	if (NULL == t->tap) {
		retval = ERROR_FAIL;
		LOG_ERROR("%s invalid target tap", __func__);
		return retval;
	}
	if (len > MAX_SCAN_SIZE || 0 == len) {
		retval = ERROR_FAIL;
		LOG_ERROR("%s data len is %d bits, max is %d bits",
				__func__, len, MAX_SCAN_SIZE);
		return retval;
	}
	struct scan_field *fields = &scan.field;
	fields->out_value = out;
	fields->in_value = in;
	fields->num_bits = len;
	jtag_add_dr_scan(x86_32->curr_tap, 1, fields, TAP_IDLE);
	if (x86_32->flush) {
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("%s drscan failed to execute queue", __func__);
			return retval;
		}
	}
	if (in != NULL) {
		if (len >= 8) {
			for (int n = (len / 8) - 1 ; n >= 0; n--)
				data = (data << 8) + *(in+n);
		} else
			LOG_DEBUG("dr in 0x%02" PRIx8, *in);
	} else {
		LOG_ERROR("%s no drscan data", __func__);
		retval = ERROR_FAIL;
	}
	return retval;
}

static int save_context(struct target *t)
{
	int err;
	/* read core registers from lakemont sram */
	err = read_all_core_hw_regs(t);
	if (err != ERROR_OK) {
		LOG_ERROR("%s error reading regs", __func__);
		return err;
	}
	return ERROR_OK;
}

static int restore_context(struct target *t)
{
	int err = ERROR_OK;
	uint32_t i;
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	/* write core regs into the core PM SRAM from the reg_cache */
	err = write_all_core_hw_regs(t);
	if (err != ERROR_OK) {
		LOG_ERROR("%s error writing regs", __func__);
		return err;
	}

	for (i = 0; i < (x86_32->cache->num_regs); i++) {
		x86_32->cache->reg_list[i].dirty = false;
		x86_32->cache->reg_list[i].valid = false;
	}
	return err;
}

/*
 * we keep reg_cache in sync with hardware at halt/resume time, we avoid
 * writing to real hardware here because pm_regs reflects the hardware
 * while we are halted then reg_cache syncs with hw on resume
 * TODO - in order for "reg eip force" to work it assume get/set reads
 * and writes from hardware, may be other reasons also because generally
 * other openocd targets read/write from hardware in get/set - watch this!
 */
static int lakemont_get_core_reg(struct reg *reg)
{
	int retval = ERROR_OK;
	struct lakemont_core_reg *lakemont_reg = reg->arch_info;
	struct target *t = lakemont_reg->target;
	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	LOG_DEBUG("reg=%s, value=0x%08" PRIx32, reg->name,
			buf_get_u32(reg->value, 0, 32));
	return retval;
}

static int lakemont_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct lakemont_core_reg *lakemont_reg = reg->arch_info;
	struct target *t = lakemont_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);
	LOG_DEBUG("reg=%s, newval=0x%08" PRIx32, reg->name, value);
	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = true;
	reg->valid = true;
	return ERROR_OK;
}

static const struct reg_arch_type lakemont_reg_type = {
	/* these get called if reg_cache doesn't have a "valid" value
	 * of an individual reg eg "reg eip" but not for "reg" block
	 */
	.get = lakemont_get_core_reg,
	.set = lakemont_set_core_reg,
};

struct reg_cache *lakemont_build_reg_cache(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	int num_regs = ARRAY_SIZE(regs);
	struct reg_cache **cache_p = register_get_last_cache_p(&t->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct lakemont_core_reg *arch_info = malloc(sizeof(struct lakemont_core_reg) * num_regs);
	struct reg_feature *feature;
	int i;

	if (cache == NULL || reg_list == NULL || arch_info == NULL) {
		free(cache);
		free(reg_list);
		free(arch_info);
		LOG_ERROR("%s out of memory", __func__);
		return NULL;
	}

	/* Build the process context cache */
	cache->name = "lakemont registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	x86_32->cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].target = t;
		arch_info[i].x86_32_common = x86_32;
		arch_info[i].op = regs[i].op;
		arch_info[i].pm_idx = regs[i].pm_idx;
		reg_list[i].name = regs[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].type = &lakemont_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].group = regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("%s unable to allocate feature list", __func__);

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = regs[i].type;
		else
			LOG_ERROR("%s unable to allocate reg type list", __func__);
	}
	return cache;
}

static uint32_t get_tapstatus(struct target *t)
{
	scan.out[0] = TAPSTATUS;
	if (irscan(t, scan.out, NULL, LMT_IRLEN) != ERROR_OK)
		return 0;
	if (drscan(t, NULL, scan.out, TS_SIZE) != ERROR_OK)
		return 0;
	return buf_get_u32(scan.out, 0, 32);
}

static int enter_probemode(struct target *t)
{
	uint32_t tapstatus = 0;
	int retries = 100;

	tapstatus = get_tapstatus(t);
	LOG_DEBUG("TS before PM enter = 0x%08" PRIx32, tapstatus);
	if (tapstatus & TS_PM_BIT) {
		LOG_DEBUG("core already in probemode");
		return ERROR_OK;
	}
	scan.out[0] = PROBEMODE;
	if (irscan(t, scan.out, NULL, LMT_IRLEN) != ERROR_OK)
		return ERROR_FAIL;
	scan.out[0] = 1;
	if (drscan(t, scan.out, scan.in, 1) != ERROR_OK)
		return ERROR_FAIL;

	while (retries--) {
		tapstatus = get_tapstatus(t);
		LOG_DEBUG("TS after PM enter = 0x%08" PRIx32, tapstatus);
		if ((tapstatus & TS_PM_BIT) && (!(tapstatus & TS_EN_PM_BIT)))
			return ERROR_OK;
	}

	LOG_ERROR("%s PM enter error, tapstatus = 0x%08" PRIx32
			, __func__, tapstatus);
	return ERROR_FAIL;
}

static int exit_probemode(struct target *t)
{
	uint32_t tapstatus = get_tapstatus(t);
	LOG_DEBUG("TS before PM exit = 0x%08" PRIx32, tapstatus);

	if (!(tapstatus & TS_PM_BIT)) {
		LOG_USER("core not in PM");
		return ERROR_OK;
	}
	scan.out[0] = PROBEMODE;
	if (irscan(t, scan.out, NULL, LMT_IRLEN) != ERROR_OK)
		return ERROR_FAIL;
	scan.out[0] = 0;
	if (drscan(t, scan.out, scan.in, 1) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

/* do whats needed to properly enter probemode for debug on lakemont */
static int halt_prep(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	if (write_hw_reg(t, DSB, PM_DSB, 0) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("write %s 0x%08" PRIx32, regs[DSB].name, PM_DSB);
	if (write_hw_reg(t, DSL, PM_DSL, 0) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("write %s 0x%08" PRIx32, regs[DSL].name, PM_DSL);
	if (write_hw_reg(t, DSAR, PM_DSAR, 0) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("write DSAR 0x%08" PRIx32, PM_DSAR);
	if (write_hw_reg(t, CSB, PM_DSB, 0) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("write %s 0x%08" PRIx32, regs[CSB].name, PM_DSB);
	if (write_hw_reg(t, CSL, PM_DSL, 0) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("write %s 0x%08" PRIx32, regs[CSL].name, PM_DSL);
	if (write_hw_reg(t, DR7, PM_DR7, 0) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("write DR7 0x%08" PRIx32, PM_DR7);

	uint32_t eflags = buf_get_u32(x86_32->cache->reg_list[EFLAGS].value, 0, 32);
	uint32_t csar = buf_get_u32(x86_32->cache->reg_list[CSAR].value, 0, 32);
	uint32_t ssar = buf_get_u32(x86_32->cache->reg_list[SSAR].value, 0, 32);
	uint32_t cr0 = buf_get_u32(x86_32->cache->reg_list[CR0].value, 0, 32);

	/* clear VM86 and IF bits if they are set */
	LOG_DEBUG("EFLAGS = 0x%08" PRIx32 ", VM86 = %d, IF = %d", eflags,
			eflags & EFLAGS_VM86 ? 1 : 0,
			eflags & EFLAGS_IF ? 1 : 0);
	if ((eflags & EFLAGS_VM86) || (eflags & EFLAGS_IF)) {
		x86_32->pm_regs[I(EFLAGS)] = eflags & ~(EFLAGS_VM86 | EFLAGS_IF);
		if (write_hw_reg(t, EFLAGS, x86_32->pm_regs[I(EFLAGS)], 0) != ERROR_OK)
			return ERROR_FAIL;
		LOG_DEBUG("EFLAGS now = 0x%08" PRIx32 ", VM86 = %d, IF = %d",
				x86_32->pm_regs[I(EFLAGS)],
				x86_32->pm_regs[I(EFLAGS)] & EFLAGS_VM86 ? 1 : 0,
				x86_32->pm_regs[I(EFLAGS)] & EFLAGS_IF ? 1 : 0);
	}

	/* set CPL to 0 for memory access */
	if (csar & CSAR_DPL) {
		x86_32->pm_regs[I(CSAR)] = csar & ~CSAR_DPL;
		if (write_hw_reg(t, CSAR, x86_32->pm_regs[I(CSAR)], 0) != ERROR_OK)
			return ERROR_FAIL;
		LOG_DEBUG("write CSAR_CPL to 0 0x%08" PRIx32, x86_32->pm_regs[I(CSAR)]);
	}
	if (ssar & SSAR_DPL) {
		x86_32->pm_regs[I(SSAR)] = ssar & ~SSAR_DPL;
		if (write_hw_reg(t, SSAR, x86_32->pm_regs[I(SSAR)], 0) != ERROR_OK)
			return ERROR_FAIL;
		LOG_DEBUG("write SSAR_CPL to 0 0x%08" PRIx32, x86_32->pm_regs[I(SSAR)]);
	}

	/* if cache's are enabled, disable and flush, depending on the core version */
	if (!(x86_32->core_type == LMT3_5) && !(cr0 & CR0_CD)) {
		LOG_DEBUG("caching enabled CR0 = 0x%08" PRIx32, cr0);
		if (cr0 & CR0_PG) {
			x86_32->pm_regs[I(CR0)] = cr0 & ~CR0_PG;
			if (write_hw_reg(t, CR0, x86_32->pm_regs[I(CR0)], 0) != ERROR_OK)
				return ERROR_FAIL;
			LOG_DEBUG("cleared paging CR0_PG = 0x%08" PRIx32, x86_32->pm_regs[I(CR0)]);
			/* submit wbinvd to flush cache */
			if (submit_reg_pir(t, WBINVD) != ERROR_OK)
				return ERROR_FAIL;
			x86_32->pm_regs[I(CR0)] =
				x86_32->pm_regs[I(CR0)] | (CR0_CD | CR0_NW | CR0_PG);
			if (write_hw_reg(t, CR0, x86_32->pm_regs[I(CR0)], 0) != ERROR_OK)
				return ERROR_FAIL;
			LOG_DEBUG("set CD, NW and PG, CR0 = 0x%08" PRIx32, x86_32->pm_regs[I(CR0)]);
		}
	}
	return ERROR_OK;
}

static int do_halt(struct target *t)
{
	/* needs proper handling later if doing a halt errors out */
	t->state = TARGET_DEBUG_RUNNING;
	if (enter_probemode(t) != ERROR_OK)
		return ERROR_FAIL;

	return lakemont_update_after_probemode_entry(t);
}

/* we need to expose the update to be able to complete the reset at SoC level */
int lakemont_update_after_probemode_entry(struct target *t)
{
	if (save_context(t) != ERROR_OK)
		return ERROR_FAIL;
	if (halt_prep(t) != ERROR_OK)
		return ERROR_FAIL;
	t->state = TARGET_HALTED;

	return target_call_event_callbacks(t, TARGET_EVENT_HALTED);
}

static int do_resume(struct target *t)
{
	/* needs proper handling later */
	t->state = TARGET_DEBUG_RUNNING;
	if (restore_context(t) != ERROR_OK)
		return ERROR_FAIL;
	if (exit_probemode(t) != ERROR_OK)
		return ERROR_FAIL;
	t->state = TARGET_RUNNING;

	t->debug_reason = DBG_REASON_NOTHALTED;
	LOG_USER("target running");

	return target_call_event_callbacks(t, TARGET_EVENT_RESUMED);
}

static int read_all_core_hw_regs(struct target *t)
{
	int err;
	uint32_t regval;
	unsigned i;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	for (i = 0; i < (x86_32->cache->num_regs); i++) {
		if (NOT_AVAIL_REG == regs[i].pm_idx)
			continue;
		err = read_hw_reg(t, regs[i].id, &regval, 1);
		if (err != ERROR_OK) {
			LOG_ERROR("%s error saving reg %s",
					__func__, x86_32->cache->reg_list[i].name);
			return err;
		}
	}
	LOG_DEBUG("read_all_core_hw_regs read %u registers ok", i);
	return ERROR_OK;
}

static int write_all_core_hw_regs(struct target *t)
{
	int err;
	unsigned i;
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	for (i = 0; i < (x86_32->cache->num_regs); i++) {
		if (NOT_AVAIL_REG == regs[i].pm_idx)
			continue;
		err = write_hw_reg(t, i, 0, 1);
		if (err != ERROR_OK) {
			LOG_ERROR("%s error restoring reg %s",
					__func__, x86_32->cache->reg_list[i].name);
			return err;
		}
	}
	LOG_DEBUG("write_all_core_hw_regs wrote %u registers ok", i);
	return ERROR_OK;
}

/* read reg from lakemont core shadow ram, update reg cache if needed */
static int read_hw_reg(struct target *t, int reg, uint32_t *regval, uint8_t cache)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct lakemont_core_reg *arch_info;
	arch_info = x86_32->cache->reg_list[reg].arch_info;
	x86_32->flush = 0; /* don't flush scans till we have a batch */
	if (submit_reg_pir(t, reg) != ERROR_OK)
		return ERROR_FAIL;
	if (submit_instruction_pir(t, SRAMACCESS) != ERROR_OK)
		return ERROR_FAIL;
	if (submit_instruction_pir(t, SRAM2PDR) != ERROR_OK)
		return ERROR_FAIL;
	x86_32->flush = 1;
	scan.out[0] = RDWRPDR;
	if (irscan(t, scan.out, NULL, LMT_IRLEN) != ERROR_OK)
		return ERROR_FAIL;
	if (drscan(t, NULL, scan.out, PDR_SIZE) != ERROR_OK)
		return ERROR_FAIL;

	jtag_add_sleep(DELAY_SUBMITPIR);
	*regval = buf_get_u32(scan.out, 0, 32);
	if (cache) {
		buf_set_u32(x86_32->cache->reg_list[reg].value, 0, 32, *regval);
		x86_32->cache->reg_list[reg].valid = true;
		x86_32->cache->reg_list[reg].dirty = false;
	}
	LOG_DEBUG("reg=%s, op=0x%016" PRIx64 ", val=0x%08" PRIx32,
			x86_32->cache->reg_list[reg].name,
			arch_info->op,
			*regval);
	return ERROR_OK;
}

/* write lakemont core shadow ram reg, update reg cache if needed */
static int write_hw_reg(struct target *t, int reg, uint32_t regval, uint8_t cache)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct lakemont_core_reg *arch_info;
	arch_info = x86_32->cache->reg_list[reg].arch_info;

	uint8_t reg_buf[4];
	if (cache)
		regval = buf_get_u32(x86_32->cache->reg_list[reg].value, 0, 32);
	buf_set_u32(reg_buf, 0, 32, regval);
	LOG_DEBUG("reg=%s, op=0x%016" PRIx64 ", val=0x%08" PRIx32,
			x86_32->cache->reg_list[reg].name,
			arch_info->op,
			regval);

	x86_32->flush = 0; /* don't flush scans till we have a batch */
	if (submit_reg_pir(t, reg) != ERROR_OK)
		return ERROR_FAIL;
	if (submit_instruction_pir(t, SRAMACCESS) != ERROR_OK)
		return ERROR_FAIL;
	scan.out[0] = RDWRPDR;
	if (irscan(t, scan.out, NULL, LMT_IRLEN) != ERROR_OK)
		return ERROR_FAIL;
	if (drscan(t, reg_buf, scan.out, PDR_SIZE) != ERROR_OK)
		return ERROR_FAIL;
	x86_32->flush = 1;
	if (submit_instruction_pir(t, PDR2SRAM) != ERROR_OK)
		return ERROR_FAIL;

	/* we are writing from the cache so ensure we reset flags */
	if (cache) {
		x86_32->cache->reg_list[reg].dirty = false;
		x86_32->cache->reg_list[reg].valid = false;
	}
	return ERROR_OK;
}

static bool is_paging_enabled(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	if (x86_32->pm_regs[I(CR0)] & CR0_PG)
		return true;
	else
		return false;
}

static uint8_t get_num_user_regs(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	return x86_32->cache->num_regs;
}
/* value of the CR0.PG (paging enabled) bit influences memory reads/writes */
static int disable_paging(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	x86_32->pm_regs[I(CR0)] = x86_32->pm_regs[I(CR0)] & ~CR0_PG;
	int err = x86_32->write_hw_reg(t, CR0, x86_32->pm_regs[I(CR0)], 0);
	if (err != ERROR_OK) {
		LOG_ERROR("%s error disabling paging", __func__);
		return err;
	}
	return err;
}

static int enable_paging(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	x86_32->pm_regs[I(CR0)] = (x86_32->pm_regs[I(CR0)] | CR0_PG);
	int err = x86_32->write_hw_reg(t, CR0, x86_32->pm_regs[I(CR0)], 0);
	if (err != ERROR_OK) {
		LOG_ERROR("%s error enabling paging", __func__);
		return err;
	}
	return err;
}

static bool sw_bpts_supported(struct target *t)
{
	uint32_t tapstatus = get_tapstatus(t);
	if (tapstatus & TS_SBP_BIT)
		return true;
	else
		return false;
}

static int transaction_status(struct target *t)
{
	uint32_t tapstatus = get_tapstatus(t);
	if ((TS_EN_PM_BIT | TS_PRDY_BIT) & tapstatus) {
		LOG_ERROR("%s transaction error tapstatus = 0x%08" PRIx32
				, __func__, tapstatus);
		return ERROR_FAIL;
	} else {
		return ERROR_OK;
	}
}

static int submit_instruction(struct target *t, int num)
{
	int err = submit_instruction_pir(t, num);
	if (err != ERROR_OK) {
		LOG_ERROR("%s error submitting pir", __func__);
		return err;
	}
	return err;
}

static int submit_reg_pir(struct target *t, int num)
{
	LOG_DEBUG("reg %s op=0x%016" PRIx64, regs[num].name, regs[num].op);
	int err = submit_pir(t, regs[num].op);
	if (err != ERROR_OK) {
		LOG_ERROR("%s error submitting pir", __func__);
		return err;
	}
	return err;
}

static int submit_instruction_pir(struct target *t, int num)
{
	LOG_DEBUG("%s op=0x%016" PRIx64, instructions[num].name,
			instructions[num].op);
	int err = submit_pir(t, instructions[num].op);
	if (err != ERROR_OK) {
		LOG_ERROR("%s error submitting pir", __func__);
		return err;
	}
	return err;
}

/*
 * PIR (Probe Mode Instruction Register), SUBMITPIR is an "IR only" TAP
 * command; there is no corresponding data register
 */
static int submit_pir(struct target *t, uint64_t op)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	uint8_t op_buf[8];
	buf_set_u64(op_buf, 0, 64, op);
	int flush = x86_32->flush;
	x86_32->flush = 0;
	scan.out[0] = WRPIR;
	if (irscan(t, scan.out, NULL, LMT_IRLEN) != ERROR_OK)
		return ERROR_FAIL;
	if (drscan(t, op_buf, scan.out, PIR_SIZE) != ERROR_OK)
		return ERROR_FAIL;
	scan.out[0] = SUBMITPIR;
	x86_32->flush = flush;
	if (irscan(t, scan.out, NULL, LMT_IRLEN) != ERROR_OK)
		return ERROR_FAIL;
	jtag_add_sleep(DELAY_SUBMITPIR);
	return ERROR_OK;
}

int lakemont_init_target(struct command_context *cmd_ctx, struct target *t)
{
	lakemont_build_reg_cache(t);
	t->state = TARGET_RUNNING;
	t->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

int lakemont_init_arch_info(struct target *t, struct x86_32_common *x86_32)
{
	x86_32->submit_instruction = submit_instruction;
	x86_32->transaction_status = transaction_status;
	x86_32->read_hw_reg = read_hw_reg;
	x86_32->write_hw_reg = write_hw_reg;
	x86_32->sw_bpts_supported = sw_bpts_supported;
	x86_32->get_num_user_regs = get_num_user_regs;
	x86_32->is_paging_enabled = is_paging_enabled;
	x86_32->disable_paging = disable_paging;
	x86_32->enable_paging = enable_paging;
	return ERROR_OK;
}

int lakemont_poll(struct target *t)
{
	/* LMT1 PMCR register currently allows code breakpoints, data breakpoints,
	 * single stepping and shutdowns to be redirected to PM but does not allow
	 * redirecting into PM as a result of SMM enter and SMM exit
	 */
	uint32_t ts = get_tapstatus(t);

	if (ts == 0xFFFFFFFF && t->state != TARGET_DEBUG_RUNNING) {
		/* something is wrong here */
		LOG_ERROR("tapstatus invalid - scan_chain serialization or locked JTAG access issues");
		/* TODO: Give a hint that unlocking is wrong or maybe a
		 * 'jtag arp_init' helps
		 */
		t->state = TARGET_DEBUG_RUNNING;
		return ERROR_OK;
	}

	if (t->state == TARGET_HALTED && (!(ts & TS_PM_BIT))) {
		LOG_INFO("target running for unknown reason");
		t->state = TARGET_RUNNING;
	}

	if (t->state == TARGET_RUNNING &&
		t->state != TARGET_DEBUG_RUNNING) {

		if ((ts & TS_PM_BIT) && (ts & TS_PMCR_BIT)) {

			LOG_DEBUG("redirect to PM, tapstatus=0x%08" PRIx32, get_tapstatus(t));

			t->state = TARGET_DEBUG_RUNNING;
			if (save_context(t) != ERROR_OK)
				return ERROR_FAIL;
			if (halt_prep(t) != ERROR_OK)
				return ERROR_FAIL;
			t->state = TARGET_HALTED;
			t->debug_reason = DBG_REASON_UNDEFINED;

			struct x86_32_common *x86_32 = target_to_x86_32(t);
			uint32_t eip = buf_get_u32(x86_32->cache->reg_list[EIP].value, 0, 32);
			uint32_t dr6 = buf_get_u32(x86_32->cache->reg_list[DR6].value, 0, 32);
			uint32_t hwbreakpoint = (uint32_t)-1;

			if (dr6 & DR6_BRKDETECT_0)
				hwbreakpoint = 0;
			if (dr6 & DR6_BRKDETECT_1)
				hwbreakpoint = 1;
			if (dr6 & DR6_BRKDETECT_2)
				hwbreakpoint = 2;
			if (dr6 & DR6_BRKDETECT_3)
				hwbreakpoint = 3;

			if (hwbreakpoint != (uint32_t)-1) {
				uint32_t dr7 = buf_get_u32(x86_32->cache->reg_list[DR7].value, 0, 32);
				uint32_t type = dr7 & (0x03 << (DR7_RW_SHIFT + hwbreakpoint*DR7_RW_LEN_SIZE));
				if (type == DR7_BP_EXECUTE) {
					LOG_USER("hit hardware breakpoint (hwreg=%" PRIu32 ") at 0x%08" PRIx32, hwbreakpoint, eip);
				} else {
					uint32_t address = 0;
					switch (hwbreakpoint) {
					default:
					case 0:
						address = buf_get_u32(x86_32->cache->reg_list[DR0].value, 0, 32);
					break;
					case 1:
						address = buf_get_u32(x86_32->cache->reg_list[DR1].value, 0, 32);
					break;
					case 2:
						address = buf_get_u32(x86_32->cache->reg_list[DR2].value, 0, 32);
					break;
					case 3:
						address = buf_get_u32(x86_32->cache->reg_list[DR3].value, 0, 32);
					break;
					}
					LOG_USER("hit '%s' watchpoint for 0x%08" PRIx32 " (hwreg=%" PRIu32 ") at 0x%08" PRIx32,
								type == DR7_BP_WRITE ? "write" : "access", address,
								hwbreakpoint, eip);
				}
				t->debug_reason = DBG_REASON_BREAKPOINT;
			} else {
				/* Check if the target hit a software breakpoint.
				 * ! Watch out: EIP is currently pointing after the breakpoint opcode
				 */
				struct breakpoint *bp = NULL;
				bp = breakpoint_find(t, eip-1);
				if (bp != NULL) {
					t->debug_reason = DBG_REASON_BREAKPOINT;
					if (bp->type == BKPT_SOFT) {
						/* The EIP is now pointing the next byte after the
						 * breakpoint instruction. This needs to be corrected.
						 */
						buf_set_u32(x86_32->cache->reg_list[EIP].value, 0, 32, eip-1);
						x86_32->cache->reg_list[EIP].dirty = true;
						x86_32->cache->reg_list[EIP].valid = true;
						LOG_USER("hit software breakpoint at 0x%08" PRIx32, eip-1);
					} else {
						/* it's not a hardware breakpoint (checked already in DR6 state)
						 * and it's also not a software breakpoint ...
						 */
						LOG_USER("hit unknown breakpoint at 0x%08" PRIx32, eip);
					}
				} else {

					/* There is also the case that we hit an breakpoint instruction,
					 * which was not set by us. This needs to be handled be the
					 * application that introduced the breakpoint.
					 */

					LOG_USER("unknown break reason at 0x%08" PRIx32, eip);
				}
			}

			return target_call_event_callbacks(t, TARGET_EVENT_HALTED);
		}
	}

	return ERROR_OK;
}

int lakemont_arch_state(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	LOG_USER("target halted due to %s at 0x%08" PRIx32 " in %s mode",
			debug_reason_name(t),
			buf_get_u32(x86_32->cache->reg_list[EIP].value, 0, 32),
			(buf_get_u32(x86_32->cache->reg_list[CR0].value, 0, 32) & CR0_PE) ? "protected" : "real");

	return ERROR_OK;
}

int lakemont_halt(struct target *t)
{
	if (t->state == TARGET_RUNNING) {
		t->debug_reason = DBG_REASON_DBGRQ;
		if (do_halt(t) != ERROR_OK)
			return ERROR_FAIL;
		return ERROR_OK;
	} else {
		LOG_ERROR("%s target not running", __func__);
		return ERROR_FAIL;
	}
}

int lakemont_resume(struct target *t, int current, target_addr_t address,
			int handle_breakpoints, int debug_execution)
{
	struct breakpoint *bp = NULL;
	struct x86_32_common *x86_32 = target_to_x86_32(t);

	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	/* TODO lakemont_enable_breakpoints(t); */
	if (t->state == TARGET_HALTED) {

		/* running away for a software breakpoint needs some special handling */
		uint32_t eip = buf_get_u32(x86_32->cache->reg_list[EIP].value, 0, 32);
		bp = breakpoint_find(t, eip);
		if (bp != NULL /*&& bp->type == BKPT_SOFT*/) {
			/* the step will step over the breakpoint */
			if (lakemont_step(t, 0, 0, 1) != ERROR_OK) {
				LOG_ERROR("%s stepping over a software breakpoint at 0x%08" PRIx32 " "
						"failed to resume the target", __func__, eip);
				return ERROR_FAIL;
			}
		}

		/* if breakpoints are enabled, we need to redirect these into probe mode */
		struct breakpoint *activeswbp = t->breakpoints;
		while (activeswbp != NULL && activeswbp->set == 0)
			activeswbp = activeswbp->next;
		struct watchpoint *activehwbp = t->watchpoints;
		while (activehwbp != NULL && activehwbp->set == 0)
			activehwbp = activehwbp->next;
		if (activeswbp != NULL || activehwbp != NULL)
			buf_set_u32(x86_32->cache->reg_list[PMCR].value, 0, 32, 1);
		if (do_resume(t) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		LOG_USER("target not halted");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int lakemont_step(struct target *t, int current,
			target_addr_t address, int handle_breakpoints)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	uint32_t eflags = buf_get_u32(x86_32->cache->reg_list[EFLAGS].value, 0, 32);
	uint32_t eip = buf_get_u32(x86_32->cache->reg_list[EIP].value, 0, 32);
	uint32_t pmcr = buf_get_u32(x86_32->cache->reg_list[PMCR].value, 0, 32);
	struct breakpoint *bp = NULL;
	int retval = ERROR_OK;
	uint32_t tapstatus = 0;

	if (check_not_halted(t))
		return ERROR_TARGET_NOT_HALTED;
	bp = breakpoint_find(t, eip);
	if (retval == ERROR_OK && bp != NULL/*&& bp->type == BKPT_SOFT*/) {
		/* TODO: This should only be done for software breakpoints.
		 * Stepping from hardware breakpoints should be possible with the resume flag
		 * Needs testing.
		 */
		retval = x86_32_common_remove_breakpoint(t, bp);
	}

	/* Set EFLAGS[TF] and PMCR[IR], exit pm and wait for PRDY# */
	LOG_DEBUG("modifying PMCR = 0x%08" PRIx32 " and EFLAGS = 0x%08" PRIx32, pmcr, eflags);
	eflags = eflags | (EFLAGS_TF | EFLAGS_RF);
	buf_set_u32(x86_32->cache->reg_list[EFLAGS].value, 0, 32, eflags);
	buf_set_u32(x86_32->cache->reg_list[PMCR].value, 0, 32, 1);
	LOG_DEBUG("EFLAGS [TF] [RF] bits set=0x%08" PRIx32 ", PMCR=0x%08" PRIx32 ", EIP=0x%08" PRIx32,
			eflags, pmcr, eip);

	tapstatus = get_tapstatus(t);

	t->debug_reason = DBG_REASON_SINGLESTEP;
	t->state = TARGET_DEBUG_RUNNING;
	if (restore_context(t) != ERROR_OK)
		return ERROR_FAIL;
	if (exit_probemode(t) != ERROR_OK)
		return ERROR_FAIL;

	target_call_event_callbacks(t, TARGET_EVENT_RESUMED);

	tapstatus = get_tapstatus(t);
	if (tapstatus & (TS_PM_BIT | TS_EN_PM_BIT | TS_PRDY_BIT | TS_PMCR_BIT)) {
		/* target has stopped */
		if (save_context(t) != ERROR_OK)
			return ERROR_FAIL;
		if (halt_prep(t) != ERROR_OK)
			return ERROR_FAIL;
		t->state = TARGET_HALTED;

		LOG_USER("step done from EIP 0x%08" PRIx32 " to 0x%08" PRIx32, eip,
				buf_get_u32(x86_32->cache->reg_list[EIP].value, 0, 32));
		target_call_event_callbacks(t, TARGET_EVENT_HALTED);
	} else {
		/* target didn't stop
		 * I hope the poll() will catch it, but the deleted breakpoint is gone
		 */
		LOG_ERROR("%s target didn't stop after executing a single step", __func__);
		t->state = TARGET_RUNNING;
		return ERROR_FAIL;
	}

	/* try to re-apply the breakpoint, even of step failed
	 * TODO: When a bp was set, we should try to stop the target - fix the return above
	 */
	if (bp != NULL/*&& bp->type == BKPT_SOFT*/) {
		/* TODO: This should only be done for software breakpoints.
		 * Stepping from hardware breakpoints should be possible with the resume flag
		 * Needs testing.
		 */
		retval = x86_32_common_add_breakpoint(t, bp);
	}

	return retval;
}

static int lakemont_reset_break(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	struct jtag_tap *saved_tap = x86_32->curr_tap;
	struct scan_field *fields = &scan.field;

	int retval = ERROR_OK;

	LOG_DEBUG("issuing port 0xcf9 reset");

	/* prepare resetbreak setting the proper bits in CLTAPC_CPU_VPREQ */
	x86_32->curr_tap = jtag_tap_by_position(1);
	if (x86_32->curr_tap == NULL) {
		x86_32->curr_tap = saved_tap;
		LOG_ERROR("%s could not select quark_x10xx.cltap", __func__);
		return ERROR_FAIL;
	}

	fields->in_value  = NULL;
	fields->num_bits  = 8;

	/* select CLTAPC_CPU_VPREQ instruction*/
	scan.out[0] = 0x51;
	fields->out_value = ((uint8_t *)scan.out);
	jtag_add_ir_scan(x86_32->curr_tap, fields, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		x86_32->curr_tap = saved_tap;
		LOG_ERROR("%s irscan failed to execute queue", __func__);
		return retval;
	}

	/* set enable_preq_on_reset & enable_preq_on_reset2 bits*/
	scan.out[0] = 0x06;
	fields->out_value  = ((uint8_t *)scan.out);
	jtag_add_dr_scan(x86_32->curr_tap, 1, fields, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("%s drscan failed to execute queue", __func__);
		x86_32->curr_tap = saved_tap;
		return retval;
	}

	/* restore current tap */
	x86_32->curr_tap = saved_tap;

	return ERROR_OK;
}

/*
 * If we ever get an adapter with support for PREQ# and PRDY#, we should
 * update this function to add support for using those two signals.
 *
 * Meanwhile, we're assuming that we only support reset break.
 */
int lakemont_reset_assert(struct target *t)
{
	struct x86_32_common *x86_32 = target_to_x86_32(t);
	/* write 0x6 to I/O port 0xcf9 to cause the reset */
	uint8_t cf9_reset_val = 0x6;
	int retval;

	LOG_DEBUG(" ");

	if (t->state != TARGET_HALTED) {
		LOG_DEBUG("target must be halted first");
		retval = lakemont_halt(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not halt target");
			return retval;
		}
		x86_32->forced_halt_for_reset = true;
	}

	if (t->reset_halt) {
		retval = lakemont_reset_break(t);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = x86_32_common_write_io(t, 0xcf9, BYTE, &cf9_reset_val);
	if (retval != ERROR_OK) {
		LOG_ERROR("could not write to port 0xcf9");
		return retval;
	}

	if (!t->reset_halt && x86_32->forced_halt_for_reset) {
		x86_32->forced_halt_for_reset = false;
		retval = lakemont_resume(t, true, 0x00, false, true);
		if (retval != ERROR_OK)
			return retval;
	}

	/* remove breakpoints and watchpoints */
	x86_32_common_reset_breakpoints_watchpoints(t);

	return ERROR_OK;
}

int lakemont_reset_deassert(struct target *t)
{
	int retval;

	LOG_DEBUG(" ");

	if (target_was_examined(t)) {
		retval = lakemont_poll(t);
		if (retval != ERROR_OK)
			return retval;
	}

	if (t->reset_halt) {
		/* entered PM after reset, update the state */
		retval = lakemont_update_after_probemode_entry(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not update state after probemode entry");
			return retval;
		}

		if (t->state != TARGET_HALTED) {
			LOG_WARNING("%s: ran after reset and before halt ...",
				target_name(t));
			if (target_was_examined(t)) {
				retval = target_halt(t);
				if (retval != ERROR_OK)
					return retval;
			} else {
				t->state = TARGET_UNKNOWN;
			}
		}
	}

	return ERROR_OK;
}
