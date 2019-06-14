/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#include <helper/log.h>
#include <helper/binarybuffer.h>
#include "nds32.h"
#include "nds32_aice.h"
#include "nds32_tlb.h"
#include "nds32_disassembler.h"

const int NDS32_BREAK_16 = 0x00EA;      /* 0xEA00 */
const int NDS32_BREAK_32 = 0x0A000064;  /* 0x6400000A */

struct nds32_edm_operation nds32_edm_ops[NDS32_EDM_OPERATION_MAX_NUM];
uint32_t nds32_edm_ops_num;

const char *nds32_debug_type_name[11] = {
	"SOFTWARE BREAK",
	"SOFTWARE BREAK_16",
	"HARDWARE BREAKPOINT",
	"DATA ADDR WATCHPOINT PRECISE",
	"DATA VALUE WATCHPOINT PRECISE",
	"DATA VALUE WATCHPOINT IMPRECISE",
	"DEBUG INTERRUPT",
	"HARDWARE SINGLE STEP",
	"DATA ADDR WATCHPOINT NEXT PRECISE",
	"DATA VALUE WATCHPOINT NEXT PRECISE",
	"LOAD STORE GLOBAL STOP",
};

static const int NDS32_LM_SIZE_TABLE[16] = {
	4 * 1024,
	8 * 1024,
	16 * 1024,
	32 * 1024,
	64 * 1024,
	128 * 1024,
	256 * 1024,
	512 * 1024,
	1024 * 1024,
	1 * 1024,
	2 * 1024,
};

static const int NDS32_LINE_SIZE_TABLE[6] = {
	0,
	8,
	16,
	32,
	64,
	128,
};

static int nds32_get_core_reg(struct reg *reg)
{
	int retval;
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (reg->valid) {
		uint32_t val = buf_get_u32(reg_arch_info->value, 0, 32);
		LOG_DEBUG("reading register(cached) %" PRIi32 "(%s), value: 0x%8.8" PRIx32,
				reg_arch_info->num, reg->name, val);
		return ERROR_OK;
	}

	int mapped_regnum = nds32->register_map(nds32, reg_arch_info->num);

	if (reg_arch_info->enable == false) {
		buf_set_u32(reg_arch_info->value, 0, 32, NDS32_REGISTER_DISABLE);
		retval = ERROR_FAIL;
	} else {
		uint32_t val = 0;
		if ((nds32->fpu_enable == false)
				&& (NDS32_REG_TYPE_FPU == nds32_reg_type(mapped_regnum))) {
			retval = ERROR_OK;
		} else if ((nds32->audio_enable == false)
				&& (NDS32_REG_TYPE_AUMR == nds32_reg_type(mapped_regnum))) {
			retval = ERROR_OK;
		} else {
			retval = aice_read_register(aice, mapped_regnum, &val);
		}
		buf_set_u32(reg_arch_info->value, 0, 32, val);

		LOG_DEBUG("reading register %" PRIi32 "(%s), value: 0x%8.8" PRIx32,
				reg_arch_info->num, reg->name, val);
	}

	if (retval == ERROR_OK) {
		reg->valid = true;
		reg->dirty = false;
	}

	return retval;
}

static int nds32_get_core_reg_64(struct reg *reg)
{
	int retval;
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (reg->valid)
		return ERROR_OK;

	if (reg_arch_info->enable == false) {
		buf_set_u64(reg_arch_info->value, 0, 64, NDS32_REGISTER_DISABLE);
		retval = ERROR_FAIL;
	} else {
		uint64_t val = 0;
		if ((nds32->fpu_enable == false)
				&& ((FD0 <= reg_arch_info->num) && (reg_arch_info->num <= FD31))) {
			retval = ERROR_OK;
		} else {
			retval = aice_read_reg_64(aice, reg_arch_info->num, &val);
		}
		buf_set_u64(reg_arch_info->value, 0, 64, val);
	}

	if (retval == ERROR_OK) {
		reg->valid = true;
		reg->dirty = false;
	}

	return retval;
}

static int nds32_update_psw(struct nds32 *nds32)
{
	uint32_t value_ir0;
	struct aice_port_s *aice = target_to_aice(nds32->target);

	nds32_get_mapped_reg(nds32, IR0, &value_ir0);

	/* Save data memory endian */
	if ((value_ir0 >> 5) & 0x1) {
		nds32->data_endian = TARGET_BIG_ENDIAN;
		aice_set_data_endian(aice, AICE_BIG_ENDIAN);
	} else {
		nds32->data_endian = TARGET_LITTLE_ENDIAN;
		aice_set_data_endian(aice, AICE_LITTLE_ENDIAN);
	}

	/* Save translation status */
	nds32->memory.address_translation = ((value_ir0 >> 7) & 0x1) ? true : false;

	return ERROR_OK;
}

static int nds32_update_mmu_info(struct nds32 *nds32)
{
	uint32_t value;

	/* Update MMU control status */
	nds32_get_mapped_reg(nds32, MR0, &value);
	nds32->mmu_config.default_min_page_size = value & 0x1;
	nds32->mmu_config.multiple_page_size_in_use = (value >> 10) & 0x1;

	return ERROR_OK;
}

static int nds32_update_cache_info(struct nds32 *nds32)
{
	uint32_t value;

	if (ERROR_OK == nds32_get_mapped_reg(nds32, MR8, &value)) {
		if (value & 0x1)
			nds32->memory.icache.enable = true;
		else
			nds32->memory.icache.enable = false;

		if (value & 0x2)
			nds32->memory.dcache.enable = true;
		else
			nds32->memory.dcache.enable = false;
	} else {
		nds32->memory.icache.enable = false;
		nds32->memory.dcache.enable = false;
	}

	return ERROR_OK;
}

static int nds32_update_lm_info(struct nds32 *nds32)
{
	struct nds32_memory *memory = &(nds32->memory);
	uint32_t value_mr6;
	uint32_t value_mr7;

	nds32_get_mapped_reg(nds32, MR6, &value_mr6);
	if (value_mr6 & 0x1)
		memory->ilm_enable = true;
	else
		memory->ilm_enable = false;

	if (memory->ilm_align_ver == 0) { /* 1MB aligned */
		memory->ilm_start = value_mr6 & 0xFFF00000;
		memory->ilm_end = memory->ilm_start + memory->ilm_size;
	} else if (memory->ilm_align_ver == 1) { /* aligned to local memory size */
		memory->ilm_start = value_mr6 & 0xFFFFFC00;
		memory->ilm_end = memory->ilm_start + memory->ilm_size;
	} else {
		memory->ilm_start = -1;
		memory->ilm_end = -1;
	}

	nds32_get_mapped_reg(nds32, MR7, &value_mr7);
	if (value_mr7 & 0x1)
		memory->dlm_enable = true;
	else
		memory->dlm_enable = false;

	if (memory->dlm_align_ver == 0) { /* 1MB aligned */
		memory->dlm_start = value_mr7 & 0xFFF00000;
		memory->dlm_end = memory->dlm_start + memory->dlm_size;
	} else if (memory->dlm_align_ver == 1) { /* aligned to local memory size */
		memory->dlm_start = value_mr7 & 0xFFFFFC00;
		memory->dlm_end = memory->dlm_start + memory->dlm_size;
	} else {
		memory->dlm_start = -1;
		memory->dlm_end = -1;
	}

	return ERROR_OK;
}

/**
 * If fpu/audio is disabled, to access fpu/audio registers will cause
 * exceptions. So, we need to check if fpu/audio is enabled or not as
 * target is halted. If fpu/audio is disabled, as users access fpu/audio
 * registers, OpenOCD will return fake value 0 instead of accessing
 * registers through DIM.
 */
static int nds32_check_extension(struct nds32 *nds32)
{
	uint32_t value;

	nds32_get_mapped_reg(nds32, FUCPR, &value);
	if (value == NDS32_REGISTER_DISABLE) {
		nds32->fpu_enable = false;
		nds32->audio_enable = false;
		return ERROR_OK;
	}

	if (value & 0x1)
		nds32->fpu_enable = true;
	else
		nds32->fpu_enable = false;

	if (value & 0x80000000)
		nds32->audio_enable = true;
	else
		nds32->audio_enable = false;

	return ERROR_OK;
}

static int nds32_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int mapped_regnum = nds32->register_map(nds32, reg_arch_info->num);

	/* ignore values that will generate exception */
	if (nds32_reg_exception(mapped_regnum, value))
		return ERROR_OK;

	LOG_DEBUG("writing register %" PRIi32 "(%s) with value 0x%8.8" PRIx32,
			reg_arch_info->num, reg->name, value);

	if ((nds32->fpu_enable == false) &&
		(NDS32_REG_TYPE_FPU == nds32_reg_type(mapped_regnum))) {

		buf_set_u32(reg->value, 0, 32, 0);
	} else if ((nds32->audio_enable == false) &&
		(NDS32_REG_TYPE_AUMR == nds32_reg_type(mapped_regnum))) {

		buf_set_u32(reg->value, 0, 32, 0);
	} else {
		buf_set_u32(reg->value, 0, 32, value);
		uint32_t val = buf_get_u32(reg_arch_info->value, 0, 32);
		aice_write_register(aice, mapped_regnum, val);

		/* After set value to registers, read the value from target
		 * to avoid W1C inconsistency. */
		aice_read_register(aice, mapped_regnum, &val);
		buf_set_u32(reg_arch_info->value, 0, 32, val);
	}

	reg->valid = true;
	reg->dirty = false;

	/* update registers to take effect right now */
	if (IR0 == mapped_regnum) {
		nds32_update_psw(nds32);
	} else if (MR0 == mapped_regnum) {
		nds32_update_mmu_info(nds32);
	} else if ((MR6 == mapped_regnum) || (MR7 == mapped_regnum)) {
		/* update lm information */
		nds32_update_lm_info(nds32);
	} else if (MR8 == mapped_regnum) {
		nds32_update_cache_info(nds32);
	} else if (FUCPR == mapped_regnum) {
		/* update audio/fpu setting */
		nds32_check_extension(nds32);
	}

	return ERROR_OK;
}

static int nds32_set_core_reg_64(struct reg *reg, uint8_t *buf)
{
	struct nds32_reg *reg_arch_info = reg->arch_info;
	struct target *target = reg_arch_info->target;
	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t low_part = buf_get_u32(buf, 0, 32);
	uint32_t high_part = buf_get_u32(buf, 32, 32);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((nds32->fpu_enable == false) &&
		((FD0 <= reg_arch_info->num) && (reg_arch_info->num <= FD31))) {

		buf_set_u32(reg->value, 0, 32, 0);
		buf_set_u32(reg->value, 32, 32, 0);

		reg->valid = true;
		reg->dirty = false;
	} else {
		buf_set_u32(reg->value, 0, 32, low_part);
		buf_set_u32(reg->value, 32, 32, high_part);

		reg->valid = true;
		reg->dirty = true;
	}

	return ERROR_OK;
}

static const struct reg_arch_type nds32_reg_access_type = {
	.get = nds32_get_core_reg,
	.set = nds32_set_core_reg,
};

static const struct reg_arch_type nds32_reg_access_type_64 = {
	.get = nds32_get_core_reg_64,
	.set = nds32_set_core_reg_64,
};

static struct reg_cache *nds32_build_reg_cache(struct target *target,
		struct nds32 *nds32)
{
	struct reg_cache *cache = calloc(sizeof(struct reg_cache), 1);
	struct reg *reg_list = calloc(TOTAL_REG_NUM, sizeof(struct reg));
	struct nds32_reg *reg_arch_info = calloc(TOTAL_REG_NUM, sizeof(struct nds32_reg));
	int i;

	if (!cache || !reg_list || !reg_arch_info) {
		free(cache);
		free(reg_list);
		free(reg_arch_info);
		return NULL;
	}

	cache->name = "Andes registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = 0;

	for (i = 0; i < TOTAL_REG_NUM; i++) {
		reg_arch_info[i].num = i;
		reg_arch_info[i].target = target;
		reg_arch_info[i].nds32 = nds32;
		reg_arch_info[i].enable = false;

		reg_list[i].name = nds32_reg_simple_name(i);
		reg_list[i].number = reg_arch_info[i].num;
		reg_list[i].size = nds32_reg_size(i);
		reg_list[i].arch_info = &reg_arch_info[i];

		reg_list[i].reg_data_type = calloc(sizeof(struct reg_data_type), 1);

		if (FD0 <= reg_arch_info[i].num && reg_arch_info[i].num <= FD31) {
			reg_list[i].value = reg_arch_info[i].value;
			reg_list[i].type = &nds32_reg_access_type_64;

			reg_list[i].reg_data_type->type = REG_TYPE_IEEE_DOUBLE;
			reg_list[i].reg_data_type->id = "ieee_double";
			reg_list[i].group = "float";
		} else {
			reg_list[i].value = reg_arch_info[i].value;
			reg_list[i].type = &nds32_reg_access_type;
			reg_list[i].group = "general";

			if ((FS0 <= reg_arch_info[i].num) && (reg_arch_info[i].num <= FS31)) {
				reg_list[i].reg_data_type->type = REG_TYPE_IEEE_SINGLE;
				reg_list[i].reg_data_type->id = "ieee_single";
				reg_list[i].group = "float";
			} else if ((reg_arch_info[i].num == FPCSR) ||
				   (reg_arch_info[i].num == FPCFG)) {
				reg_list[i].group = "float";
			} else if ((reg_arch_info[i].num == R28) ||
				   (reg_arch_info[i].num == R29) ||
				   (reg_arch_info[i].num == R31)) {
				reg_list[i].reg_data_type->type = REG_TYPE_DATA_PTR;
				reg_list[i].reg_data_type->id = "data_ptr";
			} else if ((reg_arch_info[i].num == R30) ||
				   (reg_arch_info[i].num == PC)) {
				reg_list[i].reg_data_type->type = REG_TYPE_CODE_PTR;
				reg_list[i].reg_data_type->id = "code_ptr";
			} else {
				reg_list[i].reg_data_type->type = REG_TYPE_UINT32;
				reg_list[i].reg_data_type->id = "uint32";
			}
		}

		if (R16 <= reg_arch_info[i].num && reg_arch_info[i].num <= R25)
			reg_list[i].caller_save = true;
		else
			reg_list[i].caller_save = false;

		reg_list[i].feature = malloc(sizeof(struct reg_feature));

		if (R0 <= reg_arch_info[i].num && reg_arch_info[i].num <= IFC_LP)
			reg_list[i].feature->name = "org.gnu.gdb.nds32.core";
		else if (CR0 <= reg_arch_info[i].num && reg_arch_info[i].num <= SECUR0)
			reg_list[i].feature->name = "org.gnu.gdb.nds32.system";
		else if (D0L24 <= reg_arch_info[i].num && reg_arch_info[i].num <= CBE3)
			reg_list[i].feature->name = "org.gnu.gdb.nds32.audio";
		else if (FPCSR <= reg_arch_info[i].num && reg_arch_info[i].num <= FD31)
			reg_list[i].feature->name = "org.gnu.gdb.nds32.fpu";

		cache->num_regs++;
	}

	nds32->core_cache = cache;

	return cache;
}

static int nds32_reg_cache_init(struct target *target, struct nds32 *nds32)
{
	struct reg_cache *cache;

	cache = nds32_build_reg_cache(target, nds32);
	if (!cache)
		return ERROR_FAIL;

	*register_get_last_cache_p(&target->reg_cache) = cache;

	return ERROR_OK;
}

static struct reg *nds32_reg_current(struct nds32 *nds32, unsigned regnum)
{
	struct reg *r;

	r = nds32->core_cache->reg_list + regnum;

	return r;
}

int nds32_full_context(struct nds32 *nds32)
{
	uint32_t value, value_ir0;

	/* save $pc & $psw */
	nds32_get_mapped_reg(nds32, PC, &value);
	nds32_get_mapped_reg(nds32, IR0, &value_ir0);

	nds32_update_psw(nds32);
	nds32_update_mmu_info(nds32);
	nds32_update_cache_info(nds32);
	nds32_update_lm_info(nds32);

	nds32_check_extension(nds32);

	return ERROR_OK;
}

/* get register value internally */
int nds32_get_mapped_reg(struct nds32 *nds32, unsigned regnum, uint32_t *value)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *r;

	if (regnum > reg_cache->num_regs)
		return ERROR_FAIL;

	r = nds32_reg_current(nds32, regnum);

	if (ERROR_OK != r->type->get(r))
		return ERROR_FAIL;

	*value = buf_get_u32(r->value, 0, 32);

	return ERROR_OK;
}

/** set register internally */
int nds32_set_mapped_reg(struct nds32 *nds32, unsigned regnum, uint32_t value)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *r;
	uint8_t set_value[4];

	if (regnum > reg_cache->num_regs)
		return ERROR_FAIL;

	r = nds32_reg_current(nds32, regnum);

	buf_set_u32(set_value, 0, 32, value);

	return r->type->set(r, set_value);
}

/** get general register list */
static int nds32_get_general_reg_list(struct nds32 *nds32,
		struct reg **reg_list[], int *reg_list_size)
{
	struct reg *reg_current;
	int i;
	int current_idx;

	/** freed in gdb_server.c */
	*reg_list = malloc(sizeof(struct reg *) * (IFC_LP - R0 + 1));
	current_idx = 0;

	for (i = R0; i < IFC_LP + 1; i++) {
		reg_current = nds32_reg_current(nds32, i);
		if (((struct nds32_reg *)reg_current->arch_info)->enable) {
			(*reg_list)[current_idx] = reg_current;
			current_idx++;
		}
	}
	*reg_list_size = current_idx;

	return ERROR_OK;
}

/** get all register list */
static int nds32_get_all_reg_list(struct nds32 *nds32,
		struct reg **reg_list[], int *reg_list_size)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *reg_current;
	unsigned int i;

	*reg_list_size = reg_cache->num_regs;

	/** freed in gdb_server.c */
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < reg_cache->num_regs; i++) {
		reg_current = nds32_reg_current(nds32, i);
		reg_current->exist = ((struct nds32_reg *)
				reg_current->arch_info)->enable;
		(*reg_list)[i] = reg_current;
	}

	return ERROR_OK;
}

/** get all register list */
int nds32_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	struct nds32 *nds32 = target_to_nds32(target);

	switch (reg_class) {
		case REG_CLASS_ALL:
			return nds32_get_all_reg_list(nds32, reg_list, reg_list_size);
		case REG_CLASS_GENERAL:
			return nds32_get_general_reg_list(nds32, reg_list, reg_list_size);
		default:
			return ERROR_FAIL;
	}

	return ERROR_FAIL;
}

static int nds32_select_memory_mode(struct target *target, uint32_t address,
		uint32_t length, uint32_t *end_address)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32_memory *memory = &(nds32->memory);
	struct nds32_edm *edm = &(nds32->edm);
	uint32_t dlm_start, dlm_end;
	uint32_t ilm_start, ilm_end;
	uint32_t address_end = address + length;

	/* init end_address */
	*end_address = address_end;

	if (NDS_MEMORY_ACC_CPU == memory->access_channel)
		return ERROR_OK;

	if (edm->access_control == false) {
		LOG_DEBUG("EDM does not support ACC_CTL");
		return ERROR_OK;
	}

	if (edm->direct_access_local_memory == false) {
		LOG_DEBUG("EDM does not support DALM");
		aice_memory_mode(aice, NDS_MEMORY_SELECT_MEM);
		return ERROR_OK;
	}

	if (NDS_MEMORY_SELECT_AUTO != memory->mode) {
		LOG_DEBUG("Memory mode is not AUTO");
		return ERROR_OK;
	}

	/* set default mode */
	aice_memory_mode(aice, NDS_MEMORY_SELECT_MEM);

	if ((memory->ilm_base != 0) && (memory->ilm_enable == true)) {
		ilm_start = memory->ilm_start;
		ilm_end = memory->ilm_end;

		/* case 1, address < ilm_start */
		if (address < ilm_start) {
			if (ilm_start < address_end) {
				/* update end_address to split non-ILM from ILM */
				*end_address = ilm_start;
			}
			/* MEM mode */
			aice_memory_mode(aice, NDS_MEMORY_SELECT_MEM);
		} else if ((ilm_start <= address) && (address < ilm_end)) {
			/* case 2, ilm_start <= address < ilm_end */
			if (ilm_end < address_end) {
				/* update end_address to split non-ILM from ILM */
				*end_address = ilm_end;
			}
			/* ILM mode */
			aice_memory_mode(aice, NDS_MEMORY_SELECT_ILM);
		} else { /* case 3, ilm_end <= address */
			/* MEM mode */
			aice_memory_mode(aice, NDS_MEMORY_SELECT_MEM);
		}

		return ERROR_OK;
	} else {
		LOG_DEBUG("ILM is not enabled");
	}

	if ((memory->dlm_base != 0) && (memory->dlm_enable == true)) {
		dlm_start = memory->dlm_start;
		dlm_end = memory->dlm_end;

		/* case 1, address < dlm_start */
		if (address < dlm_start) {
			if (dlm_start < address_end) {
				/* update end_address to split non-DLM from DLM */
				*end_address = dlm_start;
			}
			/* MEM mode */
			aice_memory_mode(aice, NDS_MEMORY_SELECT_MEM);
		} else if ((dlm_start <= address) && (address < dlm_end)) {
			/* case 2, dlm_start <= address < dlm_end */
			if (dlm_end < address_end) {
				/* update end_address to split non-DLM from DLM */
				*end_address = dlm_end;
			}
			/* DLM mode */
			aice_memory_mode(aice, NDS_MEMORY_SELECT_DLM);
		} else { /* case 3, dlm_end <= address */
			/* MEM mode */
			aice_memory_mode(aice, NDS_MEMORY_SELECT_MEM);
		}

		return ERROR_OK;
	} else {
		LOG_DEBUG("DLM is not enabled");
	}

	return ERROR_OK;
}

int nds32_read_buffer(struct target *target, uint32_t address,
		uint32_t size, uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	if ((NDS_MEMORY_ACC_CPU == memory->access_channel) &&
			(target->state != TARGET_HALTED)) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("READ BUFFER: ADDR %08" PRIx32 "  SIZE %08" PRIx32,
			address,
			size);

	int retval = ERROR_OK;
	struct aice_port_s *aice = target_to_aice(target);
	uint32_t end_address;

	if (((address % 2) == 0) && (size == 2)) {
		nds32_select_memory_mode(target, address, 2, &end_address);
		return aice_read_mem_unit(aice, address, 2, 1, buffer);
	}

	/* handle unaligned head bytes */
	if (address % 4) {
		uint32_t unaligned = 4 - (address % 4);

		if (unaligned > size)
			unaligned = size;

		nds32_select_memory_mode(target, address, unaligned, &end_address);
		retval = aice_read_mem_unit(aice, address, 1, unaligned, buffer);
		if (retval != ERROR_OK)
			return retval;

		buffer += unaligned;
		address += unaligned;
		size -= unaligned;
	}

	/* handle aligned words */
	if (size >= 4) {
		int aligned = size - (size % 4);
		int read_len;

		do {
			nds32_select_memory_mode(target, address, aligned, &end_address);

			read_len = end_address - address;

			if (read_len > 8)
				retval = aice_read_mem_bulk(aice, address, read_len, buffer);
			else
				retval = aice_read_mem_unit(aice, address, 4, read_len / 4, buffer);

			if (retval != ERROR_OK)
				return retval;

			buffer += read_len;
			address += read_len;
			size -= read_len;
			aligned -= read_len;

		} while (aligned != 0);
	}

	/*prevent byte access when possible (avoid AHB access limitations in some cases)*/
	if (size >= 2) {
		int aligned = size - (size % 2);
		nds32_select_memory_mode(target, address, aligned, &end_address);
		retval = aice_read_mem_unit(aice, address, 2, aligned / 2, buffer);
		if (retval != ERROR_OK)
			return retval;

		buffer += aligned;
		address += aligned;
		size -= aligned;
	}
	/* handle tail writes of less than 4 bytes */
	if (size > 0) {
		nds32_select_memory_mode(target, address, size, &end_address);
		retval = aice_read_mem_unit(aice, address, 1, size, buffer);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int nds32_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct aice_port_s *aice = target_to_aice(target);

	return aice_read_mem_unit(aice, address, size, count, buffer);
}

int nds32_read_phys_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	enum nds_memory_access orig_channel;
	int result;

	/* switch to BUS access mode to skip MMU */
	orig_channel = memory->access_channel;
	memory->access_channel = NDS_MEMORY_ACC_BUS;
	aice_memory_access(aice, memory->access_channel);

	/* The input address is physical address.  No need to do address translation. */
	result = aice_read_mem_unit(aice, address, size, count, buffer);

	/* restore to origin access mode */
	memory->access_channel = orig_channel;
	aice_memory_access(aice, memory->access_channel);

	return result;
}

int nds32_write_buffer(struct target *target, uint32_t address,
		uint32_t size, const uint8_t *buffer)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);

	if ((NDS_MEMORY_ACC_CPU == memory->access_channel) &&
			(target->state != TARGET_HALTED)) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("WRITE BUFFER: ADDR %08" PRIx32 "  SIZE %08" PRIx32,
			address,
			size);

	struct aice_port_s *aice = target_to_aice(target);
	int retval = ERROR_OK;
	uint32_t end_address;

	if (((address % 2) == 0) && (size == 2)) {
		nds32_select_memory_mode(target, address, 2, &end_address);
		return aice_write_mem_unit(aice, address, 2, 1, buffer);
	}

	/* handle unaligned head bytes */
	if (address % 4) {
		uint32_t unaligned = 4 - (address % 4);

		if (unaligned > size)
			unaligned = size;

		nds32_select_memory_mode(target, address, unaligned, &end_address);
		retval = aice_write_mem_unit(aice, address, 1, unaligned, buffer);
		if (retval != ERROR_OK)
			return retval;

		buffer += unaligned;
		address += unaligned;
		size -= unaligned;
	}

	/* handle aligned words */
	if (size >= 4) {
		int aligned = size - (size % 4);
		int write_len;

		do {
			nds32_select_memory_mode(target, address, aligned, &end_address);

			write_len = end_address - address;
			if (write_len > 8)
				retval = aice_write_mem_bulk(aice, address, write_len, buffer);
			else
				retval = aice_write_mem_unit(aice, address, 4, write_len / 4, buffer);
			if (retval != ERROR_OK)
				return retval;

			buffer += write_len;
			address += write_len;
			size -= write_len;
			aligned -= write_len;

		} while (aligned != 0);
	}

	/* handle tail writes of less than 4 bytes */
	if (size > 0) {
		nds32_select_memory_mode(target, address, size, &end_address);
		retval = aice_write_mem_unit(aice, address, 1, size, buffer);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

int nds32_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct aice_port_s *aice = target_to_aice(target);

	return aice_write_mem_unit(aice, address, size, count, buffer);
}

int nds32_write_phys_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	enum nds_memory_access orig_channel;
	int result;

	/* switch to BUS access mode to skip MMU */
	orig_channel = memory->access_channel;
	memory->access_channel = NDS_MEMORY_ACC_BUS;
	aice_memory_access(aice, memory->access_channel);

	/* The input address is physical address.  No need to do address translation. */
	result = aice_write_mem_unit(aice, address, size, count, buffer);

	/* restore to origin access mode */
	memory->access_channel = orig_channel;
	aice_memory_access(aice, memory->access_channel);

	return result;
}

int nds32_mmu(struct target *target, int *enabled)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_INVALID;
	}

	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_memory *memory = &(nds32->memory);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);

	if ((mmu_config->memory_protection == 2) && (memory->address_translation == true))
		*enabled = 1;
	else
		*enabled = 0;

	return ERROR_OK;
}

int nds32_arch_state(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->common_magic != NDS32_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-Andes target");
		return ERROR_FAIL;
	}

	uint32_t value_pc, value_psw;

	nds32_get_mapped_reg(nds32, PC, &value_pc);
	nds32_get_mapped_reg(nds32, IR0, &value_psw);

	LOG_USER("target halted due to %s\n"
			"psw: 0x%8.8" PRIx32 " pc: 0x%8.8" PRIx32 "%s",
			debug_reason_name(target),
			value_psw,
			value_pc,
			nds32->virtual_hosting ? ", virtual hosting" : "");

	/* save pc value to pseudo register pc */
	struct reg *reg = register_get_by_name(target->reg_cache, "pc", 1);
	buf_set_u32(reg->value, 0, 32, value_pc);

	return ERROR_OK;
}

static void nds32_init_must_have_registers(struct nds32 *nds32)
{
	struct reg_cache *reg_cache = nds32->core_cache;

	/** MUST have general registers */
	((struct nds32_reg *)reg_cache->reg_list[R0].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R1].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R2].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R3].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R4].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R5].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R6].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R7].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R8].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R9].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R10].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R15].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R28].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R29].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R30].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[R31].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[PC].arch_info)->enable = true;

	/** MUST have configuration system registers */
	((struct nds32_reg *)reg_cache->reg_list[CR0].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR1].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR2].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR3].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[CR4].arch_info)->enable = true;

	/** MUST have interrupt system registers */
	((struct nds32_reg *)reg_cache->reg_list[IR0].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR1].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR3].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR4].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR6].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR9].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR11].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR14].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[IR15].arch_info)->enable = true;

	/** MUST have MMU system registers */
	((struct nds32_reg *)reg_cache->reg_list[MR0].arch_info)->enable = true;

	/** MUST have EDM system registers */
	((struct nds32_reg *)reg_cache->reg_list[DR40].arch_info)->enable = true;
	((struct nds32_reg *)reg_cache->reg_list[DR42].arch_info)->enable = true;
}

static int nds32_init_memory_config(struct nds32 *nds32)
{
	uint32_t value_cr1; /* ICM_CFG */
	uint32_t value_cr2; /* DCM_CFG */
	struct nds32_memory *memory = &(nds32->memory);

	/* read $cr1 to init instruction memory information */
	nds32_get_mapped_reg(nds32, CR1, &value_cr1);
	memory->icache.set = value_cr1 & 0x7;
	memory->icache.way = (value_cr1 >> 3) & 0x7;
	memory->icache.line_size = (value_cr1 >> 6) & 0x7;
	memory->icache.lock_support = (value_cr1 >> 9) & 0x1;

	memory->ilm_base = (value_cr1 >> 10) & 0x7;
	memory->ilm_align_ver = (value_cr1 >> 13) & 0x3;

	/* read $cr2 to init data memory information */
	nds32_get_mapped_reg(nds32, CR2, &value_cr2);
	memory->dcache.set = value_cr2 & 0x7;
	memory->dcache.way = (value_cr2 >> 3) & 0x7;
	memory->dcache.line_size = (value_cr2 >> 6) & 0x7;
	memory->dcache.lock_support = (value_cr2 >> 9) & 0x1;

	memory->dlm_base = (value_cr2 >> 10) & 0x7;
	memory->dlm_align_ver = (value_cr2 >> 13) & 0x3;

	return ERROR_OK;
}

static void nds32_init_config(struct nds32 *nds32)
{
	uint32_t value_cr0;
	uint32_t value_cr3;
	uint32_t value_cr4;
	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	struct nds32_misc_config *misc_config = &(nds32->misc_config);

	nds32_get_mapped_reg(nds32, CR0, &value_cr0);
	nds32_get_mapped_reg(nds32, CR3, &value_cr3);
	nds32_get_mapped_reg(nds32, CR4, &value_cr4);

	/* config cpu version */
	cpu_version->performance_extension = value_cr0 & 0x1;
	cpu_version->_16bit_extension = (value_cr0 >> 1) & 0x1;
	cpu_version->performance_extension_2 = (value_cr0 >> 2) & 0x1;
	cpu_version->cop_fpu_extension = (value_cr0 >> 3) & 0x1;
	cpu_version->string_extension = (value_cr0 >> 4) & 0x1;
	cpu_version->revision = (value_cr0 >> 16) & 0xFF;
	cpu_version->cpu_id_family = (value_cr0 >> 24) & 0xF;
	cpu_version->cpu_id_version = (value_cr0 >> 28) & 0xF;

	/* config MMU */
	mmu_config->memory_protection = value_cr3 & 0x3;
	mmu_config->memory_protection_version = (value_cr3 >> 2) & 0x1F;
	mmu_config->fully_associative_tlb = (value_cr3 >> 7) & 0x1;
	if (mmu_config->fully_associative_tlb) {
		mmu_config->tlb_size = (value_cr3 >> 8) & 0x7F;
	} else {
		mmu_config->tlb_ways = (value_cr3 >> 8) & 0x7;
		mmu_config->tlb_sets = (value_cr3 >> 11) & 0x7;
	}
	mmu_config->_8k_page_support = (value_cr3 >> 15) & 0x1;
	mmu_config->extra_page_size_support = (value_cr3 >> 16) & 0xFF;
	mmu_config->tlb_lock = (value_cr3 >> 24) & 0x1;
	mmu_config->hardware_page_table_walker = (value_cr3 >> 25) & 0x1;
	mmu_config->default_endian = (value_cr3 >> 26) & 0x1;
	mmu_config->partition_num = (value_cr3 >> 27) & 0x1;
	mmu_config->invisible_tlb = (value_cr3 >> 28) & 0x1;
	mmu_config->vlpt = (value_cr3 >> 29) & 0x1;
	mmu_config->ntme = (value_cr3 >> 30) & 0x1;
	mmu_config->drde = (value_cr3 >> 31) & 0x1;

	/* config misc */
	misc_config->edm = value_cr4 & 0x1;
	misc_config->local_memory_dma = (value_cr4 >> 1) & 0x1;
	misc_config->performance_monitor = (value_cr4 >> 2) & 0x1;
	misc_config->high_speed_memory_port = (value_cr4 >> 3) & 0x1;
	misc_config->debug_tracer = (value_cr4 >> 4) & 0x1;
	misc_config->div_instruction = (value_cr4 >> 5) & 0x1;
	misc_config->mac_instruction = (value_cr4 >> 6) & 0x1;
	misc_config->audio_isa = (value_cr4 >> 7) & 0x3;
	misc_config->L2_cache = (value_cr4 >> 9) & 0x1;
	misc_config->reduce_register = (value_cr4 >> 10) & 0x1;
	misc_config->addr_24 = (value_cr4 >> 11) & 0x1;
	misc_config->interruption_level = (value_cr4 >> 12) & 0x1;
	misc_config->baseline_instruction = (value_cr4 >> 13) & 0x7;
	misc_config->no_dx_register = (value_cr4 >> 16) & 0x1;
	misc_config->implement_dependant_register = (value_cr4 >> 17) & 0x1;
	misc_config->implement_dependant_sr_encoding = (value_cr4 >> 18) & 0x1;
	misc_config->ifc = (value_cr4 >> 19) & 0x1;
	misc_config->mcu = (value_cr4 >> 20) & 0x1;
	misc_config->shadow = (value_cr4 >> 21) & 0x7;
	misc_config->ex9 = (value_cr4 >> 24) & 0x1;

	nds32_init_memory_config(nds32);
}

static int nds32_init_option_registers(struct nds32 *nds32)
{
	struct reg_cache *reg_cache = nds32->core_cache;
	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);
	struct nds32_mmu_config *mmu_config = &(nds32->mmu_config);
	struct nds32_misc_config *misc_config = &(nds32->misc_config);
	struct nds32_memory *memory_config = &(nds32->memory);

	bool no_cr5;
	bool mr10_exist;
	bool no_racr0;

	if (((cpu_version->cpu_id_family == 0xC) || (cpu_version->cpu_id_family == 0xD)) &&
			((cpu_version->revision & 0xFC) == 0)) {
		no_cr5 = true;
		mr10_exist = true;
		no_racr0 = true;
	} else {
		no_cr5 = false;
		mr10_exist = false;
		no_racr0 = false;
	}

	if (misc_config->reduce_register == false) {
		((struct nds32_reg *)reg_cache->reg_list[R11].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R12].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R13].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R14].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R16].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R17].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R18].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R19].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R20].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R21].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R22].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R23].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R24].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R25].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R26].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[R27].arch_info)->enable = true;
	}

	if (misc_config->no_dx_register == false) {
		((struct nds32_reg *)reg_cache->reg_list[D0LO].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[D0HI].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[D1LO].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[D1HI].arch_info)->enable = true;
	}

	if (misc_config->ex9)
		((struct nds32_reg *)reg_cache->reg_list[ITB].arch_info)->enable = true;

	if (no_cr5 == false)
		((struct nds32_reg *)reg_cache->reg_list[CR5].arch_info)->enable = true;

	if (cpu_version->cop_fpu_extension) {
		((struct nds32_reg *)reg_cache->reg_list[CR6].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[FPCSR].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[FPCFG].arch_info)->enable = true;
	}

	if (mmu_config->memory_protection == 1) {
		/* Secure MPU has no IPC, IPSW, P_ITYPE */
		((struct nds32_reg *)reg_cache->reg_list[IR1].arch_info)->enable = false;
		((struct nds32_reg *)reg_cache->reg_list[IR9].arch_info)->enable = false;
	}

	if (nds32->privilege_level != 0)
		((struct nds32_reg *)reg_cache->reg_list[IR3].arch_info)->enable = false;

	if (misc_config->mcu == true)
		((struct nds32_reg *)reg_cache->reg_list[IR4].arch_info)->enable = false;

	if (misc_config->interruption_level == false) {
		((struct nds32_reg *)reg_cache->reg_list[IR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR10].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR12].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR13].arch_info)->enable = true;

		/* Secure MPU has no IPC, IPSW, P_ITYPE */
		if (mmu_config->memory_protection != 1)
			((struct nds32_reg *)reg_cache->reg_list[IR7].arch_info)->enable = true;
	}

	if ((cpu_version->cpu_id_family == 0x9) ||
			(cpu_version->cpu_id_family == 0xA) ||
			(cpu_version->cpu_id_family == 0xC) ||
			(cpu_version->cpu_id_family == 0xD))
		((struct nds32_reg *)reg_cache->reg_list[IR8].arch_info)->enable = true;

	if (misc_config->shadow == 1) {
		((struct nds32_reg *)reg_cache->reg_list[IR16].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR17].arch_info)->enable = true;
	}

	if (misc_config->ifc)
		((struct nds32_reg *)reg_cache->reg_list[IFC_LP].arch_info)->enable = true;

	if (nds32->privilege_level != 0)
		((struct nds32_reg *)reg_cache->reg_list[MR0].arch_info)->enable = false;

	if (mmu_config->memory_protection == 1) {
		if (mmu_config->memory_protection_version == 24)
			((struct nds32_reg *)reg_cache->reg_list[MR4].arch_info)->enable = true;

		if (nds32->privilege_level == 0) {
			if ((mmu_config->memory_protection_version == 16) ||
				(mmu_config->memory_protection_version == 24)) {
				((struct nds32_reg *)reg_cache->reg_list[MR11].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[SECUR0].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR20].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR22].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR24].arch_info)->enable = true;
				((struct nds32_reg *)reg_cache->reg_list[IR30].arch_info)->enable = true;

				if (misc_config->shadow == 1) {
					((struct nds32_reg *)reg_cache->reg_list[IR21].arch_info)->enable = true;
					((struct nds32_reg *)reg_cache->reg_list[IR23].arch_info)->enable = true;
					((struct nds32_reg *)reg_cache->reg_list[IR25].arch_info)->enable = true;
				}
			}
		}
	} else if (mmu_config->memory_protection == 2) {
		((struct nds32_reg *)reg_cache->reg_list[MR1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[MR4].arch_info)->enable = true;

		if ((cpu_version->cpu_id_family != 0xA) && (cpu_version->cpu_id_family != 0xC) &&
				(cpu_version->cpu_id_family != 0xD))
			((struct nds32_reg *)reg_cache->reg_list[MR5].arch_info)->enable = true;
	}

	if (mmu_config->memory_protection > 0) {
		((struct nds32_reg *)reg_cache->reg_list[MR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[MR3].arch_info)->enable = true;
	}

	if (memory_config->ilm_base != 0)
		if (nds32->privilege_level == 0)
			((struct nds32_reg *)reg_cache->reg_list[MR6].arch_info)->enable = true;

	if (memory_config->dlm_base != 0)
		if (nds32->privilege_level == 0)
			((struct nds32_reg *)reg_cache->reg_list[MR7].arch_info)->enable = true;

	if ((memory_config->icache.line_size != 0) && (memory_config->dcache.line_size != 0))
		((struct nds32_reg *)reg_cache->reg_list[MR8].arch_info)->enable = true;

	if (misc_config->high_speed_memory_port)
		((struct nds32_reg *)reg_cache->reg_list[MR9].arch_info)->enable = true;

	if (mr10_exist)
		((struct nds32_reg *)reg_cache->reg_list[MR10].arch_info)->enable = true;

	if (misc_config->edm) {
		int dr_reg_n = nds32->edm.breakpoint_num * 5;

		for (int i = 0 ; i < dr_reg_n ; i++)
			((struct nds32_reg *)reg_cache->reg_list[DR0 + i].arch_info)->enable = true;

		((struct nds32_reg *)reg_cache->reg_list[DR41].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR43].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR44].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR45].arch_info)->enable = true;
	}

	if (misc_config->debug_tracer) {
		((struct nds32_reg *)reg_cache->reg_list[DR46].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DR47].arch_info)->enable = true;
	}

	if (misc_config->performance_monitor) {
		((struct nds32_reg *)reg_cache->reg_list[PFR0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[PFR1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[PFR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[PFR3].arch_info)->enable = true;
	}

	if (misc_config->local_memory_dma) {
		((struct nds32_reg *)reg_cache->reg_list[DMAR0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR3].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR4].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR6].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR7].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR8].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR9].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[DMAR10].arch_info)->enable = true;
	}

	if ((misc_config->local_memory_dma || misc_config->performance_monitor) &&
			(no_racr0 == false))
		((struct nds32_reg *)reg_cache->reg_list[RACR].arch_info)->enable = true;

	if (cpu_version->cop_fpu_extension || (misc_config->audio_isa != 0))
		((struct nds32_reg *)reg_cache->reg_list[FUCPR].arch_info)->enable = true;

	if (misc_config->audio_isa != 0) {
		if (misc_config->audio_isa > 1) {
			((struct nds32_reg *)reg_cache->reg_list[D0L24].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[D1L24].arch_info)->enable = true;
		}

		((struct nds32_reg *)reg_cache->reg_list[I0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I3].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I4].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I6].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[I7].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M1].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M2].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M3].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M5].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M6].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[M7].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[MOD].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[LBE].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[LE].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[LC].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[ADM_VBASE].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[SHFT_CTL0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[SHFT_CTL1].arch_info)->enable = true;

		uint32_t value_mod;
		uint32_t fucpr_backup;
		/* enable fpu and get configuration */
		nds32_get_mapped_reg(nds32, FUCPR, &fucpr_backup);
		if ((fucpr_backup & 0x80000000) == 0)
			nds32_set_mapped_reg(nds32, FUCPR, fucpr_backup | 0x80000000);
		nds32_get_mapped_reg(nds32, MOD, &value_mod);
		/* restore origin fucpr value */
		if ((fucpr_backup & 0x80000000) == 0)
			nds32_set_mapped_reg(nds32, FUCPR, fucpr_backup);

		if ((value_mod >> 6) & 0x1) {
			((struct nds32_reg *)reg_cache->reg_list[CB_CTL].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB0].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB1].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB2].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBB3].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE0].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE1].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE2].arch_info)->enable = true;
			((struct nds32_reg *)reg_cache->reg_list[CBE3].arch_info)->enable = true;
		}
	}

	if ((cpu_version->cpu_id_family == 0x9) ||
			(cpu_version->cpu_id_family == 0xA) ||
			(cpu_version->cpu_id_family == 0xC)) {

		((struct nds32_reg *)reg_cache->reg_list[IDR0].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IDR1].arch_info)->enable = true;

		if ((cpu_version->cpu_id_family == 0xC) && (cpu_version->revision == 0x0C))
			((struct nds32_reg *)reg_cache->reg_list[IDR0].arch_info)->enable = false;
	}

	uint32_t ir3_value;
	uint32_t ivb_prog_pri_lvl;
	uint32_t ivb_ivic_ver;

	nds32_get_mapped_reg(nds32, IR3, &ir3_value);
	ivb_prog_pri_lvl = ir3_value & 0x1;
	ivb_ivic_ver = (ir3_value >> 11) & 0x3;

	if ((ivb_prog_pri_lvl == 1) || (ivb_ivic_ver >= 1)) {
		((struct nds32_reg *)reg_cache->reg_list[IR18].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR19].arch_info)->enable = true;
	}

	if (ivb_ivic_ver >= 1) {
		((struct nds32_reg *)reg_cache->reg_list[IR26].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR27].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR28].arch_info)->enable = true;
		((struct nds32_reg *)reg_cache->reg_list[IR29].arch_info)->enable = true;
	}

	return ERROR_OK;
}

int nds32_init_register_table(struct nds32 *nds32)
{
	nds32_init_must_have_registers(nds32);

	return ERROR_OK;
}

int nds32_add_software_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	uint32_t data;
	uint32_t check_data;
	uint32_t break_insn;

	/* check the breakpoint size */
	target->type->read_buffer(target, breakpoint->address, 4, (uint8_t *)&data);

	/* backup origin instruction
	 * instruction is big-endian */
	if (*(char *)&data & 0x80) { /* 16-bits instruction */
		breakpoint->length = 2;
		break_insn = NDS32_BREAK_16;
	} else { /* 32-bits instruction */
		breakpoint->length = 4;
		break_insn = NDS32_BREAK_32;
	}

	free(breakpoint->orig_instr);

	breakpoint->orig_instr = malloc(breakpoint->length);
	memcpy(breakpoint->orig_instr, &data, breakpoint->length);

	/* self-modified code */
	target->type->write_buffer(target, breakpoint->address, breakpoint->length, (const uint8_t *)&break_insn);
	/* write_back & invalidate dcache & invalidate icache */
	nds32_cache_sync(target, breakpoint->address, breakpoint->length);

	/* read back to check */
	target->type->read_buffer(target, breakpoint->address, breakpoint->length, (uint8_t *)&check_data);
	if (memcmp(&check_data, &break_insn, breakpoint->length) == 0)
		return ERROR_OK;

	return ERROR_FAIL;
}

int nds32_remove_software_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	uint32_t check_data;
	uint32_t break_insn;

	if (breakpoint->length == 2)
		break_insn = NDS32_BREAK_16;
	else if (breakpoint->length == 4)
		break_insn = NDS32_BREAK_32;
	else
		return ERROR_FAIL;

	target->type->read_buffer(target, breakpoint->address, breakpoint->length,
			(uint8_t *)&check_data);

	/* break instruction is modified */
	if (memcmp(&check_data, &break_insn, breakpoint->length) != 0)
		return ERROR_FAIL;

	/* self-modified code */
	target->type->write_buffer(target, breakpoint->address, breakpoint->length,
			breakpoint->orig_instr);

	/* write_back & invalidate dcache & invalidate icache */
	nds32_cache_sync(target, breakpoint->address, breakpoint->length);

	return ERROR_OK;
}

/**
 * Restore the processor context on an Andes target.  The full processor
 * context is analyzed to see if any of the registers are dirty on this end, but
 * have a valid new value.  If this is the case, the processor is changed to the
 * appropriate mode and the new register values are written out to the
 * processor.  If there happens to be a dirty register with an invalid value, an
 * error will be logged.
 *
 * @param target Pointer to the Andes target to have its context restored
 * @return Error status if the target is not halted.
 */
int nds32_restore_context(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct reg_cache *reg_cache = nds32->core_cache;
	struct reg *reg;
	struct nds32_reg *reg_arch_info;
	unsigned int i;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* check if there are dirty registers */
	for (i = 0; i < reg_cache->num_regs; i++) {
		reg = &(reg_cache->reg_list[i]);
		if (reg->dirty == true) {
			if (reg->valid == true) {

				LOG_DEBUG("examining dirty reg: %s", reg->name);
				LOG_DEBUG("writing register %d with value 0x%8.8" PRIx32,
						i, buf_get_u32(reg->value, 0, 32));

				reg_arch_info = reg->arch_info;
				if (FD0 <= reg_arch_info->num && reg_arch_info->num <= FD31) {
					uint64_t val = buf_get_u64(reg_arch_info->value, 0, 64);
					aice_write_reg_64(aice, reg_arch_info->num, val);
				} else {
					uint32_t val = buf_get_u32(reg_arch_info->value, 0, 32);
					aice_write_register(aice, reg_arch_info->num, val);
				}

				reg->valid = true;
				reg->dirty = false;
			}
		}
	}

	return ERROR_OK;
}

int nds32_edm_config(struct nds32 *nds32)
{
	struct target *target = nds32->target;
	struct aice_port_s *aice = target_to_aice(target);
	uint32_t edm_cfg;
	uint32_t edm_ctl;

	aice_read_debug_reg(aice, NDS_EDM_SR_EDM_CFG, &edm_cfg);

	nds32->edm.version = (edm_cfg >> 16) & 0xFFFF;
	LOG_INFO("EDM version 0x%04x", nds32->edm.version);

	nds32->edm.breakpoint_num = (edm_cfg & 0x7) + 1;

	if ((nds32->edm.version & 0x1000) || (0x60 <= nds32->edm.version))
		nds32->edm.access_control = true;
	else
		nds32->edm.access_control = false;

	if ((edm_cfg >> 4) & 0x1)
		nds32->edm.direct_access_local_memory = true;
	else
		nds32->edm.direct_access_local_memory = false;

	if (nds32->edm.version <= 0x20)
		nds32->edm.direct_access_local_memory = false;

	aice_read_debug_reg(aice, NDS_EDM_SR_EDM_CTL, &edm_ctl);
	if (edm_ctl & (0x1 << 29))
		nds32->edm.support_max_stop = true;
	else
		nds32->edm.support_max_stop = false;

	/* set passcode for secure MCU */
	nds32_login(nds32);

	return ERROR_OK;
}

int nds32_config(struct nds32 *nds32)
{
	nds32_init_config(nds32);

	/* init optional system registers according to config registers */
	nds32_init_option_registers(nds32);

	/* get max interrupt level */
	if (nds32->misc_config.interruption_level)
		nds32->max_interrupt_level = 2;
	else
		nds32->max_interrupt_level = 3;

	/* get ILM/DLM size from MR6/MR7 */
	uint32_t value_mr6, value_mr7;
	uint32_t size_index;
	nds32_get_mapped_reg(nds32, MR6, &value_mr6);
	size_index = (value_mr6 >> 1) & 0xF;
	nds32->memory.ilm_size = NDS32_LM_SIZE_TABLE[size_index];

	nds32_get_mapped_reg(nds32, MR7, &value_mr7);
	size_index = (value_mr7 >> 1) & 0xF;
	nds32->memory.dlm_size = NDS32_LM_SIZE_TABLE[size_index];

	return ERROR_OK;
}

int nds32_init_arch_info(struct target *target, struct nds32 *nds32)
{
	target->arch_info = nds32;
	nds32->target = target;

	nds32->common_magic = NDS32_COMMON_MAGIC;
	nds32->init_arch_info_after_halted = false;
	nds32->auto_convert_hw_bp = true;
	nds32->global_stop = false;
	nds32->soft_reset_halt = false;
	nds32->edm_passcode = NULL;
	nds32->privilege_level = 0;
	nds32->boot_time = 1500;
	nds32->reset_halt_as_examine = false;
	nds32->keep_target_edm_ctl = false;
	nds32->word_access_mem = false;
	nds32->virtual_hosting = true;
	nds32->hit_syscall = false;
	nds32->active_syscall_id = NDS32_SYSCALL_UNDEFINED;
	nds32->virtual_hosting_errno = 0;
	nds32->virtual_hosting_ctrl_c = false;
	nds32->attached = false;

	nds32->syscall_break.asid = 0;
	nds32->syscall_break.length = 4;
	nds32->syscall_break.set = 0;
	nds32->syscall_break.orig_instr = NULL;
	nds32->syscall_break.next = NULL;
	nds32->syscall_break.unique_id = 0x515CAll + target->target_number;
	nds32->syscall_break.linked_BRP = 0;

	nds32_reg_init();

	if (ERROR_FAIL == nds32_reg_cache_init(target, nds32))
		return ERROR_FAIL;

	if (ERROR_OK != nds32_init_register_table(nds32))
		return ERROR_FAIL;

	return ERROR_OK;
}

int nds32_virtual_to_physical(struct target *target, target_addr_t address, target_addr_t *physical)
{
	struct nds32 *nds32 = target_to_nds32(target);

	if (nds32->memory.address_translation == false) {
		*physical = address;
		return ERROR_OK;
	}

	if (ERROR_OK == nds32_probe_tlb(nds32, address, physical))
		return ERROR_OK;

	if (ERROR_OK == nds32_walk_page_table(nds32, address, physical))
		return ERROR_OK;

	return ERROR_FAIL;
}

int nds32_cache_sync(struct target *target, target_addr_t address, uint32_t length)
{
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32 *nds32 = target_to_nds32(target);
	struct nds32_cache *dcache = &(nds32->memory.dcache);
	struct nds32_cache *icache = &(nds32->memory.icache);
	uint32_t dcache_line_size = NDS32_LINE_SIZE_TABLE[dcache->line_size];
	uint32_t icache_line_size = NDS32_LINE_SIZE_TABLE[icache->line_size];
	uint32_t cur_address;
	int result;
	uint32_t start_line, end_line;
	uint32_t cur_line;

	if ((dcache->line_size != 0) && (dcache->enable == true)) {
		/* address / dcache_line_size */
		start_line = address >> (dcache->line_size + 2);
		/* (address + length - 1) / dcache_line_size */
		end_line = (address + length - 1) >> (dcache->line_size + 2);

		for (cur_address = address, cur_line = start_line;
				cur_line <= end_line;
				cur_address += dcache_line_size, cur_line++) {
			/* D$ write back */
			result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1D_VA_WB, cur_address);
			if (result != ERROR_OK)
				return result;

			/* D$ invalidate */
			result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1D_VA_INVAL, cur_address);
			if (result != ERROR_OK)
				return result;
		}
	}

	if ((icache->line_size != 0) && (icache->enable == true)) {
		/*  address / icache_line_size */
		start_line = address >> (icache->line_size + 2);
		/* (address + length - 1) / icache_line_size */
		end_line = (address + length - 1) >> (icache->line_size + 2);

		for (cur_address = address, cur_line = start_line;
				cur_line <= end_line;
				cur_address += icache_line_size, cur_line++) {
			/* Because PSW.IT is turned off under debug exception, address MUST
			 * be physical address.  L1I_VA_INVALIDATE uses PSW.IT to decide
			 * address translation or not. */
			target_addr_t physical_addr;
			if (ERROR_FAIL == target->type->virt2phys(target, cur_address,
						&physical_addr))
				return ERROR_FAIL;

			/* I$ invalidate */
			result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1I_VA_INVAL, physical_addr);
			if (result != ERROR_OK)
				return result;
		}
	}

	return ERROR_OK;
}

uint32_t nds32_nextpc(struct nds32 *nds32, int current, uint32_t address)
{
	if (!current)
		nds32_set_mapped_reg(nds32, PC, address);
	else
		nds32_get_mapped_reg(nds32, PC, &address);

	return address;
}

int nds32_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	LOG_DEBUG("target->state: %s",
			target_state_name(target));

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct nds32 *nds32 = target_to_nds32(target);

	address = nds32_nextpc(nds32, current, address);

	LOG_DEBUG("STEP PC %08" TARGET_PRIxADDR "%s", address, !current ? "!" : "");

	/** set DSSIM */
	uint32_t ir14_value;
	nds32_get_mapped_reg(nds32, IR14, &ir14_value);
	if (nds32->step_isr_enable)
		ir14_value |= (0x1 << 31);
	else
		ir14_value &= ~(0x1 << 31);
	nds32_set_mapped_reg(nds32, IR14, ir14_value);

	/* check hit_syscall before leave_debug_state() because
	 * leave_debug_state() may clear hit_syscall flag */
	bool no_step = false;
	if (nds32->hit_syscall)
		/* step after hit_syscall should be ignored because
		 * leave_debug_state will step implicitly to skip the
		 * syscall */
		no_step = true;

	/********* TODO: maybe create another function to handle this part */
	CHECK_RETVAL(nds32->leave_debug_state(nds32, true));
	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));

	if (no_step == false) {
		struct aice_port_s *aice = target_to_aice(target);
		if (ERROR_OK != aice_step(aice))
			return ERROR_FAIL;
	}

	/* save state */
	CHECK_RETVAL(nds32->enter_debug_state(nds32, true));
	/********* TODO: maybe create another function to handle this part */

	/* restore DSSIM */
	if (nds32->step_isr_enable) {
		nds32_get_mapped_reg(nds32, IR14, &ir14_value);
		ir14_value &= ~(0x1 << 31);
		nds32_set_mapped_reg(nds32, IR14, ir14_value);
	}

	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	return ERROR_OK;
}

static int nds32_step_without_watchpoint(struct nds32 *nds32)
{
	struct target *target = nds32->target;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/** set DSSIM */
	uint32_t ir14_value;
	nds32_get_mapped_reg(nds32, IR14, &ir14_value);
	if (nds32->step_isr_enable)
		ir14_value |= (0x1 << 31);
	else
		ir14_value &= ~(0x1 << 31);
	nds32_set_mapped_reg(nds32, IR14, ir14_value);

	/********* TODO: maybe create another function to handle this part */
	CHECK_RETVAL(nds32->leave_debug_state(nds32, false));

	struct aice_port_s *aice = target_to_aice(target);

	if (ERROR_OK != aice_step(aice))
		return ERROR_FAIL;

	/* save state */
	CHECK_RETVAL(nds32->enter_debug_state(nds32, false));
	/********* TODO: maybe create another function to handle this part */

	/* restore DSSIM */
	if (nds32->step_isr_enable) {
		nds32_get_mapped_reg(nds32, IR14, &ir14_value);
		ir14_value &= ~(0x1 << 31);
		nds32_set_mapped_reg(nds32, IR14, ir14_value);
	}

	return ERROR_OK;
}

int nds32_target_state(struct nds32 *nds32, enum target_state *state)
{
	struct aice_port_s *aice = target_to_aice(nds32->target);
	enum aice_target_state_s nds32_state;

	if (aice_state(aice, &nds32_state) != ERROR_OK)
		return ERROR_FAIL;

	switch (nds32_state) {
		case AICE_DISCONNECT:
			LOG_INFO("USB is disconnected");
			return ERROR_FAIL;
		case AICE_TARGET_DETACH:
			LOG_INFO("Target is disconnected");
			return ERROR_FAIL;
		case AICE_TARGET_UNKNOWN:
			*state = TARGET_UNKNOWN;
			break;
		case AICE_TARGET_RUNNING:
			*state = TARGET_RUNNING;
			break;
		case AICE_TARGET_HALTED:
			*state = TARGET_HALTED;
			break;
		case AICE_TARGET_RESET:
			*state = TARGET_RESET;
			break;
		case AICE_TARGET_DEBUG_RUNNING:
			*state = TARGET_DEBUG_RUNNING;
			break;
		default:
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

int nds32_examine_debug_reason(struct nds32 *nds32)
{
	uint32_t reason;
	struct target *target = nds32->target;

	if (nds32->hit_syscall == true) {
		LOG_DEBUG("Hit syscall breakpoint");
		target->debug_reason = DBG_REASON_BREAKPOINT;
		return ERROR_OK;
	}

	nds32->get_debug_reason(nds32, &reason);

	LOG_DEBUG("nds32 examines debug reason: %s", nds32_debug_type_name[reason]);

	/* Examine debug reason */
	switch (reason) {
		case NDS32_DEBUG_BREAK:
		case NDS32_DEBUG_BREAK_16:
		case NDS32_DEBUG_INST_BREAK:
			{
				uint32_t value_pc;
				uint32_t opcode;
				struct nds32_instruction instruction;

				nds32_get_mapped_reg(nds32, PC, &value_pc);

				if (ERROR_OK != nds32_read_opcode(nds32, value_pc, &opcode))
					return ERROR_FAIL;
				if (ERROR_OK != nds32_evaluate_opcode(nds32, opcode, value_pc,
							&instruction))
					return ERROR_FAIL;

				/* hit 'break 0x7FFF' */
				if ((instruction.info.opc_6 == 0x32) &&
					(instruction.info.sub_opc == 0xA) &&
					(instruction.info.imm == 0x7FFF)) {
					target->debug_reason = DBG_REASON_EXIT;
				} else
					target->debug_reason = DBG_REASON_BREAKPOINT;
			}
			break;
		case NDS32_DEBUG_DATA_ADDR_WATCHPOINT_PRECISE:
		case NDS32_DEBUG_DATA_VALUE_WATCHPOINT_PRECISE:
		case NDS32_DEBUG_LOAD_STORE_GLOBAL_STOP: /* GLOBAL_STOP is precise exception */
			{
				int result;

				result = nds32->get_watched_address(nds32,
						&(nds32->watched_address), reason);
				/* do single step(without watchpoints) to skip the "watched" instruction */
				nds32_step_without_watchpoint(nds32);

				/* before single_step, save exception address */
				if (ERROR_OK != result)
					return ERROR_FAIL;

				target->debug_reason = DBG_REASON_WATCHPOINT;
			}
			break;
		case NDS32_DEBUG_DEBUG_INTERRUPT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case NDS32_DEBUG_HARDWARE_SINGLE_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case NDS32_DEBUG_DATA_VALUE_WATCHPOINT_IMPRECISE:
		case NDS32_DEBUG_DATA_ADDR_WATCHPOINT_NEXT_PRECISE:
		case NDS32_DEBUG_DATA_VALUE_WATCHPOINT_NEXT_PRECISE:
			if (ERROR_OK != nds32->get_watched_address(nds32,
						&(nds32->watched_address), reason))
				return ERROR_FAIL;

			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		default:
			target->debug_reason = DBG_REASON_UNDEFINED;
			break;
	}

	return ERROR_OK;
}

int nds32_login(struct nds32 *nds32)
{
	struct target *target = nds32->target;
	struct aice_port_s *aice = target_to_aice(target);
	uint32_t passcode_length;
	char command_sequence[129];
	char command_str[33];
	char code_str[9];
	uint32_t copy_length;
	uint32_t code;
	uint32_t i;

	LOG_DEBUG("nds32_login");

	if (nds32->edm_passcode != NULL) {
		/* convert EDM passcode to command sequences */
		passcode_length = strlen(nds32->edm_passcode);
		command_sequence[0] = '\0';
		for (i = 0; i < passcode_length; i += 8) {
			if (passcode_length - i < 8)
				copy_length = passcode_length - i;
			else
				copy_length = 8;

			strncpy(code_str, nds32->edm_passcode + i, copy_length);
			code_str[copy_length] = '\0';
			code = strtoul(code_str, NULL, 16);

			sprintf(command_str, "write_misc gen_port0 0x%" PRIx32 ";", code);
			strcat(command_sequence, command_str);
		}

		if (ERROR_OK != aice_program_edm(aice, command_sequence))
			return ERROR_FAIL;

		/* get current privilege level */
		uint32_t value_edmsw;
		aice_read_debug_reg(aice, NDS_EDM_SR_EDMSW, &value_edmsw);
		nds32->privilege_level = (value_edmsw >> 16) & 0x3;
		LOG_INFO("Current privilege level: %d", nds32->privilege_level);
	}

	if (nds32_edm_ops_num > 0) {
		const char *reg_name;
		for (i = 0 ; i < nds32_edm_ops_num ; i++) {
			code = nds32_edm_ops[i].value;
			if (nds32_edm_ops[i].reg_no == 6)
				reg_name = "gen_port0";
			else if (nds32_edm_ops[i].reg_no == 7)
				reg_name = "gen_port1";
			else
				return ERROR_FAIL;

			sprintf(command_str, "write_misc %s 0x%" PRIx32 ";", reg_name, code);
			if (ERROR_OK != aice_program_edm(aice, command_str))
				return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

int nds32_halt(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	enum target_state state;

	LOG_DEBUG("target->state: %s",
			target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (nds32_target_state(nds32, &state) != ERROR_OK)
		return ERROR_FAIL;

	if (TARGET_HALTED != state)
		/* TODO: if state == TARGET_HALTED, check ETYPE is DBGI or not */
		if (ERROR_OK != aice_halt(aice))
			return ERROR_FAIL;

	CHECK_RETVAL(nds32->enter_debug_state(nds32, true));

	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	return ERROR_OK;
}

/* poll current target status */
int nds32_poll(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	enum target_state state;

	if (nds32_target_state(nds32, &state) != ERROR_OK)
		return ERROR_FAIL;

	if (state == TARGET_HALTED) {
		if (target->state != TARGET_HALTED) {
			/* if false_hit, continue free_run */
			if (ERROR_OK != nds32->enter_debug_state(nds32, true)) {
				struct aice_port_s *aice = target_to_aice(target);
				aice_run(aice);
				return ERROR_OK;
			}

			LOG_DEBUG("Change target state to TARGET_HALTED.");

			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	} else if (state == TARGET_RESET) {
		if (target->state == TARGET_HALTED) {
			/* similar to assert srst */
			register_cache_invalidate(nds32->core_cache);
			target->state = TARGET_RESET;

			/* TODO: deassert srst */
		} else if (target->state == TARGET_RUNNING) {
			/* reset as running */
			LOG_WARNING("<-- TARGET WARNING! The debug target has been reset. -->");
		}
	} else {
		if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING) {
			LOG_DEBUG("Change target state to TARGET_RUNNING.");
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}

	return ERROR_OK;
}

int nds32_resume(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	LOG_DEBUG("current %d address %08" TARGET_PRIxADDR
			" handle_breakpoints %d"
			" debug_execution %d",
			current, address, handle_breakpoints, debug_execution);

	struct nds32 *nds32 = target_to_nds32(target);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	address = nds32_nextpc(nds32, current, address);

	LOG_DEBUG("RESUME PC %08" TARGET_PRIxADDR "%s", address, !current ? "!" : "");

	if (!debug_execution)
		target_free_all_working_areas(target);

	/* Disable HSS to avoid users misuse HSS */
	if (nds32_reach_max_interrupt_level(nds32) == false) {
		uint32_t value_ir0;
		nds32_get_mapped_reg(nds32, IR0, &value_ir0);
		value_ir0 &= ~(0x1 << 11);
		nds32_set_mapped_reg(nds32, IR0, value_ir0);
	}

	CHECK_RETVAL(nds32->leave_debug_state(nds32, true));
	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));

	if (nds32->virtual_hosting_ctrl_c == false) {
		struct aice_port_s *aice = target_to_aice(target);
		aice_run(aice);
	} else
		nds32->virtual_hosting_ctrl_c = false;

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;

	LOG_DEBUG("target->state: %s",
			target_state_name(target));

	return ERROR_OK;
}

static int nds32_soft_reset_halt(struct target *target)
{
	/* TODO: test it */
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);

	aice_assert_srst(aice, AICE_SRST);

	/* halt core and set pc to 0x0 */
	int retval = target_halt(target);
	if (retval != ERROR_OK)
		return retval;

	/* start fetching from IVB */
	uint32_t value_ir3;
	nds32_get_mapped_reg(nds32, IR3, &value_ir3);
	nds32_set_mapped_reg(nds32, PC, value_ir3 & 0xFFFF0000);

	return ERROR_OK;
}

int nds32_assert_reset(struct target *target)
{
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32_cpu_version *cpu_version = &(nds32->cpu_version);

	/* TODO: apply hw reset signal in not examined state */
	if (!(target_was_examined(target))) {
		LOG_WARNING("Reset is not asserted because the target is not examined.");
		LOG_WARNING("Use a reset button or power cycle the target.");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	if (target->reset_halt) {
		if ((nds32->soft_reset_halt)
			|| (nds32->edm.version < 0x51)
			|| ((nds32->edm.version == 0x51)
				&& (cpu_version->revision == 0x1C)
				&& (cpu_version->cpu_id_family == 0xC)
				&& (cpu_version->cpu_id_version == 0x0)))
			nds32_soft_reset_halt(target);
		else
			aice_assert_srst(aice, AICE_RESET_HOLD);
	} else {
		aice_assert_srst(aice, AICE_SRST);
		alive_sleep(nds32->boot_time);
	}

	/* set passcode for secure MCU after core reset */
	nds32_login(nds32);

	/* registers are now invalid */
	register_cache_invalidate(nds32->core_cache);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int nds32_gdb_attach(struct nds32 *nds32)
{
	LOG_DEBUG("nds32_gdb_attach, target coreid: %" PRId32, nds32->target->coreid);

	if (nds32->attached == false) {

		if (nds32->keep_target_edm_ctl) {
			/* backup target EDM_CTL */
			struct aice_port_s *aice = target_to_aice(nds32->target);
			aice_read_debug_reg(aice, NDS_EDM_SR_EDM_CTL, &nds32->backup_edm_ctl);
		}

		target_halt(nds32->target);

		nds32->attached = true;
	}

	return ERROR_OK;
}

static int nds32_gdb_detach(struct nds32 *nds32)
{
	LOG_DEBUG("nds32_gdb_detach");
	bool backup_virtual_hosting_setting;

	if (nds32->attached) {

		backup_virtual_hosting_setting = nds32->virtual_hosting;
		/* turn off virtual hosting before resume as gdb-detach */
		nds32->virtual_hosting = false;
		target_resume(nds32->target, 1, 0, 0, 0);
		nds32->virtual_hosting = backup_virtual_hosting_setting;

		if (nds32->keep_target_edm_ctl) {
			/* restore target EDM_CTL */
			struct aice_port_s *aice = target_to_aice(nds32->target);
			aice_write_debug_reg(aice, NDS_EDM_SR_EDM_CTL, nds32->backup_edm_ctl);
		}

		nds32->attached = false;
	}

	return ERROR_OK;
}

static int nds32_callback_event_handler(struct target *target,
		enum target_event event, void *priv)
{
	int retval = ERROR_OK;
	int target_number = *(int *)priv;

	if (target_number != target->target_number)
		return ERROR_OK;

	struct nds32 *nds32 = target_to_nds32(target);

	switch (event) {
		case TARGET_EVENT_GDB_ATTACH:
			retval = nds32_gdb_attach(nds32);
			break;
		case TARGET_EVENT_GDB_DETACH:
			retval = nds32_gdb_detach(nds32);
			break;
		default:
			break;
	}

	return retval;
}

int nds32_init(struct nds32 *nds32)
{
	/* Initialize anything we can set up without talking to the target */
	nds32->memory.access_channel = NDS_MEMORY_ACC_CPU;

	/* register event callback */
	target_register_event_callback(nds32_callback_event_handler,
			&(nds32->target->target_number));

	return ERROR_OK;
}

int nds32_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info)
{
	/* fill syscall parameters to file-I/O info */
	if (NULL == fileio_info) {
		LOG_ERROR("Target has not initial file-I/O data structure");
		return ERROR_FAIL;
	}

	struct nds32 *nds32 = target_to_nds32(target);
	uint32_t value_ir6;
	uint32_t syscall_id;

	if (nds32->hit_syscall == false)
		return ERROR_FAIL;

	nds32_get_mapped_reg(nds32, IR6, &value_ir6);
	syscall_id = (value_ir6 >> 16) & 0x7FFF;
	nds32->active_syscall_id = syscall_id;

	LOG_DEBUG("hit syscall ID: 0x%" PRIx32, syscall_id);

	/* free previous identifier storage */
	free(fileio_info->identifier);
	fileio_info->identifier = NULL;

	uint32_t reg_r0, reg_r1, reg_r2;
	nds32_get_mapped_reg(nds32, R0, &reg_r0);
	nds32_get_mapped_reg(nds32, R1, &reg_r1);
	nds32_get_mapped_reg(nds32, R2, &reg_r2);

	switch (syscall_id) {
		case NDS32_SYSCALL_EXIT:
			fileio_info->identifier = malloc(5);
			sprintf(fileio_info->identifier, "exit");
			fileio_info->param_1 = reg_r0;
			break;
		case NDS32_SYSCALL_OPEN:
			{
				uint8_t filename[256];
				fileio_info->identifier = malloc(5);
				sprintf(fileio_info->identifier, "open");
				fileio_info->param_1 = reg_r0;
				/* reserve fileio_info->param_2 for length of path */
				fileio_info->param_3 = reg_r1;
				fileio_info->param_4 = reg_r2;

				target->type->read_buffer(target, reg_r0, 256, filename);
				fileio_info->param_2 = strlen((char *)filename);
			}
			break;
		case NDS32_SYSCALL_CLOSE:
			fileio_info->identifier = malloc(6);
			sprintf(fileio_info->identifier, "close");
			fileio_info->param_1 = reg_r0;
			break;
		case NDS32_SYSCALL_READ:
			fileio_info->identifier = malloc(5);
			sprintf(fileio_info->identifier, "read");
			fileio_info->param_1 = reg_r0;
			fileio_info->param_2 = reg_r1;
			fileio_info->param_3 = reg_r2;
			break;
		case NDS32_SYSCALL_WRITE:
			fileio_info->identifier = malloc(6);
			sprintf(fileio_info->identifier, "write");
			fileio_info->param_1 = reg_r0;
			fileio_info->param_2 = reg_r1;
			fileio_info->param_3 = reg_r2;
			break;
		case NDS32_SYSCALL_LSEEK:
			fileio_info->identifier = malloc(6);
			sprintf(fileio_info->identifier, "lseek");
			fileio_info->param_1 = reg_r0;
			fileio_info->param_2 = reg_r1;
			fileio_info->param_3 = reg_r2;
			break;
		case NDS32_SYSCALL_UNLINK:
			{
				uint8_t filename[256];
				fileio_info->identifier = malloc(7);
				sprintf(fileio_info->identifier, "unlink");
				fileio_info->param_1 = reg_r0;
				/* reserve fileio_info->param_2 for length of path */

				target->type->read_buffer(target, reg_r0, 256, filename);
				fileio_info->param_2 = strlen((char *)filename);
			}
			break;
		case NDS32_SYSCALL_RENAME:
			{
				uint8_t filename[256];
				fileio_info->identifier = malloc(7);
				sprintf(fileio_info->identifier, "rename");
				fileio_info->param_1 = reg_r0;
				/* reserve fileio_info->param_2 for length of old path */
				fileio_info->param_3 = reg_r1;
				/* reserve fileio_info->param_4 for length of new path */

				target->type->read_buffer(target, reg_r0, 256, filename);
				fileio_info->param_2 = strlen((char *)filename);

				target->type->read_buffer(target, reg_r1, 256, filename);
				fileio_info->param_4 = strlen((char *)filename);
			}
			break;
		case NDS32_SYSCALL_FSTAT:
			fileio_info->identifier = malloc(6);
			sprintf(fileio_info->identifier, "fstat");
			fileio_info->param_1 = reg_r0;
			fileio_info->param_2 = reg_r1;
			break;
		case NDS32_SYSCALL_STAT:
			{
				uint8_t filename[256];
				fileio_info->identifier = malloc(5);
				sprintf(fileio_info->identifier, "stat");
				fileio_info->param_1 = reg_r0;
				/* reserve fileio_info->param_2 for length of old path */
				fileio_info->param_3 = reg_r1;

				target->type->read_buffer(target, reg_r0, 256, filename);
				fileio_info->param_2 = strlen((char *)filename) + 1;
			}
			break;
		case NDS32_SYSCALL_GETTIMEOFDAY:
			fileio_info->identifier = malloc(13);
			sprintf(fileio_info->identifier, "gettimeofday");
			fileio_info->param_1 = reg_r0;
			fileio_info->param_2 = reg_r1;
			break;
		case NDS32_SYSCALL_ISATTY:
			fileio_info->identifier = malloc(7);
			sprintf(fileio_info->identifier, "isatty");
			fileio_info->param_1 = reg_r0;
			break;
		case NDS32_SYSCALL_SYSTEM:
			{
				uint8_t command[256];
				fileio_info->identifier = malloc(7);
				sprintf(fileio_info->identifier, "system");
				fileio_info->param_1 = reg_r0;
				/* reserve fileio_info->param_2 for length of old path */

				target->type->read_buffer(target, reg_r0, 256, command);
				fileio_info->param_2 = strlen((char *)command);
			}
			break;
		case NDS32_SYSCALL_ERRNO:
			fileio_info->identifier = malloc(6);
			sprintf(fileio_info->identifier, "errno");
			nds32_set_mapped_reg(nds32, R0, nds32->virtual_hosting_errno);
			break;
		default:
			fileio_info->identifier = malloc(8);
			sprintf(fileio_info->identifier, "unknown");
			break;
	}

	return ERROR_OK;
}

int nds32_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c)
{
	LOG_DEBUG("syscall return code: 0x%x, errno: 0x%x , ctrl_c: %s",
			retcode, fileio_errno, ctrl_c ? "true" : "false");

	struct nds32 *nds32 = target_to_nds32(target);

	nds32_set_mapped_reg(nds32, R0, (uint32_t)retcode);

	nds32->virtual_hosting_errno = fileio_errno;
	nds32->virtual_hosting_ctrl_c = ctrl_c;
	nds32->active_syscall_id = NDS32_SYSCALL_UNDEFINED;

	return ERROR_OK;
}

int nds32_profiling(struct target *target, uint32_t *samples,
			uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds)
{
	/* sample $PC every 10 milliseconds */
	uint32_t iteration = seconds * 100;
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32 *nds32 = target_to_nds32(target);

	/* REVISIT: can nds32 profile without halting? */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target %s is not halted (profiling)", target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (max_num_samples < iteration)
		iteration = max_num_samples;

	int pc_regnum = nds32->register_map(nds32, PC);
	aice_profiling(aice, 10, iteration, pc_regnum, samples, num_samples);

	register_cache_invalidate(nds32->core_cache);

	return ERROR_OK;
}

int nds32_gdb_fileio_write_memory(struct nds32 *nds32, uint32_t address,
		uint32_t size, const uint8_t *buffer)
{
	if ((NDS32_SYSCALL_FSTAT == nds32->active_syscall_id) ||
			(NDS32_SYSCALL_STAT == nds32->active_syscall_id)) {
		/* If doing GDB file-I/O, target should convert 'struct stat'
		 * from gdb-format to target-format */
		uint8_t stat_buffer[NDS32_STRUCT_STAT_SIZE];
		/* st_dev 2 */
		stat_buffer[0] = buffer[3];
		stat_buffer[1] = buffer[2];
		/* st_ino 2 */
		stat_buffer[2] = buffer[7];
		stat_buffer[3] = buffer[6];
		/* st_mode 4 */
		stat_buffer[4] = buffer[11];
		stat_buffer[5] = buffer[10];
		stat_buffer[6] = buffer[9];
		stat_buffer[7] = buffer[8];
		/* st_nlink 2 */
		stat_buffer[8] = buffer[15];
		stat_buffer[9] = buffer[16];
		/* st_uid 2 */
		stat_buffer[10] = buffer[19];
		stat_buffer[11] = buffer[18];
		/* st_gid 2 */
		stat_buffer[12] = buffer[23];
		stat_buffer[13] = buffer[22];
		/* st_rdev 2 */
		stat_buffer[14] = buffer[27];
		stat_buffer[15] = buffer[26];
		/* st_size 4 */
		stat_buffer[16] = buffer[35];
		stat_buffer[17] = buffer[34];
		stat_buffer[18] = buffer[33];
		stat_buffer[19] = buffer[32];
		/* st_atime 4 */
		stat_buffer[20] = buffer[55];
		stat_buffer[21] = buffer[54];
		stat_buffer[22] = buffer[53];
		stat_buffer[23] = buffer[52];
		/* st_spare1 4 */
		stat_buffer[24] = 0;
		stat_buffer[25] = 0;
		stat_buffer[26] = 0;
		stat_buffer[27] = 0;
		/* st_mtime 4 */
		stat_buffer[28] = buffer[59];
		stat_buffer[29] = buffer[58];
		stat_buffer[30] = buffer[57];
		stat_buffer[31] = buffer[56];
		/* st_spare2 4 */
		stat_buffer[32] = 0;
		stat_buffer[33] = 0;
		stat_buffer[34] = 0;
		stat_buffer[35] = 0;
		/* st_ctime 4 */
		stat_buffer[36] = buffer[63];
		stat_buffer[37] = buffer[62];
		stat_buffer[38] = buffer[61];
		stat_buffer[39] = buffer[60];
		/* st_spare3 4 */
		stat_buffer[40] = 0;
		stat_buffer[41] = 0;
		stat_buffer[42] = 0;
		stat_buffer[43] = 0;
		/* st_blksize 4 */
		stat_buffer[44] = buffer[43];
		stat_buffer[45] = buffer[42];
		stat_buffer[46] = buffer[41];
		stat_buffer[47] = buffer[40];
		/* st_blocks 4 */
		stat_buffer[48] = buffer[51];
		stat_buffer[49] = buffer[50];
		stat_buffer[50] = buffer[49];
		stat_buffer[51] = buffer[48];
		/* st_spare4 8 */
		stat_buffer[52] = 0;
		stat_buffer[53] = 0;
		stat_buffer[54] = 0;
		stat_buffer[55] = 0;
		stat_buffer[56] = 0;
		stat_buffer[57] = 0;
		stat_buffer[58] = 0;
		stat_buffer[59] = 0;

		return nds32_write_buffer(nds32->target, address, NDS32_STRUCT_STAT_SIZE, stat_buffer);
	} else if (NDS32_SYSCALL_GETTIMEOFDAY == nds32->active_syscall_id) {
		/* If doing GDB file-I/O, target should convert 'struct timeval'
		 * from gdb-format to target-format */
		uint8_t timeval_buffer[NDS32_STRUCT_TIMEVAL_SIZE];
		timeval_buffer[0] = buffer[3];
		timeval_buffer[1] = buffer[2];
		timeval_buffer[2] = buffer[1];
		timeval_buffer[3] = buffer[0];
		timeval_buffer[4] = buffer[11];
		timeval_buffer[5] = buffer[10];
		timeval_buffer[6] = buffer[9];
		timeval_buffer[7] = buffer[8];

		return nds32_write_buffer(nds32->target, address, NDS32_STRUCT_TIMEVAL_SIZE, timeval_buffer);
	}

	return nds32_write_buffer(nds32->target, address, size, buffer);
}

int nds32_reset_halt(struct nds32 *nds32)
{
	LOG_INFO("reset halt as init");

	struct aice_port_s *aice = target_to_aice(nds32->target);
	aice_assert_srst(aice, AICE_RESET_HOLD);

	return ERROR_OK;
}
