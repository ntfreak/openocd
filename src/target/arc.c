/***************************************************************************
 *   Copyright (C) 2013-2015,2019-2020 Synopsys, Inc.                      *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Mischa Jonker <mischa.jonker@synopsys.com>                            *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>                                    *
 *                                                                         *
 *   SPDX-License-Identifier: GPL-2.0-or-later                             *
 ***************************************************************************/


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arc.h"



/*
 * ARC architecture specific details.
 *
 * ARC has two types of registers:
 *  1) core registers(e.g. r0,r1..) [is_core = true]
 *  2) Auxiliary registers [is_core = false]..
 *
 * Auxiliary registers at the same time can be divided into
 * read-only BCR(build configuration regs, e.g. isa_config, mpu_build) and
 * R/RW non-BCR ("control" register, e.g. pc, status32_t, debug).
 *
 * The way of accessing to Core and AUX registers differs on Jtag level.
 * BCR/non-BCR describes if the register is immutable and that reading
 * unexisting register is safe RAZ, rather then an error.
 * Note, core registers cannot be BCR.
 *
 * In arc/cpu/ tcl files all registers are defined as core, non-BCR aux
 * and BCR aux, in "add-reg" command they are passed to three lists
 * respectively:  core_reg_descriptions, aux_reg_descriptions,
 * bcr_reg_descriptions.
 *
 * Due to the specifics of accessing to BCR/non-BCR registers there are two
 * register caches:
 *  1) core_and_aux_cache - includes registers described in
 *     core_reg_descriptions and aux_reg_descriptions lists.
 *     Used during save/restore context step.
 *  2) bcr_cache - includes registers described bcr_reg_descriptions.
 *     Currently used internally during configure step.
 */


static int arc_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint);

void arc_reg_data_type_add(struct target *target,
		struct arc_reg_data_type *data_type)
{
	LOG_DEBUG("Adding %s reg_data_type", data_type->data_type.id);
	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	list_add_tail(&data_type->list, &arc->reg_data_types);
}

/**
 * Private implementation of register_get_by_name() for ARC that
 * doesn't skip not [yet] existing registers. Used in many places
 * for iteration through registers and even for marking required registers as
 * existing.
 */
struct reg *arc_reg_get_by_name(struct reg_cache *first,
		const char *name, bool search_all)
{
	unsigned int i;
	struct reg_cache *cache = first;

	while (cache) {
		for (i = 0; i < cache->num_regs; i++) {
			if (!strcmp(cache->reg_list[i].name, name))
				return &(cache->reg_list[i]);
		}

		if (search_all)
			cache = cache->next;
		else
			break;
	}

	return NULL;
}

/**
 * Reset internal states of caches. Must be called when entering debugging.
 *
 * @param target Target for which to reset caches states.
 */
int arc_reset_caches_states(struct target *target)
{
	struct arc_common *arc = target_to_arc(target);

	LOG_DEBUG("Resetting internal variables of caches states");

	/* Reset caches states. */
	arc->dcache_flushed = false;
	arc->l2cache_flushed = false;
	arc->icache_invalidated = false;
	arc->dcache_invalidated = false;
	arc->l2cache_invalidated = false;

	return ERROR_OK;
}

/* Initialize arc_common structure, which passes to openocd target instance */
static int arc_init_arch_info(struct target *target, struct arc_common *arc,
	struct jtag_tap *tap)
{
	arc->common_magic = ARC_COMMON_MAGIC;
	target->arch_info = arc;

	arc->jtag_info.tap = tap;

	/* The only allowed ir_length is 4 for ARC jtag. */
	if (tap->ir_length != 4) {
		LOG_ERROR("ARC jtag instruction length should be equal to 4");
		return ERROR_FAIL;
	}

	/* On most ARC targets there is a dcache, so we enable its flushing
	 * by default. If there no dcache, there will be no error, just a slight
	 * performance penalty from unnecessary JTAG operations. */
	arc->has_dcache = true;
	arc->has_icache = true;
	/* L2$ is not available in a target by default. */
	arc->has_l2cache = false;
	arc_reset_caches_states(target);

	/* Add standard GDB data types */
	INIT_LIST_HEAD(&arc->reg_data_types);
	struct arc_reg_data_type *std_types = calloc(ARRAY_SIZE(standard_gdb_types),
		sizeof(*std_types));

	if (!std_types) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(standard_gdb_types); i++) {
		std_types[i].data_type.type = standard_gdb_types[i].type;
		std_types[i].data_type.id = standard_gdb_types[i].id;
		arc_reg_data_type_add(target, &(std_types[i]));
	}

	/* Fields related to target descriptions */
	INIT_LIST_HEAD(&arc->core_reg_descriptions);
	INIT_LIST_HEAD(&arc->aux_reg_descriptions);
	INIT_LIST_HEAD(&arc->bcr_reg_descriptions);
	arc->num_regs = 0;
	arc->num_core_regs = 0;
	arc->num_aux_regs = 0;
	arc->num_bcr_regs = 0;
	arc->last_general_reg = ULONG_MAX;
	arc->pc_index_in_cache = ULONG_MAX;
	arc->debug_index_in_cache = ULONG_MAX;

	return ERROR_OK;
}

int arc_reg_add(struct target *target, struct arc_reg_desc *arc_reg,
		const char * const type_name, const size_t type_name_len)
{
	assert(target);
	assert(arc_reg);

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	/* Find register type */
	{
		struct arc_reg_data_type *type;
		list_for_each_entry(type, &arc->reg_data_types, list)
			if (!strncmp(type->data_type.id, type_name, type_name_len)) {
				arc_reg->data_type = &(type->data_type);
				break;
			}

		if (!arc_reg->data_type)
			return ERROR_ARC_REGTYPE_NOT_FOUND;
	}

	if (arc_reg->is_core) {
		list_add_tail(&arc_reg->list, &arc->core_reg_descriptions);
		arc->num_core_regs += 1;
	} else if (arc_reg->is_bcr) {
		list_add_tail(&arc_reg->list, &arc->bcr_reg_descriptions);
		arc->num_bcr_regs += 1;
	} else {
		list_add_tail(&arc_reg->list, &arc->aux_reg_descriptions);
		arc->num_aux_regs += 1;
	}
	arc->num_regs += 1;

	LOG_DEBUG(
			"added register {name=%s, num=0x%" PRIx32 ", type=%s%s%s%s}",
			arc_reg->name, arc_reg->arch_num, arc_reg->data_type->id,
			arc_reg->is_core ? ", core" : "",  arc_reg->is_bcr ? ", bcr" : "",
			arc_reg->is_general ? ", general" : ""
		);

	return ERROR_OK;
}

/* Reading core or aux register */
static int arc_get_register(struct reg *reg)
{
	assert(reg);

	struct arc_reg_desc *desc = reg->arch_info;
	struct target *target = desc->target;
	struct arc_common *arc = target_to_arc(target);

	uint32_t value;

	if (reg->valid) {
		LOG_DEBUG("Get register (cached) gdb_num=%" PRIu32 ", name=%s, value=0x%" PRIx32,
				reg->number, desc->name, target_buffer_get_u32(target, reg->value));
		return ERROR_OK;
	}

	if (desc->is_core) {
		/* Accessing to R61/R62 registers causes Jtag hang */
		if (desc->arch_num == CORE_R61_NUM || desc->arch_num == CORE_R62_NUM) {
			LOG_ERROR("It is forbidden to read core registers 61 and 62.");
			return ERROR_FAIL;
		}
		CHECK_RETVAL(arc_jtag_read_core_reg_one(&arc->jtag_info, desc->arch_num,
					&value));
	} else {
		CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, desc->arch_num,
			&value));
	}

	target_buffer_set_u32(target, reg->value, value);

	/* If target is unhalted all register reads should be uncached. */
	if (target->state == TARGET_HALTED)
		reg->valid = true;
	else
		reg->valid = false;

	reg->dirty = false;

	LOG_DEBUG("Get register gdb_num=%" PRIu32 ", name=%s, value=0x%" PRIx32,
			reg->number, desc->name, value);


	return ERROR_OK;
}

/* Writing core or aux register */
static int arc_set_register(struct reg *reg, uint8_t *buf)
{
	struct arc_reg_desc *desc = reg->arch_info;
	struct target *target = desc->target;
	uint32_t value = target_buffer_get_u32(target, buf);
	/* Unlike "get" function "set" is supported only if target
	* is in halt mode. Async writes are not supported yet. */
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* Accessing to R61/R62 registers causes Jtag hang */
	if (desc->is_core && (desc->arch_num == CORE_R61_NUM ||
			desc->arch_num == CORE_R62_NUM)) {
		LOG_ERROR("It is forbidden to write core registers 61 and 62.");
		return ERROR_FAIL;
	}
	target_buffer_set_u32(target, reg->value, value);

	LOG_DEBUG("Set register gdb_num=%" PRIu32 ", name=%s, value=0x%08" PRIx32,
			reg->number, desc->name, value);

	reg->valid = true;
	reg->dirty = true;

	return ERROR_OK;
}

const struct reg_arch_type arc_reg_type = {
	.get = arc_get_register,
	.set = arc_set_register,
};

/* GDB register groups. For now we support only general and "empty" */
static const char * const reg_group_general = "general";
static const char * const reg_group_other = "";

/* Common code to initialize `struct reg` for different registers: core, aux, bcr. */
static int arc_init_reg(struct target *target, struct reg *reg,
			struct arc_reg_desc *reg_desc, unsigned long number)
{
	assert(target);
	assert(reg);
	assert(reg_desc);

	struct arc_common *arc = target_to_arc(target);

	/* Initialize struct reg */
	reg->name = reg_desc->name;
	reg->size = 32; /* All register in ARC are 32-bit */
	reg->value = reg_desc->reg_value;
	reg->type = &arc_reg_type;
	reg->arch_info = reg_desc;
	reg->caller_save = true; /* @todo should be configurable. */
	reg->reg_data_type = reg_desc->data_type;
	reg->feature = &reg_desc->feature;

	reg->feature->name = reg_desc->gdb_xml_feature;

	/* reg->number is used by OpenOCD as value for @regnum. Thus when setting
	 * value of a register GDB will use it as a number of register in
	 * P-packet. OpenOCD gdbserver will then use number of register in
	 * P-packet as an array index in the reg_list returned by
	 * arc_regs_get_gdb_reg_list. So to ensure that registers are assigned
	 * correctly it would be required to either sort registers in
	 * arc_regs_get_gdb_reg_list or to assign numbers sequentially here and
	 * according to how registers will be sorted in
	 * arc_regs_get_gdb_reg_list. Second options is much more simpler. */
	reg->number = number;

	if (reg_desc->is_general) {
		arc->last_general_reg = reg->number;
		reg->group = reg_group_general;
	} else {
		reg->group = reg_group_other;
	}

	return ERROR_OK;
}

/* Building aux/core reg_cache */
static int arc_build_reg_cache(struct target *target)
{
	unsigned long i = 0;
	struct arc_reg_desc *reg_desc;
	/* get pointers to arch-specific information */
	struct arc_common *arc = target_to_arc(target);
	const unsigned long num_regs = arc->num_core_regs + arc->num_aux_regs;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = calloc(1, sizeof(*cache));
	struct reg *reg_list = calloc(num_regs, sizeof(*reg_list));

	if (!cache || !reg_list)  {
		LOG_ERROR("Not enough memory");
		goto fail;
	}

	/* Build the process context cache */
	cache->name = "arc registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	arc->core_and_aux_cache = cache;
	(*cache_p) = cache;

	if (list_empty(&arc->core_reg_descriptions)) {
		LOG_ERROR("No core registers were defined");
		goto fail;
	}

	list_for_each_entry(reg_desc, &arc->core_reg_descriptions, list) {
		CHECK_RETVAL(arc_init_reg(target, &reg_list[i], reg_desc, i));

		LOG_DEBUG("reg n=%3li name=%3s group=%s feature=%s", i,
			reg_list[i].name, reg_list[i].group,
			reg_list[i].feature->name);

		i += 1;
	}

	if (list_empty(&arc->aux_reg_descriptions)) {
		LOG_ERROR("No aux registers were defined");
		goto fail;
	}

	list_for_each_entry(reg_desc, &arc->aux_reg_descriptions, list) {
		 CHECK_RETVAL(arc_init_reg(target, &reg_list[i],  reg_desc, i));

		LOG_DEBUG("reg n=%3li name=%3s group=%s feature=%s", i,
			reg_list[i].name, reg_list[i].group,
			reg_list[i].feature->name);

		/* PC and DEBUG are essential so we search for them. */
		if (!strcmp("pc", reg_desc->name)) {
			if (arc->pc_index_in_cache != ULONG_MAX) {
				LOG_ERROR("Double definition of PC in configuration");
				goto fail;
			}
			arc->pc_index_in_cache = i;
		} else if (!strcmp("debug", reg_desc->name)) {
			if (arc->debug_index_in_cache != ULONG_MAX) {
				LOG_ERROR("Double definition of DEBUG in configuration");
				goto fail;
			}
			arc->debug_index_in_cache = i;
		}
		i += 1;
	}

	if (arc->pc_index_in_cache == ULONG_MAX
			|| arc->debug_index_in_cache == ULONG_MAX) {
		LOG_ERROR("`pc' and `debug' registers must be present in target description.");
		goto fail;
	}

	assert(i == (arc->num_core_regs + arc->num_aux_regs));

	arc->core_aux_cache_built = true;

	return ERROR_OK;

fail:
	free(cache);
	free(reg_list);

	return ERROR_FAIL;
}

/* Build bcr reg_cache.
 * This function must be called only after arc_build_reg_cache */
static int arc_build_bcr_reg_cache(struct target *target)
{
	/* get pointers to arch-specific information */
	struct arc_common *arc = target_to_arc(target);
	const unsigned long num_regs = arc->num_bcr_regs;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(*cache));
	struct reg *reg_list = calloc(num_regs, sizeof(*reg_list));

	struct arc_reg_desc *reg_desc;
	unsigned long i = 0;
	unsigned long gdb_regnum = arc->core_and_aux_cache->num_regs;

	if (!cache || !reg_list)  {
		LOG_ERROR("Unable to allocate memory");
		goto fail;
	}

	/* Build the process context cache */
	cache->name = "arc.bcr";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	arc->bcr_cache = cache;
	(*cache_p) = cache;

	if (list_empty(&arc->bcr_reg_descriptions)) {
		LOG_ERROR("No BCR registers are defined");
		goto fail;
	}

	list_for_each_entry(reg_desc, &arc->bcr_reg_descriptions, list) {
		 CHECK_RETVAL(arc_init_reg(target, &reg_list[i], reg_desc, gdb_regnum));
		/* BCRs always semantically, they are just read-as-zero, if there is
		 * not real register. */
		reg_list[i].exist = true;

		LOG_DEBUG("reg n=%3li name=%3s group=%s feature=%s", i,
			reg_list[i].name, reg_list[i].group,
			reg_list[i].feature->name);
		i += 1;
		gdb_regnum += 1;
	}

	assert(i == arc->num_bcr_regs);

	arc->bcr_cache_built = true;


	return ERROR_OK;
fail:
	free(cache);
	free(reg_list);

	return ERROR_FAIL;
}


static int arc_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
	int *reg_list_size, enum target_register_class reg_class)
{
	assert(target->reg_cache);
	struct arc_common *arc = target_to_arc(target);

	/* get pointers to arch-specific information storage */
	*reg_list_size = arc->num_regs;
	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));

	if (!*reg_list) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	/* OpenOCD gdb_server API seems to be inconsistent here: when it generates
	 * XML tdesc it filters out !exist registers, however when creating a
	 * g-packet it doesn't do so. REG_CLASS_ALL is used in first case, and
	 * REG_CLASS_GENERAL used in the latter one. Due to this we had to filter
	 * out !exist register for "general", but not for "all". Attempts to filter out
	 * !exist for "all" as well will cause a failed check in OpenOCD GDB
	 * server. */
	if (reg_class == REG_CLASS_ALL) {
		unsigned long i = 0;
		struct reg_cache *reg_cache = target->reg_cache;
		while (reg_cache) {
			for (unsigned j = 0; j < reg_cache->num_regs; j++, i++)
				(*reg_list)[i] =  &reg_cache->reg_list[j];
			reg_cache = reg_cache->next;
		}
		assert(i == arc->num_regs);
		LOG_DEBUG("REG_CLASS_ALL: number of regs=%i", *reg_list_size);
	} else {
		unsigned long i = 0;
		unsigned long gdb_reg_number = 0;
		struct reg_cache *reg_cache = target->reg_cache;
		while (reg_cache) {
			for (unsigned j = 0;
				 j < reg_cache->num_regs && gdb_reg_number <= arc->last_general_reg;
				 j++) {
				if (reg_cache->reg_list[j].exist) {
					(*reg_list)[i] =  &reg_cache->reg_list[j];
					i++;
				}
				gdb_reg_number += 1;
			}
			reg_cache = reg_cache->next;
		}
		*reg_list_size = i;
		LOG_DEBUG("REG_CLASS_GENERAL: number of regs=%i", *reg_list_size);
	}

	return ERROR_OK;
}

/* Reading field of struct_type register */
int arc_reg_get_field(struct target *target, const char *reg_name,
		const char *field_name, uint32_t *value_ptr)
{
	struct reg_data_type_struct_field *field;

	LOG_DEBUG("getting register field (reg_name=%s, field_name=%s)", reg_name, field_name);

	/* Get register */
	struct reg *reg = arc_reg_get_by_name(target->reg_cache, reg_name, true);

	if (!reg) {
		LOG_ERROR("Requested register `%s' doesn't exist.", reg_name);
		return ERROR_ARC_REGISTER_NOT_FOUND;
	}

	if (reg->reg_data_type->type != REG_TYPE_ARCH_DEFINED
	    || reg->reg_data_type->type_class != REG_TYPE_CLASS_STRUCT)
		return ERROR_ARC_REGISTER_IS_NOT_STRUCT;

	/* Get field in a register */
	struct reg_data_type_struct *reg_struct =
		reg->reg_data_type->reg_type_struct;
	for (field = reg_struct->fields;
	     field;
	     field = field->next) {
		if (!strcmp(field->name, field_name))
			break;
	}

	if (!field)
		return ERROR_ARC_REGISTER_FIELD_NOT_FOUND;

	if (!field->use_bitfields)
		return ERROR_ARC_FIELD_IS_NOT_BITFIELD;

	if (!reg->valid)
		CHECK_RETVAL(reg->type->get(reg));

	/* First do endianness-safe read of register value
	 * then convert it to binary buffer for further
	 * field extraction */

	*value_ptr = buf_get_u32(reg->value, field->bitfield->start,
		field->bitfield->end - field->bitfield->start + 1);

	return ERROR_OK;
}

static int arc_get_register_value(struct target *target, const char *reg_name,
		uint32_t *value_ptr)
{
	LOG_DEBUG("reg_name=%s", reg_name);

	struct reg *reg = arc_reg_get_by_name(target->reg_cache, reg_name, true);

	if (!reg)
		return ERROR_ARC_REGISTER_NOT_FOUND;

	if (!reg->valid)
		CHECK_RETVAL(reg->type->get(reg));

	*value_ptr = target_buffer_get_u32(target, reg->value);

	return ERROR_OK;
}

static int arc_set_register_value(struct target *target, const char *reg_name,
		uint32_t value)
{
	LOG_DEBUG("reg_name=%s value=0x%08" PRIx32, reg_name, value);

	if (!(target && reg_name)) {
		LOG_ERROR("Arguments cannot be NULL.");
		return ERROR_FAIL;
	}

	struct reg *reg = arc_reg_get_by_name(target->reg_cache, reg_name, true);

	if (!reg)
		return ERROR_ARC_REGISTER_NOT_FOUND;

	uint8_t value_buf[4];
	buf_set_u32(value_buf, 0, 32, value);
	CHECK_RETVAL(reg->type->set(reg, value_buf));

	return ERROR_OK;
}

/* Configure DCCM's */
static int arc_configure_dccm(struct target  *target)
{
	struct arc_common *arc = target_to_arc(target);

	uint32_t dccm_build_version, dccm_build_size0, dccm_build_size1;
	CHECK_RETVAL(arc_reg_get_field(target, "dccm_build", "version",
		&dccm_build_version));
	CHECK_RETVAL(arc_reg_get_field(target, "dccm_build", "size0",
		&dccm_build_size0));
	CHECK_RETVAL(arc_reg_get_field(target, "dccm_build", "size1",
		&dccm_build_size1));
	/* There is no yet support of configurable number of cycles,
	 * So there is no difference between v3 and v4 */
	if ((dccm_build_version == 3 || dccm_build_version == 4) && dccm_build_size0 > 0) {
		CHECK_RETVAL(arc_get_register_value(target, "aux_dccm", &(arc->dccm_start)));
		uint32_t dccm_size = 0x100;
		dccm_size <<= dccm_build_size0;
		if (dccm_build_size0 == 0xF)
			dccm_size <<= dccm_build_size1;
		arc->dccm_end = arc->dccm_start + dccm_size;
		LOG_DEBUG("DCCM detected start=0x%" PRIx32 " end=0x%" PRIx32,
				arc->dccm_start, arc->dccm_end);

	}
	return ERROR_OK;
}


/* Configure ICCM's */

static int arc_configure_iccm(struct target  *target)
{
	struct arc_common *arc = target_to_arc(target);

	/* ICCM0 */
	uint32_t iccm_build_version, iccm_build_size00, iccm_build_size01;
	uint32_t aux_iccm = 0;
	CHECK_RETVAL(arc_reg_get_field(target, "iccm_build", "version",
		&iccm_build_version));
	CHECK_RETVAL(arc_reg_get_field(target, "iccm_build", "iccm0_size0",
		&iccm_build_size00));
	CHECK_RETVAL(arc_reg_get_field(target, "iccm_build", "iccm0_size1",
		&iccm_build_size01));
	if (iccm_build_version == 4 && iccm_build_size00 > 0) {
		CHECK_RETVAL(arc_get_register_value(target, "aux_iccm", &aux_iccm));
		uint32_t iccm0_size = 0x100;
		iccm0_size <<= iccm_build_size00;
		if (iccm_build_size00 == 0xF)
			iccm0_size <<= iccm_build_size01;
		/* iccm0 start is located in highest 4 bits of aux_iccm */
		arc->iccm0_start = aux_iccm & 0xF0000000;
		arc->iccm0_end = arc->iccm0_start + iccm0_size;
		LOG_DEBUG("ICCM0 detected start=0x%" PRIx32 " end=0x%" PRIx32,
				arc->iccm0_start, arc->iccm0_end);
	}

	/* ICCM1 */
	uint32_t iccm_build_size10, iccm_build_size11;
	CHECK_RETVAL(arc_reg_get_field(target, "iccm_build", "iccm1_size0",
		&iccm_build_size10));
	CHECK_RETVAL(arc_reg_get_field(target, "iccm_build", "iccm1_size1",
		&iccm_build_size11));
	if (iccm_build_version == 4 && iccm_build_size10 > 0) {
		/* Use value read for ICCM0 */
		if (!aux_iccm)
			CHECK_RETVAL(arc_get_register_value(target, "aux_iccm", &aux_iccm));
		uint32_t iccm1_size = 0x100;
		iccm1_size <<= iccm_build_size10;
		if (iccm_build_size10 == 0xF)
			iccm1_size <<= iccm_build_size11;
		arc->iccm1_start = aux_iccm & 0x0F000000;
		arc->iccm1_end = arc->iccm1_start + iccm1_size;
		LOG_DEBUG("ICCM1 detected start=0x%" PRIx32 " end=0x%" PRIx32,
				arc->iccm1_start, arc->iccm1_end);
	}
	return ERROR_OK;
}

/* Configure some core features, depending on BCRs. */
static int arc_configure(struct target *target)
{
	LOG_DEBUG("Configuring ARC ICCM and DCCM");

	/* Configuring DCCM if DCCM_BUILD and AUX_DCCM are known registers. */
	if (arc_reg_get_by_name(target->reg_cache, "dccm_build", true) &&
	    arc_reg_get_by_name(target->reg_cache, "aux_dccm", true))
				CHECK_RETVAL(arc_configure_dccm(target));

	/* Configuring ICCM if ICCM_BUILD and AUX_ICCM are known registers. */
	if (arc_reg_get_by_name(target->reg_cache, "iccm_build", true) &&
	    arc_reg_get_by_name(target->reg_cache, "aux_iccm", true))
				CHECK_RETVAL(arc_configure_iccm(target));

	return ERROR_OK;
}

/* arc_examine is function, which is used for all arc targets*/
static int arc_examine(struct target *target)
{
	uint32_t status;
	struct arc_common *arc = target_to_arc(target);

	CHECK_RETVAL(arc_jtag_startup(&arc->jtag_info));

	if (!target_was_examined(target)) {
		CHECK_RETVAL(arc_jtag_status(&arc->jtag_info, &status));
		if (status & ARC_JTAG_STAT_RU)
			target->state = TARGET_RUNNING;
		else
			target->state = TARGET_HALTED;

		/* Read BCRs and configure optional registers. */
		CHECK_RETVAL(arc_configure(target));

		target_set_examined(target);
	}

	return ERROR_OK;
}

static int arc_halt(struct target *target)
{
	uint32_t value, irq_state;
	struct arc_common *arc = target_to_arc(target);

	LOG_DEBUG("target->state: %s", target_state_name(target));

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
		}
	}

	/* Break (stop) processor.
	 * Do read-modify-write sequence, or DEBUG.UB will be reset unintentionally.
	 * We do not use here arc_get/set_core_reg functions here because they imply
	 * that the processor is already halted. */
	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_DEBUG_REG, &value));
	value |= SET_CORE_FORCE_HALT; /* set the HALT bit */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DEBUG_REG, value));
	alive_sleep(1);

	/* Save current IRQ state */
	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG, &irq_state));

	if (irq_state & AUX_STATUS32_REG_IE_BIT)
		arc->irq_state = 1;
	else
		arc->irq_state = 0;

	/* update state and notify gdb*/
	target->state = TARGET_HALTED;
	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	/* some more debug information */
	if (debug_level >= LOG_LVL_DEBUG) {
		LOG_DEBUG("core stopped (halted) DEGUB-REG: 0x%08" PRIx32, value);
		CHECK_RETVAL(arc_get_register_value(target, "status32", &value));
		LOG_DEBUG("core STATUS32: 0x%08" PRIx32, value);
	}

	return ERROR_OK;
}

/**
 * Read registers that are used in GDB g-packet. We don't read them one-by-one,
 * but do that in one batch operation to improve speed. Calls to JTAG layer are
 * expensive so it is better to make one big call that reads all necessary
 * registers, instead of many calls, one for one register.
 */
static int arc_save_context(struct target *target)
{
	int retval = ERROR_OK;
	unsigned int i;
	struct arc_common *arc = target_to_arc(target);
	struct reg *reg_list = arc->core_and_aux_cache->reg_list;

	LOG_DEBUG("Saving aux and core registers values");
	assert(reg_list);

	/* It is assumed that there is at least one AUX register in the list, for
	 * example PC. */
	const uint32_t core_regs_size = arc->num_core_regs * sizeof(uint32_t);
	/* last_general_reg is inclusive number. To get count of registers it is
	 * required to do +1. */
	const uint32_t regs_to_scan =
		MIN(arc->last_general_reg + 1, arc->num_regs);
	const uint32_t aux_regs_size = arc->num_aux_regs * sizeof(uint32_t);
	uint32_t *core_values = malloc(core_regs_size);
	uint32_t *aux_values = malloc(aux_regs_size);
	uint32_t *core_addrs = malloc(core_regs_size);
	uint32_t *aux_addrs = malloc(aux_regs_size);
	unsigned int core_cnt = 0;
	unsigned int aux_cnt = 0;

	if (!core_values || !core_addrs || !aux_values || !aux_addrs)  {
		LOG_ERROR("Unable to allocate memory");
		retval = ERROR_FAIL;
		goto exit;
	}

	memset(core_values, 0xff, core_regs_size);
	memset(core_addrs, 0xff, core_regs_size);
	memset(aux_values, 0xff, aux_regs_size);
	memset(aux_addrs, 0xff, aux_regs_size);

	for (i = 0; i < MIN(arc->num_core_regs, regs_to_scan); i++) {
		struct reg *reg = &(reg_list[i]);
		struct arc_reg_desc *arc_reg = reg->arch_info;
		if (!reg->valid && reg->exist) {
			core_addrs[core_cnt] = arc_reg->arch_num;
			core_cnt += 1;
		}
	}

	for (i = arc->num_core_regs; i < regs_to_scan; i++) {
		struct reg *reg = &(reg_list[i]);
		struct arc_reg_desc *arc_reg = reg->arch_info;
		if (!reg->valid && reg->exist) {
			aux_addrs[aux_cnt] = arc_reg->arch_num;
			aux_cnt += 1;
		}
	}

	/* Read data from target. */
	if (core_cnt > 0) {
		retval = arc_jtag_read_core_reg(&arc->jtag_info, core_addrs, core_cnt, core_values);
		if (ERROR_OK != retval) {
			LOG_ERROR("Attempt to read core registers failed.");
			retval = ERROR_FAIL;
			goto exit;
		}
	}
	if (aux_cnt > 0) {
		retval = arc_jtag_read_aux_reg(&arc->jtag_info, aux_addrs, aux_cnt, aux_values);
		if (ERROR_OK != retval) {
			LOG_ERROR("Attempt to read aux registers failed.");
			retval = ERROR_FAIL;
			goto exit;
		}
	}

	/* Parse core regs */
	core_cnt = 0;
	for (i = 0; i < MIN(arc->num_core_regs, regs_to_scan); i++) {
		struct reg *reg = &(reg_list[i]);
		struct arc_reg_desc *arc_reg = reg->arch_info;
		if (!reg->valid && reg->exist) {
			target_buffer_set_u32(target, reg->value, core_values[core_cnt]);
			core_cnt += 1;
			reg->valid = true;
			reg->dirty = false;
			LOG_DEBUG("Get core register regnum=%u, name=%s, value=0x%08" PRIx32,
				i, arc_reg->name, core_values[core_cnt]);
		}
	}

	/* Parse aux regs */
	aux_cnt = 0;
	for (i = arc->num_core_regs; i < regs_to_scan; i++) {
		struct reg *reg = &(reg_list[i]);
		struct arc_reg_desc *arc_reg = reg->arch_info;
		if (!reg->valid && reg->exist) {
			target_buffer_set_u32(target, reg->value, aux_values[aux_cnt]);
			aux_cnt += 1;
			reg->valid = true;
			reg->dirty = false;
			LOG_DEBUG("Get aux register regnum=%u, name=%s, value=0x%08" PRIx32,
				i, arc_reg->name, aux_values[aux_cnt]);
		}
	}

exit:
	free(core_values);
	free(core_addrs);
	free(aux_values);
	free(aux_addrs);

	return retval;
}

/**
 * Finds an actionpoint that triggered last actionpoint event, as specified by
 * DEBUG.ASR.
 *
 * @param target
 * @param actionpoint Pointer to be set to last active actionpoint. Pointer
 *                    will be set to NULL if DEBUG.AH is 0.
 */
static int get_current_actionpoint(struct target *target,
		struct arc_actionpoint **actionpoint)
{
	assert(target != NULL);
	assert(actionpoint != NULL);

	uint32_t debug_ah;
	/* Check if actionpoint caused halt */
	CHECK_RETVAL(arc_reg_get_field(target, "debug", "ah",
				&debug_ah));

	if (debug_ah) {
		struct arc_common *arc = target_to_arc(target);
		unsigned int ap;
		uint32_t debug_asr;
		CHECK_RETVAL(arc_reg_get_field(target, "debug",
					"asr", &debug_asr));

		for (ap = 0; debug_asr > 1; debug_asr >>= 1)
			ap += 1;

		assert(ap < arc->actionpoints_num);

		*actionpoint = &(arc->actionpoints_list[ap]);
	} else {
		*actionpoint = NULL;
	}

	return ERROR_OK;
}

static int arc_examine_debug_reason(struct target *target)
{
	uint32_t debug_bh;

	/* Only check for reason if don't know it already. */
	/* BTW After singlestep at this point core is not marked as halted, so
	 * reading from memory to get current instruction wouldn't work anyway.  */
	if (target->debug_reason == DBG_REASON_DBGRQ ||
	    target->debug_reason == DBG_REASON_SINGLESTEP) {
		return ERROR_OK;
	}

	CHECK_RETVAL(arc_reg_get_field(target, "debug", "bh",
				&debug_bh));

	if (debug_bh) {
		/* DEBUG.BH is set if core halted due to BRK instruction.  */
		target->debug_reason = DBG_REASON_BREAKPOINT;
	} else {
		struct arc_actionpoint *actionpoint = NULL;
		CHECK_RETVAL(get_current_actionpoint(target, &actionpoint));

		if (actionpoint != NULL) {
			if (!actionpoint->used)
				LOG_WARNING("Target halted by an unused actionpoint.");

			if (actionpoint->type == ARC_AP_BREAKPOINT)
				target->debug_reason = DBG_REASON_BREAKPOINT;
			else if (actionpoint->type == ARC_AP_WATCHPOINT)
				target->debug_reason = DBG_REASON_WATCHPOINT;
			else
				LOG_WARNING("Unknown type of actionpoint.");
		}
	}

	return ERROR_OK;
}

static int arc_debug_entry(struct target *target)
{
	CHECK_RETVAL(arc_save_context(target));

	/* TODO: reset internal indicators of caches states, otherwise D$/I$
	 * will not be flushed/invalidated when required. */
	CHECK_RETVAL(arc_reset_caches_states(target));
	CHECK_RETVAL(arc_examine_debug_reason(target));

	return ERROR_OK;
}

static int arc_poll(struct target *target)
{
	uint32_t status, value;
	struct arc_common *arc = target_to_arc(target);

	/* gdb calls continuously through this arc_poll() function  */
	CHECK_RETVAL(arc_jtag_status(&arc->jtag_info, &status));

	/* check for processor halted */
	if (status & ARC_JTAG_STAT_RU) {
		if (target->state != TARGET_RUNNING) {
			LOG_WARNING("target is still running!");
			target->state = TARGET_RUNNING;
		}
		return ERROR_OK;
	}
	/* In some cases JTAG status register indicates that
	 *  processor is in halt mode, but processor is still running.
	 *  We check halt bit of AUX STATUS32 register for setting correct state. */
	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_RESET)) {
		CHECK_RETVAL(arc_get_register_value(target, "status32", &value));
		if (value & AUX_STATUS32_REG_HALT_BIT) {
			LOG_DEBUG("ARC core in halt or reset state.");
			/* Save context if target was not in reset state */
			if (target->state == TARGET_RUNNING)
				CHECK_RETVAL(arc_debug_entry(target));
			target->state = TARGET_HALTED;
			CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));
		} else {
		LOG_DEBUG("Discrepancy of STATUS32[0] HALT bit and ARC_JTAG_STAT_RU, "
						"target is still running");
		}

	} else if (target->state == TARGET_DEBUG_RUNNING) {

		target->state = TARGET_HALTED;
		LOG_DEBUG("ARC core is in debug running mode");

		CHECK_RETVAL(arc_debug_entry(target));

		CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED));
	}

	return ERROR_OK;
}

static int arc_assert_reset(struct target *target)
{
	struct arc_common *arc = target_to_arc(target);
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	bool srst_asserted = false;

	LOG_DEBUG("target->state: %s", target_state_name(target));

	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		/* allow scripts to override the reset event */

		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
		register_cache_invalidate(arc->core_and_aux_cache);
		 /* An ARC target might be in halt state after reset, so
		 * if script requested processor to resume, then it must
		 * be manually started to ensure that this request
		 * is satisfied. */
		if (target->state == TARGET_HALTED && !target->reset_halt) {
			/* Resume the target and continue from the current
			 * PC register value. */
			LOG_DEBUG("Starting CPU execution after reset");
			CHECK_RETVAL(target_resume(target, 1, 0, 0, 0));
		}
		target->state = TARGET_RESET;

		return ERROR_OK;
	}

	/* some cores support connecting while srst is asserted
	 * use that mode if it has been configured */
	if (!(jtag_reset_config & RESET_SRST_PULLS_TRST) &&
			(jtag_reset_config & RESET_SRST_NO_GATING)) {
		jtag_add_reset(0, 1);
		srst_asserted = true;
	}

	if (jtag_reset_config & RESET_HAS_SRST) {
		/* should issue a srst only, but we may have to assert trst as well */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
			jtag_add_reset(1, 1);
		else if (!srst_asserted)
			jtag_add_reset(0, 1);
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(50000);

	register_cache_invalidate(arc->core_and_aux_cache);

	if (target->reset_halt)
		CHECK_RETVAL(target_halt(target));

	return ERROR_OK;
}

static int arc_deassert_reset(struct target *target)
{
	LOG_DEBUG("target->state: %s", target_state_name(target));

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	return ERROR_OK;
}

static int arc_arch_state(struct target *target)
{
	uint32_t pc_value;

	if (debug_level < LOG_LVL_DEBUG)
		return ERROR_OK;

	CHECK_RETVAL(arc_get_register_value(target, "pc", &pc_value));

	LOG_DEBUG("target state: %s;  PC at: 0x%08" PRIx32,
		target_state_name(target),
		pc_value);

	return ERROR_OK;
}

/**
 * See arc_save_context() for reason why we want to dump all regs at once.
 * This however means that if there are dependencies between registers they
 * will not be observable until target will be resumed.
 */
static int arc_restore_context(struct target *target)
{
	int retval = ERROR_OK;
	unsigned int i;
	struct arc_common *arc = target_to_arc(target);
	struct reg *reg_list = arc->core_and_aux_cache->reg_list;

	LOG_DEBUG("Restoring registers values");
	assert(reg_list);

	const uint32_t core_regs_size = arc->num_core_regs  * sizeof(uint32_t);
	const uint32_t aux_regs_size =  arc->num_aux_regs * sizeof(uint32_t);
	uint32_t *core_values = malloc(core_regs_size);
	uint32_t *aux_values = malloc(aux_regs_size);
	uint32_t *core_addrs = malloc(core_regs_size);
	uint32_t *aux_addrs = malloc(aux_regs_size);
	unsigned int core_cnt = 0;
	unsigned int aux_cnt = 0;

	if (!core_values || !core_addrs || !aux_values || !aux_addrs)  {
		LOG_ERROR("Unable to allocate memory");
		retval = ERROR_FAIL;
		goto exit;
	}

	memset(core_values, 0xff, core_regs_size);
	memset(core_addrs, 0xff, core_regs_size);
	memset(aux_values, 0xff, aux_regs_size);
	memset(aux_addrs, 0xff, aux_regs_size);

	for (i = 0; i < arc->num_core_regs; i++) {
		struct reg *reg = &(reg_list[i]);
		struct arc_reg_desc *arc_reg = reg->arch_info;
		if (reg->valid && reg->exist && reg->dirty) {
			LOG_DEBUG("Will write regnum=%u", i);
			core_addrs[core_cnt] = arc_reg->arch_num;
			core_values[core_cnt] = target_buffer_get_u32(target, reg->value);
			core_cnt += 1;
		}
	}

	for (i = 0; i < arc->num_aux_regs; i++) {
		struct reg *reg = &(reg_list[arc->num_core_regs + i]);
		struct arc_reg_desc *arc_reg = reg->arch_info;
		if (reg->valid && reg->exist && reg->dirty) {
			LOG_DEBUG("Will write regnum=%lu", arc->num_core_regs + i);
			aux_addrs[aux_cnt] = arc_reg->arch_num;
			aux_values[aux_cnt] = target_buffer_get_u32(target, reg->value);
			aux_cnt += 1;
		}
	}

	/* Write data to target.
	 * Check before write, if aux and core count is greater than 0. */
	if (core_cnt > 0) {
		retval = arc_jtag_write_core_reg(&arc->jtag_info, core_addrs, core_cnt, core_values);
		if (ERROR_OK != retval) {
			LOG_ERROR("Attempt to write to core registers failed.");
			retval = ERROR_FAIL;
			goto exit;
		}
	}

	if (aux_cnt > 0) {
		retval = arc_jtag_write_aux_reg(&arc->jtag_info, aux_addrs, aux_cnt, aux_values);
		if (ERROR_OK != retval) {
			LOG_ERROR("Attempt to write to aux registers failed.");
			retval = ERROR_FAIL;
			goto exit;
		}
	}

exit:
	free(core_values);
	free(core_addrs);
	free(aux_values);
	free(aux_addrs);

	return retval;
}

static int arc_enable_interrupts(struct target *target, int enable)
{
	uint32_t value;

	struct arc_common *arc = target_to_arc(target);

	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG, &value));

	if (enable) {
		/* enable interrupts */
		value |= SET_CORE_ENABLE_INTERRUPTS;
		CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG, value));
		LOG_DEBUG("interrupts enabled");
	} else {
		/* disable interrupts */
		value &= ~SET_CORE_ENABLE_INTERRUPTS;
		CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG, value));
		LOG_DEBUG("interrupts disabled");
	}

	return ERROR_OK;
}

static int arc_resume(struct target *target, int current, target_addr_t address,
	int handle_breakpoints, int debug_execution)
{
	struct arc_common *arc = target_to_arc(target);
	uint32_t resume_pc = 0;
	uint32_t value;
	struct reg *pc = &arc->core_and_aux_cache->reg_list[arc->pc_index_in_cache];

	LOG_DEBUG("current:%i, address:0x%08" TARGET_PRIxADDR ", handle_breakpoints(not supported yet):%i,"
		" debug_execution:%i", current, address, handle_breakpoints, debug_execution);

	/* We need to reset ARC cache variables so caches
	 * would be invalidated and actual data
	 * would be fetched from memory. */
	CHECK_RETVAL(arc_reset_caches_states(target));

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current PC, otherwise continue at <address> */
	if (!current) {
		target_buffer_set_u32(target, pc->value, address);
		pc->dirty = 1;
		pc->valid = 1;
		LOG_DEBUG("Changing the value of current PC to 0x%08" TARGET_PRIxADDR, address);
	}

	if (!current)
		resume_pc = address;
	else
		resume_pc = target_buffer_get_u32(target, pc->value);

	CHECK_RETVAL(arc_restore_context(target));

	LOG_DEBUG("Target resumes from PC=0x%" PRIx32 ", pc.dirty=%i, pc.valid=%i",
		resume_pc, pc->dirty, pc->valid);

	/* check if GDB tells to set our PC where to continue from */
	if ((pc->valid == 1) && (resume_pc == target_buffer_get_u32(target, pc->value))) {
		value = target_buffer_get_u32(target, pc->value);
		LOG_DEBUG("resume Core (when start-core) with PC @:0x%08" PRIx32, value);
		CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_PC_REG, value));
	}

	/* Restore IRQ state if not in debug_execution*/
	if (!debug_execution)
		CHECK_RETVAL(arc_enable_interrupts(target, arc->irq_state));
	else
		CHECK_RETVAL(arc_enable_interrupts(target, !debug_execution));

	target->debug_reason = DBG_REASON_NOTHALTED;

	/* ready to get us going again */
	target->state = TARGET_RUNNING;
	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG, &value));
	value &= ~SET_CORE_HALT_BIT;        /* clear the HALT bit */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG, value));
	LOG_DEBUG("Core started to run");

	/* registers are now invalid */
	register_cache_invalidate(arc->core_and_aux_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));
		LOG_DEBUG("target resumed at 0x%08" PRIx32, resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED));
		LOG_DEBUG("target debug resumed at 0x%08" PRIx32, resume_pc);
	}

	return ERROR_OK;
}

static int arc_init_target(struct command_context *cmd_ctx, struct target *target)
{
	CHECK_RETVAL(arc_build_reg_cache(target));
	CHECK_RETVAL(arc_build_bcr_reg_cache(target));
	target->debug_reason = DBG_REASON_DBGRQ;
	return ERROR_OK;
}

static void arc_free_reg_cache(struct reg_cache *cache)
{
	free(cache->reg_list);
	free(cache);
}

static void arc_deinit_target(struct target *target)
{
	struct arc_common *arc = target_to_arc(target);

	LOG_DEBUG("deinitialization of target");
	if (arc->core_aux_cache_built)
		arc_free_reg_cache(arc->core_and_aux_cache);
	if (arc->bcr_cache_built)
		arc_free_reg_cache(arc->bcr_cache);

	struct arc_reg_data_type *type, *n;
	struct arc_reg_desc *desc, *k;

	/* Free arc-specific reg_data_types allocations*/
	list_for_each_entry_safe_reverse(type, n, &arc->reg_data_types, list) {
		if (type->data_type.type_class == REG_TYPE_CLASS_STRUCT) {
			free(type->reg_type_struct_field);
			free(type->bitfields);
			free(type);
		}	else if (type->data_type.type_class == REG_TYPE_CLASS_FLAGS) {
			free(type->reg_type_flags_field);
			free(type->bitfields);
			free(type);
		}
	}

	/* Free standard_gdb_types reg_data_types allocations */
	type = list_first_entry(&arc->reg_data_types, struct arc_reg_data_type, list);
	free(type);

	list_for_each_entry_safe(desc, k, &arc->aux_reg_descriptions, list)
		free_reg_desc(desc);

	list_for_each_entry_safe(desc, k, &arc->core_reg_descriptions, list)
		free_reg_desc(desc);

	list_for_each_entry_safe(desc, k, &arc->bcr_reg_descriptions, list)
		free_reg_desc(desc);

	free(arc->actionpoints_list);
	free(arc);
}


static int arc_target_create(struct target *target, Jim_Interp *interp)
{
	struct arc_common *arc = calloc(1, sizeof(*arc));

	if (!arc) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	LOG_DEBUG("Entering");
	CHECK_RETVAL(arc_init_arch_info(target, arc, target->tap));

	return ERROR_OK;
}

/**
 * Write 4-byte instruction to memory. This is like target_write_u32, however
 * in case of little endian ARC instructions are in middle endian format, not
 * little endian, so different type of conversion should be done.
 * Middle endian: instruction "aabbccdd", stored as "bbaaddcc"
 */
int arc_write_instruction_u32(struct target *target, uint32_t address,
	uint32_t instr)
{
	uint8_t value_buf[4];
	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	LOG_DEBUG("Address: 0x%08" PRIx32 ", value: 0x%08" PRIx32, address,
		instr);

	if (target->endianness == TARGET_LITTLE_ENDIAN)
		arc_h_u32_to_me(value_buf, instr);
	else
		h_u32_to_be(value_buf, instr);

	CHECK_RETVAL(target_write_buffer(target, address, 4, value_buf));

	return ERROR_OK;
}

/**
 * Read 32-bit instruction from memory. It is like target_read_u32, however in
 * case of little endian ARC instructions are in middle endian format, so
 * different type of conversion should be done.
 */
int arc_read_instruction_u32(struct target *target, uint32_t address,
		uint32_t *value)
{
	uint8_t value_buf[4];

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	*value = 0;
	CHECK_RETVAL(target_read_buffer(target, address, 4, value_buf));

	if (target->endianness == TARGET_LITTLE_ENDIAN)
		*value = arc_me_to_h_u32(value_buf);
	else
		*value = be_to_h_u32(value_buf);

	LOG_DEBUG("Address: 0x%08" PRIx32 ", value: 0x%08" PRIx32, address,
		*value);

	return ERROR_OK;
}

/* Actionpoint mechanism allows to setup HW breakpoints
 * and watchpoints. Each actionpoint is controlled by
 * 3 aux registers: Actionpoint(AP) match mask(AP_AMM), AP match value(AP_AMV)
 * and AP control(AC).
 * This function is for setting/unsetting actionpoints:
 * at - actionpoint target: trigger on mem/reg access
 * tt - transaction type : trigger on r/w. */
static int arc_configure_actionpoint(struct target *target, uint32_t ap_num,
	uint32_t match_value, uint32_t control_tt, uint32_t control_at)
{
	struct arc_common *arc = target_to_arc(target);

	if (control_tt != AP_AC_TT_DISABLE) {

		if (arc->actionpoints_num_avail < 1) {
			LOG_ERROR("No free actionpoints, maximim amount is %u",
					arc->actionpoints_num);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		/* Names of register to set - 24 chars should be enough. Looks a little
		 * bit out-of-place for C code, but makes it aligned to the bigger
		 * concept of "ARC registers are defined in TCL" as far as possible.
		 */
		char ap_amv_reg_name[24], ap_amm_reg_name[24], ap_ac_reg_name[24];
		snprintf(ap_amv_reg_name, 24, "ap_amv%" PRIu32, ap_num);
		snprintf(ap_amm_reg_name, 24, "ap_amm%" PRIu32, ap_num);
		snprintf(ap_ac_reg_name, 24, "ap_ac%" PRIu32, ap_num);
		CHECK_RETVAL(arc_set_register_value(target, ap_amv_reg_name,
					 match_value));
		CHECK_RETVAL(arc_set_register_value(target, ap_amm_reg_name, 0));
		CHECK_RETVAL(arc_set_register_value(target, ap_ac_reg_name,
					 control_tt | control_at));
		arc->actionpoints_num_avail--;
	} else {
		char ap_ac_reg_name[24];
		snprintf(ap_ac_reg_name, 24, "ap_ac%" PRIu32, ap_num);
		CHECK_RETVAL(arc_set_register_value(target, ap_ac_reg_name,
					 AP_AC_TT_DISABLE));
		arc->actionpoints_num_avail++;
	}

	return ERROR_OK;
}

static int arc_set_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_SOFT) {
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);

		if (breakpoint->length == 4) {
			uint32_t verify = 0xffffffff;

			CHECK_RETVAL(target_read_buffer(target, breakpoint->address, breakpoint->length,
					breakpoint->orig_instr));

			CHECK_RETVAL(arc_write_instruction_u32(target, breakpoint->address,
					ARC_SDBBP_32));

			CHECK_RETVAL(arc_read_instruction_u32(target, breakpoint->address, &verify));

			if (verify != ARC_SDBBP_32) {
				LOG_ERROR("Unable to set 32bit breakpoint at address @0x%" TARGET_PRIxADDR
						" - check that memory is read/writable", breakpoint->address);
				return ERROR_FAIL;
			}
		} else if (breakpoint->length == 2) {
			uint16_t verify = 0xffff;

			CHECK_RETVAL(target_read_buffer(target, breakpoint->address, breakpoint->length,
					breakpoint->orig_instr));
			CHECK_RETVAL(target_write_u16(target, breakpoint->address, ARC_SDBBP_16));

			CHECK_RETVAL(target_read_u16(target, breakpoint->address, &verify));
			if (verify != ARC_SDBBP_16) {
				LOG_ERROR("Unable to set 16bit breakpoint at address @0x%" TARGET_PRIxADDR
						" - check that memory is read/writable", breakpoint->address);
				return ERROR_FAIL;
			}
		} else {
			LOG_ERROR("Invalid breakpoint length: target supports only 2 or 4");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}

		breakpoint->set = 64; /* Any nice value but 0 */
	} else if (breakpoint->type == BKPT_HARD) {
		struct arc_common *arc = target_to_arc(target);
		struct arc_actionpoint *ap_list = arc->actionpoints_list;
		unsigned int bp_num;

		for (bp_num = 0; bp_num < arc->actionpoints_num; bp_num++) {
			if (!ap_list[bp_num].used)
				break;
		}

		if (bp_num >= arc->actionpoints_num) {
			LOG_ERROR("No free actionpoints, maximum amount is %u",
					arc->actionpoints_num);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		int retval = arc_configure_actionpoint(target, bp_num,
				breakpoint->address, AP_AC_TT_READWRITE, AP_AC_AT_INST_ADDR);

		if (retval == ERROR_OK) {
			breakpoint->set = bp_num + 1;
			ap_list[bp_num].used = 1;
			ap_list[bp_num].bp_value = breakpoint->address;
			ap_list[bp_num].type = ARC_AP_BREAKPOINT;

			LOG_DEBUG("bpid: %" PRIu32 ", bp_num %u bp_value 0x%" PRIx32,
					breakpoint->unique_id, bp_num, ap_list[bp_num].bp_value);
		}

	} else {
		LOG_DEBUG("ERROR: setting unknown breakpoint type");
		return ERROR_FAIL;
	}

	/* core instruction cache is now invalid. */
	CHECK_RETVAL(arc_cache_invalidate(target));

	return ERROR_OK;
}

static int arc_unset_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	int retval = ERROR_OK;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_SOFT) {
		/* restore original instruction (kept in target endianness) */
		LOG_DEBUG("bpid: %" PRIu32, breakpoint->unique_id);
		if (breakpoint->length == 4) {
			uint32_t current_instr;

			/* check that user program has not modified breakpoint instruction */
			CHECK_RETVAL(arc_read_instruction_u32(target, breakpoint->address, &current_instr));

			if (current_instr == ARC_SDBBP_32) {
				retval = target_write_buffer(target, breakpoint->address,
					breakpoint->length, breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;
			} else {
				LOG_WARNING("Software breakpoint @0x%" TARGET_PRIxADDR
					" has been overwritten outside of debugger."
					"Expected: @0x%x, got: @0x%" PRIx32,
					breakpoint->address, ARC_SDBBP_32, current_instr);
			}
		} else if (breakpoint->length == 2) {
			uint16_t current_instr;

			/* check that user program has not modified breakpoint instruction */
			CHECK_RETVAL(target_read_u16(target, breakpoint->address, &current_instr));
			if (current_instr == ARC_SDBBP_16) {
				retval = target_write_buffer(target, breakpoint->address,
					breakpoint->length, breakpoint->orig_instr);
				if (retval != ERROR_OK)
					return retval;
			} else {
				LOG_WARNING("Software breakpoint @0x%" TARGET_PRIxADDR
					" has been overwritten outside of debugger. "
					"Expected: 0x%04x, got: 0x%04" PRIx16,
					breakpoint->address, ARC_SDBBP_16, current_instr);
			}
		} else {
			LOG_ERROR("Invalid breakpoint length: target supports only 2 or 4");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		breakpoint->set = 0;

	}	else if (breakpoint->type == BKPT_HARD) {
		struct arc_common *arc = target_to_arc(target);
		struct arc_actionpoint *ap_list = arc->actionpoints_list;
		unsigned int bp_num = breakpoint->set - 1;

		if ((breakpoint->set == 0) || (bp_num >= arc->actionpoints_num)) {
			LOG_DEBUG("Invalid actionpoint ID: %u in breakpoint: %" PRIu32,
					  bp_num, breakpoint->unique_id);
			return ERROR_OK;
		}

		retval = arc_configure_actionpoint(target, bp_num,
						breakpoint->address, AP_AC_TT_DISABLE, AP_AC_AT_INST_ADDR);

		if (retval == ERROR_OK) {
			breakpoint->set = 0;
			ap_list[bp_num].used = 0;
			ap_list[bp_num].bp_value = 0;

			LOG_DEBUG("bpid: %" PRIu32 " - released actionpoint ID: %i",
					breakpoint->unique_id, bp_num);
		}
	} else {
			LOG_DEBUG("ERROR: unsetting unknown breakpoint type");
			return ERROR_FAIL;
	}

	/* core instruction cache is now invalid. */
	CHECK_RETVAL(arc_cache_invalidate(target));

	return retval;
}


static int arc_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (target->state == TARGET_HALTED) {
		return arc_set_breakpoint(target, breakpoint);

	} else {
		LOG_WARNING(" > core was not halted, please try again.");
		return ERROR_TARGET_NOT_HALTED;
	}
}

static int arc_remove_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	if (target->state == TARGET_HALTED) {
		if (breakpoint->set)
			CHECK_RETVAL(arc_unset_breakpoint(target, breakpoint));
	} else {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

void arc_reset_actionpoints(struct target *target)
{
	struct arc_common *arc = target_to_arc(target);
	struct arc_actionpoint *ap_list = arc->actionpoints_list;
	struct breakpoint *next_b;
	struct watchpoint *next_w;

	while (target->breakpoints) {
		next_b = target->breakpoints->next;
		arc_remove_breakpoint(target, target->breakpoints);
		free(target->breakpoints->orig_instr);
		free(target->breakpoints);
		target->breakpoints = next_b;
	}
	while (target->watchpoints) {
		next_w = target->watchpoints->next;
		arc_remove_watchpoint(target, target->watchpoints);
		free(target->watchpoints);
		target->watchpoints = next_w;
	}
	for (unsigned int i = 0; i < arc->actionpoints_num; i++) {
		if ((ap_list[i].used) && (ap_list[i].reg_address))
			arc_remove_auxreg_actionpoint(target, ap_list[i].reg_address);
	}
}

int arc_set_actionpoints_num(struct target *target, uint32_t ap_num)
{
	LOG_DEBUG("target=%s actionpoints=%" PRIu32, target_name(target), ap_num);
	struct arc_common *arc = target_to_arc(target);

	/* Make sure that there are no enabled actionpoints in target. */
	arc_reset_actionpoints(target);

	/* Assume that all points have been removed from target.  */
	free(arc->actionpoints_list);

	arc->actionpoints_num_avail = ap_num;
	arc->actionpoints_num = ap_num;
	/* calloc can be safely called when ncount == 0.  */
	arc->actionpoints_list = calloc(ap_num, sizeof(struct arc_actionpoint));

	if (!arc->actionpoints_list) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}


int arc_add_auxreg_actionpoint(struct target *target,
	uint32_t auxreg_addr, uint32_t transaction)
{
	unsigned int ap_num = 0;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	struct arc_common *arc = target_to_arc(target);
	struct arc_actionpoint *ap_list = arc->actionpoints_list;

	while (ap_list[ap_num].used)
		ap_num++;

	if (ap_num >= arc->actionpoints_num) {
		LOG_ERROR("No actionpoint free, maximum amount is %u",
				arc->actionpoints_num);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval =  arc_configure_actionpoint(target, ap_num,
			auxreg_addr, transaction, AP_AC_AT_AUXREG_ADDR);

	if (retval == ERROR_OK) {
		ap_list[ap_num].used = 1;
		ap_list[ap_num].reg_address = auxreg_addr;
	}

	return retval;
}

int arc_remove_auxreg_actionpoint(struct target *target, uint32_t auxreg_addr)
{
	int retval = ERROR_OK;
	bool ap_found = false;
	unsigned int ap_num = 0;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	struct arc_common *arc = target_to_arc(target);
	struct arc_actionpoint *ap_list = arc->actionpoints_list;

	while ((ap_list[ap_num].used) && (ap_num < arc->actionpoints_num)) {
		if (ap_list[ap_num].reg_address == auxreg_addr) {
			ap_found = true;
			break;
		}
		ap_num++;
	}

	if (ap_found) {
		retval =  arc_configure_actionpoint(target, ap_num,
				auxreg_addr, AP_AC_TT_DISABLE, AP_AC_AT_AUXREG_ADDR);

		if (retval == ERROR_OK) {
			ap_list[ap_num].used = 0;
			ap_list[ap_num].bp_value = 0;
		}
	} else {
		LOG_ERROR("Register actionpoint not found");
	}
	return retval;
}


static int arc_set_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	unsigned int wp_num;
	struct arc_common *arc = target_to_arc(target);
	struct arc_actionpoint *ap_list = arc->actionpoints_list;

	if (watchpoint->set) {
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}

	for (wp_num = 0; wp_num < arc->actionpoints_num; wp_num++) {
		if (!ap_list[wp_num].used)
			break;
	}

	if (wp_num >= arc->actionpoints_num) {
		LOG_ERROR("No free actionpoints, maximum amount is %u",
				arc->actionpoints_num);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (watchpoint->length != 4) {
		LOG_ERROR("Only watchpoints of length 4 are supported");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	int enable = AP_AC_TT_DISABLE;
	switch (watchpoint->rw) {
		case WPT_READ:
			enable = AP_AC_TT_READ;
			break;
		case WPT_WRITE:
			enable = AP_AC_TT_WRITE;
			break;
		case WPT_ACCESS:
			enable = AP_AC_TT_READWRITE;
			break;
		default:
			LOG_ERROR("BUG: watchpoint->rw neither read, write nor access");
			return ERROR_FAIL;
	}

	int retval =  arc_configure_actionpoint(target, wp_num,
					watchpoint->address, enable, AP_AC_AT_MEMORY_ADDR);

	if (retval == ERROR_OK) {
		watchpoint->set = wp_num + 1;
		ap_list[wp_num].used = 1;
		ap_list[wp_num].bp_value = watchpoint->address;
		ap_list[wp_num].type = ARC_AP_WATCHPOINT;

		LOG_DEBUG("wpid: %" PRIu32 ", wp_num %u wp_value 0x%" PRIx32,
				watchpoint->unique_id, wp_num, ap_list[wp_num].bp_value);
	}

	return retval;
}

static int arc_unset_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	/* get pointers to arch-specific information */
	struct arc_common *arc = target_to_arc(target);
	struct arc_actionpoint *ap_list = arc->actionpoints_list;

	if (!watchpoint->set) {
		LOG_WARNING("watchpoint not set");
		return ERROR_OK;
	}

	unsigned int wp_num = watchpoint->set - 1;
	if ((watchpoint->set == 0) || (wp_num >= arc->actionpoints_num)) {
		LOG_DEBUG("Invalid actionpoint ID: %u in watchpoint: %" PRIu32,
				wp_num, watchpoint->unique_id);
		return ERROR_OK;
	}

	int retval =  arc_configure_actionpoint(target, wp_num,
				watchpoint->address, AP_AC_TT_DISABLE, AP_AC_AT_MEMORY_ADDR);

	if (retval == ERROR_OK) {
		watchpoint->set = 0;
		ap_list[wp_num].used = 0;
		ap_list[wp_num].bp_value = 0;

		LOG_DEBUG("wpid: %" PRIu32 " - releasing actionpoint ID: %u",
				watchpoint->unique_id, wp_num);
	}

	return retval;
}

static int arc_add_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	CHECK_RETVAL(arc_set_watchpoint(target, watchpoint));

	return ERROR_OK;
}

static int arc_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
		CHECK_RETVAL(arc_unset_watchpoint(target, watchpoint));

	return ERROR_OK;
}

static int arc_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	assert(target);
	assert(hit_watchpoint);

	struct arc_actionpoint *actionpoint = NULL;
	CHECK_RETVAL(get_current_actionpoint(target, &actionpoint));

	if (actionpoint != NULL) {
		if (!actionpoint->used)
			LOG_WARNING("Target halted by unused actionpoint.");

		/* If this check fails - that is some sort of an error in OpenOCD. */
		if (actionpoint->type != ARC_AP_WATCHPOINT)
			LOG_WARNING("Target halted by breakpoint, but is treated as a watchpoint.");

		for (struct watchpoint *watchpoint = target->watchpoints;
				watchpoint != NULL;
				watchpoint = watchpoint->next) {
			if (actionpoint->bp_value == watchpoint->address) {
				*hit_watchpoint = watchpoint;
				LOG_DEBUG("Hit watchpoint, wpid: %" PRIu32 ", watchpoint num: %i",
							watchpoint->unique_id, watchpoint->set - 1);
				return ERROR_OK;
			}
		}
	}

	return ERROR_FAIL;
}

/* Helper function which switches core to single_step mode by
 * doing aux r/w operations.  */
int arc_config_step(struct target *target, int enable_step)
{
	uint32_t value;

	struct arc_common *arc = target_to_arc(target);

	/* enable core debug step mode */
	if (enable_step) {
		CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG,
			&value));
		value &= ~SET_CORE_AE_BIT; /* clear the AE bit */
		CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_STATUS32_REG,
			value));
		LOG_DEBUG(" [status32:0x%08" PRIx32 "]", value);

		/* Doing read-modify-write, because DEBUG might contain manually set
		 * bits like UB or ED, which should be preserved.  */
		CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info,
					AUX_DEBUG_REG, &value));
		value |= SET_CORE_SINGLE_INSTR_STEP; /* set the IS bit */
		CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DEBUG_REG,
			value));
		LOG_DEBUG("core debug step mode enabled [debug-reg:0x%08" PRIx32 "]", value);

	} else {	/* disable core debug step mode */
		CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_DEBUG_REG,
			&value));
		value &= ~SET_CORE_SINGLE_INSTR_STEP; /* clear the IS bit */
		CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DEBUG_REG,
			value));
		LOG_DEBUG("core debug step mode disabled");
	}

	return ERROR_OK;
}

int arc_step(struct target *target, int current, target_addr_t address,
	int handle_breakpoints)
{
	/* get pointers to arch-specific information */
	struct arc_common *arc = target_to_arc(target);
	struct breakpoint *breakpoint = NULL;
	struct reg *pc = &(arc->core_and_aux_cache->reg_list[arc->pc_index_in_cache]);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current) {
		buf_set_u32(pc->value, 0, 32, address);
		pc->dirty = 1;
		pc->valid = 1;
	}

	LOG_DEBUG("Target steps one instruction from PC=0x%" PRIx32,
		buf_get_u32(pc->value, 0, 32));

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, buf_get_u32(pc->value, 0, 32));
		if (breakpoint)
			CHECK_RETVAL(arc_unset_breakpoint(target, breakpoint));
	}

	/* restore context */
	CHECK_RETVAL(arc_restore_context(target));

	target->debug_reason = DBG_REASON_SINGLESTEP;

	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));

	/* disable interrupts while stepping */
	CHECK_RETVAL(arc_enable_interrupts(target, 0));

	/* do a single step */
	CHECK_RETVAL(arc_config_step(target, 1));

	/* make sure we done our step */
	alive_sleep(1);

	/* registers are now invalid */
	register_cache_invalidate(arc->core_and_aux_cache);

	if (breakpoint)
		CHECK_RETVAL(arc_set_breakpoint(target, breakpoint));

	LOG_DEBUG("target stepped ");

	target->state = TARGET_HALTED;

	/* Saving context */
	CHECK_RETVAL(arc_debug_entry(target));
	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	return ERROR_OK;
}


/* This function invalidates icache. */
static int arc_icache_invalidate(struct target *target)
{
	uint32_t value;

	struct arc_common *arc = target_to_arc(target);

	/* Don't waste time if already done. */
	if (!arc->has_icache || arc->icache_invalidated)
	    return ERROR_OK;

	LOG_DEBUG("Invalidating I$.");

	value = IC_IVIC_INVALIDATE;	/* invalidate I$ */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_IC_IVIC_REG, value));

	arc->icache_invalidated = true;

	return ERROR_OK;
}

/* This function invalidates dcache */
static int arc_dcache_invalidate(struct target *target)
{
	uint32_t value, dc_ctrl_value;

	struct arc_common *arc = target_to_arc(target);

	if (!arc->has_dcache || arc->dcache_invalidated)
	    return ERROR_OK;

	LOG_DEBUG("Invalidating D$.");

	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_DC_CTRL_REG, &value));
	dc_ctrl_value = value;
	value &= ~DC_CTRL_IM;

	/* set DC_CTRL invalidate mode to invalidate-only (no flushing!!) */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DC_CTRL_REG, value));
	value = DC_IVDC_INVALIDATE;	/* invalidate D$ */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DC_IVDC_REG, value));

	/* restore DC_CTRL invalidate mode */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DC_CTRL_REG, dc_ctrl_value));

	arc->dcache_invalidated = true;

	return ERROR_OK;
}

/* This function invalidates l2 cache. */
static int arc_l2cache_invalidate(struct target *target)
{
	uint32_t value, slc_ctrl_value;

	struct arc_common *arc = target_to_arc(target);

	if (!arc->has_l2cache || arc->l2cache_invalidated)
	    return ERROR_OK;

	LOG_DEBUG("Invalidating L2$.");

	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, SLC_AUX_CACHE_CTRL, &value));
	slc_ctrl_value = value;
	value &= ~L2_CTRL_IM;

	/* set L2_CTRL invalidate mode to invalidate-only (no flushing!!) */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, SLC_AUX_CACHE_CTRL, value));
	/* invalidate L2$ */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, SLC_AUX_CACHE_INV, L2_INV_IV));

	/* Wait until invalidate operation ends */
	do {
	    LOG_DEBUG("Waiting for invalidation end.");
	    CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, SLC_AUX_CACHE_CTRL, &value));
	} while (value & L2_CTRL_BS);

	/* restore L2_CTRL invalidate mode */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, SLC_AUX_CACHE_CTRL, slc_ctrl_value));

	arc->l2cache_invalidated = true;

	return ERROR_OK;
}


int arc_cache_invalidate(struct target *target)
{
	CHECK_RETVAL(arc_icache_invalidate(target));
	CHECK_RETVAL(arc_dcache_invalidate(target));
	CHECK_RETVAL(arc_l2cache_invalidate(target));

	return ERROR_OK;
}

/* Flush data cache. This function is cheap to call and return quickly if D$
 * already has been flushed since target had been halted. JTAG debugger reads
 * values directly from memory, bypassing cache, so if there are unflushed
 * lines debugger will read invalid values, which will cause a lot of troubles.
 * */
int arc_dcache_flush(struct target *target)
{
	uint32_t value, dc_ctrl_value;
	bool has_to_set_dc_ctrl_im;

	struct arc_common *arc = target_to_arc(target);

	/* Don't waste time if already done. */
	if (!arc->has_dcache || arc->dcache_flushed)
	    return ERROR_OK;

	LOG_DEBUG("Flushing D$.");

	/* Store current value of DC_CTRL */
	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, AUX_DC_CTRL_REG, &dc_ctrl_value));

	/* Set DC_CTRL invalidate mode to flush (if not already set) */
	has_to_set_dc_ctrl_im = (dc_ctrl_value & DC_CTRL_IM) == 0;
	if (has_to_set_dc_ctrl_im) {
		value = dc_ctrl_value | DC_CTRL_IM;
		CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DC_CTRL_REG, value));
	}

	/* Flush D$ */
	value = DC_IVDC_INVALIDATE;
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DC_IVDC_REG, value));

	/* Restore DC_CTRL invalidate mode (even of flush failed) */
	if (has_to_set_dc_ctrl_im)
	    CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, AUX_DC_CTRL_REG, dc_ctrl_value));

	arc->dcache_flushed = true;

	return ERROR_OK;
}

/* This function flushes l2cache. */
static int arc_l2cache_flush(struct target *target)
{
	uint32_t value;

	struct arc_common *arc = target_to_arc(target);

	/* Don't waste time if already done. */
	if (!arc->has_l2cache || arc->l2cache_flushed)
	    return ERROR_OK;

	LOG_DEBUG("Flushing L2$.");

	/* Flush L2 cache */
	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, SLC_AUX_CACHE_FLUSH, L2_FLUSH_FL));

	/* Wait until flush operation ends */
	do {
	    LOG_DEBUG("Waiting for flushing end.");
	    CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, SLC_AUX_CACHE_CTRL, &value));
	} while (value & L2_CTRL_BS);

	arc->l2cache_flushed = true;

	return ERROR_OK;
}

int arc_cache_flush(struct target *target)
{
	CHECK_RETVAL(arc_dcache_flush(target));
	CHECK_RETVAL(arc_l2cache_flush(target));

	return ERROR_OK;
}

/* ARC v2 target */
struct target_type arcv2_target = {
	.name = "arcv2",

	.poll =	arc_poll,

	.arch_state = arc_arch_state,

	/* TODO That seems like something similar to metaware hostlink, so perhaps
	 * we can exploit this in the future. */
	.target_request_data = NULL,

	.halt = arc_halt,
	.resume = arc_resume,
	.step = arc_step,

	.assert_reset = arc_assert_reset,
	.deassert_reset = arc_deassert_reset,

	/* TODO Implement soft_reset_halt */
	.soft_reset_halt = NULL,

	.get_gdb_reg_list = arc_get_gdb_reg_list,

	.read_memory = arc_mem_read,
	.write_memory = arc_mem_write,
	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.add_breakpoint = arc_add_breakpoint,
	.add_context_breakpoint = NULL,
	.add_hybrid_breakpoint = NULL,
	.remove_breakpoint = arc_remove_breakpoint,
	.add_watchpoint = arc_add_watchpoint,
	.remove_watchpoint = arc_remove_watchpoint,
	.hit_watchpoint = arc_hit_watchpoint,

	.run_algorithm = NULL,
	.start_algorithm = NULL,
	.wait_algorithm = NULL,

	.commands = arc_monitor_command_handlers,

	.target_create = arc_target_create,
	.init_target = arc_init_target,
	.deinit_target = arc_deinit_target,
	.examine = arc_examine,

	.virt2phys = NULL,
	.read_phys_memory = NULL,
	.write_phys_memory = NULL,
	.mmu = NULL,
};
