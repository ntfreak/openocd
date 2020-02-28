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
 * In arc/cpu/ tcl files all regiters are defined as core, non-BCR aux
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
			"added register {name=%s, num=0x%x, type=%s%s%s%s}",
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
			reg->number , desc->name, value);


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

/* GDB register groups. For now we suport only general and "empty" */
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
	reg->value = &reg_desc->reg_value;
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
		LOG_ERROR("Requested register `%s' doens't exist.", reg_name);
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

	/* First do endiannes-safe read of register value
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
			LOG_DEBUG("Get core register regnum=%" PRIu32 ", name=%s, value=0x%08" PRIx32,
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
			LOG_DEBUG("Get aux register regnum=%" PRIu32 ", name=%s, value=0x%08" PRIx32,
				i , arc_reg->name, aux_values[aux_cnt]);
		}
	}

exit:
	free(core_values);
	free(core_addrs);
	free(aux_values);
	free(aux_addrs);

	return retval;
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
		/* TODO: Add Actionpoint check when AP support will be introduced*/
		LOG_WARNING("Unknown debug reason");
	}

	return ERROR_OK;
}

static int arc_debug_entry(struct target *target)
{
	CHECK_RETVAL(arc_save_context(target));

	/* TODO: reset internal indicators of caches states, otherwise D$/I$
	 * will not be flushed/invalidated when required. */
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
			target->state = TARGET_HALTED;
			CHECK_RETVAL(arc_debug_entry(target));
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


/* ARC v2 target */
struct target_type arcv2_target = {
	.name = "arcv2",

	.poll =	arc_poll,

	.arch_state = arc_arch_state,

	/* TODO That seems like something similiar to metaware hostlink, so perhaps
	 * we can exploit this in the future. */
	.target_request_data = NULL,

	.halt = arc_halt,
	.resume = arc_resume,
	.step = NULL,

	.assert_reset = arc_assert_reset,
	.deassert_reset = arc_deassert_reset,

	/* TODO Implement soft_reset_halt */
	.soft_reset_halt = NULL,

	.get_gdb_reg_list = arc_get_gdb_reg_list,

	.read_memory = arc_mem_read,
	.write_memory = arc_mem_write,
	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.add_breakpoint = NULL,
	.add_context_breakpoint = NULL,
	.add_hybrid_breakpoint = NULL,
	.remove_breakpoint = NULL,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,
	.hit_watchpoint = NULL,

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
