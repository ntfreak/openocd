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

/* --------------------------------------------------------------------------
 *
 *   ARC targets expose command interface.
 *   It can be accessed via GDB through the (gdb) monitor command.
 *
 * ------------------------------------------------------------------------- */


static int arc_cmd_jim_get_uint32(struct jim_getopt_info *goi, uint32_t *value)
{
	jim_wide value_wide;
	JIM_CHECK_RETVAL(jim_getopt_wide(goi, &value_wide));
	*value = (uint32_t)value_wide;
	return JIM_OK;
}

enum add_reg_types {
	CFG_ADD_REG_TYPE_FLAG,
	CFG_ADD_REG_TYPE_STRUCT,
};
/* Add flags register data type */
enum add_reg_type_flags {
	CFG_ADD_REG_TYPE_FLAGS_NAME,
	CFG_ADD_REG_TYPE_FLAGS_FLAG,
};

static struct jim_nvp nvp_add_reg_type_flags_opts[] = {
	{ .name = "-name",  .value = CFG_ADD_REG_TYPE_FLAGS_NAME },
	{ .name = "-flag",  .value = CFG_ADD_REG_TYPE_FLAGS_FLAG },
	{ .name = NULL,     .value = -1 }
};

/* Helper function to check if all field required for register
 * are set up */
static const char *validate_register(const struct arc_reg_desc * const reg, bool arch_num_set)
{
	/* Check that required fields are set */
	if (!reg->name)
		return "-name option is required";
	if (!reg->gdb_xml_feature)
		return "-feature option is required";
	if (!arch_num_set)
		return "-num option is required";
	if (reg->is_bcr && reg->is_core)
		return "Register cannot be both -core and -bcr.";
	return NULL;
}

/* Helper function to read the name of register type or register from
 * configure files  */
static int jim_arc_read_reg_name_field(struct jim_getopt_info *goi,
	const char **name, int *name_len)
{
	int e = JIM_OK;

	if (!goi->argc) {
		Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-name <name> ...");
		return JIM_ERR;
	}
	e = jim_getopt_string(goi, name, name_len);
	return e;
}

/* Helper function to read bitfields/flags of register type. */
static int jim_arc_read_reg_type_field(struct jim_getopt_info *goi, const char **field_name, int *field_name_len,
	 struct arc_reg_bitfield *bitfields, int cur_field, int type)
{
		jim_wide start_pos, end_pos;

		int e = JIM_OK;
		if ((type == CFG_ADD_REG_TYPE_STRUCT && goi->argc < 3) ||
		 (type == CFG_ADD_REG_TYPE_FLAG && goi->argc < 2)) {
			Jim_SetResultFormatted(goi->interp, "Not enough arguments after -flag/-bitfield");
			return JIM_ERR;
		}

		e = jim_getopt_string(goi, field_name, field_name_len);
		if (e != JIM_OK)
					return e;

		/* read start position of bitfield/flag */
		e = jim_getopt_wide(goi, &start_pos);
		if (e != JIM_OK)
					return e;

		end_pos = start_pos;

		/* Check if any arguments remain,
		 * set bitfields[cur_field].end if flag is multibit */
		if (goi->argc > 0)
			/* Check current argv[0], if it is equal to "-flag",
			 * than bitfields[cur_field].end remains start */
			if ((strcmp(Jim_String(goi->argv[0]), "-flag") && type == CFG_ADD_REG_TYPE_FLAG)
					|| (type == CFG_ADD_REG_TYPE_STRUCT)) {
								e = jim_getopt_wide(goi, &end_pos);
								if (e != JIM_OK) {
									Jim_SetResultFormatted(goi->interp, "Error reading end position");
									return e;
								}
							}

		bitfields[cur_field].bitfield.start = start_pos;
		bitfields[cur_field].bitfield.end = end_pos;
		if ((end_pos != start_pos) || (type == CFG_ADD_REG_TYPE_STRUCT))
			bitfields[cur_field].bitfield.type = REG_TYPE_INT;
	return e;
}

static int jim_arc_add_reg_type_flags(Jim_Interp *interp, int argc,
	Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	LOG_DEBUG("-");

	struct command_context *ctx;
	struct target *target;

	ctx = current_command_context(interp);
	assert(ctx);
	target = get_current_target(ctx);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	int e = JIM_OK;

	/* Check if the amount of arguments is not zero */
	if (goi.argc <= 0) {
		Jim_SetResultFormatted(goi.interp, "The command has no arguments");
		return JIM_ERR;
	}

	/* Estimate number of registers as (argc - 2)/3 as each -flag option has 2
	 * arguments while -name is required. */
	unsigned int fields_sz = (goi.argc - 2) / 3;
	unsigned int cur_field = 0;

	/* The maximum amount of bitfields is 32 */
	if (fields_sz > 32) {
		Jim_SetResultFormatted(goi.interp, "The amount of bitfields exceed 32");
		return JIM_ERR;
	}

	struct arc_reg_data_type *type = calloc(1, sizeof(*type));
	struct reg_data_type_flags *flags = &type->data_type_flags;
	struct reg_data_type_flags_field *fields = calloc(fields_sz, sizeof(*fields));
	type->reg_type_flags_field = fields;
	struct arc_reg_bitfield *bitfields = calloc(fields_sz, sizeof(*bitfields));
	if (!(type && fields && bitfields)) {
		Jim_SetResultFormatted(goi.interp, "Failed to allocate memory.");
		goto fail;
	}

	/* Initialize type */
	type->bitfields = bitfields;
	type->data_type.id = type->data_type_id;
	type->data_type.type = REG_TYPE_ARCH_DEFINED;
	type->data_type.type_class = REG_TYPE_CLASS_FLAGS;
	type->data_type.reg_type_flags = flags;
	flags->size = 4; /* For now ARC has only 32-bit registers */

	while (goi.argc > 0 && e == JIM_OK) {
		struct jim_nvp *n;
		e = jim_getopt_nvp(&goi, nvp_add_reg_type_flags_opts, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(&goi, nvp_add_reg_type_flags_opts, 0);
			continue;
		}

		switch (n->value) {
			case CFG_ADD_REG_TYPE_FLAGS_NAME:
			{
				const char *name = NULL;
				int name_len = 0;

				e = jim_arc_read_reg_name_field(&goi, &name, &name_len);
				if (e != JIM_OK) {
					Jim_SetResultFormatted(goi.interp, "Unable to read reg name.");
					goto fail;
				}

				if (name_len > REG_TYPE_MAX_NAME_LENGTH) {
					Jim_SetResultFormatted(goi.interp, "Reg type name is too big.");
					goto fail;
				}

				strncpy((void *)type->data_type.id, name, name_len);
				if (!type->data_type.id) {
					Jim_SetResultFormatted(goi.interp, "Unable to setup reg type name.");
					goto fail;
				}

				break;
			}

			case CFG_ADD_REG_TYPE_FLAGS_FLAG:
			{
				const char *field_name = NULL;
				int field_name_len = 0;

				e = jim_arc_read_reg_type_field(&goi, &field_name, &field_name_len, bitfields,
									cur_field, CFG_ADD_REG_TYPE_FLAG);
				if (e != JIM_OK) {
					Jim_SetResultFormatted(goi.interp, "Unable to add reg_type_flag field.");
					goto fail;
				}

				if (field_name_len > REG_TYPE_MAX_NAME_LENGTH) {
					Jim_SetResultFormatted(goi.interp, "Reg type field_name_len is too big.");
					goto fail;
				}

				fields[cur_field].name = bitfields[cur_field].name;
				strncpy(bitfields[cur_field].name, field_name, field_name_len);
				if (!fields[cur_field].name) {
					Jim_SetResultFormatted(goi.interp, "Unable to setup field name. ");
					goto fail;
				}

				fields[cur_field].bitfield = &(bitfields[cur_field].bitfield);
				if (cur_field > 0)
					fields[cur_field - 1].next = &(fields[cur_field]);
				else
					flags->fields = fields;

				cur_field += 1;
				break;
			}
		}
	}

	if (!type->data_type.id) {
		Jim_SetResultFormatted(goi.interp, "-name is a required option");
		goto fail;
	}

	arc_reg_data_type_add(target, type);

	LOG_DEBUG("added flags type {name=%s}", type->data_type.id);

	return JIM_OK;
fail:
	free(type);
	free(fields);
	free(bitfields);

	return JIM_ERR;
}

/* Add struct register data type */
enum add_reg_type_struct {
	CFG_ADD_REG_TYPE_STRUCT_NAME,
	CFG_ADD_REG_TYPE_STRUCT_BITFIELD,
};

static struct jim_nvp nvp_add_reg_type_struct_opts[] = {
	{ .name = "-name",     .value = CFG_ADD_REG_TYPE_STRUCT_NAME },
	{ .name = "-bitfield", .value = CFG_ADD_REG_TYPE_STRUCT_BITFIELD },
	{ .name = NULL,     .value = -1 }
};

static int jim_arc_set_aux_reg(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{

	struct command_context *context;
	struct target *target;
	uint32_t regnum;
	uint32_t value;

	struct jim_getopt_info goi;
	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	if (goi.argc != 2) {
		Jim_SetResultFormatted(goi.interp,
			"usage: %s <aux_reg_num> <aux_reg_value>", Jim_GetString(argv[0], NULL));
		return JIM_ERR;
	}

	context = current_command_context(interp);
	assert(context);

	target = get_current_target(context);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	/* Register number */
	JIM_CHECK_RETVAL(arc_cmd_jim_get_uint32(&goi, &regnum));

	/* Register value */
	JIM_CHECK_RETVAL(arc_cmd_jim_get_uint32(&goi, &value));

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, regnum, value));

	return ERROR_OK;
}

static int jim_arc_get_aux_reg(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct command_context *context;
	struct target *target;
	uint32_t regnum;
	uint32_t value;

	struct jim_getopt_info goi;
	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	if (goi.argc != 1) {
		Jim_SetResultFormatted(goi.interp,
			"usage: %s <aux_reg_num>", Jim_GetString(argv[0], NULL));
		return JIM_ERR;
	}

	context = current_command_context(interp);
	assert(context);

	target = get_current_target(context);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	/* Register number */
	JIM_CHECK_RETVAL(arc_cmd_jim_get_uint32(&goi, &regnum));

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, regnum, &value));
	Jim_SetResultInt(interp, value);

	return ERROR_OK;
}

static int jim_arc_get_core_reg(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct command_context *context;
	struct target *target;
	uint32_t regnum;
	uint32_t value;

	struct jim_getopt_info goi;
	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	if (goi.argc != 1) {
		Jim_SetResultFormatted(goi.interp,
			"usage: %s <core_reg_num>", Jim_GetString(argv[0], NULL));
		return JIM_ERR;
	}

	context = current_command_context(interp);
	assert(context);

	target = get_current_target(context);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	/* Register number */
	JIM_CHECK_RETVAL(arc_cmd_jim_get_uint32(&goi, &regnum));
	if (regnum > CORE_REG_MAX_NUMBER || regnum == ARC_R61 || regnum == ARC_R62) {
		Jim_SetResultFormatted(goi.interp, "Core register number %i "
			"is invalid. Must less then 64 and not 61 and 62.", regnum);
		return JIM_ERR;
	}

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	/* Read value */
	CHECK_RETVAL(arc_jtag_read_core_reg_one(&arc->jtag_info, regnum, &value));
	Jim_SetResultInt(interp, value);

	return ERROR_OK;
}

static int jim_arc_set_core_reg(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct command_context *context;
	struct target *target;
	uint32_t regnum;
	uint32_t value;

	struct jim_getopt_info goi;
	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	if (goi.argc != 2) {
		Jim_SetResultFormatted(goi.interp,
			"usage: %s <core_reg_num> <core_reg_value>", Jim_GetString(argv[0], NULL));
		return JIM_ERR;
	}

	context = current_command_context(interp);
	assert(context);

	target = get_current_target(context);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	/* Register number */
	JIM_CHECK_RETVAL(arc_cmd_jim_get_uint32(&goi, &regnum));
	if (regnum > CORE_REG_MAX_NUMBER || regnum == ARC_R61 || regnum == ARC_R62) {
		Jim_SetResultFormatted(goi.interp, "Core register number %i "
			"is invalid. Must less then 64 and not 61 and 62.", regnum);
		return JIM_ERR;
	}

	/* Register value */
	JIM_CHECK_RETVAL(arc_cmd_jim_get_uint32(&goi, &value));

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	CHECK_RETVAL(arc_jtag_write_core_reg_one(&arc->jtag_info, regnum, value));

	return ERROR_OK;
}

static const struct command_registration arc_jtag_command_group[] = {
	{
		.name = "get-aux-reg",
		.jim_handler = jim_arc_get_aux_reg,
		.mode = COMMAND_EXEC,
		.help = "Get AUX register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum>"
	},
	{
		.name = "set-aux-reg",
		.jim_handler = jim_arc_set_aux_reg,
		.mode = COMMAND_EXEC,
		.help = "Set AUX register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum> <value>"
	},
	{
		.name = "get-core-reg",
		.jim_handler = jim_arc_get_core_reg,
		.mode = COMMAND_EXEC,
		.help = "Get/Set core register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum> [<value>]"
	},
	{
		.name = "set-core-reg",
		.jim_handler = jim_arc_set_core_reg,
		.mode = COMMAND_EXEC,
		.help = "Get/Set core register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum> [<value>]"
	},
	COMMAND_REGISTRATION_DONE
};


/* This function supports only bitfields. */
static int jim_arc_add_reg_type_struct(Jim_Interp *interp, int argc,
	Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	LOG_DEBUG("-");

	struct command_context *ctx;
	struct target *target;

	ctx = current_command_context(interp);
	assert(ctx);
	target = get_current_target(ctx);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	int e = JIM_OK;

	/* Check if the amount of arguments is not zero */
	if (goi.argc <= 0) {
		Jim_SetResultFormatted(goi.interp, "The command has no arguments");
		return JIM_ERR;
	}

	/* Estimate number of registers as (argc - 2)/4 as each -bitfield option has 3
	 * arguments while -name is required. */
	unsigned int fields_sz = (goi.argc - 2) / 4;
	unsigned int cur_field = 0;

	/* The maximum amount of bitfields is 32 */
	if (fields_sz > 32) {
			Jim_SetResultFormatted(goi.interp, "The amount of bitfields exceed 32");
			return JIM_ERR;
	}

	struct arc_reg_data_type *type = calloc(1, sizeof(*type));
	struct reg_data_type_struct *struct_type = &type->data_type_struct;
	struct reg_data_type_struct_field *fields = calloc(fields_sz, sizeof(*fields));
	type->reg_type_struct_field = fields;
	struct arc_reg_bitfield *bitfields = calloc(fields_sz, sizeof(*bitfields));
	if (!(type && fields && bitfields)) {
		Jim_SetResultFormatted(goi.interp, "Failed to allocate memory.");
		goto fail;
	}

	/* Initialize type */
	type->data_type.id = type->data_type_id;
	type->bitfields = bitfields;
	type->data_type.type = REG_TYPE_ARCH_DEFINED;
	type->data_type.type_class = REG_TYPE_CLASS_STRUCT;
	type->data_type.reg_type_struct = struct_type;
	struct_type->size = 4; /* For now ARC has only 32-bit registers */

	while (goi.argc > 0 && e == JIM_OK) {
		struct jim_nvp *n;
		e = jim_getopt_nvp(&goi, nvp_add_reg_type_struct_opts, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(&goi, nvp_add_reg_type_struct_opts, 0);
			continue;
		}

		switch (n->value) {
			case CFG_ADD_REG_TYPE_STRUCT_NAME:
			{
				const char *name = NULL;
				int name_len = 0;

				e = jim_arc_read_reg_name_field(&goi, &name, &name_len);
				if (e != JIM_OK) {
					Jim_SetResultFormatted(goi.interp, "Unable to read reg name.");
					goto fail;
				}

				if (name_len > REG_TYPE_MAX_NAME_LENGTH) {
					Jim_SetResultFormatted(goi.interp, "Reg type name is too big.");
					goto fail;
				}

				strncpy((void *)type->data_type.id, name, name_len);
				if (!type->data_type.id) {
					Jim_SetResultFormatted(goi.interp, "Unable to setup reg type name.");
					goto fail;
				}

				break;
			}
			case CFG_ADD_REG_TYPE_STRUCT_BITFIELD:
			{
				const char *field_name = NULL;
				int field_name_len = 0;
				e = jim_arc_read_reg_type_field(&goi, &field_name, &field_name_len, bitfields,
									cur_field, CFG_ADD_REG_TYPE_STRUCT);
				if (e != JIM_OK) {
					Jim_SetResultFormatted(goi.interp, "Unable to add reg_type_struct field.");
					goto fail;
				}

				if (field_name_len > REG_TYPE_MAX_NAME_LENGTH) {
					Jim_SetResultFormatted(goi.interp, "Reg type field_name_len is too big.");
					goto fail;
				}

				fields[cur_field].name = bitfields[cur_field].name;
				strncpy(bitfields[cur_field].name, field_name, field_name_len);
				if (!fields[cur_field].name) {
					Jim_SetResultFormatted(goi.interp, "Unable to setup field name. ");
					goto fail;
				}

				fields[cur_field].bitfield = &(bitfields[cur_field].bitfield);
				fields[cur_field].use_bitfields = true;
				if (cur_field > 0)
					fields[cur_field - 1].next = &(fields[cur_field]);
				else
					struct_type->fields = fields;

				cur_field += 1;

				break;
			}
		}
	}

	if (!type->data_type.id) {
		Jim_SetResultFormatted(goi.interp, "-name is a required option");
		goto fail;
	}

	arc_reg_data_type_add(target, type);
	LOG_DEBUG("added struct type {name=%s}", type->data_type.id);
	return JIM_OK;

fail:
			free(type);
			free(fields);
			free(bitfields);

			return JIM_ERR;
}

/* Add register */
enum opts_add_reg {
	CFG_ADD_REG_NAME,
	CFG_ADD_REG_ARCH_NUM,
	CFG_ADD_REG_IS_CORE,
	CFG_ADD_REG_IS_BCR,
	CFG_ADD_REG_GDB_FEATURE,
	CFG_ADD_REG_TYPE,
	CFG_ADD_REG_GENERAL,
};

static struct jim_nvp opts_nvp_add_reg[] = {
	{ .name = "-name",    .value = CFG_ADD_REG_NAME },
	{ .name = "-num",     .value = CFG_ADD_REG_ARCH_NUM },
	{ .name = "-core",    .value = CFG_ADD_REG_IS_CORE },
	{ .name = "-bcr",     .value = CFG_ADD_REG_IS_BCR },
	{ .name = "-feature", .value = CFG_ADD_REG_GDB_FEATURE },
	{ .name = "-type",    .value = CFG_ADD_REG_TYPE },
	{ .name = "-g",       .value = CFG_ADD_REG_GENERAL },
	{ .name = NULL,       .value = -1 }
};

void free_reg_desc(struct arc_reg_desc *r)
{
	free(r->name);
	free(r->gdb_xml_feature);
	free(r);
}

static int jim_arc_add_reg(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	struct arc_reg_desc *reg = calloc(1, sizeof(*reg));
	if (!reg) {
		Jim_SetResultFormatted(goi.interp, "Failed to allocate memory.");
		return JIM_ERR;
	}

	/* There is no architecture number that we could treat as invalid, so
	 * separate variable required to ensure that arch num has been set. */
	bool arch_num_set = false;
	const char *type_name = "int"; /* Default type */
	int type_name_len = strlen(type_name);
	int e = ERROR_OK;

	/* At least we need to specify 4 parameters: name, number and gdb_feature,
	 * which means there should be 6 arguments. Also there can be additional parameters
	 * "-type <type>", "-g" and  "-core" or "-bcr" which makes maximum 10 parameters. */
	if (goi.argc < 6 || goi.argc > 10) {
		free_reg_desc(reg);
		Jim_SetResultFormatted(goi.interp,
			"Should be at least 6 arguments and not greater than 10: "
			" -name <name> -num <num> -feature <gdb_feature> "
			" [-type <type_name>] [-core|-bcr] [-g].");
		return JIM_ERR;
	}

	/* Parse options. */
	while (goi.argc > 0) {
		struct jim_nvp *n;
		e = jim_getopt_nvp(&goi, opts_nvp_add_reg, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(&goi, opts_nvp_add_reg, 0);
			free_reg_desc(reg);
			return e;
		}

		switch (n->value) {
			case CFG_ADD_REG_NAME:
			{
				const char *reg_name = NULL;
				int reg_name_len = 0;

				e = jim_arc_read_reg_name_field(&goi, &reg_name, &reg_name_len);
				if (e != JIM_OK) {
					Jim_SetResultFormatted(goi.interp, "Unable to read register name.");
					free_reg_desc(reg);
					return e;
				}

				reg->name = strndup(reg_name, reg_name_len);
				break;
			}
			case CFG_ADD_REG_IS_CORE:
				reg->is_core = true;
				break;
			case CFG_ADD_REG_IS_BCR:
				reg->is_bcr = true;
				break;
			case CFG_ADD_REG_ARCH_NUM:
			{
				jim_wide archnum;

				if (!goi.argc) {
					free_reg_desc(reg);
					Jim_WrongNumArgs(interp, goi.argc, goi.argv, "-num <int> ...");
					return JIM_ERR;
				}

				e = jim_getopt_wide(&goi, &archnum);
				if (e != JIM_OK) {
					free_reg_desc(reg);
					return e;
				}

				reg->arch_num = archnum;
				arch_num_set = true;
				break;
			}
			case CFG_ADD_REG_GDB_FEATURE:
			{
				const char *feature = NULL;
				int feature_len = 0;

				e = jim_arc_read_reg_name_field(&goi, &feature, &feature_len);
				if (e != JIM_OK) {
					Jim_SetResultFormatted(goi.interp, "Unable to read gdb_feature.");
					free_reg_desc(reg);
					return e;
				}

				reg->gdb_xml_feature = strndup(feature, feature_len);
				break;
			}
			case CFG_ADD_REG_TYPE:
				e = jim_arc_read_reg_name_field(&goi, &type_name, &type_name_len);
				if (e != JIM_OK) {
					Jim_SetResultFormatted(goi.interp, "Unable to read register type.");
					free_reg_desc(reg);
					return e;
				}

				break;
			case CFG_ADD_REG_GENERAL:
				reg->is_general = true;
				break;
			default:
				LOG_DEBUG("Error: Unknown parameter");
				free_reg_desc(reg);
				return JIM_ERR;
		}
	}

	/* Check that required fields are set */
	const char * const errmsg = validate_register(reg, arch_num_set);
	if (errmsg) {
		Jim_SetResultFormatted(goi.interp, errmsg);
		free_reg_desc(reg);
		return JIM_ERR;
	}

	/* Add new register */
	struct command_context *ctx;
	struct target *target;

	ctx = current_command_context(interp);
	assert(ctx);
	target = get_current_target(ctx);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		free_reg_desc(reg);
		return JIM_ERR;
	}

	reg->target = target;

	e = arc_reg_add(target, reg, type_name, type_name_len);
	if (e == ERROR_ARC_REGTYPE_NOT_FOUND) {
		Jim_SetResultFormatted(goi.interp,
			"Cannot find type `%s' for register `%s'.",
			type_name, reg->name);
		free_reg_desc(reg);
		return JIM_ERR;
	}

	return e;
}

/* arc set-reg-exists ($reg_name)+
 * Accepts any amount of register names - will set them as existing in a loop.*/
COMMAND_HANDLER(arc_set_reg_exists)
{
	struct target * const target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "Unable to get current target.");
		return JIM_ERR;
	}

	if (!CMD_ARGC) {
		command_print(CMD, "At least one register name must be specified.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		const char * const reg_name = CMD_ARGV[i];
		struct reg * const r = arc_reg_get_by_name(target->reg_cache, reg_name, true);

		if (!r) {
			command_print(CMD, "Register `%s' is not found.", reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}

		r->exist = true;
	}

	return JIM_OK;
}

/* arc reg-field  ($reg_name) ($reg_field)
 * Reads struct type register field */
static int jim_arc_get_reg_field(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	const char *reg_name, *field_name;
	uint32_t value;
	int retval;

	JIM_CHECK_RETVAL(jim_getopt_setup(&goi, interp, argc-1, argv+1));

	LOG_DEBUG("Reading register field");
	if (goi.argc != 2) {
		if (!goi.argc)
			Jim_WrongNumArgs(interp, goi.argc, goi.argv, "<regname> <fieldname>");
		else if (goi.argc == 1)
			Jim_WrongNumArgs(interp, goi.argc, goi.argv, "<fieldname>");
		else
			Jim_WrongNumArgs(interp, goi.argc, goi.argv, "<regname> <fieldname>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	JIM_CHECK_RETVAL(jim_getopt_string(&goi, &reg_name, NULL));
	JIM_CHECK_RETVAL(jim_getopt_string(&goi, &field_name, NULL));
	assert(reg_name);
	assert(field_name);

	struct command_context * const ctx = current_command_context(interp);
	assert(ctx);
	struct target * const target = get_current_target(ctx);
	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	retval = arc_reg_get_field(target, reg_name, field_name, &value);

	switch (retval) {
		case ERROR_OK:
			break;
		case ERROR_ARC_REGISTER_NOT_FOUND:
			Jim_SetResultFormatted(goi.interp,
				"Register `%s' has not been found.", reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		case ERROR_ARC_REGISTER_IS_NOT_STRUCT:
			Jim_SetResultFormatted(goi.interp,
				"Register `%s' must have 'struct' type.", reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		case ERROR_ARC_REGISTER_FIELD_NOT_FOUND:
			Jim_SetResultFormatted(goi.interp,
				"Field `%s' has not been found in register `%s'.",
				field_name, reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		case ERROR_ARC_FIELD_IS_NOT_BITFIELD:
			Jim_SetResultFormatted(goi.interp,
				"Field `%s' is not a 'bitfield' field in a structure.",
				field_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		default:
			/* Pass through other errors. */
			return retval;
	}

	Jim_SetResultInt(interp, value);

	return JIM_OK;
}

COMMAND_HANDLER(arc_l1_cache_disable_auto_cmd)
{
	bool value;
	int retval = 0;
	struct arc_common *arc = target_to_arc(get_current_target(CMD_CTX));
	retval = CALL_COMMAND_HANDLER(handle_command_parse_bool,
		&value, "target has caches enabled");
	arc->has_l2cache = value;
	arc->has_dcache = value;
	arc->has_icache = value;
	return retval;
}

COMMAND_HANDLER(arc_l2_cache_disable_auto_cmd)
{
	struct arc_common *arc = target_to_arc(get_current_target(CMD_CTX));
	return CALL_COMMAND_HANDLER(handle_command_parse_bool,
		&arc->has_l2cache, "target has l2 cache enabled");
}

static int jim_handle_actionpoints_num(Jim_Interp *interp, int argc,
	Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc - 1, argv + 1);

	LOG_DEBUG("-");

	if (goi.argc >= 2) {
		Jim_WrongNumArgs(interp, goi.argc, goi.argv, "[<unsigned integer>]");
		return JIM_ERR;
	}

	struct command_context *context = current_command_context(interp);
	assert(context);

	struct target *target = get_current_target(context);

	if (!target) {
		Jim_SetResultFormatted(goi.interp, "No current target");
		return JIM_ERR;
	}

	struct arc_common *arc = target_to_arc(target);
	/* It is not possible to pass &arc->actionpoints_num directly to
	 * handle_command_parse_uint, because this value should be valid during
	 * "actionpoint reset, initiated by arc_set_actionpoints_num.  */
	uint32_t ap_num = arc->actionpoints_num;

	if (goi.argc == 1) {
		JIM_CHECK_RETVAL(arc_cmd_jim_get_uint32(&goi, &ap_num));
		int e = arc_set_actionpoints_num(target, ap_num);
		if (e != ERROR_OK) {
			Jim_SetResultFormatted(goi.interp,
				"Failed to set number of actionpoints");
			return JIM_ERR;
		}
	}

	Jim_SetResultInt(interp, ap_num);

	return JIM_OK;
}

/* ----- Exported target commands ------------------------------------------ */

const struct command_registration arc_l2_cache_group_handlers[] = {
	{
		.name = "auto",
		.handler = arc_l2_cache_disable_auto_cmd,
		.mode = COMMAND_ANY,
		.usage = "(1|0)",
		.help = "Disable or enable L2",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arc_cache_group_handlers[] = {
	{
		.name = "auto",
		.handler = arc_l1_cache_disable_auto_cmd,
		.mode = COMMAND_ANY,
		.help = "Disable or enable L1",
		.usage = "(1|0)",
	},
	{
		.name = "l2",
		.mode = COMMAND_ANY,
		.help = "L2 cache command group",
		.usage = "",
		.chain = arc_l2_cache_group_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


static const struct command_registration arc_core_command_handlers[] = {
	{
		.name = "add-reg-type-flags",
		.jim_handler = jim_arc_add_reg_type_flags,
		.mode = COMMAND_CONFIG,
		.usage = "-name <string> -flag <name> <position> "
			"[-flag <name> <position>]...",
		.help = "Add new 'flags' register data type. Only single bit flags "
			"are supported. Type name is global. Bitsize of register is fixed "
			"at 32 bits.",
	},
	{
		.name = "add-reg-type-struct",
		.jim_handler = jim_arc_add_reg_type_struct,
		.mode = COMMAND_CONFIG,
		.usage = "-name <string> -bitfield <name> <start> <end> "
			"[-bitfield <name> <start> <end>]...",
		.help = "Add new 'struct' register data type. Only bit-fields are "
			"supported so far, which means that for each bitfield start and end "
			"position bits must be specified. GDB also support type-fields, "
			"where common type can be used instead. Type name is global. Bitsize of "
			"register is fixed at 32 bits.",
	},
	{
		.name = "add-reg",
		.jim_handler = jim_arc_add_reg,
		.mode = COMMAND_CONFIG,
		.usage = "-name <string> -num <int> -feature <string> [-gdbnum <int>] "
			"[-core|-bcr] [-type <type_name>] [-g]",
		.help = "Add new register. Name, architectural number and feature name "
			"are required options. GDB regnum will default to previous register "
			"(gdbnum + 1) and shouldn't be specified in most cases. Type "
			"defaults to default GDB 'int'.",
	},
	{
		.name = "set-reg-exists",
		.handler = arc_set_reg_exists,
		.mode = COMMAND_ANY,
		.usage = "<register-name> [<register-name>]...",
		.help = "Set that register exists. Accepts multiple register names as "
			"arguments.",
	},
	{
		.name = "get-reg-field",
		.jim_handler = jim_arc_get_reg_field,
		.mode = COMMAND_ANY,
		.usage = "<regname> <field_name>",
		.help = "Returns value of field in a register with 'struct' type.",
	},
	{
		.name = "jtag",
		.mode = COMMAND_ANY,
		.help = "ARC JTAG specific commands",
		.usage = "",
		.chain = arc_jtag_command_group,
	},
	{
		.name = "cache",
		.mode = COMMAND_ANY,
		.help = "cache command group",
		.usage = "",
		.chain = arc_cache_group_handlers,
	},
	{
		.name = "num-actionpoints",
		.jim_handler = jim_handle_actionpoints_num,
		.mode = COMMAND_ANY,
		.usage = "[<unsigned integer>]",
		.help = "Prints or sets amount of actionpoints in the processor.",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arc_monitor_command_handlers[] = {
	{
		.name = "arc",
		.mode = COMMAND_ANY,
		.help = "ARC monitor command group",
		.usage = "",
		.chain = arc_core_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
