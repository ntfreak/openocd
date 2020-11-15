/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "arm.h"
#include "etm.h"
#include "etb.h"
#include "image.h"
#include "arm_disassembler.h"
#include "register.h"
#include "etm_dummy.h"

#if BUILD_OOCD_TRACE == 1
#include "oocd_trace.h"
#endif


/*
 * ARM "Embedded Trace Macrocell" (ETM) support -- direct JTAG access.
 *
 * ETM modules collect instruction and/or data trace information, compress
 * it, and transfer it to a debugging host through either a (buffered) trace
 * port (often a 38-pin Mictor connector) or an Embedded Trace Buffer (ETB).
 *
 * There are several generations of these modules.  Original versions have
 * JTAG access through a dedicated scan chain.  Recent versions have added
 * access via coprocessor instructions, memory addressing, and the ARM Debug
 * Interface v5 (ADIv5); and phased out direct JTAG access.
 *
 * This code supports up to the ETMv1.3 architecture, as seen in ETM9 and
 * most common ARM9 systems.  Note: "CoreSight ETM9" implements ETMv3.2,
 * implying non-JTAG connectivity options.
 *
 * Relevant documentation includes:
 *  ARM DDI 0157G ... ETM9 (r2p2) Technical Reference Manual
 *  ARM DDI 0315B ... CoreSight ETM9 (r0p1) Technical Reference Manual
 *  ARM IHI 0014O ... Embedded Trace Macrocell, Architecture Specification
 */

enum {
	RO,				/* read/only */
	WO,				/* write/only */
	RW,				/* read/write */
};

struct etm_reg_info {
	uint8_t addr;
	uint8_t size;			/* low-N of 32 bits */
	uint8_t mode;			/* RO, WO, RW */
	uint8_t bcd_vers;		/* 1.0, 2.0, etc */
	const char *name;
};

/*
 * Registers 0..0x7f are JTAG-addressable using scanchain 6.
 * (Or on some processors, through coprocessor operations.)
 * Newer versions of ETM make some W/O registers R/W, and
 * provide definitions for some previously-unused bits.
 */

/* core registers used to version/configure the ETM */
static const struct etm_reg_info etm_core[] = {
	/* NOTE: we "know" the order here ... */
	{ ETM_CONFIG, 32, RO, 0x10, "ETM_config", },
	{ ETM_ID, 32, RO, 0x20, "ETM_id", },
};

/* basic registers that are always there given the right ETM version */
static const struct etm_reg_info etm_basic[] = {
	/* ETM Trace Registers */
	{ ETM_CTRL, 32, RW, 0x10, "ETM_ctrl", },
	{ ETM_TRIG_EVENT, 17, WO, 0x10, "ETM_trig_event", },
	{ ETM_ASIC_CTRL,  8, WO, 0x10, "ETM_asic_ctrl", },
	{ ETM_STATUS,  3, RO, 0x11, "ETM_status", },
	{ ETM_SYS_CONFIG,  9, RO, 0x12, "ETM_sys_config", },

	/* TraceEnable configuration */
	{ ETM_TRACE_RESOURCE_CTRL, 32, WO, 0x12, "ETM_trace_resource_ctrl", },
	{ ETM_TRACE_EN_CTRL2, 16, WO, 0x12, "ETM_trace_en_ctrl2", },
	{ ETM_TRACE_EN_EVENT, 17, WO, 0x10, "ETM_trace_en_event", },
	{ ETM_TRACE_EN_CTRL1, 26, WO, 0x10, "ETM_trace_en_ctrl1", },

	/* ViewData configuration (data trace) */
	{ ETM_VIEWDATA_EVENT, 17, WO, 0x10, "ETM_viewdata_event", },
	{ ETM_VIEWDATA_CTRL1, 32, WO, 0x10, "ETM_viewdata_ctrl1", },
	{ ETM_VIEWDATA_CTRL2, 32, WO, 0x10, "ETM_viewdata_ctrl2", },
	{ ETM_VIEWDATA_CTRL3, 17, WO, 0x10, "ETM_viewdata_ctrl3", },

	/* REVISIT exclude VIEWDATA_CTRL2 when it's not there */

	{ 0x78, 12, WO, 0x20, "ETM_sync_freq", },
	{ 0x7a, 22, RO, 0x31, "ETM_config_code_ext", },
	{ 0x7b, 32, WO, 0x31, "ETM_ext_input_select", },
	{ 0x7c, 32, WO, 0x34, "ETM_trace_start_stop", },
	{ 0x7d, 8, WO, 0x34, "ETM_behavior_control", },
};

static const struct etm_reg_info etm_fifofull[] = {
	/* FIFOFULL configuration */
	{ ETM_FIFOFULL_REGION, 25, WO, 0x10, "ETM_fifofull_region", },
	{ ETM_FIFOFULL_LEVEL,  8, WO, 0x10, "ETM_fifofull_level", },
};

static const struct etm_reg_info etm_addr_comp[] = {
	/* Address comparator register pairs */
#define ADDR_COMPARATOR(i) \
		{ ETM_ADDR_COMPARATOR_VALUE + (i) - 1, 32, WO, 0x10, \
				"ETM_addr_" #i "_comparator_value", }, \
		{ ETM_ADDR_ACCESS_TYPE + (i) - 1,  7, WO, 0x10, \
				"ETM_addr_" #i "_access_type", }
	ADDR_COMPARATOR(1),
	ADDR_COMPARATOR(2),
	ADDR_COMPARATOR(3),
	ADDR_COMPARATOR(4),
	ADDR_COMPARATOR(5),
	ADDR_COMPARATOR(6),
	ADDR_COMPARATOR(7),
	ADDR_COMPARATOR(8),

	ADDR_COMPARATOR(9),
	ADDR_COMPARATOR(10),
	ADDR_COMPARATOR(11),
	ADDR_COMPARATOR(12),
	ADDR_COMPARATOR(13),
	ADDR_COMPARATOR(14),
	ADDR_COMPARATOR(15),
	ADDR_COMPARATOR(16),
	{ 0, 0, 0, 0, NULL }
#undef ADDR_COMPARATOR
};

static const struct etm_reg_info etm_data_comp[] = {
	/* Data Value Comparators (NOTE: odd addresses are reserved) */
#define DATA_COMPARATOR(i) \
		{ ETM_DATA_COMPARATOR_VALUE + 2*(i) - 1, 32, WO, 0x10, \
				"ETM_data_" #i "_comparator_value", }, \
		{ ETM_DATA_COMPARATOR_MASK + 2*(i) - 1, 32, WO, 0x10, \
				"ETM_data_" #i "_comparator_mask", }
	DATA_COMPARATOR(1),
	DATA_COMPARATOR(2),
	DATA_COMPARATOR(3),
	DATA_COMPARATOR(4),
	DATA_COMPARATOR(5),
	DATA_COMPARATOR(6),
	DATA_COMPARATOR(7),
	DATA_COMPARATOR(8),
	{ 0, 0, 0, 0, NULL }
#undef DATA_COMPARATOR
};

static const struct etm_reg_info etm_counters[] = {
#define ETM_COUNTER(i) \
		{ ETM_COUNTER_RELOAD_VALUE + (i) - 1, 16, WO, 0x10, \
				"ETM_counter_" #i "_reload_value", }, \
		{ ETM_COUNTER_ENABLE + (i) - 1, 18, WO, 0x10, \
				"ETM_counter_" #i "_enable", }, \
		{ ETM_COUNTER_RELOAD_EVENT + (i) - 1, 17, WO, 0x10, \
				"ETM_counter_" #i "_reload_event", }, \
		{ ETM_COUNTER_VALUE + (i) - 1, 16, RO, 0x10, \
				"ETM_counter_" #i "_value", }
	ETM_COUNTER(1),
	ETM_COUNTER(2),
	ETM_COUNTER(3),
	ETM_COUNTER(4),
	{ 0, 0, 0, 0, NULL }
#undef ETM_COUNTER
};

static const struct etm_reg_info etm_sequencer[] = {
#define ETM_SEQ(i) \
		{ ETM_SEQUENCER_EVENT + (i), 17, WO, 0x10, \
				"ETM_sequencer_event" #i, }
	ETM_SEQ(0),				/* 1->2 */
	ETM_SEQ(1),				/* 2->1 */
	ETM_SEQ(2),				/* 2->3 */
	ETM_SEQ(3),				/* 3->1 */
	ETM_SEQ(4),				/* 3->2 */
	ETM_SEQ(5),				/* 1->3 */
#undef ETM_SEQ
	/* 0x66 reserved */
	{ ETM_SEQUENCER_STATE,  2, RO, 0x10, "ETM_sequencer_state", },
};

static const struct etm_reg_info etm_outputs[] = {
#define ETM_OUTPUT(i) \
		{ ETM_EXTERNAL_OUTPUT + (i) - 1, 17, WO, 0x10, \
				"ETM_external_output" #i, }

	ETM_OUTPUT(1),
	ETM_OUTPUT(2),
	ETM_OUTPUT(3),
	ETM_OUTPUT(4),
	{ 0, 0, 0, 0, NULL }
#undef ETM_OUTPUT
};

#if 0
	/* registers from 0x6c..0x7f were added after ETMv1.3 */

	/* Context ID Comparators */
	{ 0x6c, 32, RO, 0x20, "ETM_contextid_comparator_value1", }
	{ 0x6d, 32, RO, 0x20, "ETM_contextid_comparator_value2", }
	{ 0x6e, 32, RO, 0x20, "ETM_contextid_comparator_value3", }
	{ 0x6f, 32, RO, 0x20, "ETM_contextid_comparator_mask", }
#endif

static int etm_get_reg(struct reg *reg);
static int etm_read_reg_w_check(struct reg *reg,
	uint8_t *check_value, uint8_t *check_mask);
static int etm_register_user_commands(struct command_context *cmd_ctx);
static int etm_set_reg_w_exec(struct reg *reg, uint8_t *buf);
static int etm_write_reg(struct reg *reg, uint32_t value);

static const struct reg_arch_type etm_scan6_type = {
	.get = etm_get_reg,
	.set = etm_set_reg_w_exec,
};

/* Look up register by ID ... most ETM instances only
 * support a subset of the possible registers.
 */
static struct reg *etm_reg_lookup(struct etm_context *etm_ctx, unsigned id)
{
	struct reg_cache *cache = etm_ctx->reg_cache;
	unsigned i;

	for (i = 0; i < cache->num_regs; i++) {
		struct etm_reg *reg = cache->reg_list[i].arch_info;

		if (reg->reg_info->addr == id)
			return &cache->reg_list[i];
	}

	/* caller asking for nonexistent register is a bug!
	 * REVISIT say which of the N targets was involved */
	LOG_ERROR("ETM: register 0x%02x not available", id);
	return NULL;
}

static void etm_reg_add(unsigned bcd_vers, struct arm_jtag *jtag_info,
	struct reg_cache *cache, struct etm_reg *ereg,
	const struct etm_reg_info *r, unsigned nreg)
{
	struct reg *reg = cache->reg_list;

	reg += cache->num_regs;
	ereg += cache->num_regs;

	/* add up to "nreg" registers from "r", if supported by this
	 * version of the ETM, to the specified cache.
	 */
	for (; nreg--; r++) {
		/* No more registers to add */
		if (!r->size) {
			LOG_ERROR("etm_reg_add is requested to add non-existing registers, ETM config might be bogus");
			return;
		}

		/* this ETM may be too old to have some registers */
		if (r->bcd_vers > bcd_vers)
			continue;

		reg->name = r->name;
		reg->size = r->size;
		reg->value = ereg->value;
		reg->arch_info = ereg;
		reg->type = &etm_scan6_type;
		reg++;
		cache->num_regs++;

		ereg->reg_info = r;
		ereg->jtag_info = jtag_info;
		ereg++;
	}
}

struct reg_cache *etm_build_reg_cache(struct target *target,
	struct arm_jtag *jtag_info, struct etm_context *etm_ctx)
{
	struct reg_cache *reg_cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = NULL;
	struct etm_reg *arch_info = NULL;
	unsigned bcd_vers, config;

	/* the actual registers are kept in two arrays */
	reg_list = calloc(128, sizeof(struct reg));
	arch_info = calloc(128, sizeof(struct etm_reg));

	if (reg_cache == NULL || reg_list == NULL || arch_info == NULL) {
		LOG_ERROR("No memory");
		goto fail;
	}

	/* fill in values for the reg cache */
	reg_cache->name = "etm registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = 0;

	/* add ETM_CONFIG, then parse its values to see
	 * which other registers exist in this ETM
	 */
	etm_reg_add(0x10, jtag_info, reg_cache, arch_info,
		etm_core, 1);

	etm_get_reg(reg_list);
	etm_ctx->config = buf_get_u32(arch_info->value, 0, 32);
	config = etm_ctx->config;

	/* figure ETM version then add base registers */
	if (config & (1 << 31)) {
		LOG_WARNING("ETMv2+ support is incomplete");

		/* REVISIT more registers may exist; they may now be
		 * readable; more register bits have defined meanings;
		 * don't presume trace start/stop support is present;
		 * and include any context ID comparator registers.
		 */
		etm_reg_add(0x20, jtag_info, reg_cache, arch_info,
			etm_core + 1, 1);
		etm_get_reg(reg_list + 1);
		etm_ctx->id = buf_get_u32(
				arch_info[1].value, 0, 32);
		LOG_DEBUG("ETM ID: %08x", (unsigned) etm_ctx->id);
		bcd_vers = 0x10 + (((etm_ctx->id) >> 4) & 0xff);

	} else {
		switch (config >> 28) {
			case 7:
			case 5:
			case 3:
				bcd_vers = 0x13;
				break;
			case 4:
			case 2:
				bcd_vers = 0x12;
				break;
			case 1:
				bcd_vers = 0x11;
				break;
			case 0:
				bcd_vers = 0x10;
				break;
			default:
				LOG_WARNING("Bad ETMv1 protocol %d", config >> 28);
				goto fail;
		}
	}
	etm_ctx->bcd_vers = bcd_vers;
	LOG_INFO("ETM v%d.%d", bcd_vers >> 4, bcd_vers & 0xf);

	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
		etm_basic, ARRAY_SIZE(etm_basic));

	/* address and data comparators; counters; outputs */
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
		etm_addr_comp, 4 * (0x0f & (config >> 0)));
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
		etm_data_comp, 2 * (0x0f & (config >> 4)));
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
		etm_counters, 4 * (0x07 & (config >> 13)));
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
		etm_outputs, (0x07 & (config >> 20)));

	/* FIFOFULL presence is optional
	 * REVISIT for ETMv1.2 and later, don't bother adding this
	 * unless ETM_SYS_CONFIG says it's also *supported* ...
	 */
	if (config & (1 << 23))
		etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
			etm_fifofull, ARRAY_SIZE(etm_fifofull));

	/* sequencer is optional (for state-dependant triggering) */
	if (config & (1 << 16))
		etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
			etm_sequencer, ARRAY_SIZE(etm_sequencer));

	/* REVISIT could realloc and likely save half the memory
	 * in the two chunks we allocated...
	 */

	/* the ETM might have an ETB connected */
	if (strcmp(etm_ctx->capture_driver->name, "etb") == 0) {
		struct etb *etb = etm_ctx->capture_driver_priv;

		if (!etb) {
			LOG_ERROR("etb selected as etm capture driver, but no ETB configured");
			goto fail;
		}

		reg_cache->next = etb_build_reg_cache(etb);

		etb->reg_cache = reg_cache->next;
	}

	etm_ctx->reg_cache = reg_cache;
	return reg_cache;

fail:
	free(reg_cache);
	free(reg_list);
	free(arch_info);
	return NULL;
}

static int etm_read_reg(struct reg *reg)
{
	return etm_read_reg_w_check(reg, NULL, NULL);
}

static int etm_store_reg(struct reg *reg)
{
	return etm_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

int etm_setup(struct target *target)
{
	int retval;
	uint32_t etm_ctrl_value;
	struct arm *arm = target_to_arm(target);
	struct etm_context *etm_ctx = arm->etm;
	struct reg *etm_ctrl_reg;

	etm_ctrl_reg = etm_reg_lookup(etm_ctx, ETM_CTRL);
	if (!etm_ctrl_reg)
		return ERROR_OK;

	/* initialize some ETM control register settings */
	etm_get_reg(etm_ctrl_reg);
	etm_ctrl_value = buf_get_u32(etm_ctrl_reg->value, 0, 32);

	/* clear the ETM powerdown bit (0) */
	etm_ctrl_value &= ~ETM_CTRL_POWERDOWN;

	/* configure port width (21,6:4), mode (13,17:16) and
	 * for older modules clocking (13)
	 */
	etm_ctrl_value = (etm_ctrl_value
		& ~ETM_PORT_WIDTH_MASK
		& ~ETM_PORT_MODE_MASK
		& ~ETM_CTRL_DBGRQ
		& ~ETM_PORT_CLOCK_MASK)
		| etm_ctx->control;

	buf_set_u32(etm_ctrl_reg->value, 0, 32, etm_ctrl_value);
	etm_store_reg(etm_ctrl_reg);

	etm_ctx->control = etm_ctrl_value;

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* REVISIT for ETMv3.0 and later, read ETM_sys_config to
	 * verify that those width and mode settings are OK ...
	 */

	retval = etm_ctx->capture_driver->init(etm_ctx);
	if (retval != ERROR_OK) {
		LOG_ERROR("ETM capture driver initialization failed");
		return retval;
	}
	return ERROR_OK;
}

static int etm_get_reg(struct reg *reg)
{
	int retval;

	retval = etm_read_reg(reg);
	if (retval != ERROR_OK) {
		LOG_ERROR("BUG: error scheduling etm register read");
		return retval;
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	return ERROR_OK;
}

static int etm_read_reg_w_check(struct reg *reg,
	uint8_t *check_value, uint8_t *check_mask)
{
	struct etm_reg *etm_reg = reg->arch_info;
	assert(etm_reg);
	const struct etm_reg_info *r = etm_reg->reg_info;
	uint8_t reg_addr = r->addr & 0x7f;
	struct scan_field fields[3];
	int retval;

	if (etm_reg->reg_info->mode == WO) {
		LOG_ERROR("BUG: can't read write-only register %s", r->name);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	LOG_DEBUG("%s (%u)", r->name, reg_addr);

	retval = arm_jtag_scann(etm_reg->jtag_info, 0x6, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(etm_reg->jtag_info->tap,
			etm_reg->jtag_info->intest_instr,
			NULL,
			TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].in_value = NULL;
	fields[0].check_value = NULL;
	fields[0].check_mask = NULL;

	fields[1].num_bits = 7;
	uint8_t temp1 = 0;
	fields[1].out_value = &temp1;
	buf_set_u32(&temp1, 0, 7, reg_addr);
	fields[1].in_value = NULL;
	fields[1].check_value = NULL;
	fields[1].check_mask = NULL;

	fields[2].num_bits = 1;
	uint8_t temp2 = 0;
	fields[2].out_value = &temp2;
	buf_set_u32(&temp2, 0, 1, 0);
	fields[2].in_value = NULL;
	fields[2].check_value = NULL;
	fields[2].check_mask = NULL;

	jtag_add_dr_scan(etm_reg->jtag_info->tap, 3, fields, TAP_IDLE);

	fields[0].in_value = reg->value;
	fields[0].check_value = check_value;
	fields[0].check_mask = check_mask;

	jtag_add_dr_scan_check(etm_reg->jtag_info->tap, 3, fields, TAP_IDLE);

	return ERROR_OK;
}

static int etm_set_reg(struct reg *reg, uint32_t value)
{
	int retval = etm_write_reg(reg, value);
	if (retval != ERROR_OK) {
		LOG_ERROR("BUG: error scheduling etm register write");
		return retval;
	}

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;

	return ERROR_OK;
}

static int etm_set_reg_w_exec(struct reg *reg, uint8_t *buf)
{
	int retval;

	etm_set_reg(reg, buf_get_u32(buf, 0, reg->size));

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register write failed");
		return retval;
	}
	return ERROR_OK;
}

static int etm_write_reg(struct reg *reg, uint32_t value)
{
	struct etm_reg *etm_reg = reg->arch_info;
	const struct etm_reg_info *r = etm_reg->reg_info;
	uint8_t reg_addr = r->addr & 0x7f;
	struct scan_field fields[3];
	int retval;

	if (etm_reg->reg_info->mode == RO) {
		LOG_ERROR("BUG: can't write read--only register %s", r->name);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	LOG_DEBUG("%s (%u): 0x%8.8" PRIx32 "", r->name, reg_addr, value);

	retval = arm_jtag_scann(etm_reg->jtag_info, 0x6, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(etm_reg->jtag_info->tap,
			etm_reg->jtag_info->intest_instr,
			NULL,
			TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	uint8_t tmp1[4];
	fields[0].out_value = tmp1;
	buf_set_u32(tmp1, 0, 32, value);
	fields[0].in_value = NULL;

	fields[1].num_bits = 7;
	uint8_t tmp2 = 0;
	fields[1].out_value = &tmp2;
	buf_set_u32(&tmp2, 0, 7, reg_addr);
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	uint8_t tmp3 = 0;
	fields[2].out_value = &tmp3;
	buf_set_u32(&tmp3, 0, 1, 1);
	fields[2].in_value = NULL;

	jtag_add_dr_scan(etm_reg->jtag_info->tap, 3, fields, TAP_IDLE);

	return ERROR_OK;
}


/* ETM trace analysis functionality */

static struct etm_capture_driver *etm_capture_drivers[] = {
	&etb_capture_driver,
	&etm_dummy_capture_driver,
#if BUILD_OOCD_TRACE == 1
	&oocd_trace_capture_driver,
#endif
	NULL
};

static int etm_read_instruction(struct etm_context *ctx, struct arm_instruction *instruction)
{
	int section = -1;
	size_t size_read;
	uint32_t opcode;
	int retval;

	if (!ctx->image)
		return ERROR_TRACE_IMAGE_UNAVAILABLE;

	/* search for the section the current instruction belongs to */
	for (unsigned int i = 0; i < ctx->image->num_sections; i++) {
		if ((ctx->image->sections[i].base_address <= ctx->current_pc) &&
			(ctx->image->sections[i].base_address + ctx->image->sections[i].size >
			ctx->current_pc)) {
			section = i;
			break;
		}
	}

	if (section == -1) {
		/* current instruction couldn't be found in the image */
		return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
	}

	if (ctx->core_state == ARM_STATE_ARM) {
		uint8_t buf[4];
		retval = image_read_section(ctx->image, section,
				ctx->current_pc -
				ctx->image->sections[section].base_address,
				4, buf, &size_read);
		if (retval != ERROR_OK) {
			LOG_ERROR("error while reading instruction");
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u32(ctx->target, buf);
		arm_evaluate_opcode(opcode, ctx->current_pc, instruction);
	} else if (ctx->core_state == ARM_STATE_THUMB) {
		uint8_t buf[2];
		retval = image_read_section(ctx->image, section,
				ctx->current_pc -
				ctx->image->sections[section].base_address,
				2, buf, &size_read);
		if (retval != ERROR_OK) {
			LOG_ERROR("error while reading instruction");
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u16(ctx->target, buf);
		thumb_evaluate_opcode(opcode, ctx->current_pc, instruction);
	} else if (ctx->core_state == ARM_STATE_JAZELLE) {
		LOG_ERROR("BUG: tracing of jazelle code not supported");
		return ERROR_FAIL;
	} else {
		LOG_ERROR("BUG: unknown core state encountered");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int etmv1_next_packet(struct etm_context *ctx, uint8_t *packet, int apo)
{
	while (ctx->data_index < ctx->trace_depth) {
		/* if the caller specified an address packet offset, skip until the
		 * we reach the n-th cycle marked with tracesync */
		if (apo > 0) {
			if (ctx->trace_data[ctx->data_index].flags & ETMV1_TRACESYNC_CYCLE)
				apo--;

			if (apo > 0) {
				ctx->data_index++;
				ctx->data_half = 0;
			}
			continue;
		}

		/* no tracedata output during a TD cycle
		 * or in a trigger cycle */
		if ((ctx->trace_data[ctx->data_index].pipestat == STAT_TD)
			|| (ctx->trace_data[ctx->data_index].flags & ETMV1_TRIGGER_CYCLE)) {
			ctx->data_index++;
			ctx->data_half = 0;
			continue;
		}

		/* FIXME there are more port widths than these... */
		if ((ctx->control & ETM_PORT_WIDTH_MASK) == ETM_PORT_16BIT) {
			if (ctx->data_half == 0) {
				*packet = ctx->trace_data[ctx->data_index].packet & 0xff;
				ctx->data_half = 1;
			} else {
				*packet = (ctx->trace_data[ctx->data_index].packet & 0xff00) >> 8;
				ctx->data_half = 0;
				ctx->data_index++;
			}
		} else if ((ctx->control & ETM_PORT_WIDTH_MASK) == ETM_PORT_8BIT) {
			*packet = ctx->trace_data[ctx->data_index].packet & 0xff;
			ctx->data_index++;
		} else {
			/* on a 4-bit port, a packet will be output during two consecutive cycles */
			if (ctx->data_index > (ctx->trace_depth - 2))
				return -1;

			*packet = ctx->trace_data[ctx->data_index].packet & 0xf;
			*packet |= (ctx->trace_data[ctx->data_index + 1].packet & 0xf) << 4;
			ctx->data_index += 2;
		}

		return 0;
	}

	return -1;
}

static int etmv1_branch_address(struct etm_context *ctx)
{
	int retval;
	uint8_t packet;
	int shift = 0;
	int apo;
	uint32_t i;

	/* quit analysis if less than two cycles are left in the trace
	 * because we can't extract the APO */
	if (ctx->data_index > (ctx->trace_depth - 2))
		return -1;

	/* a BE could be output during an APO cycle, skip the current
	 * and continue with the new one */
	if (ctx->trace_data[ctx->pipe_index + 1].pipestat & 0x4)
		return 1;
	if (ctx->trace_data[ctx->pipe_index + 2].pipestat & 0x4)
		return 2;

	/* address packet offset encoded in the next two cycles' pipestat bits */
	apo = ctx->trace_data[ctx->pipe_index + 1].pipestat & 0x3;
	apo |= (ctx->trace_data[ctx->pipe_index + 2].pipestat & 0x3) << 2;

	/* count number of tracesync cycles between current pipe_index and data_index
	 * i.e. the number of tracesyncs that data_index already passed by
	 * to subtract them from the APO */
	for (i = ctx->pipe_index; i < ctx->data_index; i++) {
		if (ctx->trace_data[ctx->pipe_index + 1].pipestat & ETMV1_TRACESYNC_CYCLE)
			apo--;
	}

	/* extract up to four 7-bit packets */
	do {
		retval = etmv1_next_packet(ctx, &packet, (shift == 0) ? apo + 1 : 0);
		if (retval != 0)
			return -1;
		ctx->last_branch &= ~(0x7f << shift);
		ctx->last_branch |= (packet & 0x7f) << shift;
		shift += 7;
	} while ((packet & 0x80) && (shift < 28));

	/* one last packet holding 4 bits of the address, plus the branch reason code */
	if ((shift == 28) && (packet & 0x80)) {
		retval = etmv1_next_packet(ctx, &packet, 0);
		if (retval != 0)
			return -1;
		ctx->last_branch &= 0x0fffffff;
		ctx->last_branch |= (packet & 0x0f) << 28;
		ctx->last_branch_reason = (packet & 0x70) >> 4;
		shift += 4;
	} else
		ctx->last_branch_reason = 0;

	if (shift == 32)
		ctx->pc_ok = 1;

	/* if a full address was output, we might have branched into Jazelle state */
	if ((shift == 32) && (packet & 0x80))
		ctx->core_state = ARM_STATE_JAZELLE;
	else {
		/* if we didn't branch into Jazelle state, the current processor state is
		 * encoded in bit 0 of the branch target address */
		if (ctx->last_branch & 0x1) {
			ctx->core_state = ARM_STATE_THUMB;
			ctx->last_branch &= ~0x1;
		} else {
			ctx->core_state = ARM_STATE_ARM;
			ctx->last_branch &= ~0x3;
		}
	}

	return 0;
}

static int etmv1_data(struct etm_context *ctx, int size, uint32_t *data)
{
	int j;
	uint8_t buf[4];
	int retval;

	for (j = 0; j < size; j++) {
		retval = etmv1_next_packet(ctx, &buf[j], 0);
		if (retval != 0)
			return -1;
	}

	if (size == 8) {
		LOG_ERROR("TODO: add support for 64-bit values");
		return -1;
	} else if (size == 4)
		*data = target_buffer_get_u32(ctx->target, buf);
	else if (size == 2)
		*data = target_buffer_get_u16(ctx->target, buf);
	else if (size == 1)
		*data = buf[0];
	else
		return -1;

	return 0;
}

static int etmv1_analyze_trace(struct etm_context *ctx, struct command_invocation *cmd)
{
	int retval;
	struct arm_instruction instruction;

	/* read the trace data if it wasn't read already */
	if (ctx->trace_depth == 0)
		ctx->capture_driver->read_trace(ctx);

	if (ctx->trace_depth == 0) {
		command_print(cmd, "Trace is empty.");
		return ERROR_OK;
	}

	/* start at the beginning of the captured trace */
	ctx->pipe_index = 0;
	ctx->data_index = 0;
	ctx->data_half = 0;

	/* neither the PC nor the data pointer are valid */
	ctx->pc_ok = 0;
	ctx->ptr_ok = 0;

	while (ctx->pipe_index < ctx->trace_depth) {
		uint8_t pipestat = ctx->trace_data[ctx->pipe_index].pipestat;
		uint32_t next_pc = ctx->current_pc;
		uint32_t old_data_index = ctx->data_index;
		uint32_t old_data_half = ctx->data_half;
		uint32_t old_index = ctx->pipe_index;
		uint32_t last_instruction = ctx->last_instruction;
		uint32_t cycles = 0;
		int current_pc_ok = ctx->pc_ok;

		if (ctx->trace_data[ctx->pipe_index].flags & ETMV1_TRIGGER_CYCLE)
			command_print(cmd, "--- trigger ---");

		/* instructions execute in IE/D or BE/D cycles */
		if ((pipestat == STAT_IE) || (pipestat == STAT_ID))
			ctx->last_instruction = ctx->pipe_index;

		/* if we don't have a valid pc skip until we reach an indirect branch */
		if ((!ctx->pc_ok) && (pipestat != STAT_BE)) {
			ctx->pipe_index++;
			continue;
		}

		/* any indirect branch could have interrupted instruction flow
		 * - the branch reason code could indicate a trace discontinuity
		 * - a branch to the exception vectors indicates an exception
		 */
		if ((pipestat == STAT_BE) || (pipestat == STAT_BD)) {
			/* backup current data index, to be able to consume the branch address
			 * before examining data address and values
			 */
			old_data_index = ctx->data_index;
			old_data_half = ctx->data_half;

			ctx->last_instruction = ctx->pipe_index;

			retval = etmv1_branch_address(ctx);
			if (retval != 0) {
				/* negative return value from etmv1_branch_address means we ran out of packets,
				 * quit analysing the trace */
				if (retval < 0)
					break;

				/* a positive return values means the current branch was abandoned,
				 * and a new branch was encountered in cycle ctx->pipe_index + retval;
				 */
				LOG_WARNING(
					"abandoned branch encountered, correctness of analysis uncertain");
				ctx->pipe_index += retval;
				continue;
			}

			/* skip over APO cycles */
			ctx->pipe_index += 2;

			switch (ctx->last_branch_reason) {
				case 0x0:	/* normal PC change */
					next_pc = ctx->last_branch;
					break;
				case 0x1:	/* tracing enabled */
					command_print(cmd,
						"--- tracing enabled at 0x%8.8" PRIx32 " ---",
						ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x2:	/* trace restarted after FIFO overflow */
					command_print(cmd,
						"--- trace restarted after FIFO overflow at 0x%8.8" PRIx32 " ---",
						ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x3:	/* exit from debug state */
					command_print(cmd,
						"--- exit from debug state at 0x%8.8" PRIx32 " ---",
						ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x4:	/* periodic synchronization point */
					next_pc = ctx->last_branch;
					/* if we had no valid PC prior to this synchronization point,
					 * we have to move on with the next trace cycle
					 */
					if (!current_pc_ok) {
						command_print(cmd,
							"--- periodic synchronization point at 0x%8.8" PRIx32 " ---",
							next_pc);
						ctx->current_pc = next_pc;
						ctx->pipe_index++;
						continue;
					}
					break;
				default:	/* reserved */
					LOG_ERROR(
						"BUG: branch reason code 0x%" PRIx32 " is reserved",
						ctx->last_branch_reason);
					return ERROR_FAIL;
			}

			/* if we got here the branch was a normal PC change
			 * (or a periodic synchronization point, which means the same for that matter)
			 * if we didn't acquire a complete PC continue with the next cycle
			 */
			if (!ctx->pc_ok)
				continue;

			/* indirect branch to the exception vector means an exception occurred */
			if ((ctx->last_branch <= 0x20)
				|| ((ctx->last_branch >= 0xffff0000) &&
				(ctx->last_branch <= 0xffff0020))) {
				if ((ctx->last_branch & 0xff) == 0x10)
					command_print(cmd, "data abort");
				else {
					command_print(cmd,
						"exception vector 0x%2.2" PRIx32 "",
						ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
				}
			}
		}

		/* an instruction was executed (or not, depending on the condition flags)
		 * retrieve it from the image for displaying */
		if (ctx->pc_ok && (pipestat != STAT_WT) && (pipestat != STAT_TD) &&
			!(((pipestat == STAT_BE) || (pipestat == STAT_BD)) &&
			((ctx->last_branch_reason != 0x0) && (ctx->last_branch_reason != 0x4)))) {
			retval = etm_read_instruction(ctx, &instruction);
			if (retval != ERROR_OK) {
				/* can't continue tracing with no image available */
				if (retval == ERROR_TRACE_IMAGE_UNAVAILABLE)
					return retval;
				else if (retval == ERROR_TRACE_INSTRUCTION_UNAVAILABLE) {
					/* TODO: handle incomplete images
					 * for now we just quit the analysis*/
					return retval;
				}
			}

			cycles = old_index - last_instruction;
		}

		if ((pipestat == STAT_ID) || (pipestat == STAT_BD)) {
			uint32_t new_data_index = ctx->data_index;
			uint32_t new_data_half = ctx->data_half;

			/* in case of a branch with data, the branch target address was consumed before
			 * we temporarily go back to the saved data index */
			if (pipestat == STAT_BD) {
				ctx->data_index = old_data_index;
				ctx->data_half = old_data_half;
			}

			if (ctx->control & ETM_CTRL_TRACE_ADDR) {
				uint8_t packet;
				int shift = 0;

				do {
					retval = etmv1_next_packet(ctx, &packet, 0);
					if (retval != 0)
						return ERROR_ETM_ANALYSIS_FAILED;
					ctx->last_ptr &= ~(0x7f << shift);
					ctx->last_ptr |= (packet & 0x7f) << shift;
					shift += 7;
				} while ((packet & 0x80) && (shift < 32));

				if (shift >= 32)
					ctx->ptr_ok = 1;

				if (ctx->ptr_ok)
					command_print(cmd,
						"address: 0x%8.8" PRIx32 "",
						ctx->last_ptr);
			}

			if (ctx->control & ETM_CTRL_TRACE_DATA) {
				if ((instruction.type == ARM_LDM) ||
					(instruction.type == ARM_STM)) {
					int i;
					for (i = 0; i < 16; i++) {
						if (instruction.info.load_store_multiple.register_list
							& (1 << i)) {
							uint32_t data;
							if (etmv1_data(ctx, 4, &data) != 0)
								return ERROR_ETM_ANALYSIS_FAILED;
							command_print(cmd,
								"data: 0x%8.8" PRIx32 "",
								data);
						}
					}
				} else if ((instruction.type >= ARM_LDR) &&
					(instruction.type <= ARM_STRH)) {
					uint32_t data;
					if (etmv1_data(ctx, arm_access_size(&instruction),
						&data) != 0)
						return ERROR_ETM_ANALYSIS_FAILED;
					command_print(cmd, "data: 0x%8.8" PRIx32 "", data);
				}
			}

			/* restore data index after consuming BD address and data */
			if (pipestat == STAT_BD) {
				ctx->data_index = new_data_index;
				ctx->data_half = new_data_half;
			}
		}

		/* adjust PC */
		if ((pipestat == STAT_IE) || (pipestat == STAT_ID)) {
			if (((instruction.type == ARM_B) ||
				(instruction.type == ARM_BL) ||
				(instruction.type == ARM_BLX)) &&
				(instruction.info.b_bl_bx_blx.target_address != 0xffffffff))
				next_pc = instruction.info.b_bl_bx_blx.target_address;
			else
				next_pc += (ctx->core_state == ARM_STATE_ARM) ? 4 : 2;
		} else if (pipestat == STAT_IN)
			next_pc += (ctx->core_state == ARM_STATE_ARM) ? 4 : 2;

		if ((pipestat != STAT_TD) && (pipestat != STAT_WT)) {
			char cycles_text[32] = "";

			/* if the trace was captured with cycle accurate tracing enabled,
			 * output the number of cycles since the last executed instruction
			 */
			if (ctx->control & ETM_CTRL_CYCLE_ACCURATE) {
				snprintf(cycles_text, 32, " (%i %s)",
					(int)cycles,
					(cycles == 1) ? "cycle" : "cycles");
			}

			command_print(cmd, "%s%s%s",
				instruction.text,
				(pipestat == STAT_IN) ? " (not executed)" : "",
				cycles_text);

			ctx->current_pc = next_pc;

			/* packets for an instruction don't start on or before the preceding
			 * functional pipestat (i.e. other than WT or TD)
			 */
			if (ctx->data_index <= ctx->pipe_index) {
				ctx->data_index = ctx->pipe_index + 1;
				ctx->data_half = 0;
			}
		}

		ctx->pipe_index += 1;
	}

	return ERROR_OK;
}

static COMMAND_HELPER(handle_etm_tracemode_command_update,
	uint32_t *mode)
{
	uint32_t tracemode;

	/* what parts of data access are traced? */
	if (strcmp(CMD_ARGV[0], "none") == 0)
		tracemode = 0;
	else if (strcmp(CMD_ARGV[0], "data") == 0)
		tracemode = ETM_CTRL_TRACE_DATA;
	else if (strcmp(CMD_ARGV[0], "address") == 0)
		tracemode = ETM_CTRL_TRACE_ADDR;
	else if (strcmp(CMD_ARGV[0], "all") == 0)
		tracemode = ETM_CTRL_TRACE_DATA | ETM_CTRL_TRACE_ADDR;
	else {
		command_print(CMD, "invalid option '%s'", CMD_ARGV[0]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint8_t context_id;
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], context_id);
	switch (context_id) {
		case 0:
			tracemode |= ETM_CTRL_CONTEXTID_NONE;
			break;
		case 8:
			tracemode |= ETM_CTRL_CONTEXTID_8;
			break;
		case 16:
			tracemode |= ETM_CTRL_CONTEXTID_16;
			break;
		case 32:
			tracemode |= ETM_CTRL_CONTEXTID_32;
			break;
		default:
			command_print(CMD, "invalid option '%s'", CMD_ARGV[1]);
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	bool etmv1_cycle_accurate;
	COMMAND_PARSE_ENABLE(CMD_ARGV[2], etmv1_cycle_accurate);
	if (etmv1_cycle_accurate)
		tracemode |= ETM_CTRL_CYCLE_ACCURATE;

	bool etmv1_branch_output;
	COMMAND_PARSE_ENABLE(CMD_ARGV[3], etmv1_branch_output);
	if (etmv1_branch_output)
		tracemode |= ETM_CTRL_BRANCH_OUTPUT;

	/* IGNORED:
	 *  - CPRT tracing (coprocessor register transfers)
	 *  - debug request (causes debug entry on trigger)
	 *  - stall on FIFOFULL (preventing tracedata loss)
	 */
	*mode = tracemode;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_tracemode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct arm *arm = target_to_arm(target);
	struct etm_context *etm;

	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm = arm->etm;
	if (!etm) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	uint32_t tracemode = etm->control;

	switch (CMD_ARGC) {
		case 0:
			break;
		case 4:
			CALL_COMMAND_HANDLER(handle_etm_tracemode_command_update,
				&tracemode);
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/**
	 * todo: fail if parameters were invalid for this hardware,
	 * or couldn't be written; display actual hardware state...
	 */

	command_print(CMD, "current tracemode configuration:");

	switch (tracemode & ETM_CTRL_TRACE_MASK) {
		default:
			command_print(CMD, "data tracing: none");
			break;
		case ETM_CTRL_TRACE_DATA:
			command_print(CMD, "data tracing: data only");
			break;
		case ETM_CTRL_TRACE_ADDR:
			command_print(CMD, "data tracing: address only");
			break;
		case ETM_CTRL_TRACE_DATA | ETM_CTRL_TRACE_ADDR:
			command_print(CMD, "data tracing: address and data");
			break;
	}

	switch (tracemode & ETM_CTRL_CONTEXTID_MASK) {
		case ETM_CTRL_CONTEXTID_NONE:
			command_print(CMD, "contextid tracing: none");
			break;
		case ETM_CTRL_CONTEXTID_8:
			command_print(CMD, "contextid tracing: 8 bit");
			break;
		case ETM_CTRL_CONTEXTID_16:
			command_print(CMD, "contextid tracing: 16 bit");
			break;
		case ETM_CTRL_CONTEXTID_32:
			command_print(CMD, "contextid tracing: 32 bit");
			break;
	}

	if (tracemode & ETM_CTRL_CYCLE_ACCURATE)
		command_print(CMD, "cycle-accurate tracing enabled");
	else
		command_print(CMD, "cycle-accurate tracing disabled");

	if (tracemode & ETM_CTRL_BRANCH_OUTPUT)
		command_print(CMD, "full branch address output enabled");
	else
		command_print(CMD, "full branch address output disabled");

#define TRACEMODE_MASK ( \
		ETM_CTRL_CONTEXTID_MASK	\
		| ETM_CTRL_BRANCH_OUTPUT \
		| ETM_CTRL_CYCLE_ACCURATE \
		| ETM_CTRL_TRACE_MASK \
		)

	/* only update ETM_CTRL register if tracemode changed */
	if ((etm->control & TRACEMODE_MASK) != tracemode) {
		struct reg *etm_ctrl_reg;

		etm_ctrl_reg = etm_reg_lookup(etm, ETM_CTRL);
		if (!etm_ctrl_reg)
			return ERROR_FAIL;

		etm->control &= ~TRACEMODE_MASK;
		etm->control |= tracemode & TRACEMODE_MASK;

		buf_set_u32(etm_ctrl_reg->value, 0, 32, etm->control);
		etm_store_reg(etm_ctrl_reg);

		/* invalidate old trace data */
		etm->capture_status = TRACE_IDLE;
		if (etm->trace_depth > 0) {
			free(etm->trace_data);
			etm->trace_data = NULL;
		}
		etm->trace_depth = 0;
	}

#undef TRACEMODE_MASK

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_config_command)
{
	struct target *target;
	struct arm *arm;
	uint32_t portmode = 0x0;
	struct etm_context *etm_ctx;
	int i;

	if (CMD_ARGC != 5)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target = get_target(CMD_ARGV[0]);
	if (!target) {
		LOG_ERROR("target '%s' not defined", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "target '%s' is '%s'; not an ARM",
			target_name(target),
			target_type_name(target));
		return ERROR_FAIL;
	}

	/* FIXME for ETMv3.0 and above -- and we don't yet know what ETM
	 * version we'll be using!! -- so we can't know how to validate
	 * params yet.  "etm config" should likely be *AFTER* hookup...
	 *
	 *  - Many more widths might be supported ... and we can easily
	 *    check whether our setting "took".
	 *
	 *  - The "clock" and "mode" bits are interpreted differently.
	 *    See ARM IHI 0014O table 2-17 for the old behaviour, and
	 *    table 2-18 for the new.  With ETB it's best to specify
	 *    "normal full" ...
	 */
	uint8_t port_width;
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], port_width);
	switch (port_width) {
		/* before ETMv3.0 */
		case 4:
			portmode |= ETM_PORT_4BIT;
			break;
		case 8:
			portmode |= ETM_PORT_8BIT;
			break;
		case 16:
			portmode |= ETM_PORT_16BIT;
			break;
		/* ETMv3.0 and later*/
		case 24:
			portmode |= ETM_PORT_24BIT;
			break;
		case 32:
			portmode |= ETM_PORT_32BIT;
			break;
		case 48:
			portmode |= ETM_PORT_48BIT;
			break;
		case 64:
			portmode |= ETM_PORT_64BIT;
			break;
		case 1:
			portmode |= ETM_PORT_1BIT;
			break;
		case 2:
			portmode |= ETM_PORT_2BIT;
			break;
		default:
			command_print(CMD,
				"unsupported ETM port width '%s'", CMD_ARGV[1]);
			return ERROR_FAIL;
	}

	if (strcmp("normal", CMD_ARGV[2]) == 0)
		portmode |= ETM_PORT_NORMAL;
	else if (strcmp("multiplexed", CMD_ARGV[2]) == 0)
		portmode |= ETM_PORT_MUXED;
	else if (strcmp("demultiplexed", CMD_ARGV[2]) == 0)
		portmode |= ETM_PORT_DEMUXED;
	else {
		command_print(CMD,
			"unsupported ETM port mode '%s', must be 'normal', 'multiplexed' or 'demultiplexed'",
			CMD_ARGV[2]);
		return ERROR_FAIL;
	}

	if (strcmp("half", CMD_ARGV[3]) == 0)
		portmode |= ETM_PORT_HALF_CLOCK;
	else if (strcmp("full", CMD_ARGV[3]) == 0)
		portmode |= ETM_PORT_FULL_CLOCK;
	else {
		command_print(CMD,
			"unsupported ETM port clocking '%s', must be 'full' or 'half'",
			CMD_ARGV[3]);
		return ERROR_FAIL;
	}

	etm_ctx = calloc(1, sizeof(struct etm_context));
	if (!etm_ctx) {
		LOG_DEBUG("out of memory");
		return ERROR_FAIL;
	}

	for (i = 0; etm_capture_drivers[i]; i++) {
		if (strcmp(CMD_ARGV[4], etm_capture_drivers[i]->name) == 0) {
			int retval = register_commands(CMD_CTX, NULL,
					etm_capture_drivers[i]->commands);
			if (ERROR_OK != retval) {
				free(etm_ctx);
				return retval;
			}

			etm_ctx->capture_driver = etm_capture_drivers[i];

			break;
		}
	}

	if (!etm_capture_drivers[i]) {
		/* no supported capture driver found, don't register an ETM */
		free(etm_ctx);
		LOG_ERROR("trace capture driver '%s' not found", CMD_ARGV[4]);
		return ERROR_FAIL;
	}

	etm_ctx->target = target;
	etm_ctx->trace_data = NULL;
	etm_ctx->control = portmode;
	etm_ctx->core_state = ARM_STATE_ARM;

	arm->etm = etm_ctx;

	return etm_register_user_commands(CMD_CTX);
}

COMMAND_HANDLER(handle_etm_info_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm;
	struct reg *etm_sys_config_reg;
	int max_port_size;
	uint32_t config;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm = arm->etm;
	if (!etm) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	command_print(CMD, "ETM v%d.%d",
		etm->bcd_vers >> 4, etm->bcd_vers & 0xf);
	command_print(CMD, "pairs of address comparators: %i",
		(int) (etm->config >> 0) & 0x0f);
	command_print(CMD, "data comparators: %i",
		(int) (etm->config >> 4) & 0x0f);
	command_print(CMD, "memory map decoders: %i",
		(int) (etm->config >> 8) & 0x1f);
	command_print(CMD, "number of counters: %i",
		(int) (etm->config >> 13) & 0x07);
	command_print(CMD, "sequencer %spresent",
		(int) (etm->config & (1 << 16)) ? "" : "not ");
	command_print(CMD, "number of ext. inputs: %i",
		(int) (etm->config >> 17) & 0x07);
	command_print(CMD, "number of ext. outputs: %i",
		(int) (etm->config >> 20) & 0x07);
	command_print(CMD, "FIFO full %spresent",
		(int) (etm->config & (1 << 23)) ? "" : "not ");
	if (etm->bcd_vers < 0x20)
		command_print(CMD, "protocol version: %i",
			(int) (etm->config >> 28) & 0x07);
	else {
		command_print(CMD,
			"coprocessor and memory access %ssupported",
			(etm->config & (1 << 26)) ? "" : "not ");
		command_print(CMD, "trace start/stop %spresent",
			(etm->config & (1 << 26)) ? "" : "not ");
		command_print(CMD, "number of context comparators: %i",
			(int) (etm->config >> 24) & 0x03);
	}

	/* SYS_CONFIG isn't present before ETMv1.2 */
	etm_sys_config_reg = etm_reg_lookup(etm, ETM_SYS_CONFIG);
	if (!etm_sys_config_reg)
		return ERROR_OK;

	etm_get_reg(etm_sys_config_reg);
	config = buf_get_u32(etm_sys_config_reg->value, 0, 32);

	LOG_DEBUG("ETM SYS CONFIG %08x", (unsigned) config);

	max_port_size = config & 0x7;
	if (etm->bcd_vers >= 0x30)
		max_port_size |= (config >> 6) & 0x08;
	switch (max_port_size) {
		/* before ETMv3.0 */
		case 0:
			max_port_size = 4;
			break;
		case 1:
			max_port_size = 8;
			break;
		case 2:
			max_port_size = 16;
			break;
		/* ETMv3.0 and later*/
		case 3:
			max_port_size = 24;
			break;
		case 4:
			max_port_size = 32;
			break;
		case 5:
			max_port_size = 48;
			break;
		case 6:
			max_port_size = 64;
			break;
		case 8:
			max_port_size = 1;
			break;
		case 9:
			max_port_size = 2;
			break;
		default:
			LOG_ERROR("Illegal max_port_size");
			return ERROR_FAIL;
	}
	command_print(CMD, "max. port size: %i", max_port_size);

	if (etm->bcd_vers < 0x30) {
		command_print(CMD, "half-rate clocking %ssupported",
			(config & (1 << 3)) ? "" : "not ");
		command_print(CMD, "full-rate clocking %ssupported",
			(config & (1 << 4)) ? "" : "not ");
		command_print(CMD, "normal trace format %ssupported",
			(config & (1 << 5)) ? "" : "not ");
		command_print(CMD, "multiplex trace format %ssupported",
			(config & (1 << 6)) ? "" : "not ");
		command_print(CMD, "demultiplex trace format %ssupported",
			(config & (1 << 7)) ? "" : "not ");
	} else {
		/* REVISIT show which size and format are selected ... */
		command_print(CMD, "current port size %ssupported",
			(config & (1 << 10)) ? "" : "not ");
		command_print(CMD, "current trace format %ssupported",
			(config & (1 << 11)) ? "" : "not ");
	}
	if (etm->bcd_vers >= 0x21)
		command_print(CMD, "fetch comparisons %ssupported",
			(config & (1 << 17)) ? "not " : "");
	command_print(CMD, "FIFO full %ssupported",
		(config & (1 << 8)) ? "" : "not ");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_status_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm;
	trace_status_t trace_status;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm = arm->etm;
	if (!etm) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	/* ETM status */
	if (etm->bcd_vers >= 0x11) {
		struct reg *reg;

		reg = etm_reg_lookup(etm, ETM_STATUS);
		if (!reg)
			return ERROR_FAIL;
		if (etm_get_reg(reg) == ERROR_OK) {
			unsigned s = buf_get_u32(reg->value, 0, reg->size);

			command_print(CMD, "etm: %s%s%s%s",
				/* bit(1) == progbit */
				(etm->bcd_vers >= 0x12)
				? ((s & (1 << 1))
				? "disabled" : "enabled")
				: "?",
				((s & (1 << 3)) && etm->bcd_vers >= 0x31)
				? " triggered" : "",
				((s & (1 << 2)) && etm->bcd_vers >= 0x12)
				? " start/stop" : "",
				((s & (1 << 0)) && etm->bcd_vers >= 0x11)
				? " untraced-overflow" : "");
		}	/* else ignore and try showing trace port status */
	}

	/* Trace Port Driver status */
	trace_status = etm->capture_driver->status(etm);
	if (trace_status == TRACE_IDLE)
		command_print(CMD, "%s: idle", etm->capture_driver->name);
	else {
		static char *completed = " completed";
		static char *running = " is running";
		static char *overflowed = ", overflowed";
		static char *triggered = ", triggered";

		command_print(CMD, "%s: trace collection%s%s%s",
			etm->capture_driver->name,
			(trace_status & TRACE_RUNNING) ? running : completed,
			(trace_status & TRACE_OVERFLOWED) ? overflowed : "",
			(trace_status & TRACE_TRIGGERED) ? triggered : "");

		if (etm->trace_depth > 0) {
			command_print(CMD, "%i frames of trace data read",
				(int)(etm->trace_depth));
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_image_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm_ctx;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm_ctx = arm->etm;
	if (!etm_ctx) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	if (etm_ctx->image) {
		image_close(etm_ctx->image);
		free(etm_ctx->image);
		command_print(CMD, "previously loaded image found and closed");
	}

	etm_ctx->image = malloc(sizeof(struct image));
	etm_ctx->image->base_address_set = false;
	etm_ctx->image->start_address_set = false;

	/* a base address isn't always necessary, default to 0x0 (i.e. don't relocate) */
	if (CMD_ARGC >= 2) {
		etm_ctx->image->base_address_set = true;
		COMMAND_PARSE_NUMBER(llong, CMD_ARGV[1], etm_ctx->image->base_address);
	} else
		etm_ctx->image->base_address_set = false;

	if (image_open(etm_ctx->image, CMD_ARGV[0],
		(CMD_ARGC >= 3) ? CMD_ARGV[2] : NULL) != ERROR_OK) {
		free(etm_ctx->image);
		etm_ctx->image = NULL;
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_dump_command)
{
	struct fileio *file;
	struct target *target;
	struct arm *arm;
	struct etm_context *etm_ctx;
	uint32_t i;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm_ctx = arm->etm;
	if (!etm_ctx) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	if (etm_ctx->capture_driver->status == TRACE_IDLE) {
		command_print(CMD, "trace capture wasn't enabled, no trace data captured");
		return ERROR_OK;
	}

	if (etm_ctx->capture_driver->status(etm_ctx) & TRACE_RUNNING) {
		/* TODO: if on-the-fly capture is to be supported, this needs to be changed */
		command_print(CMD, "trace capture not completed");
		return ERROR_FAIL;
	}

	/* read the trace data if it wasn't read already */
	if (etm_ctx->trace_depth == 0)
		etm_ctx->capture_driver->read_trace(etm_ctx);

	if (fileio_open(&file, CMD_ARGV[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
		return ERROR_FAIL;

	fileio_write_u32(file, etm_ctx->capture_status);
	fileio_write_u32(file, etm_ctx->control);
	fileio_write_u32(file, etm_ctx->trace_depth);

	for (i = 0; i < etm_ctx->trace_depth; i++) {
		fileio_write_u32(file, etm_ctx->trace_data[i].pipestat);
		fileio_write_u32(file, etm_ctx->trace_data[i].packet);
		fileio_write_u32(file, etm_ctx->trace_data[i].flags);
	}

	fileio_close(file);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_load_command)
{
	struct fileio *file;
	struct target *target;
	struct arm *arm;
	struct etm_context *etm_ctx;
	uint32_t i;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm_ctx = arm->etm;
	if (!etm_ctx) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	if (etm_ctx->capture_driver->status(etm_ctx) & TRACE_RUNNING) {
		command_print(CMD, "trace capture running, stop first");
		return ERROR_FAIL;
	}

	if (fileio_open(&file, CMD_ARGV[0], FILEIO_READ, FILEIO_BINARY) != ERROR_OK)
		return ERROR_FAIL;

	size_t filesize;
	int retval = fileio_size(file, &filesize);
	if (retval != ERROR_OK) {
		fileio_close(file);
		return retval;
	}

	if (filesize % 4) {
		command_print(CMD, "size isn't a multiple of 4, no valid trace data");
		fileio_close(file);
		return ERROR_FAIL;
	}

	if (etm_ctx->trace_depth > 0) {
		free(etm_ctx->trace_data);
		etm_ctx->trace_data = NULL;
	}

	{
		uint32_t tmp;
		fileio_read_u32(file, &tmp); etm_ctx->capture_status = tmp;
		fileio_read_u32(file, &tmp); etm_ctx->control = tmp;
		fileio_read_u32(file, &etm_ctx->trace_depth);
	}
	etm_ctx->trace_data = malloc(sizeof(struct etmv1_trace_data) * etm_ctx->trace_depth);
	if (etm_ctx->trace_data == NULL) {
		command_print(CMD, "not enough memory to perform operation");
		fileio_close(file);
		return ERROR_FAIL;
	}

	for (i = 0; i < etm_ctx->trace_depth; i++) {
		uint32_t pipestat, packet, flags;
		fileio_read_u32(file, &pipestat);
		fileio_read_u32(file, &packet);
		fileio_read_u32(file, &flags);
		etm_ctx->trace_data[i].pipestat = pipestat & 0xff;
		etm_ctx->trace_data[i].packet = packet & 0xffff;
		etm_ctx->trace_data[i].flags = flags;
	}

	fileio_close(file);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_start_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm_ctx;
	struct reg *etm_ctrl_reg;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm_ctx = arm->etm;
	if (!etm_ctx) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	/* invalidate old tracing data */
	etm_ctx->capture_status = TRACE_IDLE;
	if (etm_ctx->trace_depth > 0) {
		free(etm_ctx->trace_data);
		etm_ctx->trace_data = NULL;
	}
	etm_ctx->trace_depth = 0;

	etm_ctrl_reg = etm_reg_lookup(etm_ctx, ETM_CTRL);
	if (!etm_ctrl_reg)
		return ERROR_FAIL;

	etm_get_reg(etm_ctrl_reg);

	/* Clear programming bit (10), set port selection bit (11) */
	buf_set_u32(etm_ctrl_reg->value, 10, 2, 0x2);

	etm_store_reg(etm_ctrl_reg);
	jtag_execute_queue();

	etm_ctx->capture_driver->start_capture(etm_ctx);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_stop_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm_ctx;
	struct reg *etm_ctrl_reg;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm_ctx = arm->etm;
	if (!etm_ctx) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	etm_ctrl_reg = etm_reg_lookup(etm_ctx, ETM_CTRL);
	if (!etm_ctrl_reg)
		return ERROR_FAIL;

	etm_get_reg(etm_ctrl_reg);

	/* Set programming bit (10), clear port selection bit (11) */
	buf_set_u32(etm_ctrl_reg->value, 10, 2, 0x1);

	etm_store_reg(etm_ctrl_reg);
	jtag_execute_queue();

	etm_ctx->capture_driver->stop_capture(etm_ctx);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_trigger_debug_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: %s isn't an ARM",
			target_name(target));
		return ERROR_FAIL;
	}

	etm = arm->etm;
	if (!etm) {
		command_print(CMD, "ETM: no ETM configured for %s",
			target_name(target));
		return ERROR_FAIL;
	}

	if (CMD_ARGC == 1) {
		struct reg *etm_ctrl_reg;
		bool dbgrq;

		etm_ctrl_reg = etm_reg_lookup(etm, ETM_CTRL);
		if (!etm_ctrl_reg)
			return ERROR_FAIL;

		COMMAND_PARSE_ENABLE(CMD_ARGV[0], dbgrq);
		if (dbgrq)
			etm->control |= ETM_CTRL_DBGRQ;
		else
			etm->control &= ~ETM_CTRL_DBGRQ;

		/* etm->control will be written to hardware
		 * the next time an "etm start" is issued.
		 */
		buf_set_u32(etm_ctrl_reg->value, 0, 32, etm->control);
	}

	command_print(CMD, "ETM: %s debug halt",
		(etm->control & ETM_CTRL_DBGRQ)
		? "triggers"
		: "does not trigger");
	return ERROR_OK;
}

COMMAND_HANDLER(handle_etm_analyze_command)
{
	struct target *target;
	struct arm *arm;
	struct etm_context *etm_ctx;
	int retval;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "ETM: current target isn't an ARM");
		return ERROR_FAIL;
	}

	etm_ctx = arm->etm;
	if (!etm_ctx) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	retval = etmv1_analyze_trace(etm_ctx, CMD);
	if (retval != ERROR_OK) {
		/* FIX! error should be reported inside etmv1_analyze_trace() */
		switch (retval) {
			case ERROR_ETM_ANALYSIS_FAILED:
				command_print(CMD,
					"further analysis failed (corrupted trace data or just end of data");
				break;
			case ERROR_TRACE_INSTRUCTION_UNAVAILABLE:
				command_print(CMD,
					"no instruction for current address available, analysis aborted");
				break;
			case ERROR_TRACE_IMAGE_UNAVAILABLE:
				command_print(CMD, "no image available for trace analysis");
				break;
			default:
				command_print(CMD, "unknown error");
		}
	}

	return retval;
}

static const struct command_registration etm_config_command_handlers[] = {
	{
		/* NOTE:  with ADIv5, ETMs are accessed by DAP operations,
		 * possibly over SWD, not JTAG scanchain 6 of 'target'.
		 *
		 * Also, these parameters don't match ETM v3+ modules...
		 */
		.name = "config",
		.handler = handle_etm_config_command,
		.mode = COMMAND_CONFIG,
		.help = "Set up ETM output port.",
		.usage = "target port_width port_mode clocking capture_driver",
	},
	COMMAND_REGISTRATION_DONE
};
const struct command_registration etm_command_handlers[] = {
	{
		.name = "etm",
		.mode = COMMAND_ANY,
		.help = "Embedded Trace Macrocell command group",
		.usage = "",
		.chain = etm_config_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration etm_exec_command_handlers[] = {
	{
		.name = "tracemode",
		.handler = handle_etm_tracemode_command,
		.mode = COMMAND_EXEC,
		.help = "configure/display trace mode",
		.usage = "('none'|'data'|'address'|'all') "
			"context_id_bits "
			"['enable'|'disable'] "
			"['enable'|'disable']",
	},
	{
		.name = "info",
		.handler = handle_etm_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "display info about the current target's ETM",
	},
	{
		.name = "status",
		.handler = handle_etm_status_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "display current target's ETM status",
	},
	{
		.name = "start",
		.handler = handle_etm_start_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "start ETM trace collection",
	},
	{
		.name = "stop",
		.handler = handle_etm_stop_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "stop ETM trace collection",
	},
	{
		.name = "trigger_debug",
		.handler = handle_etm_trigger_debug_command,
		.mode = COMMAND_EXEC,
		.help = "enable/disable debug entry on trigger",
		.usage = "['enable'|'disable']",
	},
	{
		.name = "analyze",
		.handler = handle_etm_analyze_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "analyze collected ETM trace",
	},
	{
		.name = "image",
		.handler = handle_etm_image_command,
		.mode = COMMAND_EXEC,
		.help = "load image from file with optional offset",
		.usage = "<file> [base address] [type]",
	},
	{
		.name = "dump",
		.handler = handle_etm_dump_command,
		.mode = COMMAND_EXEC,
		.help = "dump captured trace data to file",
		.usage = "filename",
	},
	{
		.name = "load",
		.handler = handle_etm_load_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "load trace data for analysis <file>",
	},
	COMMAND_REGISTRATION_DONE
};

static int etm_register_user_commands(struct command_context *cmd_ctx)
{
	struct command *etm_cmd = command_find_in_context(cmd_ctx, "etm");
	return register_commands(cmd_ctx, etm_cmd, etm_exec_command_handlers);
}
