/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/algorithm.h"
#include "target/target_type.h"
#include "log.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/time_support.h"
#include "riscv.h"
#include "gdb_regs.h"
#include "rtos/rtos.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

/* Constants for legacy SiFive hardware breakpoints. */
#define CSR_BPCONTROL_X			(1<<0)
#define CSR_BPCONTROL_W			(1<<1)
#define CSR_BPCONTROL_R			(1<<2)
#define CSR_BPCONTROL_U			(1<<3)
#define CSR_BPCONTROL_S			(1<<4)
#define CSR_BPCONTROL_H			(1<<5)
#define CSR_BPCONTROL_M			(1<<6)
#define CSR_BPCONTROL_BPMATCH	(0xf<<7)
#define CSR_BPCONTROL_BPACTION	(0xff<<11)

#define DEBUG_ROM_START         0x800
#define DEBUG_ROM_RESUME	(DEBUG_ROM_START + 4)
#define DEBUG_ROM_EXCEPTION	(DEBUG_ROM_START + 8)
#define DEBUG_RAM_START         0x400

#define SETHALTNOT				0x10c

/*** JTAG registers. ***/

#define DTMCONTROL					0x10
#define DTMCONTROL_DBUS_RESET		(1<<16)
#define DTMCONTROL_IDLE				(7<<10)
#define DTMCONTROL_ADDRBITS			(0xf<<4)
#define DTMCONTROL_VERSION			(0xf)

#define DBUS						0x11
#define DBUS_OP_START				0
#define DBUS_OP_SIZE				2
typedef enum {
	DBUS_OP_NOP = 0,
	DBUS_OP_READ = 1,
	DBUS_OP_WRITE = 2
} dbus_op_t;
typedef enum {
	DBUS_STATUS_SUCCESS = 0,
	DBUS_STATUS_FAILED = 2,
	DBUS_STATUS_BUSY = 3
} dbus_status_t;
#define DBUS_DATA_START				2
#define DBUS_DATA_SIZE				34
#define DBUS_ADDRESS_START			36

typedef enum slot {
	SLOT0,
	SLOT1,
	SLOT_LAST,
} slot_t;

/*** Debug Bus registers. ***/

#define DMCONTROL				0x10
#define DMCONTROL_INTERRUPT		(((uint64_t)1)<<33)
#define DMCONTROL_HALTNOT		(((uint64_t)1)<<32)
#define DMCONTROL_BUSERROR		(7<<19)
#define DMCONTROL_SERIAL		(3<<16)
#define DMCONTROL_AUTOINCREMENT	(1<<15)
#define DMCONTROL_ACCESS		(7<<12)
#define DMCONTROL_HARTID		(0x3ff<<2)
#define DMCONTROL_NDRESET		(1<<1)
#define DMCONTROL_FULLRESET		1

#define DMINFO					0x11
#define DMINFO_ABUSSIZE			(0x7fU<<25)
#define DMINFO_SERIALCOUNT		(0xf<<21)
#define DMINFO_ACCESS128		(1<<20)
#define DMINFO_ACCESS64			(1<<19)
#define DMINFO_ACCESS32			(1<<18)
#define DMINFO_ACCESS16			(1<<17)
#define DMINFO_ACCESS8			(1<<16)
#define DMINFO_DRAMSIZE			(0x3f<<10)
#define DMINFO_AUTHENTICATED	(1<<5)
#define DMINFO_AUTHBUSY			(1<<4)
#define DMINFO_AUTHTYPE			(3<<2)
#define DMINFO_VERSION			3

/*** Info about the core being debugged. ***/

#define DBUS_ADDRESS_UNKNOWN	0xffff

#define MAX_HWBPS			16
#define DRAM_CACHE_SIZE		16

uint8_t ir_dtmcontrol[4] = {DTMCONTROL};
struct scan_field select_dtmcontrol = {
	.in_value = NULL,
	.out_value = ir_dtmcontrol
};
uint8_t ir_dbus[4] = {DBUS};
struct scan_field select_dbus = {
	.in_value = NULL,
	.out_value = ir_dbus
};
uint8_t ir_idcode[4] = {0x1};
struct scan_field select_idcode = {
	.in_value = NULL,
	.out_value = ir_idcode
};

bscan_tunnel_type_t bscan_tunnel_type;
int bscan_tunnel_ir_width; /* if zero, then tunneling is not present/active */

static uint8_t bscan_zero[4] = {0};
static uint8_t bscan_one[4] = {1};

uint8_t ir_user4[4] = {0x23};
struct scan_field select_user4 = {
	.in_value = NULL,
	.out_value = ir_user4
};


uint8_t bscan_tunneled_ir_width[4] = {5};  /* overridden by assignment in riscv_init_target */
struct scan_field _bscan_tunnel_data_register_select_dmi[] = {
		{
			.num_bits = 3,
			.out_value = bscan_zero,
			.in_value = NULL,
		},
		{
			.num_bits = 5, /* initialized in riscv_init_target to ir width of DM */
			.out_value = ir_dbus,
			.in_value = NULL,
		},
		{
			.num_bits = 7,
			.out_value = bscan_tunneled_ir_width,
			.in_value = NULL,
		},
		{
			.num_bits = 1,
			.out_value = bscan_zero,
			.in_value = NULL,
		}
};

struct scan_field _bscan_tunnel_nested_tap_select_dmi[] = {
		{
			.num_bits = 1,
			.out_value = bscan_zero,
			.in_value = NULL,
		},
		{
			.num_bits = 7,
			.out_value = bscan_tunneled_ir_width,
			.in_value = NULL,
		},
		{
			.num_bits = 0, /* initialized in riscv_init_target to ir width of DM */
			.out_value = ir_dbus,
			.in_value = NULL,
		},
		{
			.num_bits = 3,
			.out_value = bscan_zero,
			.in_value = NULL,
		}
};
struct scan_field *bscan_tunnel_nested_tap_select_dmi = _bscan_tunnel_nested_tap_select_dmi;
uint32_t bscan_tunnel_nested_tap_select_dmi_num_fields = DIM(_bscan_tunnel_nested_tap_select_dmi);

struct scan_field *bscan_tunnel_data_register_select_dmi = _bscan_tunnel_data_register_select_dmi;
uint32_t bscan_tunnel_data_register_select_dmi_num_fields = DIM(_bscan_tunnel_data_register_select_dmi);

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
int riscv_command_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;

/* Wall-clock timeout after reset. Settable via RISC-V Target commands.*/
int riscv_reset_timeout_sec = DEFAULT_RESET_TIMEOUT_SEC;

bool riscv_prefer_sba;
bool riscv_enable_virt2phys = true;
bool riscv_ebreakm = true;
bool riscv_ebreaks = true;
bool riscv_ebreaku = true;

bool riscv_enable_virtual;

typedef struct {
	uint16_t low, high;
} range_t;

/* In addition to the ones in the standard spec, we'll also expose additional
 * CSRs in this list.
 * The list is either NULL, or a series of ranges (inclusive), terminated with
 * 1,0. */
range_t *expose_csr;
/* Same, but for custom registers. */
range_t *expose_custom;

static enum {
	RO_NORMAL,
	RO_REVERSED
} resume_order;

virt2phys_info_t sv32 = {
	.name = "Sv32",
	.va_bits = 32,
	.level = 2,
	.pte_shift = 2,
	.vpn_shift = {12, 22},
	.vpn_mask = {0x3ff, 0x3ff},
	.pte_ppn_shift = {10, 20},
	.pte_ppn_mask = {0x3ff, 0xfff},
	.pa_ppn_shift = {12, 22},
	.pa_ppn_mask = {0x3ff, 0xfff},
};

virt2phys_info_t sv39 = {
	.name = "Sv39",
	.va_bits = 39,
	.level = 3,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30},
	.vpn_mask = {0x1ff, 0x1ff, 0x1ff},
	.pte_ppn_shift = {10, 19, 28},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x3ffffff},
	.pa_ppn_shift = {12, 21, 30},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x3ffffff},
};

virt2phys_info_t sv48 = {
	.name = "Sv48",
	.va_bits = 48,
	.level = 4,
	.pte_shift = 3,
	.vpn_shift = {12, 21, 30, 39},
	.vpn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ff},
	.pte_ppn_shift = {10, 19, 28, 37},
	.pte_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ffff},
	.pa_ppn_shift = {12, 21, 30, 39},
	.pa_ppn_mask = {0x1ff, 0x1ff, 0x1ff, 0x1ffff},
};

static int riscv_resume_go_all_harts(struct target *target);

void select_dmi_via_bscan(struct target *target)
{
	jtag_add_ir_scan(target->tap, &select_user4, TAP_IDLE);
	if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER)
		jtag_add_dr_scan(target->tap, bscan_tunnel_data_register_select_dmi_num_fields,
										bscan_tunnel_data_register_select_dmi, TAP_IDLE);
	else /* BSCAN_TUNNEL_NESTED_TAP */
		jtag_add_dr_scan(target->tap, bscan_tunnel_nested_tap_select_dmi_num_fields,
										bscan_tunnel_nested_tap_select_dmi, TAP_IDLE);
}

uint32_t dtmcontrol_scan_via_bscan(struct target *target, uint32_t out)
{
	/* On BSCAN TAP: Select IR=USER4, issue tunneled IR scan via BSCAN TAP's DR */
	uint8_t tunneled_ir_width[4] = {bscan_tunnel_ir_width};
	uint8_t tunneled_dr_width[4] = {32};
	uint8_t out_value[5] = {0};
	uint8_t in_value[5] = {0};

	buf_set_u32(out_value, 0, 32, out);
	struct scan_field tunneled_ir[4] = {};
	struct scan_field tunneled_dr[4] = {};

	if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER) {
		tunneled_ir[0].num_bits = 3;
		tunneled_ir[0].out_value = bscan_zero;
		tunneled_ir[0].in_value = NULL;
		tunneled_ir[1].num_bits = bscan_tunnel_ir_width;
		tunneled_ir[1].out_value = ir_dtmcontrol;
		tunneled_ir[1].in_value = NULL;
		tunneled_ir[2].num_bits = 7;
		tunneled_ir[2].out_value = tunneled_ir_width;
		tunneled_ir[2].in_value = NULL;
		tunneled_ir[3].num_bits = 1;
		tunneled_ir[3].out_value = bscan_zero;
		tunneled_ir[3].in_value = NULL;

		tunneled_dr[0].num_bits = 3;
		tunneled_dr[0].out_value = bscan_zero;
		tunneled_dr[0].in_value = NULL;
		tunneled_dr[1].num_bits = 32 + 1;
		tunneled_dr[1].out_value = out_value;
		tunneled_dr[1].in_value = in_value;
		tunneled_dr[2].num_bits = 7;
		tunneled_dr[2].out_value = tunneled_dr_width;
		tunneled_dr[2].in_value = NULL;
		tunneled_dr[3].num_bits = 1;
		tunneled_dr[3].out_value = bscan_one;
		tunneled_dr[3].in_value = NULL;
	} else {
		/* BSCAN_TUNNEL_NESTED_TAP */
		tunneled_ir[3].num_bits = 3;
		tunneled_ir[3].out_value = bscan_zero;
		tunneled_ir[3].in_value = NULL;
		tunneled_ir[2].num_bits = bscan_tunnel_ir_width;
		tunneled_ir[2].out_value = ir_dtmcontrol;
		tunneled_ir[1].in_value = NULL;
		tunneled_ir[1].num_bits = 7;
		tunneled_ir[1].out_value = tunneled_ir_width;
		tunneled_ir[2].in_value = NULL;
		tunneled_ir[0].num_bits = 1;
		tunneled_ir[0].out_value = bscan_zero;
		tunneled_ir[0].in_value = NULL;

		tunneled_dr[3].num_bits = 3;
		tunneled_dr[3].out_value = bscan_zero;
		tunneled_dr[3].in_value = NULL;
		tunneled_dr[2].num_bits = 32 + 1;
		tunneled_dr[2].out_value = out_value;
		tunneled_dr[2].in_value = in_value;
		tunneled_dr[1].num_bits = 7;
		tunneled_dr[1].out_value = tunneled_dr_width;
		tunneled_dr[1].in_value = NULL;
		tunneled_dr[0].num_bits = 1;
		tunneled_dr[0].out_value = bscan_one;
		tunneled_dr[0].in_value = NULL;
	}
	jtag_add_ir_scan(target->tap, &select_user4, TAP_IDLE);
	jtag_add_dr_scan(target->tap, DIM(tunneled_ir), tunneled_ir, TAP_IDLE);
	jtag_add_dr_scan(target->tap, DIM(tunneled_dr), tunneled_dr, TAP_IDLE);
	select_dmi_via_bscan(target);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}
	/* Note the starting offset is bit 1, not bit 0.  In BSCAN tunnel, there is a one-bit TCK skew between
	   output and input */
	uint32_t in = buf_get_u32(in_value, 1, 32);
	LOG_DEBUG("DTMCS: 0x%x -> 0x%x", out, in);

	return in;
}



static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4] = { 0 };

	if (bscan_tunnel_ir_width != 0)
		return dtmcontrol_scan_via_bscan(target, out);


	buf_set_u32(out_value, 0, 32, out);

	jtag_add_ir_scan(target->tap, &select_dtmcontrol, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = out_value;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCONTROL: 0x%x -> 0x%x", out, in);

	return in;
}

static struct target_type *get_target_type(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	if (!info) {
		LOG_ERROR("Target has not been initialized");
		return NULL;
	}

	switch (info->dtm_version) {
		case 0:
			return &riscv011_target;
		case 1:
			return &riscv013_target;
		default:
			LOG_ERROR("Unsupported DTM version: %d", info->dtm_version);
			return NULL;
	}
}

static int riscv_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("riscv_init_target()");
	target->arch_info = calloc(1, sizeof(riscv_info_t));
	if (!target->arch_info)
		return ERROR_FAIL;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	riscv_info_init(target, info);
	info->cmd_ctx = cmd_ctx;

	select_dtmcontrol.num_bits = target->tap->ir_length;
	select_dbus.num_bits = target->tap->ir_length;
	select_idcode.num_bits = target->tap->ir_length;

	if (bscan_tunnel_ir_width != 0) {
		select_user4.num_bits = target->tap->ir_length;
		bscan_tunneled_ir_width[0] = bscan_tunnel_ir_width;
		if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER)
			bscan_tunnel_data_register_select_dmi[1].num_bits = bscan_tunnel_ir_width;
		else /* BSCAN_TUNNEL_NESTED_TAP */
			bscan_tunnel_nested_tap_select_dmi[2].num_bits = bscan_tunnel_ir_width;
	}

	riscv_semihosting_init(target);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static void riscv_free_registers(struct target *target)
{
	/* Free the shared structure use for most registers. */
	if (target->reg_cache) {
		if (target->reg_cache->reg_list) {
			free(target->reg_cache->reg_list[0].arch_info);
			/* Free the ones we allocated separately. */
			for (unsigned i = GDB_REGNO_COUNT; i < target->reg_cache->num_regs; i++)
				free(target->reg_cache->reg_list[i].arch_info);
			free(target->reg_cache->reg_list);
		}
		free(target->reg_cache);
	}
}

static void riscv_deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	struct target_type *tt = get_target_type(target);
	if (tt) {
		tt->deinit_target(target);
		riscv_info_t *info = (riscv_info_t *) target->arch_info;
		free(info->reg_names);
		free(info);
	}

	riscv_free_registers(target);

	target->arch_info = NULL;
}

static void trigger_from_breakpoint(struct trigger *trigger,
		const struct breakpoint *breakpoint)
{
	trigger->address = breakpoint->address;
	trigger->length = breakpoint->length;
	trigger->mask = ~0LL;
	trigger->read = false;
	trigger->write = false;
	trigger->execute = true;
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = breakpoint->unique_id;
}

static int maybe_add_trigger_t1(struct target *target, unsigned hartid,
		struct trigger *trigger, uint64_t tdata1)
{
	RISCV_INFO(r);

	const uint32_t bpcontrol_x = 1<<0;
	const uint32_t bpcontrol_w = 1<<1;
	const uint32_t bpcontrol_r = 1<<2;
	const uint32_t bpcontrol_u = 1<<3;
	const uint32_t bpcontrol_s = 1<<4;
	const uint32_t bpcontrol_h = 1<<5;
	const uint32_t bpcontrol_m = 1<<6;
	const uint32_t bpcontrol_bpmatch = 0xf << 7;
	const uint32_t bpcontrol_bpaction = 0xff << 11;

	if (tdata1 & (bpcontrol_r | bpcontrol_w | bpcontrol_x)) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	tdata1 = set_field(tdata1, bpcontrol_r, trigger->read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = set_field(tdata1, bpcontrol_u,
			!!(r->misa[hartid] & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_s,
			!!(r->misa[hartid] & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_h,
			!!(r->misa[hartid] & (1 << ('H' - 'A'))));
	tdata1 |= bpcontrol_m;
	tdata1 = set_field(tdata1, bpcontrol_bpmatch, 0); /* exact match */
	tdata1 = set_field(tdata1, bpcontrol_bpaction, 0); /* cause bp exception */

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	riscv_reg_t tdata1_rb;
	if (riscv_get_register_on_hart(target, &tdata1_rb, hartid,
				GDB_REGNO_TDATA1) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

static int maybe_add_trigger_t2(struct target *target, unsigned hartid,
		struct trigger *trigger, uint64_t tdata1)
{
	RISCV_INFO(r);

	/* tselect is already set */
	if (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD)) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* address/data match trigger */
	tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
	tdata1 = set_field(tdata1, MCONTROL_ACTION,
			MCONTROL_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
	tdata1 |= MCONTROL_M;
	if (r->misa[hartid] & (1 << ('H' - 'A')))
		tdata1 |= MCONTROL_H;
	if (r->misa[hartid] & (1 << ('S' - 'A')))
		tdata1 |= MCONTROL_S;
	if (r->misa[hartid] & (1 << ('U' - 'A')))
		tdata1 |= MCONTROL_U;

	if (trigger->execute)
		tdata1 |= MCONTROL_EXECUTE;
	if (trigger->read)
		tdata1 |= MCONTROL_LOAD;
	if (trigger->write)
		tdata1 |= MCONTROL_STORE;

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	uint64_t tdata1_rb;
	int result = riscv_get_register_on_hart(target, &tdata1_rb, hartid, GDB_REGNO_TDATA1);
	if (result != ERROR_OK)
		return result;
	LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

static int add_trigger(struct target *target, struct trigger *trigger)
{
	RISCV_INFO(r);

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	/* In RTOS mode, we need to set the same trigger in the same slot on every
	 * hart, to keep up the illusion that each hart is a thread running on the
	 * same core. */

	/* Otherwise, we just set the trigger on the one hart this target deals
	 * with. */

	riscv_reg_t tselect[RISCV_MAX_HARTS];

	int first_hart = -1;
	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		if (first_hart < 0)
			first_hart = hartid;
		int result = riscv_get_register_on_hart(target, &tselect[hartid],
				hartid, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
	}
	assert(first_hart >= 0);

	unsigned int i;
	for (i = 0; i < r->trigger_count[first_hart]; i++) {
		if (r->trigger_unique_id[i] != -1)
			continue;

		riscv_set_register_on_hart(target, first_hart, GDB_REGNO_TSELECT, i);

		uint64_t tdata1;
		int result = riscv_get_register_on_hart(target, &tdata1, first_hart,
				GDB_REGNO_TDATA1);
		if (result != ERROR_OK)
			return result;
		int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

		result = ERROR_OK;
		for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
			if (!riscv_hart_enabled(target, hartid))
				continue;
			if (hartid > first_hart)
				riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
			switch (type) {
				case 1:
					result = maybe_add_trigger_t1(target, hartid, trigger, tdata1);
					break;
				case 2:
					result = maybe_add_trigger_t2(target, hartid, trigger, tdata1);
					break;
				default:
					LOG_DEBUG("trigger %d has unknown type %d", i, type);
					continue;
			}

			if (result != ERROR_OK)
				continue;
		}

		if (result != ERROR_OK)
			continue;

		LOG_DEBUG("[%d] Using trigger %d (type %d) for bp %d", target->coreid,
				i, type, trigger->unique_id);
		r->trigger_unique_id[i] = trigger->unique_id;
		break;
	}

	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT,
				tselect[hartid]);
	}

	if (i >= r->trigger_count[first_hart]) {
		LOG_ERROR("Couldn't find an available hardware trigger.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	LOG_DEBUG("[%d] @0x%" TARGET_PRIxADDR, target->coreid, breakpoint->address);
	assert(breakpoint);
	if (breakpoint->type == BKPT_SOFT) {
		/** @todo check RVC for size/alignment */
		if (!(breakpoint->length == 4 || breakpoint->length == 2)) {
			LOG_ERROR("Invalid breakpoint length %d", breakpoint->length);
			return ERROR_FAIL;
		}

		if (0 != (breakpoint->address % 2)) {
			LOG_ERROR("Invalid breakpoint alignment for address 0x%" TARGET_PRIxADDR, breakpoint->address);
			return ERROR_FAIL;
		}

		if (target_read_memory(target, breakpoint->address, 2, breakpoint->length / 2,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to read original instruction at 0x%" TARGET_PRIxADDR,
					breakpoint->address);
			return ERROR_FAIL;
		}

		uint8_t buff[4] = { 0 };
		buf_set_u32(buff, 0, breakpoint->length * CHAR_BIT, breakpoint->length == 4 ? ebreak() : ebreak_c());
		int const retval = target_write_memory(target, breakpoint->address, 2, breakpoint->length / 2, buff);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write %d-byte breakpoint instruction at 0x%"
					TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int const result = add_trigger(target, &trigger);
		if (result != ERROR_OK)
			return result;
	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = true;
	return ERROR_OK;
}

static int remove_trigger(struct target *target, struct trigger *trigger)
{
	RISCV_INFO(r);

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	int first_hart = -1;
	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		if (first_hart < 0) {
			first_hart = hartid;
			break;
		}
	}
	assert(first_hart >= 0);

	unsigned int i;
	for (i = 0; i < r->trigger_count[first_hart]; i++) {
		if (r->trigger_unique_id[i] == trigger->unique_id)
			break;
	}
	if (i >= r->trigger_count[first_hart]) {
		LOG_ERROR("Couldn't find the hardware resources used by hardware "
				"trigger.");
		return ERROR_FAIL;
	}
	LOG_DEBUG("[%d] Stop using resource %d for bp %d", target->coreid, i,
			trigger->unique_id);
	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;
		riscv_reg_t tselect;
		int result = riscv_get_register_on_hart(target, &tselect, hartid, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);
	}
	r->trigger_unique_id[i] = -1;

	return ERROR_OK;
}

int riscv_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		if (target_write_memory(target, breakpoint->address, 2, breakpoint->length / 2,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to restore instruction for %d-byte breakpoint at "
					"0x%" TARGET_PRIxADDR, breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = remove_trigger(target, &trigger);
		if (result != ERROR_OK)
			return result;

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = false;

	return ERROR_OK;
}

static void trigger_from_watchpoint(struct trigger *trigger,
		const struct watchpoint *watchpoint)
{
	trigger->address = watchpoint->address;
	trigger->length = watchpoint->length;
	trigger->mask = watchpoint->mask;
	trigger->value = watchpoint->value;
	trigger->read = (watchpoint->rw == WPT_READ || watchpoint->rw == WPT_ACCESS);
	trigger->write = (watchpoint->rw == WPT_WRITE || watchpoint->rw == WPT_ACCESS);
	trigger->execute = false;
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = watchpoint->unique_id;
}

int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = add_trigger(target, &trigger);
	if (result != ERROR_OK)
		return result;
	watchpoint->set = true;

	return ERROR_OK;
}

int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	LOG_DEBUG("[%d] @0x%" TARGET_PRIxADDR, target->coreid, watchpoint->address);

	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = remove_trigger(target, &trigger);
	if (result != ERROR_OK)
		return result;
	watchpoint->set = false;

	return ERROR_OK;
}

/* Sets *hit_watchpoint to the first watchpoint identified as causing the
 * current halt.
 *
 * The GDB server uses this information to tell GDB what data address has
 * been hit, which enables GDB to print the hit variable along with its old
 * and new value. */
int riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	struct watchpoint *wp = target->watchpoints;

	if (riscv_rtos_enabled(target))
		riscv_set_current_hartid(target, target->rtos->current_thread - 1);
	LOG_DEBUG("Current hartid = %d", riscv_current_hartid(target));

	/*TODO instead of disassembling the instruction that we think caused the
	 * trigger, check the hit bit of each watchpoint first. The hit bit is
	 * simpler and more reliable to check but as it is optional and relatively
	 * new, not all hardware will implement it  */
	riscv_reg_t dpc;
	riscv_get_register(target, &dpc, GDB_REGNO_DPC);
	const uint8_t length = 4;
	LOG_DEBUG("dpc is 0x%" PRIx64, dpc);

	/* fetch the instruction at dpc */
	uint8_t buffer[length];
	if (target_read_buffer(target, dpc, length, buffer) != ERROR_OK) {
		LOG_ERROR("Failed to read instruction at dpc 0x%" PRIx64, dpc);
		return ERROR_FAIL;
	}

	uint32_t instruction = 0;

	for (int i = 0; i < length; i++) {
		LOG_DEBUG("Next byte is %x", buffer[i]);
		instruction += (buffer[i] << 8 * i);
	}
	LOG_DEBUG("Full instruction is %x", instruction);

	/* find out which memory address is accessed by the instruction at dpc */
	/* opcode is first 7 bits of the instruction */
	uint8_t opcode = instruction & 0x7F;
	uint32_t rs1;
	int16_t imm;
	riscv_reg_t mem_addr;

	if (opcode == MATCH_LB || opcode == MATCH_SB) {
		rs1 = (instruction & 0xf8000) >> 15;
		riscv_get_register(target, &mem_addr, rs1);

		if (opcode == MATCH_SB) {
			LOG_DEBUG("%x is store instruction", instruction);
			imm = ((instruction & 0xf80) >> 7) | ((instruction & 0xfe000000) >> 20);
		} else {
			LOG_DEBUG("%x is load instruction", instruction);
			imm = (instruction & 0xfff00000) >> 20;
		}
		/* sign extend 12-bit imm to 16-bits */
		if (imm & (1 << 11))
			imm |= 0xf000;
		mem_addr += imm;
		LOG_DEBUG("memory address=0x%" PRIx64, mem_addr);
	} else {
		LOG_DEBUG("%x is not a RV32I load or store", instruction);
		return ERROR_FAIL;
	}

	while (wp) {
		/*TODO support length/mask */
		if (wp->address == mem_addr) {
			*hit_watchpoint = wp;
			LOG_DEBUG("Hit address=%" TARGET_PRIxADDR, wp->address);
			return ERROR_OK;
		}
		wp = wp->next;
	}

	/* No match found - either we hit a watchpoint caused by an instruction that
	 * this function does not yet disassemble, or we hit a breakpoint.
	 *
	 * OpenOCD will behave as if this function had never been implemented i.e.
	 * report the halt to GDB with no address information. */
	return ERROR_FAIL;
}


static int oldriscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	struct target_type *tt = get_target_type(target);
	return tt->step(target, current, address, handle_breakpoints);
}

static int old_or_new_riscv_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	RISCV_INFO(r);
	LOG_DEBUG("handle_breakpoints=%d", handle_breakpoints);
	if (r->is_halted == NULL)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else
		return riscv_openocd_step(target, current, address, handle_breakpoints);
}


static int riscv_examine(struct target *target)
{
	LOG_DEBUG("riscv_examine()");
	if (target_was_examined(target)) {
		LOG_DEBUG("Target was already examined.");
		return ERROR_OK;
	}

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("  version=0x%x", info->dtm_version);

	struct target_type *tt = get_target_type(target);
	if (tt == NULL)
		return ERROR_FAIL;

	int result = tt->init_target(info->cmd_ctx, target);
	if (result != ERROR_OK)
		return result;

	return tt->examine(target);
}

static int oldriscv_poll(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->poll(target);
}

static int old_or_new_riscv_poll(struct target *target)
{
	RISCV_INFO(r);
	if (r->is_halted == NULL)
		return oldriscv_poll(target);
	else
		return riscv_openocd_poll(target);
}

int halt_prep(struct target *target)
{
	RISCV_INFO(r);
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		LOG_DEBUG("[%s] prep hart, debug_reason=%d", target_name(target),
				  target->debug_reason);
		if (riscv_set_current_hartid(target, i) != ERROR_OK)
			return ERROR_FAIL;
		if (riscv_is_halted(target)) {
			LOG_DEBUG("Hart %d is already halted (reason=%d).", i,
					  target->debug_reason);
		} else {
			if (r->halt_prep(target) != ERROR_OK)
				return ERROR_FAIL;
			r->prepped = true;
		}
	}
	return ERROR_OK;
}

int riscv_halt_go_all_harts(struct target *target)
{
	RISCV_INFO(r);
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		if (riscv_set_current_hartid(target, i) != ERROR_OK)
			return ERROR_FAIL;
		if (riscv_is_halted(target)) {
			LOG_DEBUG("Hart %d is already halted.", i);
		} else {
			if (r->halt_go(target) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	riscv_invalidate_register_cache(target);

	return ERROR_OK;
}

int halt_go(struct target *target)
{
	riscv_info_t *r = riscv_info(target);
	int result;
	if (r->is_halted == NULL) {
		struct target_type *tt = get_target_type(target);
		result = tt->halt(target);
	} else {
		result = riscv_halt_go_all_harts(target);
	}
	target->state = TARGET_HALTED;
	if (target->debug_reason == DBG_REASON_NOTHALTED)
		target->debug_reason = DBG_REASON_DBGRQ;

	return result;
}

static int halt_finish(struct target *target)
{
	return target_call_event_callbacks(target, TARGET_EVENT_HALTED);
}

int riscv_halt(struct target *target)
{
	RISCV_INFO(r);

	if (r->is_halted == NULL) {
		struct target_type *tt = get_target_type(target);
		return tt->halt(target);
	}

	LOG_DEBUG("[%d] halting all harts", target->coreid);

	int result = ERROR_OK;
	if (target->smp) {
		for (struct target_list *tlist = target->head; tlist; tlist = tlist->next) {
			struct target *t = tlist->target;
			if (halt_prep(t) != ERROR_OK)
				result = ERROR_FAIL;
		}

		for (struct target_list *tlist = target->head; tlist; tlist = tlist->next) {
			struct target *t = tlist->target;
			riscv_info_t *i = riscv_info(t);
			if (i->prepped) {
				if (halt_go(t) != ERROR_OK)
					result = ERROR_FAIL;
			}
		}

		for (struct target_list *tlist = target->head; tlist; tlist = tlist->next) {
			struct target *t = tlist->target;
			if (halt_finish(t) != ERROR_OK)
				return ERROR_FAIL;
		}

	} else {
		if (halt_prep(target) != ERROR_OK)
			result = ERROR_FAIL;
		if (halt_go(target) != ERROR_OK)
			result = ERROR_FAIL;
		if (halt_finish(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (riscv_rtos_enabled(target)) {
		if (r->rtos_hartid != -1) {
			LOG_DEBUG("halt requested on RTOS hartid %d", r->rtos_hartid);
			target->rtos->current_threadid = r->rtos_hartid + 1;
			target->rtos->current_thread = r->rtos_hartid + 1;
		} else
			LOG_DEBUG("halt requested, but no known RTOS hartid");
	}

	return result;
}

static int riscv_assert_reset(struct target *target)
{
	LOG_DEBUG("[%d]", target->coreid);
	struct target_type *tt = get_target_type(target);
	riscv_invalidate_register_cache(target);
	return tt->assert_reset(target);
}

static int riscv_deassert_reset(struct target *target)
{
	LOG_DEBUG("[%d]", target->coreid);
	struct target_type *tt = get_target_type(target);
	return tt->deassert_reset(target);
}

int riscv_resume_prep_all_harts(struct target *target)
{
	RISCV_INFO(r);
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		LOG_DEBUG("prep hart %d", i);
		if (riscv_set_current_hartid(target, i) != ERROR_OK)
			return ERROR_FAIL;
		if (riscv_is_halted(target)) {
			if (r->resume_prep(target) != ERROR_OK)
				return ERROR_FAIL;
		} else {
			LOG_DEBUG("  hart %d requested resume, but was already resumed", i);
		}
	}

	LOG_DEBUG("[%d] mark as prepped", target->coreid);
	r->prepped = true;

	return ERROR_OK;
}

/* state must be riscv_reg_t state[RISCV_MAX_HWBPS] = {0}; */
static int disable_triggers(struct target *target, riscv_reg_t *state)
{
	RISCV_INFO(r);

	LOG_DEBUG("deal with triggers");

	if (riscv_enumerate_triggers(target) != ERROR_OK)
		return ERROR_FAIL;

	int hartid = riscv_current_hartid(target);
	if (r->manual_hwbp_set) {
		/* Look at every trigger that may have been set. */
		riscv_reg_t tselect;
		if (riscv_get_register(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
			return ERROR_FAIL;
		for (unsigned t = 0; t < r->trigger_count[hartid]; t++) {
			if (riscv_set_register(target, GDB_REGNO_TSELECT, t) != ERROR_OK)
				return ERROR_FAIL;
			riscv_reg_t tdata1;
			if (riscv_get_register(target, &tdata1, GDB_REGNO_TDATA1) != ERROR_OK)
				return ERROR_FAIL;
			if (tdata1 & MCONTROL_DMODE(riscv_xlen(target))) {
				state[t] = tdata1;
				if (riscv_set_register(target, GDB_REGNO_TDATA1, 0) != ERROR_OK)
					return ERROR_FAIL;
			}
		}
		if (riscv_set_register(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
			return ERROR_FAIL;

	} else {
		/* Just go through the triggers we manage. */
		struct watchpoint *watchpoint = target->watchpoints;
		int i = 0;
		while (watchpoint) {
			LOG_DEBUG("watchpoint %d: set=%d", i, watchpoint->set);
			state[i] = watchpoint->set;
			if (watchpoint->set) {
				if (riscv_remove_watchpoint(target, watchpoint) != ERROR_OK)
					return ERROR_FAIL;
			}
			watchpoint = watchpoint->next;
			i++;
		}
	}

	return ERROR_OK;
}

static int enable_triggers(struct target *target, riscv_reg_t *state)
{
	RISCV_INFO(r);

	int hartid = riscv_current_hartid(target);

	if (r->manual_hwbp_set) {
		/* Look at every trigger that may have been set. */
		riscv_reg_t tselect;
		if (riscv_get_register(target, &tselect, GDB_REGNO_TSELECT) != ERROR_OK)
			return ERROR_FAIL;
		for (unsigned t = 0; t < r->trigger_count[hartid]; t++) {
			if (state[t] != 0) {
				if (riscv_set_register(target, GDB_REGNO_TSELECT, t) != ERROR_OK)
					return ERROR_FAIL;
				if (riscv_set_register(target, GDB_REGNO_TDATA1, state[t]) != ERROR_OK)
					return ERROR_FAIL;
			}
		}
		if (riscv_set_register(target, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
			return ERROR_FAIL;

	} else {
		struct watchpoint *watchpoint = target->watchpoints;
		int i = 0;
		while (watchpoint) {
			LOG_DEBUG("watchpoint %d: cleared=%" PRId64, i, state[i]);
			if (state[i]) {
				if (riscv_add_watchpoint(target, watchpoint) != ERROR_OK)
					return ERROR_FAIL;
			}
			watchpoint = watchpoint->next;
			i++;
		}
	}

	return ERROR_OK;
}

/**
 * Get everything ready to resume.
 */
static int resume_prep(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	RISCV_INFO(r);
	LOG_DEBUG("[%d]", target->coreid);

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		/* To be able to run off a trigger, disable all the triggers, step, and
		 * then resume as usual. */
		riscv_reg_t trigger_state[RISCV_MAX_HWBPS] = {0};

		if (disable_triggers(target, trigger_state) != ERROR_OK)
			return ERROR_FAIL;

		if (old_or_new_riscv_step(target, true, 0, false) != ERROR_OK)
			return ERROR_FAIL;

		if (enable_triggers(target, trigger_state) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (r->is_halted) {
		if (riscv_resume_prep_all_harts(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	LOG_DEBUG("[%d] mark as prepped", target->coreid);
	r->prepped = true;

	return ERROR_OK;
}

/**
 * Resume all the harts that have been prepped, as close to instantaneous as
 * possible.
 */
static int resume_go(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	riscv_info_t *r = riscv_info(target);
	int result;
	if (r->is_halted == NULL) {
		struct target_type *tt = get_target_type(target);
		result = tt->resume(target, current, address, handle_breakpoints,
				debug_execution);
	} else {
		result = riscv_resume_go_all_harts(target);
	}

	return result;
}

static int resume_finish(struct target *target)
{
	register_cache_invalidate(target->reg_cache);

	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
}

/**
 * @par single_hart When true, only resume a single hart even if SMP is
 * configured.  This is used to run algorithms on just one hart.
 */
int riscv_resume(
		struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution,
		bool single_hart)
{
	LOG_DEBUG("handle_breakpoints=%d", handle_breakpoints);
	int result = ERROR_OK;
	if (target->smp && !single_hart) {
		for (struct target_list *tlist = target->head; tlist; tlist = tlist->next) {
			struct target *t = tlist->target;
			if (resume_prep(t, current, address, handle_breakpoints,
						debug_execution) != ERROR_OK)
				result = ERROR_FAIL;
		}

		for (struct target_list *tlist = target->head; tlist; tlist = tlist->next) {
			struct target *t = tlist->target;
			riscv_info_t *i = riscv_info(t);
			if (i->prepped) {
				if (resume_go(t, current, address, handle_breakpoints,
							debug_execution) != ERROR_OK)
					result = ERROR_FAIL;
			}
		}

		for (struct target_list *tlist = target->head; tlist; tlist = tlist->next) {
			struct target *t = tlist->target;
			if (resume_finish(t) != ERROR_OK)
				return ERROR_FAIL;
		}

	} else {
		if (resume_prep(target, current, address, handle_breakpoints,
					debug_execution) != ERROR_OK)
			result = ERROR_FAIL;
		if (resume_go(target, current, address, handle_breakpoints,
					debug_execution) != ERROR_OK)
			result = ERROR_FAIL;
		if (resume_finish(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	return result;
}

static int riscv_target_resume(struct target *target, int current, target_addr_t address,
		int handle_breakpoints, int debug_execution)
{
	return riscv_resume(target, current, address, handle_breakpoints,
			debug_execution, false);
}

static int riscv_select_current_hart(struct target *target)
{
	RISCV_INFO(r);
	if (riscv_rtos_enabled(target)) {
		if (r->rtos_hartid == -1)
			r->rtos_hartid = target->rtos->current_threadid - 1;
		return riscv_set_current_hartid(target, r->rtos_hartid);
	} else
		return riscv_set_current_hartid(target, target->coreid);
}

static int riscv_mmu(struct target *target, int *enabled)
{
	if (!riscv_enable_virt2phys) {
		*enabled = 0;
		return ERROR_OK;
	}

	if (riscv_rtos_enabled(target))
		riscv_set_current_hartid(target, target->rtos->current_thread - 1);

	/* Don't use MMU in explicit or effective M (machine) mode */
	riscv_reg_t priv;
	if (riscv_get_register(target, &priv, GDB_REGNO_PRIV) != ERROR_OK) {
		LOG_ERROR("Failed to read priv register.");
		return ERROR_FAIL;
	}

	riscv_reg_t mstatus;
	if (riscv_get_register(target, &mstatus, GDB_REGNO_MSTATUS) != ERROR_OK) {
		LOG_ERROR("Failed to read mstatus register.");
		return ERROR_FAIL;
	}

	if ((get_field(mstatus, MSTATUS_MPRV) ? get_field(mstatus, MSTATUS_MPP) : priv) == PRV_M) {
		LOG_DEBUG("SATP/MMU ignored in Machine mode (mstatus=0x%" PRIx64 ").", mstatus);
		*enabled = 0;
		return ERROR_OK;
	}

	riscv_reg_t satp;
	if (riscv_get_register(target, &satp, GDB_REGNO_SATP) != ERROR_OK) {
		LOG_DEBUG("Couldn't read SATP.");
		/* If we can't read SATP, then there must not be an MMU. */
		*enabled = 0;
		return ERROR_OK;
	}

	if (get_field(satp, RISCV_SATP_MODE(riscv_xlen(target))) == SATP_MODE_OFF) {
		LOG_DEBUG("MMU is disabled.");
		*enabled = 0;
	} else {
		LOG_DEBUG("MMU is enabled.");
		*enabled = 1;
	}

	return ERROR_OK;
}

static int riscv_address_translate(struct target *target,
		target_addr_t virtual, target_addr_t *physical)
{
	RISCV_INFO(r);
	riscv_reg_t satp_value;
	int mode;
	uint64_t ppn_value;
	target_addr_t table_address;
	virt2phys_info_t *info;
	uint64_t pte = 0;
	int i;

	if (riscv_rtos_enabled(target))
		riscv_set_current_hartid(target, target->rtos->current_thread - 1);

	int result = riscv_get_register(target, &satp_value, GDB_REGNO_SATP);
	if (result != ERROR_OK)
		return result;

	unsigned xlen = riscv_xlen(target);
	mode = get_field(satp_value, RISCV_SATP_MODE(xlen));
	switch (mode) {
		case SATP_MODE_SV32:
			info = &sv32;
			break;
		case SATP_MODE_SV39:
			info = &sv39;
			break;
		case SATP_MODE_SV48:
			info = &sv48;
			break;
		case SATP_MODE_OFF:
			LOG_ERROR("No translation or protection." \
				      " (satp: 0x%" PRIx64 ")", satp_value);
			return ERROR_FAIL;
		default:
			LOG_ERROR("The translation mode is not supported." \
				      " (satp: 0x%" PRIx64 ")", satp_value);
			return ERROR_FAIL;
	}
	LOG_DEBUG("virtual=0x%" TARGET_PRIxADDR "; mode=%s", virtual, info->name);

	/* verify bits xlen-1:va_bits-1 are all equal */
	target_addr_t mask = ((target_addr_t)1 << (xlen - (info->va_bits - 1))) - 1;
	target_addr_t masked_msbs = (virtual >> (info->va_bits - 1)) & mask;
	if (masked_msbs != 0 && masked_msbs != mask) {
		LOG_ERROR("Virtual address 0x%" TARGET_PRIxADDR " is not sign-extended "
				"for %s mode.", virtual, info->name);
		return ERROR_FAIL;
	}

	ppn_value = get_field(satp_value, RISCV_SATP_PPN(xlen));
	table_address = ppn_value << RISCV_PGSHIFT;
	i = info->level - 1;
	while (i >= 0) {
		uint64_t vpn = virtual >> info->vpn_shift[i];
		vpn &= info->vpn_mask[i];
		target_addr_t pte_address = table_address +
									(vpn << info->pte_shift);
		uint8_t buffer[8];
		assert(info->pte_shift <= 3);
		int retval = r->read_memory(target, pte_address,
				4, (1 << info->pte_shift) / 4, buffer, 4);
		if (retval != ERROR_OK)
			return ERROR_FAIL;

		if (info->pte_shift == 2)
			pte = buf_get_u32(buffer, 0, 32);
		else
			pte = buf_get_u64(buffer, 0, 64);

		LOG_DEBUG("i=%d; PTE @0x%" TARGET_PRIxADDR " = 0x%" PRIx64, i,
				pte_address, pte);

		if (!(pte & PTE_V) || (!(pte & PTE_R) && (pte & PTE_W)))
			return ERROR_FAIL;

		if ((pte & PTE_R) || (pte & PTE_X)) /* Found leaf PTE. */
			break;

		i--;
		if (i < 0)
			break;
		ppn_value = pte >> PTE_PPN_SHIFT;
		table_address = ppn_value << RISCV_PGSHIFT;
	}

	if (i < 0) {
		LOG_ERROR("Couldn't find the PTE.");
		return ERROR_FAIL;
	}

	/* Make sure to clear out the high bits that may be set. */
	*physical = virtual & (((target_addr_t)1 << info->va_bits) - 1);

	while (i < info->level) {
		ppn_value = pte >> info->pte_ppn_shift[i];
		ppn_value &= info->pte_ppn_mask[i];
		*physical &= ~(((target_addr_t)info->pa_ppn_mask[i]) <<
				info->pa_ppn_shift[i]);
		*physical |= (ppn_value << info->pa_ppn_shift[i]);
		i++;
	}
	LOG_DEBUG("0x%" TARGET_PRIxADDR " -> 0x%" TARGET_PRIxADDR, virtual,
			*physical);

	return ERROR_OK;
}

static int riscv_virt2phys(struct target *target, target_addr_t virtual, target_addr_t *physical)
{
	int enabled;
	if (riscv_mmu(target, &enabled) == ERROR_OK) {
		if (!enabled)
			return ERROR_FAIL;

		if (riscv_address_translate(target, virtual, physical) == ERROR_OK)
			return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int riscv_read_phys_memory(struct target *target, target_addr_t phys_address,
			uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV_INFO(r);
	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	return r->read_memory(target, phys_address, size, count, buffer, size);
}

static int riscv_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (count == 0) {
		LOG_WARNING("0-length read from 0x%" TARGET_PRIxADDR, address);
		return ERROR_OK;
	}

	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	target_addr_t physical_addr;
	if (target->type->virt2phys(target, address, &physical_addr) == ERROR_OK)
		address = physical_addr;

	RISCV_INFO(r);
	return r->read_memory(target, address, size, count, buffer, size);
}

static int riscv_write_phys_memory(struct target *target, target_addr_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	struct target_type *tt = get_target_type(target);
	return tt->write_memory(target, phys_address, size, count, buffer);
}

static int riscv_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (count == 0) {
		LOG_WARNING("0-length write to 0x%" TARGET_PRIxADDR, address);
		return ERROR_OK;
	}

	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	target_addr_t physical_addr;
	if (target->type->virt2phys(target, address, &physical_addr) == ERROR_OK)
		address = physical_addr;

	struct target_type *tt = get_target_type(target);
	return tt->write_memory(target, address, size, count, buffer);
}

static int riscv_get_gdb_reg_list_internal(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class, bool read)
{
	RISCV_INFO(r);
	LOG_DEBUG("rtos_hartid=%d, current_hartid=%d, reg_class=%d, read=%d",
			r->rtos_hartid, r->current_hartid, reg_class, read);

	if (!target->reg_cache) {
		LOG_ERROR("Target not initialized. Return ERROR_FAIL.");
		return ERROR_FAIL;
	}

	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = 33;
			break;
		case REG_CLASS_ALL:
			*reg_list_size = target->reg_cache->num_regs;
			break;
		default:
			LOG_ERROR("Unsupported reg_class: %d", reg_class);
			return ERROR_FAIL;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	for (int i = 0; i < *reg_list_size; i++) {
		assert(!target->reg_cache->reg_list[i].valid ||
				target->reg_cache->reg_list[i].size > 0);
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
		if (read &&
				target->reg_cache->reg_list[i].exist &&
				!target->reg_cache->reg_list[i].valid) {
			if (target->reg_cache->reg_list[i].type->get(
						&target->reg_cache->reg_list[i]) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int riscv_get_gdb_reg_list_noread(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	return riscv_get_gdb_reg_list_internal(target, reg_list, reg_list_size,
			reg_class, false);
}

static int riscv_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	return riscv_get_gdb_reg_list_internal(target, reg_list, reg_list_size,
			reg_class, true);
}

static int riscv_arch_state(struct target *target)
{
	struct target_type *tt = get_target_type(target);
	return tt->arch_state(target);
}

/* Algorithm must end with a software breakpoint instruction. */
static int riscv_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_params,
		struct reg_param *reg_params, target_addr_t entry_point,
		target_addr_t exit_point, int timeout_ms, void *arch_info)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	int hartid = riscv_current_hartid(target);

	if (num_mem_params > 0) {
		LOG_ERROR("Memory parameters are not supported for RISC-V algorithms.");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", 1);
	if (!reg_pc || reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	LOG_DEBUG("saved_pc=0x%" PRIx64, saved_pc);

	uint64_t saved_regs[32];
	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("save %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		if (!r) {
			LOG_ERROR("Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("Register %s is %d bits instead of %d bits.",
					reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK)
			return ERROR_FAIL;
		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);

		if (reg_params[i].direction == PARAM_OUT || reg_params[i].direction == PARAM_IN_OUT) {
			if (r->type->set(r, reg_params[i].value) != ERROR_OK)
				return ERROR_FAIL;
		}
	}


	/* Disable Interrupts before attempting to run the algorithm. */
	uint64_t current_mstatus;
	uint8_t mstatus_bytes[8] = { 0 };

	LOG_DEBUG("Disabling Interrupts");
	struct reg *reg_mstatus = register_get_by_name(target->reg_cache,
			"mstatus", 1);
	if (!reg_mstatus) {
		LOG_ERROR("Couldn't find mstatus!");
		return ERROR_FAIL;
	}

	reg_mstatus->type->get(reg_mstatus);
	current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	uint64_t ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], set_field(current_mstatus,
				ie_mask, 0));

	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Run algorithm */
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	if (riscv_resume(target, 0, entry_point, 0, 0, true) != ERROR_OK)
		return ERROR_FAIL;

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %" PRId64 " ms.", now - start);
			riscv_halt(target);
			old_or_new_riscv_poll(target);
			enum gdb_regno regnums[] = {
				GDB_REGNO_RA, GDB_REGNO_SP, GDB_REGNO_GP, GDB_REGNO_TP,
				GDB_REGNO_T0, GDB_REGNO_T1, GDB_REGNO_T2, GDB_REGNO_FP,
				GDB_REGNO_S1, GDB_REGNO_A0, GDB_REGNO_A1, GDB_REGNO_A2,
				GDB_REGNO_A3, GDB_REGNO_A4, GDB_REGNO_A5, GDB_REGNO_A6,
				GDB_REGNO_A7, GDB_REGNO_S2, GDB_REGNO_S3, GDB_REGNO_S4,
				GDB_REGNO_S5, GDB_REGNO_S6, GDB_REGNO_S7, GDB_REGNO_S8,
				GDB_REGNO_S9, GDB_REGNO_S10, GDB_REGNO_S11, GDB_REGNO_T3,
				GDB_REGNO_T4, GDB_REGNO_T5, GDB_REGNO_T6,
				GDB_REGNO_PC,
				GDB_REGNO_MSTATUS, GDB_REGNO_MEPC, GDB_REGNO_MCAUSE,
			};
			for (unsigned i = 0; i < DIM(regnums); i++) {
				enum gdb_regno regno = regnums[i];
				riscv_reg_t reg_value;
				if (riscv_get_register(target, &reg_value, regno) != ERROR_OK)
					break;
				LOG_ERROR("%s = 0x%" PRIx64, gdb_regno_name(regno), reg_value);
			}
			return ERROR_TARGET_TIMEOUT;
		}

		int result = old_or_new_riscv_poll(target);
		if (result != ERROR_OK)
			return result;
	}

	/* The current hart id might have been changed in poll(). */
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return ERROR_FAIL;

	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (exit_point && final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	/* Restore Interrupts */
	LOG_DEBUG("Restoring Interrupts");
	buf_set_u64(mstatus_bytes, 0, info->xlen[0], current_mstatus);
	reg_mstatus->type->set(reg_mstatus, mstatus_bytes);

	/* Restore registers */
	uint8_t buf[8] = { 0 };
	buf_set_u64(buf, 0, info->xlen[0], saved_pc);
	if (reg_pc->type->set(reg_pc, buf) != ERROR_OK)
		return ERROR_FAIL;

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN ||
				reg_params[i].direction == PARAM_IN_OUT) {
			struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
			if (r->type->get(r) != ERROR_OK) {
				LOG_ERROR("get(%s) failed", r->name);
				return ERROR_FAIL;
			}
			buf_cpy(r->value, reg_params[i].value, reg_params[i].size);
		}
		LOG_DEBUG("restore %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		buf_set_u64(buf, 0, info->xlen[0], saved_regs[r->number]);
		if (r->type->set(r, buf) != ERROR_OK) {
			LOG_ERROR("set(%s) failed", r->name);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int riscv_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count,
		uint32_t *checksum)
{
	return ERROR_FAIL;
}

/*** OpenOCD Helper Functions ***/

enum riscv_poll_hart {
	RPH_NO_CHANGE,
	RPH_DISCOVERED_HALTED,
	RPH_DISCOVERED_RUNNING,
	RPH_ERROR
};
static enum riscv_poll_hart riscv_poll_hart(struct target *target, int hartid)
{
	RISCV_INFO(r);
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return RPH_ERROR;

	LOG_DEBUG("polling hart %d, target->state=%d", hartid, target->state);

	/* If OpenOCD thinks we're running but this hart is halted then it's time
	 * to raise an event. */
	bool halted = riscv_is_halted(target);
	if (target->state != TARGET_HALTED && halted) {
		LOG_DEBUG("  triggered a halt");
		r->on_halt(target);
		return RPH_DISCOVERED_HALTED;
	} else if (target->state != TARGET_RUNNING && !halted) {
		LOG_DEBUG("  triggered running");
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
		return RPH_DISCOVERED_RUNNING;
	}

	return RPH_NO_CHANGE;
}

int set_debug_reason(struct target *target, enum riscv_halt_reason halt_reason)
{
	switch (halt_reason) {
		case RISCV_HALT_BREAKPOINT:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case RISCV_HALT_TRIGGER:
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case RISCV_HALT_INTERRUPT:
		case RISCV_HALT_GROUP:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case RISCV_HALT_SINGLESTEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case RISCV_HALT_UNKNOWN:
			target->debug_reason = DBG_REASON_UNDEFINED;
			break;
		case RISCV_HALT_ERROR:
			return ERROR_FAIL;
	}
	LOG_DEBUG("[%s] debug_reason=%d", target_name(target), target->debug_reason);
	return ERROR_OK;
}

/*** OpenOCD Interface ***/
int riscv_openocd_poll(struct target *target)
{
	LOG_DEBUG("polling all harts");
	int halted_hart = -1;
	if (riscv_rtos_enabled(target)) {
		/* Check every hart for an event. */
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			enum riscv_poll_hart out = riscv_poll_hart(target, i);
			switch (out) {
			case RPH_NO_CHANGE:
			case RPH_DISCOVERED_RUNNING:
				continue;
			case RPH_DISCOVERED_HALTED:
				halted_hart = i;
				break;
			case RPH_ERROR:
				return ERROR_FAIL;
			}
		}
		if (halted_hart == -1) {
			LOG_DEBUG("  no harts just halted, target->state=%d", target->state);
			return ERROR_OK;
		}
		LOG_DEBUG("  hart %d halted", halted_hart);

		target->state = TARGET_HALTED;
		enum riscv_halt_reason halt_reason = riscv_halt_reason(target, halted_hart);
		if (set_debug_reason(target, halt_reason) != ERROR_OK)
			return ERROR_FAIL;

		target->rtos->current_threadid = halted_hart + 1;
		target->rtos->current_thread = halted_hart + 1;
		riscv_set_rtos_hartid(target, halted_hart);

		/* If we're here then at least one hart triggered.  That means we want
		 * to go and halt _every_ hart (configured with -rtos riscv) in the
		 * system, as that's the invariant we hold here.  Some harts might have
		 * already halted (as we're either in single-step mode or they also
		 * triggered a breakpoint), so don't attempt to halt those harts.
		 * riscv_halt() will do all that for us. */
		riscv_halt(target);

	} else if (target->smp) {
		unsigned halts_discovered = 0;
		unsigned total_targets = 0;
		bool newly_halted[RISCV_MAX_HARTS] = {0};
		unsigned should_remain_halted = 0;
		unsigned should_resume = 0;
		unsigned i = 0;
		for (struct target_list *list = target->head; list != NULL;
				list = list->next, i++) {
			total_targets++;
			struct target *t = list->target;
			riscv_info_t *r = riscv_info(t);
			assert(i < DIM(newly_halted));
			enum riscv_poll_hart out = riscv_poll_hart(t, r->current_hartid);
			switch (out) {
			case RPH_NO_CHANGE:
				break;
			case RPH_DISCOVERED_RUNNING:
				t->state = TARGET_RUNNING;
				t->debug_reason = DBG_REASON_NOTHALTED;
				break;
			case RPH_DISCOVERED_HALTED:
				halts_discovered++;
				newly_halted[i] = true;
				t->state = TARGET_HALTED;
				enum riscv_halt_reason halt_reason =
					riscv_halt_reason(t, r->current_hartid);
				if (set_debug_reason(t, halt_reason) != ERROR_OK)
					return ERROR_FAIL;

				if (halt_reason == RISCV_HALT_BREAKPOINT) {
					int retval;
					switch (riscv_semihosting(t, &retval)) {
					case SEMI_NONE:
					case SEMI_WAITING:
						/* This hart should remain halted. */
						should_remain_halted++;
						break;
					case SEMI_HANDLED:
						/* This hart should be resumed, along with any other
							 * harts that halted due to haltgroups. */
						should_resume++;
						break;
					case SEMI_ERROR:
						return retval;
					}
				} else if (halt_reason != RISCV_HALT_GROUP) {
					should_remain_halted++;
				}
				break;

			case RPH_ERROR:
				return ERROR_FAIL;
			}
		}

		LOG_DEBUG("should_remain_halted=%d, should_resume=%d",
				  should_remain_halted, should_resume);
		if (should_remain_halted && should_resume) {
			LOG_WARNING("%d harts should remain halted, and %d should resume.",
						should_remain_halted, should_resume);
		}
		if (should_remain_halted) {
			LOG_DEBUG("halt all");
			riscv_halt(target);
		} else if (should_resume) {
			LOG_DEBUG("resume all");
			riscv_resume(target, true, 0, 0, 0, false);
		}
		return ERROR_OK;

	} else {
		enum riscv_poll_hart out = riscv_poll_hart(target,
				riscv_current_hartid(target));
		if (out == RPH_NO_CHANGE || out == RPH_DISCOVERED_RUNNING)
			return ERROR_OK;
		else if (out == RPH_ERROR)
			return ERROR_FAIL;

		halted_hart = riscv_current_hartid(target);
		LOG_DEBUG("  hart %d halted", halted_hart);

		enum riscv_halt_reason halt_reason = riscv_halt_reason(target, halted_hart);
		if (set_debug_reason(target, halt_reason) != ERROR_OK)
			return ERROR_FAIL;
		target->state = TARGET_HALTED;
	}

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		int retval;
		switch (riscv_semihosting(target, &retval)) {
			case SEMI_NONE:
			case SEMI_WAITING:
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
				break;
			case SEMI_HANDLED:
				if (riscv_resume(target, true, 0, 0, 0, false) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case SEMI_ERROR:
				return retval;
		}
	} else {
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}

	return ERROR_OK;
}

int riscv_openocd_step(struct target *target, int current,
		target_addr_t address, int handle_breakpoints)
{
	LOG_DEBUG("stepping rtos hart");

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	riscv_reg_t trigger_state[RISCV_MAX_HWBPS] = {0};
	if (disable_triggers(target, trigger_state) != ERROR_OK)
		return ERROR_FAIL;

	int out = riscv_step_rtos_hart(target);
	if (out != ERROR_OK) {
		LOG_ERROR("unable to step rtos hart");
		return out;
	}

	register_cache_invalidate(target->reg_cache);

	if (enable_triggers(target, trigger_state) != ERROR_OK)
		return ERROR_FAIL;

	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_SINGLESTEP;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return out;
}

/* Command Handlers */
COMMAND_HANDLER(riscv_set_command_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int timeout = atoi(CMD_ARGV[0]);
	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	riscv_command_timeout_sec = timeout;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_reset_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int timeout = atoi(CMD_ARGV[0]);
	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	riscv_reset_timeout_sec = timeout;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_test_compliance) {

	struct target *target = get_current_target(CMD_CTX);

	RISCV_INFO(r);

	if (CMD_ARGC > 0) {
		LOG_ERROR("Command does not take any parameters.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (r->test_compliance) {
		return r->test_compliance(target);
	} else {
		LOG_ERROR("This target does not support this command (may implement an older version of the spec).");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_set_prefer_sba)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_prefer_sba);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_enable_virtual)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_enable_virtual);
	return ERROR_OK;
}

void parse_error(const char *string, char c, unsigned position)
{
	char buf[position+2];
	for (unsigned i = 0; i < position; i++)
		buf[i] = ' ';
	buf[position] = '^';
	buf[position + 1] = 0;

	LOG_ERROR("Parse error at character %c in:", c);
	LOG_ERROR("%s", string);
	LOG_ERROR("%s", buf);
}

int parse_ranges(range_t **ranges, const char **argv)
{
	for (unsigned pass = 0; pass < 2; pass++) {
		unsigned range = 0;
		unsigned low = 0;
		bool parse_low = true;
		unsigned high = 0;
		for (unsigned i = 0; i == 0 || argv[0][i-1]; i++) {
			char c = argv[0][i];
			if (isspace(c)) {
				/* Ignore whitespace. */
				continue;
			}

			if (parse_low) {
				if (isdigit(c)) {
					low *= 10;
					low += c - '0';
				} else if (c == '-') {
					parse_low = false;
				} else if (c == ',' || c == 0) {
					if (pass == 1) {
						(*ranges)[range].low = low;
						(*ranges)[range].high = low;
					}
					low = 0;
					range++;
				} else {
					parse_error(argv[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

			} else {
				if (isdigit(c)) {
					high *= 10;
					high += c - '0';
				} else if (c == ',' || c == 0) {
					parse_low = true;
					if (pass == 1) {
						(*ranges)[range].low = low;
						(*ranges)[range].high = high;
					}
					low = 0;
					high = 0;
					range++;
				} else {
					parse_error(argv[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}
		}

		if (pass == 0) {
			free(*ranges);
			*ranges = calloc(range + 2, sizeof(range_t));
			if (!*ranges)
				return ERROR_FAIL;
		} else {
			(*ranges)[range].low = 1;
			(*ranges)[range].high = 0;
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_expose_csrs)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return parse_ranges(&expose_csr, CMD_ARGV);
}

COMMAND_HANDLER(riscv_set_expose_custom)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return parse_ranges(&expose_custom, CMD_ARGV);
}

COMMAND_HANDLER(riscv_authdata_read)
{
	if (CMD_ARGC != 0) {
		LOG_ERROR("Command takes no parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}

	RISCV_INFO(r);
	if (!r) {
		LOG_ERROR("riscv_info is NULL!");
		return ERROR_FAIL;
	}

	if (r->authdata_read) {
		uint32_t value;
		if (r->authdata_read(target, &value) != ERROR_OK)
			return ERROR_FAIL;
		command_print(CMD, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("authdata_read is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_authdata_write)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 argument");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], value);

	if (r->authdata_write) {
		return r->authdata_write(target, value);
	} else {
		LOG_ERROR("authdata_write is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_dmi_read)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}

	RISCV_INFO(r);
	if (!r) {
		LOG_ERROR("riscv_info is NULL!");
		return ERROR_FAIL;
	}

	if (r->dmi_read) {
		uint32_t address, value;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
		if (r->dmi_read(target, &value, address) != ERROR_OK)
			return ERROR_FAIL;
		command_print(CMD, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("dmi_read is not implemented for this target.");
		return ERROR_FAIL;
	}
}


COMMAND_HANDLER(riscv_dmi_write)
{
	if (CMD_ARGC != 2) {
		LOG_ERROR("Command takes exactly 2 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	uint32_t address, value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	if (r->dmi_write) {
		return r->dmi_write(target, address, value);
	} else {
		LOG_ERROR("dmi_write is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_test_sba_config_reg)
{
	if (CMD_ARGC != 4) {
		LOG_ERROR("Command takes exactly 4 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);

	target_addr_t legal_address;
	uint32_t num_words;
	target_addr_t illegal_address;
	bool run_sbbusyerror_test;

	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[0], legal_address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], num_words);
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[2], illegal_address);
	COMMAND_PARSE_ON_OFF(CMD_ARGV[3], run_sbbusyerror_test);

	if (r->test_sba_config_reg) {
		return r->test_sba_config_reg(target, legal_address, num_words,
				illegal_address, run_sbbusyerror_test);
	} else {
		LOG_ERROR("test_sba_config_reg is not implemented for this target.");
		return ERROR_FAIL;
	}
}

COMMAND_HANDLER(riscv_reset_delays)
{
	int wait = 0;

	if (CMD_ARGC > 1) {
		LOG_ERROR("Command takes at most one argument");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], wait);

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);
	r->reset_delays_wait = wait;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ir)
{
	if (CMD_ARGC != 2) {
		LOG_ERROR("Command takes exactly 2 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	if (!strcmp(CMD_ARGV[0], "idcode"))
		buf_set_u32(ir_idcode, 0, 32, value);
	else if (!strcmp(CMD_ARGV[0], "dtmcs"))
		buf_set_u32(ir_dtmcontrol, 0, 32, value);
	else if (!strcmp(CMD_ARGV[0], "dmi"))
		buf_set_u32(ir_dbus, 0, 32, value);
	else
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_resume_order)
{
	if (CMD_ARGC > 1) {
		LOG_ERROR("Command takes at most one argument");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!strcmp(CMD_ARGV[0], "normal")) {
		resume_order = RO_NORMAL;
	} else if (!strcmp(CMD_ARGV[0], "reversed")) {
		resume_order = RO_REVERSED;
	} else {
		LOG_ERROR("Unsupported resume order: %s", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_use_bscan_tunnel)
{
	int irwidth = 0;
	int tunnel_type = BSCAN_TUNNEL_NESTED_TAP;

	if (CMD_ARGC > 2) {
		LOG_ERROR("Command takes at most two arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	} else if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], irwidth);
	} else if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], irwidth);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tunnel_type);
	}
	if (tunnel_type == BSCAN_TUNNEL_NESTED_TAP)
		LOG_INFO("Nested Tap based Bscan Tunnel Selected");
	else if (tunnel_type == BSCAN_TUNNEL_DATA_REGISTER)
		LOG_INFO("Simple Register based Bscan Tunnel Selected");
	else
		LOG_INFO("Invalid Tunnel type selected ! : selecting default Nested Tap Type");

	bscan_tunnel_type = tunnel_type;
	bscan_tunnel_ir_width = irwidth;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_enable_virt2phys)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_enable_virt2phys);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ebreakm)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_ebreakm);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ebreaks)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_ebreaks);
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_ebreaku)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_ebreaku);
	return ERROR_OK;
}

static const struct command_registration riscv_exec_command_handlers[] = {
	{
		.name = "test_compliance",
		.handler = riscv_test_compliance,
		.usage = "",
		.mode = COMMAND_EXEC,
		.help = "Runs a basic compliance test suite against the RISC-V Debug Spec."
	},
	{
		.name = "set_command_timeout_sec",
		.handler = riscv_set_command_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "[sec]",
		.help = "Set the wall-clock timeout (in seconds) for individual commands"
	},
	{
		.name = "set_reset_timeout_sec",
		.handler = riscv_set_reset_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "[sec]",
		.help = "Set the wall-clock timeout (in seconds) after reset is deasserted"
	},
	{
		.name = "set_prefer_sba",
		.handler = riscv_set_prefer_sba,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "When on, prefer to use System Bus Access to access memory. "
			"When off (default), prefer to use the Program Buffer to access memory."
	},
	{
		.name = "set_enable_virtual",
		.handler = riscv_set_enable_virtual,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "When on, memory accesses are performed on physical or virtual "
				"memory depending on the current system configuration. "
				"When off (default), all memory accessses are performed on physical memory."
	},
	{
		.name = "expose_csrs",
		.handler = riscv_set_expose_csrs,
		.mode = COMMAND_ANY,
		.usage = "n0[-m0][,n1[-m1]]...",
		.help = "Configure a list of inclusive ranges for CSRs to expose in "
				"addition to the standard ones. This must be executed before "
				"`init`."
	},
	{
		.name = "expose_custom",
		.handler = riscv_set_expose_custom,
		.mode = COMMAND_ANY,
		.usage = "n0[-m0][,n1[-m1]]...",
		.help = "Configure a list of inclusive ranges for custom registers to "
			"expose. custom0 is accessed as abstract register number 0xc000, "
			"etc. This must be executed before `init`."
	},
	{
		.name = "authdata_read",
		.handler = riscv_authdata_read,
		.usage = "",
		.mode = COMMAND_ANY,
		.help = "Return the 32-bit value read from authdata."
	},
	{
		.name = "authdata_write",
		.handler = riscv_authdata_write,
		.mode = COMMAND_ANY,
		.usage = "value",
		.help = "Write the 32-bit value to authdata."
	},
	{
		.name = "dmi_read",
		.handler = riscv_dmi_read,
		.mode = COMMAND_ANY,
		.usage = "address",
		.help = "Perform a 32-bit DMI read at address, returning the value."
	},
	{
		.name = "dmi_write",
		.handler = riscv_dmi_write,
		.mode = COMMAND_ANY,
		.usage = "address value",
		.help = "Perform a 32-bit DMI write of value at address."
	},
	{
		.name = "test_sba_config_reg",
		.handler = riscv_test_sba_config_reg,
		.mode = COMMAND_ANY,
		.usage = "legal_address num_words "
			"illegal_address run_sbbusyerror_test[on/off]",
		.help = "Perform a series of tests on the SBCS register. "
			"Inputs are a legal, 128-byte aligned address and a number of words to "
			"read/write starting at that address (i.e., address range [legal address, "
			"legal_address+word_size*num_words) must be legally readable/writable), "
			"an illegal, 128-byte aligned address for error flag/handling cases, "
			"and whether sbbusyerror test should be run."
	},
	{
		.name = "reset_delays",
		.handler = riscv_reset_delays,
		.mode = COMMAND_ANY,
		.usage = "[wait]",
		.help = "OpenOCD learns how many Run-Test/Idle cycles are required "
			"between scans to avoid encountering the target being busy. This "
			"command resets those learned values after `wait` scans. It's only "
			"useful for testing OpenOCD itself."
	},
	{
		.name = "resume_order",
		.handler = riscv_resume_order,
		.mode = COMMAND_ANY,
		.usage = "normal|reversed",
		.help = "Choose the order that harts are resumed in when `hasel` is not "
			"supported. Normal order is from lowest hart index to highest. "
			"Reversed order is from highest hart index to lowest."
	},
	{
		.name = "set_ir",
		.handler = riscv_set_ir,
		.mode = COMMAND_ANY,
		.usage = "[idcode|dtmcs|dmi] value",
		.help = "Set IR value for specified JTAG register."
	},
	{
		.name = "use_bscan_tunnel",
		.handler = riscv_use_bscan_tunnel,
		.mode = COMMAND_ANY,
		.usage = "value [type]",
		.help = "Enable or disable use of a BSCAN tunnel to reach DM.  Supply "
			"the width of the DM transport TAP's instruction register to "
			"enable.  Supply a value of 0 to disable. Pass A second argument "
			"(optional) to indicate Bscan Tunnel Type {0:(default) NESTED_TAP , "
			"1: DATA_REGISTER}"
	},
	{
		.name = "set_enable_virt2phys",
		.handler = riscv_set_enable_virt2phys,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "When on (default), enable translation from virtual address to "
			"physical address."
	},
	{
		.name = "set_ebreakm",
		.handler = riscv_set_ebreakm,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "Control dcsr.ebreakm. When off, M-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	{
		.name = "set_ebreaks",
		.handler = riscv_set_ebreaks,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "Control dcsr.ebreaks. When off, S-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	{
		.name = "set_ebreaku",
		.handler = riscv_set_ebreaku,
		.mode = COMMAND_ANY,
		.usage = "on|off",
		.help = "Control dcsr.ebreaku. When off, U-mode ebreak instructions "
			"don't trap to OpenOCD. Defaults to on."
	},
	COMMAND_REGISTRATION_DONE
};

/*
 * To be noted that RISC-V targets use the same semihosting commands as
 * ARM targets.
 *
 * The main reason is compatibility with existing tools. For example the
 * Eclipse OpenOCD/SEGGER J-Link/QEMU plug-ins have several widgets to
 * configure semihosting, which generate commands like `arm semihosting
 * enable`.
 * A secondary reason is the fact that the protocol used is exactly the
 * one specified by ARM. If RISC-V will ever define its own semihosting
 * protocol, then a command like `riscv semihosting enable` will make
 * sense, but for now all semihosting commands are prefixed with `arm`.
 */
extern const struct command_registration semihosting_common_handlers[];

const struct command_registration riscv_command_handlers[] = {
	{
		.name = "riscv",
		.mode = COMMAND_ANY,
		.help = "RISC-V Command Group",
		.usage = "",
		.chain = riscv_exec_command_handlers
	},
	{
		.name = "arm",
		.mode = COMMAND_ANY,
		.help = "ARM Command Group",
		.usage = "",
		.chain = semihosting_common_handlers
	},
	COMMAND_REGISTRATION_DONE
};

static unsigned riscv_xlen_nonconst(struct target *target)
{
	return riscv_xlen(target);
}

struct target_type riscv_target = {
	.name = "riscv",

	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = riscv_examine,

	/* poll current target status */
	.poll = old_or_new_riscv_poll,

	.halt = riscv_halt,
	.resume = riscv_target_resume,
	.step = old_or_new_riscv_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,
	.read_phys_memory = riscv_read_phys_memory,
	.write_phys_memory = riscv_write_phys_memory,

	.checksum_memory = riscv_checksum_memory,

	.mmu = riscv_mmu,
	.virt2phys = riscv_virt2phys,

	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = riscv_get_gdb_reg_list_noread,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = riscv_run_algorithm,

	.commands = riscv_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};

/*** RISC-V Interface ***/

void riscv_info_init(struct target *target, riscv_info_t *r)
{
	memset(r, 0, sizeof(*r));
	r->dtm_version = 1;
	r->registers_initialized = false;
	r->current_hartid = target->coreid;

	memset(r->trigger_unique_id, 0xff, sizeof(r->trigger_unique_id));

	for (size_t h = 0; h < RISCV_MAX_HARTS; ++h)
		r->xlen[h] = -1;
}

static int riscv_resume_go_all_harts(struct target *target)
{
	RISCV_INFO(r);

	/* Dummy variables to make mingw32-gcc happy. */
	int first = 0;
	int last = 1;
	int step = 1;
	switch (resume_order) {
		case RO_NORMAL:
			first = 0;
			last = riscv_count_harts(target) - 1;
			step = 1;
			break;
		case RO_REVERSED:
			first = riscv_count_harts(target) - 1;
			last = 0;
			step = -1;
			break;
		default:
			assert(0);
	}

	for (int i = first; i != last + step; i += step) {
		if (!riscv_hart_enabled(target, i))
			continue;

		LOG_DEBUG("resuming hart %d", i);
		if (riscv_set_current_hartid(target, i) != ERROR_OK)
			return ERROR_FAIL;
		if (riscv_is_halted(target)) {
			if (r->resume_go(target) != ERROR_OK)
				return ERROR_FAIL;
		} else {
			LOG_DEBUG("  hart %d requested resume, but was already resumed", i);
		}
	}

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

int riscv_step_rtos_hart(struct target *target)
{
	RISCV_INFO(r);
	int hartid = r->current_hartid;
	if (riscv_rtos_enabled(target)) {
		hartid = r->rtos_hartid;
		if (hartid == -1) {
			LOG_DEBUG("GDB has asked me to step \"any\" thread, so I'm stepping hart 0.");
			hartid = 0;
		}
	}
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("stepping hart %d", hartid);

	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart isn't halted before single step!");
		return ERROR_FAIL;
	}
	riscv_invalidate_register_cache(target);
	r->on_step(target);
	if (r->step_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;
	riscv_invalidate_register_cache(target);
	r->on_halt(target);
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart was not halted after single step!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

bool riscv_supports_extension(struct target *target, int hartid, char letter)
{
	RISCV_INFO(r);
	unsigned num;
	if (letter >= 'a' && letter <= 'z')
		num = letter - 'a';
	else if (letter >= 'A' && letter <= 'Z')
		num = letter - 'A';
	else
		return false;
	return r->misa[hartid] & (1 << num);
}

unsigned riscv_xlen(const struct target *target)
{
	return riscv_xlen_of_hart(target, riscv_current_hartid(target));
}

int riscv_xlen_of_hart(const struct target *target, int hartid)
{
	RISCV_INFO(r);
	assert(r->xlen[hartid] != -1);
	return r->xlen[hartid];
}

bool riscv_rtos_enabled(const struct target *target)
{
	return false;
}

int riscv_set_current_hartid(struct target *target, int hartid)
{
	RISCV_INFO(r);
	if (!r->select_current_hart)
		return ERROR_OK;

	int previous_hartid = riscv_current_hartid(target);
	r->current_hartid = hartid;
	assert(riscv_hart_enabled(target, hartid));
	LOG_DEBUG("setting hartid to %d, was %d", hartid, previous_hartid);
	if (r->select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	/* This might get called during init, in which case we shouldn't be
	 * setting up the register cache. */
	if (target_was_examined(target) && riscv_rtos_enabled(target))
		riscv_invalidate_register_cache(target);

	return ERROR_OK;
}

void riscv_invalidate_register_cache(struct target *target)
{
	RISCV_INFO(r);

	LOG_DEBUG("[%d]", target->coreid);
	register_cache_invalidate(target->reg_cache);
	for (size_t i = 0; i < target->reg_cache->num_regs; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];
		reg->valid = false;
	}

	r->registers_initialized = true;
}

int riscv_current_hartid(const struct target *target)
{
	RISCV_INFO(r);
	return r->current_hartid;
}

void riscv_set_all_rtos_harts(struct target *target)
{
	RISCV_INFO(r);
	r->rtos_hartid = -1;
}

void riscv_set_rtos_hartid(struct target *target, int hartid)
{
	LOG_DEBUG("setting RTOS hartid %d", hartid);
	RISCV_INFO(r);
	r->rtos_hartid = hartid;
}

int riscv_count_harts(struct target *target)
{
	if (target == NULL)
		return 1;
	RISCV_INFO(r);
	if (r == NULL || r->hart_count == NULL)
		return 1;
	return r->hart_count(target);
}

bool riscv_has_register(struct target *target, int hartid, int regid)
{
	return 1;
}

/**
 * If write is true:
 *   return true iff we are guaranteed that the register will contain exactly
 *       the value we just wrote when it's read.
 * If write is false:
 *   return true iff we are guaranteed that the register will read the same
 *       value in the future as the value we just read.
 */
static bool gdb_regno_cacheable(enum gdb_regno regno, bool write)
{
	/* GPRs, FPRs, vector registers are just normal data stores. */
	if (regno <= GDB_REGNO_XPR31 ||
			(regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31) ||
			(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31))
		return true;

	/* Most CSRs won't change value on us, but we can't assume it about rbitrary
	 * CSRs. */
	switch (regno) {
		case GDB_REGNO_DPC:
			return true;

		case GDB_REGNO_VSTART:
		case GDB_REGNO_VXSAT:
		case GDB_REGNO_VXRM:
		case GDB_REGNO_VLENB:
		case GDB_REGNO_VL:
		case GDB_REGNO_VTYPE:
		case GDB_REGNO_MISA:
		case GDB_REGNO_DCSR:
		case GDB_REGNO_DSCRATCH0:
		case GDB_REGNO_MSTATUS:
		case GDB_REGNO_MEPC:
		case GDB_REGNO_MCAUSE:
		case GDB_REGNO_SATP:
			/*
			 * WARL registers might not contain the value we just wrote, but
			 * these ones won't spontaneously change their value either. *
			 */
			return !write;

		case GDB_REGNO_TSELECT:	/* I think this should be above, but then it doesn't work. */
		case GDB_REGNO_TDATA1:	/* Changes value when tselect is changed. */
		case GDB_REGNO_TDATA2:  /* Changse value when tselect is changed. */
		default:
			return false;
	}
}

/**
 * This function is called when the debug user wants to change the value of a
 * register. The new value may be cached, and may not be written until the hart
 * is resumed. */
int riscv_set_register(struct target *target, enum gdb_regno r, riscv_reg_t v)
{
	return riscv_set_register_on_hart(target, riscv_current_hartid(target), r, v);
}

int riscv_set_register_on_hart(struct target *target, int hartid,
		enum gdb_regno regid, uint64_t value)
{
	RISCV_INFO(r);
	LOG_DEBUG("{%d} %s <- %" PRIx64, hartid, gdb_regno_name(regid), value);
	assert(r->set_register);

	/* TODO: Hack to deal with gdb that thinks these registers still exist. */
	if (regid > GDB_REGNO_XPR15 && regid <= GDB_REGNO_XPR31 && value == 0 &&
			riscv_supports_extension(target, hartid, 'E'))
		return ERROR_OK;

	struct reg *reg = &target->reg_cache->reg_list[regid];
	buf_set_u64(reg->value, 0, reg->size, value);

	int result = r->set_register(target, hartid, regid, value);
	if (result == ERROR_OK)
		reg->valid = gdb_regno_cacheable(regid, true);
	else
		reg->valid = false;
	LOG_DEBUG("[%s]{%d} wrote 0x%" PRIx64 " to %s valid=%d",
			  target_name(target), hartid, value, reg->name, reg->valid);
	return result;
}

int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno r)
{
	return riscv_get_register_on_hart(target, value,
			riscv_current_hartid(target), r);
}

int riscv_get_register_on_hart(struct target *target, riscv_reg_t *value,
		int hartid, enum gdb_regno regid)
{
	RISCV_INFO(r);

	struct reg *reg = &target->reg_cache->reg_list[regid];
	if (!reg->exist) {
		LOG_DEBUG("[%s]{%d} %s does not exist.",
				  target_name(target), hartid, gdb_regno_name(regid));
		return ERROR_FAIL;
	}

	if (reg && reg->valid && hartid == riscv_current_hartid(target)) {
		*value = buf_get_u64(reg->value, 0, reg->size);
		LOG_DEBUG("{%d} %s: %" PRIx64 " (cached)", hartid,
				  gdb_regno_name(regid), *value);
		return ERROR_OK;
	}

	/* TODO: Hack to deal with gdb that thinks these registers still exist. */
	if (regid > GDB_REGNO_XPR15 && regid <= GDB_REGNO_XPR31 &&
			riscv_supports_extension(target, hartid, 'E')) {
		*value = 0;
		return ERROR_OK;
	}

	int result = r->get_register(target, value, hartid, regid);

	if (result == ERROR_OK)
		reg->valid = gdb_regno_cacheable(regid, false);

	LOG_DEBUG("{%d} %s: %" PRIx64, hartid, gdb_regno_name(regid), *value);
	return result;
}

bool riscv_is_halted(struct target *target)
{
	RISCV_INFO(r);
	assert(r->is_halted);
	return r->is_halted(target);
}

enum riscv_halt_reason riscv_halt_reason(struct target *target, int hartid)
{
	RISCV_INFO(r);
	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return RISCV_HALT_ERROR;
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart is not halted!");
		return RISCV_HALT_UNKNOWN;
	}
	return r->halt_reason(target);
}

size_t riscv_debug_buffer_size(struct target *target)
{
	RISCV_INFO(r);
	return r->debug_buffer_size[riscv_current_hartid(target)];
}

int riscv_write_debug_buffer(struct target *target, int index, riscv_insn_t insn)
{
	RISCV_INFO(r);
	r->write_debug_buffer(target, index, insn);
	return ERROR_OK;
}

riscv_insn_t riscv_read_debug_buffer(struct target *target, int index)
{
	RISCV_INFO(r);
	return r->read_debug_buffer(target, index);
}

int riscv_execute_debug_buffer(struct target *target)
{
	RISCV_INFO(r);
	return r->execute_debug_buffer(target);
}

void riscv_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d)
{
	RISCV_INFO(r);
	r->fill_dmi_write_u64(target, buf, a, d);
}

void riscv_fill_dmi_read_u64(struct target *target, char *buf, int a)
{
	RISCV_INFO(r);
	r->fill_dmi_read_u64(target, buf, a);
}

void riscv_fill_dmi_nop_u64(struct target *target, char *buf)
{
	RISCV_INFO(r);
	r->fill_dmi_nop_u64(target, buf);
}

int riscv_dmi_write_u64_bits(struct target *target)
{
	RISCV_INFO(r);
	return r->dmi_write_u64_bits(target);
}

bool riscv_hart_enabled(struct target *target, int hartid)
{
	/* FIXME: Add a hart mask to the RTOS. */
	if (riscv_rtos_enabled(target))
		return hartid < riscv_count_harts(target);

	return hartid == target->coreid;
}

/**
 * Count triggers, and initialize trigger_count for each hart.
 * trigger_count is initialized even if this function fails to discover
 * something.
 * Disable any hardware triggers that have dmode set. We can't have set them
 * ourselves. Maybe they're left over from some killed debug session.
 * */
int riscv_enumerate_triggers(struct target *target)
{
	RISCV_INFO(r);

	if (r->triggers_enumerated)
		return ERROR_OK;

	r->triggers_enumerated = true;	/* At the very least we tried. */

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect;
		int result = riscv_get_register_on_hart(target, &tselect, hartid,
				GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;

		for (unsigned t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
			r->trigger_count[hartid] = t;

			/* If we can't write tselect, then this hart does not support triggers. */
			if (riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, t) != ERROR_OK)
				break;
			uint64_t tselect_rb;
			result = riscv_get_register_on_hart(target, &tselect_rb, hartid,
					GDB_REGNO_TSELECT);
			if (result != ERROR_OK)
				return result;
			/* Mask off the top bit, which is used as tdrmode in old
			 * implementations. */
			tselect_rb &= ~(1ULL << (riscv_xlen(target)-1));
			if (tselect_rb != t)
				break;
			uint64_t tdata1;
			result = riscv_get_register_on_hart(target, &tdata1, hartid,
					GDB_REGNO_TDATA1);
			if (result != ERROR_OK)
				return result;

			int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));
			if (type == 0)
				break;
			switch (type) {
				case 1:
					/* On these older cores we don't support software using
					 * triggers. */
					riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
					break;
				case 2:
					if (tdata1 & MCONTROL_DMODE(riscv_xlen(target)))
						riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
					break;
			}
		}

		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);

		LOG_INFO("[%d] Found %d triggers", hartid, r->trigger_count[hartid]);
	}

	return ERROR_OK;
}

const char *gdb_regno_name(enum gdb_regno regno)
{
	static char buf[32];

	switch (regno) {
		case GDB_REGNO_ZERO:
			return "zero";
		case GDB_REGNO_RA:
			return "ra";
		case GDB_REGNO_SP:
			return "sp";
		case GDB_REGNO_GP:
			return "gp";
		case GDB_REGNO_TP:
			return "tp";
		case GDB_REGNO_T0:
			return "t0";
		case GDB_REGNO_T1:
			return "t1";
		case GDB_REGNO_T2:
			return "t2";
		case GDB_REGNO_S0:
			return "s0";
		case GDB_REGNO_S1:
			return "s1";
		case GDB_REGNO_A0:
			return "a0";
		case GDB_REGNO_A1:
			return "a1";
		case GDB_REGNO_A2:
			return "a2";
		case GDB_REGNO_A3:
			return "a3";
		case GDB_REGNO_A4:
			return "a4";
		case GDB_REGNO_A5:
			return "a5";
		case GDB_REGNO_A6:
			return "a6";
		case GDB_REGNO_A7:
			return "a7";
		case GDB_REGNO_S2:
			return "s2";
		case GDB_REGNO_S3:
			return "s3";
		case GDB_REGNO_S4:
			return "s4";
		case GDB_REGNO_S5:
			return "s5";
		case GDB_REGNO_S6:
			return "s6";
		case GDB_REGNO_S7:
			return "s7";
		case GDB_REGNO_S8:
			return "s8";
		case GDB_REGNO_S9:
			return "s9";
		case GDB_REGNO_S10:
			return "s10";
		case GDB_REGNO_S11:
			return "s11";
		case GDB_REGNO_T3:
			return "t3";
		case GDB_REGNO_T4:
			return "t4";
		case GDB_REGNO_T5:
			return "t5";
		case GDB_REGNO_T6:
			return "t6";
		case GDB_REGNO_PC:
			return "pc";
		case GDB_REGNO_FPR0:
			return "fpr0";
		case GDB_REGNO_FPR31:
			return "fpr31";
		case GDB_REGNO_CSR0:
			return "csr0";
		case GDB_REGNO_TSELECT:
			return "tselect";
		case GDB_REGNO_TDATA1:
			return "tdata1";
		case GDB_REGNO_TDATA2:
			return "tdata2";
		case GDB_REGNO_MISA:
			return "misa";
		case GDB_REGNO_DPC:
			return "dpc";
		case GDB_REGNO_DCSR:
			return "dcsr";
		case GDB_REGNO_DSCRATCH0:
			return "dscratch0";
		case GDB_REGNO_MSTATUS:
			return "mstatus";
		case GDB_REGNO_MEPC:
			return "mepc";
		case GDB_REGNO_MCAUSE:
			return "mcause";
		case GDB_REGNO_PRIV:
			return "priv";
		case GDB_REGNO_SATP:
			return "satp";
		case GDB_REGNO_VTYPE:
			return "vtype";
		case GDB_REGNO_VL:
			return "vl";
		case GDB_REGNO_V0:
			return "v0";
		case GDB_REGNO_V1:
			return "v1";
		case GDB_REGNO_V2:
			return "v2";
		case GDB_REGNO_V3:
			return "v3";
		case GDB_REGNO_V4:
			return "v4";
		case GDB_REGNO_V5:
			return "v5";
		case GDB_REGNO_V6:
			return "v6";
		case GDB_REGNO_V7:
			return "v7";
		case GDB_REGNO_V8:
			return "v8";
		case GDB_REGNO_V9:
			return "v9";
		case GDB_REGNO_V10:
			return "v10";
		case GDB_REGNO_V11:
			return "v11";
		case GDB_REGNO_V12:
			return "v12";
		case GDB_REGNO_V13:
			return "v13";
		case GDB_REGNO_V14:
			return "v14";
		case GDB_REGNO_V15:
			return "v15";
		case GDB_REGNO_V16:
			return "v16";
		case GDB_REGNO_V17:
			return "v17";
		case GDB_REGNO_V18:
			return "v18";
		case GDB_REGNO_V19:
			return "v19";
		case GDB_REGNO_V20:
			return "v20";
		case GDB_REGNO_V21:
			return "v21";
		case GDB_REGNO_V22:
			return "v22";
		case GDB_REGNO_V23:
			return "v23";
		case GDB_REGNO_V24:
			return "v24";
		case GDB_REGNO_V25:
			return "v25";
		case GDB_REGNO_V26:
			return "v26";
		case GDB_REGNO_V27:
			return "v27";
		case GDB_REGNO_V28:
			return "v28";
		case GDB_REGNO_V29:
			return "v29";
		case GDB_REGNO_V30:
			return "v30";
		case GDB_REGNO_V31:
			return "v31";
		default:
			if (regno <= GDB_REGNO_XPR31)
				sprintf(buf, "x%d", regno - GDB_REGNO_ZERO);
			else if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095)
				sprintf(buf, "csr%d", regno - GDB_REGNO_CSR0);
			else if (regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31)
				sprintf(buf, "f%d", regno - GDB_REGNO_FPR0);
			else
				sprintf(buf, "gdb_regno_%d", regno);
			return buf;
	}
}

static int register_get(struct reg *reg)
{
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	RISCV_INFO(r);

	if (reg->number >= GDB_REGNO_V0 && reg->number <= GDB_REGNO_V31) {
		if (!r->get_register_buf) {
			LOG_ERROR("Reading register %s not supported on this RISC-V target.",
					gdb_regno_name(reg->number));
			return ERROR_FAIL;
		}

		if (r->get_register_buf(target, reg->value, reg->number) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		uint64_t value;
		int result = riscv_get_register(target, &value, reg->number);
		if (result != ERROR_OK)
			return result;
		buf_set_u64(reg->value, 0, reg->size, value);
	}
	reg->valid = gdb_regno_cacheable(reg->number, false);
	char *str = buf_to_hex_str(reg->value, reg->size);
	LOG_DEBUG("[%d]{%d} read 0x%s from %s (valid=%d)", target->coreid,
			riscv_current_hartid(target), str, reg->name, reg->valid);
	free(str);
	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	riscv_reg_info_t *reg_info = reg->arch_info;
	struct target *target = reg_info->target;
	RISCV_INFO(r);

	char *str = buf_to_hex_str(buf, reg->size);
	LOG_DEBUG("[%d]{%d} write 0x%s to %s (valid=%d)", target->coreid,
			riscv_current_hartid(target), str, reg->name, reg->valid);
	free(str);

	memcpy(reg->value, buf, DIV_ROUND_UP(reg->size, 8));
	reg->valid = gdb_regno_cacheable(reg->number, true);

	if (reg->number == GDB_REGNO_TDATA1 ||
			reg->number == GDB_REGNO_TDATA2) {
		r->manual_hwbp_set = true;
		/* When enumerating triggers, we clear any triggers with DMODE set,
		 * assuming they were left over from a previous debug session. So make
		 * sure that is done before a user might be setting their own triggers.
		 */
		if (riscv_enumerate_triggers(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (reg->number >= GDB_REGNO_V0 && reg->number <= GDB_REGNO_V31) {
		if (!r->set_register_buf) {
			LOG_ERROR("Writing register %s not supported on this RISC-V target.",
					gdb_regno_name(reg->number));
			return ERROR_FAIL;
		}

		if (r->set_register_buf(target, reg->number, reg->value) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		uint64_t value = buf_get_u64(buf, 0, reg->size);
		if (riscv_set_register(target, reg->number, value) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static struct reg_arch_type riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};

struct csr_info {
	unsigned number;
	const char *name;
};

static int cmp_csr_info(const void *p1, const void *p2)
{
	return (int) (((struct csr_info *)p1)->number) - (int) (((struct csr_info *)p2)->number);
}

int riscv_init_registers(struct target *target)
{
	RISCV_INFO(info);

	riscv_free_registers(target);

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	if (!target->reg_cache)
		return ERROR_FAIL;
	target->reg_cache->name = "RISC-V Registers";
	target->reg_cache->num_regs = GDB_REGNO_COUNT;

	if (expose_custom) {
		for (unsigned i = 0; expose_custom[i].low <= expose_custom[i].high; i++) {
			for (unsigned number = expose_custom[i].low;
					number <= expose_custom[i].high;
					number++)
				target->reg_cache->num_regs++;
		}
	}

	LOG_DEBUG("create register cache for %d registers",
			target->reg_cache->num_regs);

	target->reg_cache->reg_list =
		calloc(target->reg_cache->num_regs, sizeof(struct reg));
	if (!target->reg_cache->reg_list)
		return ERROR_FAIL;

	const unsigned int max_reg_name_len = 12;
	free(info->reg_names);
	info->reg_names =
		calloc(target->reg_cache->num_regs, max_reg_name_len);
	if (!info->reg_names)
		return ERROR_FAIL;
	char *reg_name = info->reg_names;

	int hartid = riscv_current_hartid(target);

	static struct reg_feature feature_cpu = {
		.name = "org.gnu.gdb.riscv.cpu"
	};
	static struct reg_feature feature_fpu = {
		.name = "org.gnu.gdb.riscv.fpu"
	};
	static struct reg_feature feature_csr = {
		.name = "org.gnu.gdb.riscv.csr"
	};
	static struct reg_feature feature_vector = {
		.name = "org.gnu.gdb.riscv.vector"
	};
	static struct reg_feature feature_virtual = {
		.name = "org.gnu.gdb.riscv.virtual"
	};
	static struct reg_feature feature_custom = {
		.name = "org.gnu.gdb.riscv.custom"
	};

	/* These types are built into gdb. */
	static struct reg_data_type type_ieee_single = { .type = REG_TYPE_IEEE_SINGLE, .id = "ieee_single" };
	static struct reg_data_type type_ieee_double = { .type = REG_TYPE_IEEE_DOUBLE, .id = "ieee_double" };
	static struct reg_data_type_union_field single_double_fields[] = {
		{"float", &type_ieee_single, single_double_fields + 1},
		{"double", &type_ieee_double, NULL},
	};
	static struct reg_data_type_union single_double_union = {
		.fields = single_double_fields
	};
	static struct reg_data_type type_ieee_single_double = {
		.type = REG_TYPE_ARCH_DEFINED,
		.id = "FPU_FD",
		.type_class = REG_TYPE_CLASS_UNION,
		.reg_type_union = &single_double_union
	};
	static struct reg_data_type type_uint8 = { .type = REG_TYPE_UINT8, .id = "uint8" };
	static struct reg_data_type type_uint16 = { .type = REG_TYPE_UINT16, .id = "uint16" };
	static struct reg_data_type type_uint32 = { .type = REG_TYPE_UINT32, .id = "uint32" };
	static struct reg_data_type type_uint64 = { .type = REG_TYPE_UINT64, .id = "uint64" };
	static struct reg_data_type type_uint128 = { .type = REG_TYPE_UINT128, .id = "uint128" };

	/* This is roughly the XML we want:
	 * <vector id="bytes" type="uint8" count="16"/>
	 * <vector id="shorts" type="uint16" count="8"/>
	 * <vector id="words" type="uint32" count="4"/>
	 * <vector id="longs" type="uint64" count="2"/>
	 * <vector id="quads" type="uint128" count="1"/>
	 * <union id="riscv_vector_type">
	 *   <field name="b" type="bytes"/>
	 *   <field name="s" type="shorts"/>
	 *   <field name="w" type="words"/>
	 *   <field name="l" type="longs"/>
	 *   <field name="q" type="quads"/>
	 * </union>
	 */

	info->vector_uint8.type = &type_uint8;
	info->vector_uint8.count = info->vlenb[hartid];
	info->type_uint8_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint8_vector.id = "bytes";
	info->type_uint8_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint8_vector.reg_type_vector = &info->vector_uint8;

	info->vector_uint16.type = &type_uint16;
	info->vector_uint16.count = info->vlenb[hartid] / 2;
	info->type_uint16_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint16_vector.id = "shorts";
	info->type_uint16_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint16_vector.reg_type_vector = &info->vector_uint16;

	info->vector_uint32.type = &type_uint32;
	info->vector_uint32.count = info->vlenb[hartid] / 4;
	info->type_uint32_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint32_vector.id = "words";
	info->type_uint32_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint32_vector.reg_type_vector = &info->vector_uint32;

	info->vector_uint64.type = &type_uint64;
	info->vector_uint64.count = info->vlenb[hartid] / 8;
	info->type_uint64_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint64_vector.id = "longs";
	info->type_uint64_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint64_vector.reg_type_vector = &info->vector_uint64;

	info->vector_uint128.type = &type_uint128;
	info->vector_uint128.count = info->vlenb[hartid] / 16;
	info->type_uint128_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint128_vector.id = "quads";
	info->type_uint128_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint128_vector.reg_type_vector = &info->vector_uint128;

	info->vector_fields[0].name = "b";
	info->vector_fields[0].type = &info->type_uint8_vector;
	if (info->vlenb[hartid] >= 2) {
		info->vector_fields[0].next = info->vector_fields + 1;
		info->vector_fields[1].name = "s";
		info->vector_fields[1].type = &info->type_uint16_vector;
	} else {
		info->vector_fields[0].next = NULL;
	}
	if (info->vlenb[hartid] >= 4) {
		info->vector_fields[1].next = info->vector_fields + 2;
		info->vector_fields[2].name = "w";
		info->vector_fields[2].type = &info->type_uint32_vector;
	} else {
		info->vector_fields[1].next = NULL;
	}
	if (info->vlenb[hartid] >= 8) {
		info->vector_fields[2].next = info->vector_fields + 3;
		info->vector_fields[3].name = "l";
		info->vector_fields[3].type = &info->type_uint64_vector;
	} else {
		info->vector_fields[2].next = NULL;
	}
	if (info->vlenb[hartid] >= 16) {
		info->vector_fields[3].next = info->vector_fields + 4;
		info->vector_fields[4].name = "q";
		info->vector_fields[4].type = &info->type_uint128_vector;
	} else {
		info->vector_fields[3].next = NULL;
	}
	info->vector_fields[4].next = NULL;

	info->vector_union.fields = info->vector_fields;

	info->type_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_vector.id = "riscv_vector";
	info->type_vector.type_class = REG_TYPE_CLASS_UNION;
	info->type_vector.reg_type_union = &info->vector_union;

	struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "encoding.h"
#undef DECLARE_CSR
	};
	/* encoding.h does not contain the registers in sorted order. */
	qsort(csr_info, DIM(csr_info), sizeof(*csr_info), cmp_csr_info);
	unsigned csr_info_index = 0;

	unsigned custom_range_index = 0;
	int custom_within_range = 0;

	riscv_reg_info_t *shared_reg_info = calloc(1, sizeof(riscv_reg_info_t));
	if (!shared_reg_info)
		return ERROR_FAIL;
	shared_reg_info->target = target;

	/* When gdb requests register N, gdb_get_register_packet() assumes that this
	 * is register at index N in reg_list. So if there are certain registers
	 * that don't exist, we need to leave holes in the list (or renumber, but
	 * it would be nice not to have yet another set of numbers to translate
	 * between). */
	for (uint32_t number = 0; number < target->reg_cache->num_regs; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];
		r->dirty = false;
		r->valid = false;
		r->exist = true;
		r->type = &riscv_reg_arch_type;
		r->arch_info = shared_reg_info;
		r->number = number;
		r->size = riscv_xlen(target);
		/* r->size is set in riscv_invalidate_register_cache, maybe because the
		 * target is in theory allowed to change XLEN on us. But I expect a lot
		 * of other things to break in that case as well. */
		if (number <= GDB_REGNO_XPR31) {
			r->exist = number <= GDB_REGNO_XPR15 ||
				!riscv_supports_extension(target, hartid, 'E');
			/* TODO: For now we fake that all GPRs exist because otherwise gdb
			 * doesn't work. */
			r->exist = true;
			r->caller_save = true;
			switch (number) {
				case GDB_REGNO_ZERO:
					r->name = "zero";
					break;
				case GDB_REGNO_RA:
					r->name = "ra";
					break;
				case GDB_REGNO_SP:
					r->name = "sp";
					break;
				case GDB_REGNO_GP:
					r->name = "gp";
					break;
				case GDB_REGNO_TP:
					r->name = "tp";
					break;
				case GDB_REGNO_T0:
					r->name = "t0";
					break;
				case GDB_REGNO_T1:
					r->name = "t1";
					break;
				case GDB_REGNO_T2:
					r->name = "t2";
					break;
				case GDB_REGNO_FP:
					r->name = "fp";
					break;
				case GDB_REGNO_S1:
					r->name = "s1";
					break;
				case GDB_REGNO_A0:
					r->name = "a0";
					break;
				case GDB_REGNO_A1:
					r->name = "a1";
					break;
				case GDB_REGNO_A2:
					r->name = "a2";
					break;
				case GDB_REGNO_A3:
					r->name = "a3";
					break;
				case GDB_REGNO_A4:
					r->name = "a4";
					break;
				case GDB_REGNO_A5:
					r->name = "a5";
					break;
				case GDB_REGNO_A6:
					r->name = "a6";
					break;
				case GDB_REGNO_A7:
					r->name = "a7";
					break;
				case GDB_REGNO_S2:
					r->name = "s2";
					break;
				case GDB_REGNO_S3:
					r->name = "s3";
					break;
				case GDB_REGNO_S4:
					r->name = "s4";
					break;
				case GDB_REGNO_S5:
					r->name = "s5";
					break;
				case GDB_REGNO_S6:
					r->name = "s6";
					break;
				case GDB_REGNO_S7:
					r->name = "s7";
					break;
				case GDB_REGNO_S8:
					r->name = "s8";
					break;
				case GDB_REGNO_S9:
					r->name = "s9";
					break;
				case GDB_REGNO_S10:
					r->name = "s10";
					break;
				case GDB_REGNO_S11:
					r->name = "s11";
					break;
				case GDB_REGNO_T3:
					r->name = "t3";
					break;
				case GDB_REGNO_T4:
					r->name = "t4";
					break;
				case GDB_REGNO_T5:
					r->name = "t5";
					break;
				case GDB_REGNO_T6:
					r->name = "t6";
					break;
			}
			r->group = "general";
			r->feature = &feature_cpu;
		} else if (number == GDB_REGNO_PC) {
			r->caller_save = true;
			sprintf(reg_name, "pc");
			r->group = "general";
			r->feature = &feature_cpu;
		} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			r->caller_save = true;
			if (riscv_supports_extension(target, hartid, 'D')) {
				r->size = 64;
				if (riscv_supports_extension(target, hartid, 'F'))
					r->reg_data_type = &type_ieee_single_double;
				else
					r->reg_data_type = &type_ieee_double;
			} else if (riscv_supports_extension(target, hartid, 'F')) {
				r->reg_data_type = &type_ieee_single;
				r->size = 32;
			} else {
				r->exist = false;
			}
			switch (number) {
				case GDB_REGNO_FT0:
					r->name = "ft0";
					break;
				case GDB_REGNO_FT1:
					r->name = "ft1";
					break;
				case GDB_REGNO_FT2:
					r->name = "ft2";
					break;
				case GDB_REGNO_FT3:
					r->name = "ft3";
					break;
				case GDB_REGNO_FT4:
					r->name = "ft4";
					break;
				case GDB_REGNO_FT5:
					r->name = "ft5";
					break;
				case GDB_REGNO_FT6:
					r->name = "ft6";
					break;
				case GDB_REGNO_FT7:
					r->name = "ft7";
					break;
				case GDB_REGNO_FS0:
					r->name = "fs0";
					break;
				case GDB_REGNO_FS1:
					r->name = "fs1";
					break;
				case GDB_REGNO_FA0:
					r->name = "fa0";
					break;
				case GDB_REGNO_FA1:
					r->name = "fa1";
					break;
				case GDB_REGNO_FA2:
					r->name = "fa2";
					break;
				case GDB_REGNO_FA3:
					r->name = "fa3";
					break;
				case GDB_REGNO_FA4:
					r->name = "fa4";
					break;
				case GDB_REGNO_FA5:
					r->name = "fa5";
					break;
				case GDB_REGNO_FA6:
					r->name = "fa6";
					break;
				case GDB_REGNO_FA7:
					r->name = "fa7";
					break;
				case GDB_REGNO_FS2:
					r->name = "fs2";
					break;
				case GDB_REGNO_FS3:
					r->name = "fs3";
					break;
				case GDB_REGNO_FS4:
					r->name = "fs4";
					break;
				case GDB_REGNO_FS5:
					r->name = "fs5";
					break;
				case GDB_REGNO_FS6:
					r->name = "fs6";
					break;
				case GDB_REGNO_FS7:
					r->name = "fs7";
					break;
				case GDB_REGNO_FS8:
					r->name = "fs8";
					break;
				case GDB_REGNO_FS9:
					r->name = "fs9";
					break;
				case GDB_REGNO_FS10:
					r->name = "fs10";
					break;
				case GDB_REGNO_FS11:
					r->name = "fs11";
					break;
				case GDB_REGNO_FT8:
					r->name = "ft8";
					break;
				case GDB_REGNO_FT9:
					r->name = "ft9";
					break;
				case GDB_REGNO_FT10:
					r->name = "ft10";
					break;
				case GDB_REGNO_FT11:
					r->name = "ft11";
					break;
			}
			r->group = "float";
			r->feature = &feature_fpu;
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			r->group = "csr";
			r->feature = &feature_csr;
			unsigned csr_number = number - GDB_REGNO_CSR0;

			while (csr_info[csr_info_index].number < csr_number &&
					csr_info_index < DIM(csr_info) - 1) {
				csr_info_index++;
			}
			if (csr_info[csr_info_index].number == csr_number) {
				r->name = csr_info[csr_info_index].name;
			} else {
				sprintf(reg_name, "csr%d", csr_number);
				/* Assume unnamed registers don't exist, unless we have some
				 * configuration that tells us otherwise. That's important
				 * because eg. Eclipse crashes if a target has too many
				 * registers, and apparently has no way of only showing a
				 * subset of registers in any case. */
				r->exist = false;
			}

			switch (csr_number) {
				case CSR_FFLAGS:
				case CSR_FRM:
				case CSR_FCSR:
					r->exist = riscv_supports_extension(target, hartid, 'F');
					r->group = "float";
					r->feature = &feature_fpu;
					break;
				case CSR_SSTATUS:
				case CSR_STVEC:
				case CSR_SIP:
				case CSR_SIE:
				case CSR_SCOUNTEREN:
				case CSR_SSCRATCH:
				case CSR_SEPC:
				case CSR_SCAUSE:
				case CSR_STVAL:
				case CSR_SATP:
					r->exist = riscv_supports_extension(target, hartid, 'S');
					break;
				case CSR_MEDELEG:
				case CSR_MIDELEG:
					/* "In systems with only M-mode, or with both M-mode and
					 * U-mode but without U-mode trap support, the medeleg and
					 * mideleg registers should not exist." */
					r->exist = riscv_supports_extension(target, hartid, 'S') ||
						riscv_supports_extension(target, hartid, 'N');
					break;

				case CSR_PMPCFG1:
				case CSR_PMPCFG3:
				case CSR_CYCLEH:
				case CSR_TIMEH:
				case CSR_INSTRETH:
				case CSR_HPMCOUNTER3H:
				case CSR_HPMCOUNTER4H:
				case CSR_HPMCOUNTER5H:
				case CSR_HPMCOUNTER6H:
				case CSR_HPMCOUNTER7H:
				case CSR_HPMCOUNTER8H:
				case CSR_HPMCOUNTER9H:
				case CSR_HPMCOUNTER10H:
				case CSR_HPMCOUNTER11H:
				case CSR_HPMCOUNTER12H:
				case CSR_HPMCOUNTER13H:
				case CSR_HPMCOUNTER14H:
				case CSR_HPMCOUNTER15H:
				case CSR_HPMCOUNTER16H:
				case CSR_HPMCOUNTER17H:
				case CSR_HPMCOUNTER18H:
				case CSR_HPMCOUNTER19H:
				case CSR_HPMCOUNTER20H:
				case CSR_HPMCOUNTER21H:
				case CSR_HPMCOUNTER22H:
				case CSR_HPMCOUNTER23H:
				case CSR_HPMCOUNTER24H:
				case CSR_HPMCOUNTER25H:
				case CSR_HPMCOUNTER26H:
				case CSR_HPMCOUNTER27H:
				case CSR_HPMCOUNTER28H:
				case CSR_HPMCOUNTER29H:
				case CSR_HPMCOUNTER30H:
				case CSR_HPMCOUNTER31H:
				case CSR_MCYCLEH:
				case CSR_MINSTRETH:
				case CSR_MHPMCOUNTER3H:
				case CSR_MHPMCOUNTER4H:
				case CSR_MHPMCOUNTER5H:
				case CSR_MHPMCOUNTER6H:
				case CSR_MHPMCOUNTER7H:
				case CSR_MHPMCOUNTER8H:
				case CSR_MHPMCOUNTER9H:
				case CSR_MHPMCOUNTER10H:
				case CSR_MHPMCOUNTER11H:
				case CSR_MHPMCOUNTER12H:
				case CSR_MHPMCOUNTER13H:
				case CSR_MHPMCOUNTER14H:
				case CSR_MHPMCOUNTER15H:
				case CSR_MHPMCOUNTER16H:
				case CSR_MHPMCOUNTER17H:
				case CSR_MHPMCOUNTER18H:
				case CSR_MHPMCOUNTER19H:
				case CSR_MHPMCOUNTER20H:
				case CSR_MHPMCOUNTER21H:
				case CSR_MHPMCOUNTER22H:
				case CSR_MHPMCOUNTER23H:
				case CSR_MHPMCOUNTER24H:
				case CSR_MHPMCOUNTER25H:
				case CSR_MHPMCOUNTER26H:
				case CSR_MHPMCOUNTER27H:
				case CSR_MHPMCOUNTER28H:
				case CSR_MHPMCOUNTER29H:
				case CSR_MHPMCOUNTER30H:
				case CSR_MHPMCOUNTER31H:
					r->exist = riscv_xlen(target) == 32;
					break;

				case CSR_VSTART:
				case CSR_VXSAT:
				case CSR_VXRM:
				case CSR_VL:
				case CSR_VTYPE:
				case CSR_VLENB:
					r->exist = riscv_supports_extension(target, hartid, 'V');
					break;
			}

			if (!r->exist && expose_csr) {
				for (unsigned i = 0; expose_csr[i].low <= expose_csr[i].high; i++) {
					if (csr_number >= expose_csr[i].low && csr_number <= expose_csr[i].high) {
						LOG_INFO("Exposing additional CSR %d", csr_number);
						r->exist = true;
						break;
					}
				}
			}

		} else if (number == GDB_REGNO_PRIV) {
			sprintf(reg_name, "priv");
			r->group = "general";
			r->feature = &feature_virtual;
			r->size = 8;

		} else if (number >= GDB_REGNO_V0 && number <= GDB_REGNO_V31) {
			r->caller_save = false;
			r->exist = riscv_supports_extension(target, hartid, 'V') && info->vlenb[hartid];
			r->size = info->vlenb[hartid] * 8;
			sprintf(reg_name, "v%d", number - GDB_REGNO_V0);
			r->group = "vector";
			r->feature = &feature_vector;
			r->reg_data_type = &info->type_vector;

		} else if (number >= GDB_REGNO_COUNT) {
			/* Custom registers. */
			assert(expose_custom);

			range_t *range = &expose_custom[custom_range_index];
			assert(range->low <= range->high);
			unsigned custom_number = range->low + custom_within_range;

			r->group = "custom";
			r->feature = &feature_custom;
			r->arch_info = calloc(1, sizeof(riscv_reg_info_t));
			if (!r->arch_info)
				return ERROR_FAIL;
			((riscv_reg_info_t *) r->arch_info)->target = target;
			((riscv_reg_info_t *) r->arch_info)->custom_number = custom_number;
			sprintf(reg_name, "custom%d", custom_number);

			custom_within_range++;
			if (custom_within_range > range->high - range->low) {
				custom_within_range = 0;
				custom_range_index++;
			}
		}

		if (reg_name[0])
			r->name = reg_name;
		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + target->reg_cache->num_regs *
				max_reg_name_len);
		r->value = info->reg_cache_values[number];
	}

	return ERROR_OK;
}


void riscv_add_bscan_tunneled_scan(struct target *target, struct scan_field *field,
					riscv_bscan_tunneled_scan_context_t *ctxt)
{
	jtag_add_ir_scan(target->tap, &select_user4, TAP_IDLE);

	memset(ctxt->tunneled_dr, 0, sizeof(ctxt->tunneled_dr));
	if (bscan_tunnel_type == BSCAN_TUNNEL_DATA_REGISTER) {
		ctxt->tunneled_dr[3].num_bits = 1;
		ctxt->tunneled_dr[3].out_value = bscan_one;
		ctxt->tunneled_dr[2].num_bits = 7;
		ctxt->tunneled_dr_width = field->num_bits;
		ctxt->tunneled_dr[2].out_value = &ctxt->tunneled_dr_width;
		/* for BSCAN tunnel, there is a one-TCK skew between shift in and shift out, so
		   scanning num_bits + 1, and then will right shift the input field after executing the queues */

		ctxt->tunneled_dr[1].num_bits = field->num_bits + 1;
		ctxt->tunneled_dr[1].out_value = field->out_value;
		ctxt->tunneled_dr[1].in_value = field->in_value;

		ctxt->tunneled_dr[0].num_bits = 3;
		ctxt->tunneled_dr[0].out_value = bscan_zero;
	} else {
		/* BSCAN_TUNNEL_NESTED_TAP */
		ctxt->tunneled_dr[0].num_bits = 1;
		ctxt->tunneled_dr[0].out_value = bscan_one;
		ctxt->tunneled_dr[1].num_bits = 7;
		ctxt->tunneled_dr_width = field->num_bits;
		ctxt->tunneled_dr[1].out_value = &ctxt->tunneled_dr_width;
		/* for BSCAN tunnel, there is a one-TCK skew between shift in and shift out, so
		   scanning num_bits + 1, and then will right shift the input field after executing the queues */
		ctxt->tunneled_dr[2].num_bits = field->num_bits + 1;
		ctxt->tunneled_dr[2].out_value = field->out_value;
		ctxt->tunneled_dr[2].in_value = field->in_value;
		ctxt->tunneled_dr[3].num_bits = 3;
		ctxt->tunneled_dr[3].out_value = bscan_zero;
	}
	jtag_add_dr_scan(target->tap, ARRAY_SIZE(ctxt->tunneled_dr), ctxt->tunneled_dr, TAP_IDLE);
}
