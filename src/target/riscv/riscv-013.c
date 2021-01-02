/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Support for RISC-V, debug version 0.13, which is currently (2/4/17) the
 * latest draft.
 */

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
#include "helper/list.h"
#include "riscv.h"
#include "debug_defines.h"
#include "rtos/rtos.h"
#include "program.h"
#include "asm.h"
#include "batch.h"

#define DM_DATA1 (DM_DATA0 + 1)
#define DM_PROGBUF1 (DM_PROGBUF0 + 1)

static int riscv013_on_step_or_resume(struct target *target, bool step);
static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step, bool use_hasel);
static void riscv013_clear_abstract_error(struct target *target);

/* Implementations of the functions in riscv_info_t. */
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int hid, int rid);
static int riscv013_set_register(struct target *target, int hartid, int regid, uint64_t value);
static int riscv013_select_current_hart(struct target *target);
static int riscv013_halt_prep(struct target *target);
static int riscv013_halt_go(struct target *target);
static int riscv013_resume_go(struct target *target);
static int riscv013_step_current_hart(struct target *target);
static int riscv013_on_halt(struct target *target);
static int riscv013_on_step(struct target *target);
static int riscv013_resume_prep(struct target *target);
static bool riscv013_is_halted(struct target *target);
static enum riscv_halt_reason riscv013_halt_reason(struct target *target);
static int riscv013_write_debug_buffer(struct target *target, unsigned index,
		riscv_insn_t d);
static riscv_insn_t riscv013_read_debug_buffer(struct target *target, unsigned
		index);
static int riscv013_execute_debug_buffer(struct target *target);
static void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
static void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a);
static int riscv013_dmi_write_u64_bits(struct target *target);
static void riscv013_fill_dmi_nop_u64(struct target *target, char *buf);
static int register_read(struct target *target, uint64_t *value, uint32_t number);
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number);
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value);
static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment);
static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
static int riscv013_test_sba_config_reg(struct target *target, target_addr_t legal_address,
		uint32_t num_words, target_addr_t illegal_address, bool run_sbbusyerror_test);
void write_memory_sba_simple(struct target *target, target_addr_t addr, uint32_t *write_data,
		uint32_t write_size, uint32_t sbcs);
void read_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *rd_buf, uint32_t read_size, uint32_t sbcs);
static int	riscv013_test_compliance(struct target *target);

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

#define CSR_DCSR_CAUSE_SWBP		1
#define CSR_DCSR_CAUSE_TRIGGER	2
#define CSR_DCSR_CAUSE_DEBUGINT	3
#define CSR_DCSR_CAUSE_STEP		4
#define CSR_DCSR_CAUSE_HALT		5
#define CSR_DCSR_CAUSE_GROUP	6

#define RISCV013_INFO(r) riscv013_info_t *r = get_info(target)

/*** JTAG registers. ***/

typedef enum {
	DMI_OP_NOP = 0,
	DMI_OP_READ = 1,
	DMI_OP_WRITE = 2
} dmi_op_t;
typedef enum {
	DMI_STATUS_SUCCESS = 0,
	DMI_STATUS_FAILED = 2,
	DMI_STATUS_BUSY = 3
} dmi_status_t;

typedef enum slot {
	SLOT0,
	SLOT1,
	SLOT_LAST,
} slot_t;

/*** Debug Bus registers. ***/

#define CMDERR_NONE				0
#define CMDERR_BUSY				1
#define CMDERR_NOT_SUPPORTED	2
#define CMDERR_EXCEPTION		3
#define CMDERR_HALT_RESUME		4
#define CMDERR_OTHER			7

/*** Info about the core being debugged. ***/

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

typedef enum {
	YNM_MAYBE,
	YNM_YES,
	YNM_NO
} yes_no_maybe_t;

typedef struct {
	struct list_head list;
	int abs_chain_position;

	/* The number of harts connected to this DM. */
	int hart_count;
	/* Indicates we already reset this DM, so don't need to do it again. */
	bool was_reset;
	/* Targets that are connected to this DM. */
	struct list_head target_list;
	/* The currently selected hartid on this DM. */
	int current_hartid;
	bool hasel_supported;

	/* The program buffer stores executable code. 0 is an illegal instruction,
	 * so we use 0 to mean the cached value is invalid. */
	uint32_t progbuf_cache[16];
} dm013_info_t;

typedef struct {
	struct list_head list;
	struct target *target;
} target_list_t;

typedef struct {
	/* The indexed used to address this hart in its DM. */
	unsigned index;
	/* Number of address bits in the dbus register. */
	unsigned abits;
	/* Number of abstract command data registers. */
	unsigned datacount;
	/* Number of words in the Program Buffer. */
	unsigned progbufsize;

	/* We cache the read-only bits of sbcs here. */
	uint32_t sbcs;

	yes_no_maybe_t progbuf_writable;
	/* We only need the address so that we know the alignment of the buffer. */
	riscv_addr_t progbuf_address;

	/* Number of run-test/idle cycles the target requests we do after each dbus
	 * access. */
	unsigned int dtmcs_idle;

	/* This value is incremented every time a dbus access comes back as "busy".
	 * It's used to determine how many run-test/idle cycles to feed the target
	 * in between accesses. */
	unsigned int dmi_busy_delay;

	/* Number of run-test/idle cycles to add between consecutive bus master
	 * reads/writes respectively. */
	unsigned int bus_master_write_delay, bus_master_read_delay;

	/* This value is increased every time we tried to execute two commands
	 * consecutively, and the second one failed because the previous hadn't
	 * completed yet.  It's used to add extra run-test/idle cycles after
	 * starting a command, so we don't have to waste time checking for busy to
	 * go low. */
	unsigned int ac_busy_delay;

	bool abstract_read_csr_supported;
	bool abstract_write_csr_supported;
	bool abstract_read_fpr_supported;
	bool abstract_write_fpr_supported;

	/* When a function returns some error due to a failure indicated by the
	 * target in cmderr, the caller can look here to see what that error was.
	 * (Compare with errno.) */
	uint8_t cmderr;

	/* Some fields from hartinfo. */
	uint8_t datasize;
	uint8_t dataaccess;
	int16_t dataaddr;

	/* The width of the hartsel field. */
	unsigned hartsellen;

	/* DM that provides access to this target. */
	dm013_info_t *dm;
} riscv013_info_t;

LIST_HEAD(dm_list);

static riscv013_info_t *get_info(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	return (riscv013_info_t *) info->version_specific;
}

/**
 * Return the DM structure for this target. If there isn't one, find it in the
 * global list of DMs. If it's not in there, then create one and initialize it
 * to 0.
 */
dm013_info_t *get_dm(struct target *target)
{
	RISCV013_INFO(info);
	if (info->dm)
		return info->dm;

	int abs_chain_position = target->tap->abs_chain_position;

	dm013_info_t *entry;
	dm013_info_t *dm = NULL;
	list_for_each_entry(entry, &dm_list, list) {
		if (entry->abs_chain_position == abs_chain_position) {
			dm = entry;
			break;
		}
	}

	if (!dm) {
		LOG_DEBUG("[%d] Allocating new DM", target->coreid);
		dm = calloc(1, sizeof(dm013_info_t));
		if (!dm)
			return NULL;
		dm->abs_chain_position = abs_chain_position;
		dm->current_hartid = -1;
		dm->hart_count = -1;
		INIT_LIST_HEAD(&dm->target_list);
		list_add(&dm->list, &dm_list);
	}

	info->dm = dm;
	target_list_t *target_entry;
	list_for_each_entry(target_entry, &dm->target_list, list) {
		if (target_entry->target == target)
			return dm;
	}
	target_entry = calloc(1, sizeof(*target_entry));
	if (!target_entry) {
		info->dm = NULL;
		return NULL;
	}
	target_entry->target = target;
	list_add(&target_entry->list, &dm->target_list);

	return dm;
}

static uint32_t set_hartsel(uint32_t initial, uint32_t index)
{
	initial &= ~DM_DMCONTROL_HARTSELLO;
	initial &= ~DM_DMCONTROL_HARTSELHI;

	uint32_t index_lo = index & ((1 << DM_DMCONTROL_HARTSELLO_LENGTH) - 1);
	initial |= index_lo << DM_DMCONTROL_HARTSELLO_OFFSET;
	uint32_t index_hi = index >> DM_DMCONTROL_HARTSELLO_LENGTH;
	assert(index_hi < 1 << DM_DMCONTROL_HARTSELHI_LENGTH);
	initial |= index_hi << DM_DMCONTROL_HARTSELHI_OFFSET;

	return initial;
}

static void decode_dmi(char *text, unsigned address, unsigned data)
{
	static const struct {
		unsigned address;
		uint64_t mask;
		const char *name;
	} description[] = {
		{ DM_DMCONTROL, DM_DMCONTROL_HALTREQ, "haltreq" },
		{ DM_DMCONTROL, DM_DMCONTROL_RESUMEREQ, "resumereq" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTRESET, "hartreset" },
		{ DM_DMCONTROL, DM_DMCONTROL_HASEL, "hasel" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTSELHI, "hartselhi" },
		{ DM_DMCONTROL, DM_DMCONTROL_HARTSELLO, "hartsello" },
		{ DM_DMCONTROL, DM_DMCONTROL_NDMRESET, "ndmreset" },
		{ DM_DMCONTROL, DM_DMCONTROL_DMACTIVE, "dmactive" },
		{ DM_DMCONTROL, DM_DMCONTROL_ACKHAVERESET, "ackhavereset" },

		{ DM_DMSTATUS, DM_DMSTATUS_IMPEBREAK, "impebreak" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLHAVERESET, "allhavereset" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYHAVERESET, "anyhavereset" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLRESUMEACK, "allresumeack" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYRESUMEACK, "anyresumeack" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLNONEXISTENT, "allnonexistent" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYNONEXISTENT, "anynonexistent" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLUNAVAIL, "allunavail" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYUNAVAIL, "anyunavail" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLRUNNING, "allrunning" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYRUNNING, "anyrunning" },
		{ DM_DMSTATUS, DM_DMSTATUS_ALLHALTED, "allhalted" },
		{ DM_DMSTATUS, DM_DMSTATUS_ANYHALTED, "anyhalted" },
		{ DM_DMSTATUS, DM_DMSTATUS_AUTHENTICATED, "authenticated" },
		{ DM_DMSTATUS, DM_DMSTATUS_AUTHBUSY, "authbusy" },
		{ DM_DMSTATUS, DM_DMSTATUS_HASRESETHALTREQ, "hasresethaltreq" },
		{ DM_DMSTATUS, DM_DMSTATUS_CONFSTRPTRVALID, "confstrptrvalid" },
		{ DM_DMSTATUS, DM_DMSTATUS_VERSION, "version" },

		{ DM_ABSTRACTCS, DM_ABSTRACTCS_PROGBUFSIZE, "progbufsize" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_BUSY, "busy" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR, "cmderr" },
		{ DM_ABSTRACTCS, DM_ABSTRACTCS_DATACOUNT, "datacount" },

		{ DM_COMMAND, DM_COMMAND_CMDTYPE, "cmdtype" },

		{ DM_SBCS, DM_SBCS_SBVERSION, "sbversion" },
		{ DM_SBCS, DM_SBCS_SBBUSYERROR, "sbbusyerror" },
		{ DM_SBCS, DM_SBCS_SBBUSY, "sbbusy" },
		{ DM_SBCS, DM_SBCS_SBREADONADDR, "sbreadonaddr" },
		{ DM_SBCS, DM_SBCS_SBACCESS, "sbaccess" },
		{ DM_SBCS, DM_SBCS_SBAUTOINCREMENT, "sbautoincrement" },
		{ DM_SBCS, DM_SBCS_SBREADONDATA, "sbreadondata" },
		{ DM_SBCS, DM_SBCS_SBERROR, "sberror" },
		{ DM_SBCS, DM_SBCS_SBASIZE, "sbasize" },
		{ DM_SBCS, DM_SBCS_SBACCESS128, "sbaccess128" },
		{ DM_SBCS, DM_SBCS_SBACCESS64, "sbaccess64" },
		{ DM_SBCS, DM_SBCS_SBACCESS32, "sbaccess32" },
		{ DM_SBCS, DM_SBCS_SBACCESS16, "sbaccess16" },
		{ DM_SBCS, DM_SBCS_SBACCESS8, "sbaccess8" },
	};

	text[0] = 0;
	for (unsigned i = 0; i < DIM(description); i++) {
		if (description[i].address == address) {
			uint64_t mask = description[i].mask;
			unsigned value = get_field(data, mask);
			if (value) {
				if (i > 0)
					*(text++) = ' ';
				if (mask & (mask >> 1)) {
					/* If the field is more than 1 bit wide. */
					sprintf(text, "%s=%d", description[i].name, value);
				} else {
					strcpy(text, description[i].name);
				}
				text += strlen(text);
			}
		}
	}
}

static void dump_field(int idle, const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned int in_op = get_field(in, DTM_DMI_OP);
	unsigned int in_data = get_field(in, DTM_DMI_DATA);
	unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;

	log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, "scan",
			"%db %s %08x @%02x -> %s %08x @%02x; %di",
			field->num_bits, op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address, idle);

	char out_text[500];
	char in_text[500];
	decode_dmi(out_text, out_address, out_data);
	decode_dmi(in_text, in_address, in_data);
	if (in_text[0] || out_text[0]) {
		log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, "scan", "%s -> %s",
				out_text, in_text);
	}
}

/*** Utility functions. ***/

static void select_dmi(struct target *target)
{
	if (bscan_tunnel_ir_width != 0) {
		select_dmi_via_bscan(target);
		return;
	}
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
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

	/* Always return to dmi. */
	select_dmi(target);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCS: 0x%x -> 0x%x", out, in);

	return in;
}

static void increase_dmi_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->dmi_busy_delay += info->dmi_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcs_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcs_idle, info->dmi_busy_delay,
			info->ac_busy_delay);

	dtmcontrol_scan(target, DTM_DTMCS_DMIRESET);
}

/**
 * exec: If this is set, assume the scan results in an execution, so more
 * run-test/idle cycles may be required.
 */
static dmi_status_t dmi_scan(struct target *target, uint32_t *address_in,
		uint32_t *data_in, dmi_op_t op, uint32_t address_out, uint32_t data_out,
		bool exec)
{
	riscv013_info_t *info = get_info(target);
	RISCV_INFO(r);
	unsigned num_bits = info->abits + DTM_DMI_OP_LENGTH + DTM_DMI_DATA_LENGTH;
	size_t num_bytes = (num_bits + 7) / 8;
	uint8_t in[num_bytes];
	uint8_t out[num_bytes];
	struct scan_field field = {
		.num_bits = num_bits,
		.out_value = out,
		.in_value = in
	};
	riscv_bscan_tunneled_scan_context_t bscan_ctxt;

	if (r->reset_delays_wait >= 0) {
		r->reset_delays_wait--;
		if (r->reset_delays_wait < 0) {
			info->dmi_busy_delay = 0;
			info->ac_busy_delay = 0;
		}
	}

	memset(in, 0, num_bytes);
	memset(out, 0, num_bytes);

	assert(info->abits != 0);

	buf_set_u32(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	buf_set_u32(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	buf_set_u32(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);

	/* I wanted to place this code in a different function, but the way JTAG command
	   queueing works in the jtag handling functions, the scan fields either have to be
	   heap allocated, global/static, or else they need to stay on the stack until
	   the jtag_execute_queue() call.  Heap or static fields in this case doesn't seem
	   the best fit.  Declaring stack based field values in a subsidiary function call wouldn't
	   work. */
	if (bscan_tunnel_ir_width != 0) {
		riscv_add_bscan_tunneled_scan(target, &field, &bscan_ctxt);
	} else {
		/* Assume dbus is already selected. */
		jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	}

	int idle_count = info->dmi_busy_delay;
	if (exec)
		idle_count += info->ac_busy_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		if (data_in)
			*data_in = ~0;
		return DMI_STATUS_FAILED;
	}

	if (bscan_tunnel_ir_width != 0) {
		/* need to right-shift "in" by one bit, because of clock skew between BSCAN TAP and DM TAP */
		buffer_shr(in, num_bytes, 1);
	}

	if (data_in)
		*data_in = buf_get_u32(in, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);

	if (address_in)
		*address_in = buf_get_u32(in, DTM_DMI_ADDRESS_OFFSET, info->abits);
	dump_field(idle_count, &field);
	return buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

/**
 * @param target
 * @param data_in  The data we received from the target.
 * @param dmi_busy_encountered
 *                 If non-NULL, will be updated to reflect whether DMI busy was
 *                 encountered while executing this operation or not.
 * @param dmi_op   The operation to perform (read/write/nop).
 * @param address  The address argument to that operation.
 * @param data_out The data to send to the target.
 * @param timeout_sec
 * @param exec     When true, this scan will execute something, so extra RTI
 *                 cycles may be added.
 * @param ensure_success
 *                 Scan a nop after the requested operation, ensuring the
 *                 DMI operation succeeded.
 */
static int dmi_op_timeout(struct target *target, uint32_t *data_in,
		bool *dmi_busy_encountered, int dmi_op, uint32_t address,
		uint32_t data_out, int timeout_sec, bool exec, bool ensure_success)
{
	select_dmi(target);

	dmi_status_t status;
	uint32_t address_in;

	if (dmi_busy_encountered)
		*dmi_busy_encountered = false;

	const char *op_name;
	switch (dmi_op) {
		case DMI_OP_NOP:
			op_name = "nop";
			break;
		case DMI_OP_READ:
			op_name = "read";
			break;
		case DMI_OP_WRITE:
			op_name = "write";
			break;
		default:
			LOG_ERROR("Invalid DMI operation: %d", dmi_op);
			return ERROR_FAIL;
	}

	time_t start = time(NULL);
	/* This first loop performs the request.  Note that if for some reason this
	 * stays busy, it is actually due to the previous access. */
	while (1) {
		status = dmi_scan(target, NULL, NULL, dmi_op, address, data_out,
				exec);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
			if (dmi_busy_encountered)
				*dmi_busy_encountered = true;
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed %s at 0x%x, status=%d", op_name, address, status);
			return ERROR_FAIL;
		}
		if (time(NULL) - start > timeout_sec)
			return ERROR_TIMEOUT_REACHED;
	}

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("Failed %s at 0x%x; status=%d", op_name, address, status);
		return ERROR_FAIL;
	}

	if (ensure_success) {
		/* This second loop ensures the request succeeded, and gets back data.
		 * Note that NOP can result in a 'busy' result as well, but that would be
		 * noticed on the next DMI access we do. */
		while (1) {
			status = dmi_scan(target, &address_in, data_in, DMI_OP_NOP, address, 0,
					false);
			if (status == DMI_STATUS_BUSY) {
				increase_dmi_busy_delay(target);
				if (dmi_busy_encountered)
					*dmi_busy_encountered = true;
			} else if (status == DMI_STATUS_SUCCESS) {
				break;
			} else {
				if (data_in) {
					LOG_ERROR("Failed %s (NOP) at 0x%x; value=0x%x, status=%d",
							op_name, address, *data_in, status);
				} else {
					LOG_ERROR("Failed %s (NOP) at 0x%x; status=%d", op_name, address,
							status);
				}
				return ERROR_FAIL;
			}
			if (time(NULL) - start > timeout_sec)
				return ERROR_TIMEOUT_REACHED;
		}
	}

	return ERROR_OK;
}

static int dmi_op(struct target *target, uint32_t *data_in,
		bool *dmi_busy_encountered, int dmi_op, uint32_t address,
		uint32_t data_out, bool exec, bool ensure_success)
{
	int result = dmi_op_timeout(target, data_in, dmi_busy_encountered, dmi_op,
			address, data_out, riscv_command_timeout_sec, exec, ensure_success);
	if (result == ERROR_TIMEOUT_REACHED) {
		LOG_ERROR("DMI operation didn't complete in %d seconds. The target is "
				"either really slow or broken. You could increase the "
				"timeout with riscv set_command_timeout_sec.",
				riscv_command_timeout_sec);
		return ERROR_FAIL;
	}
	return result;
}

static int dmi_read(struct target *target, uint32_t *value, uint32_t address)
{
	return dmi_op(target, value, NULL, DMI_OP_READ, address, 0, false, true);
}

static int dmi_read_exec(struct target *target, uint32_t *value, uint32_t address)
{
	return dmi_op(target, value, NULL, DMI_OP_READ, address, 0, true, true);
}

static int dmi_write(struct target *target, uint32_t address, uint32_t value)
{
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, address, value, false, true);
}

static int dmi_write_exec(struct target *target, uint32_t address,
		uint32_t value, bool ensure_success)
{
	return dmi_op(target, NULL, NULL, DMI_OP_WRITE, address, value, true, ensure_success);
}

int dmstatus_read_timeout(struct target *target, uint32_t *dmstatus,
		bool authenticated, unsigned timeout_sec)
{
	int result = dmi_op_timeout(target, dmstatus, NULL, DMI_OP_READ,
			DM_DMSTATUS, 0, timeout_sec, false, true);
	if (result != ERROR_OK)
		return result;
	int dmstatus_version = get_field(*dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2 (0.13) and 3 (0.14), not "
				"%d (dmstatus=0x%x). This error might be caused by a JTAG "
				"signal issue. Try reducing the JTAG clock speed.",
				get_field(*dmstatus, DM_DMSTATUS_VERSION), *dmstatus);
	} else if (authenticated && !get_field(*dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", *dmstatus);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int dmstatus_read(struct target *target, uint32_t *dmstatus,
		bool authenticated)
{
	return dmstatus_read_timeout(target, dmstatus, authenticated,
			riscv_command_timeout_sec);
}

static void increase_ac_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->ac_busy_delay += info->ac_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcs_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcs_idle, info->dmi_busy_delay,
			info->ac_busy_delay);
}

uint32_t abstract_register_size(unsigned width)
{
	switch (width) {
		case 32:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 2);
		case 64:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 3);
		case 128:
			return set_field(0, AC_ACCESS_REGISTER_AARSIZE, 4);
		default:
			LOG_ERROR("Unsupported register width: %d", width);
			return 0;
	}
}

static int wait_for_idle(struct target *target, uint32_t *abstractcs)
{
	RISCV013_INFO(info);
	time_t start = time(NULL);
	while (1) {
		if (dmi_read(target, abstractcs, DM_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;

		if (get_field(*abstractcs, DM_ABSTRACTCS_BUSY) == 0)
			return ERROR_OK;

		if (time(NULL) - start > riscv_command_timeout_sec) {
			info->cmderr = get_field(*abstractcs, DM_ABSTRACTCS_CMDERR);
			if (info->cmderr != CMDERR_NONE) {
				const char *errors[8] = {
					"none",
					"busy",
					"not supported",
					"exception",
					"halt/resume",
					"reserved",
					"reserved",
					"other" };

				LOG_ERROR("Abstract command ended in error '%s' (abstractcs=0x%x)",
						errors[info->cmderr], *abstractcs);
			}

			LOG_ERROR("Timed out after %ds waiting for busy to go low (abstractcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					*abstractcs);
			return ERROR_FAIL;
		}
	}
}

static int execute_abstract_command(struct target *target, uint32_t command)
{
	RISCV013_INFO(info);
	if (debug_level >= LOG_LVL_DEBUG) {
		switch (get_field(command, DM_COMMAND_CMDTYPE)) {
			case 0:
				LOG_DEBUG("command=0x%x; access register, size=%d, postexec=%d, "
						"transfer=%d, write=%d, regno=0x%x",
						command,
						8 << get_field(command, AC_ACCESS_REGISTER_AARSIZE),
						get_field(command, AC_ACCESS_REGISTER_POSTEXEC),
						get_field(command, AC_ACCESS_REGISTER_TRANSFER),
						get_field(command, AC_ACCESS_REGISTER_WRITE),
						get_field(command, AC_ACCESS_REGISTER_REGNO));
				break;
			default:
				LOG_DEBUG("command=0x%x", command);
				break;
		}
	}

	if (dmi_write_exec(target, DM_COMMAND, command, false) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t abstractcs = 0;
	int result = wait_for_idle(target, &abstractcs);

	info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);
	if (info->cmderr != 0 || result != ERROR_OK) {
		LOG_DEBUG("command 0x%x failed; abstractcs=0x%x", command, abstractcs);
		/* Clear the error. */
		dmi_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static riscv_reg_t read_abstract_arg(struct target *target, unsigned index,
		unsigned size_bits)
{
	riscv_reg_t value = 0;
	uint32_t v;
	unsigned offset = index * size_bits / 32;
	switch (size_bits) {
		default:
			LOG_ERROR("Unsupported size: %d bits", size_bits);
			return ~0;
		case 64:
			dmi_read(target, &v, DM_DATA0 + offset + 1);
			value |= ((uint64_t) v) << 32;
			/* falls through */
		case 32:
			dmi_read(target, &v, DM_DATA0 + offset);
			value |= v;
	}
	return value;
}

static int write_abstract_arg(struct target *target, unsigned index,
		riscv_reg_t value, unsigned size_bits)
{
	unsigned offset = index * size_bits / 32;
	switch (size_bits) {
		default:
			LOG_ERROR("Unsupported size: %d bits", size_bits);
			return ERROR_FAIL;
		case 64:
			dmi_write(target, DM_DATA0 + offset + 1, value >> 32);
			/* falls through */
		case 32:
			dmi_write(target, DM_DATA0 + offset, value);
	}
	return ERROR_OK;
}

/**
 * @par size in bits
 */
static uint32_t access_register_command(struct target *target, uint32_t number,
		unsigned size, uint32_t flags)
{
	uint32_t command = set_field(0, DM_COMMAND_CMDTYPE, 0);
	switch (size) {
		case 32:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 2);
			break;
		case 64:
			command = set_field(command, AC_ACCESS_REGISTER_AARSIZE, 3);
			break;
		default:
			LOG_ERROR("%d-bit register %s not supported.", size,
					gdb_regno_name(number));
			assert(0);
	}

	if (number <= GDB_REGNO_XPR31) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0x1000 + number - GDB_REGNO_ZERO);
	} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0x1020 + number - GDB_REGNO_FPR0);
	} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				number - GDB_REGNO_CSR0);
	} else if (number >= GDB_REGNO_COUNT) {
		/* Custom register. */
		assert(target->reg_cache->reg_list[number].arch_info);
		riscv_reg_info_t *reg_info = target->reg_cache->reg_list[number].arch_info;
		assert(reg_info);
		command = set_field(command, AC_ACCESS_REGISTER_REGNO,
				0xc000 + reg_info->custom_number);
	} else {
		assert(0);
	}

	command |= flags;

	return command;
}

static int register_read_abstract(struct target *target, uint64_t *value,
		uint32_t number, unsigned size)
{
	RISCV013_INFO(info);

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_read_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_read_csr_supported)
		return ERROR_FAIL;
	/* The spec doesn't define abstract register numbers for vector registers. */
	if (number >= GDB_REGNO_V0 && number <= GDB_REGNO_V31)
		return ERROR_FAIL;

	uint32_t command = access_register_command(target, number, size,
			AC_ACCESS_REGISTER_TRANSFER);

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		if (info->cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_read_fpr_supported = false;
				LOG_INFO("Disabling abstract command reads from FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_read_csr_supported = false;
				LOG_INFO("Disabling abstract command reads from CSRs.");
			}
		}
		return result;
	}

	if (value)
		*value = read_abstract_arg(target, 0, size);

	return ERROR_OK;
}

static int register_write_abstract(struct target *target, uint32_t number,
		uint64_t value, unsigned size)
{
	RISCV013_INFO(info);

	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			!info->abstract_write_fpr_supported)
		return ERROR_FAIL;
	if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095 &&
			!info->abstract_write_csr_supported)
		return ERROR_FAIL;

	uint32_t command = access_register_command(target, number, size,
			AC_ACCESS_REGISTER_TRANSFER |
			AC_ACCESS_REGISTER_WRITE);

	if (write_abstract_arg(target, 0, value, size) != ERROR_OK)
		return ERROR_FAIL;

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		if (info->cmderr == CMDERR_NOT_SUPPORTED) {
			if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
				info->abstract_write_fpr_supported = false;
				LOG_INFO("Disabling abstract command writes to FPRs.");
			} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
				info->abstract_write_csr_supported = false;
				LOG_INFO("Disabling abstract command writes to CSRs.");
			}
		}
		return result;
	}

	return ERROR_OK;
}

/*
 * Sets the AAMSIZE field of a memory access abstract command based on
 * the width (bits).
 */
static uint32_t abstract_memory_size(unsigned width)
{
	switch (width) {
		case 8:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 0);
		case 16:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 1);
		case 32:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 2);
		case 64:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 3);
		case 128:
			return set_field(0, AC_ACCESS_MEMORY_AAMSIZE, 4);
		default:
			LOG_ERROR("Unsupported memory width: %d", width);
			return 0;
	}
}

/*
 * Creates a memory access abstract command.
 */
static uint32_t access_memory_command(struct target *target, bool virtual,
		unsigned width, bool postincrement, bool write)
{
	uint32_t command = set_field(0, AC_ACCESS_MEMORY_CMDTYPE, 2);
	command = set_field(command, AC_ACCESS_MEMORY_AAMVIRTUAL, virtual);
	command |= abstract_memory_size(width);
	command = set_field(command, AC_ACCESS_MEMORY_AAMPOSTINCREMENT,
						postincrement);
	command = set_field(command, AC_ACCESS_MEMORY_WRITE, write);

	return command;
}

static int examine_progbuf(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	if (info->progbuf_writable != YNM_MAYBE)
		return ERROR_OK;

	/* Figure out if progbuf is writable. */

	if (info->progbufsize < 1) {
		info->progbuf_writable = YNM_NO;
		LOG_INFO("No program buffer present.");
		return ERROR_OK;
	}

	uint64_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, auipc(S0));
	if (riscv_program_exec(&program, target) != ERROR_OK)
		return ERROR_FAIL;

	if (register_read_direct(target, &info->progbuf_address, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	riscv_program_init(&program, target);
	riscv_program_insert(&program, sw(S0, S0, 0));
	int result = riscv_program_exec(&program, target);

	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	if (result != ERROR_OK) {
		/* This program might have failed if the program buffer is not
		 * writable. */
		info->progbuf_writable = YNM_NO;
		return ERROR_OK;
	}

	uint32_t written;
	if (dmi_read(target, &written, DM_PROGBUF0) != ERROR_OK)
		return ERROR_FAIL;
	if (written == (uint32_t) info->progbuf_address) {
		LOG_INFO("progbuf is writable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_YES;

	} else {
		LOG_INFO("progbuf is not writeable at 0x%" PRIx64,
				info->progbuf_address);
		info->progbuf_writable = YNM_NO;
	}

	return ERROR_OK;
}

static int is_fpu_reg(uint32_t gdb_regno)
{
	return (gdb_regno >= GDB_REGNO_FPR0 && gdb_regno <= GDB_REGNO_FPR31) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FFLAGS) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FRM) ||
		(gdb_regno == GDB_REGNO_CSR0 + CSR_FCSR);
}

static int is_vector_reg(uint32_t gdb_regno)
{
	return (gdb_regno >= GDB_REGNO_V0 && gdb_regno <= GDB_REGNO_V31) ||
		gdb_regno == GDB_REGNO_VSTART ||
		gdb_regno == GDB_REGNO_VXSAT ||
		gdb_regno == GDB_REGNO_VXRM ||
		gdb_regno == GDB_REGNO_VL ||
		gdb_regno == GDB_REGNO_VTYPE ||
		gdb_regno == GDB_REGNO_VLENB;
}

static int prep_for_register_access(struct target *target, uint64_t *mstatus,
		int regno)
{
	if (is_fpu_reg(regno) || is_vector_reg(regno)) {
		if (register_read(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if (is_fpu_reg(regno) && (*mstatus & MSTATUS_FS) == 0) {
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
						set_field(*mstatus, MSTATUS_FS, 1)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (is_vector_reg(regno) && (*mstatus & MSTATUS_VS) == 0) {
			if (register_write_direct(target, GDB_REGNO_MSTATUS,
						set_field(*mstatus, MSTATUS_VS, 1)) != ERROR_OK)
				return ERROR_FAIL;
		}
	} else {
		*mstatus = 0;
	}
	return ERROR_OK;
}

static int cleanup_after_register_access(struct target *target,
		uint64_t mstatus, int regno)
{
	if ((is_fpu_reg(regno) && (mstatus & MSTATUS_FS) == 0) ||
			(is_vector_reg(regno) && (mstatus & MSTATUS_VS) == 0))
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus) != ERROR_OK)
			return ERROR_FAIL;
	return ERROR_OK;
}

typedef enum {
	SPACE_DM_DATA,
	SPACE_DMI_PROGBUF,
	SPACE_DMI_RAM
} memory_space_t;

typedef struct {
	/* How can the debugger access this memory? */
	memory_space_t memory_space;
	/* Memory address to access the scratch memory from the hart. */
	riscv_addr_t hart_address;
	/* Memory address to access the scratch memory from the debugger. */
	riscv_addr_t debug_address;
	struct working_area *area;
} scratch_mem_t;

/**
 * Find some scratch memory to be used with the given program.
 */
static int scratch_reserve(struct target *target,
		scratch_mem_t *scratch,
		struct riscv_program *program,
		unsigned size_bytes)
{
	riscv_addr_t alignment = 1;
	while (alignment < size_bytes)
		alignment *= 2;

	scratch->area = NULL;

	riscv013_info_t *info = get_info(target);

	/* Option 1: See if data# registers can be used as the scratch memory */
	if (info->dataaccess == 1) {
		/* Sign extend dataaddr. */
		scratch->hart_address = info->dataaddr;
		if (info->dataaddr & (1<<11))
			scratch->hart_address |= 0xfffffffffffff000ULL;
		/* Align. */
		scratch->hart_address = (scratch->hart_address + alignment - 1) & ~(alignment - 1);

		if ((size_bytes + scratch->hart_address - info->dataaddr + 3) / 4 >=
				info->datasize) {
			scratch->memory_space = SPACE_DM_DATA;
			scratch->debug_address = (scratch->hart_address - info->dataaddr) / 4;
			return ERROR_OK;
		}
	}

	/* Option 2: See if progbuf can be used as the scratch memory */
	if (examine_progbuf(target) != ERROR_OK)
		return ERROR_FAIL;

	/* Allow for ebreak at the end of the program. */
	unsigned program_size = (program->instruction_count + 1) * 4;
	scratch->hart_address = (info->progbuf_address + program_size + alignment - 1) &
		~(alignment - 1);
	if ((info->progbuf_writable == YNM_YES) &&
			((size_bytes + scratch->hart_address - info->progbuf_address + 3) / 4 >=
			info->progbufsize)) {
		scratch->memory_space = SPACE_DMI_PROGBUF;
		scratch->debug_address = (scratch->hart_address - info->progbuf_address) / 4;
		return ERROR_OK;
	}

	/* Option 3: User-configured memory area as scratch RAM */
	if (target_alloc_working_area(target, size_bytes + alignment - 1,
				&scratch->area) == ERROR_OK) {
		scratch->hart_address = (scratch->area->address + alignment - 1) &
			~(alignment - 1);
		scratch->memory_space = SPACE_DMI_RAM;
		scratch->debug_address = scratch->hart_address;
		return ERROR_OK;
	}

	LOG_ERROR("Couldn't find %d bytes of scratch RAM to use. Please configure "
			"a work area with 'configure -work-area-phys'.", size_bytes);
	return ERROR_FAIL;
}

static int scratch_release(struct target *target,
		scratch_mem_t *scratch)
{
	if (scratch->area)
		return target_free_working_area(target, scratch->area);

	return ERROR_OK;
}

static int scratch_read64(struct target *target, scratch_mem_t *scratch,
		uint64_t *value)
{
	uint32_t v;
	switch (scratch->memory_space) {
		case SPACE_DM_DATA:
			if (dmi_read(target, &v, DM_DATA0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DM_DATA1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_PROGBUF:
			if (dmi_read(target, &v, DM_PROGBUF0 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value = v;
			if (dmi_read(target, &v, DM_PROGBUF1 + scratch->debug_address) != ERROR_OK)
				return ERROR_FAIL;
			*value |= ((uint64_t) v) << 32;
			break;
		case SPACE_DMI_RAM:
			{
				uint8_t buffer[8] = {0};
				if (read_memory(target, scratch->debug_address, 4, 2, buffer, 4) != ERROR_OK)
					return ERROR_FAIL;
				*value = buffer[0] |
					(((uint64_t) buffer[1]) << 8) |
					(((uint64_t) buffer[2]) << 16) |
					(((uint64_t) buffer[3]) << 24) |
					(((uint64_t) buffer[4]) << 32) |
					(((uint64_t) buffer[5]) << 40) |
					(((uint64_t) buffer[6]) << 48) |
					(((uint64_t) buffer[7]) << 56);
			}
			break;
	}
	return ERROR_OK;
}

static int scratch_write64(struct target *target, scratch_mem_t *scratch,
		uint64_t value)
{
	switch (scratch->memory_space) {
		case SPACE_DM_DATA:
			dmi_write(target, DM_DATA0 + scratch->debug_address, value);
			dmi_write(target, DM_DATA1 + scratch->debug_address, value >> 32);
			break;
		case SPACE_DMI_PROGBUF:
			dmi_write(target, DM_PROGBUF0 + scratch->debug_address, value);
			dmi_write(target, DM_PROGBUF1 + scratch->debug_address, value >> 32);
			break;
		case SPACE_DMI_RAM:
			{
				uint8_t buffer[8] = {
					value,
					value >> 8,
					value >> 16,
					value >> 24,
					value >> 32,
					value >> 40,
					value >> 48,
					value >> 56
				};
				if (write_memory(target, scratch->debug_address, 4, 2, buffer) != ERROR_OK)
					return ERROR_FAIL;
			}
			break;
	}
	return ERROR_OK;
}

/** Return register size in bits. */
static unsigned register_size(struct target *target, unsigned number)
{
	/* If reg_cache hasn't been initialized yet, make a guess. We need this for
	 * when this function is called during examine(). */
	if (target->reg_cache)
		return target->reg_cache->reg_list[number].size;
	else
		return riscv_xlen(target);
}

static bool has_sufficient_progbuf(struct target *target, unsigned size)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);

	return info->progbufsize + r->impebreak >= size;
}

/**
 * Immediately write the new value to the requested register. This mechanism
 * bypasses any caches.
 */
static int register_write_direct(struct target *target, unsigned number,
		uint64_t value)
{
	LOG_DEBUG("{%d} %s <- 0x%" PRIx64, riscv_current_hartid(target),
			gdb_regno_name(number), value);

	int result = register_write_abstract(target, number, value,
			register_size(target, number));
	if (result == ERROR_OK || !has_sufficient_progbuf(target, 2) ||
			!riscv_is_halted(target))
		return result;

	struct riscv_program program;
	riscv_program_init(&program, target);

	uint64_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	scratch_mem_t scratch;
	bool use_scratch = false;
	if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31 &&
			riscv_supports_extension(target, riscv_current_hartid(target), 'D') &&
			riscv_xlen(target) < 64) {
		/* There are no instructions to move all the bits from a register, so
		 * we need to use some scratch RAM. */
		use_scratch = true;
		riscv_program_insert(&program, fld(number - GDB_REGNO_FPR0, S0, 0));

		if (scratch_reserve(target, &scratch, &program, 8) != ERROR_OK)
			return ERROR_FAIL;

		if (register_write_direct(target, GDB_REGNO_S0, scratch.hart_address)
				!= ERROR_OK) {
			scratch_release(target, &scratch);
			return ERROR_FAIL;
		}

		if (scratch_write64(target, &scratch, value) != ERROR_OK) {
			scratch_release(target, &scratch);
			return ERROR_FAIL;
		}

	} else if (number == GDB_REGNO_VTYPE) {
		riscv_program_insert(&program, csrr(S0, CSR_VL));
		riscv_program_insert(&program, vsetvli(ZERO, S0, value));

	} else {
		if (register_write_direct(target, GDB_REGNO_S0, value) != ERROR_OK)
			return ERROR_FAIL;

		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D'))
				riscv_program_insert(&program, fmv_d_x(number - GDB_REGNO_FPR0, S0));
			else
				riscv_program_insert(&program, fmv_w_x(number - GDB_REGNO_FPR0, S0));
		} else if (number == GDB_REGNO_VL) {
			/* "The XLEN-bit-wide read-only vl CSR can only be updated by the
			 * vsetvli and vsetvl instructions, and the fault-only-rst vector
			 * load instruction variants." */
			riscv_reg_t vtype;
			if (register_read(target, &vtype, GDB_REGNO_VTYPE) != ERROR_OK)
				return ERROR_FAIL;
			if (riscv_program_insert(&program, vsetvli(ZERO, S0, vtype)) != ERROR_OK)
				return ERROR_FAIL;
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			riscv_program_csrw(&program, S0, number);
		} else {
			LOG_ERROR("Unsupported register (enum gdb_regno)(%d)", number);
			return ERROR_FAIL;
		}
	}

	int exec_out = riscv_program_exec(&program, target);
	/* Don't message on error. Probably the register doesn't exist. */
	if (exec_out == ERROR_OK && target->reg_cache) {
		struct reg *reg = &target->reg_cache->reg_list[number];
		buf_set_u64(reg->value, 0, reg->size, value);
	}

	if (use_scratch)
		scratch_release(target, &scratch);

	if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
		return ERROR_FAIL;

	/* Restore S0. */
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	return exec_out;
}

/** Return the cached value, or read from the target if necessary. */
static int register_read(struct target *target, uint64_t *value, uint32_t number)
{
	if (number == GDB_REGNO_ZERO) {
		*value = 0;
		return ERROR_OK;
	}
	int result = register_read_direct(target, value, number);
	if (result != ERROR_OK)
		return ERROR_FAIL;
	if (target->reg_cache) {
		struct reg *reg = &target->reg_cache->reg_list[number];
		buf_set_u64(reg->value, 0, reg->size, *value);
	}
	return ERROR_OK;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
	int result = register_read_abstract(target, value, number,
			register_size(target, number));

	if (result != ERROR_OK &&
			has_sufficient_progbuf(target, 2) &&
			number > GDB_REGNO_XPR31) {
		struct riscv_program program;
		riscv_program_init(&program, target);

		scratch_mem_t scratch;
		bool use_scratch = false;

		riscv_reg_t s0;
		if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
			return ERROR_FAIL;

		/* Write program to move data into s0. */

		uint64_t mstatus;
		if (prep_for_register_access(target, &mstatus, number) != ERROR_OK)
			return ERROR_FAIL;

		if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D')
					&& riscv_xlen(target) < 64) {
				/* There are no instructions to move all the bits from a
				 * register, so we need to use some scratch RAM. */
				riscv_program_insert(&program, fsd(number - GDB_REGNO_FPR0, S0,
							0));

				if (scratch_reserve(target, &scratch, &program, 8) != ERROR_OK)
					return ERROR_FAIL;
				use_scratch = true;

				if (register_write_direct(target, GDB_REGNO_S0,
							scratch.hart_address) != ERROR_OK) {
					scratch_release(target, &scratch);
					return ERROR_FAIL;
				}
			} else if (riscv_supports_extension(target,
						riscv_current_hartid(target), 'D')) {
				riscv_program_insert(&program, fmv_x_d(S0, number - GDB_REGNO_FPR0));
			} else {
				riscv_program_insert(&program, fmv_x_w(S0, number - GDB_REGNO_FPR0));
			}
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			riscv_program_csrr(&program, S0, number);
		} else {
			LOG_ERROR("Unsupported register: %s", gdb_regno_name(number));
			return ERROR_FAIL;
		}

		/* Execute program. */
		result = riscv_program_exec(&program, target);
		/* Don't message on error. Probably the register doesn't exist. */

		if (use_scratch) {
			result = scratch_read64(target, &scratch, value);
			scratch_release(target, &scratch);
			if (result != ERROR_OK)
				return result;
		} else {
			/* Read S0 */
			if (register_read_direct(target, value, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (cleanup_after_register_access(target, mstatus, number) != ERROR_OK)
			return ERROR_FAIL;

		/* Restore S0. */
		if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (result == ERROR_OK) {
		LOG_DEBUG("{%d} %s = 0x%" PRIx64, riscv_current_hartid(target),
				gdb_regno_name(number), *value);
	}

	return result;
}

int wait_for_authbusy(struct target *target, uint32_t *dmstatus)
{
	time_t start = time(NULL);
	while (1) {
		uint32_t value;
		if (dmstatus_read(target, &value, false) != ERROR_OK)
			return ERROR_FAIL;
		if (dmstatus)
			*dmstatus = value;
		if (!get_field(value, DM_DMSTATUS_AUTHBUSY))
			break;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out after %ds waiting for authbusy to go low (dmstatus=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec,
					value);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/*** OpenOCD target functions. ***/

static void deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	free(info->version_specific);
	/* TODO: free register arch_info */
	info->version_specific = NULL;
}

static int set_haltgroup(struct target *target, bool *supported)
{
	uint32_t write = set_field(DM_DMCS2_HGWRITE, DM_DMCS2_GROUP, target->smp);
	if (dmi_write(target, DM_DMCS2, write) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t read;
	if (dmi_read(target, &read, DM_DMCS2) != ERROR_OK)
		return ERROR_FAIL;
	*supported = get_field(read, DM_DMCS2_GROUP) == (unsigned)target->smp;
	return ERROR_OK;
}

static int discover_vlenb(struct target *target, int hartid)
{
	RISCV_INFO(r);
	riscv_reg_t vlenb;

	if (register_read(target, &vlenb, GDB_REGNO_VLENB) != ERROR_OK) {
		LOG_WARNING("Couldn't read vlenb for %s; vector register access won't work.",
				target_name(target));
		r->vlenb[hartid] = 0;
		return ERROR_OK;
	}
	r->vlenb[hartid] = vlenb;

	LOG_INFO("hart %d: Vector support with vlenb=%d", hartid, r->vlenb[hartid]);

	return ERROR_OK;
}

static int examine(struct target *target)
{
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	LOG_DEBUG("  dmireset=%d", get_field(dtmcontrol, DTM_DTMCS_DMIRESET));
	LOG_DEBUG("  idle=%d", get_field(dtmcontrol, DTM_DTMCS_IDLE));
	LOG_DEBUG("  dmistat=%d", get_field(dtmcontrol, DTM_DTMCS_DMISTAT));
	LOG_DEBUG("  abits=%d", get_field(dtmcontrol, DTM_DTMCS_ABITS));
	LOG_DEBUG("  version=%d", get_field(dtmcontrol, DTM_DTMCS_VERSION));
	if (dtmcontrol == 0) {
		LOG_ERROR("dtmcontrol is 0. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	if (get_field(dtmcontrol, DTM_DTMCS_VERSION) != 1) {
		LOG_ERROR("Unsupported DTM version %d. (dtmcontrol=0x%x)",
				get_field(dtmcontrol, DTM_DTMCS_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	riscv013_info_t *info = get_info(target);
	/* TODO: This won't be true if there are multiple DMs. */
	info->index = target->coreid;
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcs_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	/* Reset the Debug Module. */
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (!dm->was_reset) {
		dmi_write(target, DM_DMCONTROL, 0);
		dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_DMACTIVE);
		dm->was_reset = true;
	}

	dmi_write(target, DM_DMCONTROL, DM_DMCONTROL_HARTSELLO |
			DM_DMCONTROL_HARTSELHI | DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_HASEL);
	uint32_t dmcontrol;
	if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(dmcontrol, DM_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("Debug Module did not become active. dmcontrol=0x%x",
				dmcontrol);
		return ERROR_FAIL;
	}

	dm->hasel_supported = get_field(dmcontrol, DM_DMCONTROL_HASEL);

	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, false) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("dmstatus:  0x%08x", dmstatus);
	int dmstatus_version = get_field(dmstatus, DM_DMSTATUS_VERSION);
	if (dmstatus_version != 2 && dmstatus_version != 3) {
		/* Error was already printed out in dmstatus_read(). */
		return ERROR_FAIL;
	}

	uint32_t hartsel =
		(get_field(dmcontrol, DM_DMCONTROL_HARTSELHI) <<
		 DM_DMCONTROL_HARTSELLO_LENGTH) |
		get_field(dmcontrol, DM_DMCONTROL_HARTSELLO);
	info->hartsellen = 0;
	while (hartsel & 1) {
		info->hartsellen++;
		hartsel >>= 1;
	}
	LOG_DEBUG("hartsellen=%d", info->hartsellen);

	uint32_t hartinfo;
	if (dmi_read(target, &hartinfo, DM_HARTINFO) != ERROR_OK)
		return ERROR_FAIL;

	info->datasize = get_field(hartinfo, DM_HARTINFO_DATASIZE);
	info->dataaccess = get_field(hartinfo, DM_HARTINFO_DATAACCESS);
	info->dataaddr = get_field(hartinfo, DM_HARTINFO_DATAADDR);

	if (!get_field(dmstatus, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Debugger is not authenticated to target Debug Module. "
				"(dmstatus=0x%x). Use `riscv authdata_read` and "
				"`riscv authdata_write` commands to authenticate.", dmstatus);
		/* If we return ERROR_FAIL here, then in a multicore setup the next
		 * core won't be examined, which means we won't set up the
		 * authentication commands for them, which means the config script
		 * needs to be a lot more complex. */
		return ERROR_OK;
	}

	if (dmi_read(target, &info->sbcs, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;

	/* Check that abstract data registers are accessible. */
	uint32_t abstractcs;
	if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
		return ERROR_FAIL;
	info->datacount = get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT);
	info->progbufsize = get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE);

	LOG_INFO("datacount=%d progbufsize=%d", info->datacount, info->progbufsize);

	RISCV_INFO(r);
	r->impebreak = get_field(dmstatus, DM_DMSTATUS_IMPEBREAK);

	if (!has_sufficient_progbuf(target, 2)) {
		LOG_WARNING("We won't be able to execute fence instructions on this "
				"target. Memory may not always appear consistent. "
				"(progbufsize=%d, impebreak=%d)", info->progbufsize,
				r->impebreak);
	}

	if (info->progbufsize < 4 && riscv_enable_virtual) {
		LOG_ERROR("set_enable_virtual is not available on this target. It "
				"requires a program buffer size of at least 4. (progbufsize=%d) "
				"Use `riscv set_enable_virtual off` to continue."
					, info->progbufsize);
	}

	/* Before doing anything else we must first enumerate the harts. */
	if (dm->hart_count < 0) {
		for (int i = 0; i < MIN(RISCV_MAX_HARTS, 1 << info->hartsellen); ++i) {
			r->current_hartid = i;
			if (riscv013_select_current_hart(target) != ERROR_OK)
				return ERROR_FAIL;

			uint32_t s;
			if (dmstatus_read(target, &s, true) != ERROR_OK)
				return ERROR_FAIL;
			if (get_field(s, DM_DMSTATUS_ANYNONEXISTENT))
				break;
			dm->hart_count = i + 1;

			if (get_field(s, DM_DMSTATUS_ANYHAVERESET))
				dmi_write(target, DM_DMCONTROL,
						set_hartsel(DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_ACKHAVERESET, i));
		}

		LOG_DEBUG("Detected %d harts.", dm->hart_count);
	}

	if (dm->hart_count == 0) {
		LOG_ERROR("No harts found!");
		return ERROR_FAIL;
	}

	/* Don't call any riscv_* functions until after we've counted the number of
	 * cores and initialized registers. */
	for (int i = 0; i < dm->hart_count; ++i) {
		if (!riscv_rtos_enabled(target) && i != target->coreid)
			continue;

		r->current_hartid = i;
		if (riscv013_select_current_hart(target) != ERROR_OK)
			return ERROR_FAIL;

		bool halted = riscv_is_halted(target);
		if (!halted) {
			if (riscv013_halt_go(target) != ERROR_OK) {
				LOG_ERROR("Fatal: Hart %d failed to halt during examine()", i);
				return ERROR_FAIL;
			}
		}

		/* Without knowing anything else we can at least mess with the
		 * program buffer. */
		r->debug_buffer_size[i] = info->progbufsize;

		int result = register_read_abstract(target, NULL, GDB_REGNO_S0, 64);
		if (result == ERROR_OK)
			r->xlen[i] = 64;
		else
			r->xlen[i] = 32;

		if (register_read(target, &r->misa[i], GDB_REGNO_MISA)) {
			LOG_ERROR("Fatal: Failed to read MISA from hart %d.", i);
			return ERROR_FAIL;
		}

		if (riscv_supports_extension(target, i, 'V')) {
			if (discover_vlenb(target, i) != ERROR_OK)
				return ERROR_FAIL;
		}

		/* Now init registers based on what we discovered. */
		if (riscv_init_registers(target) != ERROR_OK)
			return ERROR_FAIL;

		/* Display this as early as possible to help people who are using
		 * really slow simulators. */
		LOG_DEBUG(" hart %d: XLEN=%d, misa=0x%" PRIx64, i, r->xlen[i],
				r->misa[i]);

		if (!halted)
			riscv013_step_or_resume_current_hart(target, false, false);
	}

	target_set_examined(target);

	if (target->smp) {
		bool haltgroup_supported;
		if (set_haltgroup(target, &haltgroup_supported) != ERROR_OK)
			return ERROR_FAIL;
		if (haltgroup_supported)
			LOG_INFO("Core %d made part of halt group %d.", target->coreid,
					target->smp);
		else
			LOG_INFO("Core %d could not be made part of halt group %d.",
					target->coreid, target->smp);
	}

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	 * when they can connect with gdb/telnet.
	 * We will need to update those suites if we want to change that text. */
	LOG_INFO("Examined RISC-V core; found %d harts",
			riscv_count_harts(target));
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (riscv_hart_enabled(target, i)) {
			LOG_INFO(" hart %d: XLEN=%d, misa=0x%" PRIx64, i, r->xlen[i],
					r->misa[i]);
		} else {
			LOG_INFO(" hart %d: currently disabled", i);
		}
	}
	return ERROR_OK;
}

int riscv013_authdata_read(struct target *target, uint32_t *value)
{
	if (wait_for_authbusy(target, NULL) != ERROR_OK)
		return ERROR_FAIL;

	return dmi_read(target, value, DM_AUTHDATA);
}

int riscv013_authdata_write(struct target *target, uint32_t value)
{
	uint32_t before, after;
	if (wait_for_authbusy(target, &before) != ERROR_OK)
		return ERROR_FAIL;

	dmi_write(target, DM_AUTHDATA, value);

	if (wait_for_authbusy(target, &after) != ERROR_OK)
		return ERROR_FAIL;

	if (!get_field(before, DM_DMSTATUS_AUTHENTICATED) &&
			get_field(after, DM_DMSTATUS_AUTHENTICATED)) {
		LOG_INFO("authdata_write resulted in successful authentication");
		int result = ERROR_OK;
		dm013_info_t *dm = get_dm(target);
		if (!dm)
			return ERROR_FAIL;
		target_list_t *entry;
		list_for_each_entry(entry, &dm->target_list, list) {
			if (examine(entry->target) != ERROR_OK)
				result = ERROR_FAIL;
		}
		return result;
	}

	return ERROR_OK;
}

static int riscv013_hart_count(struct target *target)
{
	dm013_info_t *dm = get_dm(target);
	assert(dm);
	return dm->hart_count;
}

static unsigned riscv013_data_bits(struct target *target)
{
	RISCV013_INFO(info);
	/* TODO: Once there is a spec for discovering abstract commands, we can
	 * take those into account as well.  For now we assume abstract commands
	 * support XLEN-wide accesses. */
	if (has_sufficient_progbuf(target, 3) && !riscv_prefer_sba)
		return riscv_xlen(target);

	if (get_field(info->sbcs, DM_SBCS_SBACCESS128))
		return 128;
	if (get_field(info->sbcs, DM_SBCS_SBACCESS64))
		return 64;
	if (get_field(info->sbcs, DM_SBCS_SBACCESS32))
		return 32;
	if (get_field(info->sbcs, DM_SBCS_SBACCESS16))
		return 16;
	if (get_field(info->sbcs, DM_SBCS_SBACCESS8))
		return 8;

	return riscv_xlen(target);
}

static int prep_for_vector_access(struct target *target, uint64_t *vtype,
		uint64_t *vl, unsigned *debug_vl)
{
	RISCV_INFO(r);
	/* TODO: this continuous save/restore is terrible for performance. */
	/* Write vtype and vl. */
	unsigned encoded_vsew;
	switch (riscv_xlen(target)) {
		case 32:
			encoded_vsew = 2;
			break;
		case 64:
			encoded_vsew = 3;
			break;
		default:
			LOG_ERROR("Unsupported xlen: %d", riscv_xlen(target));
			return ERROR_FAIL;
	}

	/* Save vtype and vl. */
	if (register_read(target, vtype, GDB_REGNO_VTYPE) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read(target, vl, GDB_REGNO_VL) != ERROR_OK)
		return ERROR_FAIL;

	if (register_write_direct(target, GDB_REGNO_VTYPE, encoded_vsew << 3) != ERROR_OK)
		return ERROR_FAIL;
	*debug_vl = DIV_ROUND_UP(r->vlenb[r->current_hartid] * 8,
			riscv_xlen(target));
	if (register_write_direct(target, GDB_REGNO_VL, *debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int cleanup_after_vector_access(struct target *target, uint64_t vtype,
		uint64_t vl)
{
	/* Restore vtype and vl. */
	if (register_write_direct(target, GDB_REGNO_VTYPE, vtype) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_VL, vl) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int riscv013_get_register_buf(struct target *target,
		uint8_t *value, int regno)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	riscv_reg_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t vtype, vl;
	unsigned debug_vl;
	if (prep_for_vector_access(target, &vtype, &vl, &debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	unsigned vnum = regno - GDB_REGNO_V0;
	unsigned xlen = riscv_xlen(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, vmv_x_s(S0, vnum));
	riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));

	int result = ERROR_OK;
	for (unsigned i = 0; i < debug_vl; i++) {
		/* Executing the program might result in an exception if there is some
		 * issue with the vector implementation/instructions we're using. If that
		 * happens, attempt to restore as usual. We may have clobbered the
		 * vector register we tried to read already.
		 * For other failures, we just return error because things are probably
		 * so messed up that attempting to restore isn't going to help. */
		result = riscv_program_exec(&program, target);
		if (result == ERROR_OK) {
			uint64_t v;
			if (register_read_direct(target, &v, GDB_REGNO_S0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u64(value, xlen * i, xlen, v);
		} else {
			break;
		}
	}

	if (cleanup_after_vector_access(target, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	if (cleanup_after_register_access(target, mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int riscv013_set_register_buf(struct target *target,
		int regno, const uint8_t *value)
{
	assert(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31);

	riscv_reg_t s0;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t mstatus;
	if (prep_for_register_access(target, &mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t vtype, vl;
	unsigned debug_vl;
	if (prep_for_vector_access(target, &vtype, &vl, &debug_vl) != ERROR_OK)
		return ERROR_FAIL;

	unsigned vnum = regno - GDB_REGNO_V0;
	unsigned xlen = riscv_xlen(target);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, vslide1down_vx(vnum, vnum, S0, true));
	int result = ERROR_OK;
	for (unsigned i = 0; i < debug_vl; i++) {
		if (register_write_direct(target, GDB_REGNO_S0,
					buf_get_u64(value, xlen * i, xlen)) != ERROR_OK)
			return ERROR_FAIL;
		result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			break;
	}

	if (cleanup_after_vector_access(target, vtype, vl) != ERROR_OK)
		return ERROR_FAIL;

	if (cleanup_after_register_access(target, mstatus, regno) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;

	generic_info->get_register = &riscv013_get_register;
	generic_info->set_register = &riscv013_set_register;
	generic_info->get_register_buf = &riscv013_get_register_buf;
	generic_info->set_register_buf = &riscv013_set_register_buf;
	generic_info->select_current_hart = &riscv013_select_current_hart;
	generic_info->is_halted = &riscv013_is_halted;
	generic_info->resume_go = &riscv013_resume_go;
	generic_info->step_current_hart = &riscv013_step_current_hart;
	generic_info->on_halt = &riscv013_on_halt;
	generic_info->resume_prep = &riscv013_resume_prep;
	generic_info->halt_prep = &riscv013_halt_prep;
	generic_info->halt_go = &riscv013_halt_go;
	generic_info->on_step = &riscv013_on_step;
	generic_info->halt_reason = &riscv013_halt_reason;
	generic_info->read_debug_buffer = &riscv013_read_debug_buffer;
	generic_info->write_debug_buffer = &riscv013_write_debug_buffer;
	generic_info->execute_debug_buffer = &riscv013_execute_debug_buffer;
	generic_info->fill_dmi_write_u64 = &riscv013_fill_dmi_write_u64;
	generic_info->fill_dmi_read_u64 = &riscv013_fill_dmi_read_u64;
	generic_info->fill_dmi_nop_u64 = &riscv013_fill_dmi_nop_u64;
	generic_info->dmi_write_u64_bits = &riscv013_dmi_write_u64_bits;
	generic_info->authdata_read = &riscv013_authdata_read;
	generic_info->authdata_write = &riscv013_authdata_write;
	generic_info->dmi_read = &dmi_read;
	generic_info->dmi_write = &dmi_write;
	generic_info->read_memory = read_memory;
	generic_info->test_sba_config_reg = &riscv013_test_sba_config_reg;
	generic_info->test_compliance = &riscv013_test_compliance;
	generic_info->hart_count = &riscv013_hart_count;
	generic_info->data_bits = &riscv013_data_bits;
	generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
	if (!generic_info->version_specific)
		return ERROR_FAIL;
	riscv013_info_t *info = get_info(target);

	info->progbufsize = -1;

	info->dmi_busy_delay = 0;
	info->bus_master_read_delay = 0;
	info->bus_master_write_delay = 0;
	info->ac_busy_delay = 0;

	/* Assume all these abstract commands are supported until we learn
	 * otherwise.
	 * TODO: The spec allows eg. one CSR to be able to be accessed abstractly
	 * while another one isn't. We don't track that this closely here, but in
	 * the future we probably should. */
	info->abstract_read_csr_supported = true;
	info->abstract_write_csr_supported = true;
	info->abstract_read_fpr_supported = true;
	info->abstract_write_fpr_supported = true;

	return ERROR_OK;
}

static int assert_reset(struct target *target)
{
	RISCV_INFO(r);

	select_dmi(target);

	uint32_t control_base = set_field(0, DM_DMCONTROL_DMACTIVE, 1);

	if (target->rtos) {
		/* There's only one target, and OpenOCD thinks each hart is a thread.
		 * We must reset them all. */

		/* TODO: Try to use hasel in dmcontrol */

		/* Set haltreq for each hart. */
		uint32_t control = control_base;
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			if (!riscv_hart_enabled(target, i))
				continue;

			control = set_hartsel(control_base, i);
			control = set_field(control, DM_DMCONTROL_HALTREQ,
					target->reset_halt ? 1 : 0);
			dmi_write(target, DM_DMCONTROL, control);
		}
		/* Assert ndmreset */
		control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DM_DMCONTROL, control);

	} else {
		/* Reset just this hart. */
		uint32_t control = set_hartsel(control_base, r->current_hartid);
		control = set_field(control, DM_DMCONTROL_HALTREQ,
				target->reset_halt ? 1 : 0);
		control = set_field(control, DM_DMCONTROL_NDMRESET, 1);
		dmi_write(target, DM_DMCONTROL, control);
	}

	target->state = TARGET_RESET;

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;

	/* The DM might have gotten reset if OpenOCD called us in some reset that
	 * involves SRST being toggled. So clear our cache which may be out of
	 * date. */
	memset(dm->progbuf_cache, 0, sizeof(dm->progbuf_cache));

	return ERROR_OK;
}

static int deassert_reset(struct target *target)
{
	RISCV_INFO(r);
	RISCV013_INFO(info);
	select_dmi(target);

	/* Clear the reset, but make sure haltreq is still set */
	uint32_t control = 0;
	control = set_field(control, DM_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
	control = set_field(control, DM_DMCONTROL_DMACTIVE, 1);
	dmi_write(target, DM_DMCONTROL,
			set_hartsel(control, r->current_hartid));

	uint32_t dmstatus;
	int dmi_busy_delay = info->dmi_busy_delay;
	time_t start = time(NULL);

	for (int i = 0; i < riscv_count_harts(target); ++i) {
		int index = i;
		if (target->rtos) {
			if (!riscv_hart_enabled(target, index))
				continue;
			dmi_write(target, DM_DMCONTROL,
					set_hartsel(control, index));
		} else {
			index = r->current_hartid;
		}

		char *operation;
		uint32_t expected_field;
		if (target->reset_halt) {
			operation = "halt";
			expected_field = DM_DMSTATUS_ALLHALTED;
		} else {
			operation = "run";
			expected_field = DM_DMSTATUS_ALLRUNNING;
		}
		LOG_DEBUG("Waiting for hart %d to %s out of reset.", index, operation);
		while (1) {
			int result = dmstatus_read_timeout(target, &dmstatus, true,
					riscv_reset_timeout_sec);
			if (result == ERROR_TIMEOUT_REACHED)
				LOG_ERROR("Hart %d didn't complete a DMI read coming out of "
						"reset in %ds; Increase the timeout with riscv "
						"set_reset_timeout_sec.",
						index, riscv_reset_timeout_sec);
			if (result != ERROR_OK)
				return result;
			if (get_field(dmstatus, expected_field))
				break;
			if (time(NULL) - start > riscv_reset_timeout_sec) {
				LOG_ERROR("Hart %d didn't %s coming out of reset in %ds; "
						"dmstatus=0x%x; "
						"Increase the timeout with riscv set_reset_timeout_sec.",
						index, operation, riscv_reset_timeout_sec, dmstatus);
				return ERROR_FAIL;
			}
		}
		target->state = TARGET_HALTED;

		if (get_field(dmstatus, DM_DMSTATUS_ALLHAVERESET)) {
			/* Ack reset. */
			dmi_write(target, DM_DMCONTROL,
					set_hartsel(control, index) |
					DM_DMCONTROL_ACKHAVERESET);
		}

		if (!target->rtos)
			break;
	}
	info->dmi_busy_delay = dmi_busy_delay;
	return ERROR_OK;
}

static int execute_fence(struct target *target)
{
	int old_hartid = riscv_current_hartid(target);

	/* FIXME: For non-coherent systems we need to flush the caches right
	 * here, but there's no ISA-defined way of doing that. */
	{
		struct riscv_program program;
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		riscv_program_fence(&program);
		int result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			LOG_DEBUG("Unable to execute pre-fence");
	}

	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		if (i == old_hartid)
			/* Fence already executed for this hart */
			continue;

		riscv_set_current_hartid(target, i);

		struct riscv_program program;
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		riscv_program_fence(&program);
		int result = riscv_program_exec(&program, target);
		if (result != ERROR_OK)
			LOG_DEBUG("Unable to execute fence on hart %d", i);
	}

	riscv_set_current_hartid(target, old_hartid);

	return ERROR_OK;
}

static void log_memory_access(target_addr_t address, uint64_t value,
		unsigned size_bytes, bool read)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char fmt[80];
	sprintf(fmt, "M[0x%" TARGET_PRIxADDR "] %ss 0x%%0%d" PRIx64,
			address, read ? "read" : "write", size_bytes * 2);
	switch (size_bytes) {
		case 1:
			value &= 0xff;
			break;
		case 2:
			value &= 0xffff;
			break;
		case 4:
			value &= 0xffffffffUL;
			break;
		case 8:
			break;
		default:
			assert(false);
	}
	LOG_DEBUG(fmt, value);
}

/* Read the relevant sbdata regs depending on size, and put the results into
 * buffer. */
static int read_memory_bus_word(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	uint32_t value;
	int result;
	static int sbdata[4] = { DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3 };
	assert(size <= 16);
	for (int i = (size - 1) / 4; i >= 0; i--) {
		result = dmi_op(target, &value, NULL, DMI_OP_READ, sbdata[i], 0, false, true);
		if (result != ERROR_OK)
			return result;
		buf_set_u32(buffer + i * 4, 0, 8 * MIN(size, 4), value);
		log_memory_access(address + i * 4, value, MIN(size, 4), true);
	}
	return ERROR_OK;
}

static uint32_t sb_sbaccess(unsigned size_bytes)
{
	switch (size_bytes) {
		case 1:
			return set_field(0, DM_SBCS_SBACCESS, 0);
		case 2:
			return set_field(0, DM_SBCS_SBACCESS, 1);
		case 4:
			return set_field(0, DM_SBCS_SBACCESS, 2);
		case 8:
			return set_field(0, DM_SBCS_SBACCESS, 3);
		case 16:
			return set_field(0, DM_SBCS_SBACCESS, 4);
	}
	assert(0);
	return 0;	/* Make mingw happy. */
}

static target_addr_t sb_read_address(struct target *target)
{
	RISCV013_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	target_addr_t address = 0;
	uint32_t v;
	if (sbasize > 32) {
		dmi_read(target, &v, DM_SBADDRESS1);
		address |= v;
		address <<= 32;
	}
	dmi_read(target, &v, DM_SBADDRESS0);
	address |= v;
	return address;
}

static int sb_write_address(struct target *target, target_addr_t address)
{
	RISCV013_INFO(info);
	unsigned sbasize = get_field(info->sbcs, DM_SBCS_SBASIZE);
	/* There currently is no support for >64-bit addresses in OpenOCD. */
	if (sbasize > 96)
		dmi_write(target, DM_SBADDRESS3, 0);
	if (sbasize > 64)
		dmi_write(target, DM_SBADDRESS2, 0);
	if (sbasize > 32)
		dmi_write(target, DM_SBADDRESS1, address >> 32);
	return dmi_write(target, DM_SBADDRESS0, address);
}

static int read_sbcs_nonbusy(struct target *target, uint32_t *sbcs)
{
	time_t start = time(NULL);
	while (1) {
		if (dmi_read(target, sbcs, DM_SBCS) != ERROR_OK)
			return ERROR_FAIL;
		if (!get_field(*sbcs, DM_SBCS_SBBUSY))
			return ERROR_OK;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
					"Increase the timeout with riscv set_command_timeout_sec.",
					riscv_command_timeout_sec, *sbcs);
			return ERROR_FAIL;
		}
	}
}

static int modify_privilege(struct target *target, uint64_t *mstatus, uint64_t *mstatus_old)
{
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5)) {
		/* Read DCSR */
		uint64_t dcsr;
		if (register_read(target, &dcsr, GDB_REGNO_DCSR) != ERROR_OK)
			return ERROR_FAIL;

		/* Read and save MSTATUS */
		if (register_read(target, mstatus, GDB_REGNO_MSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		*mstatus_old = *mstatus;

		/* If we come from m-mode with mprv set, we want to keep mpp */
		if (get_field(dcsr, DCSR_PRV) < 3) {
			/* MPP = PRIV */
			*mstatus = set_field(*mstatus, MSTATUS_MPP, get_field(dcsr, DCSR_PRV));

			/* MPRV = 1 */
			*mstatus = set_field(*mstatus, MSTATUS_MPRV, 1);

			/* Write MSTATUS */
			if (*mstatus != *mstatus_old)
				if (register_write_direct(target, GDB_REGNO_MSTATUS, *mstatus) != ERROR_OK)
					return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int read_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (size != increment) {
		LOG_ERROR("sba v0 reads only support size==increment");
		return ERROR_NOT_IMPLEMENTED;
	}

	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	uint8_t *t_buffer = buffer;
	riscv_addr_t cur_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	uint32_t access = 0;

	const int DM_SBCS_SBSINGLEREAD_OFFSET = 20;
	const uint32_t DM_SBCS_SBSINGLEREAD = (0x1U << DM_SBCS_SBSINGLEREAD_OFFSET);

	const int DM_SBCS_SBAUTOREAD_OFFSET = 15;
	const uint32_t DM_SBCS_SBAUTOREAD = (0x1U << DM_SBCS_SBAUTOREAD_OFFSET);

	/* ww favorise one off reading if there is an issue */
	if (count == 1) {
		for (uint32_t i = 0; i < count; i++) {
			if (dmi_read(target, &access, DM_SBCS) != ERROR_OK)
				return ERROR_FAIL;
			dmi_write(target, DM_SBADDRESS0, cur_addr);
			/* size/2 matching the bit access of the spec 0.13 */
			access = set_field(access, DM_SBCS_SBACCESS, size/2);
			access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
			LOG_DEBUG("\r\nread_memory: sab: access:  0x%08x", access);
			dmi_write(target, DM_SBCS, access);
			/* 3) read */
			uint32_t value;
			if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			LOG_DEBUG("\r\nread_memory: sab: value:  0x%08x", value);
			buf_set_u32(t_buffer, 0, 8 * size, value);
			t_buffer += size;
			cur_addr += size;
		}
		return ERROR_OK;
	}

	/* has to be the same size if we want to read a block */
	LOG_DEBUG("reading block until final address 0x%" PRIx64, fin_addr);
	if (dmi_read(target, &access, DM_SBCS) != ERROR_OK)
		return ERROR_FAIL;
	/* set current address */
	dmi_write(target, DM_SBADDRESS0, cur_addr);
	/* 2) write sbaccess=2, sbsingleread,sbautoread,sbautoincrement
	 * size/2 matching the bit access of the spec 0.13 */
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOREAD, 1);
	access = set_field(access, DM_SBCS_SBSINGLEREAD, 1);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08x", access);
	dmi_write(target, DM_SBCS, access);

	while (cur_addr < fin_addr) {
		LOG_DEBUG("\r\nsab:autoincrement: \r\n size: %d\tcount:%d\taddress: 0x%08"
				PRIx64, size, count, cur_addr);
		/* read */
		uint32_t value;
		if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
			return ERROR_FAIL;
		buf_set_u32(t_buffer, 0, 8 * size, value);
		cur_addr += size;
		t_buffer += size;

		/* if we are reaching last address, we must clear autoread */
		if (cur_addr == fin_addr && count != 1) {
			dmi_write(target, DM_SBCS, 0);
			if (dmi_read(target, &value, DM_SBDATA0) != ERROR_OK)
				return ERROR_FAIL;
			buf_set_u32(t_buffer, 0, 8 * size, value);
		}
	}

	return ERROR_OK;
}

/**
 * Read the requested memory using the system bus interface.
 */
static int read_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (increment != size && increment != 0) {
		LOG_ERROR("sba v1 reads only support increment of size or 0");
		return ERROR_NOT_IMPLEMENTED;
	}

	RISCV013_INFO(info);
	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	while (next_address < end_address) {
		uint32_t sbcs_write = set_field(0, DM_SBCS_SBREADONADDR, 1);
		sbcs_write |= sb_sbaccess(size);
		if (increment == size)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBAUTOINCREMENT, 1);
		if (count > 1)
			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, count > 1);
		if (dmi_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
			return ERROR_FAIL;

		/* This address write will trigger the first read. */
		if (sb_write_address(target, next_address) != ERROR_OK)
			return ERROR_FAIL;

		if (info->bus_master_read_delay) {
			jtag_add_runtest(info->bus_master_read_delay, TAP_IDLE);
			if (jtag_execute_queue() != ERROR_OK) {
				LOG_ERROR("Failed to scan idle sequence");
				return ERROR_FAIL;
			}
		}

		/* First value has been read, and is waiting for us to issue a DMI read
		 * to get it. */

		static int sbdata[4] = {DM_SBDATA0, DM_SBDATA1, DM_SBDATA2, DM_SBDATA3};
		assert(size <= 16);
		target_addr_t next_read = address - 1;
		for (uint32_t i = (next_address - address) / size; i < count - 1; i++) {
			for (int j = (size - 1) / 4; j >= 0; j--) {
				uint32_t value;
				unsigned attempt = 0;
				while (1) {
					if (attempt++ > 100) {
						LOG_ERROR("DMI keeps being busy in while reading memory just past " TARGET_ADDR_FMT,
								  next_read);
						return ERROR_FAIL;
					}
					dmi_status_t status = dmi_scan(target, NULL, &value,
												   DMI_OP_READ, sbdata[j], 0, false);
					if (status == DMI_STATUS_BUSY)
						increase_dmi_busy_delay(target);
					else if (status == DMI_STATUS_SUCCESS)
						break;
					else
						return ERROR_FAIL;
				}
				if (next_read != address - 1) {
					buf_set_u32(buffer + next_read - address, 0, 8 * MIN(size, 4), value);
					log_memory_access(next_read, value, MIN(size, 4), true);
				}
				next_read = address + i * size + j * 4;
			}
		}

		uint32_t sbcs_read = 0;
		if (count > 1) {
			uint32_t value;
			unsigned attempt = 0;
			while (1) {
				if (attempt++ > 100) {
					LOG_ERROR("DMI keeps being busy in while reading memory just past " TARGET_ADDR_FMT,
								next_read);
					return ERROR_FAIL;
				}
				dmi_status_t status = dmi_scan(target, NULL, &value, DMI_OP_NOP, 0, 0, false);
				if (status == DMI_STATUS_BUSY)
					increase_dmi_busy_delay(target);
				else if (status == DMI_STATUS_SUCCESS)
					break;
				else
					return ERROR_FAIL;
			}
			buf_set_u32(buffer + next_read - address, 0, 8 * MIN(size, 4), value);
			log_memory_access(next_read, value, MIN(size, 4), true);

			/* "Writes to sbcs while sbbusy is high result in undefined behavior.
			 * A debugger must not write to sbcs until it reads sbbusy as 0." */
			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;

			sbcs_write = set_field(sbcs_write, DM_SBCS_SBREADONDATA, 0);
			if (dmi_write(target, DM_SBCS, sbcs_write) != ERROR_OK)
				return ERROR_FAIL;
		}

		/* Read the last word, after we disabled sbreadondata if necessary. */
		if (!get_field(sbcs_read, DM_SBCS_SBERROR) &&
				!get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			if (read_memory_bus_word(target, address + (count - 1) * size, size,
						buffer + (count - 1) * size) != ERROR_OK)
				return ERROR_FAIL;

			if (read_sbcs_nonbusy(target, &sbcs_read) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs_read, DM_SBCS_SBBUSYERROR)) {
			/* We read while the target was busy. Slow down and try again. */
			if (dmi_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR) != ERROR_OK)
				return ERROR_FAIL;
			next_address = sb_read_address(target);
			info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
			continue;
		}

		unsigned error = get_field(sbcs_read, DM_SBCS_SBERROR);
		if (error == 0) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			if (dmi_write(target, DM_SBCS, DM_SBCS_SBERROR) != ERROR_OK)
				return ERROR_FAIL;
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int batch_run(const struct target *target, struct riscv_batch *batch)
{
	RISCV013_INFO(info);
	RISCV_INFO(r);
	if (r->reset_delays_wait >= 0) {
		r->reset_delays_wait -= batch->used_scans;
		if (r->reset_delays_wait <= 0) {
			batch->idle_count = 0;
			info->dmi_busy_delay = 0;
			info->ac_busy_delay = 0;
		}
	}
	return riscv_batch_run(batch);
}

/*
 * Performs a memory read using memory access abstract commands. The read sizes
 * supported are 1, 2, and 4 bytes despite the spec's support of 8 and 16 byte
 * aamsize fields in the memory access abstract command.
 */
static int read_memory_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (size != increment) {
		LOG_ERROR("abstract command reads only support size==increment");
		return ERROR_NOT_IMPLEMENTED;
	}

	int result = ERROR_OK;

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	memset(buffer, 0, count * size);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;
	if (width > 64) {
		/* TODO: Add 128b support if it's ever used. Involves modifying
				 read/write_abstract_arg() to work on two 64b values. */
		LOG_ERROR("Unsupported size: %d bits", size);
		return ERROR_FAIL;
	}

	/* Create the command (physical address, postincrement, read) */
	uint32_t command = access_memory_command(target, false, width, true, false);

	/* Execute the reads */
	uint8_t *p = buffer;
	bool updateaddr = true;
	unsigned width32 = (width + 31) / 32 * 32;
	for (uint32_t c = 0; c < count; c++) {
		/* Only update the address initially and let postincrement update it */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_ERROR("Failed to write arg1 during read_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to execute command read_memory_abstract().");
			return result;
		}

		/* Copy arg0 to buffer (rounded width up to nearest 32) */
		riscv_reg_t value = read_abstract_arg(target, 0, width32);
		buf_set_u64(p, 0, 8 * size, value);

		updateaddr = false;
		p += size;
	}

	return result;
}

/*
 * Performs a memory write using memory access abstract commands. The write
 * sizes supported are 1, 2, and 4 bytes despite the spec's support of 8 and 16
 * byte aamsize fields in the memory access abstract command.
 */
static int write_memory_abstract(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int result = ERROR_OK;

	LOG_DEBUG("writing %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			  size, address);

	/* Convert the size (bytes) to width (bits) */
	unsigned width = size << 3;
	if (width > 64) {
		/* TODO: Add 128b support if it's ever used. Involves modifying
				 read/write_abstract_arg() to work on two 64b values. */
		LOG_ERROR("Unsupported size: %d bits", width);
		return ERROR_FAIL;
	}

	/* Create the command (physical address, postincrement, write) */
	uint32_t command = access_memory_command(target, false, width, true, true);

	/* Execute the writes */
	const uint8_t *p = buffer;
	bool updateaddr = true;
	for (uint32_t c = 0; c < count; c++) {
		/* Move data to arg0 */
		riscv_reg_t value = buf_get_u64(p, 0, 8 * size);
		result = write_abstract_arg(target, 0, value, riscv_xlen(target));
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to write arg0 during write_memory_abstract().");
			return result;
		}

		/* Only update the address initially and let postincrement update it */
		if (updateaddr) {
			/* Set arg1 to the address: address + c * size */
			result = write_abstract_arg(target, 1, address, riscv_xlen(target));
			if (result != ERROR_OK) {
				LOG_ERROR("Failed to write arg1 during write_memory_abstract().");
				return result;
			}
		}

		/* Execute the command */
		result = execute_abstract_command(target, command);
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to execute command write_memory_abstract().");
			return result;
		}

		updateaddr = false;
		p += size;
	}

	return result;
}

/**
 * Read the requested memory, taking care to execute every read exactly once,
 * even if cmderr=busy is encountered.
 */
static int read_memory_progbuf_inner(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	RISCV013_INFO(info);

	int result = ERROR_OK;

	/* Write address to S0. */
	result = register_write_direct(target, GDB_REGNO_S0, address);
	if (result != ERROR_OK)
		return result;

	if (increment == 0 &&
			register_write_direct(target, GDB_REGNO_S2, 0) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t command = access_register_command(target, GDB_REGNO_S1,
			riscv_xlen(target),
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	if (execute_abstract_command(target, command) != ERROR_OK)
		return ERROR_FAIL;

	/* First read has just triggered. Result is in s1. */
	if (count == 1) {
		uint64_t value;
		if (register_read_direct(target, &value, GDB_REGNO_S1) != ERROR_OK)
			return ERROR_FAIL;
		buf_set_u64(buffer, 0, 8 * size, value);
		log_memory_access(address, value, size, true);
		return ERROR_OK;
	}

	if (dmi_write(target, DM_ABSTRACTAUTO,
			1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET) != ERROR_OK)
		goto error;
	/* Read garbage from dmi_data0, which triggers another execution of the
	 * program. Now dmi_data0 contains the first good result, and s1 the next
	 * memory value. */
	if (dmi_read_exec(target, NULL, DM_DATA0) != ERROR_OK)
		goto error;

	/* read_addr is the next address that the hart will read from, which is the
	 * value in s0. */
	unsigned index = 2;
	while (index < count) {
		riscv_addr_t read_addr = address + index * increment;
		LOG_DEBUG("i=%d, count=%d, read_addr=0x%" PRIx64, index, count, read_addr);
		/* The pipeline looks like this:
		 * memory -> s1 -> dm_data0 -> debugger
		 * Right now:
		 * s0 contains read_addr
		 * s1 contains mem[read_addr-size]
		 * dm_data0 contains[read_addr-size*2]
		 */

		struct riscv_batch *batch = riscv_batch_alloc(target, 32,
				info->dmi_busy_delay + info->ac_busy_delay);
		if (!batch)
			return ERROR_FAIL;

		unsigned reads = 0;
		for (unsigned j = index; j < count; j++) {
			if (size > 4)
				riscv_batch_add_dmi_read(batch, DM_DATA1);
			riscv_batch_add_dmi_read(batch, DM_DATA0);

			reads++;
			if (riscv_batch_full(batch))
				break;
		}

		batch_run(target, batch);

		/* Wait for the target to finish performing the last abstract command,
		 * and update our copy of cmderr. If we see that DMI is busy here,
		 * dmi_busy_delay will be incremented. */
		uint32_t abstractcs;
		if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
			return ERROR_FAIL;
		while (get_field(abstractcs, DM_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);

		unsigned next_index;
		unsigned ignore_last = 0;
		switch (info->cmderr) {
			case CMDERR_NONE:
				LOG_DEBUG("successful (partial?) memory read");
				next_index = index + reads;
				break;
			case CMDERR_BUSY:
				LOG_DEBUG("memory read resulted in busy response");

				increase_ac_busy_delay(target);
				riscv013_clear_abstract_error(target);

				dmi_write(target, DM_ABSTRACTAUTO, 0);

				uint32_t dmi_data0, dmi_data1 = 0;
				/* This is definitely a good version of the value that we
				 * attempted to read when we discovered that the target was
				 * busy. */
				if (dmi_read(target, &dmi_data0, DM_DATA0) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}
				if (size > 4 && dmi_read(target, &dmi_data1, DM_DATA1) != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* See how far we got, clobbering dmi_data0. */
				if (increment == 0) {
					uint64_t counter;
					result = register_read_direct(target, &counter, GDB_REGNO_S2);
					next_index = counter;
				} else {
					uint64_t next_read_addr;
					result = register_read_direct(target, &next_read_addr,
												  GDB_REGNO_S0);
					next_index = (next_read_addr - address) / increment;
				}
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				uint64_t value64 = (((uint64_t)dmi_data1) << 32) | dmi_data0;
				buf_set_u64(buffer + (next_index - 2) * size, 0, 8 * size, value64);
				log_memory_access(address + (next_index - 2) * size, value64, size, true);

				/* Restore the command, and execute it.
				 * Now DM_DATA0 contains the next value just as it would if no
				 * error had occurred. */
				dmi_write_exec(target, DM_COMMAND, command, true);
				next_index++;

				dmi_write(target, DM_ABSTRACTAUTO,
						1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				ignore_last = 1;

				break;
			default:
				LOG_DEBUG("error when reading memory, abstractcs=0x%08lx", (long)abstractcs);
				riscv013_clear_abstract_error(target);
				riscv_batch_free(batch);
				result = ERROR_FAIL;
				goto error;
		}

		/* Now read whatever we got out of the batch. */
		dmi_status_t status = DMI_STATUS_SUCCESS;
		unsigned read = 0;
		assert(index >= 2);
		for (unsigned j = index - 2; j < index + reads; j++) {
			assert(j < count);
			LOG_DEBUG("index=%d, reads=%d, next_index=%d, ignore_last=%d, j=%d",
				index, reads, next_index, ignore_last, j);
			if (j + 3 + ignore_last > next_index)
				break;

			status = riscv_batch_get_dmi_read_op(batch, read);
			uint64_t value = riscv_batch_get_dmi_read_data(batch, read);
			read++;
			if (status != DMI_STATUS_SUCCESS) {
				/* If we're here because of busy count, dmi_busy_delay will
				 * already have been increased and busy state will have been
				 * cleared in dmi_read(). */
				/* In at least some implementations, we issue a read, and then
				 * can get busy back when we try to scan out the read result,
				 * and the actual read value is lost forever. Since this is
				 * rare in any case, we return error here and rely on our
				 * caller to reread the entire block. */
				LOG_WARNING("Batch memory read encountered DMI error %d. "
						"Falling back on slower reads.", status);
				riscv_batch_free(batch);
				result = ERROR_FAIL;
				goto error;
			}
			if (size > 4) {
				status = riscv_batch_get_dmi_read_op(batch, read);
				if (status != DMI_STATUS_SUCCESS) {
					LOG_WARNING("Batch memory read encountered DMI error %d. "
							"Falling back on slower reads.", status);
					riscv_batch_free(batch);
					result = ERROR_FAIL;
					goto error;
				}
				value <<= 32;
				value |= riscv_batch_get_dmi_read_data(batch, read);
				read++;
			}
			riscv_addr_t offset = j * size;
			buf_set_u64(buffer + offset, 0, 8 * size, value);
			log_memory_access(address + j * increment, value, size, true);
		}

		index = next_index;

		riscv_batch_free(batch);
	}

	dmi_write(target, DM_ABSTRACTAUTO, 0);

	if (count > 1) {
		/* Read the penultimate word. */
		uint32_t dmi_data0, dmi_data1 = 0;
		if (dmi_read(target, &dmi_data0, DM_DATA0) != ERROR_OK)
			return ERROR_FAIL;
		if (size > 4 && dmi_read(target, &dmi_data1, DM_DATA1) != ERROR_OK)
			return ERROR_FAIL;
		uint64_t value64 = (((uint64_t)dmi_data1) << 32) | dmi_data0;
		buf_set_u64(buffer + size * (count - 2), 0, 8 * size, value64);
		log_memory_access(address + size * (count - 2), value64, size, true);
	}

	/* Read the last word. */
	uint64_t value;
	result = register_read_direct(target, &value, GDB_REGNO_S1);
	if (result != ERROR_OK)
		goto error;
	buf_set_u64(buffer + size * (count-1), 0, 8 * size, value);
	log_memory_access(address + size * (count-1), value, size, true);

	return ERROR_OK;

error:
	dmi_write(target, DM_ABSTRACTAUTO, 0);

	return result;
}

/* Only need to save/restore one GPR to read a single word, and the progbuf
 * program doesn't need to increment. */
static int read_memory_progbuf_one(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t s0;

	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_ldr(&program, GDB_REGNO_S0, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	/* Write address to S0, and execute buffer. */
	if (write_abstract_arg(target, 0, address, riscv_xlen(target)) != ERROR_OK)
		return ERROR_FAIL;
	uint32_t command = access_register_command(target, GDB_REGNO_S0,
			riscv_xlen(target), AC_ACCESS_REGISTER_WRITE |
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);
	if (execute_abstract_command(target, command) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t value;
	if (register_read(target, &value, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	buf_set_u64(buffer, 0, 8 * size, value);
	log_memory_access(address, value, size, true);

	if (riscv_set_register(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * Read the requested memory, silently handling memory access errors.
 */
static int read_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (riscv_xlen(target) < size * 8) {
		LOG_ERROR("XLEN (%d) is too short for %d-bit memory read.",
				riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	int result = ERROR_OK;

	LOG_DEBUG("reading %d words of %d bytes from 0x%" TARGET_PRIxADDR, count,
			size, address);

	select_dmi(target);

	memset(buffer, 0, count*size);

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	if (count == 1)
		return read_memory_progbuf_one(target, address, size, buffer);

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	/* s0 holds the next address to write to
	 * s1 holds the next data value to write
	 * s2 is a counter in case increment is 0
	 */
	uint64_t s0, s1, s2;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read(target, &s1, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;
	if (increment == 0 && register_read(target, &s2, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_ldr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}

	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
	if (increment == 0)
		riscv_program_addi(&program, GDB_REGNO_S2, GDB_REGNO_S2, 1);
	else
		riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, increment);

	if (riscv_program_ebreak(&program) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_program_write(&program) != ERROR_OK)
		return ERROR_FAIL;

	result = read_memory_progbuf_inner(target, address, size, count, buffer, increment);

	if (result != ERROR_OK) {
		/* The full read did not succeed, so we will try to read each word individually. */
		/* This will not be fast, but reading outside actual memory is a special case anyway. */
		/* It will make the toolchain happier, especially Eclipse Memory View as it reads ahead. */
		target_addr_t address_i = address;
		uint32_t count_i = 1;
		uint8_t *buffer_i = buffer;

		for (uint32_t i = 0; i < count; i++, address_i += increment, buffer_i += size) {
			keep_alive();
			/* TODO: This is much slower than it needs to be because we end up
			 * writing the address to read for every word we read. */
			result = read_memory_progbuf_inner(target, address_i, size, count_i, buffer_i, increment);

			/* The read of a single word failed, so we will just return 0 for that instead */
			if (result != ERROR_OK) {
				LOG_DEBUG("error reading single word of %d bytes from 0x%" TARGET_PRIxADDR,
						size, address_i);

				buf_set_u64(buffer_i, 0, 8 * size, 0);
			}
		}
		result = ERROR_OK;
	}

	riscv_set_register(target, GDB_REGNO_S0, s0);
	riscv_set_register(target, GDB_REGNO_S1, s1);
	if (increment == 0)
		riscv_set_register(target, GDB_REGNO_S2, s2);

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	return result;
}

static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (count == 0)
		return ERROR_OK;

	RISCV013_INFO(info);
	if (has_sufficient_progbuf(target, 3) && !riscv_prefer_sba)
		return read_memory_progbuf(target, address, size, count, buffer,
			increment);

	if ((get_field(info->sbcs, DM_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS128) && size == 16)) {
		if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
			return read_memory_bus_v0(target, address, size, count, buffer,
				increment);
		else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
			return read_memory_bus_v1(target, address, size, count, buffer,
				increment);
	}

	if (has_sufficient_progbuf(target, 3))
		return read_memory_progbuf(target, address, size, count, buffer,
			increment);

	return read_memory_abstract(target, address, size, count, buffer,
		increment);
}

static int write_memory_bus_v0(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/*1) write sbaddress: for singlewrite and autoincrement, we need to write the address once*/
	LOG_DEBUG("System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, size, count, address);
	dmi_write(target, DM_SBADDRESS0, address);
	int64_t value = 0;
	int64_t access = 0;
	riscv_addr_t offset = 0;
	riscv_addr_t t_addr = 0;
	const uint8_t *t_buffer = buffer + offset;

	/* B.8 Writing Memory, single write check if we write in one go */
	if (count == 1) { /* count is in bytes here */
		value = buf_get_u64(t_buffer, 0, 8 * size);

		access = 0;
		access = set_field(access, DM_SBCS_SBACCESS, size/2);
		dmi_write(target, DM_SBCS, access);
		LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
		LOG_DEBUG("\r\nwrite_memory:SAB: ONE OFF: value 0x%08" PRIx64, value);
		dmi_write(target, DM_SBDATA0, value);
		return ERROR_OK;
	}

	/*B.8 Writing Memory, using autoincrement*/

	access = 0;
	access = set_field(access, DM_SBCS_SBACCESS, size/2);
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 1);
	LOG_DEBUG("\r\naccess:  0x%08" PRIx64, access);
	dmi_write(target, DM_SBCS, access);

	/*2)set the value according to the size required and write*/
	for (riscv_addr_t i = 0; i < count; ++i) {
		offset = size*i;
		/* for monitoring only */
		t_addr = address + offset;
		t_buffer = buffer + offset;

		value = buf_get_u64(t_buffer, 0, 8 * size);
		LOG_DEBUG("SAB:autoincrement: expected address: 0x%08x value: 0x%08x"
				PRIx64, (uint32_t)t_addr, (uint32_t)value);
		dmi_write(target, DM_SBDATA0, value);
	}
	/*reset the autoincrement when finished (something weird is happening if this is not done at the end*/
	access = set_field(access, DM_SBCS_SBAUTOINCREMENT, 0);
	dmi_write(target, DM_SBCS, access);

	return ERROR_OK;
}

static int write_memory_bus_v1(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);
	uint32_t sbcs = sb_sbaccess(size);
	sbcs = set_field(sbcs, DM_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DM_SBCS, sbcs);

	target_addr_t next_address = address;
	target_addr_t end_address = address + count * size;

	int result;

	sb_write_address(target, next_address);
	while (next_address < end_address) {
		LOG_DEBUG("transferring burst starting at address 0x%" TARGET_PRIxADDR,
				next_address);

		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				32,
				info->dmi_busy_delay + info->bus_master_write_delay);
		if (!batch)
			return ERROR_FAIL;

		for (uint32_t i = (next_address - address) / size; i < count; i++) {
			const uint8_t *p = buffer + i * size;

			if (riscv_batch_available_scans(batch) < (size + 3) / 4)
				break;

			if (size > 12)
				riscv_batch_add_dmi_write(batch, DM_SBDATA3,
						((uint32_t) p[12]) |
						(((uint32_t) p[13]) << 8) |
						(((uint32_t) p[14]) << 16) |
						(((uint32_t) p[15]) << 24));

			if (size > 8)
				riscv_batch_add_dmi_write(batch, DM_SBDATA2,
						((uint32_t) p[8]) |
						(((uint32_t) p[9]) << 8) |
						(((uint32_t) p[10]) << 16) |
						(((uint32_t) p[11]) << 24));
			if (size > 4)
				riscv_batch_add_dmi_write(batch, DM_SBDATA1,
						((uint32_t) p[4]) |
						(((uint32_t) p[5]) << 8) |
						(((uint32_t) p[6]) << 16) |
						(((uint32_t) p[7]) << 24));
			uint32_t value = p[0];
			if (size > 2) {
				value |= ((uint32_t) p[2]) << 16;
				value |= ((uint32_t) p[3]) << 24;
			}
			if (size > 1)
				value |= ((uint32_t) p[1]) << 8;
			riscv_batch_add_dmi_write(batch, DM_SBDATA0, value);

			log_memory_access(address + i * size, value, size, false);
			next_address += size;
		}

		result = batch_run(target, batch);
		riscv_batch_free(batch);
		if (result != ERROR_OK)
			return result;

		bool dmi_busy_encountered;
		if (dmi_op(target, &sbcs, &dmi_busy_encountered, DMI_OP_READ,
				DM_SBCS, 0, false, false) != ERROR_OK)
			return ERROR_FAIL;

		time_t start = time(NULL);
		bool dmi_busy = dmi_busy_encountered;
		while (get_field(sbcs, DM_SBCS_SBBUSY) || dmi_busy) {
			if (time(NULL) - start > riscv_command_timeout_sec) {
				LOG_ERROR("Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x). "
					  "Increase the timeout with riscv set_command_timeout_sec.",
					  riscv_command_timeout_sec, sbcs);
				return ERROR_FAIL;
			}

			if (dmi_op(target, &sbcs, &dmi_busy, DMI_OP_READ,
						DM_SBCS, 0, false, true) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. Slow down and try again. */
			dmi_write(target, DM_SBCS, DM_SBCS_SBBUSYERROR);
			info->bus_master_write_delay += info->bus_master_write_delay / 10 + 1;
		}

		if (get_field(sbcs, DM_SBCS_SBBUSYERROR) || dmi_busy_encountered) {
			next_address = sb_read_address(target);
			if (next_address < address) {
				/* This should never happen, probably buggy hardware. */
				LOG_DEBUG("unexpected system bus address 0x%" TARGET_PRIxADDR,
					  next_address);
				return ERROR_FAIL;
			}

			continue;
		}

		unsigned error = get_field(sbcs, DM_SBCS_SBERROR);
		if (error != 0) {
			/* Some error indicating the bus access failed, but not because of
			 * something we did wrong. */
			dmi_write(target, DM_SBCS, DM_SBCS_SBERROR);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int write_memory_progbuf(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);

	if (riscv_xlen(target) < size * 8) {
		LOG_ERROR("XLEN (%d) is too short for %d-bit memory write.",
				riscv_xlen(target), size * 8);
		return ERROR_FAIL;
	}

	LOG_DEBUG("writing %d words of %d bytes to 0x%08lx", count, size, (long)address);

	select_dmi(target);

	uint64_t mstatus = 0;
	uint64_t mstatus_old = 0;
	if (modify_privilege(target, &mstatus, &mstatus_old) != ERROR_OK)
		return ERROR_FAIL;

	/* s0 holds the next address to write to
	 * s1 holds the next data value to write
	 */

	int result = ERROR_OK;
	uint64_t s0, s1;
	if (register_read(target, &s0, GDB_REGNO_S0) != ERROR_OK)
		return ERROR_FAIL;
	if (register_read(target, &s1, GDB_REGNO_S1) != ERROR_OK)
		return ERROR_FAIL;

	/* Write the program (store, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrsi(&program, GDB_REGNO_ZERO, CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);

	switch (size) {
		case 1:
			riscv_program_sbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_shr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_swr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 8:
			riscv_program_sdr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("write_memory_progbuf(): Unsupported size: %d", size);
			result = ERROR_FAIL;
			goto error;
	}

	if (riscv_enable_virtual && has_sufficient_progbuf(target, 5) && get_field(mstatus, MSTATUS_MPRV))
		riscv_program_csrrci(&program, GDB_REGNO_ZERO,  CSR_DCSR_MPRVEN, GDB_REGNO_DCSR);
	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);

	result = riscv_program_ebreak(&program);
	if (result != ERROR_OK)
		goto error;
	riscv_program_write(&program);

	riscv_addr_t cur_addr = address;
	riscv_addr_t fin_addr = address + (count * size);
	bool setup_needed = true;
	LOG_DEBUG("writing until final address 0x%016" PRIx64, fin_addr);
	while (cur_addr < fin_addr) {
		LOG_DEBUG("transferring burst starting at address 0x%016" PRIx64,
				cur_addr);

		struct riscv_batch *batch = riscv_batch_alloc(
				target,
				32,
				info->dmi_busy_delay + info->ac_busy_delay);
		if (!batch)
			goto error;

		/* To write another word, we put it in S1 and execute the program. */
		unsigned start = (cur_addr - address) / size;
		for (unsigned i = start; i < count; ++i) {
			unsigned offset = size*i;
			const uint8_t *t_buffer = buffer + offset;

			uint64_t value = buf_get_u64(t_buffer, 0, 8 * size);

			log_memory_access(address + offset, value, size, false);
			cur_addr += size;

			if (setup_needed) {
				result = register_write_direct(target, GDB_REGNO_S0,
						address + offset);
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Write value. */
				if (size > 4)
					dmi_write(target, DM_DATA1, value >> 32);
				dmi_write(target, DM_DATA0, value);

				/* Write and execute command that moves value into S1 and
				 * executes program buffer. */
				uint32_t command = access_register_command(target,
						GDB_REGNO_S1, riscv_xlen(target),
						AC_ACCESS_REGISTER_POSTEXEC |
						AC_ACCESS_REGISTER_TRANSFER |
						AC_ACCESS_REGISTER_WRITE);
				result = execute_abstract_command(target, command);
				if (result != ERROR_OK) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Turn on autoexec */
				dmi_write(target, DM_ABSTRACTAUTO,
						1 << DM_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

				setup_needed = false;
			} else {
				if (size > 4)
					riscv_batch_add_dmi_write(batch, DM_DATA1, value >> 32);
				riscv_batch_add_dmi_write(batch, DM_DATA0, value);
				if (riscv_batch_full(batch))
					break;
			}
		}

		result = batch_run(target, batch);
		riscv_batch_free(batch);
		if (result != ERROR_OK)
			goto error;

		/* Note that if the scan resulted in a Busy DMI response, it
		 * is this read to abstractcs that will cause the dmi_busy_delay
		 * to be incremented if necessary. */

		uint32_t abstractcs;
		bool dmi_busy_encountered;
		result = dmi_op(target, &abstractcs, &dmi_busy_encountered,
				DMI_OP_READ, DM_ABSTRACTCS, 0, false, true);
		if (result != ERROR_OK)
			goto error;
		while (get_field(abstractcs, DM_ABSTRACTCS_BUSY))
			if (dmi_read(target, &abstractcs, DM_ABSTRACTCS) != ERROR_OK)
				return ERROR_FAIL;
		info->cmderr = get_field(abstractcs, DM_ABSTRACTCS_CMDERR);
		if (info->cmderr == CMDERR_NONE && !dmi_busy_encountered) {
			LOG_DEBUG("successful (partial?) memory write");
		} else if (info->cmderr == CMDERR_BUSY || dmi_busy_encountered) {
			if (info->cmderr == CMDERR_BUSY)
				LOG_DEBUG("Memory write resulted in abstract command busy response.");
			else if (dmi_busy_encountered)
				LOG_DEBUG("Memory write resulted in DMI busy response.");
			riscv013_clear_abstract_error(target);
			increase_ac_busy_delay(target);

			dmi_write(target, DM_ABSTRACTAUTO, 0);
			result = register_read_direct(target, &cur_addr, GDB_REGNO_S0);
			if (result != ERROR_OK)
				goto error;
			setup_needed = true;
		} else {
			LOG_ERROR("error when writing memory, abstractcs=0x%08lx", (long)abstractcs);
			riscv013_clear_abstract_error(target);
			result = ERROR_FAIL;
			goto error;
		}
	}

error:
	dmi_write(target, DM_ABSTRACTAUTO, 0);

	if (register_write_direct(target, GDB_REGNO_S1, s1) != ERROR_OK)
		return ERROR_FAIL;
	if (register_write_direct(target, GDB_REGNO_S0, s0) != ERROR_OK)
		return ERROR_FAIL;

	/* Restore MSTATUS */
	if (mstatus != mstatus_old)
		if (register_write_direct(target, GDB_REGNO_MSTATUS, mstatus_old))
			return ERROR_FAIL;

	if (execute_fence(target) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);

	if (has_sufficient_progbuf(target, 3) && !riscv_prefer_sba)
		return write_memory_progbuf(target, address, size, count, buffer);

	if ((get_field(info->sbcs, DM_SBCS_SBACCESS8) && size == 1) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS16) && size == 2) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS32) && size == 4) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS64) && size == 8) ||
			(get_field(info->sbcs, DM_SBCS_SBACCESS128) && size == 16)) {
		if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 0)
			return write_memory_bus_v0(target, address, size, count, buffer);
		else if (get_field(info->sbcs, DM_SBCS_SBVERSION) == 1)
			return write_memory_bus_v1(target, address, size, count, buffer);
	}

	if (has_sufficient_progbuf(target, 3))
		return write_memory_progbuf(target, address, size, count, buffer);

	return write_memory_abstract(target, address, size, count, buffer);
}

static int arch_state(struct target *target)
{
	return ERROR_OK;
}

struct target_type riscv013_target = {
	.name = "riscv",

	.init_target = init_target,
	.deinit_target = deinit_target,
	.examine = examine,

	.poll = &riscv_openocd_poll,
	.halt = &riscv_halt,
	.step = &riscv_openocd_step,

	.assert_reset = assert_reset,
	.deassert_reset = deassert_reset,

	.write_memory = write_memory,

	.arch_state = arch_state,
};

/*** 0.13-specific implementations of various RISC-V helper functions. ***/
static int riscv013_get_register(struct target *target,
		riscv_reg_t *value, int hid, int rid)
{
	LOG_DEBUG("[%d] reading register %s on hart %d", target->coreid,
			gdb_regno_name(rid), hid);

	riscv_set_current_hartid(target, hid);

	int result = ERROR_OK;
	if (rid == GDB_REGNO_PC) {
		/* TODO: move this into riscv.c. */
		result = register_read(target, value, GDB_REGNO_DPC);
		LOG_DEBUG("[%d] read PC from DPC: 0x%" PRIx64, target->coreid, *value);
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		/* TODO: move this into riscv.c. */
		result = register_read(target, &dcsr, GDB_REGNO_DCSR);
		*value = get_field(dcsr, CSR_DCSR_PRV);
	} else {
		result = register_read(target, value, rid);
		if (result != ERROR_OK)
			*value = -1;
	}

	return result;
}

static int riscv013_set_register(struct target *target, int hid, int rid, uint64_t value)
{
	LOG_DEBUG("[%d] writing 0x%" PRIx64 " to register %s on hart %d",
			target->coreid, value, gdb_regno_name(rid), hid);

	riscv_set_current_hartid(target, hid);

	if (rid <= GDB_REGNO_XPR31) {
		return register_write_direct(target, rid, value);
	} else if (rid == GDB_REGNO_PC) {
		LOG_DEBUG("[%d] writing PC to DPC: 0x%" PRIx64, target->coreid, value);
		register_write_direct(target, GDB_REGNO_DPC, value);
		uint64_t actual_value;
		register_read_direct(target, &actual_value, GDB_REGNO_DPC);
		LOG_DEBUG("[%d]   actual DPC written: 0x%016" PRIx64, target->coreid, actual_value);
		if (value != actual_value) {
			LOG_ERROR("Written PC (0x%" PRIx64 ") does not match read back "
					"value (0x%" PRIx64 ")", value, actual_value);
			return ERROR_FAIL;
		}
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		register_read(target, &dcsr, GDB_REGNO_DCSR);
		dcsr = set_field(dcsr, CSR_DCSR_PRV, value);
		return register_write_direct(target, GDB_REGNO_DCSR, dcsr);
	} else {
		return register_write_direct(target, rid, value);
	}

	return ERROR_OK;
}

static int riscv013_select_current_hart(struct target *target)
{
	RISCV_INFO(r);

	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (r->current_hartid == dm->current_hartid)
		return ERROR_OK;

	uint32_t dmcontrol;
	/* TODO: can't we just "dmcontrol = DMI_DMACTIVE"? */
	if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
		return ERROR_FAIL;
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	int result = dmi_write(target, DM_DMCONTROL, dmcontrol);
	dm->current_hartid = r->current_hartid;
	return result;
}

/* Select all harts that were prepped and that are selectable, clearing the
 * prepped flag on the harts that actually were selected. */
static int select_prepped_harts(struct target *target, bool *use_hasel)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (!dm->hasel_supported) {
		RISCV_INFO(r);
		r->prepped = false;
		*use_hasel = false;
		return ERROR_OK;
	}

	assert(dm->hart_count);
	unsigned hawindow_count = (dm->hart_count + 31) / 32;
	uint32_t hawindow[hawindow_count];

	memset(hawindow, 0, sizeof(uint32_t) * hawindow_count);

	target_list_t *entry;
	unsigned total_selected = 0;
	list_for_each_entry(entry, &dm->target_list, list) {
		struct target *t = entry->target;
		riscv_info_t *r = riscv_info(t);
		riscv013_info_t *info = get_info(t);
		unsigned index = info->index;
		LOG_DEBUG("index=%d, coreid=%d, prepped=%d", index, t->coreid, r->prepped);
		r->selected = r->prepped;
		if (r->prepped) {
			hawindow[index / 32] |= 1 << (index % 32);
			r->prepped = false;
			total_selected++;
		}
		index++;
	}

	/* Don't use hasel if we only need to talk to one hart. */
	if (total_selected <= 1) {
		*use_hasel = false;
		return ERROR_OK;
	}

	for (unsigned i = 0; i < hawindow_count; i++) {
		if (dmi_write(target, DM_HAWINDOWSEL, i) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_write(target, DM_HAWINDOW, hawindow[i]) != ERROR_OK)
			return ERROR_FAIL;
	}

	*use_hasel = true;
	return ERROR_OK;
}

static int riscv013_halt_prep(struct target *target)
{
	return ERROR_OK;
}

static int riscv013_halt_go(struct target *target)
{
	bool use_hasel = false;
	if (!riscv_rtos_enabled(target)) {
		if (select_prepped_harts(target, &use_hasel) != ERROR_OK)
			return ERROR_FAIL;
	}

	RISCV_INFO(r);
	LOG_DEBUG("halting hart %d", r->current_hartid);

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_HALTREQ;
	if (use_hasel)
		dmcontrol |= DM_DMCONTROL_HASEL;
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	dmi_write(target, DM_DMCONTROL, dmcontrol);
	for (size_t i = 0; i < 256; ++i)
		if (riscv_is_halted(target))
			break;

	if (!riscv_is_halted(target)) {
		uint32_t dmstatus;
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		if (dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
			return ERROR_FAIL;

		LOG_ERROR("unable to halt hart %d", r->current_hartid);
		LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
		LOG_ERROR("  dmstatus =0x%08x", dmstatus);
		return ERROR_FAIL;
	}

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HALTREQ, 0);
	dmi_write(target, DM_DMCONTROL, dmcontrol);

	if (use_hasel) {
		target_list_t *entry;
		dm013_info_t *dm = get_dm(target);
		if (!dm)
			return ERROR_FAIL;
		list_for_each_entry(entry, &dm->target_list, list) {
			struct target *t = entry->target;
			t->state = TARGET_HALTED;
			if (t->debug_reason == DBG_REASON_NOTHALTED)
				t->debug_reason = DBG_REASON_DBGRQ;
		}
	}
	/* The "else" case is handled in halt_go(). */

	return ERROR_OK;
}

static int riscv013_resume_go(struct target *target)
{
	bool use_hasel = false;
	if (!riscv_rtos_enabled(target)) {
		if (select_prepped_harts(target, &use_hasel) != ERROR_OK)
			return ERROR_FAIL;
	}

	return riscv013_step_or_resume_current_hart(target, false, use_hasel);
}

static int riscv013_step_current_hart(struct target *target)
{
	return riscv013_step_or_resume_current_hart(target, true, false);
}

static int riscv013_resume_prep(struct target *target)
{
	return riscv013_on_step_or_resume(target, false);
}

static int riscv013_on_step(struct target *target)
{
	return riscv013_on_step_or_resume(target, true);
}

static int riscv013_on_halt(struct target *target)
{
	return ERROR_OK;
}

static bool riscv013_is_halted(struct target *target)
{
	uint32_t dmstatus;
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return false;
	if (get_field(dmstatus, DM_DMSTATUS_ANYUNAVAIL))
		LOG_ERROR("Hart %d is unavailable.", riscv_current_hartid(target));
	if (get_field(dmstatus, DM_DMSTATUS_ANYNONEXISTENT))
		LOG_ERROR("Hart %d doesn't exist.", riscv_current_hartid(target));
	if (get_field(dmstatus, DM_DMSTATUS_ANYHAVERESET)) {
		int hartid = riscv_current_hartid(target);
		LOG_INFO("Hart %d unexpectedly reset!", hartid);
		/* TODO: Can we make this more obvious to eg. a gdb user? */
		uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE |
			DM_DMCONTROL_ACKHAVERESET;
		dmcontrol = set_hartsel(dmcontrol, hartid);
		/* If we had been halted when we reset, request another halt. If we
		 * ended up running out of reset, then the user will (hopefully) get a
		 * message that a reset happened, that the target is running, and then
		 * that it is halted again once the request goes through.
		 */
		if (target->state == TARGET_HALTED)
			dmcontrol |= DM_DMCONTROL_HALTREQ;
		dmi_write(target, DM_DMCONTROL, dmcontrol);
	}
	return get_field(dmstatus, DM_DMSTATUS_ALLHALTED);
}

static enum riscv_halt_reason riscv013_halt_reason(struct target *target)
{
	riscv_reg_t dcsr;
	int result = register_read(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return RISCV_HALT_UNKNOWN;

	switch (get_field(dcsr, CSR_DCSR_CAUSE)) {
	case CSR_DCSR_CAUSE_SWBP:
		return RISCV_HALT_BREAKPOINT;
	case CSR_DCSR_CAUSE_TRIGGER:
		/* We could get here before triggers are enumerated if a trigger was
		 * already set when we connected. Force enumeration now, which has the
		 * side effect of clearing any triggers we did not set. */
		riscv_enumerate_triggers(target);
		LOG_DEBUG("{%d} halted because of trigger", target->coreid);
		return RISCV_HALT_TRIGGER;
	case CSR_DCSR_CAUSE_STEP:
		return RISCV_HALT_SINGLESTEP;
	case CSR_DCSR_CAUSE_DEBUGINT:
	case CSR_DCSR_CAUSE_HALT:
		return RISCV_HALT_INTERRUPT;
	case CSR_DCSR_CAUSE_GROUP:
		return RISCV_HALT_GROUP;
	}

	LOG_ERROR("Unknown DCSR cause field: %x", (int)get_field(dcsr, CSR_DCSR_CAUSE));
	LOG_ERROR("  dcsr=0x%016lx", (long)dcsr);
	return RISCV_HALT_UNKNOWN;
}

int riscv013_write_debug_buffer(struct target *target, unsigned index, riscv_insn_t data)
{
	dm013_info_t *dm = get_dm(target);
	if (!dm)
		return ERROR_FAIL;
	if (dm->progbuf_cache[index] != data) {
		if (dmi_write(target, DM_PROGBUF0 + index, data) != ERROR_OK)
			return ERROR_FAIL;
		dm->progbuf_cache[index] = data;
	} else {
		LOG_DEBUG("cache hit for 0x%" PRIx32 " @%d", data, index);
	}
	return ERROR_OK;
}

riscv_insn_t riscv013_read_debug_buffer(struct target *target, unsigned index)
{
	uint32_t value;
	dmi_read(target, &value, DM_PROGBUF0 + index);
	return value;
}

int riscv013_execute_debug_buffer(struct target *target)
{
	uint32_t run_program = 0;
	run_program = set_field(run_program, AC_ACCESS_REGISTER_AARSIZE, 2);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_POSTEXEC, 1);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_TRANSFER, 0);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_REGNO, 0x1000);

	return execute_abstract_command(target, run_program);
}

void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_WRITE);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, d);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_READ);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_nop_u64(struct target *target, char *buf)
{
	RISCV013_INFO(info);
	buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_NOP);
	buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, 0);
}

/* Helper function for riscv013_test_sba_config_reg */
static int get_max_sbaccess(struct target *target)
{
	RISCV013_INFO(info);

	uint32_t sbaccess128 = get_field(info->sbcs, DM_SBCS_SBACCESS128);
	uint32_t sbaccess64 = get_field(info->sbcs, DM_SBCS_SBACCESS64);
	uint32_t sbaccess32 = get_field(info->sbcs, DM_SBCS_SBACCESS32);
	uint32_t sbaccess16 = get_field(info->sbcs, DM_SBCS_SBACCESS16);
	uint32_t sbaccess8 = get_field(info->sbcs, DM_SBCS_SBACCESS8);

	if (sbaccess128)
		return 4;
	else if (sbaccess64)
		return 3;
	else if (sbaccess32)
		return 2;
	else if (sbaccess16)
		return 1;
	else if (sbaccess8)
		return 0;
	else
		return -1;
}

static uint32_t get_num_sbdata_regs(struct target *target)
{
	RISCV013_INFO(info);

	uint32_t sbaccess128 = get_field(info->sbcs, DM_SBCS_SBACCESS128);
	uint32_t sbaccess64 = get_field(info->sbcs, DM_SBCS_SBACCESS64);
	uint32_t sbaccess32 = get_field(info->sbcs, DM_SBCS_SBACCESS32);

	if (sbaccess128)
		return 4;
	else if (sbaccess64)
		return 2;
	else if (sbaccess32)
		return 1;
	else
		return 0;
}

static int riscv013_test_sba_config_reg(struct target *target,
		target_addr_t legal_address, uint32_t num_words,
		target_addr_t illegal_address, bool run_sbbusyerror_test)
{
	LOG_INFO("Testing System Bus Access as defined by RISC-V Debug Spec v0.13");

	uint32_t tests_failed = 0;

	uint32_t rd_val;
	uint32_t sbcs_orig;
	dmi_read(target, &sbcs_orig, DM_SBCS);

	uint32_t sbcs = sbcs_orig;
	bool test_passed;

	int max_sbaccess = get_max_sbaccess(target);

	if (max_sbaccess == -1) {
		LOG_ERROR("System Bus Access not supported in this config.");
		return ERROR_FAIL;
	}

	if (get_field(sbcs, DM_SBCS_SBVERSION) != 1) {
		LOG_ERROR("System Bus Access unsupported SBVERSION (%d). Only version 1 is supported.",
				get_field(sbcs, DM_SBCS_SBVERSION));
		return ERROR_FAIL;
	}

	uint32_t num_sbdata_regs = get_num_sbdata_regs(target);
	assert(num_sbdata_regs);

	uint32_t rd_buf[num_sbdata_regs];

	/* Test 1: Simple write/read test */
	test_passed = true;
	sbcs = set_field(sbcs_orig, DM_SBCS_SBAUTOINCREMENT, 0);
	dmi_write(target, DM_SBCS, sbcs);

	uint32_t test_patterns[4] = {0xdeadbeef, 0xfeedbabe, 0x12345678, 0x08675309};
	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)max_sbaccess; sbaccess++) {
		sbcs = set_field(sbcs, DM_SBCS_SBACCESS, sbaccess);
		dmi_write(target, DM_SBCS, sbcs);

		uint32_t compare_mask = (sbaccess == 0) ? 0xff : (sbaccess == 1) ? 0xffff : 0xffffffff;

		for (uint32_t i = 0; i < num_words; i++) {
			uint32_t addr = legal_address + (i << sbaccess);
			uint32_t wr_data[num_sbdata_regs];
			for (uint32_t j = 0; j < num_sbdata_regs; j++)
				wr_data[j] = test_patterns[j] + i;
			write_memory_sba_simple(target, addr, wr_data, num_sbdata_regs, sbcs);
		}

		for (uint32_t i = 0; i < num_words; i++) {
			uint32_t addr = legal_address + (i << sbaccess);
			read_memory_sba_simple(target, addr, rd_buf, num_sbdata_regs, sbcs);
			for (uint32_t j = 0; j < num_sbdata_regs; j++) {
				if (((test_patterns[j]+i)&compare_mask) != (rd_buf[j]&compare_mask)) {
					LOG_ERROR("System Bus Access Test 1: Error reading non-autoincremented address %x,"
							"expected val = %x, read val = %x", addr, test_patterns[j]+i, rd_buf[j]);
					test_passed = false;
					tests_failed++;
				}
			}
		}
	}
	if (test_passed)
		LOG_INFO("System Bus Access Test 1: Simple write/read test PASSED.");

	/* Test 2: Address autoincrement test */
	target_addr_t curr_addr;
	target_addr_t prev_addr;
	test_passed = true;
	sbcs = set_field(sbcs_orig, DM_SBCS_SBAUTOINCREMENT, 1);
	dmi_write(target, DM_SBCS, sbcs);

	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)max_sbaccess; sbaccess++) {
		sbcs = set_field(sbcs, DM_SBCS_SBACCESS, sbaccess);
		dmi_write(target, DM_SBCS, sbcs);

		dmi_write(target, DM_SBADDRESS0, legal_address);
		read_sbcs_nonbusy(target, &sbcs);
		curr_addr = legal_address;
		for (uint32_t i = 0; i < num_words; i++) {
			prev_addr = curr_addr;
			read_sbcs_nonbusy(target, &sbcs);
			curr_addr = sb_read_address(target);
			if ((curr_addr - prev_addr != (uint32_t)(1 << sbaccess)) && (i != 0)) {
				LOG_ERROR("System Bus Access Test 2: Error with address auto-increment, sbaccess = %x.", sbaccess);
				test_passed = false;
				tests_failed++;
			}
			dmi_write(target, DM_SBDATA0, i);
		}

		read_sbcs_nonbusy(target, &sbcs);

		dmi_write(target, DM_SBADDRESS0, legal_address);

		uint32_t val;
		sbcs = set_field(sbcs, DM_SBCS_SBREADONDATA, 1);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &val, DM_SBDATA0); /* Dummy read to trigger first system bus read */
		curr_addr = legal_address;
		for (uint32_t i = 0; i < num_words; i++) {
			prev_addr = curr_addr;
			read_sbcs_nonbusy(target, &sbcs);
			curr_addr = sb_read_address(target);
			if ((curr_addr - prev_addr != (uint32_t)(1 << sbaccess)) && (i != 0)) {
				LOG_ERROR("System Bus Access Test 2: Error with address auto-increment, sbaccess = %x", sbaccess);
				test_passed = false;
				tests_failed++;
			}
			dmi_read(target, &val, DM_SBDATA0);
			read_sbcs_nonbusy(target, &sbcs);
			if (i != val) {
				LOG_ERROR("System Bus Access Test 2: Error reading auto-incremented address,"
						"expected val = %x, read val = %x.", i, val);
				test_passed = false;
				tests_failed++;
			}
		}
	}
	if (test_passed)
		LOG_INFO("System Bus Access Test 2: Address auto-increment test PASSED.");

	/* Test 3: Read from illegal address */
	read_memory_sba_simple(target, illegal_address, rd_buf, 1, sbcs_orig);

	dmi_read(target, &rd_val, DM_SBCS);
	if (get_field(rd_val, DM_SBCS_SBERROR) == 2) {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 2);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
			LOG_INFO("System Bus Access Test 3: Illegal address read test PASSED.");
		else
			LOG_ERROR("System Bus Access Test 3: Illegal address read test FAILED, unable to clear to 0.");
	} else {
		LOG_ERROR("System Bus Access Test 3: Illegal address read test FAILED, unable to set error code.");
	}

	/* Test 4: Write to illegal address */
	write_memory_sba_simple(target, illegal_address, test_patterns, 1, sbcs_orig);

	dmi_read(target, &rd_val, DM_SBCS);
	if (get_field(rd_val, DM_SBCS_SBERROR) == 2) {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 2);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
			LOG_INFO("System Bus Access Test 4: Illegal address write test PASSED.");
		else {
			LOG_ERROR("System Bus Access Test 4: Illegal address write test FAILED, unable to clear to 0.");
			tests_failed++;
		}
	} else {
		LOG_ERROR("System Bus Access Test 4: Illegal address write test FAILED, unable to set error code.");
		tests_failed++;
	}

	/* Test 5: Write with unsupported sbaccess size */
	uint32_t sbaccess128 = get_field(sbcs_orig, DM_SBCS_SBACCESS128);

	if (sbaccess128) {
		LOG_INFO("System Bus Access Test 5: SBCS sbaccess error test PASSED, all sbaccess sizes supported.");
	} else {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBACCESS, 4);

		write_memory_sba_simple(target, legal_address, test_patterns, 1, sbcs);

		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 4) {
			sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 4);
			dmi_write(target, DM_SBCS, sbcs);
			dmi_read(target, &rd_val, DM_SBCS);
			if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
				LOG_INFO("System Bus Access Test 5: SBCS sbaccess error test PASSED.");
			else {
				LOG_ERROR("System Bus Access Test 5: SBCS sbaccess error test FAILED, unable to clear to 0.");
				tests_failed++;
			}
		} else {
			LOG_ERROR("System Bus Access Test 5: SBCS sbaccess error test FAILED, unable to set error code.");
			tests_failed++;
		}
	}

	/* Test 6: Write to misaligned address */
	sbcs = set_field(sbcs_orig, DM_SBCS_SBACCESS, 1);

	write_memory_sba_simple(target, legal_address+1, test_patterns, 1, sbcs);

	dmi_read(target, &rd_val, DM_SBCS);
	if (get_field(rd_val, DM_SBCS_SBERROR) == 3) {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBERROR, 3);
		dmi_write(target, DM_SBCS, sbcs);
		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBERROR) == 0)
			LOG_INFO("System Bus Access Test 6: SBCS address alignment error test PASSED");
		else {
			LOG_ERROR("System Bus Access Test 6: SBCS address alignment error test FAILED, unable to clear to 0.");
			tests_failed++;
		}
	} else {
		LOG_ERROR("System Bus Access Test 6: SBCS address alignment error test FAILED, unable to set error code.");
		tests_failed++;
	}

	/* Test 7: Set sbbusyerror, only run this case in simulation as it is likely
	 * impossible to hit otherwise */
	if (run_sbbusyerror_test) {
		sbcs = set_field(sbcs_orig, DM_SBCS_SBREADONADDR, 1);
		dmi_write(target, DM_SBCS, sbcs);

		for (int i = 0; i < 16; i++)
			dmi_write(target, DM_SBDATA0, 0xdeadbeef);

		for (int i = 0; i < 16; i++)
			dmi_write(target, DM_SBADDRESS0, legal_address);

		dmi_read(target, &rd_val, DM_SBCS);
		if (get_field(rd_val, DM_SBCS_SBBUSYERROR)) {
			sbcs = set_field(sbcs_orig, DM_SBCS_SBBUSYERROR, 1);
			dmi_write(target, DM_SBCS, sbcs);
			dmi_read(target, &rd_val, DM_SBCS);
			if (get_field(rd_val, DM_SBCS_SBBUSYERROR) == 0)
				LOG_INFO("System Bus Access Test 7: SBCS sbbusyerror test PASSED.");
			else {
				LOG_ERROR("System Bus Access Test 7: SBCS sbbusyerror test FAILED, unable to clear to 0.");
				tests_failed++;
			}
		} else {
			LOG_ERROR("System Bus Access Test 7: SBCS sbbusyerror test FAILED, unable to set error code.");
			tests_failed++;
		}
	}

	if (tests_failed == 0) {
		LOG_INFO("ALL TESTS PASSED");
		return ERROR_OK;
	} else {
		LOG_ERROR("%d TESTS FAILED", tests_failed);
		return ERROR_FAIL;
	}

}

void write_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *write_data, uint32_t write_size, uint32_t sbcs)
{
	RISCV013_INFO(info);

	uint32_t rd_sbcs;
	uint32_t masked_addr;

	uint32_t sba_size = get_field(info->sbcs, DM_SBCS_SBASIZE);

	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_no_readonaddr = set_field(sbcs, DM_SBCS_SBREADONADDR, 0);
	dmi_write(target, DM_SBCS, sbcs_no_readonaddr);

	for (uint32_t i = 0; i < sba_size/32; i++) {
		masked_addr = (addr >> 32*i) & 0xffffffff;

		if (i != 3)
			dmi_write(target, DM_SBADDRESS0+i, masked_addr);
		else
			dmi_write(target, DM_SBADDRESS3, masked_addr);
	}

	/* Write SBDATA registers starting with highest address, since write to
	 * SBDATA0 triggers write */
	for (int i = write_size-1; i >= 0; i--)
		dmi_write(target, DM_SBDATA0+i, write_data[i]);
}

void read_memory_sba_simple(struct target *target, target_addr_t addr,
		uint32_t *rd_buf, uint32_t read_size, uint32_t sbcs)
{
	RISCV013_INFO(info);

	uint32_t rd_sbcs;
	uint32_t masked_addr;

	uint32_t sba_size = get_field(info->sbcs, DM_SBCS_SBASIZE);

	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_readonaddr = set_field(sbcs, DM_SBCS_SBREADONADDR, 1);
	dmi_write(target, DM_SBCS, sbcs_readonaddr);

	/* Write addresses starting with highest address register */
	for (int i = sba_size/32-1; i >= 0; i--) {
		masked_addr = (addr >> 32*i) & 0xffffffff;

		if (i != 3)
			dmi_write(target, DM_SBADDRESS0+i, masked_addr);
		else
			dmi_write(target, DM_SBADDRESS3, masked_addr);
	}

	read_sbcs_nonbusy(target, &rd_sbcs);

	for (uint32_t i = 0; i < read_size; i++)
		dmi_read(target, &(rd_buf[i]), DM_SBDATA0+i);
}

int riscv013_dmi_write_u64_bits(struct target *target)
{
	RISCV013_INFO(info);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

static int maybe_execute_fence_i(struct target *target)
{
	if (has_sufficient_progbuf(target, 3))
		return execute_fence(target);
	return ERROR_OK;
}

/* Helper Functions. */
static int riscv013_on_step_or_resume(struct target *target, bool step)
{
	if (maybe_execute_fence_i(target) != ERROR_OK)
		return ERROR_FAIL;

	/* We want to twiddle some bits in the debug CSR so debugging works. */
	riscv_reg_t dcsr;
	int result = register_read(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return result;
	dcsr = set_field(dcsr, CSR_DCSR_STEP, step);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, riscv_ebreakm);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, riscv_ebreaks);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, riscv_ebreaku);
	return riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
}

static int riscv013_step_or_resume_current_hart(struct target *target,
		bool step, bool use_hasel)
{
	RISCV_INFO(r);
	LOG_DEBUG("resuming hart %d (for step?=%d)", r->current_hartid, step);
	if (!riscv_is_halted(target)) {
		LOG_ERROR("Hart %d is not halted!", r->current_hartid);
		return ERROR_FAIL;
	}

	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_RESUMEREQ;
	if (use_hasel)
		dmcontrol |= DM_DMCONTROL_HASEL;
	dmcontrol = set_hartsel(dmcontrol, r->current_hartid);
	dmi_write(target, DM_DMCONTROL, dmcontrol);

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HASEL, 0);
	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_RESUMEREQ, 0);

	uint32_t dmstatus;
	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
			return ERROR_FAIL;
		if (get_field(dmstatus, DM_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		if (step && get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0)
			continue;

		dmi_write(target, DM_DMCONTROL, dmcontrol);
		return ERROR_OK;
	}

	dmi_write(target, DM_DMCONTROL, dmcontrol);

	LOG_ERROR("unable to resume hart %d", r->current_hartid);
	if (dmstatus_read(target, &dmstatus, true) != ERROR_OK)
		return ERROR_FAIL;
	LOG_ERROR("  dmstatus =0x%08x", dmstatus);

	if (step) {
		LOG_ERROR("  was stepping, halting");
		riscv_halt(target);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

void riscv013_clear_abstract_error(struct target *target)
{
	/* Wait for busy to go away. */
	time_t start = time(NULL);
	uint32_t abstractcs;
	dmi_read(target, &abstractcs, DM_ABSTRACTCS);
	while (get_field(abstractcs, DM_ABSTRACTCS_BUSY)) {
		dmi_read(target, &abstractcs, DM_ABSTRACTCS);

		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("abstractcs.busy is not going low after %d seconds "
					"(abstractcs=0x%x). The target is either really slow or "
					"broken. You could increase the timeout with riscv "
					"set_command_timeout_sec.",
					riscv_command_timeout_sec, abstractcs);
			break;
		}
	}
	/* Clear the error status. */
	dmi_write(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
}

#ifdef _WIN32
#define FILE_SEP '\\'
#else
#define FILE_SEP '/'
#endif
#define COMPLIANCE_TEST(b, message) \
{ \
	const char *last_sep = strrchr(__FILE__, FILE_SEP); \
	const char *fname = (last_sep == NULL ? __FILE__ : last_sep + 1); \
	LOG_INFO("Executing test %d (%s:%d): %s", total_tests, fname, __LINE__, message); \
	int pass = 0;		    \
	if (b) {		    \
		pass = 1;	    \
		passed_tests++;     \
	}			    \
	LOG_INFO("  %s", (pass) ? "PASSED" : "FAILED"); \
	assert(pass);		    \
	total_tests++;		    \
}

#define COMPLIANCE_MUST_PASS(b) COMPLIANCE_TEST(ERROR_OK == (b), "Regular calls must return ERROR_OK")

#define COMPLIANCE_READ(target, addr, value) COMPLIANCE_MUST_PASS(dmi_read(target, addr, value))
#define COMPLIANCE_WRITE(target, addr, value) COMPLIANCE_MUST_PASS(dmi_write(target, addr, value))

#define COMPLIANCE_CHECK_RO(target, addr)                               \
{                                                                       \
	uint32_t orig;                                                      \
	uint32_t inverse;                                                   \
	COMPLIANCE_READ(target, &orig, addr);                               \
	COMPLIANCE_WRITE(target, addr, ~orig);                              \
	COMPLIANCE_READ(target, &inverse, addr);                            \
	COMPLIANCE_TEST(orig == inverse, "Register must be read-only");     \
}

int riscv013_test_compliance(struct target *target)
{
	LOG_INFO("Basic compliance test against RISC-V Debug Spec v0.13");
	LOG_INFO("This test is not complete, and not well supported.");
	LOG_INFO("Your core might pass this test without being compliant.");
	LOG_INFO("Your core might fail this test while being compliant.");
	LOG_INFO("Use your judgment, and please contribute improvements.");

	if (!riscv_rtos_enabled(target)) {
		LOG_ERROR("Please run with -rtos riscv to run compliance test.");
		return ERROR_FAIL;
	}

	if (!target_was_examined(target)) {
		LOG_ERROR("Cannot run compliance test, because target has not yet "
			"been examined, or the examination failed.\n");
		return ERROR_FAIL;
	}

	int total_tests = 0;
	int passed_tests = 0;

	uint32_t dmcontrol_orig = DM_DMCONTROL_DMACTIVE;
	uint32_t dmcontrol;
	uint32_t testvar;
	uint32_t testvar_read;
	riscv_reg_t value;
	RISCV013_INFO(info);

	/* All the bits of HARTSEL are covered by the examine sequence. */

	/* hartreset */
	/* This field is optional. Either we can read and write it to 1/0,
	or it is tied to 0. This check doesn't really do anything, but
	it does attempt to set the bit to 1 and then back to 0, which needs to
	work if its implemented. */
	COMPLIANCE_WRITE(target, DM_DMCONTROL, set_field(dmcontrol_orig, DM_DMCONTROL_HARTRESET, 1));
	COMPLIANCE_WRITE(target, DM_DMCONTROL, set_field(dmcontrol_orig, DM_DMCONTROL_HARTRESET, 0));
	COMPLIANCE_READ(target, &dmcontrol, DM_DMCONTROL);
	COMPLIANCE_TEST((get_field(dmcontrol, DM_DMCONTROL_HARTRESET) == 0),
			"DMCONTROL.hartreset can be 0 or RW.");

	/* hasel */
	COMPLIANCE_WRITE(target, DM_DMCONTROL, set_field(dmcontrol_orig, DM_DMCONTROL_HASEL, 1));
	COMPLIANCE_WRITE(target, DM_DMCONTROL, set_field(dmcontrol_orig, DM_DMCONTROL_HASEL, 0));
	COMPLIANCE_READ(target, &dmcontrol, DM_DMCONTROL);
	COMPLIANCE_TEST((get_field(dmcontrol, DM_DMCONTROL_HASEL) == 0),
			"DMCONTROL.hasel can be 0 or RW.");
	/* TODO: test that hamask registers exist if hasel does. */

	/* haltreq */
	COMPLIANCE_MUST_PASS(riscv_halt(target));
	/* This bit is not actually readable according to the spec, so nothing to check.*/

	/* DMSTATUS */
	COMPLIANCE_CHECK_RO(target, DM_DMSTATUS);

	/* resumereq */
	/* This bit is not actually readable according to the spec, so nothing to check.*/
	COMPLIANCE_MUST_PASS(riscv_resume(target, true, 0, false, false, false));

	/* Halt all harts again so the test can continue.*/
	COMPLIANCE_MUST_PASS(riscv_halt(target));

	/* HARTINFO: Read-Only. This is per-hart, so need to adjust hartsel. */
	uint32_t hartinfo;
	COMPLIANCE_READ(target, &hartinfo, DM_HARTINFO);
	for (int hartsel = 0; hartsel < riscv_count_harts(target); hartsel++) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));

		COMPLIANCE_CHECK_RO(target, DM_HARTINFO);

		/* $dscratch CSRs */
		uint32_t nscratch = get_field(hartinfo, DM_HARTINFO_NSCRATCH);
		for (unsigned int d = 0; d < nscratch; d++) {
			riscv_reg_t testval, testval_read;
			/* Because DSCRATCH0 is not guaranteed to last across PB executions, need to put
			this all into one PB execution. Which may not be possible on all implementations.*/
			if (info->progbufsize >= 5) {
				for (testval = 0x0011223300112233;
						 testval != 0xDEAD;
						 testval = testval == 0x0011223300112233 ? ~testval : 0xDEAD) {
					COMPLIANCE_TEST(register_write_direct(target, GDB_REGNO_S0, testval) == ERROR_OK,
							"Need to be able to write S0 in order to test DSCRATCH0.");
					struct riscv_program program32;
					riscv_program_init(&program32, target);
					riscv_program_csrw(&program32, GDB_REGNO_S0, GDB_REGNO_DSCRATCH0 + d);
					riscv_program_csrr(&program32, GDB_REGNO_S1, GDB_REGNO_DSCRATCH0 + d);
					riscv_program_fence(&program32);
					riscv_program_ebreak(&program32);
					COMPLIANCE_TEST(riscv_program_exec(&program32, target) == ERROR_OK,
							"Accessing DSCRATCH0 with program buffer should succeed.");
					COMPLIANCE_TEST(register_read_direct(target, &testval_read, GDB_REGNO_S1) == ERROR_OK,
							"Need to be able to read S1 in order to test DSCRATCH0.");
					if (riscv_xlen(target) > 32) {
						COMPLIANCE_TEST(testval == testval_read,
								"All DSCRATCH0 registers in HARTINFO must be R/W.");
					} else {
						COMPLIANCE_TEST(testval_read == (testval & 0xFFFFFFFF),
								"All DSCRATCH0 registers in HARTINFO must be R/W.");
					}
				}
			}
		}
		/* TODO: dataaccess */
		if (get_field(hartinfo, DM_HARTINFO_DATAACCESS)) {
			/* TODO: Shadowed in memory map. */
			/* TODO: datasize */
			/* TODO: dataaddr */
		} else {
			/* TODO: Shadowed in CSRs. */
			/* TODO: datasize */
			/* TODO: dataaddr */
		}

	}

	/* HALTSUM -- TODO: More than 32 harts. Would need to loop over this to set hartsel */
	/* TODO: HALTSUM2, HALTSUM3 */
	/* HALTSUM0 */
	uint32_t expected_haltsum0 = 0;
	for (int i = 0; i < MIN(riscv_count_harts(target), 32); i++)
		expected_haltsum0 |= (1 << i);

	COMPLIANCE_READ(target, &testvar_read, DM_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0,
			"HALTSUM0 should report summary of up to 32 halted harts");

	COMPLIANCE_WRITE(target, DM_HALTSUM0, 0xffffffff);
	COMPLIANCE_READ(target, &testvar_read, DM_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0, "HALTSUM0 should be R/O");

	COMPLIANCE_WRITE(target, DM_HALTSUM0, 0x0);
	COMPLIANCE_READ(target, &testvar_read, DM_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0, "HALTSUM0 should be R/O");

	/* HALTSUM1 */
	uint32_t expected_haltsum1 = 0;
	for (int i = 0; i < MIN(riscv_count_harts(target), 1024); i += 32)
		expected_haltsum1 |= (1 << (i/32));

	COMPLIANCE_READ(target, &testvar_read, DM_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1,
			"HALTSUM1 should report summary of up to 1024 halted harts");

	COMPLIANCE_WRITE(target, DM_HALTSUM1, 0xffffffff);
	COMPLIANCE_READ(target, &testvar_read, DM_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1, "HALTSUM1 should be R/O");

	COMPLIANCE_WRITE(target, DM_HALTSUM1, 0x0);
	COMPLIANCE_READ(target, &testvar_read, DM_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1, "HALTSUM1 should be R/O");

	/* TODO: HAWINDOWSEL */

	/* TODO: HAWINDOW */

	/* ABSTRACTCS */

	uint32_t abstractcs;
	COMPLIANCE_READ(target, &abstractcs, DM_ABSTRACTCS);

	/* Check that all reported Data Words are really R/W */
	for (int invert = 0; invert < 2; invert++) {
		for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_WRITE(target, DM_DATA0 + i, testvar);
		}
		for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_READ(target, &testvar_read, DM_DATA0 + i);
			COMPLIANCE_TEST(testvar_read == testvar, "All reported DATA words must be R/W");
		}
	}

	/* Check that all reported ProgBuf words are really R/W */
	for (int invert = 0; invert < 2; invert++) {
		for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_WRITE(target, DM_PROGBUF0 + i, testvar);
		}
		for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE); i++) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_READ(target, &testvar_read, DM_PROGBUF0 + i);
			COMPLIANCE_TEST(testvar_read == testvar, "All reported PROGBUF words must be R/W");
		}
	}

	/* TODO: Cause and clear all error types */

	/* COMMAND
	According to the spec, this register is only W, so can't really check the read result.
	But at any rate, this is not legal and should cause an error. */
	COMPLIANCE_WRITE(target, DM_COMMAND, 0xAAAAAAAA);
	COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DM_ABSTRACTCS_CMDERR) == CMDERR_NOT_SUPPORTED,
			"Illegal COMMAND should result in UNSUPPORTED");
	COMPLIANCE_WRITE(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);

	COMPLIANCE_WRITE(target, DM_COMMAND, 0x55555555);
	COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DM_ABSTRACTCS_CMDERR) == CMDERR_NOT_SUPPORTED,
			"Illegal COMMAND should result in UNSUPPORTED");
	COMPLIANCE_WRITE(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);

	/* Basic Abstract Commands */
	for (unsigned int i = 1; i < 32; i = i << 1) {
		riscv_reg_t testval =	i | ((i + 1ULL) << 32);
		riscv_reg_t testval_read;
		COMPLIANCE_TEST(ERROR_OK == register_write_direct(target, GDB_REGNO_ZERO + i, testval),
				"GPR Writes should be supported.");
		COMPLIANCE_MUST_PASS(write_abstract_arg(target, 0, 0xDEADBEEFDEADBEEF, 64));
		COMPLIANCE_TEST(ERROR_OK == register_read_direct(target, &testval_read, GDB_REGNO_ZERO + i),
				"GPR Reads should be supported.");
		if (riscv_xlen(target) > 32) {
			/* Dummy comment to satisfy linter, since removing the branches here doesn't actually compile. */
			COMPLIANCE_TEST(testval == testval_read, "GPR Reads and writes should be supported.");
		} else {
			/* Dummy comment to satisfy linter, since removing the branches here doesn't actually compile. */
			COMPLIANCE_TEST((testval & 0xFFFFFFFF) == testval_read, "GPR Reads and writes should be supported.");
		}
	}

	/* ABSTRACTAUTO
	See which bits are actually writable */
	COMPLIANCE_WRITE(target, DM_ABSTRACTAUTO, 0xFFFFFFFF);
	uint32_t abstractauto;
	uint32_t busy;
	COMPLIANCE_READ(target, &abstractauto, DM_ABSTRACTAUTO);
	COMPLIANCE_WRITE(target, DM_ABSTRACTAUTO, 0x0);
	if (abstractauto > 0) {
		/* This mechanism only works when you have a reasonable sized progbuf, which is not
		a true compliance requirement. */
		if (info->progbufsize >= 3) {

			testvar = 0;
			COMPLIANCE_TEST(ERROR_OK == register_write_direct(target, GDB_REGNO_S0, 0),
					"Need to be able to write S0 to test ABSTRACTAUTO");
			struct riscv_program program;
			COMPLIANCE_MUST_PASS(riscv_program_init(&program, target));
			/* This is also testing that WFI() is a NOP during debug mode. */
			COMPLIANCE_MUST_PASS(riscv_program_insert(&program, wfi()));
			COMPLIANCE_MUST_PASS(riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, 1));
			COMPLIANCE_MUST_PASS(riscv_program_ebreak(&program));
			COMPLIANCE_WRITE(target, DM_ABSTRACTAUTO, 0x0);
			COMPLIANCE_MUST_PASS(riscv_program_exec(&program, target));
			testvar++;
			COMPLIANCE_WRITE(target, DM_ABSTRACTAUTO, 0xFFFFFFFF);
			COMPLIANCE_READ(target, &abstractauto, DM_ABSTRACTAUTO);
			uint32_t autoexec_data = get_field(abstractauto, DM_ABSTRACTAUTO_AUTOEXECDATA);
			uint32_t autoexec_progbuf = get_field(abstractauto, DM_ABSTRACTAUTO_AUTOEXECPROGBUF);
			for (unsigned int i = 0; i < 12; i++) {
				COMPLIANCE_READ(target, &testvar_read, DM_DATA0 + i);
				do {
					COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTCS);
					busy = get_field(testvar_read, DM_ABSTRACTCS_BUSY);
				} while (busy);
				if (autoexec_data & (1 << i)) {
					COMPLIANCE_TEST(i < get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT),
							"AUTOEXEC may be writable up to DATACOUNT bits.");
					testvar++;
				}
			}
			for (unsigned int i = 0; i < 16; i++) {
				COMPLIANCE_READ(target, &testvar_read, DM_PROGBUF0 + i);
				do {
					COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTCS);
					busy = get_field(testvar_read, DM_ABSTRACTCS_BUSY);
				} while (busy);
				if (autoexec_progbuf & (1 << i)) {
					COMPLIANCE_TEST(i < get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE),
							"AUTOEXEC may be writable up to PROGBUFSIZE bits.");
					testvar++;
				}
			}

			COMPLIANCE_WRITE(target, DM_ABSTRACTAUTO, 0);
			COMPLIANCE_TEST(ERROR_OK == register_read_direct(target, &value, GDB_REGNO_S0),
					"Need to be able to read S0 to test ABSTRACTAUTO");

			COMPLIANCE_TEST(testvar == value,
					"ABSTRACTAUTO should cause COMMAND to run the expected number of times.");
		}
	}

	/* Single-Step each hart. */
	for (int hartsel = 0; hartsel < riscv_count_harts(target); hartsel++) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));
		COMPLIANCE_MUST_PASS(riscv013_on_step(target));
		COMPLIANCE_MUST_PASS(riscv013_step_current_hart(target));
		COMPLIANCE_TEST(riscv_halt_reason(target, hartsel) == RISCV_HALT_SINGLESTEP,
				"Single Step should result in SINGLESTEP");
	}

	/* Core Register Tests */
	uint64_t bogus_dpc = 0xdeadbeef;
	for (int hartsel = 0; hartsel < riscv_count_harts(target); hartsel++) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));

		/* DCSR Tests */
		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DCSR, 0x0));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &value, GDB_REGNO_DCSR));
		COMPLIANCE_TEST(value != 0,	"Not all bits in DCSR are writable by Debugger");
		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DCSR, 0xFFFFFFFF));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &value, GDB_REGNO_DCSR));
		COMPLIANCE_TEST(value != 0,	"At least some bits in DCSR must be 1");

		/* DPC. Note that DPC is sign-extended. */
		riscv_reg_t dpcmask = 0xFFFFFFFCUL;
		riscv_reg_t dpc;

		if (riscv_xlen(target) > 32)
			dpcmask |= (0xFFFFFFFFULL << 32);

		if (riscv_supports_extension(target, riscv_current_hartid(target), 'C'))
			dpcmask |= 0x2;

		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DPC, dpcmask));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &dpc, GDB_REGNO_DPC));
		COMPLIANCE_TEST(dpcmask == dpc,
				"DPC must be sign-extended to XLEN and writable to all-1s (except the least significant bits)");
		COMPLIANCE_MUST_PASS(register_write_direct(target, GDB_REGNO_DPC, 0));
		COMPLIANCE_MUST_PASS(register_read_direct(target, &dpc, GDB_REGNO_DPC));
		COMPLIANCE_TEST(dpc == 0, "DPC must be writable to 0.");
		if (hartsel == 0)
			bogus_dpc = dpc; /* For a later test step */
	}

	/* NDMRESET
	Asserting non-debug module reset should not reset Debug Module state.
	But it should reset Hart State, e.g. DPC should get a different value.
	Also make sure that DCSR reports cause of 'HALT' even though previously we single-stepped.
	*/

	/* Write some registers. They should not be impacted by ndmreset. */
	COMPLIANCE_WRITE(target, DM_COMMAND, 0xFFFFFFFF);

	for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_WRITE(target, DM_PROGBUF0 + i, testvar);
	}

	for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_WRITE(target, DM_DATA0 + i, testvar);
	}

	COMPLIANCE_WRITE(target, DM_ABSTRACTAUTO, 0xFFFFFFFF);
	COMPLIANCE_READ(target, &abstractauto, DM_ABSTRACTAUTO);

	/* Pulse reset. */
	target->reset_halt = true;
	COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, 0));
	COMPLIANCE_TEST(ERROR_OK == assert_reset(target), "Must be able to assert NDMRESET");
	COMPLIANCE_TEST(ERROR_OK == deassert_reset(target), "Must be able to deassert NDMRESET");

	/* Verify that most stuff is not affected by ndmreset. */
	COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DM_ABSTRACTCS_CMDERR)	== CMDERR_NOT_SUPPORTED,
			"NDMRESET should not affect DM_ABSTRACTCS");
	COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTAUTO);
	COMPLIANCE_TEST(testvar_read == abstractauto, "NDMRESET should not affect DM_ABSTRACTAUTO");

	/* Clean up to avoid future test failures */
	COMPLIANCE_WRITE(target, DM_ABSTRACTCS, DM_ABSTRACTCS_CMDERR);
	COMPLIANCE_WRITE(target, DM_ABSTRACTAUTO, 0);

	for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_READ(target, &testvar_read, DM_PROGBUF0 + i);
		COMPLIANCE_TEST(testvar_read == testvar, "PROGBUF words must not be affected by NDMRESET");
	}

	for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT); i++) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_READ(target, &testvar_read, DM_DATA0 + i);
		COMPLIANCE_TEST(testvar_read == testvar, "DATA words must not be affected by NDMRESET");
	}

	/* Verify that DPC *is* affected by ndmreset. Since we don't know what it *should* be,
	just verify that at least it's not the bogus value anymore. */

	COMPLIANCE_TEST(bogus_dpc != 0xdeadbeef, "BOGUS DPC should have been set somehow (bug in compliance test)");
	COMPLIANCE_MUST_PASS(register_read_direct(target, &value, GDB_REGNO_DPC));
	COMPLIANCE_TEST(bogus_dpc != value, "NDMRESET should move DPC to reset value.");

	COMPLIANCE_TEST(riscv_halt_reason(target, 0) == RISCV_HALT_INTERRUPT,
			"After NDMRESET halt, DCSR should report cause of halt");

	/* DMACTIVE -- deasserting DMACTIVE should reset all the above values. */

	/* Toggle dmactive */
	COMPLIANCE_WRITE(target, DM_DMCONTROL, 0);
	COMPLIANCE_WRITE(target, DM_DMCONTROL, DM_DMCONTROL_DMACTIVE);
	COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTCS);
	COMPLIANCE_TEST(get_field(testvar_read, DM_ABSTRACTCS_CMDERR)	== 0, "ABSTRACTCS.cmderr should reset to 0");
	COMPLIANCE_READ(target, &testvar_read, DM_ABSTRACTAUTO);
	COMPLIANCE_TEST(testvar_read == 0, "ABSTRACTAUTO should reset to 0");

	for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_PROGBUFSIZE); i++) {
		COMPLIANCE_READ(target, &testvar_read, DM_PROGBUF0 + i);
		COMPLIANCE_TEST(testvar_read == 0, "PROGBUF words should reset to 0");
	}

	for (unsigned int i = 0; i < get_field(abstractcs, DM_ABSTRACTCS_DATACOUNT); i++) {
		COMPLIANCE_READ(target, &testvar_read, DM_DATA0 + i);
		COMPLIANCE_TEST(testvar_read == 0, "DATA words should reset to 0");
	}

	/*
	* TODO:
	* DCSR.cause priorities
	* DCSR.stoptime/stopcycle
	* DCSR.stepie
	* DCSR.ebreak
	* DCSR.prv
	*/

	/* Halt every hart for any follow-up tests*/
	COMPLIANCE_MUST_PASS(riscv_halt(target));

	uint32_t failed_tests = total_tests - passed_tests;
	if (total_tests == passed_tests) {
		LOG_INFO("ALL TESTS PASSED\n");
		return ERROR_OK;
	} else {
		LOG_INFO("%d TESTS FAILED\n", failed_tests);
		return ERROR_FAIL;
	}
}
