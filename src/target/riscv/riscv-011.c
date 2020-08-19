/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Support for RISC-V, debug version 0.11. This was never an officially adopted
 * spec, but SiFive made some silicon that uses it.
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
#include "riscv.h"
#include "asm.h"
#include "gdb_regs.h"

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

/**
 * Code structure
 *
 * At the bottom of the stack are the OpenOCD JTAG functions:
 *     jtag_add_[id]r_scan
 *     jtag_execute_query
 *     jtag_add_runtest
 *
 * There are a few functions to just instantly shift a register and get its
 * value:
 *    dtmcontrol_scan
 *    idcode_scan
 *    dbus_scan
 *
 * Because doing one scan and waiting for the result is slow, most functions
 * batch up a bunch of dbus writes and then execute them all at once. They use
 * the scans "class" for this:
 *    scans_new
 *    scans_delete
 *    scans_execute
 *    scans_add_...
 * Usually you new(), call a bunch of add functions, then execute() and look
 * at the results by calling scans_get...()
 *
 * Optimized functions will directly use the scans class above, but slightly
 * lazier code will use the cache functions that in turn use the scans
 * functions:
 *    cache_get...
 *    cache_set...
 *    cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

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

#define DEBUG_ROM_START		0x800
#define DEBUG_ROM_RESUME	(DEBUG_ROM_START + 4)
#define DEBUG_ROM_EXCEPTION	(DEBUG_ROM_START + 8)
#define DEBUG_RAM_START		0x400

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

typedef enum {
	RE_OK,
	RE_FAIL,
	RE_AGAIN
} riscv_error_t;

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

#define DRAM_CACHE_SIZE		16

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

struct memory_cache_line {
	uint32_t data;
	bool valid;
	bool dirty;
};

typedef struct {
	/* Number of address bits in the dbus register. */
	uint8_t addrbits;
	/* Number of words in Debug RAM. */
	unsigned int dramsize;
	uint64_t dcsr;
	uint64_t dpc;
	uint64_t tselect;
	bool tselect_dirty;
	/* The value that mstatus actually has on the target right now. This is not
	 * the value we present to the user. That one may be stored in the
	 * reg_cache. */
	uint64_t mstatus_actual;

	struct memory_cache_line dram_cache[DRAM_CACHE_SIZE];

	/* Number of run-test/idle cycles the target requests we do after each dbus
	 * access. */
	unsigned int dtmcontrol_idle;

	/* This value is incremented every time a dbus access comes back as "busy".
	 * It's used to determine how many run-test/idle cycles to feed the target
	 * in between accesses. */
	unsigned int dbus_busy_delay;

	/* This value is incremented every time we read the debug interrupt as
	 * high.  It's used to add extra run-test/idle cycles after setting debug
	 * interrupt high, so ideally we never have to perform a whole extra scan
	 * before the interrupt is cleared. */
	unsigned int interrupt_high_delay;

	bool never_halted;
} riscv011_info_t;

typedef struct {
	bool haltnot;
	bool interrupt;
} bits_t;

/*** Necessary prototypes. ***/

static int poll_target(struct target *target, bool announce);
static int riscv011_poll(struct target *target);
static int get_register(struct target *target, riscv_reg_t *value, int hartid,
		int regid);

/*** Utility functions. ***/

#define DEBUG_LENGTH	264

static riscv011_info_t *get_info(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	return (riscv011_info_t *) info->version_specific;
}

static unsigned int slot_offset(const struct target *target, slot_t slot)
{
	riscv011_info_t *info = get_info(target);
	switch (riscv_xlen(target)) {
		case 32:
			switch (slot) {
				case SLOT0: return 4;
				case SLOT1: return 5;
				case SLOT_LAST: return info->dramsize-1;
			}
			break;
		case 64:
			switch (slot) {
				case SLOT0: return 4;
				case SLOT1: return 6;
				case SLOT_LAST: return info->dramsize-2;
			}
	}
	LOG_ERROR("slot_offset called with xlen=%d, slot=%d",
			riscv_xlen(target), slot);
	assert(0);
	return 0; /* Silence -Werror=return-type */
}

static uint32_t load_slot(const struct target *target, unsigned int dest,
		slot_t slot)
{
	unsigned int offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	return load(target, dest, ZERO, offset);
}

static uint32_t store_slot(const struct target *target, unsigned int src,
		slot_t slot)
{
	unsigned int offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	return store(target, src, ZERO, offset);
}

static uint16_t dram_address(unsigned int index)
{
	if (index < 0x10)
		return index;
	else
		return 0x40 + index - 0x10;
}

static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4] = { 0 };

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

static uint32_t idcode_scan(struct target *target)
{
	struct scan_field field;
	uint8_t in_value[4];

	jtag_add_ir_scan(target->tap, &select_idcode, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("IDCODE: 0x0 -> 0x%x", in);

	return in;
}

static void increase_dbus_busy_delay(struct target *target)
{
	riscv011_info_t *info = get_info(target);
	info->dbus_busy_delay += info->dbus_busy_delay / 10 + 1;
	LOG_DEBUG("dtmcontrol_idle=%d, dbus_busy_delay=%d, interrupt_high_delay=%d",
			info->dtmcontrol_idle, info->dbus_busy_delay,
			info->interrupt_high_delay);

	dtmcontrol_scan(target, DTMCONTROL_DBUS_RESET);
}

static void increase_interrupt_high_delay(struct target *target)
{
	riscv011_info_t *info = get_info(target);
	info->interrupt_high_delay += info->interrupt_high_delay / 10 + 1;
	LOG_DEBUG("dtmcontrol_idle=%d, dbus_busy_delay=%d, interrupt_high_delay=%d",
			info->dtmcontrol_idle, info->dbus_busy_delay,
			info->interrupt_high_delay);
}

static void add_dbus_scan(const struct target *target, struct scan_field *field,
		uint8_t *out_value, uint8_t *in_value, dbus_op_t op,
		uint16_t address, uint64_t data)
{
	riscv011_info_t *info = get_info(target);
	RISCV_INFO(r);

	if (r->reset_delays_wait >= 0) {
		r->reset_delays_wait--;
		if (r->reset_delays_wait < 0) {
			info->dbus_busy_delay = 0;
			info->interrupt_high_delay = 0;
		}
	}

	field->num_bits = info->addrbits + DBUS_OP_SIZE + DBUS_DATA_SIZE;
	field->in_value = in_value;
	field->out_value = out_value;

	buf_set_u64(out_value, DBUS_OP_START, DBUS_OP_SIZE, op);
	buf_set_u64(out_value, DBUS_DATA_START, DBUS_DATA_SIZE, data);
	buf_set_u64(out_value, DBUS_ADDRESS_START, info->addrbits, address);

	jtag_add_dr_scan(target->tap, 1, field, TAP_IDLE);

	int idle_count = info->dtmcontrol_idle + info->dbus_busy_delay;
	if (data & DMCONTROL_INTERRUPT)
		idle_count += info->interrupt_high_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);
}

static void dump_field(const struct scan_field *field)
{
	static const char * const op_string[] = {"nop", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = (out >> DBUS_OP_START) & ((1 << DBUS_OP_SIZE) - 1);
	char out_interrupt = ((out >> DBUS_DATA_START) & DMCONTROL_INTERRUPT) ? 'i' : '.';
	char out_haltnot = ((out >> DBUS_DATA_START) & DMCONTROL_HALTNOT) ? 'h' : '.';
	unsigned int out_data = out >> 2;
	unsigned int out_address = out >> DBUS_ADDRESS_START;
	uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned int in_op = (in >> DBUS_OP_START) & ((1 << DBUS_OP_SIZE) - 1);
	char in_interrupt = ((in >> DBUS_DATA_START) & DMCONTROL_INTERRUPT) ? 'i' : '.';
	char in_haltnot = ((in >> DBUS_DATA_START) & DMCONTROL_HALTNOT) ? 'h' : '.';
	unsigned int in_data = in >> 2;
	unsigned int in_address = in >> DBUS_ADDRESS_START;

	log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, "scan",
			"%db %s %c%c:%08x @%02x -> %s %c%c:%08x @%02x",
			field->num_bits,
			op_string[out_op], out_interrupt, out_haltnot, out_data,
			out_address,
			status_string[in_op], in_interrupt, in_haltnot, in_data,
			in_address);
}

static dbus_status_t dbus_scan(struct target *target, uint16_t *address_in,
		uint64_t *data_in, dbus_op_t op, uint16_t address_out, uint64_t data_out)
{
	riscv011_info_t *info = get_info(target);
	uint8_t in[8] = {0};
	uint8_t out[8] = {0};
	struct scan_field field = {
		.num_bits = info->addrbits + DBUS_OP_SIZE + DBUS_DATA_SIZE,
		.out_value = out,
		.in_value = in
	};

	assert(info->addrbits != 0);

	buf_set_u64(out, DBUS_OP_START, DBUS_OP_SIZE, op);
	buf_set_u64(out, DBUS_DATA_START, DBUS_DATA_SIZE, data_out);
	buf_set_u64(out, DBUS_ADDRESS_START, info->addrbits, address_out);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int idle_count = info->dtmcontrol_idle + info->dbus_busy_delay;

	if (idle_count)
		jtag_add_runtest(idle_count, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dbus_scan failed jtag scan");
		return DBUS_STATUS_FAILED;
	}

	if (data_in)
		*data_in = buf_get_u64(in, DBUS_DATA_START, DBUS_DATA_SIZE);

	if (address_in)
		*address_in = buf_get_u32(in, DBUS_ADDRESS_START, info->addrbits);

	dump_field(&field);

	return buf_get_u32(in, DBUS_OP_START, DBUS_OP_SIZE);
}

static uint64_t dbus_read(struct target *target, uint16_t address)
{
	uint64_t value;
	dbus_status_t status;
	uint16_t address_in;

	/* If the previous read/write was to the same address, we will get the read data
	 * from the previous access.
	 * While somewhat nonintuitive, this is an efficient way to get the data.
	 */

	unsigned i = 0;
	do {
		status = dbus_scan(target, &address_in, &value, DBUS_OP_READ, address, 0);
		if (status == DBUS_STATUS_BUSY)
			increase_dbus_busy_delay(target);
		if (status == DBUS_STATUS_FAILED) {
			LOG_ERROR("dbus_read(0x%x) failed!", address);
			return 0;
		}
	} while (((status == DBUS_STATUS_BUSY) || (address_in != address)) &&
			i++ < 256);

	if (status != DBUS_STATUS_SUCCESS)
		LOG_ERROR("failed read from 0x%x; value=0x%" PRIx64 ", status=%d\n", address, value, status);

	return value;
}

static void dbus_write(struct target *target, uint16_t address, uint64_t value)
{
	dbus_status_t status = DBUS_STATUS_BUSY;
	unsigned i = 0;
	while (status == DBUS_STATUS_BUSY && i++ < 256) {
		status = dbus_scan(target, NULL, NULL, DBUS_OP_WRITE, address, value);
		if (status == DBUS_STATUS_BUSY)
			increase_dbus_busy_delay(target);
	}
	if (status != DBUS_STATUS_SUCCESS)
		LOG_ERROR("failed to write 0x%" PRIx64 " to 0x%x; status=%d\n", value, address, status);
}

/*** scans "class" ***/

typedef struct {
	/* Number of scans that space is reserved for. */
	unsigned int scan_count;
	/* Size reserved in memory for each scan, in bytes. */
	unsigned int scan_size;
	unsigned int next_scan;
	uint8_t *in;
	uint8_t *out;
	struct scan_field *field;
	const struct target *target;
} scans_t;

static scans_t *scans_new(struct target *target, unsigned int scan_count)
{
	scans_t *scans = malloc(sizeof(scans_t));
	if (!scans)
		goto error0;
	scans->scan_count = scan_count;
	/* This code also gets called before xlen is detected. */
	if (riscv_xlen(target))
		scans->scan_size = 2 + riscv_xlen(target) / 8;
	else
		scans->scan_size = 2 + 128 / 8;
	scans->next_scan = 0;
	scans->in = calloc(scans->scan_size, scans->scan_count);
	if (!scans->in)
		goto error1;
	scans->out = calloc(scans->scan_size, scans->scan_count);
	if (!scans->out)
		goto error2;
	scans->field = calloc(scans->scan_count, sizeof(struct scan_field));
	if (!scans->field)
		goto error3;
	scans->target = target;
	return scans;

error3:
	free(scans->out);
error2:
	free(scans->in);
error1:
	free(scans);
error0:
	return NULL;
}

static scans_t *scans_delete(scans_t *scans)
{
	assert(scans);
	free(scans->field);
	free(scans->out);
	free(scans->in);
	free(scans);
	return NULL;
}

static void scans_reset(scans_t *scans)
{
	scans->next_scan = 0;
}

static void scans_dump(scans_t *scans)
{
	for (unsigned int i = 0; i < scans->next_scan; i++)
		dump_field(&scans->field[i]);
}

static int scans_execute(scans_t *scans)
{
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	scans_dump(scans);

	return ERROR_OK;
}

/** Add a 32-bit dbus write to the scans structure. */
static void scans_add_write32(scans_t *scans, uint16_t address, uint32_t data,
		bool set_interrupt)
{
	const unsigned int i = scans->next_scan;
	int data_offset = scans->scan_size * i;
	add_dbus_scan(scans->target, &scans->field[i], scans->out + data_offset,
			scans->in + data_offset, DBUS_OP_WRITE, address,
			(set_interrupt ? DMCONTROL_INTERRUPT : 0) | DMCONTROL_HALTNOT | data);
	scans->next_scan++;
	assert(scans->next_scan <= scans->scan_count);
}

/** Add a 32-bit dbus write for an instruction that jumps to the beginning of
 * debug RAM. */
static void scans_add_write_jump(scans_t *scans, uint16_t address,
		bool set_interrupt)
{
	scans_add_write32(scans, address,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*address))),
			set_interrupt);
}

/** Add a 32-bit dbus write for an instruction that loads from the indicated
 * slot. */
static void scans_add_write_load(scans_t *scans, uint16_t address,
		unsigned int reg, slot_t slot, bool set_interrupt)
{
	scans_add_write32(scans, address, load_slot(scans->target, reg, slot),
			set_interrupt);
}

/** Add a 32-bit dbus write for an instruction that stores to the indicated
 * slot. */
static void scans_add_write_store(scans_t *scans, uint16_t address,
		unsigned int reg, slot_t slot, bool set_interrupt)
{
	scans_add_write32(scans, address, store_slot(scans->target, reg, slot),
			set_interrupt);
}

/** Add a 32-bit dbus read. */
static void scans_add_read32(scans_t *scans, uint16_t address, bool set_interrupt)
{
	assert(scans->next_scan < scans->scan_count);
	const unsigned int i = scans->next_scan;
	int data_offset = scans->scan_size * i;
	add_dbus_scan(scans->target, &scans->field[i], scans->out + data_offset,
			scans->in + data_offset, DBUS_OP_READ, address,
			(set_interrupt ? DMCONTROL_INTERRUPT : 0) | DMCONTROL_HALTNOT);
	scans->next_scan++;
}

/** Add one or more scans to read the indicated slot. */
static void scans_add_read(scans_t *scans, slot_t slot, bool set_interrupt)
{
	const struct target *target = scans->target;
	switch (riscv_xlen(target)) {
		case 32:
			scans_add_read32(scans, slot_offset(target, slot), set_interrupt);
			break;
		case 64:
			scans_add_read32(scans, slot_offset(target, slot), false);
			scans_add_read32(scans, slot_offset(target, slot) + 1, set_interrupt);
			break;
	}
}

static uint32_t scans_get_u32(scans_t *scans, unsigned int index,
		unsigned first, unsigned num)
{
	return buf_get_u32(scans->in + scans->scan_size * index, first, num);
}

static uint64_t scans_get_u64(scans_t *scans, unsigned int index,
		unsigned first, unsigned num)
{
	return buf_get_u64(scans->in + scans->scan_size * index, first, num);
}

/*** end of scans class ***/

static uint32_t dram_read32(struct target *target, unsigned int index)
{
	uint16_t address = dram_address(index);
	uint32_t value = dbus_read(target, address);
	return value;
}

static void dram_write32(struct target *target, unsigned int index, uint32_t value,
		bool set_interrupt)
{
	uint64_t dbus_value = DMCONTROL_HALTNOT | value;
	if (set_interrupt)
		dbus_value |= DMCONTROL_INTERRUPT;
	dbus_write(target, dram_address(index), dbus_value);
}

/** Read the haltnot and interrupt bits. */
static bits_t read_bits(struct target *target)
{
	uint64_t value;
	dbus_status_t status;
	uint16_t address_in;
	riscv011_info_t *info = get_info(target);

	bits_t err_result = {
		.haltnot = 0,
		.interrupt = 0
	};

	do {
		unsigned i = 0;
		do {
			status = dbus_scan(target, &address_in, &value, DBUS_OP_READ, 0, 0);
			if (status == DBUS_STATUS_BUSY) {
				if (address_in == (1<<info->addrbits) - 1 &&
						value == (1ULL<<DBUS_DATA_SIZE) - 1) {
					LOG_ERROR("TDO seems to be stuck high.");
					return err_result;
				}
				increase_dbus_busy_delay(target);
			} else if (status == DBUS_STATUS_FAILED) {
				/* TODO: return an actual error */
				return err_result;
			}
		} while (status == DBUS_STATUS_BUSY && i++ < 256);

		if (i >= 256) {
			LOG_ERROR("Failed to read from 0x%x; status=%d", address_in, status);
			return err_result;
		}
	} while (address_in > 0x10 && address_in != DMCONTROL);

	bits_t result = {
		.haltnot = get_field(value, DMCONTROL_HALTNOT),
		.interrupt = get_field(value, DMCONTROL_INTERRUPT)
	};
	return result;
}

static int wait_for_debugint_clear(struct target *target, bool ignore_first)
{
	time_t start = time(NULL);
	if (ignore_first) {
		/* Throw away the results of the first read, since they'll contain the
		 * result of the read that happened just before debugint was set.
		 * (Assuming the last scan before calling this function was one that
		 * sets debugint.) */
		read_bits(target);
	}
	while (1) {
		bits_t bits = read_bits(target);
		if (!bits.interrupt)
			return ERROR_OK;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out waiting for debug int to clear."
				  "Increase timeout with riscv set_command_timeout_sec.");
			return ERROR_FAIL;
		}
	}
}

static int dram_check32(struct target *target, unsigned int index,
		uint32_t expected)
{
	uint16_t address = dram_address(index);
	uint32_t actual = dbus_read(target, address);
	if (expected != actual) {
		LOG_ERROR("Wrote 0x%x to Debug RAM at %d, but read back 0x%x",
				expected, index, actual);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static void cache_set32(struct target *target, unsigned int index, uint32_t data)
{
	riscv011_info_t *info = get_info(target);
	if (info->dram_cache[index].valid &&
			info->dram_cache[index].data == data) {
		/* This is already preset on the target. */
		LOG_DEBUG("cache[0x%x] = 0x%08x: DASM(0x%x) (hit)", index, data, data);
		return;
	}
	LOG_DEBUG("cache[0x%x] = 0x%08x: DASM(0x%x)", index, data, data);
	info->dram_cache[index].data = data;
	info->dram_cache[index].valid = true;
	info->dram_cache[index].dirty = true;
}

static void cache_set(struct target *target, slot_t slot, uint64_t data)
{
	unsigned int offset = slot_offset(target, slot);
	cache_set32(target, offset, data);
	if (riscv_xlen(target) > 32)
		cache_set32(target, offset + 1, data >> 32);
}

static void cache_set_jump(struct target *target, unsigned int index)
{
	cache_set32(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))));
}

static void cache_set_load(struct target *target, unsigned int index,
		unsigned int reg, slot_t slot)
{
	uint16_t offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	cache_set32(target, index, load(target, reg, ZERO, offset));
}

static void cache_set_store(struct target *target, unsigned int index,
		unsigned int reg, slot_t slot)
{
	uint16_t offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	cache_set32(target, index, store(target, reg, ZERO, offset));
}

static void dump_debug_ram(struct target *target)
{
	for (unsigned int i = 0; i < DRAM_CACHE_SIZE; i++) {
		uint32_t value = dram_read32(target, i);
		LOG_ERROR("Debug RAM 0x%x: 0x%08x", i, value);
	}
}

/* Call this if the code you just ran writes to debug RAM entries 0 through 3. */
static void cache_invalidate(struct target *target)
{
	riscv011_info_t *info = get_info(target);
	for (unsigned int i = 0; i < info->dramsize; i++) {
		info->dram_cache[i].valid = false;
		info->dram_cache[i].dirty = false;
	}
}

/* Called by cache_write() after the program has run. Also call this if you're
 * running programs without calling cache_write(). */
static void cache_clean(struct target *target)
{
	riscv011_info_t *info = get_info(target);
	for (unsigned int i = 0; i < info->dramsize; i++) {
		if (i >= 4)
			info->dram_cache[i].valid = false;
		info->dram_cache[i].dirty = false;
	}
}

static int cache_check(struct target *target)
{
	riscv011_info_t *info = get_info(target);
	int error = 0;

	for (unsigned int i = 0; i < info->dramsize; i++) {
		if (info->dram_cache[i].valid && !info->dram_cache[i].dirty) {
			if (dram_check32(target, i, info->dram_cache[i].data) != ERROR_OK)
				error++;
		}
	}

	if (error) {
		dump_debug_ram(target);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/** Write cache to the target, and optionally run the program.
 * Then read the value at address into the cache, assuming address < 128. */
#define CACHE_NO_READ	128
static int cache_write(struct target *target, unsigned int address, bool run)
{
	LOG_DEBUG("enter");
	riscv011_info_t *info = get_info(target);
	scans_t *scans = scans_new(target, info->dramsize + 2);
	if (!scans)
		return ERROR_FAIL;

	unsigned int last = info->dramsize;
	for (unsigned int i = 0; i < info->dramsize; i++) {
		if (info->dram_cache[i].dirty)
			last = i;
	}

	if (last == info->dramsize) {
		/* Nothing needs to be written to RAM. */
		dbus_write(target, DMCONTROL, DMCONTROL_HALTNOT | (run ? DMCONTROL_INTERRUPT : 0));

	} else {
		for (unsigned int i = 0; i < info->dramsize; i++) {
			if (info->dram_cache[i].dirty) {
				bool set_interrupt = (i == last && run);
				scans_add_write32(scans, i, info->dram_cache[i].data,
						set_interrupt);
			}
		}
	}

	if (run || address < CACHE_NO_READ) {
		/* Throw away the results of the first read, since it'll contain the
		 * result of the read that happened just before debugint was set. */
		scans_add_read32(scans, address, false);

		/* This scan contains the results of the read the caller requested, as
		 * well as an interrupt bit worth looking at. */
		scans_add_read32(scans, address, false);
	}

	int retval = scans_execute(scans);
	if (retval != ERROR_OK) {
		scans_delete(scans);
		LOG_ERROR("JTAG execute failed.");
		return retval;
	}

	int errors = 0;
	for (unsigned int i = 0; i < scans->next_scan; i++) {
		dbus_status_t status = scans_get_u32(scans, i, DBUS_OP_START,
				DBUS_OP_SIZE);
		switch (status) {
			case DBUS_STATUS_SUCCESS:
				break;
			case DBUS_STATUS_FAILED:
				LOG_ERROR("Debug RAM write failed. Hardware error?");
				scans_delete(scans);
				return ERROR_FAIL;
			case DBUS_STATUS_BUSY:
				errors++;
				break;
			default:
				LOG_ERROR("Got invalid bus access status: %d", status);
				scans_delete(scans);
				return ERROR_FAIL;
		}
	}

	if (errors) {
		increase_dbus_busy_delay(target);

		/* Try again, using the slow careful code.
		 * Write all RAM, just to be extra cautious. */
		for (unsigned int i = 0; i < info->dramsize; i++) {
			if (i == last && run)
				dram_write32(target, last, info->dram_cache[last].data, true);
			else
				dram_write32(target, i, info->dram_cache[i].data, false);
			info->dram_cache[i].dirty = false;
		}
		if (run)
			cache_clean(target);

		if (wait_for_debugint_clear(target, true) != ERROR_OK) {
			LOG_ERROR("Debug interrupt didn't clear.");
			dump_debug_ram(target);
			scans_delete(scans);
			return ERROR_FAIL;
		}

	} else {
		if (run) {
			cache_clean(target);
		} else {
			for (unsigned int i = 0; i < info->dramsize; i++)
				info->dram_cache[i].dirty = false;
		}

		if (run || address < CACHE_NO_READ) {
			int interrupt = scans_get_u32(scans, scans->next_scan-1,
					DBUS_DATA_START + 33, 1);
			if (interrupt) {
				increase_interrupt_high_delay(target);
				/* Slow path wait for it to clear. */
				if (wait_for_debugint_clear(target, false) != ERROR_OK) {
					LOG_ERROR("Debug interrupt didn't clear.");
					dump_debug_ram(target);
					scans_delete(scans);
					return ERROR_FAIL;
				}
			} else {
				/* We read a useful value in that last scan. */
				unsigned int read_addr = scans_get_u32(scans, scans->next_scan-1,
						DBUS_ADDRESS_START, info->addrbits);
				if (read_addr != address) {
					LOG_INFO("Got data from 0x%x but expected it from 0x%x",
							read_addr, address);
				}
				info->dram_cache[read_addr].data =
					scans_get_u32(scans, scans->next_scan-1, DBUS_DATA_START, 32);
				info->dram_cache[read_addr].valid = true;
			}
		}
	}

	scans_delete(scans);
	LOG_DEBUG("exit");

	return ERROR_OK;
}

static uint32_t cache_get32(struct target *target, unsigned int address)
{
	riscv011_info_t *info = get_info(target);
	if (!info->dram_cache[address].valid) {
		info->dram_cache[address].data = dram_read32(target, address);
		info->dram_cache[address].valid = true;
	}
	return info->dram_cache[address].data;
}

static uint64_t cache_get(struct target *target, slot_t slot)
{
	unsigned int offset = slot_offset(target, slot);
	uint64_t value = cache_get32(target, offset);
	if (riscv_xlen(target) > 32)
		value |= ((uint64_t) cache_get32(target, offset + 1)) << 32;
	return value;
}

/* Write instruction that jumps from the specified word in Debug RAM to resume
 * in Debug ROM. */
static void dram_write_jump(struct target *target, unsigned int index,
		bool set_interrupt)
{
	dram_write32(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))),
			set_interrupt);
}

static int wait_for_state(struct target *target, enum target_state state)
{
	time_t start = time(NULL);
	while (1) {
		int result = riscv011_poll(target);
		if (result != ERROR_OK)
			return result;
		if (target->state == state)
			return ERROR_OK;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out waiting for state %d. "
				  "Increase timeout with riscv set_command_timeout_sec.", state);
			return ERROR_FAIL;
		}
	}
}

static int read_remote_csr(struct target *target, uint64_t *value, uint32_t csr)
{
	riscv011_info_t *info = get_info(target);
	cache_set32(target, 0, csrr(S0, csr));
	cache_set_store(target, 1, S0, SLOT0);
	cache_set_jump(target, 2);
	if (cache_write(target, 4, true) != ERROR_OK)
		return ERROR_FAIL;
	*value = cache_get(target, SLOT0);
	LOG_DEBUG("csr 0x%x = 0x%" PRIx64, csr, *value);

	uint32_t exception = cache_get32(target, info->dramsize-1);
	if (exception) {
		LOG_WARNING("Got exception 0x%x when reading %s", exception,
				gdb_regno_name(GDB_REGNO_CSR0 + csr));
		*value = ~0;
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int write_remote_csr(struct target *target, uint32_t csr, uint64_t value)
{
	LOG_DEBUG("csr 0x%x <- 0x%" PRIx64, csr, value);
	cache_set_load(target, 0, S0, SLOT0);
	cache_set32(target, 1, csrw(S0, csr));
	cache_set_jump(target, 2);
	cache_set(target, SLOT0, value);
	if (cache_write(target, 4, true) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int write_gpr(struct target *target, unsigned int gpr, uint64_t value)
{
	cache_set_load(target, 0, gpr, SLOT0);
	cache_set_jump(target, 1);
	cache_set(target, SLOT0, value);
	if (cache_write(target, 4, true) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int maybe_read_tselect(struct target *target)
{
	riscv011_info_t *info = get_info(target);

	if (info->tselect_dirty) {
		int result = read_remote_csr(target, &info->tselect, CSR_TSELECT);
		if (result != ERROR_OK)
			return result;
		info->tselect_dirty = false;
	}

	return ERROR_OK;
}

static int maybe_write_tselect(struct target *target)
{
	riscv011_info_t *info = get_info(target);

	if (!info->tselect_dirty) {
		int result = write_remote_csr(target, CSR_TSELECT, info->tselect);
		if (result != ERROR_OK)
			return result;
		info->tselect_dirty = true;
	}

	return ERROR_OK;
}

static int execute_resume(struct target *target, bool step)
{
	riscv011_info_t *info = get_info(target);

	LOG_DEBUG("step=%d", step);

	maybe_write_tselect(target);

	/* TODO: check if dpc is dirty (which also is true if an exception was hit
	 * at any time) */
	cache_set_load(target, 0, S0, SLOT0);
	cache_set32(target, 1, csrw(S0, CSR_DPC));
	cache_set_jump(target, 2);
	cache_set(target, SLOT0, info->dpc);
	if (cache_write(target, 4, true) != ERROR_OK)
		return ERROR_FAIL;

	struct reg *mstatus_reg = &target->reg_cache->reg_list[GDB_REGNO_MSTATUS];
	if (mstatus_reg->valid) {
		uint64_t mstatus_user = buf_get_u64(mstatus_reg->value, 0, riscv_xlen(target));
		if (mstatus_user != info->mstatus_actual) {
			cache_set_load(target, 0, S0, SLOT0);
			cache_set32(target, 1, csrw(S0, CSR_MSTATUS));
			cache_set_jump(target, 2);
			cache_set(target, SLOT0, mstatus_user);
			if (cache_write(target, 4, true) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	info->dcsr = set_field(info->dcsr, DCSR_EBREAKM, riscv_ebreakm);
	info->dcsr = set_field(info->dcsr, DCSR_EBREAKS, riscv_ebreaks);
	info->dcsr = set_field(info->dcsr, DCSR_EBREAKU, riscv_ebreaku);
	info->dcsr = set_field(info->dcsr, DCSR_EBREAKH, 1);
	info->dcsr &= ~DCSR_HALT;

	if (step)
		info->dcsr |= DCSR_STEP;
	else
		info->dcsr &= ~DCSR_STEP;

	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write32(target, 1, csrw(S0, CSR_DCSR), false);
	dram_write32(target, 2, fence_i(), false);
	dram_write_jump(target, 3, false);

	/* Write DCSR value, set interrupt and clear haltnot. */
	uint64_t dbus_value = DMCONTROL_INTERRUPT | info->dcsr;
	dbus_write(target, dram_address(4), dbus_value);

	cache_invalidate(target);

	if (wait_for_debugint_clear(target, true) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	target->state = TARGET_RUNNING;
	register_cache_invalidate(target->reg_cache);

	return ERROR_OK;
}

/* Execute a step, and wait for reentry into Debug Mode. */
static int full_step(struct target *target, bool announce)
{
	int result = execute_resume(target, true);
	if (result != ERROR_OK)
		return result;
	time_t start = time(NULL);
	while (1) {
		result = poll_target(target, announce);
		if (result != ERROR_OK)
			return result;
		if (target->state != TARGET_DEBUG_RUNNING)
			break;
		if (time(NULL) - start > riscv_command_timeout_sec) {
			LOG_ERROR("Timed out waiting for step to complete."
					"Increase timeout with riscv set_command_timeout_sec");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int resume(struct target *target, int debug_execution, bool step)
{
	if (debug_execution) {
		LOG_ERROR("TODO: debug_execution is true");
		return ERROR_FAIL;
	}

	return execute_resume(target, step);
}

static uint64_t reg_cache_get(struct target *target, unsigned int number)
{
	struct reg *r = &target->reg_cache->reg_list[number];
	if (!r->valid) {
		LOG_ERROR("Register cache entry for %d is invalid!", number);
		assert(r->valid);
	}
	uint64_t value = buf_get_u64(r->value, 0, r->size);
	LOG_DEBUG("%s = 0x%" PRIx64, r->name, value);
	return value;
}

static void reg_cache_set(struct target *target, unsigned int number,
		uint64_t value)
{
	struct reg *r = &target->reg_cache->reg_list[number];
	LOG_DEBUG("%s <= 0x%" PRIx64, r->name, value);
	r->valid = true;
	buf_set_u64(r->value, 0, r->size, value);
}

static int update_mstatus_actual(struct target *target)
{
	struct reg *mstatus_reg = &target->reg_cache->reg_list[GDB_REGNO_MSTATUS];
	if (mstatus_reg->valid) {
		/* We previously made it valid. */
		return ERROR_OK;
	}

	/* Force reading the register. In that process mstatus_actual will be
	 * updated. */
	riscv_reg_t mstatus;
	return get_register(target, &mstatus, 0, GDB_REGNO_MSTATUS);
}

/*** OpenOCD target functions. ***/

static int register_read(struct target *target, riscv_reg_t *value, int regnum)
{
	riscv011_info_t *info = get_info(target);
	if (regnum >= GDB_REGNO_CSR0 && regnum <= GDB_REGNO_CSR4095) {
		cache_set32(target, 0, csrr(S0, regnum - GDB_REGNO_CSR0));
		cache_set_store(target, 1, S0, SLOT0);
		cache_set_jump(target, 2);
	} else {
		LOG_ERROR("Don't know how to read register %d", regnum);
		return ERROR_FAIL;
	}

	if (cache_write(target, 4, true) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t exception = cache_get32(target, info->dramsize-1);
	if (exception) {
		LOG_WARNING("Got exception 0x%x when reading %s", exception, gdb_regno_name(regnum));
		*value = ~0;
		return ERROR_FAIL;
	}

	*value = cache_get(target, SLOT0);
	LOG_DEBUG("reg[%d]=0x%" PRIx64, regnum, *value);

	if (regnum == GDB_REGNO_MSTATUS)
		info->mstatus_actual = *value;

	return ERROR_OK;
}

/* Write the register. No caching or games. */
static int register_write(struct target *target, unsigned int number,
		uint64_t value)
{
	riscv011_info_t *info = get_info(target);

	maybe_write_tselect(target);

	if (number == S0) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set32(target, 1, csrw(S0, CSR_DSCRATCH0));
		cache_set_jump(target, 2);
	} else if (number == S1) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set_store(target, 1, S0, SLOT_LAST);
		cache_set_jump(target, 2);
	} else if (number <= GDB_REGNO_XPR31) {
		cache_set_load(target, 0, number - GDB_REGNO_ZERO, SLOT0);
		cache_set_jump(target, 1);
	} else if (number == GDB_REGNO_PC) {
		info->dpc = value;
		return ERROR_OK;
	} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
		int result = update_mstatus_actual(target);
		if (result != ERROR_OK)
			return result;
		unsigned i = 0;
		if ((info->mstatus_actual & MSTATUS_FS) == 0) {
			info->mstatus_actual = set_field(info->mstatus_actual, MSTATUS_FS, 1);
			cache_set_load(target, i++, S0, SLOT1);
			cache_set32(target, i++, csrw(S0, CSR_MSTATUS));
			cache_set(target, SLOT1, info->mstatus_actual);
		}

		if (riscv_xlen(target) == 32)
			cache_set32(target, i++, flw(number - GDB_REGNO_FPR0, 0, DEBUG_RAM_START + 16));
		else
			cache_set32(target, i++, fld(number - GDB_REGNO_FPR0, 0, DEBUG_RAM_START + 16));
		cache_set_jump(target, i++);
	} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set32(target, 1, csrw(S0, number - GDB_REGNO_CSR0));
		cache_set_jump(target, 2);

		if (number == GDB_REGNO_MSTATUS)
			info->mstatus_actual = value;
	} else if (number == GDB_REGNO_PRIV) {
		info->dcsr = set_field(info->dcsr, DCSR_PRV, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("Don't know how to write register %d", number);
		return ERROR_FAIL;
	}

	cache_set(target, SLOT0, value);
	if (cache_write(target, info->dramsize - 1, true) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t exception = cache_get32(target, info->dramsize-1);
	if (exception) {
		LOG_WARNING("Got exception 0x%x when writing %s", exception,
				gdb_regno_name(number));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int get_register(struct target *target, riscv_reg_t *value, int hartid,
		int regid)
{
	assert(hartid == 0);
	riscv011_info_t *info = get_info(target);

	maybe_write_tselect(target);

	if (regid <= GDB_REGNO_XPR31) {
		*value = reg_cache_get(target, regid);
	} else if (regid == GDB_REGNO_PC) {
		*value = info->dpc;
	} else if (regid >= GDB_REGNO_FPR0 && regid <= GDB_REGNO_FPR31) {
		int result = update_mstatus_actual(target);
		if (result != ERROR_OK)
			return result;
		unsigned i = 0;
		if ((info->mstatus_actual & MSTATUS_FS) == 0) {
			info->mstatus_actual = set_field(info->mstatus_actual, MSTATUS_FS, 1);
			cache_set_load(target, i++, S0, SLOT1);
			cache_set32(target, i++, csrw(S0, CSR_MSTATUS));
			cache_set(target, SLOT1, info->mstatus_actual);
		}

		if (riscv_xlen(target) == 32)
			cache_set32(target, i++, fsw(regid - GDB_REGNO_FPR0, 0, DEBUG_RAM_START + 16));
		else
			cache_set32(target, i++, fsd(regid - GDB_REGNO_FPR0, 0, DEBUG_RAM_START + 16));
		cache_set_jump(target, i++);

		if (cache_write(target, 4, true) != ERROR_OK)
			return ERROR_FAIL;
	} else if (regid == GDB_REGNO_PRIV) {
		*value = get_field(info->dcsr, DCSR_PRV);
	} else {
		int result = register_read(target, value, regid);
		if (result != ERROR_OK)
			return result;
	}

	if (regid == GDB_REGNO_MSTATUS)
		target->reg_cache->reg_list[regid].valid = true;

	return ERROR_OK;
}

static int set_register(struct target *target, int hartid, int regid,
		uint64_t value)
{
	assert(hartid == 0);
	return register_write(target, regid, value);
}

static int halt(struct target *target)
{
	LOG_DEBUG("riscv_halt()");
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	cache_set32(target, 0, csrsi(CSR_DCSR, DCSR_HALT));
	cache_set32(target, 1, csrr(S0, CSR_MHARTID));
	cache_set32(target, 2, sw(S0, ZERO, SETHALTNOT));
	cache_set_jump(target, 3);

	if (cache_write(target, 4, true) != ERROR_OK) {
		LOG_ERROR("cache_write() failed.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	free(info->version_specific);
	info->version_specific = NULL;
}

static int strict_step(struct target *target, bool announce)
{
	LOG_DEBUG("enter");

	struct watchpoint *watchpoint = target->watchpoints;
	while (watchpoint) {
		riscv_remove_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	int result = full_step(target, announce);
	if (result != ERROR_OK)
		return result;

	watchpoint = target->watchpoints;
	while (watchpoint) {
		riscv_add_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	return ERROR_OK;
}

static int step(struct target *target, int current, target_addr_t address,
		int handle_breakpoints)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	if (!current) {
		if (riscv_xlen(target) > 32) {
			LOG_WARNING("Asked to resume at 32-bit PC on %d-bit target.",
					riscv_xlen(target));
		}
		int result = register_write(target, GDB_REGNO_PC, address);
		if (result != ERROR_OK)
			return result;
	}

	if (handle_breakpoints) {
		int result = strict_step(target, true);
		if (result != ERROR_OK)
			return result;
	} else {
		return full_step(target, false);
	}

	return ERROR_OK;
}

static int examine(struct target *target)
{
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	LOG_DEBUG("  addrbits=%d", get_field(dtmcontrol, DTMCONTROL_ADDRBITS));
	LOG_DEBUG("  version=%d", get_field(dtmcontrol, DTMCONTROL_VERSION));
	LOG_DEBUG("  idle=%d", get_field(dtmcontrol, DTMCONTROL_IDLE));
	if (dtmcontrol == 0) {
		LOG_ERROR("dtmcontrol is 0. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	if (get_field(dtmcontrol, DTMCONTROL_VERSION) != 0) {
		LOG_ERROR("Unsupported DTM version %d. (dtmcontrol=0x%x)",
				get_field(dtmcontrol, DTMCONTROL_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	RISCV_INFO(r);

	riscv011_info_t *info = get_info(target);
	info->addrbits = get_field(dtmcontrol, DTMCONTROL_ADDRBITS);
	info->dtmcontrol_idle = get_field(dtmcontrol, DTMCONTROL_IDLE);
	if (info->dtmcontrol_idle == 0) {
		/* Some old SiFive cores don't set idle but need it to be 1. */
		uint32_t idcode = idcode_scan(target);
		if (idcode == 0x10e31913)
			info->dtmcontrol_idle = 1;
	}

	uint32_t dminfo = dbus_read(target, DMINFO);
	LOG_DEBUG("dminfo: 0x%08x", dminfo);
	LOG_DEBUG("  abussize=0x%x", get_field(dminfo, DMINFO_ABUSSIZE));
	LOG_DEBUG("  serialcount=0x%x", get_field(dminfo, DMINFO_SERIALCOUNT));
	LOG_DEBUG("  access128=%d", get_field(dminfo, DMINFO_ACCESS128));
	LOG_DEBUG("  access64=%d", get_field(dminfo, DMINFO_ACCESS64));
	LOG_DEBUG("  access32=%d", get_field(dminfo, DMINFO_ACCESS32));
	LOG_DEBUG("  access16=%d", get_field(dminfo, DMINFO_ACCESS16));
	LOG_DEBUG("  access8=%d", get_field(dminfo, DMINFO_ACCESS8));
	LOG_DEBUG("  dramsize=0x%x", get_field(dminfo, DMINFO_DRAMSIZE));
	LOG_DEBUG("  authenticated=0x%x", get_field(dminfo, DMINFO_AUTHENTICATED));
	LOG_DEBUG("  authbusy=0x%x", get_field(dminfo, DMINFO_AUTHBUSY));
	LOG_DEBUG("  authtype=0x%x", get_field(dminfo, DMINFO_AUTHTYPE));
	LOG_DEBUG("  version=0x%x", get_field(dminfo, DMINFO_VERSION));

	if (get_field(dminfo, DMINFO_VERSION) != 1) {
		LOG_ERROR("OpenOCD only supports Debug Module version 1, not %d "
				"(dminfo=0x%x)", get_field(dminfo, DMINFO_VERSION), dminfo);
		return ERROR_FAIL;
	}

	info->dramsize = get_field(dminfo, DMINFO_DRAMSIZE) + 1;

	if (get_field(dminfo, DMINFO_AUTHTYPE) != 0) {
		LOG_ERROR("Authentication required by RISC-V core but not "
				"supported by OpenOCD. dminfo=0x%x", dminfo);
		return ERROR_FAIL;
	}

	/* Pretend this is a 32-bit system until we have found out the true value. */
	r->xlen[0] = 32;

	/* Figure out XLEN, and test writing all of Debug RAM while we're at it. */
	cache_set32(target, 0, xori(S1, ZERO, -1));
	/* 0xffffffff  0xffffffff:ffffffff  0xffffffff:ffffffff:ffffffff:ffffffff */
	cache_set32(target, 1, srli(S1, S1, 31));
	/* 0x00000001  0x00000001:ffffffff  0x00000001:ffffffff:ffffffff:ffffffff */
	cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START));
	cache_set32(target, 3, srli(S1, S1, 31));
	/* 0x00000000  0x00000000:00000003  0x00000000:00000003:ffffffff:ffffffff */
	cache_set32(target, 4, sw(S1, ZERO, DEBUG_RAM_START + 4));
	cache_set_jump(target, 5);
	for (unsigned i = 6; i < info->dramsize; i++)
		cache_set32(target, i, i * 0x01020304);

	cache_write(target, 0, false);

	/* Check that we can actually read/write dram. */
	if (cache_check(target) != ERROR_OK)
		return ERROR_FAIL;

	cache_write(target, 0, true);
	cache_invalidate(target);

	uint32_t word0 = cache_get32(target, 0);
	uint32_t word1 = cache_get32(target, 1);
	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;
	if (word0 == 1 && word1 == 0) {
		generic_info->xlen[0] = 32;
	} else if (word0 == 0xffffffff && word1 == 3) {
		generic_info->xlen[0] = 64;
	} else if (word0 == 0xffffffff && word1 == 0xffffffff) {
		generic_info->xlen[0] = 128;
	} else {
		uint32_t exception = cache_get32(target, info->dramsize-1);
		LOG_ERROR("Failed to discover xlen; word0=0x%x, word1=0x%x, exception=0x%x",
				word0, word1, exception);
		dump_debug_ram(target);
		return ERROR_FAIL;
	}
	LOG_DEBUG("Discovered XLEN is %d", riscv_xlen(target));

	if (read_remote_csr(target, &r->misa[0], CSR_MISA) != ERROR_OK) {
		const unsigned old_csr_misa = 0xf10;
		LOG_WARNING("Failed to read misa at 0x%x; trying 0x%x.", CSR_MISA,
				old_csr_misa);
		if (read_remote_csr(target, &r->misa[0], old_csr_misa) != ERROR_OK) {
			/* Maybe this is an old core that still has $misa at the old
			 * address. */
			LOG_ERROR("Failed to read misa at 0x%x.", old_csr_misa);
			return ERROR_FAIL;
		}
	}

	/* Update register list to match discovered XLEN/supported extensions. */
	riscv_init_registers(target);

	info->never_halted = true;

	int result = riscv011_poll(target);
	if (result != ERROR_OK)
		return result;

	target_set_examined(target);
	riscv_set_current_hartid(target, 0);
	for (size_t i = 0; i < 32; ++i)
		reg_cache_set(target, i, -1);
	LOG_INFO("Examined RISCV core; XLEN=%d, misa=0x%" PRIx64,
			riscv_xlen(target), r->misa[0]);

	return ERROR_OK;
}

static riscv_error_t handle_halt_routine(struct target *target)
{
	riscv011_info_t *info = get_info(target);

	scans_t *scans = scans_new(target, 256);
	if (!scans)
		return RE_FAIL;

	/* Read all GPRs as fast as we can, because gdb is going to ask for them
	 * anyway. Reading them one at a time is much slower. */

	/* Write the jump back to address 1. */
	scans_add_write_jump(scans, 1, false);
	for (int reg = 1; reg < 32; reg++) {
		if (reg == S0 || reg == S1)
			continue;

		/* Write store instruction. */
		scans_add_write_store(scans, 0, reg, SLOT0, true);

		/* Read value. */
		scans_add_read(scans, SLOT0, false);
	}

	/* Write store of s0 at index 1. */
	scans_add_write_store(scans, 1, S0, SLOT0, false);
	/* Write jump at index 2. */
	scans_add_write_jump(scans, 2, false);

	/* Read S1 from debug RAM */
	scans_add_write_load(scans, 0, S0, SLOT_LAST, true);
	/* Read value. */
	scans_add_read(scans, SLOT0, false);

	/* Read S0 from dscratch */
	unsigned int csr[] = {CSR_DSCRATCH0, CSR_DPC, CSR_DCSR};
	for (unsigned int i = 0; i < DIM(csr); i++) {
		scans_add_write32(scans, 0, csrr(S0, csr[i]), true);
		scans_add_read(scans, SLOT0, false);
	}

	/* Final read to get the last value out. */
	scans_add_read32(scans, 4, false);

	int retval = scans_execute(scans);
	if (retval != ERROR_OK) {
		LOG_ERROR("JTAG execute failed: %d", retval);
		goto error;
	}

	unsigned int dbus_busy = 0;
	unsigned int interrupt_set = 0;
	unsigned result = 0;
	uint64_t value = 0;
	reg_cache_set(target, 0, 0);
	/* The first scan result is the result from something old we don't care
	 * about. */
	for (unsigned int i = 1; i < scans->next_scan && dbus_busy == 0; i++) {
		dbus_status_t status = scans_get_u32(scans, i, DBUS_OP_START,
				DBUS_OP_SIZE);
		uint64_t data = scans_get_u64(scans, i, DBUS_DATA_START, DBUS_DATA_SIZE);
		uint32_t address = scans_get_u32(scans, i, DBUS_ADDRESS_START,
				info->addrbits);
		switch (status) {
			case DBUS_STATUS_SUCCESS:
				break;
			case DBUS_STATUS_FAILED:
				LOG_ERROR("Debug access failed. Hardware error?");
				goto error;
			case DBUS_STATUS_BUSY:
				dbus_busy++;
				break;
			default:
				LOG_ERROR("Got invalid bus access status: %d", status);
				goto error;
		}
		if (data & DMCONTROL_INTERRUPT) {
			interrupt_set++;
			break;
		}
		if (address == 4 || address == 5) {
			unsigned int reg;
			switch (result) {
				case 0:
					reg = 1;
					break;
				case 1:
					reg = 2;
					break;
				case 2:
					reg = 3;
					break;
				case 3:
					reg = 4;
					break;
				case 4:
					reg = 5;
					break;
				case 5:
					reg = 6;
					break;
				case 6:
					reg = 7;
					break;
					/* S0 */
					/* S1 */
				case 7:
					reg = 10;
					break;
				case 8:
					reg = 11;
					break;
				case 9:
					reg = 12;
					break;
				case 10:
					reg = 13;
					break;
				case 11:
					reg = 14;
					break;
				case 12:
					reg = 15;
					break;
				case 13:
					reg = 16;
					break;
				case 14:
					reg = 17;
					break;
				case 15:
					reg = 18;
					break;
				case 16:
					reg = 19;
					break;
				case 17:
					reg = 20;
					break;
				case 18:
					reg = 21;
					break;
				case 19:
					reg = 22;
					break;
				case 20:
					reg = 23;
					break;
				case 21:
					reg = 24;
					break;
				case 22:
					reg = 25;
					break;
				case 23:
					reg = 26;
					break;
				case 24:
					reg = 27;
					break;
				case 25:
					reg = 28;
					break;
				case 26:
					reg = 29;
					break;
				case 27:
					reg = 30;
					break;
				case 28:
					reg = 31;
					break;
				case 29:
					reg = S1;
					break;
				case 30:
					reg = S0;
					break;
				case 31:
					reg = CSR_DPC;
					break;
				case 32:
					reg = CSR_DCSR;
					break;
				default:
					assert(0);
					LOG_ERROR("Got invalid register result %d", result);
					goto error;
			}
			if (riscv_xlen(target) == 32) {
				reg_cache_set(target, reg, data & 0xffffffff);
				result++;
			} else if (riscv_xlen(target) == 64) {
				if (address == 4) {
					value = data & 0xffffffff;
				} else if (address == 5) {
					reg_cache_set(target, reg, ((data & 0xffffffff) << 32) | value);
					value = 0;
					result++;
				}
			}
		}
	}

	scans_delete(scans);

	if (dbus_busy) {
		increase_dbus_busy_delay(target);
		return RE_AGAIN;
	}
	if (interrupt_set) {
		increase_interrupt_high_delay(target);
		return RE_AGAIN;
	}

	/* TODO: get rid of those 2 variables and talk to the cache directly. */
	info->dpc = reg_cache_get(target, CSR_DPC);
	info->dcsr = reg_cache_get(target, CSR_DCSR);

	cache_invalidate(target);

	return RE_OK;

error:
	scans_delete(scans);
	return RE_FAIL;
}

static int handle_halt(struct target *target, bool announce)
{
	riscv011_info_t *info = get_info(target);
	target->state = TARGET_HALTED;

	riscv_error_t re;
	do {
		re = handle_halt_routine(target);
	} while (re == RE_AGAIN);
	if (re != RE_OK) {
		LOG_ERROR("handle_halt_routine failed");
		return ERROR_FAIL;
	}

	int cause = get_field(info->dcsr, DCSR_CAUSE);
	switch (cause) {
		case DCSR_CAUSE_SWBP:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case DCSR_CAUSE_HWBP:
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case DCSR_CAUSE_DEBUGINT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case DCSR_CAUSE_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case DCSR_CAUSE_HALT:
		default:
			LOG_ERROR("Invalid halt cause %d in DCSR (0x%" PRIx64 ")",
					cause, info->dcsr);
	}

	if (info->never_halted) {
		info->never_halted = false;

		int result = maybe_read_tselect(target);
		if (result != ERROR_OK)
			return result;
		riscv_enumerate_triggers(target);
	}

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		int retval;
		if (riscv_semihosting(target, &retval) != 0)
			return retval;
	}

	if (announce)
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	const char *cause_string[] = {
		"none",
		"software breakpoint",
		"hardware trigger",
		"debug interrupt",
		"step",
		"halt"
	};
	/* This is logged to the user so that gdb will show it when a user types
	 * 'monitor reset init'. At that time gdb appears to have the pc cached
	 * still so if a user manually inspects the pc it will still have the old
	 * value. */
	LOG_USER("halted at 0x%" PRIx64 " due to %s", info->dpc, cause_string[cause]);

	return ERROR_OK;
}

static int poll_target(struct target *target, bool announce)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	/* Inhibit debug logging during poll(), which isn't usually interesting and
	 * just fills up the screen/logs with clutter. */
	int old_debug_level = debug_level;
	if (debug_level >= LOG_LVL_DEBUG)
		debug_level = LOG_LVL_INFO;
	bits_t bits = read_bits(target);
	debug_level = old_debug_level;

	if (bits.haltnot && bits.interrupt) {
		target->state = TARGET_DEBUG_RUNNING;
		LOG_DEBUG("debug running");
	} else if (bits.haltnot && !bits.interrupt) {
		if (target->state != TARGET_HALTED)
			return handle_halt(target, announce);
	} else if (!bits.haltnot && bits.interrupt) {
		/* Target is halting. There is no state for that, so don't change anything. */
		LOG_DEBUG("halting");
	} else if (!bits.haltnot && !bits.interrupt) {
		target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

static int riscv011_poll(struct target *target)
{
	return poll_target(target, true);
}

static int riscv011_resume(struct target *target, int current,
		target_addr_t address, int handle_breakpoints, int debug_execution)
{
	RISCV_INFO(r);
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	r->prepped = false;
	return resume(target, debug_execution, false);
}

static int assert_reset(struct target *target)
{
	riscv011_info_t *info = get_info(target);
	/* TODO: Maybe what I implemented here is more like soft_reset_halt()? */

	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	/* The only assumption we can make is that the TAP was reset. */
	if (wait_for_debugint_clear(target, true) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	/* Not sure what we should do when there are multiple cores.
	 * Here just reset the single hart we're talking to. */
	info->dcsr = set_field(info->dcsr, DCSR_EBREAKM, riscv_ebreakm);
	info->dcsr = set_field(info->dcsr, DCSR_EBREAKS, riscv_ebreaks);
	info->dcsr = set_field(info->dcsr, DCSR_EBREAKU, riscv_ebreaku);
	info->dcsr = set_field(info->dcsr, DCSR_EBREAKH, 1);
	info->dcsr |= DCSR_HALT;
	if (target->reset_halt)
		info->dcsr |= DCSR_NDRESET;
	else
		info->dcsr |= DCSR_FULLRESET;
	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write32(target, 1, csrw(S0, CSR_DCSR), false);
	/* We shouldn't actually need the jump because a reset should happen. */
	dram_write_jump(target, 2, false);
	dram_write32(target, 4, info->dcsr, true);
	cache_invalidate(target);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int deassert_reset(struct target *target)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	if (target->reset_halt)
		return wait_for_state(target, TARGET_HALTED);
	else
		return wait_for_state(target, TARGET_RUNNING);
}

static int read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer, uint32_t increment)
{
	if (increment != size) {
		LOG_ERROR("read_memory with custom increment not implemented");
		return ERROR_NOT_IMPLEMENTED;
	}

	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	cache_set32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
	switch (size) {
		case 1:
			cache_set32(target, 1, lb(S1, S0, 0));
			cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;
		case 2:
			cache_set32(target, 1, lh(S1, S0, 0));
			cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;
		case 4:
			cache_set32(target, 1, lw(S1, S0, 0));
			cache_set32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	cache_set_jump(target, 3);
	cache_write(target, CACHE_NO_READ, false);

	riscv011_info_t *info = get_info(target);
	const unsigned max_batch_size = 256;
	scans_t *scans = scans_new(target, max_batch_size);
	if (!scans)
		return ERROR_FAIL;

	uint32_t result_value = 0x777;
	uint32_t i = 0;
	while (i < count + 3) {
		unsigned int batch_size = MIN(count + 3 - i, max_batch_size);
		scans_reset(scans);

		for (unsigned int j = 0; j < batch_size; j++) {
			if (i + j == count) {
				/* Just insert a read so we can scan out the last value. */
				scans_add_read32(scans, 4, false);
			} else if (i + j >= count + 1) {
				/* And check for errors. */
				scans_add_read32(scans, info->dramsize-1, false);
			} else {
				/* Write the next address and set interrupt. */
				uint32_t offset = size * (i + j);
				scans_add_write32(scans, 4, address + offset, true);
			}
		}

		int retval = scans_execute(scans);
		if (retval != ERROR_OK) {
			LOG_ERROR("JTAG execute failed: %d", retval);
			goto error;
		}

		int dbus_busy = 0;
		int execute_busy = 0;
		for (unsigned int j = 0; j < batch_size; j++) {
			dbus_status_t status = scans_get_u32(scans, j, DBUS_OP_START,
					DBUS_OP_SIZE);
			switch (status) {
				case DBUS_STATUS_SUCCESS:
					break;
				case DBUS_STATUS_FAILED:
					LOG_ERROR("Debug RAM write failed. Hardware error?");
					goto error;
				case DBUS_STATUS_BUSY:
					dbus_busy++;
					break;
				default:
					LOG_ERROR("Got invalid bus access status: %d", status);
					return ERROR_FAIL;
			}
			uint64_t data = scans_get_u64(scans, j, DBUS_DATA_START,
					DBUS_DATA_SIZE);
			if (data & DMCONTROL_INTERRUPT)
				execute_busy++;
			if (i + j == count + 2) {
				result_value = data;
			} else if (i + j > 1) {
				uint32_t offset = size * (i + j - 2);
				switch (size) {
					case 1:
						buffer[offset] = data;
						break;
					case 2:
						buffer[offset] = data;
						buffer[offset+1] = data >> 8;
						break;
					case 4:
						buffer[offset] = data;
						buffer[offset+1] = data >> 8;
						buffer[offset+2] = data >> 16;
						buffer[offset+3] = data >> 24;
						break;
				}
			}
			LOG_DEBUG("j=%d status=%d data=%09" PRIx64, j, status, data);
		}
		if (dbus_busy)
			increase_dbus_busy_delay(target);
		if (execute_busy)
			increase_interrupt_high_delay(target);
		if (dbus_busy || execute_busy) {
			wait_for_debugint_clear(target, false);

			/* Retry. */
			LOG_INFO("Retrying memory read starting from 0x%" TARGET_PRIxADDR
					" with more delays", address + size * i);
		} else {
			i += batch_size;
		}
	}

	if (result_value != 0) {
		LOG_USER("Core got an exception (0x%x) while reading from 0x%"
				TARGET_PRIxADDR, result_value, address + size * (count-1));
		if (count > 1) {
			LOG_USER("(It may have failed between 0x%" TARGET_PRIxADDR
					" and 0x%" TARGET_PRIxADDR " as well, but we "
					"didn't check then.)",
					address, address + size * (count-2) + size - 1);
		}
		goto error;
	}

	scans_delete(scans);
	cache_clean(target);
	return ERROR_OK;

error:
	scans_delete(scans);
	cache_clean(target);
	return ERROR_FAIL;
}

static int setup_write_memory(struct target *target, uint32_t size)
{
	switch (size) {
		case 1:
			cache_set32(target, 0, lb(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set32(target, 1, sb(S0, T0, 0));
			break;
		case 2:
			cache_set32(target, 0, lh(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set32(target, 1, sh(S0, T0, 0));
			break;
		case 4:
			cache_set32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set32(target, 1, sw(S0, T0, 0));
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	cache_set32(target, 2, addi(T0, T0, size));
	cache_set_jump(target, 3);
	cache_write(target, 4, false);

	return ERROR_OK;
}

static int write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	riscv011_info_t *info = get_info(target);
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	/* Set up the address. */
	cache_set_store(target, 0, T0, SLOT1);
	cache_set_load(target, 1, T0, SLOT0);
	cache_set_jump(target, 2);
	cache_set(target, SLOT0, address);
	if (cache_write(target, 5, true) != ERROR_OK)
		return ERROR_FAIL;

	uint64_t t0 = cache_get(target, SLOT1);
	LOG_DEBUG("t0 is 0x%" PRIx64, t0);

	if (setup_write_memory(target, size) != ERROR_OK)
		return ERROR_FAIL;

	const unsigned max_batch_size = 256;
	scans_t *scans = scans_new(target, max_batch_size);
	if (!scans)
		return ERROR_FAIL;

	uint32_t result_value = 0x777;
	uint32_t i = 0;
	while (i < count + 2) {
		unsigned int batch_size = MIN(count + 2 - i, max_batch_size);
		scans_reset(scans);

		for (unsigned int j = 0; j < batch_size; j++) {
			if (i + j >= count) {
				/* Check for an exception. */
				scans_add_read32(scans, info->dramsize-1, false);
			} else {
				/* Write the next value and set interrupt. */
				uint32_t value;
				uint32_t offset = size * (i + j);
				switch (size) {
					case 1:
						value = buffer[offset];
						break;
					case 2:
						value = buffer[offset] |
							(buffer[offset+1] << 8);
						break;
					case 4:
						value = buffer[offset] |
							((uint32_t) buffer[offset+1] << 8) |
							((uint32_t) buffer[offset+2] << 16) |
							((uint32_t) buffer[offset+3] << 24);
						break;
					default:
						goto error;
				}

				scans_add_write32(scans, 4, value, true);
			}
		}

		int retval = scans_execute(scans);
		if (retval != ERROR_OK) {
			LOG_ERROR("JTAG execute failed: %d", retval);
			goto error;
		}

		int dbus_busy = 0;
		int execute_busy = 0;
		for (unsigned int j = 0; j < batch_size; j++) {
			dbus_status_t status = scans_get_u32(scans, j, DBUS_OP_START,
					DBUS_OP_SIZE);
			switch (status) {
				case DBUS_STATUS_SUCCESS:
					break;
				case DBUS_STATUS_FAILED:
					LOG_ERROR("Debug RAM write failed. Hardware error?");
					goto error;
				case DBUS_STATUS_BUSY:
					dbus_busy++;
					break;
				default:
					LOG_ERROR("Got invalid bus access status: %d", status);
					return ERROR_FAIL;
			}
			int interrupt = scans_get_u32(scans, j, DBUS_DATA_START + 33, 1);
			if (interrupt)
				execute_busy++;
			if (i + j == count + 1)
				result_value = scans_get_u32(scans, j, DBUS_DATA_START, 32);
		}
		if (dbus_busy)
			increase_dbus_busy_delay(target);
		if (execute_busy)
			increase_interrupt_high_delay(target);
		if (dbus_busy || execute_busy) {
			wait_for_debugint_clear(target, false);

			/* Retry.
			 * Set t0 back to what it should have been at the beginning of this
			 * batch. */
			LOG_INFO("Retrying memory write starting from 0x%" TARGET_PRIxADDR
					" with more delays", address + size * i);

			cache_clean(target);

			if (write_gpr(target, T0, address + size * i) != ERROR_OK)
				goto error;

			if (setup_write_memory(target, size) != ERROR_OK)
				goto error;
		} else {
			i += batch_size;
		}
	}

	if (result_value != 0) {
		LOG_ERROR("Core got an exception (0x%x) while writing to 0x%"
				TARGET_PRIxADDR, result_value, address + size * (count-1));
		if (count > 1) {
			LOG_ERROR("(It may have failed between 0x%" TARGET_PRIxADDR
					" and 0x%" TARGET_PRIxADDR " as well, but we "
					"didn't check then.)",
					address, address + size * (count-2) + size - 1);
		}
		goto error;
	}

	scans_delete(scans);
	cache_clean(target);
	return register_write(target, T0, t0);

error:
	scans_delete(scans);
	cache_clean(target);
	return ERROR_FAIL;
}

static int arch_state(struct target *target)
{
	return ERROR_OK;
}

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	riscv_info_t *generic_info = (riscv_info_t *)target->arch_info;
	generic_info->get_register = get_register;
	generic_info->set_register = set_register;
	generic_info->read_memory = read_memory;

	generic_info->version_specific = calloc(1, sizeof(riscv011_info_t));
	if (!generic_info->version_specific)
		return ERROR_FAIL;

	/* Assume 32-bit until we discover the real value in examine(). */
	generic_info->xlen[0] = 32;
	riscv_init_registers(target);

	return ERROR_OK;
}

struct target_type riscv011_target = {
	.name = "riscv",

	.init_target = init_target,
	.deinit_target = deinit_target,
	.examine = examine,

	/* poll current target status */
	.poll = riscv011_poll,

	.halt = halt,
	.resume = riscv011_resume,
	.step = step,

	.assert_reset = assert_reset,
	.deassert_reset = deassert_reset,

	.write_memory = write_memory,

	.arch_state = arch_state,
};
