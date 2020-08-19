/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "batch.h"
#include "debug_defines.h"
#include "riscv.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DTM_DMI_MAX_ADDRESS_LENGTH	((1<<DTM_DTMCS_ABITS_LENGTH)-1)
#define DMI_SCAN_MAX_BIT_LENGTH (DTM_DMI_MAX_ADDRESS_LENGTH + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH)
#define DMI_SCAN_BUF_SIZE (DIV_ROUND_UP(DMI_SCAN_MAX_BIT_LENGTH, 8))

static void dump_field(int idle, const struct scan_field *field);

struct riscv_batch *riscv_batch_alloc(struct target *target, size_t scans, size_t idle)
{
	scans += 4;
	struct riscv_batch *out = calloc(1, sizeof(*out));
	if (!out)
		goto error0;
	out->target = target;
	out->allocated_scans = scans;
	out->idle_count = idle;
	out->data_out = malloc(sizeof(*out->data_out) * (scans) * DMI_SCAN_BUF_SIZE);
	if (!out->data_out)
		goto error1;
	out->data_in = malloc(sizeof(*out->data_in) * (scans) * DMI_SCAN_BUF_SIZE);
	if (!out->data_in)
		goto error2;
	out->fields = malloc(sizeof(*out->fields) * (scans));
	if (!out->fields)
		goto error3;
	if (bscan_tunnel_ir_width != 0) {
		out->bscan_ctxt = malloc(sizeof(*out->bscan_ctxt) * (scans));
		if (!out->bscan_ctxt)
			goto error4;
	}
	out->last_scan = RISCV_SCAN_TYPE_INVALID;
	out->read_keys = malloc(sizeof(*out->read_keys) * (scans));
	if (!out->read_keys)
		goto error5;
	return out;

error5:
	free(out->bscan_ctxt);
error4:
	free(out->fields);
error3:
	free(out->data_in);
error2:
	free(out->data_out);
error1:
	free(out);
error0:
	return NULL;
}

void riscv_batch_free(struct riscv_batch *batch)
{
	free(batch->data_in);
	free(batch->data_out);
	free(batch->fields);
	free(batch->bscan_ctxt);
	free(batch->read_keys);
	free(batch);
}

bool riscv_batch_full(struct riscv_batch *batch)
{
	return batch->used_scans > (batch->allocated_scans - 4);
}

int riscv_batch_run(struct riscv_batch *batch)
{
	if (batch->used_scans == 0) {
		LOG_DEBUG("Ignoring empty batch.");
		return ERROR_OK;
	}

	keep_alive();

	riscv_batch_add_nop(batch);

	for (size_t i = 0; i < batch->used_scans; ++i) {
		if (bscan_tunnel_ir_width != 0)
			riscv_add_bscan_tunneled_scan(batch->target, batch->fields+i, batch->bscan_ctxt+i);
		else
			jtag_add_dr_scan(batch->target->tap, 1, batch->fields + i, TAP_IDLE);

		if (batch->idle_count > 0)
			jtag_add_runtest(batch->idle_count, TAP_IDLE);
	}

	if (jtag_execute_queue() != ERROR_OK) {
		LOG_ERROR("Unable to execute JTAG queue");
		return ERROR_FAIL;
	}

	if (bscan_tunnel_ir_width != 0) {
		/* need to right-shift "in" by one bit, because of clock skew between BSCAN TAP and DM TAP */
		for (size_t i = 0; i < batch->used_scans; ++i)
			buffer_shr((batch->fields + i)->in_value, DMI_SCAN_BUF_SIZE, 1);
	}

	for (size_t i = 0; i < batch->used_scans; ++i)
		dump_field(batch->idle_count, batch->fields + i);

	return ERROR_OK;
}

void riscv_batch_add_dmi_write(struct riscv_batch *batch, unsigned address, uint64_t data)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE);
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * DMI_SCAN_BUF_SIZE);
	riscv_fill_dmi_write_u64(batch->target, (char *)field->out_value, address, data);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_WRITE;
	batch->used_scans++;
}

size_t riscv_batch_add_dmi_read(struct riscv_batch *batch, unsigned address)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE);
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * DMI_SCAN_BUF_SIZE);
	riscv_fill_dmi_read_u64(batch->target, (char *)field->out_value, address);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_READ;
	batch->used_scans++;

	batch->read_keys[batch->read_keys_used] = batch->used_scans;
	return batch->read_keys_used++;
}

unsigned riscv_batch_get_dmi_read_op(struct riscv_batch *batch, size_t key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index <= batch->used_scans);
	uint8_t *base = batch->data_in + DMI_SCAN_BUF_SIZE * index;
	/* extract "op" field from the DMI read result */
	return (unsigned)buf_get_u32(base, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

uint32_t riscv_batch_get_dmi_read_data(struct riscv_batch *batch, size_t key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index <= batch->used_scans);
	uint8_t *base = batch->data_in + DMI_SCAN_BUF_SIZE * index;
	/* extract "data" field from the DMI read result */
	return buf_get_u32(base, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
}

void riscv_batch_add_nop(struct riscv_batch *batch)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE);
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * DMI_SCAN_BUF_SIZE);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->out_value);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_NOP;
	batch->used_scans++;
}

void dump_field(int idle, const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field->out_value != NULL);
	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	if (field->in_value) {
		uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
		unsigned int in_op = get_field(in, DTM_DMI_OP);
		unsigned int in_data = get_field(in, DTM_DMI_DATA);
		unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;

		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__,
				"%db %s %08x @%02x -> %s %08x @%02x; %di",
				field->num_bits, op_string[out_op], out_data, out_address,
				status_string[in_op], in_data, in_address, idle);
	} else {
		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__, "%db %s %08x @%02x -> ?; %di",
				field->num_bits, op_string[out_op], out_data, out_address, idle);
	}
}

size_t riscv_batch_available_scans(struct riscv_batch *batch)
{
	return batch->allocated_scans - batch->used_scans - 4;
}
