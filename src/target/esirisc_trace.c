/***************************************************************************
 *   Copyright (C) 2018 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
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

#include <helper/binarybuffer.h>
#include <helper/command.h>
#include <helper/fileio.h>
#include <helper/log.h>
#include <helper/types.h>
#include <target/target.h>

#include "esirisc.h"

#define BIT_MASK(x)			((1 << (x)) - 1)

/* Control Fields */
#define CONTROL_ST			(1<<0)					/* Start */
#define CONTROL_SP			(1<<1)					/* Stop */
#define CONTROL_W			(1<<2)					/* Wrap */
#define CONTROL_FC			(1<<3)					/* Flow Control */
#define CONTROL_FMT(x)		(((x) <<  4) & 0x30)	/* Format */
#define CONTROL_PCB(x)		(((x) << 10) & 0x7c00)	/* PC Bits */

/* Status Fields */
#define STATUS_T			(1<<0)	/* Trace Started */
#define STATUS_TD			(1<<1)	/* Trace Disabled */
#define	STATUS_W			(1<<2)	/* Wrapped */
#define STATUS_O			(1<<3)	/* Overflow */

/* Trigger Fields */
#define TRIGGER_TST(x)		(((x) << 0) & 0xf)		/* Trigger Start */
#define TRIGGER_DST			(1<<7)					/* Delay Start */
#define TRIGGER_TSP(x)		(((x) << 8) & 0xf00)	/* Trigger Stop */
#define TRIGGER_DSP			(1<<15)					/* Delay Start */

static const char * const esirisc_trace_delay_strings[] = {
	"none", "start", "stop", "both",
};

static const char * const esirisc_trace_format_strings[] = {
	"full", "branch", "icache",
};

static const char * const esirisc_trace_id_strings[] = {
	"sequential instruction",
	"pipeline stall",
	"direct branch",
	"extended ID",
};

static const char * const esirisc_trace_ext_id_strings[] = {
	"",	/* unused */
	"exception",
	"eret",
	"stop instruction",
	"wait instruction",
	"multicycle instruction",
	"count",
	"initial",
	"indirect branch",
	"end of trace",
	"final",
};

static const char * const esirisc_trace_trigger_strings[] = {
	"none", "pc", "load", "store", "exception", "eret", "wait", "stop",
	"high", "low",	/* start only */
};

static int esirisc_trace_clear_status(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_STATUS, ~0);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: Status", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_trace_get_status(struct target *target, uint32_t *status)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	int retval = esirisc_jtag_read_csr(jtag_info, CSR_TRACE, CSR_TRACE_STATUS, status);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Trace CSR: Status", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_trace_start(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t control;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = esirisc_jtag_read_csr(jtag_info, CSR_TRACE, CSR_TRACE_CONTROL, &control);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Trace CSR: Control", target_name(target));
		return retval;
	}

	control |= CONTROL_ST;

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_CONTROL, control);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: Control", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_trace_stop(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	uint32_t control;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = esirisc_jtag_read_csr(jtag_info, CSR_TRACE, CSR_TRACE_CONTROL, &control);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Trace CSR: Control", target_name(target));
		return retval;
	}

	control |= CONTROL_SP;

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_CONTROL, control);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: Control", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_trace_init(struct target *target)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	uint32_t control, trigger;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* stop if running and clear status */
	retval = esirisc_trace_stop(target);
	if (retval != ERROR_OK)
		return retval;

	retval = esirisc_trace_clear_status(target);
	if (retval != ERROR_OK)
		return retval;

	/* initialize Control CSR */
	control = CONTROL_FMT(trace_info->format)
			| CONTROL_PCB(trace_info->pc_bits);

	if (trace_info->buffer_wrap)
		control |= CONTROL_W;

	if (trace_info->flow_control)
		control |= CONTROL_FC;

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_CONTROL, control);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: Control", target_name(target));
		return retval;
	}

	/* initialize buffer CSRs */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_BUFFER_START,
			trace_info->buffer_start);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: BufferStart", target_name(target));
		return retval;
	}

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_BUFFER_END,
			trace_info->buffer_end);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: BufferEnd", target_name(target));
		return retval;
	}

	/*
	 * The BufferCurrent CSR must be initialized to the same value as
	 * BufferStart before tracing can be enabled:
	 */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_BUFFER_CUR,
			trace_info->buffer_start);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: BufferCurrent", target_name(target));
		return retval;
	}

	/* initialize Trigger CSR */
	trigger = TRIGGER_TST(trace_info->start_trigger)
			| TRIGGER_TSP(trace_info->stop_trigger);

	if (trace_info->delay == ESIRISC_TRACE_DELAY_START
		|| trace_info->delay == ESIRISC_TRACE_DELAY_BOTH) {
		trigger |= TRIGGER_DST;
	}

	if (trace_info->delay == ESIRISC_TRACE_DELAY_STOP
		|| trace_info->delay == ESIRISC_TRACE_DELAY_BOTH) {
		trigger |= TRIGGER_DSP;
	}

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_TRIGGER, trigger);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: Trigger", target_name(target));
		return retval;
	}

	/* initialize StartData/StartMask CSRs */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_START_DATA,
			trace_info->start_data);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: StartData", target_name(target));
		return retval;
	}

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_START_MASK,
			trace_info->start_mask);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: StartMask", target_name(target));
		return retval;
	}

	/* initialize StopData/StopMask CSRs */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_STOP_DATA,
			trace_info->stop_data);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: StopData", target_name(target));
		return retval;
	}

	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_STOP_MASK,
			trace_info->stop_mask);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: StopMask", target_name(target));
		return retval;
	}

	/* initialize Delay CSR */
	retval = esirisc_jtag_write_csr(jtag_info, CSR_TRACE, CSR_TRACE_DELAY,
			trace_info->delay_cycles);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to write Trace CSR: Delay", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_trace_buf_get_u32(uint8_t *buffer, uint32_t size,
		unsigned *pos, unsigned count, uint32_t *value)
{
	const unsigned num_bits = size * 8;

	if (*pos+count > num_bits)
		return ERROR_FAIL;

	*value = buf_get_u32(buffer, *pos, count);
	*pos += count;

	return ERROR_OK;
}

static int esirisc_trace_buf_get_pc(struct target *target, uint8_t *buffer, uint32_t size,
		unsigned *pos, uint32_t *value)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	int retval;

	retval = esirisc_trace_buf_get_u32(buffer, size, pos, trace_info->pc_bits, value);
	if (retval != ERROR_OK)
		return retval;

	*value <<= esirisc->num_bits - trace_info->pc_bits;

	return ERROR_OK;
}

static int esirisc_trace_read_memory(struct target *target, target_addr_t address, uint32_t size,
		uint8_t *buffer)
{
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = target_read_memory(target, address, 1, size, buffer);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read trace data", target_name(target));
		return retval;
	}

	return ERROR_OK;
}

static int esirisc_trace_read_buffer(struct target *target, uint8_t *buffer)
{
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_jtag *jtag_info = &esirisc->jtag_info;
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	uint32_t buffer_cur, status;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = esirisc_jtag_read_csr(jtag_info, CSR_TRACE, CSR_TRACE_BUFFER_CUR, &buffer_cur);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to read Trace CSR: BufferCurrent", target_name(target));
		return retval;
	}

	/*
	 * If the buffer has wrapped, the BufferCurrent CSR indicates the
	 * next address to be written (ie. the start address). These bytes
	 * must be dumped first to maintain coherency when analyzing
	 * captured data.
	 */
	retval = esirisc_trace_get_status(target, &status);
	if (retval != ERROR_OK)
		return retval;

	if (status & STATUS_W) {
		uint32_t size = trace_info->buffer_end - buffer_cur;

		retval = esirisc_trace_read_memory(target, buffer_cur, size, buffer);
		if (retval != ERROR_OK)
			return retval;

		buffer += size;
	}

	return esirisc_trace_read_memory(target, trace_info->buffer_start,
			buffer_cur - trace_info->buffer_start, buffer);
}

static int esirisc_trace_analyze_full(struct command_invocation *cmd, uint8_t *buffer, uint32_t size)
{
	struct target *target = get_current_target(cmd->ctx);
	const uint32_t num_bits = size * 8;
	int retval;

	unsigned pos = 0;
	while (pos < num_bits) {
		uint32_t id;

		retval = esirisc_trace_buf_get_u32(buffer, size, &pos, 2, &id);
		if (retval != ERROR_OK)
			goto fail;

		switch (id) {
			case ESIRISC_TRACE_ID_EXECUTE:
			case ESIRISC_TRACE_ID_STALL:
			case ESIRISC_TRACE_ID_BRANCH:
				command_print(cmd, "%s", esirisc_trace_id_strings[id]);
				break;

			case ESIRISC_TRACE_ID_EXTENDED: {
				uint32_t ext_id;

				retval = esirisc_trace_buf_get_u32(buffer, size, &pos, 4, &ext_id);
				if (retval != ERROR_OK)
					goto fail;

				switch (ext_id) {
					case ESIRISC_TRACE_EXT_ID_STOP:
					case ESIRISC_TRACE_EXT_ID_WAIT:
					case ESIRISC_TRACE_EXT_ID_MULTICYCLE:
						command_print(cmd, "%s", esirisc_trace_ext_id_strings[ext_id]);
						break;

					case ESIRISC_TRACE_EXT_ID_ERET:
					case ESIRISC_TRACE_EXT_ID_PC:
					case ESIRISC_TRACE_EXT_ID_INDIRECT:
					case ESIRISC_TRACE_EXT_ID_END_PC: {
						uint32_t pc;

						retval = esirisc_trace_buf_get_pc(target, buffer, size, &pos, &pc);
						if (retval != ERROR_OK)
							goto fail;

						command_print(cmd, "%s PC: 0x%" PRIx32,
								esirisc_trace_ext_id_strings[ext_id], pc);

						if (ext_id == ESIRISC_TRACE_EXT_ID_END_PC) {
							command_print(cmd, "--- end of trace ---");
							return ERROR_OK;
						}
						break;
					}
					case ESIRISC_TRACE_EXT_ID_EXCEPTION: {
						uint32_t eid, epc;

						retval = esirisc_trace_buf_get_u32(buffer, size, &pos, 6, &eid);
						if (retval != ERROR_OK)
							goto fail;

						retval = esirisc_trace_buf_get_pc(target, buffer, size, &pos, &epc);
						if (retval != ERROR_OK)
							goto fail;

						command_print(cmd, "%s EID: 0x%" PRIx32 ", EPC: 0x%" PRIx32,
								esirisc_trace_ext_id_strings[ext_id], eid, epc);
						break;
					}
					case ESIRISC_TRACE_EXT_ID_COUNT: {
						uint32_t count;

						retval = esirisc_trace_buf_get_u32(buffer, size, &pos, 6, &count);
						if (retval != ERROR_OK)
							goto fail;

						command_print(cmd, "repeats %" PRIu32 " %s", count,
								(count == 1) ? "time" : "times");
						break;
					}
					case ESIRISC_TRACE_EXT_ID_END:
						command_print(cmd, "--- end of trace ---");
						return ERROR_OK;

					default:
						command_print(cmd, "invalid extended trace ID: %" PRIu32, ext_id);
						return ERROR_FAIL;
				}
				break;
			}
			default:
				command_print(cmd, "invalid trace ID: %" PRIu32, id);
				return ERROR_FAIL;
		}
	}

fail:
	command_print(cmd, "trace buffer too small");
	return ERROR_BUF_TOO_SMALL;
}

static int esirisc_trace_analyze_simple(struct command_invocation *cmd, uint8_t *buffer, uint32_t size)
{
	struct target *target = get_current_target(cmd->ctx);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	const uint32_t end_of_trace = BIT_MASK(trace_info->pc_bits) << 1;
	const uint32_t num_bits = size * 8;
	int retval;

	unsigned pos = 0;
	while (pos < num_bits) {
		uint32_t pc;

		retval = esirisc_trace_buf_get_pc(target, buffer, size, &pos, &pc);
		if (retval != ERROR_OK)
			break;

		if (pc == end_of_trace) {
			command_print(cmd, "--- end of trace ---");
			return ERROR_OK;
		}

		command_print(cmd, "PC: 0x%" PRIx32, pc);
	}

	command_print(cmd, "trace buffer too small");
	return ERROR_BUF_TOO_SMALL;
}

static int esirisc_trace_analyze(struct command_invocation *cmd, uint8_t *buffer, uint32_t size)
{
	struct target *target = get_current_target(cmd->ctx);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;

	switch (trace_info->format) {
		case ESIRISC_TRACE_FORMAT_FULL:
			command_print(cmd, "--- full pipeline ---");
			return esirisc_trace_analyze_full(cmd, buffer, size);

		case ESIRISC_TRACE_FORMAT_BRANCH:
			command_print(cmd, "--- branches taken ---");
			return esirisc_trace_analyze_full(cmd, buffer, size);

		case ESIRISC_TRACE_FORMAT_ICACHE:
			command_print(cmd, "--- icache misses ---");
			return esirisc_trace_analyze_simple(cmd, buffer, size);

		default:
			command_print(cmd, "invalid trace format: %i", trace_info->format);
			return ERROR_FAIL;
	}
}

static int esirisc_trace_analyze_buffer(struct command_invocation *cmd)
{
	struct target *target = get_current_target(cmd->ctx);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	uint8_t *buffer;
	uint32_t size;
	int retval;

	size = esirisc_trace_buffer_size(trace_info);
	buffer = calloc(1, size);
	if (buffer == NULL) {
		command_print(cmd, "out of memory");
		return ERROR_FAIL;
	}

	retval = esirisc_trace_read_buffer(target, buffer);
	if (retval != ERROR_OK)
		goto done;

	retval = esirisc_trace_analyze(cmd, buffer, size);

done:
	free(buffer);

	return retval;
}

static int esirisc_trace_analyze_memory(struct command_invocation *cmd,
		target_addr_t address, uint32_t size)
{
	struct target *target = get_current_target(cmd->ctx);
	uint8_t *buffer;
	int retval;

	buffer = calloc(1, size);
	if (buffer == NULL) {
		command_print(cmd, "out of memory");
		return ERROR_FAIL;
	}

	retval = esirisc_trace_read_memory(target, address, size, buffer);
	if (retval != ERROR_OK)
		goto done;

	retval = esirisc_trace_analyze(cmd, buffer, size);

done:
	free(buffer);

	return retval;
}

static int esirisc_trace_dump(struct command_invocation *cmd, const char *filename,
		uint8_t *buffer, uint32_t size)
{
	struct fileio *fileio;
	size_t size_written;
	int retval;

	retval = fileio_open(&fileio, filename, FILEIO_WRITE, FILEIO_BINARY);
	if (retval != ERROR_OK) {
		command_print(cmd, "could not open dump file: %s", filename);
		return retval;
	}

	retval = fileio_write(fileio, size, buffer, &size_written);
	if (retval == ERROR_OK)
		command_print(cmd, "trace data dumped to: %s", filename);
	else
		command_print(cmd, "could not write dump file: %s", filename);

	fileio_close(fileio);

	return retval;
}

static int esirisc_trace_dump_buffer(struct command_invocation *cmd, const char *filename)
{
	struct target *target = get_current_target(cmd->ctx);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	uint8_t *buffer;
	uint32_t size;
	int retval;

	size = esirisc_trace_buffer_size(trace_info);
	buffer = calloc(1, size);
	if (buffer == NULL) {
		command_print(cmd, "out of memory");
		return ERROR_FAIL;
	}

	retval = esirisc_trace_read_buffer(target, buffer);
	if (retval != ERROR_OK)
		goto done;

	retval = esirisc_trace_dump(cmd, filename, buffer, size);

done:
	free(buffer);

	return retval;
}

static int esirisc_trace_dump_memory(struct command_invocation *cmd, const char *filename,
		target_addr_t address, uint32_t size)
{
	struct target *target = get_current_target(cmd->ctx);
	uint8_t *buffer;
	int retval;

	buffer = calloc(1, size);
	if (buffer == NULL) {
		command_print(cmd, "out of memory");
		return ERROR_FAIL;
	}

	retval = esirisc_trace_read_memory(target, address, size, buffer);
	if (retval != ERROR_OK)
		goto done;

	retval = esirisc_trace_dump(cmd, filename, buffer, size);

done:
	free(buffer);

	return retval;
}

COMMAND_HANDLER(handle_esirisc_trace_init_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);

	if (!esirisc->has_trace) {
		command_print(CMD, "target does not support trace");
		return ERROR_FAIL;
	}

	int retval = esirisc_trace_init(target);
	if (retval == ERROR_OK)
		command_print(CMD, "trace initialized");

	return retval;
}

COMMAND_HANDLER(handle_esirisc_trace_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;

	if (!esirisc->has_trace) {
		command_print(CMD, "target does not support trace");
		return ERROR_FAIL;
	}

	if (esirisc_trace_is_fifo(trace_info))
		command_print(CMD, "trace FIFO address: 0x%" TARGET_PRIxADDR,
				trace_info->buffer_start);
	else {
		command_print(CMD, "trace buffer start: 0x%" TARGET_PRIxADDR,
				trace_info->buffer_start);
		command_print(CMD, "trace buffer end: 0x%" TARGET_PRIxADDR,
				trace_info->buffer_end);
		command_print(CMD, "trace buffer will %swrap",
				trace_info->buffer_wrap ? "" : "not ");
	}

	command_print(CMD, "flow control: %s",
			trace_info->flow_control ? "enabled" : "disabled");

	command_print(CMD, "trace format: %s",
			esirisc_trace_format_strings[trace_info->format]);
	command_print(CMD, "number of PC bits: %i", trace_info->pc_bits);

	command_print(CMD, "start trigger: %s",
			esirisc_trace_trigger_strings[trace_info->start_trigger]);
	command_print(CMD, "start data: 0x%" PRIx32, trace_info->start_data);
	command_print(CMD, "start mask: 0x%" PRIx32, trace_info->start_mask);

	command_print(CMD, "stop trigger: %s",
			esirisc_trace_trigger_strings[trace_info->stop_trigger]);
	command_print(CMD, "stop data: 0x%" PRIx32, trace_info->stop_data);
	command_print(CMD, "stop mask: 0x%" PRIx32, trace_info->stop_mask);

	command_print(CMD, "trigger delay: %s",
			esirisc_trace_delay_strings[trace_info->delay]);
	command_print(CMD, "trigger delay cycles: %" PRIu32, trace_info->delay_cycles);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_status_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	uint32_t status;

	if (!esirisc->has_trace) {
		command_print(CMD, "target does not support trace");
		return ERROR_FAIL;
	}

	int retval = esirisc_trace_get_status(target, &status);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "trace is %s%s%s%s",
			(status & STATUS_T)  ? "started" : "stopped",
			(status & STATUS_TD) ? ", disabled"   : "",
			(status & STATUS_W)  ? ", wrapped"    : "",
			(status & STATUS_O)  ? ", overflowed" : "");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_start_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);

	if (!esirisc->has_trace) {
		command_print(CMD, "target does not support trace");
		return ERROR_FAIL;
	}

	int retval = esirisc_trace_start(target);
	if (retval == ERROR_OK)
		command_print(CMD, "trace started");

	return retval;
}

COMMAND_HANDLER(handle_esirisc_trace_stop_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);

	if (!esirisc->has_trace) {
		command_print(CMD, "target does not support trace");
		return ERROR_FAIL;
	}

	int retval = esirisc_trace_stop(target);
	if (retval == ERROR_OK)
		command_print(CMD, "trace stopped");

	return retval;
}

COMMAND_HANDLER(handle_esirisc_trace_analyze_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	target_addr_t address;
	uint32_t size;

	if (!esirisc->has_trace) {
		command_print(CMD, "target does not support trace");
		return ERROR_FAIL;
	}

	if (CMD_ARGC != 0 && CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 0) {
		/*
		 * Use of the Trace FIFO typically involves DMA to a peripheral
		 * (eg. SPI) or a separately managed buffer in memory, neither
		 * of which may be under our control. If the destination address
		 * and size are known in the latter case, they may be specified
		 * as arguments as a workaround.
		 */
		if (esirisc_trace_is_fifo(trace_info)) {
			command_print(CMD, "analyze from FIFO not supported");
			return ERROR_FAIL;
		}

		return esirisc_trace_analyze_buffer(CMD);
	} else {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[0], address);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);

		return esirisc_trace_analyze_memory(CMD, address, size);
	}
}

COMMAND_HANDLER(handle_esirisc_trace_dump_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	target_addr_t address;
	uint32_t size;

	if (!esirisc->has_trace) {
		command_print(CMD, "target does not support trace");
		return ERROR_FAIL;
	}

	if (CMD_ARGC != 1 && CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1) {
		/* also see: handle_esirisc_trace_analyze_command() */
		if (esirisc_trace_is_fifo(trace_info)) {
			command_print(CMD, "dump from FIFO not supported");
			return ERROR_FAIL;
		}

		return esirisc_trace_dump_buffer(CMD, CMD_ARGV[0]);
	} else {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[0], address);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);

		return esirisc_trace_dump_memory(CMD, CMD_ARGV[2], address, size);
	}
}

COMMAND_HANDLER(handle_esirisc_trace_buffer_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	uint32_t size;

	if (CMD_ARGC < 2 || CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], trace_info->buffer_start);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], size);

	trace_info->buffer_end = trace_info->buffer_start + size;

	if (CMD_ARGC == 3) {
		if (strcmp("wrap", CMD_ARGV[2]) == 0)
			trace_info->buffer_wrap = true;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_fifo_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], trace_info->buffer_start);

	/* FIFOs have the same start and end address */
	trace_info->buffer_end = trace_info->buffer_start;
	trace_info->buffer_wrap = true;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_flow_control_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[0], "enable") == 0)
		trace_info->flow_control = true;
	else if (strcmp(CMD_ARGV[0], "disable") == 0)
		trace_info->flow_control = false;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_format_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;
	int pc_bits;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[0], "full") == 0)
		trace_info->format = ESIRISC_TRACE_FORMAT_FULL;
	else if (strcmp(CMD_ARGV[0], "branch") == 0)
		trace_info->format = ESIRISC_TRACE_FORMAT_BRANCH;
	else if (strcmp(CMD_ARGV[0], "icache") == 0)
		trace_info->format = ESIRISC_TRACE_FORMAT_ICACHE;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], pc_bits);

	if (pc_bits < 1 || pc_bits > 31) {
		command_print(CMD, "invalid pc_bits: %i; must be 1..31", pc_bits);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	trace_info->pc_bits = pc_bits;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_trigger_start_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;

	if (CMD_ARGC != 1 && CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[0], "none") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_NONE;
	else if (strcmp(CMD_ARGV[0], "pc") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_PC;
	else if (strcmp(CMD_ARGV[0], "load") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_LOAD;
	else if (strcmp(CMD_ARGV[0], "store") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_STORE;
	else if (strcmp(CMD_ARGV[0], "exception") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_EXCEPTION;
	else if (strcmp(CMD_ARGV[0], "eret") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_ERET;
	else if (strcmp(CMD_ARGV[0], "wait") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_WAIT;
	else if (strcmp(CMD_ARGV[0], "stop") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_STOP;
	else if (strcmp(CMD_ARGV[0], "high") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_HIGH;
	else if (strcmp(CMD_ARGV[0], "low") == 0)
		trace_info->start_trigger = ESIRISC_TRACE_TRIGGER_LOW;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 3) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], trace_info->start_data);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], trace_info->start_mask);
	} else {
		trace_info->start_data = 0;
		trace_info->start_mask = 0;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_trigger_stop_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;

	if (CMD_ARGC != 1 && CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[0], "none") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_NONE;
	else if (strcmp(CMD_ARGV[0], "pc") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_PC;
	else if (strcmp(CMD_ARGV[0], "load") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_LOAD;
	else if (strcmp(CMD_ARGV[0], "store") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_STORE;
	else if (strcmp(CMD_ARGV[0], "exception") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_EXCEPTION;
	else if (strcmp(CMD_ARGV[0], "eret") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_ERET;
	else if (strcmp(CMD_ARGV[0], "wait") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_WAIT;
	else if (strcmp(CMD_ARGV[0], "stop") == 0)
		trace_info->stop_trigger = ESIRISC_TRACE_TRIGGER_STOP;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 3) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], trace_info->stop_data);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], trace_info->stop_mask);
	} else {
		trace_info->stop_data = 0;
		trace_info->stop_mask = 0;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_trace_trigger_delay_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esirisc_common *esirisc = target_to_esirisc(target);
	struct esirisc_trace *trace_info = &esirisc->trace_info;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[0], "none") == 0)
		trace_info->delay = ESIRISC_TRACE_DELAY_NONE;
	else if (strcmp(CMD_ARGV[0], "start") == 0)
		trace_info->delay = ESIRISC_TRACE_DELAY_START;
	else if (strcmp(CMD_ARGV[0], "stop") == 0)
		trace_info->delay = ESIRISC_TRACE_DELAY_STOP;
	else if (strcmp(CMD_ARGV[0], "both") == 0)
		trace_info->delay = ESIRISC_TRACE_DELAY_BOTH;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (trace_info->delay == ESIRISC_TRACE_DELAY_NONE)
		trace_info->delay_cycles = 0;
	else {
		if (CMD_ARGC != 2)
			return ERROR_COMMAND_SYNTAX_ERROR;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], trace_info->delay_cycles);
	}

	return ERROR_OK;
}

static const struct command_registration esirisc_trace_exec_command_handlers[] = {
	{
		.name = "init",
		.handler = handle_esirisc_trace_init_command,
		.mode = COMMAND_EXEC,
		.help = "initialize trace collection",
		.usage = "",
	},
	{
		.name = "info",
		.handler = handle_esirisc_trace_info_command,
		.mode = COMMAND_EXEC,
		.help = "display trace configuration",
		.usage = "",
	},
	{
		.name = "status",
		.handler = handle_esirisc_trace_status_command,
		.mode = COMMAND_EXEC,
		.help = "display trace collection status",
		.usage = "",
	},
	{
		.name = "start",
		.handler = handle_esirisc_trace_start_command,
		.mode = COMMAND_EXEC,
		.help = "start trace collection",
		.usage = "",
	},
	{
		.name = "stop",
		.handler = handle_esirisc_trace_stop_command,
		.mode = COMMAND_EXEC,
		.help = "stop trace collection",
		.usage = "",
	},
	{
		.name = "analyze",
		.handler = handle_esirisc_trace_analyze_command,
		.mode = COMMAND_EXEC,
		.usage = "[address size]",
		.help = "analyze collected trace data",
	},
	{
		.name = "dump",
		.handler = handle_esirisc_trace_dump_command,
		.mode = COMMAND_EXEC,
		.help = "dump collected trace data to file",
		.usage = "[address size] filename",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esirisc_trace_trigger_any_command_handlers[] = {
	{
		.name = "start",
		.handler = handle_esirisc_trace_trigger_start_command,
		.mode = COMMAND_ANY,
		.help = "configure trigger start condition",
		.usage = "('none'|'pc'|'load'|'store'|'exception'|'eret'|'wait'|'stop'|'high'|'low')"
				 " [start_data start_mask]",
	},
	{
		.name = "stop",
		.handler = handle_esirisc_trace_trigger_stop_command,
		.mode = COMMAND_ANY,
		.help = "configure trigger stop condition",
		.usage = "('none'|'pc'|'load'|'store'|'exception'|'eret'|'wait'|'stop')"
				 " [stop_data stop_mask]",
	},
	{
		.name = "delay",
		.handler = handle_esirisc_trace_trigger_delay_command,
		.mode = COMMAND_ANY,
		.help = "configure trigger start/stop delay in clock cycles",
		.usage = "('none'|'start'|'stop'|'both') [cycles]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esirisc_trace_any_command_handlers[] = {
	{
		.name = "buffer",
		.handler = handle_esirisc_trace_buffer_command,
		.mode = COMMAND_ANY,
		.help = "configure trace buffer",
		.usage = "address size ['wrap']",
	},
	{
		.name = "fifo",
		.handler = handle_esirisc_trace_fifo_command,
		.mode = COMMAND_ANY,
		.help = "configure trace FIFO",
		.usage = "address",
	},
	{
		.name = "flow_control",
		.handler = handle_esirisc_trace_flow_control_command,
		.mode = COMMAND_ANY,
		.help = "enable or disable stalling CPU to collect trace data",
		.usage = "('enable'|'disable')",
	},
	{
		.name = "format",
		.handler = handle_esirisc_trace_format_command,
		.mode = COMMAND_ANY,
		.help = "configure trace format",
		.usage = "('full'|'branch'|'icache') pc_bits",
	},
	{
		.name = "trigger",
		.mode = COMMAND_ANY,
		.help = "eSi-Trace trigger command group",
		.usage = "",
		.chain = esirisc_trace_trigger_any_command_handlers,
	},
	{
		.chain = esirisc_trace_exec_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration esirisc_trace_command_handlers[] = {
	{
		.name = "trace",
		.mode = COMMAND_ANY,
		.help = "eSi-Trace command group",
		.usage = "",
		.chain = esirisc_trace_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
