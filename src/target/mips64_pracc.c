/*
 * Support for processors implementing MIPS64 instruction set
 *
 *   Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>
 *   Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>
 *   Copyright (C) 2014-2019 by Peter Mamonov <pmamonov@gmail.com>
 *
 *   Based on the work of:
 *       Copyright (C) 2008 by Spencer Oliver
 *       Copyright (C) 2008 by David T.L. Wong
 *       Copyright (C) 2010 by Konstantin Kostyukhin, Nikolay Shmyrev
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mips64.h"
#include "mips64_pracc.h"

#include "time_support.h"

#define STACK_DEPTH	32

typedef struct {
	uint64_t *local_iparam;
	unsigned num_iparam;
	uint64_t *local_oparam;
	unsigned num_oparam;
	const uint32_t *code;
	unsigned code_len;
	uint64_t stack[STACK_DEPTH];
	unsigned stack_offset;
	struct mips_ejtag *ejtag_info;
} mips64_pracc_context;

static int wait_for_pracc_rw(struct mips_ejtag *ejtag_info, uint32_t *ctrl)
{
	uint32_t ejtag_ctrl;
	int nt = 5;
	int rc;

	while (1) {
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		ejtag_ctrl = ejtag_info->ejtag_ctrl;
		rc = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		if (rc != ERROR_OK)
			return rc;

		if (ejtag_ctrl & EJTAG_CTRL_PRACC)
			break;
		LOG_DEBUG("DEBUGMODULE: No memory access in progress!\n");
		if (nt == 0)
			return ERROR_JTAG_DEVICE_ERROR;
		nt--;
	}

	*ctrl = ejtag_ctrl;
	return ERROR_OK;
}

static int mips64_pracc_exec_read(mips64_pracc_context *ctx, uint64_t address)
{
	struct mips_ejtag *ejtag_info = ctx->ejtag_info;
	unsigned offset;
	uint32_t ejtag_ctrl;
	uint64_t data;
	int rc;

	if ((address >= MIPS64_PRACC_PARAM_IN)
	    && (address < MIPS64_PRACC_PARAM_IN + ctx->num_iparam * MIPS64_PRACC_DATA_STEP)) {

		offset = (address - MIPS64_PRACC_PARAM_IN) / MIPS64_PRACC_DATA_STEP;

		if (offset >= MIPS64_PRACC_PARAM_IN_SIZE) {
			LOG_ERROR("Error: iparam size exceeds MIPS64_PRACC_PARAM_IN_SIZE");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		if (ctx->local_iparam == NULL) {
			LOG_ERROR("Error: unexpected reading of input parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->local_iparam[offset];
		LOG_DEBUG("Reading %" PRIx64 " at %" PRIx64, data, address);

	} else if ((address >= MIPS64_PRACC_PARAM_OUT)
		   && (address < MIPS64_PRACC_PARAM_OUT + ctx->num_oparam * MIPS64_PRACC_DATA_STEP)) {

		offset = (address - MIPS64_PRACC_PARAM_OUT) / MIPS64_PRACC_DATA_STEP;
		if (ctx->local_oparam == NULL) {
			LOG_ERROR("Error: unexpected reading of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->local_oparam[offset];
		LOG_DEBUG("Reading %" PRIx64 " at %" PRIx64, data, address);

	} else if ((address >= MIPS64_PRACC_TEXT)
		   && (address < MIPS64_PRACC_TEXT + ctx->code_len * MIPS64_PRACC_ADDR_STEP)) {

		offset = ((address & ~7ull) - MIPS64_PRACC_TEXT) / MIPS64_PRACC_ADDR_STEP;
		data = (uint64_t)ctx->code[offset] << 32;
		if (offset + 1 < ctx->code_len)
			data |= (uint64_t)ctx->code[offset + 1];

		LOG_DEBUG("Running commands %" PRIx64 " at %" PRIx64, data,
			  address);

	} else if ((address & ~7llu) == MIPS64_PRACC_STACK) {

		/* load from our debug stack */
		if (ctx->stack_offset == 0) {
			LOG_ERROR("Error reading from stack: stack is empty");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->stack[--ctx->stack_offset];
		LOG_DEBUG("Reading %" PRIx64 " at %" PRIx64, data, address);

	} else {
		/* TODO: send JMP 0xFF200000 instruction. Hopefully processor jump back
		 * to start of debug vector */

		data = 0;
		LOG_ERROR("Error reading unexpected address %" PRIx64, address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Send the data out */
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_DATA);
	rc = mips_ejtag_drscan_64(ctx->ejtag_info, &data);
	if (rc != ERROR_OK)
		return rc;

	/* Clear the access pending bit (let the processor eat!) */

	ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_CONTROL);
	rc = mips_ejtag_drscan_32(ctx->ejtag_info, &ejtag_ctrl);
	if (rc != ERROR_OK)
		return rc;

	jtag_add_clocks(5);

	return jtag_execute_queue();
}

static int mips64_pracc_exec_write(mips64_pracc_context *ctx, uint64_t address)
{
	uint32_t ejtag_ctrl;
	uint64_t data;
	unsigned offset;
	struct mips_ejtag *ejtag_info = ctx->ejtag_info;
	int rc;

	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_DATA);
	rc = mips_ejtag_drscan_64(ctx->ejtag_info, &data);
	if (rc != ERROR_OK)
		return rc;

	/* Clear access pending bit */
	ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_CONTROL);
	rc = mips_ejtag_drscan_32(ctx->ejtag_info, &ejtag_ctrl);
	if (rc != ERROR_OK)
		return rc;

	jtag_add_clocks(5);
	rc = jtag_execute_queue();
	if (rc != ERROR_OK)
		return rc;

	LOG_DEBUG("Writing %" PRIx64 " at %" PRIx64, data, address);

	if ((address >= MIPS64_PRACC_PARAM_IN)
		&& (address < MIPS64_PRACC_PARAM_IN + ctx->num_iparam * MIPS64_PRACC_DATA_STEP)) {
		offset = (address - MIPS64_PRACC_PARAM_IN) / MIPS64_PRACC_DATA_STEP;
		if (ctx->local_iparam == NULL) {
			LOG_ERROR("Error: unexpected writing of input parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		ctx->local_iparam[offset] = data;
	} else if ((address >= MIPS64_PRACC_PARAM_OUT)
		&& (address < MIPS64_PRACC_PARAM_OUT + ctx->num_oparam * MIPS64_PRACC_DATA_STEP)) {
		offset = (address - MIPS64_PRACC_PARAM_OUT) / MIPS64_PRACC_DATA_STEP;
		if (ctx->local_oparam == NULL) {
			LOG_ERROR("Error: unexpected writing of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		ctx->local_oparam[offset] = data;
	} else if (address == MIPS64_PRACC_STACK) {
		/* save data onto our stack */
		if (ctx->stack_offset >= STACK_DEPTH) {
			LOG_ERROR("Error: PrAcc stack depth exceeded");
			return ERROR_FAIL;
		}
		ctx->stack[ctx->stack_offset++] = data;
	} else {
		LOG_ERROR("Error writing unexpected address 0x%" PRIx64, address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

int mips64_pracc_exec(struct mips_ejtag *ejtag_info,
		      unsigned code_len, const uint32_t *code,
		      unsigned num_param_in, uint64_t *param_in,
		      unsigned num_param_out, uint64_t *param_out)
{
	uint32_t ejtag_ctrl;
	uint64_t address = 0, address_prev = 0, data;
	mips64_pracc_context ctx;
	int retval;
	int pass = 0;
	bool first_time_call = true;
	unsigned i;

	for (i = 0; i < code_len; i++)
		LOG_DEBUG("%08" PRIx32, code[i]);

	ctx.local_iparam = param_in;
	ctx.local_oparam = param_out;
	ctx.num_iparam = num_param_in;
	ctx.num_oparam = num_param_out;
	ctx.code = code;
	ctx.code_len = code_len;
	ctx.ejtag_info = ejtag_info;
	ctx.stack_offset = 0;

	while (true) {
		uint32_t address32;
		retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK) {
			LOG_DEBUG("ERROR wait_for_pracc_rw");
			return retval;
		}
		if (pass)
			address_prev = address;
		else
			address_prev = 0;
		address32 = data = 0;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
		mips_ejtag_drscan_32(ejtag_info, &address32);
		LOG_DEBUG("-> %08" PRIx32, address32);
		address = 0xffffffffff200000ull | address32;

		int psz = (ejtag_ctrl >> 29) & 3;
		int address20 = address & 7;
		switch (psz) {
		case 3:
			if (address20 != 7) {
				LOG_ERROR("PSZ=%d ADDRESS[2:0]=%d: not supported", psz, address20);
				return ERROR_FAIL;
			}
			address &= ~7ull;
			break;
		case 2:
			if (address20 != 0 && address20 != 4) {
				LOG_ERROR("PSZ=%d ADDRESS[2:0]=%d: not supported", psz, address20);
				return ERROR_FAIL;
			}
			break;
		default:
			LOG_ERROR("PSZ=%d ADDRESS[2:0]=%d: not supported", psz, address20);
			return ERROR_FAIL;
		}

		if (first_time_call && address != MIPS64_PRACC_TEXT) {
			LOG_ERROR("Error reading address " TARGET_ADDR_FMT " (0x%08llx expected)",
				address, MIPS64_PRACC_TEXT);
			return ERROR_JTAG_DEVICE_ERROR;
		}

		first_time_call = false;

		/* Check for read or write */
		if (ejtag_ctrl & EJTAG_CTRL_PRNW) {
			retval = mips64_pracc_exec_write(&ctx, address);
			if (retval != ERROR_OK) {
				printf("ERROR mips64_pracc_exec_write\n");
				return retval;
			}
		} else {
			/* Check to see if its reading at the debug vector. The first pass through
			 * the module is always read at the vector, so the first one we allow.  When
			 * the second read from the vector occurs we are done and just exit. */
			if ((address == MIPS64_PRACC_TEXT) && (pass++)) {
				LOG_DEBUG("@MIPS64_PRACC_TEXT, address_prev=%" PRIx64, address_prev);
				break;
			}
			retval = mips64_pracc_exec_read(&ctx, address);
			if (retval != ERROR_OK) {
				printf("ERROR mips64_pracc_exec_read\n");
				return retval;
			}

		}
	}

	/* stack sanity check */
	if (ctx.stack_offset != 0)
		LOG_ERROR("Pracc Stack not zero");

	return ERROR_OK;
}

static int mips64_pracc_read_u64(struct mips_ejtag *ejtag_info, uint64_t addr,
				 uint64_t *buf)
{
	const uint32_t code[] = {
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $8, ($15) */
		MIPS64_SD(8, 0, 15),
		/* load R8 @ param_in[0] = address */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),
		/* ld $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_LD(8, 0, 8),
		/* sd $8, 0($15) */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),
		/* ld $8, ($15) */
		MIPS64_LD(8, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(10)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	uint64_t param_in[1];
	param_in[0] = addr;

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, (uint64_t *) buf);
}

static int mips64_pracc_read_mem64(struct mips_ejtag *ejtag_info, uint64_t addr,
			    unsigned count, uint64_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_read_u64(ejtag_info, addr + 8*i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int mips64_pracc_read_u32(struct mips_ejtag *ejtag_info, uint64_t addr,
				 uint32_t *buf)
{
	const uint32_t code[] = {
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $8, ($15) */
		MIPS64_SD(8, 0, 15),
		/* load R8 @ param_in[0] = address */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),
		/* lw $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_LW(8, 0, 8),
		/* sd $8, 0($9) */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),
		/* ld $8, ($15) */
		MIPS64_LD(8, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(10)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	int retval = ERROR_OK;
	uint64_t param_in[1];
	uint64_t param_out[1];

	param_in[0] = addr;

	LOG_DEBUG("enter mips64_pracc_exec");
	retval = mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		1, param_in, 1, param_out);
	buf[0] = (uint32_t) param_out[0];
	return retval;
}

static int mips64_pracc_read_mem32(struct mips_ejtag *ejtag_info, uint64_t addr,
				   unsigned count, uint32_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_read_u32(ejtag_info, addr + 4 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int mips64_pracc_read_u16(struct mips_ejtag *ejtag_info, uint64_t addr,
				 uint16_t *buf)
{
	const uint32_t code[] = {
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $8, ($15) */
		MIPS64_SD(8, 0, 15),
		/* load R8 @ param_in[0] = address */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),
		/* lw $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_LHU(8, 0, 8),
		/* sd $8, 0($9) */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),
		/* ld $8, ($15) */
		MIPS64_LD(8, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(10)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	int retval;
	uint64_t param_in[1];
	uint64_t param_out[1];

	param_in[0] = addr;

	LOG_DEBUG("enter mips64_pracc_exec");
	retval = mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		1, param_in, 1, param_out);
	buf[0] = (uint16_t)param_out[0];
	return retval;
}

static int mips64_pracc_read_mem16(struct mips_ejtag *ejtag_info, uint64_t addr,
			    unsigned count, uint16_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_read_u16(ejtag_info, addr + 2*i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int mips64_pracc_read_u8(struct mips_ejtag *ejtag_info, uint64_t addr,
				uint8_t *buf)
{
	const uint32_t code[] = {
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $8, ($15) */
		MIPS64_SD(8, 0, 15),
		/* load R8 @ param_in[0] = address */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),
		/* lw $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_LBU(8, 0, 8),
		/* sd $8, 0($9) */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),
		/* ld $8, ($15) */
		MIPS64_LD(8, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(10)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	int retval;
	uint64_t param_in[1];
	uint64_t param_out[1];

	param_in[0] = addr;

	LOG_DEBUG("enter mips64_pracc_exec");
	retval = mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		1, param_in, 1, param_out);
	buf[0] = (uint8_t)param_out[0];
	return retval;
}

static int mips64_pracc_read_mem8(struct mips_ejtag *ejtag_info, uint64_t addr,
			  unsigned count, uint8_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_read_u8(ejtag_info, addr + i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_read_mem(struct mips_ejtag *ejtag_info, uint64_t addr,
			  unsigned size, unsigned count, void *buf)
{
	switch (size) {
	case 1:
		return mips64_pracc_read_mem8(ejtag_info, addr, count, buf);
	case 2:
		return mips64_pracc_read_mem16(ejtag_info, addr, count, buf);
	case 4:
		return mips64_pracc_read_mem32(ejtag_info, addr, count, buf);
	case 8:
		return mips64_pracc_read_mem64(ejtag_info, addr, count, buf);
	}
	return ERROR_FAIL;
}

static int mips64_pracc_write_u64(struct mips_ejtag *ejtag_info, uint64_t addr,
				  uint64_t *buf)
{
	const uint32_t code[] = {
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $8, ($15) */
		MIPS64_SD(8, 0, 15),
		/* sd $9, ($15) */
		MIPS64_SD(9, 0, 15),
		/* load R8 @ param_in[1] = data */
		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN)-8), 15),
		/* load R9 @ param_in[0] = address */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN), 15),
		/* sd $8, 0($9) */
		MIPS64_SD(8, 0, 9),
		MIPS64_SYNCI(9, 0),
		/* ld $9, ($15) */
		MIPS64_LD(9, 0, 15),
		/* ld $8, ($15) */
		MIPS64_LD(8, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(13)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	/* TODO remove array */
	uint64_t param_in[2];
	param_in[0] = addr;
	param_in[1] = *buf;

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int mips64_pracc_write_mem64(struct mips_ejtag *ejtag_info,
			     uint64_t addr, unsigned count, uint64_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_write_u64(ejtag_info, addr + 8 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int mips64_pracc_write_u32(struct mips_ejtag *ejtag_info, uint64_t addr,
				  uint32_t *buf)
{
	const uint32_t code[] = {
		MIPS64_DMTC0(15, 31, 0),
		/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),
		/* sd $8, ($15) */
		MIPS64_SD(9, 0, 15),
		/* sd $9, ($15) */
		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN) - 8), 15),
		/* load R8 @ param_in[1] = data */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN), 15),
		/* load R9 @ param_in[0] = address */
		MIPS64_SW(8, 0, 9),
		/* sw $8, 0($9) */
		MIPS64_SYNCI(9, 0),
		MIPS64_LD(9, 0, 15),
		/* ld $9, ($15) */
		MIPS64_LD(8, 0, 15),
		/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_B(NEG16(13)),
		/* b start */
		MIPS64_DMFC0(15, 31, 0),
		/* move COP0 DeSave to $15 */
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	/* TODO remove array */
	uint64_t param_in[1 + 1];
	param_in[0] = addr;
	param_in[1] = *buf;

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int mips64_pracc_write_mem32(struct mips_ejtag *ejtag_info, uint64_t addr,
			     unsigned count, uint32_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_write_u32(ejtag_info, addr + 4 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int mips64_pracc_write_u16(struct mips_ejtag *ejtag_info, uint64_t addr,
				  uint16_t *buf)
{
	const uint32_t code[] = {
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $8, ($15) */
		MIPS64_SD(8, 0, 15),
		/* sd $9, ($15) */
		MIPS64_SD(9, 0, 15),
		/* load R8 @ param_in[1] = data */
		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN) - 8), 15),
		/* load R9 @ param_in[0] = address */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN), 15),
		/* sh $8, 0($9) */
		MIPS64_SH(8, 0, 9),
		/* ld $9, ($15) */
		MIPS64_LD(9, 0, 15),
		/* ld $8, ($15) */
		MIPS64_LD(8, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(12)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	uint64_t param_in[2];
	param_in[0] = addr;
	param_in[1] = *buf;

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int mips64_pracc_write_mem16(struct mips_ejtag *ejtag_info,
			     uint64_t addr, unsigned count, uint16_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_write_u16(ejtag_info, addr + 2 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int mips64_pracc_write_u8(struct mips_ejtag *ejtag_info, uint64_t addr,
				 uint8_t *buf)
{
	const uint32_t code[] = {
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $8, ($15) */
		MIPS64_SD(8, 0, 15),
		/* sd $9, ($15) */
		MIPS64_SD(9, 0, 15),
		/* load R8 @ param_in[1] = data */
		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN) - 8), 15),
		/* load R9 @ param_in[0] = address */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN), 15),
		/* sh $8, 0($9) */
		MIPS64_SB(8, 0, 9),
		/* ld $9, ($15) */
		MIPS64_LD(9, 0, 15),
		/* ld $8, ($15) */
		MIPS64_LD(8, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(12)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	/* TODO remove array */
	uint64_t param_in[2];
	param_in[0] = addr;
	param_in[1] = *buf;

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int mips64_pracc_write_mem8(struct mips_ejtag *ejtag_info,
			    uint64_t addr, unsigned count, uint8_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned i = 0; i < count; i++) {
		retval = mips64_pracc_write_u8(ejtag_info, addr + i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_write_mem(struct mips_ejtag *ejtag_info,
			   uint64_t addr, unsigned size,
			   unsigned count, void *buf)
{
	switch (size) {
	case 1:
		return mips64_pracc_write_mem8(ejtag_info, addr, count, buf);
	case 2:
		return mips64_pracc_write_mem16(ejtag_info, addr, count, buf);
	case 4:
		return mips64_pracc_write_mem32(ejtag_info, addr, count, buf);
	case 8:
		return mips64_pracc_write_mem64(ejtag_info, addr, count, buf);
	}
	return ERROR_FAIL;
}

int mips64_pracc_write_regs(struct mips_ejtag *ejtag_info, uint64_t *regs)
{
	const uint32_t code[] = {
		/* move $2 to COP0 DeSave */
		MIPS64_DMTC0(2, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(2, UPPER16(MIPS64_PRACC_PARAM_IN)),
		MIPS64_ORI(2, 2, LOWER16(MIPS64_PRACC_PARAM_IN)),
		/* sd $0, 0*8($2) */
		MIPS64_LD(1, 1*8, 2),
		/* sd $1, 1*8($2) */
		MIPS64_LD(15, 15*8, 2),
		/* sd $11, ($15) */
		MIPS64_DMFC0(2, 31, 0),
		MIPS64_DMTC0(15, 31, 0),
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(1, 0, 15),
		/* $11 = MIPS64_PRACC_PARAM_OUT */
		MIPS64_LUI(1, UPPER16(MIPS64_PRACC_PARAM_IN)),
		MIPS64_ORI(1, 1, LOWER16(MIPS64_PRACC_PARAM_IN)),
		MIPS64_LD(3, 3*8, 1),
		MIPS64_LD(4, 4*8, 1),
		MIPS64_LD(5, 5*8, 1),
		MIPS64_LD(6, 6*8, 1),
		MIPS64_LD(7, 7*8, 1),
		MIPS64_LD(8, 8*8, 1),
		MIPS64_LD(9, 9*8, 1),
		MIPS64_LD(10, 10*8, 1),
		MIPS64_LD(11, 11*8, 1),
		MIPS64_LD(12, 12*8, 1),
		MIPS64_LD(13, 13*8, 1),
		MIPS64_LD(14, 14*8, 1),
		MIPS64_LD(16, 16*8, 1),
		MIPS64_LD(17, 17*8, 1),
		MIPS64_LD(18, 18*8, 1),
		MIPS64_LD(19, 19*8, 1),
		MIPS64_LD(20, 20*8, 1),
		MIPS64_LD(21, 21*8, 1),
		MIPS64_LD(22, 22*8, 1),
		MIPS64_LD(23, 23*8, 1),
		MIPS64_LD(24, 24*8, 1),
		MIPS64_LD(25, 25*8, 1),
		MIPS64_LD(26, 26*8, 1),
		MIPS64_LD(27, 27*8, 1),
		MIPS64_LD(28, 28*8, 1),
		MIPS64_LD(29, 29*8, 1),
		MIPS64_LD(30, 30*8, 1),
		MIPS64_LD(31, 31*8, 1),
		MIPS64_LD(2, 32*8, 1),
		MIPS64_MTLO(2),
		MIPS64_LD(2, 33*8, 1),
		MIPS64_MTHI(2),
		MIPS64_LD(2, MIPS64_NUM_CORE_REGS * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_DEPC, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 2) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_ENTRYLO0, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 3) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_ENTRYLO1, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 4) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_CONTEXT, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 5) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_PAGEMASK, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 6) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_WIRED, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 8) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_COUNT, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 9) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_ENTRYHI, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 10) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_COMPARE, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 11) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_STATUS, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 12) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_CAUSE, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 13) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_EPC, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 15) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_CONFIG, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 16) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_LLA, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 21) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_XCONTEXT, 1),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 22) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_MEMCTRL, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 24) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_PERFCOUNT, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 25) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_PERFCOUNT, 1),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 26) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_PERFCOUNT, 2),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 27) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_PERFCOUNT, 3),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 28) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_ECC, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 29) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_CACHERR, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 30) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_TAGLO, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 31) * 8, 1),
		MIPS64_MTC0(2, MIPS64_C0_TAGHI, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 32) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_DATAHI, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_REGS + 33) * 8, 1),
		MIPS64_DMTC0(2, MIPS64_C0_EEPC, 0),
		/* check if FPU is enabled, */
		MIPS64_MFC0(2, MIPS64_C0_STATUS, 0),
		MIPS64_SRL(2, 2, 29),
		MIPS64_ANDI(2, 2, 1),
		/* skip FPU registers restoration if not */
		MIPS64_BEQ(0, 2, 77),
		MIPS64_NOP,
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 33) * 8, 1),
		MIPS64_CTC1(2, MIPS64_C1_FIR, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 32) * 8, 1),
		MIPS64_CTC1(2, MIPS64_C1_FCSR, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 34) * 8, 1),
		MIPS64_CTC1(2, MIPS64_C1_FCONFIG, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 35) * 8, 1),
		MIPS64_CTC1(2, MIPS64_C1_FCCR, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 36) * 8, 1),
		MIPS64_CTC1(2, MIPS64_C1_FEXR, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 37) * 8, 1),
		MIPS64_CTC1(2, MIPS64_C1_FENR, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 0) * 8, 1),
		MIPS64_DMTC1(2, 0, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 1) * 8, 1),
		MIPS64_DMTC1(2, 1, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 2) * 8, 1),
		MIPS64_DMTC1(2, 2, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 3) * 8, 1),
		MIPS64_DMTC1(2, 3, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 4) * 8, 1),
		MIPS64_DMTC1(2, 4, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 5) * 8, 1),
		MIPS64_DMTC1(2, 5, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 6) * 8, 1),
		MIPS64_DMTC1(2, 6, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 7) * 8, 1),
		MIPS64_DMTC1(2, 7, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 8) * 8, 1),
		MIPS64_DMTC1(2, 8, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 9) * 8, 1),
		MIPS64_DMTC1(2, 9, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 10) * 8, 1),
		MIPS64_DMTC1(2, 10, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 11) * 8, 1),
		MIPS64_DMTC1(2, 11, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 12) * 8, 1),
		MIPS64_DMTC1(2, 12, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 13) * 8, 1),
		MIPS64_DMTC1(2, 13, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 14) * 8, 1),
		MIPS64_DMTC1(2, 14, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 15) * 8, 1),
		MIPS64_DMTC1(2, 15, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 16) * 8, 1),
		MIPS64_DMTC1(2, 16, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 17) * 8, 1),
		MIPS64_DMTC1(2, 17, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 18) * 8, 1),
		MIPS64_DMTC1(2, 18, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 19) * 8, 1),
		MIPS64_DMTC1(2, 19, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 20) * 8, 1),
		MIPS64_DMTC1(2, 20, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 21) * 8, 1),
		MIPS64_DMTC1(2, 21, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 22) * 8, 1),
		MIPS64_DMTC1(2, 22, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 23) * 8, 1),
		MIPS64_DMTC1(2, 23, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 24) * 8, 1),
		MIPS64_DMTC1(2, 24, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 25) * 8, 1),
		MIPS64_DMTC1(2, 25, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 26) * 8, 1),
		MIPS64_DMTC1(2, 26, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 27) * 8, 1),
		MIPS64_DMTC1(2, 27, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 28) * 8, 1),
		MIPS64_DMTC1(2, 28, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 29) * 8, 1),
		MIPS64_DMTC1(2, 29, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 30) * 8, 1),
		MIPS64_DMTC1(2, 30, 0),
		MIPS64_LD(2, (MIPS64_NUM_CORE_C0_REGS + 31) * 8, 1),
		MIPS64_DMTC1(2, 31, 0),
		MIPS64_LD(2, 2 * 8, 1),
		MIPS64_LD(1, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(181)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		MIPS64_NUM_REGS, regs, 0, NULL);
}

int mips64_pracc_read_regs(struct mips_ejtag *ejtag_info, uint64_t *regs)
{
	const uint32_t code[] = {
		/* move $2 to COP0 DeSave */
		MIPS64_DMTC0(2, 31, 0),
		/* $2 = MIPS64_PRACC_PARAM_OUT */
		MIPS64_LUI(2, UPPER16(MIPS64_PRACC_PARAM_OUT)),
		MIPS64_ORI(2, 2, LOWER16(MIPS64_PRACC_PARAM_OUT)),
		/* sd $0, 0*8($2) */
		MIPS64_SD(0, 0*8, 2),
		/* sd $1, 1*8($2) */
		MIPS64_SD(1, 1*8, 2),
		/* sd $15, 15*8($2) */
		MIPS64_SD(15, 15*8, 2),
		/* move COP0 DeSave to $2 */
		MIPS64_DMFC0(2, 31, 0),
		/* move $15 to COP0 DeSave */
		MIPS64_DMTC0(15, 31, 0),
		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		/* sd $1, ($15) */
		MIPS64_SD(1, 0, 15),
		/* sd $2, ($15) */
		MIPS64_SD(2, 0, 15),
		/* $1 = MIPS64_PRACC_PARAM_OUT */
		MIPS64_LUI(1, UPPER16(MIPS64_PRACC_PARAM_OUT)),
		MIPS64_ORI(1, 1, LOWER16(MIPS64_PRACC_PARAM_OUT)),
		MIPS64_SD(2, 2 * 8, 1),
		MIPS64_SD(3, 3 * 8, 1),
		MIPS64_SD(4, 4 * 8, 1),
		MIPS64_SD(5, 5 * 8, 1),
		MIPS64_SD(6, 6 * 8, 1),
		MIPS64_SD(7, 7 * 8, 1),
		MIPS64_SD(8, 8 * 8, 1),
		MIPS64_SD(9, 9 * 8, 1),
		MIPS64_SD(10, 10 * 8, 1),
		MIPS64_SD(11, 11 * 8, 1),
		MIPS64_SD(12, 12 * 8, 1),
		MIPS64_SD(13, 13 * 8, 1),
		MIPS64_SD(14, 14 * 8, 1),
		MIPS64_SD(16, 16 * 8, 1),
		MIPS64_SD(17, 17 * 8, 1),
		MIPS64_SD(18, 18 * 8, 1),
		MIPS64_SD(19, 19 * 8, 1),
		MIPS64_SD(20, 20 * 8, 1),
		MIPS64_SD(21, 21 * 8, 1),
		MIPS64_SD(22, 22 * 8, 1),
		MIPS64_SD(23, 23 * 8, 1),
		MIPS64_SD(24, 24 * 8, 1),
		MIPS64_SD(25, 25 * 8, 1),
		MIPS64_SD(26, 26 * 8, 1),
		MIPS64_SD(27, 27 * 8, 1),
		MIPS64_SD(28, 28 * 8, 1),
		MIPS64_SD(29, 29 * 8, 1),
		MIPS64_SD(30, 30 * 8, 1),
		MIPS64_SD(31, 31 * 8, 1),
		MIPS64_MFLO(2),
		MIPS64_SD(2, 32 * 8, 1),
		MIPS64_MFHI(2),
		MIPS64_SD(2, 33 * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_DEPC, 0),
		MIPS64_SD(2, MIPS64_NUM_CORE_REGS * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_RANDOM, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 1) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_ENTRYLO0, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 2) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_ENTRYLO1, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 3) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_CONTEXT, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 4) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_PAGEMASK, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 5) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_WIRED, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 6) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_BADVADDR, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 7) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_COUNT, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 8) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_ENTRYHI, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 9) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_COMPARE, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 10) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_STATUS, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 11) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_CAUSE, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 12) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_EPC, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 13) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_PRID, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 14) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_CONFIG, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 15) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_LLA, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 16) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_XCONTEXT, 1),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 21) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_MEMCTRL, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 22) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_DEBUG, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 23) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_PERFCOUNT, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 24) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_PERFCOUNT, 1),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 25) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_PERFCOUNT, 2),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 26) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_PERFCOUNT, 3),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 27) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_ECC, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 28) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_CACHERR, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 29) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_TAGLO, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 30) * 8, 1),
		MIPS64_MFC0(2, MIPS64_C0_TAGHI, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 31) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_DATAHI, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 32) * 8, 1),
		MIPS64_DMFC0(2, MIPS64_C0_EEPC, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_REGS + 33) * 8, 1),
		/* check if FPU is enabled, */
		MIPS64_MFC0(2, MIPS64_C0_STATUS, 0),
		MIPS64_SRL(2, 2, 29),
		MIPS64_ANDI(2, 2, 1),
		/* skip FPU registers dump if not */
		MIPS64_BEQ(0, 2, 77),
		MIPS64_NOP,
		MIPS64_CFC1(2, MIPS64_C1_FIR, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 33) * 8, 1),
		MIPS64_CFC1(2, MIPS64_C1_FCSR, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 32) * 8, 1),
		MIPS64_CFC1(2, MIPS64_C1_FCONFIG, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 34) * 8, 1),
		MIPS64_CFC1(2, MIPS64_C1_FCCR, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 35) * 8, 1),
		MIPS64_CFC1(2, MIPS64_C1_FEXR, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 36) * 8, 1),
		MIPS64_CFC1(2, MIPS64_C1_FENR, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 37) * 8, 1),
		MIPS64_DMFC1(2, 0, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 0) * 8, 1),
		MIPS64_DMFC1(2, 1, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 1) * 8, 1),
		MIPS64_DMFC1(2, 2, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 2) * 8, 1),
		MIPS64_DMFC1(2, 3, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 3) * 8, 1),
		MIPS64_DMFC1(2, 4, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 4) * 8, 1),
		MIPS64_DMFC1(2, 5, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 5) * 8, 1),
		MIPS64_DMFC1(2, 6, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 6) * 8, 1),
		MIPS64_DMFC1(2, 7, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 7) * 8, 1),
		MIPS64_DMFC1(2, 8, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 8) * 8, 1),
		MIPS64_DMFC1(2, 9, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 9) * 8, 1),
		MIPS64_DMFC1(2, 10, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 10) * 8, 1),
		MIPS64_DMFC1(2, 11, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 11) * 8, 1),
		MIPS64_DMFC1(2, 12, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 12) * 8, 1),
		MIPS64_DMFC1(2, 13, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 13) * 8, 1),
		MIPS64_DMFC1(2, 14, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 14) * 8, 1),
		MIPS64_DMFC1(2, 15, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 15) * 8, 1),
		MIPS64_DMFC1(2, 16, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 16) * 8, 1),
		MIPS64_DMFC1(2, 17, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 17) * 8, 1),
		MIPS64_DMFC1(2, 18, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 18) * 8, 1),
		MIPS64_DMFC1(2, 19, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 19) * 8, 1),
		MIPS64_DMFC1(2, 20, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 20) * 8, 1),
		MIPS64_DMFC1(2, 21, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 21) * 8, 1),
		MIPS64_DMFC1(2, 22, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 22) * 8, 1),
		MIPS64_DMFC1(2, 23, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 23) * 8, 1),
		MIPS64_DMFC1(2, 24, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 24) * 8, 1),
		MIPS64_DMFC1(2, 25, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 25) * 8, 1),
		MIPS64_DMFC1(2, 26, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 26) * 8, 1),
		MIPS64_DMFC1(2, 27, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 27) * 8, 1),
		MIPS64_DMFC1(2, 28, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 28) * 8, 1),
		MIPS64_DMFC1(2, 29, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 29) * 8, 1),
		MIPS64_DMFC1(2, 30, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 30) * 8, 1),
		MIPS64_DMFC1(2, 31, 0),
		MIPS64_SD(2, (MIPS64_NUM_CORE_C0_REGS + 31) * 8, 1),
		MIPS64_LD(2, 0, 15),
		MIPS64_LD(1, 0, 15),
		MIPS64_SYNC,
		/* b start */
		MIPS64_B(NEG16(192)),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		0, NULL, MIPS64_NUM_REGS, regs);
}

/* fastdata upload/download requires an initialized working area
 * to load the download code; it should not be called otherwise
 * fetch order from the fastdata area
 * 1. start addr
 * 2. end addr
 * 3. data ...
 */
int mips64_pracc_fastdata_xfer(struct mips_ejtag *ejtag_info,
			       struct working_area *source,
			       bool write_t, uint64_t addr,
			       unsigned count, uint64_t *buf)
{
	uint32_t handler_code[] = {
		/* caution when editing, table is modified below */
		/* r15 points to the start of this code */
		MIPS64_SD(8, MIPS64_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS64_SD(9, MIPS64_FASTDATA_HANDLER_SIZE - 8 * 2, 15),
		MIPS64_SD(10, MIPS64_FASTDATA_HANDLER_SIZE - 8 * 3, 15),
		MIPS64_SD(11, MIPS64_FASTDATA_HANDLER_SIZE - 8 * 4, 15),
		/* start of fastdata area in t0 */
		MIPS64_LUI(8, UPPER16(MIPS64_PRACC_FASTDATA_AREA)),
		MIPS64_ORI(8, 8, LOWER16(MIPS64_PRACC_FASTDATA_AREA)),
		/* start addr in t1 */
		MIPS64_LD(9, 0, 8),
		/* end addr to t2 */
		MIPS64_LD(10, 0, 8),

		/* loop: */
		/* lw t3,[t8 | r9] */
		/* 8 */ MIPS64_LD(11, 0, 0),
		/* sw t3,[r9 | r8] */
		/* 9 */ MIPS64_SD(11, 0, 0),
		/* bne $t2,t1,loop */
		MIPS64_BNE(10, 9, NEG16(3)),
		/* addi t1,t1,4 */
		MIPS64_DADDIU(9, 9, 8),

		MIPS64_LD(8, MIPS64_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS64_LD(9, MIPS64_FASTDATA_HANDLER_SIZE - 8 * 2, 15),
		MIPS64_LD(10, MIPS64_FASTDATA_HANDLER_SIZE - 8 * 3, 15),
		MIPS64_LD(11, MIPS64_FASTDATA_HANDLER_SIZE - 8 * 4, 15),

		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_TEXT)),
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_TEXT)),
		/* jr start */
		MIPS64_JR(15),
		/* move COP0 DeSave to $15 */
		MIPS64_DMFC0(15, 31, 0),
	};

	uint32_t jmp_code[] = {
		/* addr of working area added below */
		/* 0 */ MIPS64_LUI(15, 0),
		/* addr of working area added below */
		/* 1 */ MIPS64_ORI(15, 15, 0),
		/* jump to ram program */
		MIPS64_JR(15),
		MIPS64_NOP,
	};

	int retval;
	unsigned i;
	uint32_t ejtag_ctrl, address32;
	uint64_t address, val;

	if (source->size < MIPS64_FASTDATA_HANDLER_SIZE)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (write_t) {
		/* load data from probe at fastdata area */
		handler_code[8] = MIPS64_LD(11, 0, 8);
		/* store data to RAM @ r9 */
		handler_code[9] = MIPS64_SD(11, 0, 9);
	} else {
		/* load data from RAM @ r9 */
		handler_code[8] = MIPS64_LD(11, 0, 9);
		/* store data to probe at fastdata area */
		handler_code[9] = MIPS64_SD(11, 0, 8);
	}

	/* write program into RAM */
	if (write_t != ejtag_info->fast_access_save) {
		mips64_pracc_write_mem(ejtag_info, source->address, 4,
				       ARRAY_SIZE(handler_code), handler_code);
		/* save previous operation to speed to any consecutive read/writes */
		ejtag_info->fast_access_save = write_t;
	}

	LOG_DEBUG("%s using " TARGET_ADDR_FMT " for write handler", __func__,
		  source->address);
	LOG_DEBUG("daddiu: %08" PRIx32, handler_code[11]);

	jmp_code[0] |= UPPER16(source->address);
	jmp_code[1] |= LOWER16(source->address);
	mips64_pracc_exec(ejtag_info,
			  ARRAY_SIZE(jmp_code), jmp_code,
			  0, NULL, 0, NULL);

	/* next fetch to dmseg should be in FASTDATA_AREA, check */
	address = 0;

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	retval = mips_ejtag_drscan_32(ejtag_info, &address32);
	if (retval != ERROR_OK)
		return retval;
	address = 0xffffffffff200000ull | address32;
	if ((address & ~7ull) != MIPS64_PRACC_FASTDATA_AREA) {
		LOG_ERROR("! @MIPS64_PRACC_FASTDATA_AREA (" TARGET_ADDR_FMT ")", address);
		return ERROR_FAIL;
	}
	/* Send the load start address */
	val = addr;
	LOG_DEBUG("start: " TARGET_ADDR_FMT, val);
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips64_ejtag_fastdata_scan(ejtag_info, 1, &val);

	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		return retval;

	/* Send the load end address */
	val = addr + (count - 1) * 8;
	LOG_DEBUG("stop: " TARGET_ADDR_FMT, val);
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips64_ejtag_fastdata_scan(ejtag_info, 1, &val);

	/* like in legacy code */
	unsigned num_clocks = 0;
	if (ejtag_info->mode != 0)
		num_clocks = ((uint64_t)(ejtag_info->scan_delay) * jtag_get_speed_khz() + 500000) / 1000000;
	LOG_DEBUG("num_clocks=%d", num_clocks);
	for (i = 0; i < count; i++) {
		jtag_add_clocks(num_clocks);
		retval = mips64_ejtag_fastdata_scan(ejtag_info, write_t, buf++);
		if (retval != ERROR_OK) {
			LOG_ERROR("mips64_ejtag_fastdata_scan failed");
			return retval;
		}
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("jtag_execute_queue failed");
		return retval;
	}

	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK) {
		LOG_ERROR("wait_for_pracc_rw failed");
		return retval;
	}

	address = 0;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	retval = mips_ejtag_drscan_32(ejtag_info, &address32);
	if (retval != ERROR_OK) {
		LOG_ERROR("mips_ejtag_drscan_32 failed");
		return retval;
	}

	address = 0xffffffffff200000ull | address32;
	if ((address & ~7ull) != MIPS64_PRACC_TEXT)
		LOG_ERROR("mini program did not return to start");

	return retval;
}
