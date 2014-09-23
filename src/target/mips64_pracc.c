/***************************************************************************
 *   Support for processors implementing MIPS64 instruction set            *
 *                                                                         *
 *   Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>           *
 *   Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>             *
 *   Copyright (C) 2014 by Peter Mamonov <pmamonov@gmail.com>              *
 *                                                                         *
 *   Based on the work of:                                                 *
 *       Copyright (C) 2008 by Spencer Oliver                              *
 *       Copyright (C) 2008 by David T.L. Wong                             *
 *       Copyright (C) 2010 by Konstantin Kostyukhin, Nikolay Shmyrev      *
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
 *   along with this program. If not, see <http://www.gnu.org/licenses/>.  *
 ***************************************************************************/

/*
This version has optimized assembly routines for 32 bit operations:
- read word
- write word
- write array of words

One thing to be aware of is that the MIPS64 cpu will execute the
instruction after a branch instruction (one delay slot).

For example:


    LW $2, ($5 +10)
    B foo
    LW $1, ($2 +100)

The LW $1, ($2 +100) instruction is also executed. If this is
not wanted a NOP can be inserted:

    LW $2, ($5 +10)
    B foo
    NOP
    LW $1, ($2 +100)

or the code can be changed to:

    B foo
    LW $2, ($5 +10)
    LW $1, ($2 +100)

The original code contained NOPs. I have removed these and moved
the branches.

I also moved the PRACC_STACK to 0xFF204000. This allows
the use of 16 bits offsets to get pointers to the input
and output area relative to the stack. Note that the stack
isn't really a stack (the stack pointer is not 'moving')
but a FIFO simulated in software.

These changes result in a 35% speed increase when programming an
external flash.

More improvement could be gained if the registers do no need
to be preserved but in that case the routines should be aware
OpenOCD is used as a flash programmer or as a debug tool.

Nico Coesel
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if BUILD_TARGET64 == 1

#include "mips64.h"
#include "mips64_pracc.h"

#include "time_support.h"

typedef struct {
	uint64_t *local_iparam;
	int num_iparam;
	uint64_t *local_oparam;
	int num_oparam;
	uint32_t *code;
	int code_len;
	uint64_t stack[32];
	int stack_offset;
	struct mips_ejtag *ejtag_info;
} mips64_pracc_context;

static int wait_for_pracc_rw(struct mips_ejtag *ejtag_info, uint32_t *ctrl)
{
	uint32_t ejtag_ctrl;

	int nt = 5;
	while (1) {
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		ejtag_ctrl = ejtag_info->ejtag_ctrl;
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

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
	int offset;
	uint32_t ejtag_ctrl;
	uint64_t data;
	static uint64_t prev_text_address = MIPS64_PRACC_TEXT;

	if ((address >= MIPS64_PRACC_PARAM_IN)
		&& (address <= MIPS64_PRACC_PARAM_IN + ctx->num_iparam * MIPS64_PRACC_DATA_STEP)) {
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
		LOG_DEBUG("Reading %016llx at %016llx", (unsigned long long) data, (unsigned long long) address);

	} else if ((address >= MIPS64_PRACC_PARAM_OUT)
		&& (address <= MIPS64_PRACC_PARAM_OUT + ctx->num_oparam * MIPS64_PRACC_DATA_STEP)) {
		offset = (address - MIPS64_PRACC_PARAM_OUT) / MIPS64_PRACC_DATA_STEP;
		if (ctx->local_oparam == NULL) {
			LOG_ERROR("Error: unexpected reading of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		data = ctx->local_oparam[offset];
		LOG_DEBUG("Reading %016llx at %016llx", (unsigned long long) data, (unsigned long long) address);
	} else if ((address >= MIPS64_PRACC_TEXT)
		&& (address <= MIPS64_PRACC_TEXT + ctx->code_len * MIPS64_PRACC_ADDR_STEP)) {
		/* Check for correct text address incrementation */
		if ((address != MIPS64_PRACC_TEXT) &&
		    (address != prev_text_address + MIPS64_PRACC_ADDR_STEP * 2)) {
			LOG_DEBUG("Warning: probably reading unexpected text address or there was jump due to branch from %016llx",
				(unsigned long long) prev_text_address);
		}

		offset = ((address & ~7ull) - MIPS64_PRACC_TEXT) / MIPS64_PRACC_ADDR_STEP;
		data = (uint64_t)ctx->code[offset] << 32;
		if (offset + 1 < ctx->code_len)
			data |= (uint64_t)ctx->code[offset + 1];


		prev_text_address = address;
		LOG_DEBUG("Running commands %016llx at address %016llx",
			(unsigned long long) data, (unsigned long long) address);
	} else if ((address & ~7llu) == MIPS64_PRACC_STACK) {
		/* load from our debug stack */
		if (ctx->stack_offset == 0) {
			LOG_ERROR("Error reading from stack: stack is empty");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		data = ctx->stack[--ctx->stack_offset];
		LOG_DEBUG("Reading %llx at %llx", (unsigned long long) data, (unsigned long long) address);
	} else {
		/* TODO: send JMP 0xFF200000 instruction. Hopefully processor jump back
		 * to start of debug vector */

		data = 0;
		LOG_ERROR("Error reading unexpected address %016llx", (unsigned long long) address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Send the data out */
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_64(ctx->ejtag_info, &data);

	/* Clear the access pending bit (let the processor eat!) */

	ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32(ctx->ejtag_info, &ejtag_ctrl);

	jtag_add_clocks(5);

	jtag_execute_queue();

	return ERROR_OK;
}

static int mips64_pracc_exec_write(mips64_pracc_context *ctx, uint64_t address)
{
	uint32_t ejtag_ctrl;
	uint64_t data;
	int offset;
	struct mips_ejtag *ejtag_info = ctx->ejtag_info;

	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_64(ctx->ejtag_info, &data);

	/* Clear access pending bit */
	ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32(ctx->ejtag_info, &ejtag_ctrl);

	jtag_add_clocks(5);
	jtag_execute_queue();

	LOG_DEBUG("Writing %016llx at %016llx\n", (unsigned long long) data, (unsigned long long) address);

	if ((address >= MIPS64_PRACC_PARAM_IN)
		&& (address <= MIPS64_PRACC_PARAM_IN + ctx->num_iparam * MIPS64_PRACC_DATA_STEP)) {
		offset = (address - MIPS64_PRACC_PARAM_IN) / MIPS64_PRACC_DATA_STEP;
		if (ctx->local_iparam == NULL) {
			LOG_ERROR("Error: unexpected writing of input parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		ctx->local_iparam[offset] = data;
	} else if ((address >= MIPS64_PRACC_PARAM_OUT)
		&& (address <= MIPS64_PRACC_PARAM_OUT + ctx->num_oparam * MIPS64_PRACC_DATA_STEP)) {
		offset = (address - MIPS64_PRACC_PARAM_OUT) / MIPS64_PRACC_DATA_STEP;
		if (ctx->local_oparam == NULL) {
			LOG_ERROR("Error: unexpected writing of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		ctx->local_oparam[offset] = data;
	} else if (address == MIPS64_PRACC_STACK) {
		/* save data onto our stack */
		ctx->stack[ctx->stack_offset++] = data;
	} else {
		LOG_ERROR("Error writing unexpected address 0x%" PRIx64 "", address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

int mips64_pracc_exec(struct mips_ejtag *ejtag_info, int code_len, uint32_t *code,
	int num_param_in, uint64_t *param_in, int num_param_out, uint64_t *param_out, int cycle)
{
	uint32_t ejtag_ctrl;
	uint64_t address = 0, address_prev = 0, data;
	mips64_pracc_context ctx;
	int retval;
	int pass = 0;
	int first_time_call = 1;
	int i;

	for (i = 0; i < code_len; i++)
		LOG_DEBUG("%08x", code[i]);

	ctx.local_iparam = param_in;
	ctx.local_oparam = param_out;
	ctx.num_iparam = num_param_in;
	ctx.num_oparam = num_param_out;
	ctx.code = code;
	ctx.code_len = code_len;
	ctx.ejtag_info = ejtag_info;
	ctx.stack_offset = 0;

	while (1) {
		uint32_t address32;
		retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK) {
			LOG_ERROR("ERROR wait_for_pracc_rw");
			return retval;
		}
		if (pass)
			address_prev = address;
		else
			address_prev = 0;
		address32 = data = 0;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
		mips_ejtag_drscan_32(ejtag_info, &address32);
		LOG_DEBUG("-> %08x", address32);
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
			LOG_ERROR("Error reading address %016llx (%016llx expected)",
				address, MIPS64_PRACC_TEXT);
			return ERROR_JTAG_DEVICE_ERROR;
		}

		first_time_call = 0;

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

		if (cycle == 0)
			break;
	}

	/* stack sanity check */
	if (ctx.stack_offset != 0)
		LOG_DEBUG("Pracc Stack not zero");

	return ERROR_OK;
}

int mips64_pracc_read_mem(struct mips_ejtag *ejtag_info, uint64_t addr, int size, int count, void *buf)
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

int mips64_pracc_read_mem64(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint64_t *buf)
{
	int i, retval = ERROR_OK;
	for (i = 0; i < count; i++) {
		retval = mips64_pracc_read_u64(ejtag_info, addr + 8*i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_read_u64(struct mips_ejtag *ejtag_info, uint64_t addr, uint64_t *buf)
{
	uint32_t code[] = {
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),   /* load R8 @ param_in[0] = address */
		MIPS64_LD(8, 0, 8),					/* ld $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),	/* sd $8, 0($15) */
		MIPS64_LD(8, 0, 15),				/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(18)),				/* b start */
		MIPS64_DMFC0(15, 31, 0),				/* move COP0 DeSave to $15 */
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
		ARRAY_SIZE(param_in), param_in, 1, (uint64_t *) buf, 1);
}

int mips64_pracc_read_mem32(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint32_t *buf)
{
	int i, retval = ERROR_OK;
	for (i = 0; i < count; i++) {
		retval = mips64_pracc_read_u32(ejtag_info, addr + 4 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_read_u32(struct mips_ejtag *ejtag_info, uint64_t addr, uint32_t *buf)
{
	uint32_t code[] = {
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),   /* load R8 @ param_in[0] = address */
		MIPS64_LW(8, 0, 8),					/* lw $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),	/* sd $8, 0($9) */
		MIPS64_LD(8, 0, 15),					/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(17)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),				/* move COP0 DeSave to $15 */
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
		1, param_in, 1, param_out, 1);
	buf[0] = (uint32_t) param_out[0];
	return retval;
}

int mips64_pracc_read_mem16(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint16_t *buf)
{
	int i, retval = ERROR_OK;
	for (i = 0; i < count; i++) {
		retval = mips64_pracc_read_u16(ejtag_info, addr + 2*i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_read_u16(struct mips_ejtag *ejtag_info, uint64_t addr, uint16_t *buf)
{
	uint32_t code[] = {
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),   /* load R8 @ param_in[0] = address */
		MIPS64_LHU(8, 0, 8),					/* lw $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),	/* sd $8, 0($9) */
		MIPS64_LD(8, 0, 15),					/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(17)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),				/* move COP0 DeSave to $15 */
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
		1, param_in, 1, param_out, 1);
	buf[0] = (uint16_t)param_out[0];
	return retval;
}

int mips64_pracc_read_mem8(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint8_t *buf)
{
	int i, retval = ERROR_OK;
	for (i = 0; i < count; i++) {
		retval = mips64_pracc_read_u8(ejtag_info, addr + i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_read_u8(struct mips_ejtag *ejtag_info, uint64_t addr, uint8_t *buf)
{
	uint32_t code[] = {
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_LD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),   /* load R8 @ param_in[0] = address */
		MIPS64_LBU(8, 0, 8),					/* lw $8, 0($8),  Load $8 with the word @mem[$8] */
		MIPS64_SD(8, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_OUT), 15),	/* sd $8, 0($9) */
		MIPS64_LD(8, 0, 15),					/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(17)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),				/* move COP0 DeSave to $15 */
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
		1, param_in, 1, param_out, 1);
	buf[0] = (uint8_t)param_out[0];
	return retval;
}

int mips64_pracc_write_mem(struct mips_ejtag *ejtag_info, uint64_t addr, int size, int count, void *buf)
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

int mips64_pracc_write_mem64(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint64_t *buf)
{
	int i, retval = ERROR_OK;

	for (i = 0; i < count; i++) {
		retval = mips64_pracc_write_u64(ejtag_info, addr + 8*i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_write_u64(struct mips_ejtag *ejtag_info, uint64_t addr, uint64_t *buf)
{
	uint32_t code[] = {
									/* start: */
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_SD(9, 0, 15),					/* sd $9, ($15) */

		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN)-8),  15),	/* load R8 @ param_in[1] = data */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),	/* load R9 @ param_in[0] = address */
		MIPS64_SD(8, 0, 9),					/* sd $8, 0($9) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_SYNCI(9, 0),
		MIPS64_NOP,
		MIPS64_NOP,

		MIPS64_LD(9, 0, 15),					/* ld $9, ($15) */
		MIPS64_LD(8, 0, 15),					/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(26)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),					/* move COP0 DeSave to $15 */
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
		ARRAY_SIZE(param_in), param_in, 0, NULL, 1);
}

int mips64_pracc_write_mem32(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint32_t *buf)
{
	int i, retval = ERROR_OK;

	for (i = 0; i < count; i++) {
		retval = mips64_pracc_write_u32(ejtag_info, addr + 4*i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_write_u32(struct mips_ejtag *ejtag_info, uint64_t addr, uint32_t *buf)
{
	uint32_t code[] = {
									/* start: */
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_SD(9, 0, 15),					/* sd $9, ($15) */
		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN)-8),  15),   /* load R8 @ param_in[1] = data */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),   /* load R9 @ param_in[0] = address */
		MIPS64_SW(8, 0, 9),					/* sw $8, 0($9) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
#if 0
		MIPS64_CACHE(1 << 3 | 5, 9, 0),					/* D cache Hit invalidate */
		MIPS64_CACHE(3 << 3 | 5, 9, 0),					/* L2 cache Hit invalidate */
		MIPS64_CACHE(2 << 3 | 5, 9, 0),					/* L3 cache Hit invalidate */
		MIPS64_CACHE(0 << 3 | 4, 9, 0),					/* I cache fill */
#endif
		MIPS64_SYNCI(9, 0),
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_LD(9, 0, 15),					/* ld $9, ($15) */
		MIPS64_LD(8, 0, 15),					/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(20)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),					/* move COP0 DeSave to $15 */
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
		ARRAY_SIZE(param_in), param_in, 0, NULL, 1);
}

int mips64_pracc_write_mem16(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint16_t *buf)
{
	int i, retval = ERROR_OK;

	for (i = 0; i < count; i++) {
		retval = mips64_pracc_write_u16(ejtag_info, addr + 2*i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_write_u16(struct mips_ejtag *ejtag_info, uint64_t addr, uint16_t *buf)
{
	uint32_t code[] = {
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_SD(9, 0, 15),					/* sd $9, ($15) */
		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN)-8),  15),   /* load R8 @ param_in[1] = data */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),   /* load R9 @ param_in[0] = address */
		MIPS64_SH(8, 0, 9),					/* sh $8, 0($9) */

		MIPS64_LD(9, 0, 15),					/* ld $9, ($15) */
		MIPS64_LD(8, 0, 15),					/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(14)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),					/* move COP0 DeSave to $15 */
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
		ARRAY_SIZE(param_in), param_in, 0, NULL, 1);
}

int mips64_pracc_write_mem8(struct mips_ejtag *ejtag_info, uint64_t addr, int count, uint8_t *buf)
{
	int i, retval = ERROR_OK;

	for (i = 0; i < count; i++) {
		retval = mips64_pracc_write_u8(ejtag_info, addr + i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int mips64_pracc_write_u8(struct mips_ejtag *ejtag_info, uint64_t addr, uint8_t *buf)
{
	uint32_t code[] = {
		MIPS64_DMTC0(15, 31, 0),					/* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(8, 0, 15),					/* sd $8, ($15) */
		MIPS64_SD(9, 0, 15),					/* sd $9, ($15) */
		MIPS64_LD(8, NEG16((MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN)-8),  15),   /* load R8 @ param_in[1] = data */
		MIPS64_LD(9, NEG16(MIPS64_PRACC_STACK-MIPS64_PRACC_PARAM_IN),  15),   /* load R9 @ param_in[0] = address */
		MIPS64_SB(8, 0, 9),					/* sh $8, 0($9) */

		MIPS64_LD(9, 0, 15),					/* ld $9, ($15) */
		MIPS64_LD(8, 0, 15),					/* ld $8, ($15) */
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(14)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),					/* move COP0 DeSave to $15 */
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
		ARRAY_SIZE(param_in), param_in, 0, NULL, 1);
}

int mips64_pracc_write_regs(struct mips_ejtag *ejtag_info, uint64_t *regs)
{
	uint32_t code[] = {
		MIPS64_DMTC0(2, 31, 0),						/* move $2 to COP0 DeSave */
		MIPS64_LUI(2, UPPER16(MIPS64_PRACC_PARAM_IN)),		/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(2, 2, LOWER16(MIPS64_PRACC_PARAM_IN)),
		MIPS64_LD(1, 1*8, 2),						/* sd $0, 0*8($2) */
		MIPS64_LD(15, 15*8, 2),					/* sd $1, 1*8($2) */
		MIPS64_DMFC0(2, 31, 0),					/* sd $11, ($15) */
		MIPS64_DMTC0(15, 31, 0),
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),

		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(1, 0, 15),
		MIPS64_LUI(1, UPPER16(MIPS64_PRACC_PARAM_IN)),	/* $11 = MIPS64_PRACC_PARAM_OUT */
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
		MIPS64_DMTC0(2, 12, 0),
		MIPS64_LD(2, 33*8, 1),
		MIPS64_MTLO(2),
		MIPS64_LD(2, 34*8, 1),
		MIPS64_MTHI(2),
		MIPS64_LD(2, 35*8, 1),
		MIPS64_DMTC0(2, 8, 0),

		MIPS64_LD(2, 36*8, 1),
		MIPS64_DMTC0(2, 13, 0),
		MIPS64_LD(2, MIPS64_PC*8, 1),
		MIPS64_DMTC0(2, 24, 0),
		MIPS64_LD(2, 2*8, 1),
		MIPS64_LD(1, 0, 15),
		MIPS64_NOP,
		MIPS64_SYNC,

		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(65)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),					/* move COP0 DeSave to $15 */

		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP
	};

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		MIPS64NUMCOREREGS, regs, 0, NULL, 1);
}

int mips64_pracc_read_regs(struct mips_ejtag *ejtag_info, uint64_t *regs)
{
	uint32_t code[] = {
								/* start: */
		MIPS64_DMTC0(2, 31, 0),				/* move $2 to COP0 DeSave */
		MIPS64_LUI(2, UPPER16(MIPS64_PRACC_PARAM_OUT)),	/* $2 = MIPS64_PRACC_PARAM_OUT */
		MIPS64_ORI(2, 2, LOWER16(MIPS64_PRACC_PARAM_OUT)),
		MIPS64_SD(0, 0*8, 2),				/* sd $0, 0*8($2) */
		MIPS64_SD(1, 1*8, 2),				/* sd $1, 1*8($2) */
		MIPS64_SD(15, 15*8, 2),				/* sd $15, 15*8($2) */
		MIPS64_DMFC0(2, 31, 0),				/* move COP0 DeSave to $2 */
		MIPS64_DMTC0(15, 31, 0),				/* move $15 to COP0 DeSave */

		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),	/* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(1, 0, 15),				/* sd $1, ($15) */
		MIPS64_SD(2, 0, 15),				/* sd $2, ($15) */
		MIPS64_LUI(1, UPPER16(MIPS64_PRACC_PARAM_OUT)),	/* $1 = MIPS64_PRACC_PARAM_OUT */
		MIPS64_ORI(1, 1, LOWER16(MIPS64_PRACC_PARAM_OUT)),
		MIPS64_SD(2, 2*8, 1),				/* sd $2, 2*8($1) */
		MIPS64_SD(3, 3*8, 1),				/* sd $3, 3*8($1) */

		MIPS64_SD(4, 4*8, 1),				/* sd $4, 4*8($1) */
		MIPS64_SD(5, 5*8, 1),				/* sd $5, 5*8($1) */
		MIPS64_SD(6, 6*8, 1),				/* sd $6, 6*8($1) */
		MIPS64_SD(7, 7*8, 1),				/* sd $7, 7*8($1) */
		MIPS64_SD(8, 8*8, 1),				/* sd $8, 8*8($1) */
		MIPS64_SD(9, 9*8, 1),				/* sd $9, 9*8($1) */
		MIPS64_SD(10, 10*8, 1),				/* sd $10, 10*8($1) */
		MIPS64_SD(11, 11*8, 1),				/* sd $11, 11*8($1) */

		MIPS64_SD(12, 12*8, 1),				/* sd $12, 12*8($1) */
		MIPS64_SD(13, 13*8, 1),				/* sd $13, 13*8($1) */
		MIPS64_SD(14, 14*8, 1),				/* sd $14, 14*8($1) */
		MIPS64_SD(16, 16*8, 1),				/* sd $16, 16*8($1) */
		MIPS64_SD(17, 17*8, 1),				/* sd $17, 17*8($1) */
		MIPS64_SD(18, 18*8, 1),				/* sd $18, 18*8($1) */
		MIPS64_SD(19, 19*8, 1),				/* sd $19, 19*8($1) */
		MIPS64_SD(20, 20*8, 1),				/* sd $20, 20*8($1) */

		MIPS64_SD(21, 21*8, 1),				/* sd $21, 21*8($1) */
		MIPS64_SD(22, 22*8, 1),				/* sd $22, 22*8($1) */
		MIPS64_SD(23, 23*8, 1),				/* sd $23, 23*8($1) */
		MIPS64_SD(24, 24*8, 1),				/* sd $24, 24*8($1) */
		MIPS64_SD(25, 25*8, 1),				/* sd $25, 25*8($1) */
		MIPS64_SD(26, 26*8, 1),				/* sd $26, 26*8($1) */
		MIPS64_SD(27, 27*8, 1),				/* sd $27, 27*8($1) */
		MIPS64_SD(28, 28*8, 1),				/* sd $28, 28*8($1) */
		MIPS64_SD(29, 29*8, 1),				/* sd $29, 29*8($1) */
		MIPS64_SD(30, 30*8, 1),				/* sd $30, 30*8($1) */
		MIPS64_SD(31, 31*8, 1),				/* sd $31, 31*8($1) */

		MIPS64_DMFC0(2, 12, 0),  /* (32)cpu.cpu_core.cop0.status_reg */
		MIPS64_SD(2, 32*8, 1),
		MIPS64_MFLO(2),  /*  (33)cpu.int_mult_div.lo_reg */
		MIPS64_SD(2, 33*8, 1),
		MIPS64_MFHI(2),  /* (34)cpu.int_mul_div.hi_reg */

		MIPS64_SD(2, 34*8, 1),
		MIPS64_DMFC0(2, 8, 0),  /* (35)cpu.cpu_core.cop0.BadVAddr */
		MIPS64_SD(2, 35*8, 1),
		MIPS64_DMFC0(2, 13, 0),  /* (36)cpu.cpu_core.cop0.cause */
		MIPS64_SD(2, 36*8, 1),
		MIPS64_DMFC0(2, 23, 0),  /* DEBUG register */
		MIPS64_SD(2, 37*8, 1),
		MIPS64_DMFC0(2, 24, 0),  /* (38)cpu.cpu_core.cop0.DEPC */
		MIPS64_SD(2, 38*8, 1),
		MIPS64_LD(2, 0, 15),

		MIPS64_LD(1, 0, 15),
		MIPS64_SYNC,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_B(NEG16(69)),					/* b start */
		MIPS64_DMFC0(15, 31, 0),					/* move COP0 DeSave to $15 */

		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP
	};

	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		0, NULL, MIPS64NUMCOREREGS, regs, 1);
}

#endif /* BUILD_TARGET64 */
