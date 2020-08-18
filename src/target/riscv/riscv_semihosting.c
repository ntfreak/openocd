/***************************************************************************
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   ilg@livius.net                                                        *
 *                                                                         *
 *   Copyright (C) 2009 by Marvell Technology Group Ltd.                   *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
 *                                                                         *
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Square, Inc.                                    *
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

/**
 * @file
 * Hold RISC-V semihosting support.
 *
 * The RISC-V code is inspired from ARM semihosting.
 *
 * Details can be found in chapter 8 of DUI0203I_rvct_developer_guide.pdf
 * from ARM Ltd.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"

#include "target/target.h"
#include "target/semihosting_common.h"
#include "riscv.h"

static int riscv_semihosting_setup(struct target *target, int enable);
static int riscv_semihosting_post_result(struct target *target);

/**
 * Initialize RISC-V semihosting. Use common ARM code.
 */
void riscv_semihosting_init(struct target *target)
{
	semihosting_common_init(target, riscv_semihosting_setup,
		riscv_semihosting_post_result);
}

/**
 * Check for and process a semihosting request using the ARM protocol). This
 * is meant to be called when the target is stopped due to a debug mode entry.
 * If the value 0 is returned then there was nothing to process. A non-zero
 * return value signifies that a request was processed and the target resumed,
 * or an error was encountered, in which case the caller must return
 * immediately.
 *
 * @param target Pointer to the target to process.
 * @param retval Pointer to a location where the return code will be stored
 * @return non-zero value if a request was processed or an error encountered
 */
int riscv_semihosting(struct target *target, int *retval)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting)
		return 0;

	if (!semihosting->is_active)
		return 0;

	riscv_reg_t dpc;
	int result = riscv_get_register(target, &dpc, GDB_REGNO_DPC);
	if (result != ERROR_OK)
		return 0;

	uint8_t tmp[12];

	/* Read the current instruction, including the bracketing */
	*retval = target_read_memory(target, dpc - 4, 2, 6, tmp);
	if (*retval != ERROR_OK)
		return 0;

	/*
	 * The instructions that trigger a semihosting call,
	 * always uncompressed, should look like:
	 *
	 * 01f01013              slli    zero,zero,0x1f
	 * 00100073              ebreak
	 * 40705013              srai    zero,zero,0x7
	 */
	uint32_t pre = target_buffer_get_u32(target, tmp);
	uint32_t ebreak = target_buffer_get_u32(target, tmp + 4);
	uint32_t post = target_buffer_get_u32(target, tmp + 8);
	LOG_DEBUG("check %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " from 0x%" PRIx64 "-4", pre, ebreak, post, dpc);

	if (pre != 0x01f01013 || ebreak != 0x00100073 || post != 0x40705013) {

		/* Not the magic sequence defining semihosting. */
		return 0;
	}

	/*
	 * Perform semihosting call if we are not waiting on a fileio
	 * operation to complete.
	 */
	if (!semihosting->hit_fileio) {

		/* RISC-V uses A0 and A1 to pass function arguments */
		riscv_reg_t r0;
		riscv_reg_t r1;

		result = riscv_get_register(target, &r0, GDB_REGNO_A0);
		if (result != ERROR_OK)
			return 0;

		result = riscv_get_register(target, &r1, GDB_REGNO_A1);
		if (result != ERROR_OK)
			return 0;

		semihosting->op = r0;
		semihosting->param = r1;
		semihosting->word_size_bytes = riscv_xlen(target) / 8;

		/* Check for ARM operation numbers. */
		if (0 <= semihosting->op && semihosting->op <= 0x31) {
			*retval = semihosting_common(target);
			if (*retval != ERROR_OK) {
				LOG_ERROR("Failed semihosting operation");
				return 0;
			}
		} else {
			/* Unknown operation number, not a semihosting call. */
			return 0;
		}
	}

	/*
	 * Resume target if we are not waiting on a fileio
	 * operation to complete.
	 */
	if (semihosting->is_resumable && !semihosting->hit_fileio) {
		/* Resume right after the EBREAK 4 bytes instruction. */
		*retval = target_resume(target, 0, dpc+4, 0, 0);
		if (*retval != ERROR_OK) {
			LOG_ERROR("Failed to resume target");
			return 0;
		}

		return 1;
	}

	return 0;
}

/* -------------------------------------------------------------------------
 * Local functions. */

/**
 * Called via semihosting->setup() later, after the target is known,
 * usually on the first semihosting command.
 */
static int riscv_semihosting_setup(struct target *target, int enable)
{
	LOG_DEBUG("enable=%d", enable);

	struct semihosting *semihosting = target->semihosting;
	if (semihosting)
		semihosting->setup_time = clock();

	return ERROR_OK;
}

static int riscv_semihosting_post_result(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		/* If not enabled, silently ignored. */
		return 0;
	}

	LOG_DEBUG("0x%" PRIx64, semihosting->result);
	riscv_set_register(target, GDB_REGNO_A0, semihosting->result);
	return 0;
}
