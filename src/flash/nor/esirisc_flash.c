/***************************************************************************
 *   Copyright (C) 2018 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 *   James Zhao <hjz@squareup.com>                                         *
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

#include <flash/common.h>
#include <flash/nor/imp.h>
#include <helper/command.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <helper/types.h>
#include <target/esirisc.h>
#include <target/target.h>

/* eSi-TSMC Flash Registers */
#define CONTROL				0x00	/* Control Register */
#define TIMING0				0x04	/* Timing Register 0 */
#define TIMING1				0x08	/* Timing Register 1 */
#define TIMING2				0x0c	/* Timing Register 2 */
#define UNLOCK1				0x18	/* Unlock 1 */
#define UNLOCK2				0x1c	/* Unlock 2 */
#define ADDRESS				0x20	/* Erase/Program Address */
#define PB_DATA				0x24	/* Program Buffer Data */
#define PB_INDEX			0x28	/* Program Buffer Index */
#define STATUS				0x2c	/* Status Register */
#define REDUN_0				0x30	/* Redundant Address 0 */
#define REDUN_1				0x34	/* Redundant Address 1 */

/* Control Fields */
#define CONTROL_SLM			(1<<0)	/* Sleep Mode */
#define CONTROL_WP			(1<<1)	/* Register Write Protect */
#define CONTROL_E			(1<<3)	/* Erase */
#define CONTROL_EP			(1<<4)	/* Erase Page */
#define CONTROL_P			(1<<5)	/* Program Flash */
#define CONTROL_ERC			(1<<6)	/* Erase Reference Cell */
#define CONTROL_R			(1<<7)	/* Recall Trim Code */
#define CONTROL_AP			(1<<8)	/* Auto-Program */

/* Timing Fields */
#define TIMING0_R(x)		(((x) <<  0) & 0x3f)		/* Read Wait States */
#define TIMING0_F(x)		(((x) << 16) & 0xffff0000)	/* Tnvh Clock Cycles */
#define TIMING1_E(x)		(((x) <<  0) & 0xffffff)	/* Tme/Terase/Tre Clock Cycles */
#define TIMING2_P(x)		(((x) <<  0) & 0xffff)		/* Tprog Clock Cycles */
#define TIMING2_H(x)		(((x) << 16) & 0xff0000)	/* Clock Cycles in 100ns */
#define TIMING2_T(x)		(((x) << 24) & 0xf000000)	/* Clock Cycles in 10ns */

/* Status Fields */
#define STATUS_BUSY			(1<<0)	/* Busy (Erase/Program) */
#define STATUS_WER			(1<<1)	/* Write Protect Error */
#define STATUS_DR			(1<<2)	/* Disable Redundancy */
#define STATUS_DIS			(1<<3)	/* Discharged */
#define STATUS_BO			(1<<4)	/* Brown Out */

/* Redundant Address Fields */
#define REDUN_R				(1<<0)						/* Used */
#define REDUN_P(x)			(((x) << 12) & 0x7f000)		/* Redundant Page Address */

/*
 * The eSi-TSMC Flash manual provides two sets of timings based on the
 * underlying flash process. By default, 90nm is assumed.
 */
#if 0 /* 55nm */
#define TNVH				5000		/* 5us   */
#define TME					80000000	/* 80ms  */
#define TERASE				160000000	/* 160ms */
#define TRE					100000000	/* 100ms */
#define TPROG				8000		/* 8us   */
#else /* 90nm */
#define TNVH				5000		/* 5us   */
#define TME					20000000	/* 20ms  */
#define TERASE				40000000	/* 40ms  */
#define TRE					40000000	/* 40ms  */
#define TPROG				40000		/* 40us  */
#endif

#define CONTROL_TIMEOUT		5000		/* 5s    */
#define FLASH_PAGE_SIZE		4096
#define PB_MAX				32

#define NUM_NS_PER_S		1000000000ULL

struct esirisc_flash_bank {
	bool probed;
	uint32_t cfg;
	uint32_t clock;
	uint32_t wait_states;
};

static const struct command_registration esirisc_flash_command_handlers[];

FLASH_BANK_COMMAND_HANDLER(esirisc_flash_bank_command)
{
	struct esirisc_flash_bank *esirisc_info;

	if (CMD_ARGC < 9)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esirisc_info = calloc(1, sizeof(struct esirisc_flash_bank));

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], esirisc_info->cfg);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], esirisc_info->clock);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], esirisc_info->wait_states);

	bank->driver_priv = esirisc_info;

	/* register commands using existing esirisc context */
	register_commands(CMD_CTX, "esirisc", esirisc_flash_command_handlers);

	return ERROR_OK;
}

/*
 * Register writes are ignored if the control.WP flag is set; the
 * following sequence is required to modify this flag even when
 * protection is disabled.
 */
static int esirisc_flash_unlock(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;

	target_write_u32(target, esirisc_info->cfg + UNLOCK1, 0x7123);
	target_write_u32(target, esirisc_info->cfg + UNLOCK2, 0x812a);
	target_write_u32(target, esirisc_info->cfg + UNLOCK1, 0xbee1);

	return ERROR_OK;
}

static int esirisc_flash_disable_protect(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t control;

	target_read_u32(target, esirisc_info->cfg + CONTROL, &control);
	if (!(control & CONTROL_WP))
		return ERROR_OK;

	(void)esirisc_flash_unlock(bank);

	control &= ~CONTROL_WP;

	target_write_u32(target, esirisc_info->cfg + CONTROL, control);

	return ERROR_OK;
}

static int esirisc_flash_enable_protect(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t control;

	target_read_u32(target, esirisc_info->cfg + CONTROL, &control);
	if (control & CONTROL_WP)
		return ERROR_OK;

	(void)esirisc_flash_unlock(bank);

	control |= CONTROL_WP;

	target_write_u32(target, esirisc_info->cfg + CONTROL, control);

	return ERROR_OK;
}

static int esirisc_flash_check_status(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t status;

	target_read_u32(target, esirisc_info->cfg + STATUS, &status);
	if (status & STATUS_WER) {
		LOG_ERROR("%s: bad status: 0x%" PRIx32, bank->name, status);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int esirisc_flash_clear_status(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;

	target_write_u32(target, esirisc_info->cfg + STATUS, STATUS_WER);

	return ERROR_OK;
}

static int esirisc_flash_wait(struct flash_bank *bank, int ms)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t status;
	int64_t t;

	t = timeval_ms();
	for (;;) {
		target_read_u32(target, esirisc_info->cfg + STATUS, &status);
		if (!(status & STATUS_BUSY))
			return ERROR_OK;

		if ((timeval_ms() - t) > ms)
			return ERROR_TARGET_TIMEOUT;

		keep_alive();
	}
}

static int esirisc_flash_control(struct flash_bank *bank, uint32_t control)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;

	esirisc_flash_clear_status(bank);

	target_write_u32(target, esirisc_info->cfg + CONTROL, control);

	int retval = esirisc_flash_wait(bank, CONTROL_TIMEOUT);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: control timed out: 0x%" PRIx32, bank->name, control);
		return retval;
	}

	return esirisc_flash_check_status(bank);
}

static int esirisc_flash_recall(struct flash_bank *bank)
{
	return esirisc_flash_control(bank, CONTROL_R);
}

static int esirisc_flash_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	(void)esirisc_flash_disable_protect(bank);

	for (unsigned int page = first; page < last; ++page) {
		uint32_t address = page * FLASH_PAGE_SIZE;

		target_write_u32(target, esirisc_info->cfg + ADDRESS, address);

		retval = esirisc_flash_control(bank, CONTROL_EP);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to erase address: 0x%" PRIx32, bank->name, address);
			break;
		}
	}

	(void)esirisc_flash_enable_protect(bank);

	return retval;
}

static int esirisc_flash_mass_erase(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	(void)esirisc_flash_disable_protect(bank);

	target_write_u32(target, esirisc_info->cfg + ADDRESS, 0);

	retval = esirisc_flash_control(bank, CONTROL_E);
	if (retval != ERROR_OK)
		LOG_ERROR("%s: failed to mass erase", bank->name);

	(void)esirisc_flash_enable_protect(bank);

	return retval;
}

/*
 * Per TSMC, the reference cell should be erased once per sample. This
 * is typically done during wafer sort, however we include support for
 * those that may need to calibrate flash at a later time.
 */
static int esirisc_flash_ref_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	(void)esirisc_flash_disable_protect(bank);

	retval = esirisc_flash_control(bank, CONTROL_ERC);
	if (retval != ERROR_OK)
		LOG_ERROR("%s: failed to erase reference cell", bank->name);

	(void)esirisc_flash_enable_protect(bank);

	return retval;
}

static int esirisc_flash_fill_pb(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t count)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	struct esirisc_common *esirisc = target_to_esirisc(target);

	/*
	 * The pb_index register is auto-incremented when pb_data is written
	 * and should be cleared before each operation.
	 */
	target_write_u32(target, esirisc_info->cfg + PB_INDEX, 0);

	/*
	 * The width of the pb_data register depends on the underlying
	 * target; writing one byte at a time incurs a significant
	 * performance penalty and should be avoided.
	 */
	while (count > 0) {
		uint32_t max_bytes = DIV_ROUND_UP(esirisc->num_bits, 8);
		uint32_t num_bytes = MIN(count, max_bytes);

		target_write_buffer(target, esirisc_info->cfg + PB_DATA, num_bytes, buffer);

		buffer += num_bytes;
		count -= num_bytes;
	}

	return ERROR_OK;
}

static int esirisc_flash_write(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	(void)esirisc_flash_disable_protect(bank);

	/*
	 * The address register is auto-incremented based on the contents of
	 * the pb_index register after each operation completes. It can be
	 * set once provided pb_index is cleared before each operation.
	 */
	target_write_u32(target, esirisc_info->cfg + ADDRESS, offset);

	/*
	 * Care must be taken when filling the program buffer; a maximum of
	 * 32 bytes may be written at a time and may not cross a 32-byte
	 * boundary based on the current offset.
	 */
	while (count > 0) {
		uint32_t max_bytes = PB_MAX - (offset & 0x1f);
		uint32_t num_bytes = MIN(count, max_bytes);

		esirisc_flash_fill_pb(bank, buffer, num_bytes);

		retval = esirisc_flash_control(bank, CONTROL_P);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s: failed to program address: 0x%" PRIx32, bank->name, offset);
			break;
		}

		buffer += num_bytes;
		offset += num_bytes;
		count -= num_bytes;
	}

	(void)esirisc_flash_enable_protect(bank);

	return retval;
}

static uint32_t esirisc_flash_num_cycles(struct flash_bank *bank, uint64_t ns)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;

	/* apply scaling factor to avoid truncation */
	uint64_t hz = (uint64_t)esirisc_info->clock * 1000;
	uint64_t num_cycles = ((hz / NUM_NS_PER_S) * ns) / 1000;

	if (hz % NUM_NS_PER_S > 0)
		num_cycles++;

	return num_cycles;
}

static int esirisc_flash_init(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t value;
	int retval;

	(void)esirisc_flash_disable_protect(bank);

	/* initialize timing registers */
	value = TIMING0_F(esirisc_flash_num_cycles(bank, TNVH))
			| TIMING0_R(esirisc_info->wait_states);

	LOG_DEBUG("TIMING0: 0x%" PRIx32, value);
	target_write_u32(target, esirisc_info->cfg + TIMING0, value);

	value = TIMING1_E(esirisc_flash_num_cycles(bank, TERASE));

	LOG_DEBUG("TIMING1: 0x%" PRIx32, value);
	target_write_u32(target, esirisc_info->cfg + TIMING1, value);

	value = TIMING2_T(esirisc_flash_num_cycles(bank, 10))
			| TIMING2_H(esirisc_flash_num_cycles(bank, 100))
			| TIMING2_P(esirisc_flash_num_cycles(bank, TPROG));

	LOG_DEBUG("TIMING2: 0x%" PRIx32, value);
	target_write_u32(target, esirisc_info->cfg + TIMING2, value);

	/* recall trim code */
	retval = esirisc_flash_recall(bank);
	if (retval != ERROR_OK)
		LOG_ERROR("%s: failed to recall trim code", bank->name);

	(void)esirisc_flash_enable_protect(bank);

	return retval;
}

static int esirisc_flash_probe(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	bank->num_sectors = bank->size / FLASH_PAGE_SIZE;
	bank->sectors = alloc_block_array(0, FLASH_PAGE_SIZE, bank->num_sectors);

	retval = esirisc_flash_init(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s: failed to initialize bank", bank->name);
		return retval;
	}

	esirisc_info->probed = true;

	return ERROR_OK;
}

static int esirisc_flash_auto_probe(struct flash_bank *bank)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;

	if (esirisc_info->probed)
		return ERROR_OK;

	return esirisc_flash_probe(bank);
}

static int esirisc_flash_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct esirisc_flash_bank *esirisc_info = bank->driver_priv;

	command_print_sameline(cmd,
			"%4s cfg at 0x%" PRIx32 ", clock %" PRIu32 ", wait_states %" PRIu32,
			"",	/* align with first line */
			esirisc_info->cfg,
			esirisc_info->clock,
			esirisc_info->wait_states);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esirisc_flash_mass_erase_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = esirisc_flash_mass_erase(bank);

	command_print(CMD, "mass erase %s",
			(retval == ERROR_OK) ? "successful" : "failed");

	return retval;
}

COMMAND_HANDLER(handle_esirisc_flash_ref_erase_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = esirisc_flash_ref_erase(bank);

	command_print(CMD, "erase reference cell %s",
			(retval == ERROR_OK) ? "successful" : "failed");

	return retval;
}

static const struct command_registration esirisc_flash_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = handle_esirisc_flash_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "erase all pages in data memory",
		.usage = "bank_id",
	},
	{
		.name = "ref_erase",
		.handler = handle_esirisc_flash_ref_erase_command,
		.mode = COMMAND_EXEC,
		.help = "erase reference cell (uncommon)",
		.usage = "bank_id",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esirisc_flash_command_handlers[] = {
	{
		.name = "flash",
		.mode = COMMAND_EXEC,
		.help = "eSi-TSMC Flash command group",
		.usage = "",
		.chain = esirisc_flash_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver esirisc_flash = {
	.name = "esirisc",
	.usage = "flash bank bank_id 'esirisc' base_address size_bytes 0 0 target "
			"cfg_address clock_hz wait_states",
	.flash_bank_command = esirisc_flash_bank_command,
	.erase = esirisc_flash_erase,
	.write = esirisc_flash_write,
	.read = default_flash_read,
	.probe = esirisc_flash_probe,
	.auto_probe = esirisc_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = esirisc_flash_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
