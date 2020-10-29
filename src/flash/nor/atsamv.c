/***************************************************************************
 *   Copyright (C) 2009 by Duane Ellis                                     *
 *   openocd@duaneellis.com                                                *
 *                                                                         *
 *   Copyright (C) 2010 by Olaf Lüke (at91sam3s* support)                  *
 *   olaf@uni-paderborn.de                                                 *
 *                                                                         *
 *   Copyright (C) 2011 by Olivier Schonken, Jim Norris                    *
 *   (at91sam3x* & at91sam4 support)*                                      *
 *                                                                         *
 *   Copyright (C) 2015 Morgan Quigley                                     *
 *   (atsamv, atsams, and atsame support)                                  *
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

/* Some of the lower level code was based on code supplied by
 * ATMEL under this copyright. */

/* BEGIN ATMEL COPYRIGHT */
/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2009, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */
/* END ATMEL COPYRIGHT */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/time_support.h>

#define REG_NAME_WIDTH  (12)

#define SAMV_EFC_FCMD_GETD   (0x0)	/* (EFC) Get Flash Descriptor */
#define SAMV_EFC_FCMD_WP     (0x1)	/* (EFC) Write Page */
#define SAMV_EFC_FCMD_WPL    (0x2)	/* (EFC) Write Page and Lock */
#define SAMV_EFC_FCMD_EWP    (0x3)	/* (EFC) Erase Page and Write Page */
#define SAMV_EFC_FCMD_EWPL   (0x4)	/* (EFC) Erase Page, Write Page then Lock*/
#define SAMV_EFC_FCMD_EA     (0x5)	/* (EFC) Erase All */
#define SAMV_EFC_FCMD_EPA    (0x7)	/* (EFC) Erase pages */
#define SAMV_EFC_FCMD_SLB    (0x8)	/* (EFC) Set Lock Bit */
#define SAMV_EFC_FCMD_CLB    (0x9)	/* (EFC) Clear Lock Bit */
#define SAMV_EFC_FCMD_GLB    (0xA)	/* (EFC) Get Lock Bit */
#define SAMV_EFC_FCMD_SFB    (0xB)	/* (EFC) Set Fuse Bit */
#define SAMV_EFC_FCMD_CFB    (0xC)	/* (EFC) Clear Fuse Bit */
#define SAMV_EFC_FCMD_GFB    (0xD)	/* (EFC) Get Fuse Bit */

#define OFFSET_EFC_FMR    0
#define OFFSET_EFC_FCR    4
#define OFFSET_EFC_FSR    8
#define OFFSET_EFC_FRR   12

#define SAMV_CHIPID_CIDR       (0x400E0940)
#define SAMV_NUM_GPNVM_BITS              9
#define SAMV_CONTROLLER_ADDR   (0x400e0c00)
#define SAMV_SECTOR_SIZE             16384
#define SAMV_PAGE_SIZE                 512
#define SAMV_FLASH_BASE         0x00400000

extern const struct flash_driver atsamv_flash;

struct samv_flash_bank {
	bool      probed;
	unsigned size_bytes;
	unsigned gpnvm[SAMV_NUM_GPNVM_BITS];
};

/* The actual sector size of the SAMV7 flash memory is 128K bytes.
 * 16 sectors for a 2048KB device. The lock regions are 16KB per lock
 * region, with a 2048KB device having 128 lock regions.
 * For the best results, num_sectors is thus set to the number of lock
 * regions, and the sector_size set to the lock region size. Page
 * erases are used to erase 16KB sections when programming */

static int samv_efc_get_status(struct target *target, uint32_t *v)
{
	int r = target_read_u32(target, SAMV_CONTROLLER_ADDR + OFFSET_EFC_FSR, v);
	return r;
}

static int samv_efc_get_result(struct target *target, uint32_t *v)
{
	uint32_t rv;
	int r = target_read_u32(target, SAMV_CONTROLLER_ADDR + OFFSET_EFC_FRR, &rv);
	if (v)
		*v = rv;
	return r;
}

static int samv_efc_start_command(struct target *target,
		unsigned command, unsigned argument)
{
	uint32_t v;
	samv_efc_get_status(target, &v);
	if (!(v & 1)) {
		LOG_ERROR("flash controller is not ready");
		return ERROR_FAIL;
	}

	v = (0x5A << 24) | (argument << 8) | command;
	LOG_DEBUG("starting flash command: 0x%08x", (unsigned int)(v));
	int r = target_write_u32(target, SAMV_CONTROLLER_ADDR + OFFSET_EFC_FCR, v);
	if (r != ERROR_OK)
		LOG_DEBUG("write failed");
	return r;
}

static int samv_efc_perform_command(struct target *target,
		unsigned command, unsigned argument, uint32_t *status)
{
	int r;
	uint32_t v;
	int64_t ms_now, ms_end;

	if (status)
		*status = 0;

	r = samv_efc_start_command(target, command, argument);
	if (r != ERROR_OK)
		return r;

	ms_end = 10000 + timeval_ms();

	do {
		r = samv_efc_get_status(target, &v);
		if (r != ERROR_OK)
			return r;
		ms_now = timeval_ms();
		if (ms_now > ms_end) {
			/* error */
			LOG_ERROR("Command timeout");
			return ERROR_FAIL;
		}
	} while ((v & 1) == 0);

	/* if requested, copy the flash controller error bits back to the caller */
	if (status)
		*status = (v & 0x6);
	return ERROR_OK;
}

static int samv_erase_pages(struct target *target,
		int first_page, int num_pages, uint32_t *status)
{
	uint8_t erase_pages;
	switch (num_pages) {
		case 4:
			erase_pages = 0x00;
			break;
		case 8:
			erase_pages = 0x01;
			break;
		case 16:
			erase_pages = 0x02;
			break;
		case 32:
			erase_pages = 0x03;
			break;
		default:
			erase_pages = 0x00;
			break;
	}

	/* SAMV_EFC_FCMD_EPA
	 * According to the datasheet FARG[15:2] defines the page from which
	 * the erase will start.This page must be modulo 4, 8, 16 or 32
	 * according to the number of pages to erase. FARG[1:0] defines the
	 * number of pages to be erased. Previously (firstpage << 2) was used
	 * to conform to this, seems it should not be shifted...
	 */
	return samv_efc_perform_command(target, SAMV_EFC_FCMD_EPA,
			first_page | erase_pages, status);
}

static int samv_get_gpnvm(struct target *target, unsigned gpnvm, unsigned *out)
{
	uint32_t v;
	int r;

	if (gpnvm >= SAMV_NUM_GPNVM_BITS) {
		LOG_ERROR("invalid gpnvm %d, max: %d", gpnvm, SAMV_NUM_GPNVM_BITS);
		return ERROR_FAIL;
	}

	r = samv_efc_perform_command(target, SAMV_EFC_FCMD_GFB, 0, NULL);
	if (r != ERROR_OK) {
		LOG_ERROR("samv_get_gpnvm failed");
		return r;
	}

	r = samv_efc_get_result(target, &v);

	if (out)
		*out = (v >> gpnvm) & 1;

	return r;
}

static int samv_clear_gpnvm(struct target *target, unsigned gpnvm)
{
	int r;
	unsigned v;

	if (gpnvm >= SAMV_NUM_GPNVM_BITS) {
		LOG_ERROR("invalid gpnvm %d, max: %d", gpnvm, SAMV_NUM_GPNVM_BITS);
		return ERROR_FAIL;
	}
	r = samv_get_gpnvm(target, gpnvm, &v);
	if (r != ERROR_OK) {
		LOG_DEBUG("get gpnvm failed: %d", r);
		return r;
	}
	r = samv_efc_perform_command(target, SAMV_EFC_FCMD_CFB, gpnvm, NULL);
	LOG_DEBUG("clear gpnvm result: %d", r);
	return r;
}

static int samv_set_gpnvm(struct target *target, unsigned gpnvm)
{
	int r;
	unsigned v;
	if (gpnvm >= SAMV_NUM_GPNVM_BITS) {
		LOG_ERROR("invalid gpnvm %d, max: %d", gpnvm, SAMV_NUM_GPNVM_BITS);
		return ERROR_FAIL;
	}

	r = samv_get_gpnvm(target, gpnvm, &v);
	if (r != ERROR_OK)
		return r;
	if (v) {
		r = ERROR_OK; /* the gpnvm bit is already set */
	} else {
		/* we need to set it */
		r = samv_efc_perform_command(target, SAMV_EFC_FCMD_SFB, gpnvm, NULL);
	}
	return r;
}

static int samv_flash_unlock(struct target *target,
		unsigned start_sector, unsigned end_sector)
{
	int r;
	uint32_t status;
	uint32_t pg;
	uint32_t pages_per_sector;

	/* todo: look into this... i think this should be done on lock regions */
	pages_per_sector = SAMV_SECTOR_SIZE / SAMV_PAGE_SIZE;
	while (start_sector <= end_sector) {
		pg = start_sector * pages_per_sector;
		r = samv_efc_perform_command(target, SAMV_EFC_FCMD_CLB, pg, &status);
		if (r != ERROR_OK)
			return r;
		start_sector++;
	}
	return ERROR_OK;
}

static int samv_flash_lock(struct target *target,
		unsigned start_sector, unsigned end_sector)
{
	uint32_t status;
	uint32_t pg;
	uint32_t pages_per_sector;
	int r;

	/* todo: look into this... i think this should be done on lock regions */
	pages_per_sector = SAMV_SECTOR_SIZE / SAMV_PAGE_SIZE;
	while (start_sector <= end_sector) {
		pg = start_sector * pages_per_sector;
		r = samv_efc_perform_command(target, SAMV_EFC_FCMD_SLB, pg, &status);
		if (r != ERROR_OK)
			return r;
		start_sector++;
	}
	return ERROR_OK;
}

static int samv_protect_check(struct flash_bank *bank)
{
	int r;
	uint32_t v[4] = {0};

	r = samv_efc_perform_command(bank->target, SAMV_EFC_FCMD_GLB, 0, NULL);
	if (r == ERROR_OK)	{
		samv_efc_get_result(bank->target, &v[0]);
		samv_efc_get_result(bank->target, &v[1]);
		samv_efc_get_result(bank->target, &v[2]);
		r = samv_efc_get_result(bank->target, &v[3]);
	}
	if (r != ERROR_OK)
		return r;

	for (unsigned int x = 0; x < bank->num_sectors; x++)
		bank->sectors[x].is_protected = (!!(v[x >> 5] & (1 << (x % 32))));
	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(samv_flash_bank_command)
{
	LOG_INFO("flash bank command");
	struct samv_flash_bank *samv_info;
	samv_info = calloc(1, sizeof(struct samv_flash_bank));
	bank->driver_priv = samv_info;
	return ERROR_OK;
}

static int samv_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	return target_read_u32(bank->target, SAMV_CHIPID_CIDR, device_id);
}

static int samv_probe(struct flash_bank *bank)
{
	uint32_t device_id;
	int r = samv_get_device_id(bank, &device_id);
	if (r != ERROR_OK)
		return r;
	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	uint8_t eproc = (device_id >> 5) & 0x7;
	if (eproc != 0) {
		LOG_ERROR("unexpected eproc code: %d was expecting 0 (Cortex-M7)", eproc);
		return ERROR_FAIL;
	}

	uint8_t nvm_size_code = (device_id >> 8) & 0xf;
	switch (nvm_size_code) {
		case 10:
			bank->size = 512 * 1024;
			break;
		case 12:
			bank->size = 1024 * 1024;
			break;
		case 14:
			bank->size = 2048 * 1024;
			break;
		default:
			LOG_ERROR("unrecognized flash size code: %d", nvm_size_code);
			return ERROR_FAIL;
	}

	struct samv_flash_bank *samv_info = bank->driver_priv;
	samv_info->size_bytes = bank->size;
	samv_info->probed = true;

	bank->base = SAMV_FLASH_BASE;
	bank->num_sectors = bank->size / SAMV_SECTOR_SIZE;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	for (unsigned int s = 0; s < bank->num_sectors; s++) {
		bank->sectors[s].size = SAMV_SECTOR_SIZE;
		bank->sectors[s].offset = s * SAMV_SECTOR_SIZE;
		bank->sectors[s].is_erased = -1;
		bank->sectors[s].is_protected = -1;
	}

	r = samv_protect_check(bank);
	if (r != ERROR_OK)
		return r;

	return ERROR_OK;
}

static int samv_auto_probe(struct flash_bank *bank)
{
	struct samv_flash_bank *samv_info = bank->driver_priv;
	if (samv_info->probed)
		return ERROR_OK;
	return samv_probe(bank);
}

static int samv_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	const int page_count = 32; /* 32 pages equals 16 KB lock region */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int r = samv_auto_probe(bank);
	if (r != ERROR_OK)
		return r;

	/* easy case: we've been requested to erase the entire flash */
	if ((first == 0) && ((last + 1) == bank->num_sectors))
		return samv_efc_perform_command(bank->target, SAMV_EFC_FCMD_EA, 0, NULL);

	LOG_INFO("erasing lock regions %u-%u...", first, last);

	for (unsigned int i = first; i <= last; i++) {
		uint32_t status;
		r = samv_erase_pages(bank->target, (i * page_count), page_count, &status);
		LOG_INFO("erasing lock region %u", i);
		if (r != ERROR_OK)
			LOG_ERROR("error performing erase page @ lock region number %u", i);
		if (status & (1 << 2)) {
			LOG_ERROR("lock region %u is locked", i);
			return ERROR_FAIL;
		}
		if (status & (1 << 1)) {
			LOG_ERROR("flash command error @lock region %u", i);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int samv_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int r;
	if (set)
		r = samv_flash_lock(bank->target, first, last);
	else
		r = samv_flash_unlock(bank->target, first, last);

	return r;
}

static int samv_page_read(struct target *target,
		unsigned page_num, uint8_t *buf)
{
	uint32_t addr = SAMV_FLASH_BASE + page_num * SAMV_PAGE_SIZE;
	int r = target_read_memory(target, addr, 4, SAMV_PAGE_SIZE / 4, buf);
	if (r != ERROR_OK)
		LOG_ERROR("flash program failed to read page @ 0x%08x",
				(unsigned int)(addr));
	return r;
}

static int samv_page_write(struct target *target,
		unsigned pagenum, const uint8_t *buf)
{
	uint32_t status;
	const uint32_t addr = SAMV_FLASH_BASE + pagenum * SAMV_PAGE_SIZE;
	int r;

	LOG_DEBUG("write page %u at address 0x%08x", pagenum, (unsigned int)addr);
	r = target_write_memory(target, addr, 4, SAMV_PAGE_SIZE / 4, buf);
	if (r != ERROR_OK) {
		LOG_ERROR("failed to buffer page at 0x%08x", (unsigned int)addr);
		return r;
	}

	r = samv_efc_perform_command(target, SAMV_EFC_FCMD_WP, pagenum, &status);
	if (r != ERROR_OK)
		LOG_ERROR("error performing write page at 0x%08x", (unsigned int)addr);
	if (status & (1 << 2)) {
		LOG_ERROR("page at 0x%08x is locked", (unsigned int)addr);
		return ERROR_FAIL;
	}
	if (status & (1 << 1)) {
		LOG_ERROR("flash command error at 0x%08x", (unsigned int)addr);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int samv_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (count == 0)
		return ERROR_OK;

	if ((offset + count) > bank->size) {
		LOG_ERROR("flash write error - past end of bank");
		LOG_ERROR(" offset: 0x%08x, count 0x%08x, bank end: 0x%08x",
				(unsigned int)(offset),
				(unsigned int)(count),
				(unsigned int)(bank->size));
		return ERROR_FAIL;
	}

	uint8_t pagebuffer[SAMV_PAGE_SIZE] = {0};
	uint32_t page_cur = offset / SAMV_PAGE_SIZE;
	uint32_t page_end = (offset + count - 1) / SAMV_PAGE_SIZE;

	LOG_DEBUG("offset: 0x%08x, count: 0x%08x",
			(unsigned int)(offset), (unsigned int)(count));
	LOG_DEBUG("page start: %d, page end: %d", (int)(page_cur), (int)(page_end));

	/* Special case: all one page */
	/* Otherwise:                 */
	/*    (1) non-aligned start   */
	/*    (2) body pages          */
	/*    (3) non-aligned end.    */

	int r;
	uint32_t page_offset;

	/* handle special case - all one page. */
	if (page_cur == page_end) {
		LOG_DEBUG("special case, all in one page");
		r = samv_page_read(bank->target, page_cur, pagebuffer);
		if (r != ERROR_OK)
			return r;

		page_offset = offset & (SAMV_PAGE_SIZE-1);
		memcpy(pagebuffer + page_offset, buffer, count);

		r = samv_page_write(bank->target, page_cur, pagebuffer);
		if (r != ERROR_OK)
			return r;
		return ERROR_OK;
	}

	/* step 1) handle the non-aligned starting address */
	page_offset = offset & (SAMV_PAGE_SIZE - 1);
	if (page_offset) {
		LOG_DEBUG("non-aligned start");
		/* read the partial page */
		r = samv_page_read(bank->target, page_cur, pagebuffer);
		if (r != ERROR_OK)
			return r;

		/* over-write with new data */
		uint32_t n = SAMV_PAGE_SIZE - page_offset;
		memcpy(pagebuffer + page_offset, buffer, n);

		r = samv_page_write(bank->target, page_cur, pagebuffer);
		if (r != ERROR_OK)
			return r;

		count  -= n;
		offset += n;
		buffer += n;
		page_cur++;
	}

	/* By checking that offset is correct here, we also fix a clang warning */
	assert(offset % SAMV_PAGE_SIZE == 0);

	/* step 2) handle the full pages */
	LOG_DEBUG("full page loop: cur=%d, end=%d, count = 0x%08x",
			(int)page_cur, (int)page_end, (unsigned int)(count));

	while ((page_cur < page_end) && (count >= SAMV_PAGE_SIZE)) {
		r = samv_page_write(bank->target, page_cur, buffer);
		if (r != ERROR_OK)
			return r;
		count -= SAMV_PAGE_SIZE;
		buffer += SAMV_PAGE_SIZE;
		page_cur += 1;
	}

	/* step 3) write final page, if it's partial (otherwise it's already done) */
	if (count) {
		LOG_DEBUG("final partial page, count = 0x%08x", (unsigned int)(count));
		/* we have a partial page */
		r = samv_page_read(bank->target, page_cur, pagebuffer);
		if (r != ERROR_OK)
			return r;
		memcpy(pagebuffer, buffer, count); /* data goes at start of page */
		r = samv_page_write(bank->target, page_cur, pagebuffer);
		if (r != ERROR_OK)
			return r;
	}
	return ERROR_OK;
}

static int samv_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct samv_flash_bank *samv_info = bank->driver_priv;
	if (!samv_info->probed) {
		int r = samv_probe(bank);
		if (ERROR_OK != r)
			return r;
	}
	snprintf(buf, buf_size, "Cortex-M7 detected with %" PRIu32 " kB flash",
			bank->size / 1024);
	return ERROR_OK;
}

COMMAND_HANDLER(samv_handle_gpnvm_command)
{
	struct flash_bank *bank = get_flash_bank_by_num_noprobe(0);
	if (!bank)
		return ERROR_FAIL;
	struct samv_flash_bank *samv_info = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int r;
	if (!samv_info->probed) {
		r = samv_auto_probe(bank);
		if (r != ERROR_OK)
			return r;
	}

	int who = 0;

	switch (CMD_ARGC) {
		case 0:
			goto showall;
		case 1:
			who = -1;
			break;
		case 2:
			if (!strcmp(CMD_ARGV[0], "show") && !strcmp(CMD_ARGV[1], "all"))
				who = -1;
			else {
				uint32_t v32;
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], v32);
				who = v32;
			}
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	unsigned v;
	if (!strcmp("show", CMD_ARGV[0])) {
		if (who == -1) {
showall:
			r = ERROR_OK;
			for (int x = 0; x < SAMV_NUM_GPNVM_BITS; x++) {
				r = samv_get_gpnvm(target, x, &v);
				if (r != ERROR_OK)
					break;
				command_print(CMD, "samv-gpnvm%u: %u", x, v);
			}
			return r;
		}
		if ((who >= 0) && (((unsigned)who) < SAMV_NUM_GPNVM_BITS)) {
			r = samv_get_gpnvm(target, who, &v);
			if (r != ERROR_OK)
				return r;

			command_print(CMD, "samv-gpnvm%u: %u", who, v);
			return r;
		} else {
			command_print(CMD, "invalid gpnvm: %u", who);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (who == -1) {
		command_print(CMD, "missing gpnvm number");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!strcmp("set", CMD_ARGV[0]))
		r = samv_set_gpnvm(target, who);
	else if (!strcmp("clr", CMD_ARGV[0]) || !strcmp("clear", CMD_ARGV[0]))
		r = samv_clear_gpnvm(target, who);
	else {
		command_print(CMD, "unknown command: %s", CMD_ARGV[0]);
		r = ERROR_COMMAND_SYNTAX_ERROR;
	}
	return r;
}

static const struct command_registration atsamv_exec_command_handlers[] = {
	{
		.name = "gpnvm",
		.handler = samv_handle_gpnvm_command,
		.mode = COMMAND_EXEC,
		.usage = "[('clr'|'set'|'show') bitnum]",
		.help = "Without arguments, shows all bits in the gpnvm "
			"register.  Otherwise, clears, sets, or shows one "
			"General Purpose Non-Volatile Memory (gpnvm) bit.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration atsamv_command_handlers[] = {
	{
		.name = "atsamv",
		.mode = COMMAND_ANY,
		.help = "atsamv flash command group",
		.usage = "",
		.chain = atsamv_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver atsamv_flash = {
	.name = "atsamv",
	.commands = atsamv_command_handlers,
	.flash_bank_command = samv_flash_bank_command,
	.erase = samv_erase,
	.protect = samv_protect,
	.write = samv_write,
	.read = default_flash_read,
	.probe = samv_probe,
	.auto_probe = samv_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = samv_protect_check,
	.info = samv_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
