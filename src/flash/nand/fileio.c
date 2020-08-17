/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2002 Thomas Gleixner <tglx@linutronix.de>               *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *                                                                         *
 *   Partially based on drivers/mtd/nand_ids.c from Linux.                 *
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

#include "core.h"
#include "fileio.h"

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 6, 7},
	.oobfree = {
		{.offset = 8,
		 .length = 8}
	}
};

static struct nand_ecclayout nand_oob_64 = {
	.eccbytes = 24,
	.eccpos = {
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63
	},
	.oobfree = {
		{.offset = 2,
		 .length = 38}
	}
};

void nand_fileio_init(struct nand_fileio_state *state)
{
	memset(state, 0, sizeof(*state));
	state->oob_format = NAND_OOB_NONE;
}

int nand_fileio_start(struct command_invocation *cmd,
	struct nand_device *nand, const char *filename, int filemode,
	struct nand_fileio_state *state)
{
	if (state->address % nand->page_size) {
		command_print(cmd, "only page-aligned addresses are supported");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	duration_start(&state->bench);

	if (NULL != filename) {
		int retval = fileio_open(&state->fileio, filename, filemode, FILEIO_BINARY);
		if (ERROR_OK != retval) {
			const char *msg = (FILEIO_READ == filemode) ? "read" : "write";
			command_print(cmd, "failed to open '%s' for %s access",
				filename, msg);
			return retval;
		}
		state->file_opened = true;
	}

	if (!(state->oob_format & NAND_OOB_ONLY)) {
		state->page_size = nand->page_size;
		state->page = malloc(nand->page_size);
	}

	if (state->oob_format & (NAND_OOB_RAW | NAND_OOB_SW_ECC | NAND_OOB_SW_ECC_KW)) {
		if (nand->page_size == 512) {
			state->oob_size = 16;
			state->eccpos = nand_oob_16.eccpos;
		} else if (nand->page_size == 2048)   {
			state->oob_size = 64;
			state->eccpos = nand_oob_64.eccpos;
		}
		state->oob = malloc(state->oob_size);
	}

	return ERROR_OK;
}
int nand_fileio_cleanup(struct nand_fileio_state *state)
{
	if (state->file_opened)
		fileio_close(state->fileio);

	free(state->oob);
	state->oob = NULL;

	free(state->page);
	state->page = NULL;
	return ERROR_OK;
}
int nand_fileio_finish(struct nand_fileio_state *state)
{
	nand_fileio_cleanup(state);
	return duration_measure(&state->bench);
}

COMMAND_HELPER(nand_fileio_parse_args, struct nand_fileio_state *state,
	struct nand_device **dev, enum fileio_access filemode,
	bool need_size, bool sw_ecc)
{
	nand_fileio_init(state);

	unsigned minargs = need_size ? 4 : 3;
	if (CMD_ARGC < minargs)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct nand_device *nand;
	int retval = CALL_COMMAND_HANDLER(nand_command_get_device, 0, &nand);
	if (ERROR_OK != retval)
		return retval;

	if (NULL == nand->device) {
		command_print(CMD, "#%s: not probed", CMD_ARGV[0]);
		return ERROR_NAND_DEVICE_NOT_PROBED;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], state->address);
	if (need_size) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], state->size);
		if (state->size % nand->page_size) {
			command_print(CMD, "only page-aligned sizes are supported");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (CMD_ARGC > minargs) {
		for (unsigned i = minargs; i < CMD_ARGC; i++) {
			if (!strcmp(CMD_ARGV[i], "oob_raw"))
				state->oob_format |= NAND_OOB_RAW;
			else if (!strcmp(CMD_ARGV[i], "oob_only"))
				state->oob_format |= NAND_OOB_RAW | NAND_OOB_ONLY;
			else if (sw_ecc && !strcmp(CMD_ARGV[i], "oob_softecc"))
				state->oob_format |= NAND_OOB_SW_ECC;
			else if (sw_ecc && !strcmp(CMD_ARGV[i], "oob_softecc_kw"))
				state->oob_format |= NAND_OOB_SW_ECC_KW;
			else {
				command_print(CMD, "unknown option: %s", CMD_ARGV[i]);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		}
	}

	retval = nand_fileio_start(CMD, nand, CMD_ARGV[1], filemode, state);
	if (ERROR_OK != retval)
		return retval;

	if (!need_size) {
		size_t filesize;
		retval = fileio_size(state->fileio, &filesize);
		if (retval != ERROR_OK)
			return retval;
		state->size = filesize;
	}

	*dev = nand;

	return ERROR_OK;
}

/**
 * @returns If no error occurred, returns number of bytes consumed;
 * otherwise, returns a negative error code.)
 */
int nand_fileio_read(struct nand_device *nand, struct nand_fileio_state *s)
{
	size_t total_read = 0;
	size_t one_read;

	if (NULL != s->page) {
		fileio_read(s->fileio, s->page_size, s->page, &one_read);
		if (one_read < s->page_size)
			memset(s->page + one_read, 0xff, s->page_size - one_read);
		total_read += one_read;
	}

	if (s->oob_format & NAND_OOB_SW_ECC) {
		uint8_t ecc[3];
		memset(s->oob, 0xff, s->oob_size);
		for (uint32_t i = 0, j = 0; i < s->page_size; i += 256) {
			nand_calculate_ecc(nand, s->page + i, ecc);
			s->oob[s->eccpos[j++]] = ecc[0];
			s->oob[s->eccpos[j++]] = ecc[1];
			s->oob[s->eccpos[j++]] = ecc[2];
		}
	} else if (s->oob_format & NAND_OOB_SW_ECC_KW)   {
		/*
		 * In this case eccpos is not used as
		 * the ECC data is always stored contiguously
		 * at the end of the OOB area.  It consists
		 * of 10 bytes per 512-byte data block.
		 */
		uint8_t *ecc = s->oob + s->oob_size - s->page_size / 512 * 10;
		memset(s->oob, 0xff, s->oob_size);
		for (uint32_t i = 0; i < s->page_size; i += 512) {
			nand_calculate_ecc_kw(nand, s->page + i, ecc);
			ecc += 10;
		}
	} else if (NULL != s->oob)   {
		fileio_read(s->fileio, s->oob_size, s->oob, &one_read);
		if (one_read < s->oob_size)
			memset(s->oob + one_read, 0xff, s->oob_size - one_read);
		total_read += one_read;
	}
	return total_read;
}
