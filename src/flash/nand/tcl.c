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
#include "imp.h"
#include "fileio.h"
#include <target/target.h>

/* to be removed */
extern struct nand_device *nand_devices;

COMMAND_HANDLER(handle_nand_list_command)
{
	struct nand_device *p;
	int i;

	if (!nand_devices) {
		command_print(CMD, "no NAND flash devices configured");
		return ERROR_OK;
	}

	for (p = nand_devices, i = 0; p; p = p->next, i++) {
		if (p->device)
			command_print(CMD, "#%i: %s (%s) "
				"pagesize: %i, buswidth: %i,\n\t"
				"blocksize: %i, blocks: %i",
				i, p->device->name, p->manufacturer->name,
				p->page_size, p->bus_width,
				p->erase_size, p->num_blocks);
		else
			command_print(CMD, "#%i: not probed", i);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nand_info_command)
{
	int i = 0;
	int j = 0;
	int first = -1;
	int last = -1;

	switch (CMD_ARGC) {
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		case 1:
			first = 0;
			last = INT32_MAX;
			break;
		case 2:
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], i);
			first = last = i;
			i = 0;
			break;
		case 3:
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], first);
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], last);
			break;
	}

	struct nand_device *p;
	int retval = CALL_COMMAND_HANDLER(nand_command_get_device, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	if (NULL == p->device) {
		command_print(CMD, "#%s: not probed", CMD_ARGV[0]);
		return ERROR_OK;
	}

	if (first >= p->num_blocks)
		first = p->num_blocks - 1;

	if (last >= p->num_blocks)
		last = p->num_blocks - 1;

	command_print(CMD,
		"#%i: %s (%s) pagesize: %i, buswidth: %i, erasesize: %i",
		i++,
		p->device->name,
		p->manufacturer->name,
		p->page_size,
		p->bus_width,
		p->erase_size);

	for (j = first; j <= last; j++) {
		char *erase_state, *bad_state;

		if (p->blocks[j].is_erased == 0)
			erase_state = "not erased";
		else if (p->blocks[j].is_erased == 1)
			erase_state = "erased";
		else
			erase_state = "erase state unknown";

		if (p->blocks[j].is_bad == 0)
			bad_state = "";
		else if (p->blocks[j].is_bad == 1)
			bad_state = " (marked bad)";
		else
			bad_state = " (block condition unknown)";

		command_print(CMD,
			"\t#%i: 0x%8.8" PRIx32 " (%" PRIu32 "kB) %s%s",
			j,
			p->blocks[j].offset,
			p->blocks[j].size / 1024,
			erase_state,
			bad_state);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nand_probe_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct nand_device *p;
	int retval = CALL_COMMAND_HANDLER(nand_command_get_device, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	retval = nand_probe(p);
	if (retval == ERROR_OK) {
		command_print(CMD, "NAND flash device '%s (%s)' found",
			p->device->name, p->manufacturer->name);
	}

	return retval;
}

COMMAND_HANDLER(handle_nand_erase_command)
{
	if (CMD_ARGC != 1 && CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct nand_device *p;
	int retval = CALL_COMMAND_HANDLER(nand_command_get_device, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	unsigned long offset;
	unsigned long length;

	/* erase specified part of the chip; or else everything */
	if (CMD_ARGC == 3) {
		unsigned long size = p->erase_size * p->num_blocks;

		COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[1], offset);
		if ((offset % p->erase_size) != 0 || offset >= size)
			return ERROR_COMMAND_SYNTAX_ERROR;

		COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[2], length);
		if ((length == 0) || (length % p->erase_size) != 0
		    || (length + offset) > size)
			return ERROR_COMMAND_SYNTAX_ERROR;

		offset /= p->erase_size;
		length /= p->erase_size;
	} else {
		offset = 0;
		length = p->num_blocks;
	}

	retval = nand_erase(p, offset, offset + length - 1);
	if (retval == ERROR_OK) {
		command_print(CMD, "erased blocks %lu to %lu "
			"on NAND flash device #%s '%s'",
			offset, offset + length - 1,
			CMD_ARGV[0], p->device->name);
	}

	return retval;
}

COMMAND_HANDLER(handle_nand_check_bad_blocks_command)
{
	int first = -1;
	int last = -1;

	if ((CMD_ARGC < 1) || (CMD_ARGC > 3) || (CMD_ARGC == 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct nand_device *p;
	int retval = CALL_COMMAND_HANDLER(nand_command_get_device, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	if (CMD_ARGC == 3) {
		unsigned long offset;
		unsigned long length;

		COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[1], offset);
		if (offset % p->erase_size)
			return ERROR_COMMAND_SYNTAX_ERROR;
		offset /= p->erase_size;

		COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[2], length);
		if (length % p->erase_size)
			return ERROR_COMMAND_SYNTAX_ERROR;

		length -= 1;
		length /= p->erase_size;

		first = offset;
		last = offset + length;
	}

	retval = nand_build_bbt(p, first, last);
	if (retval == ERROR_OK) {
		command_print(CMD, "checked NAND flash device for bad blocks, "
			"use \"nand info\" command to list blocks");
	}

	return retval;
}

COMMAND_HANDLER(handle_nand_write_command)
{
	struct nand_device *nand = NULL;
	struct nand_fileio_state s;
	int retval = CALL_COMMAND_HANDLER(nand_fileio_parse_args,
			&s, &nand, FILEIO_READ, false, true);
	if (ERROR_OK != retval)
		return retval;

	uint32_t total_bytes = s.size;
	while (s.size > 0) {
		int bytes_read = nand_fileio_read(nand, &s);
		if (bytes_read <= 0) {
			command_print(CMD, "error while reading file");
			nand_fileio_cleanup(&s);
			return ERROR_FAIL;
		}
		s.size -= bytes_read;

		retval = nand_write_page(nand, s.address / nand->page_size,
				s.page, s.page_size, s.oob, s.oob_size);
		if (ERROR_OK != retval) {
			command_print(CMD, "failed writing file %s "
				"to NAND flash %s at offset 0x%8.8" PRIx32,
				CMD_ARGV[1], CMD_ARGV[0], s.address);
			nand_fileio_cleanup(&s);
			return retval;
		}
		s.address += s.page_size;
	}

	if (nand_fileio_finish(&s) == ERROR_OK) {
		command_print(CMD, "wrote file %s to NAND flash %s up to "
			"offset 0x%8.8" PRIx32 " in %fs (%0.3f KiB/s)",
			CMD_ARGV[1], CMD_ARGV[0], s.address, duration_elapsed(&s.bench),
			duration_kbps(&s.bench, total_bytes));
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nand_verify_command)
{
	struct nand_device *nand = NULL;
	struct nand_fileio_state file;
	int retval = CALL_COMMAND_HANDLER(nand_fileio_parse_args,
			&file, &nand, FILEIO_READ, false, true);
	if (ERROR_OK != retval)
		return retval;

	struct nand_fileio_state dev;
	nand_fileio_init(&dev);
	dev.address = file.address;
	dev.size = file.size;
	dev.oob_format = file.oob_format;
	retval = nand_fileio_start(CMD, nand, NULL, FILEIO_NONE, &dev);
	if (ERROR_OK != retval)
		return retval;

	while (file.size > 0) {
		retval = nand_read_page(nand, dev.address / dev.page_size,
				dev.page, dev.page_size, dev.oob, dev.oob_size);
		if (ERROR_OK != retval) {
			command_print(CMD, "reading NAND flash page failed");
			nand_fileio_cleanup(&dev);
			nand_fileio_cleanup(&file);
			return retval;
		}

		int bytes_read = nand_fileio_read(nand, &file);
		if (bytes_read <= 0) {
			command_print(CMD, "error while reading file");
			nand_fileio_cleanup(&dev);
			nand_fileio_cleanup(&file);
			return ERROR_FAIL;
		}

		if ((dev.page && memcmp(dev.page, file.page, dev.page_size)) ||
				(dev.oob && memcmp(dev.oob, file.oob, dev.oob_size))) {
			command_print(CMD, "NAND flash contents differ "
				"at 0x%8.8" PRIx32, dev.address);
			nand_fileio_cleanup(&dev);
			nand_fileio_cleanup(&file);
			return ERROR_FAIL;
		}

		file.size -= bytes_read;
		dev.address += nand->page_size;
	}

	if (nand_fileio_finish(&file) == ERROR_OK) {
		command_print(CMD, "verified file %s in NAND flash %s "
			"up to offset 0x%8.8" PRIx32 " in %fs (%0.3f KiB/s)",
			CMD_ARGV[1], CMD_ARGV[0], dev.address, duration_elapsed(&file.bench),
			duration_kbps(&file.bench, dev.size));
	}

	return nand_fileio_cleanup(&dev);
}

COMMAND_HANDLER(handle_nand_dump_command)
{
	size_t filesize;
	struct nand_device *nand = NULL;
	struct nand_fileio_state s;
	int retval = CALL_COMMAND_HANDLER(nand_fileio_parse_args,
			&s, &nand, FILEIO_WRITE, true, false);
	if (ERROR_OK != retval)
		return retval;

	while (s.size > 0) {
		size_t size_written;
		retval = nand_read_page(nand, s.address / nand->page_size,
				s.page, s.page_size, s.oob, s.oob_size);
		if (ERROR_OK != retval) {
			command_print(CMD, "reading NAND flash page failed");
			nand_fileio_cleanup(&s);
			return retval;
		}

		if (NULL != s.page)
			fileio_write(s.fileio, s.page_size, s.page, &size_written);

		if (NULL != s.oob)
			fileio_write(s.fileio, s.oob_size, s.oob, &size_written);

		s.size -= nand->page_size;
		s.address += nand->page_size;
	}

	retval = fileio_size(s.fileio, &filesize);
	if (retval != ERROR_OK)
		return retval;

	if (nand_fileio_finish(&s) == ERROR_OK) {
		command_print(CMD, "dumped %zu bytes in %fs (%0.3f KiB/s)",
			filesize, duration_elapsed(&s.bench),
			duration_kbps(&s.bench, filesize));
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nand_raw_access_command)
{
	if ((CMD_ARGC < 1) || (CMD_ARGC > 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct nand_device *p;
	int retval = CALL_COMMAND_HANDLER(nand_command_get_device, 0, &p);
	if (ERROR_OK != retval)
		return retval;

	if (NULL == p->device) {
		command_print(CMD, "#%s: not probed", CMD_ARGV[0]);
		return ERROR_OK;
	}

	if (CMD_ARGC == 2)
		COMMAND_PARSE_ENABLE(CMD_ARGV[1], p->use_raw);

	const char *msg = p->use_raw ? "enabled" : "disabled";
	command_print(CMD, "raw access is %s", msg);

	return ERROR_OK;
}

static const struct command_registration nand_exec_command_handlers[] = {
	{
		.name = "list",
		.handler = handle_nand_list_command,
		.mode = COMMAND_EXEC,
		.help = "list configured NAND flash devices",
		.usage = "",
	},
	{
		.name = "info",
		.handler = handle_nand_info_command,
		.mode = COMMAND_EXEC,
		.usage = "[banknum | first_bank_num last_bank_num]",
		.help = "print info about one or more NAND flash devices",
	},
	{
		.name = "probe",
		.handler = handle_nand_probe_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "identify NAND flash device",
	},
	{
		.name = "check_bad_blocks",
		.handler = handle_nand_check_bad_blocks_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id [offset length]",
		.help = "check all or part of NAND flash device for bad blocks",
	},
	{
		.name = "erase",
		.handler = handle_nand_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id [offset length]",
		.help = "erase all or subset of blocks on NAND flash device",
	},
	{
		.name = "dump",
		.handler = handle_nand_dump_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename offset length "
			"['oob_raw'|'oob_only']",
		.help = "dump from NAND flash device",
	},
	{
		.name = "verify",
		.handler = handle_nand_verify_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename offset "
			"['oob_raw'|'oob_only'|'oob_softecc'|'oob_softecc_kw']",
		.help = "verify NAND flash device",
	},
	{
		.name = "write",
		.handler = handle_nand_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id filename offset "
			"['oob_raw'|'oob_only'|'oob_softecc'|'oob_softecc_kw']",
		.help = "write to NAND flash device",
	},
	{
		.name = "raw_access",
		.handler = handle_nand_raw_access_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ['enable'|'disable']",
		.help = "raw access to NAND flash device",
	},
	COMMAND_REGISTRATION_DONE
};

static int nand_init(struct command_context *cmd_ctx)
{
	if (!nand_devices)
		return ERROR_OK;
	struct command *parent = command_find_in_context(cmd_ctx, "nand");
	return register_commands(cmd_ctx, parent, nand_exec_command_handlers);
}

COMMAND_HANDLER(handle_nand_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool nand_initialized;
	if (nand_initialized) {
		LOG_INFO("'nand init' has already been called");
		return ERROR_OK;
	}
	nand_initialized = true;

	LOG_DEBUG("Initializing NAND devices...");
	return nand_init(CMD_CTX);
}

static int nand_list_walker(struct nand_flash_controller *c, void *x)
{
	struct command_invocation *cmd = x;
	command_print(cmd, "  %s", c->name);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_nand_list_drivers)
{
	command_print(CMD, "Available NAND flash controller drivers:");
	return nand_driver_walk(&nand_list_walker, CMD);
}

static COMMAND_HELPER(create_nand_device, const char *bank_name,
	struct nand_flash_controller *controller)
{
	struct nand_device *c;
	struct target *target;
	int retval;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	target = get_target(CMD_ARGV[1]);
	if (!target) {
		LOG_ERROR("invalid target %s", CMD_ARGV[1]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (NULL != controller->commands) {
		retval = register_commands(CMD_CTX, NULL,
				controller->commands);
		if (ERROR_OK != retval)
			return retval;
	}
	c = malloc(sizeof(struct nand_device));
	if (c == NULL) {
		LOG_ERROR("End of memory");
		return ERROR_FAIL;
	}

	c->name = strdup(bank_name);
	c->target = target;
	c->controller = controller;
	c->controller_priv = NULL;
	c->manufacturer = NULL;
	c->device = NULL;
	c->bus_width = 0;
	c->address_cycles = 0;
	c->page_size = 0;
	c->use_raw = false;
	c->next = NULL;

	retval = CALL_COMMAND_HANDLER(controller->nand_device_command, c);
	if (ERROR_OK != retval) {
		LOG_ERROR("'%s' driver rejected nand flash. Usage: %s",
			controller->name,
			controller->usage);
		free(c);
		return retval;
	}

	if (controller->usage == NULL)
		LOG_DEBUG("'%s' driver usage field missing", controller->name);

	nand_device_add(c);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nand_device_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* save name and increment (for compatibility) with drivers */
	const char *bank_name = *CMD_ARGV++;
	CMD_ARGC--;

	const char *driver_name = CMD_ARGV[0];
	struct nand_flash_controller *controller;
	controller = nand_driver_find_by_name(CMD_ARGV[0]);
	if (NULL == controller) {
		LOG_ERROR("No valid NAND flash driver found (%s)", driver_name);
		return CALL_COMMAND_HANDLER(handle_nand_list_drivers);
	}
	return CALL_COMMAND_HANDLER(create_nand_device, bank_name, controller);
}

static const struct command_registration nand_config_command_handlers[] = {
	{
		.name = "device",
		.handler = &handle_nand_device_command,
		.mode = COMMAND_CONFIG,
		.help = "defines a new NAND bank",
		.usage = "bank_id driver target [driver_options ...]",
	},
	{
		.name = "drivers",
		.handler = &handle_nand_list_drivers,
		.mode = COMMAND_ANY,
		.help = "lists available NAND drivers",
		.usage = ""
	},
	{
		.name = "init",
		.mode = COMMAND_CONFIG,
		.handler = &handle_nand_init_command,
		.help = "initialize NAND devices",
		.usage = ""
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration nand_command_handlers[] = {
	{
		.name = "nand",
		.mode = COMMAND_ANY,
		.help = "NAND flash command group",
		.usage = "",
		.chain = nand_config_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int nand_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, nand_command_handlers);
}
