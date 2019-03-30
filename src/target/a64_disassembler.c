/***************************************************************************
 *   Copyright (C) 2019 by Mete Balci                                      *
 *   metebalci@gmail.com                                                   *
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

#include <helper/log.h>
#include "target.h"
#include "a64_disassembler.h"

#if HAVE_CAPSTONE

#include <capstone/capstone.h>

static void print_opcode(struct command_invocation *cmd, const cs_insn *insn)
{
	uint32_t opcode = 0;

	memcpy(&opcode, insn->bytes, insn->size);

	if (insn->size == 4) {

		uint16_t opcode_high = opcode >> 16;

		opcode = opcode & 0xffff;

		command_print(cmd,
				"0x%08" PRIx64"  %04x %04x\t%s\t%s",
				insn->address,
				opcode,
				opcode_high,
				insn->mnemonic,
				insn->op_str);

	} else {

		command_print(
				cmd,
				"0x%08" PRIx64"  %04x\t%s\t%s",
				insn->address,
				opcode,
				insn->mnemonic,
				insn->op_str);

	}
}

int a64_disassemble(struct command_invocation *cmd, struct target *target, target_addr_t address, size_t count)
{
	int ret;
	int csret;
	csh handle;

	csret = cs_open(CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN, &handle);

	if (csret != CS_ERR_OK) {

		LOG_ERROR("cs_open() failed: %s", cs_strerror(csret));
		return ERROR_FAIL;

	}

	csret = cs_option(handle, CS_OPT_SKIPDATA, CS_OPT_ON);

	if (csret != CS_ERR_OK) {

		LOG_ERROR("cs_option() failed: %s", cs_strerror(csret));
		cs_close(&handle);
		return ERROR_FAIL;

	}

	cs_insn *insn = cs_malloc(handle);

	if (csret != CS_ERR_OK) {

		LOG_ERROR("cs_malloc() failed: %s", cs_strerror(csret));
		cs_close(&handle);
		return ERROR_FAIL;

	}

	while (count > 0) {

		uint8_t buffer[4];

		ret = target_read_buffer(target, address, sizeof(buffer), buffer);

		if (ret != ERROR_OK) {
			cs_free(insn, 1);
			cs_close(&handle);
			return ret;
		}

		size_t size = sizeof(buffer);
		const uint8_t *tmp = buffer;

		ret = cs_disasm_iter(handle, &tmp, &size, &address, insn);

		if (!ret) {

			LOG_ERROR("cs_disasm_iter() failed: %s", cs_strerror(cs_errno(handle)));
			cs_free(insn, 1);
			cs_close(&handle);
			return ERROR_FAIL;

		}

		print_opcode(cmd, insn);
		count--;

	}

	cs_free(insn, 1);
	cs_close(&handle);

	return ERROR_OK;
}

#else

int a64_disassemble(struct command_invocation *cmd, struct target *target, target_addr_t address, size_t count)
{
	command_print(cmd, "capstone disassembly framework required");

	return ERROR_FAIL;
}

#endif
