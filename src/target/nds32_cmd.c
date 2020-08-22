/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#include <helper/command.h>
#include "nds32.h"
#include "nds32_aice.h"
#include "nds32_disassembler.h"

extern struct nds32_edm_operation nds32_edm_ops[NDS32_EDM_OPERATION_MAX_NUM];
extern uint32_t nds32_edm_ops_num;

static const char *const NDS_MEMORY_ACCESS_NAME[] = {
	"BUS",
	"CPU",
};

static const char *const NDS_MEMORY_SELECT_NAME[] = {
	"AUTO",
	"MEM",
	"ILM",
	"DLM",
};

COMMAND_HANDLER(handle_nds32_dssim_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->step_isr_enable = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->step_isr_enable = false;
	}

	command_print(CMD, "%s: $INT_MASK.DSSIM: %d", target_name(target),
			nds32->step_isr_enable);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_memory_access_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32_memory *memory = &(nds32->memory);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "bus") == 0)
			memory->access_channel = NDS_MEMORY_ACC_BUS;
		else if (strcmp(CMD_ARGV[0], "cpu") == 0)
			memory->access_channel = NDS_MEMORY_ACC_CPU;
		else /* default access channel is NDS_MEMORY_ACC_CPU */
			memory->access_channel = NDS_MEMORY_ACC_CPU;

		LOG_DEBUG("memory access channel is changed to %s",
				NDS_MEMORY_ACCESS_NAME[memory->access_channel]);

		aice_memory_access(aice, memory->access_channel);
	} else {
		command_print(CMD, "%s: memory access channel: %s",
				target_name(target),
				NDS_MEMORY_ACCESS_NAME[memory->access_channel]);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_memory_mode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {

		if (nds32->edm.access_control == false) {
			command_print(CMD, "%s does not support ACC_CTL. "
					"Set memory mode to MEMORY", target_name(target));
			nds32->memory.mode = NDS_MEMORY_SELECT_MEM;
		} else if (nds32->edm.direct_access_local_memory == false) {
			command_print(CMD, "%s does not support direct access "
					"local memory. Set memory mode to MEMORY",
					target_name(target));
			nds32->memory.mode = NDS_MEMORY_SELECT_MEM;

			/* set to ACC_CTL */
			aice_memory_mode(aice, nds32->memory.mode);
		} else {
			if (strcmp(CMD_ARGV[0], "auto") == 0) {
				nds32->memory.mode = NDS_MEMORY_SELECT_AUTO;
			} else if (strcmp(CMD_ARGV[0], "mem") == 0) {
				nds32->memory.mode = NDS_MEMORY_SELECT_MEM;
			} else if (strcmp(CMD_ARGV[0], "ilm") == 0) {
				if (nds32->memory.ilm_base == 0)
					command_print(CMD, "%s does not support ILM",
							target_name(target));
				else
					nds32->memory.mode = NDS_MEMORY_SELECT_ILM;
			} else if (strcmp(CMD_ARGV[0], "dlm") == 0) {
				if (nds32->memory.dlm_base == 0)
					command_print(CMD, "%s does not support DLM",
							target_name(target));
				else
					nds32->memory.mode = NDS_MEMORY_SELECT_DLM;
			}

			/* set to ACC_CTL */
			aice_memory_mode(aice, nds32->memory.mode);
		}
	}

	command_print(CMD, "%s: memory mode: %s",
			target_name(target),
			NDS_MEMORY_SELECT_NAME[nds32->memory.mode]);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_cache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32_cache *icache = &(nds32->memory.icache);
	struct nds32_cache *dcache = &(nds32->memory.dcache);
	int result;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {

		if (strcmp(CMD_ARGV[0], "invalidate") == 0) {
			if ((dcache->line_size != 0) && (dcache->enable == true)) {
				/* D$ write back */
				result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1D_WBALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Write back data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Write back data cache...done",
						target_name(target));

				/* D$ invalidate */
				result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1D_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate data cache...done",
						target_name(target));
			} else {
				if (dcache->line_size == 0)
					command_print(CMD, "%s: No data cache",
							target_name(target));
				else
					command_print(CMD, "%s: Data cache disabled",
							target_name(target));
			}

			if ((icache->line_size != 0) && (icache->enable == true)) {
				/* I$ invalidate */
				result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1I_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate instruction cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate instruction cache...done",
						target_name(target));
			} else {
				if (icache->line_size == 0)
					command_print(CMD, "%s: No instruction cache",
							target_name(target));
				else
					command_print(CMD, "%s: Instruction cache disabled",
							target_name(target));
			}
		} else
			command_print(CMD, "No valid parameter");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_icache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32_cache *icache = &(nds32->memory.icache);
	int result;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {

		if (icache->line_size == 0) {
			command_print(CMD, "%s: No instruction cache",
					target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0) {
			if (icache->enable == true) {
				/* I$ invalidate */
				result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1I_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate instruction cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate instruction cache...done",
						target_name(target));
			} else {
				command_print(CMD, "%s: Instruction cache disabled",
						target_name(target));
			}
		} else if (strcmp(CMD_ARGV[0], "enable") == 0) {
			uint32_t value;
			nds32_get_mapped_reg(nds32, IR8, &value);
			nds32_set_mapped_reg(nds32, IR8, value | 0x1);
		} else if (strcmp(CMD_ARGV[0], "disable") == 0) {
			uint32_t value;
			nds32_get_mapped_reg(nds32, IR8, &value);
			nds32_set_mapped_reg(nds32, IR8, value & ~0x1);
		} else if (strcmp(CMD_ARGV[0], "dump") == 0) {
			/* TODO: dump cache content */
		} else {
			command_print(CMD, "%s: No valid parameter", target_name(target));
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_dcache_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);
	struct aice_port_s *aice = target_to_aice(target);
	struct nds32_cache *dcache = &(nds32->memory.dcache);
	int result;

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {

		if (dcache->line_size == 0) {
			command_print(CMD, "%s: No data cache", target_name(target));
			return ERROR_OK;
		}

		if (strcmp(CMD_ARGV[0], "invalidate") == 0) {
			if (dcache->enable == true) {
				/* D$ write back */
				result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1D_WBALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Write back data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Write back data cache...done",
						target_name(target));

				/* D$ invalidate */
				result = aice_cache_ctl(aice, AICE_CACHE_CTL_L1D_INVALALL, 0);
				if (result != ERROR_OK) {
					command_print(CMD, "%s: Invalidate data cache...failed",
							target_name(target));
					return result;
				}

				command_print(CMD, "%s: Invalidate data cache...done",
						target_name(target));
			} else {
				command_print(CMD, "%s: Data cache disabled",
						target_name(target));
			}
		} else if (strcmp(CMD_ARGV[0], "enable") == 0) {
			uint32_t value;
			nds32_get_mapped_reg(nds32, IR8, &value);
			nds32_set_mapped_reg(nds32, IR8, value | 0x2);
		} else if (strcmp(CMD_ARGV[0], "disable") == 0) {
			uint32_t value;
			nds32_get_mapped_reg(nds32, IR8, &value);
			nds32_set_mapped_reg(nds32, IR8, value & ~0x2);
		} else if (strcmp(CMD_ARGV[0], "dump") == 0) {
			/* TODO: dump cache content */
		} else {
			command_print(CMD, "%s: No valid parameter", target_name(target));
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_auto_break_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->auto_convert_hw_bp = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->auto_convert_hw_bp = false;
	}

	if (nds32->auto_convert_hw_bp)
		command_print(CMD, "%s: convert sw break to hw break on ROM: on",
				target_name(target));
	else
		command_print(CMD, "%s: convert sw break to hw break on ROM: off",
				target_name(target));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_virtual_hosting_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->virtual_hosting = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->virtual_hosting = false;
	}

	if (nds32->virtual_hosting)
		command_print(CMD, "%s: virtual hosting: on", target_name(target));
	else
		command_print(CMD, "%s: virtual hosting: off", target_name(target));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_global_stop_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->global_stop = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->global_stop = false;
	}

	if (nds32->global_stop)
		LOG_INFO("%s: global stop: on", target_name(target));
	else
		LOG_INFO("%s: global stop: off", target_name(target));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_soft_reset_halt_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->soft_reset_halt = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->soft_reset_halt = false;
	}

	if (nds32->soft_reset_halt)
		LOG_INFO("%s: soft-reset-halt: on", target_name(target));
	else
		LOG_INFO("%s: soft-reset-halt: off", target_name(target));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_boot_time_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], nds32->boot_time);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_login_edm_passcode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	nds32->edm_passcode = strdup(CMD_ARGV[0]);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_login_edm_operation_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 1) {

		uint32_t misc_reg_no;
		uint32_t data;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], misc_reg_no);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], data);

		if (nds32_edm_ops_num >= NDS32_EDM_OPERATION_MAX_NUM)
			return ERROR_FAIL;

		/* Just save the operation. Execute it in nds32_login() */
		nds32_edm_ops[nds32_edm_ops_num].reg_no = misc_reg_no;
		nds32_edm_ops[nds32_edm_ops_num].value = data;
		nds32_edm_ops_num++;
	} else
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_reset_halt_as_init_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->reset_halt_as_examine = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->reset_halt_as_examine = false;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_keep_target_edm_ctl_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->keep_target_edm_ctl = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->keep_target_edm_ctl = false;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_decode_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 1) {

		uint32_t addr;
		uint32_t insn_count;
		uint32_t opcode;
		uint32_t read_addr;
		uint32_t i;
		struct nds32_instruction instruction;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], insn_count);

		read_addr = addr;
		i = 0;
		while (i < insn_count) {
			if (ERROR_OK != nds32_read_opcode(nds32, read_addr, &opcode))
				return ERROR_FAIL;
			if (ERROR_OK != nds32_evaluate_opcode(nds32, opcode,
						read_addr, &instruction))
				return ERROR_FAIL;

			command_print(CMD, "%s", instruction.text);

			read_addr += instruction.instruction_size;
			i++;
		}
	} else if (CMD_ARGC == 1) {

		uint32_t addr;
		uint32_t opcode;
		struct nds32_instruction instruction;

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);

		if (ERROR_OK != nds32_read_opcode(nds32, addr, &opcode))
			return ERROR_FAIL;
		if (ERROR_OK != nds32_evaluate_opcode(nds32, opcode, addr, &instruction))
			return ERROR_FAIL;

		command_print(CMD, "%s", instruction.text);
	} else
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_word_access_mem_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "on") == 0)
			nds32->word_access_mem = true;
		if (strcmp(CMD_ARGV[0], "off") == 0)
			nds32->word_access_mem = false;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_query_target_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	command_print(CMD, "OCD");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_query_endian_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	uint32_t value_psw;
	nds32_get_mapped_reg(nds32, IR0, &value_psw);

	if (value_psw & 0x20)
		command_print(CMD, "%s: BE", target_name(target));
	else
		command_print(CMD, "%s: LE", target_name(target));

	return ERROR_OK;
}

COMMAND_HANDLER(handle_nds32_query_cpuid_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct nds32 *nds32 = target_to_nds32(target);

	if (!is_nds32(nds32)) {
		command_print(CMD, "current target isn't an Andes core");
		return ERROR_FAIL;
	}

	command_print(CMD, "CPUID: %s", target_name(target));

	return ERROR_OK;
}

static int jim_nds32_bulk_write(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 3) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <address> <count> <data>", cmd_name);
		return JIM_ERR;
	}

	int e;
	jim_wide address;
	e = Jim_GetOpt_Wide(&goi, &address);
	if (e != JIM_OK)
		return e;

	jim_wide count;
	e = Jim_GetOpt_Wide(&goi, &count);
	if (e != JIM_OK)
		return e;

	uint32_t *data = malloc(count * sizeof(uint32_t));
	if (data == NULL)
		return JIM_ERR;

	jim_wide i;
	for (i = 0; i < count; i++) {
		jim_wide tmp;
		e = Jim_GetOpt_Wide(&goi, &tmp);
		if (e != JIM_OK) {
			free(data);
			return e;
		}
		data[i] = (uint32_t)tmp;
	}

	/* all args must be consumed */
	if (goi.argc != 0) {
		free(data);
		return JIM_ERR;
	}

	struct target *target = Jim_CmdPrivData(goi.interp);
	int result;

	result = target_write_buffer(target, address, count * 4, (const uint8_t *)data);

	free(data);

	return result;
}

static int jim_nds32_multi_write(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 3) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s # of pairs [<address> <data>]+", cmd_name);
		return JIM_ERR;
	}

	int e;
	jim_wide num_of_pairs;
	e = Jim_GetOpt_Wide(&goi, &num_of_pairs);
	if (e != JIM_OK)
		return e;

	struct target *target = Jim_CmdPrivData(goi.interp);
	struct aice_port_s *aice = target_to_aice(target);
	int result;
	uint32_t address;
	uint32_t data;
	jim_wide i;

	aice_set_command_mode(aice, AICE_COMMAND_MODE_PACK);
	for (i = 0; i < num_of_pairs; i++) {
		jim_wide tmp;
		e = Jim_GetOpt_Wide(&goi, &tmp);
		if (e != JIM_OK)
			break;
		address = (uint32_t)tmp;

		e = Jim_GetOpt_Wide(&goi, &tmp);
		if (e != JIM_OK)
			break;
		data = (uint32_t)tmp;

		result = target_write_buffer(target, address, 4, (const uint8_t *)&data);
		if (result != ERROR_OK)
			break;
	}
	aice_set_command_mode(aice, AICE_COMMAND_MODE_NORMAL);

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	return ERROR_OK;
}

static int jim_nds32_bulk_read(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 2) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <address> <count>", cmd_name);
		return JIM_ERR;
	}

	int e;
	jim_wide address;
	e = Jim_GetOpt_Wide(&goi, &address);
	if (e != JIM_OK)
		return e;

	jim_wide count;
	e = Jim_GetOpt_Wide(&goi, &count);
	if (e != JIM_OK)
		return e;

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	struct target *target = Jim_CmdPrivData(goi.interp);
	uint32_t *data = malloc(count * sizeof(uint32_t));
	int result;
	result = target_read_buffer(target, address, count * 4, (uint8_t *)data);
	char data_str[12];

	jim_wide i;
	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
	for (i = 0; i < count; i++) {
		sprintf(data_str, "0x%08" PRIx32 " ", data[i]);
		Jim_AppendStrings(interp, Jim_GetResult(interp), data_str, NULL);
	}

	free(data);

	return result;
}

static int jim_nds32_read_edm_sr(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 1) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <edm_sr_name>", cmd_name);
		return JIM_ERR;
	}

	int e;
	const char *edm_sr_name;
	int edm_sr_name_len;
	e = Jim_GetOpt_String(&goi, &edm_sr_name, &edm_sr_name_len);
	if (e != JIM_OK)
		return e;

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	uint32_t edm_sr_number;
	uint32_t edm_sr_value;
	if (strncmp(edm_sr_name, "edm_dtr", edm_sr_name_len) == 0)
		edm_sr_number = NDS_EDM_SR_EDM_DTR;
	else if (strncmp(edm_sr_name, "edmsw", edm_sr_name_len) == 0)
		edm_sr_number = NDS_EDM_SR_EDMSW;
	else
		return ERROR_FAIL;

	struct target *target = Jim_CmdPrivData(goi.interp);
	struct aice_port_s *aice = target_to_aice(target);
	char data_str[11];

	aice_read_debug_reg(aice, edm_sr_number, &edm_sr_value);

	sprintf(data_str, "0x%08" PRIx32, edm_sr_value);
	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
	Jim_AppendStrings(interp, Jim_GetResult(interp), data_str, NULL);

	return ERROR_OK;
}

static int jim_nds32_write_edm_sr(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);

	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);

	if (goi.argc < 2) {
		Jim_SetResultFormatted(goi.interp,
				"usage: %s <edm_sr_name> <value>", cmd_name);
		return JIM_ERR;
	}

	int e;
	const char *edm_sr_name;
	int edm_sr_name_len;
	e = Jim_GetOpt_String(&goi, &edm_sr_name, &edm_sr_name_len);
	if (e != JIM_OK)
		return e;

	jim_wide value;
	e = Jim_GetOpt_Wide(&goi, &value);
	if (e != JIM_OK)
		return e;

	/* all args must be consumed */
	if (goi.argc != 0)
		return JIM_ERR;

	uint32_t edm_sr_number;
	if (strncmp(edm_sr_name, "edm_dtr", edm_sr_name_len) == 0)
		edm_sr_number = NDS_EDM_SR_EDM_DTR;
	else
		return ERROR_FAIL;

	struct target *target = Jim_CmdPrivData(goi.interp);
	struct aice_port_s *aice = target_to_aice(target);

	aice_write_debug_reg(aice, edm_sr_number, value);

	return ERROR_OK;
}

static const struct command_registration nds32_query_command_handlers[] = {
	{
		.name = "target",
		.handler = handle_nds32_query_target_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "reply 'OCD' for gdb to identify server-side is OpenOCD",
	},
	{
		.name = "endian",
		.handler = handle_nds32_query_endian_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query target endian",
	},
	{
		.name = "cpuid",
		.handler = handle_nds32_query_cpuid_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "query CPU ID",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration nds32_exec_command_handlers[] = {
	{
		.name = "dssim",
		.handler = handle_nds32_dssim_command,
		.mode = COMMAND_EXEC,
		.usage = "['on'|'off']",
		.help = "display/change $INT_MASK.DSSIM status",
	},
	{
		.name = "mem_access",
		.handler = handle_nds32_memory_access_command,
		.mode = COMMAND_EXEC,
		.usage = "['bus'|'cpu']",
		.help = "display/change memory access channel",
	},
	{
		.name = "mem_mode",
		.handler = handle_nds32_memory_mode_command,
		.mode = COMMAND_EXEC,
		.usage = "['auto'|'mem'|'ilm'|'dlm']",
		.help = "display/change memory mode",
	},
	{
		.name = "cache",
		.handler = handle_nds32_cache_command,
		.mode = COMMAND_EXEC,
		.usage = "['invalidate']",
		.help = "cache control",
	},
	{
		.name = "icache",
		.handler = handle_nds32_icache_command,
		.mode = COMMAND_EXEC,
		.usage = "['invalidate'|'enable'|'disable'|'dump']",
		.help = "icache control",
	},
	{
		.name = "dcache",
		.handler = handle_nds32_dcache_command,
		.mode = COMMAND_EXEC,
		.usage = "['invalidate'|'enable'|'disable'|'dump']",
		.help = "dcache control",
	},
	{
		.name = "auto_break",
		.handler = handle_nds32_auto_break_command,
		.mode = COMMAND_EXEC,
		.usage = "['on'|'off']",
		.help = "convert software breakpoints to hardware breakpoints if needed",
	},
	{
		.name = "virtual_hosting",
		.handler = handle_nds32_virtual_hosting_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "turn on/off virtual hosting",
	},
	{
		.name = "global_stop",
		.handler = handle_nds32_global_stop_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "turn on/off global stop. After turning on, every load/store "
			 "instructions will be stopped to check memory access.",
	},
	{
		.name = "soft_reset_halt",
		.handler = handle_nds32_soft_reset_halt_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "as issuing rest-halt, to use soft-reset-halt or not."
			 "the feature is for backward-compatible.",
	},
	{
		.name = "boot_time",
		.handler = handle_nds32_boot_time_command,
		.mode = COMMAND_CONFIG,
		.usage = "milliseconds",
		.help = "set the period to wait after srst.",
	},
	{
		.name = "login_edm_passcode",
		.handler = handle_nds32_login_edm_passcode_command,
		.mode = COMMAND_CONFIG,
		.usage = "passcode",
		.help = "set EDM passcode for secure MCU debugging.",
	},
	{
		.name = "login_edm_operation",
		.handler = handle_nds32_login_edm_operation_command,
		.mode = COMMAND_CONFIG,
		.usage = "misc_reg_no value",
		.help = "add EDM operations for secure MCU debugging.",
	},
	{
		.name = "reset_halt_as_init",
		.handler = handle_nds32_reset_halt_as_init_command,
		.mode = COMMAND_CONFIG,
		.usage = "['on'|'off']",
		.help = "reset halt as openocd init.",
	},
	{
		.name = "keep_target_edm_ctl",
		.handler = handle_nds32_keep_target_edm_ctl_command,
		.mode = COMMAND_CONFIG,
		.usage = "['on'|'off']",
		.help = "Backup/Restore target EDM_CTL register.",
	},
	{
		.name = "decode",
		.handler = handle_nds32_decode_command,
		.mode = COMMAND_EXEC,
		.usage = "address icount",
		.help = "decode instruction.",
	},
	{
		.name = "word_access_mem",
		.handler = handle_nds32_word_access_mem_command,
		.mode = COMMAND_ANY,
		.usage = "['on'|'off']",
		.help = "Always use word-aligned address to access memory.",
	},
	{
		.name = "bulk_write",
		.jim_handler = jim_nds32_bulk_write,
		.mode = COMMAND_EXEC,
		.help = "Write multiple 32-bit words to target memory",
		.usage = "address count data",
	},
	{
		.name = "multi_write",
		.jim_handler = jim_nds32_multi_write,
		.mode = COMMAND_EXEC,
		.help = "Write multiple addresses/words to target memory",
		.usage = "num_of_pairs [address data]+",
	},
	{
		.name = "bulk_read",
		.jim_handler = jim_nds32_bulk_read,
		.mode = COMMAND_EXEC,
		.help = "Read multiple 32-bit words from target memory",
		.usage = "address count",
	},
	{
		.name = "read_edmsr",
		.jim_handler = jim_nds32_read_edm_sr,
		.mode = COMMAND_EXEC,
		.help = "Read EDM system register",
		.usage = "['edmsw'|'edm_dtr']",
	},
	{
		.name = "write_edmsr",
		.jim_handler = jim_nds32_write_edm_sr,
		.mode = COMMAND_EXEC,
		.help = "Write EDM system register",
		.usage = "['edm_dtr'] value",
	},
	{
		.name = "query",
		.mode = COMMAND_EXEC,
		.help = "Andes query command group",
		.usage = "",
		.chain = nds32_query_command_handlers,
	},

	COMMAND_REGISTRATION_DONE
};

const struct command_registration nds32_command_handlers[] = {
	{
		.name = "nds",
		.mode = COMMAND_ANY,
		.help = "Andes command group",
		.usage = "",
		.chain = nds32_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
