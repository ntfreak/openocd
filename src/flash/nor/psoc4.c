/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2014 by Tomas Vanek (PSoC 4 support derived from STM32) *
 *   vanekt@fbl.cz                                                         *
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

#include "imp.h"
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* device documents:

 PSoC(R) 4: PSoC 4200 Family Datasheet
	Document Number: 001-87197 Rev. *B  Revised August 29, 2013

 PSoC 4100/4200 Family PSoC(R) 4 Architecture TRM
	Document No. 001-85634 Rev. *E June 28, 2016

 PSoC(R) 4 Registers TRM Spec.
	Document No. 001-85847 Rev. *A June 25, 2013

 PSoC 4000 Family PSoC(R) 4 Technical Reference Manual
	Document No. 001-89309 Rev. *B May 9, 2016

 PSoC 41XX_BLE/42XX_BLE Family PSoC 4 BLE Architecture TRM
	Document No. 001-92738 Rev. *C February 12, 2016

 PSoC 4200L Family PSoC 4 Architecture TRM
	Document No. 001-97952 Rev. *A December 15, 2015

 PSoC 4200L Family PSoC 4 Registers TRM
	Document No. 001-98126 Rev. *A December 16, 2015

 PSoC 4100M/4200M Family PSoC 4 Architecture TRM
	Document No. 001-95223 Rev. *B July 29, 2015

 PSoC 4100S Family PSoC 4 Architecture TRM
	Document No. 002-10621 Rev. *A July 29, 2016

 PSoC 4100S Family PSoC 4 Registers TRM
	Document No. 002-10523 Rev. *A July 20, 2016

 PSoC Analog Coprocessor Architecture TRM
	Document No. 002-10404 Rev. ** December 18, 2015

 CY8C4Axx PSoC Analog Coprocessor Registers TRM
	Document No. 002-10405 Rev. ** December 18, 2015

 CY8C41xx, CY8C42xx Programming Specifications
	Document No. 001-81799 Rev. *C March 4, 2014

 CYBL10x6x, CY8C4127_BL, CY8C4247_BL Programming Specifications
	Document No. 001-91508 Rev. *B September 22, 2014

 http://dmitry.gr/index.php?r=05.Projects&proj=24.%20PSoC4%20confidential
*/

/* register locations */
#define PSOC4_SFLASH_MACRO0		0x0FFFF000

#define PSOC4_CPUSS_SYSREQ_LEGACY	0x40000004
#define PSOC4_CPUSS_SYSARG_LEGACY	0x40000008
#define PSOC4_SPCIF_GEOMETRY_LEGACY	0x400E0000

#define PSOC4_CPUSS_SYSREQ_NEW		0x40100004
#define PSOC4_CPUSS_SYSARG_NEW		0x40100008
#define PSOC4_SPCIF_GEOMETRY_NEW	0x40110000

#define PSOC4_TEST_MODE			0x40030014

#define PSOC4_ROMTABLE_PID0		0xF0000FE0


/* constants */
#define PSOC4_SFLASH_MACRO_SIZE		0x800
#define PSOC4_ROWS_PER_MACRO		512

#define PSOC4_SROM_KEY1			0xb6
#define PSOC4_SROM_KEY2			0xd3
#define PSOC4_SROM_SYSREQ_BIT		(1<<31)
#define PSOC4_SROM_HMASTER_BIT		(1<<30)
#define PSOC4_SROM_PRIVILEGED_BIT	(1<<28)
#define PSOC4_SROM_STATUS_SUCCEEDED	0xa0000000
#define PSOC4_SROM_STATUS_FAILED	0xf0000000
#define PSOC4_SROM_STATUS_MASK		0xf0000000

/* not documented in any TRM */
#define PSOC4_SROM_ERR_IMO_NOT_IMPLEM	0xf0000013

#define PSOC4_CMD_GET_SILICON_ID	0
#define PSOC4_CMD_LOAD_LATCH		4
#define PSOC4_CMD_WRITE_ROW		5
#define PSOC4_CMD_PROGRAM_ROW		6
#define PSOC4_CMD_ERASE_ALL		0xa
#define PSOC4_CMD_CHECKSUM		0xb
#define PSOC4_CMD_WRITE_PROTECTION	0xd
#define PSOC4_CMD_SET_IMO48		0x15
#define PSOC4_CMD_WRITE_SFLASH_ROW	0x18

#define PSOC4_CHIP_PROT_VIRGIN		0x0
#define PSOC4_CHIP_PROT_OPEN		0x1
#define PSOC4_CHIP_PROT_PROTECTED	0x2
#define PSOC4_CHIP_PROT_KILL		0x4

#define PSOC4_ROMTABLE_DESIGNER_CHECK	0xb4

#define PSOC4_FAMILY_FLAG_LEGACY	1

struct psoc4_chip_family {
	uint16_t id;
	const char *name;
	uint32_t flags;
};

static const struct psoc4_chip_family psoc4_families[] = {
	{ 0x93, "PSoC4100/4200",           .flags = PSOC4_FAMILY_FLAG_LEGACY },
	{ 0x9A, "PSoC4000",                .flags = 0 },
	{ 0x9E, "PSoC/PRoC BLE (119E)",    .flags = 0 },
	{ 0xA0, "PSoC4200L",               .flags = 0 },
	{ 0xA1, "PSoC4100M/4200M",         .flags = 0 },
	{ 0xA3, "PSoC/PRoC BLE (11A3)",    .flags = 0 },
	{ 0xA9, "PSoC4000S",               .flags = 0 },
	{ 0xAA, "PSoC/PRoC BLE (11AA)",    .flags = 0 },
	{ 0xAB, "PSoC4100S",               .flags = 0 },
	{ 0xAC, "PSoC Analog Coprocessor", .flags = 0 },
	{ 0,    "Unknown",                 .flags = 0 }
};


struct psoc4_flash_bank {
	uint32_t row_size;
	uint32_t user_bank_size;
	unsigned int num_macros;
	bool probed;
	uint8_t cmd_program_row;
	uint16_t family_id;
	bool legacy_family;
	uint32_t cpuss_sysreq_addr;
	uint32_t cpuss_sysarg_addr;
	uint32_t spcif_geometry_addr;
};


static const struct psoc4_chip_family *psoc4_family_by_id(uint16_t family_id)
{
	const struct psoc4_chip_family *p = psoc4_families;
	while (p->id && p->id != family_id)
		p++;

	return p;
}

static const char *psoc4_decode_chip_protection(uint8_t protection)
{
	switch (protection) {
	case PSOC4_CHIP_PROT_VIRGIN:
		return "protection VIRGIN";
	case PSOC4_CHIP_PROT_OPEN:
		return "protection open";
	case PSOC4_CHIP_PROT_PROTECTED:
		return "PROTECTED";
	case PSOC4_CHIP_PROT_KILL:
		return "protection KILL";
	default:
		LOG_WARNING("Unknown protection state 0x%02" PRIx8 "", protection);
		return "";
	}
}


/* flash bank <name> psoc <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(psoc4_flash_bank_command)
{
	struct psoc4_flash_bank *psoc4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	psoc4_info = calloc(1, sizeof(struct psoc4_flash_bank));

	bank->driver_priv = psoc4_info;
	bank->default_padded_value = bank->erased_value = 0x00;
	psoc4_info->user_bank_size = bank->size;
	psoc4_info->cmd_program_row = PSOC4_CMD_WRITE_ROW;

	return ERROR_OK;
}


/* PSoC 4 system ROM request
 *  Setting SROM_SYSREQ_BIT in CPUSS_SYSREQ register runs NMI service
 *  in sysrem ROM. Algorithm just waits for NMI to finish.
 *  When sysreq_params_size == 0 only one parameter is passed in CPUSS_SYSARG register.
 *  Otherwise address of memory parameter block is set in CPUSS_SYSARG
 *  and the first parameter is written to the first word of parameter block
 */
static int psoc4_sysreq(struct flash_bank *bank, uint8_t cmd,
		uint16_t cmd_param,
		uint32_t *sysreq_params, uint32_t sysreq_params_size,
		uint32_t *sysarg_out)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	struct working_area *sysreq_wait_algorithm;
	struct working_area *sysreq_mem;

	struct reg_param reg_params[1];
	struct armv7m_algorithm armv7m_info;

	int retval = ERROR_OK;

	uint32_t param1 = PSOC4_SROM_KEY1
			 | ((PSOC4_SROM_KEY2 + cmd) << 8)
			 | (cmd_param << 16);

	static uint8_t psoc4_sysreq_wait_code[] = {
		/* system request NMI is served immediately after algo run
       now we are done: break */
		0x00, 0xbe,		/* bkpt 0 */
	};

	const int code_words = (sizeof(psoc4_sysreq_wait_code) + 3) / 4;
					/* stack must be aligned */
	const int stack_size = 256;
	/* tested stack sizes on PSoC4200:
		ERASE_ALL	144
		PROGRAM_ROW	112
		other sysreq	 68
	*/

	/* allocate area for sysreq wait code and stack */
	if (target_alloc_working_area(target, code_words * 4 + stack_size,
			&sysreq_wait_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for sysreq code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Write the code */
	retval = target_write_buffer(target,
			sysreq_wait_algorithm->address,
			sizeof(psoc4_sysreq_wait_code),
			psoc4_sysreq_wait_code);
	if (retval != ERROR_OK) {
		/* we already allocated the writing code, but failed to get a
		 * buffer, free the algorithm */
		goto cleanup_algo;
	}

	if (sysreq_params_size) {
		LOG_DEBUG("SYSREQ %02" PRIx8 " %04" PRIx16 " %08" PRIx32 " size %" PRIu32,
			cmd, cmd_param, param1, sysreq_params_size);
		/* Allocate memory for sysreq_params */
		retval = target_alloc_working_area(target, sysreq_params_size, &sysreq_mem);
		if (retval != ERROR_OK) {
			LOG_WARNING("no working area for sysreq parameters");

			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto cleanup_algo;
		}

		/* Write sysreq_params */
		target_buffer_set_u32(target, (uint8_t *)sysreq_params, param1);
		retval = target_write_buffer(target, sysreq_mem->address,
				sysreq_params_size, (uint8_t *)sysreq_params);
		if (retval != ERROR_OK)
			goto cleanup_mem;

		/* Set address of sysreq parameters block */
		retval = target_write_u32(target, psoc4_info->cpuss_sysarg_addr, sysreq_mem->address);
		if (retval != ERROR_OK)
			goto cleanup_mem;

	} else {
		/* Sysreq without memory block of parameters */
		LOG_DEBUG("SYSREQ %02" PRIx8 " %04" PRIx16 " %08" PRIx32,
			cmd, cmd_param, param1);
		/* Set register parameter */
		retval = target_write_u32(target, psoc4_info->cpuss_sysarg_addr, param1);
		if (retval != ERROR_OK)
			goto cleanup_mem;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* sysreq stack */
	init_reg_param(&reg_params[0], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32,
		    sysreq_wait_algorithm->address + sysreq_wait_algorithm->size);

	struct armv7m_common *armv7m = target_to_armv7m(target);
	if (armv7m == NULL) {
		/* something is very wrong if armv7m is NULL */
		LOG_ERROR("unable to get armv7m target");
		retval = ERROR_FAIL;
		goto cleanup;
	}

	/* Set SROM request */
	retval = target_write_u32(target, psoc4_info->cpuss_sysreq_addr,
				  PSOC4_SROM_SYSREQ_BIT | PSOC4_SROM_HMASTER_BIT | cmd);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Execute wait code */
	retval = target_run_algorithm(target, 0, NULL,
				sizeof(reg_params) / sizeof(*reg_params), reg_params,
				sysreq_wait_algorithm->address, 0, 1000, &armv7m_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("sysreq wait code execution failed");
		goto cleanup;
	}

	uint32_t sysarg_out_tmp;
	retval = target_read_u32(target, psoc4_info->cpuss_sysarg_addr, &sysarg_out_tmp);
	if (retval != ERROR_OK)
		goto cleanup;

	if (sysarg_out) {
		*sysarg_out = sysarg_out_tmp;
		/* If result is an error, do not show now, let caller to decide */
	} else if ((sysarg_out_tmp & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		LOG_ERROR("sysreq error 0x%" PRIx32, sysarg_out_tmp);
		retval = ERROR_FAIL;
	}
cleanup:
	destroy_reg_param(&reg_params[0]);

cleanup_mem:
	if (sysreq_params_size)
		target_free_working_area(target, sysreq_mem);

cleanup_algo:
	target_free_working_area(target, sysreq_wait_algorithm);

	return retval;
}


/* helper routine to get silicon ID from a PSoC 4 chip */
static int psoc4_get_silicon_id(struct flash_bank *bank, uint32_t *silicon_id, uint16_t *family_id, uint8_t *protection)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	uint32_t part0, part1;

	int retval = psoc4_sysreq(bank, PSOC4_CMD_GET_SILICON_ID, 0, NULL, 0, &part0);
	if (retval != ERROR_OK)
		return retval;

	if ((part0 & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		LOG_ERROR("sysreq error 0x%" PRIx32, part0);
		return ERROR_FAIL;
	}

	retval = target_read_u32(target, psoc4_info->cpuss_sysreq_addr, &part1);
	if (retval != ERROR_OK)
		return retval;

	/* build ID as Cypress sw does:
	 * bit 31..16 silicon ID
	 * bit 15..8  revision ID (so far 0x11 for all devices)
	 * bit 7..0   family ID (lowest 8 bits)
	 */
	if (silicon_id)
			*silicon_id = ((part0 & 0x0000ffff) << 16)
				    | ((part0 & 0x00ff0000) >> 8)
				    | (part1 & 0x000000ff);

	if (family_id)
			*family_id = part1 & 0x0fff;

	if (protection)
			*protection = (part1 >> 12) & 0x0f;

	return ERROR_OK;
}


static int psoc4_get_family(struct target *target, uint16_t *family_id)
{
	int retval, i;
	uint32_t pidbf[3];
	uint8_t pid[3];

	retval = target_read_memory(target, PSOC4_ROMTABLE_PID0, 4, 3, (uint8_t *)pidbf);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < 3; i++) {
		uint32_t tmp = target_buffer_get_u32(target, (uint8_t *)(pidbf + i));
		if (tmp & 0xffffff00) {
			LOG_ERROR("Unexpected data in ROMTABLE");
			return ERROR_FAIL;
		}
		pid[i] = tmp & 0xff;
	}

	uint16_t family = pid[0] | ((pid[1] & 0xf) << 8);
	uint32_t designer = ((pid[1] & 0xf0) >> 4) | ((pid[2] & 0xf) << 4);

	if (designer != PSOC4_ROMTABLE_DESIGNER_CHECK) {
		LOG_ERROR("ROMTABLE designer is not Cypress");
		return ERROR_FAIL;
	}

	*family_id = family;
	return ERROR_OK;
}


static int psoc4_flash_prepare(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint16_t family_id;
	int retval;

	/* get family ID from SROM call */
	retval = psoc4_get_silicon_id(bank, NULL, &family_id, NULL);
	if (retval != ERROR_OK)
		return retval;

	/* and check with family ID from ROMTABLE */
	if (family_id != psoc4_info->family_id) {
		LOG_ERROR("Family mismatch");
		return ERROR_FAIL;
	}

	if (!psoc4_info->legacy_family) {
		uint32_t sysreq_status;
		retval = psoc4_sysreq(bank, PSOC4_CMD_SET_IMO48, 0, NULL, 0, &sysreq_status);
		if (retval != ERROR_OK)
			return retval;

		if ((sysreq_status & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
			/* This undocumented error code is returned probably when
			 * PSOC4_CMD_SET_IMO48 command is not implemented.
			 * Can be safely ignored, programming works.
			 */
			if (sysreq_status == PSOC4_SROM_ERR_IMO_NOT_IMPLEM)
				LOG_INFO("PSOC4_CMD_SET_IMO48 is not implemented on this device.");
			else {
				LOG_ERROR("sysreq error 0x%" PRIx32, sysreq_status);
				return ERROR_FAIL;
			}
		}
	}

	return ERROR_OK;
}


static int psoc4_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	uint32_t prot_addr = PSOC4_SFLASH_MACRO0;
	int retval;
	uint8_t bf[PSOC4_ROWS_PER_MACRO/8];
	unsigned int s = 0;

	for (unsigned int m = 0; m < psoc4_info->num_macros; m++, prot_addr += PSOC4_SFLASH_MACRO_SIZE) {
		retval = target_read_memory(target, prot_addr, 4, PSOC4_ROWS_PER_MACRO/32, bf);
		if (retval != ERROR_OK)
			return retval;

		for (unsigned int i = 0; i < PSOC4_ROWS_PER_MACRO && s < bank->num_sectors; i++, s++)
			bank->sectors[s].is_protected = bf[i/8] & (1 << (i%8)) ? 1 : 0;
	}

	return ERROR_OK;
}


static int psoc4_mass_erase(struct flash_bank *bank)
{
	int retval = psoc4_flash_prepare(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Call "Erase All" system ROM API */
	uint32_t param = 0;
	retval = psoc4_sysreq(bank, PSOC4_CMD_ERASE_ALL,
			0,
			&param, sizeof(param), NULL);

	if (retval == ERROR_OK)
		/* set all sectors as erased */
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

	return retval;
}


static int psoc4_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	if (psoc4_info->cmd_program_row == PSOC4_CMD_WRITE_ROW) {
		LOG_INFO("Autoerase enabled, erase command ignored");
		return ERROR_OK;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return psoc4_mass_erase(bank);

	LOG_ERROR("Only mass erase available! Consider using 'psoc4 flash_autoerase 0 on'");

	return ERROR_FAIL;
}


static int psoc4_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	if (!psoc4_info->probed)
		return ERROR_FAIL;

	int retval = psoc4_flash_prepare(bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t *sysrq_buffer = NULL;
	const int param_sz = 8;
	int chip_prot = PSOC4_CHIP_PROT_OPEN;
	unsigned int i;
	unsigned int num_bits = bank->num_sectors;

	if (num_bits > PSOC4_ROWS_PER_MACRO)
		num_bits = PSOC4_ROWS_PER_MACRO;

	int prot_sz = num_bits / 8;

	sysrq_buffer = malloc(param_sz + prot_sz);
	if (sysrq_buffer == NULL) {
		LOG_ERROR("no memory for row buffer");
		return ERROR_FAIL;
	}

	for (i = first; i <= last && i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = set;

	for (unsigned int m = 0, sect = 0; m < psoc4_info->num_macros; m++) {
		uint8_t *p = (uint8_t *)(sysrq_buffer + 2);
		memset(p, 0, prot_sz);
		for (i = 0; i < num_bits && sect < bank->num_sectors; i++, sect++) {
			if (bank->sectors[sect].is_protected)
				p[i/8] |= 1 << (i%8);
		}

		/* Call "Load Latch" system ROM API */
		target_buffer_set_u32(target, (uint8_t *)(sysrq_buffer + 1),
					prot_sz - 1);
		retval = psoc4_sysreq(bank, PSOC4_CMD_LOAD_LATCH,
			0	/* Byte number in latch from what to write */
			  | (m << 8), /* flash macro index */
			sysrq_buffer, param_sz + prot_sz,
			NULL);
		if (retval != ERROR_OK)
			break;

		/* Call "Write Protection" system ROM API */
		retval = psoc4_sysreq(bank, PSOC4_CMD_WRITE_PROTECTION,
			chip_prot | (m << 8), NULL, 0, NULL);
		if (retval != ERROR_OK)
			break;
	}

	free(sysrq_buffer);

	psoc4_protect_check(bank);
	return retval;
}


COMMAND_HANDLER(psoc4_handle_flash_autoerase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	bool enable = psoc4_info->cmd_program_row == PSOC4_CMD_WRITE_ROW;

	if (CMD_ARGC >= 2)
		COMMAND_PARSE_ON_OFF(CMD_ARGV[1], enable);

	if (enable) {
		psoc4_info->cmd_program_row = PSOC4_CMD_WRITE_ROW;
		LOG_INFO("Flash auto-erase enabled, non mass erase commands will be ignored.");
	} else {
		psoc4_info->cmd_program_row = PSOC4_CMD_PROGRAM_ROW;
		LOG_INFO("Flash auto-erase disabled. Use psoc mass_erase before flash programming.");
	}

	return retval;
}


static int psoc4_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	uint32_t *sysrq_buffer = NULL;
	const int param_sz = 8;

	int retval = psoc4_flash_prepare(bank);
	if (retval != ERROR_OK)
		return retval;

	sysrq_buffer = malloc(param_sz + psoc4_info->row_size);
	if (sysrq_buffer == NULL) {
		LOG_ERROR("no memory for row buffer");
		return ERROR_FAIL;
	}

	uint8_t *row_buffer = (uint8_t *)sysrq_buffer + param_sz;
	uint32_t row_num = offset / psoc4_info->row_size;
	uint32_t row_offset = offset - row_num * psoc4_info->row_size;
	if (row_offset)
		memset(row_buffer, bank->default_padded_value, row_offset);

	bool save_poll = jtag_poll_get_enabled();
	jtag_poll_set_enabled(false);

	while (count) {
		uint32_t chunk_size = psoc4_info->row_size - row_offset;
		if (chunk_size > count) {
			chunk_size = count;
			memset(row_buffer + chunk_size, bank->default_padded_value, psoc4_info->row_size - chunk_size);
		}
		memcpy(row_buffer + row_offset, buffer, chunk_size);
		LOG_DEBUG("offset / row: 0x%08" PRIx32 " / %" PRIu32 ", size %" PRIu32 "",
				offset, row_offset, chunk_size);

		uint32_t macro_idx = row_num / PSOC4_ROWS_PER_MACRO;

		/* Call "Load Latch" system ROM API */
		target_buffer_set_u32(target, (uint8_t *)(sysrq_buffer + 1),
					psoc4_info->row_size - 1);
		retval = psoc4_sysreq(bank, PSOC4_CMD_LOAD_LATCH,
				0	/* Byte number in latch from what to write */
				  | (macro_idx << 8),
				sysrq_buffer, param_sz + psoc4_info->row_size,
				NULL);
		if (retval != ERROR_OK)
			goto cleanup;

		/* Call "Program Row" or "Write Row" system ROM API */
		uint32_t sysrq_param;
		retval = psoc4_sysreq(bank, psoc4_info->cmd_program_row,
				row_num & 0xffff,
				&sysrq_param, sizeof(sysrq_param),
				NULL);
		if (retval != ERROR_OK)
			goto cleanup;

		buffer += chunk_size;
		row_num++;
		row_offset = 0;
		count -= chunk_size;
	}

cleanup:
	jtag_poll_set_enabled(save_poll);

	free(sysrq_buffer);
	return retval;
}


/* Due to Cypress's method of market segmentation some devices
 * have accessible only 1/2, 1/4 or 1/8 of SPCIF described flash */
static int psoc4_test_flash_wounding(struct target *target, uint32_t flash_size)
{
	int retval, i;
	for (i = 3; i >= 1; i--) {
		uint32_t addr = flash_size >> i;
		uint32_t dummy;
		retval = target_read_u32(target, addr, &dummy);
		if (retval != ERROR_OK)
			return i;
	}
	return 0;
}


static int psoc4_probe(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	struct target *target = bank->target;

	int retval;
	uint16_t family_id;

	psoc4_info->probed = false;

	retval = psoc4_get_family(target, &family_id);
	if (retval != ERROR_OK)
		return retval;

	const struct psoc4_chip_family *family = psoc4_family_by_id(family_id);

	if (family->id == 0) {
		LOG_ERROR("Cannot identify PSoC 4 family.");
		return ERROR_FAIL;
	}

	if (family->flags & PSOC4_FAMILY_FLAG_LEGACY) {
		LOG_INFO("%s legacy family detected.", family->name);
		psoc4_info->legacy_family = true;
		psoc4_info->cpuss_sysreq_addr = PSOC4_CPUSS_SYSREQ_LEGACY;
		psoc4_info->cpuss_sysarg_addr = PSOC4_CPUSS_SYSARG_LEGACY;
		psoc4_info->spcif_geometry_addr = PSOC4_SPCIF_GEOMETRY_LEGACY;
	} else {
		LOG_INFO("%s family detected.", family->name);
		psoc4_info->legacy_family = false;
		psoc4_info->cpuss_sysreq_addr = PSOC4_CPUSS_SYSREQ_NEW;
		psoc4_info->cpuss_sysarg_addr = PSOC4_CPUSS_SYSARG_NEW;
		psoc4_info->spcif_geometry_addr = PSOC4_SPCIF_GEOMETRY_NEW;
	}

	uint32_t spcif_geometry;
	retval = target_read_u32(target, psoc4_info->spcif_geometry_addr, &spcif_geometry);
	if (retval != ERROR_OK)
		return retval;

	uint32_t flash_size_in_kb = spcif_geometry & 0x3fff;
	/* TRM of legacy, M and L version describes FLASH field as 16-bit.
	 * S-series and PSoC Analog Coprocessor changes spec to 14-bit only.
	 * Impose PSoC Analog Coprocessor limit to all devices as it
	 * does not make any harm: flash size is safely below 4 MByte limit
	 */
	uint32_t row_size = (spcif_geometry >> 22) & 3;
	uint32_t num_macros = (spcif_geometry >> 20) & 3;

	if (psoc4_info->legacy_family) {
		flash_size_in_kb = flash_size_in_kb * 256 / 1024;
		row_size *= 128;
	} else {
		flash_size_in_kb = (flash_size_in_kb + 1) * 256 / 1024;
		row_size = 64 * (row_size + 1);
		num_macros++;
	}

	LOG_DEBUG("SPCIF geometry: %" PRIu32 " kb flash, row %" PRIu32 " bytes.",
		 flash_size_in_kb, row_size);

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (psoc4_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = psoc4_info->user_bank_size / 1024;
	}

	char macros_txt[20] = "";
	if (num_macros > 1)
		snprintf(macros_txt, sizeof(macros_txt), " in %" PRIu32 " macros", num_macros);

	LOG_INFO("flash size = %" PRIu32 " kbytes%s", flash_size_in_kb, macros_txt);

	/* calculate number of pages */
	uint32_t num_rows = flash_size_in_kb * 1024 / row_size;

	/* check number of flash macros */
	if (num_macros != (num_rows + PSOC4_ROWS_PER_MACRO - 1) / PSOC4_ROWS_PER_MACRO)
		LOG_WARNING("Number of macros does not correspond with flash size!");

	if (!psoc4_info->legacy_family) {
		int wounding = psoc4_test_flash_wounding(target, num_rows * row_size);
		if (wounding > 0) {
			flash_size_in_kb = flash_size_in_kb >> wounding;
			num_rows = num_rows >> wounding;
			LOG_INFO("WOUNDING detected: accessible flash size %" PRIu32 " kbytes", flash_size_in_kb);
		}
	}

	free(bank->sectors);

	psoc4_info->family_id = family_id;
	psoc4_info->num_macros = num_macros;
	psoc4_info->row_size = row_size;
	bank->base = 0x00000000;
	bank->size = num_rows * row_size;
	bank->num_sectors = num_rows;
	bank->sectors = alloc_block_array(0, row_size, num_rows);
	if (bank->sectors == NULL)
		return ERROR_FAIL;

	LOG_DEBUG("flash bank set %" PRIu32 " rows", num_rows);
	psoc4_info->probed = true;

	return ERROR_OK;
}

static int psoc4_auto_probe(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	if (psoc4_info->probed)
		return ERROR_OK;
	return psoc4_probe(bank);
}


static int get_psoc4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	if (!psoc4_info->probed)
		return ERROR_FAIL;

	const struct psoc4_chip_family *family = psoc4_family_by_id(psoc4_info->family_id);
	uint32_t size_in_kb = bank->size / 1024;

	if (target->state != TARGET_HALTED) {
		snprintf(buf, buf_size, "%s, flash %" PRIu32 " kb"
			" (halt target to see details)", family->name, size_in_kb);
		return ERROR_OK;
	}

	int retval;
	int printed = 0;
	uint32_t silicon_id;
	uint16_t family_id;
	uint8_t protection;

	retval = psoc4_get_silicon_id(bank, &silicon_id, &family_id, &protection);
	if (retval != ERROR_OK)
		return retval;

	if (family_id != psoc4_info->family_id)
		printed = snprintf(buf, buf_size, "Family id mismatch 0x%02" PRIx16
			"/0x%02" PRIx16 ", silicon id 0x%08" PRIx32,
			psoc4_info->family_id, family_id, silicon_id);
	else {
		printed = snprintf(buf, buf_size, "%s silicon id 0x%08" PRIx32 "",
			family->name, silicon_id);
	}

	buf += printed;
	buf_size -= printed;

	const char *prot_txt = psoc4_decode_chip_protection(protection);
	snprintf(buf, buf_size, ", flash %" PRIu32 " kb %s", size_in_kb, prot_txt);
	return ERROR_OK;
}


COMMAND_HANDLER(psoc4_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = psoc4_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "psoc mass erase complete");
	else
		command_print(CMD, "psoc mass erase failed");

	return retval;
}


static const struct command_registration psoc4_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc4_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "flash_autoerase",
		.handler = psoc4_handle_flash_autoerase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id on|off",
		.help = "Set autoerase mode for flash bank.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc4_command_handlers[] = {
	{
		.name = "psoc4",
		.mode = COMMAND_ANY,
		.help = "PSoC 4 flash command group",
		.usage = "",
		.chain = psoc4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver psoc4_flash = {
	.name = "psoc4",
	.commands = psoc4_command_handlers,
	.flash_bank_command = psoc4_flash_bank_command,
	.erase = psoc4_erase,
	.protect = psoc4_protect,
	.write = psoc4_write,
	.read = default_flash_read,
	.probe = psoc4_probe,
	.auto_probe = psoc4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = psoc4_protect_check,
	.info = get_psoc4_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
