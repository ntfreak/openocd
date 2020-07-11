/***************************************************************************
 *   Copyright (C) 2015 by Bogdan Kolbov                                   *
 *   kolbov@niiet.ru                                                       *
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
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLASH_DRIVER_VER			0x00010000
#define CHIPID_ADDR					0xF0000000
#define K1921VK01T_ID				0x00000000

/*==============================================================================
 *							FLASH CONTROL REGS
 *==============================================================================
 */

#define MAIN_MEM_TYPE				0
#define INFO_MEM_TYPE				1
#define SERVICE_MODE_ERASE_ADDR		0x80030164
#define MAGIC_KEY					0xA442

/*-- BOOTFLASH ---------------------------------------------------------------*/
#define BOOTFLASH_BASE				0xA001C000
#define FMA							(BOOTFLASH_BASE + 0x00)
#define FMD1						(BOOTFLASH_BASE + 0x04)
#define FMC							(BOOTFLASH_BASE + 0x08)
#define FCIS						(BOOTFLASH_BASE + 0x0C)
#define FCIM						(BOOTFLASH_BASE + 0x10)
#define FCIC						(BOOTFLASH_BASE + 0x14)
#define FMD2						(BOOTFLASH_BASE + 0x50)
#define FMD3						(BOOTFLASH_BASE + 0x54)
#define FMD4						(BOOTFLASH_BASE + 0x58)


/*---- FMC: Command register */
#define FMC_WRITE					(1<<0)				/* Writing in main region */
#define FMC_PAGE_ERASE				(1<<1)				/* Page erase the main region */
#define FMC_FULL_ERASE				(1<<2)				/* Erase full flash */
#define FMC_WRITE_IFB				(1<<4)				/* Writing in info region */
#define FMC_PAGEERASE_IFB			(1<<5)				/* Erase page of info region */
#define FMC_MAGIC_KEY				(MAGIC_KEY<<16)		/* Operation run command */

/*---- FCIS: Status register */
#define FCIS_OP_CMLT				(1<<0)				/* Completion flag operation */
#define FCIS_OP_ERROR				(1<<1)				/* Flag operation error */

/*---- FCIC: CLear status register */
#define FCIC_CLR_OPCMLT				(1<<0)				/* Clear completion flag in register FCIS */
#define FCIC_CLR_OPERROR			(1<<1)				/* Clear error flag in register FCIS */

/*-- USERFLASH ---------------------------------------------------------------*/
#define USERFLASH_PAGE_SIZE			256
#define USERFLASH_PAGE_TOTALNUM		256

#define USERFLASH_BASE				0xA0022000
#define UFMA						(USERFLASH_BASE + 0x00)
#define UFMD						(USERFLASH_BASE + 0x04)
#define UFMC						(USERFLASH_BASE + 0x08)
#define UFCIS						(USERFLASH_BASE + 0x0C)
#define UFCIM						(USERFLASH_BASE + 0x10)
#define UFCIC						(USERFLASH_BASE + 0x14)

/*---- UFMC: Command register */
#define UFMC_WRITE					(1<<0)				/* Writing in main region */
#define UFMC_PAGE_ERASE				(1<<1)				/* Paged erase the main region */
#define UFMC_FULL_ERASE				(1<<2)				/* Erase full flash */
#define UFMC_READ					(1<<3)				/* Reading from main region */
#define UFMC_WRITE_IFB				(1<<4)				/* Writing in info region */
#define UFMC_PAGEERASE_IFB			(1<<5)				/* Erase page of info region */
#define UFMC_READ_IFB				(1<<6)				/* Reading from info region */
#define UFMC_MAGIC_KEY				(MAGIC_KEY<<16)		/* Operation run command */

/*---- UFCIS: Status register */
#define UFCIS_OP_CMLT				(1<<0)				/* Completion flag operation */
#define UFCIS_OP_ERROR				(1<<1)				/* Flag operation error */

/*---- UFCIC: CLear status register */
#define UFCIC_CLR_OPCMLT			(1<<0)				/* Clear completion flag in register FCIS */
#define UFCIC_CLR_OPERROR			(1<<1)				/* Clear error flag in register FCIS */

/*---- In info userflash address space */
#define INFOWORD0_ADDR				0x00
#define INFOWORD0_BOOTFROM_IFB		(1<<0)				/* Boot from bootflash or bootflash_ifb */
#define INFOWORD0_EN_GPIO			(1<<1)				/* Remap to 0x00000000 extmem or bootflash */
#define INFOWORD0_BOOTFROM_IFB_POS	0
#define INFOWORD0_EN_GPIO_POS		1
#define INFOWORD0_EXTMEM_SEL_POS	3					/* Choose altfunc of gpio to work with extmem */

#define INFOWORD1_ADDR				0x01
#define INFOWORD1_PINNUM_POS		0					/* Choose gpio pin number to control extmem boot */
#define INFOWORD1_PORTNUM_POS		4					/* Choose gpio port to control extmem boot */

#define INFOWORD2_ADDR				0x02
#define INFOWORD2_LOCK_IFB_BF		(1<<0)				/* Protect info part of bootflash */

#define INFOWORD3_ADDR				0x03
#define INFOWORD3_LOCK_IFB_UF		(1<<0)				/* Protect info part of userflash */

#define BF_LOCK_ADDR				0x40
#define UF_LOCK_ADDR				0x80

/**
 * Private data for flash driver.
 */
struct niietcm4_flash_bank {
	/* target params */
	bool probed;
	uint32_t chipid;
	char *chip_name;
	char chip_brief[4096];
	/* not mapped userflash params */
	uint32_t uflash_width;
	uint32_t uflash_size;
	uint32_t uflash_pagetotal;
	uint32_t uflash_info_size;
	uint32_t uflash_info_pagetotal;
	/* boot params */
	bool bflash_info_remap;
	char *extmem_boot_port;
	uint32_t extmem_boot_pin;
	uint32_t extmem_boot_altfunc;
	bool extmem_boot;
};

/*==============================================================================
 *							HELPER FUNCTIONS
 *==============================================================================
 */

/**
 * Wait while operation with bootflash being performed and check result status
 */
static int niietcm4_opstatus_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;
	int timeout = 5000;

	uint32_t flash_status;
	retval = target_read_u32(target, FCIS, &flash_status);
	if (retval != ERROR_OK)
		return retval;

	while (flash_status == 0x00) {
		retval = target_read_u32(target, FCIS, &flash_status);
		if (retval != ERROR_OK)
			return retval;
		if (timeout-- <= 0) {
			LOG_ERROR("Bootflash operation timeout");
			return ERROR_FLASH_OPERATION_FAILED;
			}
		busy_sleep(1);	/* can use busy sleep for short times. */
	}
	if (flash_status == FCIS_OP_ERROR) {
		LOG_ERROR("Bootflash operation error");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	/* clear status */
	uint32_t flash_cmd = FCIC_CLR_OPCMLT | FCIC_CLR_OPERROR;
	retval = target_write_u32(target, FCIC, flash_cmd);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

/**
 * Wait while operation with userflash being performed and check result status
 */
static int niietcm4_uopstatus_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;
	int timeout = 5000;

	uint32_t uflash_status;
	retval = target_read_u32(target, UFCIS, &uflash_status);
	if (retval != ERROR_OK)
		return retval;

	while (uflash_status == 0x00) {
		retval = target_read_u32(target, UFCIS, &uflash_status);
		if (retval != ERROR_OK)
			return retval;
		if (timeout-- <= 0) {
			LOG_ERROR("Userflash operation timeout");
			return ERROR_FLASH_OPERATION_FAILED;
			}
		busy_sleep(1);	/* can use busy sleep for short times. */
	}
	if (uflash_status == UFCIS_OP_ERROR) {
		LOG_ERROR("Userflash operation error");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	/* clear status */
	uint32_t uflash_cmd = UFCIC_CLR_OPCMLT | UFCIC_CLR_OPERROR;
	retval = target_write_u32(target, UFCIC, uflash_cmd);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

/**
 * Dump page of userflash region.
 * If we want to change some settings, we have to dump it full, because userflash is flash(not EEPROM).
 * And correct write to flash can be performed only after erase.
 * So without dump, changing one registers will clear others.
 */
static int niietcm4_dump_uflash_page(struct flash_bank *bank, uint32_t *dump, int page_num, int mem_type)
{
	struct target *target = bank->target;
	int i;
	int retval = ERROR_OK;

	uint32_t uflash_cmd;
	if (mem_type == INFO_MEM_TYPE)
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ_IFB;
	else
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ;

	int first = page_num*USERFLASH_PAGE_SIZE;
	int last = first + USERFLASH_PAGE_SIZE;

	for (i = first; i < last; i++) {
		retval = target_write_u32(target, UFMA, i);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, UFMC, uflash_cmd);
		if (retval != ERROR_OK)
			return retval;
		retval = niietcm4_uopstatus_check(bank);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, UFMD, &dump[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

/**
 * Load modified page dump to userflash region page.
 */
static int niietcm4_load_uflash_page(struct flash_bank *bank, uint32_t *dump, int page_num, int mem_type)
{
	struct target *target = bank->target;
	int i;
	int retval = ERROR_OK;

	uint32_t uflash_cmd;
	if (mem_type == INFO_MEM_TYPE)
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_WRITE_IFB;
	else
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_WRITE;

	int first = page_num*USERFLASH_PAGE_SIZE;
	int last = first + USERFLASH_PAGE_SIZE;

	for (i = first; i < last; i++) {
		retval = target_write_u32(target, UFMA, i);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, UFMD, dump[i]);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, UFMC, uflash_cmd);
		if (retval != ERROR_OK)
			return retval;
		retval = niietcm4_uopstatus_check(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

/**
 * Erase one page of userflash info or main region
 */
static int niietcm4_uflash_page_erase(struct flash_bank *bank, int page_num, int mem_type)
{
	struct target *target = bank->target;
	int retval;

	uint32_t uflash_cmd;
	if (mem_type == INFO_MEM_TYPE)
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_PAGEERASE_IFB;
	else
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_PAGE_ERASE;

	retval = target_write_u32(target, UFMA, page_num*USERFLASH_PAGE_SIZE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, UFMD, 0xFF);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, UFMC, uflash_cmd);
	if (retval != ERROR_OK)
		return retval;
	/* status check */
	retval = niietcm4_uopstatus_check(bank);
	if (retval != ERROR_OK)
			return retval;

	return retval;
}

/**
 * Enable or disable protection of userflash pages
 */
static int niietcm4_uflash_protect(struct flash_bank *bank, int mem_type,
		int set, unsigned int first, unsigned int last)
{
	int retval;
	if (mem_type == INFO_MEM_TYPE) {
		/* read dump */
		uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
		retval = niietcm4_dump_uflash_page(bank, uflash_dump, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* modify dump */
		if (set)
			uflash_dump[INFOWORD2_ADDR] &= ~INFOWORD3_LOCK_IFB_UF;
		else
			uflash_dump[INFOWORD2_ADDR] |= INFOWORD3_LOCK_IFB_UF;
		/* erase page 0 userflash */
		retval = niietcm4_uflash_page_erase(bank, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* write dump to userflash */
		retval = niietcm4_load_uflash_page(bank, uflash_dump, 0, 1);
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* read dump */
		uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
		retval = niietcm4_dump_uflash_page(bank, uflash_dump, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* modify dump */
		for (unsigned int i = first; i <= last; i++) {
			uint32_t reg_num = i/8;
			uint32_t bit_num = i%8;
			if (set)
				uflash_dump[UF_LOCK_ADDR+reg_num] &= ~(1<<bit_num);
			else
				uflash_dump[UF_LOCK_ADDR+reg_num] |= (1<<bit_num);
		}
		/* erase page 0 info userflash */
		retval = niietcm4_uflash_page_erase(bank, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* write dump to userflash */
		retval = niietcm4_load_uflash_page(bank, uflash_dump,  0, 1);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

/*==============================================================================
 *							FLASH COMMANDS
 *==============================================================================
 */
COMMAND_HANDLER(niietcm4_handle_uflash_read_byte_command)
{
	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	uint32_t uflash_addr;
	uint32_t uflash_cmd;
	uint32_t uflash_data;

	if (strcmp("info", CMD_ARGV[0]) == 0)
			uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ_IFB;
	else if (strcmp("main", CMD_ARGV[0]) == 0)
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], uflash_addr);

	retval = target_write_u32(target, UFMA, uflash_addr);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, UFMC, uflash_cmd);
	if (retval != ERROR_OK)
		return retval;
	/* status check */
	retval = niietcm4_uopstatus_check(bank);
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u32(target, UFMD, &uflash_data);
	if (retval != ERROR_OK)
		return retval;
	command_print(CMD,  "Read userflash %s region:\n"
						"address = 0x%04x,\n"
						"value   = 0x%02x.", CMD_ARGV[0], uflash_addr, uflash_data);
	return retval;
}

COMMAND_HANDLER(niietcm4_handle_uflash_write_byte_command)
{
	if (CMD_ARGC < 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	uint32_t uflash_addr;
	uint32_t uflash_data;
	int mem_type;

	if (strcmp("info", CMD_ARGV[0]) == 0)
		mem_type = 1;
	else if (strcmp("main", CMD_ARGV[0]) == 0)
		mem_type = 0;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], uflash_addr);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[2], uflash_data);

	int page_num = uflash_addr/USERFLASH_PAGE_SIZE;

	command_print(CMD, "Write userflash %s region:\n"
					   "address = 0x%04x,\n"
					   "value   = 0x%02x.\n"
					   "Please wait ... ", CMD_ARGV[0], uflash_addr, uflash_data);
	/* dump */
	uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
	niietcm4_dump_uflash_page(bank, uflash_dump, page_num, mem_type);

	/* modify dump */
	uflash_dump[uflash_addr%USERFLASH_PAGE_SIZE] = uflash_data;

	/* erase page userflash */
	niietcm4_uflash_page_erase(bank, page_num, mem_type);

	/* write dump to userflash */
	niietcm4_load_uflash_page(bank, uflash_dump, page_num, mem_type);
	command_print(CMD, "done!");
	return retval;
}

COMMAND_HANDLER(niietcm4_handle_uflash_full_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t uflash_addr = 0;
	uint32_t uflash_data = 0xFF;
	uint32_t uflash_cmd = UFMC_MAGIC_KEY | UFMC_FULL_ERASE;

	retval = target_write_u32(target, UFMA, uflash_addr);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, UFMD, uflash_data);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, UFMC, uflash_cmd);
	if (retval != ERROR_OK)
		return retval;
	/* status check */
	retval = niietcm4_uopstatus_check(bank);
	if (retval != ERROR_OK)
		return retval;
	command_print(CMD, "Userflash full erase done!");

	return retval;
}

COMMAND_HANDLER(niietcm4_handle_uflash_erase_command)
{
	if (CMD_ARGC < 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	unsigned int first, last;
	int mem_type;

	if (strcmp("info", CMD_ARGV[0]) == 0)
			mem_type = 1;
	else if (strcmp("main", CMD_ARGV[0]) == 0)
		mem_type = 0;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], first);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[2], last);
	for (unsigned int i = first; i <= last; i++) {
		retval = niietcm4_uflash_page_erase(bank, i, mem_type);
		if (retval != ERROR_OK)
			return retval;
	}

	command_print(CMD, "Erase %s userflash pages %u through %u done!", CMD_ARGV[0], first, last);

	return retval;
}

COMMAND_HANDLER(niietcm4_handle_uflash_protect_check_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	int mem_type;
	if (strcmp("info", CMD_ARGV[0]) == 0)
		mem_type = 1;
	else if (strcmp("main", CMD_ARGV[0]) == 0)
		mem_type = 0;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	int i, j;
	uint32_t uflash_addr;
	uint32_t uflash_cmd;
	uint32_t uflash_data;

	/* chose between main userflash and info userflash */
	if (mem_type == INFO_MEM_TYPE) {
		uflash_addr = INFOWORD3_ADDR;
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ_IFB;
		retval = target_write_u32(target, UFMA, uflash_addr);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, UFMC, uflash_cmd);
		if (retval != ERROR_OK)
			return retval;

		/* status check */
		retval = niietcm4_uopstatus_check(bank);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, UFMD, &uflash_data);
		if (retval != ERROR_OK)
			return retval;

		if (uflash_data & INFOWORD3_LOCK_IFB_UF)
			command_print(CMD, "All sectors of info userflash are not protected!");
		else
			command_print(CMD, "All sectors of info userflash are protected!");
	} else {
		uflash_addr = UF_LOCK_ADDR;
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ_IFB;
		for (i = 0; i < USERFLASH_PAGE_TOTALNUM/8; i++) {
			retval = target_write_u32(target, UFMA, uflash_addr);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u32(target, UFMC, uflash_cmd);
			if (retval != ERROR_OK)
				return retval;

			/* status check */
			retval = niietcm4_uopstatus_check(bank);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_u32(target, UFMD, &uflash_data);
			if (retval != ERROR_OK)
				return retval;

			for (j = 0; j < 8; j++) {
				if (uflash_data & 0x1)
					command_print(CMD, "Userflash sector #%03d: 0x%04x (0x100) is not protected!",
											i*8+j, (i*8+j)*USERFLASH_PAGE_SIZE);
				else
					command_print(CMD, "Userflash sector #%03d: 0x%04x (0x100) is protected!",
											i*8+j, (i*8+j)*USERFLASH_PAGE_SIZE);
				uflash_data = uflash_data >> 1;
			}
			uflash_addr++;
		}
	}

	return retval;
}

COMMAND_HANDLER(niietcm4_handle_uflash_protect_command)
{
	if (CMD_ARGC < 5)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	int mem_type;
	if (strcmp("info", CMD_ARGV[0]) == 0)
		mem_type = 1;
	else if (strcmp("main", CMD_ARGV[0]) == 0)
		mem_type = 0;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned int first, last;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], first);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[2], last);

	int set;
	if (strcmp("on", CMD_ARGV[3]) == 0) {
		command_print(CMD, "Try to enable %s userflash sectors %u through %u protection. Please wait ... ",
								CMD_ARGV[0], first, last);
		set = 1;
	} else if (strcmp("off", CMD_ARGV[3]) == 0) {
		command_print(CMD, "Try to disable %s userflash sectors %u through %u protection. Please wait ... ",
								CMD_ARGV[0], first, last);
		set = 0;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = niietcm4_uflash_protect(bank, mem_type, set, first, last);
		if (retval != ERROR_OK)
			return retval;

	command_print(CMD, "done!");
	return retval;
}

COMMAND_HANDLER(niietcm4_handle_bflash_info_remap_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	int set;
	if (strcmp("on", CMD_ARGV[0]) == 0) {
		command_print(CMD, "Try to enable bootflash info region remap. Please wait ...");
		set = 1;
	} else if (strcmp("off", CMD_ARGV[0]) == 0) {
		command_print(CMD, "Try to disable bootflash info region remap. Please wait ...");
		set = 0;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* dump */
	uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
	niietcm4_dump_uflash_page(bank, uflash_dump, 0, 1);

	/* modify dump */
	if (set)
		uflash_dump[INFOWORD0_ADDR] &= ~INFOWORD0_BOOTFROM_IFB;
	else
		uflash_dump[INFOWORD0_ADDR] |= INFOWORD0_BOOTFROM_IFB;

	/* erase page userflash */
	niietcm4_uflash_page_erase(bank, 0, 1);

	/* write dump to userflash */
	niietcm4_load_uflash_page(bank, uflash_dump, 0, 1);
	command_print(CMD, "done!");

	return retval;
}

COMMAND_HANDLER(niietcm4_handle_extmem_cfg_command)
{
	if (CMD_ARGC < 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	uint32_t port;
	if (strcmp("gpioa", CMD_ARGV[0]) == 0)
		port = 8;
	else if (strcmp("gpiob", CMD_ARGV[0]) == 0)
		port = 9;
	else if (strcmp("gpioc", CMD_ARGV[0]) == 0)
		port = 10;
	else if (strcmp("gpiod", CMD_ARGV[0]) == 0)
		port = 11;
	else if (strcmp("gpioe", CMD_ARGV[0]) == 0)
		port = 12;
	else if (strcmp("gpiof", CMD_ARGV[0]) == 0)
		port = 13;
	else if (strcmp("gpiog", CMD_ARGV[0]) == 0)
		port = 14;
	else if (strcmp("gpioh", CMD_ARGV[0]) == 0)
		port = 15;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t pin;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], pin);
	if (pin > 15)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t func;
	if (strcmp("func1", CMD_ARGV[2]) == 0)
		func = 0;
	else if (strcmp("func3", CMD_ARGV[2]) == 0)
		func = 3;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD,  "Try to configure external memory boot interface:\n"
						"port = %s\n"
						"pin  = %s\n"
						"func = %s\n"
						"Please wait ...", CMD_ARGV[0], CMD_ARGV[1], CMD_ARGV[2]);
	/* dump */
	uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
	niietcm4_dump_uflash_page(bank, uflash_dump, 0, 1);

	/* modify dump */
	uflash_dump[INFOWORD0_ADDR] &= ~(3<<INFOWORD0_EXTMEM_SEL_POS);
	uflash_dump[INFOWORD0_ADDR] |= func<<INFOWORD0_EXTMEM_SEL_POS;
	uflash_dump[INFOWORD1_ADDR] = (port<<INFOWORD1_PORTNUM_POS) | (pin<<INFOWORD1_PINNUM_POS);

	/* erase page userflash */
	niietcm4_uflash_page_erase(bank, 0, 1);

	/* write dump to userflash */
	niietcm4_load_uflash_page(bank, uflash_dump, 0, 1);
	command_print(CMD, "done!");

	return retval;
}

COMMAND_HANDLER(niietcm4_handle_extmem_boot_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	int set;

	if (strcmp("on", CMD_ARGV[0]) == 0) {
		command_print(CMD, "Try to enable boot from external memory. Please wait ...");
		set = 1;
	} else if (strcmp("off", CMD_ARGV[0]) == 0) {
		command_print(CMD, "Try to disable boot from external memory. Please wait ...");
		set = 0;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* dump */
	uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
	niietcm4_dump_uflash_page(bank, uflash_dump, 0, 1);

	/* modify dump */
	if (set)
		uflash_dump[INFOWORD0_ADDR] &= ~INFOWORD0_EN_GPIO;
	else
		uflash_dump[INFOWORD0_ADDR] |= INFOWORD0_EN_GPIO;

	/* erase page userflash */
	niietcm4_uflash_page_erase(bank, 0, 1);

	/* write dump to userflash */
	niietcm4_load_uflash_page(bank, uflash_dump, 0, 1);
	command_print(CMD, "done!");

	return retval;
}

COMMAND_HANDLER(niietcm4_handle_service_mode_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	struct target *target = bank->target;

	command_print(CMD, "Try to perform service mode erase. Please wait ...");

	retval = target_write_u32(target, SERVICE_MODE_ERASE_ADDR, 1);
	if (retval != ERROR_OK)
		return retval;

	int timeout = 500;
	uint32_t status;

	retval = target_read_u32(target, SERVICE_MODE_ERASE_ADDR, &status);
	if (retval != ERROR_OK)
		return retval;

	while (status != 0x03) {
		retval = target_read_u32(target, SERVICE_MODE_ERASE_ADDR, &status);
		if (retval != ERROR_OK)
			return retval;
		if (timeout-- <= 0) {
			LOG_ERROR("Service mode erase timeout");
			return ERROR_FLASH_OPERATION_FAILED;
			}
		busy_sleep(1);	/* can use busy sleep for short times. */
	}
	command_print(CMD, "done! All data erased.");

	return retval;
}

COMMAND_HANDLER(niietcm4_handle_driver_info_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "niietcm4 flash driver\n"
						   "version: %d.%d\n"
						   "author: Bogdan Kolbov\n"
						   "mail: kolbov@niiet.ru",
						   FLASH_DRIVER_VER>>16,
						   FLASH_DRIVER_VER&0xFFFF);

	return retval;
}

static const struct command_registration niietcm4_exec_command_handlers[] = {
	{
		.name = "uflash_read_byte",
		.handler = niietcm4_handle_uflash_read_byte_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('main'|'info') address",
		.help = "Read byte from main or info userflash region",
	},
	{
		.name = "uflash_write_byte",
		.handler = niietcm4_handle_uflash_write_byte_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('main'|'info') address value",
		.help = "Write byte to main or info userflash region",
	},
	{
		.name = "uflash_full_erase",
		.handler = niietcm4_handle_uflash_full_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase all userflash including info region",
	},
	{
		.name = "uflash_erase",
		.handler = niietcm4_handle_uflash_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('main'|'info') first_sector_num last_sector_num",
		.help = "Erase sectors of main or info userflash region, starting at sector first up to and including last.",
	},
	{
		.name = "uflash_protect_check",
		.handler = niietcm4_handle_uflash_protect_check_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('main'|'info')",
		.help = "Check sectors protect.",
	},
	{
		.name = "uflash_protect",
		.handler = niietcm4_handle_uflash_protect_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('main'|'info') first_sector_num last_sector_num ('on'|'off')",
		.help = "Protect sectors of main or info userflash region, starting at sector first up to and including last.",
	},
	{
		.name = "bflash_info_remap",
		.handler = niietcm4_handle_bflash_info_remap_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('on'|'off')",
		.help = "Enable remapping bootflash info region to 0x00000000 (or 0x40000000 if external memory boot used).",
	},
	{
		.name = "extmem_cfg",
		.handler = niietcm4_handle_extmem_cfg_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('gpioa'|'gpiob'|'gpioc'|'gpiod'|'gpioe'|'gpiof'|'gpiog'|'gpioh') pin_num ('func1'|'func3')",
		.help = "Configure external memory interface for boot.",
	},
	{
		.name = "extmem_boot",
		.handler = niietcm4_handle_extmem_boot_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('on'|'off')",
		.help = "Enable boot from external memory.",
	},
	{
		.name = "service_mode_erase",
		.handler = niietcm4_handle_service_mode_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Perform emergency erase of all flash (bootflash and userflash).",
	},
	{
		.name = "driver_info",
		.handler = niietcm4_handle_driver_info_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Show information about flash driver.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration niietcm4_command_handlers[] = {
	{
		.name = "niietcm4",
		.mode = COMMAND_ANY,
		.help = "niietcm4 flash command group",
		.usage = "",
		.chain = niietcm4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/*==============================================================================
 *							FLASH INTERFACE
 *==============================================================================
 */

FLASH_BANK_COMMAND_HANDLER(niietcm4_flash_bank_command)
{
	struct niietcm4_flash_bank *niietcm4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	niietcm4_info = malloc(sizeof(struct niietcm4_flash_bank));

	bank->driver_priv = niietcm4_info;

	/* information will be updated by probing */
	niietcm4_info->probed = false;
	niietcm4_info->chipid = 0;
	niietcm4_info->chip_name = NULL;
	niietcm4_info->uflash_width = 0;
	niietcm4_info->uflash_size = 0;
	niietcm4_info->uflash_pagetotal = 0;
	niietcm4_info->uflash_info_size = 0;
	niietcm4_info->uflash_info_pagetotal = 0;
	niietcm4_info->bflash_info_remap = false;
	niietcm4_info->extmem_boot_port = NULL;
	niietcm4_info->extmem_boot_pin = 0;
	niietcm4_info->extmem_boot_altfunc = 0;
	niietcm4_info->extmem_boot = false;

	return ERROR_OK;
}

static int niietcm4_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;

	int retval = ERROR_FLASH_OPERATION_FAILED;
	int set;
	uint32_t uflash_addr;
	uint32_t uflash_cmd;
	uint32_t uflash_data;
	/* chose between main bootflash and info bootflash  */
	if (niietcm4_info->bflash_info_remap) {
		uflash_addr = INFOWORD2_ADDR;
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ_IFB;
		retval = target_write_u32(target, UFMA, uflash_addr);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, UFMC, uflash_cmd);
		if (retval != ERROR_OK)
			return retval;

		/* status check */
		retval = niietcm4_uopstatus_check(bank);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, UFMD, &uflash_data);
		if (retval != ERROR_OK)
			return retval;

		if (uflash_data & INFOWORD2_LOCK_IFB_BF)
			set = 0;
		else
			set = 1;
		bank->sectors[0].is_protected = set;
	} else {
		uflash_addr = BF_LOCK_ADDR;
		uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ_IFB;
		for (unsigned int i = 0; i < bank->num_sectors/8; i++) {
			retval = target_write_u32(target, UFMA, uflash_addr);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u32(target, UFMC, uflash_cmd);
			if (retval != ERROR_OK)
				return retval;

			/* status check */
			retval = niietcm4_uopstatus_check(bank);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_u32(target, UFMD, &uflash_data);
			if (retval != ERROR_OK)
				return retval;

			for (int j = 0; j < 8; j++) {
				if (uflash_data & 0x1)
					set = 0;
				else
					set = 1;
				bank->sectors[i*8+j].is_protected = set;
				uflash_data = uflash_data >> 1;
			}
			uflash_addr++;
		}
	}

	return retval;
}

static int niietcm4_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	int retval;
	uint32_t flash_cmd;

	/* start mass erase */
	flash_cmd = FMC_MAGIC_KEY | FMC_FULL_ERASE;
	retval = target_write_u32(target, FMC, flash_cmd);
	if (retval != ERROR_OK)
		return retval;

	/* status check */
	retval = niietcm4_opstatus_check(bank);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int niietcm4_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;
	int retval = ERROR_FLASH_OPERATION_FAILED;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1))) {
		retval = niietcm4_mass_erase(bank);
		return retval;
	}

	/* chose between main bootflash and info bootflash */
	uint32_t flash_cmd, flash_addr;
	if (niietcm4_info->bflash_info_remap)
		flash_cmd = FMC_MAGIC_KEY | FMC_PAGEERASE_IFB;
	else
		flash_cmd = FMC_MAGIC_KEY | FMC_PAGE_ERASE;

	/* erasing pages */
	unsigned int page_size = bank->size / bank->num_sectors;
	for (unsigned int i = first; i <= last; i++) {
		/* current page addr */
		flash_addr = i*page_size;
		retval = target_write_u32(target, FMA, flash_addr);
		if (retval != ERROR_OK)
			return retval;

		/* start erase */
		retval = target_write_u32(target, FMC, flash_cmd);
		if (retval != ERROR_OK)
			return retval;

		/* status check */
		retval = niietcm4_opstatus_check(bank);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	return retval;
}

static int niietcm4_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;

	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Please wait ..."); /* it`s quite a long process */
	/* chose between main bootflash and info bootflash */
	if (niietcm4_info->bflash_info_remap) {
		/* dump */
		uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
		retval = niietcm4_dump_uflash_page(bank, uflash_dump, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* modify dump */
		if (set)
			uflash_dump[INFOWORD2_ADDR] &= ~INFOWORD2_LOCK_IFB_BF;
		else
			uflash_dump[INFOWORD2_ADDR] |= INFOWORD2_LOCK_IFB_BF;
		/* erase page 0 userflash */
		retval = niietcm4_uflash_page_erase(bank, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* write dump to userflash */
		retval = niietcm4_load_uflash_page(bank, uflash_dump, 0, 1);
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* read dump*/
		uint32_t uflash_dump[USERFLASH_PAGE_SIZE];
		retval = niietcm4_dump_uflash_page(bank, uflash_dump, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* modify dump */
		for (unsigned int i = first; i <= last; i++)	{
			uint32_t reg_num = i/8;
			uint32_t bit_num = i%8;
			if (set)
				uflash_dump[BF_LOCK_ADDR+reg_num] &= ~(1<<bit_num);
			else
				uflash_dump[BF_LOCK_ADDR+reg_num] |= (1<<bit_num);
		}
		/* erase page 0 info userflash */
		retval = niietcm4_uflash_page_erase(bank, 0, 1);
		if (retval != ERROR_OK)
			return retval;
		/* write dump to userflash */
		retval = niietcm4_load_uflash_page(bank, uflash_dump,  0, 1);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int niietcm4_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;
	uint32_t buffer_size = 32768 + 8; /* 8 bytes for rp and wp */
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* see contrib/loaders/flash/k1921vk01t.S for src */
	static const uint8_t niietcm4_flash_write_code[] = {
		0x14, 0x4f, 0x16, 0x68, 0x00, 0x2e, 0x23, 0xd0, 0x55, 0x68, 0xb5, 0x42, 0xf9, 0xd0, 0x2e, 0x68,
		0x7e, 0x60, 0x04, 0x35, 0x2e, 0x68, 0x3e, 0x65, 0x04, 0x35, 0x2e, 0x68, 0x7e, 0x65, 0x04, 0x35,
		0x2e, 0x68, 0xbe, 0x65, 0x04, 0x35, 0x3c, 0x60, 0x10, 0x34, 0xb8, 0x60, 0xfe, 0x68, 0x00, 0x2e,
		0xfc, 0xd0, 0x02, 0x2e, 0x0a, 0xd0, 0x01, 0x26, 0x7e, 0x61, 0x9d, 0x42, 0x01, 0xd3, 0x15, 0x46,
		0x08, 0x35, 0x55, 0x60, 0x01, 0x39, 0x00, 0x29, 0x02, 0xd0, 0xda, 0xe7, 0x00, 0x20, 0x50, 0x60,
		0x30, 0x46, 0x00, 0xbe, 0x00, 0xc0, 0x01, 0xa0
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(niietcm4_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(niietcm4_flash_write_code), niietcm4_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~15UL; /* Make sure it's 16 byte aligned */
		buffer_size += 8; /* And 8 bytes for WP and RP */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* write_cmd base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (128bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	uint32_t flash_cmd;
	if (niietcm4_info->bflash_info_remap)
		flash_cmd = FMC_MAGIC_KEY | FMC_WRITE_IFB;
	else
		flash_cmd = FMC_MAGIC_KEY | FMC_WRITE;

	buf_set_u32(reg_params[0].value, 0, 32, flash_cmd);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, count, 16,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int niietcm4_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;
	uint8_t *new_buffer = NULL;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0xF) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-word alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of words, the data has to be padded. Duplicate
	 * the buffer and use the normal code path with a single block write since
	 * it's probably cheaper than to special case the last odd write using
	 * discrete accesses. */

	int rem = count % 16;
	if (rem) {
		new_buffer = malloc(count + 16 - rem);
		if (new_buffer == NULL) {
			LOG_ERROR("Odd number of words to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("Odd number of words to write, padding with 0xFFFFFFFF");
		buffer = memcpy(new_buffer, buffer, count);
		while (rem < 16) {
			new_buffer[count++] = 0xff;
			rem++;
		}
	}

	int retval;

	/* try using block write */
	retval = niietcm4_write_block(bank, buffer, offset, count/16);
	uint32_t flash_addr, flash_cmd, flash_data;

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("Can't use block writes, falling back to single memory accesses");
		LOG_INFO("Please wait ..."); /* it`s quite a long process */

		/* chose between main bootflash and info bootflash */
		if (niietcm4_info->bflash_info_remap)
			flash_cmd = FMC_MAGIC_KEY | FMC_WRITE_IFB;
		else
			flash_cmd = FMC_MAGIC_KEY | FMC_WRITE;

		/* write 16 bytes per try */
		for (unsigned int i = 0; i < count; i += 16) {
			/* current addr */
			LOG_INFO("%d byte of %d", i, count);
			flash_addr = offset + i;
			retval = target_write_u32(target, FMA, flash_addr);
			if (retval != ERROR_OK)
				goto free_buffer;

			/* Prepare data (4 words) */
			uint32_t value[4];
			memcpy(&value, buffer + i*16, 4*sizeof(uint32_t));

			/* place in reg 16 bytes of data */
			flash_data = value[0];
			retval = target_write_u32(target, FMD1, flash_data);
			if (retval != ERROR_OK)
				goto free_buffer;
			flash_data = value[1];
			retval = target_write_u32(target, FMD2, flash_data);
			if (retval != ERROR_OK)
				goto free_buffer;
			flash_data = value[2];
			retval = target_write_u32(target, FMD3, flash_data);
			if (retval != ERROR_OK)
				goto free_buffer;
			flash_data = value[3];
			retval = target_write_u32(target, FMD4, flash_data);
			if (retval != ERROR_OK)
				goto free_buffer;

			/* write start */
			retval = target_write_u32(target, FMC, flash_cmd);
			if (retval != ERROR_OK)
				goto free_buffer;

			/* status check */
			retval = niietcm4_opstatus_check(bank);
			if (retval != ERROR_OK)
				goto free_buffer;
		}

	}

free_buffer:
	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int niietcm4_probe_k1921vk01t(struct flash_bank *bank)
{
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;

	niietcm4_info->chip_name = "K1921VK01T";

	/* check if we in service mode */
	uint32_t service_mode;
	retval = target_read_u32(target, 0x80017000, &service_mode);
	if (retval != ERROR_OK)
		return retval;
	service_mode = (service_mode>>2) & 0x1;

	if (!service_mode) {
		niietcm4_info->uflash_width = 8;
		niietcm4_info->uflash_size = 0x10000;
		niietcm4_info->uflash_pagetotal = 256;
		niietcm4_info->uflash_info_size = 0x200;
		niietcm4_info->uflash_info_pagetotal = 2;

		uint32_t uflash_data[2];
		uint32_t uflash_cmd = UFMC_MAGIC_KEY | UFMC_READ_IFB;
		for (int i = 0; i < 2; i++) {
			retval = target_write_u32(target, UFMA, i);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u32(target, UFMC, uflash_cmd);
			if (retval != ERROR_OK)
				return retval;
			/* status check */
			retval = niietcm4_uopstatus_check(bank);
			if (retval != ERROR_OK)
				return retval;
			retval = target_read_u32(target, UFMD, &uflash_data[i]);
			if (retval != ERROR_OK)
				return retval;
		}

		int boot_from_ifb = (uflash_data[0]>>INFOWORD0_BOOTFROM_IFB_POS) & 0x1;
		int en_gpio = (uflash_data[0]>>INFOWORD0_EN_GPIO_POS) & 0x1;
		int extmem_sel = (uflash_data[0]>>INFOWORD0_EXTMEM_SEL_POS) & 0x3;
		int pinnum = (uflash_data[1]>>INFOWORD1_PINNUM_POS) & 0xF;
		int portnum = (uflash_data[1]>>INFOWORD1_PORTNUM_POS) & 0x7;

		if (boot_from_ifb)
			niietcm4_info->bflash_info_remap = false;
		else
			niietcm4_info->bflash_info_remap = true;
		if (extmem_sel == 0x2)
			niietcm4_info->extmem_boot_altfunc = 3;
		else
			niietcm4_info->extmem_boot_altfunc = 1;
		if (portnum == 0x0)
			niietcm4_info->extmem_boot_port = "GPIOA";
		else if (portnum == 0x1)
			niietcm4_info->extmem_boot_port = "GPIOB";
		else if (portnum == 0x2)
			niietcm4_info->extmem_boot_port = "GPIOC";
		else if (portnum == 0x3)
			niietcm4_info->extmem_boot_port = "GPIOD";
		else if (portnum == 0x4)
			niietcm4_info->extmem_boot_port = "GPIOE";
		else if (portnum == 0x5)
			niietcm4_info->extmem_boot_port = "GPIOF";
		else if (portnum == 0x6)
			niietcm4_info->extmem_boot_port = "GPIOG";
		else if (portnum == 0x7)
			niietcm4_info->extmem_boot_port = "GPIOH";
		else
			niietcm4_info->extmem_boot_port = "not defined";
		if (en_gpio)
			niietcm4_info->extmem_boot = false;
		else
			niietcm4_info->extmem_boot = true;
		niietcm4_info->extmem_boot_pin = pinnum;

		/* check state of extmem boot en pin, if "high", extmem remapped to 0x00000000 */
		uint32_t extmem_boot_port_data;
		retval = target_read_u32(target, 0x80010000 + 0x1000*portnum, &extmem_boot_port_data);
		if (retval != ERROR_OK)
			return retval;
		int extmem_boot_pin_data = (extmem_boot_port_data>>pinnum) & 0x1;

		uint32_t extmem_base;
		uint32_t bflash_base;
		if (extmem_boot_pin_data && niietcm4_info->extmem_boot) {
			extmem_base = 0x00000000;
			bflash_base = 0x40000000;
		} else {
			extmem_base = 0x40000000;
			bflash_base = 0x00000000;
		}

		uint32_t bflash_size = 0x100000;
		uint32_t bflash_pages = 128;
		uint32_t bflash_info_size = 0x2000;
		uint32_t bflash_info_pages = 1;
		if (niietcm4_info->bflash_info_remap) {
			bflash_base += 0x2000;
			bflash_size -= 0x2000;
			bflash_pages--;
			bank->size = bflash_info_size;
			bank->num_sectors = bflash_info_pages;
		} else {
			bank->size = bflash_size;
			bank->num_sectors = bflash_pages;
		}

		char info_bootflash_addr_str[64];
		if (niietcm4_info->bflash_info_remap)
			snprintf(info_bootflash_addr_str, sizeof(info_bootflash_addr_str),
					TARGET_ADDR_FMT " base address", bank->base);
		else
			snprintf(info_bootflash_addr_str, sizeof(info_bootflash_addr_str),
					"not mapped to global address space");

		snprintf(niietcm4_info->chip_brief,
				sizeof(niietcm4_info->chip_brief),
				"\n"
				"MEMORY CONFIGURATION\n"
				"Bootflash :\n"
				"    %d kB total\n"
				"    %d pages %d kB each\n"
				"    0x%08x base address\n"
				"%s"
				"Info bootflash :\n"
				"    %d kB total\n"
				"    %d pages %d kB each\n"
				"    %s\n"
				"%s"
				"Userflash :\n"
				"    %d kB total\n"
				"    %d pages %d B each\n"
				"    %d bit cells\n"
				"    not mapped to global address space\n"
				"Info userflash :\n"
				"    %d B total\n"
				"    %d pages of %d B each\n"
				"    %d bit cells\n"
				"    not mapped to global address space\n"
				"RAM :\n"
				"    192 kB total\n"
				"    0x20000000 base address\n"
				"External memory :\n"
				"    8/16 bit address space\n"
				"    0x%08x base address\n"
				"\n"
				"INFOWORD STATUS\n"
				"Bootflash info region remap :\n"
				"    %s\n"
				"External memory boot port :\n"
				"    %s\n"
				"External memory boot pin :\n"
				"    %d\n"
				"External memory interface alternative function :\n"
				"    %d\n"
				"Option boot from external memory :\n"
				"    %s\n",
				bflash_size/1024,
				bflash_pages,
				(bflash_size/bflash_pages)/1024,
				bflash_base,
				niietcm4_info->bflash_info_remap ? "" : "    this flash will be used for debugging, writing and etc\n",
				bflash_info_size/1024,
				bflash_info_pages,
				(bflash_info_size/bflash_info_pages)/1024,
				info_bootflash_addr_str,
				niietcm4_info->bflash_info_remap ? "    this flash will be used for debugging, writing and etc\n" : "",
				niietcm4_info->uflash_size/1024,
				niietcm4_info->uflash_pagetotal,
				niietcm4_info->uflash_size/niietcm4_info->uflash_pagetotal,
				niietcm4_info->uflash_width,
				niietcm4_info->uflash_info_size,
				niietcm4_info->uflash_info_pagetotal,
				niietcm4_info->uflash_info_size/niietcm4_info->uflash_info_pagetotal,
				niietcm4_info->uflash_width,
				extmem_base,
				niietcm4_info->bflash_info_remap ? "enable" : "disable",
				niietcm4_info->extmem_boot_port,
				niietcm4_info->extmem_boot_pin,
				niietcm4_info->extmem_boot_altfunc,
				niietcm4_info->extmem_boot ? "enable" : "disable");
	} else {
		bank->size = 0x100000;
		bank->num_sectors = 128;

		sprintf(niietcm4_info->chip_brief,
				"\n"
				"H[2] was HIGH while startup. Device entered service mode.\n"
				"All flashes were locked.\n"
				"If you want to perform emergency erase (erase all flashes),\n"
				"please use \"service_mode_erase\" command and reset device.\n"
				"Do not forget to pull H[2] down while reset for returning to normal operation mode.\n"
				);
	}

	return retval;
}

static int niietcm4_probe(struct flash_bank *bank)
{
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;
	struct target *target = bank->target;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}
	uint32_t retval;
	uint32_t chipid;

	retval = target_read_u32(target, CHIPID_ADDR, &chipid);
	if (retval != ERROR_OK) {
		chipid = K1921VK01T_ID;
		LOG_INFO("unknown chipid, assuming K1921VK01T");
	}

	if (chipid == K1921VK01T_ID)
		niietcm4_probe_k1921vk01t(bank);

	int page_total = bank->num_sectors;
	int page_size = bank->size / page_total;

	bank->sectors = malloc(sizeof(struct flash_sector) * page_total);

	for (int i = 0; i < page_total; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	niietcm4_info->probed = true;

	return ERROR_OK;
}

static int niietcm4_auto_probe(struct flash_bank *bank)
{
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;
	if (niietcm4_info->probed)
		return ERROR_OK;
	return niietcm4_probe(bank);
}

static int get_niietcm4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct niietcm4_flash_bank *niietcm4_info = bank->driver_priv;
	LOG_INFO("\nNIIET Cortex-M4F %s\n%s", niietcm4_info->chip_name, niietcm4_info->chip_brief);
	snprintf(buf, buf_size, " ");

	return ERROR_OK;
}


const struct flash_driver niietcm4_flash = {
	.name = "niietcm4",
	.usage = "flash bank <name> niietcm4 <base> <size> 0 0 <target#>",
	.commands = niietcm4_command_handlers,
	.flash_bank_command = niietcm4_flash_bank_command,
	.erase = niietcm4_erase,
	.protect = niietcm4_protect,
	.write = niietcm4_write,
	.read = default_flash_read,
	.probe = niietcm4_probe,
	.auto_probe = niietcm4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = niietcm4_protect_check,
	.info = get_niietcm4_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
