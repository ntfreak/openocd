/*
 * Spansion FM4 flash
 *
 * Copyright (c) 2015 Andreas Färber
 *
 * Based on S6E2DH_MN709-00013 for S6E2DH/DF/D5/D3 series
 * Based on S6E2CC_MN709-00007 for S6E2CC/C5/C4/C3/C2/C1 series
 * Based on MB9B560R_MN709-00005 for MB9BFx66/x67/x68 series
 * Based on MB9B560L_MN709-00006 for MB9BFx64/x65/x66 series
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLASH_BASE 0x40000000
#define FASZR   (FLASH_BASE + 0x000)
#define DFCTRLR (FLASH_BASE + 0x030)
#define DFCTRLR_DFE (1UL << 0)

#define WDG_BASE 0x40011000
#define WDG_CTL (WDG_BASE + 0x008)
#define WDG_LCK (WDG_BASE + 0xC00)

enum fm4_variant {
	mb9bfx64,
	mb9bfx65,
	mb9bfx66,
	mb9bfx67,
	mb9bfx68,

	s6e2cx8,
	s6e2cx9,
	s6e2cxa,

	s6e2dx,
};

struct fm4_flash_bank {
	enum fm4_variant variant;
	int macro_nr;
	bool probed;
};

static int fm4_disable_hw_watchdog(struct target *target)
{
	int retval;

	retval = target_write_u32(target, WDG_LCK, 0x1ACCE551);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, WDG_LCK, 0xE5331AAE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, WDG_CTL, 0);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int fm4_enter_flash_cpu_programming_mode(struct target *target)
{
	uint32_t u32_value;
	int retval;

	/* FASZR ASZ = CPU programming mode */
	retval = target_write_u32(target, FASZR, 0x00000001);
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u32(target, FASZR, &u32_value);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int fm4_enter_flash_cpu_rom_mode(struct target *target)
{
	uint32_t u32_value;
	int retval;

	/* FASZR ASZ = CPU ROM mode */
	retval = target_write_u32(target, FASZR, 0x00000002);
	if (retval != ERROR_OK)
		return retval;
	retval = target_read_u32(target, FASZR, &u32_value);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int fm4_flash_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct working_area *workarea;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_algo;
	unsigned i;
	int retval;
	const uint8_t erase_sector_code[] = {
#include "../../../contrib/loaders/flash/fm4/erase.inc"
	};

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("Spansion FM4 erase sectors %u to %u", first, last);

	retval = fm4_disable_hw_watchdog(target);
	if (retval != ERROR_OK)
		return retval;

	retval = fm4_enter_flash_cpu_programming_mode(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_alloc_working_area(target, sizeof(erase_sector_code),
			&workarea);
	if (retval != ERROR_OK) {
		LOG_ERROR("No working area available.");
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto err_alloc_code;
	}
	retval = target_write_buffer(target, workarea->address,
			sizeof(erase_sector_code), erase_sector_code);
	if (retval != ERROR_OK)
		goto err_write_code;

	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN);

	for (unsigned int sector = first; sector <= last; sector++) {
		uint32_t addr = bank->base + bank->sectors[sector].offset;
		uint32_t result;

		buf_set_u32(reg_params[0].value, 0, 32, (addr & ~0xffff) | 0xAA8);
		buf_set_u32(reg_params[1].value, 0, 32, (addr & ~0xffff) | 0x554);
		buf_set_u32(reg_params[2].value, 0, 32, addr);

		retval = target_run_algorithm(target,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				workarea->address, 0,
				1000, &armv7m_algo);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error executing flash sector erase "
				"programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto err_run;
		}

		result = buf_get_u32(reg_params[3].value, 0, 32);
		if (result == 2) {
			LOG_ERROR("Timeout error from flash sector erase programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto err_run_ret;
		} else if (result != 0) {
			LOG_ERROR("Unexpected error %" PRIu32 " from flash sector erase programming algorithm", result);
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto err_run_ret;
		} else
			retval = ERROR_OK;

		bank->sectors[sector].is_erased = 1;
	}

err_run_ret:
err_run:
	for (i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

err_write_code:
	target_free_working_area(target, workarea);

err_alloc_code:
	if (retval != ERROR_OK)
		fm4_enter_flash_cpu_rom_mode(target);
	else
		retval = fm4_enter_flash_cpu_rom_mode(target);

	return retval;
}

static int fm4_flash_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t byte_count)
{
	struct target *target = bank->target;
	struct working_area *code_workarea, *data_workarea;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_algo;
	uint32_t halfword_count = DIV_ROUND_UP(byte_count, 2);
	uint32_t result;
	unsigned i;
	int retval, retval2 = ERROR_OK;
	const uint8_t write_block_code[] = {
#include "../../../contrib/loaders/flash/fm4/write.inc"
	};

	LOG_DEBUG("Spansion FM4 write at 0x%08" PRIx32 " (%" PRIu32 " bytes)",
		offset, byte_count);

	if (offset & 0x1) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 2-byte alignment",
			offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	if (byte_count & 0x1) {
		LOG_WARNING("length %" PRIu32 " is not 2-byte aligned, rounding up",
			byte_count);
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = fm4_disable_hw_watchdog(target);
	if (retval != ERROR_OK)
		return retval;

	retval = target_alloc_working_area(target, sizeof(write_block_code),
			&code_workarea);
	if (retval != ERROR_OK) {
		LOG_ERROR("No working area available for write code.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	retval = target_write_buffer(target, code_workarea->address,
			sizeof(write_block_code), write_block_code);
	if (retval != ERROR_OK)
		goto err_write_code;

	retval = target_alloc_working_area(target,
		MIN(halfword_count * 2, target_get_working_area_avail(target)),
		&data_workarea);
	if (retval != ERROR_OK) {
		LOG_ERROR("No working area available for write data.");
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto err_alloc_data;
	}

	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "r5", 32, PARAM_IN);

	retval = fm4_enter_flash_cpu_programming_mode(target);
	if (retval != ERROR_OK)
		goto err_flash_mode;

	while (byte_count > 0) {
		uint32_t halfwords = MIN(halfword_count, data_workarea->size / 2);
		uint32_t addr = bank->base + offset;

		LOG_DEBUG("copying %" PRIu32 " bytes to SRAM " TARGET_ADDR_FMT,
			MIN(halfwords * 2, byte_count), data_workarea->address);

		retval = target_write_buffer(target, data_workarea->address,
			MIN(halfwords * 2, byte_count), buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error writing data buffer");
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto err_write_data;
		}

		LOG_DEBUG("writing 0x%08" PRIx32 "-0x%08" PRIx32 " (%" PRIu32 "x)",
			addr, addr + halfwords * 2 - 1, halfwords);

		buf_set_u32(reg_params[0].value, 0, 32, (addr & ~0xffff) | 0xAA8);
		buf_set_u32(reg_params[1].value, 0, 32, (addr & ~0xffff) | 0x554);
		buf_set_u32(reg_params[2].value, 0, 32, addr);
		buf_set_u32(reg_params[3].value, 0, 32, data_workarea->address);
		buf_set_u32(reg_params[4].value, 0, 32, halfwords);

		retval = target_run_algorithm(target,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				code_workarea->address, 0,
				5 * 60 * 1000, &armv7m_algo);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error executing flash sector erase "
				"programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto err_run;
		}

		result = buf_get_u32(reg_params[5].value, 0, 32);
		if (result == 2) {
			LOG_ERROR("Timeout error from flash write "
				"programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto err_run_ret;
		} else if (result != 0) {
			LOG_ERROR("Unexpected error %" PRIu32 " from flash write "
				"programming algorithm", result);
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto err_run_ret;
		} else
			retval = ERROR_OK;

		halfword_count -= halfwords;
		offset += halfwords * 2;
		buffer += halfwords * 2;
		byte_count -= MIN(halfwords * 2, byte_count);
	}

err_run_ret:
err_run:
err_write_data:
	retval2 = fm4_enter_flash_cpu_rom_mode(target);

err_flash_mode:
	for (i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, data_workarea);
err_alloc_data:
err_write_code:
	target_free_working_area(target, code_workarea);

	if (retval != ERROR_OK)
		return retval;
	return retval2;
}

static int mb9bf_probe(struct flash_bank *bank)
{
	struct fm4_flash_bank *fm4_bank = bank->driver_priv;
	uint32_t flash_addr = bank->base;

	switch (fm4_bank->variant) {
	case mb9bfx64:
		bank->num_sectors = 8;
		break;
	case mb9bfx65:
		bank->num_sectors = 10;
		break;
	case mb9bfx66:
		bank->num_sectors = 12;
		break;
	case mb9bfx67:
		bank->num_sectors = 16;
		break;
	case mb9bfx68:
		bank->num_sectors = 20;
		break;
	default:
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	LOG_DEBUG("%u sectors", bank->num_sectors);
	bank->sectors = calloc(bank->num_sectors,
				sizeof(struct flash_sector));
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (i < 4)
			bank->sectors[i].size = 8 * 1024;
		else if (i == 4)
			bank->sectors[i].size = 32 * 1024;
		else
			bank->sectors[i].size = 64 * 1024;
		bank->sectors[i].offset = flash_addr - bank->base;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;

		bank->size += bank->sectors[i].size;
		flash_addr += bank->sectors[i].size;
	}

	return ERROR_OK;
}

static void s6e2cc_init_sector(struct flash_sector *sector, int sa)
{
	if (sa < 8)
		sector->size = 8 * 1024;
	else if (sa == 8)
		sector->size = 32 * 1024;
	else
		sector->size = 64 * 1024;

	sector->is_erased = -1;
	sector->is_protected = -1;
}

static int s6e2cc_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct fm4_flash_bank *fm4_bank = bank->driver_priv;
	uint32_t u32_value;
	uint32_t flash_addr = bank->base;
	int retval;
	unsigned int i, num_extra_sectors, num_sectors;

	retval = target_read_u32(target, DFCTRLR, &u32_value);
	if (retval != ERROR_OK)
		return retval;
	if (u32_value & DFCTRLR_DFE) {
		LOG_WARNING("Dual Flash mode is not implemented.");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	switch (fm4_bank->variant) {
	case s6e2cx8:
		num_sectors = (fm4_bank->macro_nr == 0) ? 20 : 0;
		break;
	case s6e2cx9:
		num_sectors = (fm4_bank->macro_nr == 0) ? 20 : 12;
		break;
	case s6e2cxa:
		num_sectors = 20;
		break;
	default:
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}
	num_extra_sectors = (fm4_bank->macro_nr == 0) ? 1 : 4;
	bank->num_sectors = num_sectors + num_extra_sectors;

	LOG_DEBUG("%u sectors", bank->num_sectors);
	bank->sectors = calloc(bank->num_sectors,
				sizeof(struct flash_sector));
	for (i = 0; i < num_sectors; i++) {
		int sa = 4 + i;
		bank->sectors[i].offset = flash_addr - bank->base;
		s6e2cc_init_sector(&bank->sectors[i], sa);

		bank->size += bank->sectors[i].size;
		flash_addr += bank->sectors[i].size;
	}

	flash_addr = (fm4_bank->macro_nr == 0) ? 0x00406000 : 0x00408000;
	for (; i < bank->num_sectors; i++) {
		int sa = 4 - num_extra_sectors + (i - num_sectors);
		bank->sectors[i].offset = flash_addr - bank->base;
		s6e2cc_init_sector(&bank->sectors[i], sa);

		/*
		 * Don't increase bank->size for these sectors
		 * to avoid an overlap between Flash Macros #0 and #1.
		 */
		flash_addr += bank->sectors[i].size;
	}

	return ERROR_OK;
}

static int s6e2dh_probe(struct flash_bank *bank)
{
	uint32_t flash_addr = bank->base;

	bank->num_sectors = 10;
	bank->sectors = calloc(bank->num_sectors,
				sizeof(struct flash_sector));
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (i < 4)
			bank->sectors[i].size = 8 * 1024;
		else if (i == 4)
			bank->sectors[i].size = 32 * 1024;
		else
			bank->sectors[i].size = 64 * 1024;
		bank->sectors[i].offset = flash_addr - bank->base;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;

		bank->size += bank->sectors[i].size;
		flash_addr += bank->sectors[i].size;
	}

	return ERROR_OK;
}

static int fm4_probe(struct flash_bank *bank)
{
	struct fm4_flash_bank *fm4_bank = bank->driver_priv;
	int retval;

	if (fm4_bank->probed)
		return ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	switch (fm4_bank->variant) {
	case mb9bfx64:
	case mb9bfx65:
	case mb9bfx66:
	case mb9bfx67:
	case mb9bfx68:
		retval = mb9bf_probe(bank);
		break;
	case s6e2cx8:
	case s6e2cx9:
	case s6e2cxa:
		retval = s6e2cc_probe(bank);
		break;
	case s6e2dx:
		retval = s6e2dh_probe(bank);
		break;
	default:
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}
	if (retval != ERROR_OK)
		return retval;

	fm4_bank->probed = true;

	return ERROR_OK;
}

static int fm4_auto_probe(struct flash_bank *bank)
{
	struct fm4_flash_bank *fm4_bank = bank->driver_priv;

	if (fm4_bank->probed)
		return ERROR_OK;

	return fm4_probe(bank);
}

static int fm4_get_info_command(struct flash_bank *bank, char *buf, int buf_size)
{
	struct fm4_flash_bank *fm4_bank = bank->driver_priv;
	const char *name;

	if (bank->target->state != TARGET_HALTED) {
		LOG_WARNING("Cannot communicate... target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	switch (fm4_bank->variant) {
	case mb9bfx64:
		name = "MB9BFx64";
		break;
	case mb9bfx65:
		name = "MB9BFx65";
		break;
	case mb9bfx66:
		name = "MB9BFx66";
		break;
	case mb9bfx67:
		name = "MB9BFx67";
		break;
	case mb9bfx68:
		name = "MB9BFx68";
		break;
	case s6e2cx8:
		name = "S6E2Cx8";
		break;
	case s6e2cx9:
		name = "S6E2Cx9";
		break;
	case s6e2cxa:
		name = "S6E2CxA";
		break;
	case s6e2dx:
		name = "S6E2Dx";
		break;
	default:
		name = "unknown";
		break;
	}

	switch (fm4_bank->variant) {
	case s6e2cx8:
	case s6e2cx9:
	case s6e2cxa:
		snprintf(buf, buf_size, "%s MainFlash Macro #%i",
			name, fm4_bank->macro_nr);
		break;
	default:
		snprintf(buf, buf_size, "%s MainFlash", name);
		break;
	}

	return ERROR_OK;
}

static bool fm4_name_match(const char *s, const char *pattern)
{
	int i = 0;

	while (s[i]) {
		/* If the match string is shorter, ignore excess */
		if (!pattern[i])
			return true;
		/* Use x as wildcard */
		if (pattern[i] != 'x' && tolower(s[i]) != tolower(pattern[i]))
			return false;
		i++;
	}
	return true;
}

static int mb9bf_bank_setup(struct flash_bank *bank, const char *variant)
{
	struct fm4_flash_bank *fm4_bank = bank->driver_priv;

	if (fm4_name_match(variant, "MB9BFx64")) {
		fm4_bank->variant = mb9bfx64;
	} else if (fm4_name_match(variant, "MB9BFx65")) {
		fm4_bank->variant = mb9bfx65;
	} else if (fm4_name_match(variant, "MB9BFx66")) {
		fm4_bank->variant = mb9bfx66;
	} else if (fm4_name_match(variant, "MB9BFx67")) {
		fm4_bank->variant = mb9bfx67;
	} else if (fm4_name_match(variant, "MB9BFx68")) {
		fm4_bank->variant = mb9bfx68;
	} else {
		LOG_WARNING("MB9BF variant %s not recognized.", variant);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	return ERROR_OK;
}

static int s6e2cc_bank_setup(struct flash_bank *bank, const char *variant)
{
	struct fm4_flash_bank *fm4_bank = bank->driver_priv;

	if (fm4_name_match(variant, "S6E2Cx8")) {
		fm4_bank->variant = s6e2cx8;
	} else if (fm4_name_match(variant, "S6E2Cx9")) {
		fm4_bank->variant = s6e2cx9;
	} else if (fm4_name_match(variant, "S6E2CxA")) {
		fm4_bank->variant = s6e2cxa;
	} else {
		LOG_WARNING("S6E2CC variant %s not recognized.", variant);
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(fm4_flash_bank_command)
{
	struct fm4_flash_bank *fm4_bank;
	const char *variant;
	int ret;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	variant = CMD_ARGV[6];

	fm4_bank = malloc(sizeof(struct fm4_flash_bank));
	if (!fm4_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	fm4_bank->probed = false;
	fm4_bank->macro_nr = (bank->base == 0x00000000) ? 0 : 1;

	bank->driver_priv = fm4_bank;

	if (fm4_name_match(variant, "MB9BF"))
		ret = mb9bf_bank_setup(bank, variant);
	else if (fm4_name_match(variant, "S6E2Cx"))
		ret = s6e2cc_bank_setup(bank, variant);
	else if (fm4_name_match(variant, "S6E2Dx")) {
		fm4_bank->variant = s6e2dx;
		ret = ERROR_OK;
	} else {
		LOG_WARNING("Family %s not recognized.", variant);
		ret = ERROR_FLASH_OPER_UNSUPPORTED;
	}
	if (ret != ERROR_OK)
		free(fm4_bank);
	return ret;
}

static const struct command_registration fm4_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration fm4_command_handlers[] = {
	{
		.name = "fm4",
		.mode = COMMAND_ANY,
		.help = "fm4 flash command group",
		.usage = "",
		.chain = fm4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver fm4_flash = {
	.name = "fm4",
	.commands = fm4_command_handlers,
	.flash_bank_command = fm4_flash_bank_command,
	.info = fm4_get_info_command,
	.probe = fm4_probe,
	.auto_probe = fm4_auto_probe,
	.read = default_flash_read,
	.erase = fm4_flash_erase,
	.erase_check = default_flash_blank_check,
	.write = fm4_flash_write,
	.free_driver_priv = default_flash_free_driver_priv,
};
