#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* NOTE THAT THIS CODE REQUIRES FLASH ROUTINES in BOOTROM WITH FUNCTION TABLE PTR AT 0x00000010
   Your gdbinit should load the bootrom.elf if appropriate */

/* this is 'M' 'u', 1 (version) */
#define BOOTROM_MAGIC 0x01754d
#define BOOTROM_MAGIC_ADDR 0x00000010

/* Call a ROM function via the debug trampoline
   Up to four arguments passed in r0...r3 as per ABI
   Function address is passed in r7
   FIXME the trampoline is needed because OpenOCD "algorithm" code insists on sw breakpoints. */

char *regnames[4] = {
	"r0", "r1", "r2", "r3"
}; /* FIXME pretty sure this is defined elsewhere too */

#define MAKE_TAG(a, b) (((b)<<8) | a)
#define FUNC_FLASH_EXIT_XIP         MAKE_TAG('E', 'X')
#define FUNC_DEBUG_TRAMPOLINE       MAKE_TAG('D', 'T')
#define FUNC_DEBUG_TRAMPOLINE_END   MAKE_TAG('D', 'E')
#define FUNC_CONNECT_INTERNAL_FLASH MAKE_TAG('I', 'F')
#define FUNC_FLASH_RANGE_ERASE      MAKE_TAG('R', 'E')
#define FUNC_FLASH_RANGE_PROGRAM    MAKE_TAG('R', 'P')
#define FUNC_FLASH_FLUSH_CACHE      MAKE_TAG('F', 'C')
#define FUNC_FLASH_ENTER_CMD_XIP    MAKE_TAG('C', 'X')

static uint32_t rp2040_lookup_symbol(struct target *target, uint32_t tag, uint16_t *symbol)
{
	uint32_t magic;
	int err = target_read_u32(target, BOOTROM_MAGIC_ADDR, &magic);
	if (err != ERROR_OK)
		return err;

	magic &= 0xffffff; /* ignore bootrom version */
	if (magic != BOOTROM_MAGIC) {
		if (!((magic ^ BOOTROM_MAGIC)&0xffff))
			LOG_ERROR("Incorrect RP2040 BOOT ROM version");
		else
			LOG_ERROR("RP2040 BOOT ROM not found");
		return ERROR_FAIL;
	}

	/* dereference the table pointer */
	uint16_t table_entry;
	err = target_read_u16(target, BOOTROM_MAGIC_ADDR + 4, &table_entry);
	if (err != ERROR_OK)
		return err;

	uint16_t entry_tag;
	do {
		err = target_read_u16(target, table_entry, &entry_tag);
		if (err != ERROR_OK)
			return err;
		if (entry_tag == tag) {
			/* 16 bit symbol is next */
			return target_read_u16(target, table_entry + 2, symbol);
		}
		table_entry += 4;
	} while (entry_tag);
	return ERROR_FAIL;
}

static int rp2040_call_rom_func(struct target *target, target_addr_t stack, uint32_t func_tag, uint32_t argdata[],
		int n_args)
{
	const int max_args = 4; /* only allow register arguments */
	if (n_args > max_args) {
		LOG_ERROR("Max of 4 arguments permitted when calling RP2040 ROM functions.");
		return ERROR_FAIL;
	}
	uint16_t debug_trampoline;
	int err = rp2040_lookup_symbol(target, FUNC_DEBUG_TRAMPOLINE, &debug_trampoline);
	if (err != ERROR_OK) {
		LOG_ERROR("Debug trampoline not found in RP2040 ROM.");
		return err;
	}
	debug_trampoline &= ~1u; /* mask off thumb bit */

	uint16_t debug_trampoline_end;
	err = rp2040_lookup_symbol(target, FUNC_DEBUG_TRAMPOLINE_END, &debug_trampoline_end);
	if (err != ERROR_OK) {
		LOG_ERROR("Debug trampoline end not found in RP2040 ROM.");
		return err;
	}
	debug_trampoline_end &= ~1u; /* mask off thumb bit */

	LOG_DEBUG("Calling ROM func %c%c with %d arguments", (char)func_tag, (char)(func_tag>>8), n_args);
	LOG_DEBUG("Calling on core \"%s\"", target->cmd_name);

	uint16_t func;
	err = rp2040_lookup_symbol(target, func_tag, &func);
	if (err != ERROR_OK) {
		LOG_ERROR("Function %c%c not found in RP2040 ROM.", (char)func_tag, (char)(func_tag>>8));
		return err;
	}

	struct reg_param args[max_args + 2];
	struct armv7m_algorithm alg_info;

	for (int i = 0; i < n_args; ++i) {
		init_reg_param(&args[i], regnames[i], 32, PARAM_OUT);
		buf_set_u32(args[i].value, 0, 32, argdata[i]);
	}
	/* Pass function pointer in r7 */
	init_reg_param(&args[n_args], "r7", 32, PARAM_OUT);
	buf_set_u32(args[n_args].value, 0, 32, func);
	init_reg_param(&args[n_args + 1], "sp", 32, PARAM_OUT);
	buf_set_u32(args[n_args + 1].value, 0, 32, stack);


	for (int i = 0; i < n_args + 2; ++i)
		LOG_DEBUG("Set %s = %08x", args[i].reg_name, buf_get_u32(args[i].value, 0, 32));

	/* Actually call the function */
	alg_info.common_magic = ARMV7M_COMMON_MAGIC;
	alg_info.core_mode = ARM_MODE_THREAD;
	err = target_run_algorithm(
		target,
		0, NULL,          /* No memory arguments */
		n_args + 1, args, /* User arguments + r7 */
		debug_trampoline, debug_trampoline_end,
		3000, /* 3s timeout */
		&alg_info
	);
	for (int i = 0; i < n_args + 1; ++i)
		destroy_reg_param(&args[i]);
	if (err != ERROR_OK)
		LOG_ERROR("Failed to invoke ROM function %c%c (@%08x)\n", (char)func_tag, (char)(func_tag>>8), func);
	return err;

}

/* -----------------------------------------------------------------------------
   Flash access code

   FIXME: support other flash geometries; where to get these consts from?
   These are common values, and are accurate for W25X10CL, W25Q16JV and friends */
#define BLOCK_SIZE (1ul << 16)
#define BLOCK_ERASE_CMD 0xd8
#define SECTOR_SIZE 4096
#define PAGE_SIZE 256

struct rp2040_flash_bank {
	int probed;
	/* Infrastructure for calling into ROM code: */
	struct working_area *stack;
	target_addr_t stacktop;
};

static int rp2040_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	LOG_INFO("Writing %d bytes starting at 0x%x", count, offset);

	/* todo fixme hard coded chunk size */
	struct rp2040_flash_bank *priv = bank->driver_priv;
	const unsigned int chunk_size = 16 * 1024;
	struct target *target = bank->target;
	struct working_area *bounce;
	int err = ERROR_OK;

	if (offset % PAGE_SIZE) {
		LOG_ERROR("RP2040 flash writes must be page-aligned (%d bytes). Can't continue", PAGE_SIZE);
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	if (target_alloc_working_area(target, chunk_size, &bounce) != ERROR_OK) {
		LOG_ERROR("Could not allocate bounce buffer for flash programming. Can't continue");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	LOG_DEBUG("Allocated flash bounce buffer @%08x", (unsigned)bounce->address);

	while (count > 0) {
		uint32_t write_size = count > chunk_size ? chunk_size : count;
		LOG_DEBUG("Writing %d bytes to offset %08x", write_size, offset);
		err = target_write_buffer(target, bounce->address, write_size, buffer);
		if (err != ERROR_OK) {
			LOG_ERROR("Could not load data into target bounce buffer");
			break;
		}
		uint32_t args[3] = {
			offset,
			bounce->address,
			write_size
		};
		err = rp2040_call_rom_func(target, priv->stacktop, FUNC_FLASH_RANGE_PROGRAM, args, 3);
		if (err != ERROR_OK) {
			LOG_ERROR("Failed to invoke flash programming code on target");
			break;
		}

		buffer += write_size;
		offset += write_size;
		count -= write_size;
	}
	target_free_working_area(target, bounce);

	if (err != ERROR_OK)
		return err;

	/* Flash is successfully programmed. We can now do a bit of poking to make the flash
	   contents visible to us via memory-mapped (XIP) interface in the 0x1... memory region */
	LOG_DEBUG("Flushing flash cache after write behind");
	err = rp2040_call_rom_func(bank->target, priv->stacktop, FUNC_FLASH_FLUSH_CACHE, NULL, 0);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2040 write: failed to flush flash cache");
		return err;
	}
	LOG_DEBUG("Configuring SSI for execute-in-place");
	err = rp2040_call_rom_func(bank->target, priv->stacktop, FUNC_FLASH_ENTER_CMD_XIP, NULL, 0);
	if (err != ERROR_OK)
		LOG_ERROR("RP2040 write: failed to flush flash cache");
	return err;
}

static int rp2040_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	uint32_t start_addr = bank->sectors[first].offset;
	uint32_t length = bank->sectors[last].offset + bank->sectors[last].size - start_addr;
	LOG_DEBUG("RP2040 erase %d bytes starting at 0x%08x", length, start_addr);

	/* This is a good place to do this because flash erase always follows a reset init */
	LOG_DEBUG("Connecting internal flash");
	int err = rp2040_call_rom_func(bank->target, priv->stacktop, FUNC_CONNECT_INTERNAL_FLASH, NULL, 0);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2040 erase: failed to connect internal flash");
		return err;
	}

	LOG_DEBUG("Kicking flash out of XIP mode");
	err = rp2040_call_rom_func(bank->target, priv->stacktop, FUNC_FLASH_EXIT_XIP, NULL, 0);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2040 erase: failed to exit flash XIP mode");
		return err;
	}


	LOG_DEBUG("Remote call flash_range_erase");

	/* Erase in naturally-aligned chunks of n sectors per call. Get speed of
	   block erases, without timeout on large erase ranges: */
	const unsigned int erase_sectors_per_call = 4 * BLOCK_SIZE / SECTOR_SIZE;
	unsigned int first_of_call, last_of_call;
	for (first_of_call = first; first_of_call <= last; first_of_call = last_of_call + 1) {
		/* Try to keep our erase calls block-aligned, to avoid degeneration to
		   sector erase at start and end of each call (RP2040ch slower) */
		last_of_call = first_of_call + erase_sectors_per_call;
		last_of_call -= (last_of_call % erase_sectors_per_call) + 1;
		if (last_of_call > last)
			last_of_call = last;
		uint32_t args[4] = {
			bank->sectors[first_of_call].offset,
			bank->sectors[last_of_call].offset + bank->sectors[last_of_call].size - bank->sectors[first_of_call].offset,
			BLOCK_SIZE,
			BLOCK_ERASE_CMD
		};
		err = rp2040_call_rom_func(bank->target, priv->stacktop, FUNC_FLASH_RANGE_ERASE, args, 4);
		if (err != ERROR_OK) {
			LOG_ERROR("RP2040 erase: flash_range_erase failed");
			break;
		}
	}
	return err;
}

static int rp2040_flash_protect_check(struct flash_bank *bank)
{
	LOG_WARNING("RP2040 Flash Protect Check (ignored)");
	return ERROR_OK;
}

static int rp2040_flash_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	LOG_WARNING("RP2040 Flash Protect (ignored)");
	return ERROR_OK;
}

/* -----------------------------------------------------------------------------
   Driver probing etc */

static int rp2040_flash_probe(struct flash_bank *bank)
{
	struct rp2040_flash_bank *flash_info = bank->driver_priv;

	bank->num_sectors = bank->size / SECTOR_SIZE;
	LOG_INFO("RP2040 B0 Flash Probe: %d bytes @%08x, in %d sectors\n",
		bank->size, (uint32_t)bank->base, bank->num_sectors);
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * SECTOR_SIZE;
		bank->sectors[i].size = SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	/* target_alloc_working_area always allocates RP2040ltiple of 4 bytes, so no worry about alignment */
	const int STACK_SIZE = 256;
	if (target_alloc_working_area(bank->target, STACK_SIZE, &flash_info->stack) != ERROR_OK) {
		LOG_ERROR("Could not allocate stack for flash programming code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	flash_info->stacktop = flash_info->stack->address + flash_info->stack->size;
	LOG_DEBUG("Allocated flash algorithm stack @%08x size %d bytes",
	    (uint32_t)flash_info->stack->address,
		flash_info->stack->size);

	flash_info->probed = 1;

	return ERROR_OK;
}

static int rp2040_flash_auto_probe(struct flash_bank *bank)
{
	struct rp2040_flash_bank *flash_info = bank->driver_priv;

	if (flash_info->probed)
		return ERROR_OK;

	return rp2040_flash_probe(bank);
}

static int rp2040_flash_info(struct flash_bank *bank, char *buf, int buf_size)
{
	LOG_INFO("RP2040 Flash Info");
	return ERROR_OK;
}

/* -----------------------------------------------------------------------------
   Driver boilerplate */

FLASH_BANK_COMMAND_HANDLER(rp2040_flash_bank_command)
{
	LOG_INFO("RP2040 Flash Bank Command");
	struct rp2040_flash_bank *flash_info;

	flash_info = malloc(sizeof(struct rp2040_flash_bank));
	flash_info->probed = 0;

	/* Set up bank info */
	bank->driver_priv = flash_info;

	return ERROR_OK;
}

struct flash_driver rp2040_flash = {
	.name = "rp2040_flash",
	.commands = NULL,
	.flash_bank_command = rp2040_flash_bank_command,
	.erase =  rp2040_flash_erase,
	.protect = rp2040_flash_protect,
	.write = rp2040_flash_write,
	.read = default_flash_read,
	.probe = rp2040_flash_probe,
	.auto_probe = rp2040_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = rp2040_flash_protect_check,
	.info = rp2040_flash_info,
	.free_driver_priv = default_flash_free_driver_priv /* FIXME free working areas etc */
};
