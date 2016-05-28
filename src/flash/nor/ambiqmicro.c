/******************************************************************************
 *
 * @file ambiqmicro.c
 *
 * @brief Ambiq Micro flash driver.
 *
 *****************************************************************************/

/******************************************************************************
 * Copyright (C) 2015, David Racine <dracine at ambiqmicro.com>
 *
 * Copyright (C) 2016, Rick Foos <rfoos at solengtech.com>
 *
 * Copyright (C) 2015-2016, Ambiq Micro, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "imp.h"
#include "target/algorithm.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"
#include "target/register.h"

/* RSTGEN - MCU Reset Generator 0x40000000 */

#define RSTGEN_CFG      (0x40000000)
#define RSTGEN_POI      (0x40000004)
#define RSTGEN_POIKEY   (0x0000001B)
#define RSTGEN_POR      (0x40000008)
#define RSTGEN_PORKEY   (0x000000D4)
#define RSTGEN_STAT     (0x4000000C)
#define RSTGEN_POISTAT  (0x00000010)
#define RSTGEN_SWRSTAT  (0x00000008)	/* POR or AICR Reset occurred. */
#define RSTGEN_PORSTAT  (0x00000002)	/* POR Reset occurred. */
#define RSTGEN_CLRSTAT  (0x40000010)
#define RSTGEN_CLRKEY   (0x00000001)

/* Address and Key defines. */

#define PROGRAM_KEY      (0x12344321)
#define OTP_PROGRAM_KEY  (0x87655678)

/** Key to program info0 flash. */
#define CUSTOMER_PROGRAM_KEY    (0x87655678)
#define INFO0_ERASE_KEY         (0x56295141)
#define BRICK_KEY               (0xA35C9B6D)


/** Bootloader visible at 0x00000000 (0x1). */
#define REG_CONTROL_BOOTLOADERLOW   (0x400201a0)
/** Part number (class) and sram/flash size. */
#define REG_CONTROL_CHIPPN          (0x40020000)
/** PDM. Readable on Apollo2 only, fails on Apollo. */
#define REG_APOLLO2_ID              (0x50011000)
/** Hardware debug register. */
#define REG_DEBUG_AMBIQ             (0xF0000FE0)
#define REG_DEBUG_APOLLO_MASK       (0x000000F0)
#define REG_DEBUG_APOLLO            (0x000000E0)
#define REG_DEBUG_APOLLO2           (0x000000D0)

/** Apollo info0 base address. */
#define APOLLO_INFO0_BASE_ADDRESS   (0x50020400)
#define APOLLO_INFO0_WRITE_PROTECT0 (0x50020404)
#define APOLLO_INFO0_COPY_PROTECT0  (0x50020408)
/** Apollo2 info0 base address. */
#define APOLLO2_INFO0_BASE_ADDRESS   (0x50020000)
#define APOLLO2_INFO0_WRITE_PROTECT0 (0x50020020)
#define APOLLO2_INFO0_COPY_PROTECT0  (0x50020030)

/** Breakpoint for Bootloader to start, and location for return code. */
#define BREAKPOINT                  (0xfffffffe)

/* Bootloader Commands. */

/** @note The following commands are the same in Apollo and Apollo2. */

/** Bootloader program main flash, parameters in SRAM. */
#define FLASH_PROGRAM_MAIN_FROM_SRAM                (0x0800005d)
/** Apollo only (no instance parameter) */
#define FLASH_PROGRAM_OTP_FROM_SRAM                 (0x08000061)
/** Apollo2 only. */
#define FLASH_PROGRAM_INFO_FROM_SRAM                (0x08000061)
/** Both. */
#define FLASH_ERASE_MAIN_PAGES_FROM_SRAM            (0x08000065)
#define FLASH_MASS_ERASE_FROM_SRAM                  (0x08000069)

/** @note Apollo2 only commands. */

#define APOLLO2_FLASH_INFO_ERASE_FROM_SRAM           (0x08000085)
#define APOLLO2_FLASH_INFO_PLUS_MAIN_ERASE_FROM_SRAM (0x0800008D)
#define APOLLO2_FLASH_RECOVERY_FROM_SRAM             (0x08000099)


/** Apollo: Info space size. */
#define APOLLO_INFO_SPACE_SIZE      (256)
/** Apollo2: Info space size. */
#define APOLLO2_INFO_SPACE_SIZE     (2048)

/** Apollo Bootloader Write Buffer Start. */
#define APOLLO_WRITE_BUFFER_START    (0x10000010)
/** Apollo2 Bootloader Write Buffer Start. */
#define APOLLO2_WRITE_BUFFER_START   (0x10001000)
/** Apollo Bootloader Write Buffer Size. Max size 6k. */
#define APOLLO_WRITE_BUFFER_SIZE     (0x00001800)
/** Apollo2 Bootloader Write Buffer Size. */
#define APOLLO2_WRITE_BUFFER_SIZE     (0x00001800)


/** Check for error, then log the error. */
#define CHECK_STATUS(rc, msg) {	\
		if (rc != ERROR_OK) \
			LOG_ERROR("status(%d):%s\n", rc, msg); }

/** Bootloader SRAM parameter block start. */
#define SRAM_PARAM_START    ((uint32_t *)0x10000000ul)
/** Autoincrementing Sram pointer, increment determined by pSram type. */
#define pSRAM               ((uintptr_t)pSram++)

/** Bootloader commands, sizes, and addresses for current processor. */
typedef struct _bootldr bootldr;
struct _bootldr {
	uint32_t write_buffer_start;
	uint32_t write_buffer_size;
	uint32_t info_space_size;
	uint32_t info0_flash_write_protect;
	/* Apollo and Apollo2 the same */
	uint32_t flash_program_main_from_sram;
	uint32_t flash_program_info_from_sram;
	uint32_t flash_erase_main_pages_from_sram;
	uint32_t flash_mass_erase_from_sram;
	/* Apollo only command. */
	uint32_t flash_program_otp_from_sram;
	/* Apollo2 only commands. */
	uint32_t flash_info_erase_from_sram;
	uint32_t flash_info_plus_main_erase_from_sram;
	uint32_t flash_recovery_from_sram;
};

/** Apollo2 bootloader commands, sizes, and addresses. */
static const bootldr apollo2_bootldr = {
	APOLLO2_WRITE_BUFFER_START,	/* write_buffer_start */
	APOLLO2_WRITE_BUFFER_SIZE,	/* write_buffer_size */
	APOLLO2_INFO_SPACE_SIZE,	/* info_space_size */
	APOLLO2_INFO0_WRITE_PROTECT0,	/* info0_flash_write_protect */
	FLASH_PROGRAM_MAIN_FROM_SRAM,	/* flash_program_main_from_sram */
	FLASH_PROGRAM_INFO_FROM_SRAM,	/* flash_program_info_from_sram */
	FLASH_ERASE_MAIN_PAGES_FROM_SRAM,	/* flash_erase_main_pages_from_sram */
	FLASH_MASS_ERASE_FROM_SRAM,	/* flash_mass_erase_from_sram */
	/* Apollo only command. */
	0,				/* flash_program_otp_from_sram */
	/* Apollo2 only commands. */
	APOLLO2_FLASH_INFO_ERASE_FROM_SRAM,	/* flash_info_erase_from_sram */
	APOLLO2_FLASH_INFO_PLUS_MAIN_ERASE_FROM_SRAM,	/* flash_info_plus_main_erase_from_sram */
	APOLLO2_FLASH_RECOVERY_FROM_SRAM,	/* flash_recovery_from_sram */
};

/** Apollo bootloader commands, sizes, and addresses. */
static const bootldr apollo_bootldr = {
	APOLLO_WRITE_BUFFER_START,	/* write_buffer_start */
	APOLLO_WRITE_BUFFER_SIZE,	/* write_buffer_size */
	APOLLO_INFO_SPACE_SIZE,		/* info_space_size */
	APOLLO_INFO0_WRITE_PROTECT0,	/* info0_flash_write_protect */
	FLASH_PROGRAM_MAIN_FROM_SRAM,	/* flash_program_main_from_sram */
	0,				/* flash_program_info_from_sram */
	FLASH_ERASE_MAIN_PAGES_FROM_SRAM,	/* flash_erase_main_pages_from_sram */
	FLASH_MASS_ERASE_FROM_SRAM,	/* flash_mass_erase_from_sram */
	/* Apollo only command. */
	FLASH_PROGRAM_OTP_FROM_SRAM,	/* flash_program_otp_from_sram */
	/* Apollo2 only commands. */
	0,	/* flash_info_erase_from_sram */
	0,	/* flash_info_plus_main_erase_from_sram */
	0,	/* flash_recovery_from_sram */
};

/** Flash size from PartNum. */
static const uint32_t apollo_flash_size[] = {
	1 << 15,	/* 0x008000   32k */
	1 << 16,	/* 0x010000   64k */
	1 << 17,	/* 0x020000  128k */
	1 << 18,	/* 0x040000  256k */
	1 << 19,	/* 0x080000  512k */
	1 << 20,	/* 0x100000 1024k */
	1 << 21		/* 0x200000 2048k */
};

/** Sram size from PartNum. */
static const uint32_t apollo_sram_size[] = {
	1 << 15,
	1 << 16,
	1 << 17,
	1 << 18,
	1 << 19,
	1 << 20,
	1 << 21
};

struct ambiqmicro_flash_bank {
	/* chip id register */

	uint32_t probed;

	const char *target_name;
	uint8_t target_class;
	uint8_t target_revision;
	uint8_t target_package;
	uint8_t target_qual;

	uint32_t sramsiz;
	uint32_t flshsiz;

	/* flash geometry */

	uint32_t num_pages;
	uint32_t pagesize;
	uint32_t pages_in_lockregion;

	/* nonvolatile memory bits */

	uint16_t num_lockbits;

	/* bootloader commands, addresses, and sizes. */

	const bootldr *bootloader;
};

/** Class names to Part names. */
static struct {
	uint8_t class;
	uint8_t partno;
	const char *partname;
} ambiqmicroParts[] = {
	{0x00, 0x00, "Unknown"},
	{0x01, 0x00, "Apollo"},
	{0x02, 0x00, "Reserved"},
	{0x03, 0x00, "Apollo2"},
	{0x04, 0x00, "Apollo3"},
	{0x05, 0x00, "ApolloSBL"},
};

/** Chip names used by flash info command. */
static char *ambiqmicroClassname[] = {
	"Unknown",
	"Apollo",
	"Reserved",
	"Apollo2",
	"Apollo3",
	"ApolloSBL"
};

/** Package names used by flash info command. */
static char *ambiqmicroPackagename[] = {
	"SIP",
	"QFN",
	"BGA",
	"CSP"
};

/***************************************************************************
*	openocd command interface
***************************************************************************/

/** flash_bank ambiqmicro <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(ambiqmicro_flash_bank_command)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ambiqmicro_info = calloc(sizeof(struct ambiqmicro_flash_bank), 1);

	bank->driver_priv = ambiqmicro_info;

	ambiqmicro_info->target_name = "Unknown target";

	/* part wasn't probed yet */
	ambiqmicro_info->probed = 0;

	return ERROR_OK;
}

/** Display last line of flash info command. */
static int get_ambiqmicro_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int printed;
	char *classname;

	if (ambiqmicro_info->probed == 0) {
		LOG_ERROR("Target not probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Check class name in range. */
	if (ambiqmicro_info->target_class < ARRAY_SIZE(ambiqmicroClassname))
		classname = ambiqmicroClassname[ambiqmicro_info->target_class];
	else
		classname = ambiqmicroClassname[0];

	printed = snprintf(buf,
		buf_size,
		"\nAmbiq Micro controller "
		"class %i (%s) %s%s Rev %i.%i (%s)\n",
		ambiqmicro_info->target_class,
		classname,
		ambiqmicro_info->target_name,
		ambiqmicroPackagename[ambiqmicro_info->target_package],
		ambiqmicro_info->target_revision >> 4,
		ambiqmicro_info->target_revision & 0xF,
		(ambiqmicro_info->target_qual) ? "Qualified" : "Prototype");

	if ((printed < 0))
		return ERROR_BUF_TOO_SMALL;
	return ERROR_OK;
}

/***************************************************************************
*	chip identification and status
***************************************************************************/

/** Initial Apollo class. Others may follow. */
#define APOLLO_BASE_CLASS   1
/** Initial Apollo2 class. Others may follow. */
#define APOLLO2_BASE_CLASS  3
/** Start of SRAM */
#define SRAMSTART   (0x10000000)
/** Start of SRAM */
#define FLASHSTART   (0x00000000)

/** Default sram size 256k if all checks fail. */
#define DEFAULT_SRAM_SIZE 3
/** Default flash size 128k if all checks fail. */
#define DEFAULT_FLASH_SIZE 2
/** Default revision 0.1. */
#define DEFAULT_PARTNUM_REVISION 1
/** Default package BGA. */
#define DEFAULT_PARTNUM_PACKAGE  2

/** Determine flash size from hardware check. */
static int get_flash_size(struct flash_bank *bank)
{
	int i, retval;
	uint32_t flash;

	/* Chip has no Flash. */
	retval = target_read_u32(bank->target, FLASHSTART, &flash);
	if (retval != ERROR_OK) {
		LOG_ERROR("FLASH not found.");
		return ERROR_FAIL;
	}

	/* Read flash size we are testing. */
	for (i = 0; i < (int)ARRAY_SIZE(apollo_flash_size); i++) {
		retval = target_read_u32(bank->target, FLASHSTART+apollo_flash_size[i], &flash);
		if (retval != ERROR_OK)
			break;
	}
	LOG_INFO("Hardware FLASH size: %d KB. (ignore 'Failed to read memory' error.)",
		apollo_flash_size[i]/1024);
	return i;
}

/** Determine sram size from hardware check. */
static int get_sram_size(struct flash_bank *bank)
{
	int i, retval;
	uint32_t sram;

	/* Chip has no SRAM. */
	retval = target_read_u32(bank->target, SRAMSTART, &sram);
	if (retval != ERROR_OK) {
		LOG_ERROR("SRAM not found.");
		return ERROR_FAIL;
	}

	/* Read 4 bytes of memory size we are testing. */
	for (i = 0; i < (int)ARRAY_SIZE(apollo_sram_size); i++) {
		retval = target_read_u32(bank->target, SRAMSTART+apollo_sram_size[i], &sram);
		if (retval != ERROR_OK)
			break;
	}
	LOG_INFO("Hardware SRAM size: %d KB. (ignore 'Failed to read memory' error.)",
		apollo_sram_size[i]/1024);
	return i;
}

/** Set apollo info unique to class. */
static int set_class_info(struct ambiqmicro_flash_bank *ambiqmicro_info, uint32_t class)
{
	int retval = ERROR_OK;
	switch (class) {
		case APOLLO_BASE_CLASS:		/* 1 - Apollo */
		case 5:					/* 5 - Apollo Secure Bootloader */
			ambiqmicro_info->target_class = class;
			ambiqmicro_info->pagesize = 2048;
			ambiqmicro_info->bootloader = &apollo_bootldr;
			break;

		case APOLLO2_BASE_CLASS:	/* 3 - Apollo2 */
			ambiqmicro_info->target_class = class;
			ambiqmicro_info->pagesize = 8192;
			ambiqmicro_info->bootloader = &apollo2_bootldr;
			break;

		default:
			LOG_ERROR("Unknown Class %d.", class);
			retval = ERROR_FAIL;
			break;
	}
	/*
	 * Target Name from target class.
	 */
	if ((ambiqmicro_info->target_class > 0) &&
		(ambiqmicro_info->target_class < ARRAY_SIZE(ambiqmicroParts)))
		ambiqmicro_info->target_name =
			ambiqmicroParts[ambiqmicro_info->target_class].partname;
	else
		ambiqmicro_info->target_name =
			ambiqmicroParts[0].partname;

	return retval;
}

/** Determine base class from hardware. Apollo2 if PDM block exists. */
static uint32_t get_base_class_pdm(struct flash_bank *bank)
{
	uint32_t class;
	int retval = target_read_u32(bank->target, REG_APOLLO2_ID, &class);
	LOG_DEBUG("pReg[0x%X] 0x%X", REG_APOLLO2_ID, class);
	if (retval == ERROR_OK)
		class = APOLLO2_BASE_CLASS;
	else {
		/* Cannot disable failure message from target_read_u32. */
		LOG_INFO(
			"Apollo detected. (ignore 'Failed to read memory' error).");
		class = APOLLO_BASE_CLASS;
	}
	return class;
}

/** Determine base class from hardware check of Ambiq Debug Register. */
static uint32_t get_base_class(struct flash_bank *bank)
{
	uint32_t class;
	int retval = target_read_u32(bank->target, REG_DEBUG_AMBIQ, &class);
	LOG_DEBUG("pReg[0x%X] 0x%X", REG_DEBUG_AMBIQ, class);
	if (retval != ERROR_OK) {
		LOG_ERROR("Ambiq Debug Register not found.");
		class = get_base_class_pdm(bank);
	} else if ((class & REG_DEBUG_APOLLO_MASK) == REG_DEBUG_APOLLO) {
		LOG_DEBUG("Apollo detected from Ambiq debug register.");
		class = APOLLO_BASE_CLASS;
	} else if ((class & REG_DEBUG_APOLLO_MASK) == REG_DEBUG_APOLLO2) {
		LOG_DEBUG("Apollo2 detected from Ambiq debug register.");
		class = APOLLO2_BASE_CLASS;
	} else {
		class = APOLLO2_BASE_CLASS;
		LOG_ERROR("Ambiq Debug Register invalid.");
		class = get_base_class_pdm(bank);
	}
	return class;
}

/** Construct a valid PartNum from hardware tests. */
static uint32_t default_partnum(struct flash_bank *bank)
{
	uint32_t class = get_base_class(bank);
	int sramsize = get_sram_size(bank);
	int flashsize = get_flash_size(bank);

	if (sramsize == ERROR_FAIL)
		sramsize = DEFAULT_SRAM_SIZE;
	if (flashsize == ERROR_FAIL)
		flashsize = DEFAULT_FLASH_SIZE;

	/* guess minimum config(fpga) 2 64k flash banks, 256k sram. */
	uint32_t PartNum = (
		(class << 24) |	/* Base Class */ \
		(flashsize << 20) |	/* Flash Size 128k */ \
		(sramsize << 16) |	/* SRAM Size calculated. */ \
		(DEFAULT_PARTNUM_REVISION << 8)  |	/* Revision 0.1 */ \
		(DEFAULT_PARTNUM_PACKAGE << 6)	/* BGA Package */ \
		);
	return PartNum;
}

/** Fill in driver info structure */
static int ambiqmicro_read_part_info(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t PartNum = 0, class, pnum_class;
	int retval, size, pnum_size;

	/*
	 * Read Part Number.
	 * <class><flashsize><sramsize><revision><package>
	 */
	retval = target_read_u32(target, REG_CONTROL_CHIPPN, &PartNum);
	LOG_DEBUG("pReg[0x%X] 0x%X", REG_CONTROL_CHIPPN, PartNum);
	if (retval != ERROR_OK) {
		LOG_ERROR("status(0x%x): Could not read PartNum.", retval);
		/* Set PartNum construct a default device */
		PartNum = 0;
	}


	class = get_base_class(bank);
	pnum_class = (PartNum & 0xFF000000) >> 24;
	switch (pnum_class) {
		case APOLLO_BASE_CLASS:		/* 1 - Apollo */
		case 5:				/* 5 - Apollo Secure Bootloader */
			if (class != APOLLO_BASE_CLASS) {
				LOG_WARNING("Apollo Part class did not match hardware.");
				PartNum = default_partnum(bank);
			}
			break;

		case APOLLO2_BASE_CLASS:	/* 3 - Apollo2 */
			if (class != APOLLO2_BASE_CLASS) {
				LOG_WARNING("Apollo2 Part class did not match hardware.");
				PartNum = default_partnum(bank);
			}
			break;

		default:	/* Calculate default part. */
			LOG_WARNING("Unknown Class %d. Determining default part number.",
			pnum_class);
			PartNum = default_partnum(bank);
			break;
	}

	/*
	 * Compare hardware sram size to PartNum sram size.
	 * PartNum SRAM size can be less than Hardware SRAM size.
	 */
	size = get_sram_size(bank);
	pnum_size = ((PartNum & 0x000F0000) >> 16);
	LOG_DEBUG("SRAM (Partnum %d > Hardware %d)", pnum_size, size);
	if (size == ERROR_FAIL) {
		if ((pnum_size < 0) || (pnum_size >= (int)ARRAY_SIZE(apollo_flash_size))) {
			LOG_ERROR("Invalid flash size, using default.");
			PartNum &= ~0x00F00000;
			PartNum |= (DEFAULT_SRAM_SIZE << 16);
		}
	} else if (pnum_size > size) {
		LOG_ERROR("Hardware Sram mismatch in PartNum, overriding PartNum.");
		PartNum &= ~0x000F0000;
		PartNum |= (size << 16);
	}

	/*
	 * Compare hardware flash size to PartNum flash size.
	 * PartNum flash size must equal Hardware flash size.
	 */
	size = get_flash_size(bank);
	pnum_size = ((PartNum & 0x00F00000) >> 20);
	LOG_DEBUG("Flash (Partnum %d != Hardware %d)", pnum_size, size);
	if (size == ERROR_FAIL) {
		if ((pnum_size < 0) || (pnum_size >= (int)ARRAY_SIZE(apollo_flash_size))) {
			LOG_ERROR("Invalid flash size, using default.");
			PartNum &= ~0x00F00000;
			PartNum |= (DEFAULT_FLASH_SIZE << 20);
		}
	} else if (pnum_size != size) {
		LOG_ERROR("Hardware Flash mismatch in PartNum, overriding PartNum.");
		PartNum &= ~0x00F00000;
		PartNum |= (size << 20);
	}

	/* At this point, we're sure the PartNum is good. */
	LOG_DEBUG("Part number: 0x%x", PartNum);

	/*
	 * Set info unique to the target class.
	 */
	set_class_info(ambiqmicro_info, ((PartNum & 0xFF000000) >> 24));

	/*
	 * Set revision, package, and qualified.
	 */
	ambiqmicro_info->target_revision = (PartNum & 0x0000FF00) >> 8;
	ambiqmicro_info->target_package = (PartNum & 0x000000C0) >> 6;
	ambiqmicro_info->target_qual = (PartNum & 0x00000001);

	/*
	 * All Apollo's have two flash banks/instances sequentially in memory.
	 * The Partnum flash size is always twice the bank size.
	 */
	ambiqmicro_info->flshsiz = apollo_flash_size[(PartNum & 0x00F00000) >> 20] >> 1;
	ambiqmicro_info->sramsiz = apollo_sram_size[(PartNum & 0x000F0000) >> 16];
	ambiqmicro_info->num_pages = ambiqmicro_info->flshsiz / ambiqmicro_info->pagesize;


	LOG_INFO("\nTarget name: %s, num_pages: %d, pagesize: %d, flash: %d KB, sram: %d KB",
		ambiqmicro_info->target_name,
		ambiqmicro_info->num_pages,
		ambiqmicro_info->pagesize,
		ambiqmicro_info->flshsiz/1024,
		ambiqmicro_info->sramsiz/1024);

	return ERROR_OK;
}

/***************************************************************************
*	flash operations
***************************************************************************/

/** Target must be halted and probed before bootloader commands are executed. */
static int target_ready_for_command(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	if (ambiqmicro_info->probed == 0) {
		LOG_ERROR("Target not probed.");
		retval = ERROR_FLASH_BANK_NOT_PROBED;
	} else if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted.");
		retval = ERROR_TARGET_NOT_HALTED;
	} else
		retval = ERROR_OK;

	return retval;
}

/** write the is_erased flag to sector map. */
static int write_is_erased(struct flash_bank *bank, int first, int last, int flag)
{
	if ((first > bank->num_sectors) || (last > bank->num_sectors))
		return ERROR_FAIL;

	for (int i = first; i < last; i++)
		bank->sectors[i].is_erased = flag;
	return ERROR_OK;
}

/** Clear sram parameter space.
Sram pointer is incremented+4 beyond the last write to sram.*/
static int clear_sram_parameters(struct target *target, uint32_t *pSram, uint32_t *pStart)
{
	if (pSram < pStart) {
		LOG_DEBUG("sram pointer %p less than start address %p",
			pSram, pStart);
		return 1;
	}
	while (pSram > pStart)
		target_write_u32(target, (uintptr_t)-- pSram, 0);
	return 0;
}

/** Load bootloader arguments into SRAM. */
static uint32_t *setup_sram(struct target *target, int width, uint32_t arr[width])
{
	uint32_t *pSramRetval = NULL, *pSram = SRAM_PARAM_START;
	int retval;

	for (int i = 0; i < width; i++) {
		LOG_DEBUG("pSram[0x%X] 0x%X", (uint32_t)(uintptr_t)pSram, arr[i]);
		if (arr[i] == BREAKPOINT)
			pSramRetval = pSram;
		retval = target_write_u32(target, pSRAM, arr[i]);
		CHECK_STATUS(retval, "error writing bootloader SRAM parameters.");
		if (retval != ERROR_OK) {
			pSramRetval = 0;
			break;
		}
	}

	LOG_DEBUG("pSram[pSramRetval] 0x%X", (uint32_t)(uintptr_t)pSramRetval);
	return pSramRetval;
}

/** Read flash status from bootloader. */
static int check_flash_status(struct target *target, uint32_t *address)
{
	uint32_t retflash;
	int rc;
	rc = target_read_u32(target, (uintptr_t)address, &retflash);
	/* target connection failed. */
	if (rc != ERROR_OK) {
		LOG_DEBUG("%s:%d:%s(): status(0x%x)\n",
			__FILE__, __LINE__, __func__, rc);
		return rc;
	}
	/* target flash failed, unknown cause. */
	if (retflash != 0) {
		LOG_ERROR("Flash not happy: status(0x%x)", retflash);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

/** Execute bootloader command with SRAM parameters. */
static int ambiqmicro_exec_command(struct target *target,
	uint32_t command,
	uint32_t *flash_return_address)
{
	int retval, retflash;

	LOG_DEBUG("pROM[Bootloader] 0x%X", command);

	/* Commands invalid for this chip will come across as 0. */
	if (!command) {
		LOG_ERROR("Invalid command for this target.");
		return ERROR_FAIL;
	}

	/* Call bootloader */
	retval = target_resume(
		target,
		false,
		command,
		true,
		true);

	CHECK_STATUS(retval, "error executing ambiqmicro command");

	/*
	 * Clear Sram Parameters before first return.
	 */
	clear_sram_parameters(target, flash_return_address, SRAM_PARAM_START);

	/*
	 * Wait for halt.
	 */
	for (;; ) {
		target_poll(target);
		if (target->state == TARGET_HALTED)
			break;
		else if (target->state == TARGET_RUNNING ||
			target->state == TARGET_DEBUG_RUNNING) {
			/*
			 * Keep polling until target halts.
			 */
			target_poll(target);
			alive_sleep(100);
			LOG_DEBUG("state = %d", target->state);
		} else {
			LOG_ERROR("Target not halted or running %d", target->state);
			break;
		}
	}

	/*
	 * Read return value, flash error takes precedence.
	 */
	retflash = check_flash_status(target, flash_return_address);
	if (retflash != ERROR_OK)
		retval = retflash;

	LOG_DEBUG("pSram[0x%X] 0x%X", (uint32_t)(uintptr_t)flash_return_address, retflash);

	/* Return code from target_resume OR flash. */
	return retval;
}

static int ambiqmicro_exec_sram_command(struct flash_bank *bank, uint32_t command,
	int width, uint32_t arr[])
{
	uint32_t *bootloader_return_address;
	int retval;

	/*
	 * Load SRAM parameters.
	 */
	bootloader_return_address = setup_sram(bank->target, width, arr);

	/*
	 * Execute Bootloader command.
	 */
	retval = ambiqmicro_exec_command(bank->target, command, bootloader_return_address);
	return retval;
}

static int ambiqmicro_exec_main_command(struct flash_bank *bank, uint32_t command,
	int width, uint32_t arr[])
{
	int retval, retval1;

	/*
	 * Clear Bootloader bit.
	 */
	retval = target_write_u32(bank->target, REG_CONTROL_BOOTLOADERLOW, 0x0);
	CHECK_STATUS(retval, "error clearing bootloader bit.");

	/*
	 * Execute the command.
	 */
	retval = ambiqmicro_exec_sram_command(bank, command, width, arr);

	/*
	 * Set Bootloader bit, regardless of command execution.
	 */
	retval1 = target_write_u32(bank->target, REG_CONTROL_BOOTLOADERLOW, 0x1);
	CHECK_STATUS(retval1, "error setting bootloader bit.");

	return retval;
}

/** Power On Internal (POI). */
static int ambiqmicro_poi(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	int retval;

	/*
	 * Clear Reset Status.
	 */
	retval = target_write_u32(target, RSTGEN_CLRSTAT, RSTGEN_CLRKEY);
	CHECK_STATUS(retval, "error clearing rstgen status.");

	/*
	 * POI
	 */
	LOG_INFO("Power On Internal issued.");
	retval = target_write_u32(target, RSTGEN_POI, RSTGEN_POIKEY);
	CHECK_STATUS(retval, "error writing POI register.");

	target->state = TARGET_RESET;

	/* registers are now invalid */
	register_cache_invalidate(cortex_m->armv7m.arm.core_cache);

	LOG_INFO("Power On Internal complete.");

	return retval;
}

/** Power On Reset (POR). */
static int ambiqmicro_por(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cortex_m_common *cortex_m = target_to_cm(target);
	uint32_t rstgen_stat = 0;
	int retval, timeout = 0;

	/*
	 * Clear Reset Status.
	 */
	retval = target_write_u32(target, RSTGEN_CLRSTAT, RSTGEN_CLRKEY);
	CHECK_STATUS(retval, "error clearing rstgen status.");

	/*
	 * POR
	 */
	LOG_INFO("Power On Reset issued.");
	retval = target_write_u32(target, RSTGEN_POR, RSTGEN_PORKEY);
	CHECK_STATUS(retval, "error writing POR register.");

	target->state = TARGET_RESET;

	/* registers are now invalid */
	register_cache_invalidate(cortex_m->armv7m.arm.core_cache);

	/*
	 * Check if POR occurred (delay is needed.)
	 */
	while (timeout < 20) {
		retval = target_read_u32(target, RSTGEN_STAT, &rstgen_stat);
		CHECK_STATUS(retval, "error reading reset status.");
		alive_sleep(100);
		rstgen_stat &= (RSTGEN_PORSTAT + RSTGEN_SWRSTAT);
		if ((retval == ERROR_OK) && rstgen_stat) {
			retval = ERROR_OK;
			break;
		} else
			retval = ERROR_TARGET_FAILURE;
		timeout++;
	}
	LOG_DEBUG("RSTGEN_STAT %d", rstgen_stat);

	LOG_INFO("Power On Reset complete.");

	return retval;
}

static int ambiqmicro_protect_check(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int i, retval;

	if (ambiqmicro_info->probed == 0) {
		LOG_ERROR("Target not probed.");
		retval = ERROR_FLASH_BANK_NOT_PROBED;
	} else {
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_protected = -1;
		retval = ERROR_OK;
	}

	return retval;
}

/** Erase flash bank. */
static int ambiqmicro_mass_erase(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Set up the SRAM.
	 *          0x10000000    pointer in to flash instance #
	 *          0x10000004    customer value to pass to flash helper routine
	 *          0x10000008    return code debugger sets this to -1 all RCs are >= 0
	 */
	uint32_t sramargs[] = {
		bank->bank_number,
		PROGRAM_KEY,
		BREAKPOINT,
	};

	/*
	 * Erase the main array.
	 */
	LOG_INFO("Mass erase on bank %d.", bank->bank_number);

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_main_command(bank,
		ambiqmicro_info->bootloader->flash_mass_erase_from_sram,
		ARRAY_SIZE(sramargs),
		sramargs);


	/* if successful, set all sectors as erased */
	if (retval == ERROR_OK)
		write_is_erased(bank, 0, bank->num_sectors, 1);

	return retval;
}

/** Erase flash pages (Apollo and Apollo2).
@param bank Flash bank to use.
@param first Start page.
@param last Ending page.
*/
static int ambiqmicro_page_erase(struct flash_bank *bank, int first, int last)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Check pages.
	 * Fix num_pages for the device.
	 */
	if ((first < 0) || (last < first) || (last >= (int)ambiqmicro_info->num_pages))
		return ERROR_FLASH_SECTOR_INVALID;

	/*
	 * Just Mass Erase if all pages are given.
	 * TODO: Fix num_pages for the device
	 */
	if ((first == 0) && (last == ((int)ambiqmicro_info->num_pages-1)))
		return ambiqmicro_mass_erase(bank);

	/*
	 * Set up the SRAM.
	 * Calling this function looks up page erase information from offset 0x0 in SRAM
	 *          0x10000000  instance number
	 *          0x10000004  number of main block pages to erase  must be between 1 and 128 inclusive
	 *                      0 < number < 129
	 *          0x10000008  PROGRAM key to pass to flash helper routine
	 *          0x1000000C  return code debugger sets this to -1 all RCs are >= 0
	 *          0x10000010  PageNumber of the first flash page to erase.
	 *                      NOTE: these *HAVE* to be sequential range 0 <= PageNumber <= 127
	 */

	uint32_t sramargs[] = {
		bank->bank_number,
		(1 + (last-first)),	/* Number of pages to erase. */
		PROGRAM_KEY,
		BREAKPOINT,
		first,
	};

	/*
	 * Erase the pages.
	 */
	LOG_INFO("Erasing pages %d to %d on bank %d", first, last, bank->bank_number);

	/*
	 * Clear Bootloader bit.
	 */
	retval = target_write_u32(bank->target, REG_CONTROL_BOOTLOADERLOW, 0x0);
	CHECK_STATUS(retval, "error clearing bootloader bit.");

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_sram_command(bank,
		ambiqmicro_info->bootloader->flash_erase_main_pages_from_sram,
		ARRAY_SIZE(sramargs), sramargs);
	CHECK_STATUS(retval, "error executing flash page erase");

	/* If we erased the interrupt area, provide the bootloader interrupt table. */
	if (first == 0) {
		/*
		 * Set Bootloader bit.
		 */
		int retval1 = target_write_u32(bank->target, REG_CONTROL_BOOTLOADERLOW, 0x1);
		CHECK_STATUS(retval1, "error setting bootloader bit.");
	}

	if (retval == ERROR_OK) {
		LOG_INFO("%d pages erased!", 1+(last-first));
		write_is_erased(bank, first, last, 1);
	}

	return retval;
}

/** Write protect sectors of flash. */
static int ambiqmicro_protect(struct flash_bank *bank, int set, int first, int last)
{
	/* struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	 * struct target *target = bank->target; */

	/*
	 * TODO
	 */
	LOG_WARNING("Not yet implemented");


	if (!set) {
		LOG_ERROR("Hardware doesn't support page-level unprotect. "
			"Try the 'recover' command.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}


	return ERROR_OK;
}

/** Flash write to main. Apollo and Apollo2. */
static int ambiqmicro_write_block(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t address = bank->base + offset;
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	uint32_t buffer_pointer = ambiqmicro_info->bootloader->write_buffer_start;
	uint32_t maxbuffer = ambiqmicro_info->bootloader->write_buffer_size;
	uint32_t thisrun_count;
	int retval, retval1;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	if (((count%4) != 0) || ((offset%4) != 0)) {
		LOG_ERROR("write block must be multiple of 4 bytes in offset & length");
		return ERROR_FAIL;
	}

	LOG_INFO("Flashing main array 0x%x", buffer_pointer);

	while (count > 0) {
		if (count > maxbuffer)
			thisrun_count = maxbuffer;
		else
			thisrun_count = count;

		/*
		 * Set up the SRAM.
		 * Calling this function looks up programming information from offset 0x0 in SRAM
		 *          0x10000000  pointer in to flash
		 *          0x10000004  number of 32-bit words to program
		 *          0x10000008  customer program key to pass to flash helper routine
		 *          0x1000000C  return code debugger sets this to -1 all RCs are >= 0
		 *
		 *          0x10000010  Apollo  first 32-bit word of data buffer.
		 *          0x10001000  Apollo2 first 32-bit word of data buffer (WRITE_BUFFER_START)
		 */

		uint32_t sramargs[] = {
			address,
			thisrun_count/4,
			PROGRAM_KEY,
			BREAKPOINT,
		};

		/*
		 * Write Buffer.
		 */
		retval = target_write_buffer(target, buffer_pointer, thisrun_count, buffer);
		if (retval != ERROR_OK) {
			CHECK_STATUS(retval, "error writing target SRAM write buffer.");
			break;
		}

		LOG_DEBUG("address = 0x%08x", address);

		retval = ambiqmicro_exec_sram_command(bank,
			ambiqmicro_info->bootloader->flash_program_main_from_sram,
			ARRAY_SIZE(sramargs), sramargs);
		CHECK_STATUS(retval, "error executing ambiqmicro flash write algorithm");
		if (retval != ERROR_OK)
			break;
		buffer += thisrun_count;
		address += thisrun_count;
		count -= thisrun_count;
	}

	LOG_INFO("Main array flashed");

	/*
	 * Clear Bootloader bit regardless of command execution.
	 */
	retval1 = target_write_u32(target, REG_CONTROL_BOOTLOADERLOW, 0x0);
	CHECK_STATUS(retval1, "error clearing bootloader bit");

	return retval;
}

/** Flash write bytes, address count. */
static int ambiqmicro_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	int retval;

	/* try using a block write */
	retval = ambiqmicro_write_block(bank, buffer, offset, count);
	if (retval != ERROR_OK)
		LOG_ERROR("write block failed.");

	return retval;
}

/** Probe part info and flash banks. */
static int ambiqmicro_probe(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	/*
	 * If this is an ambiqmicro chip, it has flash; probe() is just
	 * to figure out how much is present.  Only do it once.
	 */
	if (ambiqmicro_info->probed == 1) {
		LOG_INFO("Target already probed.");
		return ERROR_OK;
	}

	/*
	 * ambiqmicro_read_part_info() already handled error checking and
	 * reporting.  Note that it doesn't write, so we don't care about
	 * whether the target is halted or not.
	 */
	retval = ambiqmicro_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* provide this for the benefit of the NOR flash framework */
	bank->base = bank->bank_number * ambiqmicro_info->flshsiz;
	bank->size = ambiqmicro_info->pagesize * ambiqmicro_info->num_pages;
	bank->num_sectors = ambiqmicro_info->num_pages;

	LOG_DEBUG("bank   number: %d, base: 0x%x, size: %d KB, num sectors: %d",
		bank->bank_number,
		bank->base,
		bank->size/1024,
		bank->num_sectors);

	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	for (int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * ambiqmicro_info->pagesize;
		bank->sectors[i].size = ambiqmicro_info->pagesize;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	/*
	 * Part has been probed.
	 */
	ambiqmicro_info->probed = 1;

	return retval;
}

/** Flash write to info space [APOLLO2].
@param flash_bank bank.
@param uint offset.
@param uint count.
@param uint instance.
 */
static int ambiqmicro_program_info(struct flash_bank *bank,
	uint32_t offset, uint32_t count)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Apollo=2048/4 = 512, Apollo2=8192/4 = 2048 */
	if (count > ambiqmicro_info->bootloader->info_space_size) {
		LOG_ERROR("Count must be < %d", ambiqmicro_info->bootloader->info_space_size);
		return ERROR_FAIL;
	}

	/*
	 * Set up the SRAM.
	 * Calling this function looks up programming information from offset 0x0 in SRAM
	 *          0x10000000  Word offset in to FLASH INFO block
	 *                      0 <= Offset < 2048
	 *          0x10000004  Instance
	 *          0x10000008  number of 32-bit words to program
	 *          0x1000000C  customer program key to pass to flash helper routine
	 *          0x10000010  return code debugger sets this to -1 all RCs are >= 0
	 *
	 *          0x10001000  first 32-bit word of data buffer to be programmed (WRITE_BUFFER_START)
	 */
	uint32_t sramargs[] = {
		offset,
		bank->bank_number,
		count,
		PROGRAM_KEY,
		BREAKPOINT,
	};

	/*
	 * Program Info.
	 */
	LOG_INFO("Programming Info\n\toffset=0x%08x\n\tinstance=0x%08x",
		offset,
		bank->bank_number);

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_sram_command(bank,
		ambiqmicro_info->bootloader->flash_program_info_from_sram,
		ARRAY_SIZE(sramargs),
		sramargs);
	CHECK_STATUS(retval, "error programming info.");

	LOG_INFO("Programming Info finished.");

	return retval;
}

/** Flash write to Apollo OTP space.
@param flash_bank bank.
@param uint offset.
@param uint count.
 */
static int ambiqmicro_otp_program(struct flash_bank *bank,
	uint32_t offset, uint32_t count)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	if (count > ambiqmicro_info->bootloader->info_space_size) {
		LOG_ERROR("Count must be < %d words.", ambiqmicro_info->bootloader->info_space_size);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/*
	 * Set up the SRAM.
	 * Calling this function looks up programming information from offset 0x0 in SRAM
	 *          0x10000000   Offset in to FLASH INFO block (customer version limited to second half)
	 *                       0 <= Offset < 256
	 *                       256 added to offset before programming
	 *          0x10000004	 number of 32-bit words to program
	 *          0x10000008	 OTP program key to pass to flash helper routine
	 *          0x1000000C	 return code debugger sets this to -1 all RCs are >= 0
	 *
	 *          0x10000010	 first 32-bit word of data buffer to be programmed
	 */
	uint32_t sramargs[] = {
		offset,
		count,
		OTP_PROGRAM_KEY,
		BREAKPOINT,
	};

	/*
	 * Program OTP INFO.
	 */
	LOG_INFO("Programming OTP offset 0x%08x", offset);

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_sram_command(bank,
		ambiqmicro_info->bootloader->flash_program_otp_from_sram,
		ARRAY_SIZE(sramargs),
		sramargs);
	CHECK_STATUS(retval, "error executing ambiqmicro otp program algorithm");

	LOG_INFO("Programming OTP finished.");

	return retval;
}

/** Extended recover and erase for bricked devices [APOLLO2]. */
static int ambiqmicro_recover(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Set up the SRAM.
	 * Calling this function looks up programming information from offset 0x0 in SRAM
	 *          0x10000000	 key value to enable recovery
	 *          0x10000004	 return code
	 */
	uint32_t sramargs[] = {
		BRICK_KEY,
		BREAKPOINT,
	};

	/*
	 * Erase the main array.
	 */
	LOG_INFO("Recovering Device started.");

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_sram_command(bank,
		ambiqmicro_info->bootloader->flash_recovery_from_sram,
		ARRAY_SIZE(sramargs),
		sramargs);
	CHECK_STATUS(retval, "error executing ambiqmicro recovery");

	LOG_INFO("Recovering Device finished.");

	return retval;
}

/** Erase info space. [APOLLO2] */
static int ambiqmicro_info_erase(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Set up the SRAM.
	 * Calling this function looks up programming information from offset 0x0 in SRAM
	 *          0x10000000  Flash Instance
	 *          0x10000004  CUSTOMER KEY value to pass to flash helper routine
	 *          0x10000008  return code debugger sets this to -1 all RCs are >= 0
	 */
	uint32_t sramargs[] = {
		bank->bank_number,
		PROGRAM_KEY,
		BREAKPOINT,
	};

	/*
	 * Erase the main array.
	 */
	LOG_INFO("Erasing Info started.");

	/*
	 * passed pc, addr = ROM function, handle breakpoints, not debugging.
	 */
	retval = ambiqmicro_exec_sram_command(bank,
		ambiqmicro_info->bootloader->flash_info_erase_from_sram,
		ARRAY_SIZE(sramargs), sramargs);
	CHECK_STATUS(retval, "error flash info erase from sram.");

	LOG_INFO("Erasing Info complete.");

	return retval;
}

/** Erase info space + main. [APOLLO2] */
static int ambiqmicro_info_plus_main_erase(struct flash_bank *bank)
{
	struct ambiqmicro_flash_bank *ambiqmicro_info = bank->driver_priv;
	int retval;

	retval = target_ready_for_command(bank);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Set up the SRAM.
	 * Calling this function looks up programming information from offset 0x0 in SRAM
	 *          0x10000000  Flash Instance
	 *          0x10000004  Customer KEY value to pass to flash helper routine
	 *          0x10000008  return code debugger sets this to -1 all RCs are >= 0
	 */
	uint32_t sramargs[] = {
		bank->bank_number,
		PROGRAM_KEY,
		BREAKPOINT,
	};

	/*
	 * Erase the main plus info array.
	 */
	LOG_INFO("Erasing Info plus Main started.");
	retval = ambiqmicro_exec_main_command(bank,
		ambiqmicro_info->bootloader->flash_info_plus_main_erase_from_sram,
		ARRAY_SIZE(sramargs),
		sramargs);
	CHECK_STATUS(retval, "error flash info plus main erase from sram.");

	/* if successful, set all sectors as erased. */
	if (retval == ERROR_OK)
		write_is_erased(bank, 0, bank->num_sectors, 1);

	LOG_INFO("Erasing Info plus Main complete.");

	return retval;
}

COMMAND_HANDLER(ambiqmicro_handle_poi_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bank = get_flash_bank_by_num_noprobe(0);
	if (!bank)
		return ERROR_FAIL;

	retval = ambiqmicro_poi(bank);

	return retval;
}

COMMAND_HANDLER(ambiqmicro_handle_por_command)
{
	struct flash_bank *bank;
	int retval;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bank = get_flash_bank_by_num_noprobe(0);
	if (!bank)
		return ERROR_FAIL;

	retval = ambiqmicro_por(bank);

	return retval;
}

COMMAND_HANDLER(ambiqmicro_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	uint32_t retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (ambiqmicro_mass_erase(bank) == ERROR_OK)
		command_print(CMD_CTX, "ambiqmicro mass erase complete");
	else
		command_print(CMD_CTX, "ambiqmicro mass erase failed");

	return ERROR_OK;
}

COMMAND_HANDLER(ambiqmicro_handle_page_erase_command)
{
	struct flash_bank *bank;
	uint32_t first, last;
	uint32_t retval;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], first);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], last);

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (ambiqmicro_page_erase(bank, first, last) == ERROR_OK)
		command_print(CMD_CTX, "ambiqmicro page erase complete");
	else
		command_print(CMD_CTX, "ambiqmicro page erase failed");

	return ERROR_OK;
}

/** Program the Apollo otp block. */
COMMAND_HANDLER(ambiqmicro_handle_program_otp_command)
{
	struct flash_bank *bank;
	uint32_t offset, count;
	uint32_t retval;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], offset);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);

	CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	retval = ambiqmicro_otp_program(bank, offset, count);

	if (retval != ERROR_OK)
		LOG_ERROR("error check log");

	return ERROR_OK;
}

/** Program the Apollo2 info block. */
COMMAND_HANDLER(ambiqmicro_handle_program_info_command)
{
	struct flash_bank *bank;
	uint32_t offset, count;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], offset);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);

	CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	ambiqmicro_program_info(bank, offset, count);

	return ERROR_OK;
}

/**
 * Perform the Recovering a Locked Device procedure. [APOLLO2]
 * This performs a mass erase and then restores all nonvolatile registers
 * (including flash lock bits) to their defaults.
 * Accordingly, flash can be reprogrammed, and SWD can be used.
 */
COMMAND_HANDLER(ambiqmicro_handle_recover_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank = get_flash_bank_by_num_noprobe(0);
	if (!bank)
		return ERROR_FAIL;

	ambiqmicro_recover(bank);

	return ERROR_OK;
}

/** Erase the info block. [APOLLO2] */
COMMAND_HANDLER(ambiqmicro_handle_erase_info_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	uint32_t retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	ambiqmicro_info_erase(bank);

	return ERROR_OK;
}

/** Erase the info plus main block. [APOLLO2] */
COMMAND_HANDLER(ambiqmicro_handle_erase_info_plus_main_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	uint32_t retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	ambiqmicro_info_plus_main_erase(bank);

	return ERROR_OK;
}


static const struct command_registration ambiqmicro_exec_command_handlers[] = {
	{
		.name = "poi",
		.usage = "Power On Internal.",
		.handler = ambiqmicro_handle_poi_command,
		.mode = COMMAND_EXEC,
		.help = "Send POI to target.",
	},
	{
		.name = "por",
		.usage = "Power On Reset.",
		.handler = ambiqmicro_handle_por_command,
		.mode = COMMAND_EXEC,
		.help = "Send POR to target.",
	},
	{
		.name = "mass_erase",
		.usage = "<bank>",
		.handler = ambiqmicro_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase entire device",
	},
	{
		.name = "page_erase",
		.usage = "<bank> <first> <last>",
		.handler = ambiqmicro_handle_page_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase flash pages.",
	},
	{
		.name = "program_otp",
		.handler = ambiqmicro_handle_program_otp_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank> <offset> <count>",
		.help =
			"Program OTP is a one time operation program info space [APOLLO ONLY]. "
			"Before issuing the command, the caller writes sectors to sram starting at 0x10000010. "
			"The writes to info space are permanent. "
			"There is no way to erase and re-program once this command is used.",
	},
	{
		.name = "program_info",
		.handler = ambiqmicro_handle_program_info_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank> <offset> <count>",
		.help =
			"Program INFO will write sectors from sram to info space [APOLLO2 ONLY]. "
			"Before issuing the command, the caller writes sectors to sram starting at 0x10001000.",
	},
	{
		.name = "recover",
		.handler = ambiqmicro_handle_recover_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "recover and erase locked device [APOLLO2 ONLY]",
	},
	{
		.name = "erase_info",
		.handler = ambiqmicro_handle_erase_info_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank>>",
		.help = "erase info block [APOLLO2 ONLY] "
			"Never returns, breakpoint back to attached debugger.",
	},
	{
		.name = "erase_info_plus_main",
		.handler = ambiqmicro_handle_erase_info_plus_main_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank>",
		.help = "erase info plus main block [APOLLO2 ONLY]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ambiqmicro_command_handlers[] = {
	{
		.name = "ambiqmicro",
		.mode = COMMAND_EXEC,
		.help = "ambiqmicro flash command group",
		.usage = "Support for Apollo Ultra Low Power Microcontrollers.",
		.chain = ambiqmicro_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver ambiqmicro_flash = {
	.name = "ambiqmicro",
	.commands = ambiqmicro_command_handlers,
	.flash_bank_command = ambiqmicro_flash_bank_command,
	.erase = ambiqmicro_page_erase,
	.write = ambiqmicro_write,
	.read = default_flash_read,
	.probe = ambiqmicro_probe,
	.auto_probe = ambiqmicro_probe,
	.erase_check = default_flash_blank_check,
	.info = get_ambiqmicro_info,
	.protect_check = ambiqmicro_protect_check,
	.protect = ambiqmicro_protect,
};
